/**
 * @file    app_chain.c
 * @brief   `control_chain` 的应用层封装实现
 *
 * ## 分工
 *
 * - **命令语义**（`move` / `gait` / `height` / `gesture` / 跨帧目标 / 节拍算术）
 *   全在 `control/control_chain_cmd.c` —— 纯 C，被**多帧序列 golden** 逐帧验证过
 *   （`tools/golden/test_control_chain_cmd.c`，820 帧零不符）。
 * - **本文件只做固件侧的那点事**：读配置、加互斥锁、把链挂到运动任务的节拍上。
 *
 * 之所以这样切：命令语义里有几个很容易漏的细节（`move()` 的"模式变了才 t=0"、
 * `height()` 会直接同步 `R_H`、目标用 `int()` 截断），而它们**只有在多帧连续
 * 运行下才暴露**。放在宿主可测的纯 C 模块里，才能被 golden 钉住。
 */

#include "app/app_chain.h"

#include <string.h>

#include "app/app_cfg_cmd.h"
#include "control/control_chain_cmd.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "app_chain";

/* ==========================================================================
 * 状态
 * ========================================================================== */

static SemaphoreHandle_t s_mutex = NULL;

/** 配置（`app_chain_reload_cfg()` 时从 app_config 重建） */
static control_chain_cfg_t   s_cfg;
/** 链的持久状态（`t` / `R_H` / `PIT_S` / `ROL_S` / `X_S` / 爬行状态机） */
static control_chain_state_t s_st;
/** 命令状态（含**跨帧保留**的四个目标） */
static control_chain_cmd_t   s_cmd;
/** 节拍（默认 65 ms，见 control_chain_cmd.h 的推导） */
static control_chain_sched_t s_sched;

/* ---- 输出缓存 ---- */
static bool     s_valid  = false;
static uint32_t s_frames = 0;
static float    s_angle[SERVO_MAP_CHANNELS];

/** 下标用的常量 */
enum { GOAL_H = CONTROL_CHAIN_GOAL_H, GOAL_PIT = CONTROL_CHAIN_GOAL_PIT,
       GOAL_ROL = CONTROL_CHAIN_GOAL_ROL, GOAL_X = CONTROL_CHAIN_GOAL_X };

/* ==========================================================================
 * 配置映射
 * ========================================================================== */

void app_chain_cfg_from_app_config(const app_config_t *c, control_chain_cfg_t *out)
{
    if (out == NULL) {
        return;
    }

    /*
     * 先铺**注入表**的全部默认值，再用 app_config 覆盖。
     *
     * 这个顺序很重要：注入表里有一批字段（`_LARGE_*` / `HIP_TURN_*` / `CRAWL_*`）
     * **本来就不是配置项**（原版里是模块级常量），只能由
     * `control_chain_cfg_defaults()` 提供。反过来"只覆盖不铺底"会让它们保持 0 ——
     * 那是静默的错误行为。
     */
    control_chain_cfg_defaults(out);

    if (c == NULL) {
        return;
    }

    /* ---- config.py / config_s.py 提供的 ---- */
    out->Ts          = c->ts;
    out->faai        = c->faai;
    out->pit_max_ang = c->pit_max_ang;
    out->rol_max_ang = c->rol_max_ang;
    out->xs_max      = c->xs_max;

    for (int leg = 0; leg < CONTROL_CHAIN_LEGS; ++leg) {
        for (int j = 0; j < CONTROL_CHAIN_JOINTS; ++j) {
            out->init[leg][j] = c->servo_center[leg][j];
        }
    }

    out->l1            = c->l1;
    out->l2            = c->l2;
    out->l             = c->l;
    out->b             = c->b;
    out->w             = c->w;
    out->speed         = c->speed;
    out->h             = c->h;
    out->Kp_H          = c->kp_h;
    out->Kp_G          = c->kp_g;
    out->CG_X          = c->cg_x;
    out->CG_Y          = c->cg_y;
    out->walk_h        = c->walk_h;
    out->walk_speed    = c->walk_speed;
    out->leg_len_ref   = c->leg_len_ref;
    out->joy_fwd_sign  = c->joy_fwd_sign;

    out->trot_cg_f     = c->trot_cg_f;
    out->trot_cg_b     = c->trot_cg_b;
    out->trot_cg_t     = c->trot_cg_t;

    out->hip_k_roll    = c->hip_k_roll;
    out->hip_k_pitch   = c->hip_k_pitch;
    out->hip_k_turn    = c->hip_k_turn;
    out->hip_delta_max = c->hip_delta_max;

    out->H_goal        = c->h_goal;
    out->in_y          = c->in_y;
    out->in_pit        = c->in_pit;
    out->in_rol        = c->in_rol;

    out->ma_case       = c->ma_case;

    /* ---- 只存在于 padog.py 注入表的（P3-3 才加进 app_config 的 19 个） ---- */
    out->shank_ik_bias_per_mm = c->shank_ik_bias_per_mm;
    out->shank_ik_bias_deg    = c->shank_ik_bias_deg;
    out->front_leg_y_offset   = c->front_leg_y_offset;
    out->rear_leg_y_offset    = c->rear_leg_y_offset;
    for (int i = 0; i < CONTROL_CHAIN_LEGS; ++i) {
        out->s_trim[i] = c->s_trim[i];
    }
    out->leg2_z_mul       = c->leg2_z_mul;
    out->leg3_z_mul       = c->leg3_z_mul;
    out->leg4_z_mul       = c->leg4_z_mul;
    out->walk_speed_scale = c->walk_speed_scale;
    out->walk_roll_trim   = c->walk_roll_trim;
    out->trot_roll_trim   = c->trot_roll_trim;
    out->trot_right_h_mul = c->trot_right_h_mul;
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

esp_err_t app_chain_init(void)
{
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == NULL) {
            ESP_LOGE(TAG, "创建互斥锁失败");
            return ESP_ERR_NO_MEM;
        }
    }

    const app_config_t *c = app_cfg_cmd_get();
    if (c == NULL) {
        ESP_LOGE(TAG, "配置未就绪 —— 必须先调用 app_cfg_cmd_init()");
        return ESP_ERR_INVALID_STATE;
    }

    app_chain_cfg_from_app_config(c, &s_cfg);
    control_chain_state_init(&s_st, &s_cfg);
    control_chain_cmd_init(&s_cmd, &s_cfg);

    const uint32_t period = control_chain_sched_period_from_cfg(&s_cfg);
    control_chain_sched_init(&s_sched, period);

    s_valid = false;
    s_frames = 0;
    memset(s_angle, 0, sizeof(s_angle));

    ESP_LOGI(TAG, "控制链就绪：节拍 %u ms (%.1f Hz)，一个步态周期 = %.2f 帧 = %.0f ms",
             (unsigned)period, 1000.0f / (float)period,
             (double)(s_cfg.Ts / s_cfg.speed),
             (double)((s_cfg.Ts / s_cfg.speed) * (float)period));
    ESP_LOGI(TAG, "  speed=%.3f Ts=%.2f faai=%.2f h=%.1f walk_faai=%.2f 初始=站立命令",
             (double)s_cfg.speed, (double)s_cfg.Ts, (double)s_cfg.faai,
             (double)s_cfg.h, (double)s_cfg.walk_faai);
    return ESP_OK;
}

esp_err_t app_chain_reload_cfg(void)
{
    const app_config_t *c = app_cfg_cmd_get();
    if (c == NULL || s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    app_chain_cfg_from_app_config(c, &s_cfg);
    /* 姿态状态与命令都复位到新配置的初值 —— 改了中位角/几何之后必须重新站 */
    control_chain_state_init(&s_st, &s_cfg);
    control_chain_cmd_init(&s_cmd, &s_cfg);
    control_chain_sched_set_period(&s_sched, control_chain_sched_period_from_cfg(&s_cfg));
    s_valid = false;
    s_frames = 0;
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "配置已重载：节拍 %u ms，姿态与命令状态已复位",
             (unsigned)control_chain_sched_period_from_cfg(&s_cfg));
    return ESP_OK;
}

/* ==========================================================================
 * 命令（全部委托给被 golden 验证过的纯 C 命令层）
 * ========================================================================== */

esp_err_t app_chain_set_period_ms(uint32_t ms)
{
    if (ms < 1 || ms > 1000) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_mutex != NULL && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        control_chain_sched_set_period(&s_sched, ms);
        xSemaphoreGive(s_mutex);
    }
    return ESP_OK;
}

uint32_t app_chain_get_period_ms(void)
{
    return (s_sched.period_ms != 0) ? s_sched.period_ms : APP_CHAIN_DEFAULT_PERIOD_MS;
}

esp_err_t app_chain_set_gait(int gait_mode)
{
    if (gait_mode != 0 && gait_mode != 1) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    control_chain_cmd_gait(&s_cmd, &s_cfg, &s_st, gait_mode);
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t app_chain_jog(float spd, int L, int R)
{
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    control_chain_cmd_move(&s_cmd, &s_cfg, &s_st, spd, L, R);
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t app_chain_set_joy_turn(float pct)
{
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    control_chain_cmd_set_turn(&s_cmd, pct);
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t app_chain_set_height(float h_goal)
{
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    control_chain_cmd_height(&s_cmd, &s_st, h_goal);
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t app_chain_stand(void)
{
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    control_chain_cmd_stand(&s_cmd, &s_cfg, &s_st);
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

void app_chain_reset_pose(void)
{
    if (s_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }
    control_chain_state_init(&s_st, &s_cfg);
    control_chain_cmd_init(&s_cmd, &s_cfg);
    s_valid = false;
    xSemaphoreGive(s_mutex);
}

/* ==========================================================================
 * 每帧推进
 * ========================================================================== */

bool app_chain_step(int64_t now_ms, float angle_out[SERVO_MAP_CHANNELS])
{
    if (angle_out == NULL || s_mutex == NULL) {
        return false;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        memcpy(angle_out, s_angle, sizeof(s_angle));
        return false;
    }

    /* 没到链的节拍：把上一帧的角度还回去（servo_out 不会重复写 I2C） */
    if (s_valid && !control_chain_sched_due(&s_sched, now_ms)) {
        memcpy(angle_out, s_angle, sizeof(s_angle));
        xSemaphoreGive(s_mutex);
        return false;
    }

    control_chain_input_t in;
    memset(&in, 0, sizeof(in));
    control_chain_cmd_make_input(&s_cmd, (int32_t)now_ms, &in);
    in.crawl_phase = 0;      /* 爬行动作是网页的"动作"，P5/P6 再接 */

    control_chain_out_t out;
    memset(&out, 0, sizeof(out));
    control_chain_tick(&s_cfg, &s_st, &in, &out);

    /* ⚠️ 把本帧实际用到的目标收回（原版那是模块级全局，改一次一直留着） */
    control_chain_cmd_absorb(&s_cmd, &out);

    memcpy(s_angle, out.angle_deg, sizeof(s_angle));
    s_valid = true;
    ++s_frames;

    memcpy(angle_out, s_angle, sizeof(s_angle));
    xSemaphoreGive(s_mutex);
    return true;
}

void app_chain_get_status(app_chain_status_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (s_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }
    out->valid     = s_valid;
    out->frames    = s_frames;
    out->period_ms = app_chain_get_period_ms();
    out->gait_mode = s_cmd.gait_mode;
    out->spd       = s_cmd.spd;
    out->L         = s_cmd.L;
    out->R         = s_cmd.R;
    out->joy_turn  = s_cmd.joy_turn;
    memcpy(out->goal, s_cmd.goal, sizeof(out->goal));
    out->t         = s_st.t;
    out->R_H       = s_st.R_H;
    out->PIT_S     = s_st.PIT_S;
    out->ROL_S     = s_st.ROL_S;
    out->X_S       = s_st.X_S;
    memcpy(out->angle_deg, s_angle, sizeof(out->angle_deg));
    xSemaphoreGive(s_mutex);
}
