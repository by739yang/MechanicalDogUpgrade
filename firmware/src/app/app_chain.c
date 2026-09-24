/**
 * @file    app_chain.c
 * @brief   `control_chain` 的应用层封装实现
 */

#include "app/app_chain.h"

#include <math.h>
#include <string.h>

#include "app/app_cfg_cmd.h"
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

/* ---- 命令（控制台任务写、运动任务读，用 s_mutex 保护） ---- */
static float s_spd      = 0.0f;
static int   s_L        = 0;
static int   s_R        = 0;
static int   s_gait     = 0;
static float s_joy_turn = 0.0f;

/**
 * 四个目标。**跨帧保留**（见 app_chain.h 的说明）：
 * 每帧推进后被 `control_chain_out_t.goal[4]` 覆写，
 * 因为原版里 WALK 的 `cal_w()` 会通过 `padog.gesture()` 改它们。
 */
static float s_goal[4]  = { 0.0f, 0.0f, 0.0f, 0.0f };  /* H, PIT, ROL, X */

/* ---- 输出缓存与节拍 ---- */
static bool     s_valid      = false;
static uint32_t s_frames     = 0;
static uint32_t s_period_ms  = APP_CHAIN_DEFAULT_PERIOD_MS;
static int64_t  s_next_due_ms = 0;
static float    s_angle[SERVO_MAP_CHANNELS];

/** 把四个目标设成配置里的初值（= 原版模块级 `PIT_goal=int(in_pit)` 等） */
static void goals_to_config_init(const control_chain_cfg_t *cfg)
{
    s_goal[0] = cfg->H_goal;
    s_goal[1] = cfg->in_pit;
    s_goal[2] = cfg->in_rol;
    s_goal[3] = cfg->in_y;
}

/** 下标用的常量，避免到处写魔法数 */
enum { GOAL_H = 0, GOAL_PIT = 1, GOAL_ROL = 2, GOAL_X = 3 };

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
     * 这个顺序很重要：注入表里有 19 个字段在 `app_config_t` 里也有对应项，
     * 但还有一批（`_LARGE_*` / `HIP_TURN_*` / `CRAWL_*`）**本来就不是配置项**
     * （原版里是模块级常量），它们只能由 `control_chain_cfg_defaults()` 提供。
     * 反过来只覆盖不铺底，那些字段就会是 0 —— 那是静默的错误行为。
     */
    control_chain_cfg_defaults(out);

    if (c == NULL) {
        return;
    }

    /* ---- config.py / config_s.py 提供的 ---- */
    out->Ts           = c->ts;
    out->faai         = c->faai;
    out->pit_max_ang  = c->pit_max_ang;
    out->rol_max_ang  = c->rol_max_ang;
    out->xs_max       = c->xs_max;

    for (int leg = 0; leg < CONTROL_CHAIN_LEGS; ++leg) {
        for (int j = 0; j < CONTROL_CHAIN_JOINTS; ++j) {
            out->init[leg][j] = c->servo_center[leg][j];
        }
    }

    out->l1           = c->l1;
    out->l2           = c->l2;
    out->l            = c->l;
    out->b            = c->b;
    out->w            = c->w;
    out->speed        = c->speed;
    out->h            = c->h;
    out->Kp_H         = c->kp_h;
    out->Kp_G         = c->kp_g;
    out->CG_X         = c->cg_x;
    out->CG_Y         = c->cg_y;
    out->walk_h       = c->walk_h;
    out->walk_speed   = c->walk_speed;
    out->leg_len_ref  = c->leg_len_ref;
    out->joy_fwd_sign = c->joy_fwd_sign;

    out->trot_cg_f    = c->trot_cg_f;
    out->trot_cg_b    = c->trot_cg_b;
    out->trot_cg_t    = c->trot_cg_t;

    out->hip_k_roll   = c->hip_k_roll;
    out->hip_k_pitch  = c->hip_k_pitch;
    out->hip_k_turn   = c->hip_k_turn;
    out->hip_delta_max = c->hip_delta_max;

    out->H_goal       = c->h_goal;
    out->in_y         = c->in_y;
    out->in_pit       = c->in_pit;
    out->in_rol       = c->in_rol;

    out->ma_case      = c->ma_case;

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

/**
 * 链节拍默认值：`speed` 秒（见 app_chain.h 的推导）。
 *
 * ⚠️ 这是"把原版隐性耦合显式化"的落点：`speed` 同时决定了
 * "每帧推进多少相位"和"一帧该有多长"。改 `speed` 会同时改这两件事，
 * 结果就是**行走速度不变**（这正是原版的行为）。
 */
static uint32_t period_from_cfg(const control_chain_cfg_t *cfg)
{
    if (cfg == NULL || cfg->speed <= 0.0f) {
        return APP_CHAIN_DEFAULT_PERIOD_MS;
    }
    const float ms = cfg->speed * 1000.0f;
    if (ms < 1.0f) {
        return 1u;
    }
    if (ms > 1000.0f) {
        return 1000u;
    }
    return (uint32_t)(ms + 0.5f);
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
    goals_to_config_init(&s_cfg);

    s_spd = 0.0f;
    s_L = 0;
    s_R = 0;
    s_gait = 0;
    s_joy_turn = 0.0f;
    s_valid = false;
    s_frames = 0;
    s_next_due_ms = 0;
    memset(s_angle, 0, sizeof(s_angle));

    s_period_ms = period_from_cfg(&s_cfg);

    ESP_LOGI(TAG, "控制链就绪：节拍 %u ms (%.1f Hz)，步态周期目标 %.2f s，初始为站立命令",
             (unsigned)s_period_ms, 1000.0f / (float)s_period_ms,
             (double)(s_period_ms / 1000.0f) * (double)(s_cfg.Ts / s_cfg.speed));
    ESP_LOGI(TAG, "  speed=%.3f Ts=%.2f faai=%.2f h=%.1f walk_faai=%.2f",
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
    /* 姿态状态复位到新配置的初值 —— 改了中位角/几何之后必须重新站 */
    control_chain_state_init(&s_st, &s_cfg);
    goals_to_config_init(&s_cfg);
    s_period_ms = period_from_cfg(&s_cfg);
    s_valid = false;
    s_frames = 0;
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "配置已重载：节拍 %u ms，姿态状态已复位", (unsigned)s_period_ms);
    return ESP_OK;
}

/* ==========================================================================
 * 命令
 * ========================================================================== */

esp_err_t app_chain_set_period_ms(uint32_t ms)
{
    if (ms < 1 || ms > 1000) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_mutex != NULL && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_period_ms = ms;
        s_next_due_ms = 0;   /* 立刻允许下一帧 */
        xSemaphoreGive(s_mutex);
    }
    return ESP_OK;
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
    if (s_gait != gait_mode) {
        /*
         * 原版 `padog.gait(mode)`：切换时 `t = 0`，且 `mode==0` 时把三个目标
         * 复位成 `in_pit/in_rol/in_y`。照抄。
         */
        s_st.t = 0.0f;
        s_gait = gait_mode;
        if (gait_mode == 0) {
            s_goal[GOAL_PIT] = s_cfg.in_pit;
            s_goal[GOAL_ROL] = s_cfg.in_rol;
            s_goal[GOAL_X]   = s_cfg.in_y;
        }
    }
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
    s_spd = spd;
    s_L = L;
    s_R = R;
    /*
     * 原版 `padog.move(spd, L, R)`：只要"有方向且有速度"就把 `t` 归零、
     * 并把 `gait_mode` 设成 0（`move()` 里调 `gait(0)`）。
     * 注意 `gait(0)` 会顺带把三个重心目标复位 —— 这是原版行为。
     */
    if ((L + R) != 0 && fabsf(spd) > 0.0f) {
        s_st.t = 0.0f;
        s_gait = 0;
        s_goal[GOAL_PIT] = s_cfg.in_pit;
        s_goal[GOAL_ROL] = s_cfg.in_rol;
        s_goal[GOAL_X]   = s_cfg.in_y;
    }
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
    s_joy_turn = pct;
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
    s_spd = 0.0f;
    s_L = 0;
    s_R = 0;
    s_gait = 0;
    s_joy_turn = 0.0f;
    s_st.t = 0.0f;
    goals_to_config_init(&s_cfg);
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
    goals_to_config_init(&s_cfg);
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

    /* ---- 取一份命令与目标的快照 ---- */
    float spd, joy_turn;
    int   L, R, gait;
    float goal[4];
    uint32_t period_ms;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        memcpy(angle_out, s_angle, sizeof(s_angle));
        return false;
    }

    /* 还没到链的节拍：直接把上一帧的角度还回去（servo_out 不会重复写 I2C） */
    if (s_valid && now_ms < s_next_due_ms) {
        memcpy(angle_out, s_angle, sizeof(s_angle));
        xSemaphoreGive(s_mutex);
        return false;
    }

    spd       = s_spd;
    L         = s_L;
    R         = s_R;
    gait      = s_gait;
    joy_turn  = s_joy_turn;
    period_ms = s_period_ms;
    memcpy(goal, s_goal, sizeof(goal));

    /* ---- 组输入并推进一帧 ---- */
    control_chain_input_t in;
    memset(&in, 0, sizeof(in));
    in.spd        = spd;
    in.L          = L;
    in.R          = R;
    in.gait_mode  = gait;
    in.joy_turn   = joy_turn;
    in.crawl_phase = 0;      /* 爬行动作是网页的"动作"，P5/P6 再接 */
    in.H_goal     = goal[GOAL_H];
    in.PIT_goal   = goal[GOAL_PIT];
    in.ROL_goal   = goal[GOAL_ROL];
    in.X_goal     = goal[GOAL_X];
    in.now_ms     = (int32_t)now_ms;

    control_chain_out_t out;
    memset(&out, 0, sizeof(out));
    control_chain_tick(&s_cfg, &s_st, &in, &out);

    /*
     * ⚠️ 把**本帧实际用到**的目标写回（`control_chain.h` 的契约）。
     * WALK 的 `cal_w()` 会通过 `gesture()` 改 `PIT/ROL/X_goal`，
     * 原版那是模块级全局、改一次一直留着，所以这里必须保留。
     */
    memcpy(s_goal, out.goal, sizeof(s_goal));

    memcpy(s_angle, out.angle_deg, sizeof(s_angle));
    s_valid = true;
    ++s_frames;

    /* 下一次该在什么时候推进：按周期累加，落后太多就不追补（宁可相位慢一点，
     * 也不要连续几帧把 t 一次推很远 —— 那会让步态瞬间跳相位） */
    s_next_due_ms += (int64_t)period_ms;
    if (now_ms > s_next_due_ms + (int64_t)period_ms) {
        s_next_due_ms = now_ms + (int64_t)period_ms;
    }

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
    out->period_ms = s_period_ms;
    out->gait_mode = s_gait;
    out->spd       = s_spd;
    out->L         = s_L;
    out->R         = s_R;
    out->joy_turn  = s_joy_turn;
    memcpy(out->goal, s_goal, sizeof(out->goal));
    out->t         = s_st.t;
    out->R_H       = s_st.R_H;
    out->PIT_S     = s_st.PIT_S;
    out->ROL_S     = s_st.ROL_S;
    out->X_S       = s_st.X_S;
    memcpy(out->angle_deg, s_angle, sizeof(out->angle_deg));
    xSemaphoreGive(s_mutex);
}
