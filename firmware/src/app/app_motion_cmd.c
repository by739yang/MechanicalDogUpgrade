/**
 * @file    app_motion_cmd.c
 * @brief   P2 串口控制台命令实现
 */

#include "app/app_motion_cmd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app/app_action.h"
#include "app/app_chain.h"
#include "app/app_cfg_cmd.h"
#include "app/motion.h"
#include "app/servo_out.h"
#include "drivers/drv_pca9685.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "p2";

/** 单通道测试时的偏移默认值（度）—— 刻意小，第一次上电不会甩舵机 */
#define LG_TEST_DELTA_DEG 12.0f
/** 单通道测试保持时间（毫秒） */
#define LG_TEST_HOLD_MS   1500

void app_motion_cmd_help(void)
{
    ESP_LOGI(TAG, "  ---- P2 运动（固定周期舵机输出）----");
    ESP_LOGI(TAG, "  motion start|stop            启动/停止固定周期控制任务");
    ESP_LOGI(TAG, "  motion stat                  周期/抖动/写次数/目标与当前角度");
    ESP_LOGI(TAG, "  motion period <ms>           控制周期（默认 10 = 100 Hz）");
    ESP_LOGI(TAG, "  motion rate <deg_per_s>      速率上限（默认 120，仅 POSE 模式）");
    ESP_LOGI(TAG, "  motion timeout <ms>          命令超时后松力停车（默认 10000，0=关闭）");
    ESP_LOGI(TAG, "  motion mode pose|chain|action  控制模式：直接 12 路 / 控制链 / 姿态动画");
    ESP_LOGI(TAG, "  stand                        站姿（走控制链 = 原版真正的站姿）");
    ESP_LOGI(TAG, "  stand direct                 标定用站姿（12 路 = 中位角，见 E8）");
    ESP_LOGI(TAG, "  action stand|sit|sit_direct|wave|step|stop");
    ESP_LOGI(TAG, "                               姿态动画/动作层（须先 motion mode action）");
    ESP_LOGI(TAG, "                                 stand/sit = 动画，wave = 挥手，step = 步态测试");
    ESP_LOGI(TAG, "  gait trot|walk               选步态（chain 模式）");
    ESP_LOGI(TAG, "  jog <spd> <L> <R>            行走命令（= 原版 move()，会切回 TROT）");
    ESP_LOGI(TAG, "  drive <spd> <L> <R>          行走命令（= 原版 drive()，不切步态）");
    ESP_LOGI(TAG, "  turn <pct>                   横杆转向百分比（|pct|>=10 才生效）");
    ESP_LOGI(TAG, "  chain                        打印控制链状态（相位/目标/角度）");
    ESP_LOGI(TAG, "  estop [reason]               急停：取消动作 + 12 路松力并停任务");
    ESP_LOGI(TAG, "  lg <ch 0..11> <deg>          直接设某逻辑通道角度（须先 motion stop）");
    ESP_LOGI(TAG, "  lg off                       12 路全部无脉冲（松力）");
    ESP_LOGI(TAG, "  lgtest <ch> [delta_deg]      单通道相对中位角偏移，用于核对硬件映射");
    ESP_LOGI(TAG, "  readback                     回读 12 路的 (ON,OFF) 与缓存对照");
}

/* ==========================================================================
 * motion ...
 * ========================================================================== */

static void cmd_motion(const char *args)
{
    char sub[24] = { 0 };
    const char *rest = args;
    size_t i = 0;
    while (rest[i] != '\0' && rest[i] != ' ' && i < sizeof(sub) - 1) {
        sub[i] = rest[i];
        ++i;
    }
    sub[i] = '\0';
    rest += i;
    while (*rest == ' ') {
        ++rest;
    }

    if (sub[0] == '\0' || strcmp(sub, "stat") == 0) {
        motion_stats_t st;
        motion_get_stats(&st);
        ESP_LOGI(TAG, "运行中=%s 急停=%s 已到位=%s 停止原因=%s",
                 st.running ? "是" : "否", st.estopped ? "是" : "否",
                 st.settled ? "是" : "否", motion_stop_reason_str(st.stop_reason));
        ESP_LOGI(TAG, "参数: 周期=%.1f ms 速率=%.0f °/s 超时=%u ms",
                 (double)st.period_ms, (double)st.rate_dps, (unsigned)st.timeout_ms);
        ESP_LOGI(TAG, "ticks=%u 周期us(平均/最长)=%lld/%lld 抖动max=%lldus 超期=%u",
                 (unsigned)st.ticks, (long long)st.period_avg_us,
                 (long long)st.period_max_us, (long long)st.jitter_max_us,
                 (unsigned)st.overruns);
        ESP_LOGI(TAG, "干活us(平均/最长)=%lld/%lld   I2C 通道写累计=%u",
                 (long long)st.work_avg_us, (long long)st.work_max_us,
                 (unsigned)st.i2c_writes);
        for (uint8_t ch = 0; ch < SERVO_MAP_CHANNELS; ++ch) {
            ESP_LOGI(TAG, "  ch%-2u %-10s 目标=%7.2f 当前=%7.2f",
                     (unsigned)ch, servo_map_channel_name(ch),
                     (double)st.target[ch], (double)st.current[ch]);
        }
        return;
    }

    if (strcmp(sub, "start") == 0) {
        motion_keepalive();
        const esp_err_t err = motion_start();
        ESP_LOGI(TAG, "motion start -> %s", esp_err_to_name(err));
        return;
    }

    if (strcmp(sub, "stop") == 0) {
        /* 停任务就是"取消动作"：先让动作层复位，免得下次 `motion mode action`
         * 又接着跑半个挥手脚本；松力由 motion_stop() 完成 */
        app_action_stop();
        motion_stop(MOTION_STOP_USER);
        ESP_LOGI(TAG, "已停止，动作已取消，12 路松力");
        return;
    }

    if (strcmp(sub, "mode") == 0) {
        const uint32_t prev = motion_get_mode();
        if (strcmp(rest, "chain") == 0) {
            ESP_LOGI(TAG, "motion mode chain -> %s", esp_err_to_name(motion_set_mode(MOTION_MODE_CHAIN)));
        } else if (strcmp(rest, "pose") == 0) {
            ESP_LOGI(TAG, "motion mode pose -> %s", esp_err_to_name(motion_set_mode(MOTION_MODE_POSE)));
        } else if (strcmp(rest, "action") == 0) {
            ESP_LOGI(TAG, "motion mode action -> %s", esp_err_to_name(motion_set_mode(MOTION_MODE_ACTION)));
            ESP_LOGI(TAG, "  用 `action stand|sit|sit_direct|wave|step|stop` 触发动作");
        } else {
            ESP_LOGE(TAG, "用法: motion mode pose|chain|action");
            return;
        }
        /* 离开 ACTION 模式 = 那些角度不会再被下发了 ⇒ 动作必须取消，
         * 否则它一直挂在"进行中"，回来时突然接着动 */
        if (prev == MOTION_MODE_ACTION && motion_get_mode() != MOTION_MODE_ACTION) {
            app_action_stop();
            ESP_LOGW(TAG, "已离开 ACTION 模式，正在进行的动作已取消");
        }
        return;
    }

    if (strcmp(sub, "period") == 0 || strcmp(sub, "rate") == 0 || strcmp(sub, "timeout") == 0) {        const int v = atoi(rest);
        esp_err_t err;
        if (strcmp(sub, "period") == 0) {
            err = motion_set_period_ms((uint32_t)v);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "控制周期 = %d ms (%.1f Hz)", v, 1000.0f / (float)v);
            }
        } else if (strcmp(sub, "rate") == 0) {
            err = motion_set_rate_dps((float)v);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "速率上限 = %d °/s", v);
            }
        } else {
            err = motion_set_timeout_ms((uint32_t)v);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "命令超时 = %d ms %s", v, (v == 0) ? "(已关闭)" : "");
            }
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "参数不合法: %s", rest);
        }
        return;
    }

    ESP_LOGW(TAG, "未知 motion 子命令 '%s'", sub);
    app_motion_cmd_help();
}

/* ==========================================================================
 * stand / estop
 * ========================================================================== */

static void cmd_stand(const char *args)
{
    if (strcmp(args, "direct") == 0) {
        /* 标定用站姿：12 路 = 中位角（= 原版 servo_output 的 else 分支）。
         * ⚠️ 与原版正常站姿**不是同一个姿态**，见核对清单 E8。 */
        motion_set_mode(MOTION_MODE_POSE);
        const esp_err_t err = motion_set_target_stand();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "设置站姿失败: %s", esp_err_to_name(err));
            return;
        }
        ESP_LOGW(TAG, "标定用站姿（12 路 = 中位角）—— 这不是原版正常站立的样子");
    } else {
        /* 原版真正的站姿：走完整控制链（cal_ges -> IK -> servo_output） */
        const esp_err_t err = motion_set_mode(MOTION_MODE_CHAIN);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "切换 CHAIN 模式失败: %s", esp_err_to_name(err));
            return;
        }
        app_chain_stand();
        ESP_LOGI(TAG, "站立命令已下发（走控制链 = 原版真正的站姿）");
    }

    if (!motion_is_running()) {
        ESP_LOGW(TAG, "控制任务没在跑 —— `motion start` 后才会动");
    }

    motion_stats_t st;
    motion_get_stats(&st);
    for (uint8_t ch = 0; ch < SERVO_MAP_CHANNELS; ++ch) {
        ESP_LOGI(TAG, "  ch%-2u %-10s 目标=%7.2f", (unsigned)ch,
                 servo_map_channel_name(ch), (double)st.target[ch]);
    }
}

static void cmd_gait(const char *args)
{
    int g = -1;
    if (strcmp(args, "trot") == 0) {
        g = 0;
    } else if (strcmp(args, "walk") == 0) {
        g = 1;
    } else if (args[0] != '\0') {
        g = atoi(args);
    }
    if (g < 0 || g > 1) {
        ESP_LOGE(TAG, "用法: gait trot|walk");
        return;
    }
    const esp_err_t err = app_chain_set_gait(g);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "设置步态失败: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "步态 = %s（相位 t 已归零）", (g == 0) ? "TROT" : "WALK");
}

static void cmd_jog(const char *args)
{
    float spd = 0.0f;
    int L = 0, R = 0;
    const int n = sscanf(args, "%f %d %d", &spd, &L, &R);
    if (n < 1) {
        ESP_LOGE(TAG, "用法: jog <spd> [L] [R]   例: jog -3 1 1（前进）");
        return;
    }
    /* 原版 `move()`：只要有方向且有速度就切回 TROT 并把相位归零 */
    const esp_err_t err = app_chain_jog(spd, L, R);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "jog 失败: %s", esp_err_to_name(err));
        return;
    }
    if (motion_get_mode() != MOTION_MODE_CHAIN) {
        ESP_LOGW(TAG, "当前是 %s 模式 —— 请先 `motion mode chain`，否则命令不生效",
                 motion_mode_name(motion_get_mode()));
    }
    ESP_LOGI(TAG, "jog: spd=%.3f L=%d R=%d", (double)spd, L, R);
}

/**
 * `drive` —— 与 `jog` 只差一点：**不切步态**。
 *
 * ⚠️ 这是让狗以 **WALK** 走路唯一可行的一条路：`jog` 对应原版 `move()`，
 * 而 `move()` 内部会 `gait(0)` 把步态改回 TROT。
 * 用法：`motion mode chain` → `gait walk` → `drive -2 1 1`。
 */
static void cmd_drive(const char *args)
{
    float spd = 0.0f;
    int L = 0, R = 0;
    if (sscanf(args, "%f %d %d", &spd, &L, &R) < 1) {
        ESP_LOGE(TAG, "用法: drive <spd> [L] [R]   例: drive -2 1 1（WALK 前进）");
        return;
    }
    const esp_err_t err = app_chain_drive(spd, L, R);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "drive 失败: %s", esp_err_to_name(err));
        return;
    }
    app_chain_status_t cs;
    app_chain_get_status(&cs);
    ESP_LOGI(TAG, "drive: spd=%.3f L=%d R=%d（步态保持 %s）",
             (double)spd, L, R, (cs.gait_mode == 0) ? "TROT" : "WALK");
    if (motion_get_mode() != MOTION_MODE_CHAIN) {
        ESP_LOGW(TAG, "当前是 %s 模式 —— 请先 `motion mode chain`",
                 motion_mode_name(motion_get_mode()));
    }
}

static void cmd_turn(const char *args)
{
    const float pct = strtof(args, NULL);
    const esp_err_t err = app_chain_set_joy_turn(pct);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "turn 失败: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "转向 = %.1f%%（|pct| >= 10 时髋角才参与）", (double)pct);
}

static void cmd_chain(void)
{
    app_chain_status_t cs;
    app_chain_get_status(&cs);

    ESP_LOGI(TAG, "控制链: 模式=%s 有效=%s 链帧=%u 节拍=%u ms",
             motion_mode_name(motion_get_mode()),
             cs.valid ? "是" : "否", (unsigned)cs.frames, (unsigned)cs.period_ms);
    ESP_LOGI(TAG, "命令: gait=%s spd=%.3f L=%d R=%d 转向=%.1f%%",
             (cs.gait_mode == 0) ? "TROT" : "WALK", (double)cs.spd, cs.L, cs.R,
             (double)cs.joy_turn);
    ESP_LOGI(TAG, "目标: H=%.2f PIT=%.2f ROL=%.2f X=%.2f",
             (double)cs.goal[0], (double)cs.goal[1], (double)cs.goal[2], (double)cs.goal[3]);
    ESP_LOGI(TAG, "状态: t=%.4f R_H=%.2f PIT_S=%.2f ROL_S=%.2f X_S=%.2f",
             (double)cs.t, (double)cs.R_H, (double)cs.PIT_S, (double)cs.ROL_S, (double)cs.X_S);
    for (uint8_t ch = 0; ch < SERVO_MAP_CHANNELS; ++ch) {
        ESP_LOGI(TAG, "  ch%-2u %-10s 角度=%7.2f 占空比=%u", (unsigned)ch,
                 servo_map_channel_name(ch), (double)cs.angle_deg[ch],
                 (unsigned)servo_map_deg_to_duty(cs.angle_deg[ch]));
    }
}

static void cmd_estop(const char *args)
{
    /* 急停也要把动作取消：否则任务停了、动作还挂在"进行中"，
     * 下次 `motion start` 会从半个挥手脚本中间接着跑 */
    app_action_stop();
    motion_estop((args != NULL && *args != '\0') ? args : "控制台命令");
    vTaskDelay(pdMS_TO_TICKS(30));   /* 给任务一个周期去执行 */
    ESP_LOGW(TAG, "急停完成：动作已取消，12 路无脉冲（舵机松力），控制任务已停");
}

/* ==========================================================================
 * action ...  —— 姿态动画 / 动作层（control/action.c，容差 0 对照过 padog.py）
 * ========================================================================== */

/**
 * 把请求转给 `app_action`。角度只在 `MOTION_MODE_ACTION` 下才会被下发
 * （那个模式的运动任务每帧调 `app_action_step()`）。
 */
static void cmd_action(const char *args)
{
    app_action_kind_t kind;
    if (strcmp(args, "stand") == 0) {
        kind = APP_ACTION_STAND;
    } else if (strcmp(args, "sit") == 0) {
        kind = APP_ACTION_SIT;
    } else if (strcmp(args, "sit_direct") == 0) {
        kind = APP_ACTION_SIT_DIRECT;
    } else if (strcmp(args, "wave") == 0) {
        kind = APP_ACTION_WAVE;
    } else if (strcmp(args, "step") == 0) {
        kind = APP_ACTION_INPLACE_STEP;
    } else if (strcmp(args, "stop") == 0) {
        app_action_stop();
        ESP_LOGI(TAG, "动作已取消（注意：松力请用 `estop` 或 `motion stop`）");
        return;
    } else {
        ESP_LOGE(TAG, "用法: action stand|sit|sit_direct|wave|step|stop");
        return;
    }

    const esp_err_t err = app_action_request(kind);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "action %s 失败: %s", args, esp_err_to_name(err));
        return;
    }
    motion_keepalive();   /* 动作也是"有人在下命令" */

    if (motion_get_mode() != MOTION_MODE_ACTION) {
        ESP_LOGW(TAG, "当前是 %s 模式 —— 请先 `motion mode action`，否则动作不会下发角度",
                 motion_mode_name(motion_get_mode()));
    }
    if (!motion_is_running()) {
        ESP_LOGW(TAG, "控制任务没在跑 —— `motion start` 后动作才会动");
    }

    app_action_status_t as;
    app_action_get_status(&as);
    ESP_LOGI(TAG, "动作已排队: %s（已产生角度的帧数=%u，累计效果=%u）",
             app_action_kind_name(kind), (unsigned)as.steps, (unsigned)as.effects);
}

/* ==========================================================================
 * lg / lgtest / readback
 * ========================================================================== */

/** 运动任务在跑的时候不允许直接写 PWM（见 motion.h 的单写者不变式） */
static bool require_stopped(void)
{
    if (motion_is_running()) {
        ESP_LOGE(TAG, "控制任务正在运行 —— 请先 `motion stop` 或 `estop`，"
                      "否则两个任务会交错写同一条 I2C");
        return false;
    }
    return true;
}

static void cmd_lg(const char *args)
{
    if (!require_stopped()) {
        return;
    }

    char a0[24] = { 0 };
    const char *rest = args;
    size_t i = 0;
    while (rest[i] != '\0' && rest[i] != ' ' && i < sizeof(a0) - 1) {
        a0[i] = rest[i];
        ++i;
    }
    a0[i] = '\0';
    rest += i;
    while (*rest == ' ') {
        ++rest;
    }

    if (strcmp(a0, "off") == 0) {
        const esp_err_t err = servo_out_all_off();
        ESP_LOGI(TAG, "12 路松力 -> %s", esp_err_to_name(err));
        return;
    }

    const int ch = atoi(a0);
    if (ch < 0 || ch >= (int)SERVO_MAP_CHANNELS) {
        ESP_LOGE(TAG, "逻辑通道必须在 0..11（收到 '%s'）", a0);
        return;
    }
    if (rest[0] == '\0') {
        ESP_LOGE(TAG, "用法: lg <ch 0..11> <deg>  或  lg off");
        return;
    }

    const float deg = strtof(rest, NULL);

    /* 先把 12 路都拉成"当前缓存里的值"，只改这一路 —— 避免其它通道保持未知状态 */
    float all[SERVO_MAP_CHANNELS];
    motion_stats_t st;
    motion_get_stats(&st);
    memcpy(all, st.current, sizeof(all));   /* 未跑过任务时是无脉冲，用中位角更像样 */
    if (!st.running && st.ticks == 0) {
        const app_config_t *cfg = app_cfg_cmd_get();
        if (cfg != NULL) {
            for (uint8_t c = 0; c < SERVO_MAP_CHANNELS; ++c) {
                const int leg = (c < 3) ? 0 : (c < 6) ? 3 : (c < 9) ? 1 : 2;
                const int joint = c % 3;
                all[c] = cfg->servo_center[leg][joint];
            }
        }
    }
    all[ch] = deg;

    const esp_err_t err = servo_out_apply_deg(all);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "写失败: %s", esp_err_to_name(err));
        return;
    }
    const uint16_t duty = servo_map_deg_to_duty(deg);
    const servo_map_hw_t *hw = servo_map_hw((uint8_t)ch);
    ESP_LOGI(TAG, "逻辑通道 %d (%s) -> %.2f° = 占空比 %u -> 板 0x%02X ch%u",
             ch, servo_map_channel_name((uint8_t)ch), (double)deg, (unsigned)duty,
             hw->board_addr, hw->pca_ch);
}

static void cmd_lgtest(const char *args)
{
    if (!require_stopped()) {
        return;
    }

    const int ch = atoi(args);
    if (ch < 0 || ch >= (int)SERVO_MAP_CHANNELS) {
        ESP_LOGE(TAG, "用法: lgtest <ch 0..11> [delta_deg]   （默认偏移 %.0f°）",
                 (double)LG_TEST_DELTA_DEG);
        return;
    }

    const char *sp = strchr(args, ' ');
    const float delta = (sp != NULL) ? strtof(sp + 1, NULL) : LG_TEST_DELTA_DEG;

    const app_config_t *cfg = app_cfg_cmd_get();
    if (cfg == NULL) {
        ESP_LOGE(TAG, "配置未就绪");
        return;
    }

    /* 从当前姿态出发，只动这一路 */
    motion_stats_t st;
    motion_get_stats(&st);
    float all[SERVO_MAP_CHANNELS];
    for (uint8_t c = 0; c < SERVO_MAP_CHANNELS; ++c) {
        const int leg = (c < 3) ? 0 : (c < 6) ? 3 : (c < 9) ? 1 : 2;
        const int joint = c % 3;
        all[c] = (st.ticks > 0) ? st.current[c] : cfg->servo_center[leg][joint];
    }

    const float base = all[ch];
    const float up   = base + delta;
    const float down = base - delta;

    const servo_map_hw_t *hw = servo_map_hw((uint8_t)ch);
    ESP_LOGW(TAG, "=== 单通道测试 ch%d (%s) ===  硬件: 板 0x%02X ch%u",
             ch, servo_map_channel_name((uint8_t)ch), hw->board_addr, hw->pca_ch);
    ESP_LOGW(TAG, "基准 %.2f° -> 先 %.2f° (%u)，再 %.2f° (%u)，最后回 %.2f° (%u)",
             (double)base, (double)up, (unsigned)servo_map_deg_to_duty(up),
             (double)down, (unsigned)servo_map_deg_to_duty(down),
             (double)base, (unsigned)servo_map_deg_to_duty(base));
    ESP_LOGW(TAG, "请观察是**哪个关节**在动、方向如何，记下来填进硬件核对清单");

    const float seq[3] = { up, down, base };
    for (int i = 0; i < 3; ++i) {
        all[ch] = seq[i];
        const esp_err_t err = servo_out_apply_deg(all);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "写失败: %s", esp_err_to_name(err));
            return;
        }
        ESP_LOGI(TAG, "  第 %d/3 步: ch%d = %.2f°", i + 1, ch, (double)seq[i]);
        vTaskDelay(pdMS_TO_TICKS(LG_TEST_HOLD_MS));
    }
    ESP_LOGW(TAG, "=== 测试结束，ch%d 已回基准 %.2f° ===", ch, (double)base);
}

static void cmd_readback(void)
{
    ESP_LOGI(TAG, "回读 12 路寄存器：");
    int bad = 0;
    for (uint8_t ch = 0; ch < SERVO_MAP_CHANNELS; ++ch) {
        uint16_t on = 0, off = 0;
        const esp_err_t err = servo_out_readback(ch, &on, &off);
        const servo_map_hw_t *hw = servo_map_hw(ch);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "  ch%-2u %-10s 0x%02X ch%u 读失败: %s", (unsigned)ch,
                     servo_map_channel_name(ch), hw->board_addr, hw->pca_ch,
                     esp_err_to_name(err));
            ++bad;
            continue;
        }
        ESP_LOGI(TAG, "  ch%-2u %-10s 0x%02X ch%u  ON=%u OFF=%u%s", (unsigned)ch,
                 servo_map_channel_name(ch), hw->board_addr, hw->pca_ch,
                 (unsigned)on, (unsigned)off,
                 (off == 4096) ? "  (无脉冲/松力)" : "");
    }
    ESP_LOGI(TAG, "读回完成，失败 %d 路", bad);
}

/* ========================================================================== */

void app_motion_cmd_handle(const char *cmd, const char *args)
{
    if (cmd == NULL) {
        return;
    }
    if (strcmp(cmd, "motion") == 0) {
        cmd_motion(args);
    } else if (strcmp(cmd, "stand") == 0) {
        cmd_stand(args);
    } else if (strcmp(cmd, "gait") == 0) {
        cmd_gait(args);
    } else if (strcmp(cmd, "jog") == 0) {
        cmd_jog(args);
    } else if (strcmp(cmd, "drive") == 0) {
        cmd_drive(args);
    } else if (strcmp(cmd, "turn") == 0) {
        cmd_turn(args);
    } else if (strcmp(cmd, "chain") == 0) {
        cmd_chain();
    } else if (strcmp(cmd, "estop") == 0) {
        cmd_estop(args);
    } else if (strcmp(cmd, "action") == 0) {
        cmd_action(args);
    } else if (strcmp(cmd, "lg") == 0) {
        cmd_lg(args);
    } else if (strcmp(cmd, "lgtest") == 0) {
        cmd_lgtest(args);
    } else if (strcmp(cmd, "readback") == 0) {
        cmd_readback();
    }
}

void app_motion_cmd_init(void)
{
    if (servo_out_init() != ESP_OK) {
        ESP_LOGE(TAG, "舵机输出层初始化失败");
        return;
    }
    if (motion_init() != ESP_OK) {
        ESP_LOGE(TAG, "运动模块初始化失败");
        return;
    }
    /* 控制链的应用层封装。必须在 app_cfg_cmd_init() 之后（要读配置）。 */
    if (app_chain_init() != ESP_OK) {
        ESP_LOGE(TAG, "控制链初始化失败");
        return;
    }
    /* 姿态动画 / 动作层。同样要在 app_cfg_cmd_init() 之后（要读配置里的中位角）。 */
    if (app_action_init() != ESP_OK) {
        ESP_LOGE(TAG, "动作层初始化失败");
        return;
    }
    /* 默认 POSE 模式：上电不会自己走 */
    (void)motion_set_mode(MOTION_MODE_POSE);

    /* 上电自检阶段就把 12 路确认成"无脉冲"，并且**不**自动启动控制任务 */
    const esp_err_t err = servo_out_all_off();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "上电置安全态时有通道失败: %s", esp_err_to_name(err));
    }

    motion_stats_t st;
    motion_get_stats(&st);
    app_chain_status_t cs;
    app_chain_get_status(&cs);
    ESP_LOGI(TAG, "P2/P3 就绪：12 路无脉冲（舵机松力），控制任务未启动，模式 POSE");
    ESP_LOGI(TAG, "  运动任务: 周期 %.0f ms / 速率 %.0f °/s / 超时 %u ms",
             (double)st.period_ms, (double)st.rate_dps, (unsigned)st.timeout_ms);
    ESP_LOGI(TAG, "  控制链:   节拍 %u ms（原版 ~65 ms 主循环的等价物，见迁移表 §8.11）",
             (unsigned)cs.period_ms);
    ESP_LOGI(TAG, "  站立: `stand` + `motion start`（走控制链，原版真正的站姿）");
    ESP_LOGI(TAG, "  走路: `motion mode chain` + `gait trot` + `jog -3 1 1` + `motion start`");
    ESP_LOGI(TAG, "  动作: `motion mode action` + `action sit|stand|wave|step` + `motion start`");
}
