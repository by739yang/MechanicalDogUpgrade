/**
 * @file    app_motion_cmd.c
 * @brief   P2 串口控制台命令实现
 */

#include "app/app_motion_cmd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
    ESP_LOGI(TAG, "  motion rate <deg_per_s>      速率上限（默认 120）");
    ESP_LOGI(TAG, "  motion timeout <ms>          命令超时后松力停车（默认 10000，0=关闭）");
    ESP_LOGI(TAG, "  stand                        目标设为直接站姿（限速走过去）");
    ESP_LOGI(TAG, "  estop [reason]               急停：12 路松力并停任务");
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
        motion_stop(MOTION_STOP_USER);
        ESP_LOGI(TAG, "已停止，12 路松力");
        return;
    }

    if (strcmp(sub, "period") == 0 || strcmp(sub, "rate") == 0 || strcmp(sub, "timeout") == 0) {
        const int v = atoi(rest);
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

static void cmd_stand(void)
{
    const esp_err_t err = motion_set_target_stand();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "设置站姿失败: %s", esp_err_to_name(err));
        return;
    }

    motion_stats_t st;
    motion_get_stats(&st);
    ESP_LOGI(TAG, "站姿目标已下发（12 路 = 中位角）。当前角度：");
    for (uint8_t ch = 0; ch < SERVO_MAP_CHANNELS; ++ch) {
        ESP_LOGI(TAG, "  ch%-2u %-10s 目标=%7.2f", (unsigned)ch,
                 servo_map_channel_name(ch), (double)st.target[ch]);
    }
    if (!motion_is_running()) {
        ESP_LOGW(TAG, "控制任务没在跑 —— 目标已记下，`motion start` 后才会动");
    }
}

static void cmd_estop(const char *args)
{
    motion_estop((args != NULL && *args != '\0') ? args : "控制台命令");
    vTaskDelay(pdMS_TO_TICKS(30));   /* 给任务一个周期去执行 */
    ESP_LOGW(TAG, "急停完成：12 路无脉冲（舵机松力），控制任务已停");
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
        cmd_stand();
    } else if (strcmp(cmd, "estop") == 0) {
        cmd_estop(args);
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

    /* 上电自检阶段就把 12 路确认成"无脉冲"，并且**不**自动启动控制任务 */
    const esp_err_t err = servo_out_all_off();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "上电置安全态时有通道失败: %s", esp_err_to_name(err));
    }

    motion_stats_t st;
    motion_get_stats(&st);
    ESP_LOGI(TAG, "P2 就绪：12 路无脉冲（舵机松力），控制任务未启动");
    ESP_LOGI(TAG, "  默认: 周期 %.0f ms / 速率 %.0f °/s / 超时 %u ms；"
                  "`stand` + `motion start` 开始站立测试",
             (double)st.period_ms, (double)st.rate_dps, (unsigned)st.timeout_ms);
}
