/**
 * @file    app_p0.c
 * @brief   P0 阶段实现：I2C 扫描 + PCA9685 双板初始化 + 单通道舵机控制台
 *
 * 设计原则（对齐 ESP-IDF_C迁移表.md 的 P0）：
 *   1. 上电只做「读」与「进入安全态」，绝不自动驱动舵机。
 *   2. 所有舵机动作必须由串口命令显式触发（便于先架空/上支架再测）。
 *   3. 脉宽强制限制在 [500, 2500] µs。
 *   4. 每个关键步骤都打日志，便于和 MicroPython 版本对照。
 */

#include "app/app_p0.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp/bsp_i2c.h"
#include "driver/uart.h"
#include "drivers/drv_pca9685.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "p0";

/** 串口控制台相关 */
#define P0_CONSOLE_UART      UART_NUM_0
#define P0_CONSOLE_BAUD      115200
#define P0_LINE_MAX          96
#define P0_SWEEP_MAX_TOTAL_MS 30000

/** 两块驱动板的地址表，索引 0 -> 0x40，1 -> 0x41 */
static const uint8_t s_boards[] = {DRV_PCA9685_ADDR_LEFT, DRV_PCA9685_ADDR_RIGHT};
#define P0_BOARD_COUNT (sizeof(s_boards) / sizeof(s_boards[0]))

/* ==========================================================================
 * 工具
 * ========================================================================== */

static bool board_addr_from_index(int idx, uint8_t *out)
{
    if (idx < 0 || (size_t)idx >= P0_BOARD_COUNT) {
        return false;
    }
    *out = s_boards[idx];
    return true;
}

static void log_separator(void)
{
    ESP_LOGI(TAG, "------------------------------------------------------------");
}

/* ==========================================================================
 * I2C 扫描
 * ========================================================================== */

static size_t do_scan(bool verbose)
{
    uint8_t found[32] = {0};
    const size_t n = bsp_i2c_scan(found, sizeof(found));

    if (n == 0) {
        ESP_LOGW(TAG, "扫描结果：总线上没有任何器件");
        return 0;
    }

    if (verbose) {
        char line[160] = {0};
        size_t used = 0;
        for (size_t i = 0; i < n && i < sizeof(found); ++i) {
            int wrote = snprintf(&line[used], sizeof(line) - used,
                                 "%s0x%02X", (used == 0) ? "" : ", ", found[i]);
            if (wrote <= 0 || (size_t)wrote >= sizeof(line) - used) {
                break;
            }
            used += (size_t)wrote;
        }
        ESP_LOGI(TAG, "扫描结果（%u 个）：%s", (unsigned)n, line);
    }
    return n;
}

/**
 * @brief 按迁移表的标准核对：0x40 与 0x41 必须出现。
 */
static bool verify_expected_devices(void)
{
    bool ok40 = (bsp_i2c_probe(DRV_PCA9685_ADDR_LEFT) == ESP_OK);
    bool ok41 = (bsp_i2c_probe(DRV_PCA9685_ADDR_RIGHT) == ESP_OK);

    ESP_LOGI(TAG, "核对 0x40 (左半身) : %s", ok40 ? "存在 ✔" : "缺失 ✘");
    ESP_LOGI(TAG, "核对 0x41 (右半身) : %s", ok41 ? "存在 ✔" : "缺失 ✘");

    if (bsp_i2c_probe(DRV_PCA9685_ADDR_ALLCALL) == ESP_OK) {
        ESP_LOGI(TAG, "0x70 也在线 —— 这是 PCA9685 的 all-call 广播地址，不是 IMU");
    }
    /* 板上无 IMU（2026-09-14 全引脚扫描确认）。这里顺带再看一眼，便于记录。 */
    if (bsp_i2c_probe(0x68) == ESP_OK || bsp_i2c_probe(0x69) == ESP_OK) {
        ESP_LOGW(TAG, "意外发现 0x68/0x69 —— 说明新增了 IMU，需更新迁移计划");
    } else {
        ESP_LOGI(TAG, "0x68/0x69 无应答：与实测一致（本机没有 IMU）");
    }

    return ok40 && ok41;
}

/* ==========================================================================
 * 自检
 * ========================================================================== */

static void report_board_state(uint8_t addr)
{
    uint8_t mode1 = 0, prescale = 0;

    if (drv_pca9685_read_reg(addr, DRV_PCA9685_REG_MODE1, &mode1) == ESP_OK &&
        drv_pca9685_read_reg(addr, DRV_PCA9685_REG_PRESCALE, &prescale) == ESP_OK) {
        ESP_LOGI(TAG, "0x%02X MODE1=0x%02X  PRESCALE=%u  (实约 %.2f Hz)",
                 addr, mode1, prescale, drv_pca9685_actual_freq(prescale));

        /* 与 MicroPython 版实测值对照：MODE1=0x21, PRESCALE=122 */
        if (prescale == 122) {
            ESP_LOGI(TAG, "     PRESCALE=122 与 MicroPython 版实测一致 ✔");
        } else {
            ESP_LOGW(TAG, "     PRESCALE=%u 与 MicroPython 版实测(122)不同", prescale);
        }
    } else {
        ESP_LOGE(TAG, "0x%02X 读 MODE1/PRESCALE 失败", addr);
    }
}

static void selftest(void)
{
    log_separator();
    ESP_LOGI(TAG, "步骤 1/4：初始化 I2C 总线");
    if (bsp_i2c_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C 初始化失败，P0 自检中止");
        return;
    }

    log_separator();
    ESP_LOGI(TAG, "步骤 2/4：扫描总线");
    do_scan(true);
    if (!verify_expected_devices()) {
        ESP_LOGE(TAG, "期望的 PCA9685 未全部出现，请检查供电/接线/共地，自检中止");
        return;
    }

    log_separator();
    ESP_LOGI(TAG, "步骤 3/4：初始化两片 PCA9685 并置为安全态（所有通道无脉冲）");
    for (size_t i = 0; i < P0_BOARD_COUNT; ++i) {
        esp_err_t err = drv_pca9685_init(s_boards[i], DRV_PCA9685_DEFAULT_HZ);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "0x%02X 初始化失败: %s", s_boards[i], esp_err_to_name(err));
            return;
        }
    }

    log_separator();
    ESP_LOGI(TAG, "步骤 4/4：回读校验");
    for (size_t i = 0; i < P0_BOARD_COUNT; ++i) {
        report_board_state(s_boards[i]);
    }

    log_separator();
    ESP_LOGI(TAG, "P0 自检完成。⚠️ 舵机当前处于「无脉冲/松力」状态，尚未动作。");
}

/* ==========================================================================
 * 命令处理
 * ========================================================================== */

static void cmd_help(void)
{
    ESP_LOGI(TAG, "可用命令（board: 0=0x40 左, 1=0x41 右）：");
    ESP_LOGI(TAG, "  help                          显示本帮助");
    ESP_LOGI(TAG, "  scan                          重新扫描 I2C 总线");
    ESP_LOGI(TAG, "  status                        回读两片板的 MODE1/PRESCALE");
    ESP_LOGI(TAG, "  freq <hz>                     设置频率（默认 50）");
    ESP_LOGI(TAG, "  set <board> <ch> <us>         单通道输出指定脉宽 (500..2500)");
    ESP_LOGI(TAG, "  all <board> <us>              该板所有 16 路输出同一脉宽");
    ESP_LOGI(TAG, "  off <board>                   该板所有通道无脉冲（松力/安全态）");
    ESP_LOGI(TAG, "  sweep <board> <ch> <from> <to> <step> <delay_ms>   慢速往返扫动");
    ESP_LOGI(TAG, "  raw <board> <reg_hex>         读一个寄存器（调试用）");
    ESP_LOGI(TAG, "  deg <board> <ch> <0..180>     按 MicroPython 的换算输出对应脉宽");
}

static void cmd_status(void)
{
    for (size_t i = 0; i < P0_BOARD_COUNT; ++i) {
        ESP_LOGI(TAG, "--- board %u = 0x%02X ---", (unsigned)i, s_boards[i]);
        report_board_state(s_boards[i]);
    }
    ESP_LOGI(TAG, "当前标称频率: %.2f Hz", drv_pca9685_get_freq());
}

static void cmd_freq(const char *args)
{
    float hz = 0.0f;
    if (sscanf(args, "%f", &hz) != 1 || hz < 24.0f || hz > 1526.0f) {
        ESP_LOGW(TAG, "用法: freq <hz>，hz 需在 24..1526");
        return;
    }
    for (size_t i = 0; i < P0_BOARD_COUNT; ++i) {
        drv_pca9685_set_freq(s_boards[i], hz);
    }
    ESP_LOGW(TAG, "频率已改。注意：改频率会改变所有通道的脉宽含义，先 off 再 set");
}

static void cmd_set(const char *args)
{
    int board = 0, ch = 0, us = 0;
    if (sscanf(args, "%d %d %d", &board, &ch, &us) != 3) {
        ESP_LOGW(TAG, "用法: set <board> <ch> <us>");
        return;
    }
    uint8_t addr = 0;
    if (!board_addr_from_index(board, &addr)) {
        ESP_LOGW(TAG, "board 只能是 0 或 1");
        return;
    }
    if (ch < 0 || ch >= DRV_PCA9685_CHANNELS) {
        ESP_LOGW(TAG, "ch 需在 0..%d", DRV_PCA9685_CHANNELS - 1);
        return;
    }

    esp_err_t err = drv_pca9685_set_channel_us(addr, (uint8_t)ch, (uint16_t)us);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "0x%02X ch%d <- %d us", addr, ch, us);
    } else {
        ESP_LOGE(TAG, "设置失败: %s", esp_err_to_name(err));
    }
}

static void cmd_deg(const char *args)
{
    int board = 0, ch = 0, deg = 0;
    if (sscanf(args, "%d %d %d", &board, &ch, &deg) != 3) {
        ESP_LOGW(TAG, "用法: deg <board> <ch> <0..180>");
        return;
    }
    if (deg < 0 || deg > 180) {
        ESP_LOGW(TAG, "角度需在 0..180");
        return;
    }
    const uint16_t us = drv_pca9685_us_from_degrees_ref((uint16_t)deg);
    ESP_LOGI(TAG, "角度 %d° -> %u us（复刻 MicroPython 换算）", deg, us);
    char tmp[32];
    snprintf(tmp, sizeof(tmp), "%d %d %u", board, ch, us);
    cmd_set(tmp);
}

static void cmd_all(const char *args)
{
    int board = 0, us = 0;
    if (sscanf(args, "%d %d", &board, &us) != 2) {
        ESP_LOGW(TAG, "用法: all <board> <us>");
        return;
    }
    uint8_t addr = 0;
    if (!board_addr_from_index(board, &addr)) {
        ESP_LOGW(TAG, "board 只能是 0 或 1");
        return;
    }
    ESP_LOGW(TAG, "即将把 0x%02X 的全部 16 路设为 %d us —— 会同时驱动该板所有舵机！",
             addr, us);
    esp_err_t err = drv_pca9685_set_all_us(addr, (uint16_t)us);
    ESP_LOGI(TAG, "结果: %s", esp_err_to_name(err));
}

static void cmd_off(const char *args)
{
    int board = 0;
    if (sscanf(args, "%d", &board) != 1) {
        ESP_LOGW(TAG, "用法: off <board>");
        return;
    }
    uint8_t addr = 0;
    if (!board_addr_from_index(board, &addr)) {
        ESP_LOGW(TAG, "board 只能是 0 或 1");
        return;
    }
    esp_err_t err = drv_pca9685_all_off(addr);
    ESP_LOGI(TAG, "0x%02X 全部通道已置为无脉冲: %s", addr, esp_err_to_name(err));
}

static void cmd_sweep(const char *args)
{
    int board = 0, ch = 0, from = 0, to = 0, step = 0, delay_ms = 0;
    if (sscanf(args, "%d %d %d %d %d %d", &board, &ch, &from, &to, &step, &delay_ms) != 6) {
        ESP_LOGW(TAG, "用法: sweep <board> <ch> <from> <to> <step> <delay_ms>");
        return;
    }
    uint8_t addr = 0;
    if (!board_addr_from_index(board, &addr)) {
        ESP_LOGW(TAG, "board 只能是 0 或 1");
        return;
    }
    if (ch < 0 || ch >= DRV_PCA9685_CHANNELS) {
        ESP_LOGW(TAG, "ch 需在 0..%d", DRV_PCA9685_CHANNELS - 1);
        return;
    }
    if (from < DRV_PCA9685_US_MIN || to > DRV_PCA9685_US_MAX ||
        from > to || step <= 0 || delay_ms < 0) {
        ESP_LOGW(TAG, "参数不合法：需 %d<=from<=to<=%d, step>0, delay_ms>=0",
                 DRV_PCA9685_US_MIN, DRV_PCA9685_US_MAX);
        return;
    }

    const int steps = (to - from) / step;
    const long total_ms = (long)steps * 2L * (long)delay_ms;
    if (total_ms > P0_SWEEP_MAX_TOTAL_MS) {
        ESP_LOGW(TAG, "预计耗时 %ld ms 超过上限 %d ms，已拒绝（先减小范围或步数）",
                 total_ms, P0_SWEEP_MAX_TOTAL_MS);
        return;
    }

    ESP_LOGW(TAG, "开始扫动 0x%02X ch%d：%d -> %d -> %d us，预计 %ld ms。",
             addr, ch, from, to, from, total_ms);
    ESP_LOGW(TAG, "请确认该关节已架空或上支架！");

    for (int us = from; us <= to; us += step) {
        drv_pca9685_set_channel_us(addr, (uint8_t)ch, (uint16_t)us);
        if (delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
    for (int us = to; us >= from; us -= step) {
        drv_pca9685_set_channel_us(addr, (uint8_t)ch, (uint16_t)us);
        if (delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
    ESP_LOGI(TAG, "扫动结束，已回到 %d us", from);
}

static void cmd_raw(const char *args)
{
    int board = 0;
    unsigned reg = 0;
    if (sscanf(args, "%d %x", &board, &reg) != 2 || reg > 0xFF) {
        ESP_LOGW(TAG, "用法: raw <board> <reg_hex>  例如 raw 0 0");
        return;
    }
    uint8_t addr = 0;
    if (!board_addr_from_index(board, &addr)) {
        ESP_LOGW(TAG, "board 只能是 0 或 1");
        return;
    }
    uint8_t val = 0;
    esp_err_t err = drv_pca9685_read_reg(addr, (uint8_t)reg, &val);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "0x%02X reg[0x%02X] = 0x%02X (%u)", addr, (unsigned)reg, val, val);
    } else {
        ESP_LOGE(TAG, "读失败: %s", esp_err_to_name(err));
    }
}

static void handle_line(char *line)
{
    /* 取第一个词作为命令 */
    char *cmd = line;
    while (*cmd == ' ') {
        ++cmd;
    }
    char *args = strchr(cmd, ' ');
    if (args != NULL) {
        *args = '\0';
        ++args;
        while (*args == ' ') {
            ++args;
        }
    } else {
        args = cmd + strlen(cmd);
    }

    if (*cmd == '\0') {
        return;
    }

    ESP_LOGI(TAG, "> %s", cmd);

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        cmd_help();
    } else if (strcmp(cmd, "scan") == 0) {
        do_scan(true);
        verify_expected_devices();
    } else if (strcmp(cmd, "status") == 0) {
        cmd_status();
    } else if (strcmp(cmd, "freq") == 0) {
        cmd_freq(args);
    } else if (strcmp(cmd, "set") == 0) {
        cmd_set(args);
    } else if (strcmp(cmd, "deg") == 0) {
        cmd_deg(args);
    } else if (strcmp(cmd, "all") == 0) {
        cmd_all(args);
    } else if (strcmp(cmd, "off") == 0) {
        cmd_off(args);
    } else if (strcmp(cmd, "sweep") == 0) {
        cmd_sweep(args);
    } else if (strcmp(cmd, "raw") == 0) {
        cmd_raw(args);
    } else {
        ESP_LOGW(TAG, "未知命令 '%s'，输入 help 查看用法", cmd);
    }
}

/* ==========================================================================
 * 控制台任务
 * ========================================================================== */

static void console_task(void *arg)
{
    (void)arg;

    const uart_config_t cfg = {
        .baud_rate  = P0_CONSOLE_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(P0_CONSOLE_UART, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uart_driver_install 失败: %s，控制台不可用", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    uart_param_config(P0_CONSOLE_UART, &cfg);

    ESP_LOGI(TAG, "控制台就绪，输入 help 回车查看命令。");

    char line[P0_LINE_MAX];
    size_t len = 0;

    for (;;) {
        uint8_t c = 0;
        const int n = uart_read_bytes(P0_CONSOLE_UART, &c, 1, pdMS_TO_TICKS(100));
        if (n != 1) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (len > 0) {
                line[len] = '\0';
                handle_line(line);
                len = 0;
            }
        } else if (c == 0x08 || c == 0x7F) { /* 退格 */
            if (len > 0) {
                --len;
            }
        } else if (len < P0_LINE_MAX - 1) {
            line[len++] = (char)c;
        }
    }
}

/* ==========================================================================
 * 入口
 * ========================================================================== */

void app_p0_start(void)
{
    ESP_LOGI(TAG, "=========== P0：I2C 扫描 + PCA9685 单通道控制 ===========");
    selftest();

    ESP_LOGW(TAG, "⚠️ 安全提示：测试舵机前请先架空狗腿或使用支架；");
    ESP_LOGW(TAG, "   `all` 命令会同时驱动该板全部 16 路，慎用。");

    xTaskCreate(console_task, "p0_console", 4096, NULL, 5, NULL);
}
