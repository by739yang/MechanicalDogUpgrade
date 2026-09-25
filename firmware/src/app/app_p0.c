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
#include "app/app_cfg_cmd.h"
#include "app/app_motion_cmd.h"
#include "app/motion.h"
#include "app/servo_out.h"

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

/**
 * @brief 全总线扫描（位操作实现，绕开驱动的 1000 ms/失败地址 限制）。
 *
 * 位操作需要独占 GPIO，所以先卸载 I2C 驱动，扫完立刻装回来。
 * 这个装卸**不影响 PCA9685**：它是自主输出 PWM 的从设备，寄存器状态不变，
 * 舵机脉冲不会中断。
 *
 * @return 命中个数
 */
static size_t do_scan(void)
{
    uint8_t found[32] = {0};
    size_t n = 0;

    bsp_i2c_deinit();                       /* 让出 GPIO */

    if (bsp_i2c_bitbang_begin() == ESP_OK) {
        n = bsp_i2c_scan_bitbang(found, sizeof(found));
    } else {
        ESP_LOGE(TAG, "位操作初始化失败");
    }

    if (bsp_i2c_init() != ESP_OK) {         /* 装回来，后续指令要用 */
        ESP_LOGE(TAG, "重新安装 I2C 驱动失败");
        return 0;
    }

    if (n == 0) {
        ESP_LOGW(TAG, "扫描结果：总线上没有任何器件");
        return 0;
    }

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
    return n;
}

/**
 * @brief 按迁移表的标准核对：0x40 与 0x41 必须出现。
 *
 * 这里用**驱动路径**再确认一次（而不是只信位操作扫描）——意义不同：
 * 位操作证明"器件在总线上"，驱动读通才证明"后续读写能正常工作"。
 *
 * @note 故意**不**在这里探测 0x68/0x69：器件不存在时每次要 1000 ms，
 *       而"有没有 IMU"已经由位操作全扫描回答了。
 */
static bool verify_expected_devices(void)
{
    const bool ok40 = (bsp_i2c_probe(DRV_PCA9685_ADDR_LEFT) == ESP_OK);
    const bool ok41 = (bsp_i2c_probe(DRV_PCA9685_ADDR_RIGHT) == ESP_OK);

    ESP_LOGI(TAG, "核对 0x40 (左半身) : %s", ok40 ? "存在 ✔" : "缺失 ✘");
    ESP_LOGI(TAG, "核对 0x41 (右半身) : %s", ok41 ? "存在 ✔" : "缺失 ✘");

    if (bsp_i2c_probe(DRV_PCA9685_ADDR_ALLCALL) == ESP_OK) {
        ESP_LOGI(TAG, "0x70 也在线 —— 这是 PCA9685 的 all-call 广播地址，不是 IMU");
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

/**
 * @brief 探测单个地址并打印结论与耗时。
 *
 * 打印耗时很关键：若"不存在"的地址探测耗时达到几百毫秒，
 * 说明是总线被拉低 / 从设备在拉伸时钟，而不是代码死循环。
 */
static bool probe_verbose(uint8_t addr, const char *what)
{
    int64_t us = 0;
    const esp_err_t err = bsp_i2c_probe_timed(addr, &us);
    const bool ok = (err == ESP_OK);
    if (ok) {
        ESP_LOGI(TAG, "  0x%02X  %-26s -> 应答 ✔   (%lld us)",
                 addr, what, (long long)us);
    } else {
        ESP_LOGW(TAG, "  0x%02X  %-26s -> 无应答  (%lld us)",
                 addr, what, (long long)us);
    }
    return ok;
}

static void selftest(void)
{
    /* 配置最先加载：后面的行为都依赖它（中位角、步态参数、限幅）。
       注意 P0 阶段只**读**配置、打印出来，还不用它驱动舵机。 */
    log_separator();
    ESP_LOGI(TAG, "步骤 1/6：加载配置（NVS）");
    app_cfg_cmd_init();
    {
        const app_config_t *c = app_cfg_cmd_get();
        ESP_LOGI(TAG, "  AP 热点: ssid=\"%s\"  密码长度 %u",
                 c->ap_ssid, (unsigned)strlen(c->ap_password));
        ESP_LOGI(TAG, "  中位角 腿1(左前) 髋/大/小 = %.1f / %.1f / %.1f",
                 (double)c->servo_center[0][0], (double)c->servo_center[0][1],
                 (double)c->servo_center[0][2]);
        ESP_LOGI(TAG, "  提示：敲 `cfg` 看全部，`cfg info` 看状态，`cfg list` 看字段名");
    }

    log_separator();
    ESP_LOGI(TAG, "步骤 2/6：I2C 总线恢复（9 个 SCL 脉冲，防止从设备卡住 SDA）");
    bsp_i2c_bus_recover();

    /* 全总线扫描放在驱动安装之前，用位操作做。
       原因：IDF 5.1.2 的 legacy 驱动把事件等待下限硬编码成 1000 ms，
       探测不存在的地址每次要 1 秒（112 个地址 = 112 秒）。位操作只要 ~20 ms。
       这一步同时给出"总线上到底有什么"的完整清单 —— 包括有没有 IMU。 */
    log_separator();
    ESP_LOGI(TAG, "步骤 3/6：位操作全总线扫描（不依赖驱动，约 20 ms）");
    const size_t n_found = do_scan();

    log_separator();
    ESP_LOGI(TAG, "步骤 4/6：初始化 I2C 驱动");
    if (bsp_i2c_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C 初始化失败，P0 自检中止");
        return;
    }

    /* 用驱动路径再确认三个关键地址：证明后续读写能正常工作。
       只探这 3 个（都存在），所以这一步是"零成本"的。 */
    log_separator();
    ESP_LOGI(TAG, "步骤 5/6：驱动路径定向探测（只探存在的 3 个地址，零等待）");
    const bool ok40 = probe_verbose(DRV_PCA9685_ADDR_LEFT, "PCA9685 左半身");
    const bool ok41 = probe_verbose(DRV_PCA9685_ADDR_RIGHT, "PCA9685 右半身");
    probe_verbose(DRV_PCA9685_ADDR_ALLCALL, "PCA9685 all-call 广播");

    if (!ok40 || !ok41) {
        ESP_LOGE(TAG, "期望的 PCA9685 未全部应答 —— 常见原因：");
        ESP_LOGE(TAG, "  1) PCA9685 的逻辑电源未供电（本机实测走 USB 5V，一般不是这个）");
        ESP_LOGE(TAG, "  2) SDA/SCL 接线错误，或未共地；");
        ESP_LOGE(TAG, "  3) 模块损坏。");
        ESP_LOGE(TAG, "自检在此中止，跳过 PCA9685 初始化。控制台仍会启动，可敲 scan 重试。");
        return;
    }
    ESP_LOGI(TAG, "两片 PCA9685 都在线 ✔（位操作扫描共发现 %u 个器件）", (unsigned)n_found);

    log_separator();
    ESP_LOGI(TAG, "步骤 6/7：初始化两片 PCA9685（置安全态）+ 回读校验");
    for (size_t i = 0; i < P0_BOARD_COUNT; ++i) {
        esp_err_t err = drv_pca9685_init(s_boards[i], DRV_PCA9685_DEFAULT_HZ);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "0x%02X 初始化失败: %s", s_boards[i], esp_err_to_name(err));
            return;
        }
    }
    for (size_t i = 0; i < P0_BOARD_COUNT; ++i) {
        report_board_state(s_boards[i]);
    }

    log_separator();
    ESP_LOGI(TAG, "步骤 7/7：舵机输出层 + 运动模块（P2）");
    app_motion_cmd_init();

    log_separator();
    ESP_LOGI(TAG, "自检完成。⚠️ 12 路舵机处于「无脉冲/松力」状态，控制任务未启动。");
}

/* ==========================================================================
 * 命令处理
 * ========================================================================== */

static void cmd_help(void)
{
    ESP_LOGI(TAG, "可用命令（board: 0=0x40 左, 1=0x41 右）：");
    ESP_LOGI(TAG, "  help                          显示本帮助");
    ESP_LOGI(TAG, "  scan                          全总线扫描 0x08..0x77（位操作，约 20 ms）");
    ESP_LOGI(TAG, "  status                        回读两片板的 MODE1/PRESCALE");
    ESP_LOGI(TAG, "  freq <hz>                     设置频率（默认 50）");
    ESP_LOGI(TAG, "  set <board> <ch> <us>         单通道输出指定脉宽 (500..2500)");
    ESP_LOGI(TAG, "  all <board> <us>              该板所有 16 路输出同一脉宽");
    ESP_LOGI(TAG, "  off <board>                   该板所有通道无脉冲（松力/安全态）");
    ESP_LOGI(TAG, "  sweep <board> <ch> <from> <to> <step> <delay_ms>   慢速往返扫动");
    ESP_LOGI(TAG, "  raw <board> <reg_hex>         读一个寄存器（调试用）");
    ESP_LOGI(TAG, "  deg <board> <ch> <0..180>     按 MicroPython 的换算输出对应脉宽");
    ESP_LOGI(TAG, "  ---- 配置（NVS 持久化）----");
    ESP_LOGI(TAG, "  cfg                           打印当前全部配置");
    ESP_LOGI(TAG, "  cfg info                      版本 / CRC / 结构大小 / NVS 用量");
    ESP_LOGI(TAG, "  cfg list                      列出所有可设的字段名");
    ESP_LOGI(TAG, "  cfg get <name>                读单项");
    ESP_LOGI(TAG, "  cfg set <name> <value>        改单项（立即校验限幅）");
    ESP_LOGI(TAG, "  cfg save                      写入 NVS（掉电重启仍生效）");
    ESP_LOGI(TAG, "  cfg load                      从 NVS 重新读取");
    ESP_LOGI(TAG, "  cfg reset                     恢复出厂默认并擦除 NVS");
    app_motion_cmd_help();
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
    /* 先把整行原样留一份用于回显 —— 下面的拆分会把参数位置改成 '\0'，
       否则日志里只会看到 "> raw" 而看不到 "raw 0 FE"。 */
    char echo[P0_LINE_MAX];
    strncpy(echo, line, sizeof(echo) - 1);
    echo[sizeof(echo) - 1] = '\0';

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

    ESP_LOGI(TAG, "> %s", echo);

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        cmd_help();
    } else if (strcmp(cmd, "scan") == 0) {
        do_scan();
        verify_expected_devices();
    } else if (strcmp(cmd, "status") == 0) {
        cmd_status();
    } else if (strcmp(cmd, "freq") == 0) {
        cmd_freq(args);
    } else if (strcmp(cmd, "set") == 0 || strcmp(cmd, "deg") == 0 ||
               strcmp(cmd, "all") == 0 || strcmp(cmd, "off") == 0 ||
               strcmp(cmd, "sweep") == 0) {
        /*
         * ⚠️ 这五个命令**绕过 `servo_out` 直接写寄存器**，所以必须服从
         * "运动任务在跑时只有一个写者"这条不变式（migration table §8.5）：
         *   - 两个任务交错写同一条 I2C 会写坏寄存器；
         *   - 更要紧的是 `servo_out` 的**占空比缓存会失真** —— 它以为通道还是旧值，
         *     于是"只写变化的通道"就永远不写，命令看起来生效了、实际被下一帧忽略。
         * （P2 只挡住了 `lg`/`lgtest`，漏了这五个，P3 补上。）
         */
        if (motion_is_running()) {
            ESP_LOGE(TAG, "'%s' 会绕过 servo_out 直接写寄存器，而控制任务正在运行 —— "
                          "请先 `motion stop` 或 `estop`", cmd);
        } else if (strcmp(cmd, "set") == 0) {
            cmd_set(args);
        } else if (strcmp(cmd, "deg") == 0) {
            cmd_deg(args);
        } else if (strcmp(cmd, "all") == 0) {
            cmd_all(args);
        } else if (strcmp(cmd, "off") == 0) {
            cmd_off(args);
        } else {
            cmd_sweep(args);
        }
        /* 这些命令改动的是真实硬件状态，缓存必须失效（下一次 servo_out 会全量重写） */
        servo_out_invalidate();
    } else if (strcmp(cmd, "raw") == 0) {
        cmd_raw(args);
    } else if (strcmp(cmd, "cfg") == 0) {
        /* 把剩余部分拆成 "子命令 参数" */
        char sub[32];
        size_t i = 0;
        while (args[i] != '\0' && args[i] != ' ' && i < sizeof(sub) - 1) {
            sub[i] = args[i];
            ++i;
        }
        sub[i] = '\0';
        const char *rest = args + i;
        while (*rest == ' ') {
            ++rest;
        }
        app_cfg_cmd_handle(sub, rest);
    } else if (strcmp(cmd, "motion") == 0 || strcmp(cmd, "stand") == 0 ||
               strcmp(cmd, "estop") == 0 || strcmp(cmd, "lg") == 0 ||
               strcmp(cmd, "lgtest") == 0 || strcmp(cmd, "readback") == 0 ||
               strcmp(cmd, "gait") == 0 || strcmp(cmd, "jog") == 0 ||
               strcmp(cmd, "drive") == 0 ||
               strcmp(cmd, "turn") == 0 || strcmp(cmd, "chain") == 0 ||
               strcmp(cmd, "action") == 0) {
        /* P2/P3：固定周期运动、站姿、行走、急停、动作层、单通道映射核对 */
        app_motion_cmd_handle(cmd, args);
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
