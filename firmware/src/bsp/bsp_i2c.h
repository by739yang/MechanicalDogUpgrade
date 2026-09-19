/**
 * @file    bsp_i2c.h
 * @brief   I2C 主机总线初始化与基础收发（机械狗唯一 I2C 总线）
 *
 * 硬件事实（2026-09-14 实测）：
 *   - SDA = GPIO21, SCL = GPIO22, 100 kHz
 *   - 总线上的从设备：PCA9685 0x40（左半身）、0x41（右半身）
 *   - 板上没有 IMU
 *
 * ⚠️ 本工程使用 legacy driver/i2c.h。
 *    原因：本机 ESP-IDF 为 v5.1.2，尚无新版 driver/i2c_master.h（IDF 5.2 才引入）。
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 使用的 I2C 外设编号 */
#define BSP_I2C_PORT            I2C_NUM_0
/** 数据线 GPIO */
#define BSP_I2C_SDA_GPIO        21
/** 时钟线 GPIO */
#define BSP_I2C_SCL_GPIO        22
/** 总线频率 */
#define BSP_I2C_CLK_HZ          100000
/** 普通读写超时（毫秒）—— 这是 FreeRTOS 等待信号量的超时，不是总线超时 */
#define BSP_I2C_TIMEOUT_MS      100
/** 扫描时单个地址的等待超时（毫秒）—— 同样是 FreeRTOS 侧 */
#define BSP_I2C_SCAN_TIMEOUT_MS 10

/**
 * I2C 硬件 SCL 超时，单位 = APB 80 MHz 时钟周期。80000 ≈ 1 ms。
 *
 * @note 实测（2026-09-19）：这个值**并不能**缩短"探测失败地址"的耗时，
 *       真正卡住的是下面注释里那个驱动硬编码。保留它只是为了给总线
 *       异常情况一个明确的时限。
 */
#define BSP_I2C_SCL_TIMEOUT_CYCLES 80000

/** 位操作扫描的半周期延时（微秒）。5 µs ≈ 100 kHz */
#define BSP_I2C_BB_DELAY_US     5
/** 位操作扫描的墙钟预算（毫秒）。正常约 20 ms 就能扫完 112 个地址 */
#define BSP_I2C_SCAN_BUDGET_MS  2000

/** I2C 扫描地址范围（7 位地址） */
#define BSP_I2C_ADDR_MIN        0x08
#define BSP_I2C_ADDR_MAX        0x77

/**
 * @brief 初始化 I2C 主机总线。
 * @return ESP_OK 成功；其它为 ESP-IDF 错误码。
 */
esp_err_t bsp_i2c_init(void);

/**
 * @brief 卸载 I2C 驱动（一般只在出错或重配时调用）。
 */
esp_err_t bsp_i2c_deinit(void);

/**
 * @brief 向从设备的寄存器写入数据。
 * @param dev   7 位从机地址
 * @param reg   寄存器地址
 * @param data  待写数据（可为 NULL 当 len 为 0）
 * @param len   数据长度
 */
esp_err_t bsp_i2c_write_reg(uint8_t dev, uint8_t reg, const uint8_t *data, size_t len);

/**
 * @brief 从从设备的寄存器读出数据。
 */
esp_err_t bsp_i2c_read_reg(uint8_t dev, uint8_t reg, uint8_t *data, size_t len);

/**
 * @brief 探测某个地址上是否有器件应答（不关心读到什么）。
 * @return ESP_OK 有应答；ESP_ERR_NOT_FOUND 无应答；其它为总线错误。
 */
esp_err_t bsp_i2c_probe(uint8_t dev);

/**
 * @brief 与 bsp_i2c_probe() 相同，但额外返回本次探测耗时（微秒）。
 *
 * 用途：排查"扫描很慢/像卡死"的问题。若单个探测耗时达到数百毫秒，
 *       说明总线被拉低或从设备在拉伸时钟，而不是代码死循环。
 */
esp_err_t bsp_i2c_probe_timed(uint8_t dev, int64_t *elapsed_us);

/**
 * @brief I2C 总线恢复：把从设备卡住的 SDA 用 9 个 SCL 脉冲释放，并补一个 STOP。
 *
 * 场景：从设备在上电过程中被复位、或在传输中途断电，会把 SDA 拉低不放，
 *       导致主机永远等不到总线空闲。标准做法是手动发 9 个时钟。
 *
 * @note 只能在 bsp_i2c_init() 之前或 deinit 之后调用（需要独占 GPIO）。
 */
esp_err_t bsp_i2c_bus_recover(void);

/**
 * @brief 扫描整条总线（**位操作实现，不依赖 I2C 驱动**）。
 *
 * ⚠️ 必须在 bsp_i2c_init() 之前、或 bsp_i2c_deinit() 之后调用。
 *
 * ## 为什么不用驱动扫描
 *
 * IDF 5.1.2 的 legacy I2C 驱动里有这么一行：
 * ```c
 * #define I2C_CMD_ALIVE_INTERVAL_TICK (1000 / portTICK_PERIOD_MS)
 * ```
 * 在 `i2c_master_cmd_begin()` 的事件等待循环里，它会把这个等待时间
 * **强制抬高到不低于 1000 ms**：
 * ```c
 * if (wait_time < I2C_CMD_ALIVE_INTERVAL_TICK) {
 *     wait_time = I2C_CMD_ALIVE_INTERVAL_TICK;   // 永远是 1000 ms
 * }
 * xQueueReceive(p_i2c->cmd_evt_queue, &evt, wait_time);
 * ```
 * 于是：
 *   - 器件 **ACK** → DONE 事件立刻到达 → 约 300 µs（实测 273~491 µs）
 *   - 器件 **NACK** → **根本不产生事件** → 只能干等 1000 ms 后报 ESP_ERR_TIMEOUT
 *
 * 实测（2026-09-19 真机）：探测不存在的 `0x68` 耗时 1 000 091 µs。
 * 112 个地址的全总线扫描因此要约 **112 秒**，看起来就是"开机卡死"。
 * 传进去的 `ticks_to_wait` 和 `i2c_set_timeout()` **都无法改变**这一点。
 *
 * MicroPython 1.13 用的是 IDF 3.3.2，那版驱动没有这个下限，所以同样
 * 112 个地址只要 28 ms —— 同一份代码换 IDF 版本，行为完全不同。
 *
 * 位操作扫描绕开整个驱动，整条总线约 **20 ms**。
 */
esp_err_t bsp_i2c_bitbang_begin(void);

/**
 * @brief 位操作全总线扫描（需先调用 bsp_i2c_bitbang_begin()）。
 * @return 应答的器件个数；found 里存放地址
 */
size_t bsp_i2c_scan_bitbang(uint8_t *found, size_t max_found);

#ifdef __cplusplus
}
#endif
