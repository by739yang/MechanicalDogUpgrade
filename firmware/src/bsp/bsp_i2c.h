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
 * I2C 硬件 SCL 超时，单位 = APB 80 MHz 时钟周期。
 *
 * ⚠️ 实测教训（2026-09-19，真机）：
 *   默认值下，探测一个**不存在**的地址耗时约 **1000 ms**！
 *   112 个地址的全总线扫描因此要 ~112 秒，看起来就像"卡死"。
 *   而 MicroPython 版扫同一总线只要 28 ms。
 *   80 000 周期 ≈ 1 ms，把单次失败探测从 ~1 s 压到 ~1 ms 量级。
 */
#define BSP_I2C_SCL_TIMEOUT_CYCLES 80000

/**
 * 全总线扫描的墙钟预算（毫秒）。超预算就提前结束并如实报告。
 *
 * 无论上面那个超时值最终被解释成什么单位，扫描都不可能失控 —— 这是最后一道保险。
 */
#define BSP_I2C_SCAN_BUDGET_MS  3000

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
 * @brief 扫描整条总线。
 * @param found      输出缓冲区，存放发现的地址
 * @param max_found  found 的容量
 * @return 发现的器件个数
 *
 * @note 用「读 1 字节是否被 ACK」来判断。注意 MicroPython 的 readfrom_mem 在
 *       无器件时可能不报错而返回残留数据，C 版这里依赖 esp_err_t，不会有该问题。
 */
size_t bsp_i2c_scan(uint8_t *found, size_t max_found);

#ifdef __cplusplus
}
#endif
