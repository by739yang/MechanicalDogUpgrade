/**
 * @file    drv_pca9685.h
 * @brief   PCA9685 16 路 PWM 驱动（机械狗舵机驱动板）
 *
 * 实测事实（2026-09-14）：
 *   - 左半身 0x40、右半身 0x41，挂在 SDA=GPIO21 / SCL=GPIO22 / 100 kHz
 *   - 代码设置的等效频率 50 Hz（实测 PRESCALE = 122）
 *
 * ⚠️ 关于 PRESCALE 的「-1」：
 *   MicroPython 参考实现 `PA_SERVO.py` 用的是
 *       prescale = int(25000000 / 4096 / freq + 0.5)      // 50Hz -> 122
 *   而 PCA9685 数据手册是
 *       prescale = round(25000000 / (4096 * freq)) - 1    // 50Hz -> 121
 *   本驱动默认复刻 MicroPython（写入 122），以保证 C 版与 Python 版舵机脉冲一致，
 *   便于迁移期做对照。改 DRV_PCA9685_PRESCALE_LEGACY 为 0 即切换为手册公式。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 左半身驱动板地址（逻辑通道 0-5 = 左前/左后，ch6 = 机械臂大臂，ch7 = 小臂） */
#define DRV_PCA9685_ADDR_LEFT   0x40
/** 右半身驱动板地址（逻辑通道 6-11 = 右前/右后，ch6 = 夹爪） */
#define DRV_PCA9685_ADDR_RIGHT  0x41
/** all-call 广播地址（只读探测时会看到，切勿用于写） */
#define DRV_PCA9685_ADDR_ALLCALL 0x70

/** 每片通道数 */
#define DRV_PCA9685_CHANNELS    16
/** 内部振荡器标称频率 */
#define DRV_PCA9685_OSC_HZ      25000000.0f
/** 默认舵机频率 */
#define DRV_PCA9685_DEFAULT_HZ  50.0f
/** 舵机脉宽安全范围（微秒） */
#define DRV_PCA9685_US_MIN      500
#define DRV_PCA9685_US_MAX      2500

/** 1 = 复刻 MicroPython 的 prescale 公式（不减 1）；0 = 用数据手册公式 */
#ifndef DRV_PCA9685_PRESCALE_LEGACY
#define DRV_PCA9685_PRESCALE_LEGACY 1
#endif

/* ---------------- 寄存器 ---------------- */
#define DRV_PCA9685_REG_MODE1       0x00
#define DRV_PCA9685_REG_MODE2       0x01
#define DRV_PCA9685_REG_LED0_ON_L   0x06
#define DRV_PCA9685_REG_PRESCALE    0xFE

/* MODE1 位 */
#define DRV_PCA9685_MODE1_RESTART   0x80
#define DRV_PCA9685_MODE1_AI        0x20
#define DRV_PCA9685_MODE1_SLEEP     0x10
#define DRV_PCA9685_MODE1_ALLCALL   0x01

/**
 * @brief 初始化一片 PCA9685 并设置频率。
 * @param addr     0x40 或 0x41
 * @param freq_hz  目标频率（舵机用 50）
 */
esp_err_t drv_pca9685_init(uint8_t addr, float freq_hz);

/**
 * @brief 设置 PWM 频率（会先进入 SLEEP 再写 PRESCALE）。
 */
esp_err_t drv_pca9685_set_freq(uint8_t addr, float freq_hz);

/** @brief 单字节寄存器读 */
esp_err_t drv_pca9685_read_reg(uint8_t addr, uint8_t reg, uint8_t *val);

/** @brief 单字节寄存器写 */
esp_err_t drv_pca9685_write_reg(uint8_t addr, uint8_t reg, uint8_t val);

/**
 * @brief 直接设置某通道的 ON/OFF 计数。
 * @param off  0..4096；4096 表示「整周期全关」（舵机无脉冲，即松力）
 */
esp_err_t drv_pca9685_set_pwm(uint8_t addr, uint8_t ch, uint16_t on, uint16_t off);

/**
 * @brief 按脉宽（微秒）设置某通道。
 * @note  使用最近一次 drv_pca9685_set_freq() 的周期换算；
 *        超出 [500, 2500] µs 会被拒绝并返回 ESP_ERR_INVALID_ARG。
 */
esp_err_t drv_pca9685_set_channel_us(uint8_t addr, uint8_t ch, uint16_t us);

/**
 * @brief 把某片板的所有 16 路都设为同一脉宽。
 */
esp_err_t drv_pca9685_set_all_us(uint8_t addr, uint16_t us);

/**
 * @brief 把某片板的所有通道置为「无脉冲」（舵机松力 / 安全态）。
 */
esp_err_t drv_pca9685_all_off(uint8_t addr);

/** @brief 复刻 MicroPython PA_SERVO 的「角度 -> 脉宽(µs)」换算，供迁移期对照 */
uint16_t drv_pca9685_us_from_degrees_ref(uint16_t degrees);

/** @brief 最近一次设置的标称频率（Hz） */
float drv_pca9685_get_freq(void);

/** @brief 给定 PRESCALE 对应的实际频率（按数据手册公式估算） */
float drv_pca9685_actual_freq(uint8_t prescale);

#ifdef __cplusplus
}
#endif
