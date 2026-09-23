/**
 * @file    servo_out.h
 * @brief   把 `servo_map` 算出来的 12 路占空比真正写进两片 PCA9685
 *
 * ## 这一层只做三件事
 *
 * 1. **粘连硬件**：把纯 C 的 `control/servo_map.c`（可在电脑上测试）接到
 *    只存在于固件里的 `drivers/drv_pca9685.c`。
 * 2. **只写变化的通道**（见下）。
 * 3. **上电安全态**：提供"12 路全部无脉冲"这个动作。
 *
 * ## 为什么要缓存上一次的占空比
 *
 * I2C 跑 100 kHz，一次 PCA9685 通道写 = 地址 + 寄存器 + 4 字节数据
 * ≈ 6 字节 × 9 bit × 10 µs/bit ≈ **0.5 ms**。
 * 12 路每帧全写 = **6 ms** —— 在 100 Hz（10 ms）的控制周期里占掉 60%，
 * 而且完全没必要：**站立不动时寄存器根本不需要重写**。
 *
 * 所以这里记下每个通道上次写的 (ON, OFF)，只写变化的那几路。
 * 站姿稳态下每帧 0 次写；TROT 运动时通常每帧 4~8 路。
 *
 * ⚠️ 缓存必须与**真实硬件状态**保持一致 —— 任何绕过本模块直接写 PCA9685
 *    的代码（P0 的 `set` / `deg` / `all` / `off` 命令）都会让缓存失真，
 *    因此那些命令会先调用 `servo_out_invalidate()`。
 */
#pragma once

#include "control/servo_map.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 复位缓存并把 12 路都置为"无脉冲"（舵机松力 / 安全态）。
 *
 * @note 必须在 `drv_pca9685_init()` 之后调用 —— 那个函数已经把两片板
 *       所有通道置为无脉冲，本函数据此初始化缓存。
 */
esp_err_t servo_out_init(void);

/**
 * @brief 按角度驱动 12 路（只写变化的通道）。
 *
 * @param deg  逻辑通道 0..11 的舵机角度（见 `servo_map.h` 的通道表）
 * @return 第一个遇到的错误；ESP_OK 表示本帧全部成功（或无需写）
 *
 * @note 写失败的通道**不会**更新缓存，下一帧会自动重试 ——
 *       这样偶发的总线错误不会让某个舵机永远停在旧位置。
 */
esp_err_t servo_out_apply_deg(const float deg[SERVO_MAP_CHANNELS]);

/**
 * @brief 12 路全部无脉冲（舵机松力）。急停与超时停车用这个动作。
 */
esp_err_t servo_out_all_off(void);

/**
 * @brief 让缓存失效：下一次 `servo_out_apply_deg()` 会重写全部 12 路。
 *
 * 供那些**绕过本模块**直接写寄存器的代码调用（例如标定/调试命令）。
 */
void servo_out_invalidate(void);

/** @brief 累计的通道写次数（性能观测用） */
uint32_t servo_out_write_count(void);

/** @brief 清零写计数 */
void servo_out_reset_write_count(void);

/**
 * @brief 读回某个逻辑通道当前**硬件里**的 (ON, OFF)，用于调试。
 * @note 这是真的 I2C 读，不是读缓存。
 */
esp_err_t servo_out_readback(uint8_t logical_ch, uint16_t *on, uint16_t *off);

#ifdef __cplusplus
}
#endif
