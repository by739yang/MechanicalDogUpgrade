/**
 * @file    app_p0.h
 * @brief   P0 阶段：硬件自检 + 串口控制台
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief P0 入口：I2C 自检 -> PCA9685 初始化 -> 启动串口控制台。
 *
 * 安全约定：**上电不会自动让任何舵机动作**。所有动作必须由串口显式命令触发。
 */
void app_p0_start(void);

#ifdef __cplusplus
}
#endif
