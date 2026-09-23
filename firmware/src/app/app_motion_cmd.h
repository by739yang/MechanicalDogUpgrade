/**
 * @file    app_motion_cmd.h
 * @brief   P2 的串口控制台命令（`motion` / `estop` / `stand` / `lg` / `lgtest`）
 *
 * | 命令 | 作用 |
 * |---|---|
 * | `motion start` / `stop` | 启动 / 停止固定周期控制任务 |
 * | `motion stat` | 打印周期、抖动、I2C 写次数、目标与当前角度 |
 * | `motion period <ms>` | 控制周期（默认 10 ms = 100 Hz） |
 * | `motion rate <deg/s>` | 速率上限（默认 120 °/s） |
 * | `motion timeout <ms>` | 命令超时后松力停车（默认 10000 ms，0 = 关闭） |
 * | `stand` | 目标设为直接站姿（12 路 = 中位角），由控制任务限速走过去 |
 * | `estop [原因]` | 急停：松力并停任务 |
 * | `lg <ch> <deg>` | **直接**设某逻辑通道的角度（要求控制任务已停） |
 * | `lg off` | 12 路全部无脉冲（松力） |
 * | `lgtest <ch> [delta]` | 单通道相对中位角偏移测试（P2 硬件映射核对用） |
 * | `readback` | 回读 12 路的 (ON, OFF)，确认写进去的和硬件里的一致 |
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 初始化运动模块与舵机输出层（在 P0 自检里、PCA9685 初始化之后调用） */
void app_motion_cmd_init(void);

/**
 * @brief 处理一条 P2 命令。
 * @param cmd   命令名（`motion` / `estop` / `stand` / `lg` / `lgtest` / `readback`）
 * @param args  其余参数（可为 NULL）
 */
void app_motion_cmd_handle(const char *cmd, const char *args);

/** @brief 打印 P2 命令帮助（供 `help` 调用） */
void app_motion_cmd_help(void);

#ifdef __cplusplus
}
#endif
