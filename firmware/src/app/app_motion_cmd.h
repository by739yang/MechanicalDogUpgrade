/**
 * @file    app_motion_cmd.h
 * @brief   P2/P3 的串口控制台命令
 *
 * | 命令 | 作用 |
 * |---|---|
 * | `motion start` / `stop` | 启动 / 停止固定周期控制任务 |
 * | `motion stat` | 周期、抖动、I2C 写次数、模式、链帧数、目标与当前角度 |
 * | `motion period <ms>` | 运动任务周期（默认 10 ms = 100 Hz） |
 * | `motion rate <deg/s>` | 速率上限（默认 120 °/s，**只对 POSE 模式生效**） |
 * | `motion timeout <ms>` | 命令超时后松力停车（默认 10000 ms，0 = 关闭） |
 * | `motion mode pose\|chain` | 控制模式：直接 12 路角度 / 走控制链（步态） |
 * | `stand` | **原版真正的站姿**（走控制链 `cal_ges`→IK→`servo_output`） |
 * | `stand direct` | **标定用站姿**（12 路 = 中位角；与原版站姿不同，见核对清单 E8） |
 * | `gait trot\|walk` | 选步态（相位 `t` 归零） |
 * | `jog <spd> <L> <R>` | 行走命令，如 `jog -3 1 1`（前进）、`jog 3 1 1`（后退） |
 * | `turn <pct>` | 横杆转向百分比（`|pct| >= 10` 髋角才参与） |
 * | `chain` | 打印控制链状态（相位 / 目标 / 12 路角度与占空比） |
 * | `estop [原因]` | 急停：松力并停任务 |
 * | `lg <ch> <deg>` | **直接**设某逻辑通道的角度（要求控制任务已停） |
 * | `lg off` | 12 路全部无脉冲（松力） |
 * | `lgtest <ch> [delta]` | 单通道相对中位角偏移测试（硬件映射核对用） |
 * | `readback` | 回读 12 路的 (ON, OFF)，确认写进去的和硬件里的一致 |
 *
 * ⚠️ **"停止走路"和"松力"是两件事**：
 *   - `jog 0 0 0` / `stand` → 狗**站着**（控制链持续输出站姿脉冲）；
 *   - `motion stop` / `estop` → 12 路**无脉冲**，狗**趴下**（松力）。
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
