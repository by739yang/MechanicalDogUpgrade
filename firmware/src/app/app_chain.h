/**
 * @file    app_chain.h
 * @brief   把 `control_chain`（原版 `mainloop()` 的一帧）接到固件上的应用层封装
 *
 * ## 职责
 *
 * 1. 把 `app_config_t` 映射成 `control_chain_cfg_t`（含注入表来源的 19 个字段）。
 * 2. 持有 `control_chain_state_t` 与**跨帧保留的四个目标**
 *    （`H_goal`/`PIT_goal`/`ROL_goal`/`X_goal`）。
 * 3. 提供"走 / 停 / 转向"这类**命令级**接口，以及每帧调用入口 `app_chain_step()`。
 *
 * ## ⚠️ 目标必须跨帧保留（这是接口契约，不是可选优化）
 *
 * 原版那四个目标是 `padog` 的**模块级全局**，被 `gesture()` / `height()` /
 * `gait()` / `move()` 改一次就**一直留着**。而 `WALK` 的 `cal_w()` 内部会调
 * `padog.gesture()` 去改 `PIT_goal`/`ROL_goal`/`X_goal`。
 *
 * ⇒ 所以本模块每帧把 `control_chain_out_t.goal[4]` **写回自己的目标**，
 * 下一帧再作为输入喂进去。golden 是逐行单帧对照，**结构上测不出**这个差别，
 * 只能靠这里遵守（详见 `control_chain.h` 与成长手册 P-25）。
 *
 * ## ⚠️ 调用节拍：不是 100 Hz
 *
 * 原版每轮主循环 `t += speed`（`speed = 0.065`），跑完一个周期要
 * `Ts/speed ≈ 15.4` 轮；而原版主循环被网页轮询拖到 **35~120 ms 一轮**，
 * `15.4 × 65 ms ≈ 1 s` ⇒ **`speed` 是照着 ≈65 ms 的循环周期调出来的**。
 *
 * ⇒ 如果在 100 Hz（10 ms）的任务里直接跑，狗会 **6.5 倍速**。
 * ⇒ 所以链有自己的节拍：**默认 65 ms**（推导：链节拍周期（秒）= `speed`，
 *    这样 `Ts = 1.0` 才真的等于"一个周期 1 秒"，且与原版实际行走速度一致）。
 *    详见 `ESP-IDF_C迁移表.md` §8.11。
 *
 * 运动任务仍跑 100 Hz（急停响应、速率限制、将来 P4 的 IMU 闭环），
 * 但只在链的节拍上真正推进；其余帧重新下发同一组角度 ——
 * 因为 `servo_out` 只写变化的通道，**这些重复帧的 I2C 开销是 0**。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app/app_config.h"
#include "control/control_chain.h"
#include "control/servo_map.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 控制链的默认调用周期（毫秒）。
 *
 * 推导：链节拍周期（秒）= `speed`（`config_s.py` 里 0.065）⇒ 65 ms。
 * 这样 `Ts = 1.0` 表示"一个周期 1 秒"真的成立。
 */
#define APP_CHAIN_DEFAULT_PERIOD_MS 65u

/** 命令与状态的快照（控制台/日志用） */
typedef struct {
    bool     valid;       /**< 是否已经算过至少一帧 */
    uint32_t frames;      /**< 累计链帧数 */
    uint32_t period_ms;   /**< 当前链节拍（毫秒） */
    int      gait_mode;   /**< 0 = TROT，1 = WALK */
    float    spd;         /**< 当前速度命令（原 `padog.spd`） */
    int      L;           /**< 左腿相位系数（-1/0/1） */
    int      R;           /**< 右腿相位系数 */
    float    joy_turn;    /**< 横杆转向百分比 */
    /** `H_goal` / `PIT_goal` / `ROL_goal` / `X_goal`（**跨帧保留**） */
    float    goal[4];
    float    t;           /**< 步态相位 */
    float    R_H;         /**< 站高当前值（slew 中） */
    float    PIT_S;       /**< 俯仰当前值 */
    float    ROL_S;       /**< 滚转当前值 */
    float    X_S;         /**< 重心 X 当前值 */
    float    angle_deg[SERVO_MAP_CHANNELS]; /**< 最近一帧的 12 路角度 */
} app_chain_status_t;

/**
 * @brief 初始化：从当前配置建立 `control_chain_cfg_t` 与状态，并置成"站立命令"。
 *
 * @note 必须在 `app_cfg_cmd_init()` 之后调用（要读配置）。
 * @note **不会**驱动舵机 —— 只是把内部状态准备成"站立"，等运动任务来取。
 */
esp_err_t app_chain_init(void);

/** @brief 重新从当前配置读取（`cfg set` 改完之后调用，让改动生效并复位姿态状态） */
esp_err_t app_chain_reload_cfg(void);

/** @brief 设置链的调用周期（毫秒）。1..1000。默认 `APP_CHAIN_DEFAULT_PERIOD_MS` */
esp_err_t app_chain_set_period_ms(uint32_t ms);

/** @brief 选择步态：0 = TROT，1 = WALK */
esp_err_t app_chain_set_gait(int gait_mode);

/**
 * @brief 行走命令：`L`/`R` 是左右腿相位系数（-1/0/1），`spd` 是速度量。
 *
 * @note 语义与原版 `padog.move(spd, L, R)` 一致：`spd=0` 或 `L+R=0` 且 `spd=0` 时
 *       原地不动（但姿态目标保持，狗就**站着**）。**"停止走路"不等于"松力"** ——
 *       松力是 `motion stop` / `estop` 的事。
 */
esp_err_t app_chain_jog(float spd, int L, int R);

/** @brief 横杆转向百分比（`|pct| >= 10` 时髋角才参与转向） */
esp_err_t app_chain_set_joy_turn(float pct);

/**
 * @brief 站立命令：速度归零、步态回 TROT、四个目标复位成配置里的初值。
 *
 * 这走的是**完整控制链**（`cal_ges` → IK → `servo_output`），也就是
 * **原版真正的站姿**；与 P2 的 `stand`（12 路 = 中位角，那是标定用姿态）
 * **不是同一个姿态**。两者差异见 `硬件实物核对清单.md` 阶段 E8。
 */
esp_err_t app_chain_stand(void);

/** @brief 复位姿态状态（`t`/`R_H`/`PIT_S`/`ROL_S`/`X_S` 与目标都回到初值） */
void app_chain_reset_pose(void);

/**
 * @brief 每帧调用一次。到了链的节拍就推进一帧，否则复用上一帧。
 *
 * @param now_ms      当前时刻（毫秒，`esp_timer_get_time()/1000`）
 * @param angle_out   输出：12 路舵机角（**逻辑通道 0..11**）。
 *                    无论是否推进了新帧都会写入，调用方可以直接拿去下发。
 * @return true  表示本次**真的推进了一帧**（新的步态相位）
 * @return false 表示还没到节拍，`angle_out` 与上一帧相同
 */
bool app_chain_step(int64_t now_ms, float angle_out[SERVO_MAP_CHANNELS]);

/** @brief 读状态快照（线程安全） */
void app_chain_get_status(app_chain_status_t *out);

/** @brief 把 `app_config_t` 映射成 `control_chain_cfg_t`（供测试与调试直接调用） */
void app_chain_cfg_from_app_config(const app_config_t *c, control_chain_cfg_t *out);

#ifdef __cplusplus
}
#endif
