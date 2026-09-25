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

/** @brief 当前链节拍（毫秒） */
uint32_t app_chain_get_period_ms(void);

/** @brief 高度命令（复刻 `padog.height()`：同时改 `H_goal` 与 `R_H`，绕开 slew） */
esp_err_t app_chain_set_height(float h_goal);

/** @brief 选择步态：0 = TROT，1 = WALK */
esp_err_t app_chain_set_gait(int gait_mode);

/**
 * @brief 行走命令：`L`/`R` 是左右腿相位系数（-1/0/1），`spd` 是速度量。
 *
 * @note 语义与原版 `padog.move(spd, L, R)` **完全一致**（由被多帧 golden
 *       验证过的 `control_chain_cmd_move()` 实现）。注意两个容易误解的点：
 *       - `L=R=0` 或 `spd=0` 时**只改** spd/L/R，**不动**目标与相位
 *         ⇒ `jog 0 0 0` = "站着不动"，狗会保持姿态；
 *       - 行进中再次 `move()`（网页摇杆每次动作都会调）**不会重置相位**
 *         （原版 `gait(0)` 只在模式真的变了时才 `t=0`）。
 */
esp_err_t app_chain_jog(float spd, int L, int R);

/**
 * @brief 行走命令（**不切步态**）—— 复刻 `padog.drive(spd, L, R)`。
 *
 * ⚠️ **这是让狗以 WALK 步态行走的唯一途径。** `app_chain_jog()` 对应原版
 * `move()`，而 `move()` 内部会 `gait(0)` 把步态改回 TROT ⇒
 * "先 `gait 1` 再 `jog`" 永远走不出 WALK。原版为此专门设了 `drive()`
 * （参数表里标注"WALK 摇杆用"，即网页的 WALK 摇杆走这条路）。
 *
 * 用法：`motion mode chain` → `gait walk` → `drive -2 1 1`。
 */
esp_err_t app_chain_drive(float spd, int L, int R);

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

/* ==================================================================== */
/*  动作层副作用的窄接口（P3 第 2 步）                                   */
/*                                                                      */
/*  `control/action.c` 把"改别的模块状态"的那几件事做成**显式效果表**，  */
/*  由调用方按顺序施加。其中四种碰的是 `app_chain` 的私有状态 ——        */
/*  下面这几个函数就是那扇门，**全部在既有互斥锁内完成**，不做任何别的  */
/*  事（不推进链、不写舵机）。                                          */
/*                                                                      */
/*  getter 是给宿主测试用的：`ACTION_EFF_*` 有没有真的落到 chain 上，   */
/*  只能靠读回来证明（只断言"输出没变"是抓不到被吞掉的副作用的，P-25）。 */
/* ==================================================================== */

/**
 * @brief 动作层 `ACTION_EFF_GESTURE`：复刻 `padog.gesture(pit, rol, x)`
 *        （直接覆写三个重心目标）。
 *
 * ⚠️ `control_chain.h` 没有暴露 `gesture()`，而 `action_stand()` / `action_sit_direct()`
 * 都会产生这条效果 —— 少了它，"跨模块副作用被吞掉"就是 P-25 那个错误。
 */
esp_err_t app_chain_gesture(float pit, float rol, float x);

/**
 * @brief 动作层 `ACTION_EFF_SIT_OFFSETS`：写 chain 的**配置**
 *        `front_leg_y_offset` / `rear_leg_y_offset`（= `set_leg_sit_offsets(front, rear)`）。
 *
 * @note 这是本工程第一次**在运行时改 chain 的 cfg** —— `control_chain.c` 里那句
 *       "所有调用点传的都是 0，所以放进只读 cfg 就够"在动作层被打破了
 *       （见 `control/action.h` 文件头第 3 条）。改动会在下次 `app_chain_init()` /
 *       `app_chain_reload_cfg()` 时被配置重新覆盖。
 */
esp_err_t app_chain_set_sit_offsets(float front_y, float rear_y);

/** @brief 读回两个腿部 Y 偏置（测试用） */
void app_chain_get_sit_offsets(float *front_y, float *rear_y);

/**
 * @brief 动作层 `ACTION_EFF_SERVO_INIT`：写 chain **state** 的 `init_case`
 *        （= `servo_init(key)`，`servo_output()` 的第二个实参：走 IK 还是直接站姿）。
 */
esp_err_t app_chain_set_init_case(int init_case);

/** @brief 读回 `init_case`（测试用） */
int app_chain_get_init_case(void);

/**
 * @brief 动作层 `ACTION_EFF_CRAWL_RESET`：把 chain state 的三个爬行量清零
 *        （`padog.py:728~730 / 773~775 / 794~796`）。
 *
 * @note 爬行状态机整个归 `control_chain.c`，动作层只负责"清掉"（见 `control/action.h`：
 *       `action_crawl()` 刻意没搬，否则两边都管 `crawl_phase` 会打架）。
 */
esp_err_t app_chain_crawl_reset(void);

/**
 * @brief 直接置爬行状态（爬行命令入口 P5/P6 用；宿主测试也用它给
 *        `ACTION_EFF_CRAWL_RESET` 制造一个"非零初值" —— 否则"清零"这件事在
 *        默认状态下测不出真假，那是 P-18 那一类无效输入）。
 */
esp_err_t app_chain_set_crawl(int crawl_phase, int32_t crawl_until_ms,
                              int32_t crawl_settle_until_ms);

/** @brief 读回三个爬行量（测试用；任一参数可为 NULL） */
void app_chain_get_crawl(int *crawl_phase, int32_t *crawl_until_ms,
                         int32_t *crawl_settle_until_ms);

/** @brief 把 `app_config_t` 映射成 `control_chain_cfg_t`（供测试与调试直接调用） */
void app_chain_cfg_from_app_config(const app_config_t *c, control_chain_cfg_t *out);

#ifdef __cplusplus
}
#endif
