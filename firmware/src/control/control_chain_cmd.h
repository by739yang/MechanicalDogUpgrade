/**
 * @file    control_chain_cmd.h
 * @brief   控制链的**命令层**：命令状态 + 跨帧目标 + 调用节拍（纯 C，可宿主测试）
 *
 * ## 为什么单独一层
 *
 * 原版 `mainloop()` **不接受任何参数** —— 它的输入全部来自 `padog` 的模块级全局：
 * `spd` / `L` / `R` / `gait_mode` / `joy_turn`，以及四个目标
 * `H_goal` / `PIT_goal` / `ROL_goal` / `X_goal`。这些全局被
 * `move()` / `gait()` / `height()` / `gesture()` 改写，**改一次就一直留着**。
 *
 * 单帧 golden 对照（`control_chain.csv`）是"每行一个干净初值 + 跑一帧"，
 * 所以它**结构上测不出**两类东西：
 *   1. **跨帧延续**（相位、姿态 slew、目标是不是被保留住了）；
 *   2. **命令语义**（`move()` / `gait()` 到底重置了哪些量）。
 *
 * 本模块把这两件事独立出来，于是可以**连续跑几百帧**与原版逐帧对照
 * （见 `tools/golden/test_control_chain_cmd.c`）。
 *
 * ## ⚠️ 原版 `move()` 的三个细节（第一版 `app_chain` 全都搞错了）
 *
 * ```python
 * def move(spd_, L_, R_):
 *     spd=float(spd_); L=L_; R=R_
 *     if (L_ + R_) != 0 and abs(spd_) > 0:
 *         gait(0)          # 只有"模式真的变了"才 t=0；并重置三个重心目标
 *         servo_init(0)    # init_case = 0
 *         direct_pose_freeze = False
 *         inplace_step_end_ms = 0
 * ```
 *
 * ```python
 * def gait(mode):
 *     if int(mode) != int(gait_mode):
 *         t = 0                       # ← 只在模式**变了**时归零
 *     if int(mode) == 0:
 *         PIT_goal = int(in_pit)      # ← int() 向零截断
 *         ROL_goal = int(in_rol)
 *         X_goal   = int(in_y)
 *     gait_mode = int(mode)
 * ```
 *
 * ⇒ 三个容易漏的点：
 * 1. **`t = 0` 只在模式变化时发生**。所以在 TROT 行进中再次 `move()`
 *    （网页摇杆每次动作都会调）**不会**重置相位 —— 第一版无条件归零，
 *    会造成走动中相位跳变。
 * 2. `move()` 还会把 `init_case` 置 0（影响 `servo_output` 走 IK 还是直接站姿）。
 * 3. 三个重心目标经过 **`int()` 向零截断**。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "control/control_chain.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 目标数组的下标 */
enum {
    CONTROL_CHAIN_GOAL_H   = 0,  /**< `H_goal` */
    CONTROL_CHAIN_GOAL_PIT = 1,  /**< `PIT_goal` */
    CONTROL_CHAIN_GOAL_ROL = 2,  /**< `ROL_goal` */
    CONTROL_CHAIN_GOAL_X   = 3,  /**< `X_goal` */
    CONTROL_CHAIN_GOAL_COUNT = 4,
};

/**
 * @brief 命令状态 —— 对应原版 `padog` 的一组模块级全局。
 *
 * 它是**持久**的：每次 `control_chain_cmd_absorb()` 之后目标会被 tick 的输出覆写，
 * 因为 `WALK` 的 `cal_w()` 会通过 `padog.gesture()` 改这三个目标（见成长手册 P-25）。
 */
typedef struct {
    float spd;        /**< 速度命令（原 `padog.spd`） */
    int   L;          /**< 左腿相位系数 -1/0/1 */
    int   R;          /**< 右腿相位系数 */
    int   gait_mode;  /**< 0 = TROT，1 = WALK */
    float joy_turn;   /**< 横杆转向百分比 */
    /** `H_goal` / `PIT_goal` / `ROL_goal` / `X_goal`，**跨帧保留** */
    float goal[CONTROL_CHAIN_GOAL_COUNT];
} control_chain_cmd_t;

/** 按原版模块级初值初始化命令状态（`PIT_goal=int(in_pit)` 等） */
void control_chain_cmd_init(control_chain_cmd_t *c, const control_chain_cfg_t *cfg);

/** 复刻 `padog.move(spd, L, R)` —— 注意上面列的三个细节 */
void control_chain_cmd_move(control_chain_cmd_t *c, const control_chain_cfg_t *cfg,
                            control_chain_state_t *st, float spd, int L, int R);

/**
 * @brief 复刻 `padog.drive(spd, L, R)` —— **就是 `move()` 去掉 `gait(0)`**。
 *
 * ```python
 * def drive(spd_, L_, R_):   # 注释原文："仅更新 spd/L/R（WALK 摇杆用，不切 gait_mode）"
 *     spd = float(spd_); L = L_; R = R_
 *     if (L_ + R_) != 0 and abs(spd_) > 0:
 *         servo_init(0)              # ← 没有 gait(0)
 *         direct_pose_freeze = False
 *         inplace_step_end_ms = 0
 * ```
 *
 * ⚠️ **这是进入 WALK 的唯一途径。** `move()` 内部会 `gait(0)`，所以
 * "先 `gait(1)` 再 `move(...)`" 会被 `move` 立刻改回 TROT ⇒
 * **用 `move()` 无法让狗以 WALK 步态行走**。原版正是因为这一点才另设
 * `drive()`，并在参数表里标注"WALK 摇杆用"。
 *
 * 这个缺口是在做可视化时发现的：我原来的多帧测试脚本写成 `gait(1)` + `move(...)`，
 * 画出来四条腿是**对角同步**（TROT）而不是四拍顺序 —— 看图才发现脚本根本没在跑 WALK。
 */
void control_chain_cmd_drive(control_chain_cmd_t *c, const control_chain_cfg_t *cfg,
                             control_chain_state_t *st, float spd, int L, int R);

/** 复刻 `padog.gait(mode)` —— 只有模式变了才 `t=0`，所以需要把状态传进来 */
void control_chain_cmd_gait(control_chain_cmd_t *c, const control_chain_cfg_t *cfg,
                            control_chain_state_t *st, int mode);

/**
 * @brief 复刻 `padog.height(goal)`。
 *
 * ⚠️ 原版同时改**两个**量：
 * ```python
 * def height(goal):
 *     global H_goal, R_H
 *     H_goal = goal
 *     R_H = goal      # ← 直接同步！否则网页滑条要等 Kp_H 慢慢逼近，"拖很久"
 * ```
 * 所以**改高度不会经过 `Kp_H` 的 slew**，是一次性到位。只设 `H_goal`
 * 会让每一帧的输出都偏 —— 这个 bug 就是多帧序列对照抓出来的
 * （单帧对照看不到，因为它是把 `R_H`/`H_goal` 当输入直接注入的）。
 */
void control_chain_cmd_height(control_chain_cmd_t *c, control_chain_state_t *st,
                              float h_goal);

/** 复刻 `padog.gesture(PIT, ROL, X)`：直接覆写三个目标 */
void control_chain_cmd_gesture(control_chain_cmd_t *c, float pit, float rol, float x);

/** 站立：速度归零、步态回 TROT、目标复位（并把相位复位） */
void control_chain_cmd_stand(control_chain_cmd_t *c, const control_chain_cfg_t *cfg,
                             control_chain_state_t *st);

/** 转向百分比（`joy_turn`），作用与原版 `set_joy_turn()` 相同 */
void control_chain_cmd_set_turn(control_chain_cmd_t *c, float pct);

/** 把命令状态组装成 `control_chain_tick()` 的一帧输入 */
void control_chain_cmd_make_input(const control_chain_cmd_t *c, int32_t now_ms,
                                  control_chain_input_t *in);

/**
 * @brief 把 tick 的输出目标收回命令状态（**必须每帧调用**）。
 *
 * 原版那三个目标是模块级全局，`WALK` 的 `gesture()` 改一次会一直留着；
 * C 版把"延续"落在这一步上。不调用就会丢掉副作用（P-25）。
 */
void control_chain_cmd_absorb(control_chain_cmd_t *c, const control_chain_out_t *out);

/* ==================================================================== */
/*  链的调用节拍                                                         */
/* ==================================================================== */

/**
 * @brief 链节拍（毫秒）从配置推导。
 *
 * `speed` 在原版里同时决定两件事：**每帧推进多少相位**，以及
 * **一帧该有多长**。要让它俩一致，链节拍周期（秒）必须等于 `speed`，
 * 于是"跑完一个周期"正好花 `Ts` 秒。推导见 `ESP-IDF_C迁移表.md` §8.11。
 */
uint32_t control_chain_sched_period_from_cfg(const control_chain_cfg_t *cfg);

/** 节拍调度器（纯算术，可宿主测试） */
typedef struct {
    uint32_t period_ms;    /**< 标称节拍 */
    int64_t  next_due_ms;  /**< 下一次该推进的时刻 */
    bool     started;      /**< 是否已经排过一次 */
} control_chain_sched_t;

void control_chain_sched_init(control_chain_sched_t *s, uint32_t period_ms);
void control_chain_sched_set_period(control_chain_sched_t *s, uint32_t period_ms);

/**
 * @brief 到了该推进的时刻吗？是则**就地排下一次**并返回 true。
 *
 * @note 落后超过一个周期时**不追补**（`next_due = now + period`）——
 *       宁可整体慢一点，也不要连续补几帧把相位一次推很远（那等于步态瞬跳）。
 */
bool control_chain_sched_due(control_chain_sched_t *s, int64_t now_ms);

#ifdef __cplusplus
}
#endif
