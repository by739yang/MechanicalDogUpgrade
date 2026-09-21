/**
 * @file    gait_walk.h
 * @brief   WALK 四足顺序步态 —— 从 micropython/PA_WALK.py 迁移
 *
 * WALK：四腿依次摆动（1→2→3→4），每腿 `faai` 比例摆动、其余支撑。
 * 实机值 `Ts=1.0`、`faai=0.30`（`padog._sync_walk_pa_timing()` 从 `walk_faai` 取）。
 *
 * ## 相对原实现的四处有意差异
 *
 * 1. **副作用改成显式输出**。原 `cal_w()` 内部会调 `_apply_cg()`，而后者执行
 *    `padog.gesture(0, int(CG_X), int(yst))` —— **直接改 padog 的重心目标**。
 *    C 版把这个副作用收进 `gait_walk_gesture_t`，由调用方决定怎么用。
 *    注意原实现用 `int()` **向零截断**，不是四舍五入。
 *
 * 2. **时序参数 `Ts`/`faai` 改成显式入参**（原来是模块级全局）。
 *
 * 3. **`body_h` 与 `gyro_p` 改成显式入参**。
 *    原实现分别来自 `_body_h()`（读 `padog.R_H`，异常时兜底 110.0）
 *    和 `_read_gyro_p()`（无 IMU 时恒为 0）。
 *
 * 4. **补了退化配置保护**（`faai*Ts <= 0` 时原实现会除零）。
 *
 * ## ⚠️ 两个必须复刻的 Python 语义陷阱
 *
 * 1. **`_leg_xy()` 里的 `local_t % T`**：`local_t` 可能为负（`t - off2`），
 *    而 Python 的 `%` 对负数返回**非负**结果（地板取模），C 的 `fmodf` 则保留
 *    被除数的符号。差一个周期，轨迹就完全错了。本模块用 `py_fmodf()` 复刻。
 * 2. **`int()` 向零截断**：`int(-7.5) == -7`。C 的 `(int32_t)` 强转行为相同，
 *    但不要写成 `floorf()`。
 *
 * ⚠️ 形参顺序同样是 `(..., t, r1, r4, r2, r3)`。
 *
 * ⚠️ 纯数学，只依赖 <math.h>，不依赖 ESP-IDF。
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GAIT_WALK_LEG_COUNT 4

/** 时序参数（原 PA_WALK.py 的模块级全局 Ts / faai） */
typedef struct {
    float ts;    /**< 步态周期 Ts（秒）。实机 1.0 */
    float faai;  /**< 每腿摆动相占比。实机 0.30 */
} gait_walk_cfg_t;

/** 四足端目标，顺序为腿1..腿4 */
typedef struct {
    float x[GAIT_WALK_LEG_COUNT];
    float y[GAIT_WALK_LEG_COUNT];
} gait_walk_out_t;

/**
 * @brief 原实现通过 `padog.gesture()` 施加的副作用，C 版改成显式输出。
 *
 * Python 里是：`padog.gesture(0, int(CG_X), int(yst))`
 *   → 等价于设置 `padog.PIT_goal = 0`、`ROL_goal = int(CG_X)`、`X_goal = int(yst)`
 */
typedef struct {
    int32_t pit;   /**< 第 1 个实参，恒为 0 */
    int32_t rol;   /**< 第 2 个实参 = `int(cg_x)`，**向零截断** */
    int32_t x;     /**< 第 3 个实参 = `int(yst)`，**向零截断** */
} gait_walk_gesture_t;

/**
 * @brief 计算 WALK 步态某相位下的四足端目标。对应 Python `PA_WALK.cal_w()`。
 *
 * @param cfg          时序参数
 * @param cg_x         重心 X（mm），原实现会 `int()` 后写进重心目标
 * @param cg_y         重心 Y（mm）
 * @param l            前后腿间距（mm），实机 230
 * @param xf_in        步幅参数（mm）。内部会经 `_xs_xf()` 变换，`0` 是特例
 * @param h            抬腿高度（mm）
 * @param t            当前相位时间（秒）
 * @param r1,r4,r2,r3  腿系数（**顺序同原实现，注意是 r1,r4,r2,r3**）
 * @param body_h       原 `_body_h()`（`padog.R_H`，兜底 110.0）
 * @param gyro_p_deg   原 `_read_gyro_p()`（俯仰角，单位度；无 IMU 时为 0）
 * @param out          足端输出（可为 NULL）
 * @param gesture_out  重心副作用输出（可为 NULL）
 */
void gait_walk_cal_w(const gait_walk_cfg_t *cfg,
                     float cg_x, float cg_y, float l, float xf_in, float h, float t,
                     float r1, float r4, float r2, float r3,
                     float body_h, float gyro_p_deg,
                     gait_walk_out_t *out,
                     gait_walk_gesture_t *gesture_out);

#ifdef __cplusplus
}
#endif
