/**
 * @file    gait_trot.h
 * @brief   TROT 小跑步态轨迹 —— 从 micropython/PA_TROT.py 迁移
 *
 * TROT（对角小跑）：对角腿同相。占空比 `faai` 决定支撑相/摆动相的比例，
 * 实机运行值 `Ts=1.0`、`faai=0.42`（config.py 的 0.5 被 config_s.py 的 0.42 覆盖）。
 *
 * ## 相对原实现的三处有意差异
 *
 * 1. **时序参数改成显式入参**。原 `cal_t()` 读模块级全局 `Ts` / `faai`，
 *    而 `padog.py` 在运行时用 `_sync_pa_step_timing()` 从 config 同步进去 ——
 *    隐藏状态。C 版收进 `gait_trot_cfg_t`，消除这个坑。
 *
 * 2. **补了相位回绕**。原 Python 只有两个分支（`t<=Ts*faai` 和 `Ts*faai<t<=Ts`），
 *    **没有 else** —— `t > Ts` 时会 `UnboundLocalError`。实机靠调用方保证 `t < Ts`。
 *    C 版对 `t < 0 || t > Ts` 先把相位回绕到 `[0, Ts)`，保证任何输入都有确定输出。
 *    注意 `t == Ts` **不**回绕，按原实现走摆动相末尾，保持一致。
 *
 * 3. **退化配置保护**。`faai*Ts <= 0` 时原实现会除零，C 版退回中立（全 0）。
 *
 * ⚠️ 形参顺序与原实现**完全一致**：`(t, xs, xf, h, r1, r4, r2, r3)`
 *    —— 注意是 `r1, r4, r2, r3`，不是 `r1, r2, r3, r4`。
 *    这是原 `padog.mainloop` 的调用顺序（`L*lr1, L*lr4, R*lr2, R*lr3`），
 *    刻意不去"整理"它，以便和 Python 逐项对照。调用时建议用有名字的局部变量：
 *    ```c
 *    const float r_leg1 = L * lr1, r_leg4 = L * lr4;
 *    const float r_leg2 = R * lr2, r_leg3 = R * lr3;
 *    gait_trot_cal_t(&cfg, t, xs, xf, h, r_leg1, r_leg4, r_leg2, r_leg3, &out);
 *    ```
 *
 * ⚠️ 本模块是**纯数学**，只依赖 <math.h>，不依赖 ESP-IDF。
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** 四足 */
#define GAIT_TROT_LEG_COUNT 4

/** 时序参数（原 PA_TROT.py 的模块级全局 Ts / faai） */
typedef struct {
    float ts;    /**< 步态周期 Ts（秒）。实机 1.0 */
    float faai;  /**< 支撑相占空比。实机 0.42 */
} gait_trot_cfg_t;

/** cal_t() 输出。x 为前后位移、y 为抬腿高度，顺序均为腿1..腿4 */
typedef struct {
    float x[GAIT_TROT_LEG_COUNT];
    float y[GAIT_TROT_LEG_COUNT];
} gait_trot_out_t;

/**
 * @brief 计算 TROT 步态某相位下的四足端目标。对应 Python `PA_TROT.cal_t()`。
 *
 * @param cfg  时序参数（Ts / faai）
 * @param t    当前相位时间（秒）
 * @param xs   摆动相起点位移
 * @param xf   摆动相终点位移
 * @param h    抬腿高度
 * @param r1   腿1 系数（原实现第 5 个形参）
 * @param r4   腿4 系数（第 6 个）
 * @param r2   腿2 系数（第 7 个）
 * @param r3   腿3 系数（第 8 个）
 * @param out  输出（可为 NULL）
 */
void gait_trot_cal_t(const gait_trot_cfg_t *cfg,
                     float t, float xs, float xf, float h,
                     float r1, float r4, float r2, float r3,
                     gait_trot_out_t *out);

#ifdef __cplusplus
}
#endif
