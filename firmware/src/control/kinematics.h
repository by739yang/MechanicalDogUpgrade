/**
 * @file    kinematics.h
 * @brief   腿部逆运动学 —— 从 micropython/PA_IK.py 迁移
 *
 * 迁移原则（见 ESP-IDF_C迁移表.md）：
 *   1. **逐行对齐原实现**，包括它的怪写法（例如 `pi - 1.5707` 而不是 `pi/2`），
 *      这样 C 版才能和 MicroPython 版做数值对照。
 *   2. 唯一有意改善之处：原实现对 acos/asin 参数**不做定义域检查**，
 *      越界会抛 ValueError / 产生 NaN。C 版把参数 clamp 到 [-1, 1]，
 *      并对除零做保护 —— 迁移表的验收标准要求「异常输入不产生 NaN」。
 *
 * 数据流（原实现在 padog.py）：
 *   PA_ATTITUDE.cal_ges() → 足端 (x, y)
 *   → PA_IK.ik(ma_case, l1, l2, ...) → ham/shank 关节角
 *   → padog.servo_output() → PCA9685
 *
 * ⚠️ 本模块是**纯数学**，只依赖 <math.h>，不依赖 ESP-IDF。
 *    这样才能用宿主 gcc 编译来做 golden test（见 tools/golden/）。
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** 四足 */
#define KIN_LEG_COUNT 4

/** 足端坐标与关节角的对应关系（顺序与 PA_IK.py 的参数顺序一致） */
typedef enum {
    /** 串联腿 —— 对应 PA_IK.ik(case=0)。当前 config_s.py 里 ma_case=0，是实机在跑的分支 */
    KIN_MODE_SERIES = 0,
    /** 并联腿 —— 对应 PA_IK.ik(case=1)。⚠️ 实机未验证过此分支 */
    KIN_MODE_PARALLEL = 1,
} kin_mode_t;

/**
 * @brief 逆运动学输出。
 *
 * 命名沿用原 Python 的 ham / shank，便于与 PA_IK.py 逐项对照。
 *
 * @note 在 KIN_MODE_PARALLEL 下，原 Python 返回的是 (sita1_*, sita2_*)，
 *       本结构按**位置**对应：ham ← sita1（fai-psai），shank ← sita2（fai+psai）。
 */
typedef struct {
    float ham[KIN_LEG_COUNT];    /**< 大腿角（度），对应 Python 的 ham1..ham4 */
    float shank[KIN_LEG_COUNT];  /**< 小腿角（度），对应 Python 的 shank1..shank4 */
} kin_ik_result_t;

/**
 * @brief 逆运动学求解。
 *
 * @param mode  KIN_MODE_SERIES 或 KIN_MODE_PARALLEL
 * @param l1    大腿连杆长（mm），config_s.py 里 130
 * @param l2    小腿连杆长（mm），config_s.py 里 138
 * @param x     四足足端 x（mm，机体坐标系；注意原实现对 x 取负号后再算）
 * @param y     四足足端 y（mm，向下为负）
 * @param out   输出关节角（度）
 *
 * @note 无返回值：任何输入都会得到有限数值（越界会 clamp，不会产生 NaN）。
 */
void kin_ik(kin_mode_t mode, float l1, float l2,
            const float x[KIN_LEG_COUNT], const float y[KIN_LEG_COUNT],
            kin_ik_result_t *out);

#ifdef __cplusplus
}
#endif
