/**
 * @file    body_pose.h
 * @brief   机身姿态 -> 四足足端目标 —— 从 micropython/PA_ATTITUDE.py 迁移
 *
 * 数据流位置：
 *   padog.mainloop() 里算出 (PIT_S, ROL_S, X_S) 与站高 Hc
 *   → 本模块 PA_ATTITUDE.cal_ges() 得到四足足端的 (x, z) 偏移
 *   → 与步态轨迹叠加后送 PA_IK.ik()
 *
 * ⚠️ 本模块是**纯数学**，只依赖 <math.h>，不依赖 ESP-IDF（便于宿主 golden 测试）。
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** 四足 */
#define BODY_POSE_LEG_COUNT 4

/**
 * @brief cal_ges() 的输出。
 *
 * 字段顺序**严格按原 Python 的返回顺序**：
 *   Python: return x1, x2, x3, x4, y1, y2, y3, y4
 *           其中 x1=AB1_x x2=AB2_x x3=AB4_x x4=AB3_x
 *                y1=AB1_z y2=AB2_z y3=AB4_z y4=AB3_z
 *
 * ⚠️ 所以 x[2] 对应**腿4**、x[3] 对应**腿3** —— 原实现在这里交换过。
 *    这是有意保留的，不要"顺手改成 1,2,3,4"，否则会和 MicroPython 版行为不一致。
 *
 * @note y[] 虽然名字叫 y，语义上是**腿部平面内的竖直坐标**（中立姿态下 = -Hc）。
 *       沿用原名是为了能和 PA_ATTITUDE.py 逐项对照。
 */
typedef struct {
    float x[BODY_POSE_LEG_COUNT];
    float y[BODY_POSE_LEG_COUNT];
} body_pose_result_t;

/**
 * @brief 计算姿态导致的足端目标。
 *
 * 对应 Python: `cal_ges(PIT, ROL, l, b, w, x, Hc)`
 *
 * @param pit_deg  俯仰角（度），config.py 限幅 ±15
 * @param rol_deg  滚转角（度），config.py 限幅 ±15
 * @param l        前后腿间距（mm），config_s.py 里 230
 * @param b        机身宽相关尺寸（mm），config_s.py 里 120
 * @param w        左右腿间距（mm），config_s.py 里 220
 * @param x_off    重心 X 平移量（mm），对应 Python 的 x 形参
 * @param hc       站高相关量（mm），padog 里为 `_ik_hc(R_H) = R_H + (l1+l2-leg_len_ref)`
 * @param out      输出（可为 NULL，此时不做任何事）
 *
 * @note 原实现内部把偏航角 `YA` 写死为 0，本函数同样如此。
 *       若要支持偏航，需要同时改 Python 与 C，并重新生成 golden 向量。
 */
void body_pose_cal_ges(float pit_deg, float rol_deg,
                       float l, float b, float w,
                       float x_off, float hc,
                       body_pose_result_t *out);

#ifdef __cplusplus
}
#endif
