/**
 * @file    body_pose.c
 * @brief   机身姿态 -> 四足足端目标，逐行对齐 micropython/PA_ATTITUDE.py
 *
 * ## 为什么表达式写得这么"啰嗦"
 *
 * Python 原式例如：
 *     ABl_x = l/2 - x -(l*cos(P)*cos(Y))/2 + (b*cos(P)*sin(Y))/2
 *
 * 下面**逐字照抄这个运算顺序**，只把 `cos(P)` 这种重复调用提到局部变量：
 *     const float AB1_x = l / 2.0f - x - (l * cP * cY) / 2.0f + (b * cP * sY) / 2.0f;
 *
 * `l * cP * cY` 在 C 里按 `((l*cP)*cY)` 结合，与 Python 的 `(l*cos(P))*cos(Y)` 一致。
 *
 * **刻意不做公因子提取**（例如把 `cos(R)*cos(Y) + sin(P)*sin(R)*sin(Y)` 抽成 t1）：
 * 那会改变浮点结合顺序，虽然误差远小于容差，但会让"逐行对照"这件事失效。
 * 这个模块存在的意义就是能和 Python 对得上号。
 */

#include "control/body_pose.h"

#include <math.h>
#include <stddef.h>

/** 圆周率（不用 M_PI：POSIX 扩展，非标准 C。理由见 kinematics.c） */
#define BODY_POSE_PI 3.14159265358979323846f

void body_pose_cal_ges(float pit_deg, float rol_deg,
                       float l, float b, float w,
                       float x_off, float hc,
                       body_pose_result_t *out)
{
    if (out == NULL) {
        return;
    }

    /* 原实现写死 YA = 0（不支持偏航） */
    const float YA = 0.0f;

    const float P = pit_deg * BODY_POSE_PI / 180.0f;
    const float R = rol_deg * BODY_POSE_PI / 180.0f;
    const float Y = YA * BODY_POSE_PI / 180.0f;

    const float cP = cosf(P), sP = sinf(P);
    const float cR = cosf(R), sR = sinf(R);
    const float cY = cosf(Y), sY = sinf(Y);

    const float x = x_off;

    /* ------------------------------------------------------------------
     * 相对原实现的第 2 处有意差异：删掉了 4 个无效计算
     *
     * 原 Python 除了 AB*_x / AB*_z，还算了 ABl_y / AB2_y / AB3_y / AB4_y
     * （横向位置），但 cal_ges() 的返回值里**根本没有它们** —— 纯无效计算。
     *
     *   return x1,x2,x3,x4,y1,y2,y3,y4
     *          其中 y1=ABl_z, y2=AB2_z, y3=AB4_z, y4=AB3_z     ← 全是 _z
     *
     * 删掉它们不可能改变输出（那些值从未被读取），而这段代码要跑在
     * 100~200 Hz 的控制循环里，白算 4 组三角函数不划算。
     *
     * ⚠️ 由此暴露一个事实：**形参 `w`（左右腿间距）对 cal_ges() 的输出毫无影响**
     *    —— 它只被那 4 个被丢弃的 AB*_y 使用。所以 config_s.py 里改 w=220
     *    不会对姿态计算产生任何效果。这一点已在 README/迁移文档中记录。
     *
     * 若将来要支持偏航（YA≠0）或返回横向位置，需连同 Python 一起改，
     * 并重新生成 golden 向量。
     * ------------------------------------------------------------------ */
    (void)w;

    /* ---------------- 腿1 ---------------- */
    const float AB1_x = l / 2.0f - x - (l * cP * cY) / 2.0f + (b * cP * sY) / 2.0f;
    const float AB1_z = -hc - (b * (cY * sR - cR * sP * sY)) / 2.0f
                            - (l * (sR * sY + cR * cY * sP)) / 2.0f;

    /* ---------------- 腿2 ---------------- */
    const float AB2_x = l / 2.0f - x - (l * cP * cY) / 2.0f - (b * cP * sY) / 2.0f;
    const float AB2_z = (b * (cY * sR - cR * sP * sY)) / 2.0f - hc
                               - (l * (sR * sY + cR * cY * sP)) / 2.0f;

    /* ---------------- 腿3 ---------------- */
    const float AB3_x = (l * cP * cY) / 2.0f - x - l / 2.0f + (b * cP * sY) / 2.0f;
    const float AB3_z = (l * (sR * sY + cR * cY * sP)) / 2.0f
                               - (b * (cY * sR - cR * sP * sY)) / 2.0f - hc;

    /* ---------------- 腿4 ---------------- */
    const float AB4_x = (l * cP * cY) / 2.0f - x - l / 2.0f - (b * cP * sY) / 2.0f;
    const float AB4_z = (b * (cY * sR - cR * sP * sY)) / 2.0f - hc
                               + (l * (sR * sY + cR * cY * sP)) / 2.0f;

    /* 原实现的返回顺序：x1 x2 x3 x4 y1 y2 y3 y4
       其中 x3 ← AB4_x, x4 ← AB3_x（腿3/腿4 交换），y 同理。 */
    out->x[0] = AB1_x;
    out->x[1] = AB2_x;
    out->x[2] = AB4_x;
    out->x[3] = AB3_x;

    out->y[0] = AB1_z;
    out->y[1] = AB2_z;
    out->y[2] = AB4_z;
    out->y[3] = AB3_z;
}
