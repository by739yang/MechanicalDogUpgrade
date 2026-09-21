/**
 * @file    gait_trot.c
 * @brief   TROT 小跑步态 —— 逐行对齐 micropython/PA_TROT.py 的 cal_t()
 *
 * Python 原实现（Ts、faai 为模块级全局）：
 *
 *   def cal_t(t,xs,xf,h,r1,r4,r2,r3):
 *       if t<=Ts*faai:                                  # 支撑/摆动切换点 1
 *           sigma = 2*pi*t/(faai*Ts)
 *           zep   = h*(1-cos(sigma))/2
 *           xep_z = (xf-xs)*((sigma-sin(sigma))/(2*pi))+xs
 *           xep_b = (xs-xf)*((sigma-sin(sigma))/(2*pi))+xf
 *           y1=zep; y2=0;   y3=zep; y4=0
 *           x1=-xep_b*r1; x2=-xep_z*r2; x3=-xep_b*r3; x4=-xep_z*r4
 *       elif t>Ts*faai and t<=Ts:
 *           swing2 = (1.0-faai)*Ts
 *           if swing2 <= 0.0: swing2 = faai*Ts
 *           sigma = 2*pi*(t-Ts*faai)/swing2
 *           ... 同上 ...
 *           y1=0;   y2=zep; y3=0;   y4=zep
 *           x1=-xep_z*r1; x2=-xep_b*r2; x3=-xep_z*r3; x4=-xep_b*r4
 *       return x1,x2,x3,x4,y1,y2,y3,y4
 *
 * 分支 1（前半段）腿1/腿3 摆动，分支 2（后半段）腿2/腿4 摆动 —— 这就是对角小跑。
 */

#include "control/gait_trot.h"

#include <math.h>
#include <stddef.h>

/** 圆周率（不用 M_PI：POSIX 扩展，非标准 C。理由见 kinematics.c） */
#define GAIT_TROT_PI 3.14159265358979323846f

void gait_trot_cal_t(const gait_trot_cfg_t *cfg,
                     float t, float xs, float xf, float h,
                     float r1, float r4, float r2, float r3,
                     gait_trot_out_t *out)
{
    if (cfg == NULL || out == NULL) {
        return;
    }

    const float Ts   = cfg->ts;
    const float faai = cfg->faai;

    /* 退化配置保护（原 Python 会 ZeroDivisionError） */
    const float stance_span = faai * Ts;
    if (stance_span <= 0.0f) {
        for (int i = 0; i < GAIT_TROT_LEG_COUNT; ++i) {
            out->x[i] = 0.0f;
            out->y[i] = 0.0f;
        }
        return;
    }

    /* 相位回绕：原 Python 在 t>Ts 时无分支可走（UnboundLocalError）。
       t == Ts 刻意不回绕，按原实现走摆动相末尾。 */
    if (Ts > 0.0f && (t < 0.0f || t > Ts)) {
        t = t - Ts * floorf(t / Ts);
    }

    float x1 = 0.0f, x2 = 0.0f, x3 = 0.0f, x4 = 0.0f;
    float y1 = 0.0f, y2 = 0.0f, y3 = 0.0f, y4 = 0.0f;

    if (t <= Ts * faai) {
        /* ---------- 分支 1：t ∈ [0, Ts*faai]，腿1/腿3 摆动 ---------- */
        const float sigma = 2.0f * GAIT_TROT_PI * t / (faai * Ts);
        const float zep   = h * (1.0f - cosf(sigma)) / 2.0f;
        const float xep_z = (xf - xs) * ((sigma - sinf(sigma)) / (2.0f * GAIT_TROT_PI)) + xs;
        const float xep_b = (xs - xf) * ((sigma - sinf(sigma)) / (2.0f * GAIT_TROT_PI)) + xf;

        y1 = zep;
        y2 = 0.0f;
        y3 = zep;
        y4 = 0.0f;

        x1 = -xep_b * r1;
        x2 = -xep_z * r2;
        x3 = -xep_b * r3;
        x4 = -xep_z * r4;
    } else {
        /* ---------- 分支 2：t ∈ (Ts*faai, Ts]，腿2/腿4 摆动 ---------- */
        float swing2 = (1.0f - faai) * Ts;
        if (swing2 <= 0.0f) {
            swing2 = faai * Ts;
        }

        const float sigma = 2.0f * GAIT_TROT_PI * (t - Ts * faai) / swing2;
        const float zep   = h * (1.0f - cosf(sigma)) / 2.0f;
        const float xep_z = (xf - xs) * ((sigma - sinf(sigma)) / (2.0f * GAIT_TROT_PI)) + xs;
        const float xep_b = (xs - xf) * ((sigma - sinf(sigma)) / (2.0f * GAIT_TROT_PI)) + xf;

        y1 = 0.0f;
        y2 = zep;
        y3 = 0.0f;
        y4 = zep;

        x1 = -xep_z * r1;
        x2 = -xep_b * r2;
        x3 = -xep_z * r3;
        x4 = -xep_b * r4;
    }

    out->x[0] = x1;
    out->x[1] = x2;
    out->x[2] = x3;
    out->x[3] = x4;

    out->y[0] = y1;
    out->y[1] = y2;
    out->y[2] = y3;
    out->y[3] = y4;
}
