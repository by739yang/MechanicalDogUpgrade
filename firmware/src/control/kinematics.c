/**
 * @file    kinematics.c
 * @brief   腿部逆运动学实现 —— 逐行对齐 micropython/PA_IK.py
 *
 * 与 PA_IK.py 的对应关系（务必保持同步，改这里要同步改那边或注明原因）：
 *
 *   Python (case=0 串联腿)                     C (KIN_MODE_SERIES)
 *   ---------------------------------------    ---------------------------------
 *   x1 = -x1                                   x = -x
 *   shank = pi - acos((x²+y²-l1²-l2²)/(-2*l1*l2))
 *   fai   = acos((l1²+x²+y²-l2²)/(2*l1*sqrt(x²+y²)))
 *   if   x>0:  ham = |atan(y/x)| - fai
 *   elif x<0:  ham = pi - |atan(y/x)| - fai
 *   else:      ham = pi - 1.5707 - fai         ← 注意是 1.5707，不是 pi/2
 *   shank = 180*shank/pi ; ham = 180*ham/pi
 *
 *   Python (case=1 并联腿)                     C (KIN_MODE_PARALLEL)
 *   ---------------------------------------    ---------------------------------
 *   y = -y ; L = sqrt(x²+y²)
 *   psai = asin(x/L)
 *   fai  = acos((L²+l1²-l2²)/(2*l1*L))
 *   sita1 = 180*(fai-psai)/pi                  -> out->ham[]
 *   sita2 = 180*(fai+psai)/pi                  -> out->shank[]
 *
 * 相对原实现的**唯一有意差异**：参数 clamp。
 * 原 Python 对 acos/asin 的参数不做检查，越界抛 ValueError；C 版 clamp 到 [-1,1]。
 * 这只影响原本就会报错的异常输入，正常可达域内结果完全一致。
 */

#include "control/kinematics.h"

#include <math.h>
#include <stddef.h>

/**
 * 圆周率。
 *
 * 不用 M_PI：它是 POSIX 扩展，不是标准 C —— `-std=c11` 下未定义。
 * 本模块要保持"只依赖 <math.h> + <stddef.h>"，这样宿主测试可以用严格
 * 标准模式（-std=c11）编译，顺便把不可移植的写法挡在门外。
 *
 * 取 float 字面量：与 Python 里 `(float)math.pi` 的结果一致。
 */
#define KIN_PI 3.14159265358979323846f

/** 把 acos/asin 的参数夹到定义域内（原 Python 不检查，越界直接抛异常） */
static float clamp_unit(float v)
{
    if (v > 1.0f) {
        return 1.0f;
    }
    if (v < -1.0f) {
        return -1.0f;
    }
    return v;
}

/** 弧度转角度（原 Python 用 180*x/pi） */
static float rad2deg(float rad)
{
    return 180.0f * rad / KIN_PI;
}

/** 单腿求解 —— 串联腿（对应 PA_IK.py 的 case==0 分支） */
static void solve_series_leg(float l1, float l2, float x_in, float y,
                             float *ham_out, float *shank_out)
{
    /* 原实现先把 x 取负 */
    const float x = -x_in;

    const float le2 = l1 * l1 + l2 * l2;
    const float r2  = x * x + y * y;
    const float r   = sqrtf(r2);

    /* shank = pi - acos((r² - l1² - l2²) / (-2*l1*l2)) */
    float shank = KIN_PI - acosf(clamp_unit((r2 - le2) / (-2.0f * l1 * l2)));

    /* fai = acos((l1² + r² - l2²) / (2*l1*r)) —— r 为 0 时原实现除零，这里保护 */
    float fai;
    if (r > 1e-6f) {
        fai = acosf(clamp_unit((l1 * l1 + r2 - l2 * l2) / (2.0f * l1 * r)));
    } else {
        fai = 0.0f;
    }

    float ham;
    if (x > 0.0f) {
        ham = fabsf(atanf(y / x)) - fai;
    } else if (x < 0.0f) {
        ham = KIN_PI - fabsf(atanf(y / x)) - fai;
    } else {
        /* 原实现写死 1.5707（不是 pi/2），故意保留以保持一致 */
        ham = KIN_PI - 1.5707f - fai;
    }

    *shank_out = rad2deg(shank);
    *ham_out   = rad2deg(ham);
}

/** 单腿求解 —— 并联腿（对应 PA_IK.py 的 case==1 分支） */
static void solve_parallel_leg(float l1, float l2, float x, float y_in,
                               float *sita1_out, float *sita2_out)
{
    /* 原实现先把 y 取负（x 不动） */
    const float y = -y_in;

    const float L2v = x * x + y * y;
    const float L   = sqrtf(L2v);

    float psai = 0.0f;
    float fai  = 0.0f;
    if (L > 1e-6f) {
        psai = asinf(clamp_unit(x / L));
        fai  = acosf(clamp_unit((L2v + l1 * l1 - l2 * l2) / (2.0f * l1 * L)));
    }

    *sita1_out = rad2deg(fai - psai);
    *sita2_out = rad2deg(fai + psai);
}

void kin_ik(kin_mode_t mode, float l1, float l2,
            const float x[KIN_LEG_COUNT], const float y[KIN_LEG_COUNT],
            kin_ik_result_t *out)
{
    if (out == NULL) {
        return;
    }

    for (int i = 0; i < KIN_LEG_COUNT; ++i) {
        if (mode == KIN_MODE_PARALLEL) {
            solve_parallel_leg(l1, l2, x[i], y[i], &out->ham[i], &out->shank[i]);
        } else {
            solve_series_leg(l1, l2, x[i], y[i], &out->ham[i], &out->shank[i]);
        }
    }
}
