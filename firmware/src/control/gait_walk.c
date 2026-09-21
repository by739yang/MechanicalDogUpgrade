/**
 * @file    gait_walk.c
 * @brief   WALK 四足顺序步态 —— 逐行对齐 micropython/PA_WALK.py
 *
 * Python 原实现结构：
 *
 *   _cycle_len()      = 4.0*faai*Ts
 *   _swing_len()      = faai*Ts
 *   _xs_xf(xf)        = (0,0) if xf==0 else (-0.35*xf, xf)
 *   _leg_xy(local_t, xs, xf, h):
 *       T = _cycle_len(); swing = _swing_len(); stance = T - swing
 *       if stance <= 0: stance = swing
 *       phi = local_t % T                      # ← Python 地板取模！
 *       if phi < swing: 摆动相，cos 抬腿曲线
 *       else:           支撑相，线性后撤
 *   _swing_leg_index(t) = int(t/swing) % 4
 *   _apply_cg(CG_X, CG_Y, l, xf, swing_idx):
 *       sita = _read_gyro_p()*pi/180
 *       swing_idx==0 -> yst = cal_adjust(CG_Y, 0,  l, sita, -1)
 *       swing_idx==1 -> yst = cal_adjust(CG_Y, l,  0, sita, -1)
 *       else         -> yst = cal_adjust(CG_Y, l, xf, sita, +1)
 *       padog.gesture(0, int(CG_X), int(yst))  # ← 副作用
 *   cal_w(CG_X,CG_Y,l,xf,h,t,r1,r4,r2,r3):
 *       xs, xf = _xs_xf(xf)
 *       _apply_cg(...)                          # ← 先做，且忽略异常
 *       x1b,y1 = _leg_xy(t-off1,...)  off1=0
 *       x2b,y2 = _leg_xy(t-off2,...)  off2=swing
 *       x3b,y3 = _leg_xy(t-off3,...)  off3=2*swing
 *       x4b,y4 = _leg_xy(t-off4,...)  off4=3*swing
 *       xi = -xib*ri
 */

#include "control/gait_walk.h"

#include <math.h>
#include <stddef.h>

/** 圆周率（不用 M_PI：POSIX 扩展，非标准 C。理由见 kinematics.c） */
#define GAIT_WALK_PI 3.14159265358979323846f

/** 摆动相起点相对步幅终点的比例（原 `_XS_RATIO`） */
#define GAIT_WALK_XS_RATIO 0.35f

/**
 * @brief Python 语义的取模：结果始终非负。
 *
 * ⚠️ 这是本模块最关键的一处复刻。
 * `_leg_xy()` 收到的 `local_t` 可能为负（`t - off2`、`t - off3`、`t - off4`），
 * 而 Python 的 `%` 对负数返回**非负**结果：
 *     -0.3 % 1.2 == 0.9
 * C 的 `fmodf` 则保留被除数符号：
 *     fmodf(-0.3, 1.2) == -0.3
 * 两者差整整一个周期，轨迹会完全错位。必须用这个函数。
 */
static float py_fmodf(float a, float b)
{
    float r = fmodf(a, b);
    if (r < 0.0f) {
        r += b;
    }
    return r;
}

/** 对应 Python `_cycle_len()` */
static float cycle_len(const gait_walk_cfg_t *cfg)
{
    return 4.0f * cfg->faai * cfg->ts;
}

/** 对应 Python `_swing_len()` */
static float swing_len(const gait_walk_cfg_t *cfg)
{
    return cfg->faai * cfg->ts;
}

/** 对应 Python `_xs_xf(xf)` */
static void xs_xf(float xf_in, float *xs_out, float *xf_out)
{
    if (xf_in == 0.0f) {
        *xs_out = 0.0f;
        *xf_out = 0.0f;
        return;
    }
    *xs_out = -GAIT_WALK_XS_RATIO * xf_in;
    *xf_out = xf_in;
}

/** 对应 Python `_leg_xy(local_t, xs, xf, h)` */
static void leg_xy(const gait_walk_cfg_t *cfg, float local_t,
                   float xs, float xf, float h, float *x_out, float *y_out)
{
    const float T = cycle_len(cfg);
    const float swing = swing_len(cfg);
    float stance = T - swing;
    if (stance <= 0.0f) {
        stance = swing;
    }

    const float phi = py_fmodf(local_t, T);   /* ← 必须用 Python 语义的取模 */

    if (phi < swing) {
        const float sigma = 2.0f * GAIT_WALK_PI * phi / swing;
        *y_out = h * (1.0f - cosf(sigma)) / 2.0f;
        *x_out = (xf - xs) * ((sigma - sinf(sigma)) / (2.0f * GAIT_WALK_PI)) + xs;
    } else {
        const float u = (phi - swing) / stance;
        *y_out = 0.0f;
        *x_out = xf + (xs - xf) * u;
    }
}

/** 对应 Python `_swing_leg_index(t)` */
static int swing_leg_index(const gait_walk_cfg_t *cfg, float t)
{
    const float swing = swing_len(cfg);
    if (swing <= 0.0f) {
        return 0;
    }
    return ((int)(t / swing)) % 4;
}

/** 对应 Python `cal_adjust(CG_Y, l, xk, sita, period)` */
static float cal_adjust(float cg_y, float l, float xk, float sita, float period,
                        float body_h)
{
    return cg_y + period * (l + xk) / 4.0f + body_h * tanf(sita * 1.5f);
}

/**
 * 对应 Python `_apply_cg()`。
 *
 * 原实现整个包在 try/except 里：任何异常（例如 `import padog` 失败）都会**静默跳过**
 * 重心调整。C 版没有这种"可能不发生"的分支，始终计算 ——
 * golden 测试里 Python 侧的 padog 由 stub 提供，所以两边行为一致。
 */
static void apply_cg(float cg_x, float cg_y, float l, float xf, int swing_idx,
                     float body_h, float gyro_p_deg,
                     gait_walk_gesture_t *gesture_out)
{
    const float sita = gyro_p_deg * GAIT_WALK_PI / 180.0f;

    float yst;
    if (swing_idx == 0) {
        yst = cal_adjust(cg_y, 0.0f, l, sita, -1.0f, body_h);
    } else if (swing_idx == 1) {
        yst = cal_adjust(cg_y, l, 0.0f, sita, -1.0f, body_h);
    } else {
        yst = cal_adjust(cg_y, l, xf, sita, 1.0f, body_h);
    }

    if (gesture_out != NULL) {
        /* 原实现用 Python 的 int()，**向零截断**（不是 floor） */
        gesture_out->pit = 0;
        gesture_out->rol = (int32_t)cg_x;
        gesture_out->x   = (int32_t)yst;
    }
}

void gait_walk_cal_w(const gait_walk_cfg_t *cfg,
                     float cg_x, float cg_y, float l, float xf_in, float h, float t,
                     float r1, float r4, float r2, float r3,
                     float body_h, float gyro_p_deg,
                     gait_walk_out_t *out,
                     gait_walk_gesture_t *gesture_out)
{
    if (cfg == NULL) {
        return;
    }

    /* 退化配置保护（原 Python 会除零） */
    if (cfg->faai * cfg->ts <= 0.0f) {
        if (out != NULL) {
            for (int i = 0; i < GAIT_WALK_LEG_COUNT; ++i) {
                out->x[i] = 0.0f;
                out->y[i] = 0.0f;
            }
        }
        if (gesture_out != NULL) {
            gesture_out->pit = 0;
            gesture_out->rol = (int32_t)cg_x;
            gesture_out->x   = (int32_t)cg_y;
        }
        return;
    }

    float xs = 0.0f, xf = 0.0f;
    xs_xf(xf_in, &xs, &xf);

    const float swing = swing_len(cfg);

    /* 先做重心调整（原实现顺序如此），用的是变换后的 xf。
       注意原 `_apply_cg()` 并不使用 Ts/faai —— 所以这里不需要 cfg。 */
    apply_cg(cg_x, cg_y, l, xf, swing_leg_index(cfg, t),
             body_h, gyro_p_deg, gesture_out);

    if (out == NULL) {
        return;
    }

    const float off[GAIT_WALK_LEG_COUNT] = { 0.0f, swing, 2.0f * swing, 3.0f * swing };
    float xb[GAIT_WALK_LEG_COUNT];
    float y[GAIT_WALK_LEG_COUNT];

    for (int i = 0; i < GAIT_WALK_LEG_COUNT; ++i) {
        leg_xy(cfg, t - off[i], xs, xf, h, &xb[i], &y[i]);
    }

    /* 腿系数按原实现的形参顺序：r1, r4, r2, r3 */
    const float r[GAIT_WALK_LEG_COUNT] = { r1, r2, r3, r4 };

    for (int i = 0; i < GAIT_WALK_LEG_COUNT; ++i) {
        out->x[i] = -xb[i] * r[i];
        out->y[i] = y[i];
    }
}
