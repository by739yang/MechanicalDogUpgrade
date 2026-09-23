/**
 * @file    servo_map.c
 * @brief   关节角 -> 12 路舵机输出（纯 C 实现，逐行对应原 MicroPython）
 */

#include "control/servo_map.h"

#include <stddef.h>

/* ================= 角度 -> 占空比 ================= */

uint16_t servo_map_min_duty(void)
{
    /* int(4095 * 500 / 20000) = int(102.375) = 102，用整数运算保证精确 */
    return (uint16_t)(SERVO_MAP_DUTY_FULL * SERVO_MAP_US_MIN / SERVO_MAP_PERIOD_US);
}

uint16_t servo_map_max_duty(void)
{
    /* int(4095 * 2500 / 20000) = int(511.875) = 511 */
    return (uint16_t)(SERVO_MAP_DUTY_FULL * SERVO_MAP_US_MAX / SERVO_MAP_PERIOD_US);
}

uint16_t servo_map_deg_to_duty(float degrees)
{
    const uint16_t min_duty = servo_map_min_duty();
    const uint16_t max_duty = servo_map_max_duty();
    const float    span     = (float)(max_duty - min_duty); /* 409 */

    /*
     * 原实现：
     *     duty = self.min_duty + span * degrees / self.degrees
     *     duty = min(self.max_duty, max(self.min_duty, int(duty)))
     *
     * 顺序是「先 int() 截断，再限幅」，且 int() 向零截断（这里 duty 可能为负，
     * 因为角度可以由 IK 算出负值）—— 用 C 的 (int32_t) 强转即可，
     * 它同样是向零截断（见成长手册 P-17 关于取整语义的教训）。
     */
    const float   duty_f = (float)min_duty + span * degrees / SERVO_MAP_DEGREES;
    int32_t       duty   = (int32_t)duty_f;

    if (duty < (int32_t)min_duty) {
        duty = (int32_t)min_duty;
    }
    if (duty > (int32_t)max_duty) {
        duty = (int32_t)max_duty;
    }
    return (uint16_t)duty;
}

void servo_map_duty_to_pwm(uint16_t duty, uint16_t *on, uint16_t *off)
{
    if (on == NULL || off == NULL) {
        return;
    }

    if (duty == 0) {
        /* 整周期全关 = 无脉冲 = 舵机松力 */
        *on  = 0;
        *off = 4096;
    } else if (duty >= SERVO_MAP_DUTY_FULL) {
        *on  = 4096;
        *off = 0;
    } else {
        *on  = 0;
        *off = duty;
    }
}

/* ================= 硬件位置 ================= */

/*
 * 逻辑通道 -> (板地址, 板上通道)。
 * 0x40 = 左半身（左前 ch0-2、左后 ch3-5）
 * 0x41 = 右半身（右前 ch0-2、右后 ch3-5）
 */
static const servo_map_hw_t s_hw[SERVO_MAP_CHANNELS] = {
    { SERVO_MAP_ADDR_LEFT,  0 }, /* 0  腿1 左前 髋   */
    { SERVO_MAP_ADDR_LEFT,  1 }, /* 1  腿1 左前 大腿 */
    { SERVO_MAP_ADDR_LEFT,  2 }, /* 2  腿1 左前 小腿 */
    { SERVO_MAP_ADDR_LEFT,  3 }, /* 3  腿4 左后 髋   */
    { SERVO_MAP_ADDR_LEFT,  4 }, /* 4  腿4 左后 大腿 */
    { SERVO_MAP_ADDR_LEFT,  5 }, /* 5  腿4 左后 小腿 */
    { SERVO_MAP_ADDR_RIGHT, 0 }, /* 6  腿2 右前 髋   */
    { SERVO_MAP_ADDR_RIGHT, 1 }, /* 7  腿2 右前 大腿 */
    { SERVO_MAP_ADDR_RIGHT, 2 }, /* 8  腿2 右前 小腿 */
    { SERVO_MAP_ADDR_RIGHT, 3 }, /* 9  腿3 右后 髋   */
    { SERVO_MAP_ADDR_RIGHT, 4 }, /* 10 腿3 右后 大腿 */
    { SERVO_MAP_ADDR_RIGHT, 5 }, /* 11 腿3 右后 小腿 */
};

const servo_map_hw_t *servo_map_hw(uint8_t logical_ch)
{
    if (logical_ch >= SERVO_MAP_CHANNELS) {
        return NULL;
    }
    return &s_hw[logical_ch];
}

const char *servo_map_channel_name(uint8_t logical_ch)
{
    /* 腿名按**物理**位置写，便于对着狗看；通道编号仍按原实现 */
    static const char *names[SERVO_MAP_CHANNELS] = {
        "L-F hip",   "L-F thigh", "L-F shank",
        "L-R hip",   "L-R thigh", "L-R shank",
        "R-F hip",   "R-F thigh", "R-F shank",
        "R-R hip",   "R-R thigh", "R-R shank",
    };
    if (logical_ch >= SERVO_MAP_CHANNELS) {
        return "?";
    }
    return names[logical_ch];
}

/* ================= 小腿二次拟合曲线 ================= */

float servo_map_shank_bias(float l1, float l2, float leg_len_ref)
{
    /*
     * 复刻 padog._shank_ik_bias() 的 per_mm 分支。
     *
     * ⚠️ per_mm = **0.25**，不是函数据里那个 0.375 的兜底值。
     *    `_shank_ik_bias()` 是这样写的：
     *        per_mm = 0.375
     *        try: per_mm = float(shank_ik_bias_per_mm)
     *        except NameError: pass
     *    而 padog.py 第 61 行的**默认值注入表**里定义了
     *        ("shank_ik_bias_per_mm", 0.25)
     *    所以 `float(...)` 永远成功，实际用的是 **0.25**，兜底值 0.375 是死代码。
     *    实测：`(130+138-149) * 0.25 = 29.75`。
     *
     *    这个坑很值得记：我的第一版参考值是在一个**手搭的命名空间**里跑的，
     *    没有跑那个默认值注入表，于是参考值也算出 0.375 —— **参考值和 C 版
     *    错在同一个地方**，测试全绿却都是错的（同类问题见成长手册 P-21/P-22）。
     *    现在 `servo_output` 与 `control_chain` 两套参考值都直接 exec 真版 padog.py。
     */
    float extra = (l1 + l2) - leg_len_ref;
    if (extra < 0.0f) {
        extra = 0.0f;
    }
    return extra * 0.25f;
}

/** `_shank_ik_bias()` 里那个 per_mm 常量，导出给测试打印用 */
float servo_map_shank_bias_per_mm(void)
{
    return 0.25f;
}

float servo_map_shank_curve(float x, float leg_trim, float bias)
{
    /*
     * 原实现：
     *     x = float(x) + _shank_ik_bias() + float(leg_trim)
     *     return 0.006649*x*x + 0.4414*x + 5.53
     * 加法结合顺序照抄（先加 bias 再加 trim）—— 浮点加法不满足结合律，
     * 换个顺序就可能差最后一位，进而跨过占空比的截断边界。
     */
    const float t = (x + bias) + leg_trim;
    return 0.006649f * t * t + 0.4414f * t + 5.53f;
}

/* ================= 关节角 -> 12 路舵机角 ================= */

static float clamp_deg(float a)
{
    /* 复刻 padog._clamp_deg() */
    if (a > 180.0f) {
        return 180.0f;
    }
    if (a < 0.0f) {
        return 0.0f;
    }
    return a;
}

void servo_map_legs_to_angles(const servo_map_input_t *in,
                              float out_deg[SERVO_MAP_CHANNELS])
{
    if (in == NULL || out_deg == NULL) {
        return;
    }

    /* 腿索引：0=腿1左前, 1=腿2右前, 2=腿3右后, 3=腿4左后（原实现的编号，非物理顺序） */
    const float i1p = in->init[0][0], i1h = in->init[0][1], i1s = in->init[0][2];
    const float i2p = in->init[1][0], i2h = in->init[1][1], i2s = in->init[1][2];
    const float i3p = in->init[2][0], i3h = in->init[2][1], i3s = in->init[2][2];
    const float i4p = in->init[3][0], i4h = in->init[3][1], i4s = in->init[3][2];

    const float h1 = in->hip[0], h2 = in->hip[1], h3 = in->hip[2], h4 = in->hip[3];

    if (in->ik_path) {
        const float ham1 = in->ham[0], ham2 = in->ham[1], ham3 = in->ham[2], ham4 = in->ham[3];
        const float sh1  = in->shank[0], sh2 = in->shank[1], sh3 = in->shank[2], sh4 = in->shank[3];
        const float cs1  = in->crawl_cs[0], cs2 = in->crawl_cs[1];
        const float cs3  = in->crawl_cs[2], cs4 = in->crawl_cs[3];

        /* 原 servo_output(case=0, init=0) 分支，逐行对应：
         *   angle(0, clamp(init_1p + h1))
         *   angle(1, init_1h + 90 - ham1)                      <- 大腿不夹
         *   angle(2, clamp((init_1s - 90) + cal_test_shank(shank1,...) + cs1))
         * 注意 ±90 与正负号在四条腿之间**不对称**，这是原实现如此。 */
        out_deg[0]  = clamp_deg(i1p + h1);
        out_deg[1]  = i1h + 90.0f - ham1;
        out_deg[2]  = clamp_deg((i1s - 90.0f) + servo_map_shank_curve(sh1, in->s_trim[0], in->shank_bias) + cs1);

        out_deg[6]  = clamp_deg(i2p + h2);
        out_deg[7]  = i2h - 90.0f + ham2;
        out_deg[8]  = clamp_deg((i2s + 90.0f) - servo_map_shank_curve(sh2, in->s_trim[1], in->shank_bias) + cs2);

        out_deg[9]  = clamp_deg(i3p + h3);
        out_deg[10] = i3h - 90.0f + ham3;
        out_deg[11] = clamp_deg((i3s + 90.0f) - servo_map_shank_curve(sh3, in->s_trim[2], in->shank_bias) + cs3);

        out_deg[3]  = clamp_deg(i4p + h4);
        out_deg[4]  = i4h + 90.0f - ham4;
        out_deg[5]  = clamp_deg((i4s - 90.0f) + servo_map_shank_curve(sh4, in->s_trim[3], in->shank_bias) + cs4);
    } else {
        /* 原 else 分支：直接站姿，只有髋叠加 h，大腿/小腿就是中位角本身 */
        out_deg[0]  = clamp_deg(i1p + h1);
        out_deg[1]  = i1h;
        out_deg[2]  = i1s;

        out_deg[6]  = clamp_deg(i2p + h2);
        out_deg[7]  = i2h;
        out_deg[8]  = i2s;

        out_deg[9]  = clamp_deg(i3p + h3);
        out_deg[10] = i3h;
        out_deg[11] = i3s;

        out_deg[3]  = clamp_deg(i4p + h4);
        out_deg[4]  = i4h;
        out_deg[5]  = i4s;
    }
}

void servo_map_legs_to_pwm(const servo_map_input_t *in,
                           uint16_t on_out[SERVO_MAP_CHANNELS],
                           uint16_t off_out[SERVO_MAP_CHANNELS])
{
    float deg[SERVO_MAP_CHANNELS];

    if (in == NULL || on_out == NULL || off_out == NULL) {
        return;
    }

    servo_map_legs_to_angles(in, deg);

    for (uint8_t ch = 0; ch < SERVO_MAP_CHANNELS; ++ch) {
        const uint16_t duty = servo_map_deg_to_duty(deg[ch]);
        servo_map_duty_to_pwm(duty, &on_out[ch], &off_out[ch]);
    }
}
