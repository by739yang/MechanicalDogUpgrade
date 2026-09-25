/**
 * @file    app_config.c
 * @brief   配置默认值 / 校验 / CRC / 持久化
 *
 * 默认值**照抄**学长的 `config.py` 与 `config_s.py`（实测标定值），不重新推导：
 *
 *   config.py    : Ts=1  faai=0.5  pit_max_ang=15  rol_max_ang=15  xs_max=80
 *   config_s.py  : init_1p=102 init_1h=84 init_1s=92 / 2: 96,91,85 / 3: 108,78,68 / 4: 92,98,102
 *                  l1=130 l2=138 l=230 b=120 w=220 speed=0.065 h=65
 *                  Kp_H=0.06 Kp_G=0.03 CG_X=0 CG_Y=28 walk_h=63 walk_speed=0
 *                  ma_case=0 leg_len_ref=149 joy_fwd_sign=-1
 *                  trot_cg_f=0.52 trot_cg_b=0.85 trot_cg_t=0 faai=0.42
 *                  hip_k_roll=0.06 hip_k_pitch=0.02 hip_k_turn=0.75 hip_delta_max=18.0
 *                  arm_* 见下  H_goal=81 in_y=18 in_pit=0 in_rol=0 cal_leg_sel=2
 *
 * 注意：`config_s.py` 的 `faai=0.42` **覆盖**了 `config.py` 的 `0.5`（padog 先 exec
 * config.py 再 exec config_s.py），所以默认 TROT 占空比取 **0.42**。
 *
 * 除了两个 config 文件，默认值还有第二个来源：`padog.py` 第 57~81 行的
 * **默认值注入表**（`if _hk not in _g: _g[_hk] = _hd`）。那张表里有 64 个键，
 * 其中 **24 个**两个 config 文件都没定义 —— 那些键就是原版真正的出厂默认值。
 * 本文件里 `shank_ik_bias_*` / `*_leg_y_offset` / `s_trim` / `leg*_z_mul` /
 * `walk_speed_scale` / `*_roll_trim` / `trot_right_h_mul` 与 7 个 `arm_*` 共
 * 19 个字段就来自那里；值一律与 `control_chain_cfg_defaults()` 保持一致，
 * 并由 `tools/golden/test_app_config.c` 逐字段断言（不靠人眼对照）。
 */

#include "app/app_config.h"

#include <stdio.h>
#include <string.h>

/* ==========================================================================
 * 默认值
 * ========================================================================== */

void app_config_defaults(app_config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }

    /* 全清零：保证 padding 字节确定，CRC 才稳定 */
    memset(cfg, 0, sizeof(*cfg));

    cfg->version  = APP_CFG_VERSION;
    cfg->reserved = 0;
    cfg->crc32    = 0;

    /* 舵机中位角 [腿][髋/大/小]，腿序 = 腿1左前 / 腿2右前 / 腿3右后 / 腿4左后 */
    static const float centers[APP_CFG_LEGS][APP_CFG_JOINTS] = {
        { 102.0f, 84.0f,  92.0f },   /* init_1p / init_1h / init_1s */
        {  96.0f, 91.0f,  85.0f },   /* init_2p / init_2h / init_2s */
        { 108.0f, 78.0f,  68.0f },   /* init_3p / init_3h / init_3s */
        {  92.0f, 98.0f, 102.0f },   /* init_4p / init_4h / init_4s */
    };
    memcpy(cfg->servo_center, centers, sizeof(centers));

    /* 几何 */
    cfg->l1          = 130.0f;
    cfg->l2          = 138.0f;
    cfg->l           = 230.0f;
    cfg->b           = 120.0f;
    cfg->w           = 220.0f;
    cfg->leg_len_ref = 149.0f;

    /* 站姿与姿态 */
    cfg->h_goal      = 81.0f;
    cfg->in_y        = 18.0f;
    cfg->in_pit      = 0.0f;
    cfg->in_rol      = 0.0f;
    cfg->pit_max_ang = 15.0f;
    cfg->rol_max_ang = 15.0f;
    cfg->xs_max      = 80.0f;
    cfg->cg_x        = 0.0f;
    cfg->cg_y        = 28.0f;
    cfg->kp_h        = 0.06f;
    cfg->kp_g        = 0.03f;

    /* 步态 */
    cfg->ts          = 1.0f;
    cfg->faai        = 0.42f;    /* config_s.py 覆盖 config.py 的 0.5 */
    cfg->speed       = 0.065f;
    cfg->h           = 65.0f;
    cfg->trot_cg_f   = 0.52f;
    cfg->trot_cg_b   = 0.85f;
    cfg->trot_cg_t   = 0.0f;
    cfg->walk_faai   = 0.30f;    /* padog.py 默认 walk_faai */
    cfg->walk_h      = 63.0f;
    cfg->walk_speed  = 0.0f;

    /* 髋辅助与转向 */
    cfg->hip_k_roll    = 0.06f;
    cfg->hip_k_pitch   = 0.02f;
    cfg->hip_k_turn    = 0.75f;
    cfg->hip_delta_max = 18.0f;

    /* 逆运动学 / 摇杆 / 标定 */
    cfg->ma_case      = 0;
    cfg->joy_fwd_sign = -1;
    cfg->cal_leg_sel  = 2;

    /* ---------- padog.py 默认值注入表：控制链相关 ----------
     * config.py / config_s.py 里**没有**这些键，原版靠 padog.py 第 57~81 行的
     * 注入表兜底，所以这些值就是这台机器的出厂默认。取值一律照抄
     * `control_chain_cfg_defaults()`（同一个仓库、同一份注入表），不在这里另算一份。
     */
    cfg->shank_ik_bias_per_mm = 0.25f;   /* 不是 _shank_ik_bias() 里的死代码 0.375 */
    cfg->shank_ik_bias_deg    = 0.0f;
    cfg->front_leg_y_offset   = 0.0f;
    cfg->rear_leg_y_offset    = 0.0f;
    for (int i = 0; i < APP_CFG_LEGS; ++i) {
        cfg->s_trim[i] = 0.0f;           /* leg1_s_trim .. leg4_s_trim */
    }
    cfg->leg2_z_mul       = 1.0f;        /* 注入表里没有 leg1_z_mul */
    cfg->leg3_z_mul       = 1.0f;
    cfg->leg4_z_mul       = 1.0f;
    cfg->walk_speed_scale = 1.4f;
    cfg->walk_roll_trim   = 3.0f;
    cfg->trot_roll_trim   = 0.0f;
    cfg->trot_right_h_mul = 0.80f;

    /* 机械臂 */
    cfg->arm_upper_init  = 145.0f;
    cfg->arm_fore_init   = 125.0f;
    cfg->arm_upper_min   = 0.0f;
    cfg->arm_upper_max   = 180.0f;
    cfg->arm_fore_min    = 30.0f;
    cfg->arm_fore_max    = 140.0f;
    cfg->arm_upper_rate  = 2.5f;
    cfg->arm_fore_rate   = 2.5f;
    cfg->arm_grip_open   = 90.0f;
    cfg->arm_grip_close  = 180.0f;
    cfg->arm_upper_ch    = 6;
    cfg->arm_fore_ch     = 7;
    cfg->arm_grip_ch     = 6;
    cfg->arm_upper_board = 0x40;
    cfg->arm_fore_board  = 0x40;
    cfg->arm_grip_board  = 0x41;
    cfg->arm_grip_gpio   = -1;   /* -1 = 走 PCA9685 */

    /* 机械臂：只存在于 padog.py 注入表的 7 个键 */
    cfg->arm_grip_digital = 0;
    cfg->arm_grip_pwm_hz  = 50;
    cfg->arm_grip_min_us  = 500;
    cfg->arm_grip_max_us  = 2500;
    cfg->arm_upper_walk   = 145;
    cfg->arm_fore_walk    = 125;
    cfg->arm_walk_rate    = 0.15f;

    /* WiFi：纯 AP 模式，默认凭据是**通用占位值**，不是学长的真实热点凭据 */
    strncpy(cfg->ap_ssid, "RobotDog", sizeof(cfg->ap_ssid) - 1);
    strncpy(cfg->ap_password, "robotdog123", sizeof(cfg->ap_password) - 1);
}

/* ==========================================================================
 * CRC32（IEEE 802.3，逐位实现，无查表）
 * ========================================================================== */

uint32_t app_config_crc(const app_config_t *cfg)
{
    if (cfg == NULL) {
        return 0;
    }

    app_config_t tmp = *cfg;
    tmp.crc32 = 0;   /* 计算时把本字段视作 0 */

    const uint8_t *p = (const uint8_t *)&tmp;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < sizeof(tmp); ++i) {
        crc ^= p[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return ~crc;
}

/* ==========================================================================
 * 校验与限幅
 * ========================================================================== */

static void note_change(const char *name, int *changed, char *msg, size_t msg_len)
{
    if (changed != NULL) {
        ++(*changed);
    }
    if (msg != NULL && msg_len > 0 && msg[0] == '\0') {
        snprintf(msg, msg_len, "%s", name);
    }
}

static void clamp_f(float *v, float lo, float hi, const char *name,
                    int *changed, char *msg, size_t msg_len)
{
    if (*v < lo) {
        *v = lo;
        note_change(name, changed, msg, msg_len);
    } else if (*v > hi) {
        *v = hi;
        note_change(name, changed, msg, msg_len);
    }
}

static void clamp_i(int32_t *v, int32_t lo, int32_t hi, const char *name,
                    int *changed, char *msg, size_t msg_len)
{
    if (*v < lo) {
        *v = lo;
        note_change(name, changed, msg, msg_len);
    } else if (*v > hi) {
        *v = hi;
        note_change(name, changed, msg, msg_len);
    }
}

int app_config_validate(app_config_t *cfg, int *changed, char *msg, size_t msg_len)
{
    if (cfg == NULL) {
        return APP_CFG_ERR_ARG;
    }
    if (changed != NULL) {
        *changed = 0;
    }
    if (msg != NULL && msg_len > 0) {
        msg[0] = '\0';
    }

    cfg->version  = APP_CFG_VERSION;   /* 版本与保留位由本函数统一保证 */
    cfg->reserved = 0;

    /* 舵机中位角：PCA9685 输出 0..180 度 */
    const char *joint_name[APP_CFG_JOINTS] = { "hip", "thigh", "shank" };
    for (int leg = 0; leg < APP_CFG_LEGS; ++leg) {
        for (int j = 0; j < APP_CFG_JOINTS; ++j) {
            char nm[32];
            snprintf(nm, sizeof(nm), "servo_center[%d].%s", leg + 1, joint_name[j]);
            clamp_f(&cfg->servo_center[leg][j], 0.0f, 180.0f, nm,
                    changed, msg, msg_len);
        }
    }

    /* 几何 */
    clamp_f(&cfg->l1, 10.0f, 500.0f, "l1", changed, msg, msg_len);
    clamp_f(&cfg->l2, 10.0f, 500.0f, "l2", changed, msg, msg_len);
    clamp_f(&cfg->l,  10.0f, 1000.0f, "l", changed, msg, msg_len);
    clamp_f(&cfg->b,  10.0f, 1000.0f, "b", changed, msg, msg_len);
    clamp_f(&cfg->w,  10.0f, 1000.0f, "w", changed, msg, msg_len);
    clamp_f(&cfg->leg_len_ref, 1.0f, 1000.0f, "leg_len_ref", changed, msg, msg_len);

    /* 姿态限幅先夹，后面 in_pit/in_rol 依赖它 */
    clamp_f(&cfg->pit_max_ang, 0.0f, 90.0f, "pit_max_ang", changed, msg, msg_len);
    clamp_f(&cfg->rol_max_ang, 0.0f, 90.0f, "rol_max_ang", changed, msg, msg_len);
    clamp_f(&cfg->xs_max,      0.0f, 200.0f, "xs_max", changed, msg, msg_len);

    clamp_f(&cfg->h_goal, 20.0f, 250.0f, "h_goal", changed, msg, msg_len);
    clamp_f(&cfg->in_y, -200.0f, 200.0f, "in_y", changed, msg, msg_len);
    clamp_f(&cfg->in_pit, -cfg->pit_max_ang, cfg->pit_max_ang, "in_pit",
            changed, msg, msg_len);
    clamp_f(&cfg->in_rol, -cfg->rol_max_ang, cfg->rol_max_ang, "in_rol",
            changed, msg, msg_len);
    clamp_f(&cfg->cg_x, -200.0f, 200.0f, "cg_x", changed, msg, msg_len);
    clamp_f(&cfg->cg_y, -200.0f, 200.0f, "cg_y", changed, msg, msg_len);
    clamp_f(&cfg->kp_h, 0.001f, 1.0f, "kp_h", changed, msg, msg_len);
    clamp_f(&cfg->kp_g, 0.001f, 1.0f, "kp_g", changed, msg, msg_len);

    /* 步态 */
    clamp_f(&cfg->ts,   0.1f, 10.0f, "ts", changed, msg, msg_len);
    clamp_f(&cfg->faai, 0.05f, 0.95f, "faai", changed, msg, msg_len);
    clamp_f(&cfg->speed, 0.001f, 0.5f, "speed", changed, msg, msg_len);
    clamp_f(&cfg->h, 0.0f, 200.0f, "h", changed, msg, msg_len);
    clamp_f(&cfg->trot_cg_f, -5.0f, 5.0f, "trot_cg_f", changed, msg, msg_len);
    clamp_f(&cfg->trot_cg_b, -5.0f, 5.0f, "trot_cg_b", changed, msg, msg_len);
    clamp_f(&cfg->trot_cg_t, -5.0f, 5.0f, "trot_cg_t", changed, msg, msg_len);
    clamp_f(&cfg->walk_faai, 0.05f, 0.95f, "walk_faai", changed, msg, msg_len);
    clamp_f(&cfg->walk_h, 0.0f, 200.0f, "walk_h", changed, msg, msg_len);
    clamp_f(&cfg->walk_speed, 0.0f, 1.0f, "walk_speed", changed, msg, msg_len);

    /* ---------- padog.py 注入表：控制链相关 ----------
     * 限幅原则（成长手册"宁可限幅也不要让链子悄悄跑飞"）：
     *   - 每一项都给一个"再大/再小就明显是配错了"的界，界内完全不动（不干扰正常调参）；
     *   - 不做"按别的字段动态限幅"的花活（唯一例外见下面 arm_grip_min/max 的交换），
     *     免得用户把某个基准调小之后，默认值反被无声改掉。
     * 这里**只管量纲上的荒谬值**（负的系数、几百毫米的足端偏置、几十度的偏置角），
     * 不做"物理可达性"判断 —— 那是 IK 限幅层的事。
     */
    /* 小腿偏置系数：负值会把偏置反号，>2 deg/mm 会让小腿角随腿长暴走 */
    clamp_f(&cfg->shank_ik_bias_per_mm, 0.0f, 2.0f, "shank_ik_bias_per_mm",
            changed, msg, msg_len);
    /* 小腿固定角偏置：±45° 之外小腿角已无意义（默认 0） */
    clamp_f(&cfg->shank_ik_bias_deg, -45.0f, 45.0f, "shank_ik_bias_deg",
            changed, msg, msg_len);
    /* 足端竖直偏置（mm）：小腿+大腿共 268 mm，±100 mm 之内还算"偏置" */
    clamp_f(&cfg->front_leg_y_offset, -100.0f, 100.0f, "front_leg_y_offset",
            changed, msg, msg_len);
    clamp_f(&cfg->rear_leg_y_offset, -100.0f, 100.0f, "rear_leg_y_offset",
            changed, msg, msg_len);
    /* 小腿角微调：±45° 之外等于把小腿指到别的象限 */
    for (int leg = 0; leg < APP_CFG_LEGS; ++leg) {
        char nm[32];
        snprintf(nm, sizeof(nm), "leg%d_s_trim", leg + 1);
        clamp_f(&cfg->s_trim[leg], -45.0f, 45.0f, nm, changed, msg, msg_len);
    }
    /* z 缩放系数：0 或负值会让几何塌陷/翻转（控制链当前不读，但将来要接） */
    clamp_f(&cfg->leg2_z_mul, 0.1f, 5.0f, "leg2_z_mul", changed, msg, msg_len);
    clamp_f(&cfg->leg3_z_mul, 0.1f, 5.0f, "leg3_z_mul", changed, msg, msg_len);
    clamp_f(&cfg->leg4_z_mul, 0.1f, 5.0f, "leg4_z_mul", changed, msg, msg_len);
    /* 下限刻意放 0：`<=0.05` 在 control_chain 里表示"不覆盖"（退回 1.0），
     * 夹到 0.05 会把"自动"这个语义改成"恰好停在阈值上" */
    clamp_f(&cfg->walk_speed_scale, 0.0f, 5.0f, "walk_speed_scale",
            changed, msg, msg_len);
    /* 滚转微调直接叠加到姿态上（不再过 pit/rol 限位），±45° 已远超"微调"；
     * 不按 rol_max_ang 限 —— 那会在用户把 rol_max_ang 调小时把默认的 3° 一起夹掉 */
    clamp_f(&cfg->walk_roll_trim, -45.0f, 45.0f, "walk_roll_trim",
            changed, msg, msg_len);
    clamp_f(&cfg->trot_roll_trim, -45.0f, 45.0f, "trot_roll_trim",
            changed, msg, msg_len);
    /* 它只用来"把右侧抬腿压低"，>=0.999 即视为关闭；所以 0..1 才是它的语义区间，
     * 超过 1 会把要修的问题反过来（右腿比左腿还高） */
    clamp_f(&cfg->trot_right_h_mul, 0.0f, 1.0f, "trot_right_h_mul",
            changed, msg, msg_len);

    /* 髋辅助 */
    clamp_f(&cfg->hip_k_roll, -5.0f, 5.0f, "hip_k_roll", changed, msg, msg_len);
    clamp_f(&cfg->hip_k_pitch, -5.0f, 5.0f, "hip_k_pitch", changed, msg, msg_len);
    clamp_f(&cfg->hip_k_turn, -5.0f, 5.0f, "hip_k_turn", changed, msg, msg_len);
    clamp_f(&cfg->hip_delta_max, 0.0f, 90.0f, "hip_delta_max", changed, msg, msg_len);

    /* 逆运动学 / 摇杆 / 标定 */
    clamp_i(&cfg->ma_case, 0, 1, "ma_case", changed, msg, msg_len);
    clamp_i(&cfg->joy_fwd_sign, -1, 1, "joy_fwd_sign", changed, msg, msg_len);
    clamp_i(&cfg->cal_leg_sel, 1, APP_CFG_LEGS, "cal_leg_sel", changed, msg, msg_len);

    /* 机械臂 */
    clamp_f(&cfg->arm_upper_min, 0.0f, 180.0f, "arm_upper_min", changed, msg, msg_len);
    clamp_f(&cfg->arm_upper_max, 0.0f, 180.0f, "arm_upper_max", changed, msg, msg_len);
    clamp_f(&cfg->arm_fore_min, 0.0f, 180.0f, "arm_fore_min", changed, msg, msg_len);
    clamp_f(&cfg->arm_fore_max, 0.0f, 180.0f, "arm_fore_max", changed, msg, msg_len);
    /* 先保证 min <= max，再夹中位角 */
    if (cfg->arm_upper_min > cfg->arm_upper_max) {
        float t = cfg->arm_upper_min;
        cfg->arm_upper_min = cfg->arm_upper_max;
        cfg->arm_upper_max = t;
        note_change("arm_upper_min>max swapped", changed, msg, msg_len);
    }
    if (cfg->arm_fore_min > cfg->arm_fore_max) {
        float t = cfg->arm_fore_min;
        cfg->arm_fore_min = cfg->arm_fore_max;
        cfg->arm_fore_max = t;
        note_change("arm_fore_min>max swapped", changed, msg, msg_len);
    }
    clamp_f(&cfg->arm_upper_init, cfg->arm_upper_min, cfg->arm_upper_max,
            "arm_upper_init", changed, msg, msg_len);
    clamp_f(&cfg->arm_fore_init, cfg->arm_fore_min, cfg->arm_fore_max,
            "arm_fore_init", changed, msg, msg_len);
    clamp_f(&cfg->arm_upper_rate, 0.01f, 90.0f, "arm_upper_rate", changed, msg, msg_len);
    clamp_f(&cfg->arm_fore_rate, 0.01f, 90.0f, "arm_fore_rate", changed, msg, msg_len);
    clamp_f(&cfg->arm_grip_open, 0.0f, 180.0f, "arm_grip_open", changed, msg, msg_len);
    clamp_f(&cfg->arm_grip_close, 0.0f, 180.0f, "arm_grip_close", changed, msg, msg_len);
    clamp_i(&cfg->arm_upper_ch, 0, 15, "arm_upper_ch", changed, msg, msg_len);
    clamp_i(&cfg->arm_fore_ch, 0, 15, "arm_fore_ch", changed, msg, msg_len);
    clamp_i(&cfg->arm_grip_ch, 0, 15, "arm_grip_ch", changed, msg, msg_len);
    clamp_i(&cfg->arm_grip_gpio, -1, 39, "arm_grip_gpio", changed, msg, msg_len);
    /* PCA9685 地址只允许 0x40 / 0x41 */
    int32_t *boards[3] = { &cfg->arm_upper_board, &cfg->arm_fore_board,
                           &cfg->arm_grip_board };
    const char *bnames[3] = { "arm_upper_board", "arm_fore_board", "arm_grip_board" };
    for (int i = 0; i < 3; ++i) {
        if (*boards[i] != 0x40 && *boards[i] != 0x41) {
            *boards[i] = 0x40;
            note_change(bnames[i], changed, msg, msg_len);
        }
    }

    /* 机械臂：只存在于 padog.py 注入表的 7 个键 */
    clamp_i(&cfg->arm_grip_digital, 0, 1, "arm_grip_digital", changed, msg, msg_len);
    clamp_i(&cfg->arm_grip_pwm_hz, 1, 1000, "arm_grip_pwm_hz", changed, msg, msg_len);
    /* 脉宽 0 会让定时器算不出占空比；20000 us 已超过 50 Hz 的一整个周期 */
    clamp_i(&cfg->arm_grip_min_us, 1, 20000, "arm_grip_min_us", changed, msg, msg_len);
    clamp_i(&cfg->arm_grip_max_us, 1, 20000, "arm_grip_max_us", changed, msg, msg_len);
    if (cfg->arm_grip_min_us > cfg->arm_grip_max_us) {
        const int32_t t = cfg->arm_grip_min_us;
        cfg->arm_grip_min_us = cfg->arm_grip_max_us;
        cfg->arm_grip_max_us = t;
        note_change("arm_grip_min>max swapped", changed, msg, msg_len);
    }
    /* WALK 姿态角按大/小臂自己的 min..max 夹（与 arm_upper_init / arm_fore_init 同规矩） */
    clamp_i(&cfg->arm_upper_walk, (int32_t)cfg->arm_upper_min, (int32_t)cfg->arm_upper_max,
            "arm_upper_walk", changed, msg, msg_len);
    clamp_i(&cfg->arm_fore_walk, (int32_t)cfg->arm_fore_min, (int32_t)cfg->arm_fore_max,
            "arm_fore_walk", changed, msg, msg_len);
    /* 与 arm_upper_rate / arm_fore_rate 同一区间 */
    clamp_f(&cfg->arm_walk_rate, 0.01f, 90.0f, "arm_walk_rate", changed, msg, msg_len);

    /* WiFi：保证 NUL 结尾并夹长度 */
    cfg->ap_ssid[sizeof(cfg->ap_ssid) - 1] = '\0';
    cfg->ap_password[sizeof(cfg->ap_password) - 1] = '\0';
    const size_t ssid_len = strlen(cfg->ap_ssid);
    if (ssid_len == 0 || ssid_len > 32) {
        strncpy(cfg->ap_ssid, "RobotDog", sizeof(cfg->ap_ssid) - 1);
        cfg->ap_ssid[sizeof(cfg->ap_ssid) - 1] = '\0';
        note_change("ap_ssid", changed, msg, msg_len);
    }
    /* WPA2 要求 8..63 字符；太短就回落默认（留空表示开放网络，这里不允许） */
    const size_t pw_len = strlen(cfg->ap_password);
    if (pw_len < 8 || pw_len > 63) {
        strncpy(cfg->ap_password, "robotdog123", sizeof(cfg->ap_password) - 1);
        cfg->ap_password[sizeof(cfg->ap_password) - 1] = '\0';
        note_change("ap_password", changed, msg, msg_len);
    }

    return APP_CFG_OK;
}

int app_config_nudge_servo_center(app_config_t *cfg, app_cfg_joint_t joint, float delta,
                                  int *changed, char *msg, size_t msg_len)
{
    if (changed != NULL) {
        *changed = 0;
    }
    if (msg != NULL && msg_len > 0) {
        msg[0] = '\0';
    }
    if (cfg == NULL || (int)joint < 0 || (int)joint >= APP_CFG_JOINTS) {
        return APP_CFG_ERR_ARG;
    }
    /* 选中的腿 = `cal_leg_sel`（原版 `web_c.py:138 / 520` 的 `user_leg_num`，1..4） */
    if (cfg->cal_leg_sel < 1 || cfg->cal_leg_sel > APP_CFG_LEGS) {
        return APP_CFG_ERR_ARG;
    }

    const int leg = (int)cfg->cal_leg_sel - 1;
    cfg->servo_center[leg][(int)joint] += delta;

    /* 限幅/校验**只有这一处实现**（`app_config_validate` 里那个 0..180 循环）。
     * 本函数因此不重复写边界、也不重复写"报改动"的逻辑：`changed`/`msg` 的语义与
     * `cfg set` 那条路完全一致（P-22/P-27：同一份限幅写两份，就有一份会是错的）。 */
    return app_config_validate(cfg, changed, msg, msg_len);
}

/* ==========================================================================
 * 持久化
 * ========================================================================== */

const char *app_cfg_strerror(int rc)
{
    switch (rc) {
    case APP_CFG_OK:          return "OK";
    case APP_CFG_ERR_NOENT:   return "no saved config (using defaults)";
    case APP_CFG_ERR_CRC:     return "CRC mismatch (using defaults)";
    case APP_CFG_ERR_VERSION: return "version mismatch (using defaults)";
    case APP_CFG_ERR_IO:      return "storage I/O error";
    case APP_CFG_ERR_ARG:     return "bad argument";
    default:                  return "unknown error";
    }
}

int app_config_load(app_config_t *cfg, const app_cfg_store_t *st)
{
    if (cfg == NULL) {
        return APP_CFG_ERR_ARG;
    }
    if (st == NULL || st->get_blob == NULL) {
        app_config_defaults(cfg);
        app_config_validate(cfg, NULL, NULL, 0);
        return APP_CFG_ERR_IO;
    }

    /* 先备一份默认值：任何一步失败都直接回落，绝不留半可信状态 */
    app_config_t tmp;
    app_config_defaults(&tmp);

    size_t len = sizeof(tmp);
    const int rc = st->get_blob(APP_CFG_NVS_KEY, &tmp, &len);

    int result = APP_CFG_OK;
    if (rc == APP_CFG_ERR_NOENT) {
        result = APP_CFG_ERR_NOENT;
        app_config_defaults(&tmp);
    } else if (rc != APP_CFG_OK) {
        result = APP_CFG_ERR_IO;
        app_config_defaults(&tmp);
    } else if (len != sizeof(tmp)) {
        /* 结构大小变了（改过结构体但没升版本，或读到旧数据） */
        result = APP_CFG_ERR_VERSION;
        app_config_defaults(&tmp);
    } else if (tmp.version != APP_CFG_VERSION) {
        result = APP_CFG_ERR_VERSION;
        app_config_defaults(&tmp);
    } else if (tmp.crc32 != app_config_crc(&tmp)) {
        result = APP_CFG_ERR_CRC;
        app_config_defaults(&tmp);
    }

    /* 无论走哪条路，最后都过一遍校验限幅 */
    app_config_validate(&tmp, NULL, NULL, 0);
    *cfg = tmp;
    return result;
}

int app_config_save(app_config_t *cfg, const app_cfg_store_t *st)
{
    if (cfg == NULL) {
        return APP_CFG_ERR_ARG;
    }
    if (st == NULL || st->set_blob == NULL) {
        return APP_CFG_ERR_IO;
    }

    /* 存之前先校验：绝不把非法值写进 Flash */
    app_config_validate(cfg, NULL, NULL, 0);

    cfg->version = APP_CFG_VERSION;
    cfg->crc32   = 0;
    cfg->crc32   = app_config_crc(cfg);

    return st->set_blob(APP_CFG_NVS_KEY, cfg, sizeof(*cfg));
}

int app_config_reset(app_config_t *cfg, const app_cfg_store_t *st)
{
    if (cfg == NULL) {
        return APP_CFG_ERR_ARG;
    }

    int rc = APP_CFG_OK;
    if (st != NULL && st->erase_key != NULL) {
        rc = st->erase_key(APP_CFG_NVS_KEY);
    }

    app_config_defaults(cfg);
    app_config_validate(cfg, NULL, NULL, 0);

    /* 擦除失败也不算致命：默认值已经装好，返回失败让调用方决定要不要报错 */
    return (rc == APP_CFG_OK || rc == APP_CFG_ERR_NOENT) ? APP_CFG_OK : rc;
}
