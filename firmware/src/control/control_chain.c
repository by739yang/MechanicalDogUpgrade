/**
 * @file    control_chain.c
 * @brief   逐行对齐 micropython/padog.py 的 mainloop()（P3 全链路）
 *
 * 原实现的结构（行号是 padog.py 的）：
 *
 *   862  def mainloop():
 *   877    _crawl_mainloop_service()               # 爬行状态机
 *   878    if not _crawl_active(): crawl_until_ms = 0
 *   880    if inplace_step_end_ms: ...             # ← 姿态动画层，不在本模块
 *   889    if _pose_anim_step(): return 0          # ← 同上
 *   896    if direct_pose_freeze: return 0         # ← 同上
 *   903    _gs = _geom_scale()                     # ← 算了不用（无效计算）
 *   905    if gait_mode==0: ... PA_TROT.cal_t ...  # TROT 分支
 *   926    elif gait_mode==1: ... PA_WALK.cal_w... # WALK 分支
 *   954    R_H   slew  -> H_goal
 *   959    PIT_S slew  -> PIT_goal
 *   964    ROL_S slew  -> ROL_goal
 *   969    X_S   slew  -> X_goal
 *   974    pit_max_ang / rol_max_ang 限位
 *   979    _hc = _ik_hc(R_H); _cgk = ...; _tr = _trot_rol_s()
 *   982    if/elif 选重心分支 -> PA_ATTITUDE.cal_ges
 *  1011    _foot_y_targets + PA_IK.ik
 *  1013    servo_output(...)
 *
 * ## 本文件里三处需要"解释一下"的地方
 *
 * 1. **爬行服务（第 877 行）也必须搬**。`control_chain.csv` 里有 `crawl_phase=1`
 *    的行，而原版 mainloop 一进门就调 `_crawl_mainloop_service()`：它会
 *    `move(0,0,0)` → 因为 `crawl_settle_until_ms`（模块初值 0）已经过期 →
 *    `crawl_phase=2` 且 `move(CRAWL_FWD_SPD, 1, 1)`。也就是说**那两行输入里的
 *    `spd=-3.0` 实际被改成了 -3.5**，而且 `gait(0)` 把三个重心目标重置成
 *    `in_pit/in_rol/in_y`。不搬这一段，那些行**必然对不上**。
 *
 * 2. **`gait_walk_cal_w()` 的副作用输出必须就地施加**。原 `_apply_cg()` 会执行
 *    `padog.gesture(0, int(CG_X), int(yst))` 去改重心目标，而 `cal_w()` 在 mainloop
 *    里的位置**在姿态 slew 环之前** ⇒ 原版**同一帧**就用了被改过的目标。
 *    ⇒ 本文件在 `cal_w()` 之后立刻把 `w_gesture` 写进 `w.PIT_goal/ROL_goal/X_goal`。
 *    ⇒ 并且通过 `out->goal[]` 把最终目标交回调用方，**app 层必须把它作为下一帧的
 *    `in` 目标喂回来** —— 原版这三个量是模块级全局，改一次会一直留着；
 *    在 C 版里"持续"这件事由调用方负责，不喂回来就只影响当帧。
 *    （这条踩过一次：第一版因为参考命名空间把 `padog.gesture` stub 成了 no-op，
 *    就把这个副作用丢掉了。修正参考环境后 90 行里 24 行变了，见 P-23。）
 *
 * 3. **陀螺仪恒 0**。本机没有 IMU（HANDOFF.md 记了 4 次实测），
 *    `gait_walk_cal_w()` 的 `gyro_p_deg` 固定传 0.0f。顺带一个推论：
 *    `cal_w` 里唯一用到 `body_h` 的项是 `bh * tan(sita*1.5)`，`sita=0` ⇒ 该项为 0，
 *    所以 `body_h` 取什么值都不影响输出（参考值里它来自 stub，这里传 `R_H`，
 *    语义上更正确，数值上等价）。
 */
#include "control/control_chain.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "control/body_pose.h"
#include "control/gait_trot.h"
#include "control/gait_walk.h"
#include "control/kinematics.h"

/* ==================================================================== */
/*  内部：一帧的工作副本                                                 */
/* ==================================================================== */

/**
 * @brief 本帧的工作副本 —— 相当于把 padog.py 的那几个模块级全局**拷贝一份**出来算。
 *
 * 为什么要有它：`move()` / `gait()` / `gesture()` / `height()` 在原实现里是直接改
 * 模块级全局的。目标值（`H_goal` / `PIT_goal` / `ROL_goal` / `X_goal`）在本模块里是
 * **每帧输入**（见 `control_chain.h` 的说明），所以 tick 里需要一个可写的副本，
 * 让爬行服务那几步的副作用在本帧内生效；帧结束后只有 state 里的量被写回。
 */
typedef struct {
    float t;
    float R_H;
    float PIT_S;
    float ROL_S;
    float X_S;

    float H_goal;
    float PIT_goal;
    float ROL_goal;
    float X_goal;

    float spd;
    int L;
    int R;
    int gait_mode;
    float joy_turn;

    int crawl_phase;
    int32_t crawl_until_ms;
    int32_t crawl_settle_until_ms;
    int crawl_saved_h;

    int init_case;
} chain_work_t;

/* ==================================================================== */
/*  配置默认值（config_s.py 实测值 + padog.py 第 57~81 行的注入表）        */
/* ==================================================================== */

void control_chain_cfg_defaults(control_chain_cfg_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));

    /* ---- config.py：Ts / faai / pit_max_ang / rol_max_ang / xs_max ---- */
    cfg->Ts          = 1.0f;
    cfg->faai        = 0.42f;   /* config_s.py 覆盖了 config.py 的 0.5 */
    cfg->pit_max_ang = 15.0f;
    cfg->rol_max_ang = 15.0f;
    cfg->xs_max      = 80.0f;

    /* ---- config_s.py：中位角 [腿][髋/大腿/小腿] ---- */
    cfg->init[0][0] = 102.0f; cfg->init[0][1] = 84.0f; cfg->init[0][2] = 92.0f;
    cfg->init[1][0] = 96.0f;  cfg->init[1][1] = 91.0f; cfg->init[1][2] = 85.0f;
    cfg->init[2][0] = 108.0f; cfg->init[2][1] = 78.0f; cfg->init[2][2] = 68.0f;
    cfg->init[3][0] = 92.0f;  cfg->init[3][1] = 98.0f; cfg->init[3][2] = 102.0f;

    /* ---- config_s.py：几何 / 时序 / 增益 ---- */
    cfg->l1   = 130.0f;
    cfg->l2   = 138.0f;
    cfg->l    = 230.0f;
    cfg->b    = 120.0f;
    cfg->w    = 220.0f;
    cfg->speed = 0.065f;
    cfg->h     = 65.0f;
    cfg->Kp_H  = 0.06f;
    cfg->Kp_G  = 0.03f;
    cfg->CG_X  = 0.0f;
    cfg->CG_Y  = 28.0f;
    cfg->walk_h     = 63.0f;
    cfg->walk_speed = 0.0f;   /* <=0.001 ⇒ _walk_phase_step() 用自动算出来的值 */

    cfg->ma_case     = 0;   /* 串联腿（实机在跑） */
    cfg->leg_len_ref = 149.0f;
    cfg->joy_fwd_sign = -1.0f;   /* ⚠️ config_s.py 是 -1，覆盖了注入表的 +1 */

    cfg->trot_cg_f = 0.52f;   /* 覆盖注入表的 0.38 */
    cfg->trot_cg_b = 0.85f;   /* 覆盖注入表的 1.6 */
    cfg->trot_cg_t = 0.0f;

    cfg->hip_k_roll   = 0.06f;
    cfg->hip_k_pitch  = 0.02f;
    cfg->hip_k_turn   = 0.75f;
    cfg->hip_delta_max = 18.0f;

    cfg->H_goal   = 81.0f;
    cfg->in_y     = 18.0f;
    cfg->in_pit   = 0.0f;
    cfg->in_rol   = 0.0f;
    cfg->cal_leg_sel = 2;

    /* ---- 注入表：config 文件里**没有**的键（值 = 注入表默认） ---- */
    cfg->shank_ik_bias_per_mm = 0.25f;   /* ⚠️ 不是 _shank_ik_bias() 里那个死代码 0.375 */
    cfg->shank_ik_bias_deg    = 0.0f;
    cfg->front_leg_y_offset   = 0.0f;
    cfg->rear_leg_y_offset    = 0.0f;
    cfg->s_trim[0] = 0.0f;
    cfg->s_trim[1] = 0.0f;
    cfg->s_trim[2] = 0.0f;
    cfg->s_trim[3] = 0.0f;
    cfg->leg2_z_mul = 1.0f;   /* 注入表里没有 leg1_z_mul */
    cfg->leg3_z_mul = 1.0f;
    cfg->leg4_z_mul = 1.0f;
    cfg->walk_faai       = 0.30f;
    cfg->walk_speed_scale = 1.4f;
    cfg->walk_roll_trim  = 3.0f;
    cfg->trot_roll_trim  = 0.0f;
    cfg->trot_right_h_mul = 0.80f;

    /* ---- padog.py 模块级常量 ---- */
    cfg->large_stride_xf_mul     = 0.90f;
    cfg->large_stride_geom_frac  = 0.52f;
    cfg->large_stride_xs_ratio   = 0.0f;
    cfg->large_cg_geom_frac      = 0.35f;
    cfg->large_bwd_cg_mul        = 0.50f;
    cfg->large_h_trot_mul        = 0.96f;
    cfg->hip_turn_dead           = 10.0f;
    cfg->hip_turn_stick_scale    = 8.0f;
    cfg->turn_hip_gain           = 1.0f;

    /* ---- CRAWL_*（padog.py 模块级常量） ---- */
    cfg->crawl_shank_front = 20.0f;
    cfg->crawl_shank_rear  = 30.0f;
    cfg->crawl_fwd_spd     = -3.5f;
    cfg->crawl_duration_ms = 5000;
    cfg->crawl_settle_ms   = 400;

    /* ---- 注入表里与控制链无关的键（机械臂等），搬全不半抄 ---- */
    cfg->arm_base_init   = 145;
    cfg->arm_grip_init   = 0;
    cfg->arm_upper_init  = 145;
    cfg->arm_fore_init   = 125;
    cfg->arm_upper_min   = 0;
    cfg->arm_upper_max   = 180;
    cfg->arm_fore_min    = 30;
    cfg->arm_fore_max    = 140;
    cfg->arm_upper_rate  = 2.5f;   /* config_s.py 覆盖注入表的 5.0 */
    cfg->arm_fore_rate   = 2.5f;
    cfg->arm_upper_dir   = 1;
    cfg->arm_fore_dir    = 1;
    cfg->arm_upper_ch    = 6;
    cfg->arm_fore_ch     = 7;
    cfg->arm_grip_ch     = 6;
    cfg->arm_upper_board = 0x40;   /* 注入表默认（config_s.py 没定义它） */
    cfg->arm_fore_board  = 0x40;   /* config_s.py 的 64 */
    cfg->arm_grip_board  = 0x41;
    cfg->arm_grip_gpio   = -1;
    cfg->arm_grip_open_level  = 0;
    cfg->arm_grip_close_level = 1;
    cfg->arm_grip_digital = 0;     /* 注入表默认 */
    cfg->arm_grip_pwm_hz  = 50;
    cfg->arm_grip_min_us  = 500;
    cfg->arm_grip_max_us  = 2500;
    cfg->arm_upper_walk   = 145;   /* 注入表默认 */
    cfg->arm_fore_walk    = 125;
    cfg->arm_walk_rate    = 0.15f;
    cfg->arm_base_min     = 0;
    cfg->arm_base_max     = 180;
    cfg->arm_grip_open    = 90;
    cfg->arm_grip_close   = 180;
    cfg->arm_base_rate    = 1.8f;
    cfg->arm_base_dir     = 1;
    cfg->arm_grip_stick_sign = 1;
}

/* ==================================================================== */
/*  状态 / 输入初值                                                      */
/* ==================================================================== */

void control_chain_state_init(control_chain_state_t *st, const control_chain_cfg_t *cfg)
{
    if (st == NULL) {
        return;
    }
    memset(st, 0, sizeof(*st));

    /* padog.py 第 140~158 行：H_goal=int(H_goal); R_H=H_goal */
    const int h_goal_int = (cfg != NULL) ? (int)cfg->H_goal : 0;
    st->t = 0.0f;
    st->R_H = (float)h_goal_int;
    st->PIT_S = 0.0f;
    st->ROL_S = 0.0f;
    st->X_S = 0.0f;

    st->gait_mode = 0;
    st->crawl_phase = 0;
    st->crawl_until_ms = 0;
    st->crawl_settle_until_ms = 0;
    /* padog.py: crawl_saved_h = int(H_goal) */
    st->crawl_saved_h = h_goal_int;
    st->init_case = 0;
}

void control_chain_input_init(control_chain_input_t *in, const control_chain_cfg_t *cfg)
{
    if (in == NULL) {
        return;
    }
    memset(in, 0, sizeof(*in));
    if (cfg == NULL) {
        return;
    }
    /* padog.py 第 146 行：PIT_goal=int(in_pit); ROL_goal=int(in_rol); X_goal=int(in_y) */
    in->H_goal   = (float)(int)cfg->H_goal;
    in->PIT_goal = (float)(int)cfg->in_pit;
    in->ROL_goal = (float)(int)cfg->in_rol;
    in->X_goal   = (float)(int)cfg->in_y;
    in->spd = 0.0f;
    in->L = 0;
    in->R = 0;
    in->gait_mode = 0;
    in->joy_turn = 0.0f;
    in->crawl_phase = 0;
    in->now_ms = 0;
}

/* ==================================================================== */
/*  原实现那些小函数（逐行）                                             */
/* ==================================================================== */

float control_chain_geom_scale(const control_chain_cfg_t *cfg)
{
    /* 原：ref = float(leg_len_ref); if ref < 1.0: ref = 149.0
     *     return (float(l1) + float(l2)) / ref */
    if (cfg == NULL) {
        return 0.0f;
    }
    float ref = cfg->leg_len_ref;
    if (ref < 1.0f) {
        ref = 149.0f;
    }
    return (cfg->l1 + cfg->l2) / ref;
}

float control_chain_ik_hc(const control_chain_cfg_t *cfg, float r_h)
{
    /* 原：站高 = R_H + max(0, l1+l2-leg_len_ref)。注释里专门写了"勿乘 geom_scale" */
    if (cfg == NULL) {
        return r_h;
    }
    float ref = cfg->leg_len_ref;
    if (ref < 1.0f) {
        ref = 149.0f;
    }
    float extra_len = (cfg->l1 + cfg->l2) - ref;
    if (extra_len < 0.0f) {
        extra_len = 0.0f;
    }
    return r_h + extra_len;
}

float control_chain_partial_geom_scale(const control_chain_cfg_t *cfg, float frac)
{
    /* 原：gs = _geom_scale(); f = clamp(float(frac),0,1); return 1.0 + (gs-1.0)*f */
    const float gs = control_chain_geom_scale(cfg);
    float f = frac;
    if (f < 0.0f) {
        f = 0.0f;
    }
    if (f > 1.0f) {
        f = 1.0f;
    }
    return 1.0f + (gs - 1.0f) * f;
}

bool control_chain_joy_forward_motion(const control_chain_cfg_t *cfg, float spd)
{
    /* 原：jfs = int(joy_fwd_sign); jfs<0 -> spd<0，否则 spd>0 */
    if (cfg == NULL) {
        return false;
    }
    if ((int)cfg->joy_fwd_sign < 0) {
        return spd < 0.0f;
    }
    return spd > 0.0f;
}

bool control_chain_joy_backward_motion(const control_chain_cfg_t *cfg, float spd)
{
    if (cfg == NULL) {
        return false;
    }
    if ((int)cfg->joy_fwd_sign < 0) {
        return spd > 0.0f;
    }
    return spd < 0.0f;
}

float control_chain_trot_right_h_mul(const control_chain_cfg_t *cfg)
{
    return (cfg != NULL) ? cfg->trot_right_h_mul : 0.0f;
}

void control_chain_apply_trot_swing_y(const control_chain_cfg_t *cfg, float p[8])
{
    /* 原：rm = _trot_right_h_mul(); if rm>=0.999: return p_
     *     y1,y2,y3,y4 = p_[4..7]; if y2>0.05: y2*=rm; if y3>0.05: y3*=rm
     *     return (p_[0..3], y1, y2, y3, y4)
     * 未改动的两项原样放回，所以就地改 p[5]（腿2）/ p[6]（腿3）等价。 */
    if (cfg == NULL || p == NULL) {
        return;
    }
    const float rm = control_chain_trot_right_h_mul(cfg);
    if (rm >= 0.999f) {
        return;
    }
    if (p[5] > 0.05f) {
        p[5] *= rm;
    }
    if (p[6] > 0.05f) {
        p[6] *= rm;
    }
}

float control_chain_walk_faai(const control_chain_cfg_t *cfg)
{
    /* 原：
     *   try:
     *     wf = float(walk_faai)
     *     if wf > 0.05: return wf
     *   except NameError: pass
     *   try: return float(faai) * 0.55
     *   except NameError: return 0.30
     * 注入表保证 walk_faai 与 faai 都存在 ⇒ 最后一个 return 0.30 是死代码。
     * ⚠️ 所以 wf<=0.05 时退到的是 **faai*0.55**，不是 0.30（哪怕默认值恰好是 0.30）。*/
    if (cfg == NULL) {
        return 0.0f;
    }
    const float wf = cfg->walk_faai;
    if (wf > 0.05f) {
        return wf;
    }
    return cfg->faai * 0.55f;
}

float control_chain_walk_speed_scale(const control_chain_cfg_t *cfg)
{
    /* 原：sc = float(walk_speed_scale); if sc > 0.05: return sc; return 1.0 */
    if (cfg == NULL) {
        return 1.0f;
    }
    const float sc = cfg->walk_speed_scale;
    if (sc > 0.05f) {
        return sc;
    }
    return 1.0f;
}

float control_chain_walk_roll_trim(const control_chain_cfg_t *cfg)
{
    return (cfg != NULL) ? cfg->walk_roll_trim : 0.0f;
}

float control_chain_trot_roll_trim(const control_chain_cfg_t *cfg)
{
    return (cfg != NULL) ? cfg->trot_roll_trim : 0.0f;
}

float control_chain_walk_phase_step(const control_chain_cfg_t *cfg)
{
    /* 原：
     *   sp, ts, fa = float(speed), float(Ts), _walk_faai()
     *   denom = ts - sp
     *   if denom < 0.02: denom = ts if ts > 0.02 else 1.0
     *   auto = sp * (4.0*fa*ts) / denom * _walk_speed_scale()
     *   if walk_speed > 0.001: return walk_speed
     *   return auto
     * 乘除顺序照抄（浮点不满足结合律，换个顺序就可能跨过占空比的截断边界）。 */
    if (cfg == NULL) {
        return 0.0f;
    }
    const float sp = cfg->speed;
    const float ts = cfg->Ts;
    const float fa = control_chain_walk_faai(cfg);
    float denom = ts - sp;
    if (denom < 0.02f) {
        denom = (ts > 0.02f) ? ts : 1.0f;
    }
    const float auto_step = sp * (4.0f * fa * ts) / denom * control_chain_walk_speed_scale(cfg);
    const float ws = cfg->walk_speed;
    if (ws > 0.001f) {
        return ws;
    }
    return auto_step;
}

float control_chain_walk_rol_s(const control_chain_cfg_t *cfg,
                               float rol_s, float joy_turn, float spd)
{
    /* 原 _walk_rol_s()：转向摇杆有效 / 速度太小 时原样返回，否则加 walk_roll_trim */
    if (cfg == NULL) {
        return rol_s;
    }
    if (fabsf(joy_turn) >= cfg->hip_turn_dead) {
        return rol_s;
    }
    if (fabsf(spd) < 0.05f) {
        return rol_s;
    }
    return rol_s + control_chain_walk_roll_trim(cfg);
}

float control_chain_trot_rol_s(const control_chain_cfg_t *cfg,
                               float rol_s, float joy_turn, float spd)
{
    if (cfg == NULL) {
        return rol_s;
    }
    if (fabsf(joy_turn) >= cfg->hip_turn_dead) {
        return rol_s;
    }
    if (fabsf(spd) < 0.05f) {
        return rol_s;
    }
    return rol_s + control_chain_trot_roll_trim(cfg);
}

control_chain_turn_lr_t control_chain_trot_turn_lr(const control_chain_cfg_t *cfg,
                                                  float spd, float joy_turn)
{
    /* 原 _trot_turn_lr()：三个分支**返回值完全相同**（全 1,1,1,1）—— 无效分支（P-19）。
     * 条件照算、分支照写，不做"清理"：将来原实现若改了某条分支，对照关系还在。 */
    control_chain_turn_lr_t lr;
    lr.lr1 = 1.0f;
    lr.lr4 = 1.0f;
    lr.lr2 = 1.0f;
    lr.lr3 = 1.0f;
    if (cfg == NULL) {
        return lr;
    }
    if (fabsf(joy_turn) >= cfg->hip_turn_dead) {
        return lr;
    }
    if (control_chain_joy_backward_motion(cfg, spd) && fabsf(spd) > 0.35f) {
        return lr;
    }
    return lr;
}

void control_chain_turn_phase_lr(const control_chain_cfg_t *cfg, float joy_turn,
                                 int *out_l, int *out_r)
{
    /* 原 _turn_phase_lr(jt)：jt>10 → (-1,1) 左转；jt<-10 → (1,-1) 右转；否则 (1,1)。
     * ⚠️ mainloop 不调用它 —— 是 padog.turn_hip() 用来算 L/R 的。 */
    if (out_l == NULL || out_r == NULL) {
        return;
    }
    const float dead = (cfg != NULL) ? cfg->hip_turn_dead : 0.0f;
    if (joy_turn > dead) {
        *out_l = -1;
        *out_r = 1;
        return;
    }
    if (joy_turn < -dead) {
        *out_l = 1;
        *out_r = -1;
        return;
    }
    *out_l = 1;
    *out_r = 1;
}

/** 原 `_clip()`：夹到 ±dmx（严格 > / <，与原实现一致） */
static float chain_clip(float x, float dmx)
{
    if (x > dmx) {
        return dmx;
    }
    if (x < -dmx) {
        return -dmx;
    }
    return x;
}

void control_chain_hip_leg_deltas(const control_chain_cfg_t *cfg,
                                  float rol_s, float pit_s, float joy_turn,
                                  float delta[CONTROL_CHAIN_LEGS])
{
    /* 原 _hip_leg_deltas()：jt>0 左转时 turn_mag 取反；四腿的符号组合是不对称的 */
    if (cfg == NULL || delta == NULL) {
        return;
    }
    const float r = rol_s;
    const float p = pit_s;
    const float jt = joy_turn;
    float turn_mag = 0.0f;
    if (fabsf(jt) >= cfg->hip_turn_dead) {
        turn_mag = (fabsf(jt) / cfg->hip_turn_stick_scale) * cfg->turn_hip_gain;
        if (jt > 0.0f) {
            turn_mag = -turn_mag;
        }
    }
    const float kr = cfg->hip_k_roll;
    const float kp = cfg->hip_k_pitch;
    const float kt = cfg->hip_k_turn;
    const float dmx = cfg->hip_delta_max;

    delta[0] = chain_clip(kr * r + kp * p + kt * turn_mag, dmx);
    delta[1] = chain_clip(-kr * r + kp * p - kt * turn_mag, dmx);
    delta[2] = chain_clip(-kr * r - kp * p - kt * turn_mag, dmx);
    delta[3] = chain_clip(kr * r - kp * p + kt * turn_mag, dmx);
}

bool control_chain_crawl_active(int crawl_phase)
{
    /* 原 _crawl_active()：int(crawl_phase) != 0 */
    return crawl_phase != 0;
}

float control_chain_crawl_shank_servodelta(const control_chain_cfg_t *cfg,
                                           int crawl_phase, int leg_n)
{
    /* 原 _crawl_shank_servodelta(leg_n)：
     *   n in (1,4) -> -front / -rear（前腿 -20、后腿 -30）
     *   其它       -> +front / +rear（腿2 +20、腿3 +30）
     * 符号是原实现的不对称写法（对应 servo_output 里 ±cal_test_shank）。 */
    if (cfg == NULL) {
        return 0.0f;
    }
    if (!control_chain_crawl_active(crawl_phase)) {
        return 0.0f;
    }
    if (leg_n == 1 || leg_n == 4) {
        return -((leg_n == 1) ? cfg->crawl_shank_front : cfg->crawl_shank_rear);
    }
    return (leg_n == 2) ? cfg->crawl_shank_front : cfg->crawl_shank_rear;
}

void control_chain_foot_y_targets(const control_chain_cfg_t *cfg,
                                 const float p_y[CONTROL_CHAIN_LEGS],
                                 float fy[CONTROL_CHAIN_LEGS])
{
    /* 原 _foot_y_targets()：腿1/腿2 加 front_leg_y_offset，腿3/腿4 加 rear_leg_y_offset */
    if (cfg == NULL || p_y == NULL || fy == NULL) {
        return;
    }
    fy[0] = p_y[0] + cfg->front_leg_y_offset;
    fy[1] = p_y[1] + cfg->front_leg_y_offset;
    fy[2] = p_y[2] + cfg->rear_leg_y_offset;
    fy[3] = p_y[3] + cfg->rear_leg_y_offset;
}

/* ==================================================================== */
/*  padog 的"动作"函数：move / gait / servo_init / height / gesture      */
/*  （原实现直接改模块级全局，这里改工作副本）                             */
/* ==================================================================== */

/** 原 `servo_init(key)` */
static void chain_servo_init(chain_work_t *w, int key)
{
    w->init_case = key;
}

/** 原 `gait(mode)`：切模式时相位归零；切到 0（TROT）时把重心目标重置回 in_* */
static void chain_gait(chain_work_t *w, const control_chain_cfg_t *cfg, int mode)
{
    const int nm = mode;
    if (nm != w->gait_mode) {
        w->t = 0.0f;
    }
    if (nm == 0) {
        w->PIT_goal = (float)(int)cfg->in_pit;
        w->ROL_goal = (float)(int)cfg->in_rol;
        w->X_goal = (float)(int)cfg->in_y;
    }
    w->gait_mode = nm;
}

/**
 * 原 `move(spd_, L_, R_)`。
 *
 * ⚠️ 原实现后面还有两句 `direct_pose_freeze = False` 与 `inplace_step_end_ms = 0` ——
 * 那两个量属于**姿态动画层**，本模块不持有（见 `control_chain.h` 的"本函数不包含"）。
 */
static void chain_move(chain_work_t *w, const control_chain_cfg_t *cfg,
                       float spd_, int L_, int R_)
{
    w->spd = spd_;
    w->L = L_;
    w->R = R_;
    if ((L_ + R_) != 0 && fabsf(spd_) > 0.0f) {
        chain_gait(w, cfg, 0);
        chain_servo_init(w, 0);
    }
}

/** 原 `height(goal)`：目标与当前站高**一起**写成 goal（网页滑条不再"拖很久"） */
static void chain_height(chain_work_t *w, float goal)
{
    w->H_goal = goal;
    w->R_H = goal;
}

/** 原 `gesture(PIT,ROL,X)` */
static void chain_gesture(chain_work_t *w, float pit, float rol, float x)
{
    w->PIT_goal = pit;
    w->ROL_goal = rol;
    w->X_goal = x;
}

/**
 * 原 `_crawl_finish()`：爬行收尾 —— 停车、回 TROT、恢复站高与姿态、清偏置。
 *
 * ⚠️ golden 的可达域里**走不到这里**（要 `crawl_phase==2` 输入且 `crawl_until_ms`
 * 已过期；CSV 只喂 0/1）。搬过来是为了爬行功能在真机上完整。
 *
 * ⚠️ 原实现最后一句 `set_leg_sit_offsets(0, 0)` 把前后腿偏置清零；C 版把偏置放在
 * **只读 cfg** 里，所以这一句没有对应动作 —— 默认值本来就是 0，而 `padog.py` 里
 * `set_leg_sit_offsets()` 的调用点（`action_stand` / `action_sit*` / `_crawl_finish`）
 * 传的都是 0。将来若要支持运行时改偏置，得把它挪进 state。
 */
static void chain_crawl_finish(chain_work_t *w, const control_chain_cfg_t *cfg)
{
    w->crawl_phase = 0;
    w->crawl_until_ms = 0;
    w->crawl_settle_until_ms = 0;
    chain_move(w, cfg, 0.0f, 0, 0);
    chain_gait(w, cfg, 0);
    chain_height(w, (float)w->crawl_saved_h);
    chain_gesture(w, (float)(int)cfg->in_pit, (float)(int)cfg->in_rol, (float)(int)cfg->in_y);
}

/**
 * 原 `_crawl_mainloop_service()` —— mainloop 的第一件事。
 *
 * 阶段 1（下蹲稳定）：先停车；`crawl_settle_until_ms` 一到就进阶段 2 并**重发前进命令**
 * （每帧重发，避免网页摇杆的 `move(0)` 把它打断）。
 * 阶段 2（前进）：每帧 `gait(0)` + `move(CRAWL_FWD_SPD,1,1)`；到点收尾。
 *
 * ⚠️ 时刻比较用的是 `utime.ticks_diff(a,b) = a-b`：`int32_t` 相减。本机模块初值
 * 两个截止时刻都是 0，所以阶段 1 第一次调用就满足 `0 - now <= 0` ⇒ 直接进阶段 2。
 */
static void chain_crawl_service(chain_work_t *w, const control_chain_cfg_t *cfg,
                                int32_t now_ms)
{
    if (!control_chain_crawl_active(w->crawl_phase)) {
        return;
    }
    if (w->crawl_phase == 1) {
        chain_move(w, cfg, 0.0f, 0, 0);
        if ((int32_t)(w->crawl_settle_until_ms - now_ms) <= 0) {
            w->crawl_phase = 2;
            chain_gait(w, cfg, 0);
            chain_move(w, cfg, cfg->crawl_fwd_spd, 1, 1);
        }
    } else if (w->crawl_phase == 2) {
        chain_gait(w, cfg, 0);
        chain_move(w, cfg, cfg->crawl_fwd_spd, 1, 1);
        if ((int32_t)(w->crawl_until_ms - now_ms) <= 0) {
            chain_crawl_finish(w, cfg);
        }
    }
}

/* ==================================================================== */
/*  主入口                                                              */
/* ==================================================================== */

void control_chain_tick(const control_chain_cfg_t *cfg,
                        control_chain_state_t *st,
                        const control_chain_input_t *in,
                        control_chain_out_t *out)
{
    if (cfg == NULL || st == NULL || in == NULL || out == NULL) {
        return;
    }

    memset(out, 0, sizeof(*out));

    /* ---- 每帧输入 -> 工作副本（等价于原实现里那些模块级全局 + 上层写入） ---- */
    chain_work_t w;
    w.t = st->t;
    w.R_H = st->R_H;
    w.PIT_S = st->PIT_S;
    w.ROL_S = st->ROL_S;
    w.X_S = st->X_S;
    w.H_goal = in->H_goal;
    w.PIT_goal = in->PIT_goal;
    w.ROL_goal = in->ROL_goal;
    w.X_goal = in->X_goal;
    w.spd = in->spd;
    w.L = in->L;
    w.R = in->R;
    w.gait_mode = in->gait_mode;
    w.joy_turn = in->joy_turn;
    w.crawl_phase = in->crawl_phase;
    w.crawl_until_ms = st->crawl_until_ms;
    w.crawl_settle_until_ms = st->crawl_settle_until_ms;
    w.crawl_saved_h = st->crawl_saved_h;
    w.init_case = st->init_case;

    /* ---- 第 877 行：爬行服务 ---- */
    chain_crawl_service(&w, cfg, in->now_ms);

    /* ---- 第 878~879 行 ---- */
    if (!control_chain_crawl_active(w.crawl_phase)) {
        w.crawl_until_ms = 0;
    }

    /* ---- 第 903 行：`_gs = _geom_scale()`，算完就没再用过（P-19 那类无效计算）。
     *      照样调用一次（纯函数、无副作用），结果显式丢弃，保持逐行对应。 ---- */
    const float gs_unused = control_chain_geom_scale(cfg);
    (void)gs_unused;

    /* ---- 第 905~951 行：步态分支 ---- */
    /* P_：x1..x4 在 [0..3]，y1..y4 在 [4..7]。原实现只在 gait_mode 0/1 里赋值，
     * 其它值会 NameError；C 版退回全 0（与 gait_trot.c 处理越界输入的风格一致）。 */
    float p_gait[8] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    if (w.gait_mode == 0) {
        /* ---- TROT ---- */
        if (w.L == 0 && w.R == 0) {
            w.t = 0.0f;
        } else if (w.spd != 0.0f) {
            w.t = w.t + cfg->speed;
            if (w.t >= cfg->Ts) {
                w.t = w.t - cfg->Ts;
            }
        }
        float h_trot = cfg->h * cfg->large_h_trot_mul;
        if ((w.L + w.R) != 0 && w.spd != 0.0f) {
            const float s = fabsf(w.spd);
            h_trot *= fmaxf(0.62f, fminf(0.92f, s / 5.5f));
            if (!control_chain_joy_forward_motion(cfg, w.spd)) {
                h_trot *= 0.82f;
            }
        }
        const float xgs = control_chain_partial_geom_scale(cfg, cfg->large_stride_geom_frac);
        const float xf = w.spd * 10.0f * cfg->large_stride_xf_mul * xgs;
        float xs = 0.0f;
        if (w.spd != 0.0f) {
            xs = -cfg->large_stride_xs_ratio * xf;
        }
        const control_chain_turn_lr_t lr =
            control_chain_trot_turn_lr(cfg, w.spd, w.joy_turn);

        /* ⚠️ 形参顺序是 (t, xs, xf, h, r1, r4, r2, r3) —— 与原实现一致，勿整理 */
        gait_trot_cfg_t tcfg;
        tcfg.ts = cfg->Ts;
        tcfg.faai = cfg->faai;
        gait_trot_out_t t_out = { { 0.0f }, { 0.0f } };
        /* t 只喂 [0, Ts]（原实现的可达域）⇒ gait_trot.c 那处刻意的越界回绕不会被触发 */
        gait_trot_cal_t(&tcfg, w.t, xs, xf, h_trot,
                        (float)w.L * lr.lr1, (float)w.L * lr.lr4,
                        (float)w.R * lr.lr2, (float)w.R * lr.lr3,
                        &t_out);
        for (int i = 0; i < CONTROL_CHAIN_LEGS; ++i) {
            p_gait[i] = t_out.x[i];
            p_gait[CONTROL_CHAIN_LEGS + i] = t_out.y[i];
        }
        control_chain_apply_trot_swing_y(cfg, p_gait);
    } else if (w.gait_mode == 1) {
        /* ---- WALK ---- */
        const float wf = control_chain_walk_faai(cfg);
        /* 原实现这里把 PA_WALK.faai / PA_WALK.Ts 写成这两个值 —— C 版就是 wcfg */
        const float walk_cycle = 4.0f * wf * cfg->Ts;
        if (w.t >= walk_cycle) {
            w.t = 0.0f;
        } else if (w.L == 0 && w.R == 0) {
            w.t = 0.0f;
        } else if (w.spd != 0.0f) {
            w.t = w.t + control_chain_walk_phase_step(cfg);
        }
        const float xgs = control_chain_partial_geom_scale(cfg, cfg->large_stride_geom_frac);
        const float wxf = w.spd * 10.0f * cfg->large_stride_xf_mul * xgs;
        float h_walk = cfg->h * cfg->large_h_trot_mul;
        if ((w.L + w.R) != 0 && w.spd != 0.0f) {
            const float s = fabsf(w.spd);
            h_walk *= fmaxf(0.62f, fminf(0.92f, s / 5.5f));
            if (!control_chain_joy_forward_motion(cfg, w.spd)) {
                h_walk *= 0.82f;
            }
        }
        const control_chain_turn_lr_t lr =
            control_chain_trot_turn_lr(cfg, w.spd, w.joy_turn);
        const float wr1 = (float)w.L * lr.lr1;
        const float wr2 = (float)w.R * lr.lr2;
        const float wr3 = (float)w.R * lr.lr3;
        const float wr4 = (float)w.L * lr.lr4;

        gait_walk_cfg_t wcfg;
        wcfg.ts = cfg->Ts;
        wcfg.faai = wf;
        gait_walk_out_t w_out = { { 0.0f }, { 0.0f } };
        gait_walk_gesture_t w_gesture = { 0, 0, 0 };
        gait_walk_cal_w(&wcfg, cfg->CG_X, cfg->CG_Y, cfg->l, wxf, h_walk, w.t,
                        wr1, wr4, wr2, wr3,
                        w.R_H /* 原 _body_h() 读 padog.R_H；gyro=0 时这项不参与 */,
                        0.0f /* 本机没有 IMU，陀螺俯仰角恒 0（见文件头第 3 点） */,
                        &w_out, &w_gesture);
        /* ⚠️ `w_gesture` **必须就地施加**，而且要在下面的 slew 环之前。
         *
         * 原实现里 `_apply_cg()` 执行 `padog.gesture(0, int(CG_X), int(yst))`，
         * 那会**直接改 `PIT_goal` / `ROL_goal` / `X_goal` 三个模块级全局**；
         * 而 `cal_w()` 在 mainloop 里的位置**就在姿态 slew 环（954~972 行）之前**
         * ⇒ 原版是**同一帧**就用了被改过的目标。
         *
         * 第一版这里写的是"丢弃"，理由是"参考命名空间里 padog 是 no-op stub" ——
         * 那是把**参考值的缺陷**当成了原版的行为。修正参考环境
         * （`load_padog_ns()` 现在把真版 padog 注册进 `sys.modules`）之后，
         * 90 行里有 24 行参考值变了，本函数也随之失败 125 处。
         * 教训与 P-23 同源：**参考环境必须是真的**，否则会"精确匹配一个
         * 原版并不产生的行为"。 */
        w.PIT_goal = (float)w_gesture.pit;
        w.ROL_goal = (float)w_gesture.rol;
        w.X_goal   = (float)w_gesture.x;
        for (int i = 0; i < CONTROL_CHAIN_LEGS; ++i) {
            p_gait[i] = w_out.x[i];
            p_gait[CONTROL_CHAIN_LEGS + i] = w_out.y[i];
        }
    }

    /* ---- 第 954~957 行：站高 slew ---- */
    if (w.R_H > w.H_goal) {
        w.R_H = w.R_H - fabsf(w.R_H - w.H_goal) * cfg->Kp_H;
    } else if (w.R_H < w.H_goal) {
        w.R_H = w.R_H + fabsf(w.R_H - w.H_goal) * cfg->Kp_H;
    }

    /* ---- 第 959~972 行：姿态 / 重心 slew（俯仰、滚转、X 位置） ---- */
    if (w.PIT_S > w.PIT_goal) {
        w.PIT_S = w.PIT_S - fabsf(w.PIT_S - w.PIT_goal) * cfg->Kp_G;
    } else if (w.PIT_S < w.PIT_goal) {
        w.PIT_S = w.PIT_S + fabsf(w.PIT_S - w.PIT_goal) * cfg->Kp_G;
    }
    if (w.ROL_S > w.ROL_goal) {
        w.ROL_S = w.ROL_S - fabsf(w.ROL_S - w.ROL_goal) * cfg->Kp_G;
    } else if (w.ROL_S < w.ROL_goal) {
        w.ROL_S = w.ROL_S + fabsf(w.ROL_S - w.ROL_goal) * cfg->Kp_G;
    }
    if (w.X_S > w.X_goal) {
        w.X_S = w.X_S - fabsf(w.X_S - w.X_goal) * cfg->Kp_G;
    } else if (w.X_S < w.X_goal) {
        w.X_S = w.X_S + fabsf(w.X_S - w.X_goal) * cfg->Kp_G;
    }

    /* ---- 第 974~977 行：姿态角度限位（注意是 >= / <=，不是 >） ---- */
    if (w.PIT_S >= cfg->pit_max_ang) {
        w.PIT_S = cfg->pit_max_ang;
    }
    if (w.PIT_S <= -cfg->pit_max_ang) {
        w.PIT_S = -cfg->pit_max_ang;
    }
    if (w.ROL_S >= cfg->rol_max_ang) {
        w.ROL_S = cfg->rol_max_ang;
    }
    if (w.ROL_S <= -cfg->rol_max_ang) {
        w.ROL_S = -cfg->rol_max_ang;
    }

    /* ---- 第 979~981 行 ---- */
    const float hc = control_chain_ik_hc(cfg, w.R_H);
    const float cgk = control_chain_partial_geom_scale(cfg, cfg->large_cg_geom_frac);
    const float tr = control_chain_trot_rol_s(cfg, w.ROL_S, w.joy_turn, w.spd);

    /* ---- 第 982~1003 行：按步态与摇杆方向选重心分支 ----
     * 注意 slew 与限位**已经跑完**，`_tr` / `_wr` 用的是更新后的 ROL_S。 */
    body_pose_result_t p_ges = { { 0.0f }, { 0.0f } };
    const float spd_abs = fabsf(w.spd);
    if (w.gait_mode == 0) {
        if ((w.L + w.R) != 0 && control_chain_joy_forward_motion(cfg, w.spd)) {
            body_pose_cal_ges(w.PIT_S, tr, cfg->l, cfg->b, cfg->w,
                              w.X_S - spd_abs * cfg->trot_cg_f * cgk, hc, &p_ges);
        } else if (control_chain_joy_backward_motion(cfg, w.spd) && spd_abs > 0.0f) {
            body_pose_cal_ges(w.PIT_S, tr, cfg->l, cfg->b, cfg->w,
                              w.X_S + spd_abs * cfg->trot_cg_b * cgk * cfg->large_bwd_cg_mul,
                              hc, &p_ges);
        } else if ((w.L + w.R) == 0 && spd_abs > 0.0f) {
            body_pose_cal_ges(w.PIT_S, tr, cfg->l, cfg->b, cfg->w,
                              w.X_S + spd_abs * cfg->trot_cg_t * cgk, hc, &p_ges);
        } else if ((w.L + w.R) != 0) {
            body_pose_cal_ges(w.PIT_S, tr, cfg->l, cfg->b, cfg->w,
                              w.X_S + spd_abs * cfg->trot_cg_b * cgk * cfg->large_bwd_cg_mul,
                              hc, &p_ges);
        } else {
            body_pose_cal_ges(w.PIT_S, tr, cfg->l, cfg->b, cfg->w, w.X_S, hc, &p_ges);
        }
    } else if (w.gait_mode == 1) {
        const float wr = control_chain_walk_rol_s(cfg, w.ROL_S, w.joy_turn, w.spd);
        if ((w.L + w.R) != 0 && spd_abs > 0.0f && control_chain_joy_forward_motion(cfg, w.spd)) {
            body_pose_cal_ges(w.PIT_S, wr, cfg->l, cfg->b, cfg->w,
                              w.X_S - spd_abs * cfg->trot_cg_f * cgk * 0.65f, hc, &p_ges);
        } else if (control_chain_joy_backward_motion(cfg, w.spd) && spd_abs > 0.0f) {
            body_pose_cal_ges(w.PIT_S, wr, cfg->l, cfg->b, cfg->w,
                              w.X_S + spd_abs * cfg->trot_cg_b * cgk * cfg->large_bwd_cg_mul * 0.65f,
                              hc, &p_ges);
        } else {
            body_pose_cal_ges(w.PIT_S, wr, cfg->l, cfg->b, cfg->w, w.X_S, hc, &p_ges);
        }
    } else {
        body_pose_cal_ges(w.PIT_S, w.ROL_S, cfg->l, cfg->b, cfg->w, w.X_S, hc, &p_ges);
    }

    /* ---- 第 1007~1009 行：自稳调节器只做静态稳定，原实现里是 `pass` ---- */

    /* ---- 第 1011 行：足端竖直目标（只加前后腿偏置） ---- */
    float fy[CONTROL_CHAIN_LEGS];
    control_chain_foot_y_targets(cfg, &p_gait[CONTROL_CHAIN_LEGS], fy);

    /* ---- 第 1012 行：PA_IK.ik(ma_case, l1, l2, P_[i]+ges_x_i, fy_i+ges_y_i) ---- */
    float ik_x[CONTROL_CHAIN_LEGS];
    float ik_y[CONTROL_CHAIN_LEGS];
    for (int i = 0; i < CONTROL_CHAIN_LEGS; ++i) {
        ik_x[i] = p_gait[i] + p_ges.x[i];
        ik_y[i] = fy[i] + p_ges.y[i];
    }
    kin_ik_result_t A;
    kin_ik((kin_mode_t)cfg->ma_case, cfg->l1, cfg->l2, ik_x, ik_y, &A);

    /* ---- 第 1013 行：servo_output(ma_case, init_case, ham…, shank…) ---- */
    float hip[CONTROL_CHAIN_LEGS];
    control_chain_hip_leg_deltas(cfg, w.ROL_S, w.PIT_S, w.joy_turn, hip);

    servo_map_input_t sm;
    memset(&sm, 0, sizeof(sm));
    sm.ik_path = (cfg->ma_case == 0 && w.init_case == 0);
    for (int i = 0; i < CONTROL_CHAIN_LEGS; ++i) {
        sm.ham[i] = A.ham[i];
        sm.shank[i] = A.shank[i];
        sm.hip[i] = hip[i];
        sm.crawl_cs[i] = control_chain_crawl_shank_servodelta(cfg, w.crawl_phase, i + 1);
        sm.s_trim[i] = cfg->s_trim[i];
        for (int j = 0; j < CONTROL_CHAIN_JOINTS; ++j) {
            sm.init[i][j] = cfg->init[i][j];
        }
    }
    /* `_shank_ik_bias()`：per_mm 分支（0.25）由 servo_map 实现（见 servo_map.c 的长注释） */
    sm.shank_bias = servo_map_shank_bias(cfg->l1, cfg->l2, cfg->leg_len_ref);
    servo_map_legs_to_angles(&sm, out->angle_deg);

    /* ---- 把本帧**实际用到**的目标交回调用方 ----
     * 原版这四个量是 padog 的模块级全局，`gesture()` 之类改一次会一直留着。
     * app 层必须把这四个值作为下一帧的 `in` 目标喂回来（见 control_chain.h）。 */
    out->goal[0] = w.H_goal;
    out->goal[1] = w.PIT_goal;
    out->goal[2] = w.ROL_goal;
    out->goal[3] = w.X_goal;

    /* ---- 诊断中间量 ---- */
    for (int i = 0; i < 8; ++i) {
        out->trace.gait[i] = p_gait[i];
        out->trace.ges[i] = (i < CONTROL_CHAIN_LEGS) ? p_ges.x[i] : p_ges.y[i - CONTROL_CHAIN_LEGS];
    }
    for (int i = 0; i < CONTROL_CHAIN_LEGS; ++i) {
        out->trace.foot_y[i] = fy[i];
        out->trace.ham[i] = A.ham[i];
        out->trace.shank[i] = A.shank[i];
        out->trace.hip[i] = hip[i];
        out->trace.crawl_cs[i] = sm.crawl_cs[i];
    }

    /* ---- 写回状态（原实现是改模块级全局） ---- */
    st->t = w.t;
    st->R_H = w.R_H;
    st->PIT_S = w.PIT_S;
    st->ROL_S = w.ROL_S;
    st->X_S = w.X_S;
    st->gait_mode = w.gait_mode;
    st->crawl_phase = w.crawl_phase;
    st->crawl_until_ms = w.crawl_until_ms;
    st->crawl_settle_until_ms = w.crawl_settle_until_ms;
    st->crawl_saved_h = w.crawl_saved_h;
    st->init_case = w.init_case;
}
