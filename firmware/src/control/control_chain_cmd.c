/**
 * @file    control_chain_cmd.c
 * @brief   控制链命令层实现（逐行对应 padog.py 的 move/gait/height/gesture）
 */

#include "control/control_chain_cmd.h"

#include <math.h>
#include <stddef.h>

/** 复刻 Python `int()`：**向零截断**（不是 `floorf`） */
static float py_int(float v)
{
    return (float)(int32_t)v;
}

/** `padog.gait(mode)` 的内核 */
static void gait_apply(control_chain_cmd_t *c, const control_chain_cfg_t *cfg,
                       control_chain_state_t *st, int mode)
{
    /* if int(mode) != int(gait_mode): t = 0     ← 只在模式**变了**时归零 */
    if (st != NULL && mode != st->gait_mode) {
        st->t = 0.0f;
    }
    /* if mode == 0: PIT_goal/ROL_goal/X_goal = int(in_pit/in_rol/in_y) */
    if (mode == 0 && cfg != NULL) {
        c->goal[CONTROL_CHAIN_GOAL_PIT] = py_int(cfg->in_pit);
        c->goal[CONTROL_CHAIN_GOAL_ROL] = py_int(cfg->in_rol);
        c->goal[CONTROL_CHAIN_GOAL_X]   = py_int(cfg->in_y);
    }
    c->gait_mode = mode;
    if (st != NULL) {
        st->gait_mode = mode;
    }
}

void control_chain_cmd_init(control_chain_cmd_t *c, const control_chain_cfg_t *cfg)
{
    if (c == NULL) {
        return;
    }
    c->spd       = 0.0f;
    c->L         = 0;
    c->R         = 0;
    c->gait_mode = 0;
    c->joy_turn  = 0.0f;

    if (cfg != NULL) {
        /* padog.py 第 146 行：
         *   PIT_goal=int(in_pit); ROL_goal=int(in_rol); X_goal=int(in_y) */
        c->goal[CONTROL_CHAIN_GOAL_H]   = cfg->H_goal;
        c->goal[CONTROL_CHAIN_GOAL_PIT] = py_int(cfg->in_pit);
        c->goal[CONTROL_CHAIN_GOAL_ROL] = py_int(cfg->in_rol);
        c->goal[CONTROL_CHAIN_GOAL_X]   = py_int(cfg->in_y);
    } else {
        for (int i = 0; i < CONTROL_CHAIN_GOAL_COUNT; ++i) {
            c->goal[i] = 0.0f;
        }
    }
}

void control_chain_cmd_gait(control_chain_cmd_t *c, const control_chain_cfg_t *cfg,
                            control_chain_state_t *st, int mode)
{
    if (c != NULL) {
        gait_apply(c, cfg, st, mode);
    }
}

/** `move()` 与 `drive()` 共用的部分：spd/L/R + 条件性的 `servo_init(0)` */
static void set_spd_lr(control_chain_cmd_t *c, control_chain_state_t *st,
                       float spd, int L, int R)
{
    c->spd = spd;
    c->L = L;
    c->R = R;
    if ((L + R) != 0 && fabsf(spd) > 0.0f && st != NULL) {
        st->init_case = 0;   /* servo_init(0) */
    }
}

void control_chain_cmd_drive(control_chain_cmd_t *c, const control_chain_cfg_t *cfg,
                             control_chain_state_t *st, float spd, int L, int R)
{
    (void)cfg;   /* drive() 不碰目标，也不需要 cfg */
    if (c == NULL) {
        return;
    }
    /*
     * padog.drive(spd_, L_, R_)：与 move() **只差一句 `gait(0)`**。
     * 注释原文："仅更新 spd/L/R（WALK 摇杆用，不切 gait_mode）"
     * ⇒ WALK 只能走这条路进来；用 move() 会被它内部的 gait(0) 改回 TROT。
     */
    set_spd_lr(c, st, spd, L, R);
}

void control_chain_cmd_move(control_chain_cmd_t *c, const control_chain_cfg_t *cfg,
                            control_chain_state_t *st, float spd, int L, int R)
{
    if (c == NULL) {
        return;
    }

    /* padog.move(spd_, L_, R_)：
     *     spd = float(spd_); L = L_; R = R_          ← 无条件生效
     *     if (L_ + R_) != 0 and abs(spd_) > 0:
     *         gait(0)                                 ← 含"模式变了才 t=0"与目标重置
     *         servo_init(0)                           ← init_case = 0
     *         direct_pose_freeze = False
     *         inplace_step_end_ms = 0
     *
     * ⚠️ 关键：`L=R=0` 或 `spd=0` 时**只**改 spd/L/R，**不动**目标与相位。
     *    所以 `move(0,0,0)` 之后目标还是之前的值（可能被 WALK 的 gesture 改过），
     *    站位由 mainloop 自己把 `t` 归零（`L==0 and R==0` 那一支）。
     *    第一版 `app_chain_jog` 在这里无条件 `t=0`，会让行进中推摇杆时相位跳变。
     *
     * ⚠️ 也正因为这里有 `gait(0)`，**用 move() 无法进入 WALK** —— 见 drive()。 */
    set_spd_lr(c, st, spd, L, R);

    if ((L + R) != 0 && fabsf(spd) > 0.0f) {
        gait_apply(c, cfg, st, 0);
    }
}

void control_chain_cmd_height(control_chain_cmd_t *c, control_chain_state_t *st, float h_goal)
{
    if (c == NULL) {
        return;
    }
    c->goal[CONTROL_CHAIN_GOAL_H] = h_goal;
    /* ⚠️ 原版 `height()` 还会 `R_H = goal`（直接同步，绕开 Kp_H 的 slew）。
     * 漏了这一行，多帧序列对照会报 876/9840 处不符。 */
    if (st != NULL) {
        st->R_H = h_goal;
    }
}

void control_chain_cmd_gesture(control_chain_cmd_t *c, float pit, float rol, float x)
{
    if (c == NULL) {
        return;
    }
    c->goal[CONTROL_CHAIN_GOAL_PIT] = pit;
    c->goal[CONTROL_CHAIN_GOAL_ROL] = rol;
    c->goal[CONTROL_CHAIN_GOAL_X]   = x;
}

void control_chain_cmd_stand(control_chain_cmd_t *c, const control_chain_cfg_t *cfg,
                             control_chain_state_t *st)
{
    if (c == NULL) {
        return;
    }
    /*
     * "站立"在原版里不是一个函数，而是两个既有调用的组合：
     *     move(0, 0, 0)    —— 速度归零（注意它**不动**目标与相位）
     *     gait(0)          —— 三个重心目标复位成 int(in_pit/in_rol/in_y)
     * 站位本身由 mainloop 完成（`L==0 and R==0` 那一支把 t 归零，
     * 重心分支走 else：「无偏置」，姿态 slew 把 PIT_S/ROL_S/X_S 拉向目标）。
     */
    c->spd = 0.0f;
    c->L = 0;
    c->R = 0;
    gait_apply(c, cfg, st, 0);
}

void control_chain_cmd_set_turn(control_chain_cmd_t *c, float pct)
{
    if (c != NULL) {
        c->joy_turn = pct;
    }
}

void control_chain_cmd_make_input(const control_chain_cmd_t *c, int32_t now_ms,
                                  control_chain_input_t *in)
{
    if (c == NULL || in == NULL) {
        return;
    }
    /* 逐字段赋值：`in` 里还有 crawl_* 之类的字段不该被顺手清掉 */
    in->spd       = c->spd;
    in->L         = c->L;
    in->R         = c->R;
    in->gait_mode = c->gait_mode;
    in->joy_turn  = c->joy_turn;
    in->H_goal    = c->goal[CONTROL_CHAIN_GOAL_H];
    in->PIT_goal  = c->goal[CONTROL_CHAIN_GOAL_PIT];
    in->ROL_goal  = c->goal[CONTROL_CHAIN_GOAL_ROL];
    in->X_goal    = c->goal[CONTROL_CHAIN_GOAL_X];
    in->now_ms    = now_ms;
}

void control_chain_cmd_absorb(control_chain_cmd_t *c, const control_chain_out_t *out)
{
    if (c == NULL || out == NULL) {
        return;
    }
    for (int i = 0; i < CONTROL_CHAIN_GOAL_COUNT; ++i) {
        c->goal[i] = out->goal[i];
    }
}

/* ==================================================================== */
/*  节拍                                                                 */
/* ==================================================================== */

uint32_t control_chain_sched_period_from_cfg(const control_chain_cfg_t *cfg)
{
    if (cfg == NULL || cfg->speed <= 0.0f) {
        return 65u;   /* 兜底：config_s.py 的 speed = 0.065 -> 65 ms */
    }
    const float ms = cfg->speed * 1000.0f;
    if (ms < 1.0f) {
        return 1u;
    }
    if (ms > 1000.0f) {
        return 1000u;
    }
    return (uint32_t)(ms + 0.5f);
}

void control_chain_sched_init(control_chain_sched_t *s, uint32_t period_ms)
{
    if (s == NULL) {
        return;
    }
    s->period_ms   = (period_ms == 0) ? 65u : period_ms;
    s->next_due_ms = 0;
    s->started     = false;
}

void control_chain_sched_set_period(control_chain_sched_t *s, uint32_t period_ms)
{
    if (s == NULL || period_ms == 0) {
        return;
    }
    s->period_ms = period_ms;
    s->started   = false;   /* 下一次调用立刻推进一帧 */
}

bool control_chain_sched_due(control_chain_sched_t *s, int64_t now_ms)
{
    if (s == NULL) {
        return false;
    }
    if (!s->started) {
        s->started     = true;
        s->next_due_ms = now_ms + (int64_t)s->period_ms;
        return true;
    }
    if (now_ms < s->next_due_ms) {
        return false;
    }

    s->next_due_ms += (int64_t)s->period_ms;
    /* 落后太多就不追补：否则会连续补几帧、把相位一次推很远（等于步态瞬跳） */
    if (now_ms > s->next_due_ms + (int64_t)s->period_ms) {
        s->next_due_ms = now_ms + (int64_t)s->period_ms;
    }
    return true;
}
