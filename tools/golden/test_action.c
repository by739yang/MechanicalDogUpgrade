/*
 * test_action.c —— 宿主侧 golden 测试：action.c（P4 姿态动画 / 动作层）
 *
 * 参考值全部来自 `tools/golden/gen_golden.py` 里**真版 padog.py 真跑一次**
 * （与 control_chain / servo_map 同一套取法），时钟由
 * `mpy_stubs.install_controllable_clock()` 钉死。四个 CSV：
 *
 *   action_pose.csv        纯函数：两张姿态表 / 插值 / 两种直写。
 *                          比 **12 路角度**（0 容差）+ 占空比
 *   action_cmd.csv         动作入口 + `mainloop` 的姿态动画段与 inplace 服务。
 *                          比 **12 路占空比 + 22 个模块级全局的事后值**
 *   action_wave.csv        `action_wave_direct()` 整条阻塞脚本的写寄存器日志（含时刻）
 *   action_wave_final.csv  上一条跑完时的状态
 *
 * ## 为什么动作那一套要连"别的模块的全局量"一起比（成长手册 P-25）
 *
 * 这一层的原函数**大量改别的模块的状态**：`move` / `gait` / `height` / `gesture` /
 * `servo_init`、`set_leg_sit_offsets`（改 chain 的**配置**）、清爬行（改 chain 的 state）。
 * 只比 12 路舵机的话，"副作用丢了"这件事在 golden 里**完全看不出来** ——
 * 那正是 P-25 的形状（C 版精确匹配了一个原版并不产生的行为）。
 * 所以 CSV 连 22 个全局量的事后值一起记；C 侧用**真实的 `control_chain_cmd_*` 命令层**
 * 按顺序施加 `action.c` 输出的效果表，再逐项对照。
 *
 * ## 容差：0
 *
 * 角度那一套之所以能做到 0，是因为采样**刻意让插值在 float 与 double 里逐位相同**
 * （t 只取二进制精确值、sit 偏移全是 4 的倍数）；生成器里还有一条
 * `_require_float_exact()` 专门挡住"事后状态里出现 float32 表示不了的值"，
 * 免得以后有人为了让它过而偷偷加容差。
 * 占空比那一套是 **0 个计数单位**，与 test_servo_map / test_control_chain 一致。
 *
 * ## 这一套**测不到**什么（说清楚，不假装）
 *
 * * `action_wave_direct()` 的 `time.sleep_ms()` **真实时长**。假时钟只钉住
 *   "原代码请求睡多久"，钉不住"实际睡了多久"（那是调度器的事）。
 * * `mainloop()` 里 `direct_pose_freeze` 那个提前 return（以及它后面的整条运动链）——
 *   那属于调度 + `control_chain.c`。本测试对 action=3 的行**只**覆盖
 *   "inplace 服务 + `_pose_anim_step`"这两段（CSV 里那些行 pose_anim_active 恒为 1）。
 * * `action_crawl()` 没搬（理由见 action.c 文件头）—— 所以爬行的**入口**不在本套范围内，
 *   爬行状态机本身由 test_control_chain 覆盖。
 *
 * NOTE: ASCII output only, for the reason recorded as P-06 in 问题与解决记录.md.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "control/action.h"
#include "control/control_chain.h"
#include "control/control_chain_cmd.h"
#include "control/servo_map.h"

/** 允许的角度偏差（度）。0 = 精确相等 */
#define ACTION_ANGLE_TOL 0.0
/** 允许的占空比偏差（计数单位）。0 = 精确相等 */
#define ACTION_DUTY_TOL  0
/** 允许的"模块级全局量"偏差。0 = 精确相等 */
#define ACTION_STATE_TOL 0.0

#define LINE_MAX   8192
#define FIELDS_MAX 128

/* ==================================================================== */
/*  公共记账                                                             */
/* ==================================================================== */

static long   g_values;             /* 比过的项数 */
static long   g_bad;                /* 不符的项数 */
static long   g_shown;              /* 已经打过几处 FIRST MISMATCH */
static double g_max_angle_err;
static long   g_max_angle_row = -1;
static long   g_max_duty_err;
static long   g_max_duty_row = -1;

static void reset_stats(void)
{
    g_values = 0;
    g_bad = 0;
    g_shown = 0;
    g_max_angle_err = 0.0;
    g_max_angle_row = -1;
    g_max_duty_err = 0;
    g_max_duty_row = -1;
}

static void note_angle(long row, int ch, double got, double exp)
{
    ++g_values;
    const double d = fabs(got - exp);
    if (d > g_max_angle_err) {
        g_max_angle_err = d;
        g_max_angle_row = row;
    }
    if (d > ACTION_ANGLE_TOL) {
        ++g_bad;
        if (g_shown < 3) {
            ++g_shown;
            printf("FIRST MISMATCH: row=%ld ch=%d (%s) angle expected=%.9f got=%.9f "
                   "err=%.9g\n",
                   row, ch, servo_map_channel_name((uint8_t)ch), exp, got, d);
        }
    }
}

static void note_duty(long row, int ch, long got, long exp)
{
    ++g_values;
    const long d = labs(got - exp);
    if (d > g_max_duty_err) {
        g_max_duty_err = d;
        g_max_duty_row = row;
    }
    if (d > ACTION_DUTY_TOL) {
        ++g_bad;
        if (g_shown < 3) {
            ++g_shown;
            printf("FIRST MISMATCH: row=%ld ch=%d (%s) duty expected=%ld got=%ld\n",
                   row, ch, servo_map_channel_name((uint8_t)ch), exp, got);
        }
    }
}

static void note_state(long row, const char *name, double got, double exp)
{
    ++g_values;
    const double d = fabs(got - exp);
    if (d > ACTION_STATE_TOL) {
        ++g_bad;
        if (g_shown < 6) {
            ++g_shown;
            printf("FIRST MISMATCH: row=%ld state %s expected=%.9f got=%.9f\n",
                   row, name, exp, got);
        }
    }
}

static int split_csv(char *line, double *out, int max)
{
    int n = 0;
    char *tok = strtok(line, ",\r\n");
    while (tok != NULL && n < max) {
        out[n++] = atof(tok);
        tok = strtok(NULL, ",\r\n");
    }
    return n;
}

/* ==================================================================== */
/*  [A] action_pose.csv —— 纯函数，比角度                                 */
/* ==================================================================== */

/** CSV 列：fn,param,init(12),deg(12),duty(12) */
#define POSE_FIELDS (2 + 12 + 12 + 12)
#define POSE_FN     0
#define POSE_PARAM  1
#define POSE_INIT   2
#define POSE_DEG    14
#define POSE_DUTY   26

static int run_pose(const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        return 2;
    }

    reset_stats();
    double v[FIELDS_MAX];
    char line[LINE_MAX];
    long rows = 0;
    long fn_rows[3] = { 0, 0, 0 };

    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        const int n = split_csv(line, v, FIELDS_MAX);
        if (n != POSE_FIELDS) {
            fprintf(stderr, "ERROR: %s: expected %d fields, got %d at row %ld\n",
                    path, POSE_FIELDS, n, rows + 1);
            fclose(f);
            return 2;
        }

        action_cfg_t cfg;
        action_cfg_defaults(&cfg);
        /* 中位角：CSV 列序 = 腿1..腿4 x (髋,大腿,小腿)，与 cfg.init 同序 */
        for (int i = 0; i < ACTION_LEGS * ACTION_JOINTS; ++i) {
            cfg.init[i / ACTION_JOINTS][i % ACTION_JOINTS] =
                (float)v[POSE_INIT + i];
        }

        const int fn = (int)v[POSE_FN];
        const float param = (float)v[POSE_PARAM];
        float deg[ACTION_CHANNELS];
        for (int i = 0; i < ACTION_CHANNELS; ++i) {
            deg[i] = 0.0f;
        }

        if (fn == 0) {
            float from[ACTION_CHANNELS];
            float to[ACTION_CHANNELS];
            action_stand_pose(&cfg, from);
            action_sit_pose(&cfg, to);
            action_apply_pose_blend(from, to, param, deg);
        } else if (fn == 1) {
            action_apply_stand_angles_direct(&cfg, deg);
        } else if (fn == 2) {
            action_apply_sit_angles_direct(&cfg, deg);
        } else {
            fprintf(stderr, "ERROR: %s: unknown fn=%d at row %ld\n", path, fn, rows);
            fclose(f);
            return 2;
        }

        /* 逐路比角度（0 容差）。占空比一起比一遍：它是"到硬件那一层"的同一条通路 */
        for (int ch = 0; ch < ACTION_CHANNELS; ++ch) {
            note_angle(rows, ch, (double)deg[ch], v[POSE_DEG + ch]);
            note_duty(rows, ch, (long)servo_map_deg_to_duty(deg[ch]),
                      (long)v[POSE_DUTY + ch]);
        }
        if (fn >= 0 && fn < 3) {
            ++fn_rows[fn];
        }
        ++rows;
    }
    fclose(f);

    printf("========================================================\n");
    printf(" action pose      (padog pose tables / blend / direct writes)\n");
    printf("========================================================\n");
    printf("golden file : %s\n", path);
    printf("rows        : %ld  (blend: %ld, stand_direct: %ld, sit_direct: %ld)\n",
           rows, fn_rows[0], fn_rows[1], fn_rows[2]);
    printf("values      : %ld  (rows x 12 channels: angle + duty)\n", g_values);
    printf("max angle   : %.9g deg  (allowed %.9g)", g_max_angle_err, ACTION_ANGLE_TOL);
    if (g_max_angle_row >= 0) {
        printf("  at row %ld", g_max_angle_row);
    }
    printf("\n");
    printf("max duty    : %ld counts  (allowed %d)", g_max_duty_err, ACTION_DUTY_TOL);
    if (g_max_duty_row >= 0) {
        printf("  at row %ld", g_max_duty_row);
    }
    printf("\n");
    printf("mismatches  : %ld\n", g_bad);
    return (g_bad == 0) ? 0 : 1;
}

/* ==================================================================== */
/*  [B] action_cmd.csv —— 动作入口 + mainloop 姿态动画段                  */
/* ==================================================================== */

/* 前置列（33） */
#define C_ACTION      0
#define C_NOW         1
#define C_PAIR        2
#define C_ANIM_START  3
#define C_ANIM_END    4
#define C_HOLD        5
#define C_H_GOAL      6
#define C_GAIT        7
#define C_T           8
#define C_CRAWL       9
#define C_UNTIL       10
#define C_SETTLE      11
#define C_INPLACE     12
#define C_FREEZE      13
#define C_ACTIVE      14
#define C_FRONT       15
#define C_REAR        16
#define C_IN_Y        17
#define C_IN_PIT      18
#define C_IN_ROL      19
#define C_INIT_CASE   20
#define C_INIT        21
#define C_INIT_N      12
/* 事后列（35） */
#define C_NWRITE      (C_INIT + C_INIT_N)          /* 33 */
#define C_DUTY        (C_NWRITE + 1)               /* 34 */
#define C_POST        (C_DUTY + 12)                /* 46 */
#define C_POST_N      22
#define CMD_FIELDS    (C_POST + C_POST_N)          /* 68 */

/** 事后 22 个量的名字 —— 顺序必须与 gen_golden.py 的 ACTION_CMD_POST 一致 */
static const char *const POST_NAMES[C_POST_N] = {
    "direct_pose_freeze", "pose_anim_active", "pose_anim_hold_freeze",
    "pose_anim_start_ms", "pose_anim_end_ms",
    "crawl_phase", "crawl_until_ms", "crawl_settle_until_ms", "inplace_step_end_ms",
    "spd", "L", "R", "gait_mode", "t",
    "H_goal", "R_H", "PIT_goal", "ROL_goal", "X_goal",
    "front_leg_y_offset", "rear_leg_y_offset", "init_case",
};

/**
 * @brief 被测侧的一整套状态：动作层 + 链的命令层 + 链的 state + 链的（可写）配置。
 *
 * 为什么要把 chain 的那三样都摆在这里：`action.c` 的副作用**就是改它们**。
 * 这里用**真实的** `control_chain_cmd_*` 去施加，而不是在测试里另写一套
 * "我以为的 move/gait" —— 那会变成 P-22/P-23 那一类"测试自己也是个实现"。
 */
typedef struct {
    action_cfg_t acfg;
    action_state_t ast;
    control_chain_cfg_t ccfg;   /* ⚠️ 必须可写：SIT_OFFSETS 改的是 chain 的**配置** */
    control_chain_cmd_t cmd;
    control_chain_state_t st;
} harness_t;

/** 按 CSV 的前置列摆好被测侧的全部状态 */
static void harness_prepare(harness_t *h, const double *v)
{
    control_chain_cfg_defaults(&h->ccfg);
    action_cfg_defaults(&h->acfg);

    for (int i = 0; i < C_INIT_N; ++i) {
        h->acfg.init[i / ACTION_JOINTS][i % ACTION_JOINTS] = (float)v[C_INIT + i];
        h->ccfg.init[i / ACTION_JOINTS][i % ACTION_JOINTS] = (float)v[C_INIT + i];
    }
    h->ccfg.in_pit = (float)v[C_IN_PIT];
    h->ccfg.in_rol = (float)v[C_IN_ROL];
    h->ccfg.in_y   = (float)v[C_IN_Y];
    /* ⚠️ 本层自己的 cfg 也要有这三个值：`action_stand()` / `action_sit_direct()`
     * 里的 `gesture(0, 0, in_y)` 读的是**本层的** in_y。少同步一个就会出现
     * "动作层与命令层用了两个不同的 in_y" —— 第一版就是这么错的（立刻被 golden 抓住）。 */
    h->acfg.in_pit = (float)v[C_IN_PIT];
    h->acfg.in_rol = (float)v[C_IN_ROL];
    h->acfg.in_y   = (float)v[C_IN_Y];
    h->ccfg.front_leg_y_offset = (float)v[C_FRONT];
    h->ccfg.rear_leg_y_offset  = (float)v[C_REAR];

    /* 命令层与 state 的初值（`PIT_goal=int(in_pit)` 那几行由 cmd_init 做） */
    control_chain_cmd_init(&h->cmd, &h->ccfg);
    control_chain_state_init(&h->st, &h->ccfg);

    h->cmd.goal[CONTROL_CHAIN_GOAL_H] = (float)v[C_H_GOAL];
    h->cmd.gait_mode = (int)v[C_GAIT];
    h->st.gait_mode  = (int)v[C_GAIT];
    h->st.t          = (float)v[C_T];
    h->st.crawl_phase          = (int)v[C_CRAWL];
    h->st.crawl_until_ms       = (int32_t)v[C_UNTIL];
    h->st.crawl_settle_until_ms = (int32_t)v[C_SETTLE];
    h->st.init_case  = (int)v[C_INIT_CASE];

    action_state_init(&h->ast);
    h->ast.direct_pose_freeze   = (v[C_FREEZE] != 0.0);
    h->ast.pose_anim_active     = (v[C_ACTIVE] != 0.0);
    h->ast.pose_anim_hold_freeze = (v[C_HOLD] != 0.0);
    h->ast.pose_anim_start_ms   = (int32_t)v[C_ANIM_START];
    h->ast.pose_anim_end_ms     = (int32_t)v[C_ANIM_END];
    h->ast.inplace_step_end_ms  = (int32_t)v[C_INPLACE];
    if (v[C_PAIR] == 0.0) {
        action_stand_pose(&h->acfg, h->ast.pose_anim_from);
        action_sit_pose(&h->acfg, h->ast.pose_anim_to);
    } else {
        action_sit_pose(&h->acfg, h->ast.pose_anim_from);
        action_stand_pose(&h->acfg, h->ast.pose_anim_to);
    }
}

/**
 * @brief 按**顺序**施加 `action.c` 输出的效果表。
 *
 * 顺序有意义：`action_stand()` 在动画支里先 `gesture(0,0,in_y)` 再 `gait(0)`，
 * 两者都写 PIT/ROL/X 目标，最终值不同（`in_pit`/`in_rol` 非 0 时立刻能看出来）。
 * 所以效果表是**有序表**，这里也必须按序施加。
 */
static void apply_effects(harness_t *h, const action_effects_t *eff)
{
    if (eff->overflow) {
        printf("ERROR: effects table overflowed (%d items) -- a side effect was DROPPED\n",
               eff->n);
        ++g_bad;
    }
    for (int i = 0; i < eff->n; ++i) {
        const action_eff_t *e = &eff->item[i];
        switch (e->kind) {
        case ACTION_EFF_MOVE:
            control_chain_cmd_move(&h->cmd, &h->ccfg, &h->st, e->a0, e->i0, e->i1);
            break;
        case ACTION_EFF_GAIT:
            control_chain_cmd_gait(&h->cmd, &h->ccfg, &h->st, e->i0);
            break;
        case ACTION_EFF_HEIGHT:
            control_chain_cmd_height(&h->cmd, &h->st, e->a0);
            break;
        case ACTION_EFF_GESTURE:
            control_chain_cmd_gesture(&h->cmd, e->a0, e->a1, e->a2);
            break;
        case ACTION_EFF_SIT_OFFSETS:
            /* ⚠️ 原 `set_leg_sit_offsets()` 改的是 chain 的**配置**，不是 state */
            h->ccfg.front_leg_y_offset = e->a0;
            h->ccfg.rear_leg_y_offset  = e->a1;
            break;
        case ACTION_EFF_SERVO_INIT:
            h->st.init_case = e->i0;
            break;
        case ACTION_EFF_CRAWL_RESET:
            h->st.crawl_phase = 0;
            h->st.crawl_until_ms = 0;
            h->st.crawl_settle_until_ms = 0;
            break;
        default:
            printf("ERROR: unknown effect kind %d\n", (int)e->kind);
            ++g_bad;
            break;
        }
    }
}

/** 事后状态 —— 顺序必须与 POST_NAMES / gen_golden.py 的 ACTION_CMD_POST 一致 */
static void harness_dump(const harness_t *h, double out[C_POST_N])
{
    out[0]  = h->ast.direct_pose_freeze ? 1.0 : 0.0;
    out[1]  = h->ast.pose_anim_active ? 1.0 : 0.0;
    out[2]  = h->ast.pose_anim_hold_freeze ? 1.0 : 0.0;
    out[3]  = (double)h->ast.pose_anim_start_ms;
    out[4]  = (double)h->ast.pose_anim_end_ms;
    out[5]  = (double)h->st.crawl_phase;
    out[6]  = (double)h->st.crawl_until_ms;
    out[7]  = (double)h->st.crawl_settle_until_ms;
    out[8]  = (double)h->ast.inplace_step_end_ms;
    out[9]  = (double)h->cmd.spd;
    out[10] = (double)h->cmd.L;
    out[11] = (double)h->cmd.R;
    out[12] = (double)h->cmd.gait_mode;
    out[13] = (double)h->st.t;
    out[14] = (double)h->cmd.goal[CONTROL_CHAIN_GOAL_H];
    out[15] = (double)h->st.R_H;
    out[16] = (double)h->cmd.goal[CONTROL_CHAIN_GOAL_PIT];
    out[17] = (double)h->cmd.goal[CONTROL_CHAIN_GOAL_ROL];
    out[18] = (double)h->cmd.goal[CONTROL_CHAIN_GOAL_X];
    out[19] = (double)h->ccfg.front_leg_y_offset;
    out[20] = (double)h->ccfg.rear_leg_y_offset;
    out[21] = (double)h->st.init_case;
}

static const char *const ACTION_NAMES[4] = {
    "action_stand", "action_sit", "action_sit_direct", "mainloop(pose+inplace)",
};

static int run_cmd(const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        return 2;
    }

    reset_stats();
    double v[FIELDS_MAX];
    char line[LINE_MAX];
    long rows = 0;
    long per_action[4] = { 0, 0, 0, 0 };
    long total_touched = 0;

    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        const int n = split_csv(line, v, FIELDS_MAX);
        if (n != CMD_FIELDS) {
            fprintf(stderr, "ERROR: %s: expected %d fields, got %d at row %ld\n",
                    path, CMD_FIELDS, n, rows + 1);
            fclose(f);
            return 2;
        }

        const int action = (int)v[C_ACTION];
        if (action < 0 || action > 3) {
            fprintf(stderr, "ERROR: %s: unknown action=%d at row %ld\n",
                    path, action, rows);
            fclose(f);
            return 2;
        }

        harness_t h;
        harness_prepare(&h, v);

        float deg[ACTION_CHANNELS];
        for (int i = 0; i < ACTION_CHANNELS; ++i) {
            deg[i] = 0.0f;
        }
        action_effects_t eff;
        action_effects_clear(&eff);
        const int32_t now = (int32_t)v[C_NOW];
        bool wrote = false;

        switch (action) {
        case 0:
            wrote = action_stand(&h.acfg, &h.ast, (float)v[C_H_GOAL], now, deg, &eff);
            break;
        case 1:
            wrote = action_sit(&h.acfg, &h.ast, now, deg, &eff);
            break;
        case 2:
            wrote = action_sit_direct(&h.acfg, &h.ast, now, deg, &eff);
            break;
        default:
            /* 参考实现是 mainloop()：第 881~888 行的 inplace 服务，
             * 然后第 889 行 `if _pose_anim_step(): ... return 0`。
             * 本套只覆盖这两段（CSV 里这些行 pose_anim_active 恒为 1），
             * 因为再往下就是整条运动链，那是 control_chain 的范围。 */
            (void)action_inplace_step(&h.ast, &h.acfg, now, &eff);
            apply_effects(&h, &eff);
            action_effects_clear(&eff);
            wrote = action_pose_anim_step(&h.ast, &h.acfg, now, deg);
            break;
        }
        apply_effects(&h, &eff);

        const long exp_n = (long)v[C_NWRITE];
        const long got_n = wrote ? ACTION_CHANNELS : 0;
        ++g_values;
        if (exp_n != got_n) {
            ++g_bad;
            if (g_shown < 6) {
                ++g_shown;
                printf("FIRST MISMATCH: row=%ld %s: wrote %ld channels, expected %ld\n",
                       rows, ACTION_NAMES[action], got_n, exp_n);
            }
        } else if (wrote) {
            for (int ch = 0; ch < ACTION_CHANNELS; ++ch) {
                note_duty(rows, ch, (long)servo_map_deg_to_duty(deg[ch]),
                          (long)v[C_DUTY + ch]);
            }
        }
        total_touched += got_n;

        double post[C_POST_N];
        harness_dump(&h, post);
        for (int i = 0; i < C_POST_N; ++i) {
            note_state(rows, POST_NAMES[i], post[i], v[C_POST + i]);
        }

        ++per_action[action];
        ++rows;
    }
    fclose(f);

    printf("========================================================\n");
    printf(" action cmd       (stand/sit + mainloop pose-anim & inplace)\n");
    printf("========================================================\n");
    printf("golden file : %s\n", path);
    printf("rows        : %ld  (stand: %ld, sit: %ld, sit_direct: %ld, mainloop: %ld)\n",
           rows, per_action[0], per_action[1], per_action[2], per_action[3]);
    printf("servo writes: %ld  (rows x 12, only for the branches that write)\n",
           total_touched);
    printf("checks      : %ld  (servo writes + 22 module globals x %ld rows)\n",
           g_values, rows);
    printf("max duty    : %ld counts  (allowed %d)\n", g_max_duty_err, ACTION_DUTY_TOL);
    printf("mismatches  : %ld\n", g_bad);
    return (g_bad == 0) ? 0 : 1;
}

/* ==================================================================== */
/*  [C] action_wave.csv + action_wave_final.csv                          */
/* ==================================================================== */

#define WAVE_MAX_WRITES 4096
#define WAVE_MAX_GROUPS 512

typedef struct {
    int   n;
    long  n_writes;
    int   group_of[WAVE_MAX_WRITES];
    int   t_off[WAVE_MAX_WRITES];
    int   ch[WAVE_MAX_WRITES];
    long  duty[WAVE_MAX_WRITES];
    int   n_groups;
    int   g_begin[WAVE_MAX_GROUPS];
    int   g_end[WAVE_MAX_GROUPS];
    int   g_t_off[WAVE_MAX_GROUPS];
} wave_golden_t;

static int wave_load(wave_golden_t *g, const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        return 2;
    }
    memset(g, 0, sizeof(*g));
    double v[FIELDS_MAX];
    char line[LINE_MAX];
    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        const int n = split_csv(line, v, FIELDS_MAX);
        if (n != 4) {
            fprintf(stderr, "ERROR: %s: expected 4 fields, got %d\n", path, n);
            fclose(f);
            return 2;
        }
        if (g->n >= WAVE_MAX_WRITES) {
            fprintf(stderr, "ERROR: %s: too many writes\n", path);
            fclose(f);
            return 2;
        }
        g->group_of[g->n] = (int)v[0];
        g->t_off[g->n]    = (int)v[1];
        g->ch[g->n]       = (int)v[2];
        g->duty[g->n]     = (long)v[3];
        ++g->n;
    }
    fclose(f);

    /* 组是连续的（生成器按时间戳切）—— 只认这一点，不重新排序 */
    g->n_groups = 0;
    for (int i = 0; i < g->n; ) {
        if (g->n_groups >= WAVE_MAX_GROUPS) {
            fprintf(stderr, "ERROR: %s: too many groups\n", path);
            return 2;
        }
        const int grp = g->group_of[i];
        if (grp != g->n_groups) {
            fprintf(stderr, "ERROR: %s: group %d out of order at row %d\n",
                    path, grp, i);
            return 2;
        }
        g->g_begin[g->n_groups] = i;
        g->g_t_off[g->n_groups] = g->t_off[i];
        while (i < g->n && g->group_of[i] == grp) {
            ++i;
        }
        g->g_end[g->n_groups] = i;
        ++g->n_groups;
    }
    return 0;
}

static int run_wave(const char *path, const char *final_path)
{
    /* ⚠️ static：这个结构 ~90 KB，别放栈上 */
    static wave_golden_t g;
    int rc = wave_load(&g, path);
    if (rc != 0) {
        return rc;
    }

    reset_stats();
    long n_writes_expected = 0;

    harness_t h;
    /* 参考侧是在一个"刚 exec 完 padog.py"的干净环境上跑的：
     * 全部模块级初值 + config 的默认值 —— 对应这里的全默认 + state_init。 */
    control_chain_cfg_defaults(&h.ccfg);
    action_cfg_defaults(&h.acfg);
    control_chain_cmd_init(&h.cmd, &h.ccfg);
    control_chain_state_init(&h.st, &h.ccfg);
    action_state_init(&h.ast);

    action_wave_t w;
    action_wave_init(&w);

    /* 起始时刻随便取（0 即可）：本动作的所有时序都只在**内部相对**
     * 使用时钟（`pose_anim_start_ms = now`），所以绝对基准不影响任何输出。
     * 最后比的是相对量 t_off。 */
    int32_t now = 0;
    const int32_t t0 = now;
    int gi = 0;
    long steps = 0;
    long n_writes = 0;

    for (;;) {
        action_wave_step_t s;
        if (!action_wave_step(&h.ast, &w, &h.acfg, now,
                              h.cmd.goal[CONTROL_CHAIN_GOAL_H], &s)) {
            break;
        }
        apply_effects(&h, &s.eff);

        if (s.ch_mask != 0u) {
            /* 本步写了几个逻辑通道（= 几个寄存器写） */
            int n_got = 0;
            for (int ch = 0; ch < ACTION_CHANNELS; ++ch) {
                if ((s.ch_mask & (1u << ch)) != 0u) {
                    ++n_got;
                }
            }
            n_writes += n_got;

            if (gi >= g.n_groups) {
                ++g_bad;
                if (g_shown < 3) {
                    ++g_shown;
                    printf("FIRST MISMATCH: extra write step at t_off=%d "
                           "(golden has only %d groups)\n", (int)(now - t0), g.n_groups);
                }
            } else {
                const int beg = g.g_begin[gi];
                const int end = g.g_end[gi];
                /* 时序（sleep 时长）也要对 */
                note_state(steps, "t_off", (double)(now - t0), (double)g.g_t_off[gi]);
                /* 通道集合必须完全一致 */
                const int n_expected = end - beg;
                ++g_values;
                if (n_got != n_expected) {
                    ++g_bad;
                    if (g_shown < 3) {
                        ++g_shown;
                        printf("FIRST MISMATCH: step %ld (t_off=%d): wrote %d channels, "
                               "golden group %d has %d\n",
                               steps, (int)(now - t0), n_got, gi, n_expected);
                    }
                }
                /* 逐项比占空比 */
                for (int k = beg; k < end; ++k) {
                    const int ch = g.ch[k];
                    if ((s.ch_mask & (1u << ch)) == 0u) {
                        ++g_values;
                        ++g_bad;
                        if (g_shown < 3) {
                            ++g_shown;
                            printf("FIRST MISMATCH: step %ld: golden wrote ch%d "
                                   "but C did not\n", steps, ch);
                        }
                        continue;
                    }
                    note_duty(steps, ch, (long)servo_map_deg_to_duty(s.deg[ch]),
                              g.duty[k]);
                }
            }
            ++gi;
        }

        now += s.delay_ms;
        ++steps;
        if (steps > 4096) {
            fprintf(stderr, "ERROR: wave stepper did not terminate\n");
            return 2;
        }
    }

    ++g_values;
    if (gi != g.n_groups) {
        ++g_bad;
        printf("FIRST MISMATCH: C produced %d write steps, golden has %d groups\n",
               gi, g.n_groups);
    }

    /* ---- 结束状态 ---- */
    double exp_final[32];
    int n_final = 0;
    FILE *ff = fopen(final_path, "r");
    if (ff == NULL) {
        fprintf(stderr, "ERROR: cannot open %s\n", final_path);
        return 2;
    }
    {
        char line[LINE_MAX];
        while (fgets(line, sizeof(line), ff) != NULL) {
            if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
                continue;
            }
            n_final = split_csv(line, exp_final, 32);
            break;
        }
    }
    fclose(ff);
    if (n_final != 23) {
        fprintf(stderr, "ERROR: %s: expected 23 fields, got %d\n", final_path, n_final);
        return 2;
    }
    n_writes_expected = (long)exp_final[2];

    static const char *const FINAL_NAMES[20] = {
        "pose_anim_active", "pose_anim_hold_freeze", "direct_pose_freeze",
        "anim_start_off", "anim_end_off",
        "inplace_step_end_ms", "crawl_phase", "gait_mode", "t",
        "spd", "L", "R", "H_goal", "R_H", "PIT_goal", "ROL_goal", "X_goal",
        "front_leg_y_offset", "rear_leg_y_offset", "init_case",
    };
    double got_final[20];
    got_final[0]  = h.ast.pose_anim_active ? 1.0 : 0.0;
    got_final[1]  = h.ast.pose_anim_hold_freeze ? 1.0 : 0.0;
    got_final[2]  = h.ast.direct_pose_freeze ? 1.0 : 0.0;
    got_final[3]  = (double)(h.ast.pose_anim_start_ms - t0);
    got_final[4]  = (double)(h.ast.pose_anim_end_ms - t0);
    got_final[5]  = (double)h.ast.inplace_step_end_ms;
    got_final[6]  = (double)h.st.crawl_phase;
    got_final[7]  = (double)h.cmd.gait_mode;
    got_final[8]  = (double)h.st.t;
    got_final[9]  = (double)h.cmd.spd;
    got_final[10] = (double)h.cmd.L;
    got_final[11] = (double)h.cmd.R;
    got_final[12] = (double)h.cmd.goal[CONTROL_CHAIN_GOAL_H];
    got_final[13] = (double)h.st.R_H;
    got_final[14] = (double)h.cmd.goal[CONTROL_CHAIN_GOAL_PIT];
    got_final[15] = (double)h.cmd.goal[CONTROL_CHAIN_GOAL_ROL];
    got_final[16] = (double)h.cmd.goal[CONTROL_CHAIN_GOAL_X];
    got_final[17] = (double)h.ccfg.front_leg_y_offset;
    got_final[18] = (double)h.ccfg.rear_leg_y_offset;
    got_final[19] = (double)h.st.init_case;
    note_state(steps, "t_off_end", (double)(now - t0), exp_final[0]);
    note_state(steps, "n_groups", (double)gi, exp_final[1]);
    note_state(steps, "n_writes", (double)n_writes, exp_final[2]);
    for (int i = 0; i < 20; ++i) {
        note_state(steps, FINAL_NAMES[i], got_final[i], exp_final[3 + i]);
    }

    printf("========================================================\n");
    printf(" action wave      (padog.action_wave_direct -> write log)\n");
    printf("========================================================\n");
    printf("golden file : %s\n", path);
    printf("steps       : %ld   (write steps: %d, golden writes: %ld)\n",
           steps, gi, n_writes_expected);
    printf("writes      : %ld\n", n_writes);
    printf("duty checks : %ld   (allowed %d counts, max %ld)\n",
           g_values, ACTION_DUTY_TOL, g_max_duty_err);
    printf("mismatches  : %ld\n", g_bad);
    return (g_bad == 0) ? 0 : 1;
}

/* ==================================================================== */

int main(int argc, char **argv)
{
    const char *pose_path  = (argc > 1) ? argv[1] : "golden/action_pose.csv";
    const char *cmd_path   = (argc > 2) ? argv[2] : "golden/action_cmd.csv";
    const char *wave_path  = (argc > 3) ? argv[3] : "golden/action_wave.csv";
    const char *final_path = (argc > 4) ? argv[4] : "golden/action_wave_final.csv";

    const int r1 = run_pose(pose_path);
    printf("\n");
    const int r2 = run_cmd(cmd_path);
    printf("\n");
    const int r3 = run_wave(wave_path, final_path);
    printf("\n");

    if (r1 == 2 || r2 == 2 || r3 == 2) {
        printf("RESULT: FAIL -- a golden file is missing or unreadable\n");
        return 2;
    }
    if (r1 != 0 || r2 != 0 || r3 != 0) {
        printf("RESULT: FAIL -- action layer differs from the MicroPython reference\n");
        return 1;
    }
    printf("RESULT: PASS -- pose tables, action entries and the whole wave script "
           "match padog.py exactly\n");
    return 0;
}
