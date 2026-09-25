/*
 * test_control_chain_cmd.c —— 宿主侧 golden 测试：多帧序列（P3）
 *
 * 单帧对照（test_control_chain.c）每行都是"干净初值 + 跑一帧"，
 * 所以**结构上**测不出三件事：
 *
 *   1. **跨帧延续**：相位 `t`、姿态 slew（`R_H`/`PIT_S`/`ROL_S`/`X_S`）、
 *      以及四个目标是否被保留住；
 *   2. **命令语义**：`move()` / `gait()` / `height()` / `gesture()` 到底重置了哪些量
 *      （原版 `move()` 里 `gait(0)` 的"模式变了才 t=0"就是个典型陷阱）；
 *   3. **长时间漂移**。
 *
 * 这一套连续跑几百帧，逐帧与"原版 `mainloop()` 连跑同样次数"的参考值对照。
 *
 * 参考值：`golden/control_chain_seq.csv`（每行一帧：`seq, frame, 12×(ON,OFF)`）
 * 命令脚本：`golden/control_chain_seq_cmds.csv`（`seq, frame, action, a0, a1, a2`）
 *          —— **从文件读**，不在本文件里硬编码，避免"同一份脚本两个副本"漂移。
 *
 * 容差 0（精确相等）。
 *
 * NOTE: ASCII output only, for the reason recorded as P-06 in 问题与解决记录.md.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "control/control_chain.h"
#include "control/control_chain_cmd.h"
#include "control/servo_map.h"

#define FRAME_MAX  512
#define LINE_MAX   2048
#define CMD_MAX    256

/* 动作编号，与 gen_golden.py 的 CHAIN_SEQ_ACTION_CODES 一致 */
enum { ACT_MOVE = 0, ACT_GAIT = 1, ACT_HEIGHT = 2, ACT_GESTURE = 3, ACT_SET_TURN = 4,
       ACT_DRIVE = 5 };

typedef struct {
    int   seq;
    int   frame;
    int   action;
    double a[3];
} seq_cmd_t;

static seq_cmd_t g_cmds[CMD_MAX];
static int       g_cmd_count = 0;

static long g_frames = 0;
static long g_pairs = 0;
static long g_bad = 0;
static int  g_shown = 0;

/* ------------------------------------------------------------------ */

static int load_cmds(const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        return -1;
    }
    char line[LINE_MAX];
    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        if (g_cmd_count >= CMD_MAX) {
            fprintf(stderr, "ERROR: too many commands (max %d)\n", CMD_MAX);
            fclose(f);
            return -1;
        }
        seq_cmd_t *c = &g_cmds[g_cmd_count];
        if (sscanf(line, "%d,%d,%d,%lf,%lf,%lf",
                   &c->seq, &c->frame, &c->action,
                   &c->a[0], &c->a[1], &c->a[2]) != 6) {
            fprintf(stderr, "ERROR: bad command row: %s", line);
            fclose(f);
            return -1;
        }
        ++g_cmd_count;
    }
    fclose(f);
    return 0;
}

/** 施加某一帧上挂着的所有命令 */
static void apply_cmds(int seq, int frame,
                       control_chain_cmd_t *cmd,
                       const control_chain_cfg_t *cfg,
                       control_chain_state_t *st)
{
    for (int i = 0; i < g_cmd_count; ++i) {
        const seq_cmd_t *c = &g_cmds[i];
        if (c->seq != seq || c->frame != frame) {
            continue;
        }
        switch (c->action) {
        case ACT_MOVE:
            control_chain_cmd_move(cmd, cfg, st, (float)c->a[0], (int)c->a[1], (int)c->a[2]);
            break;
        case ACT_GAIT:
            control_chain_cmd_gait(cmd, cfg, st, (int)c->a[0]);
            break;
        case ACT_HEIGHT:
            control_chain_cmd_height(cmd, st, (float)c->a[0]);
            break;
        case ACT_GESTURE:
            control_chain_cmd_gesture(cmd, (float)c->a[0], (float)c->a[1], (float)c->a[2]);
            break;
        case ACT_SET_TURN:
            control_chain_cmd_set_turn(cmd, (float)c->a[0]);
            break;
        case ACT_DRIVE:
            /* drive() = move() 去掉 gait(0) ⇒ 进入 WALK 的唯一途径（P-26） */
            control_chain_cmd_drive(cmd, cfg, st, (float)c->a[0], (int)c->a[1], (int)c->a[2]);
            break;
        default:
            fprintf(stderr, "ERROR: unknown action %d\n", c->action);
            break;
        }
    }
}

/* ------------------------------------------------------------------ */

static int run_seq(const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        return 2;
    }

    control_chain_cfg_t cfg;
    control_chain_cfg_defaults(&cfg);

    control_chain_state_t st;
    control_chain_cmd_t   cmd;
    control_chain_state_init(&st, &cfg);
    control_chain_cmd_init(&cmd, &cfg);

    char  line[LINE_MAX];
    int   cur_seq   = -1;
    long  seq_frames = 0;
    int   n_seqs    = 0;
    int   seen_seq[64];

    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }

        int    seq = 0, frame = 0;
        double v[24];
        const int n = sscanf(line,
                             "%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
                             "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                             &seq, &frame,
                             &v[0], &v[1], &v[2], &v[3], &v[4], &v[5],
                             &v[6], &v[7], &v[8], &v[9], &v[10], &v[11],
                             &v[12], &v[13], &v[14], &v[15], &v[16], &v[17],
                             &v[18], &v[19], &v[20], &v[21], &v[22], &v[23]);
        if (n != 26) {
            fprintf(stderr, "ERROR: expected 26 fields, got %d at frame %ld\n",
                    n, g_frames);
            fclose(f);
            return 2;
        }

        /* 换序列 -> 状态完全重置（与参考实现每序列重新 exec padog.py 等价） */
        if (seq != cur_seq) {
            control_chain_state_init(&st, &cfg);
            control_chain_cmd_init(&cmd, &cfg);
            cur_seq = seq;
            seq_frames = 0;
            if (n_seqs < (int)(sizeof(seen_seq) / sizeof(seen_seq[0]))) {
                seen_seq[n_seqs++] = seq;
            }
        }
        if (frame != seq_frames) {
            fprintf(stderr, "WARNING: seq %d frame out of order: got %d expected %ld\n",
                    seq, frame, seq_frames);
        }

        /* 1. 施加这一帧的命令 */
        apply_cmds(seq, frame, &cmd, &cfg, &st);

        /* 2. 推进一帧 */
        control_chain_input_t in;
        memset(&in, 0, sizeof(in));
        control_chain_cmd_make_input(&cmd, 0, &in);

        control_chain_out_t out;
        memset(&out, 0, sizeof(out));
        control_chain_tick(&cfg, &st, &in, &out);

        /* 3. ⚠️ 把本帧实际用到的目标收回 —— 原版那是模块级全局，改一次一直留着 */
        control_chain_cmd_absorb(&cmd, &out);

        /* 4. 逐通道比对 */
        for (int ch = 0; ch < SERVO_MAP_CHANNELS; ++ch) {
            const uint16_t exp_on  = (uint16_t)v[ch * 2 + 0];
            const uint16_t exp_off = (uint16_t)v[ch * 2 + 1];
            uint16_t duty = servo_map_deg_to_duty(out.angle_deg[ch]);
            uint16_t got_on = 0, got_off = 0;
            servo_map_duty_to_pwm(duty, &got_on, &got_off);

            if (exp_on != got_on || exp_off != got_off) {
                ++g_bad;
                if (!g_shown) {
                    g_shown = 1;
                    printf("FIRST MISMATCH: seq=%d frame=%d ch=%d (%s) "
                           "expected=(%u,%u) got=(%u,%u)  angle=%.4f\n",
                           seq, frame, ch, servo_map_channel_name((uint8_t)ch),
                           (unsigned)exp_on, (unsigned)exp_off,
                           (unsigned)got_on, (unsigned)got_off,
                           (double)out.angle_deg[ch]);
                }
            }
            ++g_pairs;
        }
        ++g_frames;
        ++seq_frames;
    }
    fclose(f);

    printf("========================================================\n");
    printf(" multi-frame sequence test   (padog.mainloop() run N times)\n");
    printf("========================================================\n");
    printf("golden file : %s\n", path);
    printf("sequences   : %d   (", n_seqs);
    for (int i = 0; i < n_seqs; ++i) {
        printf("%d%s", seen_seq[i], (i + 1 < n_seqs) ? ", " : ")\n");
    }
    printf("frames      : %ld\n", g_frames);
    printf("commands    : %d   (read from the cmds CSV, not hard-coded here)\n", g_cmd_count);
    printf("pairs       : %ld  (12 channels x frames)\n", g_pairs);
    printf("mismatches  : %ld\n", g_bad);
    return (g_bad == 0) ? 0 : 1;
}

/* ------------------------------------------------------------------ */

/** 顺带验一下节拍算术（纯函数，与上面的序列无关） */
static int check_sched(void)
{
    control_chain_cfg_t cfg;
    control_chain_cfg_defaults(&cfg);

    int fail = 0;
    const uint32_t p = control_chain_sched_period_from_cfg(&cfg);
    printf("sched: period_from_cfg(speed=%.3f) = %u ms\n", (double)cfg.speed, (unsigned)p);
    if (p != 65) {
        printf("  FAIL: expected 65 ms (speed 0.065 -> one cycle = Ts seconds)\n");
        ++fail;
    }

    control_chain_sched_t s;
    control_chain_sched_init(&s, p);

    /* 第一帧立刻推进 */
    if (!control_chain_sched_due(&s, 1000)) { printf("  FAIL: first call should be due\n"); ++fail; }
    /* 没到点不推进 */
    if (control_chain_sched_due(&s, 1030))   { printf("  FAIL: 30 ms < 65 ms should not be due\n"); ++fail; }
    if (!control_chain_sched_due(&s, 1065))  { printf("  FAIL: 65 ms should be due\n"); ++fail; }
    if (!control_chain_sched_due(&s, 1130))  { printf("  FAIL: 1130 should be due\n"); ++fail; }
    /* 落后很多：不追补（next_due 直接推到 now + period） */
    (void)control_chain_sched_due(&s, 5000);
    if (control_chain_sched_due(&s, 5030))   { printf("  FAIL: after a big lag it must not catch up\n"); ++fail; }
    if (!control_chain_sched_due(&s, 5065))  { printf("  FAIL: 5065 should be due\n"); ++fail; }

    /* 一个步态周期的帧数：Ts/speed = 15.38 -> 至少 15~16 帧 */
    const float frames_per_cycle = cfg.Ts / cfg.speed;
    printf("sched: one gait cycle = Ts/speed = %.2f frames = %.0f ms at %u ms/frame\n",
           (double)frames_per_cycle, (double)(frames_per_cycle * (float)p), (unsigned)p);

    printf("sched mismatches: %d\n", fail);
    return fail;
}

int main(int argc, char **argv)
{
    const char *seq_path  = (argc > 1) ? argv[1] : "golden/control_chain_seq.csv";
    const char *cmd_path  = (argc > 2) ? argv[2] : "golden/control_chain_seq_cmds.csv";

    if (load_cmds(cmd_path) != 0) {
        printf("RESULT: FAIL -- cannot read the command script\n");
        return 2;
    }
    const int rc = run_seq(seq_path);
    printf("\n");
    const int sched_fail = check_sched();
    printf("\n");

    if (rc == 2) {
        printf("RESULT: FAIL -- golden file missing or unreadable\n");
        return 2;
    }
    if (rc != 0 || sched_fail != 0) {
        printf("RESULT: FAIL -- multi-frame sequence or scheduler differs from the original\n");
        return 1;
    }
    printf("RESULT: PASS -- every frame matches the original mainloop() exactly, "
           "and the chain cadence arithmetic is correct\n");
    return 0;
}
