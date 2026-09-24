/*
 * test_control_chain.c —— 宿主侧 golden 测试：control_chain.c（P3 全链路）
 *
 * 这一套和前面六个 suite 的区别：参考值不是"把模块拼起来自己推的"，而是
 * `gen_golden.py` 里 **整个 exec 真版 padog.py、直接调用它的 mainloop() 一次**
 * 跑出来的（`gen_control_chain()`）。也就是说参考值就是"原版固件真正在做的那一步"。
 *
 * CSV 列（39 个，逗号分隔）：
 *   0      spd
 *   1      L
 *   2      R
 *   3      gait_mode
 *   4      t          <- 进 state
 *   5      R_H        <- 进 state
 *   6      H_goal
 *   7      PIT_S      <- 进 state
 *   8      PIT_goal
 *   9      ROL_S      <- 进 state
 *   10     ROL_goal
 *   11     X_S        <- 进 state
 *   12     X_goal
 *   13     joy_turn
 *   14     crawl_phase
 *   15..38 12 组 (ON, OFF)，顺序为逻辑通道 0..11
 *
 * 每行都是**全新的 state**：参考值每行都重新 exec 一遍 padog.py，模块级状态归零。
 *
 * 容差：**0（精确相等）**。理由见 test_servo_map.c 的注释：C 用 float、Python 用
 * double，理论上占空比的 int() 截断可能跨边界差 1；前面的 suite 实测 1004 项全精确
 * 相等，所以这里也从 0 起步。真踩到边界时看见 FAIL 再判断，比默认放水强。
 *
 * NOTE: ASCII output only, for the reason recorded as P-06 in 问题与解决记录.md.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "control/control_chain.h"
#include "control/servo_map.h"

/** 允许的占空比偏差（计数单位）。0 = 精确相等。 */
#define CHAIN_TOL 0

#define LINE_MAX 4096
#define FIELDS_MAX 64

/** CSV 输入列数（15）+ 12 组 (ON, OFF) */
#define EXPECT_FIELDS (15 + 2 * CONTROL_CHAIN_CHANNELS)

/* ------------------------------------------------------------------ */

static long g_values = 0;      /* 比较过的 (ON,OFF) 对数 */
static long g_bad = 0;         /* 不符的对数 */
static long g_exact = 0;       /* 精确相等的对数 */
static long g_max_delta = 0;   /* 最大偏差（计数单位） */
static long g_max_delta_row = -1;
static int  g_shown = 0;
static long g_ch_max_delta[CONTROL_CHAIN_CHANNELS];

static long note_delta(long ch, long row, long delta)
{
    if (delta > g_ch_max_delta[ch]) {
        g_ch_max_delta[ch] = delta;
    }
    if (delta == 0) {
        ++g_exact;
        return 0;
    }
    if (delta > g_max_delta) {
        g_max_delta = delta;
        g_max_delta_row = row;
    }
    return 1;
}

/** 打印一行失败时的中间量（诊断用；只在第一次 FAIL 时打） */
static void dump_trace(const control_chain_trace_t *tr)
{
    static const char *leg_name[CONTROL_CHAIN_LEGS] = { "1(LF)", "2(RF)", "3(RR)", "4(LR)" };
    printf("  gait x : %.9f %.9f %.9f %.9f\n",
           tr->gait[0], tr->gait[1], tr->gait[2], tr->gait[3]);
    printf("  gait y : %.9f %.9f %.9f %.9f\n",
           tr->gait[4], tr->gait[5], tr->gait[6], tr->gait[7]);
    printf("  ges  x : %.9f %.9f %.9f %.9f\n",
           tr->ges[0], tr->ges[1], tr->ges[2], tr->ges[3]);
    printf("  ges  y : %.9f %.9f %.9f %.9f\n",
           tr->ges[4], tr->ges[5], tr->ges[6], tr->ges[7]);
    for (int i = 0; i < CONTROL_CHAIN_LEGS; ++i) {
        printf("  leg %s: foot_y=%.9f ham=%.9f shank=%.9f hip=%.9f cs=%.9f\n",
               leg_name[i], tr->foot_y[i], tr->ham[i], tr->shank[i],
               tr->hip[i], tr->crawl_cs[i]);
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

/* ------------------------------------------------------------------ */

static int run_chain(const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        return 2;
    }

    double v[FIELDS_MAX];
    char line[LINE_MAX];
    long rows = 0;
    long gait_rows[4] = { 0, 0, 0, 0 };
    long crawl_rows = 0;

    g_values = g_bad = g_exact = g_max_delta = 0;
    g_max_delta_row = -1;
    g_shown = 0;
    memset(g_ch_max_delta, 0, sizeof(g_ch_max_delta));

    control_chain_cfg_t cfg;
    control_chain_cfg_defaults(&cfg);

    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }

        const int n = split_csv(line, v, FIELDS_MAX);
        if (n != EXPECT_FIELDS) {
            fprintf(stderr, "ERROR: expected %d fields, got %d at row %ld\n",
                    EXPECT_FIELDS, n, rows + 1);
            fclose(f);
            return 2;
        }

        /* ---- 15 个输入 ---- */
        control_chain_state_t st;
        control_chain_state_init(&st, &cfg);
        st.t     = (float)v[4];
        st.R_H   = (float)v[5];
        st.PIT_S = (float)v[7];
        st.ROL_S = (float)v[9];
        st.X_S   = (float)v[11];

        control_chain_input_t in;
        control_chain_input_init(&in, &cfg);
        in.spd         = (float)v[0];
        in.L           = (int)v[1];
        in.R           = (int)v[2];
        in.gait_mode   = (int)v[3];
        in.H_goal      = (float)v[6];
        in.PIT_goal    = (float)v[8];
        in.ROL_goal    = (float)v[10];
        in.X_goal      = (float)v[12];
        in.joy_turn    = (float)v[13];
        in.crawl_phase = (int)v[14];
        /* 参考实现里两个爬行截止时刻都是模块初值 0，而 `now` 是真实时钟：
         * ticks_diff(0, now) <= 0 恒成立。这里传 0 走的是同一个分支。 */
        in.now_ms = 0;

        /* ---- 跑一帧 ---- */
        control_chain_out_t out;
        control_chain_tick(&cfg, &st, &in, &out);

        /* ---- 与参考值逐通道比对（角度 -> 占空比 -> 寄存器，与参考同一套编码） ---- */
        int k = 15;
        for (int ch = 0; ch < CONTROL_CHAIN_CHANNELS; ++ch) {
            const uint16_t exp_on  = (uint16_t)v[k++];
            const uint16_t exp_off = (uint16_t)v[k++];

            const uint16_t duty = servo_map_deg_to_duty(out.angle_deg[ch]);
            uint16_t got_on = 0, got_off = 0;
            servo_map_duty_to_pwm(duty, &got_on, &got_off);

            const long d_on  = labs((long)exp_on - (long)got_on);
            const long d_off = labs((long)exp_off - (long)got_off);
            const long d     = (d_on > d_off) ? d_on : d_off;

            if (note_delta(ch, rows, d) && d > CHAIN_TOL) {
                ++g_bad;
            }
            if (d > CHAIN_TOL && !g_shown) {
                g_shown = 1;
                printf("FIRST MISMATCH: control_chain row=%ld ch=%d (%s)\n",
                       rows, ch, servo_map_channel_name((uint8_t)ch));
                printf("  expected=(%u,%u) got=(%u,%u)  angle=%.9f duty=%u\n",
                       (unsigned)exp_on, (unsigned)exp_off,
                       (unsigned)got_on, (unsigned)got_off,
                       out.angle_deg[ch], (unsigned)duty);
                printf("  state after tick: t=%.9f R_H=%.9f PIT_S=%.9f ROL_S=%.9f X_S=%.9f"
                       " gait_mode=%d crawl_phase=%d init_case=%d\n",
                       st.t, st.R_H, st.PIT_S, st.ROL_S, st.X_S,
                       st.gait_mode, st.crawl_phase, st.init_case);
                dump_trace(&out.trace);
            }
            ++g_values;
        }

        if (in.gait_mode >= 0 && in.gait_mode < 4) {
            ++gait_rows[in.gait_mode];
        }
        if (in.crawl_phase != 0) {
            ++crawl_rows;
        }
        ++rows;
    }
    fclose(f);

    printf("========================================================\n");
    printf(" control chain   (padog.mainloop -> 12 servo channels)\n");
    printf("========================================================\n");
    printf("golden file : %s\n", path);
    printf("rows        : %ld   (gait0/trot: %ld, gait1/walk: %ld, other: %ld)\n",
           rows, gait_rows[0], gait_rows[1], gait_rows[2] + gait_rows[3]);
    printf("crawl rows  : %ld   (crawl_phase != 0 in the input)\n", crawl_rows);
    printf("pairs       : %ld  (12 channels x %ld rows; each pair = ON + OFF)\n",
           g_values, rows);
    printf("exact pairs : %ld / %ld\n", g_exact, g_values);
    printf("max delta   : %ld duty counts  (allowed %d)\n", g_max_delta, CHAIN_TOL);
    if (g_max_delta_row >= 0) {
        printf("max delta at: row %ld\n", g_max_delta_row);
    }
    printf("per-channel max delta:\n");
    for (int ch = 0; ch < CONTROL_CHAIN_CHANNELS; ++ch) {
        printf("  ch%-2d %-10s %ld\n", ch, servo_map_channel_name((uint8_t)ch),
               g_ch_max_delta[ch]);
    }
    printf("mismatches  : %ld\n", g_bad);
    return (g_bad == 0) ? 0 : 1;
}

/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "golden/control_chain.csv";

    const int rc = run_chain(path);

    if (rc == 2) {
        printf("RESULT: FAIL -- golden file missing or unreadable\n");
        return 2;
    }
    if (rc != 0) {
        printf("RESULT: FAIL -- servo channels differ from the MicroPython reference\n");
        return 1;
    }
    printf("RESULT: PASS -- all 12 channels match the original mainloop() exactly\n");
    return 0;
}
