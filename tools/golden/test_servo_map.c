/*
 * test_servo_map.c —— 宿主侧 golden 测试：servo_map.c（P2 的舵机映射层）
 *
 * 两个 suite 在同一个可执行文件里跑，因为它们是同一条链的两段：
 *
 *   [A] servo_angle.csv   —— 复刻 PA_SERVO.py 的 `Servos.position()` + `PCA9685.duty()`
 *       参考值是把**真的 PA_SERVO.py** 装上一个"记录型 I2C"跑出来的，
 *       也就是原代码真正会写进 PCA9685 的字节。
 *       CSV: kind,ch,val,addr,pca_ch,on,off
 *         kind=0 -> angle(ch, val)         （val 是角度）
 *         kind=1 -> pca9685.duty(ch, val)  （val 是占空比，覆盖 0/4095 两个特殊分支）
 *
 *   [B] servo_output.csv  —— 复刻 padog.py 的 `servo_output()`
 *       参考值是把 padog.py 里的 servo_output 及其依赖函数用 ast **原样抠出来**
 *       exec 得到的（不是手抄公式）。
 *       CSV: ik,hip×4,ham×4,shank×4,cs×4,init×12,trim×4,l1,l2,ref, 然后 12×(on,off)
 *
 * 容差：**要求精确相等**（0 个计数单位的偏差）。
 *       本来担心"C 用 float、Python 用 double，占空比的 int() 截断会跨边界差 1"，
 *       于是留了 ±1 的余量。实际跑下来 720/720 + 284/284 全是精确相等，
 *       所以直接收到 0 —— 越严越好。将来真的踩到边界时看见 FAIL 再去判断，
 *       也比默认放水把真差异盖掉强。
 *
 * NOTE: ASCII output only, for the reason recorded as P-06 in 问题与解决记录.md.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "control/servo_map.h"

/** 允许的占空比偏差（计数单位）。0 = 精确相等。 */
#define ANGLE_TOL 0

#define LINE_MAX 4096
#define FIELDS_MAX 64

/* ------------------------------------------------------------------ */

static long g_rows = 0;
static long g_bad = 0;
static long g_exact = 0;
static long g_max_delta = 0;
static int  g_shown = 0;

static void note_delta(long delta)
{
    if (delta == 0) {
        ++g_exact;
        return;
    }
    if (delta > g_max_delta) {
        g_max_delta = delta;
    }
}

static int check_pair(const char *what, long row, uint16_t exp_on, uint16_t exp_off,
                      uint16_t got_on, uint16_t got_off, long tol)
{
    const long d_on  = labs((long)exp_on - (long)got_on);
    const long d_off = labs((long)exp_off - (long)got_off);
    const long d     = (d_on > d_off) ? d_on : d_off;

    note_delta(d);
    if (d > tol) {
        ++g_bad;
        if (!g_shown) {
            g_shown = 1;
            printf("FIRST MISMATCH: %s row=%ld expected=(%u,%u) got=(%u,%u)\n",
                   what, row, (unsigned)exp_on, (unsigned)exp_off,
                   (unsigned)got_on, (unsigned)got_off);
        }
        return 0;
    }
    return 1;
}

/* ------------------------------------------------------------------ */
/* [A] 角度 / 占空比                                                     */
/* ------------------------------------------------------------------ */

static int run_angle(const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        return 2;
    }

    char line[LINE_MAX];
    long rows = 0;
    long by_kind[2] = { 0, 0 };

    g_rows = g_bad = g_exact = g_max_delta = 0;
    g_shown = 0;

    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }

        int kind = 0, ch = 0, addr = 0, pca_ch = 0;
        double val = 0.0;
        unsigned exp_on = 0, exp_off = 0;

        if (sscanf(line, "%d,%d,%lf,%d,%d,%u,%u",
                   &kind, &ch, &val, &addr, &pca_ch, &exp_on, &exp_off) != 7) {
            fprintf(stderr, "ERROR: parse failed at row %ld:\n%s", rows + 1, line);
            fclose(f);
            return 2;
        }

        const servo_map_hw_t *hw = servo_map_hw((uint8_t)ch);
        if (hw == NULL) {
            fprintf(stderr, "ERROR: logical channel %d out of range\n", ch);
            fclose(f);
            return 2;
        }

        /* 硬件位置表必须与参考值一致（板地址 + 板上通道） */
        if ((int)hw->board_addr != addr || (int)hw->pca_ch != pca_ch) {
            printf("FIRST MISMATCH: hw map ch=%d expected=(0x%02X,%d) got=(0x%02X,%d)\n",
                   ch, addr, pca_ch, hw->board_addr, hw->pca_ch);
            ++g_bad;
        }

        uint16_t duty;
        long tol;
        if (kind == 0) {
            duty = servo_map_deg_to_duty((float)val);
            tol  = ANGLE_TOL;
        } else {
            duty = (uint16_t)val;
            tol  = 0;   /* 占空比是纯整数运算，必须精确 */
        }

        uint16_t got_on = 0, got_off = 0;
        servo_map_duty_to_pwm(duty, &got_on, &got_off);

        check_pair(kind == 0 ? "angle" : "duty", rows, (uint16_t)exp_on, (uint16_t)exp_off,
                   got_on, got_off, tol);
        if (kind >= 0 && kind < 2) {
            ++by_kind[kind];
        }
        ++rows;
        ++g_rows;
    }
    fclose(f);

    printf("========================================================\n");
    printf(" [A] angle / duty path   (Servos.position + PCA9685.duty)\n");
    printf("========================================================\n");
    printf("golden file : %s\n", path);
    printf("rows        : %ld   (angle: %ld, duty: %ld)\n", rows, by_kind[0], by_kind[1]);
    printf("exact rows  : %ld / %ld\n", g_exact, rows);
    printf("max delta   : %ld duty counts  (angle rows allowed %d, duty rows 0)\n",
           g_max_delta, ANGLE_TOL);
    printf("min/max duty: %u / %u\n", (unsigned)servo_map_min_duty(),
           (unsigned)servo_map_max_duty());
    printf("mismatches  : %ld\n", g_bad);
    return (g_bad == 0) ? 0 : 1;
}

/* ------------------------------------------------------------------ */
/* [B] 关节角 -> 12 路                                                   */
/* ------------------------------------------------------------------ */

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

static int run_output(const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        return 2;
    }

    /*
     * 列布局（64 列），与 gen_golden.py 的 gen_servo_output() 一一对应：
     *   0      ik（1 = 走 IK 路径）
     *   1..4   ROL_S, PIT_S, joy_turn, crawl_phase  —— 参考实现算 hip/cs 用的**输入**
     *   5..8   h1..h4   （由真版 `_hip_leg_deltas()` 算出）
     *   9..12  ham1..ham4
     *   13..16 shank1..shank4
     *   17..20 cs1..cs4 （由真版 `_crawl_shank_servodelta()` 算出）
     *   21..32 中位角 12 个（腿1..腿4 × 髋/大/小）
     *   33..36 s_trim × 4
     *   37..39 l1, l2, leg_len_ref
     *   40..63 12 组 (ON, OFF)，顺序为逻辑通道 0..11
     */
    enum { EXPECT_FIELDS = 64 };
    double v[FIELDS_MAX];

    char line[LINE_MAX];
    long rows = 0;
    long ik_rows = 0, direct_rows = 0;

    g_rows = g_bad = g_exact = g_max_delta = 0;
    g_shown = 0;

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

        servo_map_input_t in;
        memset(&in, 0, sizeof(in));

        int k = 0;
        in.ik_path = (v[k++] != 0.0);
        k += 4;   /* ROL_S / PIT_S / joy_turn / crawl_phase —— 只用来生成参考值 */
        for (int i = 0; i < 4; ++i) { in.hip[i] = (float)v[k++]; }
        for (int i = 0; i < 4; ++i) { in.ham[i] = (float)v[k++]; }
        for (int i = 0; i < 4; ++i) { in.shank[i] = (float)v[k++]; }
        for (int i = 0; i < 4; ++i) { in.crawl_cs[i] = (float)v[k++]; }
        for (int leg = 0; leg < 4; ++leg) {
            for (int j = 0; j < 3; ++j) {
                in.init[leg][j] = (float)v[k++];
            }
        }
        for (int i = 0; i < 4; ++i) { in.s_trim[i] = (float)v[k++]; }
        const float l1  = (float)v[k++];
        const float l2  = (float)v[k++];
        const float ref = (float)v[k++];

        /* bias 由 C 版自己算 —— 参考值是原 _shank_ik_bias() 算的，正好互相校验 */
        in.shank_bias = servo_map_shank_bias(l1, l2, ref);

        uint16_t on[SERVO_MAP_CHANNELS], off[SERVO_MAP_CHANNELS];
        servo_map_legs_to_pwm(&in, on, off);

        for (int ch = 0; ch < SERVO_MAP_CHANNELS; ++ch) {
            const uint16_t exp_on  = (uint16_t)v[k++];
            const uint16_t exp_off = (uint16_t)v[k++];
            check_pair("servo_output", rows, exp_on, exp_off, on[ch], off[ch], ANGLE_TOL);
            ++g_rows;
        }

        if (in.ik_path) {
            ++ik_rows;
        } else {
            ++direct_rows;
        }
        ++rows;
    }
    fclose(f);

    printf("========================================================\n");
    printf(" [B] joint angles -> 12 channels   (padog.servo_output)\n");
    printf("========================================================\n");
    printf("golden file : %s\n", path);
    printf("rows        : %ld   (ik path: %ld, direct-stand path: %ld)\n",
           rows, ik_rows, direct_rows);
    printf("values      : %ld  (12 channels x 2 registers)\n", g_rows);
    printf("exact values: %ld / %ld\n", g_exact, g_rows);
    printf("max delta   : %ld duty counts  (allowed %d)\n", g_max_delta, ANGLE_TOL);
    printf("mismatches  : %ld\n", g_bad);
    return (g_bad == 0) ? 0 : 1;
}

/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    const char *angle_path  = (argc > 1) ? argv[1] : "golden/servo_angle.csv";
    const char *output_path = (argc > 2) ? argv[2] : "golden/servo_output.csv";

    const int rc_a = run_angle(angle_path);
    printf("\n");
    const int rc_b = run_output(output_path);
    printf("\n");

    if (rc_a == 2 || rc_b == 2) {
        printf("RESULT: FAIL -- golden file missing or unreadable\n");
        return 2;
    }
    if (rc_a != 0 || rc_b != 0) {
        printf("RESULT: FAIL -- duty/PWM values differ from the MicroPython reference\n");
        return 1;
    }
    printf("RESULT: PASS -- all angle/duty/PWM values match the MicroPython reference\n");
    return 0;
}
