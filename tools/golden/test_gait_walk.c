/*
 * test_gait_walk.c —— 宿主侧 golden 测试：PA_WALK.cal_w
 *
 * 来源：golden/gait_walk.csv（由 gen_golden.py 从原始 micropython/PA_WALK.py 生成）
 *
 * 两种判据并存：
 *   - 8 个足端输出   → 容差 1.0 mm（迁移表对步态轨迹的要求）
 *   - 3 个重心整数   → 精确相等（原实现是 int()，没有误差余地）
 *
 * 重心那三项来自原实现的副作用 padog.gesture(0, int(CG_X), int(yst))，
 * C 版把它变成了显式输出。
 *
 * NOTE: ASCII output only, for the reason recorded as P-06 in 问题与解决记录.md.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "control/gait_walk.h"

#define TOL_MM      1.0
#define N_FOOT      8
#define N_GESTURE   3

static const char *foot_name[N_FOOT] = {
    "x1", "x2", "x3", "x4", "y1", "y2", "y3", "y4",
};
static const char *gesture_name[N_GESTURE] = { "gpit", "grol", "gx" };

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "golden/gait_walk.csv";

    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        return 2;
    }

    char line[4096];
    long rows = 0, over = 0, gmis = 0, nans = 0;
    double foot_max[N_FOOT];
    for (int i = 0; i < N_FOOT; ++i) {
        foot_max[i] = 0.0;
    }
    double worst = 0.0;
    int    worst_field = -1;
    double worst_exp = 0.0, worst_got = 0.0;
    int    first_gmis = 0;
    char   worst_line[512];
    worst_line[0] = '\0';

    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        if (strncmp(line, "ts,", 3) == 0) {
            continue;   /* 表头 */
        }

        double in[14];
        double exp_foot[N_FOOT];
        double exp_g[N_GESTURE];
        double *dst[25] = {
            &in[0], &in[1], &in[2], &in[3], &in[4], &in[5], &in[6],
            &in[7], &in[8], &in[9], &in[10], &in[11], &in[12], &in[13],
            &exp_foot[0], &exp_foot[1], &exp_foot[2], &exp_foot[3],
            &exp_foot[4], &exp_foot[5], &exp_foot[6], &exp_foot[7],
            &exp_g[0], &exp_g[1], &exp_g[2],
        };

        const int got = sscanf(line,
            "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
            "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
            dst[0],  dst[1],  dst[2],  dst[3],  dst[4],  dst[5],  dst[6],
            dst[7],  dst[8],  dst[9],  dst[10], dst[11], dst[12], dst[13],
            dst[14], dst[15], dst[16], dst[17], dst[18], dst[19], dst[20],
            dst[21], dst[22], dst[23], dst[24]);
        if (got != 25) {
            fprintf(stderr, "ERROR: parse failed (%d fields) at row %ld:\n%s",
                    got, rows + 1, line);
            fclose(f);
            return 2;
        }

        const gait_walk_cfg_t cfg = { (float)in[0], (float)in[1] };
        gait_walk_out_t r;
        gait_walk_gesture_t g;
        /* 形参顺序：cg_x, cg_y, l, xf, h, t, r1, r4, r2, r3, body_h, gyro */
        gait_walk_cal_w(&cfg,
                        (float)in[2], (float)in[3], (float)in[4], (float)in[5],
                        (float)in[6], (float)in[7],
                        (float)in[8], (float)in[9], (float)in[10], (float)in[11],
                        (float)in[12], (float)in[13],
                        &r, &g);

        const double gotv[N_FOOT] = {
            r.x[0], r.x[1], r.x[2], r.x[3],
            r.y[0], r.y[1], r.y[2], r.y[3],
        };

        for (int i = 0; i < N_FOOT; ++i) {
            if (isnan(gotv[i]) || isinf(gotv[i])) {
                ++nans;
                continue;
            }
            const double e = fabs(gotv[i] - exp_foot[i]);
            if (e > foot_max[i]) {
                foot_max[i] = e;
            }
            if (e > worst) {
                worst = e;
                worst_field = i;
                worst_exp = exp_foot[i];
                worst_got = gotv[i];
                strncpy(worst_line, line, sizeof(worst_line) - 1);
                worst_line[sizeof(worst_line) - 1] = '\0';
            }
            if (e > TOL_MM) {
                ++over;
            }
        }

        const long gotg[N_GESTURE] = { (long)g.pit, (long)g.rol, (long)g.x };
        for (int i = 0; i < N_GESTURE; ++i) {
            if (gotg[i] != (long)exp_g[i]) {
                ++gmis;
                if (!first_gmis) {
                    first_gmis = 1;
                    printf("FIRST GESTURE MISMATCH: %s expected=%ld got=%ld\n",
                           gesture_name[i], (long)exp_g[i], gotg[i]);
                    printf("  input line: %s", line);
                }
            }
        }

        ++rows;
    }
    fclose(f);

    printf("========================================================\n");
    printf(" gait_walk golden test  (C float  vs  MicroPython double)\n");
    printf("========================================================\n");
    printf("golden file : %s\n", path);
    printf("rows        : %ld\n", rows);
    printf("foot samples: %ld  (tolerance %.3f mm)\n", rows * N_FOOT, TOL_MM);
    printf("gesture     : %ld  (exact integer match required)\n", rows * N_GESTURE);
    printf("\nper-field max error (mm):\n");
    for (int i = 0; i < N_FOOT; ++i) {
        printf("  %-4s %14.9f\n", foot_name[i], foot_max[i]);
    }
    if (worst_field >= 0) {
        printf("\nworst foot case: %s  expected=%.7f  got=%.7f  err=%.7f\n",
               foot_name[worst_field], worst_exp, worst_got, worst);
    }
    printf("gesture mismatches: %ld\n", gmis);

    printf("\n");
    if (nans > 0) {
        printf("RESULT: FAIL -- %ld non-finite foot outputs\n", nans);
        return 1;
    }
    if (worst > TOL_MM) {
        printf("RESULT: FAIL -- foot max error %.9f mm exceeds %.3f mm (%ld over)\n",
               worst, TOL_MM, over);
        printf("  worst input: %s", worst_line);
        return 1;
    }
    if (gmis > 0) {
        printf("RESULT: FAIL -- %ld gesture values differ\n", gmis);
        return 1;
    }
    printf("RESULT: PASS -- foot max error %.9f mm (%.0fx inside %.3f mm), gestures exact\n",
           worst, TOL_MM / (worst > 0 ? worst : 1e-12), TOL_MM);
    return 0;
}
