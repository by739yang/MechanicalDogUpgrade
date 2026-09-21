/*
 * test_kinematics.c —— 宿主侧 golden 测试
 *
 * 把 golden CSV（由 gen_golden.py 从原始 PA_IK.py 生成）逐行喂给 C 版 kin_ik()，
 * 比较每一项关节角，报告最大误差。
 *
 * 验收标准（来自 ESP-IDF_C迁移表.md）：与 Python 参考角度误差 < 0.5°
 *
 * NOTE: output is deliberately ASCII/English. Printing non-ASCII from a program
 *       to a Windows console hits the codepage problem we already recorded as
 *       P-06 in 问题与解决记录.md. Numeric tool output does not need Chinese.
 *
 * Build & run: see run_golden.bat
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "control/kinematics.h"

#define TOL_DEG   0.5     /* 验收容差（度） */
#define N_FIELD   8

static const char *field_name[N_FIELD] = {
    "ham1", "ham2", "ham3", "ham4",
    "shank1", "shank2", "shank3", "shank4",
};

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "golden/ik.csv";

    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        fprintf(stderr, "Hint: run gen_golden.py first.\n");
        return 2;
    }

    char line[2048];
    long rows = 0, fails = 0, nans = 0;
    long rows_case[2] = {0, 0};
    double field_max[N_FIELD];
    for (int i = 0; i < N_FIELD; ++i) {
        field_max[i] = 0.0;
    }
    double worst = 0.0;
    int    worst_field = -1;
    double worst_exp = 0.0, worst_got = 0.0;
    int    worst_case = -1;
    char   worst_line[512];
    worst_line[0] = '\0';

    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        if (strncmp(line, "case,", 5) == 0) {
            continue;   /* 表头 */
        }

        int    cs = 0;
        double l1 = 0, l2 = 0;
        double xs[KIN_LEG_COUNT], ys[KIN_LEG_COUNT], exp[N_FIELD];

        const int got = sscanf(line,
            "%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
            "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
            &cs, &l1, &l2,
            &xs[0], &xs[1], &xs[2], &xs[3],
            &ys[0], &ys[1], &ys[2], &ys[3],
            &exp[0], &exp[1], &exp[2], &exp[3],
            &exp[4], &exp[5], &exp[6], &exp[7]);
        if (got != 19) {
            fprintf(stderr, "ERROR: parse failed (%d fields) at row %ld:\n%s",
                    got, rows + 1, line);
            fclose(f);
            return 2;
        }

        float fx[KIN_LEG_COUNT], fy[KIN_LEG_COUNT];
        for (int i = 0; i < KIN_LEG_COUNT; ++i) {
            fx[i] = (float)xs[i];
            fy[i] = (float)ys[i];
        }

        kin_ik_result_t r;
        kin_ik((kin_mode_t)cs, (float)l1, (float)l2, fx, fy, &r);

        const double gotv[N_FIELD] = {
            r.ham[0], r.ham[1], r.ham[2], r.ham[3],
            r.shank[0], r.shank[1], r.shank[2], r.shank[3],
        };

        for (int i = 0; i < N_FIELD; ++i) {
            if (isnan(gotv[i]) || isinf(gotv[i])) {
                ++nans;
                fprintf(stderr, "ERROR: non-finite output %s at row %ld\n",
                        field_name[i], rows + 1);
                continue;
            }
            const double e = fabs(gotv[i] - exp[i]);
            if (e > field_max[i]) {
                field_max[i] = e;
            }
            if (e > worst) {
                worst = e;
                worst_field = i;
                worst_exp = exp[i];
                worst_got = gotv[i];
                worst_case = cs;
                strncpy(worst_line, line, sizeof(worst_line) - 1);
                worst_line[sizeof(worst_line) - 1] = '\0';
            }
            if (e > TOL_DEG) {
                ++fails;
            }
        }

        if (cs == 0 || cs == 1) {
            ++rows_case[cs];
        }
        ++rows;
    }
    fclose(f);

    printf("========================================================\n");
    printf(" kinematics golden test  (C float  vs  MicroPython double)\n");
    printf("========================================================\n");
    printf("golden file : %s\n", path);
    printf("rows        : %ld   (case=0 series: %ld, case=1 parallel: %ld)\n",
           rows, rows_case[0], rows_case[1]);
    printf("samples     : %ld joint angles compared\n", rows * N_FIELD);
    printf("tolerance   : %.3f deg\n", TOL_DEG);
    printf("\nper-field max error (deg):\n");
    for (int i = 0; i < N_FIELD; ++i) {
        printf("  %-8s %12.7f\n", field_name[i], field_max[i]);
    }

    if (worst_field >= 0) {
        printf("\nworst case: %s  expected=%.7f  got=%.7f  err=%.7f\n",
               field_name[worst_field], worst_exp, worst_got, worst);
        printf("  (case=%d)\n", worst_case);
        printf("  input line: %s", worst_line);
    }

    printf("\n");
    if (nans > 0) {
        printf("RESULT: FAIL -- %ld non-finite outputs\n", nans);
        return 1;
    }
    if (worst > TOL_DEG) {
        printf("RESULT: FAIL -- max error %.7f deg exceeds %.3f deg (%ld samples over)\n",
               worst, TOL_DEG, fails);
        return 1;
    }
    printf("RESULT: PASS -- max error %.7f deg  (%.1fx inside the %.3f deg tolerance)\n",
           worst, TOL_DEG / (worst > 0 ? worst : 1e-9), TOL_DEG);
    return 0;
}
