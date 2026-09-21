/*
 * test_body_pose.c —— 宿主侧 golden 测试：PA_ATTITUDE.cal_ges
 *
 * 来源：golden/body_pose.csv（由 gen_golden.py 从原始 micropython/PA_ATTITUDE.py 生成）
 * 验收标准：8 项足端输出误差 < 0.5 mm（ESP-IDF_C迁移表.md）
 *
 * NOTE: ASCII output only, for the reason recorded as P-06 in 问题与解决记录.md.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "control/body_pose.h"

#define TOL_MM   0.5
#define N_FIELD  8

static const char *field_name[N_FIELD] = {
    "x1", "x2", "x3(=leg4)", "x4(=leg3)",
    "y1", "y2", "y3(=leg4)", "y4(=leg3)",
};

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "golden/body_pose.csv";

    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        return 2;
    }

    char line[2048];
    long rows = 0, over = 0, nans = 0;
    double field_max[N_FIELD];
    for (int i = 0; i < N_FIELD; ++i) {
        field_max[i] = 0.0;
    }
    double worst = 0.0;
    int    worst_field = -1;
    double worst_exp = 0.0, worst_got = 0.0;
    char   worst_line[512];
    worst_line[0] = '\0';

    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        if (strncmp(line, "pit,", 4) == 0) {
            continue;   /* 表头 */
        }

        double pit = 0, rol = 0, l = 0, b = 0, w = 0, xoff = 0, hc = 0;
        double exp[N_FIELD];

        const int got = sscanf(line,
            "%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
            "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
            &pit, &rol, &l, &b, &w, &xoff, &hc,
            &exp[0], &exp[1], &exp[2], &exp[3],
            &exp[4], &exp[5], &exp[6], &exp[7]);
        if (got != 15) {
            fprintf(stderr, "ERROR: parse failed (%d fields) at row %ld:\n%s",
                    got, rows + 1, line);
            fclose(f);
            return 2;
        }

        body_pose_result_t r;
        body_pose_cal_ges((float)pit, (float)rol, (float)l, (float)b, (float)w,
                          (float)xoff, (float)hc, &r);

        const double gotv[N_FIELD] = {
            r.x[0], r.x[1], r.x[2], r.x[3],
            r.y[0], r.y[1], r.y[2], r.y[3],
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
                strncpy(worst_line, line, sizeof(worst_line) - 1);
                worst_line[sizeof(worst_line) - 1] = '\0';
            }
            if (e > TOL_MM) {
                ++over;
            }
        }
        ++rows;
    }
    fclose(f);

    printf("========================================================\n");
    printf(" body_pose golden test  (C float  vs  MicroPython double)\n");
    printf("========================================================\n");
    printf("golden file : %s\n", path);
    printf("rows        : %ld\n", rows);
    printf("samples     : %ld foot-target values compared\n", rows * N_FIELD);
    printf("tolerance   : %.3f mm\n", TOL_MM);
    printf("\nper-field max error (mm):\n");
    for (int i = 0; i < N_FIELD; ++i) {
        printf("  %-12s %14.9f\n", field_name[i], field_max[i]);
    }
    if (worst_field >= 0) {
        printf("\nworst case: %s  expected=%.7f  got=%.7f  err=%.7f\n",
               field_name[worst_field], worst_exp, worst_got, worst);
        printf("  input line: %s", worst_line);
    }

    printf("\n");
    if (nans > 0) {
        printf("RESULT: FAIL -- %ld non-finite outputs\n", nans);
        return 1;
    }
    if (worst > TOL_MM) {
        printf("RESULT: FAIL -- max error %.9f mm exceeds %.3f mm (%ld samples over)\n",
               worst, TOL_MM, over);
        return 1;
    }
    printf("RESULT: PASS -- max error %.9f mm  (%.0fx inside the %.3f mm tolerance)\n",
           worst, TOL_MM / (worst > 0 ? worst : 1e-12), TOL_MM);
    return 0;
}
