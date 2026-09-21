/*
 * test_gait_trot.c —— 宿主侧 golden 测试：PA_TROT.cal_t
 *
 * 来源：golden/gait_trot.csv（由 gen_golden.py 从原始 micropython/PA_TROT.py 生成）
 * 验收标准：步态轨迹误差 < 1 mm（ESP-IDF_C迁移表.md）
 *
 * NOTE: ASCII output only, for the reason recorded as P-06 in 问题与解决记录.md.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "control/gait_trot.h"

#define TOL_MM   1.0
#define N_FIELD  8

static const char *field_name[N_FIELD] = {
    "x1", "x2", "x3", "x4",
    "y1", "y2", "y3", "y4",
};

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "golden/gait_trot.csv";

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
        if (strncmp(line, "ts,", 3) == 0) {
            continue;   /* 表头 */
        }

        double ts = 0, faai = 0, t = 0, xs = 0, xf = 0, h = 0;
        double r1 = 0, r4 = 0, r2 = 0, r3 = 0;
        double exp[N_FIELD];

        const int got = sscanf(line,
            "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
            "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
            &ts, &faai, &t, &xs, &xf, &h, &r1, &r4, &r2, &r3,
            &exp[0], &exp[1], &exp[2], &exp[3],
            &exp[4], &exp[5], &exp[6], &exp[7]);
        if (got != 18) {
            fprintf(stderr, "ERROR: parse failed (%d fields) at row %ld:\n%s",
                    got, rows + 1, line);
            fclose(f);
            return 2;
        }

        const gait_trot_cfg_t cfg = { (float)ts, (float)faai };
        gait_trot_out_t r;
        gait_trot_cal_t(&cfg, (float)t, (float)xs, (float)xf, (float)h,
                        (float)r1, (float)r4, (float)r2, (float)r3, &r);

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
    printf(" gait_trot golden test  (C float  vs  MicroPython double)\n");
    printf("========================================================\n");
    printf("golden file : %s\n", path);
    printf("rows        : %ld\n", rows);
    printf("samples     : %ld trajectory values compared\n", rows * N_FIELD);
    printf("tolerance   : %.3f mm\n", TOL_MM);
    printf("\nper-field max error (mm):\n");
    for (int i = 0; i < N_FIELD; ++i) {
        printf("  %-4s %14.9f\n", field_name[i], field_max[i]);
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
