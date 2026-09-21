/*
 * test_moving_avg.c —— 宿主侧 golden 测试：PA_AVGFILT.avg_filiter
 *
 * ⚠️ 这个模块是**有状态**的，所以测法和其他三个不同：
 *    CSV 是一串**按顺序的调用**，同一 window 的行必须依次喂给同一个实例。
 *    遇到新的 window 值就重新初始化（模拟 Python 里重新构造 avg_filiter）。
 *
 * 输出是整数，所以要求**精确相等**，没有容差。
 *
 * NOTE: ASCII output only, for the reason recorded as P-06 in 问题与解决记录.md.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "control/filter_moving_avg.h"

#define BUF_MAX 64

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "golden/moving_avg.csv";

    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        return 2;
    }

    int32_t buf[BUF_MAX];
    moving_avg_t filt;
    memset(&filt, 0, sizeof(filt));

    char  line[512];
    long  rows = 0, mismatches = 0;
    int   cur_window = -1;
    long  cur_step_expect = 0;
    int   windows_seen[16];
    int   n_windows = 0;
    int   first_bad_shown = 0;

    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        if (strncmp(line, "window,", 7) == 0) {
            continue;   /* 表头 */
        }

        int w = 0;
        long step = 0;
        long in = 0, out = 0;
        if (sscanf(line, "%d,%ld,%ld,%ld", &w, &step, &in, &out) != 4) {
            fprintf(stderr, "ERROR: parse failed at row %ld:\n%s", rows + 1, line);
            fclose(f);
            return 2;
        }
        if (w < 1 || w > BUF_MAX) {
            fprintf(stderr, "ERROR: window %d out of range 1..%d\n", w, BUF_MAX);
            fclose(f);
            return 2;
        }

        if (w != cur_window) {
            moving_avg_init(&filt, buf, (uint16_t)w);
            cur_window = w;
            cur_step_expect = 0;
            if (n_windows < (int)(sizeof(windows_seen) / sizeof(windows_seen[0]))) {
                windows_seen[n_windows++] = w;
            }
        }
        if (step != cur_step_expect) {
            fprintf(stderr, "WARNING: step out of order at row %ld (got %ld, expected %ld)\n",
                    rows + 1, step, cur_step_expect);
        }
        ++cur_step_expect;

        const int32_t got = moving_avg_push(&filt, (int32_t)in);

        if ((long)got != out) {
            ++mismatches;
            if (!first_bad_shown) {
                first_bad_shown = 1;
                printf("FIRST MISMATCH: window=%d step=%ld in=%ld expected=%ld got=%ld\n",
                       w, step, in, out, (long)got);
            }
        }
        ++rows;
    }
    fclose(f);

    printf("========================================================\n");
    printf(" moving_avg golden test  (integer filter, exact match)\n");
    printf("========================================================\n");
    printf("golden file : %s\n", path);
    printf("rows        : %ld   (stateful: rows are fed in order)\n", rows);
    printf("windows     : ");
    for (int i = 0; i < n_windows; ++i) {
        printf("%d%s", windows_seen[i], (i + 1 < n_windows) ? ", " : "\n");
    }
    printf("mismatches  : %ld\n", mismatches);

    printf("\n");
    if (mismatches > 0) {
        printf("RESULT: FAIL -- %ld of %ld outputs differ\n", mismatches, rows);
        return 1;
    }
    printf("RESULT: PASS -- all %ld outputs match exactly\n", rows);
    return 0;
}
