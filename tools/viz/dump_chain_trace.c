/*
 * dump_chain_trace.c —— 把控制链的**中间量**逐帧导出，供可视化用
 *
 * 为什么不直接从 golden CSV 拿数据：golden CSV 只有 12 路**占空比**，
 * 而画腿需要的是**足端目标位置**（`PA_IK.ik()` 的两个入参：
 * 前后 x 与竖直 y）。
 *
 * 这里重放与 `test_control_chain_cmd.c` **同一份命令脚本**（从 CSV 读），
 * 逐帧输出：
 *   seq, frame, ang0..ang11, ikx1..ikx4, iky1..iky4
 * 其中 `ikx = P_[i] + ges_x[i]`、`iky = foot_y[i] + ges_y[i]` —— 正是喂给 IK 的东西。
 *
 * ⚠️ 这些量的可信度来自 `control_chain` 自己：它已经被 golden 逐帧钉住
 * （单帧 1080 组 + 多帧 9840 组，零容差）。所以导出的轨迹就是原版的轨迹。
 *
 * 单位：角度 = 度；ikx / iky = 毫米（iky 向下为负）。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "control/control_chain.h"
#include "control/control_chain_cmd.h"

#include "chain_seq_script.h"   /* 命令脚本的**唯一**解析/派发实现（见该头文件说明） */

#define LINE_MAX CHAIN_SEQ_LINE_MAX

static chain_seq_script_t g_script;

int main(int argc, char **argv)
{
    const char *seq_csv = (argc > 1) ? argv[1] : "golden/control_chain_seq.csv";
    const char *cmd_csv = (argc > 2) ? argv[2] : "golden/control_chain_seq_cmds.csv";
    const char *out_csv = (argc > 3) ? argv[3] : "chain_trace.csv";

    if (chain_seq_script_load(&g_script, cmd_csv) != 0) {
        fprintf(stderr, "ERROR: bad command csv\n");
        return 2;
    }

    /* 先扫一遍 seq CSV 拿到 (seq, frame) 序列，保证与 golden 完全同序 */
    FILE *f = fopen(seq_csv, "r");
    if (f == NULL) {
        fprintf(stderr, "ERROR: cannot open %s\n", seq_csv);
        return 2;
    }
    FILE *o = fopen(out_csv, "w");
    if (o == NULL) {
        fprintf(stderr, "ERROR: cannot write %s\n", out_csv);
        fclose(f);
        return 2;
    }

    control_chain_cfg_t cfg;
    control_chain_cfg_defaults(&cfg);
    control_chain_state_t st;
    control_chain_cmd_t   cmd;

    int cur_seq = -1;
    char line[LINE_MAX];
    long rows = 0;

    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        int seq = 0, frame = 0;
        if (sscanf(line, "%d,%d", &seq, &frame) != 2) {
            continue;
        }
        if (seq != cur_seq) {
            control_chain_state_init(&st, &cfg);
            control_chain_cmd_init(&cmd, &cfg);
            cur_seq = seq;
        }

        chain_seq_script_apply(&g_script, seq, frame, &cmd, &cfg, &st);

        control_chain_input_t in;
        memset(&in, 0, sizeof(in));
        control_chain_cmd_make_input(&cmd, 0, &in);

        control_chain_out_t out;
        memset(&out, 0, sizeof(out));
        control_chain_tick(&cfg, &st, &in, &out);
        control_chain_cmd_absorb(&cmd, &out);

        fprintf(o, "%d,%d", seq, frame);
        for (int ch = 0; ch < CONTROL_CHAIN_CHANNELS; ++ch) {
            fprintf(o, ",%.4f", (double)out.angle_deg[ch]);
        }
        /* 喂给 IK 的足端目标：x = P_[i] + ges_x[i]，y = foot_y[i] + ges_y[i] */
        for (int i = 0; i < CONTROL_CHAIN_LEGS; ++i) {
            fprintf(o, ",%.3f", (double)(out.trace.gait[i] + out.trace.ges[i]));
        }
        for (int i = 0; i < CONTROL_CHAIN_LEGS; ++i) {
            fprintf(o, ",%.3f",
                    (double)(out.trace.foot_y[i] + out.trace.ges[CONTROL_CHAIN_LEGS + i]));
        }
        fprintf(o, "\n");
        ++rows;
    }

    fclose(f);
    fclose(o);
    printf("wrote %s: %ld frames\n", out_csv, rows);
    return 0;
}
