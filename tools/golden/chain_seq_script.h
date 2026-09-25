/**
 * @file    chain_seq_script.h
 * @brief   多帧命令脚本的**唯一**解析与派发实现（header-only，供多个测试共用）
 *
 * ## 为什么要有这个文件
 *
 * 命令脚本原本在**两个地方各写了一份**解析 + 派发代码：
 *   - `tools/golden/test_control_chain_cmd.c`（比对用）
 *   - `tools/viz/dump_chain_trace.c`（导出轨迹给可视化用）
 *
 * 结果就是漂移：给脚本新加 `drive` 动作时，只改了测试那一份，
 * dump 那一份把动作码 5 掉进 `default: break` 静默忽略 ⇒ 速度一直是 0 ⇒
 * 画出来 WALK 全程不动。**这是同一类错误在本项目的第二次**（第一次是
 * `cs` 那一列，成长手册 P-22）：**同一份东西写两份，就会有一份是错的。**
 *
 * ⇒ 现在两边都 `#include` 这个头，动作码表、CSV 列布局、派发逻辑都只有一份。
 * 做成 header-only（`static` 函数）是为了不再往两个 build 里各加一个 .c。
 *
 * 脚本 CSV 格式（由 `gen_golden.py` 写出）：
 *     seq, frame, action, a0, a1, a2
 */
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "control/control_chain.h"
#include "control/control_chain_cmd.h"

/** 动作编号 —— 必须与 `gen_golden.py` 的 `CHAIN_SEQ_ACTION_CODES` 一致 */
enum {
    CHAIN_SEQ_ACT_MOVE     = 0,
    CHAIN_SEQ_ACT_GAIT     = 1,
    CHAIN_SEQ_ACT_HEIGHT   = 2,
    CHAIN_SEQ_ACT_GESTURE  = 3,
    CHAIN_SEQ_ACT_SET_TURN = 4,
    /* P-26：`drive()` = `move()` 去掉 `gait(0)`，是进入 WALK 的唯一途径 */
    CHAIN_SEQ_ACT_DRIVE    = 5,
    CHAIN_SEQ_ACT_COUNT    = 6,
};

typedef struct {
    int    seq;
    int    frame;
    int    action;
    double a[3];
} chain_seq_cmd_t;

#define CHAIN_SEQ_CMD_MAX 1024
#define CHAIN_SEQ_LINE_MAX 4096

typedef struct {
    chain_seq_cmd_t cmd[CHAIN_SEQ_CMD_MAX];
    int             count;
    int             unknown_actions;   /**< 遇到未知动作码的次数（必须为 0） */
} chain_seq_script_t;

/** 读命令 CSV。返回 0 成功。 */
static int chain_seq_script_load(chain_seq_script_t *s, const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        return -1;
    }
    memset(s, 0, sizeof(*s));
    char line[CHAIN_SEQ_LINE_MAX];
    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        if (s->count >= CHAIN_SEQ_CMD_MAX) {
            fprintf(stderr, "ERROR: too many commands (max %d)\n", CHAIN_SEQ_CMD_MAX);
            fclose(f);
            return -1;
        }
        chain_seq_cmd_t *c = &s->cmd[s->count];
        if (sscanf(line, "%d,%d,%d,%lf,%lf,%lf",
                   &c->seq, &c->frame, &c->action,
                   &c->a[0], &c->a[1], &c->a[2]) != 6) {
            fprintf(stderr, "ERROR: bad command row: %s", line);
            fclose(f);
            return -1;
        }
        ++s->count;
    }
    fclose(f);
    return 0;
}

/** 施加挂在 (seq, frame) 上的所有命令 */
static void chain_seq_script_apply(const chain_seq_script_t *s, int seq, int frame,
                                   control_chain_cmd_t *cmd,
                                   const control_chain_cfg_t *cfg,
                                   control_chain_state_t *st)
{
    for (int i = 0; i < s->count; ++i) {
        const chain_seq_cmd_t *c = &s->cmd[i];
        if (c->seq != seq || c->frame != frame) {
            continue;
        }
        switch (c->action) {
        case CHAIN_SEQ_ACT_MOVE:
            control_chain_cmd_move(cmd, cfg, st, (float)c->a[0], (int)c->a[1], (int)c->a[2]);
            break;
        case CHAIN_SEQ_ACT_DRIVE:
            control_chain_cmd_drive(cmd, cfg, st, (float)c->a[0], (int)c->a[1], (int)c->a[2]);
            break;
        case CHAIN_SEQ_ACT_GAIT:
            control_chain_cmd_gait(cmd, cfg, st, (int)c->a[0]);
            break;
        case CHAIN_SEQ_ACT_HEIGHT:
            control_chain_cmd_height(cmd, st, (float)c->a[0]);
            break;
        case CHAIN_SEQ_ACT_GESTURE:
            control_chain_cmd_gesture(cmd, (float)c->a[0], (float)c->a[1], (float)c->a[2]);
            break;
        case CHAIN_SEQ_ACT_SET_TURN:
            control_chain_cmd_set_turn(cmd, (float)c->a[0]);
            break;
        default:
            /* 不再静默忽略：未知动作码会让调用方看到计数不为 0 */
            ((chain_seq_script_t *)(uintptr_t)s)->unknown_actions += 1;
            fprintf(stderr, "ERROR: unknown action code %d (seq %d frame %d)\n",
                    c->action, c->seq, c->frame);
            break;
        }
    }
}
