/*
 * dump_action_trace.c —— 把**动作层**的逐帧角度导出，供动作可视化用
 *
 * ## 为什么要用宿主桩去驱动固件自己的 app_action，而不是另写一个状态机
 *
 * 动作的时序逻辑（哪一步、停多久、动画插值、ch_mask 合并）已经在
 * `firmware/src/app/app_action.c` 里，而且它已被 `test_app_action` 用
 * 78 条断言钉住。**如果我在这里再写一份等价的状态机来"重放"动作，
 * 那就是"同一份东西写两份"—— 本项目的 P-27 已经为此吃过一次亏**
 * （命令脚本解释器写两份，导出程序那份掉进 default 静默忽略新动作码，
 * 画出一张全错的图）。
 *
 * ⇒ 所以这里**只做搬运**：用 host_stubs 把真实的 `app_action` 跑起来，
 *    把它每帧的输出写进 CSV。图上的东西就是固件里的东西。
 *
 * 用 ESP-IDF 宿主桩（`tools/golden/host_stubs/`）提供可控时钟与假 I2C。
 *
 * 输出 CSV 列：`kind, frame, ms, ang0..ang11`
 *   kind: 1=stand 2=sit 3=sit_direct 4=wave 5=inplace_step
 *
 * ⚠️ 这里的角度是**舵机角**（0..180，逻辑通道 0..11），与步态那套的
 *    足端 x/y 不是同一类量。所以可视化里动作与步态要用不同的画法：
 *    步态的腿由足端目标精确解出；动作的腿只能按"舵机角相对中位的偏差"
 *    画成**示意图**（见 tools/viz/README.md 的说明）。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app/app_action.h"
#include "app/app_chain.h"
#include "app/app_config.h"
#include "control/action.h"
#include "host_stubs/host_sim.h"

/* 测试/导出用的配置桩：宿主上没有 NVS，直接给默认值 */
static app_config_t s_cfg;

int app_cfg_cmd_init(void)
{
    app_config_defaults(&s_cfg);
    return 0;   /* APP_CFG_OK */
}

const app_config_t *app_cfg_cmd_get(void)
{
    return &s_cfg;
}

/** 每个动作最多导出多少帧（10 ms/帧）；超过就当它卡住了 */
#define MAX_FRAMES 700
/** 动作结束后至少再保持这么多帧，让"最终姿态"在动画里看得见（stand 只写一帧就结束） */
#define MIN_FRAMES 40

static const struct {
    int         kind;
    const char *name;
} ACTIONS[] = {
    { APP_ACTION_STAND,        "stand" },
    { APP_ACTION_SIT,          "sit" },
    { APP_ACTION_SIT_DIRECT,   "sit_direct" },
    { APP_ACTION_WAVE,         "wave" },
    { APP_ACTION_INPLACE_STEP, "step" },
};
#define ACTION_COUNT ((int)(sizeof(ACTIONS) / sizeof(ACTIONS[0])))

int main(int argc, char **argv)
{
    const char *out_path = (argc > 1) ? argv[1] : "action_trace.csv";
    host_log_set_quiet(1);

    /*
     * ⚠️ 顺序很重要，而且这两个都必须调：
     *   - `app_cfg_cmd_init()` 把配置填成默认值。**漏了它**，
     *     `app_cfg_cmd_get()` 返回的就是一份**全零**配置 ⇒ 中位角全 0 ⇒
     *     导出的每一路都是 0（第一版就是这样：被 check_viewer.py 的
     *     "12 路最大变化 = 0.00°" 一眼看出来了）。
     *   - `app_chain_init()`：原地踏步那一路靠 `app_action_step()` 内部
     *     驱动 `app_chain_step()`，chain 没初始化就给不出角度。
     */
    app_cfg_cmd_init();
    if (app_chain_init() != 0) {
        fprintf(stderr, "app_chain_init failed\n");
        return 2;
    }
    if (app_action_init() != 0) {
        fprintf(stderr, "app_action_init failed\n");
        return 2;
    }

    FILE *o = fopen(out_path, "w");
    if (o == NULL) {
        fprintf(stderr, "cannot write %s\n", out_path);
        return 2;
    }

    printf("%-12s %8s %10s  %s\n", "action", "frames", "ms", "结束方式");

    for (int a = 0; a < ACTION_COUNT; ++a) {
        /* 每个动作都从干净状态开始：重新初始化（等价于原版重新上电的初值） */
        host_clock_reset();
        host_pca_reset();
        if (app_action_init() != 0) {
            fprintf(stderr, "re-init failed\n");
            fclose(o);
            return 2;
        }
        app_action_stop();
        if (app_action_request((app_action_kind_t)ACTIONS[a].kind) != 0) {
            fprintf(stderr, "request %s failed\n", ACTIONS[a].name);
            continue;
        }

        int64_t now_ms = 0;
        int     written = 0;
        int     f = 0;
        bool    have = false;
        bool    ended_naturally = false;
        float   cur[ACTION_CHANNELS];
        memset(cur, 0, sizeof(cur));

        /*
         * ⚠️ 结束判据用状态里的 `busy`，**不能**用"本帧没产出角度"。
         *    挥手各阶段之间有 `delay_ms` 等待，那些帧不产出角度，
         *    但动作还在跑 —— 第一版就是按"没产出"结束，结果 wave 只导了 910 ms，
         *    而它在固件里要跑 4310 ms（`test_app_action` 测到的值）。
         *
         * 另外：没产出的帧要**沿用上一个角度**继续写行，否则时间轴是断的，
         * 动画看起来会跳。这也正是真机上舵机的行为（没收到新脉宽就保持）。
         */
        for (; f < MAX_FRAMES; ++f) {
            float deg[ACTION_CHANNELS];
            const bool produced = app_action_step(now_ms, deg);
            if (produced) {
                memcpy(cur, deg, sizeof(cur));
                have = true;
            }

            app_action_status_t st;
            app_action_get_status(&st);
            const bool active = (st.busy || st.pending);

            if (!active && have && f >= MIN_FRAMES) {
                ended_naturally = true;
                break;
            }
            if (!have) {
                now_ms += 10;
                continue;      /* 还没有任何角度可画 */
            }

            fprintf(o, "%d,%d,%d,%d", ACTIONS[a].kind, f, (int)now_ms, active ? 1 : 0);
            for (int ch = 0; ch < ACTION_CHANNELS; ++ch) {
                fprintf(o, ",%.4f", (double)cur[ch]);
            }
            fprintf(o, "\n");
            ++written;
            now_ms += 10;
        }

        if (ACTIONS[a].kind == APP_ACTION_INPLACE_STEP) {
            app_action_stop();   /* 原地踏步会一直跑，导完就撤销 */
        }

        printf("%-12s %8d %10d  %s\n", ACTIONS[a].name, written, (int)now_ms,
               (ACTIONS[a].kind == APP_ACTION_INPLACE_STEP)
                   ? "帧数上限（原地踏步会持续跑，已撤销）"
                   : (ended_naturally ? "动作结束" : "达到帧数上限"));
        fflush(o);
    }

    fclose(o);
    printf("\n已写出 %s\n", out_path);
    return 0;
}
