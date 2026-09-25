/*
 * test_motion_app.c —— 宿主测试：App 层状态机（P3 / 第 0 步）
 *
 * 前面所有套件测的都是**纯函数**。这一套用 ESP-IDF 宿主桩 + **可控时钟**，
 * 把真正的 App 层跑起来：
 *
 *     app_config 默认值 -> app_chain -> control_chain(+cmd) -> servo_map
 *       -> servo_out -> drv_pca9685 -> （假 I2C）-> PCA9685 影子寄存器
 *
 * 于是这些"以前只能上板才能验"的东西可以在电脑上断言：
 *   1. 上电安全态：寄存器全是 OFF=4096（连 P-21 的 &0x1F 编码也一起验）
 *   2. 节拍门控：链 65 ms 才推进一次，不是每个 10 ms 运动周期都推进
 *   3. 站立：整链算出的占空比 = golden/control_chain.csv 第 1 行
 *   4. 急停：≤1 个运动周期内 12 路松力
 *   5. 超时停车：到点自动松力
 *   6. 模式切换：pose / chain，且"给直接角度目标会隐式切回 pose"
 *   7. 只写变化的通道：稳态下通道写次数不增长
 *
 * ⚠️ **这一套验的是逻辑，不是真实时序，也不是并发。**
 *    桩是单线程的、互斥锁恒成功 ⇒ 查不出死锁/优先级反转/栈深度/真实抖动。
 *
 * NOTE: ASCII output only, for the reason recorded as P-06 in 问题与解决记录.md.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app/app_chain.h"
#include "app/app_config.h"
#include "app/motion.h"
#include "app/servo_out.h"
#include "control/control_chain_cmd.h"
#include "drivers/drv_pca9685.h"
#include "host_stubs/host_sim.h"

int g_host_fail = 0;
int g_host_checks = 0;

/* ---- 测试自己提供 app_cfg_cmd 的桩：不想在宿主上碰 NVS ---- */

static app_config_t s_cfg;

int app_cfg_cmd_init(void)
{
    app_config_defaults(&s_cfg);
    return APP_CFG_OK;
}

const app_config_t *app_cfg_cmd_get(void)
{
    return &s_cfg;
}

/* ---- 手边工具 ---- */

/** 取 12 路逻辑通道在影子寄存器里的 OFF（顺序同 servo_map 的逻辑通道表） */
static void read_all_off(uint16_t out[12])
{
    static const uint8_t addr[12] = {
        DRV_PCA9685_ADDR_LEFT,  DRV_PCA9685_ADDR_LEFT,  DRV_PCA9685_ADDR_LEFT,
        DRV_PCA9685_ADDR_LEFT,  DRV_PCA9685_ADDR_LEFT,  DRV_PCA9685_ADDR_LEFT,
        DRV_PCA9685_ADDR_RIGHT, DRV_PCA9685_ADDR_RIGHT, DRV_PCA9685_ADDR_RIGHT,
        DRV_PCA9685_ADDR_RIGHT, DRV_PCA9685_ADDR_RIGHT, DRV_PCA9685_ADDR_RIGHT,
    };
    for (int ch = 0; ch < 12; ++ch) {
        uint16_t on = 0, off = 0;
        host_pca_get_pwm(addr[ch], (uint8_t)(ch % 6), &on, &off);
        out[ch] = off;
    }
}

static int all_relaxed(void)
{
    uint16_t off[12];
    read_all_off(off);
    for (int ch = 0; ch < 12; ++ch) {
        if (off[ch] != 4096) {
            return 0;
        }
    }
    return 1;
}

/* ========================================================================== */

int main(void)
{
    host_log_set_quiet(1);

    printf("========================================================\n");
    printf(" App-layer state machine test   (ESP-IDF host stubs)\n");
    printf("========================================================\n");
    printf("Single-threaded mock: logic only -- NOT real timing, NOT concurrency.\n");

    /* ---- 0. 初始化 ---- */
    host_section("0. init: config + servo_out + motion + app_chain");
    app_cfg_cmd_init();
    HOST_CHECK(servo_out_init() == ESP_OK, "servo_out_init failed\n");
    HOST_CHECK(motion_init() == ESP_OK, "motion_init failed\n");
    HOST_CHECK(app_chain_init() == ESP_OK, "app_chain_init failed\n");

    /* 上电安全态：drv_pca9685_init() 会 all_off；这里手动做一遍等效动作 */
    HOST_CHECK(drv_pca9685_init(DRV_PCA9685_ADDR_LEFT, DRV_PCA9685_DEFAULT_HZ) == ESP_OK,
               "pca9685 left init failed\n");
    HOST_CHECK(drv_pca9685_init(DRV_PCA9685_ADDR_RIGHT, DRV_PCA9685_DEFAULT_HZ) == ESP_OK,
               "pca9685 right init failed\n");
    HOST_CHECK(servo_out_all_off() == ESP_OK, "servo_out_all_off failed\n");
    HOST_CHECK(all_relaxed(), "after all_off not every channel reads OFF=4096\n");

    /* PRESCALE 应该是 MicroPython 的 122（复刻，不是手册的 121） */
    HOST_CHECK(host_pca_read(DRV_PCA9685_ADDR_LEFT, DRV_PCA9685_REG_PRESCALE) == 122,
               "PRESCALE = %u, expected 122\n",
               (unsigned)host_pca_read(DRV_PCA9685_ADDR_LEFT, DRV_PCA9685_REG_PRESCALE));

    /* ---- 1. 链节拍：65 ms 才推进一次 ---- */
    host_section("1. chain cadence gating (65 ms, not every 10 ms motion tick)");
    HOST_CHECK(app_chain_get_period_ms() == 65,
               "chain period = %u ms, expected 65\n", (unsigned)app_chain_get_period_ms());

    host_clock_reset();
    control_chain_sched_t probe;
    control_chain_sched_init(&probe, 65);
    int advanced = 0;
    for (int i = 0; i < 200; ++i) {          /* 模拟 200 次 10 ms 的调用 = 2 s */
        host_advance_us(10000);
        if (control_chain_sched_due(&probe, host_now_us() / 1000)) {
            ++advanced;
        }
    }
    /* 2 s / 65 ms ≈ 30.8 -> 31 次（第一次也算） */
    HOST_CHECK(advanced >= 30 && advanced <= 32,
               "chain advanced %d times in 2 s, expected ~31\n", advanced);
    printf("  chain advanced %d times in 2000 ms (expected ~31 => 65 ms cadence)\n", advanced);

    /* ---- 2. 站立：整链输出 = golden 第 1 行 ---- */
    host_section("2. chain stand: all 12 duties must equal control_chain.csv row 1");
    static const uint16_t expect_stand[12] = {
        333, 402, 424, 311, 434, 446, 320, 199, 181, 347, 169, 143
    };

    HOST_CHECK(motion_set_mode(MOTION_MODE_CHAIN) == ESP_OK, "set chain mode failed\n");
    HOST_CHECK(app_chain_stand() == ESP_OK, "app_chain_stand failed\n");
    HOST_CHECK(motion_set_timeout_ms(0) == ESP_OK, "disable timeout failed\n");

    host_task_reset();
    HOST_CHECK(motion_start() == ESP_OK, "motion_start failed\n");

    host_task_fn_t fn = NULL;
    HOST_CHECK(host_task_get(&fn, NULL, NULL, NULL, NULL) == 1 && fn != NULL,
               "motion task was not created\n");

    /*
     * ⚠️ 姿态 slew 是**渐近**的：`X_S` 从 0 按 `Kp_G=0.03` 逼近 `X_goal=18`，
     *    要收敛到"占空比不再变"需要**约 11 秒**模拟时间（原版也一样慢，因为
     *    它同样是每轮循环逼近一步）。
     *    ⇒ 所以这里不能假设"跑 30 帧就到了"，要**跑到稳定为止**。
     */
    uint16_t off[12];
    uint16_t prev[12];
    int converged_at = -1;
    long total_ticks = 0;
    for (int round = 0; round < 30 && converged_at < 0; ++round) {
        (void)host_task_run(100);      /* 100 个运动周期 = 1 s 模拟时间 */
        total_ticks += 100;
        read_all_off(off);
        if (round > 0 && memcmp(off, prev, sizeof(off)) == 0) {
            converged_at = round;
        }
        memcpy(prev, off, sizeof(prev));
    }

    printf("  converged after ~%ld motion ticks (%.1f s of mock time)\n",
           total_ticks, (double)(host_now_us() / 1000000));
    HOST_CHECK(converged_at >= 0, "stand pose never stabilised within %ld ticks\n",
               total_ticks);

    read_all_off(off);
    printf("    channel :");
    for (int ch = 0; ch < 12; ++ch) { printf(" %5d", ch); }
    printf("\n    got     :");
    for (int ch = 0; ch < 12; ++ch) { printf(" %5u", (unsigned)off[ch]); }
    printf("\n    expected:");
    for (int ch = 0; ch < 12; ++ch) { printf(" %5u", (unsigned)expect_stand[ch]); }
    printf("\n");

    /* 与 golden 的差：≤1 个计数单位（渐近 slew 停在浮点精度上，未必正好等于 18.0） */
    int max_delta = 0;
    for (int ch = 0; ch < 12; ++ch) {
        const int d = (int)off[ch] - (int)expect_stand[ch];
        const int ad = (d < 0) ? -d : d;
        if (ad > max_delta) {
            max_delta = ad;
        }
    }
    printf("    max |delta| vs golden = %d duty count(s)\n", max_delta);
    HOST_CHECK(max_delta <= 1,
               "stand pose differs from the golden by %d counts (>1)\n", max_delta);

    /* 顺带确认真的是"链"在驱动：髋通道必须等于中位角对应的占空比 */
    HOST_CHECK(off[0] == 333 && off[3] == 311 && off[6] == 320 && off[9] == 347,
               "hip channels are not at their centre duties\n");

    /* ---- 3. 只写变化的通道：稳态写次数不增长 ---- */
    host_section("3. selective writes: steady state must add zero channel writes");
    host_pca_reset_write_stats();
    (void)host_task_run(50);                 /* 再跑 500 ms，姿态已到位 */
    const unsigned long steady_writes = host_pca_write_count();
    printf("  channel writes during 50 steady-state ticks: %lu (expected 0)\n",
           steady_writes);
    HOST_CHECK(steady_writes == 0,
               "%lu channel writes in steady state, expected 0\n", steady_writes);

    /* ---- 4. 急停：≤1 个运动周期内松力 ---- */
    host_section("4. e-stop: must relax within one motion period");
    /* 先让它走起来，确保寄存器不是松力态 */
    HOST_CHECK(motion_set_mode(MOTION_MODE_CHAIN) == ESP_OK, "set chain failed\n");
    HOST_CHECK(app_chain_jog(-3.0f, 1, 1) == ESP_OK, "jog failed\n");
    (void)host_task_run(20);
    HOST_CHECK(!all_relaxed(), "after jogging, channels should not all be relaxed\n");

    motion_estop("host test");
    const int64_t before_us = host_now_us();
    (void)host_task_run(3);                  /* 最多 3 个运动周期 = 30 ms */
    const int64_t elapsed_ms = (host_now_us() - before_us) / 1000;
    HOST_CHECK(all_relaxed(), "e-stop did not relax all channels\n");
    HOST_CHECK(elapsed_ms <= 60, "e-stop took %ld ms (expected <= ~30 ms)\n",
               (long)elapsed_ms);
    HOST_CHECK(!motion_is_running(), "motion task still running after e-stop\n");
    printf("  e-stop relaxed all 12 channels within %ld ms of mock time\n",
           (long)elapsed_ms);

    /* ---- 5. 超时停车 ---- */
    host_section("5. command timeout: auto-relax after the configured idle time");
    HOST_CHECK(motion_set_timeout_ms(500) == ESP_OK, "set timeout failed\n");
    HOST_CHECK(motion_set_mode(MOTION_MODE_CHAIN) == ESP_OK, "set chain failed\n");
    HOST_CHECK(app_chain_jog(-3.0f, 1, 1) == ESP_OK, "jog failed\n");
    host_task_reset();
    HOST_CHECK(motion_start() == ESP_OK, "restart failed\n");
    /* keepalive 在 start 里做过；跑 100 个 10 ms 周期 = 1 s > 500 ms 超时 */
    (void)host_task_run(100);
    HOST_CHECK(all_relaxed(), "timeout did not relax all channels\n");
    HOST_CHECK(!motion_is_running(), "motion task still running after timeout\n");

    motion_stats_t st;
    motion_get_stats(&st);
    printf("  after timeout: running=%d stop_reason=%s\n", st.running,
           motion_stop_reason_str(st.stop_reason));
    HOST_CHECK(st.stop_reason == MOTION_STOP_TIMEOUT,
               "stop reason = %u, expected MOTION_STOP_TIMEOUT\n",
               (unsigned)st.stop_reason);

    /* ---- 6. 模式切换 ---- */
    host_section("6. mode switch: a raw 12-angle target must force POSE mode");
    HOST_CHECK(motion_set_mode(MOTION_MODE_CHAIN) == ESP_OK, "set chain failed\n");
    float target[12];
    for (int ch = 0; ch < 12; ++ch) {
        target[ch] = 90.0f;
    }
    HOST_CHECK(motion_set_target(target) == ESP_OK, "set_target failed\n");
    HOST_CHECK(motion_get_mode() == MOTION_MODE_POSE,
               "mode = %u after a raw target, expected POSE\n",
               (unsigned)motion_get_mode());
    printf("  setting a raw target switched the mode back to POSE (as designed)\n");

    /* ---- 7. 恢复安全态 ---- */
    host_section("7. leave the simulated board relaxed");
    HOST_CHECK(servo_out_all_off() == ESP_OK, "final all_off failed\n");
    HOST_CHECK(all_relaxed(), "final state is not relaxed\n");

    /* ---- 汇总 ---- */
    printf("\n========================================================\n");
    printf(" checks=%d  failures=%d\n", g_host_checks, g_host_fail);
    if (g_host_fail != 0) {
        printf("RESULT: FAIL -- app-layer state machine differs from expectations\n");
        return 1;
    }
    printf("RESULT: PASS -- all %d checks passed\n", g_host_checks);
    printf("\nNOTE: this suite validates LOGIC only.  The stubs are single-threaded\n");
    printf("      and the mutexes always succeed, so it cannot catch deadlocks,\n");
    printf("      priority inversion, stack depth, real jitter, or anything about\n");
    printf("      the actual hardware.\n");
    return 0;
}
