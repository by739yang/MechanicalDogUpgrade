/*
 * test_app_action.c -- host test: the ACTION layer reached from the firmware (P3 / step 2)
 *
 * WHAT THIS SUITE PROVES
 *   The already-ported, already-verified action layer (control/action.c, compared against
 *   the REAL padog.py with tolerance 0 by test_action.c) is now REACHABLE from the firmware
 *   and its angles/side effects survive the whole trip:
 *
 *     console command -> app_action -> control/action -> app_chain -> control_chain_cmd
 *       -> motion task (ACTION mode) -> servo_out -> drv_pca9685 -> (fake I2C)
 *       -> PCA9685 shadow registers
 *
 *   Every command in this file is issued through app_motion_cmd_handle(), i.e. the very same
 *   dispatcher the firmware's UART console calls -- so "reachable from the firmware" is what
 *   is actually being tested, not just "the functions link".
 *
 * WHY IT IS LEGITIMATE TO CALL action_* HERE (and why this is not P-23)
 *   This suite does NOT re-derive the reference values from my own understanding.  The
 *   authority for "what a sit pose is" is the action layer itself, which test_action.c pins
 *   against the real padog.py at tolerance 0.  Here the action layer is used as the ORACLE
 *   and the thing under test is the WIRING: does the angle the verified layer computes
 *   survive app_action -> motion -> servo_out -> the shadow registers byte for byte.
 *   A wrong copy of the arithmetic in this file can only produce a loud FAIL, never a silent
 *   PASS, because the compared bytes come from the real code path.  The one place where a
 *   formula is repeated (the wave ARM step, 4 lines) is marked, and test_action.c remains the
 *   authority for those values.
 *
 * WHAT IT CHECKS
 *   1. app_action's cfg really is built from app_config (12 centre angles, in_pit/in_rol/in_y)
 *      and keeps the action layer's own defaults for everything else (field-by-field).
 *   2. `action stand` (direct write path) -> registers == action_apply_stand_angles_direct().
 *   3. `action sit`  (clock-driven pose animation) -> converges to the sit pose the action
 *      layer computes, AND the effects really reached the chain (move/gait/height/gesture/
 *      sit_offsets/servo_init/crawl_reset), each with a non-default starting value so that
 *      "did the effect arrive" is a real question (P-18: symmetric inputs test nothing).
 *   4. `action stand` again -> animation back to the stand pose, passing through a midpoint
 *      that is neither endpoint (i.e. it really interpolates on the clock).
 *   5. `action wave` -> the stepper's ch_mask merge (only 1,2,7,8 are rewritten; the other
 *      eight channels keep their previous value), and the whole script runs to completion in
 *      bounded simulated time -- the bound is DERIVED from the cfg's own delays, so dropping
 *      the stepper pacing (calling it every frame) makes the suite FAIL.
 *   6. `action step` (the original web "gait test") -> the in-place service reaches the chain
 *      command state and the chain really advances; `estop` in the middle of it leaves all 12
 *      channels relaxed.
 *   7. `motion stop` in the middle of an animation cancels the action and relaxes.
 *
 * LIMITS (same as test_motion_app): the stubs are single-threaded and the mutexes always
 * succeed, so this validates LOGIC only -- not real timing, not concurrency, not the board.
 *
 * NOTE: ASCII output only, for the reason recorded as P-06 in 问题与解决记录.md.
 */
#include <stdio.h>
#include <string.h>

#include "app/app_action.h"
#include "app/app_chain.h"
#include "app/app_config.h"
#include "app/app_motion_cmd.h"
#include "app/motion.h"
#include "app/servo_out.h"
#include "control/action.h"
#include "control/control_chain_cmd.h"
#include "control/servo_map.h"
#include "drivers/drv_pca9685.h"
#include "host_stubs/host_sim.h"

int g_host_fail = 0;
int g_host_checks = 0;

/* ==========================================================================
 * app_cfg_cmd stub: the host must not touch NVS
 * ========================================================================== */

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

/* ==========================================================================
 * helpers
 * ========================================================================== */

static uint8_t ch_board(int ch)
{
    return (ch < 6) ? DRV_PCA9685_ADDR_LEFT : DRV_PCA9685_ADDR_RIGHT;
}

static uint8_t ch_pca(int ch)
{
    return (uint8_t)(ch % 6);
}

/** the 12 logical channels' OFF registers, in logical-channel order (servo_map.h) */
static void read_all_off(uint16_t out[12])
{
    for (int ch = 0; ch < 12; ++ch) {
        uint16_t on = 0, off = 0;
        host_pca_get_pwm(ch_board(ch), ch_pca(ch), &on, &off);
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

/** angles -> expected OFF counts, through the verified servo mapping */
static void duties_of(const float deg[12], uint16_t duty[12])
{
    for (int ch = 0; ch < 12; ++ch) {
        duty[ch] = servo_map_deg_to_duty(deg[ch]);
    }
}

static int max_delta(const uint16_t got[12], const uint16_t want[12])
{
    int worst = 0;
    for (int ch = 0; ch < 12; ++ch) {
        int d = (int)got[ch] - (int)want[ch];
        if (d < 0) {
            d = -d;
        }
        if (d > worst) {
            worst = d;
        }
    }
    return worst;
}

static void print_row(const char *label, const uint16_t v[12])
{
    printf("    %-9s:", label);
    for (int ch = 0; ch < 12; ++ch) {
        printf(" %5u", (unsigned)v[ch]);
    }
    printf("\n");
}

static void print_angle_row(const char *label, const float v[12])
{
    printf("    %-9s:", label);
    for (int ch = 0; ch < 12; ++ch) {
        printf(" %5.1f", (double)v[ch]);
    }
    printf("\n");
}

/**
 * A SECOND, independent construction of the cfg the app layer is supposed to build:
 * action_cfg_defaults() (the action layer's own defaults) + the fields app_config owns.
 * It is compared field-by-field against app_action_get_cfg() in section 1 and then used to
 * ask the verified action layer for the expected poses.
 */
static void spec_cfg(action_cfg_t *c)
{
    action_cfg_defaults(c);
    for (int leg = 0; leg < ACTION_LEGS; ++leg) {
        for (int j = 0; j < ACTION_JOINTS; ++j) {
            c->init[leg][j] = s_cfg.servo_center[leg][j];
        }
    }
    c->in_pit = s_cfg.in_pit;
    c->in_rol = s_cfg.in_rol;
    c->in_y   = s_cfg.in_y;
}

/** expected angles of the "stand -> sit" animation at t = 1 (what _apply_sit_angles_direct does) */
static void expect_sit(float deg[12])
{
    action_cfg_t c;
    float from[12], to[12];
    spec_cfg(&c);
    action_stand_pose(&c, from);
    action_sit_pose(&c, to);
    action_apply_pose_blend(from, to, 1.0f, deg);
}

/** expected angles of the "sit -> stand" animation at t = 1 (action_stand()'s animated branch) */
static void expect_stand_from_sit(float deg[12])
{
    action_cfg_t c;
    float from[12], to[12];
    spec_cfg(&c);
    action_sit_pose(&c, from);
    action_stand_pose(&c, to);
    action_apply_pose_blend(from, to, 1.0f, deg);
}

/** expected angles of the direct stand write (hips clamped, legs not -- padog.py:667) */
static void expect_stand_direct(float deg[12])
{
    action_cfg_t c;
    spec_cfg(&c);
    action_apply_stand_angles_direct(&c, deg);
}

/**
 * Expected output of the wave's ARM step: the four named channels get their new values,
 * everything else keeps `prev` (the sit pose) -- i.e. exactly what the ch_mask merge must do.
 *
 * NOTE: these 4 lines repeat padog.py:838~841.  The authoritative check of those formulas is
 * test_action.c (tolerance 0 against the real padog.py); here they only need to be "the values
 * the verified layer produced", and a mistake on my side shows up as a FAIL, never a PASS.
 */
static void expect_wave_arm(const float prev[12], float deg[12])
{
    const action_cfg_t *c = app_action_get_cfg();
    memcpy(deg, prev, sizeof(float) * 12);
    deg[1] = action_clamp_deg(c->init[0][1] - c->wave_arm_delta);
    deg[2] = action_clamp_deg(c->init[0][2] - c->wave_arm_delta);
    deg[7] = action_clamp_deg(c->init[1][1] + c->wave_arm_delta);
    deg[8] = action_clamp_deg(c->init[1][2] + c->wave_arm_delta);
}

/** drive the real motion task one tick at a time until the action layer reports "not busy" */
static int run_until_idle(int max_ticks)
{
    app_action_status_t as;
    for (int i = 0; i < max_ticks; ++i) {
        app_action_get_status(&as);
        if (!as.busy) {
            return i;
        }
        (void)host_task_run(1);
    }
    return -1;
}

/** count the cfg fields where the app layer's cfg differs from the spec above */
static int cmp_cfg(const action_cfg_t *a, const action_cfg_t *b)
{
    int diff = 0;

    for (int leg = 0; leg < ACTION_LEGS; ++leg) {
        for (int j = 0; j < ACTION_JOINTS; ++j) {
            if (a->init[leg][j] != b->init[leg][j]) {
                printf("  cfg diff init[%d][%d]: app=%.4f spec=%.4f\n",
                       leg, j, (double)a->init[leg][j], (double)b->init[leg][j]);
                ++diff;
            }
        }
    }
    for (int i = 0; i < ACTION_CHANNELS; ++i) {
        if (a->sit_delta[i] != b->sit_delta[i]) {
            printf("  cfg diff sit_delta[%d]: app=%.4f spec=%.4f\n",
                   i, (double)a->sit_delta[i], (double)b->sit_delta[i]);
            ++diff;
        }
    }

#define CFG_SCALAR(field)                                                      \
    do {                                                                       \
        if (a->field != b->field) {                                            \
            printf("  cfg diff " #field ": app=%.4f spec=%.4f\n",              \
                   (double)a->field, (double)b->field);                        \
            ++diff;                                                            \
        }                                                                      \
    } while (0)

    CFG_SCALAR(pose_blend_ms);
    CFG_SCALAR(sit_height);
    CFG_SCALAR(in_pit);
    CFG_SCALAR(in_rol);
    CFG_SCALAR(in_y);
    CFG_SCALAR(inplace_spd);
    CFG_SCALAR(inplace_L);
    CFG_SCALAR(inplace_R);
    CFG_SCALAR(anim_wait_step_ms);
    CFG_SCALAR(wave_arm_delta);
    CFG_SCALAR(wave_lift_h_delta);
    CFG_SCALAR(wave_lift_s_delta);
    CFG_SCALAR(wave_swing_up);
    CFG_SCALAR(wave_swing_down);
    CFG_SCALAR(wave_repeat);
    CFG_SCALAR(wave_arm_sleep_ms);
    CFG_SCALAR(wave_lift_sleep_ms);
    CFG_SCALAR(wave_swing_sleep_ms);
    CFG_SCALAR(wave_final_sleep_ms);
#undef CFG_SCALAR

    return diff;
}

/* ========================================================================== */

int main(void)
{
    host_log_set_quiet(1);

    printf("========================================================\n");
    printf(" ACTION layer wired into the firmware   (ESP-IDF host stubs)\n");
    printf("========================================================\n");
    printf("Commands go through app_motion_cmd_handle() == the UART console path.\n");
    printf("Single-threaded mock: logic only -- NOT real timing, NOT concurrency.\n");

    /* ---- 0. init ---- */
    host_section("0. init: config stub -> drv_pca9685 -> app_motion_cmd_init()");
    HOST_CHECK(app_cfg_cmd_init() == APP_CFG_OK, "app_cfg_cmd_init failed\n");

    /*
     * Make every claim in this suite falsifiable (P-18: symmetric/default inputs test nothing):
     *   - three centre angles differ from action_cfg_defaults()  -> "did app_action copy them?"
     *   - in_y / in_pit / in_rol differ from the action defaults -> "did the gesture effect
     *     really arrive, and did it arrive AFTER gait(0)?"
     */
    s_cfg.servo_center[0][1] = 70.0f;    /* default 84 */
    s_cfg.servo_center[2][2] = 100.0f;   /* default 68 */
    s_cfg.servo_center[3][0] = 95.0f;    /* default 92 */
    s_cfg.in_y   = 27.0f;                /* default 18 */
    s_cfg.in_pit = 12.0f;                /* default 0  */
    s_cfg.in_rol = -4.0f;                /* default 0  */

    HOST_CHECK(drv_pca9685_init(DRV_PCA9685_ADDR_LEFT, DRV_PCA9685_DEFAULT_HZ) == ESP_OK,
               "pca9685 left init failed\n");
    HOST_CHECK(drv_pca9685_init(DRV_PCA9685_ADDR_RIGHT, DRV_PCA9685_DEFAULT_HZ) == ESP_OK,
               "pca9685 right init failed\n");
    HOST_CHECK(host_pca_read(DRV_PCA9685_ADDR_LEFT, DRV_PCA9685_REG_PRESCALE) == 122,
               "PRESCALE = %u, expected 122\n",
               (unsigned)host_pca_read(DRV_PCA9685_ADDR_LEFT, DRV_PCA9685_REG_PRESCALE));

    app_motion_cmd_init();   /* servo_out + motion + app_chain + app_action */
    HOST_CHECK(app_action_get_cfg() != NULL,
               "app_motion_cmd_init() did not initialise the action layer\n");
    HOST_CHECK(app_chain_get_period_ms() == 65,
               "chain period = %u ms, expected 65\n", (unsigned)app_chain_get_period_ms());
    HOST_CHECK(all_relaxed(), "power-on state is not relaxed\n");

    /* ---- 1. the cfg the app layer built ---- */
    host_section("1. app_action cfg = action_cfg_defaults() + what app_config owns");
    {
        const action_cfg_t *app_cfg = app_action_get_cfg();
        action_cfg_t spec;
        spec_cfg(&spec);
        HOST_CHECK(app_cfg != NULL, "no action cfg\n");
        const int diffs = cmp_cfg(app_cfg, &spec);
        printf("  cfg fields differing from the app_config-derived spec: %d\n", diffs);
        HOST_CHECK(diffs == 0, "%d cfg field(s) differ from the app_config-derived spec\n",
                   diffs);
        /* the action layer's own defaults must NOT have been overwritten by anything else */
        HOST_CHECK(app_cfg->pose_blend_ms == 900,
                   "pose_blend_ms = %d, expected the action layer default 900\n",
                   (int)app_cfg->pose_blend_ms);
        HOST_CHECK(app_cfg->init[0][1] == 70.0f && app_cfg->init[2][2] == 100.0f,
                   "centre angles were not taken from app_config\n");
    }

    /* ---- 2. ACTION mode + non-default chain state on purpose ---- */
    host_section("2. ACTION mode: motion task up, chain state made non-default first");
    app_motion_cmd_handle("motion", "timeout 0");
    app_motion_cmd_handle("motion", "mode action");
    HOST_CHECK(motion_get_mode() == MOTION_MODE_ACTION,
               "mode = %u, expected MOTION_MODE_ACTION\n", (unsigned)motion_get_mode());

    host_task_reset();
    app_motion_cmd_handle("motion", "start");
    HOST_CHECK(motion_is_running(), "motion task is not running\n");
    {
        host_task_fn_t fn = NULL;
        HOST_CHECK(host_task_get(&fn, NULL, NULL, NULL, NULL) == 1 && fn != NULL,
                   "motion task was not created\n");
    }

    /* Every effect this suite checks is a no-op on the defaults, so set them off-default. */
    HOST_CHECK(app_chain_set_gait(1) == ESP_OK, "set gait walk failed\n");
    HOST_CHECK(app_chain_set_sit_offsets(5.0f, 7.0f) == ESP_OK, "set sit offsets failed\n");
    HOST_CHECK(app_chain_set_init_case(1) == ESP_OK, "set init_case failed\n");
    HOST_CHECK(app_chain_set_crawl(2, 12345, 6789) == ESP_OK, "set crawl failed\n");

    /* ---- 3. action stand (direct write branch) ---- */
    host_section("3. `action stand` -> direct 12-channel write of the action layer's pose");
    {
        float want[12];
        uint16_t wd[12], off[12];
        expect_stand_direct(want);
        duties_of(want, wd);

        app_motion_cmd_handle("action", "stand");
        (void)host_task_run(2);
        read_all_off(off);

        print_angle_row("expected", want);
        print_row("want duty", wd);
        print_row("got duty", off);
        const int delta = max_delta(off, wd);
        printf("    max |delta| = %d duty count(s)\n", delta);
        HOST_CHECK(delta == 0,
                   "direct stand write differs from the action layer by %d duty counts\n", delta);

        /* the effects of that call must have gone through the command layer */
        app_chain_status_t cs;
        app_chain_get_status(&cs);
        HOST_CHECK(cs.gait_mode == 0, "gait_mode = %d, expected 0 (gait(0) effect)\n",
                   cs.gait_mode);
        HOST_CHECK(cs.spd == 0.0f && cs.L == 0 && cs.R == 0,
                   "move(0,0,0) effect did not reach the chain: spd=%.3f L=%d R=%d\n",
                   (double)cs.spd, cs.L, cs.R);
        HOST_CHECK(cs.goal[0] == 81.0f,
                   "height(int(H_goal)) effect: H_goal = %.3f, expected 81\n",
                   (double)cs.goal[0]);
        HOST_CHECK(cs.goal[1] == 0.0f && cs.goal[2] == 0.0f,
                   "gesture(0,0,in_y) must come AFTER gait(0): PIT=%.3f ROL=%.3f (expected 0/0)\n",
                   (double)cs.goal[1], (double)cs.goal[2]);
        HOST_CHECK(cs.goal[3] == 27.0f,
                   "gesture() X goal = %.3f, expected int(in_y) = 27\n", (double)cs.goal[3]);

        float f = -1.0f, r = -1.0f;
        app_chain_get_sit_offsets(&f, &r);
        HOST_CHECK(f == 0.0f && r == 0.0f,
                   "SIT_OFFSETS effect did not reach the chain cfg: front=%.3f rear=%.3f\n",
                   (double)f, (double)r);
        /* `action_stand()`'s DIRECT branch emits move(0,0,0), and the original's `move()`
         * only calls `servo_init(0)` when `(L+R) != 0 and abs(spd) > 0` -- so init_case must
         * still be the value planted above.  The SERVO_INIT effect is exercised in section 7. */
        HOST_CHECK(app_chain_get_init_case() == 1,
                   "init_case = %d: move(0,0,0) must NOT call servo_init(0)\n",
                   app_chain_get_init_case());
        int cp = -1, cu = -1, csu = -1;
        app_chain_get_crawl(&cp, &cu, &csu);
        HOST_CHECK(cp == 0 && cu == 0 && csu == 0,
                   "CRAWL_RESET effect did not reach the chain state: %d/%d/%d\n", cp, cu, csu);
    }

    /* ---- 4. action sit (clock-driven pose animation) ---- */
    host_section("4. `action sit` -> animation converges to the sit pose (and effects land)");
    {
        float want[12];
        uint16_t wd[12], off[12];
        expect_sit(want);
        duties_of(want, wd);

        const int64_t t0 = host_now_us();
        app_motion_cmd_handle("action", "sit");
        const int ticks = run_until_idle(400);
        const long elapsed_ms = (long)((host_now_us() - t0) / 1000);

        app_action_status_t as;
        app_action_get_status(&as);
        printf("  sit settled after %d motion ticks (%ld ms of mock time)\n", ticks, elapsed_ms);
        HOST_CHECK(ticks >= 0, "the sit animation never became idle within 400 ticks\n");
        HOST_CHECK(elapsed_ms >= (long)app_action_get_cfg()->pose_blend_ms,
                   "the sit animation took only %ld ms, expected >= pose_blend_ms = %d "
                   "(a direct write instead of an animation?)\n",
                   elapsed_ms, (int)app_action_get_cfg()->pose_blend_ms);
        HOST_CHECK(as.pose_anim_active == false && as.wave_running == false,
                   "sit left the action layer busy (anim=%d wave=%d)\n",
                   (int)as.pose_anim_active, (int)as.wave_running);
        HOST_CHECK(as.direct_pose_freeze == true,
                   "sit must leave the pose frozen (hold_freeze=True)\n");

        read_all_off(off);
        print_angle_row("expected", want);
        print_row("want duty", wd);
        print_row("got duty", off);
        const int delta = max_delta(off, wd);
        printf("    max |delta| = %d duty count(s)\n", delta);
        HOST_CHECK(delta == 0, "sit pose differs from the action layer by %d duty counts\n",
                   delta);

        app_chain_status_t cs;
        app_chain_get_status(&cs);
        HOST_CHECK(cs.goal[0] == 86.0f,
                   "height(86) effect: H_goal = %.3f, expected the sit height 86\n",
                   (double)cs.goal[0]);
        HOST_CHECK(cs.goal[3] == 27.0f, "X goal = %.3f, expected 27\n", (double)cs.goal[3]);
        /*
         * ⚠️ ORDER, seen from the other side.  `action_sit_direct()` emits
         *     ... gait(0) ... gesture(0,0,in_y) ... [then _pose_anim_begin emits gait(0) AGAIN]
         * so the LAST thing that touches the pitch/roll goals is that trailing `gait(0)`,
         * which resets them to int(in_pit)/int(in_rol).  A wiring that reordered or merged the
         * effects (the thing control/action.h explicitly forbids) would leave 0/0 here.
         * Section 3's direct-stand branch is the mirror image: no trailing gait(0), so there the
         * gesture must win and the goals must be 0/0.
         */
        {
            const float want_pit = (float)(int)s_cfg.in_pit;   /* int() truncates, like gait() */
            const float want_rol = (float)(int)s_cfg.in_rol;
            HOST_CHECK(cs.goal[1] == want_pit && cs.goal[2] == want_rol,
                       "PIT/ROL goal = %.3f/%.3f, expected %.3f/%.3f (the trailing gait(0) "
                       "inside _pose_anim_begin runs LAST)\n",
                       (double)cs.goal[1], (double)cs.goal[2],
                       (double)want_pit, (double)want_rol);
        }
        HOST_CHECK(cs.gait_mode == 0 && cs.spd == 0.0f && cs.L == 0 && cs.R == 0,
                   "sit's move/gait effects are wrong: gait=%d spd=%.3f L=%d R=%d\n",
                   cs.gait_mode, (double)cs.spd, cs.L, cs.R);
        HOST_CHECK(app_chain_get_init_case() == 1 && cs.R_H == 86.0f,
                   "init_case=%d R_H=%.3f, expected 1 (move(0,0,0) keeps it) and 86 "
                   "(height() syncs R_H)\n",
                   app_chain_get_init_case(), (double)cs.R_H);
    }

    /* ---- 5. action stand again: animation back, through a real midpoint ---- */
    host_section("5. `action stand` from the frozen sit -> animation back to the stand pose");
    {
        float want[12], want_sit[12];
        uint16_t wd[12], off[12], sit_d[12], mid[12];
        expect_stand_from_sit(want);
        expect_sit(want_sit);
        duties_of(want, wd);
        duties_of(want_sit, sit_d);

        const int64_t t0 = host_now_us();
        app_motion_cmd_handle("action", "stand");
        (void)host_task_run(45);            /* ~450 ms = half of the 900 ms blend */
        read_all_off(mid);

        const int ticks = run_until_idle(400);
        const long elapsed_ms = (long)((host_now_us() - t0) / 1000);

        app_action_status_t as;
        app_action_get_status(&as);
        printf("  stand settled after %d more ticks (total %ld ms of mock time)\n",
               ticks, elapsed_ms);
        HOST_CHECK(ticks >= 0, "the stand animation never became idle within 400 ticks\n");
        HOST_CHECK(elapsed_ms >= (long)app_action_get_cfg()->pose_blend_ms,
                   "the stand animation took only %ld ms, expected >= pose_blend_ms\n",
                   elapsed_ms);
        HOST_CHECK(as.direct_pose_freeze == false,
                   "the stand animation must release the freeze (hold_freeze=False)\n");

        read_all_off(off);
        print_angle_row("expected", want);
        print_row("want duty", wd);
        print_row("got duty", off);
        print_row("mid duty", mid);
        const int delta = max_delta(off, wd);
        printf("    max |delta| = %d duty count(s)\n", delta);
        HOST_CHECK(delta == 0, "final stand pose differs from the action layer by %d counts\n",
                   delta);

        /* the midpoint must be neither endpoint -> the blend really is clock-driven */
        HOST_CHECK(memcmp(mid, sit_d, sizeof(sit_d)) != 0 &&
                   memcmp(mid, wd, sizeof(wd)) != 0,
                   "the animation jumped straight to an endpoint (not interpolating)\n");

        app_chain_status_t cs;
        app_chain_get_status(&cs);
        HOST_CHECK(cs.goal[0] == 86.0f,
                   "stand must read the CURRENT H_goal (86 after sit), got %.3f\n",
                   (double)cs.goal[0]);
        HOST_CHECK(cs.goal[1] == (float)(int)s_cfg.in_pit &&
                   cs.goal[2] == (float)(int)s_cfg.in_rol,
                   "PIT/ROL goal = %.3f/%.3f, expected %.3f/%.3f (trailing gait(0) last)\n",
                   (double)cs.goal[1], (double)cs.goal[2],
                   (double)(float)(int)s_cfg.in_pit, (double)(float)(int)s_cfg.in_rol);
    }

    /* ---- 6. action wave: ch_mask merge + the whole script in bounded time ---- */
    host_section("6. `action wave` -> masked writes, merge, and bounded completion time");
    {
        const action_cfg_t *c = app_action_get_cfg();
        float sit[12], arm[12], stand[12];
        uint16_t sit_d[12], arm_d[12], stand_d[12], off[12];
        expect_sit(sit);
        expect_wave_arm(sit, arm);
        expect_stand_from_sit(stand);
        duties_of(sit, sit_d);
        duties_of(arm, arm_d);
        duties_of(stand, stand_d);

        /* the wave starts by sitting down (action_sit_direct) */
        const int64_t t0 = host_now_us();
        app_motion_cmd_handle("action", "wave");

        app_action_status_t as;
        /* first wait for the opening sit animation to START (the request is only serviced
         * when the motion task next runs), then for it to finish */
        int start_ticks = 0;
        for (; start_ticks < 20; ++start_ticks) {
            app_action_get_status(&as);
            if (as.pose_anim_active) {
                break;
            }
            (void)host_task_run(1);
        }
        HOST_CHECK(as.pose_anim_active && as.wave_running,
                   "the wave's opening sit animation never started (anim=%d wave=%d)\n",
                   (int)as.pose_anim_active, (int)as.wave_running);

        int wait_ticks = 0;
        for (; wait_ticks < 400; ++wait_ticks) {
            app_action_get_status(&as);
            if (!as.pose_anim_active) {
                break;
            }
            (void)host_task_run(1);
        }
        HOST_CHECK(!as.pose_anim_active && as.wave_running,
                   "the wave's opening sit animation never finished, or the stepper stopped\n");
        printf("  opening sit animation took %d motion ticks after starting in %d\n",
               wait_ticks, start_ticks);

        /* the ARM step is the first thing written after the sit pose is reached */
        host_pca_reset_write_stats();
        int arm_ticks = -1;
        for (int i = 0; i < 200; ++i) {
            (void)host_task_run(1);
            if (host_pca_write_count() > 0) {
                arm_ticks = i + 1;
                break;
            }
        }
        HOST_CHECK(arm_ticks > 0, "the wave never wrote the ARM step within 200 ticks\n");
        printf("  ARM step written on the %d-th tick after the sit pose settled\n", arm_ticks);

        read_all_off(off);
        print_row("sit duty", sit_d);
        print_row("arm want", arm_d);
        print_row("arm got", off);

        /* exactly channels 1,2,7,8 may have been rewritten (action.c:838~841) */
        {
            static const int masked[12] = { 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0 };
            int wrong = 0;
            for (int ch = 0; ch < 12; ++ch) {
                const int written =
                    (host_pca_channel_write_count(ch_board(ch), ch_pca(ch)) > 0) ? 1 : 0;
                if (written != masked[ch]) {
                    printf("  channel %d: %s, expected %s\n", ch,
                           written ? "written" : "kept", masked[ch] ? "written" : "kept");
                    ++wrong;
                }
            }
            HOST_CHECK(wrong == 0, "%d channel(s) written/kept against the ch_mask\n", wrong);
        }

        /* outside the mask the PREVIOUS value (the sit pose) must survive the merge */
        HOST_CHECK(off[0] == sit_d[0] && off[3] == sit_d[3] && off[4] == sit_d[4] &&
                   off[5] == sit_d[5] && off[6] == sit_d[6] && off[9] == sit_d[9] &&
                   off[10] == sit_d[10] && off[11] == sit_d[11],
                   "channels outside ch_mask did not keep their previous value\n");
        /* inside the mask, the action layer's own ARM values must have arrived */
        HOST_CHECK(off[1] == arm_d[1] && off[2] == arm_d[2] &&
                   off[7] == arm_d[7] && off[8] == arm_d[8],
                   "the masked channels do not carry the action layer's ARM values\n");

        /* the whole script must finish, and no faster than its own delays allow */
        const int ticks = run_until_idle(3000);
        const long elapsed_ms = (long)((host_now_us() - t0) / 1000);
        app_action_get_status(&as);

        const long expected_min_ms =
            (long)(2 * c->pose_blend_ms + c->wave_arm_sleep_ms + c->wave_lift_sleep_ms +
                   2 * (long)c->wave_repeat * c->wave_swing_sleep_ms + c->wave_final_sleep_ms);
        printf("  wave finished after %d more motion ticks, %ld ms of mock time "
               "(script's own delays + 2 blends = %ld ms)\n", ticks, elapsed_ms, expected_min_ms);

        HOST_CHECK(ticks >= 0, "the wave never became idle within 3000 ticks\n");
        HOST_CHECK(as.wave_running == false && as.pose_anim_active == false,
                   "wave still running (wave=%d anim=%d)\n",
                   (int)as.wave_running, (int)as.pose_anim_active);
        HOST_CHECK(elapsed_ms >= expected_min_ms,
                   "the wave finished in %ld ms, faster than its own delays allow (%ld ms) "
                   "-> the stepper pacing was dropped\n", elapsed_ms, expected_min_ms);
        HOST_CHECK(elapsed_ms <= expected_min_ms + 2000,
                   "the wave took %ld ms, far longer than the %ld ms its delays imply\n",
                   elapsed_ms, expected_min_ms);

        read_all_off(off);
        print_angle_row("expected", stand);
        print_row("want duty", stand_d);
        print_row("got duty", off);
        const int delta = max_delta(off, stand_d);
        printf("    max |delta| = %d duty count(s)\n", delta);
        HOST_CHECK(delta == 0,
                   "the wave did not end on the action layer's stand pose (%d counts off)\n",
                   delta);
        HOST_CHECK(as.steps > 0, "the wave produced no angles at all\n");
    }

    /* ---- 7. action step (the original web "gait test") + estop ---- */
    host_section("7. `action step` -> chain command state + chain frames; `estop` mid-action");
    {
        app_motion_cmd_handle("action", "step");
        (void)host_task_run(120);            /* 1.2 s -> ~18 chain frames at 65 ms */

        app_chain_status_t cs;
        app_chain_get_status(&cs);
        app_action_status_t as;
        app_action_get_status(&as);

        printf("  chain: spd=%.3f L=%d R=%d valid=%d frames=%u\n",
               (double)cs.spd, cs.L, cs.R, (int)cs.valid, (unsigned)cs.frames);
        HOST_CHECK(cs.spd == 3.0f && cs.L == 1 && cs.R == 1,
                   "ACTION_EFF_MOVE(3,1,1) did not reach the chain: spd=%.3f L=%d R=%d\n",
                   (double)cs.spd, cs.L, cs.R);
        HOST_CHECK(cs.gait_mode == 0 && cs.valid && cs.frames > 1,
                   "the in-place test did not drive the chain (gait=%d valid=%d frames=%u)\n",
                   cs.gait_mode, (int)cs.valid, (unsigned)cs.frames);
        /* `move(3,1,1)` has (L+R) != 0 and |spd| > 0, so it emits the SERVO_INIT effect
         * (and the command layer's move() sets init_case = 0 as well) -- this is where that
         * effect has teeth, since the stand/sit paths plant init_case = 1 and leave it alone. */
        HOST_CHECK(app_chain_get_init_case() == 0,
                   "SERVO_INIT effect did not reach the chain state: init_case=%d, expected 0\n",
                   app_chain_get_init_case());
        HOST_CHECK(cs.goal[1] == (float)(int)s_cfg.in_pit &&
                   cs.goal[2] == (float)(int)s_cfg.in_rol,
                   "the in-place move()'s gait(0) must reset PIT/ROL to %.3f/%.3f, got %.3f/%.3f\n",
                   (double)(float)(int)s_cfg.in_pit, (double)(float)(int)s_cfg.in_rol,
                   (double)cs.goal[1], (double)cs.goal[2]);
        HOST_CHECK(!all_relaxed(), "the in-place test left every channel relaxed\n");
        HOST_CHECK(as.busy, "the step action should still be busy before the e-stop\n");

        motion_stats_t st;
        motion_get_stats(&st);
        printf("  motion stats: action_frames=%u mode=%s\n",
               (unsigned)st.action_frames, motion_mode_name(st.mode));
        HOST_CHECK(st.action_frames > 0,
                   "the motion task never took an angle from the action layer\n");

        const int64_t before_us = host_now_us();
        app_motion_cmd_handle("estop", "host test: mid-action");
        (void)host_task_run(3);              /* at most 3 motion periods = 30 ms */
        const long elapsed_ms = (long)((host_now_us() - before_us) / 1000);

        HOST_CHECK(all_relaxed(), "e-stop in the middle of an action did not relax all 12\n");
        HOST_CHECK(!motion_is_running(), "motion task still running after e-stop\n");
        HOST_CHECK(elapsed_ms <= 200, "e-stop took %ld ms (expected <= ~30 ms plus the "
                                      "console's own 30 ms delay)\n", elapsed_ms);

        app_action_get_status(&as);
        HOST_CHECK(as.busy == false && as.pose_anim_active == false,
                   "e-stop did not cancel the running action (busy=%d anim=%d)\n",
                   (int)as.busy, (int)as.pose_anim_active);
        app_chain_get_status(&cs);
        HOST_CHECK(cs.spd == 0.0f && cs.L == 0 && cs.R == 0,
                   "e-stop left a live movement command in the chain: spd=%.3f L=%d R=%d\n",
                   (double)cs.spd, cs.L, cs.R);
        printf("  e-stop relaxed all 12 channels within %ld ms and cleared the action\n",
               elapsed_ms);
    }

    /* ---- 8. motion stop cancels a running animation ---- */
    host_section("8. `motion stop` in the middle of an animation -> cancelled + relaxed");
    {
        host_task_reset();
        app_motion_cmd_handle("motion", "start");
        app_motion_cmd_handle("action", "sit");
        (void)host_task_run(20);             /* 200 ms: animation is underway */

        app_action_status_t as;
        app_action_get_status(&as);
        HOST_CHECK(as.pose_anim_active, "the sit animation is not running?\n");
        HOST_CHECK(!all_relaxed(), "the sit animation should have energised the channels\n");

        app_motion_cmd_handle("motion", "stop");
        (void)host_task_run(3);

        HOST_CHECK(all_relaxed(), "`motion stop` mid-animation did not relax all 12 channels\n");
        HOST_CHECK(!motion_is_running(), "motion task still running after `motion stop`\n");
        app_action_get_status(&as);
        HOST_CHECK(as.busy == false && as.pose_anim_active == false && as.wave_running == false,
                   "`motion stop` did not cancel the action (busy=%d anim=%d wave=%d)\n",
                   (int)as.busy, (int)as.pose_anim_active, (int)as.wave_running);
    }

    /* ---- 9. no side effect was ever dropped ---- */
    host_section("9. no effect table ever overflowed (P-25) and the board ends relaxed");
    {
        app_action_status_t as;
        app_action_get_status(&as);
        printf("  action layer totals: steps=%u effects=%u overflow=%u\n",
               (unsigned)as.steps, (unsigned)as.effects, (unsigned)as.overflow);
        HOST_CHECK(as.overflow == 0,
                   "%u effect table(s) overflowed -> side effects were dropped\n",
                   (unsigned)as.overflow);
        HOST_CHECK(as.effects > 0, "no effect was ever applied -- the wiring is dead\n");
        HOST_CHECK(as.steps > 0, "the action layer never produced angles\n");

        HOST_CHECK(servo_out_all_off() == ESP_OK, "final all_off failed\n");
        HOST_CHECK(all_relaxed(), "final state is not relaxed\n");
    }

    /* ---- summary ---- */
    printf("\n========================================================\n");
    printf(" checks=%d  failures=%d\n", g_host_checks, g_host_fail);
    if (g_host_fail != 0) {
        printf("RESULT: FAIL -- the ACTION layer is not correctly wired into the firmware\n");
        return 1;
    }
    printf("RESULT: PASS -- all %d checks passed\n", g_host_checks);
    printf("\nNOTE: this suite validates the WIRING and the LOGIC only.  The stubs are\n");
    printf("      single-threaded and the mutexes always succeed, so it cannot catch\n");
    printf("      deadlocks, priority inversion, stack depth, real jitter, or anything\n");
    printf("      about the actual hardware.\n");
    return 0;
}
