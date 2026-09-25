/*
 * test_action_crawl.c -- host test: `action_crawl()` entry + the per-leg trim nudge (P5)
 *
 * WHAT THIS SUITE PROVES
 *
 *   [1] `padog.action_crawl()` (padog.py:810~831) is now reachable from the firmware through
 *       the SAME channel every other action uses -- app_action_request(APP_ACTION_CRAWL) +
 *       app_action_step() -- and all ~14 of its side effects really land, in the original's
 *       order, INCLUDING the ones that cross into the chain module.
 *
 *   [2] The calibration keys `hi`/`hd`, `si`/`sd`, `ip`/`id` (nudge the selected leg's joint
 *       by +-1) exist as one narrow interface on app_config_t.servo_center, and they respect
 *       the project's ONE set of limits (app_config_validate's 0..180) instead of inventing
 *       a second one.
 *
 * WHY THE REFERENCE IS TRUSTWORTHY (and why this is not P-23)
 *   golden/action_crawl.csv is produced by gen_golden.py, which execs the REAL padog.py into a
 *   real module registered in sys.modules['padog'] and calls the REAL action_crawl() with the
 *   clock pinned by mpy_stubs.install_controllable_clock().  Nothing here re-derives the
 *   expected values from my own reading of the Python source.
 *
 * WHY THE STATE IS DUMPED, NOT JUST THE SERVO OUTPUT (P-25)
 *   action_crawl() writes ZERO servo channels -- it is pure cross-module state.  Comparing
 *   angles would compare nothing.  So the CSV records 20 post-values (crawl_phase, both
 *   deadlines, crawl_saved_h, R_H, H_goal, the three gesture goals, spd/L/R, joy_turn,
 *   gait_mode, init_case, both foot offsets, both pose flags, inplace_step_end_ms) and this
 *   file reads every one of them back out of the chain/action modules.
 *
 * THE ONE PLACE WHERE `int()` MATTERS (P-18)
 *   `crawl_saved_h = int(H_goal)` and `R_H = crawl_saved_h`.  The suite therefore uses a
 *   NON-INTEGRAL H_goal (100.5 / 80.25 / -3.5 / 101.75 -- all float32-exact, so tolerance can
 *   stay 0).  With an integral H_goal this test could not tell `R_H = int(H_goal)` apart from
 *   `height(int(H_goal))`, and the latter would ALSO overwrite H_goal -- which the original
 *   does not do.  The CSV keeps an `H_goal` column precisely so that mistake fails loudly.
 *
 *   NOTE (honest scope): in the ORIGINAL's reachable domain H_goal is always an integer
 *   (padog.py:156 `H_goal=int(H_goal)`, web_c.py:366/517 `int(...)`), so the truncation is a
 *   no-op there.  The C port's app_config_t.h_goal is a float and `cfg set h_goal 100.5` is
 *   accepted, so the port's domain is WIDER -- which is exactly why the port must reproduce
 *   the truncation faithfully instead of taking the `set_height()` shortcut.
 *
 * WHAT THIS SUITE DOES NOT COVER (stated, not pretended)
 *   * The crawl STATE MACHINE (phase 1 -> 2 -> finish) -- that lives in control_chain.c and is
 *     covered by test_control_chain / test_control_chain_cmd.
 *   * `t` (the gait phase).  The chain's public API can only ever plant t = 0
 *     (app_chain_reset_pose / app_chain_stand), so a non-zero pre-phase is unreachable here;
 *     a column that is 0 on both sides would test nothing (P-18).  The "gait() only resets t
 *     when the mode really changes" rule is covered by test_control_chain_cmd.c.
 *   * Ticks wraparound: MicroPython's ticks_add/ticks_diff wrap at 2^30; the C deadline
 *     arithmetic is plain int32 addition.  The clock base here keeps the whole run far from
 *     any wrap (same as every other suite in this directory).
 *
 * LIMITS (same as test_motion_app / test_app_action): the stubs are single-threaded and the
 * mutexes always succeed, so this validates LOGIC only -- not real timing, not the board.
 *
 * NOTE: ASCII output only, for the reason recorded as P-06 in 问题与解决记录.md.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app/app_action.h"
#include "app/app_chain.h"
#include "app/app_config.h"
#include "control/action.h"
#include "control/control_chain_cmd.h"
#include "control/servo_map.h"
#include "host_stubs/host_sim.h"

int g_host_fail = 0;
int g_host_checks = 0;

/* ==========================================================================
 * app_cfg_cmd stub: the host must not touch NVS
 *
 * The config is the INPUT of the crawl cases (h_goal / in_pit / in_rol / in_y), so this stub
 * is also the injection point - exactly like test_app_action.c does it.
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
 * CSV
 * ========================================================================== */

#define LINE_MAX   4096
#define FIELDS_MAX 64

/** pre-state columns, in file order (must match gen_golden.ACTION_CRAWL_PRE) */
enum {
    PRE_NOW = 0, PRE_H_GOAL, PRE_R_H,
    PRE_IN_PIT, PRE_IN_ROL, PRE_IN_Y,
    PRE_SPD, PRE_L, PRE_R, PRE_JOY_TURN, PRE_GAIT_MODE,
    PRE_INIT_CASE, PRE_FRONT_Y, PRE_REAR_Y,
    PRE_CRAWL_PHASE, PRE_CRAWL_UNTIL, PRE_CRAWL_SETTLE, PRE_CRAWL_SAVED_H,
    PRE_DIRECT_FREEZE, PRE_POSE_ANIM, PRE_INPLACE,
    PRE_COUNT
};

/** post-state columns, in file order (must match gen_golden.ACTION_CRAWL_POST) */
static const char *const POST_NAMES[] = {
    "crawl_phase", "crawl_until_ms", "crawl_settle_until_ms", "crawl_saved_h",
    "R_H", "H_goal",
    "PIT_goal", "ROL_goal", "X_goal",
    "spd", "L", "R", "joy_turn", "gait_mode",
    "init_case", "front_y", "rear_y",
    "direct_pose_freeze", "pose_anim_active", "inplace_step_end_ms",
};
#define POST_COUNT ((int)(sizeof(POST_NAMES) / sizeof(POST_NAMES[0])))

static long g_values;
static long g_bad;
static long g_shown;
static long g_rows;

static void note(long row, const char *name, double got, double exp)
{
    ++g_values;
    if (got != exp) {
        ++g_bad;
        if (g_shown < 6) {
            ++g_shown;
            printf("  FIRST MISMATCH: row=%ld %s expected=%.9f got=%.9f\n",
                   row, name, exp, got);
        }
    }
}

static int split_csv(char *line, double *out, int max)
{
    int n = 0;
    char *tok = strtok(line, ",\r\n");
    while (tok != NULL && n < max) {
        out[n++] = atof(tok);
        tok = strtok(NULL, ",\r\n");
    }
    return n;
}

/* ==========================================================================
 * planting the pre-state
 *
 * Everything below goes through PUBLIC accessors only -- the same ones the protocol layer
 * will use.  The two pose flags are reached by actually running the action layer (a sit
 * animation), not by poking its struct: that keeps this file honest about "reachable".
 * ========================================================================== */

static void set_cfg_from_row(const double *v)
{
    s_cfg.h_goal = (float)v[PRE_H_GOAL];
    s_cfg.in_pit = (float)v[PRE_IN_PIT];
    s_cfg.in_rol = (float)v[PRE_IN_ROL];
    s_cfg.in_y   = (float)v[PRE_IN_Y];
}

/** drive the action layer to the requested (pose_anim_active, direct_pose_freeze) pair */
static void plant_pose_state(const double *v, int32_t now)
{
    if (v[PRE_POSE_ANIM] == 0.0 && v[PRE_DIRECT_FREEZE] == 0.0) {
        app_action_status_t as;
        app_action_get_status(&as);
        HOST_CHECK(!as.pose_anim_active && !as.direct_pose_freeze,
                   "fresh action layer is not (active=0, freeze=0)\n");
        return;
    }

    /* A sit starts a 900 ms stand -> sit animation: right after the first step
     * active=1 and freeze=1.  One more step 900 ms later finishes it: active=0,
     * freeze stays 1 (hold_freeze=True).  Both pairs appear in the CSV. */
    float deg[ACTION_CHANNELS];
    HOST_CHECK(app_action_request(APP_ACTION_SIT) == ESP_OK, "request sit failed\n");
    (void)app_action_step(now - 5000, deg);
    if (v[PRE_POSE_ANIM] == 0.0) {
        (void)app_action_step(now - 4100, deg);
    }

    app_action_status_t as;
    app_action_get_status(&as);
    HOST_CHECK(as.pose_anim_active == (v[PRE_POSE_ANIM] != 0.0) &&
               as.direct_pose_freeze == (v[PRE_DIRECT_FREEZE] != 0.0),
               "could not plant the pose state (want active=%d freeze=%d, got %d/%d)\n",
               (int)v[PRE_POSE_ANIM], (int)v[PRE_DIRECT_FREEZE],
               (int)as.pose_anim_active, (int)as.direct_pose_freeze);
}

static void plant_chain_state(const double *v)
{
    HOST_CHECK(app_chain_jog((float)v[PRE_SPD], (int)v[PRE_L], (int)v[PRE_R]) == ESP_OK,
               "plant jog failed\n");
    HOST_CHECK(app_chain_set_joy_turn((float)v[PRE_JOY_TURN]) == ESP_OK,
               "plant joy_turn failed\n");
    HOST_CHECK(app_chain_set_gait((int)v[PRE_GAIT_MODE]) == ESP_OK, "plant gait failed\n");
    HOST_CHECK(app_chain_set_init_case((int)v[PRE_INIT_CASE]) == ESP_OK,
               "plant init_case failed\n");
    HOST_CHECK(app_chain_set_sit_offsets((float)v[PRE_FRONT_Y],
                                         (float)v[PRE_REAR_Y]) == ESP_OK,
               "plant sit offsets failed\n");
    /* a pre-value that differs from every post-value, so "the snapshot happened" is real */
    HOST_CHECK(app_chain_gesture(11.0f, -12.0f, 13.0f) == ESP_OK, "plant gesture failed\n");
    /* H_goal first: app_chain_set_height() writes BOTH H_goal and R_H, then R_H is
     * overwritten on its own -- the same shape as the original (H_goal and R_H differ). */
    HOST_CHECK(app_chain_set_height((float)v[PRE_H_GOAL]) == ESP_OK,
               "plant height failed\n");
    HOST_CHECK(app_chain_set_r_h((float)v[PRE_R_H]) == ESP_OK, "plant R_H failed\n");
    HOST_CHECK(app_chain_set_crawl((int)v[PRE_CRAWL_PHASE],
                                   (int32_t)v[PRE_CRAWL_UNTIL],
                                   (int32_t)v[PRE_CRAWL_SETTLE]) == ESP_OK,
               "plant crawl failed\n");
    HOST_CHECK(app_chain_set_crawl_saved_h((int)v[PRE_CRAWL_SAVED_H]) == ESP_OK,
               "plant crawl_saved_h failed\n");
    HOST_CHECK(app_action_set_inplace_step_end_ms((int32_t)v[PRE_INPLACE]) == ESP_OK,
               "plant inplace_step_end_ms failed\n");
}

/* ==========================================================================
 * dumping the post-state
 * ========================================================================== */

static void dump_post(double *out)
{
    app_chain_status_t cs;
    app_chain_get_status(&cs);
    app_action_status_t as;
    app_action_get_status(&as);

    int cp = 0, cu = 0, csu = 0;
    app_chain_get_crawl(&cp, &cu, &csu);
    float front = 0.0f, rear = 0.0f;
    app_chain_get_sit_offsets(&front, &rear);

    int i = 0;
    out[i++] = (double)cp;
    out[i++] = (double)cu;
    out[i++] = (double)csu;
    out[i++] = (double)app_chain_get_crawl_saved_h();
    out[i++] = (double)cs.R_H;
    out[i++] = (double)cs.goal[CONTROL_CHAIN_GOAL_H];
    out[i++] = (double)cs.goal[CONTROL_CHAIN_GOAL_PIT];
    out[i++] = (double)cs.goal[CONTROL_CHAIN_GOAL_ROL];
    out[i++] = (double)cs.goal[CONTROL_CHAIN_GOAL_X];
    out[i++] = (double)cs.spd;
    out[i++] = (double)cs.L;
    out[i++] = (double)cs.R;
    out[i++] = (double)cs.joy_turn;
    out[i++] = (double)cs.gait_mode;
    out[i++] = (double)app_chain_get_init_case();
    out[i++] = (double)front;
    out[i++] = (double)rear;
    out[i++] = as.direct_pose_freeze ? 1.0 : 0.0;
    out[i++] = as.pose_anim_active ? 1.0 : 0.0;
    out[i++] = (double)app_action_get_inplace_step_end_ms();
    HOST_CHECK(i == POST_COUNT, "dump wrote %d of %d post columns\n", i, POST_COUNT);
}

/* ========================================================================== */

static int run_csv(const char *path)
{
    FILE *fh = fopen(path, "r");
    if (fh == NULL) {
        printf("cannot open %s\n", path);
        return 2;
    }

    printf("========================================================\n");
    printf(" action crawl   (padog.action_crawl -> APP_ACTION_CRAWL)\n");
    printf("========================================================\n");
    printf("golden file : %s\n", path);
    printf("reference   : real padog.py exec'd into sys.modules['padog']\n");

    g_values = 0;
    g_bad = 0;
    g_shown = 0;
    g_rows = 0;

    char line[LINE_MAX];
    while (fgets(line, sizeof(line), fh) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        double v[FIELDS_MAX];
        const int n = split_csv(line, v, FIELDS_MAX);
        if (n != PRE_COUNT + POST_COUNT) {
            printf("  row %ld: %d fields, expected %d\n", g_rows, n,
                   PRE_COUNT + POST_COUNT);
            fclose(fh);
            return 2;
        }

        const int32_t now = (int32_t)v[PRE_NOW];

        /* fresh config + fresh chain/action state per row (the generator re-execs
         * padog.py per row for the very same reason) */
        set_cfg_from_row(v);
        HOST_CHECK(app_chain_init() == ESP_OK, "app_chain_init failed\n");
        HOST_CHECK(app_action_init() == ESP_OK, "app_action_init failed\n");

        plant_pose_state(v, now);
        plant_chain_state(v);

        app_action_status_t before;
        app_action_get_status(&before);

        /* ---- the action under test, through the ordinary action channel ---- */
        HOST_CHECK(app_action_request(APP_ACTION_CRAWL) == ESP_OK,
                   "row %ld: app_action_request(APP_ACTION_CRAWL) failed\n", g_rows);
        float deg[ACTION_CHANNELS];
        memset(deg, 0, sizeof(deg));
        const bool produced = app_action_step(now, deg);

        /* the original writes no servo at all -> this layer must not produce angles */
        HOST_CHECK(!produced,
                   "row %ld: action_crawl produced angles -- the original writes no servo\n",
                   g_rows);

        app_action_status_t after;
        app_action_get_status(&after);
        HOST_CHECK(after.overflow == 0, "row %ld: effect overflow\n", g_rows);
        HOST_CHECK(after.effects - before.effects == 9,
                   "row %ld: %u chain effect(s) landed, expected 9\n", g_rows,
                   (unsigned)(after.effects - before.effects));

        double got[POST_COUNT];
        dump_post(got);
        for (int i = 0; i < POST_COUNT; ++i) {
            note(g_rows, POST_NAMES[i], got[i], v[PRE_COUNT + i]);
        }

        if (g_rows == 0) {
            printf("  row 0 post: ");
            for (int i = 0; i < POST_COUNT; ++i) {
                printf("%s=%.4f ", POST_NAMES[i], got[i]);
            }
            printf("\n");
        }
        ++g_rows;
    }
    fclose(fh);

    printf("rows        : %ld\n", g_rows);
    printf("state checks: %ld   (tolerance 0)\n", g_values);
    printf("mismatches  : %ld\n", g_bad);
    if (g_rows == 0) {
        printf("golden file has no data rows\n");
        return 2;
    }
    return (g_bad == 0) ? 0 : 1;
}

/* ==========================================================================
 * [2] the per-leg trim nudge (the original's hi/hd, si/sd, ip/id keys)
 * ========================================================================== */

/** the 12 centre angles, in (leg, joint) order */
static void snap12(const app_config_t *c, float out[12])
{
    for (int leg = 0; leg < APP_CFG_LEGS; ++leg) {
        for (int j = 0; j < APP_CFG_JOINTS; ++j) {
            out[leg * APP_CFG_JOINTS + j] = c->servo_center[leg][j];
        }
    }
}

/**
 * Assert that exactly `idx` of the 12 values changed (by `delta`) and everything else is
 * untouched.  P-18: the 12 values are all distinct, so a swapped leg index or a swapped
 * joint index cannot pass by coincidence.
 */
static void expect_only(const float *before, const app_config_t *after, int idx, float delta)
{
    float now12[12];
    snap12(after, now12);
    int wrong = 0;
    for (int i = 0; i < 12; ++i) {
        const float want = (i == idx) ? (before[i] + delta) : before[i];
        if (now12[i] != want) {
            printf("  servo_center[%d][%d]: got %.1f, expected %.1f%s\n",
                   i / APP_CFG_JOINTS + 1, i % APP_CFG_JOINTS, (double)now12[i],
                   (double)want, (i == idx) ? "  <- the nudged one" : "  <- must be untouched");
            ++wrong;
        }
    }
    HOST_CHECK(wrong == 0, "%d of 12 centre angle(s) are wrong after the nudge\n", wrong);
}

static void run_nudge_tests(void)
{
    host_section("[2] per-leg trim nudge: hi/hd = thigh, si/sd = shank, ip/id = hip");

    app_config_t cfg;
    app_config_defaults(&cfg);

    /* 12 mutually distinct values (P-18) + a non-default selected leg.
     * app_config_defaults() picks cal_leg_sel = 2, so 3 is a real choice. */
    static const float k[APP_CFG_LEGS][APP_CFG_JOINTS] = {
        { 101.0f, 71.0f, 91.0f },   /* leg 1 */
        { 102.0f, 72.0f, 92.0f },   /* leg 2 */
        { 103.0f, 73.0f, 93.0f },   /* leg 3 <- selected */
        { 104.0f, 74.0f, 94.0f },   /* leg 4 */
    };
    for (int leg = 0; leg < APP_CFG_LEGS; ++leg) {
        for (int j = 0; j < APP_CFG_JOINTS; ++j) {
            cfg.servo_center[leg][j] = k[leg][j];
        }
    }
    cfg.cal_leg_sel = 3;
    app_config_validate(&cfg, NULL, NULL, 0);

    int changed = 0;
    char msg[128];
    float before[12];

    /* ---- the letter -> joint mapping itself ----
     * leg index 2 (= leg 3, the selected one) x joint index == the mapping under test.
     * `h` = 大 = THIGH = [1]; `s` = 小 = SHANK = [2]; `p` = 髋 = HIP = [0]. */
    struct { app_cfg_joint_t joint; int col; const char *key; const char *name; } const map[] = {
        { APP_CFG_JOINT_HIP,   0, "ip / id", "hip  (p)" },
        { APP_CFG_JOINT_THIGH, 1, "hi / hd", "thigh(h = da)" },
        { APP_CFG_JOINT_SHANK, 2, "si / sd", "shank(s = xiao)" },
    };
    for (unsigned i = 0; i < sizeof(map) / sizeof(map[0]); ++i) {
        snap12(&cfg, before);
        changed = -1;
        HOST_CHECK(app_config_nudge_servo_center(&cfg, map[i].joint, +1.0f,
                                                 &changed, msg, sizeof(msg)) == APP_CFG_OK,
                   "%s: nudge returned an error\n", map[i].key);
        printf("  %s -> servo_center[3][%d] (%s) %s\n", map[i].key, map[i].col, map[i].name,
               msg);
        HOST_CHECK(changed == 0, "%s: a legal +1 was reported as clamped (%d, %s)\n",
                   map[i].key, changed, msg);
        expect_only(before, &cfg, 2 * APP_CFG_JOINTS + map[i].col, +1.0f);

        /* ... and the -1 direction goes back and one step further */
        snap12(&cfg, before);
        HOST_CHECK(app_config_nudge_servo_center(&cfg, map[i].joint, -1.0f,
                                                 &changed, msg, sizeof(msg)) == APP_CFG_OK,
                   "%s: -1 returned an error\n", map[i].key);
        HOST_CHECK(changed == 0, "%s: a legal -1 was reported as clamped\n", map[i].key);
        expect_only(before, &cfg, 2 * APP_CFG_JOINTS + map[i].col, -1.0f);
        /* restore the baseline for the next iteration */
        cfg.servo_center[2][map[i].col] = k[2][map[i].col];
    }
    /* the mapping really did move three DIFFERENT columns */
    HOST_CHECK(cfg.servo_center[2][0] == k[2][0] && cfg.servo_center[2][1] == k[2][1] &&
               cfg.servo_center[2][2] == k[2][2],
               "the mapping loop did not restore leg 3 to its baseline\n");

    /* ---- leg index selection: cal_leg_sel = 1 must touch leg 1 only ---- */
    cfg.cal_leg_sel = 1;
    snap12(&cfg, before);
    changed = -1;
    msg[0] = 'x';
    HOST_CHECK(app_config_nudge_servo_center(&cfg, APP_CFG_JOINT_THIGH, +1.0f,
                                             &changed, msg, sizeof(msg)) == APP_CFG_OK,
               "cal_leg_sel=1: nudge returned an error\n");
    /* the reported-changes mechanism only speaks up when it actually clamped something */
    HOST_CHECK(changed == 0 && msg[0] == '\0',
               "cal_leg_sel=1: nothing was clamped, so it must report changed=0 and an "
               "empty message (got changed=%d msg='%s')\n", changed, msg);
    expect_only(before, &cfg, 0 * APP_CFG_JOINTS + 1, +1.0f);
    cfg.cal_leg_sel = 3;

    /* ---- the upper clamp: it really bounds, and it says so ---- */
    cfg.servo_center[2][APP_CFG_JOINT_THIGH] = 180.0f;
    cfg.cal_leg_sel = 3;
    snap12(&cfg, before);
    changed = 0;
    msg[0] = '\0';
    HOST_CHECK(app_config_nudge_servo_center(&cfg, APP_CFG_JOINT_THIGH, +1.0f,
                                             &changed, msg, sizeof(msg)) == APP_CFG_OK,
               "clamp(180): nudge returned an error\n");
    printf("  180 + 1 -> %.1f   changed=%d msg=%s\n",
           (double)cfg.servo_center[2][APP_CFG_JOINT_THIGH], changed, msg);
    HOST_CHECK(cfg.servo_center[2][APP_CFG_JOINT_THIGH] == 180.0f,
               "the upper clamp did not bound the value: %.1f\n",
               (double)cfg.servo_center[2][APP_CFG_JOINT_THIGH]);
    HOST_CHECK(changed == 1 && strcmp(msg, "servo_center[3].thigh") == 0,
               "the clamp was not reported through the usual mechanism (%d, '%s')\n",
               changed, msg);
    expect_only(before, &cfg, 2 * APP_CFG_JOINTS + 1, 0.0f);

    /* ---- the lower clamp ---- */
    cfg.servo_center[2][APP_CFG_JOINT_SHANK] = 0.0f;
    snap12(&cfg, before);
    changed = 0;
    HOST_CHECK(app_config_nudge_servo_center(&cfg, APP_CFG_JOINT_SHANK, -1.0f,
                                             &changed, msg, sizeof(msg)) == APP_CFG_OK,
               "clamp(0): nudge returned an error\n");
    HOST_CHECK(cfg.servo_center[2][APP_CFG_JOINT_SHANK] == 0.0f && changed == 1 &&
               strcmp(msg, "servo_center[3].shank") == 0,
               "the lower clamp did not bound/report: %.1f changed=%d msg=%s\n",
               (double)cfg.servo_center[2][APP_CFG_JOINT_SHANK], changed, msg);
    expect_only(before, &cfg, 2 * APP_CFG_JOINTS + 2, 0.0f);

    /* ---- "no unlimited accumulation": repeated +1 stops at the bound ---- */
    cfg.servo_center[2][APP_CFG_JOINT_SHANK] = 178.0f;
    for (int i = 0; i < 5; ++i) {
        changed = -1;
        (void)app_config_nudge_servo_center(&cfg, APP_CFG_JOINT_SHANK, +1.0f,
                                            &changed, msg, sizeof(msg));
    }
    printf("  178 + 1 five times -> %.1f (last changed=%d)\n",
           (double)cfg.servo_center[2][APP_CFG_JOINT_SHANK], changed);
    HOST_CHECK(cfg.servo_center[2][APP_CFG_JOINT_SHANK] == 180.0f,
               "five +1 nudges from 178 reached %.1f, expected the 180 bound\n",
               (double)cfg.servo_center[2][APP_CFG_JOINT_SHANK]);
    HOST_CHECK(changed == 1, "the last +1 was not reported as clamped (%d)\n", changed);

    /* ---- ONE set of limits: the nudge and `cfg set` clamp to the same bound ----
     * P-22/P-27: if the nudge had invented its own limit, this would catch it. */
    {
        app_config_t a = cfg;
        app_config_t b = cfg;
        int ch_a = 0, ch_b = 0;
        a.servo_center[2][APP_CFG_JOINT_SHANK] = 178.0f;
        b.servo_center[2][APP_CFG_JOINT_SHANK] = 999.0f;
        char ma[128], mb[128];
        (void)app_config_nudge_servo_center(&a, APP_CFG_JOINT_SHANK, +1.0f,
                                            &ch_a, ma, sizeof(ma));
        (void)app_config_nudge_servo_center(&a, APP_CFG_JOINT_SHANK, +1.0f,
                                            &ch_a, ma, sizeof(ma));
        (void)app_config_nudge_servo_center(&a, APP_CFG_JOINT_SHANK, +1.0f,
                                            &ch_a, ma, sizeof(ma));
        (void)app_config_validate(&b, &ch_b, mb, sizeof(mb));
        printf("  nudge path bound=%.1f ('%s'), cfg-set path bound=%.1f ('%s')\n",
               (double)a.servo_center[2][APP_CFG_JOINT_SHANK], ma,
               (double)b.servo_center[2][APP_CFG_JOINT_SHANK], mb);
        HOST_CHECK(a.servo_center[2][APP_CFG_JOINT_SHANK] ==
                   b.servo_center[2][APP_CFG_JOINT_SHANK],
                   "the nudge and `cfg set` clamp to different bounds (%.1f vs %.1f)\n",
                   (double)a.servo_center[2][APP_CFG_JOINT_SHANK],
                   (double)b.servo_center[2][APP_CFG_JOINT_SHANK]);
    }

    /* ---- rejects nonsense instead of silently doing something ---- */
    {
        app_config_t c2 = cfg;
        int ch = 0;
        char m[128];
        snap12(&c2, before);
        HOST_CHECK(app_config_nudge_servo_center(&c2, (app_cfg_joint_t)APP_CFG_JOINTS,
                                                 +1.0f, &ch, m, sizeof(m)) == APP_CFG_ERR_ARG,
                   "joint == APP_CFG_JOINTS was accepted\n");
        HOST_CHECK(app_config_nudge_servo_center(&c2, (app_cfg_joint_t)-1,
                                                 +1.0f, &ch, m, sizeof(m)) == APP_CFG_ERR_ARG,
                   "joint == -1 was accepted\n");
        c2.cal_leg_sel = 0;
        HOST_CHECK(app_config_nudge_servo_center(&c2, APP_CFG_JOINT_HIP, +1.0f,
                                                 &ch, m, sizeof(m)) == APP_CFG_ERR_ARG,
                   "cal_leg_sel == 0 was accepted\n");
        c2.cal_leg_sel = APP_CFG_LEGS + 1;
        HOST_CHECK(app_config_nudge_servo_center(&c2, APP_CFG_JOINT_HIP, +1.0f,
                                                 &ch, m, sizeof(m)) == APP_CFG_ERR_ARG,
                   "cal_leg_sel == %d was accepted\n", APP_CFG_LEGS + 1);
        HOST_CHECK(app_config_nudge_servo_center(NULL, APP_CFG_JOINT_HIP, +1.0f,
                                                 &ch, m, sizeof(m)) == APP_CFG_ERR_ARG,
                   "cfg == NULL was accepted\n");
        expect_only(before, &c2, -1, 0.0f);
    }

    /* ---- does a runtime servo_center change reach the servos?  NO -- and this is
     * where the wiring stops: both consumers keep their own cached COPY of the 12
     * centre angles, filled at init time.  Trace it here so the answer is evidence,
     * not an opinion. ---- */
    {
        app_config_t *live = &s_cfg;
        const float old_thigh = live->servo_center[0][APP_CFG_JOINT_THIGH];
        int ch = 0;
        char m[128];
        live->cal_leg_sel = 1;
        HOST_CHECK(app_config_nudge_servo_center(live, APP_CFG_JOINT_THIGH, +1.0f,
                                                 &ch, m, sizeof(m)) == APP_CFG_OK,
                   "live-config nudge failed\n");
        HOST_CHECK(live->servo_center[0][APP_CFG_JOINT_THIGH] == old_thigh + 1.0f,
                   "the live app_config copy was not changed\n");

        /* the action layer's cached copy must still hold the OLD value */
        const action_cfg_t *ac = app_action_get_cfg();
        HOST_CHECK(ac != NULL && ac->init[0][APP_CFG_JOINT_THIGH] == old_thigh,
                   "app_action's cfg picked the change up without app_action_init(): %.1f\n",
                   (double)(ac ? ac->init[0][APP_CFG_JOINT_THIGH] : -1.0f));
        /* and the chain's cached copy (visible through a straight stand command) */
        HOST_CHECK(app_action_init() == ESP_OK, "app_action_init (reload) failed\n");
        HOST_CHECK(app_action_get_cfg()->init[0][APP_CFG_JOINT_THIGH] == old_thigh + 1.0f,
                   "app_action_init() did not pick the change up\n");
        live->servo_center[0][APP_CFG_JOINT_THIGH] = old_thigh;   /* restore */
        HOST_CHECK(app_action_init() == ESP_OK, "app_action_init (restore) failed\n");
    }
}

/* ========================================================================== */

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "golden/action_crawl.csv";

    host_log_set_quiet(1);
    host_clock_reset();

    printf("========================================================\n");
    printf(" P5 step 1/2: action_crawl entry + calibration nudges\n");
    printf("========================================================\n");
    printf("Single-threaded mock: logic only -- NOT real timing, NOT concurrency.\n");

    HOST_CHECK(app_cfg_cmd_init() == APP_CFG_OK, "app_cfg_cmd_init failed\n");

    /* log string of the new action kind (it is what the console prints) */
    HOST_CHECK(strcmp(app_action_kind_name(APP_ACTION_CRAWL), "crawl") == 0,
               "app_action_kind_name(APP_ACTION_CRAWL) = '%s'\n",
               app_action_kind_name(APP_ACTION_CRAWL));

    const int r1 = run_csv(path);
    printf("\n");
    run_nudge_tests();

    printf("\n========================================================\n");
    printf(" checks=%d  failures=%d\n", g_host_checks, g_host_fail);
    if (r1 == 2) {
        printf("RESULT: FAIL -- golden/action_crawl.csv is missing or unreadable\n");
        return 2;
    }
    if (r1 != 0 || g_host_fail != 0) {
        printf("RESULT: FAIL -- the crawl entry differs from the MicroPython reference,\n");
        printf("                or a calibration nudge misbehaved\n");
        return 1;
    }
    printf("RESULT: PASS -- action_crawl's side effects and the per-leg nudges are correct\n");
    printf("\nNOTE: validates the WIRING and the LOGIC only.  The stubs are single-threaded\n");
    printf("      and the mutexes always succeed, so this cannot catch deadlocks, priority\n");
    printf("      inversion, stack depth, real jitter, or anything about the hardware.\n");
    return 0;
}
