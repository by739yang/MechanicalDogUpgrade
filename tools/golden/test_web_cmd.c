/*
 * test_web_cmd.c -- host test: the web request -> robot command translation layer (P5,
 *                   ESP-IDF_C迁移表.md section 0.5(8))
 *
 * WHAT THIS SUITE IS
 *
 *   The protocol layer (comm/proto.{h,c}) turns a request into VALIDATED NUMBERS.  It does
 *   not decide how the dog moves.  This suite covers the layer that does:
 *
 *     web_common.py  _joy_f_to_thr   70    -> web_cmd_joy_f_to_thr()
 *                    _thr_is_backward 86   -> web_cmd_thr_is_backward()
 *                    _thr_is_forward  95   -> web_cmd_thr_is_forward()
 *                    apply_dog_stick  111  -> web_cmd_apply_dog_stick()
 *                    process_dog_from_req 142 -> web_cmd_process_request()
 *
 *   ...all driven through the REAL app_chain_* accessors (app_chain_jog / app_chain_drive /
 *   app_chain_set_joy_turn / app_chain_set_sit_offsets / app_chain_set_gait /
 *   app_chain_get_status / app_chain_get_crawl), which the callbacks in this file forward to.
 *
 * WHY THE CALL SEQUENCE, NOT THE FINAL STATE (P-26 / P-29)
 *
 *   Two separate times in this project a feature was completely unreachable while every
 *   value-comparison test was green, because what was missing was WHICH FUNCTION GETS CALLED,
 *   not what value it computes:
 *
 *     * P-26: `_go = padog.drive if gait_mode == 1 else padog.move`
 *             (web_common.py:129).  Get it wrong and WALK silently reverts to TROT -- but
 *             spd / L / R / joy_turn / gait_mode all end up IDENTICAL, so a state-only
 *             comparison cannot see it.  Two rows of golden/web_cmd.csv (gait_mode=1) are
 *             the only thing that can.
 *     * P-29: `set_leg_sit_offsets(0, 0)` is called on EVERY path of process_dog_from_req
 *             (web_common.py:152 / 158 / 163).  Drop one and no observable value changes on
 *             that path -- only the call trace does.
 *
 *   So the CSV has `n_calls` and `callseq` columns.  `callseq` uses ';' inside an entry and
 *   '|' between entries (never ','), because the row itself is CSV.
 *
 * WHY THE FLOAT COMPARISON IS STILL EXACT (P-17)
 *
 *   `thr` comes out of float32 arithmetic in C, while the generator calls the real Python
 *   `_joy_f_to_thr()` in double.  Rather than introduce an epsilon, the generator rounds the
 *   double reference to float32 (`_web_cmd_f32_ref`) and this file compares the C float32
 *   against that value bit-for-bit after an atof -> float round trip.  Measured identical for
 *   every vf in -100..100 by gen_golden's own probe.  The call-sequence text uses C's `%g`,
 *   which the generator reproduces with Python's `%g` (same default precision 6).
 *
 * THE REFERENCE IS REAL (not P-23)
 *
 *   golden/web_cmd.csv is produced by gen_golden.gen_web_cmd(), which execs the REAL
 *   micropython/padog.py and the REAL micropython/web_common.py into real module objects
 *   registered in sys.modules, pins the clock with install_controllable_clock(), and
 *   monkey-patches padog.move / padog.drive / padog.set_joy_turn / padog.set_leg_sit_offsets /
 *   padog._turn_phase_lr / padog.gait / padog.servo_init to append (name, args) to a list.
 *   Nothing here re-derives any expected value from my own reading of the Python source.
 *
 *   Reference self-check in the generator: `padog.joy_fwd_sign` must really be -1.  It comes
 *   from micropython/config_s.py:56, and the whole suite's sign convention hangs off it.
 *
 * WHAT THIS SUITE DOES NOT COVER (stated, not pretended)
 *
 *   * String parsing (`f=`, `t=`, `&`, `?`, the leading-int quirks) -- that is the other
 *     agent's comm/proto.{h,c}, and duplicating a tokeniser here would be P-22/P-27.
 *   * `mech_arm.is_enabled()` itself.  That is P6 and does not exist yet, so the predicate is
 *     an INPUT here (the `arm` column) instead of an invented piece of state.
 *   * The two flip sides of the inplace / crawl guards (does `move()` really clear
 *     inplace_step_end_ms, does crawl_phase really block the stop command) -- those are
 *     chain-internal and have their own suites; they are checked here only where they are
 *     reachable through this layer's public API.
 *   * Ticks wraparound (MicroPython wraps at 2^30, C uses plain int32 arithmetic); the clock
 *     base keeps every row far away from it, same as the other suites in this directory.
 *
 * LIMITS: single-threaded host stubs with always-succeeding mutexes -- LOGIC only, no timing,
 * no concurrency, no board.
 *
 * NOTE: ASCII output only, for the reason recorded as P-06 in 问题与解决记录.md.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app/app_action.h"
#include "app/app_chain.h"
#include "app/app_config.h"
#include "comm/web_cmd.h"
#include "control/control_chain.h"
#include "host_stubs/host_sim.h"

/* ==========================================================================
 * app_cfg_cmd stub: the host must not touch NVS.
 *
 * app_chain_init() reads the config through app_cfg_cmd_get(); the web command layer does not
 * read the config at all, so the values are irrelevant here -- they just have to exist so the
 * chain can be initialised and gait_mode / crawl_phase can be planted through public accessors.
 * ========================================================================== */
static app_config_t s_cfg;

/* host_sim.c's HOST_CHECK() macros report through these two counters */
int g_host_fail = 0;
int g_host_checks = 0;

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
 * the call-trace recorder
 * ========================================================================== */

enum {
    CALL_SIT = 0,
    CALL_MOVE,
    CALL_DRIVE,
    CALL_JOY_TURN,
    CALL_TURN_LR,
    CALL_COUNT
};

static const char *const CALL_NAMES[CALL_COUNT] = {
    "sit", "move", "drive", "joy_turn", "turn_lr",
};

#define TRACE_MAX       16
#define TRACE_TEXT_MAX  512

typedef struct {
    int   kind;
    float a;        /* sit: front_y | move/drive: spd | joy_turn: pct | turn_lr: joy_turn */
    int   l;        /* move/drive: L  | turn_lr: L                                 */
    int   r;        /* move/drive: R  | turn_lr: R                                 */
    float b;        /* sit: rear_y                                                 */
} trace_entry_t;

static trace_entry_t s_trace[TRACE_MAX];
static int s_trace_n = 0;
static int s_trace_overflow = 0;

static void trace_reset(void)
{
    s_trace_n = 0;
    s_trace_overflow = 0;
}

static void trace_push(int kind, float a, int l, int r, float b)
{
    if (s_trace_n >= TRACE_MAX) {
        s_trace_overflow = 1;
        return;
    }
    s_trace[s_trace_n].kind = kind;
    s_trace[s_trace_n].a    = a;
    s_trace[s_trace_n].l    = l;
    s_trace[s_trace_n].r    = r;
    s_trace[s_trace_n].b    = b;
    ++s_trace_n;
}

/** The exact text encoding that gen_golden._web_cmd_sequence() produces.
 *
 *  Appends exactly ONE entry (overload switch: each append is emitted right after it is
 *  produced, so the buffer can never hold a half-finished entry).
 *  `sit:<front>;<rear>` | `move|drive:<spd>;<L>;<R>` | `joy_turn:<pct>` | `turn_lr:<jt>;<L>;<R>`
 */
static void trace_append(char *out, size_t n, int kind, float a, int l, int r, float b)
{
    char item[96];
    switch (kind) {
    case CALL_SIT:
        snprintf(item, sizeof(item), "sit:%g;%g", (double)a, (double)b);
        break;
    case CALL_MOVE:
    case CALL_DRIVE:
        snprintf(item, sizeof(item), "%s:%g;%d;%d", CALL_NAMES[kind], (double)a, l, r);
        break;
    case CALL_JOY_TURN:
        snprintf(item, sizeof(item), "joy_turn:%g", (double)a);
        break;
    default:
        snprintf(item, sizeof(item), "turn_lr:%g;%d;%d", (double)a, l, r);
        break;
    }
    const size_t used = strlen(out);
    if (used != 0) {
        if (used + 1 >= n) {
            return;
        }
        out[used] = '|';
        out[used + 1] = '\0';
    }
    const size_t used2 = strlen(out);
    if (used2 + strlen(item) + 1 > n) {
        return;     /* 截断时必然比较不符，不会假通过 */
    }
    memcpy(out + used2, item, strlen(item) + 1);
}

/** encode a recorded trace (used for both the CSV rows and the contract-check cases) */
static void trace_encode(char *out, size_t n, const trace_entry_t *t, int count)
{
    out[0] = '\0';
    if (count <= 0) {
        snprintf(out, n, "-");
        return;
    }
    for (int i = 0; i < count; ++i) {
        trace_append(out, n, t[i].kind, t[i].a, t[i].l, t[i].r, t[i].b);
    }
}

/** encode the CURRENT trace */
static void trace_text(char *out, size_t n)
{
    trace_encode(out, n, s_trace, s_trace_n);
}

/* ==========================================================================
 * the web_cmd context: every callback RECORDS and then forwards to the real accessor
 *
 * Forwarding is deliberate: it makes the resulting chain state real (so the post columns are
 * a genuine consequence of the calls) instead of a local mock that could disagree with the
 * firmware.  Nothing here patches or reimplements an accessor.
 * ========================================================================== */

static int s_accessor_fail = 0;

/** counted by both the CSV pass and the contract checks; the finding is reported, not hidden */
static long g_findings = 0;

/** how many drive() / move(4,1,1) calls the CONTRACT pass produced (carried into the report) */
static long g_contract_drive = 0;
static long g_contract_move4 = 0;

static void cb_set_leg_sit_offsets(void *user, float front_y, float rear_y)
{
    (void)user;
    trace_push(CALL_SIT, front_y, 0, 0, rear_y);
    if (app_chain_set_sit_offsets(front_y, rear_y) != ESP_OK) {
        s_accessor_fail = 1;
    }
}

static void cb_move(void *user, float spd, int L, int R)
{
    (void)user;
    trace_push(CALL_MOVE, spd, L, R, 0.0f);
    if (app_chain_jog(spd, L, R) != ESP_OK) {
        s_accessor_fail = 1;
    }
}

static void cb_drive(void *user, float spd, int L, int R)
{
    (void)user;
    trace_push(CALL_DRIVE, spd, L, R, 0.0f);
    if (app_chain_drive(spd, L, R) != ESP_OK) {
        s_accessor_fail = 1;
    }
}

static void cb_set_joy_turn(void *user, float pct)
{
    (void)user;
    trace_push(CALL_JOY_TURN, pct, 0, 0, 0.0f);
    if (app_chain_set_joy_turn(pct) != ESP_OK) {
        s_accessor_fail = 1;
    }
}

static void cb_turn_phase_lr(void *user, float joy_turn, int *out_l, int *out_r)
{
    (void)user;
    /* The real mapping lives in control_chain_turn_phase_lr(); it is configured with
     * hip_turn_dead = 10 (control_chain.c:183) while this layer's turn dead zone is 20
     * (JOY_TURN_DEAD).  See the note in web_cmd.c.  The switch is what web_common.py does:
     * `L, R = padog._turn_phase_lr(float(t))`. */
    control_chain_cfg_t cfg;
    control_chain_cfg_defaults(&cfg);
    int l = 1;
    int r = 1;
    control_chain_turn_phase_lr(&cfg, joy_turn, &l, &r);
    trace_push(CALL_TURN_LR, joy_turn, l, r, 0.0f);
    *out_l = l;
    *out_r = r;
}

static int cb_gait_mode(void *user)
{
    (void)user;
    /* READ BACK from the chain, not from a local mirror: the move-vs-drive choice has to be
     * driven by the same gait_mode the real firmware would see (P-26). */
    app_chain_status_t st;
    app_chain_get_status(&st);
    return st.gait_mode;
}

static int cb_crawl_phase(void *user)
{
    (void)user;
    int phase = 0;
    app_chain_get_crawl(&phase, NULL, NULL);
    return phase;
}

static int32_t cb_inplace_step_end_ms(void *user)
{
    (void)user;
    /* THE REAL SOURCE, not a mirror: app_action owns `inplace_step_end_ms`
     * (control/action.h:57), and app_action_get_inplace_step_end_ms() is the narrow reader the
     * action layer already exposes.  Wiring this suite to a private copy would be exactly the
     * "second source of truth" mistake (P-22/P-27). */
    return app_action_get_inplace_step_end_ms();
}

static web_cmd_ctx_t make_ctx(void)
{
    web_cmd_ctx_t ctx;
    web_cmd_ctx_init(&ctx);
    ctx.set_leg_sit_offsets = cb_set_leg_sit_offsets;
    ctx.move                = cb_move;
    ctx.drive               = cb_drive;
    ctx.set_joy_turn        = cb_set_joy_turn;
    ctx.turn_phase_lr       = cb_turn_phase_lr;
    ctx.gait_mode           = cb_gait_mode;
    ctx.crawl_phase         = cb_crawl_phase;
    ctx.inplace_step_end_ms = cb_inplace_step_end_ms;
    return ctx;
}

/* ==========================================================================
 * CSV
 * ========================================================================== */

#define LINE_MAX   4096
#define FIELDS_MAX 64

/* pre columns, in file order (must match gen_golden.WEB_CMD_PRE) */
enum {
    PRE_NOW = 0, PRE_VALUE_F, PRE_HAS_F, PRE_TURN, PRE_HAS_T,
    PRE_FORCE, PRE_ARM, PRE_CRAWL, PRE_GAIT,
    PRE_COUNT
};

/* post columns, in file order (must match gen_golden.WEB_CMD_POST) */
enum {
    POST_THR = 0, POST_SPD, POST_L, POST_R, POST_JOY_TURN, POST_GAIT,
    POST_N_CALLS, POST_CALLSEQ,
    POST_COUNT
};

static long g_values;
static long g_bad;
static long g_shown;
static long g_rows;
static long g_seq_bad;
static long g_state_bad;

/* which code paths the table really exercised (P-18: prove the cases are not degenerate) */
static long g_saw_move;
static long g_saw_drive;
static long g_saw_move4;        /* the inplace move(4,1,1) */
static long g_saw_blind;        /* a path that emits ONLY set_leg_sit_offsets(0,0) */
static long g_saw_zero_cmd;     /* a request that emits NOTHING at all */

static void note(long row, const char *name, double got, double exp, const char *kind)
{
    ++g_values;
    if (got != exp) {
        ++g_bad;
        if (strcmp(kind, "seq") == 0) {
            ++g_seq_bad;
        } else {
            ++g_state_bad;
        }
        if (g_shown < 8) {
            ++g_shown;
            printf("  FIRST MISMATCH: row=%ld col=%s expected=%.9g got=%.9g\n",
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

/** the callseq column is text, and it is the LAST column.  `strtok()` destroys the line while
 *  finding the numeric fields, so the raw copy is what the text is recovered from. */
static int split_csv_line(char *raw, double *vals, int max, char *seq, size_t seq_n)
{
    seq[0] = '\0';

    char work[LINE_MAX];
    strncpy(work, raw, sizeof(work) - 1);
    work[sizeof(work) - 1] = '\0';
    const int n = split_csv(work, vals, max);
    if (n != PRE_COUNT + POST_COUNT) {
        return n;
    }

    char *p = raw;
    for (int i = 0; i < PRE_COUNT + POST_COUNT - 1 && p != NULL; ++i) {
        p = strchr(p, ',');
        if (p != NULL) {
            ++p;
        }
    }
    if (p == NULL) {
        return n;
    }
    size_t len = strcspn(p, "\r\n");
    if (len >= seq_n) {
        len = seq_n - 1;
    }
    memcpy(seq, p, len);
    seq[len] = '\0';
    return n;
}

/* ==========================================================================
 * per-row driver: plant the pre-state through PUBLIC accessors, run the layer
 * ========================================================================== */

static void plant(const double *v)
{
    /* gait_mode: the generator plants it with padog.gait(), so this uses the chain's
     * set_gait().  Note the difference from padog.move(): move() would reset gait_mode, which
     * is exactly why this happens BEFORE the request is processed. */
    HOST_CHECK(app_chain_set_gait((int)v[PRE_GAIT]) == ESP_OK, "plant gait failed\n");
    int phase = (int)v[PRE_CRAWL];
    HOST_CHECK(app_chain_set_crawl(phase, 0, 0) == ESP_OK, "plant crawl failed\n");
    HOST_CHECK(app_chain_jog(0.0f, 0, 0) == ESP_OK, "plant jog failed\n");
    HOST_CHECK(app_chain_set_joy_turn(0.0f) == ESP_OK, "plant joy_turn failed\n");
    /* Every row of this table has inplace == 0 (that is what the generator emits), so the
     * action layer's deadline must be cleared here -- otherwise a leftover from an earlier
     * case would silently reroute the row into the inplace branch (P-18: an unpinned input). */
    HOST_CHECK(app_action_set_inplace_step_end_ms(0) == ESP_OK,
               "plant inplace_step_end_ms failed\n");
}

static void dump_post(double *out)
{
    app_chain_status_t cs;
    app_chain_get_status(&cs);
    /* ⚠️ 每个 float 成员都 `(double)(float)`：参考值在 CSV 里是 float32 的十进制表示，
     * 而 `atof()` 给出的是**最接近那个十进制的 double**（例如 "1.200000" -> 1.2，
     * 而 float32 的 1.2f 是 1.2000000476837158）。两边都先压回 float32，
     * 比较才是逐位相等，不需要 epsilon（P-17）。 */
    out[POST_THR]      = 0.0;   /* filled by the caller (also float32-rounded there) */
    out[POST_SPD]      = (double)(float)cs.spd;
    out[POST_L]        = (double)cs.L;
    out[POST_R]        = (double)cs.R;
    out[POST_JOY_TURN] = (double)(float)cs.joy_turn;
    out[POST_GAIT]     = (double)cs.gait_mode;
    out[POST_N_CALLS]  = (double)s_trace_n;
}

/* ==========================================================================
 * the extra C-side contract checks (things the CSV structurally cannot express)
 * ========================================================================== */

static void contract_checks(void)
{
    printf("\n---- web_cmd contract checks (outside the CSV) ----\n");

    /* coverage counters are accumulated across BOTH passes (the CSV pass and this table), so
     * they start at zero here and are folded into the report at the end of run_csv() */
    g_saw_move = 0;
    g_saw_drive = 0;
    g_saw_move4 = 0;
    g_contract_drive = 0;
    g_contract_move4 = 0;
    g_findings = 0;

    HOST_CHECK(app_cfg_cmd_init() == APP_CFG_OK, "cfg init failed\n");
    HOST_CHECK(app_chain_init() == ESP_OK, "chain init failed\n");
    HOST_CHECK(app_action_init() == ESP_OK, "action init failed\n");

    web_cmd_ctx_t ctx = make_ctx();

    /* [1] _joy_f_to_thr: the four special points, checked against the REAL Python values
     *     captured in the generator's self-checks (dead zone is strict <, clamps are strict). */
    HOST_CHECK(web_cmd_joy_f_to_thr(0) == 0.0f, "vf=0 must map to 0\n");
    HOST_CHECK(web_cmd_joy_f_to_thr(9) == 0.0f, "vf=9 must map to 0\n");
    HOST_CHECK(web_cmd_joy_f_to_thr(-9) == 0.0f, "vf=-9 must map to 0\n");
    HOST_CHECK(web_cmd_joy_f_to_thr(10) != 0.0f, "vf=10 must NOT map to 0 (strict <)\n");
    HOST_CHECK(web_cmd_joy_f_to_thr(-10) != 0.0f, "vf=-10 must NOT map to 0\n");
    HOST_CHECK(web_cmd_joy_f_to_thr(60) == -3.0f, "vf=60 must clamp to -3.0\n");
    HOST_CHECK(web_cmd_joy_f_to_thr(1000) == -3.0f, "vf=1000 must clamp to -3.0\n");
    HOST_CHECK(web_cmd_joy_f_to_thr(-100) == 6.0f, "vf=-100 must be exactly 6.0\n");
    HOST_CHECK(web_cmd_joy_f_to_thr(-101) == 6.0f, "vf=-101 must clamp to 6.0\n");
    HOST_CHECK(web_cmd_joy_f_to_thr(-500) == 6.0f, "vf=-500 must clamp to 6.0\n");

    /* [2] the predicates, exercised on values that _joy_f_to_thr CANNOT produce.
     *     This is the one real difference from the original: the original applies the sign
     *     and the clamp only in _joy_f_to_thr, while _thr_is_forward/_thr_is_backward test the
     *     raw thr they are given.  These two calls pin the 0.35 boundary with STRICT
     *     inequalities (exactly 0.35 is neither forward nor backward). */
    HOST_CHECK(web_cmd_thr_is_forward(0.0f) == 0, "+0.0 is not forward (sign is -1)\n");
    HOST_CHECK(web_cmd_thr_is_backward(0.0f) == 0, "+0.0 is not backward\n");
    HOST_CHECK(web_cmd_thr_is_forward(-0.36f) == 1, "-0.36 is forward\n");
    HOST_CHECK(web_cmd_thr_is_forward(-0.35f) == 0, "-0.35 exactly is NOT forward (strict <)\n");
    HOST_CHECK(web_cmd_thr_is_backward(0.35f) == 0, "+0.35 exactly is NOT backward (strict >)\n");
    HOST_CHECK(web_cmd_thr_is_backward(0.36f) == 1, "+0.36 is backward\n");
    HOST_CHECK(web_cmd_thr_is_forward(0.5f) == 0, "+0.5 is not forward\n");
    HOST_CHECK(web_cmd_thr_is_backward(-0.5f) == 0, "-0.5 is not backward\n");

    /* [2b] the four-way choice, exercised directly on apply_dog_stick (so NO leading
     *      set_leg_sit_offsets -- that belongs to process_dog_from_req, one layer above).
     *      The CSV can only reach three of the four arms: `_joy_f_to_thr` clamps to
     *      [-3.0, 6.0], and +6.0 is unreachable positive (vf >= 0 gives thr <= 0 with
     *      joy_fwd_sign == -1), so "backward" can never be the arm chosen through
     *      process_dog_from_req.  This table pins it at the layer where it really lives. */
    {
        struct { float thr; int turn; const char *want; } CHOICE[] = {
            {  0.00f,   0, "joy_turn:0|move:0;1;1" },                  /* neither (L=R=1!) */
            { -1.00f,   0, "joy_turn:0|move:-1;1;1" },                 /* forward */
            {  1.00f,   0, "joy_turn:0|move:2;1;1" },                  /* backward */
            {  1.00f,  30, "joy_turn:-30|turn_lr:-30;1;-1|move:2.5;1;-1" }, /* turning */
            {  6.00f,   0, "joy_turn:0|move:2;1;1" },                  /* top of the clamp */
            {  0.34f,   0, "joy_turn:0|move:0;1;1" },                  /* below the threshold */
            { -0.34f,   0, "joy_turn:0|move:0;1;1" },
            {  1.00f, -30, "joy_turn:30|turn_lr:30;-1;1|move:2.5;-1;1" },
        };
        for (unsigned i = 0; i < sizeof(CHOICE) / sizeof(CHOICE[0]); ++i) {
            HOST_CHECK(app_cfg_cmd_init() == APP_CFG_OK, "cfg init failed\n");
            HOST_CHECK(app_chain_init() == ESP_OK, "chain init failed\n");
            trace_reset();
            HOST_CHECK(web_cmd_apply_dog_stick(&ctx, CHOICE[i].thr, CHOICE[i].turn, 0, 0)
                           == WEB_CMD_OK,
                       "choice case %u must be OK\n", i);
            char got[TRACE_TEXT_MAX];
            trace_text(got, sizeof(got));
            ++g_values;
            if (strcmp(got, CHOICE[i].want) != 0) {
                ++g_bad;
                ++g_seq_bad;
                printf("  CHOICE CASE MISMATCH (thr=%g turn=%d)\n    expected: %s\n    got     : %s\n",
                       (double)CHOICE[i].thr, CHOICE[i].turn, CHOICE[i].want, got);
            }
            for (int k = 0; k < s_trace_n; ++k) {
                if (s_trace[k].kind == CALL_MOVE) {
                    ++g_saw_move;
                }
                if (s_trace[k].kind == CALL_DRIVE) {
                    ++g_saw_drive;
                    ++g_contract_drive;
                }
            }
        }
    }

    /* [3] the clamp is a PRECONDITION of apply_dog_stick, and violating it is observable.
     *     The original would silently take the unclamped value into the predicates; this port
     *     refuses.  Proven here rather than assumed. */
    trace_reset();
    HOST_CHECK(web_cmd_apply_dog_stick(&ctx, -4.0f, 0, 0, 0) == WEB_CMD_ERR_THR_RANGE,
               "out-of-range thr must be refused\n");
    HOST_CHECK(s_trace_n == 0, "out-of-range thr must not emit any command\n");
    HOST_CHECK(web_cmd_apply_dog_stick(&ctx, 7.0f, 0, 0, 0) == WEB_CMD_ERR_THR_RANGE,
               "thr > 6.0 must be refused\n");
    HOST_CHECK(web_cmd_apply_dog_stick(&ctx, 6.0f, 0, 0, 0) == WEB_CMD_OK,
               "thr == 6.0 is inside the clamp\n");
    HOST_CHECK(web_cmd_apply_dog_stick(&ctx, -3.0f, 0, 0, 0) == WEB_CMD_OK,
               "thr == -3.0 is inside the clamp\n");

    /* [4] an incomplete context must be refused, not dereferenced. */
    web_cmd_ctx_t broken;
    web_cmd_ctx_init(&broken);
    broken.move = cb_move;      /* only one of the eight */
    HOST_CHECK(web_cmd_apply_dog_stick(&broken, 1.0f, 0, 0, 0) == WEB_CMD_ERR_INVALID_ARG,
               "incomplete ctx must be refused\n");
    HOST_CHECK(web_cmd_process_request(&broken, 50, 1, 0, 1, 0, 0, 0, NULL)
                   == WEB_CMD_ERR_INVALID_ARG,
               "incomplete ctx must be refused (process_request)\n");
    web_cmd_ctx_t null_ctx;
    web_cmd_ctx_init(&null_ctx);
    HOST_CHECK(web_cmd_apply_dog_stick(&null_ctx, 1.0f, 0, 0, 0) == WEB_CMD_ERR_INVALID_ARG,
               "all-NULL ctx must be refused\n");

    /* [5] the out_thr parameter carries the value through.  ⚠️ On the ignored path (no f= or
     *     no t=) the original returns the CACHED value untouched, so this port must NOT write
     *     out_thr either -- an ignored request must not be able to overwrite the caller's
     *     cache.  That is asserted, not assumed. */
    float thr_out = -123.0f;
    ctx = make_ctx();
    trace_reset();
    HOST_CHECK(web_cmd_process_request(&ctx, 50, 0, 0, 1, 0, 0, 0, &thr_out) == WEB_CMD_OK,
               "has_f == 0 must be OK (ignored)\n");
    HOST_CHECK(s_trace_n == 0, "has_f == 0 must emit nothing\n");
    HOST_CHECK(thr_out == -123.0f,
               "ignored request must leave out_thr UNTOUCHED (original returns thr_cache)\n");
    thr_out = -123.0f;
    trace_reset();
    HOST_CHECK(web_cmd_process_request(&ctx, 50, 1, 0, 0, 0, 0, 0, &thr_out) == WEB_CMD_OK,
               "has_t == 0 must be OK (ignored)\n");
    HOST_CHECK(s_trace_n == 0, "has_t == 0 must emit nothing\n");
    HOST_CHECK(thr_out == -123.0f, "has_t == 0 must leave out_thr UNTOUCHED\n");
    HOST_CHECK(web_cmd_process_request(&ctx, 50, 1, 0, 1, 0, 0, 0, &thr_out) == WEB_CMD_OK,
               "valid request must be OK\n");
    HOST_CHECK(thr_out == -3.0f, "out_thr must be the computed thr\n");

    /* [6] the inplace-step branch.  It is an INPUT here (the deadline comes from the action
     *     layer), so this is a table of injectable cases with their EXACT call sequences --
     *     copied from the reference values that the table of the same shape produced before the
     *     deadline turned out to be unreachable in the CSV rows (see the FINDING below). */
    {
        static const struct {
            const char *why;
            int32_t     deadline;
            int         value_f, turn, gait, crawl, force, arm;
            const char *want;
        } INPLACE[] = {
            { "TROT forward, deadline live",  6000, 50,  30, 0, 0, 0, 0,
              "sit:0;0|move:4;1;1" },
            { "zero stick, deadline live",    6000,  0,   0, 0, 0, 0, 0,
              "sit:0;0|move:4;1;1" },
            { "WALK, deadline live",          6000, 50,   0, 1, 0, 0, 0,
              "sit:0;0|move:4;1;1" },
            { "crawl active, deadline live",  6000, 50,   0, 0, 1, 0, 0,
              "sit:0;0|move:4;1;1" },
            { "arm on, deadline live",        6000, 50,   0, 0, 0, 0, 1,
              "sit:0;0|move:4;1;1" },
            { "deadline already passed",      1000, 50,  30, 0, 0, 0, 0,
              "sit:0;0|joy_turn:-30|turn_lr:-30;1;-1|move:2.5;1;-1" },
            { "deadline exactly now",         1000, 50,   0, 0, 0, 0, 0,
              "sit:0;0|joy_turn:0|move:-3;1;1" },
            { "no deadline (0 == false)",        0, 50,   0, 0, 0, 0, 0,
              "sit:0;0|joy_turn:0|move:-3;1;1" },
            { "no deadline, crawl active",       0, 50,   0, 0, 1, 0, 0,
              "sit:0;0" },
        };
        const int n_cases = (int)(sizeof(INPLACE) / sizeof(INPLACE[0]));
        for (int i = 0; i < n_cases; ++i) {
            HOST_CHECK(app_cfg_cmd_init() == APP_CFG_OK, "cfg init failed\n");
            HOST_CHECK(app_chain_init() == ESP_OK, "chain init failed\n");
            HOST_CHECK(app_action_init() == ESP_OK, "action init failed\n");
            HOST_CHECK(app_chain_set_gait(INPLACE[i].gait) == ESP_OK, "plant gait failed\n");
            HOST_CHECK(app_chain_set_crawl(INPLACE[i].crawl, 0, 0) == ESP_OK,
                       "plant crawl failed\n");
            HOST_CHECK(app_action_set_inplace_step_end_ms(INPLACE[i].deadline) == ESP_OK,
                       "inject deadline failed\n");
            trace_reset();
            (void)web_cmd_process_request(&ctx, INPLACE[i].value_f, 1,
                                          INPLACE[i].turn, 1, 1000,
                                          INPLACE[i].force, INPLACE[i].arm, NULL);
            char got[TRACE_TEXT_MAX];
            trace_text(got, sizeof(got));
            ++g_values;
            if (strcmp(got, INPLACE[i].want) != 0) {
                ++g_bad;
                ++g_seq_bad;
                printf("  INPLACE CASE MISMATCH (%s)\n    expected: %s\n    got     : %s\n",
                       INPLACE[i].why, INPLACE[i].want, got);
            }
            for (int k = 0; k < s_trace_n; ++k) {
                if (s_trace[k].kind == CALL_MOVE) {
                    ++g_saw_move;
                    if (s_trace[k].a == 4.0f && s_trace[k].l == 1 && s_trace[k].r == 1) {
                        ++g_saw_move4;
                        ++g_contract_move4;
                    }
                }
                if (s_trace[k].kind == CALL_DRIVE) {
                    ++g_saw_drive;
                    ++g_contract_drive;
                }
            }
        }
    }

    /* [6b] FINDING (reported, not papered over): the web layer has no way to CLEAR
     *      inplace_step_end_ms, so the "one frame only" property cannot hold from here.
     *
     *      The original's move() zeroed the module global, so the branch stopped firing after
     *      the first request.  In this port the deadline lives in the action layer
     *      (app_action_get/set_inplace_step_end_ms) and the chain's move path
     *      (control_chain_cmd_move -> set_spd_lr) clears only control_chain_state_t's OWN copy
     *      -- control/action.h:57 says that field belongs to the action layer, and nothing in the
     *      C tree writes the action layer's copy from the chain.  This asserts the CURRENT
     *      behaviour so a future fix cannot silently change it unnoticed; the divergence itself
     *      is for the P6 wiring to resolve, not for this file to hide. */
    {
        HOST_CHECK(app_chain_init() == ESP_OK, "chain init failed\n");
        HOST_CHECK(app_action_init() == ESP_OK, "action init failed\n");
        HOST_CHECK(app_action_set_inplace_step_end_ms(6000) == ESP_OK, "inject failed\n");
        trace_reset();
        HOST_CHECK(web_cmd_process_request(&ctx, 50, 1, 0, 1, 1000, 0, 0, NULL) == WEB_CMD_OK,
                   "inplace request must be OK\n");
        HOST_CHECK(s_trace_n == 2 && s_trace[1].kind == CALL_MOVE && s_trace[1].a == 4.0f,
                   "inplace branch must be sit + move(4,1,1)\n");
        printf("  FINDING: after move(4,1,1), app_action_get_inplace_step_end_ms() = %d "
               "(original's move() cleared it -> original's branch was one-frame-only)\n",
               (int)app_action_get_inplace_step_end_ms());
        HOST_CHECK(app_action_get_inplace_step_end_ms() == 6000,
                   "unexpected: the action layer's deadline changed on its own\n");
        printf("  FINDING -> the deadline is STILL %d after move(4,1,1).\n"
               "            web_common.py's branch was one-frame-only because padog.move()\n"
               "            cleared the module global; in C the deadline belongs to the ACTION\n"
               "            layer (control/action.h:57) and nothing in the tree clears it from\n"
               "            the chain, so this branch will keep firing while the page polls.\n"
               "            NOT fixed here: web_cmd.c is forbidden to touch existing sources.\n"
               "            Owner: the P6 wiring step (same task that calls\n"
               "            app_action_get_inplace_step_end_ms() for this module).\n",
               (int)app_action_get_inplace_step_end_ms());
        ++g_findings;
    }

    /* [7] crawl guard: nothing but set_leg_sit_offsets(0,0), in BOTH the dead-zone path and
     *     the apply_dog_stick path. */
    HOST_CHECK(app_cfg_cmd_init() == APP_CFG_OK, "cfg init failed\n");
    HOST_CHECK(app_chain_init() == ESP_OK, "chain init failed\n");
    HOST_CHECK(app_action_init() == ESP_OK, "action init failed\n");
    HOST_CHECK(app_chain_set_crawl(1, 0, 0) == ESP_OK, "plant crawl failed\n");
    trace_reset();
    (void)web_cmd_process_request(&ctx, 0, 1, 0, 1, 0, 0, 0, NULL);
    HOST_CHECK(s_trace_n == 1 && s_trace[0].kind == CALL_SIT,
               "crawl + dead zone must emit only set_leg_sit_offsets (got %d)\n", s_trace_n);
    trace_reset();
    (void)web_cmd_process_request(&ctx, 50, 1, 0, 1, 0, 0, 0, NULL);
    HOST_CHECK(s_trace_n == 1 && s_trace[0].kind == CALL_SIT,
               "crawl + stick must emit only set_leg_sit_offsets (got %d)\n", s_trace_n);
    HOST_CHECK(app_chain_crawl_reset() == ESP_OK, "crawl reset failed\n");

    /* [8] arm guard vs force. */
    trace_reset();
    (void)web_cmd_process_request(&ctx, 50, 1, 0, 1, 0, 0, 1, NULL);
    HOST_CHECK(s_trace_n == 1 && s_trace[0].kind == CALL_SIT,
               "arm on + !force must emit only set_leg_sit_offsets\n");
    trace_reset();
    (void)web_cmd_process_request(&ctx, 50, 1, 0, 1, 0, 1, 1, NULL);
    HOST_CHECK(s_trace_n == 3 && s_trace[2].kind == CALL_MOVE,
               "arm on + force must still drive the dog (dog_when_arm)\n");

    /* [9] unknown / missing callbacks are the ONE thing this module cannot guess: P6. */
    printf("---- contract checks done ----\n");
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
    printf(" web command translation  (web_common.py -> comm/web_cmd.c)\n");
    printf("========================================================\n");
    printf("golden file : %s\n", path);
    printf("reference   : real padog.py + real web_common.py in sys.modules\n");
    printf("compared    : ORDERED CALL SEQUENCE (n_calls + callseq), not just state\n");

    g_values = 0;
    g_bad = 0;
    g_shown = 0;
    g_rows = 0;
    g_seq_bad = 0;
    g_state_bad = 0;
    g_saw_move = 0;
    /* the contract-check pass already accumulated these; add to them instead of wiping them
     * (the inplace move(4,1,1) only exists in that pass) */
    g_saw_drive = g_contract_drive;
    g_saw_move4 = g_contract_move4;
    g_saw_blind = 0;
    g_saw_zero_cmd = 0;

    const web_cmd_ctx_t ctx = make_ctx();

    char line[LINE_MAX];
    while (fgets(line, sizeof(line), fh) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        double v[FIELDS_MAX];
        char want_seq[TRACE_TEXT_MAX];
        const int n = split_csv_line(line, v, FIELDS_MAX, want_seq, sizeof(want_seq));
        if (n != PRE_COUNT + POST_COUNT) {
            printf("  row %ld: %d fields, expected %d\n", g_rows, n,
                   PRE_COUNT + POST_COUNT);
            fclose(fh);
            return 2;
        }

        /* fresh chain state per row (the generator re-execs padog.py per row for the very
         * same reason) */
        HOST_CHECK(app_chain_init() == ESP_OK, "row %ld: app_chain_init failed\n", g_rows);
        plant(v);

        s_accessor_fail = 0;
        trace_reset();

        /* ---- the layer under test ---- */
        float thr_out = 0.0f;
        const web_cmd_status_t rc =
            web_cmd_process_request(&ctx,
                                    (int)v[PRE_VALUE_F], (int)v[PRE_HAS_F],
                                    (int)v[PRE_TURN], (int)v[PRE_HAS_T],
                                    (int32_t)v[PRE_NOW],
                                    (int)v[PRE_FORCE], (int)v[PRE_ARM],
                                    &thr_out);
        HOST_CHECK(rc == WEB_CMD_OK, "row %ld: unexpected status %d\n", g_rows, (int)rc);
        HOST_CHECK(!s_accessor_fail, "row %ld: an app_chain accessor failed\n", g_rows);
        HOST_CHECK(!s_trace_overflow, "row %ld: call trace overflowed\n", g_rows);

        /* ---- the post state ---- */
        double got[POST_COUNT];
        dump_post(got);
        /* float32 again (see dump_post): the CSV text is a float32 decimal, atof gives the
         * nearest double, and the C value is a float -- both sides must mean the same float32. */
        got[POST_THR] = (double)(float)thr_out;

        /* everything except the sequence: plain values, exact (no epsilon anywhere) */
        for (int i = 0; i < POST_N_CALLS; ++i) {
            note(g_rows, "post_state", got[i], (double)(float)v[PRE_COUNT + i], "state");
        }

        /* ---- and the part that actually matters: the ordered call sequence ----
         * The seq column lives at index PRE_COUNT + POST_CALLSEQ in the file, but it was
         * already pulled out as text by split_csv_line() (its own commas would break the
         * numeric parse).  Its value is compared as a string. */
        char got_seq[TRACE_TEXT_MAX];
        trace_text(got_seq, sizeof(got_seq));
        ++g_values;
        if (strcmp(got_seq, want_seq) != 0) {
            ++g_bad;
            ++g_seq_bad;
            if (g_shown < 8) {
                ++g_shown;
                printf("  FIRST SEQ MISMATCH: row=%ld\n    expected: %s\n    got     : %s\n",
                       g_rows, want_seq, got_seq);
            }
        }

        /* path coverage accounting (P-18): if the table stopped exercising move-vs-drive or
         * the "only sit offsets" paths, this suite would silently lose its teeth. */
        for (int i = 0; i < s_trace_n; ++i) {
            if (s_trace[i].kind == CALL_DRIVE) {
                ++g_saw_drive;
            }
            if (s_trace[i].kind == CALL_MOVE) {
                ++g_saw_move;
                if (s_trace[i].a == 4.0f && s_trace[i].l == 1 && s_trace[i].r == 1 &&
                    s_trace[i].b == 0.0f) {
                    ++g_saw_move4;
                }
            }
        }
        if (s_trace_n == 1 && s_trace[0].kind == CALL_SIT) {
            ++g_saw_blind;
        }
        if (s_trace_n == 0) {
            ++g_saw_zero_cmd;
        }

        ++g_rows;
    }
    fclose(fh);

    printf("\nrows        : %ld\n", g_rows);
    printf("checks      : %ld\n", g_values);
    printf("failures    : %ld  (state %ld / callseq %ld)\n", g_bad, g_state_bad, g_seq_bad);
    printf("path coverage: move=%ld drive=%ld move(4,1,1)=%ld sit-only=%ld no-command=%ld\n",
           g_saw_move, g_saw_drive, g_saw_move4, g_saw_blind, g_saw_zero_cmd);
    if (g_findings > 0) {
        printf("FINDINGS    : %ld (see the FINDING block above -- reported, not hidden)\n",
               g_findings);
    }

    /* ---- coverage assertions, so a degenerate table cannot pass silently (P-18) ---- */
    HOST_CHECK(g_rows == 48, "expected 48 golden rows, got %ld\n", g_rows);
    HOST_CHECK(g_saw_move > 0, "the table never exercised move()\n");
    HOST_CHECK(g_saw_drive > 0,
               "the table never exercised drive() -- WALK cannot be told from TROT (P-26)\n");
    /* NOTE: `g_saw_move4` is fed by the INPLACE[] table, not by the CSV -- the inplace branch
     * needs a deadline the CSV cannot carry (see the generator and the [6b] FINDING). */
    HOST_CHECK(g_saw_move4 > 0, "the inplace move(4,1,1) was never exercised\n");
    HOST_CHECK(g_saw_blind > 0,
               "the table never exercised a sit-offsets-only path (guard) -- P-29 territory\n");
    HOST_CHECK(g_saw_zero_cmd > 0,
               "the table never exercised 'both f and t required' (nothing emitted)\n");

    if (g_bad == 0) {
        printf("WEB_CMD OK\n");
    }
    return (g_bad == 0) ? 0 : 1;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: %s golden/web_cmd.csv\n", argv[0]);
        return 2;
    }
    contract_checks();

    int rc = run_csv(argv[1]);

    printf("\ntotal checks: %d, failures: %d\n", g_host_checks, g_host_fail);
    HOST_CHECK(g_host_fail == 0, "host assertions failed: %d\n", g_host_fail);
    if (rc != 0 || g_host_fail != 0) {
        printf("*** WEB_CMD TEST FAILED ***\n");
        return 1;
    }
    printf("*** WEB_CMD TEST PASSED ***\n");
    return 0;
}
