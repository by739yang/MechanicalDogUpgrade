/**
 * @file    test_cmd_queue.c
 * @brief   单槽命令邮箱 + 两级超时策略的宿主测试（check 式，无 golden CSV）
 *
 * 这是 P5 验收标准「断连自动停车」的落点，所以测试要**对抗性**地打边界：
 * 新鲜度的**分界值本身**（`age == hb` 与 `age == hb+1`）、序号回绕、
 * 急停能不能被一次"读不到"就解除、进入 HOLD/RELAX 的次数对不对、
 * 以及在**不该写输出**的时候输出有没有被写。
 *
 * ⚠️ 本项目已有 P-18 的教训：**测不出差别的输入等于没测**。
 * 所以下面每条边界都同时断言"这一侧"和"另一侧"，不允许只测一侧。
 */

#include <stdio.h>
#include <string.h>

#include "comm/cmd_queue.h"

static int g_fail = 0;
static int g_checks = 0;

static int check(int cond, const char *msg)
{
    ++g_checks;
    if (!cond) {
        ++g_fail;
        printf("  FAIL: %s\n", msg);
    }
    return cond;
}

#define SECTION(t) do { printf("\n-- %s\n", (t)); } while (0)

/*
 * 造一条命令：seq + est，其余全零。
 * 用**复合字面量**取地址（C99 起合法）；`proto_cmd_zero()` 就是全零，
 * 所以未指定的成员由 C 保证清零，两者等价。
 * （写成 `MK(...)` 取函数返回值的地址在 C 里不合法。）
 */
/* ⚠️ 宏参数名**不能**叫 `seq`/`est` —— 那会把成员名 `.seq` 也一起替换掉
 *    （预处理器不区分类成员和参数），报错是 "expected identifier before numeric constant"。 */
#define MK(s_, e_) (&(proto_cmd_t){ .seq = (s_), .est = (e_) })

/* ==========================================================================
 * 1. 初始化与阈值
 * ======================================================================== */

static void test_init(void)
{
    SECTION("init: thresholds and the 'HOLD must be reachable' invariant");

    cmd_queue_t q;

    cmd_queue_init(&q, 0, 0);
    check(q.hb_ms == CMD_QUEUE_DEFAULT_HB_MS, "hb_ms==0 should take the default");
    check(q.long_ms > q.hb_ms, "long_ms must exceed hb_ms");

    cmd_queue_init(&q, 250, 2000);
    check(q.hb_ms == 250, "explicit hb_ms kept");
    check(q.long_ms == 2000, "explicit long_ms kept");

    /* P-18: long<=hb 会让 HOLD **永远不可达**，那样两级策略就是假的 */
    cmd_queue_init(&q, 500, 100);
    check(q.hb_ms == 500, "hb kept when long is invalid");
    check(q.long_ms > q.hb_ms, "invalid long must be repaired, not accepted");

    cmd_queue_init(&q, 300, 300);
    check(q.long_ms > q.hb_ms, "long == hb must be repaired");

    cmd_queue_init(&q, 250, 2000);
    check(cmd_queue_set_timeouts(&q, 100, 900), "set_timeouts valid -> true");
    check(q.hb_ms == 100 && q.long_ms == 900, "set_timeouts applied");
    check(!cmd_queue_set_timeouts(&q, 0, 900), "hb==0 rejected");
    check(!cmd_queue_set_timeouts(&q, 900, 100), "long<hb rejected");
    check(!cmd_queue_set_timeouts(&q, 300, 300), "long==hb rejected");
    check(q.hb_ms == 100 && q.long_ms == 900, "rejected set must not change values");

    /* 空指针不许崩 */
    cmd_queue_init(NULL, 1, 2);
    check(cmd_queue_tick(NULL, 0, NULL, NULL) == CMD_QUEUE_IDLE, "tick(NULL) -> IDLE");
    check(!cmd_queue_post(NULL, NULL, 0), "post(NULL) -> false");
    cmd_queue_count_reject(NULL);
    cmd_queue_latch_estop(NULL, true);
    cmd_queue_clear_estop(NULL);
    cmd_queue_get_counters(NULL, NULL);
    check(cmd_queue_age_ms(NULL, 0) == UINT32_MAX, "age(NULL) -> UINT32_MAX");
    check(1, "null-safety pass");
}

/* ==========================================================================
 * 2. 新鲜度分界（两侧都测）
 * ======================================================================== */

static void test_freshness(void)
{
    SECTION("freshness boundaries: age == hb and age == hb+1, long and long+1");

    const uint32_t HB = 250, LG = 2000;
    cmd_queue_t q;
    proto_cmd_t out;
    bool changed;

    cmd_queue_init(&q, HB, LG);

    /* 从没收到过 -> IDLE，而且**不许碰输出** */
    memset(&out, 0x5A, sizeof(out));
    check(cmd_queue_tick(&q, 0, &out, &changed) == CMD_QUEUE_IDLE, "no command -> IDLE");
    check(!changed, "IDLE at start is not a transition (already IDLE)");
    {
        const unsigned char *p = (const unsigned char *)&out;
        int all_5a = 1;
        for (size_t i = 0; i < sizeof(out); ++i) {
            if (p[i] != 0x5A) { all_5a = 0; break; }
        }
        check(all_5a, "IDLE must not write to *out");
    }

    check(cmd_queue_age_ms(&q, 0) == UINT32_MAX, "age is UINT32_MAX before any command");

    /* t=1000 收到 */
    check(cmd_queue_post(&q, MK(1, 0), 1000), "first command accepted");
    check(cmd_queue_age_ms(&q, 1000) == 0, "age 0 right after receive");

    check(cmd_queue_tick(&q, 1000, &out, &changed) == CMD_QUEUE_FRESH, "age 0 -> FRESH");
    check(changed, "IDLE->FRESH is a transition");
    check(out.seq == 1, "output carries the posted command");

    check(cmd_queue_tick(&q, 1000 + HB, &out, &changed) == CMD_QUEUE_FRESH,
          "age == hb is still FRESH (inclusive)");
    check(cmd_queue_tick(&q, 1000 + HB + 1, &out, &changed) == CMD_QUEUE_HOLD,
          "age == hb+1 -> HOLD");
    check(changed, "FRESH->HOLD is a transition");

    check(cmd_queue_tick(&q, 1000 + LG, &out, &changed) == CMD_QUEUE_HOLD,
          "age == long is still HOLD (inclusive)");
    check(!changed, "staying in HOLD is not a transition");
    check(cmd_queue_tick(&q, 1000 + LG + 1, &out, &changed) == CMD_QUEUE_RELAX,
          "age == long+1 -> RELAX");
    check(changed, "HOLD->RELAX is a transition");
    check(cmd_queue_tick(&q, 1000 + LG + 1, &out, &changed) == CMD_QUEUE_RELAX,
          "RELAX stays RELAX");
    check(!changed, "RELAX->RELAX is not a transition");

    /* 再来一条命令 -> 立刻回到 FRESH（这就是"重连恢复"） */
    check(cmd_queue_post(&q, MK(2, 0), 1000 + LG + 500), "command after a gap accepted");
    check(cmd_queue_tick(&q, 1000 + LG + 500, &out, &changed) == CMD_QUEUE_FRESH,
          "a fresh command returns the state to FRESH");
    check(changed, "RELAX->FRESH is a transition");
    check(out.seq == 2, "output updated to the newer command");
}

/* ==========================================================================
 * 3. seq 顺序（回绕安全）
 * ======================================================================== */

static void test_seq(void)
{
    SECTION("seq: strictly newer accepted, equal/older dropped, wrap-around accepted");

    cmd_queue_t q;
    cmd_queue_init(&q, 250, 2000);

    check(cmd_queue_post(&q, MK(10, 0), 0), "seq 10 accepted on empty mailbox");
    check(!cmd_queue_post(&q, MK(10, 0), 1), "duplicate seq dropped");
    check(!cmd_queue_post(&q, MK(9, 0), 2), "older seq dropped");
    check(cmd_queue_post(&q, MK(11, 0), 3), "newer seq accepted");

    cmd_queue_counters_t c;
    cmd_queue_get_counters(&q, &c);
    check(c.accepted == 2, "accepted counted");
    check(c.dropped_seq == 2, "both drops counted");

    /* 回绕：0xFFFFFFFF -> 0 必须接受（用无符号比较写 `a > b` 会在这里判错） */
    cmd_queue_t w;
    cmd_queue_init(&w, 250, 2000);
    check(cmd_queue_post(&w, MK(0xFFFFFFFEu, 0), 0), "near-max seq accepted");
    check(cmd_queue_post(&w, MK(0xFFFFFFFFu, 0), 1), "max seq accepted");
    check(cmd_queue_post(&w, MK(0u, 0), 2), "wrap 0xFFFFFFFF -> 0 accepted");
    check(cmd_queue_post(&w, MK(1u, 0), 3), "seq 1 after wrap accepted");
    check(!cmd_queue_post(&w, MK(0xFFFFFFFFu, 0), 4), "post-wrap stale (max) dropped");

    /* 未收到过命令时，任何 seq 都接受（不能因为"还没基准"就丢第一条） */
    cmd_queue_t f;
    cmd_queue_init(&f, 250, 2000);
    check(cmd_queue_post(&f, MK(0u, 0), 0), "first command with seq 0 accepted");
}

/* ==========================================================================
 * 4. 急停
 * ======================================================================== */

static void test_estop(void)
{
    SECTION("estop: latches, overrides freshness, survives a peek that reads nothing");

    cmd_queue_t q;
    proto_cmd_t out;
    bool changed;
    cmd_queue_init(&q, 250, 2000);

    check(cmd_queue_post(&q, MK(1, 0), 0), "normal command accepted");
    check(cmd_queue_tick(&q, 0, &out, &changed) == CMD_QUEUE_FRESH, "FRESH before estop");

    /* 一条**合法**帧带 est=1 */
    check(cmd_queue_post(&q, MK(2, 1), 10), "frame with est=1 accepted");
    check(cmd_queue_tick(&q, 10, &out, &changed) == CMD_QUEUE_RELAX,
          "estop forces RELAX even though the frame is fresh");
    check(changed, "FRESH->RELAX transition reported");
    check(out.est == 1, "output carries est=1");
    check(out.spd == 0 && out.turn == 0 && out.mode == 0,
          "estop output is zeroed (caller cannot act on stale values)");

    /* 时间流逝 + 新命令都不许解除急停 */
    check(cmd_queue_post(&q, MK(3, 0), 5000), "later command accepted");
    check(cmd_queue_tick(&q, 5000, &out, &changed) == CMD_QUEUE_RELAX,
          "a later non-estop frame must NOT clear the latch");
    check(out.est == 1, "output still est=1 after a normal frame");

    /* 从被判废的帧里 peek 出来的急停：只会往安全侧失败 */
    cmd_queue_t p;
    cmd_queue_init(&p, 250, 2000);
    cmd_queue_latch_estop(&p, true);
    check(cmd_queue_tick(&p, 0, &out, &changed) == CMD_QUEUE_RELAX,
          "peeked estop applies even with no accepted command at all");
    cmd_queue_latch_estop(&p, false);       /* 读不到 -> 不许解除 */
    check(cmd_queue_tick(&p, 0, &out, &changed) == CMD_QUEUE_RELAX,
          "latch_estop(false) must not clear the latch");
    cmd_queue_clear_estop(&p);              /* 只有显式清除才行 */
    check(cmd_queue_tick(&p, 0, &out, &changed) == CMD_QUEUE_IDLE,
          "explicit clear returns to IDLE (no command was ever accepted)");

    cmd_queue_counters_t c;
    cmd_queue_get_counters(&q, &c);
    check(c.estop_latched == 1, "one estop indication counted");
    check(c.relax_entries == 1, "RELAX entered once (not once per tick)");
}

/* ==========================================================================
 * 5. 计数与"断连自动停车"场景
 * ======================================================================== */

static void test_disconnect_scenario(void)
{
    SECTION("scenario: 50 Hz polling for 1 s, then the link dies");

    cmd_queue_t q;
    proto_cmd_t out;
    bool changed;
    cmd_queue_init(&q, CMD_QUEUE_DEFAULT_HB_MS, CMD_QUEUE_DEFAULT_LONG_MS);

    uint32_t now = 0;
    uint32_t seq = 1;
    int hold_seen = 0, relax_seen = 0;

    /* 1 秒 @ 50 Hz：一直 FRESH */
    for (int i = 0; i < 50; ++i) {
        check(cmd_queue_post(&q, MK(seq++, 0), now), "polling frame accepted");
        const cmd_queue_state_t st = cmd_queue_tick(&q, now, &out, &changed);
        check(st == CMD_QUEUE_FRESH, "state is FRESH while polling");
        now += 20;
    }

    /* 断连：时间往前走，不再投递 */
    for (int i = 0; i < 300; ++i) {
        now += 10;
        const cmd_queue_state_t st = cmd_queue_tick(&q, now, &out, &changed);
        if (st == CMD_QUEUE_HOLD && changed) { ++hold_seen; }
        if (st == CMD_QUEUE_RELAX && changed) { ++relax_seen; }
    }
    check(hold_seen == 1, "entered HOLD exactly once");
    check(relax_seen == 1, "entered RELAX exactly once");

    cmd_queue_counters_t c;
    cmd_queue_get_counters(&q, &c);
    check(c.accepted == 50, "50 commands accepted");
    check(c.dropped_seq == 0, "no sequence drops during normal polling");
    check(c.rejected == 0, "no rejects");
    check(c.hold_entries == 1, "hold_entries == 1");
    check(c.relax_entries == 1, "relax_entries == 1");
    check(c.ticks == 50 + 300, "ticks counted");

    /* 重连：状态回 FRESH，且**不再**重复计 HOLD */
    check(cmd_queue_post(&q, MK(seq++, 0), now), "reconnect frame accepted");
    check(cmd_queue_tick(&q, now, &out, &changed) == CMD_QUEUE_FRESH, "reconnect -> FRESH");
    cmd_queue_get_counters(&q, &c);
    check(c.hold_entries == 1, "reconnect does not double-count HOLD");
    check(c.relax_entries == 1, "reconnect does not double-count RELAX");

    cmd_queue_count_reject(&q);
    cmd_queue_count_reject(&q);
    cmd_queue_get_counters(&q, &c);
    check(c.rejected == 2, "protocol rejects are counted separately");
}

/* ==========================================================================
 * 6. 49.7 天回绕
 * ======================================================================== */

static void test_time_wrap(void)
{
    SECTION("now_ms wrap-around must not be mistaken for a huge age");

    cmd_queue_t q;
    proto_cmd_t out;
    bool changed;
    cmd_queue_init(&q, 250, 2000);

    const uint32_t near_max = 0xFFFFFFFFu - 100u;   /* 收到命令 */
    check(cmd_queue_post(&q, MK(1, 0), near_max), "command posted just before wrap");

    /* 跨过回绕点 150 ms -> 仍然是 FRESH（这正是无符号相减的意义） */
    const uint32_t after = near_max + 150u;         /* 自动回绕 */
    check(cmd_queue_age_ms(&q, after) == 150u, "age across the wrap point is 150");
    check(cmd_queue_tick(&q, after, &out, &changed) == CMD_QUEUE_FRESH,
          "still FRESH across the wrap");

    check(cmd_queue_age_ms(&q, near_max + 2000u) == 2000u, "age 2000 across wrap");
    check(cmd_queue_tick(&q, near_max + 2000u, &out, &changed) == CMD_QUEUE_HOLD,
          "HOLD at 2000 across the wrap");
    check(cmd_queue_tick(&q, near_max + 2001u, &out, &changed) == CMD_QUEUE_RELAX,
          "RELAX at 2001 across the wrap");
}

/* ==========================================================================
 * 7. 状态名（只有一处，别写两份 —— P-27）
 * ======================================================================== */

static void test_names(void)
{
    SECTION("state names: one source, all distinct, unknown rejected");

    const char *names[4];
    names[0] = cmd_queue_state_name(CMD_QUEUE_IDLE);
    names[1] = cmd_queue_state_name(CMD_QUEUE_FRESH);
    names[2] = cmd_queue_state_name(CMD_QUEUE_HOLD);
    names[3] = cmd_queue_state_name(CMD_QUEUE_RELAX);

    for (int i = 0; i < 4; ++i) {
        check(names[i] != NULL && names[i][0] != '\0', "name is non-empty");
        for (int j = i + 1; j < 4; ++j) {
            check(strcmp(names[i], names[j]) != 0, "names are distinct");
        }
    }
    check(strcmp(names[0], "IDLE") == 0, "IDLE name");
    check(strcmp(names[1], "FRESH") == 0, "FRESH name");
    check(strcmp(names[2], "HOLD") == 0, "HOLD name");
    check(strcmp(names[3], "RELAX") == 0, "RELAX name");
    check(strcmp(cmd_queue_state_name((cmd_queue_state_t)99), "?") == 0,
          "unknown state -> \"?\" (must not read out of bounds)");
}

int main(void)
{
    printf("============================================================\n");
    printf(" cmd_queue: single-slot mailbox, freshness, two-stage timeout\n");
    printf("============================================================\n");

    test_init();
    test_freshness();
    test_seq();
    test_estop();
    test_disconnect_scenario();
    test_time_wrap();
    test_names();

    printf("\n============================================================\n");
    printf(" checks=%d  failures=%d\n", g_checks, g_fail);
    if (g_fail > 0) {
        printf("RESULT: FAIL -- %d checks failed\n", g_fail);
        return 1;
    }
    printf("RESULT: PASS -- all %d checks passed\n", g_checks);
    printf("\nNOTE: this proves the POLICY only.  It cannot prove that the board's\n");
    printf("      WiFi or HTTP server actually delivers frames (that needs the board).\n");
    return 0;
}
