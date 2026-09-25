/*
 * test_proto.c —— 宿主侧测试：机器人命令协议（firmware/src/comm/proto.c）
 *
 * 关键点：proto.c **不依赖 ESP-IDF**，时钟由调用方传进来。所以这里用
 * `host_stubs/` 的**可控时钟**（`esp_timer_get_time()` 返回模拟时间）就能把
 * 心跳超时、限速、序号回绕这些"依赖时间"的逻辑确定性地测完，不用烧板子。
 *
 * 这一套刻意**不是** happy path：
 *   - 每个字段都做"min / max / 内部值"往返，并断言 **min-1 与 max+1 被拒**
 *     （P-18：测不出区别的测试等于没测）；
 *   - 每个字段单独越界时，断言**输出结构体一个字节都没变**（拒绝是全有或全无）；
 *   - 每个字段缺失/重复/取值非法时，断言报错**点名了那个字段**；
 *   - 有效帧的**每一个前缀长度**都必须被拒（截断不能产生半条命令）；
 *   - 固定种子的 fuzz：随机字节 + 合法帧的随机变异，必须不崩、不污染输出、
 *     同一个输入给同一个判定（可复现）；
 *   - 旧页面（drive.html / control.html）的报文格式必须能真的吃下去 ——
 *     那是这一阶段**能不能上板验收**的前提。
 *
 * 表驱动：所有用例的输入都从 `proto_fields()` / `proto_events()` 现算，
 * 不在这里抄第二份键名或范围（P-22/P-27）。
 *
 * NOTE: ASCII output only, for the reason recorded as P-06 in 问题与解决记录.md.
 */
/* mingw 默认走 msvcrt 的 printf，不认 %lld（它要 %I64d）。这一行让 mingw-w64 用
 * 自家 ANSI stdio，于是 %lld 在宿主测试里是标准写法，也能过 -Werror=format。 */
#if defined(__MINGW32__) && !defined(__USE_MINGW_ANSI_STDIO)
#define __USE_MINGW_ANSI_STDIO 1
#endif

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "comm/proto.h"
#include "esp_timer.h"   /* host_stubs：esp_timer_get_time() = 模拟时间（微秒）*/

/* ==========================================================================
 * 极简测试框架
 * ========================================================================== */

static int g_checks = 0;
static int g_fail = 0;
static int g_sec_checks = 0;
static int g_sec_fail = 0;
static const char *g_sec = "";

static void section(const char *name)
{
    if (g_sec[0] != '\0') {
        printf("  [%s] checks=%d fail=%d\n", g_sec, g_sec_checks, g_sec_fail);
    }
    g_sec = name;
    g_sec_checks = 0;
    g_sec_fail = 0;
    printf("\n-- %s\n", name);
}

static int vchk(int cond, const char *fmt, ...)
{
    ++g_checks;
    ++g_sec_checks;
    if (!cond) {
        va_list ap;
        ++g_fail;
        ++g_sec_fail;
        printf("  FAIL [%s]: ", g_sec);
        va_start(ap, fmt);
        vprintf(fmt, ap);
        va_end(ap);
        printf("\n");
    }
    return cond;
}

#define check(cond, msg) vchk((cond), "%s", (msg))

/* ==========================================================================
 * 可控时钟（host_stubs/esp_timer.h -> host_sim.c）
 * ========================================================================== */

static uint32_t t_now(void)
{
    return (uint32_t)((uint64_t)esp_timer_get_time() / 1000u);
}

/** 把模拟时钟推到 target_ms（只能前进）；回绕测试靠这个跳到接近 2^32 的地方 */
static void t_clock_goto(uint32_t target_ms)
{
    const int64_t now_ms = esp_timer_get_time() / 1000;
    const int64_t delta = (int64_t)target_ms - now_ms;
    if (delta > 0) {
        host_advance_us(delta * 1000);
    }
}

static void t_advance_ms(uint32_t ms)
{
    host_advance_us((int64_t)ms * 1000);
}

/* ==========================================================================
 * 通用辅助
 * ========================================================================== */

#define FRAME_CAP 1200u

_Static_assert(sizeof(proto_cmd_t) == 16u * sizeof(uint32_t),
               "proto_cmd_t must have no padding (the tests memcmp it)");

static const proto_field_t *g_fs = NULL;
static size_t g_nf = 0;
static const char *const *g_ev = NULL;
static size_t g_ne = 0;

static void i64_str(int64_t v, char *out)
{
    sprintf(out, "%lld", (long long)v);
}

/** 上限由 proto_fields() 给出：不在这里抄字段个数 */
static void load_tables(void)
{
    g_fs = proto_fields(&g_nf);
    g_ev = proto_events(&g_ne);
}

static int field_index(const char *key)
{
    return proto_field_find(key);
}

/** 一帧合法的样板命令（走 proto_cmd_defaults，不手写键值）*/
static void mk_base(proto_cmd_t *cmd)
{
    proto_cmd_defaults(cmd);
}

static size_t mk_canon(const proto_cmd_t *cmd, char *buf, size_t cap)
{
    return proto_encode(cmd, buf, cap);
}

/** 强制写入越界值（绕开 proto_cmd_set 的范围校验），用来测"编码器也查表" */
static void force_store(proto_cmd_t *cmd, size_t idx, int64_t v)
{
    const proto_field_t *f = &g_fs[idx];
    char *p = (char *)cmd + f->offset;
    if (f->is_unsigned) {
        uint32_t u = (uint32_t)v;
        memcpy(p, &u, sizeof(u));
    } else {
        int32_t s = (int32_t)v;
        memcpy(p, &s, sizeof(s));
    }
}

/** 找 key=value 在规范帧里的位置（只认键边界，避免命中 hgt= 里的 t=）*/
static int find_span(const char *frame, const char *key, size_t *ks, size_t *vs, size_t *ve)
{
    const size_t klen = strlen(key);
    const char *p = frame;

    while (*p != '\0') {
        if ((p == frame || p[-1] == ';') && strncmp(p, key, klen) == 0 && p[klen] == '=') {
            size_t s = (size_t)(p - frame) + klen + 1u;
            size_t e = s;
            while (frame[e] != '\0' && frame[e] != ';' && frame[e] != '\n') {
                ++e;
            }
            *ks = (size_t)(p - frame);
            *vs = s;
            *ve = e;
            return 1;
        }
        ++p;
    }
    return 0;
}

/** 把 key= 的取值换成别的文本（其余字节不动）*/
static int splice_value(char *frame, size_t cap, const char *key, const char *val)
{
    char tmp[FRAME_CAP];
    size_t ks, vs, ve;
    size_t n = 0;

    if (!find_span(frame, key, &ks, &vs, &ve)) {
        return 0;
    }
    if (vs > sizeof(tmp)) {
        return 0;
    }
    memcpy(tmp, frame, vs);
    n = vs;
    n += (size_t)snprintf(tmp + n, sizeof(tmp) - n, "%s", val);
    n += (size_t)snprintf(tmp + n, sizeof(tmp) - n, "%s", frame + ve);
    if (n + 1u > cap || n >= sizeof(tmp)) {
        return 0;
    }
    memcpy(frame, tmp, n + 1u);
    return 1;
}

/** 把规范帧里某个字段的**键名**换成别的文本（取值不动）*/
static int splice_key(char *frame, size_t cap, const char *key, const char *newkey)
{
    char tmp[FRAME_CAP];
    size_t ks, vs, ve;
    size_t n = 0;

    if (!find_span(frame, key, &ks, &vs, &ve)) {
        return 0;
    }
    memcpy(tmp, frame, ks);
    n = ks;
    n += (size_t)snprintf(tmp + n, sizeof(tmp) - n, "%s", newkey);
    n += (size_t)snprintf(tmp + n, sizeof(tmp) - n, "%s", frame + ks + strlen(key));
    if (n + 1u > cap || n >= sizeof(tmp)) {
        return 0;
    }
    memcpy(frame, tmp, n + 1u);
    return 1;
}

/** 删掉整个 key=value（连带一个分隔符）*/
static int remove_field(char *frame, size_t cap, const char *key)
{
    char tmp[FRAME_CAP];
    size_t ks, vs, ve;
    size_t s, e;
    size_t n;

    if (!find_span(frame, key, &ks, &vs, &ve)) {
        return 0;
    }
    s = ks;
    e = ve;
    if (s > 0u && frame[s - 1u] == ';') {
        s--;
    } else if (frame[e] == ';') {
        e++;
    }
    memcpy(tmp, frame, s);
    n = s;
    n += (size_t)snprintf(tmp + n, sizeof(tmp) - n, "%s", frame + e);
    if (n + 1u > cap || n >= sizeof(tmp)) {
        return 0;
    }
    memcpy(frame, tmp, n + 1u);
    return 1;
}

/** 把 key=value 再追加一份（键序无关，所以追加是合法做法）*/
static int dup_field(char *frame, size_t cap, const char *key)
{
    char tmp[FRAME_CAP];
    size_t ks, vs, ve;
    size_t nl;
    size_t n;

    if (!find_span(frame, key, &ks, &vs, &ve)) {
        return 0;
    }
    nl = strlen(frame);
    while (nl > 0u && frame[nl - 1u] != '\n') {
        nl--;
    }
    if (nl == 0u) {
        return 0;
    }
    nl--;                                   /* nl = LF 的下标 */
    memcpy(tmp, frame, nl);
    n = nl;
    tmp[n++] = ';';
    memcpy(tmp + n, frame + ks, ve - ks);
    n += ve - ks;
    memcpy(tmp + n, frame + nl, strlen(frame) - nl + 1u);
    n += strlen(frame) - nl;
    if (n + 1u > cap || n >= sizeof(tmp)) {
        return 0;
    }
    memcpy(frame, tmp, n + 1u);
    return 1;
}

/* ---- 解码便捷入口：每个用例用全新解码器，判定与历史状态无关 ---- */

static int dec_strict(const char *buf, size_t len, proto_cmd_t *out, proto_error_t *err)
{
    proto_decoder_t d;
    proto_decoder_init(&d);
    return proto_decode(&d, buf, len, 1000u, out, err);
}

static int dec_legacy(const char *buf, size_t len, proto_cmd_t *out, proto_error_t *err)
{
    proto_decoder_t d;
    proto_decoder_init(&d);
    return proto_decode_legacy(&d, buf, len, 1000u, out, err);
}

/* ---- 输出结构体的"没被动过"检查 ---- */

#define CANARY 0xA5

static void poison(proto_cmd_t *c)
{
    memset(c, CANARY, sizeof(*c));
}

/** 拒绝必须是全有或全无：拿一份毒化快照和调用后的内存逐字节比 */
static int untouched(proto_cmd_t *out, const proto_cmd_t *snap)
{
    return memcmp(out, snap, sizeof(*snap)) == 0;
}

/* ==========================================================================
 * 0. 表本身：唯一一份、没有重复、范围放得进存储
 * ========================================================================== */

static void t_table(void)
{
    size_t i, j;

    section("table: single source, no duplicates, ranges fit storage");
    load_tables();

    check(g_nf >= 10 && g_ne >= 10, "tables are not empty");
    printf("  fields=%u events=%u sizeof(proto_cmd_t)=%u\n",
           (unsigned)g_nf, (unsigned)g_ne, (unsigned)sizeof(proto_cmd_t));

    for (i = 0; i < g_nf; ++i) {
        const proto_field_t *f = &g_fs[i];

        check(f->key != NULL && f->key[0] != '\0', "field key non-empty");
        check(strlen(f->key) <= PROTO_MAX_KEY, "field key fits PROTO_MAX_KEY");
        check(f->scale != NULL && f->scale[0] != '\0', "field has an ASCII scale note");
        check(f->min <= f->max, "field min <= max");
        check(f->offset + sizeof(uint32_t) <= sizeof(proto_cmd_t), "field offset inside struct");
        /* limits 必须放得进存储类型，否则 store_field 会截断出一个"看起来合法"的值 */
        if (f->is_unsigned) {
            check(f->min >= 0 && f->max <= 4294967295LL,
                  "unsigned field limits fit uint32_t");
        } else {
            check(f->min >= -2147483648LL && f->max <= 2147483647LL,
                  "signed field limits fit int32_t");
        }
        if (f->kind == PROTO_KIND_INT) {
            if (f->role == PROTO_ROLE_MAGIC) {
                check(f->min == f->max, "magic/version field has exactly one allowed value");
            } else {
                check(f->min < f->max, "numeric field has a non-degenerate range (P-18)");
            }
        }
        for (j = i + 1; j < g_nf; ++j) {
            check(strcmp(f->key, g_fs[j].key) != 0, "no duplicate canonical key");
        }
        if (f->legacy != NULL) {
            check(strlen(f->legacy) <= PROTO_MAX_KEY, "legacy key fits PROTO_MAX_KEY");
            for (j = 0; j < g_nf; ++j) {
                const char *other = (j == i) ? NULL : g_fs[j].legacy;
                if (other != NULL) {
                    check(strcmp(f->legacy, other) != 0, "no duplicate legacy key");
                }
            }
        }
        printf("  %-5s legacy=%-5s kind=%s req=%s range=[%lld,%lld] %s\n",
               f->key, (f->legacy != NULL) ? f->legacy : "-",
               (f->kind == PROTO_KIND_INT) ? "int" : "ev ",
               (f->required == PROTO_REQ_STRICT) ? "yes" : "no ",
               (long long)f->min, (long long)f->max, f->scale);
    }

    /* 三个特殊角色各**恰好一个** —— 解析器靠角色认它们，角色重复 = 语义歧义 */
    {
        const proto_role_t roles[3] = { PROTO_ROLE_MAGIC, PROTO_ROLE_ESTOP, PROTO_ROLE_SEQ };
        static const char *const rn[3] = { "magic", "estop", "seq" };
        for (i = 0; i < 3; ++i) {
            int hits = 0;
            for (j = 0; j < g_nf; ++j) {
                if (g_fs[j].role == roles[i]) {
                    ++hits;
                }
            }
            vchk(hits == 1, "exactly one field has role %s (found %d)", rn[i], hits);
        }
    }

    /* 事件名：非空、合法、唯一、唯一一份 */
    for (i = 0; i < g_ne; ++i) {
        check(g_ev[i] != NULL && g_ev[i][0] != '\0', "event name non-empty");
        check(strlen(g_ev[i]) <= PROTO_MAX_EVENT, "event name fits PROTO_MAX_EVENT");
        check(g_ev[i][0] >= 'a' && g_ev[i][0] <= 'z', "event name starts lowercase");
        check(proto_event_find(g_ev[i]) == (int)i, "event name round-trips through the table");
        for (j = i + 1; j < g_ne; ++j) {
            check(strcmp(g_ev[i], g_ev[j]) != 0, "no duplicate event name");
        }
    }

    /* ev 字段的最大掩码必须**正好**是"事件个数个 1"：表长大一点就会不一致 */
    {
        const int ei = field_index("ev");
        check(ei >= 0, "there is an ev field");
        if (ei >= 0) {
            check(g_fs[ei].kind == PROTO_KIND_EVLIST, "ev is an event list");
            vchk((uint64_t)g_fs[ei].max == ((g_ne < 32u) ? ((1ull << g_ne) - 1ull) : 0ull),
                 "ev mask limit == (1<<%u)-1 (got %lld)", (unsigned)g_ne,
                 (long long)g_fs[ei].max);
        }
    }

    /* 错误码名字表必须完整（缺一项 proto_err_name() 会返回 "?"） */
    for (i = 0; i < (size_t)PROTO_E_COUNT; ++i) {
        vchk(strcmp(proto_err_name((proto_err_t)i), "?") != 0, "err %u has a name", (unsigned)i);
    }
    check(strcmp(proto_err_name((proto_err_t)9999), "?") == 0, "unknown err code -> ?");
    check(proto_err_class(PROTO_E_RANGE) == PROTO_CLS_FIELD, "RANGE is a field error");
    check(proto_err_class(PROTO_E_NO_TERMINATOR) == PROTO_CLS_FRAME, "NO_TERMINATOR is a frame error");
    check(proto_err_class(PROTO_E_SEQ_STALE) == PROTO_CLS_SEQ, "SEQ_STALE is a seq drop");
    check(proto_err_class(PROTO_E_RATE_LIMIT) == PROTO_CLS_RATE, "RATE_LIMIT is a rate drop");

    check(PROTO_HB_DEFAULT_MS >= PROTO_HB_MIN_MS && PROTO_HB_DEFAULT_MS <= PROTO_HB_MAX_MS,
          "default heartbeat timeout is inside 200..300 ms");
}

/* ==========================================================================
 * 0b. 限幅的字面量期望（**独立第二来源**）
 *
 * 上面 t_table() 的所有检查都是"表自己跟自己一致"：如果我把 `spd` 的上限从
 * 100 改成 200，那些循环照样全绿 —— 它们只能证明"表被用到了"，不能证明
 * "表里的数是对的"。所以这里再钉一层**硬编码的字面量**，值照原版网页与
 * 迁移表写（摇杆 ±100、滑条 ±40 / 70..110、pit_max_ang=15）。
 *
 * 这就是 tools/golden/README.md 里 test_app_config 那套 [A] 交叉断言 +
 * [B] 字面量断言的同一形状（P-23：参考值和被测代码错在同一个地方，只有
 * 独立来源能抓出来）。
 * ========================================================================== */

typedef struct {
    const char *key;
    int64_t     min;
    int64_t     max;
    proto_kind_t kind;
    proto_req_t required;
} lim_expect_t;

static const lim_expect_t kLimitExpect[] = {
    /*  key     min            max             kind                required            */
    { "rdog",  1,             1,              PROTO_KIND_INT,     PROTO_REQ_STRICT },
    { "seq",   0,             4294967295LL,   PROTO_KIND_INT,     PROTO_REQ_STRICT },
    { "t",     0,             4294967295LL,   PROTO_KIND_INT,     PROTO_REQ_STRICT },
    { "mode",  0,             2,              PROTO_KIND_INT,     PROTO_REQ_OPTIONAL },
    { "est",   0,             1,              PROTO_KIND_INT,     PROTO_REQ_STRICT },
    { "spd",  -100,           100,            PROTO_KIND_INT,     PROTO_REQ_STRICT },
    { "turn", -100,           100,            PROTO_KIND_INT,     PROTO_REQ_STRICT },
    { "ay",   -100,           100,            PROTO_KIND_INT,     PROTO_REQ_STRICT },
    { "ax",   -100,           100,            PROTO_KIND_INT,     PROTO_REQ_STRICT },
    { "grip",  0,             100,            PROTO_KIND_INT,     PROTO_REQ_STRICT },
    { "pit",  -15,            15,             PROTO_KIND_INT,     PROTO_REQ_STRICT },
    { "rol",  -15,            15,             PROTO_KIND_INT,     PROTO_REQ_STRICT },
    { "yst",  -40,            40,             PROTO_KIND_INT,     PROTO_REQ_STRICT },
    { "hgt",   70,            110,            PROTO_KIND_INT,     PROTO_REQ_STRICT },
    { "ev",    0,             16777215LL,     PROTO_KIND_EVLIST,  PROTO_REQ_STRICT },
};

/* 事件白名单：一个不多、一个不少（照 web_common.CTL_KEYS + web_c.py 的标定键）*/
static const char *const kEventExpect[] = {
    "go", "gc", "g0", "g1", "is", "ss",
    "btn_stand", "btn_sit", "btn_wave", "btn_crawl",
    "t9", "sc",
    "l1", "l2", "l3", "l4",
    "hi", "hd", "si", "sd", "ip", "id",
    "am1", "am0",
};

static void t_limits(void)
{
    size_t i, k;
    const size_t nlim = sizeof(kLimitExpect) / sizeof(kLimitExpect[0]);
    const size_t nev = sizeof(kEventExpect) / sizeof(kEventExpect[0]);

    section("limits: literal expectations (independent second source, P-18/P-23)");

    check(PROTO_VERSION == 1, "protocol version literal == 1");

    for (i = 0; i < nlim; ++i) {
        const lim_expect_t *e = &kLimitExpect[i];
        const int fi = proto_field_find(e->key);
        vchk(fi >= 0, "expected field '%s' exists", e->key);
        if (fi < 0) {
            continue;
        }
        vchk(g_fs[fi].min == e->min && g_fs[fi].max == e->max,
             "%s must be [%lld,%lld] (got [%lld,%lld])", e->key,
             (long long)e->min, (long long)e->max,
             (long long)g_fs[fi].min, (long long)g_fs[fi].max);
        vchk(g_fs[fi].kind == e->kind, "%s kind mismatch", e->key);
        vchk(g_fs[fi].required == e->required, "%s required flag mismatch", e->key);
    }
    /* 表里不能有清单之外的字段：多一个字段就意味着它没被第二来源钉住 */
    for (i = 0; i < g_nf; ++i) {
        int found = 0;
        for (k = 0; k < nlim; ++k) {
            if (strcmp(g_fs[i].key, kLimitExpect[k].key) == 0) {
                found = 1;
                break;
            }
        }
        vchk(found == 1, "field '%s' is covered by the literal list", g_fs[i].key);
    }
    vchk(g_nf == nlim, "field count == %u (got %u)", (unsigned)nlim, (unsigned)g_nf);

    vchk(g_ne == nev, "event count == %u (got %u)", (unsigned)nev, (unsigned)g_ne);
    for (i = 0; i < nev; ++i) {
        vchk(proto_event_find(kEventExpect[i]) >= 0,
             "expected event '%s' exists", kEventExpect[i]);
    }
    for (i = 0; i < g_ne; ++i) {
        int found = 0;
        for (k = 0; k < nev; ++k) {
            if (strcmp(g_ev[i], kEventExpect[k]) == 0) {
                found = 1;
                break;
            }
        }
        vchk(found == 1, "event '%s' is covered by the literal list", g_ev[i]);
    }
}

/* ==========================================================================
 * 1. 规范帧长什么样
 * ========================================================================== */

static void t_canonical_frame(void)
{
    proto_cmd_t c;
    char fr[FRAME_CAP];
    size_t n;
    size_t i;

    section("canonical frame layout");
    mk_base(&c);
    n = mk_canon(&c, fr, sizeof(fr));
    check(n > 0, "defaults encode");
    check(n == strlen(fr), "encode returns the byte length");
    check(fr[n - 1u] == '\n', "frame ends with LF");
    check(fr[0] != '\0' && strchr(fr, ' ') == NULL, "no spaces in the frame");
    check(strncmp(fr, "rdog=", 5) == 0, "canonical frame starts with the magic field");
    printf("  canonical: %s", fr);
    printf("  length=%u bytes\n", (unsigned)n);

    /* 每个字段的键都恰好出现一次，且顺序 = 表序 */
    {
        const char *p = fr;
        for (i = 0; i < g_nf; ++i) {
            char pat[PROTO_MAX_KEY + 2];
            size_t hits = 0;
            const char *q;
            snprintf(pat, sizeof(pat), "%s=", g_fs[i].key);
            for (q = fr; *q != '\0'; ++q) {
                if ((q == fr || q[-1] == ';') && strncmp(q, pat, strlen(pat)) == 0) {
                    ++hits;
                }
            }
            vchk(hits == 1, "key %s appears exactly once", g_fs[i].key);
            check(strncmp(p, pat, strlen(pat)) == 0, "canonical order == table order");
            p = strchr(p, ';');
            if (p == NULL) {
                break;
            }
            ++p;
        }
    }

    /* 容量不够：返回 0，且缓冲变成空串（绝不留下半截帧）*/
    {
        char small[64];
        memset(small, 'X', sizeof(small));
        check(proto_encode(&c, small, sizeof(small)) == 0, "too-small buffer -> 0");
        check(small[0] == '\0', "failed encode leaves an empty string");
        check(proto_encode(&c, small, 0) == 0, "zero capacity -> 0");
        check(proto_encode(&c, small, 1) == 0, "capacity 1 -> 0");
    }
    check(proto_encode(NULL, fr, sizeof(fr)) == 0, "NULL cmd -> 0");
    check(proto_encode(&c, NULL, sizeof(fr)) == 0, "NULL buf -> 0");

    /* 编码器也要查表：每个字段越界都必须拒绝。
     * 注意：`seq` / `t` 的域就是 uint32 全域，所以 max+1 **存不进结构体** ——
     * 那两种情况在文本层由 t_range() 用 "seq=4294967296" / "seq=-1" 覆盖，
     * 这里显式断言"存不下"而不是假装测过。 */
    for (i = 0; i < g_nf; ++i) {
        proto_cmd_t bad;
        char fr2[FRAME_CAP];
        const int64_t smax = g_fs[i].is_unsigned ? 4294967295LL : 2147483647LL;
        const int64_t smin = g_fs[i].is_unsigned ? 0LL : -2147483648LL;

        mk_base(&bad);
        if (g_fs[i].max + 1 <= smax) {
            force_store(&bad, i, g_fs[i].max + 1);
            vchk(proto_encode(&bad, fr2, sizeof(fr2)) == 0,
                 "encode rejects %s = max+1", g_fs[i].key);
            check(fr2[0] == '\0', "rejected encode leaves an empty string");
        } else {
            vchk(g_fs[i].max == smax, "%s: max+1 is not representable, text probe covers it",
                 g_fs[i].key);
        }
        mk_base(&bad);
        if (g_fs[i].min - 1 >= smin) {
            force_store(&bad, i, g_fs[i].min - 1);
            vchk(proto_encode(&bad, fr2, sizeof(fr2)) == 0,
                 "encode rejects %s = min-1", g_fs[i].key);
        } else {
            vchk(g_fs[i].min == smin, "%s: min-1 is not representable, text probe covers it",
                 g_fs[i].key);
        }
    }
    /* proto_cmd_set 同样查表 */
    {
        proto_cmd_t bad;
        mk_base(&bad);
        check(proto_cmd_set(&bad, 0, g_fs[0].max + 1) == 0, "proto_cmd_set rejects out of range");
        check(proto_cmd_get(&bad, 0) == g_fs[0].max, "proto_cmd_set did not modify on reject");
        check(proto_cmd_set(&bad, g_nf, 0) == 0, "proto_cmd_set rejects a bad index");
        check(proto_cmd_get(&bad, g_nf) == 0, "proto_cmd_get bad index -> 0");
        check(proto_cmd_get(NULL, 0) == 0, "proto_cmd_get(NULL) -> 0");
    }
}

/* ==========================================================================
 * 2. 往返回环：每个字段的 min / max / 内部值
 * ========================================================================== */

static uint32_t s_rng = 0x9E3779B9u;

static uint32_t rnd(void)
{
    uint32_t x = s_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_rng = x;
    return x;
}

static int64_t rand_in(const proto_field_t *f)
{
    const int64_t span = f->max - f->min + 1;
    if (span <= 1) {
        return f->min;
    }
    if (span <= 65536) {
        return f->min + (int64_t)(rnd() % (uint32_t)span);
    }
    {
        const uint64_t r = ((uint64_t)rnd() << 32) ^ (uint64_t)rnd();
        return f->min + (int64_t)(r % (uint64_t)span);
    }
}

static size_t add_val(int64_t *v, size_t n, int64_t x)
{
    size_t i;
    for (i = 0; i < n; ++i) {
        if (v[i] == x) {
            return n;
        }
    }
    v[n] = x;
    return n + 1u;
}

static void t_roundtrip(void)
{
    size_t i;

    section("round trip: every field, min / max / interior values");

    for (i = 0; i < g_nf; ++i) {
        const proto_field_t *f = &g_fs[i];
        int64_t vals[8];
        size_t nv = 0;
        size_t k;

        if (f->kind == PROTO_KIND_INT) {
            nv = add_val(vals, nv, f->min);
            nv = add_val(vals, nv, f->max);
            if (f->max - f->min >= 1) {
                nv = add_val(vals, nv, f->min + 1);
                nv = add_val(vals, nv, f->max - 1);
            }
            if (f->max - f->min >= 4) {
                nv = add_val(vals, nv, (f->min + f->max) / 2);
                nv = add_val(vals, nv, f->min + (f->max - f->min) / 3);
                nv = add_val(vals, nv, f->min + (f->max - f->min) * 2 / 3);
            }
            nv = add_val(vals, nv, rand_in(f));
        } else {
            nv = add_val(vals, nv, 0);
            nv = add_val(vals, nv, 1);
            nv = add_val(vals, nv, (int64_t)(1u << (g_ne - 1u)));
            nv = add_val(vals, nv, f->max);
            nv = add_val(vals, nv, rand_in(f));
        }

        for (k = 0; k < nv; ++k) {
            proto_cmd_t c;
            proto_cmd_t out;
            proto_error_t er;
            char fr[FRAME_CAP];
            char fr2[FRAME_CAP];
            size_t n;
            int rc;

            mk_base(&c);
            c.seq = 4242u;
            vchk(proto_cmd_set(&c, i, vals[k]) == 1, "%s: set %lld", f->key, (long long)vals[k]);
            n = mk_canon(&c, fr, sizeof(fr));
            vchk(n > 0, "%s=%lld: encode", f->key, (long long)vals[k]);

            poison(&out);
            rc = dec_strict(fr, n, &out, &er);
            vchk(rc == PROTO_OK, "%s=%lld: decode (got %s)", f->key, (long long)vals[k],
                 proto_err_name(er.code));
            vchk(proto_cmd_get(&out, i) == vals[k], "%s=%lld: value survived",
                 f->key, (long long)vals[k]);
            vchk(proto_cmd_equal(&out, &c), "%s=%lld: whole struct identical",
                 f->key, (long long)vals[k]);

            /* 规范帧再编一次必须逐字节相同（规范形式唯一）*/
            n = mk_canon(&out, fr2, sizeof(fr2));
            vchk(n > 0 && strcmp(fr, fr2) == 0, "%s=%lld: canonical form is stable",
                 f->key, (long long)vals[k]);
        }
    }
}

/* ==========================================================================
 * 3. 范围边界：min/max 收，min-1/max+1 拒（P-18：必须能区分）
 * ========================================================================== */

static void t_range(void)
{
    size_t i;

    section("range: min/max accepted, min-1/max+1 rejected and named");

    for (i = 0; i < g_nf; ++i) {
        const proto_field_t *f = &g_fs[i];
        int64_t probe[2];
        size_t p;
        const int is_ev = (f->kind == PROTO_KIND_EVLIST);

        probe[0] = (f->kind == PROTO_KIND_INT) ? f->min - 1 : -1;
        probe[1] = f->max + 1;

        for (p = 0; p < 2; ++p) {
            char fr[FRAME_CAP];
            char num[32];
            proto_cmd_t base;
            proto_cmd_t out;
            proto_cmd_t snap;
            proto_error_t er;
            int rc;
            proto_err_t want = is_ev ? PROTO_E_EVENT_BAD_NAME : PROTO_E_RANGE;

            if (f->role == PROTO_ROLE_MAGIC) {
                want = PROTO_E_BAD_VERSION;   /* 版本字段越界单独报 */
            }

            mk_base(&base);
            base.seq = 55u;
            check(mk_canon(&base, fr, sizeof(fr)) > 0, "base frame");
            i64_str(probe[p], num);
            vchk(splice_value(fr, sizeof(fr), f->key, num) == 1,
                 "%s: splice probe", f->key);

            poison(&out);
            memcpy(&snap, &out, sizeof(out));
            rc = dec_strict(fr, strlen(fr), &out, &er);
            vchk(rc != PROTO_OK, "%s: %lld must be rejected (got OK)", f->key,
                 (long long)probe[p]);
            vchk(er.code == want, "%s=%lld: error is %s (got %s)", f->key,
                 (long long)probe[p], proto_err_name(want), proto_err_name(er.code));
            if (!is_ev) {
                vchk(er.field == (int)i, "%s=%lld: error names the field index",
                     f->key, (long long)probe[p]);
                vchk(strcmp(er.key, f->key) == 0, "%s=%lld: error carries the key name",
                     f->key, (long long)probe[p]);
            }
            vchk(untouched(&out, &snap), "%s=%lld: rejected frame left out untouched",
                 f->key, (long long)probe[p]);
        }

        /* 边界本身必须被接受 —— 否则上一段的"拒"不说明范围真的被用了 */
        {
            proto_cmd_t c;
            proto_cmd_t out;
            proto_error_t er;
            char fr[FRAME_CAP];
            size_t n;

            mk_base(&c);
            c.seq = 77u;
            check(proto_cmd_set(&c, i, f->min) == 1, "min is settable");
            n = mk_canon(&c, fr, sizeof(fr));
            check(n > 0, "min encodes");
            check(dec_strict(fr, n, &out, &er) == PROTO_OK, "min is accepted");
            check(proto_cmd_get(&out, i) == f->min, "min round-trips");

            mk_base(&c);
            c.seq = 77u;
            check(proto_cmd_set(&c, i, f->max) == 1, "max is settable");
            n = mk_canon(&c, fr, sizeof(fr));
            check(n > 0, "max encodes");
            check(dec_strict(fr, n, &out, &er) == PROTO_OK, "max is accepted");
            check(proto_cmd_get(&out, i) == f->max, "max round-trips");
        }
    }

    /* 事件掩码里"白名单外的位"必须被拒（setter + 编码器两条路）*/
    {
        const int ei = field_index("ev");
        proto_cmd_t c;
        char fr[FRAME_CAP];
        mk_base(&c);
        check(proto_cmd_set(&c, (size_t)ei, g_fs[ei].max) == 1, "full event mask is settable");
        check(proto_cmd_set(&c, (size_t)ei, g_fs[ei].max + 1) == 0,
              "one bit past the table -> rejected");
        force_store(&c, (size_t)ei, g_fs[ei].max + 1);
        check(proto_encode(&c, fr, sizeof(fr)) == 0, "encode rejects an out-of-table event bit");
    }
}

/* ==========================================================================
 * 4. 必填 / 重复 / 未知键
 * ========================================================================== */

static void t_keys(void)
{
    size_t i;

    section("keys: required / optional / missing / duplicated / unknown");

    for (i = 0; i < g_nf; ++i) {
        const proto_field_t *f = &g_fs[i];
        char fr[FRAME_CAP];
        proto_cmd_t out;
        proto_cmd_t snap;
        proto_error_t er;
        int rc;

        /* --- 缺失 --- */
        {
            proto_cmd_t base;
            mk_base(&base);
            base.seq = 11u;
            (void)mk_canon(&base, fr, sizeof(fr));
        }
        check(remove_field(fr, sizeof(fr), f->key) == 1, "remove field works");
        poison(&out);
        memcpy(&snap, &out, sizeof(out));
        rc = dec_strict(fr, strlen(fr), &out, &er);
        if (f->required == PROTO_REQ_STRICT) {
            vchk(rc == PROTO_E_MISSING_KEY, "%s: missing required key -> MISSING_KEY (got %s)",
                 f->key, proto_err_name(er.code));
            vchk(er.field == (int)i, "%s: MISSING_KEY names the field", f->key);
            vchk(strcmp(er.key, f->key) == 0, "%s: MISSING_KEY carries the key name", f->key);
            vchk(untouched(&out, &snap), "%s: missing-key rejection left out untouched", f->key);
        } else {
            vchk(rc == PROTO_OK, "%s: optional key may be absent (got %s)", f->key,
                 proto_err_name(er.code));
            /* 缺省 != 默认值：值可能是 0，但 present 位必须是 0 */
            vchk((out.present & (1u << i)) == 0u, "%s: absent field has present=0", f->key);
            vchk((out.present & ~(1u << i)) != 0u, "%s: other fields still present", f->key);
        }

        /* --- 重复 --- */
        {
            proto_cmd_t base;
            mk_base(&base);
            base.seq = 12u;
            (void)mk_canon(&base, fr, sizeof(fr));
        }
        check(dup_field(fr, sizeof(fr), f->key) == 1, "dup field works");
        poison(&out);
        memcpy(&snap, &out, sizeof(out));
        rc = dec_strict(fr, strlen(fr), &out, &er);
        vchk(rc == PROTO_E_DUP_KEY, "%s: duplicated key -> DUP_KEY (got %s)", f->key,
             proto_err_name(er.code));
        vchk(er.field == (int)i, "%s: DUP_KEY names the field", f->key);
        vchk(untouched(&out, &snap), "%s: duplicate rejection left out untouched", f->key);
    }

    /* 未知键 / 键名语法错（注意：键名语法先于白名单检查 ⇒ 大写/数字开头是 BAD_KEY）*/
    {
        static const struct { const char *key; proto_err_t want; } bad[] = {
            { "spdx",  PROTO_E_UNKNOWN_KEY },
            { "xx",    PROTO_E_UNKNOWN_KEY },
            { "seqx",  PROTO_E_UNKNOWN_KEY },
            { "spd2",  PROTO_E_UNKNOWN_KEY },
            { "SPD",   PROTO_E_BAD_KEY },
            { "1spd",  PROTO_E_BAD_KEY },
            { "_spd",  PROTO_E_BAD_KEY },
            { "spd-x", PROTO_E_BAD_KEY },
        };
        size_t k;
        for (k = 0; k < sizeof(bad) / sizeof(bad[0]); ++k) {
            char fr[FRAME_CAP];
            proto_cmd_t base;
            proto_cmd_t out;
            proto_cmd_t snap;
            proto_error_t er;
            mk_base(&base);
            base.seq = 13u;
            (void)mk_canon(&base, fr, sizeof(fr));
            check(splice_key(fr, sizeof(fr), "spd", bad[k].key) == 1, "splice unknown key");
            poison(&out);
            memcpy(&snap, &out, sizeof(out));
            vchk(dec_strict(fr, strlen(fr), &out, &er) == (int)bad[k].want,
                 "key '%s' -> %s (got %s)", bad[k].key,
                 proto_err_name(bad[k].want), proto_err_name(er.code));
            vchk(untouched(&out, &snap), "key '%s' left out untouched", bad[k].key);
        }
    }

    /* 超长键名 / 空键名 */
    {
        char big[FRAME_CAP];
        char small[FRAME_CAP];
        char longkey[80];
        proto_cmd_t out;
        proto_error_t er;
        size_t k;

        for (k = 0; k < sizeof(longkey) - 1u; ++k) {
            longkey[k] = 'a';
        }
        longkey[sizeof(longkey) - 1u] = '\0';

        snprintf(big, sizeof(big), "rdog=1;seq=15;t=0;mode=0;est=0;%s=1\n", longkey);
        poison(&out);
        vchk(dec_strict(big, strlen(big), &out, &er) == PROTO_E_BAD_KEY,
             "a 79-char key -> BAD_KEY (got %s)", proto_err_name(er.code));

        snprintf(small, sizeof(small), "rdog=1;seq=16;t=0;mode=0;est=0;spd=1;turn=0;ay=0;ax=0;"
                                       "grip=0;pit=0;rol=0;yst=0;hgt=90;ev=\n");
        check(dec_strict(small, strlen(small), &out, &er) == PROTO_OK, "control frame is ok");
        poison(&out);
        vchk(dec_strict("rdog=1;seq=17;t=0;mode=0;est=0;=1\n", 34u, &out, &er) == PROTO_E_BAD_KEY,
             "empty key -> BAD_KEY (got %s)", proto_err_name(er.code));
    }
}

/* ==========================================================================
 * 5. 取值：整数语法 + 事件列表
 * ========================================================================== */

static void t_values(void)
{
    size_t i;
    static const char *const bad_int[] = {
        "", "x", "+7", "--1", "1-2", "1+2", "007", "-0", "1.5", "0x10", " 1", "1 ",
        "999999999999999", "-", "+", "1e3", "001",
    };

    section("values: integer syntax and event list");

    for (i = 0; i < g_nf; ++i) {
        const proto_field_t *f = &g_fs[i];
        size_t k;

        if (f->kind != PROTO_KIND_INT) {
            continue;
        }
        for (k = 0; k < sizeof(bad_int) / sizeof(bad_int[0]); ++k) {
            char fr[FRAME_CAP];
            proto_cmd_t base;
            proto_cmd_t out;
            proto_cmd_t snap;
            proto_error_t er;
            int rc;

            mk_base(&base);
            base.seq = 21u;
            (void)mk_canon(&base, fr, sizeof(fr));
            check(splice_value(fr, sizeof(fr), f->key, bad_int[k]) == 1, "splice bad value");
            poison(&out);
            memcpy(&snap, &out, sizeof(out));
            rc = dec_strict(fr, strlen(fr), &out, &er);
            vchk(rc != PROTO_OK, "%s='%s' must be rejected", f->key, bad_int[k]);
            vchk(er.field == (int)i, "%s='%s': error names the field", f->key, bad_int[k]);
            vchk(untouched(&out, &snap), "%s='%s': out untouched", f->key, bad_int[k]);
            if (bad_int[k][0] == '\0') {
                vchk(er.code == PROTO_E_EMPTY_VALUE, "%s='': EMPTY_VALUE (got %s)", f->key,
                     proto_err_name(er.code));
            } else {
                vchk(er.code == PROTO_E_BAD_INT, "%s='%s': BAD_INT (got %s)", f->key,
                     bad_int[k], proto_err_name(er.code));
            }
        }
    }

    /* 语法合法但超出域的文本值：必须报 RANGE（和"语法错"分开） */
    {
        char fr[FRAME_CAP];
        proto_cmd_t base;
        proto_cmd_t out;
        proto_error_t er;
        static const struct { const char *key; const char *val; } rng[] = {
            { "spd", "999" }, { "spd", "-101" }, { "grip", "101" },
            { "hgt", "111" }, { "seq", "4294967296" }, { "seq", "-1" },
            { "mode", "3" }, { "est", "2" }, { "pit", "16" },
        };
        size_t k;
        for (k = 0; k < sizeof(rng) / sizeof(rng[0]); ++k) {
            mk_base(&base);
            base.seq = 23u;
            (void)mk_canon(&base, fr, sizeof(fr));
            check(splice_value(fr, sizeof(fr), rng[k].key, rng[k].val) == 1, "splice range probe");
            poison(&out);
            vchk(dec_strict(fr, strlen(fr), &out, &er) == PROTO_E_RANGE,
                 "%s=%s -> RANGE (got %s)", rng[k].key, rng[k].val, proto_err_name(er.code));
        }
    }

    /* 事件列表的合法与非法 */
    {
        char fr[FRAME_CAP];
        proto_cmd_t base;
        proto_cmd_t out;
        proto_error_t er;
        struct {
            const char *val;
            proto_err_t want;
        } cases[] = {
            { "",              PROTO_OK },
            { "g0",            PROTO_OK },
            { "g0,g1",         PROTO_OK },
            { "btn_stand",     PROTO_OK },
            { "am1",           PROTO_OK },
            { "am0,am1",       PROTO_OK },
            { "zz",            PROTO_E_EVENT_UNKNOWN },
            { "g0,zz",         PROTO_E_EVENT_UNKNOWN },
            { "G0",            PROTO_E_EVENT_BAD_NAME },
            { "g0,g0",         PROTO_E_EVENT_DUP },
            { "g0,",           PROTO_E_EVENT_BAD_NAME },
            { ",g0",           PROTO_E_EVENT_BAD_NAME },
            { "g0,,g1",        PROTO_E_EVENT_BAD_NAME },
            { "g0 g1",         PROTO_E_EVENT_BAD_NAME },
            { "0123456789abcdefg",  PROTO_E_EVENT_BAD_NAME },
            { "btn_stop",      PROTO_E_EVENT_UNKNOWN },
            { "btn_grip_open", PROTO_E_EVENT_UNKNOWN },
            { "btn_grip_close",PROTO_E_EVENT_UNKNOWN },
            { "l1",            PROTO_OK },
        };
        size_t k;

        for (k = 0; k < sizeof(cases) / sizeof(cases[0]); ++k) {
            size_t n;
            int rc;
            mk_base(&base);
            base.seq = 22u;
            (void)mk_canon(&base, fr, sizeof(fr));
            check(splice_value(fr, sizeof(fr), "ev", cases[k].val) == 1, "splice ev");
            poison(&out);
            rc = dec_strict(fr, strlen(fr), &out, &er);
            vchk(rc == (int)cases[k].want, "ev='%s' -> %s (got %s)", cases[k].val,
                 proto_err_name(cases[k].want), proto_err_name(er.code));
            if (cases[k].want == PROTO_OK) {
                n = (size_t)proto_cmd_ev_count(&out);
                if (cases[k].val[0] == '\0') {
                    check(n == 0, "empty event list -> 0 events");
                } else {
                    check(n >= 1, "event list produced at least one event");
                }
            }
        }
        mk_base(&base);
        base.seq = 23u;
        (void)mk_canon(&base, fr, sizeof(fr));
        check(splice_value(fr, sizeof(fr), "ev", "g1,btn_wave") == 1, "splice two events");
        check(dec_strict(fr, strlen(fr), &out, &er) == PROTO_OK, "two events decode");
        check(proto_cmd_has_ev(&out, "g1") == 1, "has_ev g1");
        check(proto_cmd_has_ev(&out, "btn_wave") == 1, "has_ev btn_wave");
        check(proto_cmd_has_ev(&out, "g0") == 0, "has_ev g0 false");
        check(proto_cmd_has_ev(&out, "nope") == 0, "has_ev unknown false");
        check(proto_cmd_has_ev(NULL, "g0") == 0, "has_ev NULL false");
        check(proto_cmd_ev_count(&out) == 2, "ev_count == 2");
        check(proto_cmd_ev_count(NULL) == 0, "ev_count NULL == 0");
    }
}

/* ==========================================================================
 * 6. 截断：每一个前缀长度
 * ========================================================================== */

static void t_truncation(void)
{
    char fr[FRAME_CAP];
    proto_cmd_t base;
    size_t n;
    size_t p;

    section("truncation: every prefix of a valid frame is rejected");

    mk_base(&base);
    base.seq = 31u;
    n = mk_canon(&base, fr, sizeof(fr));
    check(n > 0, "valid frame built");

    for (p = 0; p < n; ++p) {
        proto_cmd_t out;
        proto_cmd_t snap;
        proto_error_t er;
        int rc;
        poison(&out);
        memcpy(&snap, &out, sizeof(out));
        rc = dec_strict(fr, p, &out, &er);
        vchk(rc != PROTO_OK, "prefix %u/%u must be rejected (got OK)", (unsigned)p, (unsigned)n);
        vchk(untouched(&out, &snap), "prefix %u left out untouched", (unsigned)p);
    }

    /* 截到字段边界再补终止符：也不许通过（缺必填键）*/
    {
        size_t cut;
        for (cut = 0; cut < n; cut += 1u) {
            if (fr[cut] != ';') {
                continue;
            }
            {
                char part[FRAME_CAP];
                proto_cmd_t out;
                proto_error_t er;
                int rc;
                memcpy(part, fr, cut);
                part[cut] = '\n';
                part[cut + 1u] = '\0';
                poison(&out);
                rc = dec_strict(part, cut + 1u, &out, &er);
                vchk(rc == PROTO_E_MISSING_KEY || rc == PROTO_E_NO_FIELDS,
                     "early-terminated frame must be rejected (cut=%u got %s)",
                     (unsigned)cut, proto_err_name(er.code));
            }
        }
    }
}

/* ==========================================================================
 * 7. 帧结构：魔数 / 版本 / 终止符 / 超长 / 垃圾
 * ========================================================================== */

static void t_framing(void)
{
    struct {
        const char *what;
        const char *frame;
        size_t      len;      /* 0 = strlen */
        proto_err_t want;
    } cases[] = {
        { "empty",            "", 0, PROTO_E_EMPTY },
        { "only LF",          "\n", 0, PROTO_E_EMPTY },
        { "only semicolon",   ";\n", 0, PROTO_E_BAD_SEPARATOR },
        { "only spaces",      "   \n", 0, PROTO_E_NO_EQUALS },
        { "no terminator",    "rdog=1;seq=1;t=0;mode=0;est=0;spd=0;turn=0;ay=0;ax=0;grip=0;pit=0;rol=0;yst=0;hgt=90;ev=", 0, PROTO_E_NO_TERMINATOR },
        { "CRLF terminator",  "rdog=1;seq=1;t=0;mode=0;est=0;spd=0;turn=0;ay=0;ax=0;grip=0;pit=0;rol=0;yst=0;hgt=90;ev=\r\n", 0, PROTO_E_CTRL_CHAR },
        { "two frames glued", "rdog=1;seq=1;t=0;mode=0;est=0;spd=0;turn=0;ay=0;ax=0;grip=0;pit=0;rol=0;yst=0;hgt=90;ev=\nrdog=1;seq=2;t=0;mode=0;est=0;spd=0;turn=0;ay=0;ax=0;grip=0;pit=0;rol=0;yst=0;hgt=90;ev=\n", 0, PROTO_E_CTRL_CHAR },
        { "trailing semicolon","rdog=1;\n", 0, PROTO_E_BAD_SEPARATOR },
        { "double semicolon", "rdog=1;;seq=1\n", 0, PROTO_E_BAD_SEPARATOR },
        { "leading semicolon",";rdog=1\n", 0, PROTO_E_BAD_SEPARATOR },
        { "no equals",        "rdog=1;seq\n", 0, PROTO_E_NO_EQUALS },
        { "high byte",        "rdog=1;seq=\xc3\xa9\n", 0, PROTO_E_CTRL_CHAR },
        { "NUL inside",       "rdog=1;seq=1\0x\n", 15u, PROTO_E_CTRL_CHAR },
        { "HTTP request",     "GET /f=80t=0 HTTP/1.1\n", 0, PROTO_E_BAD_KEY },
        { "old page frame",   "f=80t=0\n", 0, PROTO_E_UNKNOWN_KEY },
        { "bad magic",        "proto=1;seq=1;t=0;mode=0;est=0;spd=0;turn=0;ay=0;ax=0;grip=0;pit=0;rol=0;yst=0;hgt=90;ev=\n", 0, PROTO_E_UNKNOWN_KEY },
    };
    size_t i;

    section("framing: magic / version / terminator / oversized / junk");

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        proto_cmd_t out;
        proto_cmd_t snap;
        proto_error_t er;
        const size_t len = (cases[i].len != 0u) ? cases[i].len : strlen(cases[i].frame);
        int rc;
        poison(&out);
        memcpy(&snap, &out, sizeof(out));
        rc = dec_strict(cases[i].frame, len, &out, &er);
        vchk(rc == (int)cases[i].want, "%s -> %s (got %s)", cases[i].what,
             proto_err_name(cases[i].want), proto_err_name(er.code));
        vchk(untouched(&out, &snap), "%s left out untouched", cases[i].what);
    }

    /* 版本：rdog=0 / rdog=2 报 BAD_VERSION */
    {
        char fr[FRAME_CAP];
        proto_cmd_t base;
        proto_cmd_t out;
        proto_error_t er;
        static const char *const vbad[] = { "0", "2", "99", "-1" };
        size_t k;
        for (k = 0; k < sizeof(vbad) / sizeof(vbad[0]); ++k) {
            mk_base(&base);
            base.seq = 41u;
            (void)mk_canon(&base, fr, sizeof(fr));
            check(splice_value(fr, sizeof(fr), "rdog", vbad[k]) == 1, "splice version");
            poison(&out);
            vchk(dec_strict(fr, strlen(fr), &out, &er) == PROTO_E_BAD_VERSION,
                 "rdog=%s -> BAD_VERSION (got %s)", vbad[k], proto_err_name(er.code));
        }
    }

    /* 超长：512 与 513 / 几千字节 */
    {
        char *big = (char *)malloc(5000);
        proto_cmd_t out;
        proto_error_t er;
        check(big != NULL, "malloc");
        if (big != NULL) {
            memset(big, 'a', 5000);
            big[4999] = '\n';
            check(dec_strict(big, 513u, &out, &er) == PROTO_E_TOO_LONG, "513 bytes -> TOO_LONG");
            check(dec_strict(big, 4096u, &out, &er) == PROTO_E_TOO_LONG, "4096 bytes -> TOO_LONG");
            check(dec_strict(big, 5000u, &out, &er) == PROTO_E_TOO_LONG, "5000 bytes -> TOO_LONG");
            /* 上限那一点本身要走完解析（不是 TOO_LONG），证明 512 是真边界 */
            memset(big, 'a', PROTO_MAX_FRAME);
            big[PROTO_MAX_FRAME - 1u] = '\n';
            check(dec_strict(big, PROTO_MAX_FRAME, &out, &er) != PROTO_E_TOO_LONG,
                  "exactly PROTO_MAX_FRAME is parsed (not TOO_LONG)");
            free(big);
        }
    }
}

/* ==========================================================================
 * 8. 序号：只接受"往前走"
 * ========================================================================== */

static void t_seq(void)
{
    proto_decoder_t d;
    char fr[FRAME_CAP];
    proto_cmd_t base;
    proto_cmd_t out;
    proto_error_t er;
    size_t n;

    section("seq: newer accepted, duplicate/older/replayed dropped, wrap-safe");

    mk_base(&base);
    proto_decoder_init(&d);
    proto_decoder_set_min_interval_ms(&d, 0);   /* 这一段只测序号语义 */
    proto_decoder_set_hb_ms(&d, 300);

    check(proto_seq_newer(2u, 1u) == 1, "2 > 1");
    check(proto_seq_newer(1u, 1u) == 0, "equal is not newer");
    check(proto_seq_newer(0u, 0xFFFFFFFFu) == 1, "0 is newer than 0xFFFFFFFF (wrap)");
    check(proto_seq_newer(0xFFFFFFFFu, 0xFFFFFFFFu) == 0, "equal at wrap is not newer");
    check(proto_seq_newer(0xFFFFFFFEu, 0xFFFFFFFFu) == 0, "0xFFFFFFFE is older than 0xFFFFFFFF");
    check(proto_seq_newer(0x7FFFFFFEu, 0xFFFFFFFFu) == 1, "+2^31-1 is still forward");
    check(proto_seq_newer(0x7FFFFFFFu, 0xFFFFFFFFu) == 0, "+2^31 is treated as backward");

    base.seq = 100u;
    n = mk_canon(&base, fr, sizeof(fr));
    check(proto_decode(&d, fr, n, 1000u, &out, &er) == PROTO_OK, "seq=100 accepted");
    check(d.stats.accepted == 1u, "accepted counter grew");

    check(proto_decode(&d, fr, n, 1010u, &out, &er) == PROTO_E_SEQ_STALE, "duplicate dropped");
    check(d.stats.dropped_seq == 1u, "dropped_seq counter grew");
    base.seq = 99u;
    n = mk_canon(&base, fr, sizeof(fr));
    check(proto_decode(&d, fr, n, 1020u, &out, &er) == PROTO_E_SEQ_STALE, "older dropped");
    base.seq = 101u;
    n = mk_canon(&base, fr, sizeof(fr));
    check(proto_decode(&d, fr, n, 1030u, &out, &er) == PROTO_OK, "101 accepted");

    /* 被拒的帧不能推进序号基线 */
    {
        proto_cmd_t bad;
        mk_base(&bad);
        bad.seq = 500u;
        bad.spd = 101;
        n = mk_canon(&bad, fr, sizeof(fr));
        check(n == 0, "an invalid command cannot even be encoded");
        mk_base(&bad);
        bad.seq = 500u;
        n = mk_canon(&bad, fr, sizeof(fr));
        check(splice_value(fr, sizeof(fr), "spd", "101") == 1, "splice spd=101");
        check(proto_decode(&d, fr, strlen(fr), 1040u, &out, &er) == PROTO_E_RANGE,
              "spd=101 rejected");
        base.seq = 102u;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&d, fr, n, 1050u, &out, &er) == PROTO_OK,
              "seq=102 still accepted (rejected frame did not move the baseline)");
    }
    /* 被拒的旧帧不能把基线往回拉 */
    {
        base.seq = 90u;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&d, fr, n, 1060u, &out, &er) == PROTO_E_SEQ_STALE, "90 dropped");
        base.seq = 91u;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&d, fr, n, 1070u, &out, &er) == PROTO_E_SEQ_STALE,
              "91 also dropped (baseline is still 102, not 90)");
    }

    /* 回绕：0xFFFFFFF0 -> 0xFFFFFFFF -> 0 -> 1 */
    {
        proto_decoder_t w;
        proto_decoder_init(&w);
        proto_decoder_set_min_interval_ms(&w, 0);
        base.seq = 0xFFFFFFF0u;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&w, fr, n, 2000u, &out, &er) == PROTO_OK, "0xFFFFFFF0 accepted");
        base.seq = 0xFFFFFFFEu;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&w, fr, n, 2100u, &out, &er) == PROTO_OK, "0xFFFFFFFE accepted");
        base.seq = 0xFFFFFFFFu;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&w, fr, n, 2200u, &out, &er) == PROTO_OK, "0xFFFFFFFF accepted");
        base.seq = 0u;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&w, fr, n, 2300u, &out, &er) == PROTO_OK, "0 accepted (wrap)");
        base.seq = 1u;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&w, fr, n, 2400u, &out, &er) == PROTO_OK, "1 accepted after wrap");
        base.seq = 0xFFFFFFFFu;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&w, fr, n, 2500u, &out, &er) == PROTO_E_SEQ_STALE,
              "0xFFFFFFFF is now older than 1 -> dropped");
    }

    check(d.stats.frames == d.stats.accepted + d.stats.rejected, "frames == accepted + rejected");
    check(d.stats.rejected == d.stats.bad_frame + d.stats.bad_field + d.stats.dropped_seq
                              + d.stats.rate_limited, "rejected == the four categories");
    {
        uint32_t sum = 0;
        int i;
        for (i = 1; i < (int)PROTO_E_COUNT; ++i) {
            sum += d.stats.by_err[i];
        }
        check(sum == d.stats.rejected, "by_err sums to rejected");
    }
}

/* ==========================================================================
 * 9. 限速：>100 Hz 视为洪泛（§7 目标 20~50 Hz）
 * ========================================================================== */

static void t_rate_limit(void)
{
    proto_decoder_t d;
    char fr[FRAME_CAP];
    proto_cmd_t base;
    proto_cmd_t out;
    proto_error_t er;
    size_t n;

    section("rate limit: floods dropped, commands and e-stop bypass");

    mk_base(&base);
    proto_decoder_init(&d);
    check(d.min_interval_ms == PROTO_MIN_INTERVAL_DEFAULT_MS, "default min interval");

    base.seq = 1u;
    n = mk_canon(&base, fr, sizeof(fr));
    check(proto_decode(&d, fr, n, 1000u, &out, &er) == PROTO_OK, "first frame accepted");

    base.seq = 2u;
    n = mk_canon(&base, fr, sizeof(fr));
    check(proto_decode(&d, fr, n, 1009u, &out, &er) == PROTO_E_RATE_LIMIT,
          "9 ms later -> RATE_LIMIT");
    check(d.stats.rate_limited == 1u, "rate_limited counter grew");
    check(proto_decode(&d, fr, n, 1010u, &out, &er) == PROTO_OK,
          "exactly min_interval later -> accepted");
    check(d.last_seq == 2u, "accepted seq is 2");

    /* 带事件的帧不参与限速 */
    base.seq = 3u;
    base.ev_mask = (uint32_t)(1u << (size_t)proto_event_find("g0"));
    n = mk_canon(&base, fr, sizeof(fr));
    check(proto_decode(&d, fr, n, 1011u, &out, &er) == PROTO_OK, "event frame bypasses the limiter");

    /* 急停帧也不参与限速 */
    base.seq = 4u;
    base.ev_mask = 0u;
    base.est = 1;
    n = mk_canon(&base, fr, sizeof(fr));
    check(proto_decode(&d, fr, n, 1012u, &out, &er) == PROTO_OK, "estop frame bypasses the limiter");
    check(d.stats.estop_frames == 1u, "estop_frames counter grew");

    /* 被限速的帧不能刷新心跳 */
    {
        proto_decoder_t h;
        proto_decoder_init(&h);
        base.ev_mask = 0u;
        base.est = 0;
        base.seq = 1u;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&h, fr, n, 5000u, &out, &er) == PROTO_OK, "heartbeat frame accepted");
        base.seq = 2u;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&h, fr, n, 5005u, &out, &er) == PROTO_E_RATE_LIMIT, "5 ms later dropped");
        check(proto_age_ms(&h, 5005u) == 5u, "age still measured from the accepted frame");
        /* 心跳以**被接受**的那一帧为基准：5000 + 249 还新鲜，+251 就过期 */
        check(proto_is_stale(&h, 5000u + 249u) == 0, "fresh 249 ms after the accepted frame");
        check(proto_is_stale(&h, 5000u + 251u) == 1, "stale 251 ms after the accepted frame");
    }

    /* 阈值可配 + 钳位 */
    check(proto_decoder_set_min_interval_ms(&d, 0u) == 0u, "min interval 0 = no limiting");
    base.seq = 5u;
    n = mk_canon(&base, fr, sizeof(fr));
    check(proto_decode(&d, fr, n, 1013u, &out, &er) == PROTO_OK, "same-ms frame accepted with 0");
    base.seq = 6u;
    n = mk_canon(&base, fr, sizeof(fr));
    check(proto_decode(&d, fr, n, 1013u, &out, &er) == PROTO_OK, "another same-ms frame accepted");
    check(proto_decoder_set_min_interval_ms(&d, 50u) == 50u, "set 50 ms");
    base.seq = 7u;
    n = mk_canon(&base, fr, sizeof(fr));
    check(proto_decode(&d, fr, n, 1033u, &out, &er) == PROTO_E_RATE_LIMIT, "20 ms < 50 ms -> dropped");
    check(proto_decode(&d, fr, n, 1063u, &out, &er) == PROTO_OK, "50 ms later -> accepted");
    check(proto_decoder_set_min_interval_ms(&d, 100000u) == PROTO_MIN_INTERVAL_MAX_MS,
          "min interval clamps to the max");
    check(proto_decoder_set_min_interval_ms(NULL, 5u) == PROTO_MIN_INTERVAL_DEFAULT_MS,
          "NULL decoder -> default");
}

/* ==========================================================================
 * 10. 心跳 / 新鲜度（可控时钟）
 * ========================================================================== */

static void t_heartbeat(void)
{
    proto_decoder_t d;
    char fr[FRAME_CAP];
    proto_cmd_t base;
    proto_cmd_t out;
    proto_error_t er;
    size_t n;
    uint32_t t0;

    section("heartbeat: mock clock, stale just over the threshold and never-received");

    mk_base(&base);
    proto_decoder_init(&d);

    /* 一帧都没收到：安全侧算过期 */
    check(proto_is_stale(&d, 0u) == 1, "no frame ever -> stale");
    check(proto_is_expired(&d, 0u) == 1, "no frame ever -> expired");
    check(proto_age_ms(&d, 123u) == PROTO_AGE_NEVER, "age is PROTO_AGE_NEVER");
    check(proto_last_rx_ms(&d) == 0u, "last_rx is 0");
    check(proto_is_stale(NULL, 0u) == 1, "NULL decoder -> stale");
    check(proto_is_expired(NULL, 0u) == 1, "NULL decoder -> expired");

    t_clock_goto(100000u);
    t0 = t_now();
    base.seq = 1u;
    n = mk_canon(&base, fr, sizeof(fr));
    check(proto_decode(&d, fr, n, t0, &out, &er) == PROTO_OK, "frame accepted on the mock clock");

    check(proto_is_stale(&d, t0) == 0, "age 0 -> fresh");
    check(proto_is_stale(&d, t0 + PROTO_HB_DEFAULT_MS - 1u) == 0, "just under the threshold -> fresh");
    check(proto_is_stale(&d, t0 + PROTO_HB_DEFAULT_MS) == 0, "exactly at the threshold -> still fresh");
    check(proto_is_stale(&d, t0 + PROTO_HB_DEFAULT_MS + 1u) == 1, "just over the threshold -> stale");
    check(proto_age_ms(&d, t0 + 300u) == 300u, "age is measured locally");

    /* 真的用可控时钟推进一遍 */
    t_advance_ms(249u);
    check(proto_is_stale(&d, t_now()) == 0, "mock clock +249 ms -> fresh");
    t_advance_ms(2u);
    check(proto_is_stale(&d, t_now()) == 1, "mock clock +251 ms -> stale");
    check(proto_is_expired(&d, t_now()) == 0, "long timeout not reached yet");
    t_advance_ms(PROTO_LONG_DEFAULT_MS);
    check(proto_is_expired(&d, t_now()) == 1, "long timeout reached -> expired");

    /* 阈值可配 + 钳位 */
    check(proto_decoder_set_hb_ms(&d, 100u) == PROTO_HB_MIN_MS, "hb clamps to 200");
    check(proto_decoder_set_hb_ms(&d, 9999u) == PROTO_HB_MAX_MS, "hb clamps to 300");
    check(proto_decoder_set_long_ms(&d, 10u) >= PROTO_HB_MIN_MS, "long >= hb");
    check(proto_decoder_set_long_ms(&d, 999999u) == PROTO_LONG_MAX_MS, "long clamps to 10000");
    check(proto_decoder_set_hb_ms(NULL, 250u) == PROTO_HB_DEFAULT_MS, "NULL decoder -> default");

    /* 阈值真的生效：hb=200 时 201 ms 就过期 */
    {
        proto_decoder_t e;
        proto_decoder_init(&e);
        proto_decoder_set_hb_ms(&e, 200u);
        proto_decoder_set_min_interval_ms(&e, 0u);
        base.seq = 9u;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&e, fr, n, 7000u, &out, &er) == PROTO_OK, "accepted");
        check(proto_is_stale(&e, 7000u + 200u) == 0, "hb=200: 200 ms fresh");
        check(proto_is_stale(&e, 7000u + 201u) == 1, "hb=200: 201 ms stale");
    }

    /* t 只是参考：极端 t 值不影响本地新鲜度判定 */
    {
        proto_decoder_t e;
        proto_decoder_init(&e);
        proto_decoder_set_min_interval_ms(&e, 0u);
        base.seq = 1u;
        base.t_ms = 0u;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&e, fr, n, 20000u, &out, &er) == PROTO_OK, "t=0 accepted");
        check(out.t_ms == 0u, "t=0 preserved");
        check(proto_is_stale(&e, 20000u + 100u) == 0, "fresh decides on local time only");
        base.seq = 2u;
        base.t_ms = 0xFFFFFFFFu;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&e, fr, n, 20050u, &out, &er) == PROTO_OK, "t=2^32-1 accepted");
        check(out.t_ms == 0xFFFFFFFFu, "t=2^32-1 preserved");
        check(proto_is_stale(&e, 20050u + 100u) == 0, "still fresh (t is informational)");
        check(proto_is_stale(&e, 20050u + 400u) == 1, "stale after 400 ms of local silence");
    }

    /* 断连计数：中间隔了超过阈值才算一次 */
    {
        proto_decoder_t e;
        proto_decoder_init(&e);
        proto_decoder_set_min_interval_ms(&e, 0u);
        base.seq = 1u;
        base.t_ms = 0u;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&e, fr, n, 30000u, &out, &er) == PROTO_OK, "first");
        base.seq = 2u;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&e, fr, n, 30000u + 100u, &out, &er) == PROTO_OK, "close frame");
        check(e.stats.heartbeat_gaps == 0u, "no gap yet");
        base.seq = 3u;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&e, fr, n, 30000u + 400u, &out, &er) == PROTO_OK, "after a silence");
        check(e.stats.heartbeat_gaps == 1u, "one heartbeat gap counted");
    }

    /* 本地时钟回绕：age 必须回绕安全 */
    {
        proto_decoder_t e;
        proto_decoder_init(&e);
        proto_decoder_set_min_interval_ms(&e, 0u);
        t_clock_goto(0xFFFFFF00u);
        check(t_now() == 0xFFFFFF00u, "mock clock near the wrap");
        base.seq = 1u;
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_decode(&e, fr, n, t_now(), &out, &er) == PROTO_OK, "accepted near the wrap");
        t_advance_ms(356u);
        vchk(t_now() == 100u, "mock clock wrapped (now=%u)", (unsigned)t_now());
        vchk(proto_age_ms(&e, t_now()) == 356u, "age survives the wrap (got %u)",
             (unsigned)proto_age_ms(&e, t_now()));
        check(proto_is_stale(&e, t_now()) == 1, "356 ms across the wrap -> stale");
    }
}

/* ==========================================================================
 * 11. 急停：显式字段 + 无条件可读
 * ========================================================================== */

static void t_estop(void)
{
    proto_decoder_t d;
    char fr[FRAME_CAP];
    proto_cmd_t base;
    proto_cmd_t out;
    proto_error_t er;
    size_t n;
    int est = -1;

    section("estop: explicit field, readable even from a rejected frame");

    mk_base(&base);
    base.seq = 1u;
    base.est = 1;
    n = mk_canon(&base, fr, sizeof(fr));
    proto_decoder_init(&d);
    check(proto_decode(&d, fr, n, 1000u, &out, &er) == PROTO_OK, "estop frame accepted");
    check(out.est == 1, "est field decoded");
    check(d.stats.estop_frames == 1u, "estop_frames counted");

    est = -1;
    check(proto_peek_estop(&d, fr, n, &est) == 1, "peek finds est on a valid frame");
    check(est == 1, "peek value is 1");
    check(d.stats.estop_hits == 1u, "peek hit counted");
    check(d.stats.estop_peeks >= 1u, "peek call counted");

    /* peek：整帧因别的字段被拒，急停照样读得出来（§0.5(7) 旁路）*/
    {
        char bad[FRAME_CAP];
        proto_cmd_t b2;
        proto_cmd_t o2;
        proto_error_t e2;
        mk_base(&b2);
        b2.seq = 2u;
        b2.est = 1;
        (void)mk_canon(&b2, bad, sizeof(bad));
        check(splice_value(bad, sizeof(bad), "spd", "101") == 1, "spd out of range");
        check(dec_strict(bad, strlen(bad), &o2, &e2) == PROTO_E_RANGE, "frame is rejected");
        est = -1;
        check(proto_peek_estop(NULL, bad, strlen(bad), &est) == 1,
              "peek still reads est from a rejected frame");
        check(est == 1, "rejected-frame est is 1");
    }

    est = -1;
    check(proto_peek_estop(NULL, "rdog=1;est=1", 12u, &est) == 1, "peek on a truncated frame");
    check(est == 1, "truncated est");
    est = -1;
    check(proto_peek_estop(NULL, "f=10est=1", 9u, &est) == 1, "peek on the old glued format");
    check(est == 1, "glued est");
    est = -1;
    check(proto_peek_estop(NULL, "est=1abc", 8u, &est) == 1, "peek tolerates trailing junk");
    check(est == 1, "junk-tolerant est");
    est = -1;
    check(proto_peek_estop(NULL, "est=0", 5u, &est) == 1, "peek reads est=0");
    check(est == 0, "est=0 value");
    check(proto_peek_estop(NULL, "est=2", 5u, &est) == 0, "est=2 is not a usable estop");
    check(proto_peek_estop(NULL, "est=x", 5u, &est) == 0, "est=x is not usable");
    check(proto_peek_estop(NULL, "seq=1;t=0\n", 9u, &est) == 0, "no est -> 0");
    check(proto_peek_estop(NULL, "", 0u, &est) == 0, "empty -> 0");
    check(proto_peek_estop(NULL, NULL, 9u, &est) == 0, "NULL -> 0");
    check(proto_peek_estop(NULL, "est=1", 5u, NULL) == 1, "peek without an out pointer");

    /* peek 扫描有界：丢在 512 字节之后的急停读不出来（那种帧本来也不合法）*/
    {
        char *big = (char *)malloc(1024);
        check(big != NULL, "malloc");
        if (big != NULL) {
            memset(big, 'a', 1024);
            memcpy(big + 600, "est=1", 5u);
            check(proto_peek_estop(NULL, big, 1024u, &est) == 0,
                  "est beyond PROTO_MAX_FRAME is not scanned (bounded work)");
            memcpy(big, "est=1", 5u);
            check(proto_peek_estop(NULL, big, 1024u, &est) == 1, "est at the front is found");
            free(big);
        }
    }
}

/* ==========================================================================
 * 12. 旧页面格式（能不能上板验收就看这一节）
 * ========================================================================== */

static void legacy_expect_field(const char *what, const char *req, const char *field, int64_t want)
{
    proto_cmd_t out;
    proto_error_t er;
    const int fi = field_index(field);
    const int rc = dec_legacy(req, strlen(req), &out, &er);

    vchk(rc == PROTO_OK, "%s: '%s' -> OK (got %s)", what, req, proto_err_name(er.code));
    if (rc == PROTO_OK && fi >= 0) {
        vchk(proto_cmd_get(&out, (size_t)fi) == want, "%s: %s == %lld (got %lld)",
             what, field, (long long)want, (long long)proto_cmd_get(&out, (size_t)fi));
        vchk((out.present & (1u << fi)) != 0u, "%s: %s is marked present", what, field);
    }
}

static void legacy_expect_event(const char *what, const char *req, const char *ev)
{
    proto_cmd_t out;
    proto_error_t er;
    const int rc = dec_legacy(req, strlen(req), &out, &er);

    vchk(rc == PROTO_OK, "%s: '%s' -> OK (got %s)", what, req, proto_err_name(er.code));
    if (rc == PROTO_OK) {
        vchk(proto_cmd_has_ev(&out, ev) == 1, "%s: event %s set", what, ev);
    }
}

static void legacy_expect_err(const char *what, const char *req, proto_err_t want)
{
    proto_cmd_t out;
    proto_error_t er;
    const int rc = dec_legacy(req, strlen(req), &out, &er);
    vchk(rc == (int)want, "%s: '%s' -> %s (got %s)", what, req,
         proto_err_name(want), proto_err_name(er.code));
}

static void t_legacy_page(void)
{
    proto_cmd_t out;
    proto_error_t er;

    section("legacy page format (drive.html / control.html)");

    /* ---- 这四条是验收前提：现场页面真的会这么发 ---- */
    legacy_expect_field("dog stick", "f=-100t=20", "spd", -100);
    legacy_expect_field("dog stick", "f=-100t=20", "turn", 20);
    legacy_expect_field("arm stick", "jy=10jx=-5", "ay", 10);
    legacy_expect_field("arm stick", "jy=10jx=-5", "ax", -5);
    legacy_expect_event("key button", "key=btn_stand", "btn_stand");
    legacy_expect_field("grip slider", "grip=50", "grip", 50);

    /* ---- 完整 HTTP 请求行 + 头 ---- */
    legacy_expect_field("full request line", "GET /f=80t=0 HTTP/1.1\r\nHost: 192.168.4.1\r\n\r\n",
                        "spd", 80);
    legacy_expect_field("full request line", "GET /f=80t=0 HTTP/1.1\r\nHost: 192.168.4.1\r\n\r\n",
                        "turn", 0);
    legacy_expect_event("request line with key", "GET /?key=sc HTTP/1.1\r\n\r\n", "sc");
    legacy_expect_field("control.html sliders", "pit=5&rol=-3&yst=18&hgt=90", "pit", 5);
    legacy_expect_field("control.html sliders", "pit=5&rol=-3&yst=18&hgt=90", "rol", -3);
    legacy_expect_field("control.html sliders", "pit=5&rol=-3&yst=18&hgt=90", "yst", 18);
    legacy_expect_field("control.html sliders", "pit=5&rol=-3&yst=18&hgt=90", "hgt", 90);

    /* ---- 命名陷阱：hgt= / pit= 里含有 t= 子串，绝不能被当成 turn ---- */
    {
        const int turn_i = field_index("turn");
        check(dec_legacy("hgt=110", 7u, &out, &er) == PROTO_OK, "hgt=110 decodes");
        check(out.hgt == 110, "hgt == 110");
        vchk((out.present & (1u << turn_i)) == 0u,
             "hgt=110 must NOT be read as t= (web_c.py find('t=') trap)");
        check(dec_legacy("pit=5", 5u, &out, &er) == PROTO_OK, "pit=5 decodes");
        check(out.pit == 5, "pit == 5");
        vchk((out.present & (1u << turn_i)) == 0u, "pit=5 must NOT set turn");
        check(dec_legacy("t=20", 4u, &out, &er) == PROTO_OK, "t=20 decodes as turn");
        check(out.turn == 20, "t == turn in the legacy format");
        vchk((out.present & (1u << field_index("t"))) == 0u,
             "legacy t is the stick, not the sender timestamp");
    }

    /* ---- 原版 _leading_int() 的宽容度（只准在值的尾部）---- */
    legacy_expect_field("trailing junk", "f=10abc", "spd", 10);
    legacy_expect_field("leading plus", "f=+7", "spd", 7);
    legacy_expect_field("leading zeros", "f=007", "spd", 7);
    legacy_expect_field("minus zero", "f=-0", "spd", 0);
    legacy_expect_field("glued three keys", "f=1t=2grip=3", "spd", 1);
    legacy_expect_field("glued three keys", "f=1t=2grip=3", "turn", 2);
    legacy_expect_field("glued three keys", "f=1t=2grip=3", "grip", 3);
    legacy_expect_field("glued est", "f=10est=1", "spd", 10);

    /* ---- 标定键 ---- */
    legacy_expect_event("select leg 1", "key=l1", "l1");
    legacy_expect_event("select leg 2", "key=l2", "l2");
    legacy_expect_event("select leg 3", "key=l3", "l3");
    legacy_expect_event("select leg 4", "key=l4", "l4");
    legacy_expect_event("clear pose (ss)", "key=ss", "ss");
    legacy_expect_event("nudge thigh +", "key=hi", "hi");
    legacy_expect_event("nudge thigh -", "key=hd", "hd");
    legacy_expect_event("nudge shank +", "key=si", "si");
    legacy_expect_event("nudge shank -", "key=sd", "sd");
    legacy_expect_event("nudge hip +", "key=ip", "ip");
    legacy_expect_event("nudge hip -", "key=id", "id");
    legacy_expect_event("direct stand", "key=t9", "t9");
    legacy_expect_event("save to NVS", "key=sc", "sc");
    legacy_expect_event("trot", "key=g0", "g0");
    legacy_expect_event("walk", "key=g1", "g1");
    legacy_expect_event("stable on", "key=go", "go");
    legacy_expect_event("stable off", "key=gc", "gc");
    legacy_expect_event("in-place step", "key=is", "is");
    legacy_expect_event("stand action", "key=btn_stand", "btn_stand");
    legacy_expect_event("sit action", "key=btn_sit", "btn_sit");
    legacy_expect_event("wave action", "key=btn_wave", "btn_wave");
    legacy_expect_event("crawl action", "key=btn_crawl", "btn_crawl");
    legacy_expect_event("arm on", "key=am1", "am1");
    legacy_expect_event("arm off", "key=am0", "am0");

    /* ---- 三个"等价于字段"的旧按键 ---- */
    legacy_expect_field("btn_stop -> spd", "key=btn_stop", "spd", 0);
    legacy_expect_field("btn_stop -> turn", "key=btn_stop", "turn", 0);
    legacy_expect_field("btn_grip_open", "key=btn_grip_open", "grip", 0);
    legacy_expect_field("btn_grip_close", "key=btn_grip_close", "grip", 100);
    check(dec_legacy("key=btn_stop", 12u, &out, &er) == PROTO_OK, "btn_stop decodes");
    check(out.est == 0, "btn_stop is NOT an estop");

    /* ---- 多事件 ---- */
    legacy_expect_event("event list", "key=g0,g1", "g0");
    legacy_expect_event("event list", "key=g0,g1", "g1");

    /* ---- 缺省 != 默认值：旧页面是部分更新 ---- */
    check(dec_legacy("f=40", 4u, &out, &er) == PROTO_OK, "single field frame");
    check(out.spd == 40, "spd present");
    check(out.present != 0u, "present mask not empty");
    check((out.present & (1u << field_index("pit"))) == 0u, "pit absent -> present=0");
    check(out.pit == 0, "pit value is 0 but that means 'not commanded'");
    check((out.present & (1u << field_index("spd"))) != 0u, "spd present bit set");

    /* ---- 严格的部分：键名、范围、事件名 ---- */
    legacy_expect_err("unknown key", "foo=3", PROTO_E_UNKNOWN_KEY);
    legacy_expect_err("unknown key glued", "f=10foo=3", PROTO_E_UNKNOWN_KEY);
    legacy_expect_err("range", "f=999", PROTO_E_RANGE);
    legacy_expect_err("range negative", "f=-101", PROTO_E_RANGE);
    legacy_expect_err("range mode", "mode=3", PROTO_E_RANGE);
    legacy_expect_err("non numeric", "f=abc", PROTO_E_BAD_INT);
    legacy_expect_err("empty value", "f=", PROTO_E_BAD_INT);
    legacy_expect_err("empty key command", "key=", PROTO_E_EMPTY_VALUE);
    legacy_expect_err("unknown event", "key=zz", PROTO_E_EVENT_UNKNOWN);
    legacy_expect_err("page request", "GET /control.html HTTP/1.1\r\n\r\n", PROTO_E_NO_FIELDS);
    legacy_expect_err("favicon", "GET /favicon.ico HTTP/1.1\r\n\r\n", PROTO_E_NO_FIELDS);
    legacy_expect_err("empty", "", PROTO_E_EMPTY);
    legacy_expect_err("conflict btn_stop + f", "f=50&key=btn_stop", PROTO_E_DUP_KEY);

    /* ---- 超长 ---- */
    {
        char *big = (char *)malloc(PROTO_MAX_LEGACY + 200u);
        check(big != NULL, "malloc");
        if (big != NULL) {
            memset(big, 'f', PROTO_MAX_LEGACY + 199u);
            big[PROTO_MAX_LEGACY + 199u] = '\0';
            vchk(dec_legacy(big, strlen(big), &out, &er) == PROTO_E_TOO_LONG,
                 "oversized legacy frame -> TOO_LONG (got %s)", proto_err_name(er.code));
            free(big);
        }
    }

    /* ---- 旧格式会刷新心跳（和严格格式共用一个解码器）---- */
    {
        proto_decoder_t d;
        proto_decoder_init(&d);
        check(proto_decode_legacy(&d, "f=10", 4u, 4000u, &out, &er) == PROTO_OK, "legacy accepted");
        check(proto_is_stale(&d, 4000u + 100u) == 0, "legacy frame feeds the heartbeat");
        check(proto_is_stale(&d, 4000u + 400u) == 1, "and it goes stale too");
        check(d.stats.legacy_frames == 1u, "legacy_frames counted");
        check(proto_decode_legacy(&d, "f=11", 4u, 4500u, &out, &er) == PROTO_OK,
              "legacy has no seq, so it is never dropped for staleness");
        check(d.stats.dropped_seq == 0u, "no seq drops on the legacy path");
        check(d.stats.frames == d.stats.accepted + d.stats.rejected, "counters stay consistent");
    }

    /* ---- 两种格式互不串味 ---- */
    {
        proto_cmd_t base;
        char fr[FRAME_CAP];
        size_t n;
        mk_base(&base);
        n = mk_canon(&base, fr, sizeof(fr));
        check(proto_sniff_format(fr, n) == PROTO_FMT_STRICT, "sniff: strict frame");
        check(proto_sniff_format("f=10t=20", 8u) == PROTO_FMT_LEGACY, "sniff: old page frame");
        check(proto_sniff_format("GET /f=1 HTTP/1.1", 17u) == PROTO_FMT_LEGACY, "sniff: HTTP line");
        check(proto_sniff_format("hello world", 11u) == PROTO_FMT_UNKNOWN, "sniff: junk");
        check(proto_sniff_format("", 0u) == PROTO_FMT_UNKNOWN, "sniff: empty");
        check(proto_sniff_format(NULL, 5u) == PROTO_FMT_UNKNOWN, "sniff: NULL");

        poison(&out);
        vchk(dec_strict("f=-100t=20\n", 11u, &out, &er) == PROTO_E_UNKNOWN_KEY,
             "strict parser rejects the old keys (got %s)", proto_err_name(er.code));
        check(strcmp(er.key, "f") == 0, "strict rejection names the old key");
        poison(&out);
        vchk(dec_legacy(fr, n, &out, &er) == PROTO_E_UNKNOWN_KEY,
             "legacy parser rejects the strict magic key (got %s)", proto_err_name(er.code));
    }
}

/* ==========================================================================
 * 13. 原版命令面覆盖（独立第二来源：读的是 micropython 的 .py）
 *
 * 这张表**从原版 Python 读出来**（web_common.CTL_KEYS + handle_control_key()
 * + web_c.py 的标定键 + 网页里的 f/t/jy/jx/grip/pit/rol/yst/hgt），
 * 和 proto 的表不是同一份东西 —— 所以它能抓"协议漏了一个键"（P-23 那种
 * 两个来源互相独立地对照的思路）。每行给出：原版键 -> 新协议怎么表达 -> 期望结果。
 * ========================================================================== */

typedef enum { COV_EV, COV_FLD } cov_kind_t;

typedef struct {
    const char *src;    /**< 原版键 */
    const char *frame;  /**< 旧格式报文（现场页面就是这么发的）*/
    cov_kind_t  kind;
    const char *target; /**< 事件名 或 规范字段名 */
    int64_t     value;  /**< COV_FLD 时的期望值 */
} cov_t;

static const cov_t kCoverage[] = {
    /* ---- 摇杆：f/t = 狗，jy/jx = 臂，grip = 夹爪（§0.5(3)）---- */
    { "f",              "f=-100",              COV_FLD, "spd",     -100 },
    { "t",              "t=-100",              COV_FLD, "turn",    -100 },
    { "jy",             "jy=10",               COV_FLD, "ay",        10 },
    { "jx",             "jx=-5",               COV_FLD, "ax",        -5 },
    { "grip",           "grip=50",             COV_FLD, "grip",      50 },
    /* ---- 姿态 / 身高 ---- */
    { "pit",            "pit=5",               COV_FLD, "pit",        5 },
    { "rol",            "rol=-3",              COV_FLD, "rol",       -3 },
    { "yst",            "yst=18",              COV_FLD, "yst",       18 },
    { "hgt",            "hgt=90",              COV_FLD, "hgt",       90 },
    /* ---- web_common.CTL_KEYS + handle_control_key() ---- */
    { "g0",             "key=g0",              COV_EV,  "g0",         0 },
    { "g1",             "key=g1",              COV_EV,  "g1",         0 },
    { "go",             "key=go",              COV_EV,  "go",         0 },
    { "gc",             "key=gc",              COV_EV,  "gc",         0 },
    { "is",             "key=is",              COV_EV,  "is",         0 },
    { "ss",             "key=ss",              COV_EV,  "ss",         0 },
    { "btn_stand",      "key=btn_stand",       COV_EV,  "btn_stand",  0 },
    { "btn_sit",        "key=btn_sit",         COV_EV,  "btn_sit",    0 },
    { "btn_wave",       "key=btn_wave",        COV_EV,  "btn_wave",   0 },
    { "btn_crawl",      "key=btn_crawl",       COV_EV,  "btn_crawl",  0 },
    { "am1",            "key=am1",             COV_EV,  "am1",        0 },
    { "am0",            "key=am0",             COV_EV,  "am0",        0 },
    /* ---- 旧遥控页那两个按键：等价于 grip 的两个端点 ---- */
    { "btn_grip_open",  "key=btn_grip_open",   COV_FLD, "grip",       0 },
    { "btn_grip_close", "key=btn_grip_close",  COV_FLD, "grip",     100 },
    /* ---- btn_stop = 中性快照（§0.5(7) 把它定义为"短超时"的语义）---- */
    { "btn_stop",       "key=btn_stop",        COV_FLD, "spd",        0 },
    /* ---- 标定键（web_c.py）---- */
    { "l1",             "key=l1",              COV_EV,  "l1",         0 },
    { "l2",             "key=l2",              COV_EV,  "l2",         0 },
    { "l3",             "key=l3",              COV_EV,  "l3",         0 },
    { "l4",             "key=l4",              COV_EV,  "l4",         0 },
    { "hi",             "key=hi",              COV_EV,  "hi",         0 },
    { "hd",             "key=hd",              COV_EV,  "hd",         0 },
    { "si",             "key=si",              COV_EV,  "si",         0 },
    { "sd",             "key=sd",              COV_EV,  "sd",         0 },
    { "ip",             "key=ip",              COV_EV,  "ip",         0 },
    { "id",             "key=id",              COV_EV,  "id",         0 },
    { "t9",             "key=t9",              COV_EV,  "t9",         0 },
    { "sc",             "key=sc",              COV_EV,  "sc",         0 },
};

static void t_coverage(void)
{
    size_t i;
    size_t n_ok = 0;

    section("coverage: every key of the original surface is expressible");

    for (i = 0; i < sizeof(kCoverage) / sizeof(kCoverage[0]); ++i) {
        const cov_t *c = &kCoverage[i];
        proto_cmd_t out;
        proto_error_t er;
        const int rc = dec_legacy(c->frame, strlen(c->frame), &out, &er);

        vchk(rc == PROTO_OK, "original key '%s' (as '%s') must parse (got %s)",
             c->src, c->frame, proto_err_name(er.code));
        if (rc == PROTO_OK) {
            ++n_ok;
            if (c->kind == COV_EV) {
                const int ei = proto_event_find(c->target);
                vchk(ei >= 0, "original key '%s': event '%s' exists", c->src, c->target);
                vchk(proto_cmd_has_ev(&out, c->target) == 1,
                     "original key '%s' -> event '%s'", c->src, c->target);
            } else {
                const int fi = field_index(c->target);
                vchk(fi >= 0, "original key '%s': field '%s' exists", c->src, c->target);
                if (fi >= 0) {
                    vchk(proto_cmd_get(&out, (size_t)fi) == c->value,
                         "original key '%s' -> %s=%lld (got %lld)", c->src, c->target,
                         (long long)c->value, (long long)proto_cmd_get(&out, (size_t)fi));
                }
            }
        }
    }
    printf("  original keys covered: %u/%u\n", (unsigned)n_ok,
           (unsigned)(sizeof(kCoverage) / sizeof(kCoverage[0])));

    /* 反向：协议里的事件不能在原版里凭空多出来（多了说明我发明了命令）*/
    for (i = 0; i < g_ne; ++i) {
        size_t k;
        int found = 0;
        for (k = 0; k < sizeof(kCoverage) / sizeof(kCoverage[0]); ++k) {
            if (kCoverage[k].kind == COV_EV && strcmp(kCoverage[k].target, g_ev[i]) == 0) {
                found = 1;
                break;
            }
        }
        vchk(found == 1, "event '%s' traces back to an original key (no invented commands)",
             g_ev[i]);
    }
}

/* ==========================================================================
 * 14. Fuzz：固定种子、确定性、不崩、不污染
 * ========================================================================== */

#define FUZZ_ITERS 2500

static void t_fuzz(void)
{
    char frame[1024];
    char mut[1024];
    uint32_t hash = 2166136261u;
    uint32_t n_ok = 0;
    uint32_t n_bad = 0;
    size_t i;

    section("fuzz: deterministic, never crashes, never corrupts, always consistent");

    s_rng = 0x1234ABCDu;                 /* 固定种子：重跑必须一模一样 */

    for (i = 0; i < FUZZ_ITERS; ++i) {
        const uint32_t mode = rnd() % 3u;
        size_t len = 0;
        proto_cmd_t good;
        int built = 0;

        if (mode == 0u) {
            /* [A] 纯随机字节 */
            size_t k;
            len = (size_t)(rnd() % 700u);
            for (k = 0; k < len; ++k) {
                frame[k] = (char)(rnd() & 0xFFu);
            }
            if ((rnd() & 1u) != 0u && len > 0u) {
                frame[len - 1u] = '\n';       /* 一半概率给个终止符，走得更深 */
            }
        } else if (mode == 1u) {
            /* [B] 随机合法命令（保证 accept 路径真的被走到）*/
            size_t fi;
            mk_base(&good);
            good.seq = rnd();
            good.t_ms = rnd();
            for (fi = 0; fi < g_nf; ++fi) {
                if (rnd() & 1u) {
                    (void)proto_cmd_set(&good, fi, rand_in(&g_fs[fi]));
                }
            }
            if ((rnd() & 1u) != 0u) {
                good.ev_mask = rnd() & (uint32_t)g_fs[field_index("ev")].max;
            }
            len = proto_encode(&good, frame, sizeof(frame));
            built = 1;
        } else {
            /* [C] 结构垃圾：随机键 + 随机取值（有些越界）*/
            size_t nf = 1u + (rnd() % 6u);
            size_t k;
            len = 0;
            for (k = 0; k < nf && len + 40u < sizeof(frame); ++k) {
                const size_t fi = rnd() % g_nf;
                int64_t v;
                int n;
                if (rnd() & 1u) {
                    v = rand_in(&g_fs[fi]);
                } else {
                    v = g_fs[fi].max + 1 + (int64_t)(rnd() % 5u);   /* 越界 */
                }
                if (g_fs[fi].kind == PROTO_KIND_EVLIST) {
                    n = snprintf(frame + len, sizeof(frame) - len, "%s=%s%s",
                                 g_fs[fi].key,
                                 ((rnd() & 1u) != 0u) ? g_ev[rnd() % g_ne] : "zz",
                                 (k + 1u < nf) ? ";" : "\n");
                } else {
                    n = snprintf(frame + len, sizeof(frame) - len, "%s=%lld%s",
                                 g_fs[fi].key, (long long)v, (k + 1u < nf) ? ";" : "\n");
                }
                if (n <= 0) {
                    break;
                }
                len += (size_t)n;
            }
            if (len > sizeof(frame)) {
                len = sizeof(frame);
            }
        }

        /* --- 同一个输入用两个全新解码器 + 一次重复，必须给同一个判定 --- */
        {
            proto_cmd_t a, b, c3;
            proto_cmd_t snap;
            proto_error_t ea, eb, ec;
            int ra, rb, rc;

            poison(&a);
            memcpy(&snap, &a, sizeof(a));
            poison(&b);
            poison(&c3);

            ra = dec_strict(frame, len, &a, &ea);
            rb = dec_strict(frame, len, &b, &eb);
            rc = dec_strict(frame, len, &c3, &ec);

            vchk(ra == rb && ra == rc, "iter %u: verdict not reproducible (%s/%s/%s)",
                 (unsigned)i, proto_err_name(ea.code), proto_err_name(eb.code),
                 proto_err_name(ec.code));
            vchk(ea.field == eb.field && strcmp(ea.key, eb.key) == 0,
                 "iter %u: error detail not reproducible", (unsigned)i);
            vchk(memcmp(&a, &b, sizeof(a)) == 0, "iter %u: out struct differs", (unsigned)i);

            if (ra != PROTO_OK) {
                ++n_bad;
                vchk(untouched(&a, &snap), "iter %u: rejected frame modified out", (unsigned)i);
            } else {
                ++n_ok;
                {
                    size_t fi;
                    for (fi = 0; fi < g_nf; ++fi) {
                        const int64_t v = proto_cmd_get(&a, fi);
                        vchk(v >= g_fs[fi].min && v <= g_fs[fi].max,
                             "iter %u: accepted %s out of range", (unsigned)i, g_fs[fi].key);
                    }
                }
                {
                    char again[FRAME_CAP];
                    proto_cmd_t back;
                    proto_error_t e2;
                    proto_decoder_t d2;
                    const size_t n2 = proto_encode(&a, again, sizeof(again));
                    vchk(n2 > 0, "iter %u: accepted frame re-encodes", (unsigned)i);
                    proto_decoder_init(&d2);
                    vchk(proto_decode(&d2, again, n2, 1000u, &back, &e2) == PROTO_OK,
                         "iter %u: canonical re-decode", (unsigned)i);
                    vchk(proto_cmd_equal(&back, &a), "iter %u: canonical round trip",
                         (unsigned)i);
                }
                if (mode == 1u && built) {
                    vchk(proto_cmd_equal(&a, &good), "iter %u: a built command round-trips",
                         (unsigned)i);
                }
            }
            hash = (hash ^ (uint32_t)ra) * 16777619u;
            hash = (hash ^ (uint32_t)(ea.field + 1)) * 16777619u;
            hash = (hash ^ (uint32_t)len) * 16777619u;
        }

        /* --- 变异：合法帧上随机改/删/插，重新走一遍同样的检查 --- */
        if (built && len > 2u) {
            size_t nmut = 1u + (rnd() % 4u);
            size_t l = len;
            size_t k;
            proto_cmd_t m1, m2;
            proto_cmd_t snap;
            proto_error_t e1, e2;
            int r1, r2;

            memcpy(mut, frame, len);
            for (k = 0; k < nmut; ++k) {
                const uint32_t op = rnd() % 3u;
                if (op == 0u && l > 0u) {
                    static const char pool[] = "=;,\n-+0123456789abcdefzZ \t";
                    mut[rnd() % l] = pool[rnd() % (sizeof(pool) - 1u)];
                } else if (op == 1u && l > 1u) {
                    const size_t at = rnd() % l;
                    memmove(mut + at, mut + at + 1u, l - at - 1u);
                    --l;
                } else if (l + 1u < sizeof(mut)) {
                    static const char pool[] = "=;,\n-+0123456789abcdefzZ";
                    const size_t at = rnd() % (l + 1u);
                    memmove(mut + at + 1u, mut + at, l - at);
                    mut[at] = pool[rnd() % (sizeof(pool) - 1u)];
                    ++l;
                }
            }
            poison(&m1);
            memcpy(&snap, &m1, sizeof(m1));
            poison(&m2);
            r1 = dec_strict(mut, l, &m1, &e1);
            r2 = dec_strict(mut, l, &m2, &e2);
            vchk(r1 == r2 && memcmp(&m1, &m2, sizeof(m1)) == 0,
                 "iter %u: mutant verdict not reproducible", (unsigned)i);
            if (r1 != PROTO_OK) {
                vchk(untouched(&m1, &snap), "iter %u: rejected mutant modified out", (unsigned)i);
            }
            hash = (hash ^ (uint32_t)r1) * 16777619u;
        }
    }

    printf("  iterations=%u accepted=%u rejected=%u hash=0x%08X\n",
           (unsigned)FUZZ_ITERS, (unsigned)n_ok, (unsigned)n_bad, (unsigned)hash);
    check(n_ok > 0, "fuzz actually exercised the accept path");
    check(n_bad > 0, "fuzz actually exercised the reject path");
}

/* ==========================================================================
 * 15. 计数可回读（§0.5(7)：接受 / 丢弃 / 过期 / 坏帧）
 * ========================================================================== */

static void t_counters(void)
{
    proto_decoder_t d;
    char fr[FRAME_CAP];
    proto_cmd_t base;
    proto_cmd_t out;
    proto_error_t er;
    size_t n;

    section("counters: accepted / dropped / expired / bad frames are observable");

    mk_base(&base);
    proto_decoder_init(&d);
    check(d.stats.frames == 0u && d.stats.accepted == 0u, "fresh decoder starts at zero");

    base.seq = 1u;
    n = mk_canon(&base, fr, sizeof(fr));
    check(proto_decode(&d, fr, n, 1000u, &out, &er) == PROTO_OK, "accepted");
    check(proto_decode(&d, "garbage\n", 8u, 1100u, &out, &er) != PROTO_OK, "bad frame");
    {
        char bad[FRAME_CAP];
        mk_base(&base);
        base.seq = 2u;
        (void)mk_canon(&base, bad, sizeof(bad));
        check(splice_value(bad, sizeof(bad), "hgt", "500") == 1, "splice hgt=500");
        check(proto_decode(&d, bad, strlen(bad), 1200u, &out, &er) == PROTO_E_RANGE, "bad field");
    }
    base.seq = 1u;
    n = mk_canon(&base, fr, sizeof(fr));
    check(proto_decode(&d, fr, n, 1300u, &out, &er) == PROTO_E_SEQ_STALE, "dropped");
    /* 被丢弃的帧不刷新心跳，所以 1400 这一帧距上一次**被接受**的帧已经 400 ms */
    base.seq = 3u;
    n = mk_canon(&base, fr, sizeof(fr));
    check(proto_decode(&d, fr, n, 1400u, &out, &er) == PROTO_OK, "accepted (and counted as a gap)");
    /* 紧跟着的 1 ms 帧：限速丢（限速以"上一帧被接受"为基准）*/
    base.seq = 4u;
    n = mk_canon(&base, fr, sizeof(fr));
    check(proto_decode(&d, fr, n, 1401u, &out, &er) == PROTO_E_RATE_LIMIT, "rate limited");
    check(proto_decode(&d, fr, n, 1400u + 5000u, &out, &er) == PROTO_OK, "accepted after a silence");
    check(d.stats.heartbeat_gaps >= 1u, "a heartbeat gap was counted");

    printf("  frames=%u accepted=%u rejected=%u dropped_seq=%u bad_frame=%u bad_field=%u\n",
           (unsigned)d.stats.frames, (unsigned)d.stats.accepted, (unsigned)d.stats.rejected,
           (unsigned)d.stats.dropped_seq, (unsigned)d.stats.bad_frame,
           (unsigned)d.stats.bad_field);
    printf("  rate_limited=%u heartbeat_gaps=%u estop_frames=%u legacy=%u\n",
           (unsigned)d.stats.rate_limited, (unsigned)d.stats.heartbeat_gaps,
           (unsigned)d.stats.estop_frames, (unsigned)d.stats.legacy_frames);

    check(d.stats.accepted == 3u, "accepted == 3");
    check(d.stats.dropped_seq == 1u, "dropped_seq == 1");
    check(d.stats.bad_frame == 1u, "bad_frame == 1");
    check(d.stats.bad_field == 1u, "bad_field == 1");
    check(d.stats.rate_limited == 1u, "rate_limited == 1");
    check(d.stats.frames == 7u, "frames == 7");
    check(d.stats.frames == d.stats.accepted + d.stats.rejected, "frames identity");
    check(d.stats.rejected == d.stats.bad_frame + d.stats.bad_field + d.stats.dropped_seq
                              + d.stats.rate_limited, "category identity");

    /* 空指针：不计入统计（编程错误，不是收到了一帧）*/
    {
        const uint32_t before = d.stats.frames;
        check(proto_decode(NULL, fr, n, 2000u, &out, &er) == PROTO_E_NULL, "NULL decoder");
        check(proto_decode(&d, NULL, n, 2000u, &out, &er) == PROTO_E_NULL, "NULL buffer");
        check(proto_decode(&d, fr, n, 2000u, NULL, &er) == PROTO_E_NULL, "NULL out");
        check(d.stats.frames == before, "NULL-arg calls are not counted");
        check(er.code == PROTO_E_NULL || er.code == PROTO_OK, "err filled");
        check(proto_decode(&d, "garbage\n", 8u, 2100u, &out, NULL) != PROTO_OK,
              "NULL err is allowed");
    }

    /* err 在成功时被清零 */
    mk_base(&base);
    base.seq = 500u;
    n = mk_canon(&base, fr, sizeof(fr));
    er.code = PROTO_E_RANGE;
    er.field = 7;
    strcpy(er.key, "zz");
    check(proto_decode(&d, fr, n, 9000u, &out, &er) == PROTO_OK, "accepted");
    check(er.code == PROTO_OK && er.field == -1 && er.key[0] == '\0', "err cleared on success");
}

/* ==========================================================================
 * main
 * ========================================================================== */

int main(void)
{
    printf("========================================================\n");
    printf(" proto host test  (pure C, mock clock -- no board needed)\n");
    printf("========================================================\n");

    t_table();
    t_limits();
    t_canonical_frame();
    t_roundtrip();
    t_range();
    t_keys();
    t_values();
    t_truncation();
    t_framing();
    t_seq();
    t_rate_limit();
    t_heartbeat();
    t_estop();
    t_legacy_page();
    t_coverage();
    t_fuzz();
    t_counters();

    section("(end)");

    printf("\n========================================================\n");
    printf(" checks=%d  failures=%d\n", g_checks, g_fail);
    if (g_fail > 0) {
        printf("RESULT: FAIL -- %d checks failed\n", g_fail);
        return 1;
    }
    printf("RESULT: PASS -- all %d checks passed\n", g_checks);
    printf("\nNOTE: this suite proves the PARSER/VALIDATOR. It cannot prove that\n");
    printf("      the old page + a real HTTP server behave on the board -- that\n");
    printf("      is verified on hardware (see tools/golden/README.md).\n");
    return 0;
}
