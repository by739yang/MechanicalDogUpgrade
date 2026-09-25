/**
 * @file    proto.c
 * @brief   P5 机器人命令协议实现（严格格式 + 旧页面格式，共用一张字段表）
 *
 * 设计要点（详见 proto.h 的注释）：
 *   - **零 ESP-IDF 依赖**：时钟由调用方传入，所以宿主测试能用可控时钟跑时间逻辑。
 *   - **没有动态分配、没有全局可变状态**：状态全在调用方的 `proto_decoder_t` 里。
 *   - **键名 / 范围 / 事件名只有一份**（`kFields` / `kEvents`）：解析和编码都走它。
 *     `rdog` / `est` / `seq` 这三个特殊字段靠**角色**（`proto_role_t`）识别，
 *     所以代码里不会再出现第二个 "rdog" / "est" 字面量（P-22/P-27）。
 *   - **拒绝即整帧拒绝**：先在局部 `parse_t` 里凑，全部通过才 `*out = ...`，
 *     失败时输出结构体一个字节都不动。
 *   - 对**任意字节序列**安全：长度有上限、逐字节扫描、不越界、不读未初始化内存。
 */

#include "comm/proto.h"

#include <string.h>

/* ==========================================================================
 * 编译期保护：位掩码放得下
 * ========================================================================== */

_Static_assert(PROTO_VERSION >= 1, "protocol version starts at 1");
_Static_assert(sizeof(uint32_t) == 4u, "present/seen masks assume 32-bit");

/* ==========================================================================
 * 错误码 → 名字 / 分类
 * ========================================================================== */

/* 用指定下标初始化：顺序与枚举解耦；漏写一项会在测试里被"名字必须是 '?'"抓出来 */
static const char *const kErrNames[PROTO_E_COUNT] = {
    [PROTO_OK]               = "OK",
    [PROTO_E_NULL]           = "NULL",
    [PROTO_E_EMPTY]          = "EMPTY",
    [PROTO_E_NO_FIELDS]      = "NO_FIELDS",
    [PROTO_E_TOO_LONG]       = "TOO_LONG",
    [PROTO_E_NO_TERMINATOR]  = "NO_TERMINATOR",
    [PROTO_E_CTRL_CHAR]      = "CTRL_CHAR",
    [PROTO_E_BAD_SEPARATOR]  = "BAD_SEPARATOR",
    [PROTO_E_NO_EQUALS]      = "NO_EQUALS",
    [PROTO_E_BAD_KEY]        = "BAD_KEY",
    [PROTO_E_UNKNOWN_KEY]    = "UNKNOWN_KEY",
    [PROTO_E_DUP_KEY]        = "DUP_KEY",
    [PROTO_E_EMPTY_VALUE]    = "EMPTY_VALUE",
    [PROTO_E_BAD_INT]        = "BAD_INT",
    [PROTO_E_RANGE]          = "RANGE",
    [PROTO_E_BAD_VERSION]    = "BAD_VERSION",
    [PROTO_E_EVENT_BAD_NAME] = "EVENT_BAD_NAME",
    [PROTO_E_EVENT_UNKNOWN]  = "EVENT_UNKNOWN",
    [PROTO_E_EVENT_DUP]      = "EVENT_DUP",
    [PROTO_E_MISSING_KEY]    = "MISSING_KEY",
    [PROTO_E_SEQ_STALE]      = "SEQ_STALE",
    [PROTO_E_RATE_LIMIT]     = "RATE_LIMIT",
};

const char *proto_err_name(proto_err_t code)
{
    if ((int)code < 0 || (int)code >= (int)PROTO_E_COUNT) {
        return "?";
    }
    return (kErrNames[code] != NULL) ? kErrNames[code] : "?";
}

proto_err_class_t proto_err_class(proto_err_t code)
{
    switch (code) {
    case PROTO_E_UNKNOWN_KEY:
    case PROTO_E_DUP_KEY:
    case PROTO_E_EMPTY_VALUE:
    case PROTO_E_BAD_INT:
    case PROTO_E_RANGE:
    case PROTO_E_BAD_VERSION:
    case PROTO_E_EVENT_BAD_NAME:
    case PROTO_E_EVENT_UNKNOWN:
    case PROTO_E_EVENT_DUP:
    case PROTO_E_MISSING_KEY:
        return PROTO_CLS_FIELD;
    case PROTO_E_SEQ_STALE:
        return PROTO_CLS_SEQ;
    case PROTO_E_RATE_LIMIT:
        return PROTO_CLS_RATE;
    case PROTO_OK:
    case PROTO_E_NULL:
    case PROTO_E_EMPTY:
    case PROTO_E_NO_FIELDS:
    case PROTO_E_TOO_LONG:
    case PROTO_E_NO_TERMINATOR:
    case PROTO_E_CTRL_CHAR:
    case PROTO_E_BAD_SEPARATOR:
    case PROTO_E_NO_EQUALS:
    case PROTO_E_BAD_KEY:
    case PROTO_E_COUNT:
    default:
        return PROTO_CLS_FRAME;
    }
}

/* ==========================================================================
 * 事件白名单（唯一一份；顺序就是 `ev=` 编码时的规范顺序）
 * ========================================================================== */

static const char *const kEvents[] = {
    "go", "gc", "g0", "g1", "is", "ss",
    "btn_stand", "btn_sit", "btn_wave", "btn_crawl",
    "t9", "sc",
    "l1", "l2", "l3", "l4",
    "hi", "hd", "si", "sd", "ip", "id",
    "am1", "am0",
};

#define NFIELDS (sizeof(kFields) / sizeof(kFields[0]))
#define NEVENTS (sizeof(kEvents) / sizeof(kEvents[0]))

_Static_assert(NEVENTS <= PROTO_MAX_EVENTS, "too many events for the 32-bit mask");

/* ==========================================================================
 * 字段表（唯一一份；顺序就是严格帧的规范键序）
 * ========================================================================== */

#define OFF(m) offsetof(proto_cmd_t, m)

static const proto_field_t kFields[] = {
    /*   key      legacy  kind               required           role                uns  off        min   max                    scale          */
    { "rdog",    NULL,   PROTO_KIND_INT,    PROTO_REQ_STRICT,  PROTO_ROLE_MAGIC,   1, OFF(rdog),     PROTO_VERSION, PROTO_VERSION, "magic+version" },
    { "seq",     NULL,   PROTO_KIND_INT,    PROTO_REQ_STRICT,  PROTO_ROLE_SEQ,     1, OFF(seq),      0, 4294967295LL,          "count" },
    { "t",       NULL,   PROTO_KIND_INT,    PROTO_REQ_STRICT,  PROTO_ROLE_NONE,    1, OFF(t_ms),     0, 4294967295LL,          "ms (sender)" },
    { "mode",    "mode", PROTO_KIND_INT,    PROTO_REQ_OPTIONAL,PROTO_ROLE_NONE,    0, OFF(mode),     0, 2,                     "0=pose 1=chain 2=action" },
    { "est",     "est",  PROTO_KIND_INT,    PROTO_REQ_STRICT,  PROTO_ROLE_ESTOP,   0, OFF(est),      0, 1,                     "1=estop" },
    { "spd",     "f",    PROTO_KIND_INT,    PROTO_REQ_STRICT,  PROTO_ROLE_NONE,    0, OFF(spd),    -100, 100,                   "percent" },
    { "turn",    "t",    PROTO_KIND_INT,    PROTO_REQ_STRICT,  PROTO_ROLE_NONE,    0, OFF(turn),   -100, 100,                   "percent" },
    { "ay",      "jy",   PROTO_KIND_INT,    PROTO_REQ_STRICT,  PROTO_ROLE_NONE,    0, OFF(ay),     -100, 100,                   "percent" },
    { "ax",      "jx",   PROTO_KIND_INT,    PROTO_REQ_STRICT,  PROTO_ROLE_NONE,    0, OFF(ax),     -100, 100,                   "percent" },
    { "grip",    "grip", PROTO_KIND_INT,    PROTO_REQ_STRICT,  PROTO_ROLE_NONE,    0, OFF(grip),     0, 100,                   "percent" },
    { "pit",     "pit",  PROTO_KIND_INT,    PROTO_REQ_STRICT,  PROTO_ROLE_NONE,    0, OFF(pit),     -15, 15,                    "deg" },
    { "rol",     "rol",  PROTO_KIND_INT,    PROTO_REQ_STRICT,  PROTO_ROLE_NONE,    0, OFF(rol),     -15, 15,                    "deg" },
    { "yst",     "yst",  PROTO_KIND_INT,    PROTO_REQ_STRICT,  PROTO_ROLE_NONE,    0, OFF(yst),     -40, 40,                    "mm" },
    { "hgt",     "hgt",  PROTO_KIND_INT,    PROTO_REQ_STRICT,  PROTO_ROLE_NONE,    0, OFF(hgt),      70, 110,                   "mm" },
    { "ev",      "key",  PROTO_KIND_EVLIST, PROTO_REQ_STRICT,  PROTO_ROLE_NONE,    1, OFF(ev_mask),  0, (int64_t)((1u << NEVENTS) - 1u), "event mask" },
};

_Static_assert(NFIELDS <= PROTO_MAX_FIELDS, "too many fields for the 32-bit mask");
_Static_assert(NFIELDS <= 31u, "one bit must stay free for sentinel use");

/* ==========================================================================
 * 旧页面里"没有独立事件"的三个按键：等价于若干字段赋值（只有旧路径认）
 * ========================================================================== */

typedef enum {
    PROTO_ALIAS_VALUE = 0, /**< 用表里的字面值 */
    PROTO_ALIAS_MIN,       /**< 用该字段范围的下限 */
    PROTO_ALIAS_MAX        /**< 用该字段范围的上限 */
} proto_alias_use_t;

typedef struct {
    const char        *name;  /**< 旧页面 `key=` 里的名字 */
    const char        *field; /**< 规范键名（用 proto_field_find 解析，不重复写范围）*/
    int32_t            value;
    proto_alias_use_t  use;
} proto_legacy_cmd_t;

static const proto_legacy_cmd_t kLegacyCmd[] = {
    /* `btn_stop` = set_joy_turn(0) + move(0,0,0) = 中性快照（§0.5(7) 把"短超时"
     * 的行为直接定义为 btn_stop 语义）；它不是急停，急停是 `est` 字段。 */
    { "btn_stop",       "spd",  0, PROTO_ALIAS_VALUE },
    { "btn_stop",       "turn", 0, PROTO_ALIAS_VALUE },
    /* 夹爪开/合 = grip 的两个端点，值直接取字段表的 min/max，不再抄一遍 0/100 */
    { "btn_grip_open",  "grip", 0, PROTO_ALIAS_MIN },
    { "btn_grip_close", "grip", 0, PROTO_ALIAS_MAX },
};

#define NLEGACYCMD (sizeof(kLegacyCmd) / sizeof(kLegacyCmd[0]))

/* ==========================================================================
 * 解析辅助
 * ========================================================================== */

static int is_lower(int c) { return c >= 'a' && c <= 'z'; }
static int is_digit(int c) { return c >= '0' && c <= '9'; }

/** 标识符字符（键名/事件名）。非 ASCII 一律不算 —— 它们是垃圾，不是名字。 */
static int ident_char(int c)
{
    return is_lower(c) || is_digit(c) || c == '_';
}

static int ident_start(int c)
{
    return is_lower(c);
}

/** 名字合法性：非空、不超过 `max`、首字符小写字母、其余 [a-z0-9_] */
static int name_ok(const char *s, size_t n, size_t max)
{
    size_t i;
    if (n == 0 || n > max) {
        return 0;
    }
    if (!ident_start((unsigned char)s[0])) {
        return 0;
    }
    for (i = 1; i < n; ++i) {
        if (!ident_char((unsigned char)s[i])) {
            return 0;
        }
    }
    return 1;
}

/** 表查找：`legacy` = 1 时按旧键名找（NULL 项表示该字段旧路径不认）*/
static int field_find_n(const char *key, size_t n, int legacy)
{
    size_t i;
    for (i = 0; i < NFIELDS; ++i) {
        const char *k = legacy ? kFields[i].legacy : kFields[i].key;
        if (k == NULL) {
            continue;
        }
        if (strlen(k) == n && memcmp(k, key, n) == 0) {
            return (int)i;
        }
    }
    return -1;
}

static int event_find_n(const char *name, size_t n)
{
    size_t i;
    for (i = 0; i < NEVENTS; ++i) {
        if (strlen(kEvents[i]) == n && memcmp(kEvents[i], name, n) == 0) {
            return (int)i;
        }
    }
    return -1;
}

static int64_t load_field(const proto_cmd_t *c, const proto_field_t *f);

static int field_by_role(proto_role_t role)
{
    size_t i;
    for (i = 0; i < NFIELDS; ++i) {
        if (kFields[i].role == role) {
            return (int)i;
        }
    }
    return -1;
}

/** 按类型找字段（事件表那个字段）—— 代码里同样不出现它的键名字面量 */
static int field_by_kind(proto_kind_t kind)
{
    size_t i;
    for (i = 0; i < NFIELDS; ++i) {
        if (kFields[i].kind == kind) {
            return (int)i;
        }
    }
    return -1;
}

/** 这一帧带的事件掩码（按类型找字段，不写 "ev" 字面量）*/
static uint32_t ev_mask_of(const proto_cmd_t *c)
{
    const int fi = field_by_kind(PROTO_KIND_EVLIST);
    return (fi < 0) ? 0u : (uint32_t)load_field(c, &kFields[fi]);
}


/**
 * 规范形式整数：可选 '-'、无前导零（"0" 单独允许）、无 "-0"、无尾部垃圾。
 * 严格格式用它 —— 一种值只有一种写法。
 */
static int parse_int_strict(const char *s, size_t n, int64_t *out)
{
    size_t i = 0;
    int neg = 0;
    int64_t v = 0;

    if (n == 0 || n > PROTO_MAX_NUM) {
        return 0;
    }
    if (s[0] == '-') {
        neg = 1;
        i = 1;
        if (i >= n) {
            return 0;
        }
    }
    if (s[i] == '0' && (n - i) > 1) {
        return 0;                       /* 前导零 */
    }
    for (; i < n; ++i) {
        if (!is_digit((unsigned char)s[i])) {
            return 0;                   /* 非数字 / 夹符号 / '+' / 空白 */
        }
        v = v * 10 + (int64_t)(s[i] - '0');
        if (v > 10000000000LL) {
            return 0;                   /* 远大于任何字段域：一定越界 */
        }
    }
    if (neg) {
        if (v == 0) {
            return 0;                   /* "-0" 不合法 */
        }
        v = -v;
    }
    *out = v;
    return 1;
}

/**
 * 旧页面 `_leading_int()` 的语义：可选单个 '+'/'-'、允许前导零、
 * **只取前导整数、忽略值尾部垃圾**（`f=10abc` → 10，`f=-100t=20` → -100）。
 * 通过 `consumed` 回报吃了多少字节，调用方从那里接着找下一个键。
 */
static int parse_int_legacy(const char *s, size_t n, int64_t *out, size_t *consumed)
{
    size_t i = 0;
    size_t digits = 0;
    int neg = 0;
    uint64_t v = 0;

    if (n == 0) {
        return 0;
    }
    if (s[0] == '+' || s[0] == '-') {
        neg = (s[0] == '-');
        i = 1;
    }
    while (i < n && is_digit((unsigned char)s[i])) {
        if (digits < 12u) {             /* 最多累 12 位，绝不溢出 */
            v = v * 10u + (uint64_t)(s[i] - '0');
        }
        ++digits;
        ++i;
    }
    if (digits == 0) {
        return 0;                       /* 没有前导整数 */
    }
    if (digits > 12u) {
        v = 10000000000ULL;             /* 太长 → 一定越界（交给范围检查报错）*/
    }
    *out = neg ? -(int64_t)v : (int64_t)v;
    if (consumed != NULL) {
        *consumed = i;
    }
    return 1;
}

/* 字段值存取：走偏移表，所以"哪个字段在结构体哪里"只有表里那一份 */
static int64_t load_field(const proto_cmd_t *c, const proto_field_t *f)
{
    const char *p = (const char *)c + f->offset;
    if (f->is_unsigned) {
        uint32_t u = 0;
        memcpy(&u, p, sizeof(u));
        return (int64_t)u;
    }
    {
        int32_t s = 0;
        memcpy(&s, p, sizeof(s));
        return (int64_t)s;
    }
}

static void store_field(proto_cmd_t *c, const proto_field_t *f, int64_t v)
{
    char *p = (char *)c + f->offset;
    if (f->is_unsigned) {
        uint32_t u = (uint32_t)v;
        memcpy(p, &u, sizeof(u));
    } else {
        int32_t s = (int32_t)v;
        memcpy(p, &s, sizeof(s));
    }
}

/** 按角色取当前值（`rdog`/`est`/`seq` 用；代码里不出现它们的键名字面量）*/
static int64_t role_value(const proto_cmd_t *c, proto_role_t role)
{
    const int fi = field_by_role(role);
    return (fi < 0) ? 0 : load_field(c, &kFields[fi]);
}

/* ==========================================================================
 * 解析状态 / 报错
 * ========================================================================== */

typedef struct {
    proto_cmd_t cmd;    /**< 局部凑帧：全部通过才交给调用方 */
    uint32_t    seen;   /**< 已经出现过的字段位（查重 + 查必填）*/
    size_t      nfields;
} parse_t;

/**
 * 记一次拒绝并填 error。计数分四类，与 §0.5(7) 要回读的
 * "接受 / 丢弃 / 过期 / 坏帧" 一一对应（过期在 finish 里单独计）。
 */
static int fail(proto_decoder_t *d, proto_error_t *err, proto_err_t code, int field,
                const char *key, size_t klen)
{
    d->stats.rejected++;
    if ((int)code > 0 && (int)code < (int)PROTO_E_COUNT) {
        d->stats.by_err[code]++;
    }
    switch (proto_err_class(code)) {
    case PROTO_CLS_FIELD:
        d->stats.bad_field++;
        break;
    case PROTO_CLS_SEQ:
        d->stats.dropped_seq++;
        break;
    case PROTO_CLS_RATE:
        d->stats.rate_limited++;
        break;
    case PROTO_CLS_FRAME:
    default:
        d->stats.bad_frame++;
        break;
    }
    if (err != NULL) {
        size_t n = 0;
        err->code = code;
        err->field = field;
        if (key != NULL) {
            n = klen;
            if (n > PROTO_MAX_KEY) {
                n = PROTO_MAX_KEY;
            }
            memcpy(err->key, key, n);
        }
        err->key[n] = '\0';
    }
    return (int)code;
}

/** 落一个整数字段：查重 → 范围 → 存值 → 置 present */
static int commit_int(proto_decoder_t *d, proto_error_t *err, parse_t *p, int fi,
                      const char *kb, size_t klen, int64_t v)
{
    const proto_field_t *f = &kFields[fi];

    if ((p->seen & (1u << fi)) != 0u) {
        return fail(d, err, PROTO_E_DUP_KEY, fi, kb, klen);
    }
    if (v < f->min || v > f->max) {
        const proto_err_t code = (f->role == PROTO_ROLE_MAGIC) ? PROTO_E_BAD_VERSION
                                                              : PROTO_E_RANGE;
        return fail(d, err, code, fi, kb, klen);
    }
    p->seen |= (1u << fi);
    store_field(&p->cmd, f, v);
    p->cmd.present |= (1u << fi);
    p->nfields++;
    return PROTO_OK;
}

/** 旧页面里那三个"等价于字段赋值"的按键；`*hits_out` = 落地的赋值个数（0 = 不认识）*/
static int commit_legacy_cmd(proto_decoder_t *d, proto_error_t *err, parse_t *p,
                             const char *name, size_t nlen, int *hits_out)
{
    size_t i;
    int hits = 0;

    *hits_out = 0;
    for (i = 0; i < NLEGACYCMD; ++i) {
        const proto_legacy_cmd_t *lc = &kLegacyCmd[i];
        int fi;
        int64_t v;
        int rc;

        if (strlen(lc->name) != nlen || memcmp(lc->name, name, nlen) != 0) {
            continue;
        }
        fi = proto_field_find(lc->field);   /* 规范键名 → 下标（范围也在表里）*/
        if (fi < 0) {
            continue;
        }
        switch (lc->use) {
        case PROTO_ALIAS_MIN:
            v = kFields[fi].min;
            break;
        case PROTO_ALIAS_MAX:
            v = kFields[fi].max;
            break;
        case PROTO_ALIAS_VALUE:
        default:
            v = lc->value;
            break;
        }
        rc = commit_int(d, err, p, fi, name, nlen, v);
        if (rc != PROTO_OK) {
            return rc;
        }
        ++hits;
    }
    *hits_out = hits;
    return PROTO_OK;
}

/**
 * 解析 `ev` / `key` 的取值。
 *
 * 严格格式：整个取值必须是逗号分隔的事件名表（允许为空 = 这一帧没有事件）。
 * 旧格式：`key=<名字>` 是**一条命令**，所以必须非空；取值允许是逗号表，
 *         消费到第一个非名字/非逗号字符为止（值尾部垃圾忽略）。
 * 旧格式下，事件表里没有的名字再去 `kLegacyCmd` 找（`btn_stop` 等）。
 */
static int parse_evlist(proto_decoder_t *d, proto_error_t *err, parse_t *p, int fi,
                        const char *kb, size_t klen, const char *vb, size_t vlen,
                        int legacy, size_t *consumed)
{
    uint32_t mask = 0;
    size_t k = 0;

    if (legacy && vlen == 0) {
        return fail(d, err, PROTO_E_EMPTY_VALUE, fi, kb, klen);
    }
    while (k < vlen) {
        const size_t ns = k;
        size_t nlen;
        int ei;

        while (k < vlen && ident_char((unsigned char)vb[k])) {
            ++k;
        }
        nlen = k - ns;
        if (!name_ok(vb + ns, nlen, PROTO_MAX_EVENT)) {
            *consumed = k;
            return fail(d, err, PROTO_E_EVENT_BAD_NAME, fi, nlen ? (vb + ns) : "",
                        nlen);
        }
        ei = event_find_n(vb + ns, nlen);
        if (ei >= 0) {
            if ((mask & (1u << ei)) != 0u) {
                *consumed = k;
                return fail(d, err, PROTO_E_EVENT_DUP, fi, vb + ns, nlen);
            }
            mask |= (1u << ei);
        } else if (legacy) {
            int hits = 0;
            const int rc = commit_legacy_cmd(d, err, p, vb + ns, nlen, &hits);
            if (rc != PROTO_OK) {
                *consumed = k;
                return rc;
            }
            if (hits == 0) {
                *consumed = k;
                return fail(d, err, PROTO_E_EVENT_UNKNOWN, fi, vb + ns, nlen);
            }
        } else {
            *consumed = k;
            return fail(d, err, PROTO_E_EVENT_UNKNOWN, fi, vb + ns, nlen);
        }
        if (k >= vlen) {
            break;
        }
        if (vb[k] == ',') {
            ++k;
            if (k >= vlen) {
                *consumed = k;
                return fail(d, err, PROTO_E_EVENT_BAD_NAME, fi, "", 0);
            }
            continue;
        }
        if (legacy) {
            break;                      /* 值尾部垃圾：从 k 接着扫下一个键 */
        }
        *consumed = k;
        return fail(d, err, PROTO_E_EVENT_BAD_NAME, fi, vb + k, vlen - k);
    }
    *consumed = k;

    /* 事件掩码也走 commit_int：present 位、查重、范围（白名单外的位）一次搞定 */
    return commit_int(d, err, p, fi, kb, klen, (int64_t)mask);
}

/* ==========================================================================
 * 严格格式
 * ========================================================================== */

static int parse_strict(proto_decoder_t *d, const char *buf, size_t len,
                        parse_t *p, proto_error_t *err)
{
    size_t body;
    size_t i;
    size_t pos = 0;

    if (len == 0) {
        return fail(d, err, PROTO_E_EMPTY, -1, NULL, 0);
    }
    if (len > PROTO_MAX_FRAME) {
        return fail(d, err, PROTO_E_TOO_LONG, -1, NULL, 0);
    }
    if (buf[len - 1] != '\n') {
        return fail(d, err, PROTO_E_NO_TERMINATOR, -1, NULL, 0);
    }
    body = len - 1;
    if (body == 0) {
        return fail(d, err, PROTO_E_EMPTY, -1, NULL, 0);
    }
    /* 正文只允许可打印 ASCII：这条同时挡掉了中间换行、'\r'、NUL、UTF-8 垃圾 */
    for (i = 0; i < body; ++i) {
        const unsigned char c = (unsigned char)buf[i];
        if (c < 0x20u || c > 0x7Eu) {
            return fail(d, err, PROTO_E_CTRL_CHAR, -1, NULL, 0);
        }
    }

    for (;;) {
        const size_t start = pos;
        size_t seglen;
        size_t eq = 0;
        const char *seg;
        const char *kb;
        const char *vb;
        size_t klen;
        size_t vlen;
        int fi;
        int rc;

        while (pos < body && buf[pos] != ';') {
            ++pos;
        }
        seglen = pos - start;
        if (seglen == 0) {
            return fail(d, err, PROTO_E_BAD_SEPARATOR, -1, NULL, 0);
        }
        seg = buf + start;
        while (eq < seglen && seg[eq] != '=') {
            ++eq;
        }
        if (eq == seglen) {
            return fail(d, err, PROTO_E_NO_EQUALS, -1, seg, seglen);
        }
        kb = seg;
        klen = eq;
        vb = seg + eq + 1;
        vlen = seglen - eq - 1;

        if (!name_ok(kb, klen, PROTO_MAX_KEY)) {
            return fail(d, err, PROTO_E_BAD_KEY, -1, kb, klen);
        }
        fi = field_find_n(kb, klen, 0);
        if (fi < 0) {
            return fail(d, err, PROTO_E_UNKNOWN_KEY, -1, kb, klen);
        }
        if (kFields[fi].kind == PROTO_KIND_EVLIST) {
            size_t used = 0;
            rc = parse_evlist(d, err, p, fi, kb, klen, vb, vlen, 0, &used);
            if (rc == PROTO_OK && used != vlen) {
                rc = fail(d, err, PROTO_E_EVENT_BAD_NAME, fi, vb + used, vlen - used);
            }
        } else {
            int64_t v = 0;
            if (vlen == 0) {
                return fail(d, err, PROTO_E_EMPTY_VALUE, fi, kb, klen);
            }
            if (!parse_int_strict(vb, vlen, &v)) {
                return fail(d, err, PROTO_E_BAD_INT, fi, kb, klen);
            }
            rc = commit_int(d, err, p, fi, kb, klen, v);
        }
        if (rc != PROTO_OK) {
            return rc;
        }

        if (pos == body) {
            break;
        }
        ++pos;                          /* 跳过 ';' */
        if (pos == body) {
            return fail(d, err, PROTO_E_BAD_SEPARATOR, -1, NULL, 0);
        }
    }
    return PROTO_OK;
}

/* ==========================================================================
 * 旧页面格式
 * ========================================================================== */

/**
 * 逐字节扫描 `名字=` 形式的赋值。
 * 不在赋值里的字节（`GET /`、`?`、`&`、` HTTP/1.1`）都是垃圾，跳过；
 * 一旦认出 `名字=`，这个名字必须白名单里有，否则整帧拒绝（键名严格）。
 */
static int parse_legacy(proto_decoder_t *d, const char *buf, size_t len,
                        parse_t *p, proto_error_t *err)
{
    size_t i = 0;

    if (len == 0) {
        return fail(d, err, PROTO_E_EMPTY, -1, NULL, 0);
    }
    if (len > PROTO_MAX_LEGACY) {
        return fail(d, err, PROTO_E_TOO_LONG, -1, NULL, 0);
    }

    while (i < len) {
        size_t ks;
        size_t ke;
        int fi;
        int rc;

        if (!ident_start((unsigned char)buf[i])) {
            ++i;
            continue;
        }
        ks = i;
        ke = i;
        while (ke < len && ident_char((unsigned char)buf[ke])) {
            ++ke;
        }
        if (ke >= len || buf[ke] != '=') {
            i = ke;                     /* 不是赋值（HTTP 词、文件名…），整段跳过 */
            continue;
        }
        if ((ke - ks) == 0 || (ke - ks) > PROTO_MAX_KEY) {
            return fail(d, err, PROTO_E_BAD_KEY, -1, buf + ks, ke - ks);
        }
        fi = field_find_n(buf + ks, ke - ks, 1);
        if (fi < 0) {
            return fail(d, err, PROTO_E_UNKNOWN_KEY, -1, buf + ks, ke - ks);
        }
        i = ke + 1;                     /* 跳过 '=' */

        if (kFields[fi].kind == PROTO_KIND_EVLIST) {
            size_t used = 0;
            rc = parse_evlist(d, err, p, fi, buf + ks, ke - ks, buf + i, len - i, 1, &used);
            i += used;
        } else {
            int64_t v = 0;
            size_t used = 0;
            if (!parse_int_legacy(buf + i, len - i, &v, &used)) {
                return fail(d, err, PROTO_E_BAD_INT, fi, buf + ks, ke - ks);
            }
            i += used;
            rc = commit_int(d, err, p, fi, buf + ks, ke - ks, v);
        }
        if (rc != PROTO_OK) {
            return rc;
        }
    }
    return PROTO_OK;
}

/* ==========================================================================
 * 收尾：必填 / 限速 / 序号 / 接受
 * ========================================================================== */

static int finish(proto_decoder_t *d, proto_error_t *err, const parse_t *p,
                  uint32_t now_ms, proto_cmd_t *out, int strict)
{
    size_t i;

    if (p->nfields == 0) {
        /* 一个字段都没认出来 —— 多半是页面请求（`GET /control.html`），不是命令 */
        return fail(d, err, PROTO_E_NO_FIELDS, -1, NULL, 0);
    }
    if (strict) {
        for (i = 0; i < NFIELDS; ++i) {
            if (kFields[i].required == PROTO_REQ_STRICT && (p->seen & (1u << i)) == 0u) {
                return fail(d, err, PROTO_E_MISSING_KEY, (int)i, kFields[i].key,
                            strlen(kFields[i].key));
            }
        }
    }

    /* 限速（§7：目标 20~50 Hz，>100 Hz 视为洪泛）。
     * 带事件或带急停的帧不参与：丢一个按键就是丢一条命令，急停更不能丢。 */
    if (d->min_interval_ms != 0u && d->have_frame != 0u
        && role_value(&p->cmd, PROTO_ROLE_ESTOP) != 1 && ev_mask_of(&p->cmd) == 0u) {
        const uint32_t gap = (uint32_t)(now_ms - d->last_rx_ms);
        if (gap < d->min_interval_ms) {
            return fail(d, err, PROTO_E_RATE_LIMIT, -1, NULL, 0);
        }
    }

    /* 序号：只有严格格式有 seq（旧页面根本没有），所以旧格式没有重放保护 */
    if (strict && d->have_frame != 0u) {
        const int si = field_by_role(PROTO_ROLE_SEQ);
        const uint32_t seq = (uint32_t)role_value(&p->cmd, PROTO_ROLE_SEQ);
        if (!proto_seq_newer(seq, d->last_seq)) {
            return fail(d, err, PROTO_E_SEQ_STALE, si, NULL, 0);
        }
    }

    /* 断连计数：上一帧在**这一帧到达之前**就已经过期了 ⇒ 中间断过（§0.5(7) 的"过期"）*/
    if (d->have_frame != 0u) {
        const uint32_t gap = (uint32_t)(now_ms - d->last_rx_ms);
        if (gap > d->hb_ms) {
            d->stats.heartbeat_gaps++;
        }
    }

    *out = p->cmd;
    /* seq 只在真的带了 seq 的帧上推进（否则旧格式帧会把序号基线冲成 0，
     * 让一条重放的严格帧重新变"新"）*/
    {
        const int si = field_by_role(PROTO_ROLE_SEQ);
        if (si >= 0 && (p->cmd.present & (1u << si)) != 0u) {
            d->last_seq = (uint32_t)role_value(&p->cmd, PROTO_ROLE_SEQ);
        }
    }
    d->last_rx_ms = now_ms;
    d->have_frame = 1u;
    d->stats.accepted++;
    if (role_value(&p->cmd, PROTO_ROLE_ESTOP) == 1) {
        d->stats.estop_frames++;
    }
    return PROTO_OK;
}

/* ==========================================================================
 * 对外入口
 * ========================================================================== */

static int decode_common(proto_decoder_t *d, const char *buf, size_t len, uint32_t now_ms,
                         proto_cmd_t *out, proto_error_t *err, int strict)
{
    parse_t p;
    int rc;

    if (err != NULL) {
        err->code = PROTO_OK;
        err->field = -1;
        err->key[0] = '\0';
    }
    /* 编程错误：空指针不计入统计（计数只统计"真的收到了一帧"）*/
    if (d == NULL || out == NULL || buf == NULL) {
        if (err != NULL) {
            err->code = PROTO_E_NULL;
        }
        return (int)PROTO_E_NULL;
    }
    d->stats.frames++;
    memset(&p, 0, sizeof(p));

    rc = strict ? parse_strict(d, buf, len, &p, err) : parse_legacy(d, buf, len, &p, err);
    if (rc != PROTO_OK) {
        return rc;
    }
    if (!strict) {
        d->stats.legacy_frames++;
    }
    return finish(d, err, &p, now_ms, out, strict);
}

int proto_decode(proto_decoder_t *d, const char *buf, size_t len, uint32_t now_ms,
                 proto_cmd_t *out, proto_error_t *err)
{
    return decode_common(d, buf, len, now_ms, out, err, 1);
}

int proto_decode_legacy(proto_decoder_t *d, const char *buf, size_t len, uint32_t now_ms,
                        proto_cmd_t *out, proto_error_t *err)
{
    return decode_common(d, buf, len, now_ms, out, err, 0);
}

/* ==========================================================================
 * 编码
 * ========================================================================== */

/** 追加（保证留得下结尾 NUL）；空间不够返回 0 */
static int put(char *buf, size_t cap, size_t *w, const char *s, size_t n)
{
    if (*w + n + 1u > cap) {
        return 0;
    }
    memcpy(buf + *w, s, n);
    *w += n;
    return 1;
}

/** 十进制整数（手写，不碰 snprintf ⇒ 不受 locale / 格式符影响）*/
static int put_i64(char *buf, size_t cap, size_t *w, int64_t v)
{
    char digits[24];
    char text[24];
    size_t nd = 0;
    size_t nt = 0;
    uint64_t u;
    const int neg = (v < 0);

    u = neg ? (uint64_t)(-(v + 1)) + 1u : (uint64_t)v;   /* INT64_MIN 也安全 */
    do {
        digits[nd++] = (char)('0' + (int)(u % 10u));
        u /= 10u;
    } while (u != 0u);
    if (neg) {
        text[nt++] = '-';
    }
    while (nd > 0u) {
        text[nt++] = digits[--nd];
    }
    return put(buf, cap, w, text, nt);
}

/** 事件掩码 → 逗号分隔的规范名表（空掩码 → 空串，合法）*/
static int put_events(char *buf, size_t cap, size_t *w, uint32_t mask)
{
    size_t i;
    int first = 1;

    for (i = 0; i < NEVENTS; ++i) {
        if ((mask & (1u << i)) == 0u) {
            continue;
        }
        if (!first && !put(buf, cap, w, ",", 1)) {
            return 0;
        }
        if (!put(buf, cap, w, kEvents[i], strlen(kEvents[i]))) {
            return 0;
        }
        first = 0;
    }
    return 1;
}

size_t proto_encode(const proto_cmd_t *cmd, char *buf, size_t cap)
{
    size_t i;
    size_t w = 0;

    if (buf != NULL && cap > 0u) {
        buf[0] = '\0';
    }
    if (cmd == NULL || buf == NULL || cap == 0u) {
        return 0;
    }
    /* 第一遍：整帧校验（同一张字段表）。绝不写出半截非法帧。 */
    for (i = 0; i < NFIELDS; ++i) {
        const int64_t v = load_field(cmd, &kFields[i]);
        if (v < kFields[i].min || v > kFields[i].max) {
            return 0;
        }
    }
    /* 第二遍：格式化 */
    for (i = 0; i < NFIELDS; ++i) {
        const proto_field_t *f = &kFields[i];
        int ok;
        if (f->kind == PROTO_KIND_INT) {
            ok = put(buf, cap, &w, f->key, strlen(f->key)) && put(buf, cap, &w, "=", 1)
                 && put_i64(buf, cap, &w, load_field(cmd, f));
        } else {
            ok = put(buf, cap, &w, f->key, strlen(f->key)) && put(buf, cap, &w, "=", 1)
                 && put_events(buf, cap, &w, (uint32_t)load_field(cmd, f));
        }
        if (!ok || (i + 1u < NFIELDS && !put(buf, cap, &w, ";", 1))) {
            buf[0] = '\0';
            return 0;
        }
    }
    if (!put(buf, cap, &w, "\n", 1)) {
        buf[0] = '\0';
        return 0;
    }
    buf[w] = '\0';
    return w;
}

/* ==========================================================================
 * 命令结构体工具
 * ========================================================================== */

void proto_cmd_zero(proto_cmd_t *cmd)
{
    if (cmd != NULL) {
        memset(cmd, 0, sizeof(*cmd));
    }
}

void proto_cmd_defaults(proto_cmd_t *cmd)
{
    size_t i;

    if (cmd == NULL) {
        return;
    }
    memset(cmd, 0, sizeof(*cmd));
    for (i = 0; i < NFIELDS; ++i) {
        const proto_field_t *f = &kFields[i];
        int64_t v = 0;

        if (f->kind == PROTO_KIND_INT) {
            if (f->role == PROTO_ROLE_MAGIC) {
                v = PROTO_VERSION;
            } else if (f->role == PROTO_ROLE_SEQ) {
                v = 1;
            } else if (f->min > 0) {
                v = (f->min + f->max) / 2;   /* 例：hgt 70..110 → 90 */
            }
        }
        /* 事件字段取 0（空事件表，合法）；所有字段都置 present */
        (void)proto_cmd_set(cmd, i, v);
    }
}

int64_t proto_cmd_get(const proto_cmd_t *cmd, size_t index)
{
    if (cmd == NULL || index >= NFIELDS) {
        return 0;
    }
    return load_field(cmd, &kFields[index]);
}

int proto_cmd_set(proto_cmd_t *cmd, size_t index, int64_t value)
{
    if (cmd == NULL || index >= NFIELDS) {
        return 0;
    }
    if (value < kFields[index].min || value > kFields[index].max) {
        return 0;                      /* 越界：什么都不改（默认拒绝，不 clamp）*/
    }
    store_field(cmd, &kFields[index], value);
    cmd->present |= (1u << index);
    return 1;
}

int proto_cmd_equal(const proto_cmd_t *a, const proto_cmd_t *b)
{
    size_t i;

    if (a == NULL || b == NULL) {
        return 0;
    }
    if (a->present != b->present) {
        return 0;
    }
    for (i = 0; i < NFIELDS; ++i) {
        if (load_field(a, &kFields[i]) != load_field(b, &kFields[i])) {
            return 0;
        }
    }
    return 1;
}

int proto_cmd_has_ev_index(const proto_cmd_t *cmd, int ev_index)
{
    if (cmd == NULL || ev_index < 0 || (size_t)ev_index >= NEVENTS) {
        return 0;
    }
    return (cmd->ev_mask & (1u << ev_index)) != 0u;
}

int proto_cmd_has_ev(const proto_cmd_t *cmd, const char *name)
{
    size_t n;
    int ei;

    if (cmd == NULL || name == NULL) {
        return 0;
    }
    n = strlen(name);
    ei = event_find_n(name, n);
    return proto_cmd_has_ev_index(cmd, ei);
}

uint32_t proto_cmd_ev_count(const proto_cmd_t *cmd)
{
    uint32_t m;
    uint32_t n = 0;

    if (cmd == NULL) {
        return 0;
    }
    for (m = cmd->ev_mask; m != 0u; m &= (m - 1u)) {
        ++n;
    }
    return n;
}

/* ==========================================================================
 * 急停旁路
 * ========================================================================== */

int proto_peek_estop(proto_decoder_t *d, const char *buf, size_t len, int *est_out)
{
    size_t i = 0;
    size_t limit;

    if (est_out != NULL) {
        *est_out = 0;
    }
    if (d != NULL) {
        d->stats.estop_peeks++;
    }
    if (buf == NULL || len == 0u) {
        return 0;
    }
    limit = (len > PROTO_MAX_FRAME) ? (size_t)PROTO_MAX_FRAME : len;  /* 扫描有界 */

    while (i < limit) {
        size_t ke;
        if (!ident_start((unsigned char)buf[i])) {
            ++i;
            continue;
        }
        ke = i;
        while (ke < limit && ident_char((unsigned char)buf[ke])) {
            ++ke;
        }
        if (ke < limit && buf[ke] == '=') {
            const size_t klen = ke - i;
            int fi = field_find_n(buf + i, klen, 0);   /* 严格键名 */
            if (fi < 0) {
                fi = field_find_n(buf + i, klen, 1);   /* 旧键名 */
            }
            if (fi >= 0 && kFields[fi].role == PROTO_ROLE_ESTOP) {
                int64_t v = 0;
                size_t used = 0;
                /* 故意用宽容解析：急停要尽可能读得出来（读出来只会更安全）*/
                if (parse_int_legacy(buf + ke + 1, limit - ke - 1, &v, &used)
                    && v >= kFields[fi].min && v <= kFields[fi].max) {
                    if (d != NULL && v == 1) {
                        d->stats.estop_hits++;
                    }
                    if (est_out != NULL) {
                        *est_out = (int)v;
                    }
                    return 1;
                }
            }
            i = ke + 1;
        } else {
            i = (ke > i) ? ke : (i + 1u);
        }
    }
    return 0;
}

/* ==========================================================================
 * 格式嗅探
 * ========================================================================== */

proto_fmt_t proto_sniff_format(const char *buf, size_t len)
{
    size_t i = 0;
    size_t limit;
    int magic = field_by_role(PROTO_ROLE_MAGIC);
    int legacy_seen = 0;

    if (buf == NULL || len == 0u) {
        return PROTO_FMT_UNKNOWN;
    }
    limit = (len > PROTO_MAX_FRAME) ? (size_t)PROTO_MAX_FRAME : len;

    while (i < limit) {
        size_t ke;
        if (!ident_start((unsigned char)buf[i])) {
            ++i;
            continue;
        }
        ke = i;
        while (ke < limit && ident_char((unsigned char)buf[ke])) {
            ++ke;
        }
        if (ke < limit && buf[ke] == '=') {
            const size_t klen = ke - i;
            const int fi = field_find_n(buf + i, klen, 0);
            if (fi >= 0) {
                if (fi == magic) {
                    return PROTO_FMT_STRICT;   /* 魔数键在 ⇒ 严格格式（键序不重要）*/
                }
                legacy_seen = 1;
            } else if (field_find_n(buf + i, klen, 1) >= 0) {
                legacy_seen = 1;
            }
            i = ke + 1;
        } else {
            i = (ke > i) ? ke : (i + 1u);
        }
    }
    return legacy_seen ? PROTO_FMT_LEGACY : PROTO_FMT_UNKNOWN;
}

/* ==========================================================================
 * 阈值 / 时钟 / 序号
 * ========================================================================== */

void proto_decoder_init(proto_decoder_t *d)
{
    if (d == NULL) {
        return;
    }
    memset(d, 0, sizeof(*d));
    d->hb_ms = PROTO_HB_DEFAULT_MS;
    d->long_ms = PROTO_LONG_DEFAULT_MS;
    d->min_interval_ms = PROTO_MIN_INTERVAL_DEFAULT_MS;
}

uint32_t proto_decoder_set_hb_ms(proto_decoder_t *d, uint32_t ms)
{
    if (d == NULL) {
        return PROTO_HB_DEFAULT_MS;
    }
    if (ms < PROTO_HB_MIN_MS) {
        ms = PROTO_HB_MIN_MS;
    }
    if (ms > PROTO_HB_MAX_MS) {
        ms = PROTO_HB_MAX_MS;
    }
    d->hb_ms = ms;
    return ms;
}

uint32_t proto_decoder_set_long_ms(proto_decoder_t *d, uint32_t ms)
{
    if (d == NULL) {
        return PROTO_LONG_DEFAULT_MS;
    }
    if (ms < d->hb_ms) {
        ms = d->hb_ms;                 /* 长超时不能短于短超时 */
    }
    if (ms > PROTO_LONG_MAX_MS) {
        ms = PROTO_LONG_MAX_MS;
    }
    if (ms < PROTO_LONG_MIN_MS) {
        ms = PROTO_LONG_MIN_MS;
    }
    d->long_ms = ms;
    return ms;
}

uint32_t proto_decoder_set_min_interval_ms(proto_decoder_t *d, uint32_t ms)
{
    if (d == NULL) {
        return PROTO_MIN_INTERVAL_DEFAULT_MS;
    }
    if (ms > PROTO_MIN_INTERVAL_MAX_MS) {
        ms = PROTO_MIN_INTERVAL_MAX_MS;
    }
    d->min_interval_ms = ms;
    return ms;
}

int proto_seq_newer(uint32_t candidate, uint32_t last)
{
    /* 回绕安全：差值当成有符号看，"往前走了一点点"才算更新（§0.5(7) 明说不许写 a > b）*/
    return (int32_t)(candidate - last) > 0;
}

uint32_t proto_age_ms(const proto_decoder_t *d, uint32_t now_ms)
{
    if (d == NULL || d->have_frame == 0u) {
        return PROTO_AGE_NEVER;
    }
    return (uint32_t)(now_ms - d->last_rx_ms);
}

int proto_is_stale_at(const proto_decoder_t *d, uint32_t now_ms, uint32_t timeout_ms)
{
    if (d == NULL || d->have_frame == 0u) {
        return 1;                      /* 从没收到过帧 ⇒ 安全侧：算过期 */
    }
    return proto_age_ms(d, now_ms) > timeout_ms;
}

int proto_is_stale(const proto_decoder_t *d, uint32_t now_ms)
{
    if (d == NULL) {
        return 1;
    }
    return proto_is_stale_at(d, now_ms, d->hb_ms);
}

int proto_is_expired(const proto_decoder_t *d, uint32_t now_ms)
{
    if (d == NULL) {
        return 1;
    }
    return proto_is_stale_at(d, now_ms, d->long_ms);
}

uint32_t proto_last_rx_ms(const proto_decoder_t *d)
{
    return (d == NULL || d->have_frame == 0u) ? 0u : d->last_rx_ms;
}

/* ==========================================================================
 * 表查询
 * ========================================================================== */

const proto_field_t *proto_fields(size_t *count)
{
    if (count != NULL) {
        *count = NFIELDS;
    }
    return kFields;
}

const char *const *proto_events(size_t *count)
{
    if (count != NULL) {
        *count = NEVENTS;
    }
    return kEvents;
}

const char *proto_event_at(size_t idx)
{
    return (idx < NEVENTS) ? kEvents[idx] : NULL;
}

int proto_field_find(const char *key)
{
    return (key == NULL) ? -1 : field_find_n(key, strlen(key), 0);
}

int proto_field_find_legacy(const char *key)
{
    return (key == NULL) ? -1 : field_find_n(key, strlen(key), 1);
}

int proto_event_find(const char *name)
{
    return (name == NULL) ? -1 : event_find_n(name, strlen(name));
}
