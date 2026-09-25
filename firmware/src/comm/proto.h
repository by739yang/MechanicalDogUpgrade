/**
 * @file    proto.h
 * @brief   P5 机器人命令协议 —— 严格文本帧的解码 / 校验 / 编码
 *
 * **纯 C，零 ESP-IDF 依赖**（不 include 任何 esp_*）；时钟由调用方以 `now_ms` 传入，
 * 所以宿主测试能用 `tools/golden/host_stubs/` 的可控时钟把心跳、限速、序号
 * 这些"依赖时间"的逻辑全部确定性地跑完，不用烧板子。
 *
 * ==========================================================================
 * 为什么要有这一层（对应 ESP-IDF_C迁移表.md §8.2 / §8.3 / §0.5）
 * ==========================================================================
 * 原版 web_c.py 把整个 query string 交给 `exec()`（第 303 行），并用
 * `exec("padog.init_"+user_leg_num+"h=...")` 拼字符串改标定值。§8.3 要求
 * **不执行 exec()，改用白名单字段 + 范围校验**。本文件就是那个白名单。
 *
 * 原版还有三个真实缺陷（§0.5(2)），协议层要能补：
 *   1. 完全没有断连检测 —— 浏览器一关，狗带着最后一组 spd/L/R 一直走；
 *      ⇒ `seq` + 心跳超时（本文件给出 wrap-safe 的序号比较与新鲜度判定）。
 *   2. 没有序号 —— 并发 GET 的到达顺序不保证，旧命令能覆盖新命令；
 *      ⇒ `seq` 严格递增才接受，重复/重放/乱序被拒并计数。
 *   3. 部分更新 —— `thr/turn/Pitch/Roll/Yst/Hgt` 是 web_c.py 的模块级全局，
 *      只在参数出现时才更新，缺参数就**悄悄沿用旧值**；
 *      ⇒ 严格格式是"完整快照"（必填键），缺键整帧拒绝；
 *        旧格式则用 `present` 位把"这一帧没提这个字段"**显式**表达出来。
 *
 * ==========================================================================
 * 两种格式，一张表（P-22 / P-27：同一份东西只能有一份）
 * ==========================================================================
 * 本文件有**两个**解码入口，因为现场同时存在两种发送方：
 *
 *   [A] 严格格式 —— 新发送方（本工程后续的 Web / 上位机）
 *       `proto_decode()`：`;` 分隔、`\n` 结尾、键名/键序/键集/取值全是规范形式、
 *       必填键缺失即整帧拒绝。这是 §3「命令中必须包含 seq/时间戳/模式/速度/转向/
 *       姿态/身高/机械臂/急停」那一套要表达的东西。
 *
 *   [B] 旧格式 —— **现场唯一能上板验收的 `micropython/drive.html` / `control.html`**
 *       `proto_decode_legacy()`：`GET /f=-100t=20` 这种**没有 `?`、参数之间连 `&`
 *       都没有**的报文（web_common.py 的 `_parse_pair()` 就是拿"下一个键"当分隔符的），
 *       取值沿用原版 `_leading_int()` 的宽容度（允许前导 `+`/`-`/前导零，**只取前导
 *       整数、忽略值尾部垃圾**），没有 `seq`、没有 `mode`、没有结尾 `\n`。
 *       ⚠️ 宽容**只准出现在值的尾部**：键名、键集、范围、事件名一律严格。
 *
 * 两者的**字段表、范围、事件名、取值解析器完全共用**（下面那张 `kFields`）。
 * 差别只在"分隔符怎么找、值允许多宽、缺键怎么算"这三处，各自写在对应的循环里。
 * 旧格式只认**旧键名**（`f`/`t`/`jy`/`jx`/`key`），严格格式只认**新键名**
 * ⇒ `t` 在严格格式里是"发送端时间戳"，在旧格式里是"转向摇杆"，两者永不串味。
 *
 * ==========================================================================
 * 帧语法（严格格式）
 * ==========================================================================
 * ```text
 * frame   = field *( ";" field ) "\n"
 * field   = key "=" value
 * key     = 1*( %x61-7A / %x30-39 / "_" )            ; 必须小写字母开头，<= 16 字节
 * value   = int / event-list
 * int     = [ "-" ] ( "0" / %x31-39 *DIGIT )         ; 规范形式：不许 '+'、不许前导零、
 *                                                    ; 不许 "-0"、不许空、不许夹符号
 * event-list = [ event *( "," event ) ]              ; 允许为空（= 这一帧没有任何事件）
 * event   = 1*( %x61-7A / %x30-39 / "_" )            ; <= 16 字节，必须在事件白名单里
 * ```
 * 规则（**全部**由解析器强制，且都有对应测试）：
 *   - **整数只有一种**：所有取值都是定标整数，协议里没有浮点 ⇒ 没有 NaN/Inf/
 *     locale 问题。标度写在字段表的 `scale` 列里（ASCII，可直接打印）。
 *   - **键可任意顺序**（自描述文本帧），但**未知键 / 重复键 / 必填键缺失 → 整帧拒绝**，
 *     绝不悄悄取默认值。
 *   - 每个取值都过**范围校验**；越界 → **整帧拒绝，不 clamp** ——
 *     一个畸形或恶意的发送方**不能驱动机器人哪怕一点点**。
 *   - `ev` 里的事件名必须在白名单里；未知/重复/空/非小写名 → 整帧拒绝。
 *   - `seq` 必须**严格更新**（回绕安全比较）；重复、重放、乱序被拒并计数。
 *   - `t` 是**发送端**时间戳，仅供参考；新鲜度用**本地** `now_ms` 判定。
 *   - 整帧正文只允许可打印 ASCII（0x20..0x7E）；出现控制字符/非 ASCII/中间换行 → 拒绝。
 *   - 限速：**没有事件且没有急停**的帧，若距上一帧被接受不到
 *     `min_interval_ms`（默认 10 ms = 100 Hz）则丢弃并计数（§7 目标 20~50 Hz）。
 *     带事件的帧**不参与限速**（丢一个按键就是丢一条命令），`est=1` 也不参与。
 *
 * ==========================================================================
 * 字段表（**唯一一份**；解析器和编码器都走它）
 * ==========================================================================
 * | 键 | 旧键名 | 取值/标度 | 必填(严格) | 说明 |
 * |---|---|---|---|---|
 * | `rdog` | —      | 1       | ✅ | 帧魔数 + 协议版本（当前 =1）。越界报 `BAD_VERSION` |
 * | `seq`  | —      | 0..2^32-1 | ✅ | 单调递增，回绕安全 |
 * | `t`    | —      | 0..2^32-1 | ✅ | 发送端本地毫秒，仅参考 |
 * | `mode` | `mode` | 0..2    | ❌ | 0=POSE 1=CHAIN 2=ACTION（见 `app/motion.h`）|
 * | `est`  | `est`  | 0..1    | ✅ | 1=急停。可被 `proto_peek_estop()` 无条件读出 |
 * | `spd`  | `f`    | -100..100 | ✅ | 前后摇杆，**百分比**（原版 f 的原始值）|
 * | `turn` | `t`    | -100..100 | ✅ | 转向摇杆，百分比 |
 * | `ay`   | `jy`   | -100..100 | ✅ | 机械臂上下摇杆，百分比 |
 * | `ax`   | `jx`   | -100..100 | ✅ | 机械臂前后摇杆，百分比 |
 * | `grip` | `grip` | 0..100  | ✅ | 夹爪闭合百分比（0=张 100=合）|
 * | `pit`  | `pit`  | -15..15 | ✅ | 俯仰目标，整度（= `pit_max_ang`）|
 * | `rol`  | `rol`  | -15..15 | ✅ | 横滚目标，整度（= `rol_max_ang`）|
 * | `yst`  | `yst`  | -40..40 | ✅ | 重心横向目标，mm（网页滑条 ±40）|
 * | `hgt`  | `hgt`  | 70..110 | ✅ | 机身高度，mm（网页滑条 70..110）|
 * | `ev`   | `key`  | 事件位掩码 | ✅ | 见下表；空列表合法 |
 *
 * ⚠️ **协议层不做单位换算。** 原版的 `thr = f * JOY_THR_MAX/100`、死区 10/20、
 * `t = -turn`、`joy_fwd_sign` 取反，**全部留给上层**（那是 `app_chain` 的活）。
 * 本层只做类型 + 范围校验，范围就照抄原版网页摇杆的 ±100。
 *
 * ==========================================================================
 * 事件白名单（**唯一一份**，`kEvents`）
 * ==========================================================================
 * `go` `gc` `g0` `g1` `is` `ss` `btn_stand` `btn_sit` `btn_wave` `btn_crawl`
 * `t9` `sc` `l1` `l2` `l3` `l4` `hi` `hd` `si` `sd` `ip` `id` `am1` `am0`
 *
 * 语义（权威来源 = `web_common.py` 的 `handle_control_key()` + `web_c.py` 的标定键，
 * 逐键对照见 `ESP-IDF_C迁移表.md` §0.5(3)）：
 *   - `g0`/`g1` = `stable(False)` + `gait(0/1)`（TROT/WALK）
 *   - `go`/`gc` = `stable(True/False)`。**没有 IMU ⇒ 上层必须显式回"不支持：无 IMU"**
 *     （§0.5(4).1 / §8.7：不能返回"成功"却什么也没做）
 *   - `is` = `gait(0)` + 原地踏步 5 s（唯一带本地截止时刻的事件）
 *   - `ss` = **清姿态**（`Pitch=0; Roll=0`）+ `stable(False)` + `gait(0)` + 回一个
 *     "进入标定"标志。⚠️ **`ss` 不是停车**（§0.5(4).2）；要停车请用 `spd=0;turn=0`
 *   - `btn_stand`/`btn_sit`/`btn_wave`/`btn_crawl` = 动作层动画（要求 `mode=ACTION`）
 *   - `t9` = `servo_init(1)`（切"直接站姿"）；`sc` = 标定结果存 NVS（§8.4，不再写 config_s.py）
 *   - `l1`..`l4` = 选标定腿（= `cal_leg_sel`，存在 `app_config` 里，是**持久状态**）
 *   - `hi`/`hd`/`si`/`sd`/`ip`/`id` = 选中腿的 大腿/小腿/髋 中位角 ∓1（`h` 是"大"不是"髋"，
 *     §0.5(3) 的命名陷阱）
 *   - `am1`/`am0` = 机械臂使能（P6；本轮**协议只需要能表达**，行为在 P6 实现）
 *
 * **故意没有做成事件的旧按键**（各自有等价的字段写法，避免一件事两套机制，P-27）：
 *   - `btn_stop` → `spd=0;turn=0`（§0.5(7) 把"短超时 = 输入归零、保持姿态"
 *     直接定义为 `btn_stop` 语义，也就是中性快照）
 *   - `btn_grip_open` / `btn_grip_close` → `grip=0` / `grip=100`
 *   - `l1`..`l4` 是**事件**（旧页面就是 `?key=l2` 这么发的），不另设 `leg` 字段
 * 旧页面用 `key=btn_stop` 时，`kLegacyCmd` 会把上面这些等价关系补上（只有旧路径有）。
 * 另：严格格式的 `ev=` 允许为空列表（= 这一帧没有任何事件）；旧格式的 `key=` 不允许为空
 * （那是一条命令，空命令没有意义 → `EMPTY_VALUE`）。
 *
 * ==========================================================================
 * 接收端规则（协议只提供数据；下面这些**语义**由上层实现，写在这里只写一份）
 * ==========================================================================
 *  1. **缺省 ≠ 默认值。** 可选键（如 `mode`）缺失时结构体里是 0，但 `present`
 *     对应位是 0；上层**必须**先看 `present`，不能把 0 当成"被命令成 0"。
 *  2. **急停永远优先**：`proto_peek_estop()` 不要求整帧合法，能在被拒的帧里
 *     把 `est` 读出来（旁路新鲜度，§0.5(7)）。急停帧也不参与限速。
 *     读不出来时（帧损坏到连 `est` 都没有）→ 按心跳策略停（见 4）。
 *  3. **`ss` 覆盖同帧的 `pit`/`rol`**（原版是强制 `Pitch=Roll=0`），`yst` 不动。
 *  4. **两级超时**（§0.5(4).3 / (7)）：短超时（`proto_is_stale()`，200~300 ms）
 *     ⇒ 输入归零、**保持姿态**；长超时（`proto_is_expired()`，默认 2 s）
 *     ⇒ 才放松舵机。两个阈值都是显式参数。
 *  5. **动作期间**（§0.5(6)）：忽略摇杆与 `g0`/`g1`，但急停永远有效；
 *     心跳超时只表示"不再接受新的运动命令"，**不硬停动作**。
 *  6. **模式与事件不自洽的帧**（如 `mode=CHAIN` + `btn_wave`）**不是协议错误**
 *     —— 协议层只保证"数据合法"；上层按 §0.5(6) 的规则决定忽略并计数。
 *     （刻意不在协议层拒绝：§0.5(6).1 明确要求动作期间"忽略"而不是"拒绝"。）
 *  7. 计数必须可读回（§0.5(7)"接受/丢弃/过期/坏帧"）：`proto_stats_t`。
 *
 * NOTE: 本文件只依赖 <stddef.h>/<stdint.h>；没有动态分配，没有全局可变状态，
 *       输出结构体由调用方提供，解析器对**任意字节序列**都安全（长度有上限、
 *       不越界、不读未初始化内存）。
 */

#ifndef PROTO_H
#define PROTO_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 常量
 * ========================================================================== */

/** 协议版本（`rdog` 键的取值；越界报 PROTO_E_BAD_VERSION） */
#define PROTO_VERSION 1

/** 严格格式单帧字节上限（含结尾 '\n'）—— 规范帧满打满算约 225 字节 */
#define PROTO_MAX_FRAME 512u

/** 旧格式单帧字节上限（原版 `cl.recv(1024)` 的规模，含 HTTP 头） */
#define PROTO_MAX_LEGACY 1024u

/** 键名 / 事件名最大长度 */
#define PROTO_MAX_KEY   16u
#define PROTO_MAX_EVENT 16u

/** 规范形式整数字面量最大字符数（"-2147483648" 11 字符，留 1 位余量） */
#define PROTO_MAX_NUM   12u

/** 心跳（短超时）：迁移表 §7 要求 200~300 ms 超时后停车/保持姿态 */
#define PROTO_HB_MIN_MS 200u
#define PROTO_HB_MAX_MS 300u
/** 默认取中值；由 `proto_decoder_set_hb_ms()` 在 [200,300] 内可配（§0.5(7)） */
#define PROTO_HB_DEFAULT_MS 250u

/** 长超时：超了才放松舵机（§0.5(4).3） */
#define PROTO_LONG_MIN_MS     1000u
#define PROTO_LONG_MAX_MS     10000u
#define PROTO_LONG_DEFAULT_MS 2000u

/** 限速：更快的纯状态帧视为洪泛（§7 目标 20~50 Hz，上限 100 Hz） */
#define PROTO_MIN_INTERVAL_MIN_MS     0u
#define PROTO_MIN_INTERVAL_MAX_MS     1000u
#define PROTO_MIN_INTERVAL_DEFAULT_MS 10u

/** `proto_age_ms()` 在没有收到过任何帧时的返回值 */
#define PROTO_AGE_NEVER 0xFFFFFFFFu

/** 字段 / 事件数量上限（内部用 32 位位掩码记 present/seen/ev） */
#define PROTO_MAX_FIELDS 32u
#define PROTO_MAX_EVENTS 32u

/* ==========================================================================
 * 错误码 / 分类
 * ========================================================================== */

typedef enum {
    PROTO_OK = 0,
    PROTO_E_NULL,            /**< 参数为空指针 */
    PROTO_E_EMPTY,           /**< 空帧（或严格格式只有终止符）*/
    PROTO_E_NO_FIELDS,       /**< 一个字段都没认出（多半是页面请求，不是命令）*/
    PROTO_E_TOO_LONG,        /**< 超过该格式的单帧上限 */
    PROTO_E_NO_TERMINATOR,   /**< 严格格式缺结尾 '\n' */
    PROTO_E_CTRL_CHAR,       /**< 严格格式正文里有控制字符 / 非 ASCII */
    PROTO_E_BAD_SEPARATOR,   /**< 空字段（`;;` / 开头 / 结尾多一个 ';'）*/
    PROTO_E_NO_EQUALS,       /**< 段里没有 '=' */
    PROTO_E_BAD_KEY,         /**< 键名非法（首字符非小写字母 / 非法字符 / 超长）*/
    PROTO_E_UNKNOWN_KEY,     /**< 不在白名单里的键 */
    PROTO_E_DUP_KEY,         /**< 同一个键出现两次 */
    PROTO_E_EMPTY_VALUE,     /**< 空取值 */
    PROTO_E_BAD_INT,         /**< 整数语法错（'+' / 夹符号 / 前导零 / "-0" / 非数字 / 过长）*/
    PROTO_E_RANGE,           /**< 超出字段表的范围 */
    PROTO_E_BAD_VERSION,     /**< `rdog` 版本不对（也是范围错，但单独报）*/
    PROTO_E_EVENT_BAD_NAME,  /**< 事件名非法 / 空（含列表里的空项）*/
    PROTO_E_EVENT_UNKNOWN,   /**< 事件名不在白名单里 */
    PROTO_E_EVENT_DUP,       /**< 同一个事件出现两次 */
    PROTO_E_MISSING_KEY,     /**< 严格格式必填键缺失 */
    PROTO_E_SEQ_STALE,       /**< seq 不严格更新（重复 / 重放 / 乱序）*/
    PROTO_E_RATE_LIMIT,      /**< 帧率超过上限 */
    PROTO_E_COUNT
} proto_err_t;

/** 错误的四个类别 —— 和 §0.5(7) 要回读的四个计数一一对应 */
typedef enum {
    PROTO_CLS_FRAME = 0, /**< 坏帧（结构级） */
    PROTO_CLS_FIELD,     /**< 坏帧（字段级） */
    PROTO_CLS_SEQ,       /**< 丢弃（seq 不严格更新） */
    PROTO_CLS_RATE       /**< 限速丢弃 */
} proto_err_class_t;

/** 报错详情：既能报"哪个字段"，也能报"哪个键名"（未知名也留得住）*/
typedef struct {
    proto_err_t code;                 /**< PROTO_OK = 成功 */
    int         field;                /**< 命中的字段下标；-1 = 不是字段级错误 */
    char        key[PROTO_MAX_KEY + 1]; /**< 出问题的键名/事件名（NUL 结尾，已截断）*/
} proto_error_t;

/* ==========================================================================
 * 字段表
 * ========================================================================== */

typedef enum {
    PROTO_KIND_INT = 0, /**< 整数，范围由 min/max 给 */
    PROTO_KIND_EVLIST   /**< 逗号分隔的事件名表（存成位掩码）*/
} proto_kind_t;

/** 字段在协议里的特殊角色（解析器只认角色，不认具体键名 —— 不写第二份键名）*/
typedef enum {
    PROTO_ROLE_NONE = 0,
    PROTO_ROLE_MAGIC, /**< 帧魔数 + 版本：越界报 BAD_VERSION；`proto_sniff_format()` 靠它认帧 */
    PROTO_ROLE_ESTOP, /**< 急停：可被 `proto_peek_estop()` 无条件读出 */
    PROTO_ROLE_SEQ    /**< 序号：严格更新 + 回绕安全比较 */
} proto_role_t;

/** 必填性：严格格式里缺失即整帧拒绝；旧格式一律按可选（现场页面没有 seq/mode）*/
typedef enum {
    PROTO_REQ_OPTIONAL = 0,
    PROTO_REQ_STRICT   = 1
} proto_req_t;

typedef struct {
    const char   *key;        /**< 严格格式的规范键名（编码器也用它）*/
    const char   *legacy;     /**< 旧格式接受的键名；NULL = 旧格式不认这个字段 */
    proto_kind_t  kind;
    proto_req_t   required;   /**< 严格格式里是否必填 */
    proto_role_t  role;
    uint8_t       is_unsigned;/**< 1 = 结构体里存成 uint32_t（否则 int32_t，都是 4 字节）*/
    uint16_t      offset;     /**< offsetof(proto_cmd_t, <成员>) */
    int64_t       min;
    int64_t       max;
    const char   *scale;      /**< ASCII 标度说明，仅供打印/文档 */
} proto_field_t;

/* ==========================================================================
 * 命令
 * ========================================================================== */

/**
 * @brief 一帧命令解出来的东西。
 *
 * 所有成员都是**定标整数**（没有浮点）。`present` 每一位对应字段表里同下标的字段：
 * **1 = 这一帧确实命令了它，0 = 这一帧没提**（旧格式的部分更新语义）。
 * 编码器不看 `present`（严格帧永远写全字段）。
 */
typedef struct {
    uint32_t rdog;   /**< 帧魔数 + 版本（解出来恒 = PROTO_VERSION）*/
    uint32_t seq;    /**< 序号 */
    uint32_t t_ms;   /**< 发送端时间戳（毫秒），仅供参考 */
    int32_t  mode;   /**< 0=POSE 1=CHAIN 2=ACTION */
    int32_t  est;    /**< 1 = 急停 */
    int32_t  spd;    /**< 前后摇杆 %   -100..100 */
    int32_t  turn;   /**< 转向摇杆 %   -100..100 */
    int32_t  ay;     /**< 臂上下摇杆 % -100..100 */
    int32_t  ax;     /**< 臂前后摇杆 % -100..100 */
    int32_t  grip;   /**< 夹爪 %       0..100 */
    int32_t  pit;    /**< 俯仰目标度   -15..15 */
    int32_t  rol;    /**< 横滚目标度   -15..15 */
    int32_t  yst;    /**< 重心横向 mm  -40..40 */
    int32_t  hgt;    /**< 机身高度 mm  70..110 */
    uint32_t ev_mask;/**< 事件位掩码（下标见事件表）*/
    uint32_t present;/**< 哪些字段这一帧真的出现了 */
} proto_cmd_t;

/* ==========================================================================
 * 统计 / 解码器状态
 * ========================================================================== */

/**
 * @brief 四个可回读的计数（§0.5(7)：接受 / 丢弃 / 过期 / 坏帧）+ 细分。
 *
 * 恒等式（测试会断言）：
 *   frames   == accepted + rejected
 *   rejected == bad_frame + bad_field + dropped_seq + rate_limited
 *   rejected == sum(by_err[0..PROTO_E_COUNT-1])
 */
typedef struct {
    uint32_t frames;         /**< decode 被调用的次数 */
    uint32_t accepted;       /**< 接受 */
    uint32_t rejected;       /**< 丢弃（总数）*/
    uint32_t bad_frame;      /**< 坏帧（结构级）*/
    uint32_t bad_field;      /**< 坏帧（字段级）*/
    uint32_t dropped_seq;    /**< 丢弃（seq 重复/重放/乱序）*/
    uint32_t rate_limited;   /**< 限速丢弃 */
    uint32_t heartbeat_gaps; /**< 过期：上一帧已经超时了这一帧才来（= 断连次数）*/
    uint32_t estop_frames;   /**< 接受的帧里 est=1 的个数 */
    uint32_t estop_peeks;    /**< `proto_peek_estop()` 调用次数 */
    uint32_t estop_hits;     /**< `proto_peek_estop()` 读出 est=1 的次数 */
    uint32_t legacy_frames;  /**< 走旧格式入口的次数（诊断用）*/
    uint32_t by_err[PROTO_E_COUNT];
} proto_stats_t;

/** 解码器状态（序号 / 新鲜度 / 限速阈值 / 计数）*/
typedef struct {
    proto_stats_t stats;
    uint32_t      last_seq;        /**< 上一次被接受的 seq */
    uint32_t      last_rx_ms;      /**< 上一次被接受的本地时刻 */
    uint32_t      hb_ms;           /**< 短超时 200..300 */
    uint32_t      long_ms;         /**< 长超时（放松舵机）*/
    uint32_t      min_interval_ms; /**< 限速下限（0 = 不限速）*/
    uint8_t       have_frame;      /**< 收到过至少一帧 */
    uint8_t       reserved[3];
} proto_decoder_t;

/** 帧格式（`proto_sniff_format()` 的返回值）*/
typedef enum {
    PROTO_FMT_STRICT = 0,
    PROTO_FMT_LEGACY = 1,
    PROTO_FMT_UNKNOWN = 2 /**< 一个已知键都没认出来 */
} proto_fmt_t;

/* ==========================================================================
 * 解码器生命周期
 * ========================================================================== */

/** @brief 复位解码器（零序号、零计数、默认阈值）。`d` 为空则什么也不做。*/
void proto_decoder_init(proto_decoder_t *d);

/** @brief 设短超时（毫秒），钳到 [200,300]；返回生效值。*/
uint32_t proto_decoder_set_hb_ms(proto_decoder_t *d, uint32_t ms);

/** @brief 设长超时（毫秒），钳到 [hb_ms,10000]；返回生效值。*/
uint32_t proto_decoder_set_long_ms(proto_decoder_t *d, uint32_t ms);

/** @brief 设限速下限（毫秒），钳到 [0,1000]；返回生效值。0 = 不限速。*/
uint32_t proto_decoder_set_min_interval_ms(proto_decoder_t *d, uint32_t ms);

/* ==========================================================================
 * 解码 / 编码
 * ========================================================================== */

/**
 * @brief 解严格格式的一帧（新发送方）。
 *
 * @param d      解码器状态（序号、计数、阈值）；不能为空
 * @param buf    待解字节（**不必** NUL 结尾）
 * @param len    buf 的可读长度
 * @param now_ms **本地**时刻（毫秒，可回绕），只用来记到达时刻与判限速
 * @param out    成功时写满；**失败时一个字节都不动**（全有或全无）
 * @param err    可空；成功时 `code=PROTO_OK`、`field=-1`、`key[0]=0`
 * @return PROTO_OK 或 proto_err_t
 */
int proto_decode(proto_decoder_t *d, const char *buf, size_t len, uint32_t now_ms,
                 proto_cmd_t *out, proto_error_t *err);

/**
 * @brief 解旧格式的一帧（现场 `drive.html` / `control.html` 的报文）。
 *
 * 宽容点：`;`/`&`/`?`/HTTP 前缀后缀都当垃圾跳过；键可以粘连（`f=-100t=20`）；
 * 取值取前导整数、忽略值尾部垃圾（`f=10abc` → 10，允许前导 `+`/前导零）；
 * 不要求结尾 `\n`；没有 `seq`（所以没有重放保护）。
 * 严格点：键名必须在白名单里（出现任何未知的 `名字=` 都拒绝）、取值范围、
 * 事件名一律照字段表/事件表校验。缺少的键**就是"这一帧没提"**（`present` 位表达），
 * 不是"取默认值"。
 */
int proto_decode_legacy(proto_decoder_t *d, const char *buf, size_t len, uint32_t now_ms,
                        proto_cmd_t *out, proto_error_t *err);

/** @brief 看一帧像哪种格式（严格格式以魔数键开头）。用于 comm 层分派。*/
proto_fmt_t proto_sniff_format(const char *buf, size_t len);

/**
 * @brief 编码成严格格式的规范帧（测试与将来的发送方共用同一个布局）。
 *
 * 先整帧校验（同一张字段表）：任何取值越界、事件掩码有白名单外的位 → 拒绝。
 * 键顺序 = 字段表顺序，`ev` 里的事件顺序 = 事件表顺序（规范形式）。
 *
 * @param cmd  待编码命令（`present` 不参与；总是写全字段）
 * @param buf  输出缓冲
 * @param cap  buf 的容量（含结尾 NUL）
 * @return 写入的字节数（不含 NUL）；失败返回 0 且 `buf` 变成空串
 *         （**绝不会**留下半截帧）
 */
size_t proto_encode(const proto_cmd_t *cmd, char *buf, size_t cap);

/** @brief 清零（此时 `rdog=0`，**不合法**，编码会失败 —— 故意的）*/
void proto_cmd_zero(proto_cmd_t *cmd);

/**
 * @brief 填一帧合法的完整命令（发端便利）。
 *
 * ⚠️ 这是**发端**的样板，不是解析器的默认值：解析失败永远不写 out；可选键缺失时
 * 结构体里是 0 但 `present` 位为 0。
 * 取值全部**从字段表推导**（状态量取范围中点、摇杆取 0、`seq=1`、`rdog=版本`），
 * **刻意不复制 `app_config` 的默认值**（P-22/P-27：同一份东西只有一份）。
 * 真正的初始状态由上层从 `app_config` 填进 `app_chain`。
 */
void proto_cmd_defaults(proto_cmd_t *cmd);

/** @brief 逐字段比较（走同一张表）包含 `present`；padding 不参与。*/
int proto_cmd_equal(const proto_cmd_t *a, const proto_cmd_t *b);

/** @brief 取第 `index` 个字段的当前值（下标越界返回 0）。*/
int64_t proto_cmd_get(const proto_cmd_t *cmd, size_t index);

/**
 * @brief 设第 `index` 个字段（顺手按同一张表做范围校验）并置 `present` 位。
 * @return 1 = 设置成功；0 = 下标越界或取值越界（**不改任何东西**）
 *
 * ⚠️ 这是"通用层"用的入口（例如把命令转发给别的模块）；业务层直接用有名成员更清楚。
 */
int proto_cmd_set(proto_cmd_t *cmd, size_t index, int64_t value);

/** @brief 这一帧是否带指定事件（按名字查表）。*/
int proto_cmd_has_ev(const proto_cmd_t *cmd, const char *name);

/** @brief 这一帧是否带下标为 `ev_index` 的事件。*/
int proto_cmd_has_ev_index(const proto_cmd_t *cmd, int ev_index);

/** @brief 事件个数（popcount，不另存字段）。*/
uint32_t proto_cmd_ev_count(const proto_cmd_t *cmd);

/* ==========================================================================
 * 急停旁路（安全路径）
 * ========================================================================== */

/**
 * @brief 从**任意**字节流里把 `est` 读出来，**不要求整帧合法**。
 *
 * §0.5(7)：「急停……要能从帧里无条件读出来」。所以这个函数故意宽松：
 * 只找 `est=<前导整数>`，取值必须是 0 或 1。找不到 / 读不出 → 返回 0
 * （**不会**返回一个它没验证过的值）。
 * 扫描上限 = 前 PROTO_MAX_FRAME 个字节（有界，恶意超长帧不会拖死任务）。
 * 读出来的 `est=1` 只可能让机器**更安全**（停），不可能更危险。
 *
 * @param d        可空；非空时累加 `estop_peeks` / `estop_hits`
 * @param est_out  可空；读到 0/1 时写入
 * @return 1 = 读到了（`*est_out` 有效），0 = 这一帧里没有可用的急停
 */
int proto_peek_estop(proto_decoder_t *d, const char *buf, size_t len, int *est_out);

/* ==========================================================================
 * 新鲜度（心跳）/ 序号
 * ========================================================================== */

/** @brief 距上一帧被接受的毫秒数（回绕安全）。没收到过帧 → PROTO_AGE_NEVER。*/
uint32_t proto_age_ms(const proto_decoder_t *d, uint32_t now_ms);

/** @brief 距离超过阈值？没收到过帧也算超时（安全侧）。*/
int proto_is_stale_at(const proto_decoder_t *d, uint32_t now_ms, uint32_t timeout_ms);

/** @brief 短超时判定（用 `d->hb_ms`，200~300 ms）：超了 ⇒ 输入归零、保持姿态。*/
int proto_is_stale(const proto_decoder_t *d, uint32_t now_ms);

/** @brief 长超时判定（用 `d->long_ms`，默认 2 s）：超了 ⇒ 才放松舵机。*/
int proto_is_expired(const proto_decoder_t *d, uint32_t now_ms);

/** @brief 上一帧被接受的本地时刻；没收到过帧 → 0。*/
uint32_t proto_last_rx_ms(const proto_decoder_t *d);

/** @brief 序号比较：`candidate` 是否**严格新于** `last`（回绕安全，不是 `a > b`）。*/
int proto_seq_newer(uint32_t candidate, uint32_t last);

/* ==========================================================================
 * 表查询（给上层和测试用；表本身是只读的）
 * ========================================================================== */

/** @brief 取字段表（唯一一份）与条目数。*/
const proto_field_t *proto_fields(size_t *count);

/** @brief 取事件名表（唯一一份）与条目数。*/
const char *const *proto_events(size_t *count);

/** @brief 取第 `idx` 个事件名（越界返回 NULL）。*/
const char *proto_event_at(size_t idx);

/** @brief 规范键名 → 字段下标；找不到返回 -1。*/
int proto_field_find(const char *key);

/** @brief 旧格式键名 → 字段下标；找不到返回 -1。*/
int proto_field_find_legacy(const char *key);

/** @brief 事件名 → 事件下标；找不到返回 -1。*/
int proto_event_find(const char *name);

/** @brief 错误码属于哪一类计数。*/
proto_err_class_t proto_err_class(proto_err_t code);

/** @brief 错误码的 ASCII 名字（可打印；越界返回 "?"）。*/
const char *proto_err_name(proto_err_t code);

#ifdef __cplusplus
}
#endif

#endif /* PROTO_H */
