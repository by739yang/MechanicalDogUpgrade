/**
 * @file    web_cmd.h
 * @brief   网页遥控的**动作翻译层**：已验证的数字 -> 机器人命令
 *          （原版 `micropython/web_common.py` 的 `_joy_f_to_thr` /
 *            `_thr_is_forward` / `_thr_is_backward` / `apply_dog_stick` /
 *            `process_dog_from_req`）
 *
 * ## 这一层是什么，不是"协议解析"
 *
 * `comm/proto.{h,c}` 只把一段 HTTP 请求切成**已验证的数**：`f=` / `t=` 的整数值、
 * `key=` 的字符串。**它不决定狗怎么动**。真正决定"狗怎么动"的是本模块：
 *
 * ```
 * f=50 ──_joy_f_to_thr──> thr = -3.0
 *        ──apply_dog_stick──> t = -int(turn)
 *                             |t| < 20 ? set_joy_turn(0), L=R=1
 *                                      : set_joy_turn(t), L,R = _turn_phase_lr(t)
 *                             四选一 -> _go(spd, L, R)
 *        _go = drive (gait_mode==1) 还是 move (否则)   ← ⚠️ P-26
 * ```
 *
 * ## 为什么这个模块必须单独做"调用序列"对照
 *
 * P-26（`move` 还是 `drive`）与 P-29（某条命令根本没被调用到）**都是"调用哪个函数"
 * 的错误，不是"算错哪个数"**：数值对照两边可能同时为绿。所以
 * `tools/golden/test_web_cmd.c` 对照的是**有序调用序列**，
 * 见 `golden/web_cmd.csv` 的 `n_calls` / `callseq` 两列。
 *
 * ## 零 ESP-IDF 依赖
 *
 * 本文件与 `web_cmd.c` **不 include 任何 ESP-IDF 头**（无 `esp_err.h`、
 * 无 `esp_log.h`、无 FreeRTOS）。所有对外部世界的依赖都通过下面的
 * `web_cmd_ctx_t` **函数指针表**显式传入 —— 于是：
 *   * 宿主测试可以直接换成记录器（`tools/golden/test_web_cmd.c`）；
 *   * 固件侧由 P5 接线代码填上 `app_chain_*` / `app_action_*` 的真函数。
 * 这不是为了好测才抽的接口：**"这一层调了哪些下游函数"本身就是被测行为**，
 * 所以那些调用点必须是显式可见的。
 *
 * ## 两个必须显式传入、本模块**不自己造状态**的量
 *
 * 1. **`arm_enabled`（机械臂是否开启）是 P6，现在还不存在。**
 *    原版是 `mech_arm.is_enabled()`（`web_common.py:115`）。本模块**不猜**这个状态，
 *    由调用方通过 `web_cmd_process_request()` 的参数传进来；
 *    留着它是因为 `force` 标志是有意义的：drive 轻量页传 `dog_when_arm=true`，
 *    机械臂开着时**仍然能控狗**（`web_ctl.py:70`）。
 * 2. **`now_ms`** 由调用方传入（原版读 `utime.ticks_ms()`），
 *    因为"原地踏步还没到期"这条分支只有在**可控时钟**下才能对照（P-4 那一类）。
 *
 * ## P-18 / P-24 自查：这个模块**不**做的事
 *
 *   * 它**不**解析字符串、**不**找 `f=` / `&` / `?` —— 那是 `proto.{h,c}` 的活。
 *     本模块的输入是**已经解析好的整数**。重复实现一套分词器 = P-22/P-27（同一件事
 *     写两份，迟早不一致）。
 *   * 它**不**改 `gait_mode`。`g0`/`g1` 按键走 `app_chain_set_gait()`，
 *     属于按键分发（`web_common.handle_control_key`），不属于摇杆这一层。
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================== */
/*  常量（逐条对应 web_common.py 第 6~9 行）                              */
/* ====================================================================== */

/** `JOY_THR_MAX = 6.0`  —— `_joy_f_to_thr` 的上夹（`web_common.py:6`） */
#define WEB_CMD_JOY_THR_MAX      6.0f
/** `JOY_THR_MIN = -3.0` —— `_joy_f_to_thr` 的下夹（`web_common.py:7`） */
#define WEB_CMD_JOY_THR_MIN     (-3.0f)
/** `JOY_DEAD = 10` —— 前后死区：`|vf| < 10` 才算零（`web_common.py:8`） */
#define WEB_CMD_JOY_DEAD         10
/** `JOY_TURN_DEAD = 20` —— **转向**死区，与 `HIP_TURN_DEAD=10` 不是一回事 */
#define WEB_CMD_JOY_TURN_DEAD    20
/** `TURN_DRV_SPD = 2.5`（`padog.py:151`；原版用 `getattr` 兜底同值） */
#define WEB_CMD_TURN_DRV_SPD     2.5f
/** `BACK_DRV_SPD = 2.0`（`padog.py:150`） */
#define WEB_CMD_BACK_DRV_SPD     2.0f
/** `_thr_is_forward/_thr_is_backward` 的阈值（`web_common.py:89/98`） */
#define WEB_CMD_THR_SIGN_THRESH  0.35f
/**
 * `joy_fwd_sign`（`padog.py` 的实测值 = **-1**）。
 *
 * 它把"摇杆往前推"翻成**负**的 `thr`（`_joy_f_to_thr` 里 `thr *= sign`），
 * 于是 `_thr_is_forward` 的判据也跟着翻成 `thr < -0.35`
 * （`web_common.py:98`，**不是** `thr > 0.35`）。
 *
 * ⚠️ 这是**编译期常量而不是配置项**，因为 C 侧没有等价的 `padog.joy_fwd_sign`
 * 运行期全局；改成别的值会让整套网页摇杆方向反掉，所以把它钉在这里并让
 * "符号为 +1"的分支永远不可达（原版那两条 `getattr` 兜底也永远不走）。
 */
#define WEB_CMD_JOY_FWD_SIGN    (-1)

/* ====================================================================== */
/*  函数指针表：本模块对外部世界的**全部**依赖                            */
/* ====================================================================== */

typedef struct {
    /**
     * `padog.set_leg_sit_offsets(front, rear)`
     * （固件 = `app_chain_set_sit_offsets`）。
     *
     * ⚠️ `process_dog_from_req` 的**每一条**路径都会调它（`web_common.py:152/158/163`），
     * 漏一处就是 P-25/P-29 那类"副作用被吞掉"。测试的调用序列对照专门钉这一条。
     */
    void (*set_leg_sit_offsets)(void *user, float front_y, float rear_y);

    /** `padog.move(spd, L, R)`（固件 = `app_chain_jog`）。**会**把步态改回 TROT。 */
    void (*move)(void *user, float spd, int L, int R);

    /** `padog.drive(spd, L, R)`（固件 = `app_chain_drive`）。**不切**步态，WALK 的唯一入口。 */
    void (*drive)(void *user, float spd, int L, int R);

    /** `padog.set_joy_turn(t)`（固件 = `app_chain_set_joy_turn`） */
    void (*set_joy_turn)(void *user, float pct);

    /**
     * `padog._turn_phase_lr(jt)`（固件 = `control_chain_turn_phase_lr`）。
     *
     * ⚠️ 它内部用的阈值是 **`HIP_TURN_DEAD = 10`**（`padog.py:153`），
     * 而本模块的**转向死区是 20**（`JOY_TURN_DEAD`）—— 两个不同的数，别混。
     * 详见 `web_cmd.c` 里 `apply_dog_stick` 的注释（本项目 §0.5(8) 第 4 条）。
     */
    void (*turn_phase_lr)(void *user, float joy_turn, int *out_l, int *out_r);

    /**
     * 读 `padog.gait_mode`（固件 = `app_chain_get_status().gait_mode`）。
     *
     * 只用于**一个**判断：`_walk = (gait_mode == 1)` ⇒ `_go` 取 `drive` 而不是 `move`
     * （`web_common.py:128~129`）。**这一条就是 P-26**：写错的话"先 `g1` 再推摇杆"
     * 会静默退回 TROT，而数值对照完全看不出来。
     */
    int (*gait_mode)(void *user);

    /**
     * 读 `padog.crawl_phase`（固件 = `app_chain_get_crawl` 的第一个出参）。
     *
     * `apply_dog_stick` 的第一句就是爬行守卫：非 0 → **直接 return**
     * （摇杆在爬行期间完全无效，`web_common.py:113`）。
     */
    int (*crawl_phase)(void *user);

    /**
     * 读 `padog.inplace_step_end_ms`（固件 = `app_action_get_inplace_step_end_ms`）。
     *
     * ⚠️ 故意**不**在 `web_cmd.c` 里直接调 `app_action_get_inplace_step_end_ms()`：
     * 那会把整个动作层拖进宿主链接（也就拖进 ESP-IDF 头）。它是"读一个时刻"，
     * 由接线方一行转发最省事，也让"这一层读了哪个模块的状态"保持显式。
     */
    int32_t (*inplace_step_end_ms)(void *user);

    /**
     * 回传给上面**每一个**回调的裸指针（调用方的上下文，例如测试的记录器）。
     *
     * ⚠️ 放在结构体**最后**，且必须由调用方显式赋值；`web_cmd_ctx_init()` 会把它
     * 置 NULL。本模块从不解引用它 —— 只是原样转交。
     */
    void *user;
} web_cmd_ctx_t;

/**
 * @brief 把上下文填成默认值：函数指针**全为 NULL**（不是猜一组默认实现）。
 *
 * 存在的意义：漏填一项时 `web_cmd_process_request()` 会返回
 * `WEB_CMD_ERR_INVALID_ARG`（而不是空指针解引用），且宿主测试能证明"读不到状态
 * 就没有命令被发出"。
 */
void web_cmd_ctx_init(web_cmd_ctx_t *ctx);

/* ====================================================================== */
/*  返回值                                                                 */
/* ====================================================================== */

typedef enum {
    WEB_CMD_OK = 0,                 /**< 正常处理完（含"输入无效、整条忽略"） */
    WEB_CMD_ERR_INVALID_ARG = -1,   /**< 上下文或必填函数指针为 NULL */
    /**
     * `thr` 落在 `[JOY_THR_MIN, JOY_THR_MAX]` 之外。
     *
     * 原版把夹取放在 `_joy_f_to_thr` 里、而 `apply_dog_stick` 是**公开函数**，
     * 所以它自己其实不夹。C 版把这条前置条件**变成可观测的返回值**：
     * 越界即拒绝，不静默照做。这样"某个调用点绕过了夹取"就是一个会现形的错误，
     * 而不是一个只在摇杆推到极限时才歪掉的行为。
     */
    WEB_CMD_ERR_THR_RANGE = -2,
} web_cmd_status_t;

/* ====================================================================== */
/*  纯函数（原版前两层，不碰任何状态）                                    */
/* ====================================================================== */

/**
 * @brief `_joy_f_to_thr(value_f)`：摇杆百分比 → 前后速度量。
 *
 * 逐行对应 `web_common.py:70~83`：
 * ```python
 * vf = float(value_f)
 * if abs(vf) < JOY_DEAD:      return 0.0        # 严格 <，所以 |vf|==10 不是 0
 * thr = vf * JOY_THR_MAX / 100.0                # 6.0/100 = 0.06
 * thr = thr * float(joy_fwd_sign)               # -1 ⇒ 往前推得负 thr
 * if thr >  JOY_THR_MAX: return JOY_THR_MAX
 * if thr <  JOY_THR_MIN: return JOY_THR_MIN
 * return thr
 * ```
 *
 * @param value_f `f=` 的整数值（**已经解析好的数**，不是字符串）
 * @return `[-3.0, 6.0]` 内的 `thr`
 *
 * @note 运算顺序照抄（`vf * 6.0 / 100.0` 再乘符号），**不**重排成
 *       `vf * -0.06`：只有顺序一致，float 舍入才逐位一致（P-13 那一类）。
 */
float web_cmd_joy_f_to_thr(int value_f);

/**
 * @brief `_thr_is_forward(thr)` —— `joy_fwd_sign < 0` 时 **`thr < -0.35` 才算前进**。
 *
 * 严格小于（`web_common.py:98`）：**恰好 -0.35 不算前进**，那一点是"前后都不算"
 * 的空档，`apply_dog_stick` 会走"四选一"的第一个分支（`_go(0, L, R)`）。
 */
int web_cmd_thr_is_forward(float thr);

/** @brief `_thr_is_backward(thr)`：`joy_fwd_sign < 0` 时 **`thr > 0.35`**（严格大于）。 */
int web_cmd_thr_is_backward(float thr);

/* ====================================================================== */
/*  翻译层                                                                 */
/* ====================================================================== */

/**
 * @brief `apply_dog_stick(thr, turn, force, *ctx)`：把一对量变成 0~2 条机器人命令。
 *
 * 顺序**就是行为**（`web_common.py:111~139`，本项目的 §0.5(8)）：
 *
 * 1. `crawl_phase != 0` → **直接 return**（爬行期间摇杆完全无效）；
 * 2. `arm_enabled && !force` → return（机械臂开启时摇杆归机械臂）；
 * 3. `t = -int(turn)` —— **取负**，且 `int()` **向零截断**（不是 floor）；
 * 4. `|t| < 20` → `set_joy_turn(0)`、`L=R=1`；否则 `set_joy_turn(t)`、
 *    `L,R = _turn_phase_lr(t)`；
 * 5. 四选一：停 / `_go(2.5, L, R)` / `_go(2.0, 1, 1)` / `_go(thr, L, R)`；
 * 6. `_go` = `drive`（`gait_mode == 1`）否则 `move`  ← ⚠️ P-26。
 *
 * @param thr   前后速度量。**必须是 `web_cmd_joy_f_to_thr()` 的输出**
 *              （= 已夹到 `[-3.0, 6.0]`），否则返回 `WEB_CMD_ERR_THR_RANGE`。
 *              这不是多余的：原版的 `_thr_is_forward/_thr_is_backward` 内部用的是
 *              **未夹取**的 `thr`，所以"绕过夹取"在原版里是个真实存在的差别；
 *              C 版把它变成显式错误，见 `web_cmd_status_t`。
 * @param turn  横杆原始值（`t=` 的整数值）。**符号会在此翻转**，不要在外面先翻。
 * @param force 原版 `force`：`true` = 纯遥控页，机械臂开着也照控狗。
 */
web_cmd_status_t web_cmd_apply_dog_stick(const web_cmd_ctx_t *ctx,
                                         float thr, int turn, int force,
                                         int arm_enabled);

/**
 * @brief `process_dog_from_req(req, thr_cache, turn_cache, dog_when_arm)` 的 C 版。
 *
 * 输入是**已经解析好**的 `f` / `t`：
 *   * `has_f == 0`（请求里没有 `f=`）⇒ **整条忽略**，一个命令都不发
 *     （原版 `if vf is None: return`，`web_common.py:144`）；
 *     ⚠️ 原版这条"同时要求 `f` 和 `t`"来自 `_parse_pair`：缺 `t=` 也算 `None`
 *     ⇒ 只发 `t=` 不动（§0.5(8) 第 1 条）。所以调用方要传 `has_t`。
 *   * 注意原版**没有**"只要 `f` 在就处理"的例外 —— `web_ctl.py:69` 那层
 *     `if req_data.find('f=') >= 0` 是**分发**侧的过滤，属于 P5 协议层。
 *
 * 分支（`web_common.py:142~165`）：
 *
 * 1. `inplace_step_end_ms != 0 && ticks_diff(end, now) > 0`（原地踏步还没到期）
 *    → `set_leg_sit_offsets(0,0)` + **`move(4, 1, 1)`** + return。
 *    ⚠️ 这条分支**不看 `thr` / `turn`**，而且用的**永远是 `move`**（不是 `drive`）。
 *    也正因为 `move()` 自己会清 `inplace_step_end_ms`，原版这个"网页原地步态测试"
 *    **只生效一帧**（P-26 已记录）。
 * 2. 死区（`thr == 0` 且 `|turn| < 20`）→ `set_leg_sit_offsets(0,0)`；
 *    非爬行时再 `set_joy_turn(0)` + `move(0, 0, 0)`（= **完全停车**）。
 *    ⚠️ 爬行期间**只**清腿部偏置、不发停车命令（爬行状态机自己在走）。
 * 3. 否则 → `set_leg_sit_offsets(0,0)` + `apply_dog_stick(thr, turn, force)`。
 *
 * 三条路径**都会** `set_leg_sit_offsets(0, 0)`（`web_common.py:152/158/163`）。
 *
 * @param ctx     上下文（函数指针表）
 * @param value_f `f=` 的整数值
 * @param has_f   请求里是否有 `f=`
 * @param turn    `t=` 的整数值
 * @param has_t   请求里是否有 `t=`
 * @param now_ms  当前毫秒（原版读 `utime.ticks_ms()`）
 * @param force   原版 `dog_when_arm`（drive 轻量页传 `true`）
 * @param arm_enabled 机械臂是否开启（**P6，本模块不自己造这个状态**）
 * @param out_thr     出参，可为 NULL：算出来的 `thr`（原版 `thr_cache` 的返回值）
 */
web_cmd_status_t web_cmd_process_request(const web_cmd_ctx_t *ctx,
                                         int value_f, int has_f,
                                         int turn, int has_t,
                                         int32_t now_ms,
                                         int force, int arm_enabled,
                                         float *out_thr);

#ifdef __cplusplus
}
#endif
