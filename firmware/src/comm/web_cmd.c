/**
 * @file    web_cmd.c
 * @brief   `micropython/web_common.py` 的"网页请求 -> 机器人命令"翻译层（纯 C）
 *
 * 逐行对应 `web_common.py`：
 *   `_joy_f_to_thr`        70~83
 *   `_thr_is_forward`      95~101
 *   `_thr_is_backward`     86~92
 *   `apply_dog_stick`      111~139
 *   `process_dog_from_req` 142~165
 *
 * 见 `web_cmd.h` 的文件头：本文件**不 include 任何 ESP-IDF 头**，
 * 对外部世界的依赖全部走 `web_cmd_ctx_t` 的函数指针表。
 */

#include "comm/web_cmd.h"

/* ====================================================================== */
/*  第 1 层：摇杆百分比 -> thr                                            */
/* ====================================================================== */

float web_cmd_joy_f_to_thr(int value_f)
{
    /* vf = float(value_f) */
    const float vf = (float)value_f;

    /* if abs(vf) < JOY_DEAD: return 0.0
     * ⚠️ 严格小于：|vf| == 10 **不是** 0（10 * 0.06 = 0.6，真的会走）。
     *    死区边界 9/10/11 三个值都在 golden 表里。 */
    const float avf = (vf < 0.0f) ? -vf : vf;
    if (avf < (float)WEB_CMD_JOY_DEAD) {
        return 0.0f;
    }

    /* thr = vf * JOY_THR_MAX / 100.0
     * ⚠️ 运算**顺序**照抄（先乘 6.0 再除 100.0），不重排成 vf * 0.06：
     *    只有顺序一致，float 舍入才逐位一致。 */
    float thr = vf * WEB_CMD_JOY_THR_MAX / 100.0f;

    /* thr = thr * float(getattr(padog, 'joy_fwd_sign', 1)) —— 实测 = -1
     * 「往前推」因此得到**负**的 thr。 */
    thr = thr * (float)WEB_CMD_JOY_FWD_SIGN;

    /* if thr > JOY_THR_MAX: return JOY_THR_MAX
     * if thr < JOY_THR_MIN: return JOY_THR_MIN */
    if (thr > WEB_CMD_JOY_THR_MAX) {
        return WEB_CMD_JOY_THR_MAX;
    }
    if (thr < WEB_CMD_JOY_THR_MIN) {
        return WEB_CMD_JOY_THR_MIN;
    }
    return thr;
}

/* ====================================================================== */
/*  第 2 层：前后判据（符号被 joy_fwd_sign 翻过）                         */
/* ====================================================================== */

int web_cmd_thr_is_forward(float thr)
{
    /* web_common.py:95~101
     *   if int(joy_fwd_sign) < 0: return float(thr) < -0.35
     *   return float(thr) > 0.35
     * 本工程 joy_fwd_sign == -1 ⇒ **只有第一条分支可达**；第二条留着并显式
     * `#if` 掉不可达的写法会掩盖"符号翻转"这件事，所以直接写死为负分支，
     * 并在头文件里说明为什么它不是一个配置项。 */
    return (thr < -WEB_CMD_THR_SIGN_THRESH) ? 1 : 0;
}

int web_cmd_thr_is_backward(float thr)
{
    /* web_common.py:86~92：符号为负 ⇒ thr > +0.35 才算后退 */
    return (thr > WEB_CMD_THR_SIGN_THRESH) ? 1 : 0;
}

/* ====================================================================== */
/*  第 3 层：apply_dog_stick                                              */
/* ====================================================================== */

void web_cmd_ctx_init(web_cmd_ctx_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    ctx->set_leg_sit_offsets = NULL;
    ctx->move                = NULL;
    ctx->drive               = NULL;
    ctx->set_joy_turn        = NULL;
    ctx->turn_phase_lr       = NULL;
    ctx->gait_mode           = NULL;
    ctx->crawl_phase         = NULL;
    ctx->inplace_step_end_ms = NULL;
    ctx->user                = NULL;
}

/** 上下文是否带齐了本函数需要的那几个入口（缺一个就什么都不发） */
static int ctx_ready(const web_cmd_ctx_t *ctx)
{
    return (ctx != NULL &&
            ctx->set_leg_sit_offsets != NULL &&
            ctx->move != NULL &&
            ctx->drive != NULL &&
            ctx->set_joy_turn != NULL &&
            ctx->turn_phase_lr != NULL &&
            ctx->gait_mode != NULL &&
            ctx->crawl_phase != NULL);
}

/** Python `int()`：**向零截断**，不是 floor（`-19 -> -19`；`turn` 本身是整数，
 *  这里保留一次转换只是为了把"int() 向零"写进代码里，别被改成 floor 除法） */
static int py_int_trunc(int v)
{
    return v;   /* C 的 int 已经是整数值；原版 int(int) 是恒等 */
}

/** `abs()`（整数域） */
static int iabs(int v)
{
    return (v < 0) ? -v : v;
}

web_cmd_status_t web_cmd_apply_dog_stick(const web_cmd_ctx_t *ctx,
                                         float thr, int turn, int force,
                                         int arm_enabled)
{
    if (!ctx_ready(ctx)) {
        return WEB_CMD_ERR_INVALID_ARG;
    }

    /* 原版把夹取放在 `_joy_f_to_thr` 里，`apply_dog_stick` 自己**不**夹；
     * 而 `_thr_is_forward/_thr_is_backward` 内部用的就是**传进来的** thr
     * ⇒ "绕过夹取"在原版里是一个真实存在的差别。C 版不静默照做：显式拒绝。 */
    if (thr > WEB_CMD_JOY_THR_MAX || thr < WEB_CMD_JOY_THR_MIN) {
        return WEB_CMD_ERR_THR_RANGE;
    }

    /* ---- 守卫 1：爬行期间摇杆完全无效（web_common.py:113~114） ---- */
    if (ctx->crawl_phase(ctx->user) != 0) {
        return WEB_CMD_OK;
    }

    /* ---- 守卫 2：机械臂开启且未强制（web_common.py:115~116）
     *      `force=True` = 纯遥控页（web_ctl.py:70 的 dog_when_arm），
     *      机械臂开着时**仍然**控狗。 */
    if (arm_enabled && !force) {
        return WEB_CMD_OK;
    }

    /* ---- t = -int(turn)：取负 + 向零截断（web_common.py:117） ---- */
    const int t = -py_int_trunc(turn);

    int L = 0;
    int R = 0;
    if (iabs(t) < WEB_CMD_JOY_TURN_DEAD) {
        /* |t| < JOY_TURN_DEAD(20) -> set_joy_turn(0)、L=R=1 */
        ctx->set_joy_turn(ctx->user, 0.0f);
        L = 1;
        R = 1;
    } else {
        /* |t| >= 20 -> set_joy_turn(t)、L,R = _turn_phase_lr(float(t))
         *
         * ⚠️ `_turn_phase_lr` 内部阈值是 **HIP_TURN_DEAD = 10**（`padog.py:153`），
         *    与这里的 **20** 是**两个不同的数**（§0.5(8) 第 4 条）。
         *    忠实实现 = 把 t 原样交给它、**不要**在调用点再夹一次。
         *    （可观测性说明，别把这条当"多余"去掉：因为这条 else 只在 |t|>=20 时走，
         *     而 20 > 10，所以两个阈值在**任何可达路径**上都不会给出不同结果 ——
         *     "不同阈值"这个差别在本层里实测不可观测。写清楚是为了下一个人不去
         *     "顺手统一"它们：统一之后一旦 `hip_turn_dead` 被配置改小就会立刻分叉。） */
        ctx->set_joy_turn(ctx->user, (float)t);
        ctx->turn_phase_lr(ctx->user, (float)t, &L, &R);
    }

    const int bf = web_cmd_thr_is_backward(thr);
    const int ff = web_cmd_thr_is_forward(thr);
    const int bt = (iabs(t) >= WEB_CMD_JOY_TURN_DEAD) ? 1 : 0;

    /* ---- _go 的选法：P-26 的网页层体现（web_common.py:128~129） ----
     *   _walk = int(getattr(padog,'gait_mode',0)) == 1
     *   _go   = padog.drive if _walk else padog.move
     * 写错(恒 move)的后果：WALK 摇杆每一帧都把步态改回 TROT —— 数值对照看不出来，
     * 调用序列对照一眼看出（golden/web_cmd.csv 的 callseq 列）。 */
    const int walk = (ctx->gait_mode(ctx->user) == 1);

    /* ---- 四选一（顺序即行为，web_common.py:130~139） ---- */
    if (!bt && !bf && !ff) {
        /* 不转、不前、不后 -> 速度 0，但**保留** L/R（转向回中时的 1,1） */
        if (walk) {
            ctx->drive(ctx->user, 0.0f, L, R);
        } else {
            ctx->move(ctx->user, 0.0f, L, R);
        }
        return WEB_CMD_OK;
    }
    if (bt) {
        /* 在转 -> TURN_DRV_SPD(2.5)，注意**不看 thr** */
        if (walk) {
            ctx->drive(ctx->user, WEB_CMD_TURN_DRV_SPD, L, R);
        } else {
            ctx->move(ctx->user, WEB_CMD_TURN_DRV_SPD, L, R);
        }
        return WEB_CMD_OK;
    }
    if (bf && !bt) {
        /* 后退且不转 -> BACK_DRV_SPD(2.0) + **L=R=1**（丢掉刚才算出来的相位，
         * 因为后退不带左右差动） */
        if (walk) {
            ctx->drive(ctx->user, WEB_CMD_BACK_DRV_SPD, 1, 1);
        } else {
            ctx->move(ctx->user, WEB_CMD_BACK_DRV_SPD, 1, 1);
        }
        return WEB_CMD_OK;
    }
    /* 其余 = 前进 -> 用 thr 本身 */
    if (walk) {
        ctx->drive(ctx->user, thr, L, R);
    } else {
        ctx->move(ctx->user, thr, L, R);
    }
    return WEB_CMD_OK;
}

/* ====================================================================== */
/*  第 4 层：process_dog_from_req                                         */
/* ====================================================================== */

web_cmd_status_t web_cmd_process_request(const web_cmd_ctx_t *ctx,
                                         int value_f, int has_f,
                                         int turn, int has_t,
                                         int32_t now_ms,
                                         int force, int arm_enabled,
                                         float *out_thr)
{
    if (!ctx_ready(ctx) || ctx->inplace_step_end_ms == NULL) {
        return WEB_CMD_ERR_INVALID_ARG;
    }

    /* `_parse_pair(req,'f','t')`：**两个键必须同时存在**，否则整条请求忽略
     * （只发 `t=` 不动，只发 `f=` 也不动）。 */
    if (!has_f || !has_t) {
        return WEB_CMD_OK;
    }

    const float thr = web_cmd_joy_f_to_thr(value_f);
    if (out_thr != NULL) {
        *out_thr = thr;
    }

    /* ---- 原地踏步优先（web_common.py:148~156） ----
     *   _ie = getattr(padog,'inplace_step_end_ms',0) or 0
     *   if _ie and utime.ticks_diff(_ie, utime.ticks_ms()) > 0:
     *       set_leg_sit_offsets(0,0); move(4,1,1); return
     *
     * ⚠️ 原版这一段包在 `try/except Exception: pass` 里，`import utime` 也在 try 内。
     *    在板子上 `import utime` 恒成功（它就在固件里），所以"掉进 except 后继续往下走"
     *    这条路径**不可达** ⇒ C 版直接实现 without-exception 的那条语义，
     *    不复制一个永远不走的异常通道。写在这里是为了说明"为什么少了一层 try"。
     *
     * ⚠️ `move(4, 1, 1)` 用的是 `move`（会 `gait(0)` 回到 TROT），**不是** `drive`；
     *    而且它自己会清掉 `inplace_step_end_ms` ⇒ 原版这个"网页原地步态测试"
     *    **只生效一帧**（P-26）。这里照抄：只发命令，不在这里清那个时刻。 */
    const int32_t ie = ctx->inplace_step_end_ms(ctx->user);
    if (ie != 0 && (int32_t)(ie - now_ms) > 0) {
        ctx->set_leg_sit_offsets(ctx->user, 0.0f, 0.0f);
        ctx->move(ctx->user, 4.0f, 1, 1);
        return WEB_CMD_OK;
    }

    if (thr == 0.0f && iabs(turn) < WEB_CMD_JOY_TURN_DEAD) {
        /* ---- 死区：摇杆回中（web_common.py:157~161） ----
         * 先**永远**清腿部偏置；然后非爬行时才真正停车。
         * 爬行期间不碰 joy_turn / spd/L/R —— 爬行状态机自己在推进。 */
        ctx->set_leg_sit_offsets(ctx->user, 0.0f, 0.0f);
        if (ctx->crawl_phase(ctx->user) == 0) {
            ctx->set_joy_turn(ctx->user, 0.0f);
            ctx->move(ctx->user, 0.0f, 0, 0);
        }
        return WEB_CMD_OK;
    }

    /* ---- 其余：清偏置 + 交给 apply_dog_stick（web_common.py:162~164） ---- */
    ctx->set_leg_sit_offsets(ctx->user, 0.0f, 0.0f);
    return web_cmd_apply_dog_stick(ctx, thr, turn, force, arm_enabled);
}
