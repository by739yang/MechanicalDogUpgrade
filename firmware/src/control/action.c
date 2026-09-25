/**
 * @file    action.c
 * @brief   姿态动画 / 动作层的实现（逐行对齐 padog.py 第 661~859 行 + mainloop 的 881~888）
 *
 * 原实现的结构（行号是 padog.py 的）：
 *
 *   661  def set_leg_sit_offsets(front_y, rear_y)        # 改 chain 的配置
 *   667  def _apply_stand_angles_direct()                # 直写 12 路（髋夹、腿不夹）
 *   683  def _apply_sit_angles_direct()                  # = blend(stand, sit, 1.0)
 *   688  def _pose_blend_ms()                            # 读未定义的全局 -> 恒 900
 *   695  def _stand_pose() /  704 def _sit_pose()        # 两张 12 项姿态表
 *   713  def _apply_pose_blend(p0, p1, t)                # 夹 t、逐路插值、逐路夹限
 *   724  def _pose_anim_begin(p_from, p_to, hold_freeze) # 状态 + move/gait + 清爬行
 *   743  def _pose_anim_step()                           # 时钟 + 插值 + 状态
 *   763  def _wait_pose_anim_done()                      # 阻塞：mainloop + sleep 20
 *   769  def action_stand()
 *   788  def action_sit_direct() / 806 def action_sit()
 *   810  def action_crawl()                             # ← **不搬**，见下
 *   834  def action_wave_direct() / 858 def action_wave()
 *   881  if inplace_step_end_ms: ... move(3,1,1) ...     # mainloop 开头那一小段
 *
 * ## 刻意**没有**搬进来的东西
 *
 * * `action_crawl()`（810~831）—— 它属于**爬行**功能，而爬行状态机
 *   （`_crawl_mainloop_service()` / `_crawl_finish()`）已经整个搬进
 *   `control_chain.c` 了（那里连 `crawl_saved_h`、`CRAWL_*` 常量都齐）。
 *   把 `action_crawl()` 搬到这里，会把"谁拥有 `crawl_phase`"变成两个模块打架。
 *   它是"爬行命令入口"，不是"姿态动画入口"。
 * * `mainloop()` 里 `direct_pose_freeze` 那个提前 return（896）—— 它是**调度**：
 *   "冻结时不要跑运动链"。本层只负责产生/消费这个标志，不在本层 return。
 *
 * ## 三个移植细节
 *
 * 1. `_apply_pose_blend()` 的 pin 字段。原实现遍历 `p0[i][0]` 当通道号用
 *    （`pin, deg` 二元组）。两张表的 pin **恰好是 0..11 的自然顺序**，所以 C 版直接用
 *    "通道序数组"就够了；这一点由 golden 的 12 个通道逐路对照钉住（顺序错了立刻 FAIL）。
 * 2. `_clamp_deg` 的位置。三个写舵机的地方夹限范围**不一样**：
 *    `_apply_pose_blend` 每路都夹；`_apply_stand_angles_direct` **只夹髋**；
 *    `action_wave_direct` 里 4 路 + 3 路都夹，而挥舞的两句用的是 `min/max`。
 *    C 版逐处照抄，没有统一。
 * 3. `int()` 是**向零截断**，不是 `floorf`（P-17）。`height(int(H_goal))` 与
 *    `gait(0)` 里的 `int(in_pit)` 都属于 chain 层的命令语义，本层只把值原样输出。
 */
#include "control/action.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* ==================================================================== */
/*  内部小工具                                                          */
/* ==================================================================== */

/** 复刻 Python `int()`：**向零截断**（不是 `floorf`，见成长手册 P-17） */
static float py_int(float v)
{
    return (float)(int32_t)v;
}

/** `utime.ticks_diff(a, b)`：参考实现的 32 位环绕语义 */
static int32_t ticks_diff(int32_t a, int32_t b)
{
    return (int32_t)(a - b);
}

static void eff_push(action_effects_t *eff, action_eff_kind_t kind,
                     float a0, float a1, float a2, int i0, int i1)
{
    if (eff == NULL) {
        return;
    }
    if (eff->n >= ACTION_EFFECTS_MAX) {
        /* 绝不静默丢弃副作用（P-25）；置位让调用方/测试看得见 */
        eff->overflow = true;
        return;
    }
    action_eff_t *e = &eff->item[eff->n++];
    e->kind = kind;
    e->a0 = a0;
    e->a1 = a1;
    e->a2 = a2;
    e->i0 = i0;
    e->i1 = i1;
}

static void eff_push_crawl_reset(action_effects_t *eff)
{
    eff_push(eff, ACTION_EFF_CRAWL_RESET, 0.0f, 0.0f, 0.0f, 0, 0);
}

static void eff_push_gait(action_effects_t *eff, int mode)
{
    eff_push(eff, ACTION_EFF_GAIT, 0.0f, 0.0f, 0.0f, mode, 0);
}

/**
 * @brief 复刻 `padog.move(spd_, L_, R_)` 的**全部**效果。
 *
 * ```python
 * def move(spd_,L_,R_):
 *     spd=float(spd_);L=L_;R=R_
 *     if (L_ + R_) != 0 and abs(spd_) > 0:
 *       gait(0)
 *       servo_init(0)
 *       direct_pose_freeze = False
 *       inplace_step_end_ms = 0
 * ```
 *
 * 后两句改的是**本层自己的状态**（`direct_pose_freeze` / `inplace_step_end_ms` 归
 * `action_state_t`），所以在这里就地做；前两句改的是别的模块，做成两条效果。
 *
 * ⚠️ `abs(spd_) > 0` 里的 `spd_` 是**形参**（未 float 化），但 `float(x) > 0` 与
 * `x > 0` 在数值上等价，照抄成 `fabsf(spd) > 0.0f`。
 */
static void emit_move(const action_cfg_t *cfg, action_state_t *st,
                      action_effects_t *eff, float spd, int L, int R)
{
    (void)cfg;
    eff_push(eff, ACTION_EFF_MOVE, spd, 0.0f, 0.0f, L, R);
    if ((L + R) != 0 && fabsf(spd) > 0.0f) {
        eff_push_gait(eff, 0);
        eff_push(eff, ACTION_EFF_SERVO_INIT, 0.0f, 0.0f, 0.0f, 0, 0);
        if (st != NULL) {
            st->direct_pose_freeze = false;
            st->inplace_step_end_ms = 0;
        }
    }
}

static void emit_sit_offsets(action_effects_t *eff, float front_y, float rear_y)
{
    eff_push(eff, ACTION_EFF_SIT_OFFSETS, front_y, rear_y, 0.0f, 0, 0);
}

static void emit_height(action_effects_t *eff, float goal)
{
    eff_push(eff, ACTION_EFF_HEIGHT, goal, 0.0f, 0.0f, 0, 0);
}

static void emit_gesture(action_effects_t *eff, float pit, float rol, float x)
{
    eff_push(eff, ACTION_EFF_GESTURE, pit, rol, x, 0, 0);
}

/** 逻辑通道 -> 中位角。通道表见 `servo_map.h`：0~2=腿1、3~5=腿4、6~8=腿2、9~11=腿3 */
static void init_by_channel(const action_cfg_t *cfg, float out[ACTION_CHANNELS])
{
    /* 腿序不是物理顺序：原实现 _stand_pose() 里的 pin 排列就是
     *   0,1,2 -> init_1p,init_1h,init_1s      3,4,5 -> init_4p,init_4h,init_4s
     *   6,7,8 -> init_2p,init_2h,init_2s      9,10,11 -> init_3p,init_3h,init_3s
     * （padog.py:697~700）。下面这 12 行是那张表的直译，不要"整理"成 0..3。 */
    out[0]  = cfg->init[0][0];  out[1]  = cfg->init[0][1];  out[2]  = cfg->init[0][2];
    out[3]  = cfg->init[3][0];  out[4]  = cfg->init[3][1];  out[5]  = cfg->init[3][2];
    out[6]  = cfg->init[1][0];  out[7]  = cfg->init[1][1];  out[8]  = cfg->init[1][2];
    out[9]  = cfg->init[2][0];  out[10] = cfg->init[2][1];  out[11] = cfg->init[2][2];
}

/* ==================================================================== */
/*  配置默认值                                                          */
/* ==================================================================== */

void action_cfg_defaults(action_cfg_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));

    /* config_s.py: init_1p=102 init_1h=84 init_1s=92
     *               init_2p=96  init_2h=91 init_2s=85
     *               init_3p=108 init_3h=78 init_3s=68
     *               init_4p=92  init_4h=98 init_4s=102
     * 腿序 = 腿1..腿4（不是物理顺序），与 control_chain_cfg_t.init 同源同序。 */
    static const float k_init[ACTION_LEGS][ACTION_JOINTS] = {
        { 102.0f, 84.0f, 92.0f },   /* 腿1 左前 */
        {  96.0f, 91.0f, 85.0f },   /* 腿2 右前 */
        { 108.0f, 78.0f, 68.0f },   /* 腿3 右后 */
        {  92.0f, 98.0f, 102.0f },  /* 腿4 左后 */
    };
    memcpy(cfg->init, k_init, sizeof(k_init));

    /* `_sit_pose()`（padog.py:706~709）相对 `_stand_pose()` 的偏移，按**逻辑通道序**：
     *   ch0..2  = init_1p-4,  init_1h-16, init_1s-12
     *   ch3..5  = init_4p,    init_4h-28, init_4s+24
     *   ch6..8  = init_2p+4,  init_2h+16, init_2s+12
     *   ch9..11 = init_3p,    init_3h+28, init_3s-24
     * 注意通道 3~5 是**腿4**、6~8 是**腿2**、9~11 是**腿3** —— 符号组合
     * (`-16/+16`、`-28/+28`、`-12/+12`、`+24/-24`) 就是照这张通道表来的。 */
    static const float k_sit_delta[ACTION_CHANNELS] = {
        -4.0f, -16.0f, -12.0f,   /* 腿1（左前）：整体压低 */
          0.0f, -28.0f, +24.0f,  /* 腿4（左后）：大腿下压、小腿上抬 */
        +4.0f, +16.0f, +12.0f,   /* 腿2（右前）：与腿1 反号 */
          0.0f, +28.0f, -24.0f,  /* 腿3（右后）：与腿4 反号 */
    };
    memcpy(cfg->sit_delta, k_sit_delta, sizeof(k_sit_delta));

    /* `pose_blend_ms` 在原实现里**从未被定义**（config.py / config_s.py / 注入表都没有），
     * 所以 `_pose_blend_ms()` 永远走 except 分支返回 900。在真版命名空间里求值过：
     * `ns['_pose_blend_ms']() == 900`。 */
    cfg->pose_blend_ms = 900;

    /* `action_sit_direct()` 里的 `height(86)`（padog.py:801）字面量 */
    cfg->sit_height = 86.0f;

    /* config_s.py: in_pit=0, in_rol=0, in_y=18（注入表里也有 in_pit/in_rol 的 0，
     * 但**没有 in_y** —— in_y 只来自 config_s.py） */
    cfg->in_pit = 0.0f;
    cfg->in_rol = 0.0f;
    cfg->in_y   = 18.0f;

    /* mainloop 第 885 行 `move(3, 1, 1)` */
    cfg->inplace_spd = 3.0f;
    cfg->inplace_L   = 1;
    cfg->inplace_R   = 1;

    /* `_wait_pose_anim_done()` 里的 `time.sleep_ms(20)` */
    cfg->anim_wait_step_ms = 20;

    /* `action_wave_direct()` 的常量（padog.py:838~854） */
    cfg->wave_arm_delta       = 38.0f;   /* 838~841: ±38 */
    cfg->wave_lift_h_delta    = 34.0f;   /* 843: init_1h - 34 */
    cfg->wave_lift_s_delta    = 42.0f;   /* 844: init_1s - 42 */
    cfg->wave_swing_up        = 20.0f;   /* 850: min(180, lift_h + 20) */
    cfg->wave_swing_down      = 12.0f;   /* 852: max(0,   lift_h - 12) */
    cfg->wave_repeat          = 3;       /* 849: for _ in range(3) */
    cfg->wave_arm_sleep_ms    = 420;     /* 842 */
    cfg->wave_lift_sleep_ms   = 300;     /* 848 */
    cfg->wave_swing_sleep_ms  = 260;     /* 851 / 853 */
    cfg->wave_final_sleep_ms  = 200;     /* 854 */
}

/* ==================================================================== */
/*  纯函数                                                              */
/* ==================================================================== */

float action_clamp_deg(float a)
{
    /* padog.py:383~388。两个 if 原样照抄（对 NaN 的行为与 fminf/fmaxf 不同） */
    if (a > 180.0f) {
        return 180.0f;
    }
    if (a < 0.0f) {
        return 0.0f;
    }
    return a;
}

int32_t action_pose_blend_ms(const action_cfg_t *cfg)
{
    if (cfg == NULL) {
        return 900;   /* 原实现的 except 兜底值 */
    }
    return cfg->pose_blend_ms;
}

void action_stand_pose(const action_cfg_t *cfg, float deg[ACTION_CHANNELS])
{
    if (cfg == NULL || deg == NULL) {
        return;
    }
    init_by_channel(cfg, deg);   /* `_stand_pose()` 就是 init_* 本身 */
}

void action_sit_pose(const action_cfg_t *cfg, float deg[ACTION_CHANNELS])
{
    if (cfg == NULL || deg == NULL) {
        return;
    }
    float base[ACTION_CHANNELS];
    init_by_channel(cfg, base);
    for (int i = 0; i < ACTION_CHANNELS; ++i) {
        /* `_sit_pose()` 是 `float(init_1p - 4)` 这种"中位角 + 整数字面量"，
         * 这里等价成 `base + sit_delta`。 */
        deg[i] = base[i] + cfg->sit_delta[i];
    }
}

void action_apply_pose_blend(const float p0[ACTION_CHANNELS],
                             const float p1[ACTION_CHANNELS], float t,
                             float deg[ACTION_CHANNELS])
{
    if (p0 == NULL || p1 == NULL || deg == NULL) {
        return;
    }
    /* padog.py:713~721：先夹 t，再逐路插值，**每路都过 _clamp_deg** */
    if (t < 0.0f) {
        t = 0.0f;
    }
    if (t > 1.0f) {
        t = 1.0f;
    }
    for (int i = 0; i < ACTION_CHANNELS; ++i) {
        const float a = p0[i] + (p1[i] - p0[i]) * t;
        deg[i] = action_clamp_deg(a);
    }
}

void action_apply_stand_angles_direct(const action_cfg_t *cfg, float deg[ACTION_CHANNELS])
{
    if (cfg == NULL || deg == NULL) {
        return;
    }
    float base[ACTION_CHANNELS];
    init_by_channel(cfg, base);
    /* padog.py:667~680。⚠️ 只有髋过 `_clamp_deg`；大腿/小腿原样写。
     * 通道序：0,1,2,3,...,11 = (1p,1h,1s,4p,4h,4s,2p,2h,2s,3p,3h,3s) */
    for (int i = 0; i < ACTION_CHANNELS; ++i) {
        const bool is_hip = (i % 3) == 0;
        deg[i] = is_hip ? action_clamp_deg(base[i]) : base[i];
    }
}

void action_apply_sit_angles_direct(const action_cfg_t *cfg, float deg[ACTION_CHANNELS])
{
    /* padog.py:683~685 只有一行：`_apply_pose_blend(_stand_pose(), _sit_pose(), 1.0)`
     * ⇒ 坐姿表、**每路都夹**（与 `_apply_stand_angles_direct` 只夹髋不同） */
    float from[ACTION_CHANNELS];
    float to[ACTION_CHANNELS];
    action_stand_pose(cfg, from);
    action_sit_pose(cfg, to);
    action_apply_pose_blend(from, to, 1.0f, deg);
}

void action_effects_clear(action_effects_t *eff)
{
    if (eff == NULL) {
        return;
    }
    eff->n = 0;
    eff->overflow = false;
}

/* ==================================================================== */
/*  状态                                                                */
/* ==================================================================== */

void action_state_init(action_state_t *st)
{
    if (st == NULL) {
        return;
    }
    memset(st, 0, sizeof(*st));
    /* padog.py:176~183 的模块级初值：
     *   inplace_step_end_ms=0  direct_pose_freeze=False  pose_anim_active=False
     *   pose_anim_from=None    pose_anim_to=None         pose_anim_start_ms=0
     *   pose_anim_end_ms=0     pose_anim_hold_freeze=False
     * `pose_anim_from/to` 的 `None` 用全 0 表示；只有 `pose_anim_active` 为 true 时
     * 才会被读（原实现若在 active 状态下读它们是 `None`，那样会直接 TypeError）。 */
}

/* ==================================================================== */
/*  姿态动画                                                            */
/* ==================================================================== */

void action_pose_anim_begin(action_state_t *st, const action_cfg_t *cfg,
                            const float from[ACTION_CHANNELS],
                            const float to[ACTION_CHANNELS],
                            bool hold_freeze, int32_t now_ms,
                            action_effects_t *eff)
{
    if (st == NULL) {
        return;
    }
    /* 728~731：清爬行 + 清原地踏步 */
    eff_push_crawl_reset(eff);
    st->inplace_step_end_ms = 0;
    /* 732~733：move(0,0,0) 与 gait(0)。注意 `move(0,0,0)` 的 L+R==0 ⇒ 它**不会**
     * 联带 gait(0)/servo_init(0)（这与 `action_stand()` 里那两句是分开的两次调用）。 */
    emit_move(cfg, st, eff, 0.0f, 0, 0);
    eff_push_gait(eff, 0);

    if (from != NULL) {
        memcpy(st->pose_anim_from, from, sizeof(st->pose_anim_from));
    }
    if (to != NULL) {
        memcpy(st->pose_anim_to, to, sizeof(st->pose_anim_to));
    }
    st->pose_anim_start_ms = now_ms;
    st->pose_anim_end_ms = now_ms + action_pose_blend_ms(cfg);
    st->pose_anim_hold_freeze = hold_freeze;
    st->pose_anim_active = true;
    st->direct_pose_freeze = true;
}

bool action_pose_anim_step(action_state_t *st, const action_cfg_t *cfg,
                           int32_t now_ms, float deg[ACTION_CHANNELS])
{
    if (st == NULL || !st->pose_anim_active) {
        return false;
    }
    int32_t total = ticks_diff(st->pose_anim_end_ms, st->pose_anim_start_ms);
    if (total <= 0) {
        total = 1;
    }
    const int32_t elapsed = ticks_diff(now_ms, st->pose_anim_start_ms);
    float t = (float)elapsed / (float)total;
    if (t >= 1.0f) {
        t = 1.0f;
        action_apply_pose_blend(st->pose_anim_from, st->pose_anim_to, t, deg);
        st->pose_anim_active = false;
        st->direct_pose_freeze = st->pose_anim_hold_freeze;
    } else {
        action_apply_pose_blend(st->pose_anim_from, st->pose_anim_to, t, deg);
    }
    return true;
}

bool action_wait_pose_anim_done(action_state_t *st, const action_cfg_t *cfg,
                                int32_t now_ms, float deg[ACTION_CHANNELS],
                                int32_t *delay_ms)
{
    if (st == NULL || !st->pose_anim_active) {
        return false;   /* 原实现：`while pose_anim_active` 条件为假，循环体一次都不执行 */
    }
    /* 原循环体：mainloop() —— 在这个状态下 mainloop 只会走到 `_pose_anim_step()`
     * 那一步就 return 0（爬行已清、inplace 已清），所以就地展开成这一次调用。 */
    const bool stepped = action_pose_anim_step(st, cfg, now_ms, deg);
    if (delay_ms != NULL) {
        *delay_ms = (cfg != NULL) ? cfg->anim_wait_step_ms : 20;
    }
    return stepped;
}

/* ==================================================================== */
/*  动作入口                                                            */
/* ==================================================================== */

bool action_stand(const action_cfg_t *cfg, action_state_t *st, float cur_h_goal,
                  int32_t now_ms, float deg[ACTION_CHANNELS],
                  action_effects_t *eff)
{
    if (st == NULL) {
        return false;
    }
    /* 771~772：动画进行中直接 return —— **连爬行都不清、一个副作用都不做** */
    if (st->pose_anim_active) {
        return false;
    }
    /* 773~775：清爬行 */
    eff_push_crawl_reset(eff);
    /* 776~777 */
    emit_move(cfg, st, eff, 0.0f, 0, 0);
    eff_push_gait(eff, 0);
    /* 778：height(int(H_goal))。`int()` 向零截断 */
    emit_height(eff, py_int(cur_h_goal));
    /* 779：gesture(0, 0, in_y)。⚠️ 它在 `gait(0)` **之后**，两者都写 PIT/ROL/X 目标 ——
     * 顺序不能合并（in_pit/in_rol 非 0 时结果不同）。 */
    emit_gesture(eff, 0.0f, 0.0f, (cfg != NULL) ? cfg->in_y : 0.0f);
    /* 780：set_leg_sit_offsets(0, 0) */
    emit_sit_offsets(eff, 0.0f, 0.0f);

    if (st->direct_pose_freeze) {
        /* 781~782：从坐姿动回站姿。⚠️ 这里 `move()`/`gait()` 会被**再调一次**
         * （在 `action_pose_anim_begin` 里）。原实现确实如此；两次调用幂等
         * （`gait(0)` 第二次模式没变、不再归零 t；目标写成同一组值），
         * 所以 C 版也照抄成两条效果，不做"优化掉重复"。
         * ⇒ golden 比对的是**施加完效果表之后的状态**，不是调用序列。 */
        float from[ACTION_CHANNELS];
        float to[ACTION_CHANNELS];
        action_sit_pose(cfg, from);
        action_stand_pose(cfg, to);
        action_pose_anim_begin(st, cfg, from, to, false, now_ms, eff);
        return false;
    }
    /* 783~785：直写站姿 */
    st->direct_pose_freeze = false;
    action_apply_stand_angles_direct(cfg, deg);
    return true;
}

bool action_sit_direct(const action_cfg_t *cfg, action_state_t *st, int32_t now_ms,
                       float deg[ACTION_CHANNELS], action_effects_t *eff)
{
    (void)deg;   /* 这条路径一个舵机都不写；参数只为接口一致 */
    if (st == NULL) {
        return false;
    }
    /* 790~793：两个提前 return，**顺序照抄**（先动画、再冻结） */
    if (st->pose_anim_active) {
        return false;
    }
    if (st->direct_pose_freeze) {
        return false;
    }
    /* 794~797：清爬行 + 清原地踏步 */
    eff_push_crawl_reset(eff);
    st->inplace_step_end_ms = 0;
    /* 798~800 */
    emit_move(cfg, st, eff, 0.0f, 0, 0);
    eff_push_gait(eff, 0);
    emit_sit_offsets(eff, 0.0f, 0.0f);
    /* 801~802：height(86) 与 gesture(0, 0, in_y) —— 注意顺序：height 在前 */
    emit_height(eff, (cfg != NULL) ? cfg->sit_height : 86.0f);
    emit_gesture(eff, 0.0f, 0.0f, (cfg != NULL) ? cfg->in_y : 0.0f);
    /* 803：站姿 -> 坐姿，`hold_freeze=True`（动画做完后保持冻结） */
    {
        float from[ACTION_CHANNELS];
        float to[ACTION_CHANNELS];
        action_stand_pose(cfg, from);
        action_sit_pose(cfg, to);
        action_pose_anim_begin(st, cfg, from, to, true, now_ms, eff);
    }
    return false;
}

bool action_sit(const action_cfg_t *cfg, action_state_t *st, int32_t now_ms,
                float deg[ACTION_CHANNELS], action_effects_t *eff)
{
    /* padog.py:806~807：`action_sit()` 就是 `action_sit_direct()` */
    return action_sit_direct(cfg, st, now_ms, deg, eff);
}

bool action_inplace_step(action_state_t *st, const action_cfg_t *cfg, int32_t now_ms,
                         action_effects_t *eff)
{
    if (st == NULL) {
        return false;
    }
    if (!st->inplace_step_end_ms) {
        return false;   /* `if inplace_step_end_ms:` —— 0 是假 */
    }
    if (ticks_diff(st->inplace_step_end_ms, now_ms) > 0) {
        /* 881~885：还在测试窗口内 —— 每帧重发 TROT 前进 */
        emit_sit_offsets(eff, 0.0f, 0.0f);
        eff_push_gait(eff, 0);
        /* ⚠️ `move(3,1,1)` 会走 `move()` 的 if 分支，于是**自己把
         * `inplace_step_end_ms` 清零**（emit_move 里做的）。⇒ 这个"原地步态测试"
         * 原版只生效一帧。照抄，不"修正"。 */
        emit_move(cfg, st, eff,
                  (cfg != NULL) ? cfg->inplace_spd : 3.0f,
                  (cfg != NULL) ? cfg->inplace_L : 1,
                  (cfg != NULL) ? cfg->inplace_R : 1);
        return true;
    }
    /* 886~888：过期 —— 清截止时刻，只清偏置 */
    st->inplace_step_end_ms = 0;
    emit_sit_offsets(eff, 0.0f, 0.0f);
    return false;
}

/* ==================================================================== */
/*  挥手状态机                                                          */
/* ==================================================================== */

/** 挥手的阶段（与 `action_wave_direct()` 的语句一一对应） */
enum {
    ACTION_WAVE_PHASE_SIT = 0,  /**< `action_sit_direct()` */
    ACTION_WAVE_PHASE_WAIT,     /**< `_wait_pose_anim_done()` 的 46 次循环体 */
    ACTION_WAVE_PHASE_ARM,      /**< 4 路：双腿前抬 ±38 */
    ACTION_WAVE_PHASE_LIFT,     /**< 3 路：左前腿抬起（ch0/1/2） */
    ACTION_WAVE_PHASE_SWING,    /**< 3 × (上摆 + 下摆)，每次只写 ch1 */
    ACTION_WAVE_PHASE_FINAL,    /**< `time.sleep_ms(200)` */
    ACTION_WAVE_PHASE_STAND,    /**< `action_stand()` */
    ACTION_WAVE_PHASE_DONE,     /**< 结束 */
};

void action_wave_init(action_wave_t *w)
{
    if (w == NULL) {
        return;
    }
    w->phase = ACTION_WAVE_PHASE_SIT;
    w->swing = 0;
}

/** `lift_h = _clamp_deg(init_1h - 34)`（padog.py:843）。纯函数，可重复求值 */
static float wave_lift_h(const action_cfg_t *cfg)
{
    return action_clamp_deg(cfg->init[0][1] - cfg->wave_lift_h_delta);
}

/** `lift_s = _clamp_deg(init_1s - 42)`（padog.py:844） */
static float wave_lift_s(const action_cfg_t *cfg)
{
    return action_clamp_deg(cfg->init[0][2] - cfg->wave_lift_s_delta);
}

bool action_wave_step(action_state_t *st, action_wave_t *w, const action_cfg_t *cfg,
                      int32_t now_ms, float cur_h_goal, action_wave_step_t *out)
{
    if (st == NULL || w == NULL || out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    action_effects_clear(&out->eff);

    for (;;) {
        switch (w->phase) {
        case ACTION_WAVE_PHASE_SIT: {
            /* 836：先坐下（纯舵机 -> 其实是启动动画，不写舵机） */
            float dummy[ACTION_CHANNELS];
            action_sit_direct(cfg, st, now_ms, dummy, &out->eff);
            w->phase = ACTION_WAVE_PHASE_WAIT;
            return true;   /* ch_mask=0, delay_ms=0 */
        }
        case ACTION_WAVE_PHASE_WAIT: {
            /* 837：_wait_pose_anim_done() 的一次循环体（mainloop + sleep 20）。
             * 每次写 12 路（mainloop 里的 _pose_anim_step）并推进 anim_wait_step_ms。 */
            int32_t delay = 0;
            if (action_wait_pose_anim_done(st, cfg, now_ms, out->deg, &delay)) {
                out->ch_mask = (1u << ACTION_CHANNELS) - 1u;
                out->delay_ms = delay;
                return true;
            }
            w->phase = ACTION_WAVE_PHASE_ARM;   /* 条件转了假：本步不产生任何输出 */
            break;
        }
        case ACTION_WAVE_PHASE_ARM: {
            /* 838~841：两条前腿的髋/小腿各偏 ±38（都过 _clamp_deg）。
             * 通道 1,2 = 腿1(左前) 髋/小腿；7,8 = 腿2(右前) 髋/小腿。 */
            const float d = cfg->wave_arm_delta;
            out->deg[1] = action_clamp_deg(cfg->init[0][1] - d);
            out->deg[2] = action_clamp_deg(cfg->init[0][2] - d);
            out->deg[7] = action_clamp_deg(cfg->init[1][1] + d);
            out->deg[8] = action_clamp_deg(cfg->init[1][2] + d);
            out->ch_mask = (1u << 1) | (1u << 2) | (1u << 7) | (1u << 8);
            out->delay_ms = cfg->wave_arm_sleep_ms;   /* 842 */
            w->phase = ACTION_WAVE_PHASE_LIFT;
            return true;
        }
        case ACTION_WAVE_PHASE_LIFT: {
            /* 843~847：左前腿抬起（ch0 中位角、ch1 lift_h、ch2 lift_s） */
            out->deg[0] = action_clamp_deg(cfg->init[0][0]);
            out->deg[1] = wave_lift_h(cfg);
            out->deg[2] = wave_lift_s(cfg);
            out->ch_mask = (1u << 0) | (1u << 1) | (1u << 2);
            out->delay_ms = cfg->wave_lift_sleep_ms;   /* 848 */
            w->swing = 0;
            w->phase = ACTION_WAVE_PHASE_SWING;
            return true;
        }
        case ACTION_WAVE_PHASE_SWING: {
            if (w->swing >= 2 * cfg->wave_repeat) {
                w->phase = ACTION_WAVE_PHASE_FINAL;
                break;
            }
            const float lift = wave_lift_h(cfg);
            const bool up = ((w->swing % 2) == 0);
            /* 850 / 852：⚠️ 这里用的是 `min` / `max`，**不是** `_clamp_deg`
             * （对当前的 cfg 数值两者等价，但字面照抄，不"统一"） */
            out->deg[1] = up ? fminf(180.0f, lift + cfg->wave_swing_up)
                             : fmaxf(0.0f, lift - cfg->wave_swing_down);
            out->ch_mask = (1u << 1);
            out->delay_ms = cfg->wave_swing_sleep_ms;   /* 851 / 853 */
            w->swing += 1;
            return true;
        }
        case ACTION_WAVE_PHASE_FINAL: {
            out->delay_ms = cfg->wave_final_sleep_ms;   /* 854 */
            w->phase = ACTION_WAVE_PHASE_STAND;
            return true;   /* ch_mask=0 */
        }
        case ACTION_WAVE_PHASE_STAND: {
            /* 855：action_stand()。此时 direct_pose_freeze=True（动画做完时被
             * `_pose_anim_step` 置成 hold_freeze=True）⇒ 走"坐姿 -> 站姿"动画支，
             * 不写舵机；本动作到此结束（动画留在 active 状态由调用方继续推）。 */
            float dummy[ACTION_CHANNELS];
            action_stand(cfg, st, cur_h_goal, now_ms, dummy, &out->eff);
            w->phase = ACTION_WAVE_PHASE_DONE;
            return true;   /* ch_mask=0, delay_ms=0 */
        }
        case ACTION_WAVE_PHASE_DONE:
        default:
            return false;
        }
    }
}
