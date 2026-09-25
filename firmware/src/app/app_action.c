/**
 * @file    app_action.c
 * @brief   姿态动画 / 动作层的应用层封装实现
 *
 * ## 这一层刻意"薄"
 *
 * 所有**行为**都在被 golden 逐数值验证过的 `control/action.c` 里；本文件只做四件事：
 *
 * 1. 从 `app_config` 建出 `action_cfg_t`（覆盖 action 层真正能给出去的那几个字段）；
 * 2. 把 `action_effects_t` **按顺序**翻译成 `control_chain_cmd_*` 调用
 *    （经 `app_chain` 的窄接口，见 `app_chain.h`）；
 * 3. 给动作层喂时钟（`now_ms`）并持有它的状态（`action_state_t` / `action_wave_t`）；
 * 4. 处理"步进器 + 掩码"这个接口形态，把 12 路拼完整交给运动任务。
 *
 * ## 为什么掩码要用"上一次的输出"补齐
 *
 * 原版 `action_wave_direct()` 里那几句 `angle(1, ...)` 只写**它点名的通道**，
 * 别的通道的 PWM 寄存器**根本没被碰**，于是保持在上一帧写进去的值。C 版的
 * `action_wave_step()` 如实返回 `ch_mask` + `deg`。所以调用方必须把掩码之外的通道
 * 填成"上一帧的输出"—— 不是填 0、也不是填中位角（填错会让没被点名的舵机乱跳，
 * 而 golden 对照对此完全沉默：那边的参照物就是"只有这几路被写"）。
 *
 * 合并用的起点（`s_last`）在 `app_action_init()` 里用动作层自己的**直写站姿**
 * 播种：挥手脚本在第一次写掩码之前必然先写满 12 路（"等动画"那一段每步都写 12 路），
 * 所以这个种子在实际时序里不会被用到，它的作用只是让"没有未定义内存"这件事成立。
 */

#include "app/app_action.h"

#include <string.h>

#include "app/app_chain.h"
#include "app/app_cfg_cmd.h"
#include "control/control_chain_cmd.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "app_action";

/**
 * 网页"步态测试"按钮给 `inplace_step_end_ms` 的窗口（`micropython/web_c.py:398` /
 * `web_common.py:206`：`ticks_add(ticks_ms(), 5000)`）。
 *
 * ⚠️ 这个量原本是**网络命令**写进来的，动作层只提供 `mainloop` 881~888 那一段服务。
 * 本层既然要提供 `action step` 命令，就得把网页那一步补上，否则那个服务永远空转
 * （`if inplace_step_end_ms:` 恒假）。这个 5000 是原版网页里的字面量，不是我编的。
 */
#define APP_ACTION_INPLACE_STEP_MS 5000

/** 一帧内最多连续走几步"不需要等待"的挥手段（掩码 0 且 delay 0），防死循环 */
#define APP_ACTION_WAVE_SAME_FRAME_MAX 16u

/** 12 路全写 */
#define APP_ACTION_MASK_ALL ((1u << ACTION_CHANNELS) - 1u)

/* ==========================================================================
 * 状态
 * ========================================================================== */

static SemaphoreHandle_t s_mutex = NULL;
static bool              s_ready = false;

static action_cfg_t   s_cfg;    /**< 由 `action_cfg_defaults()` + app_config 覆盖而来 */
static action_state_t s_ast;    /**< 动作层的跨帧状态（动画 / 冻结 / 原地踏步截止） */
static action_wave_t  s_wave;   /**< 挥手脚本进度 */

static app_action_kind_t s_kind = APP_ACTION_NONE;
static bool     s_pending       = false;  /**< 有请求还没服务 */
static bool     s_running       = false;  /**< 原地踏步测试持续驱动控制链，直到 stop */
static bool     s_wave_running  = false;  /**< 挥手步进器在跑 */
static int64_t  s_wave_next_ms  = 0;      /**< 下一次可以调用挥手步进器的时刻 */

/** 最近一次输出的 12 路角度（掩码合并用；见文件头） */
static float    s_last[ACTION_CHANNELS];

static uint32_t s_steps    = 0;   /**< 产生了角度的帧数 */
static uint32_t s_effects  = 0;   /**< 施加过的效果条数 */
static uint32_t s_overflow = 0;   /**< 效果表溢出（有副作用被丢）次数 */

/* ==========================================================================
 * 内部工具
 * ========================================================================== */

/** 当前链上的 `H_goal`（原版 `action_stand()` 读的模块级全局） */
static float chain_h_goal(void)
{
    app_chain_status_t cs;
    app_chain_get_status(&cs);
    return cs.goal[CONTROL_CHAIN_GOAL_H];
}

/**
 * @brief 把一步的效果表**按顺序**交给命令层。
 *
 * 顺序不能合并（`action_stand()` 先 `gesture(0,0,in_y)` 再 `gait(0)`，两者都写重心目标），
 * 所以这里就是一个 `for`，不做任何"优化"。
 *
 * 与 `control_chain_cmd_*` 的对应关系：
 * | 效果 | 命令层 |
 * |---|---|
 * | `ACTION_EFF_MOVE` | `control_chain_cmd_move()` ← `app_chain_jog()` |
 * | `ACTION_EFF_GAIT` | `control_chain_cmd_gait()` ← `app_chain_set_gait()` |
 * | `ACTION_EFF_HEIGHT` | `control_chain_cmd_height()` ← `app_chain_set_height()` |
 * | `ACTION_EFF_GESTURE` | `control_chain_cmd_gesture()` ← `app_chain_gesture()` |
 * | `ACTION_EFF_SIT_OFFSETS` | 写 chain **cfg** 的两个偏置 ← `app_chain_set_sit_offsets()` |
 * | `ACTION_EFF_SERVO_INIT` | 写 chain **state** 的 `init_case` ← `app_chain_set_init_case()` |
 * | `ACTION_EFF_CRAWL_RESET` | 清 chain state 的三个爬行量 ← `app_chain_crawl_reset()` |
 */
static void apply_effects(const action_effects_t *eff)
{
    if (eff == NULL) {
        return;
    }
    if (eff->overflow) {
        /* 绝不静默丢弃副作用（P-25）。这个计数会被测试断言为 0 */
        ++s_overflow;
        ESP_LOGE(TAG, "效果表装不下 —— 有跨模块副作用被丢掉了（P-25 那一类错误）");
    }

    for (int i = 0; i < eff->n; ++i) {
        const action_eff_t *e = &eff->item[i];
        esp_err_t err = ESP_OK;

        switch (e->kind) {
        case ACTION_EFF_MOVE:
            err = app_chain_jog(e->a0, e->i0, e->i1);
            break;
        case ACTION_EFF_GAIT:
            err = app_chain_set_gait(e->i0);
            break;
        case ACTION_EFF_HEIGHT:
            err = app_chain_set_height(e->a0);
            break;
        case ACTION_EFF_GESTURE:
            err = app_chain_gesture(e->a0, e->a1, e->a2);
            break;
        case ACTION_EFF_SIT_OFFSETS:
            err = app_chain_set_sit_offsets(e->a0, e->a1);
            break;
        case ACTION_EFF_SERVO_INIT:
            err = app_chain_set_init_case(e->i0);
            break;
        case ACTION_EFF_CRAWL_RESET:
            err = app_chain_crawl_reset();
            break;
        default:
            ESP_LOGE(TAG, "未知效果 kind=%d（新增效果种类后忘了在这里接上？）", (int)e->kind);
            continue;
        }

        if (err != ESP_OK) {
            ESP_LOGW(TAG, "施加效果 kind=%d 失败: %s", (int)e->kind, esp_err_to_name(err));
        }
        ++s_effects;
    }
}

/** 把 `mask` 里的通道取 `deg`，其余通道沿用 `s_last`，结果写进 `dst` 并更新 `s_last` */
static void merge_mask(uint32_t mask, const float deg[ACTION_CHANNELS],
                       float dst[ACTION_CHANNELS])
{
    for (int ch = 0; ch < ACTION_CHANNELS; ++ch) {
        if (mask & (1u << ch)) {
            s_last[ch] = deg[ch];
        }
        dst[ch] = s_last[ch];
    }
}

/* ==========================================================================
 * 爬行入口（`padog.action_crawl()`）
 * ========================================================================== */

/**
 * @brief 复刻 Python `int(float)`：**向零截断**。
 *
 * 不要换成 `floorf`/`truncf` 之外的任何东西，也不要以为"负数要 floor"：
 * `int(-3.5) == -3`、`int(-0.75) == 0`。`golden/action_crawl.csv` 里有这三类样本。
 * （`control_chain_cmd.c` 里有一个同语义的 static `py_int()`；那边是命令层自己的一份。
 *   两处各自 static 是有意的：这是**语言语义**的一行，不是"限幅/常量"那种
 *   必须唯一来源的东西。）
 */
static int32_t action_py_int(float v)
{
    return (int32_t)v;
}

/**
 * @brief 复刻 `padog.action_crawl()`（`padog.py:810~831`）的**入口序列**。
 *
 * `padog.py:815~831` 共 17 行语句，其中 `now = utime.ticks_ms()`（828）不是副作用；
 * 剩下 16 步一条不漏、顺序照抄。按"副作用分组"数正好是 `§0.5(5)` 说的 **14 组**
 * （前三行清零算一组、三个重心目标算一组、两个截止时刻算一组）：
 *
 * | # | 原版 | C 侧 |
 * |---|---|---|
 * | 1 | `pose_anim_active = False` | 本层 state |
 * | 2 | `direct_pose_freeze = False` | 本层 state |
 * | 3 | `inplace_step_end_ms = 0` | 本层 state |
 * | 4 | `set_joy_turn(0)` | `app_chain_set_joy_turn(0)` |
 * | 5 | `move(0, 0, 0)` | `app_chain_jog(0,0,0)` |
 * | 6 | `gait(0)` | `app_chain_set_gait(0)` |
 * | 7 | `servo_init(0)` | `app_chain_set_init_case(0)` |
 * | 8 | `set_leg_sit_offsets(0, 0)` | `app_chain_set_sit_offsets(0,0)` |
 * | 9 | `crawl_saved_h = int(H_goal)` | `app_chain_set_crawl_saved_h(int(H_goal))` |
 * | 10 | `R_H = crawl_saved_h` | `app_chain_set_r_h(saved_h)` ← **不是** `set_height` |
 * | 11~13 | `PIT/ROL/X_goal = int(in_pit/in_rol/in_y)` | `app_chain_gesture(...)` |
 * | 14 | `crawl_settle_until_ms = now + CRAWL_SETTLE_MS` | `app_chain_get_crawl_ms()` |
 * | 15 | `crawl_until_ms = now + CRAWL_SETTLE_MS + CRAWL_DURATION_MS` | 同上 |
 * | 16 | `crawl_phase = 1` | `app_chain_set_crawl(1, until, settle)` |
 *
 * ## 三个必须说清楚的点
 *
 * 1. **`H_goal` 是"调用这一刻"链上的值**（原版读模块级全局）。C 版它住在命令层
 *    `control_chain_cmd_t.goal[CONTROL_CHAIN_GOAL_H]`，所以先读回来再截断。
 * 2. **`app_chain_set_height()` 不能拿来搬第 10 行**：它还写 `H_goal`
 *    （`padog.height()` 是"两个都写"）。原版这里是 `R_H` 单写 ⇒ 一个窄接口。
 * 3. **第 11~13 行与第 6 行的 `gait(0)` 结果相同**（`gait(0)` 里也有同一组
 *    `int(in_*)`）⇒ 原版这三行是冗余的，它们**单独被漏掉也看不出来**。
 *    照抄的理由是"逐行等价"，而不是"它们有独立效果"——这里如实写明，
 *    免得以后有人以为 golden 能测出这三行（它测不出）。
 * 4. ⚠️ **入口落地 ≠ 爬行会跑起来。** 本函数只把 `crawl_phase = 1` 写进
 *    `control_chain_state_t`；而 `app_chain_step()` 目前喂给链的
 *    `in.crawl_phase` **恒为 0**（`app_chain.c` 里那句"P5/P6 再接"），
 *    `control_chain_tick()` 又会把 state 里的值覆盖回去 ⇒ 下一帧 `crawl_phase`
 *    就变回 0，爬行状态机（`chain_crawl_service()`）不会启动。
 *    P5 要接线时改的是**那一行**（把 `s_st.crawl_phase` 喂进去），不是这里。
 *
 * @param now 当前时刻（`utime.ticks_ms()` 语义）——原版自己读时钟，本层不读
 */
static void crawl_entry(int32_t now)
{
    uint32_t applied = 0;

    /* ---- 1~3. 本层自己的三个量（原版直接把 padog 的模块级全局置 False/0） ---- */
    s_ast.pose_anim_active    = false;
    s_ast.direct_pose_freeze  = false;
    s_ast.inplace_step_end_ms = 0;

    /* ---- 4~8. 命令层。顺序不能合并（`action.h` 第 3 条：效果表就是有序的） ---- */
    if (app_chain_set_joy_turn(0.0f) == ESP_OK) { ++applied; }
    if (app_chain_jog(0.0f, 0, 0) == ESP_OK) { ++applied; }
    if (app_chain_set_gait(0) == ESP_OK) { ++applied; }
    if (app_chain_set_init_case(0) == ESP_OK) { ++applied; }
    if (app_chain_set_sit_offsets(0.0f, 0.0f) == ESP_OK) { ++applied; }

    /* ---- 9~10. `crawl_saved_h = int(H_goal)`；`R_H = crawl_saved_h` ---- */
    app_chain_status_t cs;
    app_chain_get_status(&cs);
    const int32_t saved_h = action_py_int(cs.goal[CONTROL_CHAIN_GOAL_H]);
    if (app_chain_set_crawl_saved_h((int)saved_h) == ESP_OK) { ++applied; }
    if (app_chain_set_r_h((float)saved_h) == ESP_OK) { ++applied; }

    /* ---- 11~13. 三个重心目标快照 ----
     * `in_pit/in_rol/in_y` 取**本层 cfg**（与 `action_stand()` 里那个
     * `gesture(0, 0, in_y)` 同一份来源）。它与链 cfg 的那一份同源于 app_config。 */
    if (app_chain_gesture((float)action_py_int(s_cfg.in_pit),
                          (float)action_py_int(s_cfg.in_rol),
                          (float)action_py_int(s_cfg.in_y)) == ESP_OK) { ++applied; }

    /* ---- 14~16. 两个截止时刻 + `crawl_phase = 1` ----
     * 两个时长来自链 cfg（= padog.py:170~171 的 CRAWL_SETTLE_MS / CRAWL_DURATION_MS），
     * 不在这里再抄一份 400/5000。 */
    int32_t settle_ms = 0, duration_ms = 0;
    app_chain_get_crawl_ms(&settle_ms, &duration_ms);
    if (app_chain_set_crawl(1, now + settle_ms + duration_ms, now + settle_ms) == ESP_OK) {
        ++applied;
    }

    /* `effects` 是"副作用真的走了命令层"的证据；这里如实累加（3 条本层状态不算） */
    s_effects += applied;
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

esp_err_t app_action_init(void)
{
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == NULL) {
            ESP_LOGE(TAG, "创建互斥锁失败");
            return ESP_ERR_NO_MEM;
        }
    }

    const app_config_t *c = app_cfg_cmd_get();
    if (c == NULL) {
        ESP_LOGE(TAG, "配置未就绪 —— 必须先调用 app_cfg_cmd_init()");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /*
     * 先铺动作层自己的默认值（= 原版 `padog.py` / `config_s.py` 的实测值），
     * 再用 `app_config` **真正拥有**的字段覆盖。没被覆盖的字段（sit_delta、
     * pose_blend_ms=900、sit_height=86、inplace_*、wave_* 那批字面量）保持
     * `action_cfg_defaults()` 的值 —— 它们在原版里也不是配置项，而是代码里的字面量。
     */
    action_cfg_defaults(&s_cfg);

    /* 覆盖 1：12 个中位角。两边腿序相同（0..3 = 腿1..腿4），所以直接对拷；
     * 动作层内部再按"逻辑通道序"（0~2=腿1、3~5=腿4、6~8=腿2、9~11=腿3）摊开。 */
    for (int leg = 0; leg < ACTION_LEGS; ++leg) {
        for (int j = 0; j < ACTION_JOINTS; ++j) {
            s_cfg.init[leg][j] = c->servo_center[leg][j];
        }
    }

    /* 覆盖 2：三个重心初值（`gesture(0,0,in_y)` 用的是动作层这份 `in_y`；
     * `gait(0)` 用的是链 cfg 那份，两者同源同值） */
    s_cfg.in_pit = c->in_pit;
    s_cfg.in_rol = c->in_rol;
    s_cfg.in_y   = c->in_y;

    action_state_init(&s_ast);
    action_wave_init(&s_wave);

    s_kind          = APP_ACTION_NONE;
    s_pending       = false;
    s_running       = false;
    s_wave_running  = false;
    s_wave_next_ms  = 0;
    s_steps         = 0;
    s_effects       = 0;
    s_overflow      = 0;

    /* 掩码合并的起点：动作层自己的直写站姿（理由见文件头） */
    action_apply_stand_angles_direct(&s_cfg, s_last);

    s_ready = true;
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "动作层就绪：中位角来自 app_config，pose_blend_ms=%d ms, "
                  "sit_height=%.0f, in_pit=%.0f in_rol=%.0f in_y=%.0f",
             (int)s_cfg.pose_blend_ms, (double)s_cfg.sit_height,
             (double)s_cfg.in_pit, (double)s_cfg.in_rol, (double)s_cfg.in_y);
    return ESP_OK;
}

const action_cfg_t *app_action_get_cfg(void)
{
    return s_ready ? &s_cfg : NULL;
}

const char *app_action_kind_name(app_action_kind_t kind)
{
    switch (kind) {
    case APP_ACTION_STAND:        return "stand";
    case APP_ACTION_SIT:          return "sit";
    case APP_ACTION_SIT_DIRECT:   return "sit_direct";
    case APP_ACTION_WAVE:         return "wave";
    case APP_ACTION_INPLACE_STEP: return "step";
    case APP_ACTION_CRAWL:        return "crawl";
    case APP_ACTION_NONE:
    default:                      return "?";
    }
}

/* ==========================================================================
 * 请求 / 取消
 * ========================================================================== */

esp_err_t app_action_request(app_action_kind_t kind)
{
    if (kind == APP_ACTION_NONE) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_mutex == NULL || !s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    s_kind    = kind;
    s_pending = true;
    /* 只有"原地踏步测试"要在窗口结束后继续驱动控制链，别的动作都是"产生角度"型的 */
    s_running = (kind == APP_ACTION_INPLACE_STEP);
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

void app_action_stop(void)
{
    if (s_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    s_pending      = false;
    s_running      = false;
    s_wave_running = false;
    s_wave_next_ms = 0;
    s_kind         = APP_ACTION_NONE;

    /* 姿态动画 / 冻结 / 原地踏步截止时刻一起复位（原版没有"取消"这个动作，
     * 这里给的是"回到没有动作的状态"，不给任何动作留半截状态） */
    action_state_init(&s_ast);
    action_wave_init(&s_wave);

    xSemaphoreGive(s_mutex);

    /* 把"原地踏步测试"下过的行走命令收回来：`action_inplace_step()` 的效果是
     * `move(3,1,1)`，它留在命令层里 ⇒ 取消动作却不收回，下次切回 CHAIN 会突然走起来。
     * L=R=0 的 `move(0,0,0)` 只改 spd/L/R，不动目标与相位（见 control_chain_cmd.h）。 */
    (void)app_chain_jog(0.0f, 0, 0);

    ESP_LOGI(TAG, "动作已取消（姿态动画状态已复位，行走命令已归零）");
}

/* ==========================================================================
 * 一帧
 * ========================================================================== */

/** 服务一次请求。返回 true = 这一步就产生了 12 路角度 */
static bool service_request(int32_t now, float out[ACTION_CHANNELS])
{
    const app_action_kind_t kind = s_kind;
    action_effects_t eff;
    action_effects_clear(&eff);
    bool produced = false;

    switch (kind) {
    case APP_ACTION_STAND: {
        /* `action_stand()` 内部会 `height(int(H_goal))` —— 用的是**调用那一刻**的
         * H_goal，所以必须在施加本次效果之前取（施加之后 goal[0] 会被改成这个值）。 */
        const float h_before = chain_h_goal();
        float deg[ACTION_CHANNELS];
        if (action_stand(&s_cfg, &s_ast, h_before, now, deg, &eff)) {
            /* 直写站姿那一支：12 路全是新值 */
            merge_mask(APP_ACTION_MASK_ALL, deg, out);
            produced = true;
        }
        /* 动画那一支：`action_stand()` 返回 false，动画在下面的步骤 4 里被推进 */
        break;
    }

    case APP_ACTION_SIT:
    case APP_ACTION_SIT_DIRECT: {
        /* `action_sit()` 就是 `action_sit_direct()`：**从不写舵机**，只启动动画。
         * 注意它有两个提前 return（动画中 / 已冻结），那两个分支连效果都不产生。 */
        float deg[ACTION_CHANNELS];
        (void)action_sit_direct(&s_cfg, &s_ast, now, deg, &eff);
        break;
    }

    case APP_ACTION_WAVE:
        /* 挥手的第一步就是 `action_sit_direct()`；步进器在步骤 3 里推 */
        action_wave_init(&s_wave);
        s_wave_running = true;
        s_wave_next_ms = (int64_t)now;   /* 本帧就可以走第一步 */
        break;

    case APP_ACTION_INPLACE_STEP:
        /* 网页那条命令：先把窗口摆上（见 APP_ACTION_INPLACE_STEP_MS 的说明），
         * 下面步骤 2 的服务本帧就会消费掉它。
         * （`s_running` 由 `app_action_request()` 置位 —— 只写一处，见 P-27） */
        s_ast.inplace_step_end_ms = now + APP_ACTION_INPLACE_STEP_MS;
        break;

    case APP_ACTION_CRAWL:
        /* `action_crawl()`（`padog.py:810~831`）：**没有**提前 return（与 stand/sit
         * 不同），而且一个舵机都不写 —— 它只把爬行的入口状态摆好。
         * 爬行状态机本身在 `control_chain.c`，本帧因此**不产生角度**。 */
        crawl_entry(now);
        break;

    case APP_ACTION_NONE:
    default:
        break;
    }

    apply_effects(&eff);
    return produced;
}

/**
 * @brief 推进挥手脚本，直到本帧拿到角度 / 需要等待 / 脚本结束。
 *
 * `action_wave_step()` 的契约是"施加效果 → 只写掩码里的通道 → 等 `delay_ms` 再调"。
 * 掩码为 0 且 `delay_ms` 为 0 的那几步（坐下、动画刚结束那一步、收尾的 stand）
 * 不需要等，本帧连着走完就行，不必白白浪费一帧。
 */
static bool wave_pump(int32_t now, float out[ACTION_CHANNELS])
{
    for (unsigned guard = 0; guard < APP_ACTION_WAVE_SAME_FRAME_MAX; ++guard) {
        if (!s_wave_running) {
            return false;
        }
        if ((int64_t)now < s_wave_next_ms) {
            return false;   /* 还在原版的 time.sleep_ms(...) 里 */
        }

        action_wave_step_t ws;
        /* 原版走到收尾那句 `action_stand()` 时读的是"那一刻"的 H_goal，
         * 所以在**每一步之前**取，而不是在挥手开始时取一次 */
        const float h_now = chain_h_goal();

        if (!action_wave_step(&s_ast, &s_wave, &s_cfg, now, h_now, &ws)) {
            s_wave_running = false;   /* 脚本走完（收尾的站姿动画由步骤 4 继续推） */
            return false;
        }

        apply_effects(&ws.eff);
        s_wave_next_ms = (int64_t)now + (int64_t)ws.delay_ms;

        if (ws.ch_mask != 0) {
            merge_mask(ws.ch_mask, ws.deg, out);
            return true;
        }
        if (ws.delay_ms > 0) {
            return false;   /* 本步不写舵机，下一个动作要等 delay_ms */
        }
        /* 掩码 0、不用等：继续下一步 */
    }

    ESP_LOGW(TAG, "挥手步进器在同一帧内连续走了 %u 步仍未产生输出，本帧放弃",
             (unsigned)APP_ACTION_WAVE_SAME_FRAME_MAX);
    return false;
}

bool app_action_step(int64_t now_ms, float out[ACTION_CHANNELS])
{
    if (out == NULL || s_mutex == NULL) {
        return false;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return false;
    }
    if (!s_ready) {
        xSemaphoreGive(s_mutex);
        return false;
    }

    const int32_t now = (int32_t)now_ms;
    float  tmp[ACTION_CHANNELS];
    bool   produced = false;

    /* ---- 1. 服务请求（一次就够；行为全在 action 层） ---- */
    if (s_pending) {
        s_pending = false;
        produced = service_request(now, tmp);
    }

    /* ---- 2. `mainloop` 881~888 的原地踏步服务（归动作层，见 action.h 第 4 条） ---- */
    bool drive_chain = false;
    if (s_running && s_kind == APP_ACTION_INPLACE_STEP) {
        action_effects_t eff;
        action_effects_clear(&eff);
        (void)action_inplace_step(&s_ast, &s_cfg, now, &eff);
        apply_effects(&eff);
        /* ⚠️ 两种情况都要继续驱动控制链：
         *   - 本帧还在测试窗口内：`action.h` 明写"调用方据此知道本帧要跑 TROT"；
         *   - 窗口过了（原版第一帧里 `move(3,1,1)` 就把它清掉了）：那条行走命令
         *     已经留在命令层，原版 mainloop 之后每帧照样按 spd/L/R 跑 TROT。
         * 直到 `action stop` / `estop` 为止。 */
        drive_chain = true;
    }

    /* ---- 3. 挥手步进器 ---- */
    if (!produced && s_wave_running) {
        produced = wave_pump(now, tmp);
    }

    /* ---- 4. 姿态动画（时钟驱动；`_pose_anim_step()` 写满 12 路） ---- */
    if (!produced && s_ast.pose_anim_active) {
        float deg[ACTION_CHANNELS];
        if (action_pose_anim_step(&s_ast, &s_cfg, now, deg)) {
            merge_mask(APP_ACTION_MASK_ALL, deg, tmp);
            produced = true;
        }
    }

    /* ---- 5. 原地踏步：本帧真的跑一次控制链（它自己也按 65 ms 节拍门控） ---- */
    if (!produced && drive_chain) {
        (void)app_chain_step(now_ms, tmp);   /* 没到节拍时它还回上一帧的 12 路 */
        produced = true;
    }

    if (produced) {
        memcpy(out, tmp, sizeof(float) * ACTION_CHANNELS);
        ++s_steps;
    }

    xSemaphoreGive(s_mutex);
    return produced;
}

/* ==========================================================================
 * `inplace_step_end_ms` 的读写（见 app_action.h）
 * ========================================================================== */

int32_t app_action_get_inplace_step_end_ms(void)
{
    int32_t v = 0;
    if (s_mutex == NULL) {
        return 0;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return 0;
    }
    v = s_ast.inplace_step_end_ms;
    xSemaphoreGive(s_mutex);
    return v;
}

esp_err_t app_action_set_inplace_step_end_ms(int32_t end_ms)
{
    if (s_mutex == NULL || !s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    s_ast.inplace_step_end_ms = end_ms;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

void app_action_get_status(app_action_status_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (s_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    out->kind             = s_kind;
    out->pending          = s_pending;
    out->wave_running     = s_wave_running;
    out->pose_anim_active = s_ast.pose_anim_active;
    out->direct_pose_freeze = s_ast.direct_pose_freeze;
    out->wave_phase       = s_wave.phase;
    out->wave_swing       = s_wave.swing;
    out->steps            = s_steps;
    out->effects          = s_effects;
    out->overflow         = s_overflow;
    out->busy             = s_pending || s_wave_running || s_ast.pose_anim_active ||
                            (s_running && s_kind == APP_ACTION_INPLACE_STEP);

    xSemaphoreGive(s_mutex);
}
