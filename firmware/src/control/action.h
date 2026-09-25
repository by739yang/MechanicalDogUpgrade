/**
 * @file    action.h
 * @brief   姿态动画 / 动作层：`padog.py` 第 661~859 行那一整段（纯 C，零 ESP-IDF 依赖）
 *
 * ## 它复刻的是哪一段原代码
 *
 * `micropython/padog.py` 里从 `set_leg_sit_offsets()` 到 `action_wave()` 的那一段
 * （含中间那批 `_pose_*` 小函数）。逐个列在下面，**并注明每个函数靠什么机制起作用**
 * —— 因为这一层里有三种完全不同的机制混在一起，是最容易搬错的地方：
 *
 * | 原函数 | 行 | 机制 |
 * |---|---|---|
 * | `set_leg_sit_offsets(front, rear)` | 661 | **改别的模块的状态**（`front_leg_y_offset` / `rear_leg_y_offset` 被 `_foot_y_targets()` 读）⇒ 本模块做成显式输出 |
 * | `_apply_stand_angles_direct()` | 667 | **直接写 12 路舵机**（`PA_SERVO.angle`）；髋夹 `_clamp_deg`，大腿/小腿**不夹** |
 * | `_apply_sit_angles_direct()` | 683 | 表插值到 t=1.0（等价于直接写坐姿），**每路都夹** |
 * | `_pose_blend_ms()` | 688 | 纯函数；读模块级 `pose_blend_ms` |
 * | `_stand_pose()` / `_sit_pose()` | 695 / 704 | 纯函数：两张 12 项姿态表（逻辑通道 0..11） |
 * | `_apply_pose_blend(p0, p1, t)` | 713 | 纯数学 + **直接写 12 路舵机**（每路都夹 `_clamp_deg`） |
 * | `_pose_anim_begin(p_from,p_to,hold)` | 724 | 状态 + **改 chain 的命令状态**（`move` / `gait`）+ 清爬行/原地踏步 |
 * | `_pose_anim_step()` | 743 | **读时钟**（`utime.ticks_ms`）+ 插值写舵机 + 状态 |
 * | `_wait_pose_anim_done()` | 763 | **阻塞循环：反复调 `mainloop()` + `time.sleep_ms(20)`** ⇒ 不能照抄 |
 * | `action_stand()` | 769 | 命令副作用 + 二选一（动画 / 直写舵机） |
 * | `action_sit_direct()` | 788 | 命令副作用 + 启动 stand→sit 动画（`hold_freeze=True`） |
 * | `action_sit()` | 806 | 一行别名 → `action_sit_direct()` |
 * | `action_wave_direct()` | 834 | **阻塞脚本**：坐下 → 等动画 → 若干次直写舵机 + `time.sleep_ms` → `action_stand()` |
 * | `action_wave()` | 858 | 一行别名 → `action_wave_direct()` |
 * | `inplace_step_end_ms` 服务 | 881~888 | `mainloop` 开头的一小段（`move(3,1,1)`）⇒ 归本层，见下 |
 *
 * ## 四条移植决定（都有理由，别"顺手改回去"）
 *
 * 1. **时间必须是显式入参。** 原实现三处读 `utime.ticks_ms()`（`_pose_anim_begin`、
 *    `_pose_anim_step`、`action_crawl`），本模块**一次时钟都不读**：所有涉及时间的
 *    入口都多一个 `now_ms` 参数（`utime.ticks_ms()` 语义）。这样宿主测试可以拿假时钟
 *    逐毫秒对照，真机上由调用方传 `esp_timer_get_time()/1000`。
 *
 * 2. **不写寄存器。** 原实现通过 `PA_SERVO.angle(logical_ch, deg)` 直接驱动舵机。
 *    本模块只把 **12 路角度按逻辑通道 0..11 输出**（与 `servo_map` / `control_chain`
 *    一致），`servo_map.c` 再编码成 PCA9685 的 (ON, OFF)。
 *
 * 3. **凡是"改别的模块状态"的，一律做成显式输出，既不静默丢弃、也不静默施加。**
 *    这一层里有一大把：`move()` / `gait()` / `height()` / `gesture()` /
 *    `servo_init()` 改的是命令层的 `spd/L/R/gait_mode/四个目标/init_case`，爬行三个量
 *    改的是 chain 的 state，`set_leg_sit_offsets()` 改的是 chain 的**配置**
 *    （`front_leg_y_offset` / `rear_leg_y_offset`）。
 *
 *    输出形态是**有序的效果表** `action_effects_t`（不是"合并后的位掩码"）：
 *    `action_stand()` 在动画分支里会**先 `gesture(0,0,in_y)` 再 `gait(0)`**，两者
 *    都写 PIT/ROL/X 目标，**先后顺序决定最终值**（`in_pit`/`in_rol` 非 0 时结果不同）。
 *    所以效果必须按顺序交回调用方，由调用方用 `control_chain_cmd_*` 依次施加。
 *    成长手册 **P-25** 就是"跨模块副作用被吞掉"的教训。
 *
 *    ⚠️ `set_leg_sit_offsets()` 没有对应的命令层函数：它是**写 chain 的配置**。
 *    调用方需要一个**可写的** `control_chain_cfg_t` 副本，收到 `ACTION_EFF_SIT_OFFSETS`
 *    时改那两个字段。（`control_chain.c` 里那句注释"所有调用点传的都是 0，所以放进
 *    只读 cfg 就够"在本层被打破了 —— 本层现在如实输出这两个值。）
 *
 * 4. **`inplace_step_end_ms` 归本层。** `control_chain.h` 已经把 `inplace_step_end_ms`
 *    与 `pose_anim_active` / `direct_pose_freeze` 一起划给"姿态动画层"，
 *    `control_chain_tick()` 明确不包含它。本模块持有这个截止时刻，并提供
 *    `action_inplace_step()` 复刻 mainloop 第 881~888 行那一小段。
 *
 *    ⚠️ **照抄出来的一个原实现事实**：活跃分支里那句 `move(3, 1, 1)` 会走
 *    `move()` 的 `if (L_+R_) != 0 and abs(spd_) > 0` 分支，而那个分支里有
 *    `inplace_step_end_ms = 0`。也就是说 **"网页原地步态测试"在原版里只生效一帧**：
 *    第一帧把它清掉，第二帧 `if inplace_step_end_ms:` 就是假了。`action_inplace_step()`
 *    照抄这个行为（`emit_move()` 的 if 分支里同样清零），并在 golden 里用**真的
 *    `mainloop()`** 钉住它。**没有"修正"它** —— 那是行为变更，不是移植。
 *
 * ## 可测试性
 *
 * 零 ESP-IDF、零时钟、零寄存器。宿主 gcc 直接编译，与"真版 `padog.py` 真跑一次"
 * 的输出对照（`tools/golden/test_action.c`，**容差 0**）。参考值的时钟由
 * `mpy_stubs.install_controllable_clock()` 钉死，见 `gen_golden.py` 的说明。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "control/servo_map.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 逻辑通道数（= 舵机路数）。通道 0..11 的含义见 `servo_map.h` 那张表 */
#define ACTION_CHANNELS SERVO_MAP_CHANNELS
/** 腿数（原实现编号 1..4） */
#define ACTION_LEGS     SERVO_MAP_LEGS
/** 每腿关节数（髋 / 大腿 / 小腿） */
#define ACTION_JOINTS   SERVO_MAP_JOINTS

/** `action_effects_t` 能装下的效果条数（最长的一条是 `action_stand()` 的动画分支 = 9 条） */
#define ACTION_EFFECTS_MAX 16

/* ==================================================================== */
/*  配置                                                                */
/* ==================================================================== */

/**
 * @brief 姿态 / 动作层的全部参数。
 *
 * 字段名尽量保留原 Python 的拼写；注释里的"来源"标明这个值在**这台机器**上从哪来。
 */
typedef struct {
    /* ---------------- 来自 config_s.py ---------------- */

    /** 中位角 `[腿][关节]`，腿 0..3 = 腿1..腿4，关节 0=髋 1=大腿 2=小腿。
     *  config_s.py: `init_1p=102 / init_1h=84 / init_1s=92 / init_2p=96 / init_2h=91 /
     *  init_2s=85 / init_3p=108 / init_3h=78 / init_3s=68 / init_4p=92 / init_4h=98 /
     *  init_4s=102`。与 `control_chain_cfg_t.init` 同源同序。 */
    float init[ACTION_LEGS][ACTION_JOINTS];

    /* ---------------- 原实现里的字面量 ---------------- */

    /** `_sit_pose()` 相对 `_stand_pose()` 的 12 个偏移（**逻辑通道序** 0..11）。
     *  来源 `padog.py:706~709` 那一串 `init_1p - 4` / `init_4h - 28` …
     *  ＝ `{-4,-16,-12, 0,-28,+24, +4,+16,+12, 0,+28,-24}`。
     *  原注释写着"数值可按实机微调"（`padog.py:684`），所以做成配置项。 */
    float sit_delta[ACTION_CHANNELS];

    /** `_pose_blend_ms()` 的返回值（ms）。⚠️ `pose_blend_ms` 这个全局**在整份原代码里
     *  没有任何地方定义过**（config.py / config_s.py / padog.py 的默认值注入表里都没有），
     *  所以 `int(pose_blend_ms)` 永远 `NameError`，`_pose_blend_ms()` 永远走
     *  `except Exception: return 900` —— **那个 900 才是实机生效值**。
     *  （这里不是 P-19 那种死代码：兜底分支正是唯一被走到的分支。判断依据是在真版
     *  padog 命名空间里求值 `_pose_blend_ms() == 900`，不是在源码里搜。） */
    int32_t pose_blend_ms;

    /** `action_sit_direct()` 里的 `height(86)`（`padog.py:801`）。坐下时的站高目标 */
    float sit_height;

    /** `gait(0)` 复位重心目标用的三个 config 值。config_s.py: `in_pit=0 in_rol=0 in_y=18`。
     *  ⚠️ `action_stand()` 里的 `gesture(0, 0, in_y)` 与随后的 `gait(0)` 都写
     *  PIT/ROL/X 目标，**前者用 0/0，后者用 int(in_pit)/int(in_rol)/int(in_y)** ——
     *  所以效果表的顺序不能合并。 */
    float in_pit;
    float in_rol;
    float in_y;

    /* ---------------- `inplace_step_end_ms` 服务（mainloop 第 881~888 行） ---------------- */

    /** `move(3, 1, 1)` 里那个 3（`padog.py:885`）。注释说"与前进同 TROT" */
    float inplace_spd;
    /** `move(3, 1, 1)` 的两个相位系数 */
    int inplace_L;
    int inplace_R;

    /* ---------------- `_wait_pose_anim_done()` / `action_wave_direct()` ---------------- */

    /** `_wait_pose_anim_done()` 每次循环末尾的 `time.sleep_ms(20)`（`padog.py:766`） */
    int32_t anim_wait_step_ms;

    /** 挥手第一段：两条前腿的髋/小腿各偏 38°（`padog.py:838~841`） */
    float wave_arm_delta;
    /** 挥手第二段：左前腿抬起的髋偏移（`padog.py:843`，`init_1h - 34`） */
    float wave_lift_h_delta;
    /** 挥手第二段：左前腿抬起的小腿偏移（`padog.py:844`，`init_1s - 42`） */
    float wave_lift_s_delta;
    /** 三段挥舞的"上摆"增量（`padog.py:850`，`lift_h + 20`） */
    float wave_swing_up;
    /** 三段挥舞的"下摆"增量（`padog.py:852`，`lift_h - 12`） */
    float wave_swing_down;
    /** 挥舞次数（`padog.py:849`，`for _ in range(3)`） */
    int wave_repeat;
    /** 第一段之后的 `time.sleep_ms(420)`（`padog.py:842`） */
    int32_t wave_arm_sleep_ms;
    /** 第二段之后的 `time.sleep_ms(300)`（`padog.py:848`） */
    int32_t wave_lift_sleep_ms;
    /** 每次挥舞之后的 `time.sleep_ms(260)`（`padog.py:851` 与 `853`，两次同值） */
    int32_t wave_swing_sleep_ms;
    /** 收尾的 `time.sleep_ms(200)`（`padog.py:854`） */
    int32_t wave_final_sleep_ms;
} action_cfg_t;

/**
 * @brief 填入**这台机器**的实际生效值。
 *
 * 与 `control_chain_cfg_defaults()` 同一套取法：`config_s.py` 里有的用它，
 * 没有的用 `padog.py` 第 57~81 行默认值注入表的兜底值，再没有的用原实现的字面量。
 *
 * `init` 与 `action_cfg_t` 的其余字段的来源逐条写在结构体注释里。
 *
 * @param cfg 目标（为 NULL 时不做任何事）
 */
void action_cfg_defaults(action_cfg_t *cfg);

/* ==================================================================== */
/*  纯函数（原实现里那几个"算角度"的小函数）                              */
/* ==================================================================== */

/** 复刻 `padog._clamp_deg(a)`：`a>180 -> 180`，`a<0 -> 0`，否则原样。
 *  ⚠️ 两个 `if`，不是 `fminf/fmaxf` —— 对 NaN 的行为一样（原样返回 NaN），但不改。 */
float action_clamp_deg(float a);

/** 复刻 `padog._pose_blend_ms()`。`pose_blend_ms` 在原实现里没定义 ⇒ 恒返回 900 */
int32_t action_pose_blend_ms(const action_cfg_t *cfg);

/**
 * @brief 复刻 `padog._stand_pose()`：把 `init` 按**逻辑通道序**摊成 12 个角度。
 *
 * 原实现返回的是 `((pin, deg), ...)` 12 元组；两张表的 `pin` 都恰好是 `0..11` 的
 * 自然顺序（`(0, init_1p), (1, init_1h), (2, init_1s), (3, init_4p), ...`），
 * 所以按通道序的数组与之等价，`_apply_pose_blend()` 里 `p0[i][0]` 那个 pin 就是 `i`。
 *
 * 腿序**不是**物理顺序：通道 0~2 = 腿1(左前)、3~5 = 腿4(左后)、6~8 = 腿2(右前)、
 * 9~11 = 腿3(右后)。见 `servo_map.h` 的表。
 */
void action_stand_pose(const action_cfg_t *cfg, float deg[ACTION_CHANNELS]);

/** 复刻 `padog._sit_pose()`：`stand_pose[ch] + sit_delta[ch]`（不做夹限） */
void action_sit_pose(const action_cfg_t *cfg, float deg[ACTION_CHANNELS]);

/**
 * @brief 复刻 `padog._apply_pose_blend(p0, p1, t)`：`t` 夹到 `[0,1]` 后逐路线性插值，
 *        **每路都过 `_clamp_deg`**，输出 12 个舵机角。
 *
 * `t` 的夹限在原实现里是 `if t < 0.0: t = 0.0` / `if t > 1.0: t = 1.0`（NaN 会
 * 原样穿过去，与 `_clamp_deg` 同风格）。这里照抄。
 */
void action_apply_pose_blend(const float p0[ACTION_CHANNELS],
                             const float p1[ACTION_CHANNELS], float t,
                             float deg[ACTION_CHANNELS]);

/**
 * @brief 复刻 `padog._apply_stand_angles_direct()`：12 路直接等于中位角。
 *
 * ⚠️ **只有髋（通道 0/3/6/9）过 `_clamp_deg`，大腿与小腿（通道 1,2,4,5,7,8,10,11）
 * 原样写**。原注释说明了理由："标定站立：12 路直接等于 init_*，不叠加髋姿态耦合
 * （避免「站立变矮」）"。别"顺手"给大腿也加上夹限 —— 那是行为变更。
 */
void action_apply_stand_angles_direct(const action_cfg_t *cfg, float deg[ACTION_CHANNELS]);

/** 复刻 `padog._apply_sit_angles_direct()`：等价于 `_apply_pose_blend(stand, sit, 1.0)`，
 *  也就是**坐姿表、每路都夹**（与原函数只写三行不同，本函数是对那一行的诚实展开）。 */
void action_apply_sit_angles_direct(const action_cfg_t *cfg, float deg[ACTION_CHANNELS]);

/* ==================================================================== */
/*  跨模块副作用：显式输出，由调用方按顺序施加                             */
/* ==================================================================== */

/** 效果的种类 —— 每一项对应原实现里一次"改别的模块状态"的调用 */
typedef enum {
    /** `move(spd, L, R)`（`padog.py:625`）：改命令层的 `spd/L/R`；若
     *  `(L+R)!=0 && |spd|>0` 还会联带 `gait(0)` + `servo_init(0)`（**本层会拆成三条**）。 */
    ACTION_EFF_MOVE = 0,
    /** `gait(mode)`（`padog.py:642`）：改 `gait_mode`；mode==0 时把三个重心目标重置成
     *  `int(in_pit)/int(in_rol)/int(in_y)`（**截断**）。模式变了还会把 `t` 归零
     *  —— 那属于 chain 的 state，`control_chain_cmd_gait()` 会处理。 */
    ACTION_EFF_GAIT,
    /** `height(goal)`（`padog.py:558`）：**同时**把 `H_goal` 与 `R_H` 写成 goal（绕过 slew） */
    ACTION_EFF_HEIGHT,
    /** `gesture(pit, rol, x)`（`padog.py:564`）：直接覆写三个重心目标 */
    ACTION_EFF_GESTURE,
    /** `set_leg_sit_offsets(front, rear)`（`padog.py:661`）：改的是 chain 的**配置**
     *  `front_leg_y_offset` / `rear_leg_y_offset`，被 `_foot_y_targets()` 读走。 */
    ACTION_EFF_SIT_OFFSETS,
    /** `servo_init(key)`（`padog.py:638`）：改 chain 的 `init_case`
     *  （`servo_output()` 的第二个实参，决定走 IK 还是直接站姿） */
    ACTION_EFF_SERVO_INIT,
    /** 清爬行状态机（`padog.py:728~730` / `773~775` / `794~796`）：
     *  `crawl_phase = crawl_until_ms = crawl_settle_until_ms = 0` —— 改的是 chain 的 state */
    ACTION_EFF_CRAWL_RESET,
} action_eff_kind_t;

/** 一条效果：`kind` + 实参。用不上的字段恒为 0 */
typedef struct {
    action_eff_kind_t kind;
    /** `MOVE`: spd | `HEIGHT`: goal | `GESTURE`: pit | `SIT_OFFSETS`: front */
    float a0;
    /** `GESTURE`: rol | `SIT_OFFSETS`: rear */
    float a1;
    /** `GESTURE`: x */
    float a2;
    /** `MOVE`: L | `GAIT`: mode | `SERVO_INIT`: key */
    int i0;
    /** `MOVE`: R */
    int i1;
} action_eff_t;

/** 一次动作产生的**有序**效果表。顺序有意义（见文件头第 3 条决定） */
typedef struct {
    int n;
    /** 装不下时置位。`ACTION_EFFECTS_MAX` 按最长的一条（9 条）留了余量，
     *  **它一旦为 true 就说明有副作用被丢掉了** —— 那是 P-25 那一类错误，
     *  测试里对每个用例都断言它为 false。 */
    bool overflow;
    action_eff_t item[ACTION_EFFECTS_MAX];
} action_effects_t;

/** 把效果表清空（每次调用动作函数之前先清） */
void action_effects_clear(action_effects_t *eff);

/* ==================================================================== */
/*  状态 —— 对应 padog 的模块级可变全局（只包含本层拥有的那些）           */
/* ==================================================================== */

/**
 * @brief 本层跨帧保留的量。
 *
 * | 字段 | 原实现 | 谁改它 |
 * |---|---|---|
 * | `direct_pose_freeze` | `padog.py:177` | `_pose_anim_begin`（置 true）/ `_pose_anim_step`（置 hold）/ `action_stand`、`action_sit_direct`（置 false）/ `move()`、`drive()` |
 * | `pose_anim_active` | 178 | `_pose_anim_begin` / `_pose_anim_step` |
 * | `pose_anim_from` / `_to` | 179 / 180 | `_pose_anim_begin`（初值是 `None`） |
 * | `pose_anim_start_ms` / `_end_ms` | 181 / 182 | `_pose_anim_begin` |
 * | `pose_anim_hold_freeze` | 183 | `_pose_anim_begin` / `_pose_anim_step`（读） |
 * | `inplace_step_end_ms` | 176 | 见 `action_inplace_step()`；网络命令写入，`move()`/`drive()`/`action_*` 清零 |
 *
 * `crawl_*` 不在这里 —— 它们归 `control_chain_state_t`，本层只通过
 * `ACTION_EFF_CRAWL_RESET` 影响它们。
 */
typedef struct {
    bool direct_pose_freeze;
    bool pose_anim_active;
    bool pose_anim_hold_freeze;
    /** `pose_anim_from` / `pose_anim_to`。原实现初值是 `None`；本结构用全 0 表示
     *  "没有姿态表"，但**只有 `pose_anim_active` 为 true 时才会被读**。 */
    float pose_anim_from[ACTION_CHANNELS];
    float pose_anim_to[ACTION_CHANNELS];
    int32_t pose_anim_start_ms;
    int32_t pose_anim_end_ms;
    int32_t inplace_step_end_ms;
} action_state_t;

/**
 * @brief 按 `padog.py` 第 176~183 行的**模块级初值**初始化状态。
 *
 * `inplace_step_end_ms=0; direct_pose_freeze=False; pose_anim_active=False;
 * pose_anim_from=None; pose_anim_to=None; pose_anim_start_ms=0;
 * pose_anim_end_ms=0; pose_anim_hold_freeze=False`
 *
 * @param st 目标（为 NULL 时不做任何事）
 */
void action_state_init(action_state_t *st);

/* ==================================================================== */
/*  姿态动画                                                            */
/* ==================================================================== */

/**
 * @brief 复刻 `padog._pose_anim_begin(p_from, p_to, hold_freeze)`。
 *
 * 原实现做的事（顺序照抄）：
 * ```python
 * crawl_phase = crawl_until_ms = crawl_settle_until_ms = 0   # 副作用
 * inplace_step_end_ms = 0                                     # 本层自己的状态
 * move(0, 0, 0)                                               # 副作用
 * gait(0)                                                     # 副作用（含目标重置）
 * pose_anim_from/to = p_*; start = ticks_ms(); end = start + _pose_blend_ms()
 * pose_anim_hold_freeze = bool(hold_freeze)
 * pose_anim_active = True
 * direct_pose_freeze = True
 * ```
 *
 * @param now_ms `utime.ticks_ms()` 语义的当前时刻（原实现自己读时钟，本模块不读）
 * @param eff    输出：按顺序记录上面那几条副作用（调用方负责施加）
 */
void action_pose_anim_begin(action_state_t *st, const action_cfg_t *cfg,
                            const float from[ACTION_CHANNELS],
                            const float to[ACTION_CHANNELS],
                            bool hold_freeze, int32_t now_ms,
                            action_effects_t *eff);

/**
 * @brief 复刻 `padog._pose_anim_step()` 一次调用。
 *
 * ```python
 * if not pose_anim_active: return False
 * total = ticks_diff(end, start);  if total <= 0: total = 1
 * t = ticks_diff(now, start) / total
 * if t >= 1.0: t = 1.0; blend(...); active = False; direct_pose_freeze = hold_freeze
 * else:        blend(...)
 * return True
 * ```
 *
 * @param deg 输出：**当且仅当返回 true 时**写入 12 路舵机角（原实现是"算出来就写"，
 *            返回 false 时它一个寄存器都没碰）
 * @return 原函数的返回值：true = 本帧确实走了一次动画（`mainloop` 会据此提前 return）
 */
bool action_pose_anim_step(action_state_t *st, const action_cfg_t *cfg,
                           int32_t now_ms, float deg[ACTION_CHANNELS]);

/**
 * @brief 复刻 `padog._wait_pose_anim_done()` 的**一次循环体**。
 *
 * ⚠️ 原实现是个阻塞循环，**照抄不行**：
 * ```python
 * def _wait_pose_anim_done():
 *     while pose_anim_active:
 *         mainloop()          # ← 宿主上没有 mainloop，真机上也会和任务周期打架
 *         time.sleep_ms(20)   # ← 阻塞
 * ```
 *
 * 关键在于：**这个状态下 `mainloop()` 其实只做一件事** —— 第 889 行
 * `if _pose_anim_step(): ... return 0`。爬行刚被 `action_sit_direct()` 清掉、
 * `inplace_step_end_ms` 也被清零，所以 mainloop 前面的几步都是空的。
 * 于是本函数把那次 `mainloop()` 就地展开成 `action_pose_anim_step()`，
 * 把 `time.sleep_ms(20)` 变成输出参数 `delay_ms` 交给调用方推进时钟。
 *
 * 等价调用形态：
 * ```c
 * int32_t delay;
 * while (action_wait_pose_anim_done(&st, &cfg, now, deg, &delay)) {
 *     apply_angles(deg);        // 原 mainloop 里的 _apply_pose_blend
 *     now += delay;             // 原 time.sleep_ms(20)
 * }
 * ```
 *
 * @param delay_ms 输出：原实现在这一步之后会 `sleep` 多久（`cfg->anim_wait_step_ms`）
 * @return true = 执行了一次循环体（`deg` 有效）；false = 循环条件已假（什么都没做）
 */
bool action_wait_pose_anim_done(action_state_t *st, const action_cfg_t *cfg,
                                int32_t now_ms, float deg[ACTION_CHANNELS],
                                int32_t *delay_ms);

/* ==================================================================== */
/*  动作入口                                                            */
/* ==================================================================== */

/**
 * @brief 复刻 `padog.action_stand()`。
 *
 * ```python
 * if pose_anim_active: return                    # ← 一个副作用都不做
 * crawl 清零; move(0,0,0); gait(0); height(int(H_goal)); gesture(0,0,in_y)
 * set_leg_sit_offsets(0, 0)
 * if direct_pose_freeze: _pose_anim_begin(_sit_pose(), _stand_pose(), False)
 * else:                  direct_pose_freeze = False; _apply_stand_angles_direct()
 * ```
 *
 * @param cur_h_goal 当前 `H_goal`（原实现读的模块级全局）。`height(int(H_goal))`
 *                   里的 `int()` 是**向零截断**
 * @return true = 走了直写舵机那一支（`deg` 有效）；false = 动画支或提前 return
 */
bool action_stand(const action_cfg_t *cfg, action_state_t *st, float cur_h_goal,
                  int32_t now_ms, float deg[ACTION_CHANNELS],
                  action_effects_t *eff);

/**
 * @brief 复刻 `padog.action_sit_direct()`（`action_sit()` 只是它的一行别名）。
 *
 * ```python
 * if pose_anim_active: return
 * if direct_pose_freeze: return                  # ← 两个提前 return，顺序照抄
 * crawl 清零; inplace_step_end_ms = 0
 * move(0,0,0); gait(0); set_leg_sit_offsets(0,0)
 * height(86); gesture(0, 0, in_y)
 * _pose_anim_begin(_stand_pose(), _sit_pose(), True)
 * ```
 *
 * ⚠️ **本函数从不写舵机**（它只启动动画）。`deg` 参数留着是为了接口一致。
 *
 * @return 恒为 false（原实现这条路径没有任何直接写舵机的语句）
 */
bool action_sit_direct(const action_cfg_t *cfg, action_state_t *st, int32_t now_ms,
                       float deg[ACTION_CHANNELS], action_effects_t *eff);

/** 复刻 `padog.action_sit()`：一行别名 */
bool action_sit(const action_cfg_t *cfg, action_state_t *st, int32_t now_ms,
                float deg[ACTION_CHANNELS], action_effects_t *eff);

/**
 * @brief 复刻 `mainloop()` 第 881~888 行的 `inplace_step_end_ms` 服务。
 *
 * ```python
 * if inplace_step_end_ms:
 *     if ticks_diff(inplace_step_end_ms, now) > 0:
 *         set_leg_sit_offsets(0, 0); gait(0); move(3, 1, 1)
 *     else:
 *         inplace_step_end_ms = 0; set_leg_sit_offsets(0, 0)
 * ```
 *
 * ⚠️ 活跃分支里的 `move(3,1,1)` **自己会把 `inplace_step_end_ms` 清零**（原 `move()`
 * 的 if 分支），所以这个"原地步态测试"只生效一帧。见文件头第 4 条。
 *
 * @return true = 走了"仍然活跃"那一支（调用方据此知道本帧要跑 TROT）
 */
bool action_inplace_step(action_state_t *st, const action_cfg_t *cfg, int32_t now_ms,
                         action_effects_t *eff);

/* ==================================================================== */
/*  挥手：把原实现的**阻塞脚本**拆成可驱动的状态机                        */
/* ==================================================================== */

/** 挥手进度。初值全 0 即可（用 `action_wave_init()`） */
typedef struct {
    int phase;
    int swing;
} action_wave_t;

void action_wave_init(action_wave_t *w);

/** `action_wave_step()` 的一步输出 */
typedef struct {
    /** 本步写了哪些逻辑通道（位 `ch` = 逻辑通道 `ch`）。0 = 本步没写舵机 */
    uint32_t ch_mask;
    /** 与 `ch_mask` 对应的 12 路角度（未置位的项无意义） */
    float deg[ACTION_CHANNELS];
    /** 本步之后调用方要推进的时钟（原实现的 `time.sleep_ms(...)`） */
    int32_t delay_ms;
    /** 本步产生的跨模块副作用（原实现是调用 `action_sit_direct()` / `action_stand()`） */
    action_effects_t eff;
} action_wave_step_t;

/**
 * @brief 复刻 `padog.action_wave_direct()`（`action_wave()` 是它的一行别名）的**一步**。
 *
 * 原实现是这么一串阻塞调用：
 * ```python
 * action_sit_direct()                 # 坐下（启动动画）
 * _wait_pose_anim_done()              # 阻塞等动画做完（46 x [mainloop + sleep 20]）
 * angle(1, -38); angle(2, -38); angle(7, +38); angle(8, +38)   # 双前腿抬起
 * time.sleep_ms(420)
 * angle(0, init_1p); angle(1, lift_h); angle(2, lift_s)        # 左前腿抬起
 * time.sleep_ms(300)
 * for _ in range(3):
 *     angle(1, min(180, lift_h + 20)); time.sleep_ms(260)      # 上摆
 *     angle(1, max(0, lift_h - 12));   time.sleep_ms(260)      # 下摆
 * time.sleep_ms(200)
 * action_stand()                      # 回站立（因为 direct_pose_freeze=True，走动画支）
 * ```
 *
 * 本函数一次推进一个阶段，把 `time.sleep_ms()` 变成 `delay_ms`、把写舵机变成
 * `ch_mask` + `deg`。等价驱动循环：
 * ```c
 * action_wave_step_t s;
 * while (action_wave_step(&st, &wave, &cfg, now, &s)) {
 *     apply_effects(&s.eff);                 // 用 control_chain_cmd_* 依次施加
 *     if (s.ch_mask) { write_channels(s.ch_mask, s.deg); }
 *     now += s.delay_ms;
 * }
 * ```
 *
 * 各阶段的 `delay_ms` 与写通道数（也就是原实现的时序）**由 golden 逐毫秒钉住**
 * （`tools/golden/golden/action_wave.csv` 的 `t_off` 列）。
 *
 * @param now_ms 当前时刻（只有"等动画"那一阶段用得到，`utime.ticks_ms()` 语义）
 * @param cur_h_goal 当前 `H_goal`。原实现里 `action_wave_direct()` 最后调的
 *                   `action_stand()` 会读这个模块级全局；本模块不持有它，所以由调用方传
 *                   （走到那一步时它已经是 `sit_height`，因为前面 `action_sit_direct()`
 *                   执行过 `height(86)`）
 * @return false = 整个动作已结束（此次调用**没有**产生任何输出）
 */
bool action_wave_step(action_state_t *st, action_wave_t *w, const action_cfg_t *cfg,
                      int32_t now_ms, float cur_h_goal, action_wave_step_t *out);

#ifdef __cplusplus
}
#endif
