/**
 * @file    control_chain.h
 * @brief   全链路编排：把摇杆/步态输入算成 12 路舵机角（P3，纯 C，可在电脑上测试）
 *
 * ## 它复刻的是哪一段原代码
 *
 * `micropython/padog.py` 的 `mainloop()` —— 也就是**原版固件每帧真正在做的事**：
 *
 * ```
 * _crawl_mainloop_service()                  # 爬行状态机（本模块 tick 的第一步）
 * if gait_mode==0:  t 递增 + PA_TROT.cal_t   # 否则
 * elif gait_mode==1: t 递增 + PA_WALK.cal_w
 * R_H / PIT_S / ROL_S / X_S 按 Kp 逼近目标 + 姿态限位   # slew 环
 * _ik_hc / _partial_geom_scale / _trot_rol_s / _walk_rol_s
 * if/elif 链选择重心分支 -> PA_ATTITUDE.cal_ges
 * _foot_y_targets + PA_IK.ik -> servo_output() -> 12 路舵机角
 * ```
 *
 * 前面几个模块（kinematics / body_pose / gait_trot / gait_walk / servo_map）各自
 * 只对照了**一个函数**；这一层对照的是**它们的编排方式**，包括那些从来没被单独
 * 测过的东西：大狗缩放层（`_geom_scale` / `_partial_geom_scale` / `_ik_hc` 与
 * 6 个 `_LARGE_*` 系数）、姿态 slew 环与限位、按步态与摇杆方向选重心分支的那一大串
 * `if/elif`、`_hip_leg_deltas` / `_apply_trot_swing_y` / `_trot_rol_s` / `_walk_rol_s`。
 *
 * 参考值不是"我把模块拼起来自己推的"，而是 `tools/golden/gen_golden.py` 里
 * **整个 exec 真版 padog.py、直接调用它的 `mainloop()`** 一次跑出来的
 * （见 `gen_control_chain()`），所以连编排的错位都会被抓到。
 *
 * ## 为什么配置要放进结构体
 *
 * 原实现的参数来自三个地方，全都以**模块级全局**存在：
 *
 * | 来源 | 例子 | 本模块怎么处理 |
 * |---|---|---|
 * | `config.py` / `config_s.py`（`exec` 进来） | `Ts` / `faai` / `h` / `Kp_H` / `speed` … | `control_chain_cfg_t` 字段 |
 * | padog.py 第 52~83 行的**默认值注入表** | `walk_faai=0.30` / `shank_ik_bias_per_mm=0.25` / `trot_right_h_mul=0.80` … | 同上（见下面的"来源"列） |
 * | padog.py 的模块级常量 | `_LARGE_*` / `HIP_TURN_DEAD` / `TURN_HIP_GAIN` … | 同上（也做成字段，便于上机调参） |
 *
 * `control_chain_cfg_defaults()` 填的是**这台机器的实际生效值**：config 文件里有的
 * 用 config 文件的值（`config_s.py` 的实测值），没有的用注入表的默认值。
 * 每个字段的注释里都写了它来自哪一边。
 *
 * ⚠️ **不要**把 app 层的 `app_config_t` 直接塞进来：两者的语义不同（那一个是
 * "可持久化的配置项"，本结构是"控制链一次运算需要的全部参数"），接线是后面单独的任务。
 *
 * ## 三个必须知道的移植事实
 *
 * 1. **本机没有 IMU**（`HANDOFF.md` 记了 4 次独立实测）。原实现在
 *    `PA_WALK.cal_w()` 里会读陀螺仪俯仰角，本机该值**恒为 0**，所以陀螺分量
 *    在 `gait_walk_cal_w()` 的入参里被显式写成 `gyro_p_deg = 0.0f`。
 *    将来真装了 IMU，这里改成传实测值即可，其余一行不用动。
 *
 * 2. **`PA_TROT.cal_t()` 没有 else 分支**（成长手册 P-19），`t > Ts` 在原实现里
 *    会 `UnboundLocalError`。C 版 `gait_trot.c` **故意**对 `t<0 || t>Ts` 做了相位
 *    回绕（迁移表 §8.10），所以喂越界输入等于在测"我自己的改进"而不是等价性。
 *    `tools/golden/golden/control_chain.csv` 里 `t` **只取 `[0, Ts]`**
 *    （原实现的可达域，`mainloop()` 里 `if t >= Ts: t = t - Ts` 保证），
 *    这条分歧**刻意没有被触发**。越界行为的差异不在本模块的验收范围内。
 *
 * 3. **`cal_w()` 的副作用在参考值里是"没生效"的**。原 `_apply_cg()` 会调
 *    `padog.gesture(0, int(CG_X), int(yst))` 去改重心目标；而 golden 的参考命名
 *    空间里 `padog` 是最小 stub，`gesture` 是 `lambda *a, **k: None`（no-op）。
 *    于是**参考值里 `X_goal` 没有被 `cal_w` 改过**。C 版因此**丢弃**
 *    `gait_walk_cal_w()` 的 `gesture_out`，把这条差异写在 `control_chain.c` 里。
 *    真机上原代码是会改目标的 —— 接线 app 层时必须显式决定要不要复刻这一条。
 *
 * ## 可测试性
 *
 * 纯 C，只依赖 `<math.h>` 与同目录的几个纯数学模块，不碰 ESP-IDF、不碰 I2C。
 * 宿主机 gcc 直接编译，与"原版 mainloop 真跑一次"的输出逐位对照
 * （`tools/golden/test_control_chain.c`，**容差 0**）。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "control/servo_map.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 逻辑通道数（= 舵机路数） */
#define CONTROL_CHAIN_CHANNELS SERVO_MAP_CHANNELS
/** 腿数（原实现编号 1..4：1 左前 / 2 右前 / 3 右后 / 4 左后） */
#define CONTROL_CHAIN_LEGS     4
/** 每腿关节数（髋 / 大腿 / 小腿） */
#define CONTROL_CHAIN_JOINTS   3

/* ==================================================================== */
/*  配置                                                                */
/* ==================================================================== */

/**
 * @brief 控制链的全部参数。
 *
 * 字段名尽量保留原 Python 的拼写（`Ts` / `faai` / `Kp_H` …），这样和 padog.py
 * 能逐行对着读；注释里的"来源"标明这个值在**这台机器**上是从哪来的。
 */
typedef struct {
    /* ---------------- 来自 config.py / config_s.py ---------------- */

    /** 步态周期（s）。config.py: `Ts=1`（config_s 未覆盖） */
    float Ts;
    /** 支撑相占空比。config_s.py: `faai=0.42`（覆盖 config.py 的 0.5） */
    float faai;
    /** 俯仰限位（度）。config.py: `pit_max_ang=15` */
    float pit_max_ang;
    /** 滚转限位（度）。config.py: `rol_max_ang=15` */
    float rol_max_ang;
    /** 最大 X 位移（mm）。config.py: `xs_max=80`（mainloop 数学路径不读它） */
    float xs_max;

    /** 中位角 `[腿][关节]`，腿 0..3 = 腿1..腿4，关节 0=髋 1=大腿 2=小腿。
     *  config_s.py: `init_1p..init_4s`（数值见 `control_chain_cfg_defaults()`）。
     *  注入表里 `"init_%dp" = 90` 是**兜底**：config_s.py 定义了全部 12 个，
     *  所以兜底不生效。 */
    float init[CONTROL_CHAIN_LEGS][CONTROL_CHAIN_JOINTS];

    /** 大腿连杆长（mm）。config_s.py: 130 */
    float l1;
    /** 小腿连杆长（mm）。config_s.py: 138 */
    float l2;
    /** 前后腿间距（mm）。config_s.py: 230 */
    float l;
    /** 机身宽相关尺寸（mm）。config_s.py: 120 */
    float b;
    /** 左右腿间距（mm）。config_s.py: 220。⚠️ `cal_ges()` 里那个 `w` 形参
     *  对输出**没有影响**（成长手册 P-19，body_pose.c 有实证），留着只为签名一致 */
    float w;

    /** 每帧 TROT 相位增量（s）。config_s.py: 0.065 */
    float speed;
    /** TROT/WALK 抬腿高度基准（mm）。config_s.py: 65 */
    float h;
    /** 站高 slew 系数。config_s.py: 0.06 */
    float Kp_H;
    /** 姿态 slew 系数。config_s.py: 0.03 */
    float Kp_G;
    /** WALK 的重心 X 入参（mm）。config_s.py: `CG_X=0`（注意 cal_w 的第 1 个实参） */
    float CG_X;
    /** WALK 的重心 Y 入参（mm）。config_s.py: `CG_Y=28` */
    float CG_Y;
    /** WALK 专用抬腿高（mm）。config_s.py: `walk_h=63`（mainloop 不读它，
     *  walk 分支的抬腿高同样来自 `h * _LARGE_H_TROT_MUL`） */
    float walk_h;
    /** WALK 相位增量覆盖值（s）。config_s.py: `walk_speed=0` ⇒ `<=0.001`，
     *  于是 `_walk_phase_step()` 用自动算出来的那个值 */
    float walk_speed;

    /** PA_IK 的 case。config_s.py: 0（串联腿，实机在跑） */
    int ma_case;
    /** 腿长参考值（小机 80+69）。config_s.py: 149 */
    float leg_len_ref;
    /** 摇杆前推对应的 spd 符号。config_s.py: **-1**（覆盖注入表的 +1） */
    float joy_fwd_sign;

    /** TROT 前进重心系数。config_s.py: 0.52（覆盖注入表的 0.38） */
    float trot_cg_f;
    /** TROT 后退重心系数。config_s.py: 0.85（覆盖注入表的 1.6） */
    float trot_cg_b;
    /** TROT 原地重心系数。config_s.py: 0（注入表也是 0） */
    float trot_cg_t;

    /** 髋辅助偏航：滚转增益。config_s.py: 0.06 */
    float hip_k_roll;
    /** 髋辅助偏航：俯仰增益。config_s.py: 0.02 */
    float hip_k_pitch;
    /** 髋辅助偏航：转向增益。config_s.py: 0.75 */
    float hip_k_turn;
    /** 髋辅助偏航限幅（度）。config_s.py: 18.0 */
    float hip_delta_max;

    /** 上电默认站高目标。config_s.py: `H_goal=81`（注入表是 100） */
    float H_goal;
    /** 上电默认重心 X 目标。config_s.py: `in_y=18` */
    float in_y;
    /** 上电默认俯仰目标。config_s.py: `in_pit=0` */
    float in_pit;
    /** 上电默认滚转目标。config_s.py: `in_rol=0` */
    float in_rol;
    /** 标定腿选择。config_s.py: 2（mainloop 数学路径不读它） */
    int cal_leg_sel;

    /* ---------- padog.py 注入表（config 文件里没有的键，值 = 注入表默认） ---------- */

    /** 小腿 IK 偏置系数（deg/mm）。注入表: **0.25**。
     *  ⚠️ `_shank_ik_bias()` 里那个 `per_mm = 0.375` 是死代码（注入表总会定义这个键），
     *  见 `servo_map.c` 的长注释。 */
    float shank_ik_bias_per_mm;
    /** 小腿 IK 固定偏置（deg）。注入表: 0.0（`>0` 时优先于 per_mm 分支 —— 
     *  ⚠️ `servo_map.c` 目前只实现了 per_mm 分支，改这个字段需要同步改那边） */
    float shank_ik_bias_deg;

    /** 前腿足端竖直偏置（mm）。注入表: 0.0 */
    float front_leg_y_offset;
    /** 后腿足端竖直偏置（mm）。注入表: 0.0 */
    float rear_leg_y_offset;

    /** 每条腿的小腿微调 `leg{n}_s_trim`（度），索引 0..3 = 腿1..腿4。
     *  注入表: 全 0.0。是 `servo_output()` 里 `_leg_cfg("s_trim", n)` 的取值来源 */
    float s_trim[CONTROL_CHAIN_LEGS];

    /** `leg2_z_mul` / `leg3_z_mul` / `leg4_z_mul`（注入表: 全 1.0）。
     *  ⚠️ 注入表里有、但控制链的数学路径**不读**它们（`_leg_cfg()` 只被
     *  `s_trim` 用过）。搬过来是为了这张表不被"抄一半"，
     *  将来 `mech_arm` 之类要读的话有地方取。注意注入表里**没有** `leg1_z_mul`。 */
    float leg2_z_mul;
    float leg3_z_mul;
    float leg4_z_mul;

    /** WALK 每腿摆动相占比。注入表: 0.30。⚠️ `_walk_faai()` 的逻辑是
     *  `>0.05 → 用它`，否则退到 `faai*0.55`（**不是**退到 0.30，尽管
     *  注入表的默认值恰好是 0.30 —— 那个分支只在键不存在时才走） */
    float walk_faai;
    /** WALK 相位增量倍率。注入表: 1.4（`<=0.05` 时退回 1.0） */
    float walk_speed_scale;
    /** 直行 WALK 滚转微调（度）。注入表: 3 */
    float walk_roll_trim;
    /** 直行 TROT 滚转微调（度）。注入表: 0 */
    float trot_roll_trim;
    /** 右侧两条腿（腿2/腿3）摆动抬腿高度倍率。注入表: 0.80。
     *  `>=0.999` 时整段跳过（`_apply_trot_swing_y` 提前 return） */
    float trot_right_h_mul;

    /* ---------------- padog.py 的模块级常量（不是配置项） ---------------- */

    /** 步幅系数。`_LARGE_STRIDE_XF_MUL = 0.90` */
    float large_stride_xf_mul;
    /** 步幅几何缩放比例。`_LARGE_STRIDE_GEOM_FRAC = 0.52` */
    float large_stride_geom_frac;
    /** 摆动起点比例。`_LARGE_STRIDE_XS_RATIO = 0.0`
     *  ⚠️ 为 0 ⇒ `_xs` 恒为 0（`_xs = -0.0 * _xf`），TROT 的 `xs` 永远是 0 */
    float large_stride_xs_ratio;
    /** 重心几何缩放比例。`_LARGE_CG_GEOM_FRAC = 0.35` */
    float large_cg_geom_frac;
    /** 后退重心倍率。`_LARGE_BWD_CG_MUL = 0.50` */
    float large_bwd_cg_mul;
    /** TROT/WALK 抬腿高度倍率。`_LARGE_H_TROT_MUL = 0.96` */
    float large_h_trot_mul;

    /** 转向生效的死区（%）。`HIP_TURN_DEAD = 10` */
    float hip_turn_dead;
    /** 摇杆百分比 -> 髋角增量的比例。`HIP_TURN_STICK_SCALE = 8.0` */
    float hip_turn_stick_scale;
    /** 髋转向增益。`TURN_HIP_GAIN = 1.0` */
    float turn_hip_gain;

    /* ---------------- 爬行（CRAWL_*，padog.py 模块级常量） ---------------- */

    /** 爬行时前腿小腿压低量（度）。`CRAWL_SHANK_FRONT = 20` */
    float crawl_shank_front;
    /** 爬行时后腿小腿压低量（度）。`CRAWL_SHANK_REAR = 30` */
    float crawl_shank_rear;
    /** 爬行前进用的 spd。`CRAWL_FWD_SPD = -3.5`（负数 = 前进，因为 joy_fwd_sign=-1） */
    float crawl_fwd_spd;
    /** 爬行持续时长（ms）。`CRAWL_DURATION_MS = 5000` */
    int32_t crawl_duration_ms;
    /** 爬行前下蹲稳定时长（ms）。`CRAWL_SETTLE_MS = 400` */
    int32_t crawl_settle_ms;

    /* ---------------- 注入表里与控制链无关的键（搬全，不半抄） ---------------- */

    /** 机械臂相关。注入表: 见 `control_chain_cfg_defaults()`。
     *  ⚠️ 控制链的数学路径**一个都不读** —— 它们属于 `mech_arm`。
     *  放在这里只是为了让 padog.py 那张默认值注入表**在这一个文件里是完整的**，
     *  将来 mech_arm 的 cfg 可以从这里取值，不必回头再翻 padog.py。 */
    int arm_base_init;
    int arm_grip_init;
    int arm_upper_init;
    int arm_fore_init;
    int arm_upper_min;
    int arm_upper_max;
    int arm_fore_min;
    int arm_fore_max;
    float arm_upper_rate;
    float arm_fore_rate;
    int arm_upper_dir;
    int arm_fore_dir;
    int arm_upper_ch;
    int arm_fore_ch;
    int arm_grip_ch;
    int arm_upper_board;
    int arm_fore_board;
    int arm_grip_board;
    int arm_grip_gpio;
    int arm_grip_open_level;
    int arm_grip_close_level;
    int arm_grip_digital;
    int arm_grip_pwm_hz;
    int arm_grip_min_us;
    int arm_grip_max_us;
    int arm_upper_walk;
    int arm_fore_walk;
    float arm_walk_rate;
    int arm_base_min;
    int arm_base_max;
    int arm_grip_open;
    int arm_grip_close;
    float arm_base_rate;
    int arm_base_dir;
    int arm_grip_stick_sign;
} control_chain_cfg_t;

/**
 * @brief 填入**这台机器**的实际生效默认值（config_s.py 的实测值 + 注入表的兜底值）。
 *
 * 与 `padog.py` 模块级那两步等价：
 * ```python
 * exec(open('config.py').read()); exec(open('config_s.py').read())
 * for _hk, _hd in (...):        # 第 57~81 行的默认值注入表
 *     if _hk not in _g: _g[_hk] = _hd
 * ```
 *
 * @param cfg 目标（为 NULL 时不做任何事）
 */
void control_chain_cfg_defaults(control_chain_cfg_t *cfg);

/* ==================================================================== */
/*  状态                                                                */
/* ==================================================================== */

/**
 * @brief mainloop 会改写、而且**必须跨帧保留**的量。
 *
 * 前 5 个是这条控制链的"积分器"：`t` 是步态相位、`R_H` 是站高、后三个是姿态/重心
 * 的当前值（都被 slew 环一步步逼向目标）。
 *
 * 后 6 个同样是 padog.py 的模块级可变全局，`mainloop()` 通过
 * `move()` / `gait()` / `servo_init()` / `_crawl_mainloop_service()` 改它们：
 *
 * | 字段 | 谁改它 | 为什么必须留 |
 * |---|---|---|
 * | `gait_mode` | `gait()`（爬行服务里会调） | 爬行会把步态切回 TROT，后续数学链按新值走 |
 * | `crawl_phase` | `_crawl_mainloop_service()`（1→2→0） | 不留就永远停在 1，"爬行 5 秒后恢复"永远不结束 |
 * | `crawl_until_ms` / `crawl_settle_until_ms` | `action_crawl()` / `_crawl_mainloop_service()` | 爬行的两个截止时刻 |
 * | `crawl_saved_h` | `action_crawl()` | 爬行结束后要恢复的站高 |
 * | `init_case` | `servo_init()` | `servo_output()` 的第二个实参，决定走 IK 还是直接站姿 |
 *
 * ⚠️ `gait_mode` 与 `crawl_phase` 同时也是 `control_chain_input_t` 的字段：
 * 原实现里它们是全局量，由**上层**（串口/网页命令、golden 生成器每行的注入）写。
 * `control_chain_tick()` 在入口把输入值写进 state（等价于那一步注入），
 * 之后 mainloop 自己的代码（`gait(0)` 等）可能再改它 —— 与参考实现完全一致。
 */
typedef struct {
    /** 步态相位时间（s） */
    float t;
    /** 站高当前值（mm），被 Kp_H slew 逼向 `H_goal` */
    float R_H;
    /** 俯仰当前值（度） */
    float PIT_S;
    /** 滚转当前值（度） */
    float ROL_S;
    /** 重心 X 当前值（mm） */
    float X_S;

    /** 步态模式：0 = TROT，其它 = WALK / 直通（原实现只有 0/1 有分支） */
    int gait_mode;
    /** 爬行状态机：0 关，1 下蹲稳定，2 前进中 */
    int crawl_phase;
    /** 爬行前进截止时刻（`utime.ticks_ms()` 语义） */
    int32_t crawl_until_ms;
    /** 爬行下蹲稳定截止时刻 */
    int32_t crawl_settle_until_ms;
    /** 爬行前保存的站高（`action_crawl()` 里 `int(H_goal)`，恢复时用） */
    int crawl_saved_h;
    /** `servo_output()` 的第二个实参（0 = 走 IK 路径） */
    int init_case;
} control_chain_state_t;

/**
 * @brief 按 padog.py 的**模块级初值**初始化状态。
 *
 * 对应 padog.py 第 140~166 行：
 * ```python
 * t=0; PIT_S=0; ROL_S=0; X_S=0
 * H_goal=int(H_goal); R_H=H_goal
 * gait_mode=0; crawl_phase=0; crawl_until_ms=0; crawl_settle_until_ms=0
 * crawl_saved_h=int(H_goal); init_case=0
 * ```
 * 注意 `H_goal` / `crawl_saved_h` 都经过 `int()`（**向零截断**），所以默认的
 * `H_goal=81` 取 81；若把 `H_goal` 配成 81.7，初值仍是 81。
 *
 * @param st  目标（为 NULL 时不做任何事）
 * @param cfg 配置（为 NULL 时用 0 兜底）
 */
void control_chain_state_init(control_chain_state_t *st, const control_chain_cfg_t *cfg);

/* ==================================================================== */
/*  每 tick 输入 / 输出                                                  */
/* ==================================================================== */

/**
 * @brief 每帧输入。字段顺序与 `golden/control_chain.csv` 的前 15 列一一对应。
 *
 * 这个结构体对应的是"上层每帧告诉控制链什么"：摇杆命令（`spd` / `L` / `R` /
 * `joy_turn`）、步态选择、爬行命令，以及姿态/站高**目标**（`H_goal` / `PIT_goal` /
 * `ROL_goal` / `X_goal`）。原实现里这些是模块级全局，由 `move()` / `gait()` /
 * `gesture()` / `height()` 以及网页/串口命令写入。
 *
 * ⚠️ **本模块把它们当"每帧输入"，所以目标是调用方持有的**。tick 内部的
 * `gait(0)` / `gesture()` / `height()`（只有爬行服务会调）改的是**本帧的工作副本**，
 * 不会写回调用方；真正持久化到 `control_chain_state_t` 的只有
 * `t` / `R_H` / `PIT_S` / `ROL_S` / `X_S` / `gait_mode` / 爬行状态 / `init_case`。
 * 换句话说：`mainloop()` 里"`gait(0)` 把三个重心目标重置成 `in_pit/in_rol/in_y`"
 * 这一条，在本模块里**只影响当前这一帧**。将来接 app 层时，若要让它在帧间持续生效，
 * 得由 app 层按同样的规则更新自己的目标字段（原实现是全局量，天然持续生效）。
 * golden 每一行都是"注入输入 -> 跑一帧"，所以这条差异在对照里**看不出来**；
 * 这里显式写出来，免得以后踩。
 */
typedef struct {
    /** 前进速度命令（含符号）。config_s.py `joy_fwd_sign=-1` ⇒ **负数才是前进** */
    float spd;
    /** 左腿相位系数（-1/0/1），`servo_output` 与 `cal_t` 的腿系数 */
    int L;
    /** 右腿相位系数（-1/0/1） */
    int R;
    /** 步态：0 = TROT，1 = WALK */
    int gait_mode;
    /** 横杆转向百分比（%）。`|joy_turn| >= 10` 时髋角才参与转向 */
    float joy_turn;
    /** 爬行状态机当前相位（0/1/2）。tick 内部会按爬行服务更新 state 里的副本 */
    int crawl_phase;

    /** 站高目标（mm） */
    float H_goal;
    /** 俯仰目标（度） */
    float PIT_goal;
    /** 滚转目标（度） */
    float ROL_goal;
    /** 重心 X 目标（mm） */
    float X_goal;

    /** 本帧时刻（`utime.ticks_ms()` 语义）。只有爬行状态机用它比截止时刻；
     *  与运动数学无关。golden 里两个截止时刻都是 0，所以传 0 与传真实时刻
     *  走的是同一个分支（`ticks_diff(0, now) <= 0`） */
    int32_t now_ms;
} control_chain_input_t;

/**
 * @brief 按 padog.py 的**模块级初值**初始化每帧输入。
 *
 * 对应 `PIT_goal=int(in_pit); ROL_goal=int(in_rol); X_goal=int(in_y);
 * spd=0; L=0; R=0; joy_turn=0; gait_mode=0; crawl_phase=0`，以及
 * `H_goal=int(H_goal)`。
 */
void control_chain_input_init(control_chain_input_t *in, const control_chain_cfg_t *cfg);

/**
 * @brief 本 tick 的中间量（诊断用，不参与控制）。
 *
 * 值都取自参考实现里同名的中间变量，便于 golden FAIL 时一眼看出是哪一段先错：
 * 是步态轨迹（`gait`）、姿态叠加（`ges`）、还是 IK / 舵机映射（`ham` / `shank`）。
 */
typedef struct {
    /** `P_`：步态足端目标，`[0..3]` = x1..x4，`[4..7]` = y1..y4 */
    float gait[8];
    /** `P_G`：`cal_ges()` 的输出，`[0..3]` = x1..x4，`[4..7]` = y1..y4 */
    float ges[8];
    /** `_foot_y_targets()` 的结果（已含前后腿偏置） */
    float foot_y[CONTROL_CHAIN_LEGS];
    /** `PA_IK.ik()` 的大腿角 */
    float ham[CONTROL_CHAIN_LEGS];
    /** `PA_IK.ik()` 的小腿角 */
    float shank[CONTROL_CHAIN_LEGS];
    /** `_hip_leg_deltas()`：髋辅助偏航增量（叠加在中位角上） */
    float hip[CONTROL_CHAIN_LEGS];
    /** `_crawl_shank_servodelta()`：爬行小腿压低增量 */
    float crawl_cs[CONTROL_CHAIN_LEGS];
} control_chain_trace_t;

/** @brief 每帧输出 */
typedef struct {
    /** 12 路舵机角，**逻辑通道顺序 0..11**（交 `servo_map` 编码成寄存器值） */
    float angle_deg[CONTROL_CHAIN_CHANNELS];
    /**
     * 本帧**实际用到**的四个目标：`[0]=H_goal, [1]=PIT_goal, [2]=ROL_goal, [3]=X_goal`。
     *
     * ⚠️ **app 层必须把它作为下一帧的 `in` 目标喂回来。**
     *
     * 原因：原实现里这四个量是 `padog` 的**模块级全局**，被
     * `gesture()` / `height()` / `gait()` / `move()` 改一次就**一直留着**。
     * `WALK` 的 `cal_w()` 内部会调 `padog.gesture(0, int(CG_X), int(yst))`，
     * 于是原版每帧都在改写重心目标并保留到后续帧。
     *
     * C 版把目标设计成"每帧输入"，所以"保留"这件事必须由调用方完成 ——
     * 不喂回来，副作用就只影响当帧，**多帧行为会与原版分道扬镳**。
     * （golden 是单帧逐行对照，看不出这个差别，所以这里必须靠文档约束。）
     */
    float goal[4];
    /** 中间量（诊断用） */
    control_chain_trace_t trace;
} control_chain_out_t;

/* ==================================================================== */
/*  主入口                                                              */
/* ==================================================================== */

/**
 * @brief 跑一帧控制链。对应 `padog.mainloop()` 的运动数学部分。
 *
 * 顺序（与参考实现逐行对应）：
 * 1. `_crawl_mainloop_service()` —— 爬行状态机（含 `move()` / `gait()` 的副作用）
 * 2. 步态分支：TROT（`t` 递增 + `gait_trot_cal_t`）或 WALK（+ `gait_walk_cal_w`）
 * 3. `R_H` / `PIT_S` / `ROL_S` / `X_S` 按 `Kp_H` / `Kp_G` 逼近目标
 * 4. 姿态限位 `pit_max_ang` / `rol_max_ang`
 * 5. `_ik_hc` / `_partial_geom_scale` / `_trot_rol_s` / `_walk_rol_s`
 * 6. `if/elif` 链选重心分支 → `body_pose_cal_ges`
 * 7. `_foot_y_targets` + `kin_ik` → `servo_map_legs_to_angles`
 *
 * ⚠️ **本函数不包含** mainloop 开头的那几个"冻结/动画"提前返回
 * （`stop_run_node`、`pose_anim_active`、`direct_pose_freeze`、`inplace_step_end_ms`）——
 * 它们属于姿态动画层，golden 参考值也把它们全置为"未激活"。调用方负责在这些
 * 状态下**不要调用**本函数。
 *
 * @param cfg 配置（只读）
 * @param st  状态（**会被就地更新**：`t` / `R_H` / `PIT_S` / `ROL_S` / `X_S` /
 *            `gait_mode` / `crawl_phase` / 爬行截止时刻 / `init_case`）
 * @param in  本帧输入（只读）
 * @param out 12 路舵机角 + 中间量
 */
void control_chain_tick(const control_chain_cfg_t *cfg,
                        control_chain_state_t *st,
                        const control_chain_input_t *in,
                        control_chain_out_t *out);

/* ==================================================================== */
/*  被 tick 调用的那些原实现函数（导出便于单测与逐项对照）                 */
/* ==================================================================== */

/** 对应 `padog._geom_scale()`：`(l1+l2)/leg_len_ref`（`leg_len_ref<1` 时按 149 算） */
float control_chain_geom_scale(const control_chain_cfg_t *cfg);

/** 对应 `padog._ik_hc(r_h)`：`r_h + max(0, l1+l2-leg_len_ref)`。
 *  ⚠️ 是**加 mm 偏移**而不是乘 `geom_scale` —— 原实现注释里专门写了"乘了反而更矮" */
float control_chain_ik_hc(const control_chain_cfg_t *cfg, float r_h);

/** 对应 `padog._partial_geom_scale(frac)`：`1 + (gs-1)*clamp(frac,0,1)`。
 *  步幅/重心只按比例部分缩放，不整体乘 1.43，否则大狗水平命令过大 → 滑步 */
float control_chain_partial_geom_scale(const control_chain_cfg_t *cfg, float frac);

/** 对应 `padog._joy_forward_motion()`：摇杆前推为前进时 true（看 `joy_fwd_sign`） */
bool control_chain_joy_forward_motion(const control_chain_cfg_t *cfg, float spd);

/** 对应 `padog._joy_backward_motion()`：摇杆后拉为后退时 true */
bool control_chain_joy_backward_motion(const control_chain_cfg_t *cfg, float spd);

/** 对应 `padog._trot_right_h_mul()`（注入表默认 0.80） */
float control_chain_trot_right_h_mul(const control_chain_cfg_t *cfg);

/**
 * @brief 对应 `padog._apply_trot_swing_y(p_)`：右侧两条腿抬腿高度略降，减轻左高右低。
 *
 * 就地修改 `p[5]`（腿2）与 `p[6]`（腿3）（仅当 `>0.05` 时乘倍率）。
 * `trot_right_h_mul >= 0.999` 时整段跳过。
 */
void control_chain_apply_trot_swing_y(const control_chain_cfg_t *cfg, float p[8]);

/** 对应 `padog._walk_faai()`。⚠️ 注意 `walk_faai<=0.05` 时退到 `faai*0.55` */
float control_chain_walk_faai(const control_chain_cfg_t *cfg);

/** 对应 `padog._walk_speed_scale()`（`<=0.05` 时退回 1.0） */
float control_chain_walk_speed_scale(const control_chain_cfg_t *cfg);

/** 对应 `padog._walk_roll_trim()` */
float control_chain_walk_roll_trim(const control_chain_cfg_t *cfg);

/** 对应 `padog._walk_phase_step()`：WALK 每帧相位增量（`walk_speed` 覆盖优先） */
float control_chain_walk_phase_step(const control_chain_cfg_t *cfg);

/**
 * @brief 对应 `padog._walk_rol_s()`：直行 WALK 的滚转微调。
 *
 * `|joy_turn| >= HIP_TURN_DEAD` 或 `|spd| < 0.05` 时**原样返回** `rol_s`，
 * 否则 `rol_s + walk_roll_trim`。⚠️ 传进来的 `rol_s` 是**已经过 slew 与限位**的值。
 */
float control_chain_walk_rol_s(const control_chain_cfg_t *cfg,
                               float rol_s, float joy_turn, float spd);

/** 对应 `padog._trot_roll_trim()` */
float control_chain_trot_roll_trim(const control_chain_cfg_t *cfg);

/** 对应 `padog._trot_rol_s()`：直行 TROT 的滚转微调（同上，微调量默认 0） */
float control_chain_trot_rol_s(const control_chain_cfg_t *cfg,
                               float rol_s, float joy_turn, float spd);

/**
 * @brief 对应 `padog._trot_turn_lr()` 的四个腿系数。
 *
 * ⚠️ 字段顺序**必须**保持 `lr1, lr4, lr2, lr3` —— 原实现形参就是这个顺序
 * （成长手册 P-18：映射错位在对称输入下测不出来，所以结构体字段名也分开写）。
 */
typedef struct {
    float lr1;
    float lr4;
    float lr2;
    float lr3;
} control_chain_turn_lr_t;

/**
 * @brief 对应 `padog._trot_turn_lr()`。
 *
 * ⚠️ 原实现三个分支**返回值完全相同**（全是 1,1,1,1）—— 属于成长手册 P-19 那类
 * "改了也不影响结果"的无效分支。这里照样逐行照抄（条件也照算），不做"清理"：
 * 万一将来原实现改了某条分支，对照关系还在。
 */
control_chain_turn_lr_t control_chain_trot_turn_lr(const control_chain_cfg_t *cfg,
                                                  float spd, float joy_turn);

/**
 * @brief 对应 `padog._turn_phase_lr(jt)`：`jt>10` → 左转 `(-1, 1)`；
 *        `jt<-10` → 右转 `(1, -1)`；否则 `(1, 1)`。
 *
 * ⚠️ `mainloop()` **不调用**它 —— 它被 `padog.turn_hip()` 用来设 `L`/`R`，
 * 也就是"算好相位再喂给 `move()`"。搬过来是为了 `turn_hip()` 那一层将来要接线。
 */
void control_chain_turn_phase_lr(const control_chain_cfg_t *cfg, float joy_turn,
                                 int *out_l, int *out_r);

/**
 * @brief 对应 `padog._hip_leg_deltas()`：髋辅助偏航增量 `h1..h4`。
 *
 * `turn_mag = |jt|/HIP_TURN_STICK_SCALE * TURN_HIP_GAIN`（`jt>0` 取反），
 * 四条腿的符号组合是 `+-+` / `-+-` 那种**不对称**写法，逐行照抄；
 * 每项再夹到 `±hip_delta_max`。
 *
 * ⚠️ `rol_s` / `pit_s` 传的必须是**已过 slew 与限位**的值（原实现就是这么读全局的）。
 */
void control_chain_hip_leg_deltas(const control_chain_cfg_t *cfg,
                                  float rol_s, float pit_s, float joy_turn,
                                  float delta[CONTROL_CHAIN_LEGS]);

/** 对应 `padog._crawl_active()`：`int(crawl_phase) != 0` */
bool control_chain_crawl_active(int crawl_phase);

/**
 * @brief 对应 `padog._crawl_shank_servodelta(leg_n)`。
 *
 * ⚠️ 腿1/腿4 取**负**（前腿 -20、后腿 -30），腿2/腿3 取正（+20、+30）——
 * 因为 `servo_output()` 里前两条腿的小腿是 `+cal_test_shank`、另两条是
 * `-cal_test_shank`。符号是原实现的不对称写法，别"统一"。
 *
 * @param leg_n 1..4（原实现的腿编号）
 */
float control_chain_crawl_shank_servodelta(const control_chain_cfg_t *cfg,
                                           int crawl_phase, int leg_n);

/**
 * @brief 对应 `padog._foot_y_targets(p_y1..p_y4)`：只加前后腿偏置，四腿一致。
 *
 * @param p_y 步态输出的四个竖直目标（`P_[4..7]`）
 * @param fy  输出：腿1..腿4
 */
void control_chain_foot_y_targets(const control_chain_cfg_t *cfg,
                                 const float p_y[CONTROL_CHAIN_LEGS],
                                 float fy[CONTROL_CHAIN_LEGS]);

#ifdef __cplusplus
}
#endif
