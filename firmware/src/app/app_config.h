/**
 * @file    app_config.h
 * @brief   配置结构体、默认值、校验与持久化 —— 取代 MicroPython 版"写回源码文件"的做法
 *
 * ## 为什么需要它
 *
 * MicroPython 版的标定保存（`web_c.py` 里 `key=sc`）是这样干的：
 * ```python
 * s_f = open("config_s.py", "w+")            # 直接重写 Python 源码
 * s_f.write("init_1p=" + str(padog.init_1p) + "\n")
 * ...
 * ```
 * 编译成 C 之后**没有源码可写**，所以必须换成真正的持久化存储 —— ESP32 的 **NVS**
 * （Flash 里一块专用键值分区，启动日志里能看到：`nvs 01 02 00009000 00006000`，24 KB）。
 *
 * ## 设计要点
 *
 * 1. **本模块是纯 C**，只依赖 `<stdint.h>` / `<stddef.h>` / `<string.h>`，
 *    **不依赖 ESP-IDF** —— 存储后端通过 `app_cfg_store_t` 函数指针注入。
 *    固件注入基于 `nvs_flash` 的实现，宿主测试注入 RAM 假后端，
 *    于是"默认值/限幅/CRC/版本/往返"这些逻辑全部能在电脑上测，不用烧板子。
 *
 * 2. **版本号 + CRC32**。没有校验的话，读到半写或损坏的数据会把**舵机中位角**
 *    变成垃圾 —— 那等于让 12 个舵机乱冲。宁可回落到默认值并把错误报出来。
 *
 * 3. **合法值限幅**。迁移表要求"非法值被限幅"，`app_config_validate()` 负责这件事，
 *    并把被改动的项数报给调用方（便于日志）。
 *
 * 4. **上电不自动动作**这条原则不受影响：配置只在初始化时读，读完照样把所有通道
 *    置为"无脉冲"。
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 当前配置结构版本。**改动结构体就要 +1**，旧版本会被判为不兼容并回落默认值。
 *
 * 历史：
 *   1 —— 只含 `config.py` / `config_s.py` 派生的字段
 *   2 —— 补上 padog.py 默认值注入表里那些**只存在于注入表**的键
 *        （19 个字段，见下面的"padog.py 默认值注入表"两段），
 *        并因此改变 `sizeof(app_config_t)`（360 → 448 字节）
 */
#define APP_CFG_VERSION 2u

/** NVS 里存配置用的键名 */
#define APP_CFG_NVS_KEY   "appcfg"
/** NVS 命名空间 */
#define APP_CFG_NVS_NS    "dogcfg"

/** 腿部数量 */
#define APP_CFG_LEGS      4
/** 每腿关节数（髋 / 大腿 / 小腿） */
#define APP_CFG_JOINTS    3

/** 返回码（纯 C，不用 esp_err_t） */
typedef enum {
    APP_CFG_OK          =  0,
    APP_CFG_ERR_NOENT   = -1,   /**< NVS 里没有配置，用了默认值（首次开机属于正常） */
    APP_CFG_ERR_CRC     = -2,   /**< 校验和不符，已回落默认值 */
    APP_CFG_ERR_VERSION = -3,   /**< 版本不兼容，已回落默认值 */
    APP_CFG_ERR_IO      = -4,   /**< 存储读写失败 */
    APP_CFG_ERR_ARG     = -5,   /**< 参数错误 */
    APP_CFG_ERR_UNKNOWN = -6,
} app_cfg_ret_t;

/**
 * @brief 存储后端。返回值用 `app_cfg_ret_t`（0 = 成功，负 = 失败）。
 *
 * `get_blob` 在键不存在时应返回 `APP_CFG_ERR_NOENT`；
 * 并且无论成败都要把 `*len` 更新为实际长度（或所需长度）。
 */
typedef struct {
    int (*get_blob)(const char *key, void *out, size_t *len);
    int (*set_blob)(const char *key, const void *data, size_t len);
    int (*erase_key)(const char *key);
} app_cfg_store_t;

/**
 * @brief 全部可配置项。
 *
 * 默认值来自学长的 `config.py` 与 `config_s.py`（**照抄实测值，不重新推导**），
 * 外加 `padog.py` 第 57~81 行**默认值注入表**里那些两个 config 文件都没定义的键
 * （见下面两段"padog.py 默认值注入表"，值一律以 `control_chain_cfg_defaults()` 为准）。
 * 这些键是**实证**找出来的，不是手写清单：判据是"在真版 padog.py 命名空间里，
 * 只有注入表提供了它"（成长手册 P-23/P-24 —— 搜索会漏报，求值不会）。
 *
 * ⚠️ 腿部编号与原实现一致：**腿1=左前、腿2=右前、腿3=右后、腿4=左后**。
 *    注意这**不是**按物理顺序排的（3 是右后、4 是左后）—— 原实现如此，不要"整理"。
 *    逻辑通道与腿的映射是另一件事（见 `servo_map`），本结构体保持与 `config_s.py` 同构。
 */
typedef struct {
    /* ---------- 头部（版本 + 校验） ---------- */
    uint16_t version;          /**< 结构版本，见 APP_CFG_VERSION */
    uint16_t reserved;         /**< 对齐用，必须为 0 */
    uint32_t crc32;            /**< 覆盖除本字段外整个结构；保存时由 save 计算 */

    /* ---------- 舵机中位角（来自 config_s.py，是实测标定值） ---------- */
    /** `servo_center[leg][joint]`，leg 0..3 = 腿1..腿4，joint 0=髋 1=大腿 2=小腿 */
    float servo_center[APP_CFG_LEGS][APP_CFG_JOINTS];

    /* ---------- 几何（mm） ---------- */
    float l1;      /**< 大腿连杆长，130 */
    float l2;      /**< 小腿连杆长，138 */
    float l;       /**< 前后腿间距，230 */
    float b;       /**< 机身宽相关，120 */
    float w;       /**< 左右腿间距，220。⚠️ 实测对 cal_ges 输出无影响（见迁移表 §8.10） */
    float leg_len_ref; /**< 腿长参考，149（用于几何比例换算） */

    /* ---------- 站姿与姿态 ---------- */
    float h_goal;      /**< 站立高度目标，81 */
    float in_y;        /**< 上电默认重心 X 平移，18 */
    float in_pit;      /**< 上电默认俯仰，0 */
    float in_rol;      /**< 上电默认滚转，0 */
    float pit_max_ang; /**< 俯仰限幅，15（config.py） */
    float rol_max_ang; /**< 滚转限幅，15（config.py） */
    float xs_max;      /**< X 平移限幅，80（config.py） */
    float cg_x;        /**< CG_X，0 */
    float cg_y;        /**< CG_Y，28 */
    float kp_h;        /**< 高度调节 P，0.06 */
    float kp_g;        /**< 姿态调节 P，0.03 */

    /* ---------- 步态 ---------- */
    float ts;          /**< 周期，1.0（config.py） */
    float faai;        /**< TROT 占空比，0.42（config_s.py 覆盖了 config.py 的 0.5） */
    float speed;       /**< TROT 相位增量，0.065 */
    float h;           /**< TROT 抬腿高度，65 */
    float trot_cg_f;   /**< TROT 前进重心，0.52 */
    float trot_cg_b;   /**< TROT 后退重心，0.85 */
    float trot_cg_t;   /**< TROT 转向重心，0 */
    float walk_faai;   /**< WALK 每腿摆动占比，0.30（padog 默认 walk_faai） */
    float walk_h;      /**< WALK 抬腿高度，63 */
    float walk_speed;  /**< WALK 步频，0（0 = 自动） */

    /* ---------- 髋辅助与转向 ---------- */
    float hip_k_roll;    /**< 0.06 */
    float hip_k_pitch;   /**< 0.02 */
    float hip_k_turn;    /**< 0.75 */
    float hip_delta_max; /**< 18.0 */

    /* ---------- 逆运动学 / 标定选择 ---------- */
    int32_t ma_case;      /**< 0 = 串联腿（实机在用），1 = 并联腿（未验证） */
    int32_t joy_fwd_sign; /**< 摇杆前推方向，-1 */
    int32_t cal_leg_sel;  /**< 标定页当前选中腿，1..4 */

    /* ---------- padog.py 默认值注入表：控制链相关 ----------
     * 下面这些键**只存在于** padog.py 第 57~81 行的默认值注入表里
     * （`config.py` / `config_s.py` 都没有定义它们），所以"只按 config 文件派生"的
     * 本结构体一开始漏掉了它们，app 层也就没法把 `control_chain_cfg_t` 填全。
     *
     * 默认值与 `control_chain_cfg_defaults()`（control_chain.c）**逐项相等** ——
     * 那才是这台机器上真正生效的值；`tools/golden/test_app_config.c` 里有一组
     * 断言直接把两边逐字段对比，不允许凭印象填数（成长手册 P-23/P-24）。
     */
    /** 小腿 IK 偏置系数（deg/mm）。注入表 `shank_ik_bias_per_mm = 0.25`
     *  （⚠️ 不是 `_shank_ik_bias()` 里那个死代码 0.375）。默认值取自
     *  `control_chain_cfg_defaults()` */
    float shank_ik_bias_per_mm;
    /** 小腿 IK 固定偏置（度）。注入表 `shank_ik_bias_deg = 0.0`；
     *  `>0` 时优先于 per_mm 分支。默认值取自 `control_chain_cfg_defaults()` */
    float shank_ik_bias_deg;
    /** 前腿足端竖直偏置（mm）。注入表 `front_leg_y_offset = 0.0`。
     *  默认值取自 `control_chain_cfg_defaults()` */
    float front_leg_y_offset;
    /** 后腿足端竖直偏置（mm）。注入表 `rear_leg_y_offset = 0.0`。
     *  默认值取自 `control_chain_cfg_defaults()` */
    float rear_leg_y_offset;
    /** 每条腿的小腿微调 `leg1_s_trim`..`leg4_s_trim`（度），索引 0..3 = 腿1..腿4。
     *  注入表全 0.0。默认值取自 `control_chain_cfg_defaults()` */
    float s_trim[APP_CFG_LEGS];
    /** `leg2_z_mul`（注入表 1.0）。控制链的数学路径目前不读它，搬过来只为
     *  不把注入表"抄一半"。默认值取自 `control_chain_cfg_defaults()` */
    float leg2_z_mul;
    /** `leg3_z_mul`（注入表 1.0）。默认值取自 `control_chain_cfg_defaults()` */
    float leg3_z_mul;
    /** `leg4_z_mul`（注入表 1.0）。注入表里**没有** `leg1_z_mul`。
     *  默认值取自 `control_chain_cfg_defaults()` */
    float leg4_z_mul;
    /** WALK 相位增量倍率。注入表 `walk_speed_scale = 1.4`；
     *  `<=0.05` 时 control_chain 退回 1.0（= 用自动算出来的步频）。
     *  默认值取自 `control_chain_cfg_defaults()` */
    float walk_speed_scale;
    /** 直行 WALK 滚转微调（度）。注入表 `walk_roll_trim = 3`。
     *  默认值取自 `control_chain_cfg_defaults()` */
    float walk_roll_trim;
    /** 直行 TROT 滚转微调（度）。注入表 `trot_roll_trim = 0`。
     *  默认值取自 `control_chain_cfg_defaults()` */
    float trot_roll_trim;
    /** 右侧两条腿（腿2/腿3）摆动抬腿高度倍率。注入表 `trot_right_h_mul = 0.80`；
     *  `>=0.999` 时 control_chain 整段跳过。默认值取自
     *  `control_chain_cfg_defaults()` */
    float trot_right_h_mul;

    /* ---------- 机械臂 ---------- */
    float arm_upper_init; /**< 大臂中位，145 */
    float arm_fore_init;  /**< 小臂中位，125 */
    float arm_upper_min;  /**< 0 */
    float arm_upper_max;  /**< 180 */
    float arm_fore_min;   /**< 30 */
    float arm_fore_max;   /**< 140 */
    float arm_upper_rate; /**< 2.5 */
    float arm_fore_rate;  /**< 2.5 */
    float arm_grip_open;  /**< 夹爪开，90 */
    float arm_grip_close; /**< 夹爪合，180 */
    int32_t arm_upper_ch; /**< 6 */
    int32_t arm_fore_ch;  /**< 7 */
    int32_t arm_grip_ch;  /**< 6 */
    int32_t arm_upper_board; /**< 0x40 */
    int32_t arm_fore_board;  /**< 0x40 */
    int32_t arm_grip_board;  /**< 0x41 */
    int32_t arm_grip_gpio;   /**< -1 = 走 PCA9685；>=0 = 用该 GPIO 的 PWM */

    /* ---------- padog.py 默认值注入表：机械臂侧 ----------
     * 同样是**只存在于注入表**（`config_s.py` 没定义的）7 个键。搬过来是为了
     * `control_chain_cfg_t` 的机械臂段也能被如实填满；默认值与
     * `control_chain_cfg_defaults()` 逐项相等。 */
    /** 夹爪是否用数字电平驱动。注入表 `arm_grip_digital = 0`（0 = 走 PWM）。
     *  默认值取自 `control_chain_cfg_defaults()` */
    int32_t arm_grip_digital;
    /** 夹爪 PWM 频率（Hz）。注入表 `arm_grip_pwm_hz = 50`。
     *  默认值取自 `control_chain_cfg_defaults()` */
    int32_t arm_grip_pwm_hz;
    /** 夹爪 PWM 最小脉宽（微秒）。注入表 `arm_grip_min_us = 500`。
     *  默认值取自 `control_chain_cfg_defaults()` */
    int32_t arm_grip_min_us;
    /** 夹爪 PWM 最大脉宽（微秒）。注入表 `arm_grip_max_us = 2500`。
     *  默认值取自 `control_chain_cfg_defaults()` */
    int32_t arm_grip_max_us;
    /** WALK 时大臂角（度）。注入表 `arm_upper_walk = 145`。
     *  默认值取自 `control_chain_cfg_defaults()` */
    int32_t arm_upper_walk;
    /** WALK 时小臂角（度）。注入表 `arm_fore_walk = 125`。
     *  默认值取自 `control_chain_cfg_defaults()` */
    int32_t arm_fore_walk;
    /** WALK 时机械臂摆动速率。注入表 `arm_walk_rate = 0.15`。
     *  默认值取自 `control_chain_cfg_defaults()` */
    float arm_walk_rate;

    /* ---------- WiFi（纯 AP 模式） ---------- */
    /**
     * 纯 AP 模式：狗自己开热点，手机/电脑直连，**不依赖任何外部热点**。
     * 这样测试时"连不上"永远不是死角。
     *
     * ⚠️ 这里放的是**默认值**，不是学长 `config.py` 里的真实凭据。
     *    真实凭据属于秘密，绝不进公开仓库；需要时用控制台命令改并存入 NVS。
     */
    char ap_ssid[33];      /**< AP 名称，默认 "RobotDog" */
    char ap_password[65];  /**< AP 密码，WPA2 要求 8~63 字符 */
} app_config_t;

/**
 * @brief 填入出厂默认值（= 学长 `config.py` + `config_s.py` 的实测值）。
 *
 * @note 会清零整个结构体，包括把 `version` 设为 `APP_CFG_VERSION`、`crc32` 设为 0。
 */
void app_config_defaults(app_config_t *cfg);

/**
 * @brief 范围校验与限幅。
 *
 * @param cfg      待校验配置（**就地修改**）
 * @param changed  输出：被改动的项数（可为 NULL）
 * @param msg      输出：第一处被改动的说明（可为 NULL）
 * @param msg_len  msg 缓冲区长度
 * @return `APP_CFG_OK`；参数错误返回 `APP_CFG_ERR_ARG`
 */
int app_config_validate(app_config_t *cfg, int *changed, char *msg, size_t msg_len);

/**
 * @brief 从存储读取配置。
 *
 * 失败（不存在/CRC 错/版本不符/IO 错）时**一律回落默认值**，并把具体原因作为返回值，
 * 绝不留下一个半可信的配置。
 *
 * @return `APP_CFG_OK` 或具体的错误码（此时 cfg 已被填成默认值并通过校验）
 */
int app_config_load(app_config_t *cfg, const app_cfg_store_t *st);

/**
 * @brief 写入存储（自动更新 version 与 crc32）。
 */
int app_config_save(app_config_t *cfg, const app_cfg_store_t *st);

/**
 * @brief 删除存储中的配置并回落默认值（恢复出厂）。
 */
int app_config_reset(app_config_t *cfg, const app_cfg_store_t *st);

/** @brief 计算配置的 CRC32（计算时把 crc32 字段视为 0）。 */
uint32_t app_config_crc(const app_config_t *cfg);

/** @brief 错误码转字符串（用于日志） */
const char *app_cfg_strerror(int rc);

#ifdef __cplusplus
}
#endif
