/*
 * test_app_config.c —— 宿主侧测试：配置默认值 / 校验 / CRC / 持久化
 *
 * 关键点：app_config.c **不依赖 ESP-IDF**，存储后端是注入的。
 * 所以这里用一个 RAM 假后端，就能把"存→读→一致"、"CRC 坏了怎么办"、
 * "版本不符怎么办"、"越界值被限幅"这些逻辑全部在电脑上测完，不用烧板子。
 *
 * 真正需要上机验证的只剩一件事：**掉电重启后值还在**（那是 Flash 行为，
 * 宿主测不了，见 tools/golden/README.md 的说明）。
 *
 * NOTE: ASCII output only, for the reason recorded as P-06 in 问题与解决记录.md.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app/app_config.h"

/* ==========================================================================
 * RAM 假后端
 * ========================================================================== */

#define FAKE_MAX 4096
static uint8_t g_fake[FAKE_MAX];
static size_t  g_fake_len = 0;
static int     g_fake_present = 0;

static int fake_get(const char *key, void *out, size_t *len)
{
    (void)key;
    if (!g_fake_present) {
        *len = 0;
        return APP_CFG_ERR_NOENT;
    }
    /* 模仿真实 NVS：长度不符时回报实际长度，由上层判为"版本不符" */
    if (*len != g_fake_len) {
        *len = g_fake_len;
        if (*len <= FAKE_MAX) {
            memcpy(out, g_fake, g_fake_len);
        }
        return APP_CFG_OK;
    }
    memcpy(out, g_fake, g_fake_len);
    *len = g_fake_len;
    return APP_CFG_OK;
}

static int fake_set(const char *key, const void *data, size_t len)
{
    (void)key;
    if (len > FAKE_MAX) {
        return APP_CFG_ERR_IO;
    }
    memcpy(g_fake, data, len);
    g_fake_len = len;
    g_fake_present = 1;
    return APP_CFG_OK;
}

static int fake_erase(const char *key)
{
    (void)key;
    g_fake_present = 0;
    g_fake_len = 0;
    return APP_CFG_OK;
}

static const app_cfg_store_t FAKE = {
    .get_blob  = fake_get,
    .set_blob  = fake_set,
    .erase_key = fake_erase,
};

/* ==========================================================================
 * 极简测试框架
 * ========================================================================== */

static int g_fail = 0;
static int g_checks = 0;

static int check(int cond, const char *msg)
{
    ++g_checks;
    if (!cond) {
        ++g_fail;
        printf("  FAIL: %s\n", msg);
    }
    return cond;
}

static int feq(float a, float b)
{
    const float d = a - b;
    return (d < 0 ? -d : d) < 1e-4f;
}

#define SECTION(t) do { printf("\n-- %s\n", (t)); } while (0)

/* ==========================================================================
 * 用例
 * ========================================================================== */

static void t_defaults(void)
{
    SECTION("defaults match config.py / config_s.py");
    app_config_t c;
    app_config_defaults(&c);

    check(c.version == APP_CFG_VERSION, "version");
    check(c.crc32 == 0, "crc starts at 0");

    /* 舵机中位角：init_1p=102 init_1h=84 init_1s=92 ... */
    check(feq(c.servo_center[0][0], 102.0f), "servo_center leg1 hip = 102");
    check(feq(c.servo_center[0][1],  84.0f), "servo_center leg1 thigh = 84");
    check(feq(c.servo_center[0][2],  92.0f), "servo_center leg1 shank = 92");
    check(feq(c.servo_center[1][0],  96.0f), "servo_center leg2 hip = 96");
    check(feq(c.servo_center[2][0], 108.0f), "servo_center leg3 hip = 108");
    check(feq(c.servo_center[3][2], 102.0f), "servo_center leg4 shank = 102");

    check(feq(c.l1, 130.0f) && feq(c.l2, 138.0f), "l1/l2 = 130/138");
    check(feq(c.l, 230.0f) && feq(c.b, 120.0f) && feq(c.w, 220.0f), "l/b/w = 230/120/220");
    check(feq(c.leg_len_ref, 149.0f), "leg_len_ref = 149");

    check(feq(c.h_goal, 81.0f) && feq(c.in_y, 18.0f), "h_goal=81 in_y=18");
    check(feq(c.cg_y, 28.0f) && feq(c.cg_x, 0.0f), "cg = 0/28");
    check(feq(c.kp_h, 0.06f) && feq(c.kp_g, 0.03f), "kp_h/kp_g = 0.06/0.03");
    check(feq(c.pit_max_ang, 15.0f) && feq(c.rol_max_ang, 15.0f), "max_ang = 15");
    check(feq(c.xs_max, 80.0f), "xs_max = 80");

    check(feq(c.ts, 1.0f), "ts = 1.0");
    check(feq(c.faai, 0.42f),
          "faai = 0.42 (config_s overrides config.py's 0.5)");
    check(feq(c.walk_faai, 0.30f), "walk_faai = 0.30");
    check(feq(c.trot_cg_f, 0.52f) && feq(c.trot_cg_b, 0.85f), "trot_cg = 0.52/0.85");
    check(c.joy_fwd_sign == -1, "joy_fwd_sign = -1");
    check(c.ma_case == 0, "ma_case = 0 (series leg)");
    check(c.cal_leg_sel == 2, "cal_leg_sel = 2");

    check(feq(c.arm_upper_init, 145.0f) && feq(c.arm_fore_init, 125.0f), "arm init");
    check(c.arm_upper_ch == 6 && c.arm_fore_ch == 7 && c.arm_grip_ch == 6, "arm ch");
    check(c.arm_upper_board == 0x40 && c.arm_fore_board == 0x40
          && c.arm_grip_board == 0x41, "arm boards");
    check(c.arm_grip_gpio == -1, "arm_grip_gpio = -1 (via PCA9685)");

    check(strcmp(c.ap_ssid, "RobotDog") == 0, "ap_ssid default");
    check(strlen(c.ap_password) >= 8, "ap_password >= 8 chars (WPA2)");
    check(strstr(c.ap_password, "ak12345678") == NULL,
          "default AP password is NOT the retired real hotspot password");

    printf("  sizeof(app_config_t) = %u\n", (unsigned)sizeof(app_config_t));
    printf("  checks=%d fail=%d\n", g_checks, g_fail);
}

static void t_validate(void)
{
    SECTION("validate clamps out-of-range values");
    app_config_t c;
    app_config_defaults(&c);

    c.servo_center[0][0] = 999.0f;    /* > 180 */
    c.servo_center[1][1] = -50.0f;    /* < 0 */
    c.pit_max_ang = 500.0f;           /* > 90 */
    c.faai = 5.0f;                    /* > 0.95 */
    c.faai = 5.0f;
    c.ts = 0.0f;                      /* < 0.1 */
    c.ma_case = 7;                    /* > 1 */
    c.cal_leg_sel = 99;               /* > 4 */
    c.arm_upper_board = 0x7F;         /* 不是 0x40/0x41 */
    c.arm_grip_gpio = -99;            /* < -1 */
    c.ap_password[0] = 'x';           /* 太短 */
    c.ap_password[1] = '\0';
    strcpy(c.ap_ssid, "");            /* 空 SSID */

    int changed = 0;
    char msg[128];
    const int rc = app_config_validate(&c, &changed, msg, sizeof(msg));

    check(rc == APP_CFG_OK, "validate returns OK");
    check(changed > 0, "validate reports changes");
    check(msg[0] != '\0', "validate fills a message");
    printf("  changed=%d first=\"%s\"\n", changed, msg);

    check(feq(c.servo_center[0][0], 180.0f), "servo_center clamped to 180");
    check(feq(c.servo_center[1][1], 0.0f), "servo_center clamped to 0");
    check(feq(c.pit_max_ang, 90.0f), "pit_max_ang clamped to 90");
    check(feq(c.faai, 0.95f), "faai clamped to 0.95");
    check(feq(c.ts, 0.1f), "ts clamped to 0.1");
    check(c.ma_case == 1, "ma_case clamped to 1");
    check(c.cal_leg_sel == 4, "cal_leg_sel clamped to 4");
    check(c.arm_upper_board == 0x40, "bad board address falls back to 0x40");
    check(c.arm_grip_gpio == -1, "arm_grip_gpio clamped to -1");
    check(strlen(c.ap_password) >= 8, "short AP password replaced");
    check(strcmp(c.ap_ssid, "RobotDog") == 0, "empty SSID replaced");

    /* in_pit/in_rol 依赖 pit_max_ang，限幅后不应超出 */
    c.in_pit = 999.0f;
    app_config_validate(&c, NULL, NULL, 0);
    check(c.in_pit <= c.pit_max_ang, "in_pit clamped against pit_max_ang");

    /* min > max 应被交换 */
    c.arm_fore_min = 170.0f;
    c.arm_fore_max = 20.0f;
    app_config_validate(&c, NULL, NULL, 0);
    check(c.arm_fore_min <= c.arm_fore_max, "arm min/max swapped when inverted");

    printf("  checks=%d fail=%d\n", g_checks, g_fail);
}

static void t_roundtrip(void)
{
    SECTION("save -> load round trip");
    app_config_t a, b;
    app_config_defaults(&a);

    /* 改几个值，模拟"标定后保存" */
    a.servo_center[0][0] = 111.0f;
    a.servo_center[3][2] = 77.0f;
    a.h_goal = 70.0f;
    a.faai = 0.37f;
    strcpy(a.ap_ssid, "MyDog");

    check(app_config_save(&a, &FAKE) == APP_CFG_OK, "save ok");
    check(a.crc32 != 0, "save computed a nonzero CRC");

    memset(&b, 0xAA, sizeof(b));   /* 故意搞脏，确认 load 真的覆盖了 */
    const int rc = app_config_load(&b, &FAKE);
    check(rc == APP_CFG_OK, "load ok");
    check(feq(b.servo_center[0][0], 111.0f), "servo_center[0][0] preserved");
    check(feq(b.servo_center[3][2],  77.0f), "servo_center[3][2] preserved");
    check(feq(b.h_goal, 70.0f), "h_goal preserved");
    check(feq(b.faai, 0.37f), "faai preserved");
    check(strcmp(b.ap_ssid, "MyDog") == 0, "ap_ssid preserved");
    check(b.crc32 == a.crc32, "CRC stable across save/load");

    /* CRC 必须只取决于内容 */
    app_config_t c1, c2;
    app_config_defaults(&c1);
    app_config_defaults(&c2);
    check(app_config_crc(&c1) == app_config_crc(&c2), "CRC deterministic");
    c2.h_goal += 1.0f;
    check(app_config_crc(&c1) != app_config_crc(&c2), "CRC changes with content");

    printf("  checks=%d fail=%d\n", g_checks, g_fail);
}

static void t_missing(void)
{
    SECTION("no saved config -> defaults + NOENT");
    fake_erase(APP_CFG_NVS_KEY);
    app_config_t c;
    memset(&c, 0xAA, sizeof(c));
    const int rc = app_config_load(&c, &FAKE);
    check(rc == APP_CFG_ERR_NOENT, "returns ERR_NOENT");
    check(feq(c.h_goal, 81.0f), "defaults applied");
    check(c.version == APP_CFG_VERSION, "version set");
    printf("  checks=%d fail=%d\n", g_checks, g_fail);
}

static void t_corrupt(void)
{
    SECTION("corrupted CRC -> defaults + CRC error");
    app_config_t a;
    app_config_defaults(&a);
    a.h_goal = 123.0f;
    app_config_save(&a, &FAKE);

    /* 破坏载荷里一个字节（不动 crc 字段本身） */
    g_fake[8] ^= 0xFF;

    app_config_t b;
    const int rc = app_config_load(&b, &FAKE);
    check(rc == APP_CFG_ERR_CRC, "returns ERR_CRC");
    check(feq(b.h_goal, 81.0f), "fell back to default h_goal (not the corrupted 123)");
    printf("  checks=%d fail=%d\n", g_checks, g_fail);
}

static void t_version(void)
{
    SECTION("version mismatch -> defaults + VERSION error");
    app_config_t a;
    app_config_defaults(&a);
    a.h_goal = 123.0f;
    app_config_save(&a, &FAKE);

    /* 篡改版本号（并重算 CRC，模拟"结构升级了"） */
    app_config_t *stored = (app_config_t *)g_fake;
    stored->version = APP_CFG_VERSION + 1;
    stored->crc32 = 0;
    stored->crc32 = app_config_crc(stored);

    app_config_t b;
    const int rc = app_config_load(&b, &FAKE);
    check(rc == APP_CFG_ERR_VERSION, "returns ERR_VERSION");
    check(feq(b.h_goal, 81.0f), "fell back to defaults");
    printf("  checks=%d fail=%d\n", g_checks, g_fail);
}

static void t_reset(void)
{
    SECTION("reset -> erase + defaults");
    app_config_t a, b;
    app_config_defaults(&a);
    a.h_goal = 55.0f;
    app_config_save(&a, &FAKE);

    check(app_config_reset(&b, &FAKE) == APP_CFG_OK, "reset ok");
    check(g_fake_present == 0, "storage erased");
    check(feq(b.h_goal, 81.0f), "cfg back to defaults");

    /* 擦除后再 load 应该是 NOENT */
    app_config_t c;
    check(app_config_load(&c, &FAKE) == APP_CFG_ERR_NOENT, "load after reset = NOENT");
    printf("  checks=%d fail=%d\n", g_checks, g_fail);
}

int main(void)
{
    printf("========================================================\n");
    printf(" app_config host test  (no board needed)\n");
    printf("========================================================\n");

    t_defaults();
    t_validate();
    t_roundtrip();
    t_missing();
    t_corrupt();
    t_version();
    t_reset();

    printf("\n========================================================\n");
    printf(" checks=%d  failures=%d\n", g_checks, g_fail);
    if (g_fail > 0) {
        printf("RESULT: FAIL -- %d checks failed\n", g_fail);
        return 1;
    }
    printf("RESULT: PASS -- all %d checks passed\n", g_checks);
    printf("\nNOTE: 'survives a power cycle' cannot be tested here -- that is\n");
    printf("      Flash behaviour and is verified on the board.\n");
    return 0;
}
