/**
 * @file    comm/net.c
 * @brief   P5 传输层实现（WiFi 纯 AP + HTTP 服务 + 命令任务）—— 见 `net.h` 的数据流图
 *
 * 本文件**只在固件里编译**（用了 ESP-IDF 的 WiFi / HTTP / FreeRTOS）。
 * 协议解析、邮箱策略、摇杆翻译三件事都在纯 C 模块里，那些才是被逐值验证过的部分。
 */

#include "comm/net.h"

#include <stdio.h>
#include <string.h>

#include "app/app_action.h"
#include "app/app_chain.h"
#include "app/app_cfg_cmd.h"
#include "app/motion.h"
#include "comm/cmd_queue.h"
#include "comm/pages_gen.h"
#include "comm/proto.h"
#include "comm/web_cmd.h"
#include "control/control_chain.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "net";

/* 阈值只在本文件出现一次；`cmd_queue_init()` 是权威来源（见 cmd_queue.h 的说明） */
#define NET_HTTP_PORT        80
#define NET_AP_CHANNEL       1
#define NET_AP_MAX_CONN      4
#define NET_COMM_TASK_HZ     100
#define NET_MOTION_BACKSTOP_MS 3000u   /**< 故意**长于**长超时：只兜底"comm 任务自己卡死" */
#define NET_MIN_INTERVAL_MS  10u       /**< 100 Hz 上限（§7 的目标是 20~50 Hz）*/

/* ==========================================================================
 * 状态
 * ======================================================================== */

static cmd_queue_t      s_q;
static proto_decoder_t  s_dec;
static web_cmd_ctx_t    s_wctx;
static control_chain_cfg_t s_cfg;      /**< `turn_phase_lr` 要用（`hip_turn_dead`）*/
static SemaphoreHandle_t s_lock;       /**< 保护 `s_q` / `s_dec`（httpd 与 comm 任务之间）*/
static httpd_handle_t   s_srv;
static esp_netif_t     *s_ap_netif;
static TaskHandle_t     s_task;
static volatile bool    s_up;

/* 字段下标（在 `net_start()` 里用**协议层的表**查出来，不自己写死数字）*/
static int s_f_mode = -1, s_f_spd = -1, s_f_turn = -1, s_f_hgt = -1;
static int s_f_pit = -1, s_f_rol = -1, s_f_yst = -1;

static bool present(const proto_cmd_t *c, int idx)
{
    return (idx >= 0) && ((c->present & (1u << (unsigned)idx)) != 0u);
}

/* ==========================================================================
 * web_cmd 的函数指针表 —— 每一项都落到已经验证过的 app_chain 接口上
 * ======================================================================== */

static void cb_set_leg_sit_offsets(void *user, float front_y, float rear_y)
{
    (void)user;
    (void)app_chain_set_sit_offsets(front_y, rear_y);
}

static void cb_move(void *user, float spd, int L, int R)
{
    (void)user;
    (void)app_chain_jog(spd, L, R);
}

static void cb_drive(void *user, float spd, int L, int R)
{
    (void)user;
    (void)app_chain_drive(spd, L, R);
}

static void cb_set_joy_turn(void *user, float pct)
{
    (void)user;
    (void)app_chain_set_joy_turn(pct);
}

static void cb_turn_phase_lr(void *user, float joy_turn, int *out_l, int *out_r)
{
    (void)user;
    control_chain_turn_phase_lr(&s_cfg, joy_turn, out_l, out_r);
}

static int cb_gait_mode(void *user)
{
    (void)user;
    app_chain_status_t st;
    memset(&st, 0, sizeof(st));
    app_chain_get_status(&st);
    return st.gait_mode;
}

static int cb_crawl_phase(void *user)
{
    (void)user;
    int ph = 0;
    app_chain_get_crawl(&ph, NULL, NULL);
    return ph;
}

static int32_t cb_inplace_step_end_ms(void *user)
{
    (void)user;
    return app_action_get_inplace_step_end_ms();
}

/* ==========================================================================
 * 事件
 * ======================================================================== */

/**
 * @brief 处理一个事件名（名字来自**协议层唯一的事件表**，本函数只做名→动作的映射）。
 *
 * ⚠️ `go` / `gc` 必须**明确拒绝**并说清原因 —— §8.7 专门禁止"`stable()` 变成一个
 * 无效的布尔开关"，§0.5(4).1 要求上层显式回"不支持：无 IMU"。
 * 机械臂（`am1`/`am0`）同理：P6 没做，不许假装成功。
 */
static void handle_event(const char *name)
{
    if (strcmp(name, "g0") == 0) {
        (void)app_chain_set_gait(0);
    } else if (strcmp(name, "g1") == 0) {
        (void)app_chain_set_gait(1);
    } else if (strcmp(name, "is") == 0) {
        /* 网页"步态测试"：原版是 `gait(0)` + 写 inplace 截止时刻 */
        (void)app_chain_set_gait(0);
        (void)app_action_set_inplace_step_end_ms(
            (int32_t)((int64_t)(esp_timer_get_time() / 1000) + 5000));
    } else if (strcmp(name, "btn_stand") == 0) {
        (void)app_action_request(APP_ACTION_STAND);
    } else if (strcmp(name, "btn_sit") == 0) {
        (void)app_action_request(APP_ACTION_SIT);
    } else if (strcmp(name, "btn_wave") == 0) {
        (void)app_action_request(APP_ACTION_WAVE);
    } else if (strcmp(name, "btn_crawl") == 0) {
        /*
         * ⚠️ 爬行的**执行**在控制链里（`chain_crawl_service`），而 `motion` 的模式是互斥的
         * —— ACTION 模式不调 `app_chain_step()`。所以这里**必须**切到 CHAIN 模式，
         * 和别的动作恰好相反（见 `app_motion_cmd.c` 里同样的警告）。
         */
        (void)motion_set_mode(MOTION_MODE_CHAIN);
        (void)app_action_request(APP_ACTION_CRAWL);
    } else if (strcmp(name, "ss") == 0) {
        /* ⚠️ `ss` **不是停车**！它是"清姿态 + 进标定页"（§0.5(3)）。
           页面跳转是 Web 层的事，这里只做机器人侧那三件事。 */
        (void)app_chain_set_gait(0);
        (void)app_chain_gesture(0.0f, 0.0f, 0.0f);
        ESP_LOGI(TAG, "ss：已清姿态与步态（页面跳转由浏览器负责，C 版没有页面状态）");
    } else if (strcmp(name, "t9") == 0) {
        /* = `servo_init(1)`：切"直接站姿"（就是 P2 的 `stand direct`）*/
        (void)app_chain_set_init_case(1);
    } else if (strcmp(name, "go") == 0 || strcmp(name, "gc") == 0) {
        ESP_LOGW(TAG, "事件 '%s'（陀螺仪稳定）**不支持**：本机没有 IMU（§8.7 禁止假开关）",
                 name);
    } else if (strcmp(name, "am1") == 0 || strcmp(name, "am0") == 0 ||
               strcmp(name, "sc") == 0) {
        ESP_LOGW(TAG, "事件 '%s' 需要 P6 / 配置通道，本阶段未接线（见迁移表 §0.5(10)）",
                 name);
    } else if (name[0] == 'l' && name[1] >= '1' && name[1] <= '4' && name[2] == '\0') {
        ESP_LOGW(TAG, "标定选腿 '%s' 需要配置写入通道，本阶段未接线；请用串口 "
                      "`cfg set cal_leg_sel %c`", name, name[1]);
    } else if (strcmp(name, "hi") == 0 || strcmp(name, "hd") == 0 ||
               strcmp(name, "si") == 0 || strcmp(name, "sd") == 0 ||
               strcmp(name, "ip") == 0 || strcmp(name, "id") == 0) {
        ESP_LOGW(TAG, "中位角微调 '%s' 需要配置写入通道，本阶段未接线；见迁移表 §0.5(5)",
                 name);
    } else {
        /* 协议层保证了名字在白名单里，走到这里说明"表里有、这里没接" —— 必须报出来 */
        ESP_LOGW(TAG, "事件 '%s' 已在协议白名单里，但本阶段没有接线", name);
    }
}

static void apply_events(const proto_cmd_t *c)
{
    const size_t n = proto_cmd_ev_count(c);
    for (size_t i = 0; i < n; ++i) {
        const char *nm = proto_event_at((int)i);
        if (nm != NULL && proto_cmd_has_ev_index(c, (int)i)) {
            handle_event(nm);
        }
    }
}

/* ==========================================================================
 * 应用一条新鲜命令
 * ======================================================================== */

static void apply_fresh(const proto_cmd_t *c, uint32_t now_ms)
{
    const uint32_t mode_now = motion_get_mode();

    /*
     * §0.5(6).1 的明文规则：**动作期间忽略摇杆、步态、姿态、高度**，
     * 但"动作事件"照收（用户可以按另一个动作），**急停永远有效**（在下面单独处理）。
     *
     * 为什么必须这样：原版挥手是 3.4 秒的完全死区（连 HTTP 都不应答），
     * 那时根本收不到摇杆；C 版动作层不阻塞，所以"收得到"是新出现的情况，
     * 必须显式规定怎么办 —— 否则一个 80 ms 轮询的页面会一边挥手一边把腿摇起来。
     */
    if (mode_now == MOTION_MODE_ACTION) {
        apply_events(c);
        return;
    }

    apply_events(c);

    if (present(c, s_f_mode)) {
        if (c->mode >= 0 && c->mode <= 2) {
            (void)motion_set_mode((uint32_t)c->mode);
        }
    }

    /*
     * 姿态 / 高度 / 重心横向：只有**这一帧真的命令了它们**才动（旧格式靠 `present` 位，
     * 严格格式永远写全字段 ⇒ 等于每次都是全量命令）。
     */
    if (present(c, s_f_hgt)) {
        (void)app_chain_set_height((float)c->hgt);
    }
    if (present(c, s_f_pit) || present(c, s_f_rol) || present(c, s_f_yst)) {
        (void)app_chain_gesture((float)c->pit, (float)c->rol, (float)c->yst);
    }

    /* 摇杆：只有请求里出现过 f/t 才处理（原版 `if req_data.find('f=') >= 0`）*/
    if (present(c, s_f_spd) || present(c, s_f_turn)) {
        const bool inplace_was_active =
            ((int32_t)now_ms < app_action_get_inplace_step_end_ms());

        float thr = 0.0f;
        (void)web_cmd_process_request(&s_wctx,
                                      (int)c->spd, present(c, s_f_spd) ? 1 : 0,
                                      (int)c->turn, present(c, s_f_turn) ? 1 : 0,
                                      (int32_t)now_ms,
                                      1 /* force：轻量页传 dog_when_arm=True */,
                                      0 /* arm_enabled：P6，本机恒为关 */,
                                      &thr);

        /*
         * 复刻原版的"网页原地步态测试**只生效一帧**"（迁移表 §0.5(8b)）。
         *
         * 原版里 `move(4,1,1)` 自己会把模块级全局 `inplace_step_end_ms` 清零；
         * C 版那个截止时刻归**动作层**，而链的 move 路径清的是**链自己那份**，
         * 于是没人清它 ⇒ 页面每轮询一次这个分支就再触发一次。
         * 这里按原版补上：**刚走过原地踏步分支就把它清掉**。
         */
        if (inplace_was_active) {
            (void)app_action_set_inplace_step_end_ms(0);
        }
    }
}

/* ==========================================================================
 * 命令任务：消费邮箱，执行两级超时策略
 * ======================================================================== */

static void comm_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / NET_COMM_TASK_HZ);

    uint32_t last_applied_seq = 0;
    bool     has_applied = false;

    for (;;) {
        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        proto_cmd_t cmd;
        memset(&cmd, 0, sizeof(cmd));
        bool changed = false;
        cmd_queue_state_t st;

        if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
            st = cmd_queue_tick(&s_q, now_ms, &cmd, &changed);
            xSemaphoreGive(s_lock);
        } else {
            st = CMD_QUEUE_IDLE;
        }

        switch (st) {
        case CMD_QUEUE_FRESH:
            if (cmd.est != 0) {
                motion_estop("网络急停");
                break;
            }
            /* 只在**换了新命令**时应用，避免 100 Hz 重复下发同一条 */
            if (!has_applied || cmd.seq != last_applied_seq) {
                last_applied_seq = cmd.seq;
                has_applied = true;
                motion_keepalive();       /* "客户端活着"的唯一证据，别在别处调 */
                apply_fresh(&cmd, now_ms);
            }
            break;

        case CMD_QUEUE_HOLD:
            if (changed) {
                /*
                 * 短超时：**输入归零、保持姿态**（= `btn_stop` 语义，不是放松）。
                 * 动作期间不动手（§0.5(6).2）—— 挥手演到一半被打断会停在
                 * "前腿抬着后腿站着"的中间姿态上。
                 */
                ESP_LOGW(TAG, "心跳超时（>%u ms）：输入归零，保持姿态",
                         (unsigned)s_q.hb_ms);
                if (motion_get_mode() == MOTION_MODE_CHAIN) {
                    (void)app_chain_set_joy_turn(0.0f);
                    (void)app_chain_jog(0.0f, 0, 0);
                }
            }
            break;

        case CMD_QUEUE_RELAX:
            if (changed) {
                if (cmd.est != 0) {
                    ESP_LOGE(TAG, "急停：放松舵机");
                    motion_estop("网络急停");
                } else {
                    ESP_LOGE(TAG, "断连超过 %u ms：放松舵机（安全态）",
                             (unsigned)s_q.long_ms);
                    motion_stop(MOTION_STOP_TIMEOUT);
                }
                has_applied = false;     /* 下次连上要重新应用一次 */
            }
            break;

        case CMD_QUEUE_IDLE:
        default:
            break;
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/* ==========================================================================
 * HTTP
 * ======================================================================== */

static esp_err_t send_page(httpd_req_t *req, const char *page, size_t len)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, page, (ssize_t)len);
}

static esp_err_t send_204(httpd_req_t *req)
{
    /* 原版数据请求回 204 + keep-alive（`web_ctl.py` 的 `_HDR_204`）*/
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t send_status(httpd_req_t *req)
{
    char buf[768];
    cmd_queue_counters_t qc;
    cmd_queue_state_t    qst;
    motion_stats_t       ms;

    if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
        cmd_queue_get_counters(&s_q, &qc);
        qst = s_q.last_state;
        xSemaphoreGive(s_lock);
    } else {
        memset(&qc, 0, sizeof(qc));
        qst = CMD_QUEUE_IDLE;
    }
    motion_get_stats(&ms);

    const int n = snprintf(
        buf, sizeof(buf),
        "ip=%s clients=%u up=%d\n"
        "mode=%s running=%d estopped=%d stop_reason=%s\n"
        "link=%s age_ms=%u hb_ms=%u long_ms=%u\n"
        "q: accepted=%u rejected=%u dropped_seq=%u hold=%u relax=%u estop=%u ticks=%u\n"
        "motion: ticks=%u overruns=%u period_us=%lld work_us=%lld i2c_writes=%u\n",
        net_ip_str(), (unsigned)net_client_count(), s_up ? 1 : 0,
        motion_mode_name(ms.mode), ms.running ? 1 : 0, ms.estopped ? 1 : 0,
        motion_stop_reason_str(ms.stop_reason),
        cmd_queue_state_name(qst), (unsigned)cmd_queue_age_ms(&s_q, (uint32_t)(esp_timer_get_time() / 1000)),
        (unsigned)s_q.hb_ms, (unsigned)s_q.long_ms,
        (unsigned)qc.accepted, (unsigned)qc.rejected, (unsigned)qc.dropped_seq,
        (unsigned)qc.hold_entries, (unsigned)qc.relax_entries,
        (unsigned)qc.estop_latched, (unsigned)qc.ticks,
        (unsigned)ms.ticks, (unsigned)ms.overruns,
        (long long)ms.period_last_us, (long long)ms.work_last_us,
        (unsigned)ms.i2c_writes);

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_send(req, buf, (ssize_t)n);
}

/**
 * @brief 把一段字节交给协议层；解析成功就入队。返回是否被接受。
 *
 * ⚠️ 顺序很重要：**先 peek 急停**，再做完整校验。理由是急停必须在"整帧因别的
 * 字段越界而被拒"时也能生效（§0.5(7)）。peek 只会往安全侧失败（假 1 → 停）。
 */
static bool feed_bytes(const char *buf, size_t len)
{
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    proto_cmd_t cmd;
    proto_error_t err;
    int rc;

    memset(&cmd, 0, sizeof(cmd));
    memset(&err, 0, sizeof(err));

    int est = 0;
    if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
        if (proto_peek_estop(&s_dec, buf, len, &est) != 0 && est != 0) {
            cmd_queue_latch_estop(&s_q, true);
        }

        const proto_fmt_t fmt = proto_sniff_format(buf, len);
        if (fmt == PROTO_FMT_STRICT) {
            rc = proto_decode(&s_dec, buf, len, now_ms, &cmd, &err);
        } else {
            rc = proto_decode_legacy(&s_dec, buf, len, now_ms, &cmd, &err);
        }

        if (rc != PROTO_OK) {
            cmd_queue_count_reject(&s_q);
            xSemaphoreGive(s_lock);
            ESP_LOGW(TAG, "命令被拒: %s（字段 %d, 键 '%s'）",
                     proto_err_name(err.code), err.field, err.key);
            return false;
        }

        const bool ok = cmd_queue_post(&s_q, &cmd, now_ms);
        xSemaphoreGive(s_lock);
        return ok;
    }
    return false;
}

/** 处理 `GET /<...>`：路径里带 `=` 的是命令，否则是页面请求（对应原版 `_wants_page()`）*/
static esp_err_t handler_get(httpd_req_t *req)
{
    const char *uri = req->uri;

    if (strstr(uri, "favicon.ico") != NULL) {
        return send_204(req);
    }
    if (strcmp(uri, "/status") == 0) {
        return send_status(req);
    }
    if (strcmp(uri, "/") == 0 || strcmp(uri, "/index.html") == 0 ||
        strcmp(uri, "/drive.html") == 0) {
        return send_page(req, kPageDrive, kPageDrive_LEN);
    }
    if (strcmp(uri, "/control.html") == 0) {
        return send_page(req, kPageControl, kPageControl_LEN);
    }
    if (strcmp(uri, "/cal.html") == 0) {
        return send_page(req, kPageCal, kPageCal_LEN);
    }

    /*
     * 其余一律当命令。
     *
     * ⚠️ 原版页面的报文是**不标准**的：`GET /f=10t=-5` —— 参数在**路径**里，
     * 没有 `?`、两个键之间连分隔符都没有（见迁移表 §0.5(9)）。
     * 所以这里把 path 里开头的 `/` 和 `?` 都剥掉，剩下的整段交给协议层的
     * 旧格式分词器（它认得以"下一个键"为分隔符的那种写法）。
     */
    const char *body = uri;
    while (*body == '/' || *body == '?') {
        ++body;
    }
    if (*body == '\0') {
        return send_page(req, kPageDrive, kPageDrive_LEN);
    }

    (void)feed_bytes(body, strlen(body));
    return send_204(req);
}

/** `POST /cmd`：新格式走这里（严格帧带 `seq`/`est`，有重放保护）*/
static esp_err_t handler_post(httpd_req_t *req)
{
    /* 非 static：httpd 若将来起多个任务也不会互相踩（代价只是栈上 512 字节）*/
    char buf[PROTO_MAX_FRAME];
    if (req->content_len <= 0 || req->content_len > (int)sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad length");
        return ESP_FAIL;
    }
    int got = 0;
    while (got < req->content_len) {
        const int r = httpd_req_recv(req, buf + got, req->content_len - got);
        if (r <= 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recv failed");
            return ESP_FAIL;
        }
        got += r;
    }

    const bool ok = feed_bytes(buf, (size_t)got);
    if (!ok) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "rejected\n", 9);
    }
    return send_204(req);
}

static esp_err_t http_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = NET_HTTP_PORT;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.max_uri_handlers = 8;
    cfg.lru_purge_enable = true;
    cfg.stack_size = 6144;   /* handler_post 在栈上放 512 字节的帧缓冲 */

    esp_err_t err = httpd_start(&s_srv, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start 失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 一条通配 GET 覆盖"页面 + 命令 + 状态"；一条 POST 给新格式 */
    const httpd_uri_t u_get  = { .uri = "/*",   .method = HTTP_GET,  .handler = handler_get };
    const httpd_uri_t u_post = { .uri = "/cmd", .method = HTTP_POST, .handler = handler_post };
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_srv, &u_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_srv, &u_post));
    return ESP_OK;
}

/* ==========================================================================
 * WiFi 纯 AP
 * ======================================================================== */

static esp_err_t ap_start(const app_config_t *cfg)
{
    ESP_ERROR_CHECK(esp_netif_init());
    if (esp_event_loop_create_default() != ESP_OK) {
        ESP_LOGW(TAG, "默认事件循环已存在，继续");
    }
    s_ap_netif = esp_netif_create_default_wifi_ap();
    if (s_ap_netif == NULL) {
        ESP_LOGE(TAG, "创建 AP netif 失败");
        return ESP_FAIL;
    }

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    wifi_config_t wc;
    memset(&wc, 0, sizeof(wc));
    const size_t ssid_len = strlen(cfg->ap_ssid);
    const size_t pass_len = strlen(cfg->ap_password);
    if (ssid_len == 0 || ssid_len > sizeof(wc.ap.ssid)) {
        ESP_LOGE(TAG, "AP SSID 非法（长度 %u）", (unsigned)ssid_len);
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(wc.ap.ssid, cfg->ap_ssid, ssid_len);
    wc.ap.ssid_len = (uint8_t)ssid_len;
    wc.ap.channel = NET_AP_CHANNEL;
    wc.ap.max_connection = NET_AP_MAX_CONN;

    if (pass_len == 0) {
        wc.ap.authmode = WIFI_AUTH_OPEN;
        ESP_LOGW(TAG, "AP 无密码（开放网络）");
    } else if (pass_len < 8) {
        /* WPA2 要求 8~63；太短就退回开放并**明确告警**，不假装加密了 */
        wc.ap.authmode = WIFI_AUTH_OPEN;
        ESP_LOGW(TAG, "AP 密码只有 %u 位（WPA2 要求 ≥8），已按**开放网络**启动",
                 (unsigned)pass_len);
    } else {
        memcpy(wc.ap.password, cfg->ap_password, pass_len);
        wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* 打印实际生效的 IP（不要假设一定是 192.168.4.1）*/
    esp_netif_ip_info_t ip;
    memset(&ip, 0, sizeof(ip));
    if (esp_netif_get_ip_info(s_ap_netif, &ip) == ESP_OK) {
        ESP_LOGI(TAG, "AP 已启动: ssid=\"%s\" 加密=%s 信道=%u",
                 cfg->ap_ssid,
                 (wc.ap.authmode == WIFI_AUTH_OPEN) ? "OPEN" : "WPA2",
                 (unsigned)wc.ap.channel);
        ESP_LOGI(TAG, "  浏览器打开: http://" IPSTR "/", IP2STR(&ip.ip));
        ESP_LOGI(TAG, "  状态页    : http://" IPSTR "/status", IP2STR(&ip.ip));
    }
    return ESP_OK;
}

uint32_t net_client_count(void)
{
    wifi_sta_list_t list;
    memset(&list, 0, sizeof(list));
    if (esp_wifi_ap_get_sta_list(&list) == ESP_OK) {
        return (uint32_t)list.num;
    }
    return 0;
}

const char *net_ip_str(void)
{
    static char s_ip[20] = "-";
    if (s_ap_netif == NULL) {
        return "-";
    }
    esp_netif_ip_info_t ip;
    memset(&ip, 0, sizeof(ip));
    if (esp_netif_get_ip_info(s_ap_netif, &ip) == ESP_OK) {
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&ip.ip));
    }
    return s_ip;
}

bool net_is_up(void)
{
    return s_up;
}

/* ==========================================================================
 * 启动 / 停止
 * ======================================================================== */

esp_err_t net_start(void)
{
    if (s_up) {
        return ESP_OK;
    }

    const app_config_t *c = app_cfg_cmd_get();

    /* 阈值只有一处：先定邮箱的，再推给协议层解码器（它内部会各自钳到合法区间）*/
    cmd_queue_init(&s_q, CMD_QUEUE_DEFAULT_HB_MS, CMD_QUEUE_DEFAULT_LONG_MS);
    proto_decoder_init(&s_dec);
    (void)proto_decoder_set_hb_ms(&s_dec, s_q.hb_ms);
    (void)proto_decoder_set_long_ms(&s_dec, s_q.long_ms);
    (void)proto_decoder_set_min_interval_ms(&s_dec, NET_MIN_INTERVAL_MS);

    /* 字段下标从**协议层的表**查，不写死数字 */
    s_f_mode = proto_field_find("mode");
    s_f_spd  = proto_field_find("spd");
    s_f_turn = proto_field_find("turn");
    s_f_hgt  = proto_field_find("hgt");
    s_f_pit  = proto_field_find("pit");
    s_f_rol  = proto_field_find("rol");
    s_f_yst  = proto_field_find("yst");
    if (s_f_mode < 0 || s_f_spd < 0 || s_f_turn < 0 || s_f_hgt < 0 ||
        s_f_pit < 0 || s_f_rol < 0 || s_f_yst < 0) {
        ESP_LOGE(TAG, "协议字段表里缺少本层需要的键 —— proto.c 与 net.c 不同步");
        return ESP_ERR_NOT_FOUND;
    }

    /* web_cmd 的函数指针表 */
    web_cmd_ctx_init(&s_wctx);
    s_wctx.set_leg_sit_offsets = cb_set_leg_sit_offsets;
    s_wctx.move                = cb_move;
    s_wctx.drive               = cb_drive;
    s_wctx.set_joy_turn        = cb_set_joy_turn;
    s_wctx.turn_phase_lr       = cb_turn_phase_lr;
    s_wctx.gait_mode           = cb_gait_mode;
    s_wctx.crawl_phase         = cb_crawl_phase;
    s_wctx.inplace_step_end_ms = cb_inplace_step_end_ms;
    s_wctx.user                = NULL;

    /* `turn_phase_lr` 要 cfg（`hip_turn_dead`）—— 从当前配置映射一份 */
    app_chain_cfg_from_app_config(c, &s_cfg);

    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        ESP_LOGE(TAG, "建互斥锁失败");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = ap_start(c);
    if (err != ESP_OK) {
        return err;
    }
    err = http_start();
    if (err != ESP_OK) {
        return err;
    }

    /*
     * motion 自己的命令超时当**兜底**：一定要**长于** cmd_queue 的长超时，
     * 这样正常路径上永远是 cmd_queue 先决定"放松"，motion 的兜底只在
     * comm 任务自己卡死时才会触发（见 net.h 里的顺序说明）。
     */
    (void)motion_set_timeout_ms(NET_MOTION_BACKSTOP_MS);

    if (xTaskCreate(comm_task, "net_cmd", 4096, NULL, 6, &s_task) != pdPASS) {
        ESP_LOGE(TAG, "建命令任务失败");
        return ESP_ERR_NO_MEM;
    }

    s_up = true;
    ESP_LOGI(TAG, "传输层就绪：httpd :%d，命令任务 %d Hz，心跳 %u/%u ms，motion 兜底 %u ms",
             NET_HTTP_PORT, NET_COMM_TASK_HZ,
             (unsigned)s_q.hb_ms, (unsigned)s_q.long_ms,
             (unsigned)NET_MOTION_BACKSTOP_MS);
    return ESP_OK;
}

void net_stop(void)
{
    if (s_srv != NULL) {
        httpd_stop(s_srv);
        s_srv = NULL;
    }
    s_up = false;
}

/* ==========================================================================
 * 控制台：net status | net counters
 * ======================================================================== */

void net_cmd_handle(const char *sub, const char *args)
{
    (void)args;

    if (sub == NULL || strcmp(sub, "status") == 0) {
        if (!s_up) {
            ESP_LOGW(TAG, "传输层未启动");
            return;
        }
        ESP_LOGI(TAG, "AP    : ssid=\"%s\" ip=%s clients=%u",
                 app_cfg_cmd_get()->ap_ssid, net_ip_str(),
                 (unsigned)net_client_count());
        ESP_LOGI(TAG, "阈值  : 心跳 %u ms，放松 %u ms，motion 兜底 %u ms",
                 (unsigned)s_q.hb_ms, (unsigned)s_q.long_ms,
                 (unsigned)NET_MOTION_BACKSTOP_MS);
        return;
    }

    if (strcmp(sub, "counters") == 0) {
        cmd_queue_counters_t qc;
        cmd_queue_state_t st;
        if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
            cmd_queue_get_counters(&s_q, &qc);
            st = s_q.last_state;
            xSemaphoreGive(s_lock);
        } else {
            memset(&qc, 0, sizeof(qc));
            st = CMD_QUEUE_IDLE;
        }
        ESP_LOGI(TAG, "链路状态: %s（距上一帧 %u ms）",
                 cmd_queue_state_name(st),
                 (unsigned)cmd_queue_age_ms(&s_q,
                     (uint32_t)(esp_timer_get_time() / 1000)));
        ESP_LOGI(TAG, "邮箱计数: 收下 %u / 协议拒 %u / 序号丢 %u / 进HOLD %u / "
                      "进RELAX %u / 急停 %u / tick %u",
                 (unsigned)qc.accepted, (unsigned)qc.rejected,
                 (unsigned)qc.dropped_seq, (unsigned)qc.hold_entries,
                 (unsigned)qc.relax_entries, (unsigned)qc.estop_latched,
                 (unsigned)qc.ticks);
        return;
    }

    ESP_LOGE(TAG, "用法: net status | net counters");
}
