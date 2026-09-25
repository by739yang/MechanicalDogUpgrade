/**
 * @file    esp_log.h
 * @brief   宿主测试用的 ESP-IDF `esp_log.h` 桩
 *
 * 打到 stdout，前缀是**模拟时钟**的毫秒数，这样多帧/长跑测试的日志能看出时序。
 * 可用 `host_log_set_quiet(1)` 静音（默认静音，避免刷屏；测试只打印断言结果）。
 */
#pragma once

#include <stdio.h>

/** 模拟时钟（微秒），定义在 host_sim.c */
int64_t host_now_us(void);

void host_log_set_quiet(int quiet);

void host_log_write(const char *level, const char *tag, const char *fmt, ...)
    /*
     * ⚠️ 用 `gnu_printf` 而不是 `printf`：
     *    mingw 的 gcc 在 `-std=c11` 下按 **MSVCRT** 语义检查格式串，
     *    会把固件里完全合法的 `%lld` 报成 "unknown conversion type character 'l'"。
     *    固件（xtensa gcc）里 `%lld` 是对的，所以**不该为了宿主去改固件**；
     *    这里用 gnu_printf 语义检查即可 —— 既消掉假警告，又保留格式检查。
     */
    __attribute__((format(gnu_printf, 3, 4)));

#define ESP_LOGE(tag, fmt, ...) host_log_write("E", tag, fmt, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) host_log_write("W", tag, fmt, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) host_log_write("I", tag, fmt, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) host_log_write("D", tag, fmt, ##__VA_ARGS__)
#define ESP_LOGV(tag, fmt, ...) host_log_write("V", tag, fmt, ##__VA_ARGS__)
