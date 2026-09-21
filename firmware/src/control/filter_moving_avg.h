/**
 * @file    filter_moving_avg.h
 * @brief   滑动平均滤波器 —— 从 micropython/PA_AVGFILT.py 迁移
 *
 * ## 原实现的三个要点（都必须复刻）
 *
 * 1. **窗口大小 = `len(cache_data) - 3`**。
 *    Python 把 cache 的前 3 个槽挪用为「长度标记 / 累加和 / 未用」，
 *    真正的数据窗口在后面：
 *      - `array('i',[0]*5)`  → 窗口 **2**（`PA_STABLIZE` 的用法）
 *      - `array('i',[0]*10)` → 窗口 **7**（`PA_WALK` 的用法）
 *    用 `MOVING_AVG_WINDOW_FROM_CACHE_LEN()` 可以从 cache 长度推算窗口大小。
 *
 * 2. **返回整数，且是向下取整**。
 *    原式 `return self.cache[1] // (self.len - 3)` 用的是 Python 的 `//`，
 *    对负数**向 -∞ 取整**；C 的 `/` 是**向 0 截断**，两者在负数上不同。
 *    陀螺仪原始值有负数，所以这个差别是真实存在的 —— 本模块显式实现向下取整。
 *
 * 3. **有状态**。`avg()` 会改内部累加和与窗口，所以 golden 测试测的是
 *    **一串按顺序的调用**，不是彼此独立的点。
 *
 * ## 相对原实现的有意差异
 *
 * 内部用**环形缓冲**替代原实现"整体左移一格"的写法：
 * 原实现每次调用搬 `window_len` 个元素（O(N)），环形缓冲是 O(1)。
 * 输出**完全一致** —— 整数运算精确，增量维护的和与重算的和相同。
 *
 * ⚠️ 一个理论差异：原实现的和是 Python 任意精度整数，本模块用 `int32_t`。
 *    用 int16 量级的传感器数据、窗口 ≤ 16 时不可能溢出。
 *
 * ⚠️ 纯数学/纯逻辑，不依赖 ESP-IDF。
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 从原 Python 的 `cache_data` 长度推算窗口大小（原实现是 len-3） */
#define MOVING_AVG_WINDOW_FROM_CACHE_LEN(cache_len) ((uint16_t)((cache_len) - 3))

/** 滑动平均滤波器状态 */
typedef struct {
    int32_t  *buf;         /**< 窗口缓冲，长度 `window_len`，**由调用方提供** */
    uint16_t  window_len;  /**< 窗口大小（= 原 Python 的 len(cache_data) - 3） */
    uint16_t  pos;         /**< 环形写指针，指向当前最老的元素 */
    int32_t   sum;         /**< 增量维护的窗口和 */
} moving_avg_t;

/**
 * @brief 初始化，窗口清零（对应 Python `avg_filiter(array('i',[0]*n))`）。
 *
 * @param f           滤波器状态
 * @param buf         窗口缓冲（调用方持有，长度必须 >= window_len）
 * @param window_len  窗口大小
 */
void moving_avg_init(moving_avg_t *f, int32_t *buf, uint16_t window_len);

/**
 * @brief 初始化，并填入非零初值。
 *
 * 原 Python 允许传入非零的 cache（`cache_data[3:]` 会成为初始窗口）。
 * 本函数对应那种用法；`init_values` 传 NULL 等价于 `moving_avg_init()`。
 *
 * @param init_values 初值数组，长度 window_len；NULL 表示全 0
 */
void moving_avg_init_from(moving_avg_t *f, int32_t *buf, uint16_t window_len,
                          const int32_t *init_values);

/**
 * @brief 推入一个新值，返回当前窗口的平均值。
 *
 * 对应 Python `avg_filiter.avg(new_data)`。
 *
 * @return `floor(sum / window_len)`（**向下取整**，与 Python `//` 一致）
 *
 * @note 未初始化或 `window_len == 0` 时返回 0（原 Python 会除零）。
 */
int32_t moving_avg_push(moving_avg_t *f, int32_t new_data);

#ifdef __cplusplus
}
#endif
