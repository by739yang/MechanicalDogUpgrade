/**
 * @file    filter_moving_avg.c
 * @brief   滑动平均滤波器 —— 逐行对齐 micropython/PA_AVGFILT.py
 *
 * Python 原实现（cache 由调用方传入，前 3 槽被挪用）：
 *
 *   class avg_filiter():
 *       def __init__(self, cache_data):
 *           self.cache = cache_data
 *           self.len = len(cache_data)
 *           self.cache[0] = self.len          # 槽0：长度标记
 *           self.sum = 0
 *           for item in cache_data[3:]:       # 窗口初值 = cache[3:]
 *               self.sum += item
 *           self.cache[1] = self.sum          # 槽1：累加和
 *
 *       def avg(self, new_data):
 *           self.cache[1] = self.cache[1] - self.cache[3]   # 减掉最老
 *           self.cache[1] = self.cache[1] + new_data        # 加上最新
 *           self.cache[3:-1] = self.cache[4:]               # 整体左移一格
 *           self.cache[-1] = new_data                       # 尾部写入最新
 *           return self.cache[1] // (self.len - 3)          # 向下取整
 *
 * 本实现把"整体左移"换成环形缓冲：`buf[pos]` 就等价于原实现里的 `cache[3]`
 * （始终是最老的那个）。输出完全一致，但每次调用从 O(N) 降到 O(1)。
 */

#include "control/filter_moving_avg.h"

#include <stddef.h>

/**
 * @brief 整数向下取整除法，语义与 Python 的 `//` 一致。
 *
 * C 的 `/` 向 0 截断：`-1 / 2 == 0`。
 * Python 的 `//` 向 -∞ 取整：`-1 // 2 == -1`。
 * 陀螺仪原始值有负数，这个差别会真实影响输出，所以必须显式实现。
 */
static int32_t floor_div(int32_t a, int32_t b)
{
    int32_t q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0))) {
        --q;
    }
    return q;
}

void moving_avg_init_from(moving_avg_t *f, int32_t *buf, uint16_t window_len,
                          const int32_t *init_values)
{
    if (f == NULL) {
        return;
    }

    f->buf        = buf;
    f->window_len = window_len;
    f->pos        = 0;
    f->sum        = 0;

    if (buf == NULL || window_len == 0) {
        return;
    }

    for (uint16_t i = 0; i < window_len; ++i) {
        const int32_t v = (init_values != NULL) ? init_values[i] : 0;
        buf[i] = v;
        f->sum += v;    /* 对应 Python __init__ 里对 cache_data[3:] 的求和 */
    }
}

void moving_avg_init(moving_avg_t *f, int32_t *buf, uint16_t window_len)
{
    moving_avg_init_from(f, buf, window_len, NULL);
}

int32_t moving_avg_push(moving_avg_t *f, int32_t new_data)
{
    if (f == NULL || f->buf == NULL || f->window_len == 0) {
        return 0;   /* 原 Python 这里会 ZeroDivisionError */
    }

    /* buf[pos] 就是最老的那个元素 —— 等价于原实现的 cache[3] */
    const int32_t old = f->buf[f->pos];

    f->sum = f->sum - old;      /* cache[1] -= cache[3] */
    f->sum = f->sum + new_data; /* cache[1] += new_data */

    f->buf[f->pos] = new_data;  /* 左移之后尾部写入的就是它 */

    f->pos = (uint16_t)((f->pos + 1u) % f->window_len);

    return floor_div(f->sum, (int32_t)f->window_len);
}
