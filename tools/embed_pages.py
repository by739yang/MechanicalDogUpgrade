#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 `micropython/*.html` 生成成 `firmware/src/comm/pages_gen.h`（C 字符串字面量）。

为什么要内嵌而不是从文件系统读：
  - 原版 `web_ctl.py` / `web_c.py` 是从板上文件系统读页面的；C 版没有那个文件系统
    （裸 ESP-IDF + 单 app 分区），挂 LittleFS 要改分区表 —— 为一个 30 KB 的静态页面
    不值当。
  - 内嵌之后**页面和固件版本永远同步**，不会出现"刷了新固件、板上还是旧页面"。
  - 顺带满足 §8.6「不在控制任务里加载/发送大块 HTML」：页面在 flash 里，直接发。

⚠️ 页面是**原版逐字节照搬**的，一个字都没改 —— 目的就是让用户手上熟悉的那套
   `drive.html` 在新固件上照样能遥控（它是本轮唯一能在板子上验收 P5 的手段）。

用法：`python tools/embed_pages.py`   （在仓库根目录跑）
"""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(ROOT, "micropython")
OUT = os.path.join(ROOT, "firmware", "src", "comm", "pages_gen.h")

# (文件名, C 标识符, 注释)
PAGES = [
    ("drive.html",   "kPageDrive",   "轻量遥控页（原版 web_ctl.py 提供；狗 80 ms / 臂 35 ms 轮询）"),
    ("control.html", "kPageControl", "完整控制页（原版 web_c.py 提供，120 ms 轮询）"),
    ("cal.html",     "kPageCal",     "标定页（原版 web_c.py 动态生成后段参数表）"),
]


def c_escape(data: bytes) -> str:
    """字节 -> C 字符串字面量内容（\" \\ 转义；换行保留为 \\n 以便生成文件可读）。"""
    out = []
    for b in data:
        if b == 0x22:          # "
            out.append('\\"')
        elif b == 0x5C:        # backslash
            out.append("\\\\")
        elif b == 0x0A:        # \n
            out.append("\\n")
        elif b == 0x0D:        # \r  —— 丢掉：HTTP 里 CRLF 由页面自己带，这里保持原样更安全
            out.append("\\r")
        elif b == 0x09:
            out.append("\\t")
        elif 0x20 <= b <= 0x7E:
            out.append(chr(b))
        else:
            # 非 ASCII（页面里的中文）：用八进制转义，避免任何编码/行尾被工具链改写
            out.append("\\%03o" % b)
    return "".join(out)


def wrap(s: str, width: int = 100) -> list:
    """按宽度切行 —— 但**只在转义序列边界切**，所以逐段累加。"""
    lines, cur = [], ""
    # 逐 token 累加，保证不切断 "\123" 这种转义
    i = 0
    while i < len(s):
        if s[i] == "\\":
            tok = s[i:i + 4] if s[i + 1:i + 2].isdigit() else s[i:i + 2]
        else:
            tok = s[i]
        if len(cur) + len(tok) > width:
            lines.append(cur)
            cur = ""
        cur += tok
        i += len(tok)
    if cur:
        lines.append(cur)
    return lines


def main() -> int:
    parts = []
    parts.append("""/**
 * @file    comm/pages_gen.h
 * @brief   **自动生成，不要手改** —— 由 `tools/embed_pages.py` 从 `micropython/` 下的
 *          `.html` 源文件生成。
 *
 * 页面内容与原版**逐字节相同**，好让用户手上那套 `drive.html` 在新固件上照样能用
 * （它是本轮唯一能在板子上验收 P5 的手段）。要改页面就改 `micropython/` 下的源文件，
 * 然后重跑 `python tools/embed_pages.py`。
 *
 * 每页两个符号：`<id>` 是内容，`<id>_LEN` 是长度（HTTP 响应要它）。
 *
 * ⚠️ 本文件的注释里**不能出现**斜杠加星号那个组合（C 注释不可嵌套，
 *    `-Werror=comment` 会直接编译失败）—— 所以上面写的是"`micropython/` 下的 `.html`"。
 */

#ifndef COMM_PAGES_GEN_H
#define COMM_PAGES_GEN_H

#include <stddef.h>

""")

    total = 0
    for fname, ident, comment in PAGES:
        path = os.path.join(SRC_DIR, fname)
        if not os.path.exists(path):
            print("缺少源文件: %s" % path, file=sys.stderr)
            return 1
        with open(path, "rb") as f:
            data = f.read()
        total += len(data)

        parts.append("/** `%s` （%d 字节）—— %s */\n" % (fname, len(data), comment))
        parts.append("static const char %s[] =\n" % ident)
        for line in wrap(c_escape(data)):
            parts.append('    "%s"\n' % line)
        parts.append("    ;\n")
        parts.append("static const size_t %s_LEN = sizeof(%s) - 1u;\n\n" % (ident, ident))

    parts.append("#endif /* COMM_PAGES_GEN_H */\n")

    text = "".join(parts)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    # UTF-8：本文件的**注释**是中文（和 `firmware/src/` 下其它源文件一致）；
    # 页面内容里的非 ASCII 已经被 `c_escape()` 转成八进制转义，不会受编码影响。
    # 注意 `firmware/platformio.ini` 与 `firmware/sdkconfig.defaults` **必须保持纯 ASCII**
    # （kconfgen 按系统 GBK 读，P-01），本文件不是那两个，所以 UTF-8 没问题。
    with open(OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)

    print("生成 %s" % OUT)
    print("  页面共 %d 字节，头文件 %d 字节" % (total, len(text)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
