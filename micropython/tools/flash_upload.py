# -*- coding: utf-8 -*-
"""
workSpace 一键上传 / 可选刷 MicroPython 固件（ESP32）。

依赖：pip install mpremote pyserial
      刷固件：pip install esptool，本目录下放 *.bin

交互（默认，无 --upload-only / --flash-if-bin 时）：
  每次都会询问是否刷固件（有/无 *.bin 都会问；选 y 且无 .bin 会提示错误并退出）。
  选刷写后，仍会二次确认「整片擦除」。

命令行：
  --upload-only          仅上传，不询问、不刷固件。
  --erase-and-flash      在询问前提示已带刷写参数；是否刷写仍由询问决定（选 y 须已有 .bin）。
  --flash-if-bin         有 .bin 则直接刷写+上传（仅此模式跳过询问）。

配置（flash_config.json）：串口、波特率、重试等。

上传：本目录所有 .py/.html（排除 flash_upload.py、flash_config.json），main.py 最后上传。
"""
from __future__ import print_function

import glob
import json
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
CONFIG_NAME = "flash_config.json"

SKIP_NAMES = frozenset(
    {
        "flash_upload.py",
        "flash_config.json",
    }
)


def _say(msg):
    print(msg)
    sys.stdout.flush()


def _banner(title):
    _say("")
    _say("--- %s ---" % title)
    sys.stdout.flush()


def _cfg():
    path = os.path.join(ROOT, CONFIG_NAME)
    if not os.path.isfile(path):
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        print("读取 %s 失败: %s" % (CONFIG_NAME, e))
        return {}


def _find_firmware():
    bins = glob.glob(os.path.join(ROOT, "*.bin"))
    if not bins:
        return None
    bins.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return bins[0]


def _pick_com_port(preferred):
    preferred = (preferred or "").strip()
    if preferred:
        return preferred
    try:
        import serial.tools.list_ports
    except ImportError:
        print("请: pip install pyserial")
        print("或在 flash_config.json 填写 \"com_port\": \"COM5\"")
        return None
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("未发现串口。")
        return None
    print("可用串口：")
    for i, p in enumerate(ports):
        print("  [%d] %s  %s" % (i, p.device, p.description))
    if len(ports) == 1:
        print("自动选择:", ports[0].device)
        return ports[0].device
    try:
        n = input("序号 (0-%d)，回车=0: " % (len(ports) - 1))
        idx = 0 if (n is None or str(n).strip() == "") else int(str(n).strip())
        return ports[idx].device
    except (ValueError, IndexError):
        return ports[0].device


def _esptool_subcommand(py, chip, com, baud, name_new, name_old, tail_args):
    base = [py, "-m", "esptool", "--chip", chip, "--port", com, "--baud", baud]
    r = subprocess.call(base + [name_new] + tail_args)
    if r != 0 and name_old:
        _say("  (回退子命令 %s)" % name_old)
        r = subprocess.call(base + [name_old] + tail_args)
    return r


def _wait_reset(seconds):
    _banner("等待板子复位（%d 秒）" % seconds)
    for left in range(seconds, 0, -1):
        _say("  ... %d" % left)
        time.sleep(1)


def _list_upload_files():
    names = []
    for name in os.listdir(ROOT):
        if name in SKIP_NAMES or name.startswith("."):
            continue
        path = os.path.join(ROOT, name)
        if not os.path.isfile(path):
            continue
        low = name.lower()
        if not (low.endswith(".py") or low.endswith(".html")):
            continue
        names.append(name)
    names.sort(key=lambda x: (x == "main.py", x))
    return names


def _confirm_erase():
    s = input("\n【最后确认】将擦除整片 Flash，板上程序与文件会清空。继续? [y/N]: ")
    return str(s).strip().lower() in ("y", "yes", "是")


def main():
    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8")
        except Exception:
            pass

    os.chdir(ROOT)
    cfg = _cfg()
    argv = [a.strip().lower() for a in sys.argv[1:]]

    upload_only = "--upload-only" in argv
    force_erase = "--erase-and-flash" in argv or "--full" in argv
    flash_if_bin_cli = "--flash-if-bin" in argv or "--flash-if-present" in argv

    fw = _find_firmware()

    if upload_only:
        skip_flash = True
    elif flash_if_bin_cli:
        skip_flash = False if fw else True
        _say("模式 --flash-if-bin: %s" % ("刷写+上传" if fw else "无 .bin，仅上传"))
    else:
        # 默认与 --erase-and-flash：每次都询问，不因无 .bin 而跳过
        _say("")
        if force_erase:
            _say("已带参数 --erase-and-flash / --full，仍须您确认是否刷写固件。")
        if fw:
            mb = os.path.getsize(fw) / (1024.0 * 1024.0)
            _say("本目录固件: %s  (约 %.2f MB)" % (os.path.basename(fw), mb))
        else:
            _say("本目录【未】检测到 *.bin；若选刷写须先把固件放进本目录。")
        _say("  [y] 整片擦除 Flash + 刷写固件 + 再上传脚本")
        _say("  [n] 或 回车：不刷固件，仅上传脚本")
        s = input("是否刷固件? [y/N]: ")
        want_flash = str(s).strip().lower() in ("y", "yes", "是")
        if want_flash and not fw:
            print("错误：已选择刷固件，但未找到 *.bin。请将固件放入本目录后重试，或选 n 仅上传。")
            sys.exit(1)
        skip_flash = not want_flash

    if not skip_flash and not fw:
        print("错误：将刷写但未找到 *.bin。")
        sys.exit(1)

    chip = str(cfg.get("chip", "esp32")).strip().lower()
    offset = str(cfg.get("flash_offset", "0x1000")).strip()
    com = _pick_com_port(cfg.get("com_port", ""))
    if not com:
        sys.exit(1)

    baud = str(cfg.get("esptool_baud", "460800"))
    post_wait = int(cfg.get("post_flash_wait_sec", 8))
    retries = int(cfg.get("mpremote_retries", 5))
    retry_delay = float(cfg.get("mpremote_retry_delay_sec", 2.0))
    py = sys.executable

    files = _list_upload_files()
    if not files:
        print("错误：没有可上传的 .py/.html 文件")
        sys.exit(1)

    _banner("概览")
    _say("目录: %s" % ROOT)
    _say("串口: %s" % com)
    _say("固件: %s" % (os.path.basename(fw) if fw else "(无，不刷写)"))
    _say("执行: %s" % ("擦除 + 刷固件 + 上传" if not skip_flash else "仅上传"))
    _say("待传 (%d): %s" % (len(files), ", ".join(files)))

    if not skip_flash:
        if not _confirm_erase():
            _say("已取消刷写，改为仅上传。")
            skip_flash = True

    step = 0

    def step_inc(title):
        nonlocal step
        step += 1
        _banner("步骤 %d — %s" % (step, title))

    if not skip_flash:
        step_inc("擦除 Flash")
        r = _esptool_subcommand(py, chip, com, baud, "erase-flash", "erase_flash", [])
        if r != 0:
            sys.exit(r)
        step_inc("写入固件")
        r = _esptool_subcommand(
            py, chip, com, baud, "write-flash", "write_flash", ["-z", offset, fw]
        )
        if r != 0:
            sys.exit(r)
        _wait_reset(post_wait)
    else:
        step_inc("跳过刷写，开始上传")

    time.sleep(float(cfg.get("mpremote_first_delay_sec", 1.0)))

    def mpr(args, check=True, attempts=None):
        n = retries if attempts is None else int(attempts)
        c = [py, "-m", "mpremote", "connect", com] + args
        last = 1
        for attempt in range(n):
            if attempt:
                _say("  重试 %d/%d …" % (attempt + 1, n))
                time.sleep(retry_delay)
            _say(">> " + " ".join(c))
            sys.stdout.flush()
            last = subprocess.call(c)
            if last == 0:
                return last
        if check and last != 0:
            sys.exit(last)
        return last

    step_inc("上传文件到设备根目录 /")
    for i, name in enumerate(files):
        local = os.path.join(ROOT, name)
        _say("[%d/%d] %s" % (i + 1, len(files), name))
        mpr(["cp", local, ":"])
        time.sleep(0.15)

    _banner("完成")
    _say("已上传 %d 个文件。若 mpremote 断连，请按 EN 复位后再试。" % len(files))


if __name__ == "__main__":
    main()
