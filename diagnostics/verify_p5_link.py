# -*- coding: utf-8 -*-
"""
verify_p5_link.py —— P5 全链路自动验收（HTTP 侧 + 串口侧在同一个进程里做）

为什么要有这个脚本：验收需要"一边发网页命令、一边读串口寄存器"，而这两件事
必须**同时**发生。电脑一旦连到 `RobotDog` 纯 AP 就没有外网，云端那头就发不出新
指令了 —— 所以整段验证必须**先挂成后台任务**，由它自己等网络、自己跑完、自己落盘。

它会做：
  1. 打开串口并**一直握着**（关串口会让 CH340 跳 DTR/RTS 把 ESP32 复位，见 README）
  2. 轮询 `GET /status`，直到能连上 AP（给你留足切网络的时间）
  3. 基线 `readback`（应为 12 路全 `OFF=4096` 松力）
  4. 串口 `motion mode chain` + `motion start`，等姿态收敛（站姿 slew 约 9 秒）
  5. **静止对照**：摇杆居中，连读两次 `readback` —— 收敛后两次应**完全相同**
  6. **推杆**：HTTP 以 ~10 Hz 连发 `f=80t=0` 约 4 秒，其间连读两次 `readback`
     —— 步态在走，两次应**明显不同**（这一条就是"网页摇杆真的驱动了舵机"）
  7. **断连**：停发命令，等 3 秒，读 `net counters` —— `进HOLD` / `进RELAX` 应各 +1
  8. 收尾 `estop` 让舵机回松力（安全态），并把全过程写进 diagnostics/p5_link_log.txt

用法：
    python verify_p5_link.py [COM4] [等待AP的秒数]
"""

import os
import re
import sys
import time
import urllib.error
import urllib.request

import serial

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

HERE = os.path.dirname(os.path.abspath(__file__))
LOGPATH = os.path.join(HERE, "p5_link_log.txt")

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
WAIT_AP_S = float(sys.argv[2]) if len(sys.argv) > 2 else 300.0

BAUD = 115200
BASE = "http://192.168.4.1"
ANSI = re.compile(r"\x1b\[[0-9;]*m")

_log_lines = []


def log(msg):
    line = "[%7.2fs] %s" % (time.time() - T0, msg)
    print(line, flush=True)
    _log_lines.append(line)


def flush_log():
    try:
        with open(LOGPATH, "w", encoding="utf-8") as f:
            f.write("\n".join(_log_lines) + "\n")
    except Exception as e:
        print("写日志失败: %s" % e)


# ----------------------------------------------------------------------------
# 串口
# ----------------------------------------------------------------------------

def drain(s, seconds):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < seconds:
        n = s.in_waiting
        if n:
            buf += s.read(n)
        else:
            time.sleep(0.02)
    return ANSI.sub("", buf.decode("utf-8", "replace"))


def cmd(s, text, wait=1.5):
    log("SERIAL >>> %s" % text)
    s.write((text + "\r\n").encode("utf-8"))
    s.flush()
    out = drain(s, wait)
    for ln in out.splitlines():
        if ln.strip():
            log("    %s" % ln.strip())
    flush_log()
    return out


# ----------------------------------------------------------------------------
# HTTP
# ----------------------------------------------------------------------------

def http_get(path, timeout=2.0):
    """成功返回正文，失败返回 None（切网络时失败是正常的，不抛异常）"""
    try:
        with urllib.request.urlopen(BASE + path, timeout=timeout) as r:
            return r.read().decode("utf-8", "replace")
    except Exception:
        return None


# ----------------------------------------------------------------------------
# 持续发送线程 —— 必须一直发，不能只在某个瞬间发
# ----------------------------------------------------------------------------

STICK = {"f": 0, "run": False, "sent": 0, "fail": 0}


def sender_loop():
    """
    以 ~10 Hz 持续发 `f=<STICK[f]>t=0`，模拟"页面一直播着"。

    ⚠️ 这不是可有可无的：心跳是 250 ms、长超时 2 s。如果只在读寄存器**之前**
    发几条、然后停手去读，邮箱会在读的过程中进 HOLD（输入归零）乃至 RELAX
    （松力停车），读回来的东西就没有意义了 —— 第一次跑就是这样，12 路全松力，
    比较"两次相同"其实是因为**全都松力**，属于假通过（P-18 那一类）。
    """
    while STICK["run"]:
        if http_get("/f=%dt=0" % STICK["f"], timeout=1.0) is not None:
            STICK["sent"] += 1
        else:
            STICK["fail"] += 1
        time.sleep(0.1)


def start_sender():
    import threading
    STICK["run"] = True
    t = threading.Thread(target=sender_loop, daemon=True)
    t.start()
    return t


def stop_sender(t):
    STICK["run"] = False
    t.join(timeout=3.0)


def wait_for_ap(max_s):
    log("等待 AP 可达（最多 %.0f 秒）—— 请现在把电脑连到 WiFi RobotDog" % max_s)
    t0 = time.time()
    n = 0
    while time.time() - t0 < max_s:
        body = http_get("/status", timeout=1.5)
        if body is not None:
            log("AP 已可达 ✔")
            for ln in body.strip().splitlines():
                log("    /status | %s" % ln)
            return True
        n += 1
        if n % 10 == 0:
            log("  仍在等待…（已等 %.0f 秒）" % (time.time() - t0))
            flush_log()
        time.sleep(2.0)
    log("*** 超时：%.0f 秒内没能连上 AP，后面的步骤无法执行 ***" % max_s)
    return False


# ----------------------------------------------------------------------------
# readback 解析（用来做数值比较，不靠肉眼）
# ----------------------------------------------------------------------------

# ⚠️ 通道名里**带空格**（`L-F hip` / `L-R shank` …），所以中间必须用 `.*?` 兜住，
#    不能用 `\S+` —— 用 `\S+` 会匹配不到任何一行（真机上抓到的输出就是这样）。
CH_RE = re.compile(r"ch(\d+)\s+.*?0x4[01]\s+ch\d+\s+ON=(\d+)\s+OFF=(\d+)")


def parse_readback(text):
    """返回 {通道号: (ON, OFF)}"""
    out = {}
    for m in CH_RE.finditer(text):
        out[int(m.group(1))] = (int(m.group(2)), int(m.group(3)))
    return out


def parse_cfg_value(text, name):
    """从 `cfg get <name>` 的回显里抠出数值（格式容忍两种写法）"""
    for ln in text.splitlines():
        if name in ln and "=" in ln:
            m = re.search(r"=\s*([-+]?\d+(?:\.\d+)?)", ln)
            if m:
                return float(m.group(1))
    return None


def status_int(text, key):
    m = re.search(key + r"=(\d+)", text or "")
    return int(m.group(1)) if m else None


# ----------------------------------------------------------------------------
# ⑦ 配置写入通道 + 序号重放保护（C 阶段新增的两块）
# ----------------------------------------------------------------------------

def phase_config_and_seq(s, results):
    log("")
    log("=== ⑦ 配置写入通道（标定键 / sc / 参数表单）===")

    # --- 标定键：选腿 1 → 大腿 +1 三次，看配置值是否跟着涨 ---
    out0 = cmd(s, "cfg get c1_thigh", 1.5)
    v0 = parse_cfg_value(out0, "c1_thigh")
    log("起始 c1_thigh = %s" % v0)

    for k in ("key=l1", "key=hi", "key=hi", "key=hi"):
        http_get("/" + k, timeout=1.5)
        time.sleep(0.2)
    time.sleep(0.8)

    out1 = cmd(s, "cfg get c1_thigh", 1.5)
    v1 = parse_cfg_value(out1, "c1_thigh")
    log("选腿1 + hi×3 之后 c1_thigh = %s" % v1)
    ok_nudge = (v0 is not None and v1 is not None and abs((v1 - v0) - 3.0) < 1e-6)
    log("⇒ 标定键生效（+3）= %s" % ok_nudge)
    results["cal_nudge_ok"] = ok_nudge

    # --- 参数表单路由：`GET /?c1_thigh=NN`（原版 `<form action="/">`）---
    # ⚠️ 复原要**精确**（用 %g，不取整）：下面会 `sc` 存 NVS，
    #    若这里悄悄截断成整数，就会把用户的标定值永久改掉零点几度。
    target_txt = ("%g" % v0) if v0 is not None else "84"
    http_get("/?c1_thigh=%s" % target_txt, timeout=1.5)
    time.sleep(0.8)
    out2 = cmd(s, "cfg get c1_thigh", 1.5)
    v2 = parse_cfg_value(out2, "c1_thigh")
    log("表单路由设为 %s 之后 c1_thigh = %s" % (target_txt, v2))
    ok_form = (v0 is not None and v2 is not None and abs(v2 - v0) < 1e-6)
    log("⇒ 参数表单生效且复原 = %s" % ok_form)
    results["config_form_ok"] = ok_form

    # --- sc 写 NVS ---
    http_get("/key=sc", timeout=1.5)
    time.sleep(0.5)
    cmd(s, "cfg info", 1.5)
    results["sc_sent"] = True

    # --- 序号重放保护 ---
    log("")
    log("=== ⑦b 序号重放保护 ===")
    st0 = http_get("/status", timeout=2.0)
    a0, d0 = status_int(st0, "accepted"), status_int(st0, "dropped_seq")
    log("重放前：accepted=%s dropped_seq=%s" % (a0, d0))

    # 同一个 seq 发两次 + 一个更新的 seq：应当 收下2 / 序号丢1
    http_get("/f=0t=0&seq=777000", timeout=1.5)
    time.sleep(0.15)
    http_get("/f=0t=0&seq=777000", timeout=1.5)   # 重放 → 应被丢弃
    time.sleep(0.15)
    http_get("/f=0t=0&seq=777001", timeout=1.5)   # 更新 → 应收下
    time.sleep(0.5)

    st1 = http_get("/status", timeout=2.0)
    a1, d1 = status_int(st1, "accepted"), status_int(st1, "dropped_seq")
    log("重放后：accepted=%s dropped_seq=%s" % (a1, d1))
    ok_replay = (a0 is not None and d0 is not None and a1 is not None and d1 is not None
                 and (a1 - a0) == 2 and (d1 - d0) == 1)
    log("⇒ 重放被丢、新序号被收（期望 accepted +2 / dropped_seq +1）= %s" % ok_replay)
    results["replay_protection_ok"] = ok_replay

    cmd(s, "net counters", 2.0)


def diff_channels(a, b):
    """返回不同的通道号列表"""
    keys = sorted(set(a) | set(b))
    return [k for k in keys if a.get(k) != b.get(k)]


# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------

def main():
    s = serial.Serial(PORT, BAUD, timeout=0.2)
    log("已打开 %s @ %d（全程保持打开：关串口会复位板子）" % (PORT, BAUD))
    s.reset_input_buffer()
    drain(s, 1.5)

    if not wait_for_ap(WAIT_AP_S):
        s.close()
        flush_log()
        return 2

    results = {}

    # ---- 1. 基线：应当是 12 路全松力 ----
    log("")
    log("=== ① 基线 readback（期望 12 路全 OFF=4096 松力）===")
    rb0 = parse_readback(cmd(s, "readback", 2.0))
    all_off = all(v == (0, 4096) for v in rb0.values()) if rb0 else False
    log("解析到 %d 路；全部松力 = %s" % (len(rb0), all_off))
    results["baseline_all_relaxed"] = all_off

    # ---- 2. 起控制任务（**先开持续发送线程**，否则等收敛期间心跳就超时了）----
    log("")
    log("=== ② 起控制链与运动任务（摇杆居中、持续发送）===")
    STICK["f"] = 0
    th = start_sender()
    log("持续发送线程已启动（~10 Hz，f=0）")
    cmd(s, "motion mode chain", 1.2)
    cmd(s, "motion start", 2.0)

    # 站姿 slew 收敛约 9 秒；全程持续发送，链路保持 FRESH
    log("等站姿收敛 12 秒（全程持续发送）...")
    time.sleep(12.0)

    # ---- 3. 静止对照：摇杆居中，两次 readback 应相同 ----
    log("")
    log("=== ③ 静止对照：摇杆居中，连读两次 ===")
    a1 = parse_readback(cmd(s, "readback", 2.0))
    time.sleep(1.5)
    a2 = parse_readback(cmd(s, "readback", 2.0))
    d_static = diff_channels(a1, a2)
    log("居中止静时两次 readback 不同的通道 = %s（期望：空）" % (d_static or "无"))

    # ⚠️ 有效性闸门：如果 12 路全是松力，那"两次相同"毫无意义（都松力当然相同）。
    #    第一次跑就掉进过这个陷阱：控制任务被超时杀掉 ⇒ 12 路全 OFF=4096 ⇒
    #    ③ 报"通过"，其实通过的原因是**什么都没在动**（P-18 那一类假通过）。
    energized = bool(a2) and not all(v == (0, 4096) for v in a2.values())
    if not energized:
        log("*** 无效：12 路仍是全松力，说明舵机根本没出力 ——")
        log("*** 后面的『两次是否相同 / 是否不同』都没有意义，先查控制任务为什么没在跑。")
    results["servos_energized"] = energized
    results["static_identical"] = energized and (len(d_static) == 0)

    # ---- 4. 推杆：把持续发送的值改成 f=80，其间两次 readback 应明显不同 ----
    log("")
    log("=== ④ 推杆：持续发送的值改为 f=80，等 3 秒后连读两次 ===")
    STICK["f"] = 80
    time.sleep(3.0)
    b1 = parse_readback(cmd(s, "readback", 2.0))
    b2 = parse_readback(cmd(s, "readback", 2.0))
    d_moving = diff_channels(b1, b2)
    log("推杆时两次 readback 不同的通道 = %s（期望：非空）" % (d_moving or "无"))
    log("发送线程统计：成功 %d 条 / 失败 %d 条" % (STICK["sent"], STICK["fail"]))
    results["moving_differs"] = (len(d_moving) > 0)
    log("推杆 vs 静止 的整体差异通道 = %s" % (diff_channels(a2, b2) or "无"))
    results["pushed_vs_static_differs"] = (len(diff_channels(a2, b2)) > 0)

    # ---- 5. 断连：停掉发送线程，等 3 秒，看两级超时 ----
    log("")
    log("=== ⑤ 断连：停掉发送线程，等 3 秒，读计数 ===")
    stop_sender(th)
    time.sleep(3.0)
    cmd(s, "net counters", 2.0)
    st = http_get("/status", timeout=2.0)
    if st:
        for ln in st.strip().splitlines():
            log("    /status | %s" % ln)
        results["status_after_disconnect"] = st
    # HTTP 能读回来说明电脑此刻还在 AP 上
    log("（注意：这条 /status 能读回来说明电脑此刻仍连在 AP 上）")

    # ---- 6. 收尾：回松力 ----
    log("")
    log("=== ⑥ 收尾：estop 让舵机回松力（安全态）===")
    cmd(s, "estop", 2.0)
    time.sleep(0.5)
    rb_end = parse_readback(cmd(s, "readback", 2.0))
    results["final_all_relaxed"] = (rb_end and all(v == (0, 4096) for v in rb_end.values()))

    # ---- 7. C 阶段新增的两块：配置写入通道 + 序号重放保护 ----
    phase_config_and_seq(s, results)

    # ---- 汇总 ----
    log("")
    log("================= 结论 =================")
    log("① 起始 12 路全松力              : %s" % results.get("baseline_all_relaxed"))
    log("② 控制任务起来后舵机有出力      : %s  <== 假通过闸门"
        % results.get("servos_energized"))
    log("③ 静止时两次 readback 完全相同  : %s  （差别 %s）"
        % (results.get("static_identical"), d_static or "无"))
    log("④ 推杆时两次 readback 不同      : %s  （差别 %s）"
        % (results.get("moving_differs"), d_moving or "无"))
    log("④ 推杆 vs 静止 不同             : %s" % results.get("pushed_vs_static_differs"))
    log("⑥ 收尾回松力                    : %s" % results.get("final_all_relaxed"))
    log("⑦ 标定键 ±1 生效（配置通道）     : %s" % results.get("cal_nudge_ok"))
    log("⑦ 参数表单 `/?name=value` 生效   : %s" % results.get("config_form_ok"))
    log("⑦b 序号重放被丢 / 新序号被收     : %s" % results.get("replay_protection_ok"))
    log("")
    log("判定『网页摇杆驱动了舵机』= ②舵机有出力 且 ③静止时两次相同 且 ④推杆时两次不同")
    verdict = (results.get("servos_energized") and results.get("static_identical")
               and results.get("moving_differs"))
    log("=== 摇杆链路 %s ===" % ("PASS" if verdict else "FAIL / 需人工看日志"))
    verdict_c = (results.get("cal_nudge_ok") and results.get("config_form_ok")
                 and results.get("replay_protection_ok"))
    log("=== 配置通道 + 重放保护 %s ===" % ("PASS" if verdict_c else "FAIL / 需人工看日志"))

    s.close()
    flush_log()
    log("日志已写入 %s" % LOGPATH)
    flush_log()
    return 0 if (verdict and verdict_c) else 1


T0 = time.time()
try:
    sys.exit(main())
except Exception as e:
    import traceback
    log("*** 异常: %s" % e)
    log(traceback.format_exc())
    flush_log()
    sys.exit(3)
