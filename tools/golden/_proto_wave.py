"""scratch prototype: pin the clock and run the REAL padog.action_wave_direct."""
import sys, time, types, tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mpy_stubs, gen_golden


class Clock:
    def __init__(self, t=100000):
        self.now = t
    def advance(self, ms):
        self.now += int(ms)


clock = Clock()

ut = types.ModuleType("utime")
ut.ticks_ms = lambda: clock.now
ut.ticks_us = lambda: clock.now * 1000
ut.ticks_add = lambda a, b: a + b
ut.ticks_diff = lambda a, b: a - b
ut.sleep_ms = lambda ms: clock.advance(ms)
ut.sleep_us = lambda us: clock.advance(us // 1000)
ut.sleep = lambda s: clock.advance(s * 1000)
sys.modules["utime"] = ut
time.sleep_ms = clock.advance

with tempfile.TemporaryDirectory() as td:
    ns = gen_golden.load_padog_ns(td)
    rec = sys.modules["PA_SERVO"]._i2c_servo
    orig_angle = sys.modules["PA_SERVO"].angle
    log = []

    def angle(ch, deg):
        log.append((clock.now, int(ch), float(deg)))
        return orig_angle(ch, deg)

    sys.modules["PA_SERVO"].angle = angle
    clock.now = 100000
    rec.clear()
    ns["action_wave_direct"]()

    writes = rec.led_writes()
    print("angle calls :", len(log))
    print("i2c writes  :", len(writes))
    # group by timestamp
    groups = []
    for (t, ch, deg) in log:
        if not groups or groups[-1][0] != t:
            groups.append((t, []))
        groups[-1][1].append((ch, deg))
    print("groups      :", len(groups))
    for (t, items) in groups[:6]:
        print("   t=%d off=%d chans=%s" % (t, t - 100000, [c for c, _ in items]))
    print("   ...")
    for (t, items) in groups[-8:]:
        print("   t=%d off=%d chans=%s degs=%s" % (
            t, t - 100000, [c for c, _ in items],
            ["%.6f" % d for _, d in items]))
    print("final clock :", clock.now - 100000)
    print("active=%s hold=%s freeze=%s start=%d end=%d" % (
        ns["pose_anim_active"], ns["pose_anim_hold_freeze"],
        ns["direct_pose_freeze"], ns["pose_anim_start_ms"] - 100000,
        ns["pose_anim_end_ms"] - 100000))
    print("inplace=%s crawl=%s gait=%s t=%s spd=%s H_goal=%s R_H=%s" % (
        ns["inplace_step_end_ms"], ns["crawl_phase"], ns["gait_mode"], ns["t"],
        ns["spd"], ns["H_goal"], ns["R_H"]))
    print("PIT_goal=%s ROL_goal=%s X_goal=%s init_case=%s front=%s rear=%s" % (
        ns["PIT_goal"], ns["ROL_goal"], ns["X_goal"], ns["init_case"],
        ns["front_leg_y_offset"], ns["rear_leg_y_offset"]))
    # aliases
    print("action_wave is not action_wave_direct:", ns["action_wave"] is not ns["action_wave_direct"])
