# 机器狗主程序：网页线程 + 运动主线程
# 板载模块布局：web_ui_mode=1 -> web_c.py + control.html（完整控制/标定）
#               web_ui_mode=0 -> web_ctl.py + drive.html（轻量遥控页）

import _thread
import gc
import time
import utime
import padog


web_ui_mode = 1
DEBUG_MOVE = True


def web_thread():
    gc.collect()
    if web_ui_mode == 0:
        import web_ctl
    else:
        import web_c


def control_loop():
    global DEBUG_MOVE
    last_state = None
    err_count = 0
    while True:
        try:
            padog.mainloop()
        except Exception as e:
            err_count += 1
            if err_count <= 5 or (err_count % 500) == 0:
                print('MAINLOOP ERR', repr(e))
        else:
            err_count = 0

        if DEBUG_MOVE:
            state = (padog.spd, padog.L, padog.R, padog.gait_mode)
            if state != last_state:
                last_state = state
                print('MOVE spd=%s L=%s R=%s gait=%s' % state)

        # 让出 GIL，保证 HTTP 请求不会长期得不到处理
        time.sleep_ms(2)


def main():
    try:
        import mech_arm
        mech_arm.init_arm_pose()
        mech_arm.init_grip()
    except Exception as e:
        print('mech_arm init skip:', e)

    _thread.start_new_thread(web_thread, ())
    control_loop()


main()
