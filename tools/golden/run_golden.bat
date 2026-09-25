@echo off
rem KEEP THIS FILE'S LINE ENDINGS AS CRLF (and keep it readable by cmd.exe).
rem A LF-only copy makes cmd.exe mis-parse it: the '^' line continuations get split,
rem 'goto :err' stops being recognised, and fragments of the gcc command lines are
rem executed as commands -- the script then reports PASS/FAIL from a run it never
rem really performed. Same content with CRLF parses cleanly.
chcp 65001 >nul
setlocal enabledelayedexpansion
cd /d "%~dp0"
rem If a compile line silently fails (e.g. the line endings were converted to LF,
rem so cmd.exe splits the '^' continuations), a stale exe would still run and the
rem script would still print ALL GOLDEN TESTS PASSED -- a wrapper false-pass.
rem Deleting them first makes a compile failure impossible to mask.
rem Compile the host tests with -Werror: a warning must fail the build.  A reverted edit once
rem removed a (void)cfg; and left an unused-parameter warning that this script happily ignored --
rem a green run that hid a regression.
if exist build del /q build\*.exe >nul 2>nul
set FAILED=0

echo ==========================================================
echo  golden tests: MicroPython reference  --^>  C port
echo ==========================================================
echo.

echo [1/4] generate golden vectors from the original MicroPython modules
python gen_golden.py
if errorlevel 1 goto :err

echo.
echo [2/4] locate a host C compiler
where gcc >nul 2>nul
if errorlevel 1 (
  echo   gcc not found on PATH. Install mingw-w64 or add it to PATH.
  goto :err
)
echo   found: & where gcc

if not exist build mkdir build

echo.
echo [3/4] build host tests
gcc -O2 -Wall -Wextra -Werror -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_kinematics.exe ^
    test_kinematics.c "..\..\firmware\src\control\kinematics.c" -lm
if errorlevel 1 goto :err

gcc -O2 -Wall -Wextra -Werror -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_body_pose.exe ^
    test_body_pose.c "..\..\firmware\src\control\body_pose.c" -lm
if errorlevel 1 goto :err

gcc -O2 -Wall -Wextra -Werror -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_gait_trot.exe ^
    test_gait_trot.c "..\..\firmware\src\control\gait_trot.c" -lm
if errorlevel 1 goto :err

gcc -O2 -Wall -Wextra -Werror -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_moving_avg.exe ^
    test_moving_avg.c "..\..\firmware\src\control\filter_moving_avg.c" -lm
if errorlevel 1 goto :err

gcc -O2 -Wall -Wextra -Werror -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_gait_walk.exe ^
    test_gait_walk.c "..\..\firmware\src\control\gait_walk.c" -lm
if errorlevel 1 goto :err

rem app_config has no golden CSV: it is a check-based test that injects a RAM
rem storage backend, so defaults / clamping / CRC / version / round trip are all
rem verified on the PC without a board.
rem
rem control_chain.c is linked in on purpose: the injection-table defaults have to be
rem compared field-by-field against control_chain_cfg_defaults(), i.e. against the
rem function that the app layer will actually fill control_chain_cfg_t from. It is
rem pure C, but it calls the five control/ math modules, so they come along.
gcc -O2 -Wall -Wextra -Werror -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_app_config.exe ^
    test_app_config.c ^
    "..\..\firmware\src\app\app_config.c" ^
    "..\..\firmware\src\control\control_chain.c" ^
    "..\..\firmware\src\control\kinematics.c" ^
    "..\..\firmware\src\control\body_pose.c" ^
    "..\..\firmware\src\control\gait_trot.c" ^
    "..\..\firmware\src\control\gait_walk.c" ^
    "..\..\firmware\src\control\servo_map.c" -lm
if errorlevel 1 goto :err

gcc -O2 -Wall -Wextra -Werror -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_servo_map.exe ^
    test_servo_map.c "..\..\firmware\src\control\servo_map.c" -lm
if errorlevel 1 goto :err

rem control_chain is the whole P3 orchestration: it links every control/ math
rem module it drives (trot/walk trajectory, body pose, IK, servo mapping).
gcc -O2 -Wall -Wextra -Werror -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_control_chain.exe ^
    test_control_chain.c ^
    "..\..\firmware\src\control\control_chain.c" ^
    "..\..\firmware\src\control\kinematics.c" ^
    "..\..\firmware\src\control\body_pose.c" ^
    "..\..\firmware\src\control\gait_trot.c" ^
    "..\..\firmware\src\control\gait_walk.c" ^
    "..\..\firmware\src\control\servo_map.c" -lm
if errorlevel 1 goto :err

rem Multi-frame sequence: runs mainloop() hundreds of times and compares every
rem frame.  This covers what a single-frame comparison structurally cannot:
rem cross-frame carry-over, command semantics and long-run drift.
rem The command script is READ from its own CSV, not duplicated here (P-22/P-27).
gcc -O2 -Wall -Wextra -Werror -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_control_chain_cmd.exe ^
    test_control_chain_cmd.c ^
    "..\..\firmware\src\control\control_chain_cmd.c" ^
    "..\..\firmware\src\control\control_chain.c" ^
    "..\..\firmware\src\control\kinematics.c" ^
    "..\..\firmware\src\control\body_pose.c" ^
    "..\..\firmware\src\control\gait_trot.c" ^
    "..\..\firmware\src\control\gait_walk.c" ^
    "..\..\firmware\src\control\servo_map.c" -lm
if errorlevel 1 goto :err

rem Action / pose layer: compares against the REAL original, including the
rem side effects it has on OTHER modules' state (move/gait/height/gesture/
rem servo_init/set_leg_sit_offsets) and on the chain's state, plus the final 12
rem servo angles and 22 module-level globals.  Servo output alone is not enough
rem (P-25); the state dump is what gives side effects any teeth.




gcc -O2 -Wall -Wextra -Werror -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_action.exe ^
    test_action.c ^
    "..\..\firmware\src\control\action.c" ^
    "..\..\firmware\src\control\control_chain_cmd.c" ^
    "..\..\firmware\src\control\control_chain.c" ^
    "..\..\firmware\src\control\kinematics.c" ^
    "..\..\firmware\src\control\body_pose.c" ^
    "..\..\firmware\src\control\gait_trot.c" ^
    "..\..\firmware\src\control\gait_walk.c" ^
    "..\..\firmware\src\control\servo_map.c" -lm
if errorlevel 1 goto :err

rem App-layer state machine: uses the ESP-IDF host stubs (host_stubs/) with a
rem CONTROLLABLE CLOCK to run the real motion task loop all the way to the
rem PCA9685 shadow registers.  Checks cadence gating, the stand pose, e-stop,
rem timeout stop, mode switching and selective write behaviour.
rem Single-threaded stubs: LOGIC only, not concurrency and not real timing.
rem The stubs are single-threaded with always-succeeding mutexes, so this cannot catch deadlocks, priority inversion, stack depth or real jitter.
gcc -O2 -Wall -Wextra -Werror -std=c11 ^
    -I"..\..\firmware\src" -Ihost_stubs ^
    -o build\test_motion_app.exe ^
    test_motion_app.c host_stubs\host_sim.c ^
    "..\..\firmware\src\app\app_action.c" ^
    "..\..\firmware\src\app\app_chain.c" ^
    "..\..\firmware\src\app\app_config.c" ^
    "..\..\firmware\src\app\motion.c" ^
    "..\..\firmware\src\app\servo_out.c" ^
    "..\..\firmware\src\drivers\drv_pca9685.c" ^
    "..\..\firmware\src\control\action.c" ^
    "..\..\firmware\src\control\control_chain.c" ^
    "..\..\firmware\src\control\control_chain_cmd.c" ^
    "..\..\firmware\src\control\kinematics.c" ^
    "..\..\firmware\src\control\body_pose.c" ^
    "..\..\firmware\src\control\gait_trot.c" ^
    "..\..\firmware\src\control\gait_walk.c" ^
    "..\..\firmware\src\control\servo_map.c" -lm
if errorlevel 1 goto :err

rem ACTION layer reached from the firmware: the console dispatcher (app_motion_cmd.c) is
rem linked in on purpose, so the suite proves the commands really are reachable from the UART
rem console and not merely linked.  It drives the real motion task in ACTION mode with the
rem controllable clock all the way down to the PCA9685 shadow registers and checks the pose
rem animation convergence, the ch_mask merge of the wave stepper, every cross-module effect
rem (move/gait/height/gesture/sit_offsets/servo_init/crawl_reset) and e-stop mid-action.
gcc -O2 -Wall -Wextra -Werror -std=c11 ^
    -I"..\..\firmware\src" -Ihost_stubs ^
    -o build\test_app_action.exe ^
    test_app_action.c host_stubs\host_sim.c ^
    "..\..\firmware\src\app\app_action.c" ^
    "..\..\firmware\src\app\app_motion_cmd.c" ^
    "..\..\firmware\src\app\app_chain.c" ^
    "..\..\firmware\src\app\app_config.c" ^
    "..\..\firmware\src\app\motion.c" ^
    "..\..\firmware\src\app\servo_out.c" ^
    "..\..\firmware\src\drivers\drv_pca9685.c" ^
    "..\..\firmware\src\control\action.c" ^
    "..\..\firmware\src\control\control_chain.c" ^
    "..\..\firmware\src\control\control_chain_cmd.c" ^
    "..\..\firmware\src\control\kinematics.c" ^
    "..\..\firmware\src\control\body_pose.c" ^
    "..\..\firmware\src\control\gait_trot.c" ^
    "..\..\firmware\src\control\gait_walk.c" ^
    "..\..\firmware\src\control\servo_map.c" -lm
if errorlevel 1 goto :err

echo.
echo [4/4] run
build\test_kinematics.exe golden\ik.csv
if errorlevel 1 set FAILED=1
echo.
build\test_body_pose.exe golden\body_pose.csv
if errorlevel 1 set FAILED=1
echo.
build\test_gait_trot.exe golden\gait_trot.csv
if errorlevel 1 set FAILED=1
echo.
build\test_gait_walk.exe golden\gait_walk.csv
if errorlevel 1 set FAILED=1
echo.
build\test_moving_avg.exe golden\moving_avg.csv
if errorlevel 1 set FAILED=1
echo.
build\test_app_config.exe
if errorlevel 1 set FAILED=1
echo.
build\test_servo_map.exe golden\servo_angle.csv golden\servo_output.csv
if errorlevel 1 set FAILED=1
echo.
build\test_control_chain.exe golden\control_chain.csv
if errorlevel 1 set FAILED=1
echo.
build\test_control_chain_cmd.exe golden\control_chain_seq.csv golden\control_chain_seq_cmds.csv
if errorlevel 1 set FAILED=1
echo.
build\test_action.exe golden\action_pose.csv golden\action_cmd.csv golden\action_wave.csv golden\action_wave_final.csv
if errorlevel 1 set FAILED=1
echo.
build\test_motion_app.exe
if errorlevel 1 set FAILED=1
echo.
build\test_app_action.exe
if errorlevel 1 set FAILED=1

echo.
echo ==========================================================
if "!FAILED!"=="1" (
  echo  *** SOME GOLDEN TESTS FAILED ***
  exit /b 1
)
echo  ALL GOLDEN TESTS PASSED
exit /b 0

:err
echo.
echo *** GENERATE/BUILD FAILED ***
exit /b 1
