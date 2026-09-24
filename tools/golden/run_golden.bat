@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion
cd /d "%~dp0"
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
gcc -O2 -Wall -Wextra -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_kinematics.exe ^
    test_kinematics.c "..\..\firmware\src\control\kinematics.c" -lm
if errorlevel 1 goto :err

gcc -O2 -Wall -Wextra -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_body_pose.exe ^
    test_body_pose.c "..\..\firmware\src\control\body_pose.c" -lm
if errorlevel 1 goto :err

gcc -O2 -Wall -Wextra -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_gait_trot.exe ^
    test_gait_trot.c "..\..\firmware\src\control\gait_trot.c" -lm
if errorlevel 1 goto :err

gcc -O2 -Wall -Wextra -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_moving_avg.exe ^
    test_moving_avg.c "..\..\firmware\src\control\filter_moving_avg.c" -lm
if errorlevel 1 goto :err

gcc -O2 -Wall -Wextra -std=c11 ^
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
gcc -O2 -Wall -Wextra -std=c11 ^
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

gcc -O2 -Wall -Wextra -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_servo_map.exe ^
    test_servo_map.c "..\..\firmware\src\control\servo_map.c" -lm
if errorlevel 1 goto :err

rem control_chain is the whole P3 orchestration: it links every control/ math
rem module it drives (trot/walk trajectory, body pose, IK, servo mapping).
gcc -O2 -Wall -Wextra -std=c11 ^
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
