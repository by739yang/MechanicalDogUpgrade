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
build\test_moving_avg.exe golden\moving_avg.csv
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
