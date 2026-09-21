@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"

echo ==========================================================
echo  golden test: PA_IK.py  --^>  firmware/src/control/kinematics.c
echo ==========================================================
echo.

echo [1/4] generate golden vectors from the original MicroPython module
python gen_golden.py
if errorlevel 1 goto :err

echo.
echo [2/4] locate a host C compiler
where gcc >nul 2>nul
if errorlevel 1 (
  echo   gcc not found on PATH. Install mingw-w64 or add it to PATH.
  goto :err
)
gcc --version | findstr /r "." | findstr /n "^1:" 

echo.
echo [3/4] build host test
if not exist build mkdir build
gcc -O2 -Wall -Wextra -std=c11 ^
    -I"..\..\firmware\src" ^
    -o build\test_kinematics.exe ^
    test_kinematics.c "..\..\firmware\src\control\kinematics.c" -lm
if errorlevel 1 goto :err

echo.
echo [4/4] run
build\test_kinematics.exe golden\ik.csv
if errorlevel 1 goto :err

echo.
echo ALL GOLDEN TESTS PASSED
exit /b 0

:err
echo.
echo *** FAILED ***
exit /b 1
