@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo.
echo ========================================
echo   一键上传（可选刷 MicroPython 固件）
echo ========================================
echo 用法:
echo   直接运行 — 询问是否刷固件（y=擦除+刷写+上传，n/回车=仅上传）
echo   一键上传.bat --upload-only     不询问，仅上传脚本
echo   一键上传.bat --erase-and-flash 提示后仍须在询问里选 y（须本目录有 *.bin）
echo   一键上传.bat --flash-if-bin    有 .bin 则自动刷写+上传（不询问）
echo.
py -3 flash_upload.py %*
if errorlevel 1 python flash_upload.py %*
pause
