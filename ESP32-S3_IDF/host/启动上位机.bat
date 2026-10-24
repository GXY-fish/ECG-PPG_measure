@echo off
chcp 65001 >nul
cd /d %~dp0
python ecg_host.py %*
echo.
echo ====== 上位机已退出，按任意键关闭窗口 ======
pause >nul