@echo off
title STM32 CAN Loopback Serial Monitor
color 0A
cls
echo ======================================================================
echo     DANG KET NOI VAO COM18 (Baudrate: 115200 bps)...
echo     (Log tu STM32 se hien thi ben duoi khi ban bam F8 Resume)
echo     Nhan Ctrl + ] de thoat
echo ======================================================================
echo.
python -m serial.tools.miniterm COM18 115200
pause
