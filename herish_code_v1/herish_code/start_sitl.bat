@echo off

echo Starting ArduCopter SITL...
start "SITL" powershell -NoExit -Command "cd C:\ardu_sim; .\arducopter -w -S --model + --speedup 1 --defaults parameters/copter.parm -I0"

timeout /t 5 >nul

echo Starting MAVProxy...
start "MAVProxy" powershell -NoExit -Command "cd C:\ardu_sim; mavproxy --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14560"

echo Environment ready.
pause
