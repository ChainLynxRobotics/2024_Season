@echo off

set user="admin"
set hostname="roboRIO-8248-frc.local"
set autos_path="/home/lvuser/deploy/pathplanner"

echo Connecting to %user%@%hostname%
echo --------------------------------------
ssh %user%@%hostname% "rm -rf %autos_path%;echo %autos_path% has been cleared"
echo --------------------------------------
pause