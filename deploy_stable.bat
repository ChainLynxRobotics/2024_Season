@echo on
set JAVA_HOME="C:\Users\Public\wpilib\2024\jdk
setlocal

for /f "tokens=2 delims=:" %%a in ('"ipconfig | findstr IPv4"') do set ip=%%a
set ip=%ip:~1%

rem using ipinfo.io API to get region
for /f "tokens=3 delims=\": %%i in ('curl -s ipinfo.io/%ip% ^| findstr /C:"city"') do set city=%%i

if "%city%"=="Snohomish" (
    git switch "Glacier_Peak_functional_branch" --force
) else if "%city%"=="Bellevue" (
    git switch "SAM_functional_branch" --force
) else (
    git switch main --force
)

call gradlew build
pause
