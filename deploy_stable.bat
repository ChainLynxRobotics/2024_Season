@echo on
set JAVA_HOME="C:\Users\Public\wpilib\2024\jdk
setlocal

ping -n 1 google.com > nul
if %errorlevel% neq 0 (
    echo Not connected to the internet, defaulting to deploying main branch.
    goto :end
)

for /f "tokens=2 delims=:" %%a in ('"ipconfig | findstr IPv4"') do set ip=%%a
set ip=%ip:~1%

rem using ipinfo.io API to get region
for /f "tokens=3 delims=\": %%i in ('curl -s ipinfo.io/%ip% ^| findstr /C:"city"') do set city=%%i

if "%city%"=="Snohomish" (
    git switch --force "Glacier_Peak_functional_branch"
) else if "%city%"=="Bellevue" (
    git switch --force "SAM_functional_branch"
) else (
    :end
    git switch --force main
)

call gradlew deploy
pause
