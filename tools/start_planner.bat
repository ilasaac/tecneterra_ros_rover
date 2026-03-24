@echo off
:: Kill any process listening on port 8089
for /f "tokens=5" %%p in ('netstat -ano ^| findstr ":8089 " ^| findstr "LISTENING"') do (
    echo Killing old process %%p on port 8089...
    taskkill /f /pid %%p >nul 2>&1
)

:: Clear Python bytecode cache for tools directory
if exist "%~dp0__pycache__" (
    echo Clearing __pycache__...
    rmdir /s /q "%~dp0__pycache__"
)

:: Start planner with no-cache flag
echo Starting mission_planner.py...
cd /d "%~dp0.."
python -B tools/mission_planner.py %*
