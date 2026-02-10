@echo off
echo Building test_runner.exe ...
 gcc -O2 -Wall -std=c99 -Isrc -DAVD_DEBUG_PRINT src/avoidance_preop.c src/avoidance_opmode.c test/runner_main.c -o test_runner.exe
if %errorlevel% neq 0 (
    echo BUILD FAILED
    pause
    exit /b 1
)
echo Build OK. Running tests...
echo.
test_runner.exe
pause
