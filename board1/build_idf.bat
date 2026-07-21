@echo off
setlocal

REM Go to the directory where this script is located
cd /d "%~dp0"

REM Your ESP-IDF path
set IDF_PATH=D:\esp\v5.5.4\esp-idf

if not exist "%IDF_PATH%\export.bat" (
    echo [ERROR] Cannot find ESP-IDF export.bat:
    echo %IDF_PATH%\export.bat
    exit /b 1
)

echo [INFO] Using ESP-IDF:
echo %IDF_PATH%

REM Load ESP-IDF environment for this shell only
call "%IDF_PATH%\export.bat"

echo [INFO] Checking tools...
where python
where idf.py
where ninja
where cmake

echo [INFO] Building project...

REM Set the target only for a fresh project. Running set-target every time
REM triggers fullclean and recreates sdkconfig, losing menuconfig selections.
if not exist "sdkconfig" (
    echo [INFO] No sdkconfig found, setting target to esp32p4...
    idf.py set-target esp32p4
    if errorlevel 1 (
        echo [ERROR] Failed to set target.
        exit /b 1
    )
)

idf.py build

if errorlevel 1 (
    echo [ERROR] Build failed.
    exit /b 1
)

echo [OK] Build finished successfully.
endlocal
