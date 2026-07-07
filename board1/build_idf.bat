@echo off
setlocal

REM Go to the directory where this script is located
cd /d "%~dp0"

REM Your ESP-IDF path (change this to match your installation)
set IDF_PATH=C:\path\to\esp-idf

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

REM Change esp32s3 to esp32 if you are using normal ESP32
idf.py set-target esp32p4
idf.py build

if errorlevel 1 (
    echo [ERROR] Build failed.
    exit /b 1
)

echo [OK] Build finished successfully.
endlocal