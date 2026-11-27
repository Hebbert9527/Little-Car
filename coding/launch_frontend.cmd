@echo off
chcp 65001 >nul
setlocal EnableExtensions

set "SCRIPT_DIR=%~dp0"
set "FW_DIR=%SCRIPT_DIR%codingday4_work\firmware_source"
set "RUN_PS1=%FW_DIR%\tools\run.ps1"
set "DEFAULT_BLE=C4:25:01:17:00:02"

if not exist "%RUN_PS1%" (
    echo [ERROR] Cannot find frontend launcher: "%RUN_PS1%"
    pause
    exit /b 1
)

echo ========================================================
echo   Week4 / NoeticMaze Frontend Launcher
echo ========================================================
echo   [1] Live BLE  (default: %DEFAULT_BLE%)  [Enter]
echo   [2] Live BLE  (enter custom MAC address)
echo   [3] Live Serial COM Port
echo   [4] Demo Mode (offline preview)
echo ========================================================
set "CHOICE="
set /p "CHOICE=Select mode [1-4, default 1]: "
if "%CHOICE%"=="" set "CHOICE=1"

if "%CHOICE%"=="1" goto :RUN_BLE_DEFAULT
if "%CHOICE%"=="2" goto :RUN_BLE_CUSTOM
if "%CHOICE%"=="3" goto :RUN_SERIAL
if "%CHOICE%"=="4" goto :RUN_DEMO

echo Invalid selection: %CHOICE%
pause
exit /b 1

:RUN_BLE_DEFAULT
echo Starting Live BLE frontend (%DEFAULT_BLE%)...
powershell -NoProfile -ExecutionPolicy Bypass -File "%RUN_PS1%" -Mode Live -BleAddress "%DEFAULT_BLE%"
goto :DONE

:RUN_BLE_CUSTOM
set "CUSTOM_BLE="
set /p "CUSTOM_BLE=Enter BLE MAC address (e.g. C4:25:01:17:00:02): "
if "%CUSTOM_BLE%"=="" set "CUSTOM_BLE=%DEFAULT_BLE%"
echo Starting Live BLE frontend (%CUSTOM_BLE%)...
powershell -NoProfile -ExecutionPolicy Bypass -File "%RUN_PS1%" -Mode Live -BleAddress "%CUSTOM_BLE%"
goto :DONE

:RUN_SERIAL
echo Available COM ports:
"%FW_DIR%\.venv\Scripts\python.exe" -B "%FW_DIR%\pc\monitor.py" --list-ports
set "COM_PORT="
set /p "COM_PORT=Enter COM port (e.g. COM5): "
if "%COM_PORT%"=="" (
    echo No COM port entered.
    pause
    exit /b 1
)
echo Starting Live Serial frontend (%COM_PORT%)...
powershell -NoProfile -ExecutionPolicy Bypass -File "%RUN_PS1%" -Mode Live -Port "%COM_PORT%"
goto :DONE

:RUN_DEMO
echo Starting Demo frontend...
powershell -NoProfile -ExecutionPolicy Bypass -File "%RUN_PS1%" -Mode Demo
goto :DONE

:DONE
set "EXIT_CODE=%ERRORLEVEL%"
if not "%EXIT_CODE%"=="0" (
    echo.
    echo [Frontend exited with code %EXIT_CODE%]
    pause
)
exit /b %EXIT_CODE%
