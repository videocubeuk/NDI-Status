@echo off
setlocal

if "%~1"=="" (
  echo Usage: flash_windows.bat COM_PORT
  echo Example: flash_windows.bat COM7
  exit /b 1
)

set PORT=%~1
set BAUD=921600

where python >nul 2>&1
if errorlevel 1 (
  echo Python is required. Install Python 3 and run: pip install esptool
  exit /b 1
)

echo Flashing full image to %PORT%...
python -m esptool --chip esp32 --port %PORT% --baud %BAUD% write-flash 0x0 full_flash_0x0.bin

if errorlevel 1 (
  echo Flash failed.
  exit /b 1
)

echo Flash complete.
exit /b 0
