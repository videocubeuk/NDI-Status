# NDI Status Precompiled Firmware

Target board: `esp32-poe-iso`
Build date: `2026-03-11`

## Files
- `full_flash_0x0.bin` - single-file full image (recommended)
- `flash_windows.bat` - one-command flash script for Windows
- `bootloader.bin`, `partitions.bin`, `boot_app0.bin`, `firmware.bin` - split images (advanced/manual flashing)

## Easy upload (Windows)
Prerequisite: Python 3 and esptool (`pip install esptool`).

1. Connect the board by USB/UART.
2. Open Command Prompt in this folder.
3. Run:

```bat
flash_windows.bat COM7
```

Replace `COM7` with your board COM port.

## Manual single-file upload (any esptool)
Flash `full_flash_0x0.bin` at address `0x0`.

## Manual split-image upload
Use these offsets:
- `0x1000` -> `bootloader.bin`
- `0x8000` -> `partitions.bin`
- `0xe000` -> `boot_app0.bin`
- `0x10000` -> `firmware.bin`
