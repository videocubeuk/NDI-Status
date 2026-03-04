# NDI Status Monitor

A network-attached display that automatically discovers and shows all active NDI sources on the local network, built on the Olimex ESP32-PoE-ISO with the MOD-2.8LCD touchscreen.

## Features

- **Auto-discovery** — scans for NDI sources via mDNS every 3 seconds, no configuration required
- **Live status** — green dot = reachable, red dot = unreachable (source stays visible when lost)
- **Three-line entries** — machine name, stream name, IP address per source (up to 16 sources)
- **Touch scroll** — swipe up/down on the display to scroll through the list
- **Scroll indicators** — arrow triangles appear when more entries exist above or below
- **Ethernet status** — coloured dot in the title bar shows link state (green/red)
- **Powered over Ethernet** — no USB power required in production

## Hardware

| Component | Part |
|-----------|------|
| MCU board | [Olimex ESP32-PoE-ISO](https://www.olimex.com/Products/IoT/ESP32/ESP32-POE-ISO/) |
| Display | [Olimex MOD-2.8LCD](https://www.olimex.com/Products/Modules/LCD/MOD-2.8LCD/) (ILI9341, 240×320, XPT2046 touch) |
| Connection | UEXT connector (no additional wiring needed for SPI display) |
| Network | 10/100 Ethernet via LAN8720 PHY (PoE capable) |

### UEXT Pin Mapping

| Signal   | ESP32 GPIO |
|----------|-----------|
| TFT DC   | 15 |
| TFT CS   | 5 |
| MOSI     | 2 |
| SCLK     | 14 |
| MISO     | 35 |
| Touch CS | 4 |

> **Note:** Touch CS (GPIO4) requires a wire from the XPT2046 CS pin on the MOD-2.8LCD to GPIO4 on the ESP32-PoE-ISO. If not wired, the display works but touch scrolling is disabled.

## Software

- **Framework:** Arduino (via PlatformIO)
- **Platform:** `espressif32` @ 6.x
- **Library:** [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI) by Bodmer
- **mDNS:** ESP-IDF `mdns.h` called directly to access service instance names

## Building & Flashing

Requires [PlatformIO](https://platformio.org/).

```bash
# Build
pio run

# Flash
pio run --target upload --upload-port COM7
```

Or use the PlatformIO IDE tasks in VS Code.

## Display Layout

```
┌─────────────────────────────┐
│  NDI Sources              ● │  ← title bar, dot = Ethernet status
├─────────────────────────────┤
│ ● STUDIO-PC              ▲  │  ← machine name  (green/red dot = reachable)
│   Camera 1                  │  ← stream name   (dimmed)
│   192.168.1.42              │  ← IP address
│                             │
│ ● EDIT-SUITE                │
│   Program Out               │
│   192.168.1.55              │
│                          ▼  │  ← down arrow = more entries below
└─────────────────────────────┘
```

Swipe up to scroll down, swipe down to scroll up.

## How NDI Discovery Works

The device uses mDNS PTR queries for the `_ndi._tcp` service type. The NDI source name advertised by NDI tools follows the format `"MACHINE (Stream Name)"` — the code extracts just the stream name portion from the parentheses for a cleaner display.

Sources that become unreachable are kept in the list with a red indicator rather than disappearing, making it easy to spot dropped feeds.

## Project Structure

```
src/
  main.cpp                        — all application code
  Board_Pinout.h                  — pin definitions (reference, used by TFT_eSPI build flags)
  NDI_Live_IP_address_bookv1.ino  — empty placeholder (code moved to main.cpp)
platformio.ini                    — build configuration
```
