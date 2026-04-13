#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ETH.h>
#include <ESPmDNS.h>
#include <mdns.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <esp_wifi.h>
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <fcntl.h>

#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();

// ================== APP SETTINGS ==================
constexpr uint32_t APP_SETTINGS_MAGIC   = 0x4E445349;  // "NDSI"
constexpr uint8_t  APP_SETTINGS_VERSION = 1;
constexpr const char *APP_SETTINGS_FILE = "/app_settings.bin";

struct AppSettings {
  uint32_t magic;
  uint8_t  version;
  uint8_t  flipScreen;   // 1 = rotate display 180 degrees
  uint8_t  staticIP;     // 1 = use manual IP settings
  uint8_t  _pad;
  uint8_t  ip[4];
  uint8_t  mask[4];
  uint8_t  gw[4];
  uint8_t  dns[4];
};

AppSettings appSettings = {
  APP_SETTINGS_MAGIC, APP_SETTINGS_VERSION,
  0, 0, 0,                   // flipScreen, staticIP, _pad
  {192, 168, 0, 10},         // ip
  {255, 255, 255, 0},        // mask
  {192, 168, 0, 1},          // gw
  {8, 8, 8, 8},              // dns
};

// Settings page edit state: which IP field is open (-1=none; 0=IP,1=subnet,2=gw,3=dns)
static int     settingsEditField = -1;
static uint8_t settingsEditOcts[4];

// ================== TOUCH (AR1021 over I2C) ==================
constexpr int     AR1021_SDA_PIN  = 13;
constexpr int     AR1021_SCL_PIN  = 16;
constexpr uint8_t AR1021_I2C_ADDR = 0x4D;

constexpr int CALIB_BUTTON_PIN = 34;
constexpr bool CALIB_BUTTON_ACTIVE_LOW = true;

constexpr uint32_t TOUCH_CAL_MAGIC = 0x54434C31;  // "TCL1"
constexpr uint16_t TOUCH_CAL_VERSION = 2;           // v2: directional corner storage
constexpr const char *TOUCH_CAL_FILE = "/touch_cal.bin";

// Default raw values assume axis increases top-left to bottom-right.
// These are overwritten after calibration; direction is preserved, not just min/max.
constexpr uint16_t DEFAULT_TL_RAW_X = 120;
constexpr uint16_t DEFAULT_BR_RAW_X = 3950;
constexpr uint16_t DEFAULT_TL_RAW_Y = 120;
constexpr uint16_t DEFAULT_BR_RAW_Y = 3950;

struct TouchCalibration {
  uint32_t magic;
  uint16_t version;
  // Raw sensor values at the screen top-left and bottom-right corners.
  // Storing the actual corner values (not just min/max) preserves axis direction.
  uint16_t tlRawX;   // raw X when top-left corner was touched
  uint16_t brRawX;   // raw X when bottom-right corner was touched
  uint16_t tlRawY;   // raw Y when top-left corner was touched
  uint16_t brRawY;   // raw Y when bottom-right corner was touched
};

TouchCalibration touchCal = {
  TOUCH_CAL_MAGIC,
  TOUCH_CAL_VERSION,
  DEFAULT_TL_RAW_X,
  DEFAULT_BR_RAW_X,
  DEFAULT_TL_RAW_Y,
  DEFAULT_BR_RAW_Y
};

// ================== UI CONSTANTS ==================
#define SCREEN_W 240
#define SCREEN_H 320
#define TITLE_H  28
#define LINE_H   11
#define MARGIN   4

// ================== ETHERNET ==================
volatile bool ethConnected = false;

// ================== NDI DATA ==================
#define MAX_NDI 16

struct NDISource {
  String name;    // machine hostname
  String stream;  // NDI stream name (extracted from instance)
  String ip;
  bool reachable;
  bool isReceiver;
};

NDISource ndiList[MAX_NDI];
int ndiCount = 0;
int startIndex = 0;
int selectedIndex = -1;

bool lastEthConnected = false;
int  lastStartIndex   = -1;
int  lastSelected     = -1;

struct NDIShadow {
  String name;
  String stream;
  String ip;
  bool reachable;
  bool isReceiver;
};

NDIShadow lastNDI[MAX_NDI];
int lastNDICount = 0;

enum UiPage {
  PAGE_NDI_SOURCES = 0,
  PAGE_ALL_DEVICES = 1,
  PAGE_IP_SCANNER  = 2,
  PAGE_ARTNET      = 3,
  PAGE_DANTE       = 4,
  PAGE_HQNET       = 5,
  PAGE_WIFI_SCAN   = 6,
  PAGE_MENU        = 7,
  PAGE_NET_INFO    = 8,
  PAGE_SETTINGS    = 9,
};

// PAGE_LAST is the last page reachable by title-bar cycling (not menu/wifi).
#define PAGE_LAST PAGE_HQNET

UiPage currentPage = PAGE_NDI_SOURCES;

// "All Devices" view — shows every mDNS host found, not just NDI sources.
NDISource allHostsList[MAX_NDI];
int       allHostsCount     = 0;
int       lastAllHostsCount = -1;

// ================== IP SCANNER ==================
#define MAX_SCAN_HOSTS 128

struct IPScanHost {
  String hostname;
  String source;
  String ip;
  int openPort;
};

IPScanHost scanHosts[MAX_SCAN_HOSTS];
int        scanHostCount = 0;

bool       ipScanRunning      = false;
bool       ipScanFinished     = false;
bool       ipScanDirty        = true;
uint32_t   ipScanCurrentHost  = 0;
uint32_t   ipScanStartHost    = 0;
uint32_t   ipScanEndHost      = 0;
uint32_t   ipScanOwnIp        = 0;
uint32_t   ipScanNetworkHostCount = 0;

int        lastScanHostCount  = -1;
bool       lastIpScanRunning  = false;
uint32_t   lastIpScanCurrentHost = 0;

// ================== ART-NET SCANNER ==================
#define MAX_ARTNET_HOSTS 64

struct ArtNetHost {
  String ip;
  String shortName;   // from ArtPollReply bytes 26-43
  String longName;    // from ArtPollReply bytes 44-107
  uint16_t numPorts;
  uint8_t  oem[2];
};

ArtNetHost artnetHosts[MAX_ARTNET_HOSTS];
int        artnetHostCount   = 0;
bool       artnetScanRunning = false;
bool       artnetScanDone    = false;
bool       artnetScanDirty   = true;
int        lastArtnetCount   = -1;
bool       lastArtnetRunning = false;

// ================== DANTE / AES67 SCANNER ==================
#define MAX_DANTE_HOSTS 64

struct DanteHost {
  String ip;
  String name;
  String proto;  // "Dante", "AES67", "Ravenna"
};

DanteHost danteHosts[MAX_DANTE_HOSTS];
int       danteHostCount   = 0;
bool      danteScanRunning = false;
bool      danteScanDone    = false;
bool      danteScanDirty   = true;
int       lastDanteCount   = -1;
bool      lastDanteRunning = false;

// ================== HQNET SCANNER ==================
#define MAX_HQNET_HOSTS 64
#define HQNET_PORT 3804

struct HQNetHost {
  String ip;
  String name;       // device name from HiQnet Announce reply
  String devType;    // device type string
};

HQNetHost hqnetHosts[MAX_HQNET_HOSTS];
int       hqnetHostCount   = 0;
bool      hqnetScanRunning = false;
bool      hqnetScanDone    = false;
bool      hqnetScanDirty   = true;
int       lastHqnetCount   = -1;
bool      lastHqnetRunning = false;
bool      hqnetEnriching   = false;

// ================== WIFI CHANNEL SCANNER ==================
#define MAX_WIFI_NETWORKS 32

struct WifiNetwork {
  char ssid[33];
  int  channel;
  int  rssi;
  uint8_t authmode;  // wifi_auth_mode_t value
};

WifiNetwork wifiNetworks[MAX_WIFI_NETWORKS];
int         wifiNetworkCount = 0;
bool        wifiScanRunning  = false;
bool        wifiScanDone     = false;
bool        wifiScanDirty    = true;
int         lastWifiCount    = -1;
bool        lastWifiRunning  = false;

// ================== HELPERS ==================
int rowY(int row) {
  return TITLE_H + MARGIN + (row * LINE_H * 3);
}

int visibleRowsForCurrentPage();

#define VISIBLE_ROWS  ((SCREEN_H - TITLE_H) / (LINE_H * 3))

bool isCalButtonPressed() {
  int v = digitalRead(CALIB_BUTTON_PIN);
  return CALIB_BUTTON_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

bool isAllDevicesPage() {
  return currentPage == PAGE_ALL_DEVICES;
}

bool isIpScannerPage() {
  return currentPage == PAGE_IP_SCANNER;
}

bool isArtNetPage() {
  return currentPage == PAGE_ARTNET;
}

bool isDantePage() {
  return currentPage == PAGE_DANTE;
}

bool isHQNetPage() {
  return currentPage == PAGE_HQNET;
}

bool isMenuPage() {
  return currentPage == PAGE_MENU;
}

bool isWifiScanPage() {
  return currentPage == PAGE_WIFI_SCAN;
}

bool isNetInfoPage() {
  return currentPage == PAGE_NET_INFO;
}

bool isSettingsPage() {
  return currentPage == PAGE_SETTINGS;
}

bool isProtocolScanPage() {
  return currentPage == PAGE_ARTNET || currentPage == PAGE_DANTE ||
         currentPage == PAGE_HQNET  || currentPage == PAGE_WIFI_SCAN;
}

uint32_t ipToU32(const IPAddress &ip) {
  return ((uint32_t)ip[0] << 24) | ((uint32_t)ip[1] << 16) | ((uint32_t)ip[2] << 8) | (uint32_t)ip[3];
}

IPAddress u32ToIp(uint32_t value) {
  return IPAddress((value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF);
}

bool readAR1021Raw(uint16_t &rawX, uint16_t &rawY, bool &pressed) {
  uint8_t buf[5] = {};
  uint8_t count = Wire.requestFrom(AR1021_I2C_ADDR, (uint8_t)5);
  if (count < 5) return false;
  for (int i = 0; i < 5; i++) buf[i] = Wire.read();

  if ((buf[0] & 0x80) == 0) return false;  // invalid header byte

  pressed = (buf[0] & 0x01) != 0;
  // 12-bit coordinates packed as 7 LSB + 5 MSB bits.
  rawX = (uint16_t)(buf[1] & 0x7F) | ((uint16_t)(buf[2] & 0x1F) << 7);
  rawY = (uint16_t)(buf[3] & 0x7F) | ((uint16_t)(buf[4] & 0x1F) << 7);
  return true;
}

bool readAR1021Touch(int16_t &x, int16_t &y, bool &pressed) {
  // Rate-limit to once every 20 ms to avoid flooding the I2C bus.
  static unsigned long lastPollMs = 0;
  unsigned long now = millis();
  if (now - lastPollMs < 20) return false;
  lastPollMs = now;

  uint16_t rawX = 0;
  uint16_t rawY = 0;
  if (!readAR1021Raw(rawX, rawY, pressed)) return false;

  // Clamp to the calibrated range (use min/max regardless of axis direction).
  rawX = constrain(rawX,
    min(touchCal.tlRawX, touchCal.brRawX),
    max(touchCal.tlRawX, touchCal.brRawX));
  rawY = constrain(rawY,
    min(touchCal.tlRawY, touchCal.brRawY),
    max(touchCal.tlRawY, touchCal.brRawY));

  // map() handles both normal and reversed axes: tlRaw→0, brRaw→SCREEN-1.
  if (appSettings.flipScreen) {
    x = (int16_t)map(rawX, touchCal.brRawX, touchCal.tlRawX, 0, SCREEN_W - 1);
    y = (int16_t)map(rawY, touchCal.brRawY, touchCal.tlRawY, 0, SCREEN_H - 1);
  } else {
    x = (int16_t)map(rawX, touchCal.tlRawX, touchCal.brRawX, 0, SCREEN_W - 1);
    y = (int16_t)map(rawY, touchCal.tlRawY, touchCal.brRawY, 0, SCREEN_H - 1);
  }
  return true;
}

void clearTouchPackets() {
  // Drain any queued I2C packets from the AR1021.
  for (int i = 0; i < 16; i++) {
    uint8_t n = Wire.requestFrom(AR1021_I2C_ADDR, (uint8_t)5);
    while (Wire.available()) Wire.read();
    if (n == 0) break;
    delay(5);
  }
}

void ar1021EnableTouch() {
  // Command 0x12: Enable Sending of Coordinates (data 0x01 = enable).
  // Format: 0x55 <byte_count> <cmd_id> <data...>
  const uint8_t cmd[] = {0x55, 0x02, 0x12, 0x01};
  Wire.beginTransmission(AR1021_I2C_ADDR);
  Wire.write(cmd, sizeof(cmd));
  Wire.endTransmission();
  delay(100);
  clearTouchPackets();  // discard ACK response bytes
}

bool saveTouchCalibration() {
  File f = SPIFFS.open(TOUCH_CAL_FILE, FILE_WRITE);
  if (!f) return false;

  size_t written = f.write((const uint8_t *)&touchCal, sizeof(touchCal));
  f.close();
  return written == sizeof(touchCal);
}

bool loadTouchCalibration() {
  if (!SPIFFS.exists(TOUCH_CAL_FILE)) return false;

  File f = SPIFFS.open(TOUCH_CAL_FILE, FILE_READ);
  if (!f) return false;

  TouchCalibration loaded = {};
  size_t read = f.read((uint8_t *)&loaded, sizeof(loaded));
  f.close();
  if (read != sizeof(loaded)) return false;

  bool headerOk = loaded.magic == TOUCH_CAL_MAGIC && loaded.version == TOUCH_CAL_VERSION;
  bool rangesOk =
    abs((int)loaded.brRawX - (int)loaded.tlRawX) > 100 &&
    abs((int)loaded.brRawY - (int)loaded.tlRawY) > 100;

  if (!headerOk || !rangesOk) return false;

  touchCal = loaded;
  return true;
}

bool saveAppSettings() {
  File f = SPIFFS.open(APP_SETTINGS_FILE, FILE_WRITE);
  if (!f) return false;
  size_t written = f.write((const uint8_t*)&appSettings, sizeof(appSettings));
  f.close();
  return written == sizeof(appSettings);
}

bool loadAppSettings() {
  if (!SPIFFS.exists(APP_SETTINGS_FILE)) return false;
  File f = SPIFFS.open(APP_SETTINGS_FILE, FILE_READ);
  if (!f) return false;
  AppSettings loaded = {};
  size_t n = f.read((uint8_t*)&loaded, sizeof(loaded));
  f.close();
  if (n != sizeof(loaded)) return false;
  if (loaded.magic != APP_SETTINGS_MAGIC || loaded.version != APP_SETTINGS_VERSION) return false;
  appSettings = loaded;
  return true;
}

void drawCalTarget(int x, int y, const char *label) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 10);
  tft.print("Touch calibration");
  tft.setTextSize(1);
  tft.setCursor(10, 40);
  tft.print(label);
  tft.setCursor(10, 54);
  tft.print("Press and release target");

  tft.drawCircle(x, y, 10, TFT_YELLOW);
  tft.drawLine(x - 14, y, x + 14, y, TFT_YELLOW);
  tft.drawLine(x, y - 14, x, y + 14, TFT_YELLOW);
}

bool captureTouchAverage(uint16_t &outX, uint16_t &outY, uint32_t timeoutMs) {
  unsigned long start = millis();
  bool seenPress = false;
  uint32_t sx = 0;
  uint32_t sy = 0;
  uint16_t count = 0;
  uint32_t totalBytesReceived = 0;

  while (millis() - start < timeoutMs) {
    uint16_t rawX = 0;
    uint16_t rawY = 0;
    bool pressed = false;

    if (readAR1021Raw(rawX, rawY, pressed)) {
      totalBytesReceived++;
      if (pressed) {
        seenPress = true;
        if (count < 64) {
          sx += rawX;
          sy += rawY;
          count++;
        }
      } else if (seenPress) {
        if (count >= 3) {
          outX = (uint16_t)(sx / count);
          outY = (uint16_t)(sy / count);
          return true;
        }
        seenPress = false;
        sx = 0;
        sy = 0;
        count = 0;
      }
    }
    delay(20);  // Poll at ~50 Hz — matches AR1021 output rate
  }
  // Let caller know whether bytes were arriving at all.
  Serial.printf("[AR1021] captureTouchAverage timeout: %u raw bytes seen\n", totalBytesReceived);
  return false;
}

bool runTouchCalibration() {
  ar1021EnableTouch();
  clearTouchPackets();

  const int margin = 25;
  uint16_t tlX = 0;
  uint16_t tlY = 0;
  uint16_t brX = 0;
  uint16_t brY = 0;

  drawCalTarget(margin, margin, "1/2: top-left");
  if (!captureTouchAverage(tlX, tlY, 15000)) {
    tft.setTextSize(1);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(10, 80);
    tft.print("Timeout - check wiring/baud");
    delay(2000);
    return false;
  }

  delay(250);
  clearTouchPackets();

  drawCalTarget(SCREEN_W - margin, SCREEN_H - margin, "2/2: bottom-right");
  if (!captureTouchAverage(brX, brY, 15000)) {
    tft.setTextSize(1);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(10, 80);
    tft.print("Timeout - check wiring/baud");
    delay(2000);
    return false;
  }

  if (abs((int)brX - (int)tlX) <= 100 || abs((int)brY - (int)tlY) <= 100) {
    return false;
  }

  TouchCalibration previous = touchCal;
  // Store the actual per-corner raw values to preserve axis direction.
  touchCal.tlRawX = tlX;
  touchCal.brRawX = brX;
  touchCal.tlRawY = tlY;
  touchCal.brRawY = brY;

  if (!saveTouchCalibration()) {
    touchCal = previous;
    return false;
  }

  return true;
}

// ================== ETHERNET EVENTS ==================
void onEthEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_ETH_CONNECTED:
      ethConnected = true;
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH IP: ");
      Serial.println(ETH.localIP());
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
    case ARDUINO_EVENT_ETH_STOP:
      ethConnected = false;
      break;
    default:
      break;
  }
}

// ================== UI ==================
void drawSplashScreen() {
  tft.fillScreen(TFT_BLACK);

  // Header bar
  const uint16_t NAVY = 0x000F;
  tft.fillRect(0, 70, SCREEN_W, 54, NAVY);

  // "VideoCube" — size 4 = 24×32px per char, 9 chars = 216px wide
  tft.setTextSize(4);
  tft.setTextColor(TFT_WHITE, NAVY);
  tft.setCursor(12, 81);
  tft.print("videocube");

  // Accent line below header
  tft.drawFastHLine(0, 124, SCREEN_W, 0x8410);

  // Subtitle — "Network Tools Tester"
  tft.setTextSize(2);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(0, 136);   // 20 chars × 12px = 240px, full width
  tft.print("Network Tools Tester");

  // Bottom status hint
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(70, 295);
  tft.print("Starting up...");
}

void drawEthStatus() {
  uint16_t c = ethConnected ? TFT_GREEN : TFT_RED;
  tft.fillCircle(SCREEN_W - 10, TITLE_H / 2, 5, c);
}

void drawTitle() {
  tft.fillRect(0, 0, SCREEN_W, TITLE_H, TFT_DARKGREY);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setCursor(MARGIN, 6);
  if (currentPage == PAGE_NDI_SOURCES) {
    tft.print("NDI Sources");
  } else if (currentPage == PAGE_ALL_DEVICES) {
    tft.print("All mDNS");
  } else if (currentPage == PAGE_IP_SCANNER) {
    tft.print("IP Scanner");
  } else if (currentPage == PAGE_ARTNET) {
    tft.print("Art-Net");
  } else if (currentPage == PAGE_DANTE) {
    tft.print("Dante/AES67");
  } else if (currentPage == PAGE_HQNET) {
    tft.print("HiQnet");
  } else if (currentPage == PAGE_WIFI_SCAN) {
    tft.print("WiFi Scan");
  } else if (currentPage == PAGE_NET_INFO) {
    tft.print("Net Info");
  } else if (currentPage == PAGE_SETTINGS) {
    tft.print("Settings");
  } else {
    tft.print("Network Tools Menu");
  }
  drawEthStatus();
}

int pageItemCount() {
  if (currentPage == PAGE_NDI_SOURCES) return ndiCount;
  if (currentPage == PAGE_ALL_DEVICES) return allHostsCount;
  if (currentPage == PAGE_IP_SCANNER)  return scanHostCount;
  if (currentPage == PAGE_ARTNET)      return artnetHostCount;
  if (currentPage == PAGE_DANTE)       return danteHostCount;
  if (currentPage == PAGE_HQNET)       return hqnetHostCount;
  if (currentPage == PAGE_WIFI_SCAN)   return wifiNetworkCount;
  return 0;  // PAGE_MENU has no scrollable list
}

void drawScrollArrows() {
  int total = pageItemCount();
  int visibleRows = visibleRowsForCurrentPage();
  tft.fillRect(SCREEN_W - 12, TITLE_H, 12, SCREEN_H - TITLE_H, TFT_BLACK);
  if (startIndex > 0) {
    tft.fillTriangle(
      SCREEN_W - 11, TITLE_H + 14,
      SCREEN_W - 6,  TITLE_H + 4,
      SCREEN_W - 1,  TITLE_H + 14,
      TFT_WHITE);
  }
  if (startIndex + visibleRows < total) {
    tft.fillTriangle(
      SCREEN_W - 11, SCREEN_H - 14,
      SCREEN_W - 6,  SCREEN_H - 4,
      SCREEN_W - 1,  SCREEN_H - 14,
      TFT_WHITE);
  }
}

uint16_t dotColor(int i) {
  const NDISource &src = isAllDevicesPage() ? allHostsList[i] : ndiList[i];
  if (isAllDevicesPage())
    return src.reachable ? TFT_CYAN : TFT_DARKGREY;
  if (src.isReceiver)
    return src.reachable ? TFT_CYAN : TFT_DARKGREY;
  return src.reachable ? TFT_GREEN : TFT_RED;
}

void drawIpScannerPage() {
  tft.fillRect(0, TITLE_H, SCREEN_W, SCREEN_H - TITLE_H, TFT_BLACK);
  tft.setTextSize(1);

  uint16_t buttonBg = ipScanRunning ? TFT_DARKGREY : TFT_DARKGREEN;
  tft.fillRoundRect(8, TITLE_H + 6, SCREEN_W - 28, 20, 4, buttonBg);
  tft.setTextColor(TFT_WHITE, buttonBg);
  tft.setCursor(14, TITLE_H + 12);
  tft.print(ipScanRunning ? "Scanning..." : "Tap here to scan network");

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(10, TITLE_H + 32);
  if (!ethConnected) {
    tft.print("Ethernet disconnected");
  } else if (ipScanRunning) {
    uint32_t done = (ipScanCurrentHost >= ipScanStartHost) ? (ipScanCurrentHost - ipScanStartHost) : 0;
    tft.printf("Progress: %lu/%lu", (unsigned long)done, (unsigned long)ipScanNetworkHostCount);
  } else if (ipScanFinished) {
    tft.printf("Scan complete: %d hosts", scanHostCount);
  } else {
    tft.print("Ready");
  }

  int y = TITLE_H + 46;
  int rows = (SCREEN_H - y) / (LINE_H * 3);
  int endIndex = min(startIndex + rows, scanHostCount);

  for (int i = startIndex; i < endIndex; i++) {
    bool hasName = scanHosts[i].hostname.length() > 0;
    tft.fillCircle(6, y + 4, 4, hasName ? TFT_CYAN : TFT_GREEN);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(14, y);
    tft.print(hasName ? scanHosts[i].hostname : scanHosts[i].ip);

    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(14, y + LINE_H);
    tft.print(scanHosts[i].ip);

    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.setCursor(14, y + (LINE_H * 2));
    if (scanHosts[i].source.length() > 0) {
      tft.printf("%s  port:%d", scanHosts[i].source.c_str(), scanHosts[i].openPort);
    } else {
      tft.printf("port:%d", scanHosts[i].openPort);
    }
    y += LINE_H * 3;
  }

  drawScrollArrows();
}

// ================== ARTNET UI ==================
void drawArtNetPage() {
  tft.fillRect(0, TITLE_H, SCREEN_W, SCREEN_H - TITLE_H, TFT_BLACK);
  tft.setTextSize(1);

  uint16_t buttonBg = artnetScanRunning ? TFT_DARKGREY : 0x8400; // dark orange
  tft.fillRoundRect(8, TITLE_H + 6, SCREEN_W - 28, 20, 4, buttonBg);
  tft.setTextColor(TFT_WHITE, buttonBg);
  tft.setCursor(14, TITLE_H + 12);
  tft.print(artnetScanRunning ? "Listening..." : "Tap to send ArtPoll");

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(10, TITLE_H + 32);
  if (!ethConnected) {
    tft.print("Ethernet disconnected");
  } else if (artnetScanRunning) {
    tft.print("Collecting replies...");
  } else if (artnetScanDone) {
    tft.printf("Found: %d device(s)", artnetHostCount);
  } else {
    tft.print("Ready");
  }

  int y = TITLE_H + 46;
  int rows = (SCREEN_H - y) / (LINE_H * 3);
  int endIndex = min(startIndex + rows, artnetHostCount);

  for (int i = startIndex; i < endIndex; i++) {
    tft.fillCircle(6, y + 4, 4, 0xFC00); // amber
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(14, y);
    if (artnetHosts[i].shortName.length() > 0)
      tft.print(artnetHosts[i].shortName);
    else
      tft.print(artnetHosts[i].ip);

    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(14, y + LINE_H);
    tft.print(artnetHosts[i].ip);

    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.setCursor(14, y + LINE_H * 2);
    if (artnetHosts[i].longName.length() > 0)
      tft.print(artnetHosts[i].longName);
    else
      tft.printf("ports:%d  OEM:%02X%02X",
        artnetHosts[i].numPorts,
        artnetHosts[i].oem[0], artnetHosts[i].oem[1]);
    y += LINE_H * 3;
  }

  drawScrollArrows();
}

// ================== DANTE UI ==================
void drawDantePage() {
  tft.fillRect(0, TITLE_H, SCREEN_W, SCREEN_H - TITLE_H, TFT_BLACK);
  tft.setTextSize(1);

  uint16_t buttonBg = danteScanRunning ? TFT_DARKGREY : 0x000F; // dark navy
  tft.fillRoundRect(8, TITLE_H + 6, SCREEN_W - 28, 20, 4, buttonBg);
  tft.setTextColor(TFT_WHITE, buttonBg);
  tft.setCursor(14, TITLE_H + 12);
  tft.print(danteScanRunning ? "Scanning..." : "Tap to scan Dante/AES67");

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(10, TITLE_H + 32);
  if (!ethConnected) {
    tft.print("Ethernet disconnected");
  } else if (danteScanRunning) {
    tft.print("Listening on port 4440...");
  } else if (danteScanDone) {
    tft.printf("Found: %d device(s)", danteHostCount);
  } else {
    tft.print("Ready");
  }

  int y = TITLE_H + 46;
  int rows = (SCREEN_H - y) / (LINE_H * 3);
  int endIndex = min(startIndex + rows, danteHostCount);

  for (int i = startIndex; i < endIndex; i++) {
    uint16_t dot = (danteHosts[i].proto == "AES67" || danteHosts[i].proto == "Ravenna")
      ? 0x07FF : TFT_BLUE; // cyan for AES67/Ravenna, blue for Dante
    tft.fillCircle(6, y + 4, 4, dot);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(14, y);
    tft.print(danteHosts[i].name.length() > 0 ? danteHosts[i].name : danteHosts[i].ip);

    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(14, y + LINE_H);
    tft.print(danteHosts[i].ip);

    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.setCursor(14, y + LINE_H * 2);
    tft.print(danteHosts[i].proto);
    y += LINE_H * 3;
  }

  drawScrollArrows();
}

// ================== HQNET UI ==================
void drawHQNetPage() {
  tft.fillRect(0, TITLE_H, SCREEN_W, SCREEN_H - TITLE_H, TFT_BLACK);
  tft.setTextSize(1);

  uint16_t buttonBg = hqnetScanRunning ? TFT_DARKGREY : 0x0210; // dark green
  tft.fillRoundRect(8, TITLE_H + 6, SCREEN_W - 28, 20, 4, buttonBg);
  tft.setTextColor(TFT_WHITE, buttonBg);
  tft.setCursor(14, TITLE_H + 12);
  tft.print(hqnetScanRunning ? "Listening..." : "Tap to send HiQnet Discover");

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(10, TITLE_H + 32);
  if (!ethConnected) {
    tft.print("Ethernet disconnected");
  } else if (hqnetScanRunning && hqnetEnriching) {
    tft.print("Resolving names...");
  } else if (hqnetScanRunning) {
    tft.print("Collecting replies...");
  } else if (hqnetScanDone) {
    tft.printf("Found: %d device(s)", hqnetHostCount);
  } else {
    tft.print("Ready");
  }

  int y = TITLE_H + 46;
  int rows = (SCREEN_H - y) / (LINE_H * 3);
  int endIndex = min(startIndex + rows, hqnetHostCount);

  for (int i = startIndex; i < endIndex; i++) {
    tft.fillCircle(6, y + 4, 4, 0x07E0); // green dot
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(14, y);
    tft.print(hqnetHosts[i].name.length() > 0 ? hqnetHosts[i].name : hqnetHosts[i].devType);

    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(14, y + LINE_H);
    tft.print(hqnetHosts[i].ip);

    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.setCursor(14, y + LINE_H * 2);
    tft.print(hqnetHosts[i].devType.length() > 0 ? hqnetHosts[i].devType : "HiQnet");
    y += LINE_H * 3;
  }

  drawScrollArrows();
}

// ================== MENU PAGE ==================
// Struct that describes one tile in the menu grid.
struct MenuTile {
  const char *label;
  const char *sublabel;
  UiPage      target;
  uint16_t    color;   // tile background colour
};

static const MenuTile MENU_TILES[] = {
  { "NDI",      "Sources",   PAGE_NDI_SOURCES, 0x0320 },  // dark green
  { "mDNS",     "All Devs",  PAGE_ALL_DEVICES, 0x0210 },
  { "IP",       "Scanner",   PAGE_IP_SCANNER,  0x8400 },  // dark orange
  { "Art-Net",  "Discover",  PAGE_ARTNET,      0x8008 },  // dark purple
  { "Dante",    "AES67",     PAGE_DANTE,       0x000F },  // dark navy
  { "HiQnet",   "Discover",  PAGE_HQNET,       0x0600 },
  { "WiFi",     "Ch Scan",   PAGE_WIFI_SCAN,   0x8410 },  // grey-blue
  { "Net",      "Info",      PAGE_NET_INFO,    0x2945 },  // steel blue
  { "Settings", "IP/Flip",   PAGE_SETTINGS,    0x3186 },  // dark slate
};
static const int MENU_TILE_COUNT = sizeof(MENU_TILES) / sizeof(MENU_TILES[0]);

// Tile grid: 2 columns, rows as needed.
#define MENU_COLS     2
#define MENU_TILE_W   (SCREEN_W / MENU_COLS)
#define MENU_TILE_H   52
#define MENU_TILE_PAD 3

// Returns the tile index hit by (tx, ty) in body coordinates, or -1.
int menuHitTest(int tx, int ty) {
  int bodyY = ty - TITLE_H;
  if (bodyY < 0) return -1;
  int col = tx / MENU_TILE_W;
  int row = bodyY / MENU_TILE_H;
  int idx = row * MENU_COLS + col;
  if (idx < 0 || idx >= MENU_TILE_COUNT) return -1;
  if (col >= MENU_COLS) return -1;
  return idx;
}

void drawMenuPage() {
  tft.fillRect(0, TITLE_H, SCREEN_W, SCREEN_H - TITLE_H, 0x1082); // very dark grey

  for (int i = 0; i < MENU_TILE_COUNT; i++) {
    int col = i % MENU_COLS;
    int row = i / MENU_COLS;
    int x = col * MENU_TILE_W + MENU_TILE_PAD;
    int y = TITLE_H + row * MENU_TILE_H + MENU_TILE_PAD;
    int w = MENU_TILE_W - MENU_TILE_PAD * 2;
    int h = MENU_TILE_H - MENU_TILE_PAD * 2;

    tft.fillRoundRect(x, y, w, h, 6, MENU_TILES[i].color);
    tft.setTextColor(TFT_WHITE, MENU_TILES[i].color);
    tft.setTextSize(2);
    tft.setCursor(x + 6, y + 6);
    tft.print(MENU_TILES[i].label);
    tft.setTextSize(1);
    tft.setTextColor(TFT_LIGHTGREY, MENU_TILES[i].color);
    tft.setCursor(x + 6, y + 26);
    tft.print(MENU_TILES[i].sublabel);
  }
}

// ================== WIFI CHANNEL SCAN ==================
void drawWifiScanPage() {
  tft.fillRect(0, TITLE_H, SCREEN_W, SCREEN_H - TITLE_H, TFT_BLACK);
  tft.setTextSize(1);

  uint16_t buttonBg = wifiScanRunning ? TFT_DARKGREY : 0x8410;
  tft.fillRoundRect(8, TITLE_H + 6, SCREEN_W - 28, 20, 4, buttonBg);
  tft.setTextColor(TFT_WHITE, buttonBg);
  tft.setCursor(14, TITLE_H + 12);
  tft.print(wifiScanRunning ? "Scanning channels..." : "Tap to scan WiFi channels");

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(10, TITLE_H + 32);
  if (wifiScanRunning) {
    tft.print("Scanning...");
  } else if (wifiScanDone) {
    tft.printf("Found: %d network(s)", wifiNetworkCount);
  } else {
    tft.print("Ready");
  }

  // Column headers
  int hy = TITLE_H + 43;
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(4,  hy); tft.print("Ch");
  tft.setCursor(26, hy); tft.print("RSSI");
  tft.setCursor(58, hy); tft.print("SSID");
  tft.drawFastHLine(4, hy + 9, SCREEN_W - 16, TFT_DARKGREY);

  int y = TITLE_H + 54;
  int rowH = LINE_H + 2;
  int maxRows = (SCREEN_H - y) / rowH;
  int endIndex = min(startIndex + maxRows, wifiNetworkCount);

  for (int i = startIndex; i < endIndex; i++) {
    // Channel colour: green=1-6, yellow=7-11, cyan=12-14, red=5GHz (ch>=36)
    uint16_t chColor;
    int ch = wifiNetworks[i].channel;
    if      (ch >= 36)  chColor = TFT_MAGENTA;
    else if (ch <= 6)   chColor = TFT_GREEN;
    else if (ch <= 11)  chColor = TFT_YELLOW;
    else                chColor = TFT_CYAN;

    tft.setTextColor(chColor, TFT_BLACK);
    tft.setCursor(4, y);
    tft.printf("%2d", ch);

    // RSSI bar
    int rssi = wifiNetworks[i].rssi;
    int barW = map(constrain(rssi, -90, -30), -90, -30, 0, 28);
    tft.fillRect(26, y, 28, LINE_H - 1, TFT_DARKGREY);
    tft.fillRect(26, y, barW, LINE_H - 1, chColor);

    // SSID
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(58, y);
    char ssidBuf[22]; strncpy(ssidBuf, wifiNetworks[i].ssid, 21); ssidBuf[21] = '\0';
    tft.print(ssidBuf);

    // Auth icon (rightmost)
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(SCREEN_W - 20, y);
    tft.print(wifiNetworks[i].authmode == 0 ? "  " : "\x06"); // open vs locked glyph

    y += rowH;
  }

  drawScrollArrows();
}

void drawNetInfoPage() {
  tft.fillRect(0, TITLE_H, SCREEN_W, SCREEN_H - TITLE_H, TFT_BLACK);
  tft.setTextSize(1);

  const int LX = 6;
  const int VX = 72;
  const int LH = 14;
  int y = TITLE_H + 6;

  // Status row
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(LX, y);
  tft.print("Status:");
  tft.setTextColor(ethConnected ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.setCursor(VX, y);
  tft.print(ethConnected ? "Connected" : "Disconnected");
  if (!ethConnected) return;
  y += LH;

  auto row = [&](const char *label, const String &val) {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(LX, y);
    tft.print(label);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(VX, y);
    tft.print(val);
    y += LH;
  };

  row("IP:",        ETH.localIP().toString());
  row("Subnet:",    ETH.subnetMask().toString());
  row("Gateway:",   ETH.gatewayIP().toString());
  row("DNS:",       ETH.dnsIP().toString());
  row("Broadcast:", ETH.broadcastIP().toString());
  row("MAC:",       ETH.macAddress());

  // Link speed
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(LX, y);
  tft.print("Link:");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(VX, y);
  tft.printf("%d Mbps %s", ETH.linkSpeed(), ETH.fullDuplex() ? "Full" : "Half");
  y += LH;

  // Hostname
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(LX, y);
  tft.print("Host:");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(VX, y);
  tft.print(ETH.getHostname());
}

// ================== SETTINGS PAGE ==================
// Layout constants (absolute Y positions on 240×320 screen):
//   NETWORK section:
//     label y = TITLE_H + 6        DHCP/Static toggle y = TITLE_H + 18, h=24
//     Static fields y = TITLE_H + 46, step = 15  (4 fields → bottom = TITLE_H+106)
//     Apply button  y = TITLE_H + 112, h=24
//   DISPLAY section:
//     label y = TITLE_H + 144      Normal/Flipped toggle y = TITLE_H + 156, h=24
//   IP editor overlay top y = TITLE_H + 142:
//     [+] row  @ TITLE_H+164  [-] row  @ TITLE_H+208
//     values   @ TITLE_H+190  Cancel/Save @ TITLE_H+236

static const char *SET_FIELD_LABELS[4] = { "IP:", "Subnet:", "Gateway:", "DNS:" };

void drawSettingsPageOctEditor() {
  // Draw the IP octet editor panel overlaying the lower portion of the page.
  const int TOP  = TITLE_H + 142;
  const int COL  = 60;  // pixels per column

  tft.fillRect(0, TOP, SCREEN_W, SCREEN_H - TOP, 0x1082);
  tft.drawFastHLine(0, TOP, SCREEN_W, TFT_DARKGREY);

  // Field label
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY, 0x1082);
  tft.setCursor(MARGIN, TOP + 6);
  tft.print("Edit ");
  tft.print(SET_FIELD_LABELS[settingsEditField]);

  // [+] buttons row
  const int PLUS_Y  = TOP + 22;
  const int VAL_Y   = TOP + 48;
  const int MINUS_Y = TOP + 66;
  const int ACT_Y   = TOP + 94;
  const int BTN_H   = 20;

  for (int c = 0; c < 4; c++) {
    int bx = c * COL + 2;
    tft.fillRoundRect(bx, PLUS_Y,  56, BTN_H, 3, 0x0320);
    tft.fillRoundRect(bx, MINUS_Y, 56, BTN_H, 3, 0x8400);

    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, 0x0320);
    tft.setCursor(bx + 18, PLUS_Y + 2);
    tft.print("+");

    tft.setTextColor(TFT_WHITE, 0x8400);
    tft.setCursor(bx + 18, MINUS_Y + 2);
    tft.print("-");

    // Octet value centred in column
    char numBuf[4];
    snprintf(numBuf, sizeof(numBuf), "%d", settingsEditOcts[c]);
    int nlen = strlen(numBuf);
    tft.setTextColor(TFT_WHITE, 0x1082);
    tft.setCursor(c * COL + (60 - nlen * 12) / 2, VAL_Y);
    tft.print(numBuf);
  }

  // Cancel / Save buttons
  tft.fillRoundRect(MARGIN,        ACT_Y, 110, 24, 4, TFT_DARKGREY);
  tft.fillRoundRect(SCREEN_W - 116, ACT_Y, 110, 24, 4, TFT_DARKGREEN);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setCursor(28, ACT_Y + 8);
  tft.print("Cancel");
  tft.setTextColor(TFT_WHITE, TFT_DARKGREEN);
  tft.setCursor(SCREEN_W - 82, ACT_Y + 8);
  tft.print("Save octet");
}

void drawSettingsPage() {
  tft.fillRect(0, TITLE_H, SCREEN_W, SCREEN_H - TITLE_H, TFT_BLACK);
  tft.setTextSize(1);

  // ---- NETWORK section ----
  int y = TITLE_H + 6;
  tft.setTextColor(0x8410, TFT_BLACK);
  tft.setCursor(MARGIN, y);
  tft.print("NETWORK");
  y = TITLE_H + 18;

  // DHCP / Static toggle
  bool isDhcp = !appSettings.staticIP;
  uint16_t dhcpBg = isDhcp  ? TFT_DARKGREEN : 0x2104;
  uint16_t statBg = !isDhcp ? 0x8400        : 0x2104;
  tft.fillRoundRect(4,   y, 112, 24, 4, dhcpBg);
  tft.fillRoundRect(124, y, 112, 24, 4, statBg);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, dhcpBg);
  tft.setCursor(22,  y + 4); tft.print("DHCP");
  tft.setTextColor(TFT_WHITE, statBg);
  tft.setCursor(136, y + 4); tft.print("Static");
  y = TITLE_H + 46;

  // Static IP fields (always drawn, dimmed when DHCP)
  uint8_t *flds[4] = { appSettings.ip, appSettings.mask, appSettings.gw, appSettings.dns };
  for (int i = 0; i < 4; i++) {
    uint16_t rowBg = appSettings.staticIP ? 0x2104 : 0x1082;
    tft.fillRect(0, y, SCREEN_W, 14, rowBg);
    tft.setTextColor(TFT_DARKGREY, rowBg);
    tft.setCursor(MARGIN, y + 2);
    tft.print(SET_FIELD_LABELS[i]);
    tft.setTextColor(appSettings.staticIP ? TFT_WHITE : 0x4208, rowBg);
    tft.setCursor(56, y + 2);
    tft.printf("%d.%d.%d.%d", flds[i][0], flds[i][1], flds[i][2], flds[i][3]);
    if (appSettings.staticIP) {
      tft.setTextColor(TFT_DARKGREY, rowBg);
      tft.setCursor(SCREEN_W - 14, y + 2);
      tft.print(">");
    }
    y += 15;
  }
  y = TITLE_H + 112;

  // Apply & Restart
  tft.fillRoundRect(MARGIN, y, SCREEN_W - MARGIN * 2, 24, 4, 0x8400);
  tft.setTextColor(TFT_WHITE, 0x8400);
  tft.setTextSize(2);
  tft.setCursor(18, y + 4);
  tft.print("Apply & Restart");

  // ---- DISPLAY section ----
  y = TITLE_H + 144;
  tft.setTextSize(1);
  tft.setTextColor(0x8410, TFT_BLACK);
  tft.setCursor(MARGIN, y);
  tft.print("DISPLAY");
  y = TITLE_H + 156;

  bool isNorm = !appSettings.flipScreen;
  uint16_t normBg = isNorm  ? 0x2945 : 0x2104;
  uint16_t flipBg = !isNorm ? 0x8008 : 0x2104;
  tft.fillRoundRect(4,   y, 112, 24, 4, normBg);
  tft.fillRoundRect(124, y, 112, 24, 4, flipBg);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, normBg);
  tft.setCursor(10,  y + 4); tft.print("Normal");
  tft.setTextColor(TFT_WHITE, flipBg);
  tft.setCursor(128, y + 4); tft.print("Flipped");

  // If editor is open, overlay the octet editor panel
  if (settingsEditField >= 0) drawSettingsPageOctEditor();
}

void drawList() {
  if (isIpScannerPage()) {
    drawIpScannerPage();
    return;
  }
  if (isArtNetPage()) {
    drawArtNetPage();
    return;
  }
  if (isDantePage()) {
    drawDantePage();
    return;
  }
  if (isHQNetPage()) {
    drawHQNetPage();
    return;
  }
  if (isWifiScanPage()) {
    drawWifiScanPage();
    return;
  }
  if (isNetInfoPage()) {
    drawNetInfoPage();
    return;
  }
  if (isSettingsPage()) {
    drawSettingsPage();
    return;
  }
  if (isMenuPage()) {
    drawMenuPage();
    return;
  }

  tft.fillRect(0, TITLE_H, SCREEN_W, SCREEN_H - TITLE_H, TFT_BLACK);
  tft.setTextSize(1);

  NDISource *list = isAllDevicesPage() ? allHostsList : ndiList;
  int total       = isAllDevicesPage() ? allHostsCount : ndiCount;
  int y = TITLE_H + MARGIN;
  int endIndex = min(startIndex + VISIBLE_ROWS, total);

  for (int i = startIndex; i < endIndex; i++) {
    bool selected = (i == selectedIndex);
    uint16_t bg = selected ? TFT_DARKCYAN : TFT_BLACK;
    uint16_t fg = selected ? TFT_WHITE : TFT_LIGHTGREY;

    tft.fillRect(0, y - 2, SCREEN_W - 12, LINE_H * 3, bg);
    tft.setTextColor(fg, bg);

    tft.fillCircle(6, y + 4, 4, dotColor(i));

    tft.setCursor(14, y);
    tft.print(list[i].name);

    y += LINE_H;
    tft.setTextColor(TFT_DARKGREY, bg);
    tft.setCursor(14, y);
    tft.print(isAllDevicesPage() ? list[i].stream
                             : (list[i].isReceiver ? "NDI Receiver" : list[i].stream));

    y += LINE_H;
    tft.setTextColor(fg, bg);
    tft.setCursor(14, y);
    tft.print(list[i].ip);

    y += LINE_H;
  }

  drawScrollArrows();
}

void drawRow(int listIndex, int screenRow) {
  if (isIpScannerPage()) return;

  int y = rowY(screenRow);

  bool selected = (listIndex == selectedIndex);
  uint16_t bg = selected ? TFT_DARKCYAN : TFT_BLACK;
  uint16_t fg = selected ? TFT_WHITE : TFT_LIGHTGREY;

  const NDISource &src = isAllDevicesPage() ? allHostsList[listIndex] : ndiList[listIndex];

  tft.fillRect(0, y - 2, SCREEN_W, LINE_H * 3, bg);
  tft.setTextColor(fg, bg);
  tft.setTextSize(1);

  tft.fillCircle(6, y + 4, 4, dotColor(listIndex));

  tft.setCursor(14, y);
  tft.print(src.name);

  tft.setTextColor(TFT_DARKGREY, bg);
  tft.setCursor(14, y + LINE_H);
  tft.print(isAllDevicesPage() ? src.stream
                           : (src.isReceiver ? "NDI Receiver" : src.stream));

  tft.setTextColor(fg, bg);
  tft.setCursor(14, y + LINE_H * 2);
  tft.print(src.ip);
}

// ================== NDI DISCOVERY ==================
void scanNDI(NDISource *list, int &count, NDISource *allList, int &allCount) {
  // Keep any receivers already in the list, remove sources, then re-add sources.
  int keepCount = 0;
  for (int i = 0; i < count; i++) {
    if (list[i].isReceiver) {
      if (keepCount != i) list[keepCount] = list[i];
      keepCount++;
    }
  }
  count = keepCount;

  mdns_result_t *results = NULL;
  esp_err_t err = mdns_query_ptr("_ndi", "_tcp", 3000, MAX_NDI, &results);
  if (err != ESP_OK || !results) return;

  mdns_result_t *r = results;
  while (r && count < MAX_NDI) {
    String hostname = r->hostname ? String(r->hostname) : "";
    // Add to the all-devices list.
    if (r->hostname && allCount < MAX_NDI) {
      bool dupAll = false;
      for (int i = 0; i < allCount; i++) {
        if (allList[i].name.equalsIgnoreCase(hostname)) { dupAll = true; break; }
      }
      if (!dupAll) {
        allList[allCount].name       = hostname;
        allList[allCount].stream     = "_ndi";
        allList[allCount].isReceiver = false;
        allList[allCount].ip         = "";
        mdns_ip_addr_t *a = r->addr;
        while (a) {
          if (a->addr.type == MDNS_IP_PROTOCOL_V4) {
            allList[allCount].ip = IPAddress(a->addr.u_addr.ip4.addr).toString();
            break;
          }
          a = a->next;
        }
        allList[allCount].reachable = allList[allCount].ip.length() > 0;
        allCount++;
      }
    }
    // Skip if this host is already listed as a receiver.
    bool duplicate = false;
    for (int i = 0; i < count; i++) {
      if (list[i].isReceiver && list[i].name.equalsIgnoreCase(hostname)) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      list[count].name = hostname;
      list[count].isReceiver = false;
      // instance_name is "MACHINE (Stream Name)" - extract just the stream part
      if (r->instance_name) {
        String inst = String(r->instance_name);
        int open  = inst.indexOf('(');
        int close = inst.lastIndexOf(')');
        list[count].stream = (open >= 0 && close > open)
          ? inst.substring(open + 1, close)
          : inst;
      } else {
        list[count].stream = hostname;
      }
      list[count].ip = "";
      mdns_ip_addr_t *addr = r->addr;
      while (addr) {
        if (addr->addr.type == MDNS_IP_PROTOCOL_V4) {
          list[count].ip = IPAddress(addr->addr.u_addr.ip4.addr).toString();
          break;
        }
        addr = addr->next;
      }
      list[count].reachable = false;
      count++;
    }
    r = r->next;
  }
  mdns_query_results_free(results);
}

// Service types to probe when hunting for "ndi*" receiver hostnames.
// _workstation._tcp  – Windows Network Discovery
// _http._tcp         – most embedded hardware (decoders, gateways, appliances)
// _https._tcp        – same, on HTTPS
// NOTE: _ndi._tcp is intentionally excluded — it is queried by scanNDI() for
//       actual NDI sources and must not be harvested here as receivers.
static const char *RECEIVER_SERVICE_TYPES[] = {
  "_workstation",   // Linux/macOS computers (avahi/bonjour)
  "_http",          // generic HTTP services
  "_https",         // generic HTTPS services
  "_smb",           // Windows/macOS file sharing
  "_services._dns-sd._udp.local",    // dns
  "_airplay",       // Apple TV, AirPlay speakers
  "_raop",          // AirPlay audio (iPhones, HomePod)
  "_companion-link",// Apple Watch, Continuity
  "_spotify-connect", // Spotify-enabled speakers
};
static const int NUM_RECEIVER_SERVICES = 9;

// Add any hostname/IP to the all-devices list (no hostname filter).
static void collectAnyHost(NDISource *allList, int &allCount,
                            const char *hostnameC, mdns_ip_addr_t *addrList,
                            const char *svcType) {
  if (!hostnameC || allCount >= MAX_NDI) return;
  String hostname = String(hostnameC);
  for (int i = 0; i < allCount; i++) {
    if (allList[i].name.equalsIgnoreCase(hostname)) return;
  }
  allList[allCount].name       = hostname;
  allList[allCount].stream     = String(svcType);
  allList[allCount].isReceiver = false;
  allList[allCount].ip         = "";
  mdns_ip_addr_t *addr = addrList;
  while (addr) {
    if (addr->addr.type == MDNS_IP_PROTOCOL_V4) {
      allList[allCount].ip = IPAddress(addr->addr.u_addr.ip4.addr).toString();
      break;
    }
    addr = addr->next;
  }
  allList[allCount].reachable = allList[allCount].ip.length() > 0;
  allCount++;
}

// Add to receiver list if hostname starts with "ndi"; always add to all-devices list.
static void collectReceiverHost(NDISource *list, int &count,
                                 NDISource *allList, int &allCount,
                                 const char *hostnameC,
                                 mdns_ip_addr_t *addrList,
                                 const char *svcType) {
  if (!hostnameC) return;
  collectAnyHost(allList, allCount, hostnameC, addrList, svcType);
  if (count >= MAX_NDI) return;

  String hostname = String(hostnameC);
  String lc = hostname;
  lc.toLowerCase();
  if (!lc.startsWith("ndi")) return;

  for (int i = 0; i < count; i++) {
    if (list[i].name.equalsIgnoreCase(hostname)) return;  // already present
  }

  list[count].name       = hostname;
  list[count].stream     = "";
  list[count].isReceiver = true;
  list[count].ip         = "";
  mdns_ip_addr_t *addr = addrList;
  while (addr) {
    if (addr->addr.type == MDNS_IP_PROTOCOL_V4) {
      list[count].ip = IPAddress(addr->addr.u_addr.ip4.addr).toString();
      break;
    }
    addr = addr->next;
  }
  list[count].reachable = list[count].ip.length() > 0;
  Serial.printf("[NDI RX] found: %s  %s\n", hostname.c_str(), list[count].ip.c_str());
  count++;
}

void scanReceivers(NDISource *list, int &count, NDISource *allList, int &allCount) {
  // Remove stale receiver entries, keep sources.
  int keepCount = 0;
  for (int i = 0; i < count; i++) {
    if (!list[i].isReceiver) {
      if (keepCount != i) list[keepCount] = list[i];
      keepCount++;
    }
  }
  count = keepCount;

  // Query each service type; all results go to allList, ndi* ones also to list.
  for (int s = 0; s < NUM_RECEIVER_SERVICES && count < MAX_NDI; s++) {
    mdns_result_t *results = NULL;
    esp_err_t err = mdns_query_ptr(RECEIVER_SERVICE_TYPES[s], "_tcp",
                                   800, MAX_NDI, &results);
    if (err != ESP_OK || !results) continue;

    mdns_result_t *r = results;
    while (r) {
      collectReceiverHost(list, count, allList, allCount,
                          r->hostname, r->addr, RECEIVER_SERVICE_TYPES[s]);
      r = r->next;
    }
    mdns_query_results_free(results);
  }
}

void checkReachable(NDISource *list, int count) {
  for (int i = 0; i < count; i++) {
    list[i].reachable = list[i].ip.length() > 0;
  }
}

int visibleRowsForCurrentPage() {
  if (!isProtocolScanPage() && !isIpScannerPage()) return VISIBLE_ROWS;
  int y = TITLE_H + 46;
  int rows = (SCREEN_H - y) / (LINE_H * 3);
  return max(1, rows);
}

bool probeIpHost(const IPAddress &ip, int &openPort) {
  static const uint16_t ports[] = {80, 443, 554, 445, 22};
  WiFiClient client;
  for (uint16_t port : ports) {
    if (client.connect(ip, port, 70)) {
      openPort = port;
      client.stop();
      return true;
    }
    client.stop();
    delay(1);
  }
  return false;
}

String lookupKnownHostname(const String &ip) {
  for (int i = 0; i < ndiCount; i++) {
    if (ndiList[i].ip == ip && ndiList[i].name.length() > 0) {
      return ndiList[i].name;
    }
  }

  for (int i = 0; i < allHostsCount; i++) {
    if (allHostsList[i].ip == ip && allHostsList[i].name.length() > 0) {
      return allHostsList[i].name;
    }
  }

  return "";
}

bool lookupKnownHostnameWithSource(const String &ip, String &hostname, String &source) {
  hostname = lookupKnownHostname(ip);
  if (hostname.length() == 0) {
    source = "";
    return false;
  }

  source = "mDNS";
  return true;
}

uint16_t readU16BE(const uint8_t *buf, int offset) {
  return ((uint16_t)buf[offset] << 8) | (uint16_t)buf[offset + 1];
}

uint32_t readU32BE(const uint8_t *buf, int offset) {
  return ((uint32_t)buf[offset] << 24) |
         ((uint32_t)buf[offset + 1] << 16) |
         ((uint32_t)buf[offset + 2] << 8) |
         (uint32_t)buf[offset + 3];
}

int skipDnsName(const uint8_t *buf, int len, int offset) {
  if (offset >= len) return -1;

  while (offset < len) {
    uint8_t n = buf[offset++];
    if (n == 0) return offset;

    // Name compression pointer.
    if ((n & 0xC0) == 0xC0) {
      if (offset >= len) return -1;
      offset++;
      return offset;
    }

    if (offset + n > len) return -1;
    offset += n;
  }

  return -1;
}

String trimNetbiosName(const uint8_t *nameBytes, int len) {
  String out = "";
  for (int i = 0; i < len; i++) {
    char c = (char)nameBytes[i];
    if (c >= 32 && c <= 126) out += c;
  }
  out.trim();
  return out;
}

String parseNetbiosStatusName(const uint8_t *buf, int len) {
  // Robust NBNS node-status response parser.
  if (len < 12) return "";

  uint16_t flags = readU16BE(buf, 2);
  uint16_t qdCount = readU16BE(buf, 4);
  uint16_t anCount = readU16BE(buf, 6);
  uint8_t rcode = (uint8_t)(flags & 0x0F);
  if (rcode != 0 || anCount == 0) return "";

  int off = 12;

  for (int i = 0; i < qdCount; i++) {
    off = skipDnsName(buf, len, off);
    if (off < 0 || off + 4 > len) return "";
    off += 4;  // qtype + qclass
  }

  int rdataOffset = -1;
  int rdlen = 0;
  for (int i = 0; i < anCount; i++) {
    off = skipDnsName(buf, len, off);
    if (off < 0 || off + 10 > len) return "";

    uint16_t type = readU16BE(buf, off);
    uint16_t cls = readU16BE(buf, off + 2);
    (void)readU32BE(buf, off + 4);  // ttl
    rdlen = (int)readU16BE(buf, off + 8);
    off += 10;
    if (off + rdlen > len) return "";

    // NBSTAT answer in Internet class.
    if (type == 0x0021 && cls == 0x0001) {
      rdataOffset = off;
      break;
    }

    off += rdlen;
  }

  if (rdataOffset < 0 || rdlen <= 0) return "";

  int nameCount = buf[rdataOffset];
  int namesOffset = rdataOffset + 1;
  if (nameCount <= 0) return "";
  if (namesOffset + (nameCount * 18) > len) return "";

  String fallback = "";
  for (int i = 0; i < nameCount; i++) {
    const int entry = namesOffset + (i * 18);
    String name = trimNetbiosName(&buf[entry], 15);
    if (name.length() == 0) continue;

    uint8_t suffix = buf[entry + 15];
    uint16_t flags = readU16BE(buf, entry + 16);
    bool isGroup = (flags & 0x8000) != 0;

    if (!isGroup && suffix == 0x00) return name;
    if (!isGroup && fallback.length() == 0) fallback = name;
  }

  return fallback;
}

String queryNetbiosHostname(const IPAddress &ip) {
  // Node Status Request for wildcard name '*'.
  uint8_t query[50] = {0};
  uint16_t txid = (uint16_t)(millis() & 0xFFFF);
  query[0] = (uint8_t)(txid >> 8);
  query[1] = (uint8_t)(txid & 0xFF);
  query[4] = 0x00; query[5] = 0x01;  // QDCOUNT=1

  int o = 12;
  query[o++] = 0x20;  // encoded NetBIOS name length

  uint8_t nbName[16];
  nbName[0] = '*';
  for (int i = 1; i < 15; i++) nbName[i] = ' ';
  nbName[15] = 0x00;

  for (int i = 0; i < 16; i++) {
    uint8_t b = nbName[i];
    query[o++] = 'A' + ((b >> 4) & 0x0F);
    query[o++] = 'A' + (b & 0x0F);
  }

  query[o++] = 0x00;
  query[o++] = 0x00; query[o++] = 0x21;  // QTYPE=NBSTAT
  query[o++] = 0x00; query[o++] = 0x01;  // QCLASS=IN

  WiFiUDP udp;
  if (!udp.begin(0)) return "";

  if (!udp.beginPacket(ip, 137)) {
    udp.stop();
    return "";
  }
  udp.write(query, o);
  if (!udp.endPacket()) {
    udp.stop();
    return "";
  }

  unsigned long start = millis();
  while (millis() - start < 450) {
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
      uint8_t resp[300];
      int n = udp.read(resp, min(packetSize, (int)sizeof(resp)));
      udp.stop();
      if (n <= 0) return "";
      if (n < 2) return "";

      uint16_t rxid = readU16BE(resp, 0);
      if (rxid != txid) return "";

      return parseNetbiosStatusName(resp, n);
    }
    delay(2);
  }

  udp.stop();
  return "";
}

String reverseLookupHostname(const IPAddress &ip) {
  String ipStr = ip.toString();
  String known = lookupKnownHostname(ipStr);
  if (known.length() > 0) return known;

  String netbios = queryNetbiosHostname(ip);
  if (netbios.length() > 0) return netbios;

  return "";
}

bool probeHostWithFallback(const IPAddress &ip, int &openPort, String &hostname, String &source) {
  openPort = -1;
  hostname = "";
  source = "";

  String ipStr = ip.toString();

  if (probeIpHost(ip, openPort)) {
    if (lookupKnownHostnameWithSource(ipStr, hostname, source)) {
      return true;
    }

    hostname = queryNetbiosHostname(ip);
    if (hostname.length() > 0) {
      source = "NBNS";
    }
    return true;
  }

  // If TCP probe fails, try NetBIOS node-status as discovery fallback.
  hostname = queryNetbiosHostname(ip);
  if (hostname.length() > 0) {
    source = "NBNS";
    openPort = 137;
    return true;
  }

  return false;
}

void startIpScan() {
  scanHostCount = 0;
  startIndex = 0;

  if (!ethConnected) {
    ipScanRunning = false;
    ipScanFinished = true;
    ipScanDirty = true;
    return;
  }

  IPAddress ip = ETH.localIP();
  IPAddress mask = ETH.subnetMask();
  uint32_t ipU32 = ipToU32(ip);
  uint32_t maskU32 = ipToU32(mask);
  uint32_t network = ipU32 & maskU32;
  uint32_t broadcast = network | (~maskU32);

  if (broadcast <= network + 1) {
    ipScanRunning = false;
    ipScanFinished = true;
    ipScanDirty = true;
    return;
  }

  ipScanStartHost = network + 1;
  ipScanEndHost = broadcast - 1;
  ipScanOwnIp = ipU32;
  ipScanCurrentHost = ipScanStartHost;
  ipScanNetworkHostCount = ipScanEndHost - ipScanStartHost + 1;

  // Keep runtime predictable on large non-/24 subnets.
  const uint32_t maxSweepHosts = 1022;
  if (ipScanNetworkHostCount > maxSweepHosts) {
    ipScanEndHost = ipScanStartHost + maxSweepHosts - 1;
    ipScanNetworkHostCount = maxSweepHosts;
  }

  ipScanRunning = true;
  ipScanFinished = false;
  ipScanDirty = true;
}

void processIpScan() {
  if (!ipScanRunning) return;

  // Probe one host per loop iteration to keep UI responsive.
  if (ipScanCurrentHost <= ipScanEndHost) {
    uint32_t host = ipScanCurrentHost++;
    if (host != ipScanOwnIp) {
      int openPort = -1;
      String hostname = "";
      String source = "";
      IPAddress ip = u32ToIp(host);
      if (probeHostWithFallback(ip, openPort, hostname, source) && scanHostCount < MAX_SCAN_HOSTS) {
        scanHosts[scanHostCount].hostname = hostname;
        scanHosts[scanHostCount].source = source;
        scanHosts[scanHostCount].ip = ip.toString();
        scanHosts[scanHostCount].openPort = openPort;
        scanHostCount++;
      }
    }
    ipScanDirty = true;
    return;
  }

  ipScanRunning = false;
  ipScanFinished = true;
  ipScanDirty = true;
}

// ================== ARTNET SCAN ==================
WiFiUDP artnetUDP;

void startArtNetScan() {
  if (!ethConnected) return;
  artnetHostCount   = 0;
  artnetScanRunning = true;
  artnetScanDone    = false;
  artnetScanDirty   = true;
  artnetUDP.begin(6454);

  // Build ArtPoll packet (14 bytes)
  uint8_t poll[14] = {0};
  memcpy(poll, "Art-Net", 8);      // ID (includes null terminator)
  poll[8]  = 0x00;                  // OpCode low byte (ArtPoll = 0x2000)
  poll[9]  = 0x20;                  // OpCode high byte
  poll[10] = 0x00;                  // ProtVerHi
  poll[11] = 14;                    // ProtVerLo
  poll[12] = 0x00;                  // TalkToMe = 0: reply to me now (not "on change only")
  poll[13] = 0x00;                  // DiagPriority (DP_LOW)

  // Send to directed subnet broadcast AND limited broadcast (255.255.255.255).
  // Some receiver nodes ignore directed broadcasts or are on differing subnets.
  IPAddress broadcast = ETH.broadcastIP();
  artnetUDP.beginPacket(broadcast, 6454);
  artnetUDP.write(poll, sizeof(poll));
  artnetUDP.endPacket();

  artnetUDP.beginPacket(IPAddress(255, 255, 255, 255), 6454);
  artnetUDP.write(poll, sizeof(poll));
  artnetUDP.endPacket();

  Serial.printf("ArtPoll sent to %s and 255.255.255.255\n", broadcast.toString().c_str());
}

// Collect ArtPollReply packets for up to 3 seconds then stop.
static unsigned long artnetScanStart = 0;

void processArtNetScan() {
  if (!artnetScanRunning) return;

  if (artnetScanStart == 0) artnetScanStart = millis();

  int len;
  while ((len = artnetUDP.parsePacket()) > 0) {
    if (len < 176) { artnetUDP.flush(); continue; }

    uint8_t buf[256];
    int read = artnetUDP.read(buf, sizeof(buf));
    artnetUDP.flush();

    if (read < 10) continue;
    // Verify Art-Net header and opcode 0x2100 (ArtPollReply)
    if (memcmp(buf, "Art-Net", 8) != 0) continue;
    uint16_t opcode = buf[8] | ((uint16_t)buf[9] << 8);
    if (opcode != 0x2100) continue;

    // IP address at bytes 10-13
    IPAddress replyIP(buf[10], buf[11], buf[12], buf[13]);
    String ipStr = replyIP.toString();

    // Duplicate check
    bool dup = false;
    for (int i = 0; i < artnetHostCount; i++) {
      if (artnetHosts[i].ip == ipStr) { dup = true; break; }
    }
    if (dup || artnetHostCount >= MAX_ARTNET_HOSTS) continue;

    ArtNetHost &h = artnetHosts[artnetHostCount++];
    h.ip = ipStr;

    // shortName: bytes 26-43 (18 bytes, null-terminated)
    char sn[18]; memcpy(sn, buf + 26, 17); sn[17] = '\0';
    h.shortName = String(sn);

    // longName: bytes 44-107 (64 bytes, null-terminated)
    char ln[64]; memcpy(ln, buf + 44, 63); ln[63] = '\0';
    h.longName = String(ln);

    // numPorts: bytes 174-175 (little-endian)
    h.numPorts = (read >= 176) ? (buf[174] | ((uint16_t)buf[175] << 8)) : 0;

    // OEM: bytes 10-11 in the port area (actually at 10-11 of the reply starting spec byte)
    // OEM code is at bytes 10-11 of the ArtPollReply (after the 8-byte header+opcode)
    // Per the Art-Net spec, IpAddress=10, BindIp=16, OEM=22-23
    if (read >= 24) { h.oem[0] = buf[22]; h.oem[1] = buf[23]; }

    artnetScanDirty = true;
    Serial.printf("ArtNet host: %s sn=%s\n", ipStr.c_str(), sn);
  }

  if (millis() - artnetScanStart > 3000) {
    artnetScanRunning = false;
    artnetScanDone    = true;
    artnetScanDirty   = true;
    artnetScanStart   = 0;
    artnetUDP.stop();
    Serial.printf("ArtNet scan done: %d host(s)\n", artnetHostCount);
  }
}

// ================== DANTE SCAN ==================
// Dante ConMon uses UDP multicast to 239.255.0.1:4440 for device announcements.
// WiFiUDP::beginMulticast doesn't reliably join IGMP on the Ethernet interface,
// so we use POSIX sockets with explicit IP_ADD_MEMBERSHIP bound to the ETH IP.
#define DANTE_CONMON_PORT 4440

static int danteSock = -1;
static unsigned long danteScanStart = 0;
static int danteRawPkts = 0;

static String extractDanteString(const uint8_t *buf, int offset, int len, int maxLen = 32) {
  String s;
  for (int i = offset; i < len && (int)s.length() < maxLen; i++) {
    char c = (char)buf[i];
    if (c == '\0') { if (s.length() > 0) break; else continue; }
    if (c >= 0x20 && c < 0x7F) s += c;
    else if (s.length() > 0) break;
  }
  return s;
}

void startDanteScan() {
  if (!ethConnected) return;
  danteHostCount   = 0;
  danteRawPkts     = 0;
  danteScanRunning = true;
  danteScanDone    = false;
  danteScanDirty   = true;
  danteScanStart   = 0;

  if (danteSock >= 0) { close(danteSock); danteSock = -1; }

  danteSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (danteSock < 0) {
    Serial.printf("Dante: socket() failed errno=%d\n", errno);
    danteScanRunning = false;
    return;
  }

  int reuse = 1;
  setsockopt(danteSock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  struct sockaddr_in local = {};
  local.sin_family      = AF_INET;
  local.sin_port        = htons(DANTE_CONMON_PORT);
  local.sin_addr.s_addr = INADDR_ANY;
  if (bind(danteSock, (struct sockaddr*)&local, sizeof(local)) < 0) {
    Serial.printf("Dante: bind() failed errno=%d\n", errno);
    close(danteSock); danteSock = -1;
    danteScanRunning = false;
    return;
  }

  // Join multicast group on the Ethernet interface specifically.
  struct ip_mreq mreq = {};
  mreq.imr_multiaddr.s_addr = inet_addr("239.255.0.1");
  mreq.imr_interface.s_addr = inet_addr(ETH.localIP().toString().c_str());
  if (setsockopt(danteSock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
    Serial.printf("Dante: IP_ADD_MEMBERSHIP failed errno=%d (continuing)\n", errno);
  } else {
    Serial.printf("Dante: joined multicast 239.255.0.1 on %s\n",
                  ETH.localIP().toString().c_str());
  }

  // Non-blocking so processDanteScan() can poll in the main loop.
  int flags = fcntl(danteSock, F_GETFL, 0);
  fcntl(danteSock, F_SETFL, flags | O_NONBLOCK);

  // Force multicast TX through the Ethernet interface (not WiFi).
  struct in_addr mcIf;
  mcIf.s_addr = inet_addr(ETH.localIP().toString().c_str());
  if (setsockopt(danteSock, IPPROTO_IP, IP_MULTICAST_IF, &mcIf, sizeof(mcIf)) < 0)
    Serial.printf("Dante: IP_MULTICAST_IF failed errno=%d\n", errno);

  // Send ConMon probe to multicast + subnet broadcast + limited broadcast.
  uint8_t probe[] = { 0x27, 0x01, 0x00, 0x00 };
  struct sockaddr_in dst = {};
  dst.sin_family = AF_INET;
  dst.sin_port   = htons(DANTE_CONMON_PORT);

  dst.sin_addr.s_addr = inet_addr("239.255.0.1");
  sendto(danteSock, probe, sizeof(probe), 0, (struct sockaddr*)&dst, sizeof(dst));

  String bcStr = ETH.broadcastIP().toString();
  dst.sin_addr.s_addr = inet_addr(bcStr.c_str());
  sendto(danteSock, probe, sizeof(probe), 0, (struct sockaddr*)&dst, sizeof(dst));

  dst.sin_addr.s_addr = inet_addr("255.255.255.255");
  int bc255 = 1;
  setsockopt(danteSock, SOL_SOCKET, SO_BROADCAST, &bc255, sizeof(bc255));
  sendto(danteSock, probe, sizeof(probe), 0, (struct sockaddr*)&dst, sizeof(dst));

  Serial.printf("Dante probes sent (multicast + %s + 255.255.255.255)\n", bcStr.c_str());
}

void processDanteScan() {
  if (!danteScanRunning || danteSock < 0) return;
  if (danteScanStart == 0) danteScanStart = millis();

  uint8_t buf[256];
  struct sockaddr_in src = {};
  socklen_t srcLen = sizeof(src);
  int bread;

  while ((bread = recvfrom(danteSock, buf, sizeof(buf), 0,
                           (struct sockaddr*)&src, &srcLen)) > 0) {
    danteRawPkts++;
    if (bread < 4) continue;

    char ipBuf[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &src.sin_addr, ipBuf, sizeof(ipBuf));
    String ipStr = String(ipBuf);

    // Skip own IP and multicast source addresses
    if (ipStr == ETH.localIP().toString()) continue;
    if ((ntohl(src.sin_addr.s_addr) >> 28) == 0xE) continue;  // 224.x.x.x/4

    bool dup = false;
    for (int i = 0; i < danteHostCount; i++) {
      if (danteHosts[i].ip == ipStr) { dup = true; break; }
    }
    if (dup || danteHostCount >= MAX_DANTE_HOSTS) continue;

    DanteHost &h = danteHosts[danteHostCount++];
    h.ip    = ipStr;
    h.name  = extractDanteString(buf, 4, bread, 32);
    h.proto = "Dante";
    danteScanDirty = true;

    Serial.printf("Dante host: %s name='%s' len=%d\n  HEX:", ipStr.c_str(), h.name.c_str(), bread);
    for (int i = 0; i < bread && i < 32; i++) {
      if (i % 16 == 0) Serial.printf("\n  %02X: ", i);
      Serial.printf("%02X ", buf[i]);
    }
    Serial.println();
  }

  if (millis() - danteScanStart > 8000) {
    danteScanRunning = false;
    danteScanDone    = true;
    danteScanDirty   = true;
    danteScanStart   = 0;
    close(danteSock);
    danteSock = -1;
    Serial.printf("Dante scan done: %d host(s), %d raw UDP pkts on port %d\n",
                   danteHostCount, danteRawPkts, DANTE_CONMON_PORT);
  }
}

// ================== HQNET SCAN ==================
// HiQnet discovery: send a Hello (KeepAlive, MessageType=0x0000) broadcast on port 3804.
// Devices respond immediately, and also broadcast Hello packets periodically.
// Accept any UDP packet on port 3804 — version byte varies across firmware.

WiFiUDP hqnetUDP;
static unsigned long hqnetScanStart = 0;

// After the UDP broadcast window, query mDNS for _hiqnet._tcp to get device names.
// BSS London/BLU series register this service with the device's configured label.
static void hqnetEnrichTaskFn(void *) {
  mdns_result_t *results = NULL;
  esp_err_t err = mdns_query_ptr("_hiqnet", "_tcp", 1500, MAX_HQNET_HOSTS, &results);
  Serial.printf("HiQnet mDNS enrich: err=%d results=%s\n",
                (int)err, results ? "found" : "none");
  if (err == ESP_OK && results) {
    mdns_result_t *r = results;
    while (r) {
      String ipStr;
      mdns_ip_addr_t *a = r->addr;
      while (a) {
        if (a->addr.type == MDNS_IP_PROTOCOL_V4) {
          ipStr = IPAddress(a->addr.u_addr.ip4.addr).toString();
          break;
        }
        a = a->next;
      }
      String devName = r->instance_name ? String(r->instance_name)
                     : (r->hostname     ? String(r->hostname) : "");

      if (ipStr.length() > 0) {
        bool found = false;
        for (int i = 0; i < hqnetHostCount; i++) {
          if (hqnetHosts[i].ip == ipStr) {
            if (devName.length() > 0) {
              hqnetHosts[i].name = devName;
              hqnetScanDirty = true;
            }
            found = true;
            Serial.printf("HiQnet enriched %s -> '%s'\n", ipStr.c_str(), devName.c_str());
            break;
          }
        }
        if (!found && hqnetHostCount < MAX_HQNET_HOSTS) {
          HQNetHost &h  = hqnetHosts[hqnetHostCount++];
          h.ip      = ipStr;
          h.name    = devName;
          h.devType = "HiQnet";
          hqnetScanDirty = true;
          Serial.printf("HiQnet mDNS new: %s '%s'\n", ipStr.c_str(), devName.c_str());
        }
      }
      r = r->next;
    }
    mdns_query_results_free(results);
  }
  hqnetEnriching   = false;
  hqnetScanRunning = false;
  hqnetScanDone    = true;
  hqnetScanDirty   = true;
  Serial.printf("HiQnet scan done: %d host(s)\n", hqnetHostCount);
  vTaskDelete(NULL);
}

void startHQNetScan() {
  if (!ethConnected) return;
  hqnetHostCount   = 0;
  hqnetScanRunning = true;
  hqnetScanDone    = false;
  hqnetScanDirty   = true;
  hqnetEnriching   = false;
  hqnetScanStart   = 0;
  hqnetUDP.begin(HQNET_PORT);

  // HiQnet Hello (KeepAlive) packet — 20-byte header, MessageType=0x0000.
  // Standard header layout (big-endian):
  //   [0-3]  Length=20  [4] Version=2  [5-6] SessionID=0
  //   [7-8]  MessageType=0x0000 (Hello)  [9] HopCount=1  [10-11] Flags=0
  //   [12-13] Source Node=0  [14-15] Source VDev=0
  //   [16-17] Dest Node=0xFFFF  [18-19] Dest VDev=0xFFFF
  uint8_t pkt[20] = {
    0x00, 0x00, 0x00, 0x14,  // length = 20
    0x02,                     // version = 2
    0x00, 0x00,               // session ID = 0
    0x00, 0x00,               // MessageType = 0x0000 (Hello/KeepAlive)
    0x01,                     // HopCount = 1
    0x00, 0x00,               // Flags = 0
    0x00, 0x00,               // source node = 0 (unassigned)
    0x00, 0x00,               // source VDevice = 0
    0xFF, 0xFF,               // dest node = 0xFFFF (all nodes)
    0xFF, 0xFF,               // dest VDevice = 0xFFFF
  };

  IPAddress broadcast = ETH.broadcastIP();
  hqnetUDP.beginPacket(broadcast, HQNET_PORT);
  hqnetUDP.write(pkt, sizeof(pkt));
  hqnetUDP.endPacket();

  hqnetUDP.beginPacket(IPAddress(255, 255, 255, 255), HQNET_PORT);
  hqnetUDP.write(pkt, sizeof(pkt));
  hqnetUDP.endPacket();

  Serial.printf("HiQnet Announce sent to %s and 255.255.255.255\n", broadcast.toString().c_str());
}

// Extract a printable ASCII string from buf starting at offset, up to maxLen chars.
static String extractHQNetString(const uint8_t *buf, int offset, int bufLen, int maxLen = 32) {
  String s;
  for (int i = offset; i < bufLen && (int)s.length() < maxLen; i++) {
    char c = (char)buf[i];
    if (c == '\0') break;
    if (c >= 0x20 && c < 0x7F) s += c;
  }
  return s;
}

void processHQNetScan() {
  if (!hqnetScanRunning) return;

  if (hqnetScanStart == 0) hqnetScanStart = millis();

  int len;
  while ((len = hqnetUDP.parsePacket()) > 0) {
    if (len < 13) { hqnetUDP.flush(); continue; }

    uint8_t buf[256];
    int bread = hqnetUDP.read(buf, sizeof(buf));
    hqnetUDP.flush();
    if (bread < 13) continue;

    // Accept any packet on port 3804 — version byte varies across firmware versions.

    IPAddress replyIP = hqnetUDP.remoteIP();
    String ipStr = replyIP.toString();

    // Deduplicate
    bool dup = false;
    for (int i = 0; i < hqnetHostCount; i++) {
      if (hqnetHosts[i].ip == ipStr) { dup = true; break; }
    }
    if (dup || hqnetHostCount >= MAX_HQNET_HOSTS) continue;

    HQNetHost &h = hqnetHosts[hqnetHostCount++];
    h.ip = ipStr;

    // HiQnet header layout (v2):
    //   [0]=Version [1]=Flags [2-5]=Length [6-7]=MessageType
    //   [8-9]=SeqNum [10]=HopCount [11]=Reserved
    //   [12-13]=SourceNode  [14-15]=SourceVDev  [16-18]=SourceObj
    //   [19-20]=DestNode    [21-22]=DestVDev    [23-25]=DestObj
    // Payload starts at byte 26; no device-name string in Hello/broadcast packets.
    // Use the node address as a unique device identifier.
    uint16_t nodeAddr = (bread >= 14) ? ((uint16_t)buf[12] << 8 | buf[13]) : 0;
    char nodeBuf[16];
    snprintf(nodeBuf, sizeof(nodeBuf), "Node:0x%04X", nodeAddr);
    h.name    = "";        // not available without a follow-up unicast attribute query
    h.devType = String(nodeBuf);

    hqnetScanDirty = true;
    Serial.printf("HiQnet host: %s node=0x%04X msgType=0x%02X%02X len=%d\n",
                  ipStr.c_str(), nodeAddr, buf[6], buf[7], bread);
  }

  if (millis() - hqnetScanStart > 5000 && !hqnetEnriching) {
    hqnetEnriching = true;
    hqnetScanStart = 0;
    hqnetUDP.stop();
    // Spawn enrichment task to query mDNS for device names
    xTaskCreatePinnedToCore(hqnetEnrichTaskFn, "hqnet_enrich", 8192, nullptr, 1, nullptr, 1);
  }
}

// ================== WIFI CHANNEL SCAN ==================
// The ESP32 WiFi scan API is non-blocking when called with async=true.
// However, the radio must be initialised in STA mode first.

static bool wifiRadioReady = false;

void ensureWifiRadio() {
  if (wifiRadioReady) return;
  // Start WiFi in station mode (radio only, no connection attempt).
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);
  wifiRadioReady = true;
}

void startWifiScan() {
  ensureWifiRadio();
  wifiNetworkCount = 0;
  wifiScanRunning  = true;
  wifiScanDone     = false;
  wifiScanDirty    = true;
  // Start async scan: both 2.4GHz and 5GHz bands, show hidden SSIDs.
  WiFi.scanNetworks(/*async=*/true, /*show_hidden=*/true, /*passive=*/false,
                    /*max_ms_per_chan=*/100, /*channel=*/0);
  Serial.println("WiFi scan started");
}

void processWifiScan() {
  if (!wifiScanRunning) return;

  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) return;  // still in progress

  if (n < 0) {
    // Error or not yet started
    wifiScanRunning = false;
    wifiScanDone    = true;
    wifiScanDirty   = true;
    Serial.printf("WiFi scan error: %d\n", n);
    return;
  }

  // Sort by channel then RSSI (simple insertion sort — small n)
  // First copy into local array
  int cnt = min(n, MAX_WIFI_NETWORKS);
  for (int i = 0; i < cnt; i++) {
    strncpy(wifiNetworks[i].ssid, WiFi.SSID(i).c_str(), 32);
    wifiNetworks[i].ssid[32]    = '\0';
    wifiNetworks[i].channel     = WiFi.channel(i);
    wifiNetworks[i].rssi        = WiFi.RSSI(i);
    wifiNetworks[i].authmode    = (uint8_t)WiFi.encryptionType(i);
  }
  wifiNetworkCount = cnt;

  // Sort by channel ascending, then RSSI descending within channel.
  for (int i = 1; i < wifiNetworkCount; i++) {
    WifiNetwork key = wifiNetworks[i];
    int j = i - 1;
    while (j >= 0 && (wifiNetworks[j].channel > key.channel ||
           (wifiNetworks[j].channel == key.channel && wifiNetworks[j].rssi < key.rssi))) {
      wifiNetworks[j + 1] = wifiNetworks[j];
      j--;
    }
    wifiNetworks[j + 1] = key;
  }

  WiFi.scanDelete();
  wifiScanRunning  = false;
  wifiScanDone     = true;
  wifiScanDirty    = true;
  Serial.printf("WiFi scan done: %d network(s)\n", wifiNetworkCount);
}

void updateWifiScanIfNeeded() {
  if (!isWifiScanPage()) return;

  bool changed = wifiScanDirty ||
                 (wifiNetworkCount != lastWifiCount) ||
                 (wifiScanRunning  != lastWifiRunning);

  if (!changed) return;

  drawList();
  wifiScanDirty   = false;
  lastWifiCount   = wifiNetworkCount;
  lastWifiRunning = wifiScanRunning;
}

void updateNetInfoIfNeeded() {
  if (!isNetInfoPage()) return;
  static bool lastConn = !ethConnected;  // force first draw
  if (ethConnected != lastConn) {
    lastConn = ethConnected;
    drawList();
  }
}

// ================== TOUCH SCROLL ==================
void handleTouch() {
  constexpr int SCAN_BUTTON_TOP = TITLE_H + 6;
  constexpr int SCAN_BUTTON_BOTTOM = TITLE_H + 26;

  static int16_t touchStartX  = -1;
  static int16_t touchStartY  = -1;
  static int16_t touchLastX   = -1;
  static int16_t touchLastY   = -1;
  static bool    touching     = false;
  static bool    touchInTitle = false;
  static unsigned long lastTouchPacketMs = 0;

  int16_t tx = 0;
  int16_t ty = 0;
  bool pressed = false;
  bool gotPacket = readAR1021Touch(tx, ty, pressed);

  if (gotPacket && pressed) {
    if (!touching) {
      touching     = true;
      touchStartX  = tx;
      touchStartY  = ty;
      // Only the top title bar changes pages; lower screen area remains for scrolling.
      touchInTitle = (ty < TITLE_H);
    }
    touchLastX = tx;
    touchLastY = ty;
    lastTouchPacketMs = millis();
  }

  bool releasedByPacket  = gotPacket && !pressed;
  bool releasedByTimeout = touching && (millis() - lastTouchPacketMs > 120);

  if (touching && (releasedByPacket || releasedByTimeout)) {
    touching = false;
    int dx    = touchLastX - touchStartX;
    int dy    = touchLastY - touchStartY;
    int total = pageItemCount();

    bool isTitleTap = touchInTitle && touchLastY < TITLE_H && abs(dy) < 15 && abs(dx) < 30;
    bool isBodyTap = !touchInTitle && abs(dy) < 20 && abs(dx) < 20;

    if (isTitleTap) {
      settingsEditField = -1;  // close any open IP editor when leaving a page
      if (isMenuPage()) {
        // Tap title on menu: go back to the page we came from (NDI Sources).
        currentPage = PAGE_NDI_SOURCES;
      } else if (currentPage == PAGE_WIFI_SCAN) {
        // Go back to menu from WiFi scan.
        currentPage = PAGE_MENU;
      } else {
        // On any content page: open the menu.
        currentPage = PAGE_MENU;
      }
      startIndex     = 0;
      selectedIndex  = -1;
      drawTitle();
      drawList();
    } else if (!touchInTitle) {
      // --- Menu tile tap: use start position (where finger went DOWN) ---
      if (isMenuPage() && isBodyTap) {
        int tileIdx = menuHitTest(touchStartX, touchStartY);
        if (tileIdx >= 0 && tileIdx < MENU_TILE_COUNT) {
          settingsEditField = -1;
          currentPage   = MENU_TILES[tileIdx].target;
          startIndex    = 0;
          selectedIndex = -1;
          drawTitle();
          drawList();
        }
      // --- Scan button taps ---
      } else if (isIpScannerPage() && isBodyTap && touchLastY >= SCAN_BUTTON_TOP && touchLastY <= SCAN_BUTTON_BOTTOM) {
        if (!ipScanRunning) startIpScan();
      } else if (isArtNetPage() && isBodyTap && touchLastY >= SCAN_BUTTON_TOP && touchLastY <= SCAN_BUTTON_BOTTOM) {
        if (!artnetScanRunning) startArtNetScan();
      } else if (isDantePage() && isBodyTap && touchLastY >= SCAN_BUTTON_TOP && touchLastY <= SCAN_BUTTON_BOTTOM) {
        if (!danteScanRunning) startDanteScan();
      } else if (isHQNetPage() && isBodyTap && touchLastY >= SCAN_BUTTON_TOP && touchLastY <= SCAN_BUTTON_BOTTOM) {
        if (!hqnetScanRunning) startHQNetScan();
      } else if (isWifiScanPage() && isBodyTap && touchLastY >= SCAN_BUTTON_TOP && touchLastY <= SCAN_BUTTON_BOTTOM) {
        if (!wifiScanRunning) startWifiScan();
      } else if (isSettingsPage()) {
        // Settings page: allow slightly looser tap detection than elsewhere.
        bool isTap = abs(dy) < 30 && abs(dx) < 30;
        if (!isTap) { /* ignore swipes */ }
        else if (settingsEditField >= 0) {
          // --- IP octet editor taps ---
          const int TOP    = TITLE_H + 142;
          const int PLUS_Y = TOP + 22;
          const int VL_Y   = TOP + 66;  // [-] row
          const int ACT_Y  = TOP + 94;
          int col = touchLastX / 60;
          if (col > 3) col = 3;

          if (touchLastY >= PLUS_Y && touchLastY < PLUS_Y + 20) {
            // [+]
            settingsEditOcts[col] = (uint8_t)((settingsEditOcts[col] + 1) % 256);
            drawSettingsPageOctEditor();
          } else if (touchLastY >= VL_Y && touchLastY < VL_Y + 20) {
            // [-]
            settingsEditOcts[col] = (uint8_t)((settingsEditOcts[col] + 255) % 256);
            drawSettingsPageOctEditor();
          } else if (touchLastY >= ACT_Y) {
            if (touchLastX < 120) {
              // Cancel — discard changes
              settingsEditField = -1;
            } else {
              // Save — copy octets back into the right field
              uint8_t *flds[4] = { appSettings.ip, appSettings.mask,
                                   appSettings.gw, appSettings.dns };
              memcpy(flds[settingsEditField], settingsEditOcts, 4);
              settingsEditField = -1;
            }
            drawSettingsPage();
          }
        } else {
          // --- Normal settings view taps ---
          const int TOGGLE_Y  = TITLE_H + 18;
          const int FIELDS_Y  = TITLE_H + 46;
          const int APPLY_Y   = TITLE_H + 112;
          const int DISP_Y    = TITLE_H + 156;

          if (touchLastY >= TOGGLE_Y && touchLastY < TOGGLE_Y + 24) {
            // DHCP / Static toggle
            uint8_t newMode = (touchLastX >= 120) ? 1 : 0;
            if (newMode != appSettings.staticIP) {
              appSettings.staticIP = newMode;
              drawSettingsPage();
            }
          } else if (appSettings.staticIP &&
                     touchLastY >= FIELDS_Y && touchLastY < FIELDS_Y + 60) {
            // Tap a static IP field to open the octet editor
            int fi = (touchLastY - FIELDS_Y) / 15;
            if (fi >= 0 && fi < 4) {
              uint8_t *flds[4] = { appSettings.ip, appSettings.mask,
                                   appSettings.gw, appSettings.dns };
              settingsEditField = fi;
              memcpy(settingsEditOcts, flds[fi], 4);
              drawSettingsPage();
            }
          } else if (touchLastY >= APPLY_Y && touchLastY < APPLY_Y + 24) {
            // Apply & Restart
            saveAppSettings();
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.setTextSize(2);
            tft.setCursor(10, 110); tft.print("Settings saved.");
            tft.setCursor(10, 138); tft.print("Restarting...");
            delay(1500);
            ESP.restart();
          } else if (touchLastY >= DISP_Y && touchLastY < DISP_Y + 24) {
            // Normal / Flipped display toggle — applied immediately
            uint8_t newFlip = (touchLastX >= 120) ? 1 : 0;
            if (newFlip != appSettings.flipScreen) {
              appSettings.flipScreen = newFlip;
              saveAppSettings();
              tft.setRotation(appSettings.flipScreen ? 0 : 2);
              tft.fillScreen(TFT_BLACK);
              drawTitle();
              drawList();
            }
          }
        }
      } else if (dy < -20 && (startIndex + visibleRowsForCurrentPage()) < total) {
        startIndex++;
      } else if (dy > 20 && startIndex > 0) {
        startIndex--;
      }
    }
  }
}

// ================== STATE UPDATES ==================
void updateEthStatusIfNeeded() {
  if (ethConnected != lastEthConnected) {
    drawEthStatus();
    lastEthConnected = ethConnected;
  }
}

void updateScrollIfNeeded() {
  if (startIndex != lastStartIndex) {
    drawList();
    lastStartIndex = startIndex;
  }
}

void updateSelectionIfNeeded() {
  if (isIpScannerPage() || isArtNetPage() || isDantePage() || isHQNetPage() ||
      isWifiScanPage()  || isMenuPage()   || isNetInfoPage() || isSettingsPage()) {
    lastSelected = selectedIndex;
    return;
  }

  if (selectedIndex != lastSelected) {
    if (lastSelected >= startIndex && lastSelected < startIndex + VISIBLE_ROWS)
      drawRow(lastSelected, lastSelected - startIndex);
    if (selectedIndex >= startIndex && selectedIndex < startIndex + VISIBLE_ROWS)
      drawRow(selectedIndex, selectedIndex - startIndex);
    lastSelected = selectedIndex;
  }
}

void updateNDIIfNeeded() {
  if (isIpScannerPage() || isArtNetPage() || isDantePage() || isHQNetPage() ||
      isWifiScanPage()  || isMenuPage()   || isNetInfoPage() || isSettingsPage()) return;

  if (isAllDevicesPage()) {
    if (allHostsCount != lastAllHostsCount) {
      drawList();
      lastAllHostsCount = allHostsCount;
    }
    return;
  }

  bool changed = (ndiCount != lastNDICount);

  for (int i = 0; i < ndiCount && !changed; i++) {
    if (ndiList[i].name       != lastNDI[i].name       ||
        ndiList[i].stream     != lastNDI[i].stream     ||
        ndiList[i].ip         != lastNDI[i].ip         ||
        ndiList[i].reachable  != lastNDI[i].reachable  ||
        ndiList[i].isReceiver != lastNDI[i].isReceiver) {
      changed = true;
    }
  }

  if (!changed) return;

  drawList();

  lastNDICount = ndiCount;
  for (int i = 0; i < ndiCount; i++) {
    lastNDI[i] = { ndiList[i].name, ndiList[i].stream, ndiList[i].ip, ndiList[i].reachable, ndiList[i].isReceiver };
  }
}

void updateIpScanIfNeeded() {
  if (!isIpScannerPage()) return;

  bool changed = ipScanDirty ||
                 (scanHostCount != lastScanHostCount) ||
                 (ipScanRunning != lastIpScanRunning) ||
                 (ipScanCurrentHost != lastIpScanCurrentHost);

  if (!changed) return;

  drawList();
  ipScanDirty = false;
  lastScanHostCount = scanHostCount;
  lastIpScanRunning = ipScanRunning;
  lastIpScanCurrentHost = ipScanCurrentHost;
}

void updateArtnetIfNeeded() {
  if (!isArtNetPage()) return;

  bool changed = artnetScanDirty ||
                 (artnetHostCount  != lastArtnetCount) ||
                 (artnetScanRunning != lastArtnetRunning);

  if (!changed) return;

  drawList();
  artnetScanDirty  = false;
  lastArtnetCount  = artnetHostCount;
  lastArtnetRunning = artnetScanRunning;
}

void updateDanteIfNeeded() {
  if (!isDantePage()) return;

  bool changed = danteScanDirty ||
                 (danteHostCount   != lastDanteCount) ||
                 (danteScanRunning != lastDanteRunning);

  if (!changed) return;

  drawList();
  danteScanDirty  = false;
  lastDanteCount  = danteHostCount;
  lastDanteRunning = danteScanRunning;
}

void updateHQNetIfNeeded() {
  if (!isHQNetPage()) return;

  bool changed = hqnetScanDirty ||
                 (hqnetHostCount   != lastHqnetCount) ||
                 (hqnetScanRunning != lastHqnetRunning);

  if (!changed) return;

  drawList();
  hqnetScanDirty   = false;
  lastHqnetCount   = hqnetHostCount;
  lastHqnetRunning = hqnetScanRunning;
}

// ================== SCAN TASK ==================
NDISource         stagingList[MAX_NDI];
int               stagingCount    = 0;
NDISource         stagingAllList[MAX_NDI];
int               stagingAllCount = 0;
volatile bool     stageDirty      = false;
SemaphoreHandle_t stageMutex;

void scanTask(void *) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(3000));

    NDISource tmp[MAX_NDI];    int tmpCount    = 0;
    NDISource tmpAll[MAX_NDI]; int tmpAllCount = 0;
    scanReceivers(tmp, tmpCount, tmpAll, tmpAllCount);
    scanNDI(tmp, tmpCount, tmpAll, tmpAllCount);
    checkReachable(tmp, tmpCount);

    if (xSemaphoreTake(stageMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      stagingCount = tmpCount;
      for (int i = 0; i < tmpCount; i++) stagingList[i] = tmp[i];
      stagingAllCount = tmpAllCount;
      for (int i = 0; i < tmpAllCount; i++) stagingAllList[i] = tmpAll[i];
      stageDirty = true;
      xSemaphoreGive(stageMutex);
    }
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  pinMode(CALIB_BUTTON_PIN, INPUT);
  Wire.begin(AR1021_SDA_PIN, AR1021_SCL_PIN);
  Wire.setClock(100000);  // 100 kHz standard mode
  // Internal pull-ups supplement any external resistors on SDA/SCL.
  pinMode(AR1021_SDA_PIN, INPUT_PULLUP);
  pinMode(AR1021_SCL_PIN, INPUT_PULLUP);
  Wire.begin(AR1021_SDA_PIN, AR1021_SCL_PIN);
  delay(600);  // AR1021 needs ~500ms to fully start up

  ar1021EnableTouch();

  bool fsOk = SPIFFS.begin(true);
  if (!fsOk) {
    Serial.println("SPIFFS mount failed; using default settings");
  }
  if (fsOk) loadAppSettings();

  tft.init();
  tft.setRotation(appSettings.flipScreen ? 0 : 2);
  tft.fillScreen(TFT_BLACK);
  delay(200);

  bool calLoaded = fsOk && loadTouchCalibration();
  bool forceCal = isCalButtonPressed();
  if (!calLoaded || forceCal) {
    Serial.println("Starting touch calibration...");
    bool calOk = fsOk && runTouchCalibration();
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(calOk ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, 20);
    tft.print(calOk ? "Calibration saved" : "Calibration failed");
    delay(1000);
  }

  drawSplashScreen();

  // PHY power pulse (MANDATORY on Olimex ESP32-PoE-ISO)
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  delay(250);
  digitalWrite(12, HIGH);
  delay(250);

  WiFi.onEvent(onEthEvent);
  Serial.println("Starting Ethernet...");

  // Apply static IP configuration before ETH.begin() if enabled.
  if (appSettings.staticIP) {
    IPAddress sip(appSettings.ip[0], appSettings.ip[1], appSettings.ip[2], appSettings.ip[3]);
    IPAddress sgw(appSettings.gw[0], appSettings.gw[1], appSettings.gw[2], appSettings.gw[3]);
    IPAddress smk(appSettings.mask[0], appSettings.mask[1], appSettings.mask[2], appSettings.mask[3]);
    IPAddress sdn(appSettings.dns[0], appSettings.dns[1], appSettings.dns[2], appSettings.dns[3]);
    ETH.config(sip, sgw, smk, sdn);
    Serial.printf("Static IP configured: %s\n", sip.toString().c_str());
  }

  ETH.begin(
    ETH_PHY_LAN8720,     // phy type
    0,                   // phy_addr
    23,                  // MDC
    18,                  // MDIO
    12,                  // power pin
    ETH_CLOCK_GPIO17_OUT
  );

  MDNS.begin("ndi-browser");

  stageMutex = xSemaphoreCreateMutex();

  // Initial synchronous scan so the display has data immediately on boot.
  {
    NDISource tmp[MAX_NDI];    int tmpCount    = 0;
    NDISource tmpAll[MAX_NDI]; int tmpAllCount = 0;
    scanReceivers(tmp, tmpCount, tmpAll, tmpAllCount);
    scanNDI(tmp, tmpCount, tmpAll, tmpAllCount);
    checkReachable(tmp, tmpCount);
    ndiCount = tmpCount;
    for (int i = 0; i < ndiCount; i++) ndiList[i] = tmp[i];
    allHostsCount = tmpAllCount;
    for (int i = 0; i < allHostsCount; i++) allHostsList[i] = tmpAll[i];
  }

  // Background task handles all subsequent scans without blocking the main loop.
  xTaskCreatePinnedToCore(scanTask, "ndi_scan", 10240, nullptr, 1, nullptr, 0);

  currentPage   = PAGE_MENU;
  startIndex    = 0;
  selectedIndex = -1;
  drawTitle();
  drawList();
}

// ================== LOOP ==================
void loop() {
  // Pull scan results from the background task when ready (non-blocking).
  if (stageDirty) {
    if (xSemaphoreTake(stageMutex, 0) == pdTRUE) {
      ndiCount = stagingCount;
      for (int i = 0; i < ndiCount; i++) ndiList[i] = stagingList[i];
      allHostsCount = stagingAllCount;
      for (int i = 0; i < allHostsCount; i++) allHostsList[i] = stagingAllList[i];
      stageDirty = false;
      xSemaphoreGive(stageMutex);
    }
  }

  processIpScan();
  processArtNetScan();
  processDanteScan();
  processHQNetScan();
  processWifiScan();
  handleTouch();
  updateEthStatusIfNeeded();
  updateScrollIfNeeded();
  updateSelectionIfNeeded();
  updateNDIIfNeeded();
  updateIpScanIfNeeded();
  updateArtnetIfNeeded();
  updateDanteIfNeeded();
  updateHQNetIfNeeded();
  updateWifiScanIfNeeded();
  updateNetInfoIfNeeded();
}
