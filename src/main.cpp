#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ETH.h>
#include <ESPmDNS.h>
#include <mdns.h>
#include <Wire.h>
#include <SPIFFS.h>

#include <TFT_eSPI.h>

// ================== DISPLAY ==================
TFT_eSPI tft = TFT_eSPI();

// ================== TOUCH (AR1021 over I2C) ==================
constexpr int     AR1021_SDA_PIN  = 13;
constexpr int     AR1021_SCL_PIN  = 16;
constexpr uint8_t AR1021_I2C_ADDR = 0x4D;

constexpr int CALIB_BUTTON_PIN = 34;
constexpr bool CALIB_BUTTON_ACTIVE_LOW = true;

constexpr uint32_t TOUCH_CAL_MAGIC = 0x54434C31;  // "TCL1"
constexpr uint16_t TOUCH_CAL_VERSION = 1;
constexpr const char *TOUCH_CAL_FILE = "/touch_cal.bin";

// Fallback defaults if no calibration file is available.
constexpr uint16_t DEFAULT_RAW_MIN_X = 120;
constexpr uint16_t DEFAULT_RAW_MAX_X = 3950;
constexpr uint16_t DEFAULT_RAW_MIN_Y = 120;
constexpr uint16_t DEFAULT_RAW_MAX_Y = 3950;

struct TouchCalibration {
  uint32_t magic;
  uint16_t version;
  uint16_t minX;
  uint16_t maxX;
  uint16_t minY;
  uint16_t maxY;
};

TouchCalibration touchCal = {
  TOUCH_CAL_MAGIC,
  TOUCH_CAL_VERSION,
  DEFAULT_RAW_MIN_X,
  DEFAULT_RAW_MAX_X,
  DEFAULT_RAW_MIN_Y,
  DEFAULT_RAW_MAX_Y
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
};

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

  rawX = constrain(rawX, touchCal.minX, touchCal.maxX);
  rawY = constrain(rawY, touchCal.minY, touchCal.maxY);

  x = (int16_t)map(rawX, touchCal.minX, touchCal.maxX, 0, SCREEN_W - 1);
  y = (int16_t)map(rawY, touchCal.minY, touchCal.maxY, 0, SCREEN_H - 1);
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
    loaded.maxX > loaded.minX + 100 &&
    loaded.maxY > loaded.minY + 100;

  if (!headerOk || !rangesOk) return false;

  touchCal = loaded;
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

  uint16_t newMinX = min(tlX, brX);
  uint16_t newMaxX = max(tlX, brX);
  uint16_t newMinY = min(tlY, brY);
  uint16_t newMaxY = max(tlY, brY);

  if (newMaxX <= newMinX + 100 || newMaxY <= newMinY + 100) {
    return false;
  }

  TouchCalibration previous = touchCal;
  touchCal.minX = newMinX;
  touchCal.maxX = newMaxX;
  touchCal.minY = newMinY;
  touchCal.maxY = newMaxY;

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
    tft.print("All mDNS Devices");
  } else {
    tft.print("IP Scanner");
  }
  drawEthStatus();
}

int pageItemCount() {
  if (currentPage == PAGE_NDI_SOURCES) return ndiCount;
  if (currentPage == PAGE_ALL_DEVICES) return allHostsCount;
  return scanHostCount;
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

void drawList() {
  if (isIpScannerPage()) {
    drawIpScannerPage();
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
  if (!isIpScannerPage()) return VISIBLE_ROWS;
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

    bool isTitleTap = touchInTitle && abs(dy) < 30 && abs(dx) < 30;
    bool isBodyTap = !touchInTitle && abs(dy) < 20 && abs(dx) < 20;

    if (isTitleTap) {
      // Tap title to cycle through pages.
      currentPage = (currentPage == PAGE_IP_SCANNER)
        ? PAGE_NDI_SOURCES
        : (UiPage)((int)currentPage + 1);
      startIndex     = 0;
      selectedIndex  = -1;
      drawTitle();
      drawList();
    } else if (!touchInTitle) {
      if (isIpScannerPage() && isBodyTap && touchLastY >= SCAN_BUTTON_TOP && touchLastY <= SCAN_BUTTON_BOTTOM) {
        if (!ipScanRunning) startIpScan();
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
  if (isIpScannerPage()) {
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
  if (isIpScannerPage()) return;

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
    Serial.println("SPIFFS mount failed; using default touch calibration");
  }

  tft.init();
  tft.setRotation(2);
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

  drawTitle();

  // PHY power pulse (MANDATORY on Olimex ESP32-PoE-ISO)
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  delay(250);
  digitalWrite(12, HIGH);
  delay(250);

  WiFi.onEvent(onEthEvent);
  Serial.println("Starting Ethernet...");
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
  handleTouch();
  updateEthStatusIfNeeded();
  updateScrollIfNeeded();
  updateSelectionIfNeeded();
  updateNDIIfNeeded();
  updateIpScanIfNeeded();
}
