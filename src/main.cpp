#include <Arduino.h>
#include <WiFi.h>
#include <ETH.h>
#include <ESPmDNS.h>
#include <mdns.h>

#include <TFT_eSPI.h>

// ================== DISPLAY ==================
TFT_eSPI tft = TFT_eSPI();

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
};

NDISource ndiList[MAX_NDI];
int ndiCount = 0;
int startIndex = 0;
int selectedIndex = -1;

unsigned long lastRefresh = 0;

bool lastEthConnected = false;
int  lastStartIndex   = -1;
int  lastSelected     = -1;

struct NDIShadow {
  String name;
  String stream;
  String ip;
  bool reachable;
};

NDIShadow lastNDI[MAX_NDI];
int lastNDICount = 0;

// ================== HELPERS ==================
int rowY(int row) {
  return TITLE_H + MARGIN + (row * LINE_H * 3);
}

#define VISIBLE_ROWS  ((SCREEN_H - TITLE_H) / (LINE_H * 3))

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
  tft.print("NDI Sources");
  drawEthStatus();
}

void drawScrollArrows() {
  tft.fillRect(SCREEN_W - 12, TITLE_H, 12, SCREEN_H - TITLE_H, TFT_BLACK);
  if (startIndex > 0) {
    tft.fillTriangle(
      SCREEN_W - 11, TITLE_H + 14,
      SCREEN_W - 6,  TITLE_H + 4,
      SCREEN_W - 1,  TITLE_H + 14,
      TFT_WHITE);
  }
  if (startIndex + VISIBLE_ROWS < ndiCount) {
    tft.fillTriangle(
      SCREEN_W - 11, SCREEN_H - 14,
      SCREEN_W - 6,  SCREEN_H - 4,
      SCREEN_W - 1,  SCREEN_H - 14,
      TFT_WHITE);
  }
}

void drawList() {
  tft.fillRect(0, TITLE_H, SCREEN_W, SCREEN_H - TITLE_H, TFT_BLACK);
  tft.setTextSize(1);

  int y = TITLE_H + MARGIN;
  int endIndex = min(startIndex + VISIBLE_ROWS, ndiCount);

  for (int i = startIndex; i < endIndex; i++) {
    bool selected = (i == selectedIndex);
    uint16_t bg = selected ? TFT_DARKCYAN : TFT_BLACK;
    uint16_t fg = selected ? TFT_WHITE : TFT_LIGHTGREY;

    tft.fillRect(0, y - 2, SCREEN_W - 12, LINE_H * 3, bg);
    tft.setTextColor(fg, bg);

    tft.fillCircle(6, y + 4, 4, ndiList[i].reachable ? TFT_GREEN : TFT_RED);

    tft.setCursor(14, y);
    tft.print(ndiList[i].name);

    y += LINE_H;
    tft.setTextColor(TFT_DARKGREY, bg);
    tft.setCursor(14, y);
    tft.print(ndiList[i].stream);

    y += LINE_H;
    tft.setTextColor(fg, bg);
    tft.setCursor(14, y);
    tft.print(ndiList[i].ip);

    y += LINE_H;
  }

  drawScrollArrows();
}

void drawRow(int listIndex, int screenRow) {
  int y = rowY(screenRow);

  bool selected = (listIndex == selectedIndex);
  uint16_t bg = selected ? TFT_DARKCYAN : TFT_BLACK;
  uint16_t fg = selected ? TFT_WHITE : TFT_LIGHTGREY;

  tft.fillRect(0, y - 2, SCREEN_W, LINE_H * 3, bg);
  tft.setTextColor(fg, bg);
  tft.setTextSize(1);

  tft.fillCircle(6, y + 4, 4, ndiList[listIndex].reachable ? TFT_GREEN : TFT_RED);

  tft.setCursor(14, y);
  tft.print(ndiList[listIndex].name);

  tft.setTextColor(TFT_DARKGREY, bg);
  tft.setCursor(14, y + LINE_H);
  tft.print(ndiList[listIndex].stream);

  tft.setTextColor(fg, bg);
  tft.setCursor(14, y + LINE_H * 2);
  tft.print(ndiList[listIndex].ip);
}

// ================== NDI DISCOVERY ==================
void scanNDI() {
  ndiCount = 0;

  mdns_result_t *results = NULL;
  esp_err_t err = mdns_query_ptr("_ndi", "_tcp", 3000, MAX_NDI, &results);
  if (err != ESP_OK || !results) return;

  mdns_result_t *r = results;
  while (r && ndiCount < MAX_NDI) {
    ndiList[ndiCount].name = r->hostname ? String(r->hostname) : "";
    // instance_name is "MACHINE (Stream Name)" - extract just the stream part
    if (r->instance_name) {
      String inst = String(r->instance_name);
      int open  = inst.indexOf('(');
      int close = inst.lastIndexOf(')');
      ndiList[ndiCount].stream = (open >= 0 && close > open)
        ? inst.substring(open + 1, close)
        : inst;
    } else {
      ndiList[ndiCount].stream = ndiList[ndiCount].name;
    }
    ndiList[ndiCount].ip = "";
    mdns_ip_addr_t *addr = r->addr;
    while (addr) {
      if (addr->addr.type == MDNS_IP_PROTOCOL_V4) {
        ndiList[ndiCount].ip = IPAddress(addr->addr.u_addr.ip4.addr).toString();
        break;
      }
      addr = addr->next;
    }
    ndiList[ndiCount].reachable = false;
    ndiCount++;
    r = r->next;
  }
  mdns_query_results_free(results);
}

void checkReachable() {
  for (int i = 0; i < ndiCount; i++) {
    IPAddress ip;
    ip.fromString(ndiList[i].ip);
    ndiList[i].reachable = ip != INADDR_NONE;
  }
}

// ================== TOUCH SCROLL ==================
void handleTouch() {
  static int16_t touchStartY = -1;
  static int16_t touchLastY  = -1;
  static bool    touching    = false;

  uint16_t tx, ty;
  bool isTouched = tft.getTouch(&tx, &ty);

  if (isTouched) {
    if (!touching) {
      touching    = true;
      touchStartY = (int16_t)ty;
    }
    touchLastY = (int16_t)ty;
  } else if (touching) {
    touching = false;
    int dy = touchLastY - touchStartY;
    if (dy < -20 && (startIndex + VISIBLE_ROWS) < ndiCount) {
      startIndex++;
    } else if (dy > 20 && startIndex > 0) {
      startIndex--;
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
  if (selectedIndex != lastSelected) {
    if (lastSelected >= startIndex && lastSelected < startIndex + VISIBLE_ROWS)
      drawRow(lastSelected, lastSelected - startIndex);
    if (selectedIndex >= startIndex && selectedIndex < startIndex + VISIBLE_ROWS)
      drawRow(selectedIndex, selectedIndex - startIndex);
    lastSelected = selectedIndex;
  }
}

void updateNDIIfNeeded() {
  bool changed = (ndiCount != lastNDICount);

  for (int i = 0; i < ndiCount && !changed; i++) {
    if (ndiList[i].name     != lastNDI[i].name   ||
        ndiList[i].stream   != lastNDI[i].stream  ||
        ndiList[i].ip       != lastNDI[i].ip      ||
        ndiList[i].reachable != lastNDI[i].reachable) {
      changed = true;
    }
  }

  if (!changed) return;

  drawList();

  lastNDICount = ndiCount;
  for (int i = 0; i < ndiCount; i++) {
    lastNDI[i] = { ndiList[i].name, ndiList[i].stream, ndiList[i].ip, ndiList[i].reachable };
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);
  drawTitle();
  delay(200);

  // PHY power pulse (MANDATORY on Olimex ESP32-PoE-ISO)
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  delay(250);
  digitalWrite(12, HIGH);
  delay(250);

  WiFi.onEvent(onEthEvent);
  Serial.println("Starting Ethernet...");
  ETH.begin(
    0,                   // phy_addr
    12,                  // power pin
    23,                  // MDC
    18,                  // MDIO
    ETH_PHY_LAN8720,
    ETH_CLOCK_GPIO17_OUT
  );

  MDNS.begin("ndi-browser");

  scanNDI();
  checkReachable();
  drawList();
}

// ================== LOOP ==================
void loop() {
  if (millis() - lastRefresh > 3000) {
    lastRefresh = millis();
    scanNDI();
    checkReachable();
  }

  handleTouch();
  updateEthStatusIfNeeded();
  updateScrollIfNeeded();
  updateSelectionIfNeeded();
  updateNDIIfNeeded();
}
