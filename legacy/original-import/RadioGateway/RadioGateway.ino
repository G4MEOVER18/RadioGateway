/*
  RadioGateway (ESP32 Arduino Core 3.x / 3.3.6)
  - AP + optional STA
  - WebUI (BasicAuth) mit Settings (WiFi/MQTT/SD)
  - MQTT Subscribe + Anzeige letzter Messages
  - SD Config (/radiogateway/config.json) + Web Upload/Download
  - AP Clients Anzeige (MAC-only; robust in Core 3.x)
  - Display optional (TFT_eSPI) + Backlight PWM (Core 3.x: ledcAttach/ledcWrite)
  - Zeitzone: Europe/Zurich (CET/CEST)

  Dependencies:
  - ESP32 board package (Arduino-ESP32) >= 3.3.6
  - PubSubClient
  - ArduinoJson
  - TFT_eSPI
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <esp_wifi.h>
#include <time.h>

// ===================== TFT / DISPLAY (optional) =====================
#define TFT_ENABLED 1

#if TFT_ENABLED
  #include <TFT_eSPI.h>
  TFT_eSPI tft = TFT_eSPI();

  // If your BL pin is different, change here (must match User_Setup.h ideally)
  static const int TFT_BL_PIN = 2;

  static const uint32_t BL_PWM_FREQ = 5000;
  static const uint8_t  BL_PWM_RES  = 12; // 0..4095
  static uint8_t blPercent = 60;
  static unsigned long tft_last_draw_ms = 0;

  static void blWritePercent(uint8_t percent) {
    if (percent > 100) percent = 100;
    blPercent = percent;

    // Hard-enable in case PWM attach fails / pin mode got reset
    pinMode(TFT_BL_PIN, OUTPUT);
    digitalWrite(TFT_BL_PIN, HIGH);

    const uint32_t maxDuty = (1UL << BL_PWM_RES) - 1;
    uint32_t duty = (uint32_t)((percent * maxDuty) / 100);

    // Core 3.x LEDC API
    bool ok = ledcAttach((uint8_t)TFT_BL_PIN, BL_PWM_FREQ, BL_PWM_RES);
    if (ok) {
      ledcWrite((uint8_t)TFT_BL_PIN, duty);
    } else {
      // Fallback: just ON/OFF
      digitalWrite(TFT_BL_PIN, (percent > 0) ? HIGH : LOW);
    }
  }

  // Simple UI helpers
  static void tftBanner(const char* title) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.setTextSize(2);
    tft.drawString(title, 10, 10);
    tft.setTextSize(1);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString("RadioGateway", 10, 34);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
  }

  static void tftBox(int x, int y, int w, int h, uint16_t border, uint16_t fill) {
    tft.fillRoundRect(x, y, w, h, 8, fill);
    tft.drawRoundRect(x, y, w, h, 8, border);
  }

  static void tftDrawBoot(const char* line1, const char* line2) {
    tftBanner("BOOT");
    tftBox(10, 60, 220, 60, TFT_DARKGREY, TFT_NAVY);
    tft.setTextColor(TFT_WHITE, TFT_NAVY);
    tft.setCursor(20, 75);
    tft.print(line1);
    tft.setCursor(20, 95);
    tft.print(line2);

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(10, 140);
    tft.printf("BL: %u%%", blPercent);
  }

  static void tftDrawStatus(const String& apIp, const String& staIp, bool staConn, bool mqttConn, bool sdMounted) {
    tftBanner("STATUS");

    // Row 1 boxes
    tftBox(10, 55, 220, 42, TFT_DARKGREY, TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(18, 62);
    tft.print("AP: ");
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print(apIp);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(18, 80);
    tft.print("STA: ");
    tft.setTextColor(staConn ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print(staConn ? staIp : "disconnected");

    // Row 2 boxes
    tftBox(10, 105, 106, 40, TFT_DARKGREY, TFT_BLACK);
    tftBox(124, 105, 106, 40, TFT_DARKGREY, TFT_BLACK);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(18, 116);
    tft.print("MQTT");
    tft.setCursor(18, 132);
    tft.setTextColor(mqttConn ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print(mqttConn ? "CONNECTED" : "OFFLINE");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(132, 116);
    tft.print("SD");
    tft.setCursor(132, 132);
    tft.setTextColor(sdMounted ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print(sdMounted ? "MOUNTED" : "NO SD");

    // Footer
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(10, 155);
    tft.printf("BL:%u%%", blPercent);
  }

  static void tftDrawMqttLast(const String& topic, const String& payload) {
    tftBox(10, 175, 220, 60, TFT_DARKGREY, TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(18, 182);
    tft.print("Last RX:");

    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setCursor(18, 196);
    String t = topic; if (t.length() > 28) t = t.substring(0, 28);
    tft.print(t);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(18, 210);
    String p = payload; if (p.length() > 32) p = p.substring(0, 32);
    tft.print(p);
  }
#endif

// ===================== USER SETTINGS =====================
static const char* RG_USER = "yanis";
#define RG_PASSWORD "changeme"

static const char* RG_HOSTNAME = "RadioGateway";
static const char* RG_DEVICE   = "RadioGateway";
static const char* RG_LOCATION = "Portable";

static const char* TZ_INFO = "CET-1CEST,M3.5.0/2,M10.5.0/3";
static const char* NTP1 = "pool.ntp.org";
static const char* NTP2 = "time.cloudflare.com";

// ===================== SD / FILES =====================
static const char* CFG_DIR  = "/radiogateway";
static const char* CFG_PATH = "/radiogateway/config.json";

// CYD boards usually: SD CS = 5
#ifndef SD_CS_PIN
  #define SD_CS_PIN 5
#endif

// ===================== NETWORK DEFAULTS =====================
static const char* AP_SSID_DEFAULT = "YourDataMyData";
static const char* AP_PASS_DEFAULT = "KiCinference$94RC";
static const uint8_t AP_CHANNEL_DEFAULT = 6;

IPAddress apIP(192,168,4,1);
IPAddress apGW(192,168,4,1);
IPAddress apMASK(255,255,255,0);

// ===================== MQTT DEFAULTS =====================
static const char* MQTT_HOST_DEFAULT = "mqtt.ameseder.contact";
static const uint16_t MQTT_PORT_DEFAULT = 1883;

static const char* MQTT_TOPIC_RX_DEFAULT     = "home/radiogateway/rx";
static const char* MQTT_TOPIC_TX_DEFAULT     = "home/radiogateway/tx";
static const char* MQTT_TOPIC_STATUS_DEFAULT = "home/radiogateway/status";

// ===================== GLOBAL STATE =====================
WebServer server(80);
WiFiClient netClient;
PubSubClient mqtt(netClient);

struct RGConfig {
  bool ap_enabled = true;
  String ap_ssid = AP_SSID_DEFAULT;
  String ap_pass = AP_PASS_DEFAULT;
  uint8_t ap_channel = AP_CHANNEL_DEFAULT;

  bool sta_enabled = false;
  String sta_ssid = "";
  String sta_pass = "";

  bool mqtt_enabled = true;
  String mqtt_host = MQTT_HOST_DEFAULT;
  uint16_t mqtt_port = MQTT_PORT_DEFAULT;
  String mqtt_user = "";
  String mqtt_pass = "";
  String topic_rx = MQTT_TOPIC_RX_DEFAULT;
  String topic_tx = MQTT_TOPIC_TX_DEFAULT;
  String topic_status = MQTT_TOPIC_STATUS_DEFAULT;

  String device = RG_DEVICE;
  String hostname = RG_HOSTNAME;
  String location = RG_LOCATION;

#if TFT_ENABLED
  uint8_t bl_percent = 60;
#endif
} cfg;

static String mqtt_last_topic = "";
static String mqtt_last_payload = "";
static unsigned long mqtt_last_ms = 0;
static unsigned long mqtt_last_reconnect_try = 0;

File uploadFile;

// ===================== HELPERS =====================
static bool isAuthOk() {
  if (!server.authenticate(RG_USER, RG_PASSWORD)) {
    server.requestAuthentication();
    return false;
  }
  return true;
}

static String ipToString(const IPAddress& ip) {
  return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}

static String nowString() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 100)) return "no-time";
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buf);
}

static String htmlHeader(const String& title) {
  String h;
  h += "<!doctype html><html><head><meta charset='utf-8'>";
  h += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  h += "<title>" + title + "</title>";
  h += "<style>";
  h += "body{font-family:system-ui,Arial;margin:18px;max-width:1100px}";
  h += "a{color:#0b65c2;text-decoration:none} a:hover{text-decoration:underline}";
  h += ".card{border:1px solid #e5e5e5;border-radius:14px;padding:14px;margin:12px 0;box-shadow:0 1px 8px rgba(0,0,0,.04)}";
  h += "code,pre{background:#f6f6f6;border-radius:10px;padding:6px 8px;overflow:auto}";
  h += "input,select{padding:10px;border:1px solid #ccc;border-radius:12px;min-width:280px}";
  h += "button{padding:10px 14px;border:0;border-radius:12px;background:#0b65c2;color:#fff;cursor:pointer}";
  h += "button.secondary{background:#555}";
  h += "table{border-collapse:collapse;width:100%} td,th{border-bottom:1px solid #eee;padding:8px;text-align:left}";
  h += ".muted{color:#666;font-size:0.95em}";
  h += "</style></head><body>";
  h += "<h2>" + title + "</h2>";
  h += "<div class='muted'>Device: <b>" + cfg.device + "</b> | Hostname: <b>" + cfg.hostname + "</b> | Location: <b>" + cfg.location + "</b></div>";
  h += "<p>";
  h += "<a href='/'>Home</a> · ";
  h += "<a href='/wifi'>WiFi</a> · ";
  h += "<a href='/mqtt'>MQTT</a> · ";
  h += "<a href='/devices'>Devices</a> · ";
  h += "<a href='/sd'>SD</a> · ";
  h += "<a href='/config'>Config</a>";
#if TFT_ENABLED
  h += " · <a href='/tft'>Display</a>";
#endif
  h += "</p>";
  return h;
}

static String htmlFooter() {
  return "<hr><div class='muted'>RadioGateway WebUI · Zeitzone Zürich · Port 80</div></body></html>";
}

// ===================== SD CONFIG =====================
static bool ensureDir(fs::FS &fs, const char* path) {
  if (fs.exists(path)) return true;
  return fs.mkdir(path);
}

static bool sdBegin() {
  // Use shared SPI bus, moderate frequency for stability
  SPI.begin(14, 12, 13); // SCLK, MISO, MOSI (matches CYD typical)
  bool ok = SD.begin(SD_CS_PIN, SPI, 25000000);
  if (!ok) return false;
  ensureDir(SD, CFG_DIR);
  return true;
}

static bool sdOk() {
  return SD.cardType() != CARD_NONE;
