
// --- ESP32 Arduino Core 3.x compatibility: HSPI_HOST/VSPI_HOST renamed ---
#ifndef HSPI_HOST
  #define HSPI_HOST SPI2_HOST
#endif
#ifndef VSPI_HOST
  #define VSPI_HOST SPI3_HOST
#endif

// --- Simple runtime stats (compile-safe). Increment where you like. ---
struct GatewayStats {
  uint32_t rx_packets = 0;
  uint32_t tx_packets = 0;
};
static GatewayStats stats;

/*
  RadioGateway (ESP32 Arduino Core 3.x / 3.3.6)
  - AP + optional STA
  - WebUI (BasicAuth) mit Settings (WiFi/MQTT/SD)
  - MQTT Subscribe + Anzeige letzter Messages
  - SD Config (/radiogateway/config.json) + Web Upload/Download
  - AP Clients Anzeige (MAC-only; robust in Core 3.x)
  - Display optional (Arduino_GFX) + Backlight PWM (Core 3.x: ledcAttach/ledcWrite)
  - Zeitzone: Europe/Zurich (CET/CEST)

  Abhängigkeiten (Arduino Library Manager / Board-Paket):
  - ESP32 board package (Arduino-ESP32) >= 3.3.6
  - PubSubClient
  - ArduinoJson
  - Arduino_GFX_Library (nur wenn TFT_ENABLED=1)

  Hinweis Backlight:
  - Du hast beobachtet: "2 display beleuchtung ist an gegangen" → BL_PIN = GPIO2 scheint bei dir plausibel.
  - In vielen Board-Layouts ist GPIO2 aber auch DC. Darum:
    -> Setz TFT_BL_PIN unten explizit auf 2, wenn dein Test das bestätigt.
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
  #include <LovyanGFX.hpp>

  // Farbkonstanten (RGB565)
  #define BLACK   0x0000
  #define WHITE   0xFFFF
  #define RED     0xF800
  #define GREEN   0x07E0
  #define BLUE    0x001F
  #define YELLOW  0xFFE0
  #define CYAN    0x07FF
  #define MAGENTA 0xF81F
  #define ORANGE  0xFD20

  // ===== Board-Pinmap (ESP32-2432S028 / AliExpress-Pinmap) =====
  // TFT (ILI9341, SPI):
  //   SCLK=IO14, MOSI=IO13, MISO=IO12, CS=IO15, DC=IO2, RST=fest/keiner(-1), BL=IO21
  static constexpr int PIN_TFT_MOSI = 13;
  static constexpr int PIN_TFT_MISO = 12;   // falls nicht verdrahtet: -1
  static constexpr int PIN_TFT_SCLK = 14;
  static constexpr int PIN_TFT_CS   = 15;
  static constexpr int PIN_TFT_DC   = 2;
  static constexpr int PIN_TFT_RST  = -1;   // oft fest an 3V3 => -1
  static constexpr int PIN_TFT_BL   = 21;   // Backlight

  // Backward-Compat: älterer Code/HTML nutzt diesen Namen
  #ifndef TFT_BL_PIN
  #define TFT_BL_PIN PIN_TFT_BL
  #endif

  // Wichtig: SD nutzt auf diesem Board typischerweise VSPI Default-Pins (SCK=18 MOSI=23 MISO=19, CS=5).
  // Damit sich SD und TFT nicht in die Quere kommen, läuft TFT auf HSPI.

  class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ILI9341 _panel;
    lgfx::Bus_SPI _bus;

  public:
    LGFX() {
      { // Bus
        auto cfg = _bus.config();
              cfg.spi_host   = SPI2_HOST;     // TFT on SPI2_HOST (HSPI)
        cfg.spi_mode   = 0;
        cfg.freq_write = 40000000;      // 40 MHz
        cfg.freq_read  = 16000000;
        cfg.pin_sclk   = PIN_TFT_SCLK;
        cfg.pin_mosi   = PIN_TFT_MOSI;
        cfg.pin_miso   = PIN_TFT_MISO;
        cfg.pin_dc     = PIN_TFT_DC;
        _bus.config(cfg);
        _panel.setBus(&_bus);
      }
      { // Panel
        auto cfg = _panel.config();
        cfg.pin_cs   = PIN_TFT_CS;
        cfg.pin_rst  = PIN_TFT_RST;
        cfg.pin_busy = -1;

        cfg.memory_width  = 240;
        cfg.memory_height = 320;
        cfg.panel_width   = 240;
        cfg.panel_height  = 320;

        cfg.offset_x = 0;
        cfg.offset_y = 0;
        cfg.invert   = false;
        cfg.rgb_order = false;
        cfg.dlen_16bit = false;

        _panel.config(cfg);
      }

      setPanel(&_panel);
    }
  };

  static LGFX tft;
  static bool tftReady = false;

  // Backlight control
  // NOTE (ESP32 core 3.x): the LEDC API changed (ledcSetup/ledcAttachPin removed).
  // For maximum compatibility across cores and boards, we keep this **simple and robust**:
  // - percent == 0  -> backlight OFF
  // - percent  > 0  -> backlight ON
  // If you later want true PWM dimming, we can add a dual-API wrapper once your core version is fixed.
  static uint32_t tft_last_draw_ms = 0;

  static inline void blInitOnce() {
    static bool inited = false;
    if (inited) return;
    inited = true;
    if (TFT_BL_PIN >= 0) {
      pinMode(TFT_BL_PIN, OUTPUT);
      digitalWrite(TFT_BL_PIN, HIGH); // default ON
    }
  }

  static inline void blWritePercent(uint8_t percent) {
    blInitOnce();
    if (TFT_BL_PIN < 0) return;
    // Simple compatible policy: 0% = OFF, >0% = ON
    digitalWrite(TFT_BL_PIN, (percent == 0) ? LOW : HIGH);
  }

  static void tftDrawBoot(const char* line1, const char* line2) {
    if (!tftReady) return;
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setCursor(8, 10);
    tft.println(line1 ? line1 : "");
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(8, 40);
    tft.println(line2 ? line2 : "");
    tft_last_draw_ms = millis();
  }

  static void tftDrawStatus(
      const char* hostname,
      bool wifiUp,
      const String& ip,
      uint32_t packets,
      int16_t rssi,
      const char* note
  ) {
    if (!tftReady) return;
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(6, 6);
    tft.print(hostname ? hostname : "RadioGateway");

    tft.setTextSize(1);
    tft.setCursor(6, 32);
    tft.setTextColor(wifiUp ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print("WiFi: ");
    tft.println(wifiUp ? "UP" : "DOWN");

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(6, 46);
    tft.print("IP: ");
    tft.println(ip);

    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setCursor(6, 62);
    tft.print("RX packets: ");
    tft.println(packets);

    tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
    tft.setCursor(6, 78);
    tft.print("LoRa RSSI: ");
    tft.println(rssi);

    if (note && *note) {
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setCursor(6, 100);
      tft.println(note);
    }
    tft_last_draw_ms = millis();
  }

  static inline void tftBacklightOn(bool on = true) {
    if (PIN_TFT_BL >= 0) {
      pinMode(PIN_TFT_BL, OUTPUT);
      digitalWrite(PIN_TFT_BL, on ? HIGH : LOW);
    }
  }

  static void tftInit() {
    tftBacklightOn(true);
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(BLACK);
    tftReady = true;
  }

#else
  static void tftInit() {}
#endif

// ===================== USER SETTINGS (WICHTIG) =====================
static const char* RG_USER = "yanis";
#define RG_PASSWORD "changeme"   // WebUI Login (BasicAuth)

static const char* RG_HOSTNAME = "RadioGateway";
static const char* RG_DEVICE   = "RadioGateway";
static const char* RG_LOCATION = "Portable";

// Timezone Zürich (CET/CEST)
static const char* TZ_INFO = "CET-1CEST,M3.5.0/2,M10.5.0/3";
static const char* NTP1 = "pool.ntp.org";
static const char* NTP2 = "time.cloudflare.com";

// ===================== SD / FILES =====================
static const char* CFG_DIR  = "/radiogateway";
static const char* CFG_PATH = "/radiogateway/config.json";

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
  // AP
  bool ap_enabled = true;
  String ap_ssid = AP_SSID_DEFAULT;
  String ap_pass = AP_PASS_DEFAULT;
  uint8_t ap_channel = AP_CHANNEL_DEFAULT;

  // STA
  bool sta_enabled = false;
  String sta_ssid = "";
  String sta_pass = "";

  // MQTT
  bool mqtt_enabled = true;
  String mqtt_host = MQTT_HOST_DEFAULT;
  uint16_t mqtt_port = MQTT_PORT_DEFAULT;
  String mqtt_user = "";
  String mqtt_pass = "";
  String topic_rx = MQTT_TOPIC_RX_DEFAULT;
  String topic_tx = MQTT_TOPIC_TX_DEFAULT;
  String topic_status = MQTT_TOPIC_STATUS_DEFAULT;

  // UI / Meta
  String device = RG_DEVICE;
  String hostname = RG_HOSTNAME;
  String location = RG_LOCATION;

  #if TFT_ENABLED
    uint8_t bl_percent = 60;
  #endif
} cfg;

// MQTT runtime info
static String mqtt_last_topic = "";
static String mqtt_last_payload = "";
static unsigned long mqtt_last_ms = 0;
static unsigned long mqtt_last_reconnect_try = 0;

// Upload handling
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
  h += ".card{border:1px solid #ddd;border-radius:12px;padding:14px;margin:12px 0}";
  h += "code,pre{background:#f6f6f6;border-radius:8px;padding:2px 6px;overflow:auto}";
  h += "input,select{padding:8px;border:1px solid #ccc;border-radius:10px;min-width:280px}";
  h += "button{padding:9px 14px;border:0;border-radius:10px;background:#0b65c2;color:#fff;cursor:pointer}";
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
  if (!SD.begin(SD_CS_PIN)) return false;
  ensureDir(SD, CFG_DIR);
  return true;
}

static bool saveConfigToSD() {
  if (!SD.begin(SD_CS_PIN)) return false;
  ensureDir(SD, CFG_DIR);

  StaticJsonDocument<1400> doc;
  doc["ap_enabled"] = cfg.ap_enabled;
  doc["ap_ssid"] = cfg.ap_ssid;
  doc["ap_pass"] = cfg.ap_pass;
  doc["ap_channel"] = cfg.ap_channel;

  doc["sta_enabled"] = cfg.sta_enabled;
  doc["sta_ssid"] = cfg.sta_ssid;
  doc["sta_pass"] = cfg.sta_pass;

  doc["mqtt_enabled"] = cfg.mqtt_enabled;
  doc["mqtt_host"] = cfg.mqtt_host;
  doc["mqtt_port"] = cfg.mqtt_port;
  doc["mqtt_user"] = cfg.mqtt_user;
  doc["mqtt_pass"] = cfg.mqtt_pass;
  doc["topic_rx"] = cfg.topic_rx;
  doc["topic_tx"] = cfg.topic_tx;
  doc["topic_status"] = cfg.topic_status;

  doc["device"] = cfg.device;
  doc["hostname"] = cfg.hostname;
  doc["location"] = cfg.location;

  #if TFT_ENABLED
    doc["bl_percent"] = cfg.bl_percent;
  #endif

  if (SD.exists(CFG_PATH)) SD.remove(CFG_PATH);
  File f = SD.open(CFG_PATH, FILE_WRITE);
  if (!f) return false;
  serializeJsonPretty(doc, f);
  f.flush();
  f.close();
  return true;
}

static bool loadConfigFromSD() {
  if (!SD.begin(SD_CS_PIN)) return false;
  if (!SD.exists(CFG_PATH)) return false;
  File f = SD.open(CFG_PATH, FILE_READ);
  if (!f) return false;

  StaticJsonDocument<1400> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;

  cfg.ap_enabled = doc["ap_enabled"] | cfg.ap_enabled;
  cfg.ap_ssid    = String((const char*)(doc["ap_ssid"] | cfg.ap_ssid.c_str()));
  cfg.ap_pass    = String((const char*)(doc["ap_pass"] | cfg.ap_pass.c_str()));
  cfg.ap_channel = doc["ap_channel"] | cfg.ap_channel;

  cfg.sta_enabled = doc["sta_enabled"] | cfg.sta_enabled;
  cfg.sta_ssid    = String((const char*)(doc["sta_ssid"] | cfg.sta_ssid.c_str()));
  cfg.sta_pass    = String((const char*)(doc["sta_pass"] | cfg.sta_pass.c_str()));

  cfg.mqtt_enabled = doc["mqtt_enabled"] | cfg.mqtt_enabled;
  cfg.mqtt_host    = String((const char*)(doc["mqtt_host"] | cfg.mqtt_host.c_str()));
  cfg.mqtt_port    = doc["mqtt_port"] | cfg.mqtt_port;
  cfg.mqtt_user    = String((const char*)(doc["mqtt_user"] | cfg.mqtt_user.c_str()));
  cfg.mqtt_pass    = String((const char*)(doc["mqtt_pass"] | cfg.mqtt_pass.c_str()));
  cfg.topic_rx     = String((const char*)(doc["topic_rx"] | cfg.topic_rx.c_str()));
  cfg.topic_tx     = String((const char*)(doc["topic_tx"] | cfg.topic_tx.c_str()));
  cfg.topic_status = String((const char*)(doc["topic_status"] | cfg.topic_status.c_str()));

  cfg.device   = String((const char*)(doc["device"] | cfg.device.c_str()));
  cfg.hostname = String((const char*)(doc["hostname"] | cfg.hostname.c_str()));
  cfg.location = String((const char*)(doc["location"] | cfg.location.c_str()));

  #if TFT_ENABLED
    cfg.bl_percent = doc["bl_percent"] | cfg.bl_percent;
  #endif

  return true;
}

// ===================== WIFI START/STOP =====================
static void startAP() {
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.softAPConfig(apIP, apGW, apMASK);

  if (cfg.ap_pass.length() >= 8) {
    WiFi.softAP(cfg.ap_ssid.c_str(), cfg.ap_pass.c_str(), cfg.ap_channel, 0, 8);
  } else {
    WiFi.softAP(cfg.ap_ssid.c_str(), nullptr, cfg.ap_channel, 0, 8);
  }
}

static void stopAP() {
  WiFi.softAPdisconnect(true);
}

static void startSTA() {
  if (!cfg.sta_enabled || cfg.sta_ssid.length() == 0) return;
  WiFi.begin(cfg.sta_ssid.c_str(), cfg.sta_pass.c_str());

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
    delay(200);
  }
}

static void stopSTA() {
  WiFi.disconnect(true, true);
}

// ===================== MQTT =====================
static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  mqtt_last_topic = String(topic);
  mqtt_last_payload = "";
  mqtt_last_payload.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) mqtt_last_payload += (char)payload[i];
  mqtt_last_ms = millis();
}

static bool mqttConnect() {
  if (!cfg.mqtt_enabled) return false;
  if (cfg.mqtt_host.length() == 0) return false;

  mqtt.setServer(cfg.mqtt_host.c_str(), cfg.mqtt_port);
  mqtt.setCallback(mqttCallback);

  String clientId = String(cfg.hostname) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  bool ok;
  if (cfg.mqtt_user.length() > 0) {
    ok = mqtt.connect(clientId.c_str(), cfg.mqtt_user.c_str(), cfg.mqtt_pass.c_str());
  } else {
    ok = mqtt.connect(clientId.c_str());
  }

  if (ok) {
    mqtt.subscribe(cfg.topic_rx.c_str());

    StaticJsonDocument<256> doc;
    doc["device"] = cfg.device;
    doc["hostname"] = cfg.hostname;
    doc["location"] = cfg.location;
    doc["time"] = nowString();
    doc["ap_ip"] = ipToString(WiFi.softAPIP());
    doc["sta_ip"] = (WiFi.status() == WL_CONNECTED) ? ipToString(WiFi.localIP()) : "disconnected";

    String out;
    serializeJson(doc, out);
    mqtt.publish(cfg.topic_status.c_str(), out.c_str(), true);
  }

  return ok;
}

static void mqttLoop() {
  if (!cfg.mqtt_enabled) return;

  if (!mqtt.connected()) {
    if (millis() - mqtt_last_reconnect_try > 5000) {
      mqtt_last_reconnect_try = millis();
      mqttConnect();
    }
    return;
  }
  mqtt.loop();
}

// ===================== AP CLIENTS (Core 3.x robust) =====================
static String apClientsJson() {
  StaticJsonDocument<768> doc;
  JsonArray arr = doc.createNestedArray("clients");

  wifi_sta_list_t sta_list;
  memset(&sta_list, 0, sizeof(sta_list));

  if (esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK) {
    for (int i = 0; i < sta_list.num; i++) {
      char macStr[18];
      uint8_t* mac = sta_list.sta[i].mac;
      snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

      JsonObject o = arr.createNestedObject();
      o["mac"] = macStr;
    }
  }

  String out;
  serializeJson(doc, out);
  return out;
}

// ===================== SD UTIL =====================
static String mimeFromName(const String& name) {
  if (name.endsWith(".html")) return "text/html";
  if (name.endsWith(".css")) return "text/css";
  if (name.endsWith(".js")) return "application/javascript";
  if (name.endsWith(".json")) return "application/json";
  if (name.endsWith(".txt")) return "text/plain";
  if (name.endsWith(".csv")) return "text/csv";
  if (name.endsWith(".png")) return "image/png";
  if (name.endsWith(".jpg") || name.endsWith(".jpeg")) return "image/jpeg";
  return "application/octet-stream";
}

static bool sdOk() {
  return SD.begin(SD_CS_PIN);
}

static String listDirHtml(const char* path) {
  if (!sdOk()) return "<div class='card'>SD not mounted</div>";
  File root = SD.open(path);
  if (!root || !root.isDirectory()) return "<div class='card'>Not a directory</div>";

  String out = "<div class='card'><h3>SD: " + String(path) + "</h3><table>";
  out += "<tr><th>Name</th><th>Size</th><th>Actions</th></tr>";

  File file = root.openNextFile();
  while (file) {
    String name = String(file.name());
    size_t size = (size_t)file.size();

    out += "<tr><td><code>" + name + "</code></td>";
    out += "<td>" + String(size) + "</td>";
    out += "<td>";
    out += "<a href='/sd/download?f=" + name + "'>Download</a> · ";
    out += "<a href='/sd/delete?f=" + name + "' onclick=\"return confirm('Delete?')\">Delete</a>";
    out += "</td></tr>";

    file = root.openNextFile();
  }

  out += "</table></div>";
  return out;
}

// ===================== WEB HANDLERS =====================
static void handleHome() {
  if (!isAuthOk()) return;

  String s = htmlHeader("RadioGateway – Home");
  s += "<div class='card'><h3>Status</h3><table>";
  s += "<tr><th>Time</th><td>" + nowString() + "</td></tr>";
  s += "<tr><th>AP Enabled</th><td>" + String(cfg.ap_enabled ? "yes" : "no") + "</td></tr>";
  s += "<tr><th>AP SSID</th><td><code>" + cfg.ap_ssid + "</code></td></tr>";
  s += "<tr><th>AP IP</th><td><code>" + ipToString(WiFi.softAPIP()) + "</code></td></tr>";
  s += "<tr><th>STA Enabled</th><td>" + String(cfg.sta_enabled ? "yes" : "no") + "</td></tr>";
  s += "<tr><th>STA State</th><td>" + String(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected") + "</td></tr>";
  s += "<tr><th>STA IP</th><td><code>" + String(WiFi.status() == WL_CONNECTED ? ipToString(WiFi.localIP()) : "n/a") + "</code></td></tr>";
  s += "<tr><th>MQTT Enabled</th><td>" + String(cfg.mqtt_enabled ? "yes" : "no") + "</td></tr>";
  s += "<tr><th>MQTT Broker</th><td><code>" + cfg.mqtt_host + ":" + String(cfg.mqtt_port) + "</code></td></tr>";
  s += "<tr><th>MQTT Connected</th><td>" + String(mqtt.connected() ? "yes" : "no") + "</td></tr>";
  s += "<tr><th>SD Mounted</th><td>" + String(sdOk() ? "yes" : "no") + "</td></tr>";
  #if TFT_ENABLED
    s += "<tr><th>Backlight</th><td>" + String(cfg.bl_percent) + "% (GPIO " + String(TFT_BL_PIN) + ")</td></tr>";
  #endif
  s += "</table></div>";

  s += "<div class='card'><h3>Quick Actions</h3>";
  s += "<form method='POST' action='/action/restartwifi'><button type='submit'>Restart WiFi</button></form><br>";
  s += "<form method='POST' action='/action/reboot'><button type='submit' class='secondary'>Reboot</button></form>";
  s += "</div>";

  s += htmlFooter();
  server.send(200, "text/html", s);
}

static void handleWiFi() {
  if (!isAuthOk()) return;

  String s = htmlHeader("RadioGateway – WiFi");

  s += "<div class='card'><h3>SoftAP (Hotspot)</h3>";
  s += "<form method='POST' action='/wifi/save'>";
  s += "<p><label>AP Enabled</label><br><select name='ap_enabled'>";
  s += "<option value='1'" + String(cfg.ap_enabled ? " selected" : "") + ">Yes</option>";
  s += "<option value='0'" + String(!cfg.ap_enabled ? " selected" : "") + ">No</option>";
  s += "</select></p>";
  s += "<p><label>AP SSID</label><br><input name='ap_ssid' value='" + cfg.ap_ssid + "'></p>";
  s += "<p><label>AP Password (min 8 chars for WPA2, empty for open)</label><br>";
  s += "<input name='ap_pass' value='" + cfg.ap_pass + "'></p>";
  s += "<p><label>AP Channel</label><br><input name='ap_channel' value='" + String(cfg.ap_channel) + "'></p>";
  s += "<button type='submit'>Save WiFi Settings</button>";
  s += "</form></div>";

  s += "<div class='card'><h3>STA (Connect to WLAN)</h3>";
  s += "<form method='POST' action='/wifi/save'>";
  s += "<p><label>STA Enabled</label><br><select name='sta_enabled'>";
  s += "<option value='1'" + String(cfg.sta_enabled ? " selected" : "") + ">Yes</option>";
  s += "<option value='0'" + String(!cfg.sta_enabled ? " selected" : "") + ">No</option>";
  s += "</select></p>";
  s += "<p><label>STA SSID</label><br><input name='sta_ssid' value='" + cfg.sta_ssid + "'></p>";
  s += "<p><label>STA Password</label><br><input name='sta_pass' value='" + cfg.sta_pass + "'></p>";
  s += "<button type='submit'>Save STA Settings</button>";
  s += "</form></div>";

  s += "<div class='card'><h3>Reachability</h3><ul>";
  s += "<li>AP default: <code>http://192.168.4.1/</code> (SSID: <code>YourDataMyData</code>)</li>";
  s += "<li>If STA connected: also via <code>http://" + (WiFi.status()==WL_CONNECTED ? ipToString(WiFi.localIP()) : String("STA-IP")) + "/</code></li>";
  s += "<li>mDNS: <code>http://" + String(cfg.hostname) + ".local/</code> (depends on client support)</li>";
  s += "</ul></div>";

  s += htmlFooter();
  server.send(200, "text/html", s);
}

static void handleWiFiSave() {
  if (!isAuthOk()) return;

  if (server.hasArg("ap_enabled")) cfg.ap_enabled = (server.arg("ap_enabled") == "1");
  if (server.hasArg("ap_ssid"))    cfg.ap_ssid = server.arg("ap_ssid");
  if (server.hasArg("ap_pass"))    cfg.ap_pass = server.arg("ap_pass");
  if (server.hasArg("ap_channel")) cfg.ap_channel = (uint8_t)server.arg("ap_channel").toInt();

  if (server.hasArg("sta_enabled")) cfg.sta_enabled = (server.arg("sta_enabled") == "1");
  if (server.hasArg("sta_ssid"))    cfg.sta_ssid = server.arg("sta_ssid");
  if (server.hasArg("sta_pass"))    cfg.sta_pass = server.arg("sta_pass");

  saveConfigToSD();

  if (cfg.ap_enabled) startAP(); else stopAP();
  if (cfg.sta_enabled) startSTA(); else stopSTA();

  server.sendHeader("Location", "/wifi");
  server.send(302, "text/plain", "Redirect");
}

static void handleMQTT() {
  if (!isAuthOk()) return;

  String s = htmlHeader("RadioGateway – MQTT");

  s += "<div class='card'><h3>MQTT Settings</h3>";
  s += "<form method='POST' action='/mqtt/save'>";
  s += "<p><label>MQTT Enabled</label><br><select name='mqtt_enabled'>";
  s += "<option value='1'" + String(cfg.mqtt_enabled ? " selected" : "") + ">Yes</option>";
  s += "<option value='0'" + String(!cfg.mqtt_enabled ? " selected" : "") + ">No</option>";
  s += "</select></p>";
  s += "<p><label>Broker Host</label><br><input name='mqtt_host' value='" + cfg.mqtt_host + "'></p>";
  s += "<p><label>Broker Port</label><br><input name='mqtt_port' value='" + String(cfg.mqtt_port) + "'></p>";
  s += "<p><label>Username (optional)</label><br><input name='mqtt_user' value='" + cfg.mqtt_user + "'></p>";
  s += "<p><label>Password (optional)</label><br><input name='mqtt_pass' value='" + cfg.mqtt_pass + "'></p>";
  s += "<p><label>Topic RX (subscribe)</label><br><input name='topic_rx' value='" + cfg.topic_rx + "'></p>";
  s += "<p><label>Topic TX (publish)</label><br><input name='topic_tx' value='" + cfg.topic_tx + "'></p>";
  s += "<p><label>Topic Status (retained)</label><br><input name='topic_status' value='" + cfg.topic_status + "'></p>";
  s += "<button type='submit'>Save MQTT Settings</button>";
  s += "</form></div>";

  s += "<div class='card'><h3>Connection</h3>";
  s += "<p>Connected: <b>" + String(mqtt.connected() ? "yes" : "no") + "</b></p>";
  s += "<form method='POST' action='/mqtt/connect'><button type='submit'>Connect</button></form> ";
  s += "<form method='POST' action='/mqtt/disconnect' style='display:inline'><button type='submit' class='secondary'>Disconnect</button></form>";
  s += "<hr>";
  s += "<form method='POST' action='/mqtt/test'>";
  s += "<p><label>Test payload → publish to TX topic</label><br>";
  s += "<input name='payload' value='{\"hello\":\"from RadioGateway\"}'></p>";
  s += "<button type='submit'>Publish Test</button></form>";
  s += "</div>";

  s += "<div class='card'><h3>Last RX Message (Anzeige)</h3>";
  s += "<p>Topic: <code>" + mqtt_last_topic + "</code></p>";
  s += "<p>Age: <code>" + String(mqtt_last_ms ? (String((millis()-mqtt_last_ms)/1000) + "s") : "n/a") + "</code></p>";
  s += "<pre>" + mqtt_last_payload + "</pre>";
  s += "<div class='muted'>BreachGateway kann später Nachrichten an dieses Device senden. Vorbereitung: Topic RX.</div>";
  s += "</div>";

  s += htmlFooter();
  server.send(200, "text/html", s);
}

static void handleMQTTSave() {
  if (!isAuthOk()) return;

  if (server.hasArg("mqtt_enabled")) cfg.mqtt_enabled = (server.arg("mqtt_enabled") == "1");
  if (server.hasArg("mqtt_host"))    cfg.mqtt_host = server.arg("mqtt_host");
  if (server.hasArg("mqtt_port"))    cfg.mqtt_port = (uint16_t)server.arg("mqtt_port").toInt();
  if (server.hasArg("mqtt_user"))    cfg.mqtt_user = server.arg("mqtt_user");
  if (server.hasArg("mqtt_pass"))    cfg.mqtt_pass = server.arg("mqtt_pass");
  if (server.hasArg("topic_rx"))     cfg.topic_rx = server.arg("topic_rx");
  if (server.hasArg("topic_tx"))     cfg.topic_tx = server.arg("topic_tx");
  if (server.hasArg("topic_status")) cfg.topic_status = server.arg("topic_status");

  saveConfigToSD();
  if (cfg.mqtt_enabled) mqttConnect();

  server.sendHeader("Location", "/mqtt");
  server.send(302, "text/plain", "Redirect");
}

static void handleMQTTConnect() {
  if (!isAuthOk()) return;
  mqttConnect();
  server.sendHeader("Location", "/mqtt");
  server.send(302, "text/plain", "Redirect");
}

static void handleMQTTDisconnect() {
  if (!isAuthOk()) return;
  mqtt.disconnect();
  server.sendHeader("Location", "/mqtt");
  server.send(302, "text/plain", "Redirect");
}

static void handleMQTTTest() {
  if (!isAuthOk()) return;
  String payload = server.hasArg("payload") ? server.arg("payload") : "{\"test\":true}";
  if (mqtt.connected()) mqtt.publish(cfg.topic_tx.c_str(), payload.c_str(), false);
  server.sendHeader("Location", "/mqtt");
  server.send(302, "text/plain", "Redirect");
}

static void handleDevices() {
  if (!isAuthOk()) return;

  String s = htmlHeader("RadioGateway – Devices");
  s += "<div class='card'><h3>AP Connected Clients</h3>";

  String j = apClientsJson();
  StaticJsonDocument<768> doc;
  DeserializationError err = deserializeJson(doc, j);

  if (!err) {
    JsonArray arr = doc["clients"].as<JsonArray>();
    s += "<table><tr><th>#</th><th>MAC</th></tr>";
    int idx = 1;
    for (JsonObject o : arr) {
      s += "<tr><td>" + String(idx++) + "</td><td><code>" + String((const char*)o["mac"]) + "</code></td></tr>";
    }
    s += "</table>";
    if (arr.size() == 0) s += "<p class='muted'>No clients connected.</p>";
  } else {
    s += "<pre>" + j + "</pre>";
  }

  s += "<hr><div class='muted'>MAC-Liste ist robust (ESP32 Core 3.x). IP pro Client liefern viele Builds nicht mehr stabil.</div>";
  s += "</div>";

  s += "<div class='card'><h3>Network Interfaces</h3><table>";
  s += "<tr><th>AP IP</th><td><code>" + ipToString(WiFi.softAPIP()) + "</code></td></tr>";
  s += "<tr><th>STA IP</th><td><code>" + String(WiFi.status()==WL_CONNECTED? ipToString(WiFi.localIP()): "disconnected") + "</code></td></tr>";
  s += "<tr><th>MAC</th><td><code>" + WiFi.macAddress() + "</code></td></tr>";
  s += "</table></div>";

  s += htmlFooter();
  server.send(200, "text/html", s);
}

static void handleSD() {
  if (!isAuthOk()) return;

  String s = htmlHeader("RadioGateway – SD");
  s += "<div class='card'><h3>SD Status</h3>";
  bool ok = sdOk();
  s += "<p>Mounted: <b>" + String(ok ? "yes" : "no") + "</b> (CS pin: <code>" + String(SD_CS_PIN) + "</code>)</p>";
  s += "<p class='muted'>Wenn SD nicht mountet: SD_CS_PIN anpassen (oft 5 oder 13). Bei 64GB ist oft exFAT → FAT32 nötig.</p>";
  s += "</div>";

  s += "<div class='card'><h3>Upload File</h3>";
  s += "<form method='POST' action='/sd/upload' enctype='multipart/form-data'>";
  s += "<input type='file' name='upfile'> ";
  s += "<button type='submit'>Upload</button>";
  s += "</form>";
  s += "</div>";

  s += listDirHtml("/");
  s += htmlFooter();
  server.send(200, "text/html", s);
}

static void handleSDDownload() {
  if (!isAuthOk()) return;
  if (!server.hasArg("f")) { server.send(400, "text/plain", "Missing f"); return; }
  String fn = server.arg("f");

  if (!sdOk()) { server.send(500, "text/plain", "SD not mounted"); return; }
  if (!SD.exists(fn.c_str())) { server.send(404, "text/plain", "Not found"); return; }

  File f = SD.open(fn.c_str(), FILE_READ);
  if (!f) { server.send(500, "text/plain", "Open failed"); return; }
  server.streamFile(f, mimeFromName(fn));
  f.close();
}

static void handleSDDelete() {
  if (!isAuthOk()) return;
  if (!server.hasArg("f")) { server.send(400, "text/plain", "Missing f"); return; }
  String fn = server.arg("f");

  if (!sdOk()) { server.send(500, "text/plain", "SD not mounted"); return; }
  if (SD.exists(fn.c_str())) SD.remove(fn.c_str());

  server.sendHeader("Location", "/sd");
  server.send(302, "text/plain", "Redirect");
}

static void handleSDUpload() {
  if (!isAuthOk()) return;

  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    if (!sdOk()) { server.send(500, "text/plain", "SD not mounted"); return; }
    String filename = up.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    uploadFile = SD.open(filename.c_str(), FILE_WRITE);
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) uploadFile.write(up.buf, up.currentSize);
  } else if (up.status == UPLOAD_FILE_END) {
    if (uploadFile) { uploadFile.flush(); uploadFile.close(); }
    server.sendHeader("Location", "/sd");
    server.send(302, "text/plain", "Redirect");
  }
}

static void handleConfig() {
  if (!isAuthOk()) return;

  String s = htmlHeader("RadioGateway – Config");
  s += "<div class='card'><h3>Config File</h3>";
  s += "<p>Path: <code>" + String(CFG_PATH) + "</code></p>";
  s += "<form method='POST' action='/config/save'><button type='submit'>Save current config to SD</button></form>";
  s += "<form method='POST' action='/config/reload' style='margin-top:10px'><button type='submit' class='secondary'>Reload config from SD</button></form>";
  s += "</div>";

  s += "<div class='card'><h3>Current JSON (preview)</h3><pre>";
  StaticJsonDocument<1400> doc;
  doc["ap_enabled"] = cfg.ap_enabled;
  doc["ap_ssid"] = cfg.ap_ssid;
  doc["ap_pass"] = cfg.ap_pass;
  doc["ap_channel"] = cfg.ap_channel;
  doc["sta_enabled"] = cfg.sta_enabled;
  doc["sta_ssid"] = cfg.sta_ssid;
  doc["sta_pass"] = cfg.sta_pass;
  doc["mqtt_enabled"] = cfg.mqtt_enabled;
  doc["mqtt_host"] = cfg.mqtt_host;
  doc["mqtt_port"] = cfg.mqtt_port;
  doc["mqtt_user"] = cfg.mqtt_user;
  doc["mqtt_pass"] = cfg.mqtt_pass;
  doc["topic_rx"] = cfg.topic_rx;
  doc["topic_tx"] = cfg.topic_tx;
  doc["topic_status"] = cfg.topic_status;
  doc["device"] = cfg.device;
  doc["hostname"] = cfg.hostname;
  doc["location"] = cfg.location;
  #if TFT_ENABLED
    doc["bl_percent"] = cfg.bl_percent;
  #endif
  String out;
  serializeJsonPretty(doc, out);
  s += out;
  s += "</pre></div>";

  s += htmlFooter();
  server.send(200, "text/html", s);
}

static void handleConfigSave() {
  if (!isAuthOk()) return;
  saveConfigToSD();
  server.sendHeader("Location", "/config");
  server.send(302, "text/plain", "Redirect");
}

static void handleConfigReload() {
  if (!isAuthOk()) return;

  loadConfigFromSD();
  WiFi.setHostname(cfg.hostname.c_str());

  if (cfg.ap_enabled) startAP(); else stopAP();
  if (cfg.sta_enabled) startSTA(); else stopSTA();
  if (cfg.mqtt_enabled) mqttConnect();

  #if TFT_ENABLED
  if (!tftReady) tftInit();
    blWritePercent(cfg.bl_percent);
    tftDrawBoot("Config reloaded", cfg.hostname.c_str());
  #endif

  server.sendHeader("Location", "/config");
  server.send(302, "text/plain", "Redirect");
}

static void handleRestartWiFi() {
  if (!isAuthOk()) return;

  if (cfg.ap_enabled) startAP(); else stopAP();
  if (cfg.sta_enabled) startSTA(); else stopSTA();

  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "Redirect");
}

static void handleReboot() {
  if (!isAuthOk()) return;
  server.send(200, "text/plain", "Rebooting...");
  delay(250);
  ESP.restart();
}

#if TFT_ENABLED
  static void handleTFT() {
    if (!isAuthOk()) return;

    String s = htmlHeader("RadioGateway – Display");
    s += "<div class='card'><h3>Backlight</h3>";
    s += "<form method='POST' action='/tft/save'>";
    s += "<p><label>Brightness % (0..100)</label><br>";
    s += "<input name='bl' value='" + String(cfg.bl_percent) + "'></p>";
    s += "<button type='submit'>Apply</button>";
    s += "</form>";
    s += "<p class='muted'>Pin: GPIO <code>" + String(TFT_BL_PIN) + "</code></p>";
    s += "</div>";
    s += htmlFooter();
    server.send(200, "text/html", s);
  }

  static void handleTFTSave() {
    if (!isAuthOk()) return;

    if (server.hasArg("bl")) {
      int v = server.arg("bl").toInt();
      if (v < 0) v = 0;
      if (v > 100) v = 100;
      cfg.bl_percent = (uint8_t)v;
      blWritePercent(cfg.bl_percent);
      saveConfigToSD();
    }

    server.sendHeader("Location", "/tft");
    server.send(302, "text/plain", "Redirect");
  }
#endif

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  #if TFT_ENABLED
  if (!tftReady) tftInit();
  blWritePercent(cfg.bl_percent);
  tftDrawBoot("Booting...", "Init");
#endif

  // SD mount attempt + config load
  bool sdMounted = sdBegin();
  if (sdMounted) {
    loadConfigFromSD();
  }

  #if TFT_ENABLED
    // falls config geladen wurde
    blWritePercent(cfg.bl_percent);
  #endif

  WiFi.setHostname(cfg.hostname.c_str());

  WiFi.mode(WIFI_MODE_APSTA);
  if (cfg.ap_enabled) startAP();
  if (cfg.sta_enabled) startSTA();

  if (MDNS.begin(cfg.hostname.c_str())) {
    MDNS.addService("http", "tcp", 80);
  }

  configTzTime(TZ_INFO, NTP1, NTP2);

  if (cfg.mqtt_enabled) mqttConnect();

  // Web routes
  server.on("/", handleHome);

  server.on("/wifi", handleWiFi);
  server.on("/wifi/save", HTTP_POST, handleWiFiSave);

  server.on("/mqtt", handleMQTT);
  server.on("/mqtt/save", HTTP_POST, handleMQTTSave);
  server.on("/mqtt/connect", HTTP_POST, handleMQTTConnect);
  server.on("/mqtt/disconnect", HTTP_POST, handleMQTTDisconnect);
  server.on("/mqtt/test", HTTP_POST, handleMQTTTest);

  server.on("/devices", handleDevices);

  server.on("/sd", handleSD);
  server.on("/sd/download", handleSDDownload);
  server.on("/sd/delete", handleSDDelete);
  server.on("/sd/upload", HTTP_POST, [](){}, handleSDUpload);

  server.on("/config", handleConfig);
  server.on("/config/save", HTTP_POST, handleConfigSave);
  server.on("/config/reload", HTTP_POST, handleConfigReload);

  server.on("/action/restartwifi", HTTP_POST, handleRestartWiFi);
  server.on("/action/reboot", HTTP_POST, handleReboot);

  // JSON endpoint for clients
  server.on("/api/clients", [](){
    if (!isAuthOk()) return;
    server.send(200, "application/json", apClientsJson());
  });

  #if TFT_ENABLED
    server.on("/tft", handleTFT);
    server.on("/tft/save", HTTP_POST, handleTFTSave);
  #endif

  server.begin();

  Serial.println();
  Serial.println("RadioGateway started");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("STA IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("STA not connected");
  }

  #if TFT_ENABLED
    String apLine = "AP " + ipToString(WiFi.softAPIP());
    tftDrawBoot("Started", apLine.c_str());
  #endif
}

// ===================== LOOP =====================
void loop() {
  server.handleClient();
  mqttLoop();

  #if TFT_ENABLED
    if (millis() - tft_last_draw_ms > 2000) {
      tft_last_draw_ms = millis();
      const bool wifiUp = (WiFi.status() == WL_CONNECTED);
      const bool mqttOk = mqtt.connected();
      const bool sdM = sdOk();
      const String ipShown = wifiUp ? ipToString(WiFi.localIP()) : ipToString(WiFi.softAPIP());
      const int16_t rssi = wifiUp ? WiFi.RSSI() : -127;
      const char* note = mqttOk ? "MQTT OK" : (wifiUp ? "WiFi OK" : "SoftAP");
      tftDrawStatus(cfg.hostname.c_str(), wifiUp, ipShown, stats.rx_packets, rssi, note);
    }
  #endif

  delay(2);
}
