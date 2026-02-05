/*
  RadioGateway (ESP32 Arduino Core 3.x / 3.3.6)
  Ziel: stabiler AP/STA + WebUI + MQTT + SD-Config + Display (LovyanGFX) + Backlight PWM

  WICHTIG (LovyanGFX):
  - LovyanGFX braucht eine passende Pin-/Driver-Config in der Library:
    Arduino IDE -> Bibliotheken -> LovyanGFX -> User_Setup.h (oder User_Setups/.. + User_Setup_Select.h)
  - In diesem Sketch werden KEINE Arduino_GFX Includes mehr verwendet.

  Deine Hauptprobleme:
  1) Display: läuft jetzt über LovyanGFX (kein Arduino_GFX, kein LovyanGFX)
  2) SD: robustere Init-Logik + SPI Pins/CS sauber konfigurierbar + Diagnose

  ------------------------------------------------------------
  1) PINNING – HIER zuerst prüfen/anpassen
  ------------------------------------------------------------

  A) Display (LovyanGFX):
     -> Pins/Driver sind im Sketch definiert (siehe RG_TFT_*_PIN).
     -> Dieser Sketch nutzt LovyanGFX nur als "Frontend".

  B) Backlight (separat):
     Du hattest: "GPIO2 -> Backlight geht an"
     ABER: bei vielen Boards ist GPIO2 auch DC.
     => Setz RG_TFT_BL_PIN so, wie dein Test es gezeigt hat.

  C) SD:
     Viele ESP32 TFT-Boards nutzen SD über SPI:
       SCK=18, MISO=19, MOSI=23, CS=5  (häufig)
     Manche nutzen andere CS (z.B. 13) – du testest gerade.
     => Setz RG_SD_CS_PIN passend und ggf. RG_SD_SCK/MISO/MOSI

  ------------------------------------------------------------
  Dependencies (Library Manager / Board-Paket):
  - ESP32 board package (Arduino-ESP32) >= 3.3.6
  - PubSubClient
  - ArduinoJson
  - LovyanGFX
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

#if RG_TFT_ENABLED
  #include <LovyanGFX.hpp>
#endif

// ===================== BUILD FLAGS =====================
#define RG_TFT_ENABLED 1

// ===================== USER SETTINGS (AUTH) =====================
static const char* RG_USER = "yanis";
#define RG_PASSWORD "changeme"   // WebUI Login (BasicAuth)

// ===================== DEVICE META =====================
static const char* RG_HOSTNAME_DEFAULT = "RadioGateway";
static const char* RG_DEVICE_DEFAULT   = "RadioGateway";
static const char* RG_LOCATION_DEFAULT = "Portable";

// ===================== TIMEZONE Zurich =====================
static const char* TZ_INFO = "CET-1CEST,M3.5.0/2,M10.5.0/3"; // Europe/Zurich
static const char* NTP1 = "pool.ntp.org";
static const char* NTP2 = "time.cloudflare.com";

// ===================== AP DEFAULT =====================
static const char* AP_SSID_DEFAULT = "YourDataMyData";
static const char* AP_PASS_DEFAULT = "KiCinference$94RC";
static const uint8_t AP_CHANNEL_DEFAULT = 6;

// AP IP
IPAddress apIP(192,168,4,1);
IPAddress apGW(192,168,4,1);
IPAddress apMASK(255,255,255,0);

// ===================== MQTT DEFAULT =====================
static const char* MQTT_HOST_DEFAULT = "mqtt.ameseder.contact";
static const uint16_t MQTT_PORT_DEFAULT = 1883;
static const char* MQTT_TOPIC_RX_DEFAULT     = "home/radiogateway/rx";
static const char* MQTT_TOPIC_TX_DEFAULT     = "home/radiogateway/tx";
static const char* MQTT_TOPIC_STATUS_DEFAULT = "home/radiogateway/status";

// ===================== SD / CONFIG =====================
static const char* CFG_DIR  = "/radiogateway";
static const char* CFG_PATH = "/radiogateway/config.json";

// --------------------- SD PINNING (robust) ---------------------
// Wenn deine SD Karte nicht mountet: als erstes CS testen.
#ifndef RG_SD_CS_PIN
  #define RG_SD_CS_PIN 5   // du testest; häufig ist 5. (Setz hier auf 5 oder 13 wenn dein Board so ist)
#endif

// Optional: eigene SPI Pins für SD (falls nicht Standard-VSPI)
#ifndef RG_SD_SCK_PIN
  #define RG_SD_SCK_PIN 18
#endif
#ifndef RG_SD_MISO_PIN
  #define RG_SD_MISO_PIN 19
#endif
#ifndef RG_SD_MOSI_PIN
  #define RG_SD_MOSI_PIN 23
#endif

// SD SPI Takt (robuster Start eher niedriger, dann hoch)
static const uint32_t RG_SD_SPI_HZ = 8000000;   // 8 MHz (stabil für viele Module)
static const uint32_t RG_SD_SPI_HZ_FALLBACK = 4000000; // 4 MHz fallback

// ===================== TFT / BACKLIGHT =====================
#if RG_TFT_ENABLED
  // Display-Pins (LovyanGFX / SPI) – basierend auf deinem funktionierenden Pinmap-Sketch
  #ifndef RG_TFT_MOSI_PIN
    #define RG_TFT_MOSI_PIN 13
  #endif
  #ifndef RG_TFT_MISO_PIN
    #define RG_TFT_MISO_PIN 12   // falls nicht verdrahtet: -1
  #endif
  #ifndef RG_TFT_SCLK_PIN
    #define RG_TFT_SCLK_PIN 14
  #endif
  #ifndef RG_TFT_CS_PIN
    #define RG_TFT_CS_PIN   15
  #endif
  #ifndef RG_TFT_DC_PIN
    #define RG_TFT_DC_PIN   2
  #endif
  #ifndef RG_TFT_RST_PIN
    #define RG_TFT_RST_PIN  -1   // oft fest an 3V3 => -1
  #endif
  #ifndef RG_TFT_BL_PIN
    #define RG_TFT_BL_PIN   21   // Backlight
  #endif
  #ifndef RG_TFT_ROT
    #define RG_TFT_ROT      1    // 0..3 (1 = landscape, wie in deinem Test)
  #endif

  // Panel-Typ (bei deinem Test: ILI9341)
  #ifndef RG_TFT_PANEL_ILI9341
    #define RG_TFT_PANEL_ILI9341 1
  #endif

  // SPI Frequenzen
  static const uint32_t RG_TFT_SPI_FREQ_WRITE = 40000000; // 40 MHz
  static const uint32_t RG_TFT_SPI_FREQ_READ  = 16000000; // 16 MHz

  // Backlight PWM (Core 3.x)
  static const uint32_t BL_PWM_FREQ = 5000;
  static const uint8_t  BL_PWM_RES  = 12; // 0..4095
#endif

// ===================== GLOBALS =====================
WebServer server(80);
WiFiClient netClient;
PubSubClient mqtt(netClient);

struct RGConfig {
  // AP
  bool   ap_enabled = true;
  String ap_ssid = AP_SSID_DEFAULT;
  String ap_pass = AP_PASS_DEFAULT;
  uint8_t ap_channel = AP_CHANNEL_DEFAULT;

  // STA
  bool   sta_enabled = false;
  String sta_ssid = "";
  String sta_pass = "";

  // MQTT
  bool     mqtt_enabled = true;
  String   mqtt_host = MQTT_HOST_DEFAULT;
  uint16_t mqtt_port = MQTT_PORT_DEFAULT;
  String   mqtt_user = "";
  String   mqtt_pass = "";
  String   topic_rx     = MQTT_TOPIC_RX_DEFAULT;
  String   topic_tx     = MQTT_TOPIC_TX_DEFAULT;
  String   topic_status = MQTT_TOPIC_STATUS_DEFAULT;

  // Meta
  String device   = RG_DEVICE_DEFAULT;
  String hostname = RG_HOSTNAME_DEFAULT;
  String location = RG_LOCATION_DEFAULT;

  // UI
  uint8_t bl_percent = 60;
} cfg;

// MQTT runtime info
static String mqtt_last_topic = "";
static String mqtt_last_payload = "";
static unsigned long mqtt_last_ms = 0;
static unsigned long mqtt_last_reconnect_try = 0;

// SD runtime info
static bool sd_mounted = false;
static String sd_diag = "not init";


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
  if (!getLocalTime(&timeinfo, 120)) return "no-time";
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buf);
}

// ===================== HTML =====================
static String htmlHeader(const String& title) {
  String h;
  h += "<!doctype html><html><head><meta charset='utf-8'>";
  h += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  h += "<title>" + title + "</title>";
  h += "<style>";
  h += ":root{--bg:#0b0f17;--card:#121a28;--muted:#8da2c0;--txt:#e8eef9;--pri:#4da3ff;--bad:#ff5d5d;--ok:#36d399}";
  h += "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial;margin:0;background:var(--bg);color:var(--txt)}";
  h += "header{padding:18px 18px 10px;border-bottom:1px solid #1e2a40;background:linear-gradient(180deg,#0c1320,#0b0f17)}";
  h += "h1{margin:0;font-size:18px} .sub{margin-top:6px;color:var(--muted);font-size:13px}";
  h += "main{padding:18px;max-width:1100px;margin:auto}";
  h += "a{color:var(--pri);text-decoration:none} a:hover{text-decoration:underline}";
  h += ".nav{margin-top:10px;display:flex;flex-wrap:wrap;gap:10px}";
  h += ".nav a{background:#0e1626;border:1px solid #1e2a40;padding:8px 10px;border-radius:12px;font-size:13px}";
  h += ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px}";
  h += ".card{background:var(--card);border:1px solid #1e2a40;border-radius:16px;padding:14px}";
  h += ".card h3{margin:0 0 10px;font-size:14px;color:#cfe3ff}";
  h += "table{width:100%;border-collapse:collapse;font-size:13px}";
  h += "td,th{padding:8px;border-bottom:1px solid #1e2a40;text-align:left;vertical-align:top}";
  h += "code,pre{background:#0e1626;border:1px solid #1e2a40;border-radius:12px;padding:10px;display:block;overflow:auto;color:#d7e6ff}";
  h += "input,select{width:100%;box-sizing:border-box;padding:10px;border-radius:12px;border:1px solid #1e2a40;background:#0e1626;color:var(--txt)}";
  h += "button{padding:10px 14px;border:0;border-radius:12px;background:var(--pri);color:#001220;font-weight:700;cursor:pointer}";
  h += "button.secondary{background:#22314d;color:#d7e6ff;border:1px solid #2b3e63}";
  h += ".pill{display:inline-block;padding:2px 8px;border-radius:999px;font-size:12px;border:1px solid #1e2a40}";
  h += ".ok{color:var(--ok)} .bad{color:var(--bad)} .muted{color:var(--muted)}";
  h += "</style></head><body>";
  h += "<header>";
  h += "<h1>" + title + "</h1>";
  h += "<div class='sub'>Device: <b>" + cfg.device + "</b> · Hostname: <b>" + cfg.hostname + "</b> · Location: <b>" + cfg.location + "</b></div>";
  h += "<div class='nav'>";
  h += "<a href='/'>Home</a>";
  h += "<a href='/wifi'>WiFi</a>";
  h += "<a href='/mqtt'>MQTT</a>";
  h += "<a href='/devices'>Devices</a>";
  h += "<a href='/sd'>SD</a>";
  h += "<a href='/config'>Config</a>";
#if RG_TFT_ENABLED
  h += "<a href='/tft'>Display</a>";
#endif
  h += "</div></header><main>";
  return h;
}

static String htmlFooter() {
  return "<div class='sub muted' style='padding:18px 0'>RadioGateway WebUI · Zurich TZ · Port 80</div></main></body></html>";
}

// ===================== TFT UI (LovyanGFX) =====================
#if RG_TFT_ENABLED

// ------------------ SPI HOST robust wählen ------------------
#if defined(SPI3_HOST)
  static constexpr int RG_SPI_HOST_ID = SPI3_HOST;
  static constexpr const char* RG_SPI_HOST_NAME = "SPI3_HOST";
#elif defined(VSPI_HOST)
  static constexpr int RG_SPI_HOST_ID = VSPI_HOST;
  static constexpr const char* RG_SPI_HOST_NAME = "VSPI_HOST";
#elif defined(SPI2_HOST)
  static constexpr int RG_SPI_HOST_ID = SPI2_HOST;
  static constexpr const char* RG_SPI_HOST_NAME = "SPI2_HOST";
#elif defined(HSPI_HOST)
  static constexpr int RG_SPI_HOST_ID = HSPI_HOST;
  static constexpr const char* RG_SPI_HOST_NAME = "HSPI_HOST";
#else
  static constexpr int RG_SPI_HOST_ID = 0;
  static constexpr const char* RG_SPI_HOST_NAME = "SPI_HOST?";
#endif

// ------------------ LGFX Device ------------------
class LGFX : public lgfx::LGFX_Device {
#if RG_TFT_PANEL_ILI9341
  lgfx::Panel_ILI9341 _panel;
#else
  lgfx::Panel_ILI9341 _panel;   // Default fallback
#endif
  lgfx::Bus_SPI _bus;

public:
  LGFX() {
    { // Bus
      auto cfg = _bus.config();
      cfg.spi_host   = static_cast<decltype(cfg.spi_host)>(RG_SPI_HOST_ID);
      cfg.spi_mode   = 0;
      cfg.freq_write = RG_TFT_SPI_FREQ_WRITE;
      cfg.freq_read  = RG_TFT_SPI_FREQ_READ;
      cfg.pin_sclk   = RG_TFT_SCLK_PIN;
      cfg.pin_mosi   = RG_TFT_MOSI_PIN;
      cfg.pin_miso   = RG_TFT_MISO_PIN;
      cfg.pin_dc     = RG_TFT_DC_PIN;
      // optional: cfg.use_lock = true;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    { // Panel
      auto cfg = _panel.config();
      cfg.pin_cs   = RG_TFT_CS_PIN;
      cfg.pin_rst  = RG_TFT_RST_PIN;
      cfg.pin_busy = -1;

      cfg.memory_width  = 240;
      cfg.memory_height = 320;
      cfg.panel_width   = 240;
      cfg.panel_height  = 320;

      cfg.offset_x = 0;
      cfg.offset_y = 0;

      cfg.invert    = false;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;

      _panel.config(cfg);
    }
    setPanel(&_panel);
  }
};

static LGFX tft;
static bool tft_ok = false;
static unsigned long tft_last_draw_ms = 0;

// ------------------ Backlight ------------------
static bool bl_inited = false;
static bool bl_pwm_ok = false;

static void blInit() {
  if (bl_inited) return;
  if (RG_TFT_BL_PIN < 0) { bl_inited = true; return; }

  bl_pwm_ok = ledcAttach((uint8_t)RG_TFT_BL_PIN, BL_PWM_FREQ, BL_PWM_RES);
  if (!bl_pwm_ok) {
    pinMode(RG_TFT_BL_PIN, OUTPUT);
    digitalWrite(RG_TFT_BL_PIN, HIGH);
  }
  bl_inited = true;
}

static void blWritePercent(uint8_t percent) {
  if (percent > 100) percent = 100;
  cfg.bl_percent = percent;

  blInit();
  if (RG_TFT_BL_PIN < 0) return;

  const uint32_t maxDuty = (1UL << BL_PWM_RES) - 1;
  uint32_t duty = (uint32_t)((percent * maxDuty) / 100);

  if (bl_pwm_ok) {
    ledcWrite((uint8_t)RG_TFT_BL_PIN, duty);
  } else {
    digitalWrite(RG_TFT_BL_PIN, (percent > 0) ? HIGH : LOW);
  }
}

// ------------------ UI helpers ------------------
static String tftTrim(const String& s, size_t maxLen) {
  if (s.length() <= maxLen) return s;
  return s.substring(0, maxLen - 1) + "…";
}

static String tftUptime() {
  uint32_t s = millis() / 1000;
  uint32_t h = s / 3600;
  uint32_t m = (s % 3600) / 60;
  uint32_t ss = s % 60;
  char buf[20];
  snprintf(buf, sizeof(buf), "%luh %02lum %02lus", (unsigned long)h, (unsigned long)m, (unsigned long)ss);
  return String(buf);
}

static String tftTimeShort() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10)) return "no-time";
  char buf[16];
  strftime(buf, sizeof(buf), "%H:%M:%S", &timeinfo);
  return String(buf);
}

// ------------------ UI drawing ------------------
static void tftSafeBegin() {
  // Sofortige, sichtbare Ausgabe (damit du nicht im "nichts passiert"-Nebel landest)
  tft.init();
  tft.setRotation(RG_TFT_ROT);

  // Wenn aus Config BL=0 kommt, wirkt das wie "Display tot". Für Debug minimum erzwingen.
  if (cfg.bl_percent < 5) cfg.bl_percent = 60;
  blWritePercent(cfg.bl_percent);

  // Farben (schöne dunkle UI)
  const uint16_t C_BG     = lgfx::color565(11, 15, 23);
  const uint16_t C_CARD   = lgfx::color565(18, 26, 40);
  const uint16_t C_LINE   = lgfx::color565(35, 50, 75);
  const uint16_t C_TEXT   = lgfx::color565(232, 238, 249);
  const uint16_t C_MUTED  = lgfx::color565(141, 162, 192);
  const uint16_t C_BLUE   = lgfx::color565(77, 163, 255);
  const uint16_t C_GREEN  = lgfx::color565(54, 211, 153);
  const uint16_t C_RED    = lgfx::color565(255, 93, 93);
  const uint16_t C_YELLOW = lgfx::color565(255, 210, 77);

  tft.fillScreen(C_BG);
  tft.setTextSize(2);
  tft.setTextColor(C_TEXT, C_BG);
  tft.setCursor(12, 10);
  tft.print("RadioGateway");
  tft.setTextSize(1);
  tft.setTextColor(C_MUTED, C_BG);
  tft.setCursor(12, 34);
  tft.print("Display init OK · ");
  tft.print(RG_SPI_HOST_NAME);

  // kleine Progress-Bar, damit du sofort siehst, dass SPI wirklich schiebt
  tft.fillRoundRect(12, 54, tft.width() - 24, 10, 4, C_CARD);
  tft.fillRoundRect(12, 54, (tft.width() - 24) / 2, 10, 4, C_BLUE);

  tft_ok = true;
}

static void tftDrawCard(int x, int y, int w, int h,
                        const String& title, const String& value,
                        uint16_t accent, bool okState) {
  const uint16_t C_BG     = lgfx::color565(11, 15, 23);
  const uint16_t C_CARD   = lgfx::color565(18, 26, 40);
  const uint16_t C_LINE   = lgfx::color565(35, 50, 75);
  const uint16_t C_TEXT   = lgfx::color565(232, 238, 249);
  const uint16_t C_MUTED  = lgfx::color565(141, 162, 192);
  const uint16_t C_OK     = lgfx::color565(54, 211, 153);
  const uint16_t C_BAD    = lgfx::color565(255, 93, 93);

  tft.fillRoundRect(x, y, w, h, 10, C_CARD);
  tft.drawRoundRect(x, y, w, h, 10, C_LINE);

  // Accent bar
  tft.fillRoundRect(x, y, w, 6, 10, accent);

  // Title
  tft.setTextSize(1);
  tft.setTextColor(C_MUTED, C_CARD);
  tft.setCursor(x + 10, y + 12);
  tft.print(title);

  // Status dot
  uint16_t dot = okState ? C_OK : C_BAD;
  tft.fillCircle(x + w - 14, y + 15, 5, dot);

  // Value
  tft.setTextSize(1);
  tft.setTextColor(C_TEXT, C_CARD);
  tft.setCursor(x + 10, y + 30);

  // simple wrap (2 lines max)
  String v = value;
  if (v.length() > 44) v = tftTrim(v, 44);

  int mid = v.length();
  if (v.length() > 22) {
    mid = 22;
    // cut at nearest space
    int sp = v.lastIndexOf(' ', 22);
    if (sp > 10) mid = sp;
  }
  String l1 = v.substring(0, mid);
  String l2 = (mid < (int)v.length()) ? v.substring(mid + 1) : "";

  tft.print(l1);
  if (l2.length()) {
    tft.setCursor(x + 10, y + 44);
    tft.print(tftTrim(l2, 22));
  }
}

static void tftDrawHeaderBar() {
  const uint16_t C_BG     = lgfx::color565(11, 15, 23);
  const uint16_t C_TOP    = lgfx::color565(12, 19, 32);
  const uint16_t C_LINE   = lgfx::color565(35, 50, 75);
  const uint16_t C_TEXT   = lgfx::color565(232, 238, 249);
  const uint16_t C_MUTED  = lgfx::color565(141, 162, 192);

  tft.fillRect(0, 0, tft.width(), 28, C_TOP);
  tft.drawFastHLine(0, 28, tft.width(), C_LINE);

  tft.setTextSize(1);
  tft.setTextColor(C_TEXT, C_TOP);
  tft.setCursor(10, 8);
  tft.print(cfg.hostname);

  tft.setTextColor(C_MUTED, C_TOP);
  String right = tftTimeShort() + "  BL " + String(cfg.bl_percent) + "%";
  int16_t tw = tft.textWidth(right.c_str());
  tft.setCursor(tft.width() - tw - 10, 8);
  tft.print(right);
}

static void tftDrawStatusScreen() {
  if (!tft_ok) return;

  // Palette
  const uint16_t C_BG     = lgfx::color565(11, 15, 23);
  const uint16_t C_BLUE   = lgfx::color565(77, 163, 255);
  const uint16_t C_GREEN  = lgfx::color565(54, 211, 153);
  const uint16_t C_RED    = lgfx::color565(255, 93, 93);
  const uint16_t C_YELLOW = lgfx::color565(255, 210, 77);
  const uint16_t C_MUTED  = lgfx::color565(141, 162, 192);

  tft.fillScreen(C_BG);
  tftDrawHeaderBar();

  // Build values
  String ap = cfg.ap_enabled ? (cfg.ap_ssid + "  " + ipToString(WiFi.softAPIP())) : String("disabled");
  bool staConn = (WiFi.status() == WL_CONNECTED);
  String sta = staConn
    ? (WiFi.SSID() + "  " + ipToString(WiFi.localIP()) + "  " + String(WiFi.RSSI()) + "dBm")
    : String("disconnected");

  bool mqttConn = (cfg.mqtt_enabled && mqtt.connected());
  String mq = mqttConn ? (cfg.mqtt_host + ":" + String(cfg.mqtt_port)) : String("disconnected");

  String sd = sd_mounted ? ("mounted  " + tftTrim(sd_diag, 18)) : ("not mounted  " + tftTrim(sd_diag, 18));

  // Layout for 320x240 (landscape)
  int W = tft.width();
  int H = tft.height();

  int margin = 10;
  int gap = 10;

  int cardW = (W - margin*2 - gap) / 2;
  int cardH = 62;

  int y0 = 40;

  tftDrawCard(margin, y0, cardW, cardH, "AP", ap, C_BLUE, cfg.ap_enabled);
  tftDrawCard(margin + cardW + gap, y0, cardW, cardH, "STA", sta, staConn ? C_GREEN : C_RED, staConn);

  y0 += cardH + gap;
  tftDrawCard(margin, y0, cardW, cardH, "MQTT", mq, mqttConn ? C_GREEN : C_RED, mqttConn);
  tftDrawCard(margin + cardW + gap, y0, cardW, cardH, "SD", sd, sd_mounted ? C_GREEN : C_YELLOW, sd_mounted);

  // Last message (full width)
  y0 += cardH + gap;
  int fullH = H - y0 - 30;
  if (fullH < 48) fullH = 48;

  String topic = mqtt_last_topic.length() ? mqtt_last_topic : "(none)";
  String payload = mqtt_last_payload;
  payload.replace("\r", ""); payload.replace("\n", " ");
  payload = tftTrim(payload, 80);

  String last = topic;
  if (payload.length()) last += " · " + payload;

  bool hasLast = mqtt_last_ms > 0;
  String age = hasLast ? (String((millis() - mqtt_last_ms)/1000) + "s ago") : String("no data");

  tftDrawCard(margin, y0, W - margin*2, fullH, "LAST", last + "  (" + age + ")", C_BLUE, hasLast);

  // Footer
  tft.setTextSize(1);
  tft.setTextColor(C_MUTED, C_BG);
  tft.setCursor(margin, H - 18);
  tft.print("Uptime ");
  tft.print(tftUptime());
  tft.print("  Heap ");
  tft.print(ESP.getFreeHeap());
}

#endif // RG_TFT_ENABLED

// ===================== WIFI =====================
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
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 9000) {
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

    StaticJsonDocument<320> doc;
    doc["device"]   = cfg.device;
    doc["hostname"] = cfg.hostname;
    doc["location"] = cfg.location;
    doc["time"]     = nowString();
    doc["ap_ip"]    = ipToString(WiFi.softAPIP());
    doc["sta_ip"]   = (WiFi.status() == WL_CONNECTED) ? ipToString(WiFi.localIP()) : "disconnected";
    doc["sd"]       = sd_mounted ? "mounted" : "not mounted";

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

// ===================== AP CLIENTS (MAC-only, robust) =====================
static String apClientsJson() {
  StaticJsonDocument<1024> doc;
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

// ===================== SD UTIL (Web file ops) =====================
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

static String listDirHtml(const char* path) {
  if (!sdMount()) return "<div class='card'><h3>SD</h3><div class='bad'>SD not mounted</div></div>";

  File root = SD.open(path);
  if (!root || !root.isDirectory()) return "<div class='card'>Not a directory</div>";

  String out = "<div class='card'><h3>SD Browser: <span class='muted'>" + String(path) + "</span></h3>";
  out += "<table><tr><th>Name</th><th>Size</th><th>Actions</th></tr>";

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

// ===================== API JSON =====================
static void handleApiStatus() {
  if (!isAuthOk()) return;

  StaticJsonDocument<768> doc;
  doc["time"] = nowString();

  doc["ap_enabled"] = cfg.ap_enabled;
  doc["ap_ssid"] = cfg.ap_ssid;
  doc["ap_ip"] = ipToString(WiFi.softAPIP());

  doc["sta_enabled"] = cfg.sta_enabled;
  doc["sta_connected"] = (WiFi.status() == WL_CONNECTED);
  doc["sta_ip"] = (WiFi.status() == WL_CONNECTED) ? ipToString(WiFi.localIP()) : "disconnected";

  doc["mqtt_enabled"] = cfg.mqtt_enabled;
  doc["mqtt_connected"] = mqtt.connected();
  doc["mqtt_host"] = cfg.mqtt_host;
  doc["mqtt_port"] = cfg.mqtt_port;

  doc["sd_mounted"] = sd_mounted;
  doc["sd_diag"] = sd_diag;

  doc["bl_percent"] = cfg.bl_percent;
  doc["tft_bl_pin"] = RG_TFT_BL_PIN;

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

static void handleApiLast() {
  if (!isAuthOk()) return;

  StaticJsonDocument<768> doc;
  doc["topic"] = mqtt_last_topic;
  doc["payload"] = mqtt_last_payload;
  doc["age_s"] = mqtt_last_ms ? (uint32_t)((millis()-mqtt_last_ms)/1000) : 0;

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// ===================== WEB HANDLERS =====================
static void handleHome() {
  if (!isAuthOk()) return;

  String s = htmlHeader("RadioGateway – Home");

  s += "<div class='grid'>";

  // Status card
  s += "<div class='card'><h3>Status</h3>";
  s += "<table>";
  s += "<tr><th>Time</th><td id='st_time' class='muted'>" + nowString() + "</td></tr>";
  s += "<tr><th>AP</th><td><span class='pill ok'>ON</span> <code id='st_apip'>" + ipToString(WiFi.softAPIP()) + "</code><div class='muted'>SSID: <b id='st_apssid'>" + cfg.ap_ssid + "</b></div></td></tr>";
  s += "<tr><th>STA</th><td id='st_sta' class='muted'>" + String(WiFi.status()==WL_CONNECTED ? ipToString(WiFi.localIP()) : "disconnected") + "</td></tr>";
  s += "<tr><th>MQTT</th><td id='st_mqtt'>" + String(mqtt.connected() ? "<span class='ok'>connected</span>" : "<span class='bad'>disconnected</span>") + "</td></tr>";
  s += "<tr><th>SD</th><td id='st_sd'>" + String(sd_mounted ? "<span class='ok'>mounted</span>" : "<span class='bad'>not mounted</span>") +
       "<div class='muted' id='st_sddiag'>" + sd_diag + "</div></td></tr>";
  s += "</table>";
  s += "</div>";

  // Last message
  s += "<div class='card'><h3>Last RX</h3>";
  s += "<div class='muted'>Topic</div><code id='rx_topic'>" + mqtt_last_topic + "</code>";
  s += "<div class='muted' style='margin-top:10px'>Payload</div><pre id='rx_payload'>" + mqtt_last_payload + "</pre>";
  s += "</div>";

  // Actions
  s += "<div class='card'><h3>Quick Actions</h3>";
  s += "<form method='POST' action='/action/restartwifi'><button type='submit'>Restart WiFi</button></form>";
  s += "<div style='height:10px'></div>";
  s += "<form method='POST' action='/action/remountsd'><button type='submit' class='secondary'>Re-Mount SD</button></form>";
  s += "<div style='height:10px'></div>";
  s += "<form method='POST' action='/action/reboot'><button type='submit' class='secondary'>Reboot</button></form>";
  s += "<div class='sub muted' style='margin-top:10px'>AP: <b>http://192.168.4.1/</b> · mDNS: <b>http://" + cfg.hostname + ".local/</b></div>";
  s += "</div>";

  s += "</div>"; // grid

  // Auto refresh
  s += "<script>";
  s += "async function rgRefresh(){";
  s += "try{";
  s += "let st=await fetch('/api/status').then(r=>r.json());";
  s += "document.getElementById('st_time').textContent=st.time;";
  s += "document.getElementById('st_apip').textContent=st.ap_ip;";
  s += "document.getElementById('st_apssid').textContent=st.ap_ssid;";
  s += "document.getElementById('st_sta').textContent=st.sta_connected?st.sta_ip:'disconnected';";
  s += "document.getElementById('st_mqtt').innerHTML=st.mqtt_connected?\"<span class='ok'>connected</span>\":\"<span class='bad'>disconnected</span>\";";
  s += "document.getElementById('st_sd').innerHTML=st.sd_mounted?\"<span class='ok'>mounted</span>\":\"<span class='bad'>not mounted</span>\";";
  s += "document.getElementById('st_sddiag').textContent=st.sd_diag||'';";
  s += "let rx=await fetch('/api/last').then(r=>r.json());";
  s += "document.getElementById('rx_topic').textContent=rx.topic||'';";
  s += "document.getElementById('rx_payload').textContent=rx.payload||'';";
  s += "}catch(e){}";
  s += "}";
  s += "setInterval(rgRefresh,1500); rgRefresh();";
  s += "</script>";

  s += htmlFooter();
  server.send(200, "text/html", s);
}

static void handleWiFi() {
  if (!isAuthOk()) return;

  String s = htmlHeader("RadioGateway – WiFi");

  s += "<div class='grid'>";

  s += "<div class='card'><h3>SoftAP (Hotspot)</h3>";
  s += "<form method='POST' action='/wifi/save'>";
  s += "<label>AP Enabled</label><select name='ap_enabled'>";
  s += "<option value='1'" + String(cfg.ap_enabled ? " selected" : "") + ">Yes</option>";
  s += "<option value='0'" + String(!cfg.ap_enabled ? " selected" : "") + ">No</option>";
  s += "</select><div style='height:10px'></div>";
  s += "<label>SSID</label><input name='ap_ssid' value='" + cfg.ap_ssid + "'><div style='height:10px'></div>";
  s += "<label>Password (>=8 WPA2; leer = open)</label><input name='ap_pass' value='" + cfg.ap_pass + "'><div style='height:10px'></div>";
  s += "<label>Channel</label><input name='ap_channel' value='" + String(cfg.ap_channel) + "'><div style='height:12px'></div>";
  s += "<button type='submit'>Save AP</button>";
  s += "</form></div>";

  s += "<div class='card'><h3>STA (Connect to WLAN)</h3>";
  s += "<form method='POST' action='/wifi/save'>";
  s += "<label>STA Enabled</label><select name='sta_enabled'>";
  s += "<option value='1'" + String(cfg.sta_enabled ? " selected" : "") + ">Yes</option>";
  s += "<option value='0'" + String(!cfg.sta_enabled ? " selected" : "") + ">No</option>";
  s += "</select><div style='height:10px'></div>";
  s += "<label>SSID</label><input name='sta_ssid' value='" + cfg.sta_ssid + "'><div style='height:10px'></div>";
  s += "<label>Password</label><input name='sta_pass' value='" + cfg.sta_pass + "'><div style='height:12px'></div>";
  s += "<button type='submit'>Save STA</button>";
  s += "</form>";
  s += "<div class='sub muted' style='margin-top:10px'>STA IP: <b>" + String(WiFi.status()==WL_CONNECTED?ipToString(WiFi.localIP()):"disconnected") + "</b></div>";
  s += "</div>";

  s += "</div>";

  s += htmlFooter();
  server.send(200, "text/html", s);
}

static void handleWiFiSave() {
  if (!isAuthOk()) return;

  if (server.hasArg("ap_enabled")) cfg.ap_enabled = server.arg("ap_enabled") == "1";
  if (server.hasArg("ap_ssid")) cfg.ap_ssid = server.arg("ap_ssid");
  if (server.hasArg("ap_pass")) cfg.ap_pass = server.arg("ap_pass");
  if (server.hasArg("ap_channel")) cfg.ap_channel = (uint8_t)server.arg("ap_channel").toInt();

  if (server.hasArg("sta_enabled")) cfg.sta_enabled = server.arg("sta_enabled") == "1";
  if (server.hasArg("sta_ssid")) cfg.sta_ssid = server.arg("sta_ssid");
  if (server.hasArg("sta_pass")) cfg.sta_pass = server.arg("sta_pass");

  saveConfigToSD();

  if (cfg.ap_enabled) startAP(); else stopAP();
  if (cfg.sta_enabled) startSTA(); else stopSTA();

  server.sendHeader("Location", "/wifi");
  server.send(302, "text/plain", "Redirect");
}

static void handleMQTT() {
  if (!isAuthOk()) return;

  String s = htmlHeader("RadioGateway – MQTT");

  s += "<div class='grid'>";

  s += "<div class='card'><h3>MQTT Settings</h3>";
  s += "<form method='POST' action='/mqtt/save'>";
  s += "<label>MQTT Enabled</label><select name='mqtt_enabled'>";
  s += "<option value='1'" + String(cfg.mqtt_enabled ? " selected" : "") + ">Yes</option>";
  s += "<option value='0'" + String(!cfg.mqtt_enabled ? " selected" : "") + ">No</option>";
  s += "</select><div style='height:10px'></div>";

  s += "<label>Broker Host</label><input name='mqtt_host' value='" + cfg.mqtt_host + "'><div style='height:10px'></div>";
  s += "<label>Broker Port</label><input name='mqtt_port' value='" + String(cfg.mqtt_port) + "'><div style='height:10px'></div>";
  s += "<label>Username (optional)</label><input name='mqtt_user' value='" + cfg.mqtt_user + "'><div style='height:10px'></div>";
  s += "<label>Password (optional)</label><input name='mqtt_pass' value='" + cfg.mqtt_pass + "'><div style='height:10px'></div>";

  s += "<label>Topic RX (subscribe)</label><input name='topic_rx' value='" + cfg.topic_rx + "'><div style='height:10px'></div>";
  s += "<label>Topic TX (publish)</label><input name='topic_tx' value='" + cfg.topic_tx + "'><div style='height:10px'></div>";
  s += "<label>Topic Status (retained)</label><input name='topic_status' value='" + cfg.topic_status + "'><div style='height:12px'></div>";
  s += "<button type='submit'>Save MQTT</button>";
  s += "</form></div>";

  s += "<div class='card'><h3>Connection</h3>";
  s += "<div>Connected: " + String(mqtt.connected() ? "<span class='ok'>yes</span>" : "<span class='bad'>no</span>") + "</div>";
  s += "<div style='height:10px'></div>";
  s += "<form method='POST' action='/mqtt/connect' style='display:inline-block;margin-right:8px'><button type='submit'>Connect</button></form>";
  s += "<form method='POST' action='/mqtt/disconnect' style='display:inline-block'><button type='submit' class='secondary'>Disconnect</button></form>";
  s += "<hr style='border:0;border-top:1px solid #1e2a40;margin:14px 0'>";
  s += "<form method='POST' action='/mqtt/test'>";
  s += "<label>Test payload → publish to TX</label>";
  s += "<input name='payload' value='{\"hello\":\"from RadioGateway\"}'>";
  s += "<div style='height:12px'></div>";
  s += "<button type='submit'>Publish Test</button>";
  s += "</form></div>";

  s += "<div class='card'><h3>Last RX (live)</h3>";
  s += "<div class='muted'>Topic</div><code id='rx_topic'>" + mqtt_last_topic + "</code>";
  s += "<div class='muted' style='margin-top:10px'>Payload</div><pre id='rx_payload'>" + mqtt_last_payload + "</pre>";
  s += "</div>";

  s += "</div>"; // grid

  s += "<script>setInterval(async()=>{try{let rx=await fetch('/api/last').then(r=>r.json());"
       "document.getElementById('rx_topic').textContent=rx.topic||'';"
       "document.getElementById('rx_payload').textContent=rx.payload||'';"
       "}catch(e){}},1200);</script>";

  s += htmlFooter();
  server.send(200, "text/html", s);
}

static void handleMQTTSave() {
  if (!isAuthOk()) return;

  if (server.hasArg("mqtt_enabled")) cfg.mqtt_enabled = server.arg("mqtt_enabled") == "1";
  if (server.hasArg("mqtt_host")) cfg.mqtt_host = server.arg("mqtt_host");
  if (server.hasArg("mqtt_port")) cfg.mqtt_port = (uint16_t)server.arg("mqtt_port").toInt();
  if (server.hasArg("mqtt_user")) cfg.mqtt_user = server.arg("mqtt_user");
  if (server.hasArg("mqtt_pass")) cfg.mqtt_pass = server.arg("mqtt_pass");
  if (server.hasArg("topic_rx")) cfg.topic_rx = server.arg("topic_rx");
  if (server.hasArg("topic_tx")) cfg.topic_tx = server.arg("topic_tx");
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
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, j);
  if (!err) {
    JsonArray arr = doc["clients"].as<JsonArray>();
    s += "<table><tr><th>#</th><th>MAC</th></tr>";
    int idx = 1;
    for (JsonObject o : arr) {
      s += "<tr><td>" + String(idx++) + "</td><td><code>" + String((const char*)o["mac"]) + "</code></td></tr>";
    }
    s += "</table>";
    if (arr.size() == 0) s += "<div class='muted'>No clients connected.</div>";
  } else {
    s += "<pre>" + j + "</pre>";
  }

  s += "<hr style='border:0;border-top:1px solid #1e2a40;margin:14px 0'>";
  s += "<div class='muted'>MAC-Liste ist robust (ESP32 Core 3.x). IP pro Client ist oft nicht stabil verfügbar.</div>";
  s += "</div>";

  s += "<div class='card'><h3>Interfaces</h3><table>";
  s += "<tr><th>AP IP</th><td><code>" + ipToString(WiFi.softAPIP()) + "</code></td></tr>";
  s += "<tr><th>STA IP</th><td><code>" + String(WiFi.status()==WL_CONNECTED ? ipToString(WiFi.localIP()) : "disconnected") + "</code></td></tr>";
  s += "<tr><th>STA MAC</th><td><code>" + WiFi.macAddress() + "</code></td></tr>";
  s += "</table></div>";

  s += htmlFooter();
  server.send(200, "text/html", s);
}

static void handleSD() {
  if (!isAuthOk()) return;

  String s = htmlHeader("RadioGateway – SD");

  sdMount(); // try mount

  s += "<div class='card'><h3>SD Status</h3>";
  s += "<div>Mounted: " + String(sd_mounted ? "<span class='ok'>yes</span>" : "<span class='bad'>no</span>") + "</div>";
  s += "<div class='muted'>CS pin: <b>" + String(RG_SD_CS_PIN) + "</b> · SPI pins: SCK=" + String(RG_SD_SCK_PIN) +
       " MISO=" + String(RG_SD_MISO_PIN) + " MOSI=" + String(RG_SD_MOSI_PIN) + "</div>";
  s += "<div class='muted'>Diag: <b>" + sd_diag + "</b></div>";
  s += "<div style='height:10px'></div>";
  s += "<form method='POST' action='/action/remountsd'><button type='submit'>Re-Mount SD</button></form>";
  s += "<div class='muted' style='margin-top:10px'>Hinweis: 64GB FAT32 MBR ist ok. Wenn Mount failt: zuerst CS wechseln (oft 5), dann SPI Pins.</div>";
  s += "</div>";

  s += "<div class='card'><h3>Upload File</h3>";
  s += "<form method='POST' action='/sd/upload' enctype='multipart/form-data'>";
  s += "<input type='file' name='upfile'> <div style='height:10px'></div>";
  s += "<button type='submit'>Upload</button>";
  s += "</form></div>";

  s += listDirHtml("/");

  s += htmlFooter();
  server.send(200, "text/html", s);
}

static File uploadFile;

static void handleSDDownload() {
  if (!isAuthOk()) return;
  if (!server.hasArg("f")) { server.send(400, "text/plain", "Missing f"); return; }

  String fn = server.arg("f");
  if (!sdMount()) { server.send(500, "text/plain", "SD not mounted"); return; }
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
  if (!sdMount()) { server.send(500, "text/plain", "SD not mounted"); return; }
  if (SD.exists(fn.c_str())) SD.remove(fn.c_str());

  server.sendHeader("Location", "/sd");
  server.send(302, "text/plain", "Redirect");
}

static void handleSDUpload() {
  if (!isAuthOk()) return;

  HTTPUpload& up = server.upload();

  if (up.status == UPLOAD_FILE_START) {
    if (!sdMount()) { server.send(500, "text/plain", "SD not mounted"); return; }
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
  s += "<div class='muted'>Path: <code>" + String(CFG_PATH) + "</code></div>";
  s += "<div style='height:10px'></div>";
  s += "<form method='POST' action='/config/save' style='display:inline-block;margin-right:10px'><button type='submit'>Save config to SD</button></form>";
  s += "<form method='POST' action='/config/reload' style='display:inline-block'><button type='submit' class='secondary'>Reload config from SD</button></form>";
  s += "</div>";

  StaticJsonDocument<1600> doc;
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
  doc["bl_percent"] = cfg.bl_percent;

  String out;
  serializeJsonPretty(doc, out);

  s += "<div class='card'><h3>Current JSON (preview)</h3><pre>" + out + "</pre></div>";

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

#if RG_TFT_ENABLED
  blWritePercent(cfg.bl_percent);
  tftDrawStatusScreen();
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

static void handleReMountSD() {
  if (!isAuthOk()) return;
  sd_mounted = false;
  sdMount();
  server.sendHeader("Location", "/sd");
  server.send(302, "text/plain", "Redirect");
}

static void handleReboot() {
  if (!isAuthOk()) return;
  server.send(200, "text/plain", "Rebooting...");
  delay(250);
  ESP.restart();
}

#if RG_TFT_ENABLED
static void handleTFT() {
  if (!isAuthOk()) return;

  String s = htmlHeader("RadioGateway – Display");

  s += "<div class='card'><h3>Backlight</h3>";
  s += "<form method='POST' action='/tft/save'>";
  s += "<label>Brightness % (0..100)</label>";
  s += "<input name='bl' value='" + String(cfg.bl_percent) + "'>";
  s += "<div style='height:12px'></div>";
  s += "<button type='submit'>Apply</button>";
  s += "</form>";
  s += "<div class='sub muted' style='margin-top:10px'>BL pin: <b>GPIO " + String(RG_TFT_BL_PIN) + "</b> (falls falsch: RG_TFT_BL_PIN oben ändern)</div>";
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
    tftDrawStatusScreen();
  }

  server.sendHeader("Location", "/tft");
  server.send(302, "text/plain", "Redirect");
}
#endif

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(250);

  // Load from SD if possible
  sdMount();
  if (sd_mounted) loadConfigFromSD();

  // Hostname
  WiFi.setHostname(cfg.hostname.c_str());

  // WiFi start
  WiFi.mode(WIFI_MODE_APSTA);
  if (cfg.ap_enabled) startAP();
  if (cfg.sta_enabled) startSTA();

  // mDNS
  if (MDNS.begin(cfg.hostname.c_str())) {
    MDNS.addService("http", "tcp", 80);
  }

  // Time
  configTzTime(TZ_INFO, NTP1, NTP2);

  // MQTT
  if (cfg.mqtt_enabled) mqttConnect();

#if RG_TFT_ENABLED
  // Display init
  tftSafeBegin();
  blWritePercent(cfg.bl_percent);
  tftDrawStatusScreen();
#endif

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
  server.on("/action/remountsd",  HTTP_POST, handleReMountSD);
  server.on("/action/reboot",     HTTP_POST, handleReboot);

  // API
  server.on("/api/status", handleApiStatus);
  server.on("/api/last", handleApiLast);
  server.on("/api/clients", [](){
    if (!isAuthOk()) return;
    server.send(200, "application/json", apClientsJson());
  });

#if RG_TFT_ENABLED
  server.on("/tft", handleTFT);
  server.on("/tft/save", HTTP_POST, handleTFTSave);
#endif

  server.begin();

  Serial.println();
  Serial.println("RadioGateway started");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  Serial.print("STA: "); Serial.println(WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString() : String("disconnected"));
  Serial.print("SD: "); Serial.println(sd_mounted ? "mounted" : "not mounted");
  Serial.print("SD diag: "); Serial.println(sd_diag);

#if RG_TFT_ENABLED
  tftDrawStatusScreen();
#endif
}

// ===================== LOOP =====================
void loop() {
  server.handleClient();
  mqttLoop();

  // SD: gelegentlich Status aktualisieren (nicht dauernd neu mounten)
  static unsigned long sd_last_check = 0;
  if (millis() - sd_last_check > 5000) {
    sd_last_check = millis();
    if (sd_mounted && SD.cardType() == CARD_NONE) {
      sd_mounted = false;
      sd_diag = "card removed?";
    }
  }

#if RG_TFT_ENABLED
  // Display: alle 2s komplett neu (stabil) + jede Sekunde "fast" wenn du später willst
  if (tft_ok && (millis() - tft_last_draw_ms > 2000)) {
    tft_last_draw_ms = millis();
    tftDrawStatusScreen();
  }
#endif

  delay(2);
}
