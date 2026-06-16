/*
  RadioGateway (ESP32 Arduino Core 3.x / 3.3.6)
  Ziel: stabiler AP/STA + WebUI + MQTT + SD-Config + Display (TFT_eSPI) + Backlight PWM

  WICHTIG (TFT_eSPI):
  - TFT_eSPI braucht eine passende Pin-/Driver-Config in der Library:
    Arduino IDE -> Bibliotheken -> TFT_eSPI -> User_Setup.h (oder User_Setups/.. + User_Setup_Select.h)
  - In diesem Sketch werden KEINE Arduino_GFX Includes mehr verwendet.

  Deine Hauptprobleme:
  1) Display: wird jetzt über TFT_eSPI gemacht (kein Arduino_GFX mehr)
  2) SD: robustere Init-Logik + SPI Pins/CS sauber konfigurierbar + Diagnose

  ------------------------------------------------------------
  1) PINNING – HIER zuerst prüfen/anpassen
  ------------------------------------------------------------

  A) Display (TFT_eSPI):
     -> Pins/Driver NICHT hier, sondern in TFT_eSPI User_Setup.h setzen.
     -> Dieser Sketch nutzt TFT_eSPI nur als "Frontend".

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


// ===================== BUILD FLAGS =====================
#define RG_TFT_ENABLED 1

// ===================== USER SETTINGS (AUTH) =====================
static const char* RG_USER = "yanis";
// WebUI Login (BasicAuth). UNBEDINGT ändern! Per build_flag überschreibbar:
//   build_flags = -DRG_PASSWORD='"DeinSicheresPasswort"'
#ifndef RG_PASSWORD
#define RG_PASSWORD "changeme"
#endif

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

// ===================== TFT / DISPLAY (optional, LovyanGFX) =====================
// Board: ESP32-2432S028 / 2.8" ILI9341 + XPT2046 Touch (typisch)
// TFT Pins (aus deinem Pin-Map-Test):
//   SCLK=14, MOSI=13, MISO=12, CS=15, DC=2, RST=NC(-1), BL=21
// SD Pins (separater SPI Bus / default VSPI): SCLK=18, MOSI=23, MISO=19, CS=5
#define TFT_ENABLED 1
#define RG_TFT_ENABLED TFT_ENABLED

#if TFT_ENABLED
  #include <LovyanGFX.hpp>

  // --------- TFT Pinout ---------
  static constexpr int TFT_SCLK_PIN = 14;
  static constexpr int TFT_MOSI_PIN = 13;
  static constexpr int TFT_MISO_PIN = 12;   // falls nicht verdrahtet: -1
  static constexpr int TFT_CS_PIN   = 15;
  static constexpr int TFT_DC_PIN   = 2;
  static constexpr int TFT_RST_PIN  = -1;   // NC
  static constexpr int TFT_BL_PIN   = 21;

  // --------- Touch (XPT2046) Pinout (optional) ---------
  // (wird nur genutzt, wenn Enable unten = 1)
  static constexpr int TP_CLK_PIN = 25;
  static constexpr int TP_CS_PIN  = 33;
  static constexpr int TP_DIN_PIN = 32;
  static constexpr int TP_OUT_PIN = 39;
  static constexpr int TP_IRQ_PIN = 36;

  #ifndef RG_UI_ENABLE_TOUCH
    #define RG_UI_ENABLE_TOUCH 1
  #endif

  // --------- Farbkonstanten (RGB565) ---------
  #ifndef BLACK
    #define BLACK   0x0000
    #define WHITE   0xFFFF
    #define RED     0xF800
    #define GREEN   0x07E0
    #define BLUE    0x001F
    #define YELLOW  0xFFE0
    #define CYAN    0x07FF
    #define MAGENTA 0xF81F
    #define ORANGE  0xFD20
    #define GRAY    0x8410
    #define DARK    0x2104
  #endif

  // --------- SPI HOST robust wählen ---------
  // Wichtig: SD läuft typischerweise auf VSPI/SPI3_HOST (Pins 18/23/19/5).
  // TFT hängt bei dir auf separaten Pins -> wir nehmen SPI2_HOST (HSPI equivalent), wenn vorhanden.
  #if defined(SPI2_HOST)
    static constexpr int TFT_SPI_HOST_ID = SPI2_HOST;
    static constexpr const char* TFT_SPI_HOST_NAME = "SPI2_HOST";
  #elif defined(HSPI_HOST)
    static constexpr int TFT_SPI_HOST_ID = HSPI_HOST;
    static constexpr const char* TFT_SPI_HOST_NAME = "HSPI_HOST";
  #elif defined(SPI3_HOST)
    static constexpr int TFT_SPI_HOST_ID = SPI3_HOST;
    static constexpr const char* TFT_SPI_HOST_NAME = "SPI3_HOST";
  #elif defined(VSPI_HOST)
    static constexpr int TFT_SPI_HOST_ID = VSPI_HOST;
    static constexpr const char* TFT_SPI_HOST_NAME = "VSPI_HOST";
  #else
    static constexpr int TFT_SPI_HOST_ID = 0;
    static constexpr const char* TFT_SPI_HOST_NAME = "SPI_HOST?";
  #endif

  static constexpr uint32_t TFT_SPI_FREQ_WRITE = 40000000;
  static constexpr uint32_t TFT_SPI_FREQ_READ  = 16000000;

  // --------- Backlight (Core 3.x LEDC API kompatibel) ---------
  static uint8_t blPercent = 60;

  static void blInitOnce() {
    static bool inited = false;
    if (inited) return;
    inited = true;

    pinMode(TFT_BL_PIN, OUTPUT);
    digitalWrite(TFT_BL_PIN, HIGH); // default ON

    // Wenn du Dimmung willst: ledcAttach/ledcWrite (Core 3.x) wird in blWritePercent genutzt.
  }

  static void blWritePercent(uint8_t percent) {
    if (percent > 100) percent = 100;
    blPercent = percent;

    // LEDC (Core 3.x): ledcAttach(pin,freq,res) + ledcWrite(pin,duty)
    #if defined(ARDUINO_ESP32_MAJOR) && (ARDUINO_ESP32_MAJOR >= 3)
      const uint32_t freq = 5000;
      const uint8_t  res  = 12; // 0..4095
      const uint32_t maxDuty = (1UL << res) - 1;
      const uint32_t duty = (uint32_t)((percent * maxDuty) / 100);

      bool ok = ledcAttach((uint8_t)TFT_BL_PIN, freq, res);
      if (ok) {
        ledcWrite((uint8_t)TFT_BL_PIN, duty);
      } else {
        digitalWrite(TFT_BL_PIN, (percent > 0) ? HIGH : LOW);
      }
    #else
      digitalWrite(TFT_BL_PIN, (percent > 0) ? HIGH : LOW);
    #endif
  }

  // --------- LovyanGFX Setup (ILI9341) ---------
  class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ILI9341 _panel;
    lgfx::Bus_SPI _bus;

  public:
    LGFX() {
      { // Bus
        auto cfg = _bus.config();
        cfg.spi_host   = static_cast<decltype(cfg.spi_host)>(TFT_SPI_HOST_ID);
        cfg.spi_mode   = 0;
        cfg.freq_write = TFT_SPI_FREQ_WRITE;
        cfg.freq_read  = TFT_SPI_FREQ_READ;
        cfg.pin_sclk   = TFT_SCLK_PIN;
        cfg.pin_mosi   = TFT_MOSI_PIN;
        cfg.pin_miso   = TFT_MISO_PIN;
        cfg.pin_dc     = TFT_DC_PIN;
        cfg.use_lock   = true;
        _bus.config(cfg);
        _panel.setBus(&_bus);
      }
      { // Panel
        auto cfg = _panel.config();
        cfg.pin_cs   = TFT_CS_PIN;
        cfg.pin_rst  = TFT_RST_PIN;
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

  // --------- UI: Seiten / Input ---------
  enum UiPage : uint8_t {
    PAGE_BOOT = 0,
    PAGE_STATUS,
    PAGE_WIFI,
    PAGE_BLE,
    PAGE_SD,
    PAGE_GALLERY,
    PAGE_ABOUT,
    PAGE_COUNT
  };

  static UiPage ui_page = PAGE_BOOT;

  // Boot/Touch-Taste: viele ESP32 Boards = GPIO0 (BOOT), active-low
  #ifndef RG_BOOT_BTN_PIN
    #define RG_BOOT_BTN_PIN 0
  #endif

  static bool bootBtnPressed() {
    static uint32_t lastMs = 0;
    static bool lastState = false;

    bool now = (digitalRead(RG_BOOT_BTN_PIN) == LOW);
    uint32_t ms = millis();
    if (ms - lastMs < 40) return false; // debounce
    lastMs = ms;

    bool pressed = (now && !lastState);
    lastState = now;
    return pressed;
  }

  // --------- Touch minimal (XPT2046 raw SPI, ohne Zusatzlib) ---------
  // Hinweis: Bei vielen Boards ist Touch separat verdrahtet (GPIO25/32/39/33/36).
  // Diese Implementierung liest nur "irgendwas" zuverlässig. Kalibrierung ist als Default gesetzt
  // und kann später feinjustiert werden.
  #if RG_UI_ENABLE_TOUCH
    static bool touchBeginDone = false;
    static SPIClass touchSPI(VSPI); // Touch nutzt "normalen" VSPI Host, aber eigene Pins -> funktioniert
    static constexpr uint32_t TOUCH_SPI_HZ = 2000000;

    // Kalibrierung (typisch) -> ggf. anpassen, wenn Touch invertiert / verschoben ist
    static int16_t touch_x_min = 200,  touch_x_max = 3800;
    static int16_t touch_y_min = 200,  touch_y_max = 3800;

    static void touchBeginOnce() {
      if (touchBeginDone) return;
      touchBeginDone = true;
      pinMode(TP_CS_PIN, OUTPUT);
      digitalWrite(TP_CS_PIN, HIGH);
      pinMode(TP_IRQ_PIN, INPUT);
      touchSPI.begin(TP_CLK_PIN, TP_OUT_PIN, TP_DIN_PIN, TP_CS_PIN);
    }

    static uint16_t touchRead12(uint8_t cmd) {
      touchSPI.beginTransaction(SPISettings(TOUCH_SPI_HZ, MSBFIRST, SPI_MODE0));
      digitalWrite(TP_CS_PIN, LOW);
      touchSPI.transfer(cmd);
      uint16_t hi = touchSPI.transfer(0x00);
      uint16_t lo = touchSPI.transfer(0x00);
      digitalWrite(TP_CS_PIN, HIGH);
      touchSPI.endTransaction();
      uint16_t v = ((hi << 8) | lo) >> 3; // 12-bit
      return v;
    }

    static bool touchGetPoint(int16_t& x, int16_t& y) {
      touchBeginOnce();
      if (digitalRead(TP_IRQ_PIN) == HIGH) return false; // not pressed (IRQ is usually active-low)

      // Mehrfach lesen + median wäre besser; für UI reicht 2x
      uint16_t rawX = touchRead12(0xD0); // X
      uint16_t rawY = touchRead12(0x90); // Y

      if (rawX < 50 || rawY < 50) return false;

      // Map raw -> screen
      x = map((int)rawX, touch_x_min, touch_x_max, 0, (int)tft.width());
      y = map((int)rawY, touch_y_min, touch_y_max, 0, (int)tft.height());

      if (x < 0) x = 0;
      if (y < 0) y = 0;
      if (x >= (int)tft.width())  x = tft.width() - 1;
      if (y >= (int)tft.height()) y = tft.height() - 1;

      return true;
    }
  #endif

  static void uiNextPage() {
    ui_page = static_cast<UiPage>((ui_page + 1) % PAGE_COUNT);
    tft_last_draw_ms = 0; // force redraw
  }

  // --------- UI: kleine Icon-Zeichner (ohne Dateien) ---------
  static void drawWifiIcon(int x, int y, uint16_t c) {
    // einfache "Funkwellen"
    tft.drawCircle(x, y, 2, c);
    tft.drawArc(x, y, 10, 10, 210, 330, c);
    tft.drawArc(x, y, 18, 18, 210, 330, c);
    tft.drawArc(x, y, 26, 26, 210, 330, c);
  }

  static void drawBleIcon(int x, int y, uint16_t c) {
    // stark vereinfachtes Bluetooth-Symbol
    tft.drawLine(x, y-18, x, y+18, c);
    tft.drawLine(x, y-18, x+10, y-8, c);
    tft.drawLine(x+10, y-8, x, y+2, c);
    tft.drawLine(x, y+2, x+10, y+12, c);
    tft.drawLine(x+10, y+12, x, y+18, c);
  }

  static void drawLoRaIcon(int x, int y, uint16_t c) {
    tft.drawCircle(x, y, 3, c);
    tft.drawArc(x, y, 12, 12, 225, 315, c);
    tft.drawArc(x, y, 20, 20, 225, 315, c);
    tft.drawArc(x, y, 28, 28, 225, 315, c);
  }

  static void drawSdIcon(int x, int y, uint16_t c) {
    tft.drawRect(x-10, y-14, 20, 28, c);
    tft.drawLine(x-10, y-10, x-2, y-14, c);
    tft.fillRect(x-7, y-4, 3, 6, c);
    tft.fillRect(x-2, y-4, 3, 6, c);
    tft.fillRect(x+3, y-4, 3, 6, c);
  }

  // --------- UI: Layout Helper ---------
  static void uiHeader(const char* title) {
    tft.fillScreen(DARK);
    tft.fillRect(0, 0, tft.width(), 34, 0x19E7); // dunkles Blau-Grau
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 8);
    tft.print("AMESEDER");
    tft.setTextSize(1);
    tft.setCursor(170, 12);
    tft.print(title);

    // Rahmen
    tft.drawRect(4, 40, tft.width()-8, tft.height()-48, GRAY);
  }

  static void uiBadge(int x, int y, const char* label, uint16_t col) {
    int w = 68;
    int h = 18;
    tft.fillRoundRect(x, y, w, h, 4, col);
    tft.setTextColor(BLACK);
    tft.setTextSize(1);
    tft.setCursor(x+6, y+5);
    tft.print(label);
  }

  // --------- UI: Boot / Loading / Hacked ---------
  static void tftBootSequence() {
    ui_page = PAGE_BOOT;
    tft.fillScreen(BLACK);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.setCursor(12, 20);
    tft.println("RadioGateway");

    // Loading bar + steps
    const char* steps[] = {
      "init display",
      "init wifi",
      "init web",
      "init sd",
      "init mqtt",
      "ready"
    };
    const int stepsN = sizeof(steps)/sizeof(steps[0]);

    int barX=20, barY=120, barW=200, barH=14;
    tft.drawRect(barX, barY, barW, barH, GRAY);

    for (int i=0;i<stepsN;i++){
      tft.fillRect(barX+2, barY+2, (barW-4)*(i+1)/stepsN, barH-4, 0x07FF);
      tft.setTextColor(WHITE, BLACK);
      tft.setTextSize(1);
      tft.fillRect(0, 150, tft.width(), 20, BLACK);
      tft.setCursor(20, 154);
      tft.print("Loading: ");
      tft.print(steps[i]);
      delay(220);
    }

    // Hacked blink 4s
    uint32_t start = millis();
    bool on=false;
    while (millis() - start < 4000) {
      on = !on;
      tft.fillRect(0, 190, tft.width(), 40, BLACK);
      tft.setTextSize(2);
      tft.setCursor(60, 200);
      tft.setTextColor(on ? GREEN : RED, BLACK);
      tft.print(".:Hacked:.");
      delay(120);
    }
  }

  // --------- UI: Render Funktionen (werden aus loop() aufgerufen) ---------
  static void uiRenderStatus(const String& apIp, const String& staIp,
                             bool staConn, bool mqttConn, bool sdMounted,
                             int rssi, uint32_t rxPackets) {

    uiHeader("STATUS");

    // Icons + States
    drawWifiIcon(26, 70, staConn ? GREEN : RED);
    drawLoRaIcon(26, 120, mqttConn ? GREEN : ORANGE);
    drawSdIcon(26, 170, sdMounted ? GREEN : RED);

    tft.setTextColor(WHITE, DARK);
    tft.setTextSize(1);

    int x = 52;
    int y = 58;
    tft.setCursor(x,y); tft.print("AP IP: "); tft.println(apIp); y+=18;
    tft.setCursor(x,y); tft.print("STA:   "); tft.println(staConn ? staIp : "disconnected"); y+=18;
    tft.setCursor(x,y); tft.print("MQTT:  "); tft.println(mqttConn ? "connected" : "disconnected"); y+=18;
    tft.setCursor(x,y); tft.print("SD:    "); tft.println(sdMounted ? "mounted" : "not mounted"); y+=18;

    tft.setCursor(x,y); tft.print("RSSI:  "); tft.print(rssi); tft.println(" dBm"); y+=18;
    tft.setCursor(x,y); tft.print("RX:    "); tft.print(rxPackets); tft.println(" pkts"); y+=18;

    // Footer hint
    tft.setTextColor(GRAY, DARK);
    tft.setCursor(12, tft.height()-18);
    tft.print("BOOT/Touch: next page");
  }

  static void uiRenderWifi(const String& staSsid, bool staConn, int rssi) {
    uiHeader("WIFI");

    tft.setTextColor(WHITE, DARK);
    tft.setTextSize(2);
    tft.setCursor(12, 52);
    tft.println("WiFi Status");

    tft.setTextSize(1);
    tft.setCursor(12, 86);
    tft.print("STA SSID: ");
    tft.println(staSsid.length() ? staSsid : "-");

    tft.setCursor(12, 106);
    tft.print("STA Link: ");
    tft.println(staConn ? "connected" : "disconnected");

    tft.setCursor(12, 126);
    tft.print("RSSI: ");
    tft.print(rssi);
    tft.println(" dBm");

    // Signal bars
    int bars = 0;
    if (staConn) {
      if (rssi > -55) bars = 4;
      else if (rssi > -65) bars = 3;
      else if (rssi > -75) bars = 2;
      else bars = 1;
    }
    int bx=12, by=160;
    for (int i=0;i<4;i++){
      int h = (i+1)*10;
      uint16_t col = (i < bars) ? GREEN : GRAY;
      tft.fillRect(bx + i*14, by - h, 10, h, col);
    }

    tft.setTextColor(GRAY, DARK);
    tft.setCursor(12, tft.height()-18);
    tft.print("Tip: WebUI has full config");
  }

  // --------- BLE Scan optional ---------
  #ifndef RG_UI_ENABLE_BLE_SCAN
    #define RG_UI_ENABLE_BLE_SCAN 1
  #endif

  #if RG_UI_ENABLE_BLE_SCAN
    #include <BLEDevice.h>
    struct BleSeen {
      String name;
      String addr;
      int rssi = -127;
      uint32_t lastMs = 0;
    };
    static BleSeen bleSeen[10];
    static bool bleInited = false;
    static uint32_t bleLastScanMs = 0;

    static void bleInitOnce() {
      if (bleInited) return;
      bleInited = true;
      BLEDevice::init("");
      BLEDevice::setPower(ESP_PWR_LVL_P9); // max
    }

    static void bleScanOnce() {
      bleInitOnce();
      BLEScan* scan = BLEDevice::getScan();
      scan->setActiveScan(true);
      scan->setInterval(60);
      scan->setWindow(30);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
      // ESP32 Arduino core v3: start() returns BLEScanResults*
      BLEScanResults* res = scan->start(2, false); // 2s
      int count = res ? res->getCount() : 0;

      for (int i = 0; i < count; i++) {
        BLEAdvertisedDevice d = res->getDevice(i);
        String addr = d.getAddress().toString().c_str();
        String name = d.haveName() ? d.getName().c_str() : "";
        int rssi = d.getRSSI();

        // insert/update in list
        int idx = -1;
        for (int j = 0; j < 10; j++) {
          if (bleSeen[j].addr == addr) { idx = j; break; }
        }
        if (idx < 0) {
          // find oldest
          uint32_t oldest = 0xFFFFFFFF; int oi = 0;
          for (int j = 0; j < 10; j++) {
            if (bleSeen[j].lastMs < oldest) { oldest = bleSeen[j].lastMs; oi = j; }
          }
          idx = oi;
        }
        bleSeen[idx].addr = addr;
        bleSeen[idx].name = name;
        bleSeen[idx].rssi = rssi;
        bleSeen[idx].lastMs = millis();
      }
#else
      // Older cores: start() returns BLEScanResults by value
      BLEScanResults res = scan->start(2, false); // 2s

      for (int i = 0; i < res.getCount(); i++) {
        BLEAdvertisedDevice d = res.getDevice(i);
        String addr = d.getAddress().toString().c_str();
        String name = d.haveName() ? d.getName().c_str() : "";
        int rssi = d.getRSSI();

        // insert/update in list
        int idx = -1;
        for (int j = 0; j < 10; j++) {
          if (bleSeen[j].addr == addr) { idx = j; break; }
        }
        if (idx < 0) {
          // find oldest
          uint32_t oldest = 0xFFFFFFFF; int oi = 0;
          for (int j = 0; j < 10; j++) {
            if (bleSeen[j].lastMs < oldest) { oldest = bleSeen[j].lastMs; oi = j; }
          }
          idx = oi;
        }
        bleSeen[idx].addr = addr;
        bleSeen[idx].name = name;
        bleSeen[idx].rssi = rssi;
        bleSeen[idx].lastMs = millis();
      }
#endif

      scan->clearResults();
    }


    static void uiRenderBle() {
      uiHeader("BLE");

      // kick scan occasionally (nicht jedes redraw)
      uint32_t ms=millis();
      if (ms - bleLastScanMs > 8000) {
        bleLastScanMs = ms;
        bleScanOnce();
      }

      tft.setTextColor(WHITE, DARK);
      tft.setTextSize(2);
      tft.setCursor(12, 52);
      tft.println("Nearby BLE");

      tft.setTextSize(1);
      int y=82;
      for(int i=0;i<10;i++){
        if (bleSeen[i].lastMs==0) continue;
        // stale
        if (ms - bleSeen[i].lastMs > 20000) continue;

        String n = bleSeen[i].name.length()? bleSeen[i].name : "(no name)";
        tft.setCursor(12, y);
        tft.print(n);
        tft.setCursor(150, y);
        tft.print(bleSeen[i].rssi);
        tft.print(" dBm");
        y+=16;
        if (y > tft.height()-30) break;
      }

      tft.setTextColor(GRAY, DARK);
      tft.setCursor(12, tft.height()-18);
      tft.print("Auto scan every ~8s (2s active)");
    }
  #else
    static void uiRenderBle() {
      uiHeader("BLE");
      tft.setTextColor(WHITE, DARK);
      tft.setTextSize(2);
      tft.setCursor(12, 60);
      tft.println("BLE disabled");
    }
  #endif

  // --------- SD Info + Gallery ---------
  static void uiRenderSd(bool sdMounted) {
    uiHeader("SD");

    tft.setTextColor(WHITE, DARK);
    tft.setTextSize(2);
    tft.setCursor(12, 52);
    tft.println("SD Card");

    tft.setTextSize(1);
    tft.setCursor(12, 86);
    tft.print("Mounted: ");
    tft.println(sdMounted ? "yes" : "no");

    if (sdMounted) {
      uint64_t total = SD.totalBytes();
      uint64_t used  = SD.usedBytes();
      tft.setCursor(12, 106);
      tft.print("Total: ");
      tft.print((uint32_t)(total / (1024ULL*1024ULL)));
      tft.println(" MB");

      tft.setCursor(12, 126);
      tft.print("Used : ");
      tft.print((uint32_t)(used / (1024ULL*1024ULL)));
      tft.println(" MB");

      // usage bar
      int barX=12, barY=150, barW=216, barH=12;
      tft.drawRect(barX, barY, barW, barH, GRAY);
      uint32_t fill = (total>0) ? (uint32_t)((used * (barW-4)) / total) : 0;
      tft.fillRect(barX+2, barY+2, fill, barH-4, ORANGE);

      tft.setCursor(12, 176);
      tft.println("Media Gallery:");
      tft.setCursor(12, 192);
      tft.println("/Media Gallery (jpg/png/bmp)");
    } else {
      tft.setCursor(12, 120);
      tft.println("Insert FAT32 SD (SPI CS=GPIO5)");
    }
  }

  // Gallery: zeigt alle 15s eine Datei aus /Media Gallery
  static uint32_t galleryLastMs = 0;
  static String galleryFiles[32];
  static int galleryCount = 0;
  static int galleryIndex = 0;
  static bool galleryScanned = false;

  static bool isImageFile(const String& n) {
    String s=n; s.toLowerCase();
    return s.endsWith(".jpg") || s.endsWith(".jpeg") || s.endsWith(".png") || s.endsWith(".bmp");
  }

// --------- BMP Renderer (24-bit uncompressed) ---------
// Damit die Gallery sicher funktioniert (ohne JPG/PNG Decoder-Abhängigkeit),
// werden BMPs direkt gerendert. Empfohlen: 24-bit BMP, max 320x240.
static bool drawBmp24FromSD(const String& path, int x, int y, int maxW, int maxH) {
  File f = SD.open(path, FILE_READ);
  if (!f) return false;

  auto read16 = [&]() -> uint16_t {
    uint16_t r = f.read(); r |= (uint16_t)f.read() << 8; return r;
  };
  auto read32 = [&]() -> uint32_t {
    uint32_t r = f.read(); r |= (uint32_t)f.read() << 8; r |= (uint32_t)f.read() << 16; r |= (uint32_t)f.read() << 24; return r;
  };

  if (read16() != 0x4D42) { f.close(); return false; } // "BM"
  (void)read32(); // file size
  (void)read32(); // reserved
  uint32_t dataOfs = read32();

  uint32_t hdrSize = read32();
  if (hdrSize < 40) { f.close(); return false; }

  int32_t bmpW = (int32_t)read32();
  int32_t bmpH = (int32_t)read32();
  if (bmpW <= 0 || bmpH == 0) { f.close(); return false; }

  uint16_t planes = read16();
  uint16_t bpp    = read16();
  uint32_t comp   = read32();
  if (planes != 1 || bpp != 24 || comp != 0) { f.close(); return false; }

  (void)read32(); // imageSize
  (void)read32(); // xPels
  (void)read32(); // yPels
  (void)read32(); // clrUsed
  (void)read32(); // clrImportant

  bool flip = true;
  if (bmpH < 0) { bmpH = -bmpH; flip = false; }

  int drawW = bmpW;
  int drawH = bmpH;
  if (drawW > maxW) drawW = maxW;
  if (drawH > maxH) drawH = maxH;

  // Row padding to 4 bytes
  uint32_t rowSize = (uint32_t)((bmpW * 3 + 3) & ~3);

  // line buffer rgb565
  static uint16_t line[320]; // max width in landscape
  if (drawW > (int)(sizeof(line)/sizeof(line[0]))) drawW = sizeof(line)/sizeof(line[0]);

  for (int row = 0; row < drawH; row++) {
    uint32_t pos = dataOfs + (flip ? (uint32_t)(bmpH - 1 - row) * rowSize : (uint32_t)row * rowSize);
    if (!f.seek(pos)) break;

    for (int col = 0; col < drawW; col++) {
      uint8_t b = f.read();
      uint8_t g = f.read();
      uint8_t r = f.read();
      // RGB888 -> RGB565
      line[col] = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }
    // skip rest of row if cropped
    if (drawW < bmpW) {
      uint32_t skip = (uint32_t)(bmpW - drawW) * 3;
      f.seek(f.position() + skip);
    }

    tft.pushImage(x, y + row, drawW, 1, line);
  }

  f.close();
  return true;
}

  static void galleryScanOnce() {
    galleryScanned = true;
    galleryCount = 0;
    File dir = SD.open("/Media Gallery");
    if (!dir || !dir.isDirectory()) return;

    File f;
    while ((f = dir.openNextFile())) {
      if (!f.isDirectory()) {
        String name = String(f.name());
        if (isImageFile(name) && galleryCount < 32) {
          // f.name() can be full path on esp32 core; normalize to absolute
          if (!name.startsWith("/")) name = String("/Media Gallery/") + name;
          galleryFiles[galleryCount++] = name;
        }
      }
      f.close();
    }
    dir.close();
    galleryIndex = 0;
  }

  static void uiRenderGallery(bool sdMounted) {
    uiHeader("GALLERY");

    tft.setTextColor(WHITE, DARK);
    tft.setTextSize(2);
    tft.setCursor(12, 52);
    tft.println("Media Gallery");

    if (!sdMounted) {
      tft.setTextSize(1);
      tft.setCursor(12, 90);
      tft.println("SD not mounted.");
      tft.setCursor(12, 110);
      tft.println("Insert SD with /Media Gallery.");
      return;
    }

    if (!galleryScanned) galleryScanOnce();

    tft.setTextSize(1);
    tft.setCursor(12, 86);
    tft.print("Files: ");
    tft.println(galleryCount);

    if (galleryCount == 0) {
      tft.setCursor(12, 110);
      tft.println("No images found.");
      tft.setCursor(12, 130);
      tft.println("Put JPG/PNG/BMP into /Media Gallery.");
      return;
    }

    // rotate every 15s
    uint32_t ms = millis();
    if (ms - galleryLastMs > 15000) {
      galleryLastMs = ms;
      galleryIndex = (galleryIndex + 1) % galleryCount;
    }

    String path = galleryFiles[galleryIndex];
    tft.setCursor(12, 106);
    tft.print("Now: ");
    tft.println(path);

    // Bildanzeige: ohne Decoder-Abhängigkeit werden BMPs (24-bit, uncompressed) direkt gerendert.
// JPG/PNG werden in der Liste akzeptiert, aber hier nicht garantiert angezeigt.
int imgX=10, imgY=130, imgW=tft.width()-20, imgH=tft.height()-142;
tft.drawRect(imgX, imgY, imgW, imgH, GRAY);

bool drawn = false;
String lower = path; lower.toLowerCase();
if (lower.endsWith(".bmp")) {
  drawn = drawBmp24FromSD(path, imgX+1, imgY+1, imgW-2, imgH-2);
}

if (!drawn) {
  tft.setTextColor(GRAY, DARK);
  tft.setCursor(12, imgY + 12);
  tft.println("No renderer for this file.");
  tft.setCursor(12, imgY + 28);
  tft.println("Use 24-bit BMP for on-screen display.");
}

tft.setTextColor(GRAY, DARK);
tft.setCursor(12, tft.height()-18);
tft.print("Tap/BOOT: next image/page");

  }

  static void uiRenderAbout() {
    uiHeader("ABOUT");

    tft.setTextColor(WHITE, DARK);
    tft.setTextSize(2);
    tft.setCursor(12, 52);
    tft.println("AMESEDER PC SUPPORT");

    tft.setTextSize(1);
    tft.setCursor(12, 90);
    tft.println("created by Yanis");
    tft.setCursor(12, 110);
    tft.println("RadioGateway UI (LovyanGFX)");
    tft.setCursor(12, 130);
    tft.print("SPI Host: ");
    tft.println(TFT_SPI_HOST_NAME);
    tft.setCursor(12, 150);
    tft.print("BL: ");
    tft.print(blPercent);
    tft.println("%");

    tft.setTextColor(GRAY, DARK);
    tft.setCursor(12, tft.height()-12);
    tft.print("made with Arduino");
  }

  // Ersetzt altes API: wird vom Sketch genutzt
  static void tftDrawBoot(const char* line1, const char* line2) {
    // Keep compatibility, but use nicer boot page after init
    (void)line1; (void)line2;
    // No-op here; boot sequence uses tftBootSequence()
  }


static void tftSafeBegin() {
  if (tft_ok) return;

  blInitOnce();    // Backlight aus Config
    blWritePercent(cfg.bl_percent);
  pinMode(RG_BOOT_BTN_PIN, INPUT_PULLUP);

  tft.init();
  tft.setRotation(1); // 320x240 Landscape
  tft.setTextWrap(false);
  tft_ok = true;

  // Boot sequence (loading + hacked)
  tftBootSequence();

  ui_page = PAGE_STATUS;
  tft_last_draw_ms = 0;
}

static void tftDrawStatusScreen() {
  if (!tft_ok) return;

  // Input: boot button
  if (bootBtnPressed()) {
    uiNextPage();
  }

  // Input: touch -> next page (tap anywhere)
  #if RG_UI_ENABLE_TOUCH
    static uint32_t lastTouchMs = 0;
    int16_t tx, ty;
    if (touchGetPoint(tx, ty)) {
      uint32_t ms = millis();
      if (ms - lastTouchMs > 350) {
        lastTouchMs = ms;
        uiNextPage();
      }
    }
  #endif

  // throttle redraw
  uint32_t ms = millis();
  if (ms - tft_last_draw_ms < 600) return;
  tft_last_draw_ms = ms;

  // live state (keine Passwoerter anzeigen!)
  bool staConn = (WiFi.status() == WL_CONNECTED);
  String staIp = staConn ? WiFi.localIP().toString() : String("");
  String apIp  = WiFi.softAPIP().toString();
  String ssid  = staConn ? WiFi.SSID() : String("");
  int rssi = staConn ? WiFi.RSSI() : -127;

  bool mqttConn = false;
  #ifdef RG_ENABLE_MQTT
    mqttConn = mqtt.connected();
  #else
    mqttConn = mqtt.connected();
  #endif

  bool sdOk = sd_mounted;

  switch (ui_page) {
    case PAGE_STATUS:  uiRenderStatus(apIp, staIp, staConn, mqttConn, sdOk, rssi, 0); break;
    case PAGE_WIFI:    uiRenderWifi(ssid, staConn, rssi); break;
    case PAGE_BLE:     uiRenderBle(); break;
    case PAGE_SD:      uiRenderSd(sdOk); break;
    case PAGE_GALLERY: uiRenderGallery(sdOk); break;
    case PAGE_ABOUT:   uiRenderAbout(); break;
    case PAGE_BOOT:
    default:
      ui_page = PAGE_STATUS;
      uiRenderStatus(apIp, staIp, staConn, mqttConn, sdOk, rssi, 0);
      break;
  }
}

#endif

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
