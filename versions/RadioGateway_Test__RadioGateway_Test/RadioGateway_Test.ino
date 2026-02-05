#include <Arduino.h>
#include <LovyanGFX.hpp>

// ------------------ PINBELEGUNG ------------------
// Passe diese Werte an dein Board an:
static constexpr int PIN_MOSI = 13;
static constexpr int PIN_MISO = 12;   // falls nicht verdrahtet: -1
static constexpr int PIN_SCLK = 14;
static constexpr int PIN_CS   = 15;
static constexpr int PIN_DC   = 2;
static constexpr int PIN_RST  = -1;   // oft fest an 3V3 => -1
static constexpr int PIN_BL   = 21;   // Backlight

// ------------------ SPI HOST robust wählen ------------------
// ESP32 classic: VSPI_HOST/HSPI_HOST
// ESP32-S3 etc.: SPI2_HOST/SPI3_HOST
#if defined(SPI3_HOST)
  static constexpr int SPI_HOST_ID = SPI3_HOST;
  static constexpr const char* SPI_HOST_NAME = "SPI3_HOST";
#elif defined(VSPI_HOST)
  static constexpr int SPI_HOST_ID = VSPI_HOST;
  static constexpr const char* SPI_HOST_NAME = "VSPI_HOST";
#elif defined(SPI2_HOST)
  static constexpr int SPI_HOST_ID = SPI2_HOST;
  static constexpr const char* SPI_HOST_NAME = "SPI2_HOST";
#elif defined(HSPI_HOST)
  static constexpr int SPI_HOST_ID = HSPI_HOST;
  static constexpr const char* SPI_HOST_NAME = "HSPI_HOST";
#else
  static constexpr int SPI_HOST_ID = 0;
  static constexpr const char* SPI_HOST_NAME = "SPI_HOST?";
#endif

static constexpr uint32_t SPI_FREQ_WRITE = 40000000; // 40 MHz
static constexpr uint32_t SPI_FREQ_READ  = 16000000; // 16 MHz

// ------------------ LGFX Konfiguration (SPI) ------------------
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel;   // Alternative: lgfx::Panel_ST7789
  lgfx::Bus_SPI _bus;

public:
  LGFX() {
    { // Bus
      auto cfg = _bus.config();
      cfg.spi_host   = static_cast<decltype(cfg.spi_host)>(SPI_HOST_ID);
      cfg.spi_mode   = 0;
      cfg.freq_write = SPI_FREQ_WRITE;
      cfg.freq_read  = SPI_FREQ_READ;
      cfg.pin_sclk   = PIN_SCLK;
      cfg.pin_mosi   = PIN_MOSI;
      cfg.pin_miso   = PIN_MISO;
      cfg.pin_dc     = PIN_DC;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    { // Panel
      auto cfg = _panel.config();
      cfg.pin_cs   = PIN_CS;
      cfg.pin_rst  = PIN_RST;
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

static String gpioStr(int pin) {
  if (pin < 0) return String("NC");
  return String("GPIO") + String(pin);
}

static void printPinMapSerial() {
  Serial.println();
  Serial.println("=== TFT Pinbelegung (laufend) ===");
  Serial.printf("Panel: ILI9341 240x320\n");
  Serial.printf("SPI Host: %s\n", SPI_HOST_NAME);
  Serial.printf("SPI freq write/read: %lu / %lu Hz\n", (unsigned long)SPI_FREQ_WRITE, (unsigned long)SPI_FREQ_READ);
  Serial.printf("SCLK: %s\n", gpioStr(PIN_SCLK).c_str());
  Serial.printf("MOSI: %s\n", gpioStr(PIN_MOSI).c_str());
  Serial.printf("MISO: %s\n", gpioStr(PIN_MISO).c_str());
  Serial.printf("CS  : %s\n", gpioStr(PIN_CS).c_str());
  Serial.printf("DC  : %s\n", gpioStr(PIN_DC).c_str());
  Serial.printf("RST : %s\n", gpioStr(PIN_RST).c_str());
  Serial.printf("BL  : %s\n", gpioStr(PIN_BL).c_str());
  Serial.println("================================");
}

static void drawPinMapTFT() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextSize(1);

  int x = 10;
  int y = 10;
  const int lh = 20; // line height

  auto line = [&](const String& s) {
    tft.setCursor(x, y);
    tft.println(s);
    y += lh;
  };

  line("TFT Pinbelegung");
  line("----------------");
  line("Panel: ILI9341");
  line(String("SPI: ") + SPI_HOST_NAME);
  line(String("W/R Hz: ") + String(SPI_FREQ_WRITE) + "/" + String(SPI_FREQ_READ));
  y += 6;

  line(String("SCLK: ") + gpioStr(PIN_SCLK));
  line(String("MOSI: ") + gpioStr(PIN_MOSI));
  line(String("MISO: ") + gpioStr(PIN_MISO));
  line(String("CS  : ") + gpioStr(PIN_CS));
  line(String("DC  : ") + gpioStr(PIN_DC));
  line(String("RST : ") + gpioStr(PIN_RST));
  line(String("BL  : ") + gpioStr(PIN_BL));
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Backlight (einfach an/aus)
  if (PIN_BL >= 0) {
    pinMode(PIN_BL, OUTPUT);
    digitalWrite(PIN_BL, HIGH);
  }

  tft.init();
  tft.setRotation(1);

  printPinMapSerial();
  drawPinMapTFT();
}

void loop() {
  // nichts nötig – Pinbelegung wird einmal beim Start angezeigt
}
