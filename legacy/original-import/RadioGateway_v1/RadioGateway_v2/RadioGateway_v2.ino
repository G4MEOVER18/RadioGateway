/*
RadioGateway für ESP32-2432S028 mit LovyanGFX
Mit WebUI, MQTT, SD-Logging und TFT-Display
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
#include <time.h>
#include <LovyanGFX.hpp>

// ===================== BUILD FLAGS =====================
#define RG_TFT_ENABLED 1
#define RG_SD_CS_PIN 13  // SD Card Chip Select Pin (angepasst)
#define RG_TFT_BL_PIN 21 // Display Backlight Pin

// ===================== GLOBAL CONFIG =====================
static const char* RG_DEVICE   = "RadioGateway";
static const char* RG_LOCATION = "Portable";

// Zeitzone Zürich (CET/CEST)
#define TZ_OFFSET_SEC 3600      // GMT+1 = 3600 Sekunden
#define TZ_DAYLIGHT_SEC 3600    // Sommerzeit = +1 Stunde

static const char* NTP_SERVER1 = "pool.ntp.org";
static const char* NTP_SERVER2 = "time.cloudflare.com";

// ===================== SD / FILES =====================
static const char* CFG_DIR  = "/radiogateway";
static const char* CFG_PATH = "/radiogateway/config.json";

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

// ===================== WEB AUTH =====================
// *** WICHTIG: PASSWORT HIER SELBST EINTRAGEN ***
static const char* ADMIN_USERNAME = "yanis";
static const char* ADMIN_PASSWORD = "changeme"; // <-- ÄNDERN!

// ===================== LOVYANGFX SETUP =====================
static constexpr int PIN_MOSI = 13;
static constexpr int PIN_MISO = 12;
static constexpr int PIN_SCLK = 14;
static constexpr int PIN_CS   = 15;
static constexpr int PIN_DC   = 2;
static constexpr int PIN_RST  = -1;   // NC
static constexpr int PIN_BL   = 21;   // Backlight

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel;     // ILI9341 Driver
  lgfx::Bus_SPI _bus;
public:
  LGFX() {
    { // Bus
      auto cfg = _bus.config();
      cfg.spi_host   = HSPI_HOST;      // HSPI verwenden
      cfg.spi_mode   = 0;
      cfg.freq_write = 20000000;       // 20 MHz
      cfg.freq_read  = 8000000;
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

// ===================== STRUCTS =====================
struct Config {
  // WiFi Settings
  String ap_ssid = AP_SSID_DEFAULT;
  String ap_pass = AP_PASS_DEFAULT;
  uint8_t ap_channel = AP_CHANNEL_DEFAULT;
  String sta_ssid = "";
  String sta_pass = "";
  bool ap_enabled = true;
  bool sta_enabled = false;
  
  // MQTT Settings
  bool mqtt_enabled = true;
  String mqtt_host = MQTT_HOST_DEFAULT;
  uint16_t mqtt_port = MQTT_PORT_DEFAULT;
  String mqtt_user = "";
  String mqtt_pass = "";
  String topic_rx = "msh/#";
  String topic_tx = "radio/outgoing";
  String topic_status = "$SYS/#";
  
  // System Settings
  String device = RG_DEVICE;
  String hostname = "RadioGateway";
  String location = RG_LOCATION;
  uint8_t bl_percent = 60;  // Backlight percentage
};

// ===================== GLOBAL VARIABLES =====================
Config cfg;
WebServer server(80);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Global state variables
String lastMessages[10]; // Ring buffer for last 10 messages
int msgIndex = 0;
unsigned long lastMsgTime = 0;
bool sdOK = false;
String logBuffer = "";
String mqtt_last_topic = "";
String mqtt_last_payload = "";

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  Serial.println("RadioGateway booting...");

  // Initialize SD Card
  pinMode(RG_SD_CS_PIN, OUTPUT);
  digitalWrite(RG_SD_CS_PIN, HIGH);
  
  if (!SD.begin(RG_SD_CS_PIN)) {
    Serial.println("SD Card Mount Failed");
    sdOK = false;
  } else {
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
      Serial.println("No SD card attached");
      sdOK = false;
    } else {
      Serial.println("SD Card initialized");
      sdOK = true;
      
      // Create config directory if not exists
      if (!SD.exists(CFG_DIR)) {
        SD.mkdir(CFG_DIR);
      }
      
      // Load config
      loadConfig();
    }
  }

  // Initialize Display
#if RG_TFT_ENABLED
  if (PIN_BL >= 0) {
    pinMode(PIN_BL, OUTPUT);
    digitalWrite(PIN_BL, HIGH);
  }
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("RadioGateway");
  tft.setTextSize(1);
  tft.setCursor(10, 40);
  tft.println("Starting up...");
#endif

  // Set hostname
  WiFi.setHostname(cfg.hostname.c_str());

  // Setup WiFi
  setupWiFi();

  // Setup time
  configTime(TZ_OFFSET_SEC, TZ_DAYLIGHT_SEC, NTP_SERVER1, NTP_SERVER2);

  // Setup MQTT
  mqttClient.setServer(cfg.mqtt_host.c_str(), cfg.mqtt_port);
  mqttClient.setCallback(mqttCallback);

  // Setup Web Server
  setupWebServer();
  server.begin();

  // Setup mDNS
  if (MDNS.begin(cfg.hostname.c_str())) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS started");
  }

  Serial.println("Setup complete");
  
#if RG_TFT_ENABLED
  tft.setCursor(10, 60);
  tft.println("Setup complete!");
#endif
}

// ===================== MAIN LOOP =====================
void loop() {
  // Handle web server
  server.handleClient();
  
  // Handle MQTT
  if (cfg.mqtt_enabled) {
    if (!mqttClient.connected()) {
      reconnectMQTT();
    }
    mqttClient.loop();
  }
  
  // Update display periodically
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 5000) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
  
  // mDNS service
}

// ===================== WIFI FUNCTIONS =====================
void setupWiFi() {
  // Configure WiFi mode
  if (cfg.ap_enabled && cfg.sta_enabled) {
    WiFi.mode(WIFI_MODE_APSTA);
  } else if (cfg.ap_enabled) {
    WiFi.mode(WIFI_MODE_AP);
  } else if (cfg.sta_enabled) {
    WiFi.mode(WIFI_MODE_STA);
  } else {
    WiFi.mode(WIFI_MODE_NULL);
    return;
  }
  
  if (cfg.ap_enabled) {
    setupAccessPoint();
  }
  
  if (cfg.sta_enabled) {
    setupStation();
  }
}

void setupAccessPoint() {
  WiFi.softAPConfig(apIP, apGW, apMASK);
  if (cfg.ap_pass.length() >= 8) {
    WiFi.softAP(cfg.ap_ssid.c_str(), cfg.ap_pass.c_str(), cfg.ap_channel);
  } else {
    WiFi.softAP(cfg.ap_ssid.c_str()); // offenes Netzwerk
  }
  Serial.println("AP started: " + cfg.ap_ssid);
}

void setupStation() {
  if (cfg.sta_ssid.length() > 0) {
    WiFi.begin(cfg.sta_ssid.c_str(), cfg.sta_pass.c_str());
    Serial.print("Connecting to STA: " + cfg.sta_ssid);
    
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
      delay(500);
      Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println(" STA connected!");
    } else {
      Serial.println(" STA connection failed!");
    }
  }
}

// ===================== MQTT FUNCTIONS =====================
void reconnectMQTT() {
  if (cfg.mqtt_host.length() == 0) return;
  
  Serial.print("Attempting MQTT connection...");
  
  bool connected;
  if (cfg.mqtt_user.length() > 0) {
    connected = mqttClient.connect(
      cfg.hostname.c_str(),
      cfg.mqtt_user.c_str(),
      cfg.mqtt_pass.c_str()
    );
  } else {
    connected = mqttClient.connect(cfg.hostname.c_str());
  }
  
  if (connected) {
    Serial.println("MQTT connected");
    
    // Subscribe to topics
    String topics[] = {cfg.topic_rx, cfg.topic_tx, cfg.topic_status};
    for (int i = 0; i < 3; i++) {
      if (topics[i].length() > 0) {
        mqttClient.subscribe(topics[i].c_str());
        Serial.println("Subscribed to: " + topics[i]);
      }
    }
  } else {
    Serial.print("MQTT failed, rc=");
    Serial.println(mqttClient.state());
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Store message in ring buffer
  String msg = "";
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  
  mqtt_last_topic = String(topic);
  mqtt_last_payload = msg;
  lastMsgTime = millis();
  
  String entry = "[" + String(millis()/1000) + "] " + String(topic) + ": " + msg;
  lastMessages[msgIndex % 10] = entry;
  msgIndex++;
  
  // Log to buffer
  logBuffer += entry + "\n";
  
  // Save to SD if buffer gets large
  if (logBuffer.length() > 1024 && sdOK) {
    saveLogToSD();
  }
  
  Serial.println(entry);
}

void saveLogToSD() {
  if (!sdOK) return;
  
  File file = SD.open("/radiogateway/mqtt.log", FILE_APPEND);
  if (file) {
    file.print(logBuffer);
    file.close();
    logBuffer = "";
    Serial.println("Log saved to SD");
  }
}

// ===================== DISPLAY FUNCTIONS =====================
void updateDisplay() {
#if RG_TFT_ENABLED
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  
  // Header
  tft.setTextColor(TFT_YELLOW);
  tft.println(cfg.device);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.println("Location: " + cfg.location);
  
  // Network info
  tft.println();
  tft.setTextColor(TFT_CYAN);
  tft.print("Mode: ");
  if (cfg.ap_enabled && cfg.sta_enabled) {
    tft.println("AP+STA");
  } else if (cfg.ap_enabled) {
    tft.println("AP");
  } else if (cfg.sta_enabled) {
    tft.println("STA");
  } else {
    tft.println("OFF");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    tft.print("IP: ");
    tft.println(WiFi.localIP().toString());
  }
  
  if (cfg.ap_enabled) {
    tft.print("AP IP: ");
    tft.println(WiFi.softAPIP().toString());
  }
  
  // MQTT status
  tft.println();
  tft.setTextColor(TFT_MAGENTA);
  tft.print("MQTT: ");
  tft.println(mqttClient.connected() ? "Connected" : "Disconnected");
  
  // Last message time
  if (lastMsgTime > 0) {
    tft.print("Last msg: ");
    tft.print((millis() - lastMsgTime)/1000);
    tft.println("s ago");
  }
  
  // SD status
  tft.println();
  tft.setTextColor(TFT_GREEN);
  tft.print("SD: ");
  tft.println(sdOK ? "OK" : "ERROR");
#endif
}

// ===================== CONFIG FUNCTIONS =====================
void loadConfig() {
  if (!sdOK) return;
  
  File file = SD.open(CFG_PATH);
  if (!file) {
    Serial.println("Config file not found, using defaults");
    return;
  }
  
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  
  if (error) {
    Serial.print("Failed to parse config file: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Load values from JSON
  cfg.ap_ssid = doc["ap_ssid"] | cfg.ap_ssid;
  cfg.ap_pass = doc["ap_pass"] | cfg.ap_pass;
  cfg.ap_channel = doc["ap_channel"] | cfg.ap_channel;
  cfg.sta_ssid = doc["sta_ssid"] | cfg.sta_ssid;
  cfg.sta_pass = doc["sta_pass"] | cfg.sta_pass;
  cfg.ap_enabled = doc["ap_enabled"] | cfg.ap_enabled;
  cfg.sta_enabled = doc["sta_enabled"] | cfg.sta_enabled;
  
  cfg.mqtt_enabled = doc["mqtt_enabled"] | cfg.mqtt_enabled;
  cfg.mqtt_host = doc["mqtt_host"] | cfg.mqtt_host;
  cfg.mqtt_port = doc["mqtt_port"] | cfg.mqtt_port;
  cfg.mqtt_user = doc["mqtt_user"] | cfg.mqtt_user;
  cfg.mqtt_pass = doc["mqtt_pass"] | cfg.mqtt_pass;
  cfg.topic_rx = doc["topic_rx"] | cfg.topic_rx;
  cfg.topic_tx = doc["topic_tx"] | cfg.topic_tx;
  cfg.topic_status = doc["topic_status"] | cfg.topic_status;
  
  cfg.device = doc["device"] | cfg.device;
  cfg.hostname = doc["hostname"] | cfg.hostname;
  cfg.location = doc["location"] | cfg.location;
  cfg.bl_percent = doc["bl_percent"] | cfg.bl_percent;
  
  Serial.println("Config loaded from SD");
}

void saveConfig() {
  if (!sdOK) return;
  
  StaticJsonDocument<1024> doc;
  
  // Save values to JSON
  doc["ap_ssid"] = cfg.ap_ssid;
  doc["ap_pass"] = cfg.ap_pass;
  doc["ap_channel"] = cfg.ap_channel;
  doc["sta_ssid"] = cfg.sta_ssid;
  doc["sta_pass"] = cfg.sta_pass;
  doc["ap_enabled"] = cfg.ap_enabled;
  doc["sta_enabled"] = cfg.sta_enabled;
  
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
  
  File file = SD.open(CFG_PATH, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open config file for writing");
    return;
  }
  
  if (serializeJson(doc, file) == 0) {
    Serial.println("Failed to write config to file");
  } else {
    Serial.println("Config saved to SD");
  }
  
  file.close();
}

// ===================== WEB SERVER =====================
bool isAuthOk() {
  if (!server.authenticate(ADMIN_USERNAME, ADMIN_PASSWORD)) {
    server.requestAuthentication();
    return false;
  }
  return true;
}

String htmlHeader(String title) {
  String h = "<!DOCTYPE html><html>";
  h += "<head>";
  h += "<meta charset='utf-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>" + title + "</title>";
  h += "<style>";
  h += ":root {";
  h += "--primary: #6200ee;";
  h += "--primary-dark: #3700b3;";
  h += "--secondary: #03dac6;";
  h += "--background: #121212;";
  h += "--surface: #1e1e1e;";
  h += "--on-primary: #ffffff;";
  h += "--on-secondary: #000000;";
  h += "--on-surface: #ffffff;";
  h += "--error: #b00020;";
  h += "--text-primary: rgba(255, 255, 255, 0.87);";
  h += "--text-secondary: rgba(255, 255, 255, 0.6);";
  h += "}";
  h += "body {";
  h += "font-family: 'Roboto', 'Helvetica Neue', Helvetica, Arial, sans-serif;";
  h += "margin: 0;";
  h += "padding: 0;";
  h += "background-color: var(--background);";
  h += "color: var(--text-primary);";
  h += "}";
  h += "header {";
  h += "background-color: var(--primary);";
  h += "color: var(--on-primary);";
  h += "padding: 1rem;";
  h += "box-shadow: 0 2px 4px rgba(0,0,0,0.2);";
  h += "}";
  h += "nav {";
  h += "background-color: var(--surface);";
  h += "box-shadow: 0 2px 4px rgba(0,0,0,0.2);";
  h += "}";
  h += "nav ul {";
  h += "display: flex;";
  h += "flex-wrap: wrap;";
  h += "list-style: none;";
  h += "margin: 0;";
  h += "padding: 0;";
  h += "}";
  h += "nav li {";
  h += "margin: 0;";
  h += "}";
  h += "nav a {";
  h += "display: block;";
  h += "padding: 1rem;";
  h += "color: var(--text-primary);";
  h += "text-decoration: none;";
  h += "transition: background-color 0.3s;";
  h += "}";
  h += "nav a:hover, nav a.active {";
  h += "background-color: rgba(255, 255, 255, 0.1);";
  h += "}";
  h += ".container {";
  h += "max-width: 1200px;";
  h += "margin: 2rem auto;";
  h += "padding: 0 1rem;";
  h += "}";
  h += ".card {";
  h += "background-color: var(--surface);";
  h += "border-radius: 8px;";
  h += "box-shadow: 0 2px 10px rgba(0,0,0,0.3);";
  h += "margin-bottom: 2rem;";
  h += "overflow: hidden;";
  h += "}";
  h += ".card-header {";
  h += "background-color: var(--primary);";
  h += "color: var(--on-primary);";
  h += "padding: 1rem;";
  h += "font-size: 1.25rem;";
  h += "font-weight: 500;";
  h += "}";
  h += ".card-content {";
  h += "padding: 1rem;";
  h += "}";
  h += ".grid {";
  h += "display: grid;";
  h += "grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));";
  h += "gap: 1.5rem;";
  h += "}";
  h += ".metric-card {";
  h += "background-color: var(--surface);";
  h += "border-radius: 8px;";
  h += "box-shadow: 0 2px 10px rgba(0,0,0,0.3);";
  h += "padding: 1.5rem;";
  h += "text-align: center;";
  h += "}";
  h += ".metric-value {";
  h += "font-size: 2.5rem;";
  h += "font-weight: 300;";
  h += "margin: 0.5rem 0;";
  h += "}";
  h += ".metric-title {";
  h += "color: var(--text-secondary);";
  h += "font-size: 1rem;";
  h += "margin: 0;";
  h += "}";
  h += ".status-ok {";
  h += "color: #4caf50;";
  h += "}";
  h += ".status-error {";
  h += "color: #f44336;";
  h += "}";
  h += ".status-warning {";
  h += "color: #ff9800;";
  h += "}";
  h += "h1, h2, h3 {";
  h += "margin-top: 0;";
  h += "}";
  h += "form {";
  h += "margin: 1rem 0;";
  h += "}";
  h += "label {";
  h += "display: block;";
  h += "margin-bottom: 0.5rem;";
  h += "font-weight: 500;";
  h += "}";
  h += "input, select, textarea {";
  h += "width: 100%;";
  h += "padding: 0.75rem;";
  h += "margin-bottom: 1rem;";
  h += "border: 1px solid #333;";
  h += "border-radius: 4px;";
  h += "background-color: #2d2d2d;";
  h += "color: var(--text-primary);";
  h += "box-sizing: border-box;";
  h += "}";
  h += "button, .btn {";
  h += "background-color: var(--primary);";
  h += "color: var(--on-primary);";
  h += "border: none;";
  h += "border-radius: 4px;";
  h += "padding: 0.75rem 1rem;";
  h += "cursor: pointer;";
  h += "font-weight: 500;";
  h += "text-transform: uppercase;";
  h += "letter-spacing: 1px;";
  h += "transition: background-color 0.3s;";
  h += "}";
  h += "button:hover, .btn:hover {";
  h += "background-color: var(--primary-dark);";
  h += "}";
  h += "button.secondary, .btn.secondary {";
  h += "background-color: var(--secondary);";
  h += "color: var(--on-secondary);";
  h += "}";
  h += "button.secondary:hover, .btn.secondary:hover {";
  h += "opacity: 0.9;";
  h += "}";
  h += "button.danger, .btn.danger {";
  h += "background-color: var(--error);";
  h += "}";
  h += "button.danger:hover, .btn.danger:hover {";
  h += "background-color: #d32f2f;";
  h += "}";
  h += "table {";
  h += "width: 100%;";
  h += "border-collapse: collapse;";
  h += "margin: 1rem 0;";
  h += "}";
  h += "th, td {";
  h += "padding: 0.75rem;";
  h += "text-align: left;";
  h += "border-bottom: 1px solid #333;";
  h += "}";
  h += "th {";
  h += "background-color: rgba(255, 255, 255, 0.05);";
  h += "}";
  h += "tr:hover {";
  h += "background-color: rgba(255, 255, 255, 0.05);";
  h += "}";
  h += "pre {";
  h += "background-color: #2d2d2d;";
  h += "padding: 1rem;";
  h += "border-radius: 4px;";
  h += "overflow-x: auto;";
  h += "}";
  h += ".muted {";
  h += "color: var(--text-secondary);";
  h += "font-size: 0.875rem;";
  h += "}";
  h += "</style>";
  h += "</head>";
  h += "<body>";
  h += "<header>";
  h += "<h1>" + title + "</h1>";
  h += "</header>";
  h += "<nav>";
  h += "<ul>";
  h += "<li><a href='/'>Dashboard</a></li>";
  h += "<li><a href='/wifi'>WiFi</a></li>";
  h += "<li><a href='/mqtt'>MQTT</a></li>";
  h += "<li><a href='/sd'>SD Card</a></li>";
  h += "<li><a href='/devices'>Devices</a></li>";
  h += "<li><a href='/config'>Config</a></li>";
  h += "</ul>";
  h += "</nav>";
  h += "<main class='container'>";
  return h;
}

String htmlFooter() {
  String f = "</main>";
  f += "</body>";
  f += "</html>";
  return f;
}

void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    if (!isAuthOk()) return;
    String html = htmlHeader("RadioGateway - Dashboard");
    
    // Status Cards
    html += "<div class='grid'>";
    
    // WiFi Status
    html += "<div class='metric-card'>";
    html += "<h3 class='metric-title'>WiFi Mode</h3>";
    html += "<div class='metric-value'>";
    if (cfg.ap_enabled && cfg.sta_enabled) html += "AP+STA";
    else if (cfg.ap_enabled) html += "AP";
    else if (cfg.sta_enabled) html += "STA";
    else html += "Disabled";
    html += "</div>";
    if (cfg.ap_enabled) {
      html += "<div>AP IP: " + WiFi.softAPIP().toString() + "</div>";
    }
    if (cfg.sta_enabled && WiFi.status() == WL_CONNECTED) {
      html += "<div>STA IP: " + WiFi.localIP().toString() + "</div>";
    }
    html += "</div>";
    
    // SD Card Status
    html += "<div class='metric-card'>";
    html += "<h3 class='metric-title'>SD Card</h3>";
    html += "<div class='metric-value " + String(sdOK ? "status-ok" : "status-error") + "'>";
    html += sdOK ? "Mounted" : "Not Mounted";
    html += "</div>";
    html += "</div>";
    
    // MQTT Status
    html += "<div class='metric-card'>";
    html += "<h3 class='metric-title'>MQTT</h3>";
    html += "<div class='metric-value " + String(mqttClient.connected() ? "status-ok" : "status-error") + "'>";
    html += mqttClient.connected() ? "Connected" : "Disconnected";
    html += "</div>";
    html += "</div>";
    
    // Last Message
    html += "<div class='metric-card'>";
    html += "<h3 class='metric-title'>Last Message</h3>";
    html += "<div class='metric-value'>";
    html += lastMsgTime > 0 ? String((millis() - lastMsgTime)/1000) + "s ago" : "None";
    html += "</div>";
    html += "</div>";
    
    html += "</div>";
    
    // Actions
    html += "<div class='card'>";
    html += "<div class='card-header'>System Actions</div>";
    html += "<div class='card-content'>";
    html += "<form method='POST' action='/action'>";
    html += "<button type='submit' name='action' value='reboot'>Reboot Device</button>";
    html += "<button type='submit' name='action' value='mqtt_connect' class='secondary'>Connect MQTT</button>";
    html += "<button type='submit' name='action' value='mqtt_disconnect' class='danger'>Disconnect MQTT</button>";
    html += "</form>";
    html += "</div>";
    html += "</div>";
    
    html += htmlFooter();
    server.send(200, "text/html", html);
  });
  
  server.on("/action", HTTP_POST, []() {
    if (!isAuthOk()) return;
    
    if (server.hasArg("action")) {
      String action = server.arg("action");
      
      if (action == "reboot") {
        server.send(200, "text/plain", "Rebooting...");
        delay(1000);
        ESP.restart();
      } 
      else if (action == "mqtt_connect") {
        reconnectMQTT();
        server.sendHeader("Location", "/");
        server.send(302, "text/plain", "MQTT connect initiated");
        return;
      }
      else if (action == "mqtt_disconnect") {
        mqttClient.disconnect();
        server.sendHeader("Location", "/");
        server.send(302, "text/plain", "MQTT disconnect initiated");
        return;
      }
    }
    
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Action completed");
  });
  
  server.on("/wifi", HTTP_GET, []() {
    if (!isAuthOk()) return;
    String html = htmlHeader("RadioGateway - WiFi Settings");
    html += "<div class='card'>";
    html += "<div class='card-header'>WiFi Configuration</div>";
    html += "<div class='card-content'>";
    html += "<form method='POST' action='/wifi/save'>";
    
    html += "<h3>Mode</h3>";
    html += "<p><label><input type='checkbox' name='ap_enabled' " + String(cfg.ap_enabled ? "checked" : "") + "> Enable AP</label></p>";
    html += "<p><label><input type='checkbox' name='sta_enabled' " + String(cfg.sta_enabled ? "checked" : "") + "> Enable STA</label></p>";
    
    html += "<h3>Access Point (AP)</h3>";
    html += "<p><label>SSID<br><input type='text' name='ap_ssid' value='" + cfg.ap_ssid + "'></label></p>";
    html += "<p><label>Password<br><input type='password' name='ap_pass' value='" + cfg.ap_pass + "'></label></p>";
    
    html += "<h3>Station (STA)</h3>";
    html += "<p><label>SSID<br><input type='text' name='sta_ssid' value='" + cfg.sta_ssid + "'></label></p>";
    html += "<p><label>Password<br><input type='password' name='sta_pass' value='" + cfg.sta_pass + "'></label></p>";
    
    html += "<button type='submit'>Save Settings</button>";
    html += "</form>";
    html += "</div>";
    html += "</div>";
    html += htmlFooter();
    
    server.send(200, "text/html", html);
  });
  
  server.on("/wifi/save", HTTP_POST, []() {
    if (!isAuthOk()) return;
    
    // Save settings
    cfg.ap_enabled = server.hasArg("ap_enabled");
    cfg.sta_enabled = server.hasArg("sta_enabled");
    
    if (server.hasArg("ap_ssid")) cfg.ap_ssid = server.arg("ap_ssid");
    if (server.hasArg("ap_pass")) cfg.ap_pass = server.arg("ap_pass");
    if (server.hasArg("sta_ssid")) cfg.sta_ssid = server.arg("sta_ssid");
    if (server.hasArg("sta_pass")) cfg.sta_pass = server.arg("sta_pass");
    
    // Apply changes
    WiFi.mode(WIFI_MODE_NULL);
    delay(100);
    
    if (cfg.ap_enabled || cfg.sta_enabled) {
      WiFi.mode(cfg.ap_enabled && cfg.sta_enabled ? WIFI_MODE_APSTA : 
                cfg.ap_enabled ? WIFI_MODE_AP : WIFI_MODE_STA);
                
      if (cfg.ap_enabled) setupAccessPoint();
      if (cfg.sta_enabled) setupStation();
    }
    
    server.sendHeader("Location", "/wifi");
    server.send(302, "text/plain", "Settings saved. You may need to reboot for all changes to take effect.");
  });
  
  server.on("/mqtt", HTTP_GET, []() {
    if (!isAuthOk()) return;
    String html = htmlHeader("RadioGateway - MQTT Settings");
    html += "<div class='card'>";
    html += "<div class='card-header'>MQTT Configuration</div>";
    html += "<div class='card-content'>";
    html += "<form method='POST' action='/mqtt/save'>";
    
    html += "<p><label><input type='checkbox' name='mqtt_enabled' " + String(cfg.mqtt_enabled ? "checked" : "") + "> Enable MQTT</label></p><br>";
    
    html += "<p><label>Broker Host<br><input name='mqtt_host' value='" + cfg.mqtt_host + "'></label></p>";
    html += "<p><label>Port<br><input name='mqtt_port' value='" + String(cfg.mqtt_port) + "'></label></p>";
    html += "<p><label>Username (optional)<br><input name='mqtt_user' value='" + cfg.mqtt_user + "'></label></p>";
    html += "<p><label>Password (optional)<br><input type='password' name='mqtt_pass' value='" + cfg.mqtt_pass + "'></label></p>";
    
    html += "<h3>Topics</h3>";
    html += "<p><label>Subscribe Topic<br><input name='topic_rx' value='" + cfg.topic_rx + "'></label></p>";
    html += "<p><label>Publish Topic<br><input name='topic_tx' value='" + cfg.topic_tx + "'></label></p>";
    html += "<p><label>Status Topic<br><input name='topic_status' value='" + cfg.topic_status + "'></label></p><br>";
    
    html += "<button type='submit'>Save Settings</button>";
    html += "</form>";
    
    html += "<hr>";
    html += "<form method='POST' action='/mqtt/test'>";
    html += "<p><label>Test payload → publish to TX topic</label><br>";
    html += "<input name='payload' value='{\"hello\":\"from RadioGateway\"}'></p>";
    html += "<button type='submit'>Publish Test</button></form>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<div class='card-header'>Last RX Message</div>";
    html += "<div class='card-content'>";
    html += "<p>Topic: <code>" + mqtt_last_topic + "</code></p>";
    html += "<p>Age: <code>" + String(lastMsgTime ? (String((millis()-lastMsgTime)/1000) + "s") : "n/a") + "</code></p>";
    html += "<pre>" + mqtt_last_payload + "</pre>";
    html += "<div class='muted'>BreachGateway can send messages to this device later. Preparation: RX Topic.</div>";
    html += "</div>";
    html += "</div>";
    
    html += htmlFooter();
    server.send(200, "text/html", html);
  });
  
  server.on("/mqtt/save", HTTP_POST, []() {
    if (!isAuthOk()) return;
    
    if (server.hasArg("mqtt_enabled")) cfg.mqtt_enabled = (server.arg("mqtt_enabled") == "on");
    if (server.hasArg("mqtt_host"))    cfg.mqtt_host = server.arg("mqtt_host");
    if (server.hasArg("mqtt_port"))    cfg.mqtt_port = server.arg("mqtt_port").toInt();
    if (server.hasArg("mqtt_user"))    cfg.mqtt_user = server.arg("mqtt_user");
    if (server.hasArg("mqtt_pass"))    cfg.mqtt_pass = server.arg("mqtt_pass");
    if (server.hasArg("topic_rx"))     cfg.topic_rx = server.arg("topic_rx");
    if (server.hasArg("topic_tx"))     cfg.topic_tx = server.arg("topic_tx");
    if (server.hasArg("topic_status")) cfg.topic_status = server.arg("topic_status");
    
    // Apply changes
    if (cfg.mqtt_enabled) {
      mqttClient.setServer(cfg.mqtt_host.c_str(), cfg.mqtt_port);
    } else {
      mqttClient.disconnect();
    }
    
    server.sendHeader("Location", "/mqtt");
    server.send(302, "text/plain", "MQTT settings saved");
  });
  
  server.on("/mqtt/test", HTTP_POST, []() {
    if (!isAuthOk()) return;
    
    if (server.hasArg("payload")) {
      String payload = server.arg("payload");
      if (mqttClient.connected()) {
        mqttClient.publish(cfg.topic_tx.c_str(), payload.c_str());
        server.sendHeader("Location", "/mqtt");
        server.send(302, "text/plain", "Test message published");
        return;
      }
    }
    
    server.sendHeader("Location", "/mqtt");
    server.send(302, "text/plain", "Failed to publish test message");
  });
  
  server.on("/sd", HTTP_GET, []() {
    if (!isAuthOk()) return;
    String html = htmlHeader("RadioGateway - SD Card Management");
    html += "<div class='card'>";
    html += "<div class='card-header'>SD Card Status</div>";
    html += "<div class='card-content'>";
    html += "<p><strong>Mounted:</strong> <span class='" + String(sdOK ? "status-ok" : "status-error") + "'>" + String(sdOK ? "Yes" : "No") + "</span></p>";
    
    if (sdOK) {
      uint32_t freeBytes = SD.totalBytes() - SD.usedBytes();
      html += "<p><strong>Total Space:</strong> " + String(SD.totalBytes() / 1024.0 / 1024.0, 2) + " MB</p>";
      html += "<p><strong>Used Space:</strong> " + String(SD.usedBytes() / 1024.0 / 1024.0, 2) + " MB</p>";
      html += "<p><strong>Free Space:</strong> " + String(freeBytes / 1024.0 / 1024.0, 2) + " MB</p>";
    }
    
    html += "</div>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<div class='card-header'>File Browser</div>";
    html += "<div class='card-content'>";
    if (sdOK) {
      html += "<table><tr><th>Name</th><th>Size</th><th>Action</th></tr>";
      
      File root = SD.open("/");
      File file = root.openNextFile();
      
      while (file) {
        if (!file.isDirectory()) {
          html += "<tr>";
          html += "<td>" + String(file.name()) + "</td>";
          html += "<td>" + String(file.size()) + " bytes</td>";
          html += "<td><a href='/sd/download?file=" + String(file.name()) + "'>Download</a> | ";
          html += "<a href='/sd/delete?file=" + String(file.name()) + "' onclick='return confirm(\"Delete this file?\")'>Delete</a></td>";
          html += "</tr>";
        }
        file = root.openNextFile();
      }
      
      html += "</table>";
    } else {
      html += "<p class='status-error'>SD card not mounted</p>";
    }
    html += "</div>";
    html += "</div>";
    html += htmlFooter();
    
    server.send(200, "text/html", html);
  });
  
  server.on("/sd/download", HTTP_GET, []() {
    if (!isAuthOk()) return;
    
    if (server.hasArg("file")) {
      String filename = "/" + server.arg("file");
      File file = SD.open(filename, FILE_READ);
      if (file) {
        server.streamFile(file, "application/octet-stream");
        file.close();
        return;
      }
    }
    
    server.send(404, "text/plain", "File not found");
  });
  
  server.on("/sd/delete", HTTP_GET, []() {
    if (!isAuthOk()) return;
    
    if (server.hasArg("file")) {
      String filename = "/" + server.arg("file");
      if (SD.remove(filename.c_str())) {
        server.sendHeader("Location", "/sd");
        server.send(302, "text/plain", "File deleted");
        return;
      }
    }
    
    server.send(500, "text/plain", "Could not delete file");
  });
  
  server.on("/devices", HTTP_GET, []() {
    if (!isAuthOk()) return;
    String html = htmlHeader("RadioGateway - Connected Devices");
    html += "<div class='card'>";
    html += "<div class='card-header'>AP Clients</div>";
    html += "<div class='card-content'>";
    html += "<p>This section would show connected clients to the AP.</p>";
    html += "<p>In a real implementation, you would use:</p>";
    html += "<pre>WiFi.softAPgetNumStations()\nWiFi.softAPgetStationMac()</pre>";
    html += "</div>";
    html += "</div>";
    html += htmlFooter();
    
    server.send(200, "text/html", html);
  });
  
  server.on("/config", HTTP_GET, []() {
    if (!isAuthOk()) return;
    String html = htmlHeader("RadioGateway - Configuration");
    html += "<div class='card'>";
    html += "<div class='card-header'>System Configuration</div>";
    html += "<div class='card-content'>";
    html += "<form method='POST' action='/config/save'>";
    
    html += "<p><label>Device Name<br><input name='device' value='" + cfg.device + "'></label></p>";
    html += "<p><label>Hostname<br><input name='hostname' value='" + cfg.hostname + "'></label></p>";
    html += "<p><label>Location<br><input name='location' value='" + cfg.location + "'></label></p>";
    html += "<p><label>Backlight Percent<br><input type='number' name='bl_percent' value='" + String(cfg.bl_percent) + "' min='0' max='100'></label></p>";
    
    html += "<button type='submit'>Save Configuration</button>";
    html += "</form>";
    
    html += "<h3>Danger Zone</h3>";
    html += "<form method='POST' action='/config/reset' style='display:inline;'>";
    html += "<button type='submit' class='danger'>Reset to Defaults</button>";
    html += "</form>";
    
    html += "</div>";
    html += "</div>";
    html += htmlFooter();
    
    server.send(200, "text/html", html);
  });
  
  server.on("/config/save", HTTP_POST, []() {
    if (!isAuthOk()) return;
    
    if (server.hasArg("device")) cfg.device = server.arg("device");
    if (server.hasArg("hostname")) cfg.hostname = server.arg("hostname");
    if (server.hasArg("location")) cfg.location = server.arg("location");
    if (server.hasArg("bl_percent")) cfg.bl_percent = server.arg("bl_percent").toInt();
    
    // Apply hostname
    WiFi.setHostname(cfg.hostname.c_str());
    
    saveConfig();
    
    server.sendHeader("Location", "/config");
    server.send(302, "text/plain", "Configuration saved");
  });
  
  server.on("/config/reset", HTTP_POST, []() {
    if (!isAuthOk()) return;
    
    // Reset to defaults
    cfg = Config();
    
    server.sendHeader("Location", "/config");
    server.send(302, "text/plain", "Configuration reset to defaults");
  });
  
  server.onNotFound([]() {
    server.send(404, "text/plain", "File Not Found");
  });
}

// ===================== HELPER FUNCTIONS =====================