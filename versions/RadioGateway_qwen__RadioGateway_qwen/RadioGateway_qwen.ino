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
#define RG_SD_CS_PIN 13  // SD Card Chip Select Pin
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
static constexpr int PIN_MOSI = 23;
static constexpr int PIN_MISO = 19;
static constexpr int PIN_SCLK = 18;
static constexpr int PIN_CS   = 15;
static constexpr int PIN_DC   = 2;
static constexpr int PIN_RST  = -1;   // oft fest an 3V3 => -1
static constexpr int PIN_BL   = 21;   // Backlight

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel;     // Alternative: lgfx::Panel_ST7789
  lgfx::Bus_SPI _bus;
public:
  LGFX() {
    { // Bus
      auto cfg = _bus.config();
      cfg.spi_host   = VSPI_HOST;      // oder HSPI_HOST
      cfg.spi_mode   = 0;
      cfg.freq_write = 40000000;       // 40 MHz
      cfg.freq_read  = 16000000;
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
  tft.println("RadioGateway starting...");
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
  
  String entry = "[" + String(millis()/1000) + "] " + String(topic) + ": " + msg;
  lastMessages[msgIndex % 10] = entry;
  msgIndex++;
  lastMsgTime = millis();
  
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

String htmlHeader(const String& title) {
  return R"=====(
<!DOCTYPE html>
<html>
<head>
  <title>)=====" + title + R"=====(</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
    .container { max-width: 1000px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
    h1, h2, h3 { color: #333; }
    form { margin: 20px 0; }
    input, select, button { padding: 8px; margin: 5px; border: 1px solid #ccc; border-radius: 4px; }
    button { background: #4CAF50; color: white; cursor: pointer; }
    button:hover { background: #45a049; }
    .status-box { padding: 15px; margin: 10px 0; border-left: 4px solid #2196F3; background: #e7f3ff; }
    table { width: 100%; border-collapse: collapse; margin: 10px 0; }
    th, td { padding: 10px; border: 1px solid #ddd; text-align: left; }
    th { background-color: #f2f2f2; }
    .message { padding: 10px; margin: 5px 0; background: #f9f9f9; border-left: 3px solid #4CAF50; }
  </style>
</head>
<body>
  <div class="container">
    <h1>)=====" + title + R"=====(</h1>
)=====";
}

String htmlFooter() {
  return R"=====(
  </div>
</body>
</html>
)=====";
}

void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    if (!isAuthOk()) return;
    String html = htmlHeader("RadioGateway Dashboard");
    html += R"=====(
    <div class="status-box">
      <h3>Status Overview</h3>
      <p><strong>Device:</strong> )=====" + cfg.device + R"=====(</p>
      <p><strong>Location:</strong> )=====" + cfg.location + R"=====(</p>
    </div>
    <h2>Main Sections</h2>
    <ul>
      <li><a href="/dashboard">Dashboard</a> - System status overview</li>
      <li><a href="/wifi">WiFi Configuration</a> - AP/STA settings</li>
      <li><a href="/mqtt">MQTT Configuration</a> - Broker and topics</li>
      <li><a href="/sd">SD Card Management</a> - Files and storage</li>
      <li><a href="/devices">Connected Devices</a> - Client list</li>
      <li><a href="/config">System Configuration</a> - General settings</li>
    </ul>
  )=====";
    html += htmlFooter();
    server.send(200, "text/html", html);
  });
  
  server.on("/dashboard", HTTP_GET, []() {
    if (!isAuthOk()) return;
    String html = htmlHeader("Dashboard - RadioGateway");
    html += R"=====(
    <div class="status-box">
      <h3>Network Status</h3>
      <p><strong>WiFi Mode:</strong> )=====";
      
    if (cfg.ap_enabled && cfg.sta_enabled) {
      html += "AP+STA";
    } else if (cfg.ap_enabled) {
      html += "AP Only";
    } else if (cfg.sta_enabled) {
      html += "STA Only";
    } else {
      html += "Disabled";
    }
    
    html += R"=====(</p>
      <p><strong>AP IP:</strong> )=====" + WiFi.softAPIP().toString() + R"=====(</p>
)=====";
    
    if (WiFi.status() == WL_CONNECTED) {
      html += "      <p><strong>STA IP:</strong> " + WiFi.localIP().toString() + "</p>\n";
      html += "      <p><strong>Signal Strength:</strong> " + String(WiFi.RSSI()) + " dBm</p>\n";
    }
    
    html += R"=====(    </div>
    <div class="status-box">
      <h3>MQTT Status</h3>
      <p><strong>Connection:</strong> )=====" + String(mqttClient.connected() ? "Connected" : "Disconnected") + R"=====(</p>
      <p><strong>Broker:</strong> )=====" + cfg.mqtt_host + R"=====(:)=====" + String(cfg.mqtt_port) + R"=====(</p>
      <p><strong>Last Message:</strong> )=====" + String((millis() - lastMsgTime)/1000) + R"=====( seconds ago</p>
    </div>
    <div class="status-box">
      <h3>Storage</h3>
      <p><strong>SD Card:</strong> )=====" + String(sdOK ? "OK" : "ERROR") + R"=====(</p>
    </div>
    <div class="status-box">
      <h3>System Controls</h3>
      <form action="/api/reboot" method="POST">
        <button type="submit">Reboot Device</button>
      </form>
    </div>
  )=====";
    html += htmlFooter();
    server.send(200, "text/html", html);
  });
  
  server.on("/wifi", HTTP_GET, []() {
    if (!isAuthOk()) return;
    String html = htmlHeader("WiFi Configuration - RadioGateway");
    html += R"=====(
    <form action="/api/config/save" method="POST">
      <h3>WiFi Mode</h3>
      <select name="wifi_mode">
        <option value="2" )=====" + String(cfg.ap_enabled && !cfg.sta_enabled ? "selected" : "") + R"=====(>Access Point Only</option>
        <option value="1" )=====" + String(!cfg.ap_enabled && cfg.sta_enabled ? "selected" : "") + R"=====(>Station Only</option>
        <option value="3" )=====" + String(cfg.ap_enabled && cfg.sta_enabled ? "selected" : "") + R"=====(>Access Point + Station</option>
      </select>
      
      <h3>Access Point Settings</h3>
      <label>SSID:<br><input type="text" name="ap_ssid" value=")=====" + cfg.ap_ssid + R"=====("></label><br>
      <label>Password:<br><input type="password" name="ap_pass" value=")=====" + cfg.ap_pass + R"=====("></label><br>
      <label>Channel:<br><input type="number" name="ap_channel" value=")=====" + String(cfg.ap_channel) + R"=====(" min="1" max="13"></label><br>
      
      <h3>Station Settings</h3>
      <label>SSID:<br><input type="text" name="sta_ssid" value=")=====" + cfg.sta_ssid + R"=====("></label><br>
      <label>Password:<br><input type="password" name="sta_pass" value=")=====" + cfg.sta_pass + R"=====("></label><br>
      
      <button type="submit">Save Configuration</button>
    </form>
  )=====";
    html += htmlFooter();
    server.send(200, "text/html", html);
  });
  
  server.on("/mqtt", HTTP_GET, []() {
    if (!isAuthOk()) return;
    String html = htmlHeader("MQTT Configuration - RadioGateway");
    html += R"=====(
    <form action="/api/config/save" method="POST">
      <label><input type="checkbox" name="mqtt_enabled" )=====" + String(cfg.mqtt_enabled ? "checked" : "") + R"=====(> Enable MQTT</label><br><br>
      
      <label>Broker Host:<br><input type="text" name="mqtt_host" value=")=====" + cfg.mqtt_host + R"=====("></label><br>
      <label>Port:<br><input type="number" name="mqtt_port" value=")=====" + String(cfg.mqtt_port) + R"=====("></label><br>
      <label>Username:<br><input type="text" name="mqtt_user" value=")=====" + cfg.mqtt_user + R"=====("></label><br>
      <label>Password:<br><input type="password" name="mqtt_pass" value=")=====" + cfg.mqtt_pass + R"=====("></label><br>
      
      <h3>Topics</h3>
      <label>RX Topic (Subscribe):<br><input type="text" name="topic_rx" value=")=====" + cfg.topic_rx + R"=====("></label><br>
      <label>TX Topic (Publish):<br><input type="text" name="topic_tx" value=")=====" + cfg.topic_tx + R"=====("></label><br>
      <label>Status Topic:<br><input type="text" name="topic_status" value=")=====" + cfg.topic_status + R"=====("></label><br><br>
      
      <button type="submit">Save Configuration</button>
    </form>
    
    <h3>MQTT Controls</h3>
    <form action="/api/mqtt/connect" method="POST" style="display:inline;">
      <button type="submit">Connect to MQTT</button>
    </form>
    <form action="/api/mqtt/disconnect" method="POST" style="display:inline;">
      <button type="submit">Disconnect from MQTT</button>
    </form>
    
    <h3>Last Messages</h3>
    <div id="messages">
)=====";
  
  // Display last messages
  for (int i = 0; i < 10 && i < msgIndex; i++) {
    int idx = (msgIndex - 1 - i) % 10;
    if (lastMessages[idx].length() > 0) {
      html += "      <div class=\"message\">" + lastMessages[idx] + "</div>\n";
    }
  }
  
  html += R"=====(
    </div>
  )=====";
    html += htmlFooter();
    server.send(200, "text/html", html);
  });
  
  server.on("/sd", HTTP_GET, []() {
    if (!isAuthOk()) return;
    String html = htmlHeader("SD Card Management - RadioGateway");
    html += R"=====(
    <div class="status-box">
      <h3>SD Card Status</h3>
      <p><strong>Status:</strong> )=====" + String(sdOK ? "OK" : "ERROR") + R"=====(</p>
    </div>
    
    <h3>Files</h3>
    <table>
      <tr><th>Name</th><th>Size</th><th>Action</th></tr>
)=====";
  
  if (sdOK) {
    File root = SD.open("/");
    File file = root.openNextFile();
    while (file) {
      if (!file.isDirectory()) {
        html += "      <tr><td>" + String(file.name()) + "</td><td>" + String(file.size()) + " bytes</td>";
        html += "<td><a href=\"/api/sd/download?file=" + String(file.name()) + "\">Download</a></td></tr>\n";
      }
      file = root.openNextFile();
    }
  }
  
  html += R"=====(
    </table>
  )=====";
    html += htmlFooter();
    server.send(200, "text/html", html);
  });
  
  server.on("/devices", HTTP_GET, []() {
    if (!isAuthOk()) return;
    String html = htmlHeader("Connected Devices - RadioGateway");
    html += R"=====(
    <h3>AP Clients</h3>
    <table>
      <tr><th>MAC Address</th><th>IP Address</th></tr>
)=====";
  
  // Get AP clients (simplified version)
  html += "      <tr><td colspan='2'>Client listing requires ESP-IDF functions not shown here</td></tr>\n";
  
  html += R"=====(
    </table>
  )=====";
    html += htmlFooter();
    server.send(200, "text/html", html);
  });
  
  server.on("/config", HTTP_GET, []() {
    if (!isAuthOk()) return;
    String html = htmlHeader("System Configuration - RadioGateway");
    html += R"=====(
    <form action="/api/config/save" method="POST">
      <label>Device Name:<br><input type="text" name="device" value=")=====" + cfg.device + R"=====("></label><br>
      <label>Hostname:<br><input type="text" name="hostname" value=")=====" + cfg.hostname + R"=====("></label><br>
      <label>Location:<br><input type="text" name="location" value=")=====" + cfg.location + R"=====("></label><br>
      <label>Backlight Percent:<br><input type="number" name="bl_percent" value=")=====" + String(cfg.bl_percent) + R"=====(" min="0" max="100"></label><br>
      
      <button type="submit">Save Configuration</button>
    </form>
  )=====";
    html += htmlFooter();
    server.send(200, "text/html", html);
  });
  
  // API handlers
  server.on("/api/config/save", HTTP_POST, []() {
    if (!isAuthOk()) return;
    
    // Update configuration from form data
    if (server.hasArg("wifi_mode")) {
      int mode = server.arg("wifi_mode").toInt();
      cfg.ap_enabled = (mode == 2 || mode == 3);
      cfg.sta_enabled = (mode == 1 || mode == 3);
    }
    if (server.hasArg("ap_ssid")) cfg.ap_ssid = server.arg("ap_ssid");
    if (server.hasArg("ap_pass")) cfg.ap_pass = server.arg("ap_pass");
    if (server.hasArg("ap_channel")) cfg.ap_channel = server.arg("ap_channel").toInt();
    if (server.hasArg("sta_ssid")) cfg.sta_ssid = server.arg("sta_ssid");
    if (server.hasArg("sta_pass")) cfg.sta_pass = server.arg("sta_pass");
    
    if (server.hasArg("mqtt_enabled")) cfg.mqtt_enabled = true;
    else cfg.mqtt_enabled = false;
    if (server.hasArg("mqtt_host")) cfg.mqtt_host = server.arg("mqtt_host");
    if (server.hasArg("mqtt_port")) cfg.mqtt_port = server.arg("mqtt_port").toInt();
    if (server.hasArg("mqtt_user")) cfg.mqtt_user = server.arg("mqtt_user");
    if (server.hasArg("mqtt_pass")) cfg.mqtt_pass = server.arg("mqtt_pass");
    if (server.hasArg("topic_rx")) cfg.topic_rx = server.arg("topic_rx");
    if (server.hasArg("topic_tx")) cfg.topic_tx = server.arg("topic_tx");
    if (server.hasArg("topic_status")) cfg.topic_status = server.arg("topic_status");
    
    if (server.hasArg("device")) cfg.device = server.arg("device");
    if (server.hasArg("hostname")) cfg.hostname = server.arg("hostname");
    if (server.hasArg("location")) cfg.location = server.arg("location");
    if (server.hasArg("bl_percent")) cfg.bl_percent = server.arg("bl_percent").toInt();
    
    saveConfig();
    server.send(200, "text/plain", "Configuration saved successfully");
  });
  
  server.on("/api/mqtt/connect", HTTP_POST, []() {
    if (!isAuthOk()) return;
    reconnectMQTT();
    server.send(200, "text/plain", mqttClient.connected() ? "Connected to MQTT" : "Failed to connect to MQTT");
  });
  
  server.on("/api/mqtt/disconnect", HTTP_POST, []() {
    if (!isAuthOk()) return;
    mqttClient.disconnect();
    server.send(200, "text/plain", "Disconnected from MQTT");
  });
  
  server.on("/api/sd/download", HTTP_GET, []() {
    if (!isAuthOk()) return;
    
    if (server.hasArg("file")) {
      String filename = "/" + server.arg("file");
      File file = SD.open(filename, FILE_READ);
      if (!file) {
        server.send(404, "text/plain", "File not found");
        return;
      }
      server.streamFile(file, "application/octet-stream");
      file.close();
      return;
    }
    
    server.send(400, "text/plain", "Missing file parameter");
  });
  
  server.on("/api/reboot", HTTP_POST, []() {
    if (!isAuthOk()) return;
    server.send(200, "text/plain", "Rebooting...");
    ESP.restart();
  });
  
  server.onNotFound([]() {
    server.send(404, "text/plain", "File Not Found");
  });
}

// ===================== HELPER FUNCTIONS =====================