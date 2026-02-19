
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_system.h>

// Wi‑Fi credentials
const char* ssid = "Tecknoserve";
const char* password = "09ea4431ae";

// TCS3430 I2C address and registers
#define TCS3430_ADDR 0x39
#define CMD_BIT      0x80
#define ENABLE_REG   0x00
#define ATIME_REG    0x01
#define CONTROL_REG  0x0F
#define XDATA_L      0x1A   // CH3DATAL (0x9A). AMUX=0 -> X, AMUX=1 -> IR2
#define YDATA_L      0x16   // CH1DATAL (0x96) -> Y
#define ZDATA_L      0x14   // CH0DATAL (0x94) -> Z
#define IR1DATA_L    0x18   // CH2DATAL (0x98) -> IR1

#define CFG3_REG  0x2B   // 0xAB on device (CFG3: High Gain bit)



// Additional DFRobot-inspired register definitions
#define ID_REG       0x12   // Device ID register
#define REVID_REG    0x11   // Revision ID register
#define STATUS_REG   0x13   // Status register
#define CFG1_REG     0x10   // Configuration 1 register
#define CFG2_REG     0x1F   // Configuration 2 register
#define AZ_CONFIG_REG 0x56  // Auto-zero configuration register
#define PERS_REG     0x0C   // Persistence register

// Device identification values
#define TCS3430_DEVICE_ID    0xDC
#define TCS3430_REVISION_ID  0x41

// Status register bits
#define STATUS_ASAT_MASK     0x80  // ALS Saturation bit
#define STATUS_AINT_MASK     0x10  // ALS Interrupt bit

// CFG2 register values for high gain
#define HGAIN_DISABLE        0x04
#define HGAIN_ENABLE         0x14

// Integration time & gain settings (now adjustable via web interface)
uint8_t INTEG_TIME = 0xF6; // ~25 ms (reduced for very bright LED conditions)
uint8_t GAIN       = 0x01; // x4
bool highGainEnabled = false;

// AMS k-factor calibration values (applied as divisors)
float kX = 1.0f;  // X correction factor (optimized for LED)
float kY = 1.0f;  // Y correction factor (optimized for LED)  
float kZ = 1.0f;  // Z correction factor (unused in 2-point mode)

// Lab color space conversion functions
// D65 white point values
const float Xn = 95.047f;  // D65 white point X
const float Yn = 100.000f; // D65 white point Y  
const float Zn = 108.883f; // D65 white point Z

float labF(float t) {
  if (t > 0.008856f) {
    return pow(t, 1.0f/3.0f);
  } else {
    return (7.787f * t) + (16.0f/116.0f);
  }
}

// XYZ -> Lab with configurable reference white (Xn,Yn,Zn)
void XYZtoLabRef(float X, float Y, float Z, float XnRef, float YnRef, float ZnRef, float &L, float &a, float &b) {
  // Avoid divide-by-zero and negative values
  X = max(0.0f, X); Y = max(0.0f, Y); Z = max(0.0f, Z);
  XnRef = max(1e-6f, XnRef); YnRef = max(1e-6f, YnRef); ZnRef = max(1e-6f, ZnRef);

  float fx = labF(X / XnRef);
  float fy = labF(Y / YnRef);
  float fz = labF(Z / ZnRef);

  L = (116.0f * fy) - 16.0f;
  a = 500.0f * (fx - fy);
  b = 200.0f * (fy - fz);
}

// Default XYZ -> Lab using D65 reference white
void XYZtoLab(float X, float Y, float Z, float &L, float &a, float &b) {
  XYZtoLabRef(X, Y, Z, Xn, Yn, Zn, L, a, b);
}
float xOffsetAdjust = 0.0;
float yOffsetAdjust = 0.0;
float zOffsetAdjust = 0.0;

// WiFi configuration
int wifiMode = 0;
bool settingsLoadedOk = false;
 // 0=AP only, 1=Station only, 2=Dual mode
String wifiSSID = "";
String wifiPassword = "";
String currentIP = "192.168.4.1";
bool stationConnected = false;

// OLED Display configuration
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

AsyncWebServer server(80);
volatile bool loopSerialEnabled = false;  // Enable/disable loop Serial debug via /loopserial/on|off


// Calibration raw samples - BCRA white tile calibration
uint16_t rawWhiteX = 2707, rawWhiteY = 7942, rawWhiteZ = 560;   // White reference (CORRECTED with proper diffuser installation)
uint16_t rawBlackX = 223,  rawBlackY = 579,  rawBlackZ = 43;   // Black reference (CORRECTED with proper diffuser)
uint16_t rawGreenX = 542,  rawGreenY = 2061,  rawGreenZ = 128;  // Green tile reference (CORRECTED with proper diffuser)
uint16_t rawRedX = 344,    rawRedY = 1415,   rawRedZ = 574;   // Red tile reference (CORRECTED with proper diffuser)
uint16_t rawOrangeX = 438, rawOrangeY = 3514, rawOrangeZ = 595; // Orange tile reference (CORRECTED with proper diffuser)
uint16_t rawYellowX = 522, rawYellowY = 5975, rawYellowZ = 489; // Yellow tile reference (CORRECTED with proper diffuser)
uint16_t rawGreyX = 892,   rawGreyY = 2512,  rawGreyZ = 174;   // Mid grey tile reference (CORRECTED with proper diffuser)
uint16_t rawPinkX = 523,   rawPinkY = 1545,   rawPinkZ = 297;   // Deep pink tile reference (CORRECTED with proper diffuser)

// IR channels saved per tile (persisted to SPIFFS)
uint16_t rawWhiteIR1 = 0, rawWhiteIR2 = 0;
uint16_t rawBlackIR1 = 0, rawBlackIR2 = 0;
uint16_t rawGreenIR1 = 0, rawGreenIR2 = 0;
uint16_t rawRedIR1   = 0, rawRedIR2   = 0;
uint16_t rawOrangeIR1= 0, rawOrangeIR2= 0;
uint16_t rawYellowIR1= 0, rawYellowIR2= 0;
uint16_t rawGreyIR1  = 0, rawGreyIR2  = 0;
uint16_t rawPinkIR1  = 0, rawPinkIR2  = 0;

// Spectrophotometer reference values (D65, 10°) - BCRA tiles for real-time comparison
struct SpectroTarget {
  float X, Y, Z, x, y;
  const char* name;
};

SpectroTarget spectroTargets[] = {
  {88.68, 88.89, 84.82, 0.3382, 0.3439, "White"},
  {64.77, 68.63, 10.42, 0.4510, 0.4778, "Yellow"},
  {3.10, 3.27, 3.12, 0.3236, 0.3413, "Black"},
  {17.89, 31.52, 6.76, 0.3195, 0.5627, "Green"},
  {32.45, 19.54, 4.06, 0.5787, 0.3486, "Red"},
  {50.86, 47.27, 7.72, 0.4814, 0.4474, "Orange"},
  {39.13, 40.0, 40.84, 0.3262, 0.3334, "Mid Grey"},  // Updated to match actual tile
  {25.41, 14.74, 15.20, 0.4590, 0.2665, "Deep Pink"},
};
const int numSpectroTargets = sizeof(spectroTargets) / sizeof(SpectroTarget);

// Calibration: green's optimal Y and Z coefficients for best color separation
// Matrix format: [TCS_X, TCS_Y, TCS_Z, 1] * T = [CIE_X, CIE_Y, CIE_Z]
float calibMatrix[4][3] = {
  {  0.0327f,   0.0000f,   0.0000f},   // X coefficient (from white, unchanged)
  {  0.0000f,   0.0153f,   0.0000f},   // Y coefficient (green optimal)
  {  0.0000f,   0.0000f,   0.0520f},   // Z coefficient (green optimal)
  {  0.0000f,   0.0000f,   0.0000f}    // No offset
};

// Settings persistence
const char* SETTINGS_FILE = "/settings.json";

// Save settings to SPIFFS
bool saveSettings() {
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS for settings save");
    return false;
  }
  
  File file = SPIFFS.open(SETTINGS_FILE, "w");
  if (!file) {
    Serial.println("Failed to open settings file for writing");
    return false;
  }
  
  // Create JSON settings
  String json = "{";
  json += "\"kX\":" + String(kX, 4) + ",";
  json += "\"kY\":" + String(kY, 4) + ",";
  json += "\"kZ\":" + String(kZ, 4) + ",";
  json += "\"xOffset\":" + String(xOffsetAdjust, 1) + ",";
  json += "\"yOffset\":" + String(yOffsetAdjust, 1) + ",";
  json += "\"zOffset\":" + String(zOffsetAdjust, 1) + ",";
  json += "\"gain\":" + String(GAIN) + ",";
  json += "\"integrationTime\":" + String(INTEG_TIME) + ",";
  json += "\"highGain\":" + String(highGainEnabled ? "true" : "false") + ",";
  json += "\"wifiMode\":" + String(wifiMode) + ",";
  json += "\"wifiSSID\":\"" + wifiSSID + "\",";
  json += "\"wifiPassword\":\"" + wifiPassword + "\",";
  json += "\"whiteX\":" + String(rawWhiteX) + ",";
json += "\"whiteY\":" + String(rawWhiteY) + ",";
json += "\"whiteZ\":" + String(rawWhiteZ) + ",";
json += "\"whiteIR1\":" + String(rawWhiteIR1) + ",";
json += "\"whiteIR2\":" + String(rawWhiteIR2) + ",";

json += "\"blackX\":" + String(rawBlackX) + ",";
json += "\"blackY\":" + String(rawBlackY) + ",";
json += "\"blackZ\":" + String(rawBlackZ) + ",";
json += "\"blackIR1\":" + String(rawBlackIR1) + ",";
json += "\"blackIR2\":" + String(rawBlackIR2) + ",";

json += "\"greenX\":" + String(rawGreenX) + ",";
json += "\"greenY\":" + String(rawGreenY) + ",";
json += "\"greenZ\":" + String(rawGreenZ) + ",";
json += "\"greenIR1\":" + String(rawGreenIR1) + ",";
json += "\"greenIR2\":" + String(rawGreenIR2) + ",";

json += "\"redX\":" + String(rawRedX) + ",";
json += "\"redY\":" + String(rawRedY) + ",";
json += "\"redZ\":" + String(rawRedZ) + ",";
json += "\"redIR1\":" + String(rawRedIR1) + ",";
json += "\"redIR2\":" + String(rawRedIR2) + ",";

json += "\"orangeX\":" + String(rawOrangeX) + ",";
json += "\"orangeY\":" + String(rawOrangeY) + ",";
json += "\"orangeZ\":" + String(rawOrangeZ) + ",";
json += "\"orangeIR1\":" + String(rawOrangeIR1) + ",";
json += "\"orangeIR2\":" + String(rawOrangeIR2) + ",";

json += "\"yellowX\":" + String(rawYellowX) + ",";
json += "\"yellowY\":" + String(rawYellowY) + ",";
json += "\"yellowZ\":" + String(rawYellowZ) + ",";
json += "\"yellowIR1\":" + String(rawYellowIR1) + ",";
json += "\"yellowIR2\":" + String(rawYellowIR2) + ",";

json += "\"greyX\":" + String(rawGreyX) + ",";
json += "\"greyY\":" + String(rawGreyY) + ",";
json += "\"greyZ\":" + String(rawGreyZ) + ",";
json += "\"greyIR1\":" + String(rawGreyIR1) + ",";
json += "\"greyIR2\":" + String(rawGreyIR2) + ",";

json += "\"pinkX\":" + String(rawPinkX) + ",";
json += "\"pinkY\":" + String(rawPinkY) + ",";
json += "\"pinkZ\":" + String(rawPinkZ) + ",";
json += "\"pinkIR1\":" + String(rawPinkIR1) + ",";
json += "\"pinkIR2\":" + String(rawPinkIR2) + ",";
json += "\"version\":\"1.02 - ir\",";
json += "\"timestamp\":" + String(millis());
json += "}";
  
  file.print(json);
  file.close();
  
  Serial.println("Settings saved successfully");
  Serial.println("Saved JSON: " + json.substring(0, 200) + "...");
  return true;
}

// Simple JSON value extractor
String extractJSONValue(const String& json, const String& key) {
  String searchKey = "\"" + key + "\":";
  int startPos = json.indexOf(searchKey);
  if (startPos == -1) return "";
  
  startPos += searchKey.length();
  
  // Skip whitespace
  while (startPos < json.length() && (json.charAt(startPos) == ' ' || json.charAt(startPos) == '\t')) {
    startPos++;
  }
  
  int endPos = startPos;
  bool inString = false;
  
  if (json.charAt(startPos) == '"') {
    // String value
    inString = true;
    endPos = startPos + 1;
    while (endPos < json.length() && json.charAt(endPos) != '"') {
      endPos++;
    }
    return json.substring(startPos + 1, endPos);
  } else {
    // Numeric or boolean value
    while (endPos < json.length() && json.charAt(endPos) != ',' && json.charAt(endPos) != '}') {
      endPos++;
    }
    return json.substring(startPos, endPos);
  }
}

// Load settings from SPIFFS
bool loadSettings() {
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS for settings load");
    return false;
  }
  
  if (!SPIFFS.exists(SETTINGS_FILE)) {
    Serial.println("Settings file not found, using defaults");
    return false;
  }
  
  File file = SPIFFS.open(SETTINGS_FILE, "r");
  if (!file) {
    Serial.println("Failed to open settings file for reading");
    return false;
  }
  
  String json = file.readString();
  file.close();
  
  Serial.println("Loading settings from JSON...");
  Serial.println("JSON preview: " + json.substring(0, 200) + "...");
  
  // Parse JSON and restore settings
  String value;
  
  value = extractJSONValue(json, "kX");
  if (value.length() > 0) kX = value.toFloat();
  
  value = extractJSONValue(json, "kY");
  if (value.length() > 0) kY = value.toFloat();
  
  value = extractJSONValue(json, "kZ");
  if (value.length() > 0) kZ = value.toFloat();
  
  value = extractJSONValue(json, "xOffset");
  if (value.length() > 0) xOffsetAdjust = value.toFloat();
  
  value = extractJSONValue(json, "yOffset");
  if (value.length() > 0) yOffsetAdjust = value.toFloat();
  
  value = extractJSONValue(json, "zOffset");
  if (value.length() > 0) zOffsetAdjust = value.toFloat();
  
  value = extractJSONValue(json, "gain");
  if (value.length() > 0) GAIN = (uint8_t)value.toInt();
  
  value = extractJSONValue(json, "integrationTime");
  if (value.length() > 0) INTEG_TIME = (uint8_t)value.toInt();
  
  value = extractJSONValue(json, "highGain");
  if (value.length() > 0) highGainEnabled = (value == "true");
  
  // Load calibration points
  value = extractJSONValue(json, "whiteX");
  if (value.length() > 0) rawWhiteX = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "whiteY");
  if (value.length() > 0) rawWhiteY = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "whiteZ");
  if (value.length() > 0) rawWhiteZ = (uint16_t)value.toInt();

  value = extractJSONValue(json, "whiteIR1");
if (value.length() > 0) rawWhiteIR1 = (uint16_t)value.toInt();

value = extractJSONValue(json, "whiteIR2");
if (value.length() > 0) rawWhiteIR2 = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "blackX");
  if (value.length() > 0) rawBlackX = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "blackY");
  if (value.length() > 0) rawBlackY = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "blackZ");
  if (value.length() > 0) rawBlackZ = (uint16_t)value.toInt();

  value = extractJSONValue(json, "blackIR1");
  if (value.length() > 0) rawBlackIR1 = (uint16_t)value.toInt();

  value = extractJSONValue(json, "blackIR2");
  if (value.length() > 0) rawBlackIR2 = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "greenX");
  if (value.length() > 0) rawGreenX = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "greenY");
  if (value.length() > 0) rawGreenY = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "greenZ");
  if (value.length() > 0) rawGreenZ = (uint16_t)value.toInt();

  value = extractJSONValue(json, "greenIR1");
  if (value.length() > 0) rawGreenIR1 = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "greenIR2");
  if (value.length() > 0) rawGreenIR2 = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "redX");
  if (value.length() > 0) rawRedX = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "redY");
  if (value.length() > 0) rawRedY = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "redZ");
  if (value.length() > 0) rawRedZ = (uint16_t)value.toInt();

  value = extractJSONValue(json, "redIR1");
  if (value.length() > 0) rawRedIR1 = (uint16_t)value.toInt();

  value = extractJSONValue(json, "redIR2");
  if (value.length() > 0) rawRedIR2 = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "orangeX");
  if (value.length() > 0) rawOrangeX = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "orangeY");
  if (value.length() > 0) rawOrangeY = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "orangeZ");
  if (value.length() > 0) rawOrangeZ = (uint16_t)value.toInt();

  value = extractJSONValue(json, "orangeIR1");
  if (value.length() > 0) rawOrangeIR1 = (uint16_t)value.toInt();

  value = extractJSONValue(json, "orangeIR2");
  if (value.length() > 0) rawOrangeIR2 = (uint16_t)value.toInt();

  value = extractJSONValue(json, "yellowX");
  if (value.length() > 0) rawYellowX = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "yellowY");
  if (value.length() > 0) rawYellowY = (uint16_t)value.toInt();  

  value = extractJSONValue(json, "yellowZ");
  if (value.length() > 0) rawYellowZ = (uint16_t)value.toInt();

  value = extractJSONValue(json, "yellowIR1");
  if (value.length() > 0) rawYellowIR1 = (uint16_t)value.toInt();

    value = extractJSONValue(json, "yellowIR2");
  if (value.length() > 0) rawYellowIR2 = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "greyX");
  if (value.length() > 0) rawGreyX = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "greyY");
  if (value.length() > 0) rawGreyY = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "greyZ");
  if (value.length() > 0) rawGreyZ = (uint16_t)value.toInt();

  value = extractJSONValue(json, "greyIR1");
  if (value.length() > 0) rawGreyIR1 = (uint16_t)value.toInt();

  value = extractJSONValue(json, "greyIR2");
  if (value.length() > 0) rawGreyIR2 = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "pinkX");
  if (value.length() > 0) rawPinkX = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "pinkY");
  if (value.length() > 0) rawPinkY = (uint16_t)value.toInt();
  
  value = extractJSONValue(json, "pinkZ");
  if (value.length() > 0) rawPinkZ = (uint16_t)value.toInt();

  value = extractJSONValue(json, "pinkIR1");
  if (value.length() > 0) rawPinkIR1 = (uint16_t)value.toInt();

  value = extractJSONValue(json, "pinkIR2");
  if (value.length() > 0) rawPinkIR2 = (uint16_t)value.toInt();
  
  // Load WiFi settings
  value = extractJSONValue(json, "wifiMode");
  if (value.length() > 0) wifiMode = value.toInt();
  
  value = extractJSONValue(json, "wifiSSID");
  if (value.length() > 0) wifiSSID = value;
  
  value = extractJSONValue(json, "wifiPassword");
  if (value.length() > 0) wifiPassword = value;
  
  Serial.println("Settings loaded successfully!");
  Serial.printf("K-factors: kX=%.4f, kY=%.4f, kZ=%.4f\n", kX, kY, kZ);
  Serial.printf("Offsets: X=%.1f, Y=%.1f, Z=%.1f\n", xOffsetAdjust, yOffsetAdjust, zOffsetAdjust);
  Serial.printf("Sensor: Gain=%d, Integration=0x%02X, HighGain=%s\n", GAIN, INTEG_TIME, highGainEnabled ? "true" : "false");
  Serial.printf("WiFi Mode: %d, SSID: %s\n", wifiMode, wifiSSID.c_str());
  
  return true;
}


void startAccessPoint() {
  Serial.println("[AP] step 0: startAccessPoint()");

  Serial.println("[AP] step 1: persistent(false)");
  WiFi.persistent(false);
  delay(50);

  Serial.println("[AP] step 2: mode(WIFI_AP)");
  WiFi.mode(WIFI_AP);
  delay(200);

  Serial.println("[AP] step 3: softAP()");
  bool ok = WiFi.softAP("TCS3430-ColorSensor", "");

  Serial.print("[AP] step 4: softAP returned = ");
  Serial.println(ok ? "OK" : "FAIL");

  Serial.print("[AP] step 5: AP SSID = ");
  Serial.println(WiFi.softAPSSID());

  Serial.print("[AP] step 6: AP IP = ");

  Serial.println("[AP] step 7: done");
}

void connectToWiFi() {
  if (wifiSSID.length() == 0) {
    Serial.println("No WiFi credentials configured");
    return;
  }
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  
  Serial.printf("Connecting to WiFi: %s", wifiSSID.c_str());
  int attempts = 0;
  
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    stationConnected = true;
    currentIP = WiFi.localIP().toString();
    Serial.println();
    Serial.printf("✓ Connected to %s\n", wifiSSID.c_str());
    Serial.printf("Station IP: %s\n", currentIP.c_str());
    
    // Setup mDNS for easy discovery
    if (MDNS.begin("tcs3430")) {
      Serial.println("✓ mDNS responder started: http://tcs3430.local");
      MDNS.addService("http", "tcp", 80);
    }
  } else {
    Serial.println();
    Serial.println("✗ WiFi connection failed");
    stationConnected = false;
  }
}

void setupWiFi() {
  switch (wifiMode) {
    case 0: // AP only
      startAccessPoint();
      currentIP = WiFi.softAPIP().toString();
      break;
      
    case 1: // Station only
      connectToWiFi();
      if (!stationConnected) {
        Serial.println("Station failed, falling back to AP mode");
        startAccessPoint();
        currentIP = WiFi.softAPIP().toString();
      }
      break;
      
    // case 2: // Dual mode
   
      Serial.println(WiFi.softAPIP());
      
    
    
    case 2: // Dual mode
  Serial.println("[WiFi] Entering Dual Mode");

  WiFi.mode(WIFI_AP_STA);
  delay(100);

  bool apOk = WiFi.softAP("TCS3430-ColorSensor", "");  // DO NOT force channel
  Serial.print("AP started: ");
  Serial.println(apOk ? "YES" : "NO");

  Serial.print("AP SSID: ");
  Serial.println(WiFi.softAPSSID());

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  Serial.print("WiFi Mode after AP start: ");
  Serial.println((int)WiFi.getMode());

  if (wifiSSID.length() > 0) {

    // Trim to avoid invisible trailing spaces from saved JSON
    String ssidTrim = wifiSSID; ssidTrim.trim();
    String passTrim = wifiPassword; passTrim.trim();

    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);

    WiFi.begin(ssidTrim.c_str(), passTrim.c_str());
    Serial.printf("Connecting to WiFi (STA): %s\n", ssidTrim.c_str());

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      stationConnected = true;
      currentIP = WiFi.localIP().toString();

      Serial.println();
      Serial.printf("✓ Also connected to %s\n", wifiSSID.c_str());

      Serial.print("Station IP: ");
      Serial.println(WiFi.localIP());

      Serial.print("WiFi Mode after STA connect: ");
      Serial.println((int)WiFi.getMode());

      if (MDNS.begin("tcs3430")) {
        Serial.println("✓ mDNS responder started: http://tcs3430.local");
        MDNS.addService("http", "tcp", 80);
      }

    } else {
      Serial.println();
      Serial.println("Station connection failed, AP still available");
      currentIP = WiFi.softAPIP().toString();
    }

  } else {
    currentIP = WiFi.softAPIP().toString();
  }

  break;
  }
}

// Apply real-time calibration adjustments to raw sensor values
// AMS k-factor calibration adjustment (division method)
void applyAMSCalibration(float &x, float &y, float &z) {
  // Apply offsets first (if any)
  x = x + xOffsetAdjust;
  y = y + yOffsetAdjust;
  z = z + zOffsetAdjust;
  
  // Apply AMS k-factor corrections (division method)
  x = x / kX;
  y = y / kY;
  z = z / kZ;
}

// Find closest spectrophotometer target for comparison using Lab color space
int findClosestSpectroTarget(float X, float Y, float Z) {
  int closest = 0;
  float minDistance = 999999.0;
  
  // Convert measured XYZ to Lab
  float L_measured, a_measured, b_measured;
  XYZtoLab(X, Y, Z, L_measured, a_measured, b_measured);
  
  for(int i = 0; i < numSpectroTargets; i++) {
    // Convert target XYZ to Lab for comparison
    float L_target, a_target, b_target;
    XYZtoLab(spectroTargets[i].X, spectroTargets[i].Y, spectroTargets[i].Z, L_target, a_target, b_target);
    
    // Use Lab distance for better perceptual color matching
    float dL = L_measured - L_target;
    float da = a_measured - a_target;
    float db = b_measured - b_target;
    float distance = sqrt(dL*dL + da*da + db*db);
    
    if(distance < minDistance) {
      minDistance = distance;
      closest = i;
    }
  }
  return closest;
}

// Transform TCS3430 raw values to calibrated CIE XYZ
void transformToXYZ(uint16_t rawX, uint16_t rawY, uint16_t rawZ, float &X, float &Y, float &Z) {
  X = calibMatrix[0][0] * rawX + calibMatrix[1][0] * rawY + calibMatrix[2][0] * rawZ + calibMatrix[3][0];
  Y = calibMatrix[0][1] * rawX + calibMatrix[1][1] * rawY + calibMatrix[2][1] * rawZ + calibMatrix[3][1];
  Z = calibMatrix[0][2] * rawX + calibMatrix[1][2] * rawY + calibMatrix[2][2] * rawZ + calibMatrix[3][2];
}


// --- Calibration helpers (AMS-style 2-point in XYZ space) ---

static inline float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

// McCamy CCT approximation from chromaticity (x,y). Returns 0 if invalid.
float computeCCT_McCamy(float x, float y) {
  if (x <= 0.01f || y <= 0.01f || x >= 0.9f || y >= 0.9f) return 0.0f;
  float denom = (0.1858f - y);
  if (fabsf(denom) < 1e-6f) return 0.0f;
  float n = (x - 0.332f) / denom;
  float CCT = 437.0f*n*n*n + 3601.0f*n*n + 6861.0f*n + 5517.0f;
  if (CCT < 100.0f || CCT > 100000.0f || isnan(CCT) || isinf(CCT)) return 0.0f;
  return CCT;
}

// Compute calibrated XYZ using 2-point calibration in XYZ space.
// - RawXYZ is produced by transformToXYZ()
// - Black/White calibration points are raw counts stored in rawBlack* / rawWhite*
// - Targets are spectroTargets: White (index 0) and Black (index 2)
void computeCalXYZ_2Point(float X_rawXYZ, float Y_rawXYZ, float Z_rawXYZ,
                          float &X_calXYZ, float &Y_calXYZ, float &Z_calXYZ) {
  // Transform stored calibration points to XYZ space
  float Xb, Yb, Zb, Xw, Yw, Zw;
  transformToXYZ(rawBlackX, rawBlackY, rawBlackZ, Xb, Yb, Zb);
  transformToXYZ(rawWhiteX, rawWhiteY, rawWhiteZ, Xw, Yw, Zw);

  // Spectro targets (D65, 10°) for Black and White tiles
  const float Xt_b = spectroTargets[2].X, Yt_b = spectroTargets[2].Y, Zt_b = spectroTargets[2].Z;
  const float Xt_w = spectroTargets[0].X, Yt_w = spectroTargets[0].Y, Zt_w = spectroTargets[0].Z;

  // Per-channel scale to map (White-Black) in sensor XYZ space to (White-Black) in spectro XYZ space
  float denomX = (Xw - Xb);
  float denomY = (Yw - Yb);
  float denomZ = (Zw - Zb);

  float scaleX = (fabsf(denomX) > 1e-6f) ? ((Xt_w - Xt_b) / denomX) : 1.0f;
  float scaleY = (fabsf(denomY) > 1e-6f) ? ((Yt_w - Yt_b) / denomY) : 1.0f;
  float scaleZ = (fabsf(denomZ) > 1e-6f) ? ((Zt_w - Zt_b) / denomZ) : 1.0f;

  // Apply 2-point calibration
  X_calXYZ = (X_rawXYZ - Xb) * scaleX + Xt_b;
  Y_calXYZ = (Y_rawXYZ - Yb) * scaleY + Yt_b;
  Z_calXYZ = (Z_rawXYZ - Zb) * scaleZ + Zt_b;

  // Keep non-negative and avoid zeros (for xy/Lab math stability)
  X_calXYZ = max(0.001f, X_calXYZ);
  Y_calXYZ = max(0.001f, Y_calXYZ);
  Z_calXYZ = max(0.001f, Z_calXYZ);
}

// Known target XYZ values (your reference)
float targetWhiteX = 0.0f, targetWhiteY = 0.0f, targetWhiteZ = 0.0f;
float targetBlackX = 0.0f, targetBlackY = 0.0f, targetBlackZ = 0.0f;
// Calibration factors
float calibOffsetX, calibOffsetY, calibOffsetZ;
float calibScaleX,  calibScaleY,  calibScaleZ;

// Main navigation page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>TCS3430 Professional Color Measurement System</title>
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; }
    .container { max-width: 800px; margin: 0 auto; }
    .header { text-align: center; margin-bottom: 30px; }
    .header h1 { font-size: 2.5em; margin: 10px 0; text-shadow: 2px 2px 4px rgba(0,0,0,0.3); }
    .header p { font-size: 1.1em; opacity: 0.9; }
    .menu-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 20px; margin-bottom: 30px; }
    .menu-card { background: rgba(255,255,255,0.1); border-radius: 15px; padding: 25px; text-decoration: none; color: white; transition: all 0.3s ease; backdrop-filter: blur(10px); border: 1px solid rgba(255,255,255,0.2); }
    .menu-card:hover { transform: translateY(-5px); background: rgba(255,255,255,0.2); box-shadow: 0 10px 25px rgba(0,0,0,0.2); }
    .menu-card h3 { margin: 0 0 15px 0; font-size: 1.4em; }
    .menu-card p { margin: 0; opacity: 0.8; line-height: 1.4; }
    .status-panel { background: rgba(255,255,255,0.1); border-radius: 15px; padding: 20px; margin-top: 30px; backdrop-filter: blur(10px); }
    .status-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; }
    .status-item { background: rgba(255,255,255,0.1); padding: 15px; border-radius: 10px; text-align: center; }
    .status-value { font-size: 1.5em; font-weight: bold; margin-top: 5px; }
    .footer { text-align: center; margin-top: 30px; opacity: 0.7; font-size: 0.9em; }
    @media (max-width: 600px) { .menu-grid { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>🌈 TCS3430 Color Measurement System</h1>
      <p>Professional XYZ Spectrophotometer-Grade Accuracy | Access Point: <span id="url">192.168.4.1</span></p>
    </div>

    <div class="menu-grid">
      <a href="/measurement" class="menu-card">
        <h3>📊 Live Measurement</h3>
        <p>Real-time color measurement with CIE 1931 chromaticity diagram, XYZ values, and spectrophotometer comparison</p>
      </a>

      <a href="/configuration" class="menu-card">
        <h3>Configuration</h3>
        <p>Adjust sensor parameters: gain, integration time, scale factors, and real-time calibration settings</p>
      </a>

      <a href="/calibration" class="menu-card">
        <h3>Calibration</h3>
        <p>BCRA tile calibration system, reference point setting, and spectrophotometer validation</p>
      </a>

      <a href="/diagnostics" class="menu-card">
        <h3>Diagnostics</h3>
        <p>System status, sensor validation, I2C communication check, and performance monitoring</p>
      </a>
      
      <a href="/wifi" class="menu-card">
        <h3>WiFi Manager</h3>
        <p>Network configuration, connection modes, WiFi scanning, and connectivity settings</p>
      </a>
    </div>

    <div class="status-panel">
      <h3 style="margin-top: 0; text-align: center;">Current System Status</h3>
      <div class="status-grid">
        <div class="status-item">
          <div>Sensor Status</div>
          <div class="status-value" id="sensorStatus">Checking...</div>
        </div>
        <div class="status-item">
          <div>Last Reading</div>
          <div class="status-value" id="lastReading">Loading...</div>
        </div>
        <div class="status-item">
          <div>Accuracy</div>
          <div class="status-value">±0.08 XYZ</div>
        </div>
        <div class="status-item">
          <div>Calibration</div>
          <div class="status-value">Professional</div>
        </div>
      </div>
    </div>

    <div class="footer">
      <p>TCS3430 Professional Color Measurement System | Spectrophotometer-Grade Precision</p>
      <p>Build Date: January 26, 2026 | WiFi: TCS3430-ColorSensor</p>
    </div>
  </div>
<script>
  document.getElementById('url').textContent = window.location.host || '192.168.4.1';
  
  // Update system status periodically
  async function updateStatus() {
    try {
      const statusRes = await fetch('/status');
      const statusData = await statusRes.json();
      
      const colorRes = await fetch('/color');
      const colorData = await colorRes.json();
      
      // Update sensor status
      const sensorOk = statusData.deviceValid && !colorData.saturated;
      document.getElementById('sensorStatus').textContent = sensorOk ? '✅ Online' : '⚠️ Issue';
      document.getElementById('sensorStatus').style.color = sensorOk ? '#4CAF50' : '#FF9800';
      
      // Update last reading
      const reading = `X:${colorData.X.toFixed(2)} Y:${colorData.Y.toFixed(2)} Z:${colorData.Z.toFixed(2)}`;
      document.getElementById('lastReading').textContent = reading;
      
    } catch (error) {
      console.log('Status update failed:', error);
      document.getElementById('sensorStatus').textContent = '❌ Error';
      document.getElementById('sensorStatus').style.color = '#f44336';
      document.getElementById('lastReading').textContent = 'No Data';
    }
  }
  
  // Initial status update and periodic refresh
  updateStatus();
  setInterval(updateStatus, 3000);
</script>
</body>
</html>
)rawliteral";

// Read single byte from a register
uint8_t readSingleByte(uint8_t reg) {
  Wire.beginTransmission(TCS3430_ADDR);
  Wire.write(CMD_BIT | reg);
  if (Wire.endTransmission(true) != 0) return 0;
  uint8_t cnt = Wire.requestFrom((uint8_t)TCS3430_ADDR, (uint8_t)1);
  if (cnt < 1) return 0;
  return Wire.read();
}

// Write single byte to a register
void writeSingleByte(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TCS3430_ADDR);
  Wire.write(CMD_BIT | reg);
  Wire.write(value);
  Wire.endTransmission(true);
}



static bool DEBUG_IR2_SERIAL = false;

void setAMUX(bool ir2Mode) {
  uint8_t before = readSingleByte(CFG1_REG);

  uint8_t cfg1 = before;
  if (ir2Mode) cfg1 |= (1 << 3);
  else         cfg1 &= ~(1 << 3);

  writeSingleByte(CFG1_REG, cfg1);
  uint8_t after = readSingleByte(CFG1_REG);

  if (DEBUG_IR2_SERIAL) {
    Serial.printf("[AMUX] set %s | CFG1 before=0x%02X after=0x%02X\n",
                  ir2Mode ? "IR2" : "X",
                  before, after);
  }
}

// Set AGAIN (gain) in CFG1 bits [1:0]. 0=1x, 1=4x, 2=16x, 3=64x
void setGain(uint8_t gainCode) {
  gainCode &= 0x03;
  uint8_t cfg1 = readSingleByte(CFG1_REG);   // CFG1_REG is 0x10 (-> 0x90 with CMD_BIT)
  cfg1 = (cfg1 & ~0x03) | gainCode;          // update AGAIN bits only
  writeSingleByte(CFG1_REG, cfg1);
}

uint16_t read16(uint8_t reg);

 static uint32_t integrationMsFromATIME(uint8_t atime)
{
  const float step_ms = 2.78f;
  uint32_t cycles = (uint32_t)(256 - atime);
  uint32_t ms = (uint32_t)(cycles * step_ms);

  if (ms < 10) ms = 10;
  if (ms > 2000) ms = 2000;
  return ms;
}

// Read IR2 via CH3 by temporarily switching AMUX; restores AMUX=0 (X) afterwards
uint16_t readIR2Raw() {
 
  uint32_t integMs = integrationMsFromATIME(INTEG_TIME);
  uint32_t waitMs  = integMs + 5;

  if (DEBUG_IR2_SERIAL) {
    Serial.printf("[IR2] ATIME=0x%02X integMs=%lu waitMs=%lu\n",
                  INTEG_TIME, (unsigned long)integMs, (unsigned long)waitMs);
  }

  // Switch CH3 to IR2 mode
  setAMUX(true);
  delay(waitMs);

  // Read twice to avoid edge timing / stale sample
  (void)read16(XDATA_L);
  uint16_t ir2 = read16(XDATA_L);

  // Restore CH3 to X mode so normal reads stay correct
  setAMUX(false);
  delay(waitMs);

  return ir2;
}

// Original readIR2Raw() without dynamic delay based on integration time; fixed 30ms wait
// uint16_t readIR2Raw() {
//   setAMUX(true);
//   // Wait one integration period so CH3 updates with IR2; 25ms in your setup
//   delay(30);
//   uint16_t ir2 = read16(XDATA_L);
//   setAMUX(false);
//   return ir2;
// }

// Read CH3 in X mode (AMUX=0) with a fresh integration
static uint16_t readXRawFresh()
{
  uint32_t waitMs = integrationMsFromATIME(INTEG_TIME) + 5;
  setAMUX(false);
  delay(waitMs);
  (void)read16(XDATA_L);
  return read16(XDATA_L);
}

// Read CH3 in IR2 mode (AMUX=1) with a fresh integration
static uint16_t readIR2RawFresh()
{
  uint32_t waitMs = integrationMsFromATIME(INTEG_TIME) + 5;
  setAMUX(true);
  delay(waitMs);
  (void)read16(XDATA_L);
  uint16_t ir2 = read16(XDATA_L);

  // restore back to X mode so normal reads remain correct
  setAMUX(false);
  delay(waitMs);

  return ir2;
}

static const char* gainToStr(uint8_t again) {
  switch (again & 0x03) {
    case 0: return "1x";
    case 1: return "4x";
    case 2: return "16x";
    case 3: return "64x";
  }
  return "?";
}


// -------------------- Averaging support --------------------
struct SensorAvg {
  uint32_t X;
  uint32_t Y;
  uint32_t Z;
  uint32_t IR1;
  uint32_t IR2;
};

// Parse optional query parameters:
//   avg   = number of samples (1..50)
//   delay = delay between samples in seconds (0..60)
//   ir2   = include IR2 (adds ~30ms per sample)
static void parseAvgParams(AsyncWebServerRequest* r, uint8_t &samples, uint16_t &delaySeconds, bool &useIR2) {
  samples = 1;
  delaySeconds = 0;
  if (r && r->hasParam("avg")) {
    samples = (uint8_t)constrain(r->getParam("avg")->value().toInt(), 1, 50);
  }
  if (r && r->hasParam("delay")) {
    delaySeconds = (uint16_t)constrain(r->getParam("delay")->value().toInt(), 0, 60);
  }
  useIR2 = (r && r->hasParam("ir2"));
}



static SensorAvg readAveragedSensor(uint8_t samples, uint16_t delaySeconds, bool useIR2) {
  SensorAvg acc{0,0,0,0,0};

  for (uint8_t i = 0; i < samples; i++) {
    acc.X   += read16(XDATA_L);
    acc.Y   += read16(YDATA_L);
    acc.Z   += read16(ZDATA_L);
    acc.IR1 += read16(IR1DATA_L);
    acc.IR2 += useIR2 ? readIR2Raw() : 0;

    if (i < samples - 1 && delaySeconds > 0) {
      delay((uint32_t)delaySeconds * 1000UL);
    }
  }

  acc.X   /= samples;
  acc.Y   /= samples;
  acc.Z   /= samples;
  acc.IR1 /= samples;
  acc.IR2 /= samples;

  return acc;
}

// Get device status register
uint8_t getDeviceStatus() {
  return readSingleByte(STATUS_REG);
}

// Check if sensor is saturated
bool isSaturated() {
  return (getDeviceStatus() & STATUS_ASAT_MASK) != 0;
}

// Get device ID
uint8_t getDeviceID() {
  return readSingleByte(ID_REG);
}

// Get revision ID
uint8_t getRevisionID() {
  return readSingleByte(REVID_REG);
}

// Configure auto-zero functionality
void configureAutoZero(bool firstMeasurementOnly = true) {
  Wire.beginTransmission(TCS3430_ADDR);
  Wire.write(CMD_BIT | AZ_CONFIG_REG);
  if (firstMeasurementOnly) {
    Wire.write(0x7F); // Run auto-zero only at first ALS cycle
  } else {
    Wire.write(0x00); // Disable auto-zero
  }
  Wire.endTransmission();
}

// Set high gain mode (128x when combined with 64x gain setting)
void setHighGain(bool enable) {
  Wire.beginTransmission(TCS3430_ADDR);
  Wire.write(CMD_BIT | CFG2_REG);
  Wire.write(enable ? HGAIN_ENABLE : HGAIN_DISABLE);
  Wire.endTransmission();
}

// Set persistence for interrupt generation
void setInterruptPersistence(uint8_t persistence) {
  Wire.beginTransmission(TCS3430_ADDR);
  Wire.write(CMD_BIT | PERS_REG);
  Wire.write(persistence & 0x0F); // Only lower 4 bits are used
  Wire.endTransmission();
}

// Initialize sensor and compute calibration
void initSensor() {
  Serial.println("Attempting to initialize TCS3430...");
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("Init TCS3430..."));
  display.display();
  
  // Test I2C communication first
  Wire.beginTransmission(TCS3430_ADDR);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.printf("TCS3430 not found! I2C error: %d\n", error);
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("TCS3430 ERROR!"));
    display.setCursor(0,10);
    display.print(F("I2C Error: "));
    display.println(error);
    display.setCursor(0,20);
    display.println(F("Check wiring:"));
    display.setCursor(0,30);
    display.println(F("SDA -> GPIO21"));
    display.setCursor(0,40);
    display.println(F("SCL -> GPIO22"));
    display.display();
    delay(5000);
    return; // Don't crash, just return
  }

  // Validate device ID
  uint8_t deviceId = getDeviceID();
  uint8_t revisionId = getRevisionID();
  
  Serial.printf("Device ID: 0x%02X (expected 0x%02X)\n", deviceId, TCS3430_DEVICE_ID);
  Serial.printf("Revision ID: 0x%02X (expected 0x%02X)\n", revisionId, TCS3430_REVISION_ID);
  
  if (deviceId != TCS3430_DEVICE_ID) {
    Serial.printf("WARNING: Unexpected device ID! Got 0x%02X, expected 0x%02X\n", deviceId, TCS3430_DEVICE_ID);
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("ID WARNING!"));
    display.setCursor(0,10);
    display.printf("Got: 0x%02X\n", deviceId);
    display.setCursor(0,20);
    display.printf("Exp: 0x%02X\n", TCS3430_DEVICE_ID);
    display.setCursor(0,30);
    display.println(F("Continuing..."));
    display.display();
    delay(3000);
  }
  
  Serial.println("TCS3430 found! Configuring...");

  // Power on + ADC enable
  Wire.beginTransmission(TCS3430_ADDR);
  Wire.write(CMD_BIT | ENABLE_REG);
  Wire.write(0x03);
  Wire.endTransmission();

  // Set integration time
  Wire.beginTransmission(TCS3430_ADDR);
  Wire.write(CMD_BIT | ATIME_REG);
  Wire.write(INTEG_TIME);
  Wire.endTransmission();

  // Set gain
  setGain(GAIN);
  // Disable auto-zero completely - we use calibrated white/black tiles instead
  configureAutoZero(false);
  Serial.println("Auto-zero DISABLED - using calibrated tile references");

  // Set interrupt persistence (optional - for future interrupt use)
  setInterruptPersistence(0x01); // 1 consecutive value out of range
  
  // Configure high gain based on current setting
  setHighGain(highGainEnabled);

  // Compute calibration factors
  calibOffsetX = rawBlackX;
  calibOffsetY = rawBlackY;
  calibOffsetZ = rawBlackZ;
  calibScaleX  = (targetWhiteX - targetBlackX) / float(rawWhiteX - rawBlackX);
  calibScaleY  = (targetWhiteY - targetBlackY) / float(rawWhiteY - rawBlackY);
  calibScaleZ  = (targetWhiteZ - targetBlackZ) / float(rawWhiteZ - rawBlackZ);
  
  Serial.println("TCS3430 initialized successfully with DFRobot enhancements!");
  Serial.printf("Device ready - ID: 0x%02X, Rev: 0x%02X\n", deviceId, revisionId);
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("TCS3430 Ready!"));
  display.setCursor(0,10);
  display.printf("ID: 0x%02X\n", deviceId);
  display.setCursor(0,20);
  display.printf("Rev: 0x%02X\n", revisionId);
  display.display();
  delay(2000);
}

// Read two bytes from a register
uint16_t read16(uint8_t reg) {
  Wire.beginTransmission(TCS3430_ADDR);
  Wire.write(CMD_BIT | reg);
  if (Wire.endTransmission(true) != 0) return 0;
  uint8_t cnt = Wire.requestFrom((uint8_t)TCS3430_ADDR, (uint8_t)2);
  if (cnt < 2) return 0;
  uint8_t lo = Wire.read(), hi = Wire.read();
  return (uint16_t)hi << 8 | lo;
}

// JSON endpoint
void handleColor(AsyncWebServerRequest* req) {
  // Check for saturation first
  bool saturated = isSaturated();
  uint8_t status = getDeviceStatus();
  
  // Use raw values like the display does
  // uint16_t Xr = read16(XDATA_L);
  // uint16_t Yr = read16(YDATA_L);
  // uint16_t Zr = read16(ZDATA_L);

  // Read raw channels (single read by default, or averaged if query params provided)
  uint8_t samples; 
  uint16_t delaySeconds;
  bool useIR2;
  parseAvgParams(req, samples, delaySeconds, useIR2);

  SensorAvg s = readAveragedSensor(samples, delaySeconds, useIR2);
  uint16_t Xr = (uint16_t)s.X;
  uint16_t Yr = (uint16_t)s.Y;
  uint16_t Zr = (uint16_t)s.Z;
  uint16_t IR1r = (uint16_t)s.IR1;
  uint16_t IR2r = (uint16_t)s.IR2;

  
  Serial.printf("DEBUG: Raw readings: X=%u, Y=%u, Z=%u, IR1=%u, IR2=%u\n", Xr, Yr, Zr, IR1r, IR2r); 
  Serial.printf("DEBUG: Status: 0x%02X, Saturated: %s\n", status, saturated ? "YES" : "NO");
  Serial.printf("DEBUG: Calibration - White: X=%u Y=%u Z=%u, Black: X=%u Y=%u Z=%u\n", 
                rawWhiteX, rawWhiteY, rawWhiteZ, rawBlackX, rawBlackY, rawBlackZ);
  
  // Apply black offset correction first
  float X_offset = max(0.0f, (float)(Xr - rawBlackX));
  float Y_offset = max(0.0f, (float)(Yr - rawBlackY));
  float Z_offset = max(0.0f, (float)(Zr - rawBlackZ));
  
  // Apply improved color space transformation
  // More balanced factors to avoid pushing all colors to the left
  // Apply professional spectrophotometer-based calibration matrix
  float X_cal, Y_cal, Z_cal;
  float x = 0, y = 0, CCT = 0, lux = 0;
  
  // Apply matrix transformation to get RawXYZ (matrix output)
  float X_rawXYZ, Y_rawXYZ, Z_rawXYZ;
  transformToXYZ(Xr, Yr, Zr, X_rawXYZ, Y_rawXYZ, Z_rawXYZ);

  // Raw chromaticity from RawXYZ (per project requirement)
  float rawxyz_sum = X_rawXYZ + Y_rawXYZ + Z_rawXYZ;
  float rawxyz_x = (rawxyz_sum > 1e-6f) ? (X_rawXYZ / rawxyz_sum) : 0.0f;
  float rawxyz_y = (rawxyz_sum > 1e-6f) ? (Y_rawXYZ / rawxyz_sum) : 0.0f;

  // CCT must be computed ONLY from RawXYZ (matrix output)
  CCT = computeCCT_McCamy(rawxyz_x, rawxyz_y);

  // AMS-style 2-point calibration in XYZ space (Black/White tiles)
  computeCalXYZ_2Point(X_rawXYZ, Y_rawXYZ, Z_rawXYZ, X_cal, Y_cal, Z_cal);

  // Chromaticity from calibrated XYZ
  float sum = X_cal + Y_cal + Z_cal;
  x = (sum > 1e-6f) ? (X_cal / sum) : 0.0f;
  y = (sum > 1e-6f) ? (Y_cal / sum) : 0.0f;

  // Safety clamp for web display
  x = max(0.001f, min(0.79f, x));
  y = max(0.001f, min(0.82f, y));

  // Lux: keep using calibrated Y as a relative lux-like output (scaling is project-specific)
  lux = Y_cal * 0.2f;

  
  // Calculate Lab values for JSON response
  float L, a_lab, b_lab;
  XYZtoLabRef(X_cal, Y_cal, Z_cal, spectroTargets[0].X, spectroTargets[0].Y, spectroTargets[0].Z, L, a_lab, b_lab);
  
  String j = "{";
  j += String("\"Xr\":")  + Xr + ",";
  j += String("\"Yr\":")  + Yr + ",";
  j += String("\"Zr\":")  + Zr + ",";
  j += String("\"IR1r\":") + IR1r + ",";
  j += String("\"IR2r\":") + IR2r + ",";
  j += String("\"X\":")   + String(X_cal,3) + ",";
  j += String("\"Y\":")   + String(Y_cal,3) + ",";
  j += String("\"Z\":")   + String(Z_cal,3) + ",";
  j += String("\"L\":")   + String(L,2) + ",";
  j += String("\"a\":")   + String(a_lab,2) + ",";
  j += String("\"b\":")   + String(b_lab,2) + ",";
  j += String("\"x\":")   + String(x,4) + ",";
  j += String("\"y\":")   + String(y,4) + ",";
  j += String("\"CCT\":") + String(CCT,0) + ",";
  j += String("\"lux\":") + String(lux,1) + ",";
  j += String("\"saturated\":") + (saturated ? "true" : "false") + ",";
  j += String("\"gain\":") + String(GAIN) + ",";
  j += String("\"high_gain\":") + String(highGainEnabled ? "true" : "false") + ",";
  j += String("\"integration_time\":") + String(INTEG_TIME) + ",";
  
  // Add spectrophotometer comparison
  int closestTarget = findClosestSpectroTarget(X_cal, Y_cal, Z_cal);
  j += String("\"spectro_match\":{\"name\":\"") + spectroTargets[closestTarget].name + "\",";
  j += String("\"target_X\":") + String(spectroTargets[closestTarget].X, 2) + ",";
  j += String("\"target_Y\":") + String(spectroTargets[closestTarget].Y, 2) + ",";
  j += String("\"target_Z\":") + String(spectroTargets[closestTarget].Z, 2) + ",";
  j += String("\"target_x\":") + String(spectroTargets[closestTarget].x, 4) + ",";
  j += String("\"target_y\":") + String(spectroTargets[closestTarget].y, 4) + ",";
  j += String("\"diff_X\":") + String(X_cal - spectroTargets[closestTarget].X, 2) + ",";
  j += String("\"diff_Y\":") + String(Y_cal - spectroTargets[closestTarget].Y, 2) + ",";
  j += String("\"diff_Z\":") + String(Z_cal - spectroTargets[closestTarget].Z, 2) + ",";
  j += String("\"diff_x\":") + String(x - spectroTargets[closestTarget].x, 4) + ",";
  j += String("\"diff_y\":") + String(y - spectroTargets[closestTarget].y, 4) + "},";
  
  // Add calibration adjustment parameters
  j += String("\"calibration\":{\"k_x\":") + String(kX, 4) + ",";
  j += String("\"k_y\":") + String(kY, 4) + ",";
  j += String("\"k_z\":") + String(kZ, 4) + ",";
  j += String("\"x_offset\":") + String(xOffsetAdjust, 1) + ",";
  j += String("\"y_offset\":") + String(yOffsetAdjust, 1) + ",";
  j += String("\"z_offset\":") + String(zOffsetAdjust, 1) + "},";
  j += String("\"status\":\"0x") + String(status, HEX) + "\"";
  j += "}";
  req->send(200, "application/json", j);
}

void setup() {
  // Disable brownout detector immediately
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  Serial.begin(115200);
  delay(200); // Give serial time to initialize
  Serial.println(">>> BOOT <<<"); // marker so we know we’re seeing the very start

  // ***** ADD THIS BLOCK *****
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("Reset reason: ");
  Serial.println(reason);

  Serial.println("\n=== TCS3430 Color Sensor Starting ===");
  Serial.println("Brownout detector disabled");
  // **************************
  
  Wire.begin(); 
  // Ensure CH3 is X channel by default (AMUX=0)
  setAMUX(false);
  Serial.println("I2C initialized");
  
    // Load persistent settings early (WiFi depends on these)
  Serial.println("Loading persistent settings...");
  settingsLoadedOk = loadSettings();
  if (settingsLoadedOk) {
    Serial.println("✓ Settings loaded from flash memory");
  } else {
    Serial.println("ℹ Using default settings (first boot or no saved settings)");
  }

  // START WIFI FIRST - before display and sensor initialization
  Serial.println("Starting WiFi early to distribute power load...");

  // Setup WiFi based on configuration
  setupWiFi();
  Serial.println("Connect to this network and visit the IP address above in your browser");
  // Initialize OLED display
  Serial.println("Initializing OLED display...");
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    // Don't hang forever, continue without display
  } else {
    Serial.println("OLED display initialized successfully");
  }
  
  display.display();
  delay(1000); // Shorter delay
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("TCS3430 Starting..."));
  display.setCursor(0,10);
  display.println(F("WiFi connecting..."));
  display.display();
  
  // Initialize sensor with error handling
  Serial.println("Initializing TCS3430 sensor...");
  initSensor();
  Serial.println("TCS3430 initialization complete");
  
  // Continue WiFi setup - for AP mode, no connection needed
  Serial.println("WiFi Access Point ready!");
  
  // Display WiFi info on OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("WiFi Access Point"));
  display.setCursor(0,10);
  display.println(F("SSID: TCS3430-ColorSensor"));
  display.setCursor(0,20);
  display.print(F("IP: "));
  display.println(WiFi.softAPIP());
  display.setCursor(0,30);
  display.println(F("Port: 80"));
  display.setCursor(0,40);
  display.println(F("Connect & browse IP"));
  display.display();
  // (No blocking WiFi wait here; AP/dual/STA handled in setupWiFi)
  Serial.println("Initializing SPIFFS...");
if (!SPIFFS.begin(true)) {
  Serial.println("SPIFFS mount failed - using embedded graphics");
} else {
  Serial.println("SPIFFS initialized successfully");
  // SPIFFS content check
  Serial.print("SPIFFS has /cie.jpg: ");
  Serial.println(SPIFFS.exists("/cie.jpg") ? "YES" : "NO");
  Serial.print("SPIFFS has /settings.json: ");
  Serial.println(SPIFFS.exists("/settings.json") ? "YES" : "NO");
}

// Apply loaded sensor settings (loaded earlier via loadSettings())
if (settingsLoadedOk) {
  setGain(GAIN);

  Wire.beginTransmission(TCS3430_ADDR);
  Wire.write(CMD_BIT | ATIME_REG);
  Wire.write(INTEG_TIME);
  Wire.endTransmission();

  setHighGain(highGainEnabled);
  Serial.println("✓ Sensor configured with saved parameters");
} else {
  Serial.println("ℹ Using default sensor settings");
}

Serial.println("Setting up web server routes...");
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){ 
    Serial.println("Root page requested");
    r->send(200, "text/html", index_html); 
  });
  server.on("/color", HTTP_GET, handleColor);
  
  // Measurement dashboard page
  server.on("/measurement", HTTP_GET, [](AsyncWebServerRequest* r){
    const char measurement_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Live Measurement - TCS3430</title>
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; }
    .container { max-width: 1200px; margin: 0 auto; }
    .header { text-align: center; margin-bottom: 30px; }
    .nav-button { display: inline-block; background: rgba(255,255,255,0.2); color: white; text-decoration: none; padding: 10px 20px; border-radius: 25px; margin: 5px; transition: all 0.3s ease; }
    .nav-button:hover { background: rgba(255,255,255,0.3); transform: translateY(-2px); }
    .measurement-grid { display: grid; grid-template-columns: 1fr 400px; gap: 20px; margin-bottom: 20px; }
    .measurement-panel { background: rgba(255,255,255,0.1); border-radius: 15px; padding: 25px; backdrop-filter: blur(10px); }
    .values-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); gap: 15px; margin-bottom: 20px; }
    .value-item { background: rgba(255,255,255,0.1); padding: 15px; border-radius: 10px; text-align: center; }
    .value-label { font-size: 0.9em; opacity: 0.8; margin-bottom: 5px; }
    .value-number { font-size: 1.5em; font-weight: bold; }
    #diagram { border: 2px solid rgba(255,255,255,0.3); border-radius: 10px; background: white; }
    .spectro-match { background: rgba(255,255,255,0.1); padding: 15px; border-radius: 10px; margin-top: 20px; }
    @media (max-width: 900px) { .measurement-grid { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>🔬 Live Color Measurement</h1>
      <a href="/" class="nav-button">🏠 Home</a>
      <a href="/configuration" class="nav-button">⚙️ Configuration</a>
      <a href="/calibration" class="nav-button">🎯 Calibration</a>
    </div>

    <div class="measurement-grid">
      <div class="measurement-panel">
        <h3>📊 Current Readings</h3>
        <div class="values-grid">
          <div class="value-item">
            <div class="value-label">X Value</div>
            <div class="value-number" id="xValue">--</div>
          </div>
          <div class="value-item">
            <div class="value-label">Y Value</div>
            <div class="value-number" id="yValue">--</div>
          </div>
          <div class="value-item">
            <div class="value-label">Z Value</div>
            <div class="value-number" id="zValue">--</div>
          </div>
          <div class="value-item">
            <div class="value-label">L* Lightness</div>
            <div class="value-number" id="lValue">--</div>
          </div>
          <div class="value-item">
            <div class="value-label">a* Green-Red</div>
            <div class="value-number" id="aValue">--</div>
          </div>
          <div class="value-item">
            <div class="value-label">b* Blue-Yellow</div>
            <div class="value-number" id="bValue">--</div>
          </div>
          <div class="value-item">
            <div class="value-label">x Chromaticity</div>
            <div class="value-number" id="xChrom">--</div>
          </div>
          <div class="value-item">
            <div class="value-label">y Chromaticity</div>
            <div class="value-number" id="yChrom">--</div>
          </div>
          <div class="value-item">
            <div class="value-label">CCT (K)</div>
            <div class="value-number" id="cctValue">--</div>
          </div>
          <div class="value-item">
            <div class="value-label">Lux</div>
            <div class="value-number" id="luxValue">--</div>
          </div>
          <div class="value-item">
            <div class="value-label">Status</div>
            <div class="value-number" id="statusValue">--</div>
          </div>
        </div>
        
        <div class="spectro-match" id="spectroMatch">
          <h4>🎯 Spectrophotometer Match</h4>
          <p id="matchInfo">Loading...</p>
        </div>
      </div>

      <div class="measurement-panel">
        <h3>🌈 CIE 1931 Chromaticity</h3>
        <canvas id="diagram" width="400" height="350"></canvas>
      </div>
    </div>
  </div>

<script>
  // Reuse CIE diagram code from original
  function createCIEBackground(canvas) {
    const ctx = canvas.getContext('2d');
    const w = canvas.width;
    const h = canvas.height;
    
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, w, h);
    
    const img = new Image();
    img.onload = function() {
      window.cieBackgroundImage = img;
      ctx.drawImage(img, 0, 0, w, h);
      drawCIEOverlays(ctx, w, h);
    };
    img.onerror = function() {
      createFallbackDiagram(ctx, w, h);
    };
    img.src = '/cie.jpg';
  }
  
  function drawCIEOverlays(ctx, w, h) {
    ctx.strokeStyle = 'rgba(200,200,200,0.3)';
    ctx.lineWidth = 1;
    for (let i = 1; i <= 8; i++) {
      const x = (i / 10) * w * (0.8 / 0.8);
      const y = (i / 10) * h;
      if (x < w) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, h);
        ctx.stroke();
      }
      if (y < h) {
        ctx.beginPath();
        ctx.moveTo(0, h - y);
        ctx.lineTo(w, h - y);
        ctx.stroke();
      }
    }
    
    ctx.fillStyle = '#000000';
    ctx.font = 'bold 14px Arial';
    ctx.fillText('CIE 1931 Chromaticity Diagram', 10, 20);
    ctx.font = '12px Arial';
    ctx.fillText('x', w - 15, h - 5);
  }
  
  function createFallbackDiagram(ctx, w, h) {
    const cieBoundary = [
      [0.1741, 0.0050], [0.7347, 0.2653], [0.1741, 0.0050]
    ];
    const scaledBoundary = cieBoundary.map(point => [
      (point[0] / 0.8) * w,
      (1 - Math.min(point[1], 0.85) / 0.9) * h
    ]);
    
    ctx.strokeStyle = '#000000';
    ctx.lineWidth = 2;
    ctx.beginPath();
    scaledBoundary.forEach((point, i) => {
      if (i === 0) ctx.moveTo(point[0], point[1]);
      else ctx.lineTo(point[0], point[1]);
    });
    ctx.closePath();
    ctx.stroke();
  }

  function draw(x, y) {
    const c = document.getElementById('diagram');
    const ctx = c.getContext('2d');
    
    ctx.clearRect(0, 0, c.width, c.height);
    
    if (window.cieBackgroundImage && window.cieBackgroundImage.complete) {
      ctx.drawImage(window.cieBackgroundImage, 0, 0, c.width, c.height);
      drawCIEOverlays(ctx, c.width, c.height);
    } else {
      createFallbackDiagram(ctx, c.width, c.height);
    }
    
    // Draw BCRA tile reference points
    drawBCRATiles(ctx, c.width, c.height);
    
    // Draw current measurement point - use expanded CIE coordinate scaling
    // Draw current measurement point (mapped to the plot area of the CIE image)
    const left = 42, right = 18, top = 18, bottom = 34; // tuned for cie.jpg axes
    const plotW = c.width - left - right;
    const plotH = c.height - top - bottom;
    const px = left + (x / 0.8) * plotW;           // x axis: 0.0 .. 0.8
    const py = top + ((0.9 - y) / 0.9) * plotH;    // y axis: 0.0 .. 0.9 (invert for canvas)
    if (px >= 0 && px <= c.width && py >= 0 && py <= c.height) {
      ctx.beginPath(); 
      ctx.arc(px, py, 8, 0, 2 * Math.PI);
      ctx.fillStyle = '#ff0000';
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 3;
      ctx.stroke();
      
      ctx.strokeStyle = '#ff0000';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(px - 15, py); ctx.lineTo(px + 15, py);
      ctx.moveTo(px, py - 15); ctx.lineTo(px, py + 15);
      ctx.stroke();
    }
  }

function drawBCRATiles(ctx, width, height) {
  // Plot the spectro reference tile chromaticities (xy) on top of cie.jpg.
  // NOTE: These are the spectroTargets[] xy values from firmware (Blue removed).
  const tiles = [
    { name: 'White',     x: 0.3382, y: 0.3439, color: '#FFFFFF', text: '#000000' },
    { name: 'Yellow',    x: 0.4510, y: 0.4778, color: '#FFFF00', text: '#000000' },
    { name: 'Black',     x: 0.3236, y: 0.3413, color: '#000000', text: '#FFFFFF' },
    { name: 'Green',     x: 0.3195, y: 0.5627, color: '#00AA00', text: '#FFFFFF' },
    { name: 'Red',       x: 0.5787, y: 0.3486, color: '#FF0000', text: '#FFFFFF' },
    { name: 'Orange',    x: 0.4814, y: 0.4474, color: '#FFA500', text: '#000000' },
    { name: 'Mid Grey',  x: 0.3262, y: 0.3334, color: '#808080', text: '#FFFFFF' },
    { name: 'Deep Pink', x: 0.4590, y: 0.2665, color: '#FF69B4', text: '#000000' }
  ];

  // Plot area margins for cie.jpg (matches the axes region, not the whole bitmap)
  const left = 42, right = 18, top = 18, bottom = 34;
  const plotW = width - left - right;
  const plotH = height - top - bottom;

  function toCanvasXY(x, y) {
    return {
      px: left + (x / 0.8) * plotW,
      py: top  + ((0.9 - y) / 0.9) * plotH
    };
  }

  tiles.forEach(t => {
    const { px, py } = toCanvasXY(t.x, t.y);

    // marker
    ctx.beginPath();
    ctx.arc(px, py, 6, 0, 2 * Math.PI);
    ctx.fillStyle = t.color;
    ctx.fill();
    ctx.strokeStyle = t.text;
    ctx.lineWidth = 2;
    ctx.stroke();

    // label
    ctx.font = 'bold 11px Arial';
    ctx.textAlign = 'center';
    ctx.fillStyle = t.text;
    ctx.fillText(t.name, px, py - 12);

    // coords
    ctx.font = '9px Arial';
    ctx.fillText(`(${t.x.toFixed(3)}, ${t.y.toFixed(3)})`, px, py + 18);
  });
}

  async function refresh() {
    try {
      const res = await fetch('/color');
      const data = await res.json();
      
      document.getElementById('xValue').textContent = data.X.toFixed(3);
      document.getElementById('yValue').textContent = data.Y.toFixed(3);
      document.getElementById('zValue').textContent = data.Z.toFixed(3);
      
      // Add Lab values if they exist
      if (data.L !== undefined) {
        document.getElementById('lValue').textContent = data.L.toFixed(1);
        document.getElementById('aValue').textContent = data.a.toFixed(1);
        document.getElementById('bValue').textContent = data.b.toFixed(1);
      }
      
      document.getElementById('xChrom').textContent = data.x.toFixed(4);
      document.getElementById('yChrom').textContent = data.y.toFixed(4);
      document.getElementById('cctValue').textContent = data.CCT > 0 ? Math.round(data.CCT) : '--';
      document.getElementById('luxValue').textContent = data.lux.toFixed(1);
      document.getElementById('statusValue').textContent = data.saturated ? '⚠️ SAT' : '✅ OK';
      document.getElementById('statusValue').style.color = data.saturated ? '#FF9800' : '#4CAF50';
      
      // Spectrophotometer match info
      if (data.spectro_match) {
        const match = data.spectro_match;
        const matchText = `Closest: ${match.name} | ΔX: ${match.diff_X.toFixed(2)} ΔY: ${match.diff_Y.toFixed(2)} ΔZ: ${match.diff_Z.toFixed(2)}`;
        document.getElementById('matchInfo').textContent = matchText;
      }
      
      draw(data.x, data.y);
    } catch (error) {
      console.error('Measurement update failed:', error);
    }
  }
  
  createCIEBackground(document.getElementById('diagram'));
  refresh();
  setInterval(refresh, 10000); // Increased to 10 seconds to allow plenty of time for input entry
</script>
</body>
</html>
)rawliteral";
    r->send(200, "text/html", measurement_html);
  });
  
  // Configuration control panel
  server.on("/configuration", HTTP_GET, [](AsyncWebServerRequest* r){
    const char config_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Configuration - TCS3430</title>
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; }
    .container { max-width: 1000px; margin: 0 auto; }
    .header { text-align: center; margin-bottom: 30px; }
    .nav-button { display: inline-block; background: rgba(255,255,255,0.2); color: white; text-decoration: none; padding: 10px 20px; border-radius: 25px; margin: 5px; transition: all 0.3s ease; }
    .nav-button:hover { background: rgba(255,255,255,0.3); transform: translateY(-2px); }
    .config-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
    .config-panel { background: rgba(255,255,255,0.1); border-radius: 15px; padding: 25px; backdrop-filter: blur(10px); }
    .form-group { margin-bottom: 20px; }
    .form-label { display: block; margin-bottom: 5px; font-weight: bold; }
    .form-input { width: 100%; padding: 10px; border: none; border-radius: 8px; background: rgba(255,255,255,0.9); color: #333; font-size: 16px; }
    .form-button { background: #4CAF50; color: white; border: none; padding: 12px 20px; border-radius: 8px; cursor: pointer; font-size: 16px; margin: 5px; transition: all 0.3s ease; }
    .form-button:hover { background: #45a049; transform: translateY(-2px); }
    .form-button.secondary { background: #2196F3; }
    .form-button.secondary:hover { background: #1976D2; }
    .current-values { background: rgba(255,255,255,0.1); padding: 15px; border-radius: 10px; margin-bottom: 20px; font-family: monospace; }
    .status-indicator { display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 8px; }
    .status-ok { background: #4CAF50; }
    .status-warning { background: #FF9800; }
    .status-error { background: #f44336; }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>⚙️ System Configuration</h1>
      <a href="/" class="nav-button">🏠 Home</a>
      <a href="/measurement" class="nav-button">📊 Measurement</a>
      <a href="/calibration" class="nav-button">🎯 Calibration</a>
      <a href="/diagnostics" class="nav-button">🔧 Diagnostics</a>
    </div>

    <div class="config-grid">
      <div class="config-panel">
        <h3>🎛️ Sensor Parameters</h3>
        <div class="current-values" id="sensorStatus">
          <div>Loading current settings...</div>
        </div>
        
        <div class="form-group">
          <label class="form-label">Gain Setting</label>
          <select id="gainSelect" class="form-input">
            <option value="0">1x Gain</option>
            <option value="1">4x Gain</option>
            <option value="2">16x Gain</option>
            <option value="3">64x Gain</option>
          </select>
          <button class="form-button" onclick="setGain()">Apply Gain</button>
        </div>

        <div class="form-group">
          <label class="form-label">Integration Time</label>
          <select id="integrationSelect" class="form-input">
            <option value="0xF6">~25ms (Bright Light)</option>
            <option value="0xEB">~50ms (Normal Light)</option>
            <option value="0xD6">~100ms (Dim Light)</option>
            <option value="0xC0">~180ms (Very Dim)</option>
          </select>
          <button class="form-button" onclick="setIntegration()">Apply Integration</button>
        </div>

        <div class="form-group">
          <label class="form-label">High Gain Mode</label>
          <button class="form-button secondary" onclick="toggleHighGain(true)">Enable High Gain</button>
          <button class="form-button" onclick="toggleHighGain(false)">Disable High Gain</button>
        </div>
      </div>

      <div class="config-panel">
        <h3>🎯 Real-Time Calibration</h3>
        <div class="current-values" id="calibrationStatus">
          <div>Loading calibration settings...</div>
        </div>

        <div class="form-group">
          <label class="form-label">AMS kX Factor</label>
          <input type="number" id="kX" class="form-input" step="0.0001" min="0.0001" max="2.0000">
        </div>

        <div class="form-group">
          <label class="form-label">AMS kY Factor</label>
          <input type="number" id="kY" class="form-input" step="0.0001" min="0.0001" max="2.0000">
        </div>

        <div class="form-group">
          <label class="form-label">AMS kZ Factor</label>
          <input type="number" id="kZ" class="form-input" step="0.0001" min="0.0001" max="2.0000">
        </div>

        <button class="form-button" onclick="applyKFactors()">Apply K-Factors</button>
        <button class="form-button secondary" onclick="resetToDefaults()">Reset to Defaults</button>
      </div>

      <div class="config-panel">
        <h3>📏 Fine Adjustments</h3>
        <div class="current-values" id="adjustmentStatus">
          <div>Offset adjustments: X=0.0, Y=0.0, Z=0.0</div>
        </div>

        <div class="form-group">
          <label class="form-label">X Offset Adjustment</label>
          <input type="number" id="xOffset" class="form-input" step="0.1" min="-50.0" max="50.0" value="0.0">
        </div>

        <div class="form-group">
          <label class="form-label">Y Offset Adjustment</label>
          <input type="number" id="yOffset" class="form-input" step="0.1" min="-50.0" max="50.0" value="0.0">
        </div>

        <div class="form-group">
          <label class="form-label">Z Offset Adjustment</label>
          <input type="number" id="zOffset" class="form-input" step="0.1" min="-50.0" max="50.0" value="0.0">
        </div>

        <button class="form-button" onclick="applyOffsets()">Apply Offsets</button>
        <button class="form-button secondary" onclick="clearOffsets()">Clear Offsets</button>
      </div>
      
      <div class="config-panel">
        <h3>💾 Settings Management</h3>
        <div class="current-values" id="settingsStatus">
          <div>🔄 Settings are automatically saved when changed</div>
          <div>📁 Stored in flash memory (survives power cycles)</div>
        </div>

        <div class="form-group">
          <button class="form-button" onclick="manualSave()">💾 Manual Save</button>
          <button class="form-button secondary" onclick="reloadSettings()">🔄 Reload from Flash</button>
        </div>
        
        <div class="form-group">
          <button class="form-button" onclick="resetToDefaults()">🏭 Factory Reset</button>
          <button class="form-button secondary" onclick="exportSettings()">📤 Export Settings</button>
        </div>
      </div>
    </div>
  </div>

<script>
  // Load current settings
  async function loadSettings() {
    try {
      const colorRes = await fetch('/color');
      const colorData = await colorRes.json();
      
      // Update sensor status
      document.getElementById('sensorStatus').innerHTML = `
        <div><span class="status-indicator ${colorData.saturated ? 'status-warning' : 'status-ok'}"></span>Gain: ${colorData.gain} | Integration: 0x${colorData.integration_time.toString(16).toUpperCase()}</div>
        <div><span class="status-indicator ${colorData.high_gain ? 'status-ok' : 'status-indicator'}"></span>High Gain: ${colorData.high_gain ? 'Enabled' : 'Disabled'}</div>
        <div><span class="status-indicator status-ok"></span>Status: ${colorData.saturated ? 'Saturated' : 'Normal'}</div>
      `;

      // Update calibration status
      if (colorData.calibration) {
        const cal = colorData.calibration;
        document.getElementById('calibrationStatus').innerHTML = `
          <div>kX: ${cal.k_x.toFixed(4)} | kY: ${cal.k_y.toFixed(4)} | kZ: ${cal.k_z.toFixed(4)}</div>
          <div>X Offset: ${cal.x_offset.toFixed(1)} | Y Offset: ${cal.y_offset.toFixed(1)} | Z Offset: ${cal.z_offset.toFixed(1)}</div>
        `;
        
        // Set input values
        document.getElementById('kX').value = cal.k_x.toFixed(4);
        document.getElementById('kY').value = cal.k_y.toFixed(4);
        document.getElementById('kZ').value = cal.k_z.toFixed(4);
        document.getElementById('xOffset').value = cal.x_offset.toFixed(1);
        document.getElementById('yOffset').value = cal.y_offset.toFixed(1);
        document.getElementById('zOffset').value = cal.z_offset.toFixed(1);
      }

      // Set dropdown values
      document.getElementById('gainSelect').value = colorData.gain;
      document.getElementById('integrationSelect').value = '0x' + colorData.integration_time.toString(16).toUpperCase();
      
    } catch (error) {
      console.error('Failed to load settings:', error);
      document.getElementById('sensorStatus').innerHTML = '<div><span class="status-indicator status-error"></span>Error loading settings</div>';
    }
  }

  async function setGain() {
    const gain = document.getElementById('gainSelect').value;
    try {
      const response = await fetch(`/config/gain?value=${gain}`);
      const result = await response.json();
      alert(`Gain set to: ${result.gain}`);
      loadSettings();
    } catch (error) {
      alert('Failed to set gain: ' + error.message);
    }
  }

  async function setIntegration() {
    const integration = document.getElementById('integrationSelect').value;
    const intValue = integration.startsWith('0x') ? parseInt(integration, 16) : parseInt(integration);
    try {
      const response = await fetch(`/config/integration?value=${intValue}`);
      const result = await response.json();
      alert(`Integration time set to: 0x${result.integration_time.toString(16).toUpperCase()}`);
      loadSettings();
    } catch (error) {
      alert('Failed to set integration time: ' + error.message);
    }
  }

  async function toggleHighGain(enable) {
    try {
      const response = await fetch(`/config/highgain_adj?enable=${enable}`);
      const result = await response.json();
      alert(`High gain ${result.highGainEnabled ? 'enabled' : 'disabled'}`);
      loadSettings();
    } catch (error) {
      alert('Failed to toggle high gain: ' + error.message);
    }
  }

  async function applyKFactors() {
    const kx = document.getElementById('kX').value;
    const ky = document.getElementById('kY').value;
    const kz = document.getElementById('kZ').value;
    
    if (kx < 0.0001 || ky < 0.0001 || kz < 0.0001) {
      alert('K-factors must be greater than 0.0001');
      return;
    }
    
    try {
      const response = await fetch(`/config/kfactor?kX=${kx}&kY=${ky}&kZ=${kz}`);
      const result = await response.json();
      alert(`K-factors applied: kX=${result.k_x.toFixed(4)}, kY=${result.k_y.toFixed(4)}, kZ=${result.k_z.toFixed(4)}`);
      loadSettings();
    } catch (error) {
      alert('Failed to apply k-factors: ' + error.message);
    }
  }

  async function applyOffsets() {
    const x = document.getElementById('xOffset').value;
    const y = document.getElementById('yOffset').value;
    const z = document.getElementById('zOffset').value;
    
    try {
      const response = await fetch(`/config/offset?x=${x}&y=${y}&z=${z}`);
      const result = await response.json();
      alert(`Offsets applied: X=${result.x_offset.toFixed(1)}, Y=${result.y_offset.toFixed(1)}, Z=${result.z_offset.toFixed(1)}`);
      loadSettings();
    } catch (error) {
      alert('Failed to apply offsets: ' + error.message);
    }
  }

  async function resetToDefaults() {
    if (confirm('Reset k-factors to AMS defaults (kX=0.96, kY=0.96, kZ=0.96)?')) {
      try {
        const response = await fetch('/config/kfactor?kX=0.96&kY=0.96&kZ=0.96');
        await response.json();
        alert('K-factors reset to AMS calibration defaults');
        loadSettings();
      } catch (error) {
        alert('Failed to reset: ' + error.message);
      }
    }
  }

  async function clearOffsets() {
    try {
      const response = await fetch('/config/offset?x=0&y=0&z=0');
      await response.json();
      alert('All offsets cleared and saved to flash memory');
      loadSettings();
    } catch (error) {
      alert('Failed to clear offsets: ' + error.message);
    }
  }
  
  async function manualSave() {
    try {
      const response = await fetch('/settings/save');
      const result = await response.json();
      alert(result.message + (result.success ? ' ✅' : ' ❌'));
      if (result.success) {
        document.getElementById('settingsStatus').innerHTML = `
          <div>✅ Settings manually saved at ${new Date().toLocaleTimeString()}</div>
          <div>💾 All configuration preserved in flash memory</div>
        `;
      }
    } catch (error) {
      alert('Save request failed: ' + error.message);
    }
  }
  
  async function reloadSettings() {
    if (confirm('Reload all settings from flash memory? This will overwrite current values.')) {
      try {
        const response = await fetch('/settings/load');
        const result = await response.json();
        alert(result.message + (result.success ? ' ✅' : ' ❌'));
        if (result.success) {
          loadSettings();
          document.getElementById('settingsStatus').innerHTML = `
            <div>🔄 Settings reloaded from flash at ${new Date().toLocaleTimeString()}</div>
            <div>⚙️ All parameters restored and applied</div>
          `;
        }
      } catch (error) {
        alert('Reload request failed: ' + error.message);
      }
    }
  }
  
  async function exportSettings() {
    try {
      const response = await fetch('/settings/export');
      const settings = await response.text();
      
      // Create downloadable JSON file
      const blob = new Blob([settings], {type: 'application/json'});
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = 'tcs3430_settings_' + new Date().toISOString().slice(0,10) + '.json';
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);
      
      alert('Settings exported successfully! 📁');
    } catch (error) {
      alert('Export failed: ' + error.message);
    }
  }

  // Auto-update display
  loadSettings();
  setInterval(loadSettings, 10000); // Increased to 10 seconds to allow time for calibration input
</script>
</body>
</html>
)rawliteral";
    r->send(200, "text/html", config_html);
  });
  
  // Diagnostics page
  server.on("/diagnostics", HTTP_GET, [](AsyncWebServerRequest* r){
    const char diagnostics_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Diagnostics - TCS3430</title>
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; }
    .container { max-width: 1200px; margin: 0 auto; }
    .header { text-align: center; margin-bottom: 30px; }
    .nav-button { display: inline-block; background: rgba(255,255,255,0.2); color: white; text-decoration: none; padding: 10px 20px; border-radius: 25px; margin: 5px; transition: all 0.3s ease; }
    .nav-button:hover { background: rgba(255,255,255,0.3); transform: translateY(-2px); }
    .diagnostics-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
    .diagnostic-panel { background: rgba(255,255,255,0.1); border-radius: 15px; padding: 25px; backdrop-filter: blur(10px); }
    .status-indicator { display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 8px; }
    .status-ok { background: #4CAF50; }
    .status-warning { background: #FF9800; }
    .status-error { background: #f44336; }
    .diagnostic-item { background: rgba(255,255,255,0.1); padding: 15px; border-radius: 10px; margin: 10px 0; }
    .diagnostic-value { font-weight: bold; font-size: 1.2em; }
    .test-button { background: #2196F3; color: white; border: none; padding: 10px 15px; border-radius: 8px; cursor: pointer; margin: 5px; }
    .test-button:hover { background: #1976D2; }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>🔧 System Diagnostics</h1>
      <a href="/" class="nav-button">🏠 Home</a>
      <a href="/measurement" class="nav-button">📊 Measurement</a>
      <a href="/configuration" class="nav-button">⚙️ Configuration</a>
      <a href="/calibration" class="nav-button">🎯 Calibration</a>
    </div>

    <div class="diagnostics-grid">
      <div class="diagnostic-panel">
        <h3>🔍 Sensor Validation</h3>
        <div class="diagnostic-item" id="deviceValidation">
          <div>Loading sensor validation...</div>
        </div>
        
        <div class="diagnostic-item" id="communicationTest">
          <div>I2C Communication: Testing...</div>
        </div>
        
        <button class="test-button" onclick="runSensorTest()">🧪 Run Sensor Test</button>
      </div>

      <div class="diagnostic-panel">
        <h3>📊 Performance Metrics</h3>
        <div class="diagnostic-item" id="performanceMetrics">
          <div>Loading performance data...</div>
        </div>
        
        <div class="diagnostic-item" id="calibrationAccuracy">
          <div>Calibration accuracy: Checking...</div>
        </div>
      </div>

      <div class="diagnostic-panel">
        <h3>🌐 Network Status</h3>
        <div class="diagnostic-item" id="networkStatus">
          <div>WiFi Mode: Access Point</div>
          <div>SSID: TCS3430-ColorSensor</div>
          <div>IP: 192.168.4.1</div>
        </div>
        
        <div class="diagnostic-item" id="systemResources">
          <div>System uptime: --</div>
          <div>Free memory: -- KB</div>
        </div>
      </div>

      <div class="diagnostic-panel">
        <h3>📋 Raw Debug Data</h3>
        <div class="diagnostic-item">
          <button class="test-button" onclick="showDebugData()">📄 View Debug Info</button>
          <button class="test-button" onclick="showSPIFFSInfo()">💾 SPIFFS Status</button>
        </div>
        
        <div id="debugOutput" class="diagnostic-item" style="display: none; font-family: monospace; font-size: 0.9em; max-height: 300px; overflow-y: auto;">
          <div>Debug data will appear here...</div>
        </div>
      </div>
    </div>
  </div>

<script>
  async function loadDiagnostics() {
    try {
      // Load sensor status
      const statusRes = await fetch('/status');
      const statusData = await statusRes.json();
      
      // Load current color data
      const colorRes = await fetch('/color');
      const colorData = await colorRes.json();
      
      // Update device validation
      const deviceValid = statusData.deviceValid;
      const expectedDevice = statusData.expectedDeviceId === statusData.deviceId;
      
      document.getElementById('deviceValidation').innerHTML = `
        <div><span class="status-indicator ${deviceValid ? 'status-ok' : 'status-error'}"></span>Device ID: ${statusData.deviceId} ${expectedDevice ? '✅' : '❌'}</div>
        <div><span class="status-indicator ${statusData.revisionId === statusData.expectedRevisionId ? 'status-ok' : 'status-warning'}"></span>Revision: ${statusData.revisionId}</div>
        <div><span class="status-indicator status-ok"></span>I2C Address: ${statusData.i2cAddress}</div>
      `;
      
      // Update communication test
      document.getElementById('communicationTest').innerHTML = `
        <div><span class="status-indicator status-ok"></span>I2C Communication: ✅ Active</div>
        <div><span class="status-indicator ${colorData.saturated ? 'status-warning' : 'status-ok'}"></span>Sensor Status: ${colorData.saturated ? '⚠️ Saturated' : '✅ Normal'}</div>
      `;
      
      // Update performance metrics
      const spectroMatch = colorData.spectro_match;
      let accuracyStatus = 'Unknown';
      let accuracyClass = 'status-warning';
      
      if (spectroMatch) {
        const maxDiff = Math.max(Math.abs(spectroMatch.diff_X), Math.abs(spectroMatch.diff_Y), Math.abs(spectroMatch.diff_Z));
        if (maxDiff < 0.1) {
          accuracyStatus = '🎯 Excellent (±0.08)';
          accuracyClass = 'status-ok';
        } else if (maxDiff < 0.5) {
          accuracyStatus = '✅ Good (±0.5)';
          accuracyClass = 'status-ok';
        } else {
          accuracyStatus = '⚠️ Needs calibration';
          accuracyClass = 'status-warning';
        }
      }
      
      document.getElementById('performanceMetrics').innerHTML = `
        <div>Current gain: ${colorData.gain}x | Integration: ~${getIntegrationTime(colorData.integration_time)}ms</div>
        <div>High gain: ${colorData.high_gain ? 'Enabled' : 'Disabled'}</div>
        <div>Data rate: ~1Hz continuous</div>
      `;
      
      document.getElementById('calibrationAccuracy').innerHTML = `
        <div><span class="status-indicator ${accuracyClass}"></span>Accuracy: ${accuracyStatus}</div>
        <div>Scale factors: X=${colorData.calibration.x_scale.toFixed(4)}, Y=${colorData.calibration.y_scale.toFixed(4)}, Z=${colorData.calibration.z_scale.toFixed(4)}</div>
      `;
      
    } catch (error) {
      console.error('Diagnostics load failed:', error);
      document.getElementById('deviceValidation').innerHTML = '<div><span class="status-indicator status-error"></span>❌ Failed to load diagnostics</div>';
    }
  }

  function getIntegrationTime(intTime) {
    switch(intTime) {
      case 0xF6: return '25';
      case 0xEB: return '50';
      case 0xD6: return '100';
      case 0xC0: return '180';
      default: return 'Unknown';
    }
  }

  async function runSensorTest() {
    document.getElementById('communicationTest').innerHTML = '<div>🧪 Running sensor test...</div>';
    
    try {
      // Test multiple reads
      const results = [];
      for (let i = 0; i < 5; i++) {
        const res = await fetch('/color');
        const data = await res.json();
        results.push(data);
        await new Promise(resolve => setTimeout(resolve, 200));
      }
      
      // Calculate stability
      const xValues = results.map(r => r.Xr);
      const yValues = results.map(r => r.Yr);
      const zValues = results.map(r => r.Zr);
      
      const xStd = calculateStdDev(xValues);
      const yStd = calculateStdDev(yValues);
      const zStd = calculateStdDev(zValues);
      
      const stability = Math.max(xStd, yStd, zStd) < 10 ? '✅ Stable' : '⚠️ Unstable';
      
      document.getElementById('communicationTest').innerHTML = `
        <div><span class="status-indicator status-ok"></span>Test completed: 5 readings taken</div>
        <div><span class="status-indicator ${stability.includes('✅') ? 'status-ok' : 'status-warning'}"></span>Reading stability: ${stability}</div>
        <div>Std deviation: X=${xStd.toFixed(1)}, Y=${yStd.toFixed(1)}, Z=${zStd.toFixed(1)}</div>
      `;
      
    } catch (error) {
      document.getElementById('communicationTest').innerHTML = '<div><span class="status-indicator status-error"></span>❌ Sensor test failed</div>';
    }
  }

  function calculateStdDev(values) {
    const mean = values.reduce((sum, val) => sum + val, 0) / values.length;
    const variance = values.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / values.length;
    return Math.sqrt(variance);
  }

  async function showDebugData() {
    try {
      const response = await fetch('/debug');
      const debugText = await response.text();
      document.getElementById('debugOutput').innerHTML = '<pre>' + debugText + '</pre>';
      document.getElementById('debugOutput').style.display = 'block';
    } catch (error) {
      document.getElementById('debugOutput').innerHTML = '<div>❌ Failed to load debug data</div>';
      document.getElementById('debugOutput').style.display = 'block';
    }
  }

  async function showSPIFFSInfo() {
    try {
      const response = await fetch('/spiffs');
      const spiffsText = await response.text();
      document.getElementById('debugOutput').innerHTML = '<pre>' + spiffsText + '</pre>';
      document.getElementById('debugOutput').style.display = 'block';
    } catch (error) {
      document.getElementById('debugOutput').innerHTML = '<div>❌ Failed to load SPIFFS data</div>';
      document.getElementById('debugOutput').style.display = 'block';
    }
  }

  // Auto-refresh diagnostics
  loadDiagnostics();
  setInterval(loadDiagnostics, 10000);
</script>
</body>
</html>
)rawliteral";
    r->send(200, "text/html", diagnostics_html);
  });
  
  // Enhanced calibration interface
  server.on("/calibration", HTTP_GET, [](AsyncWebServerRequest* r){
    const char calibration_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Calibration - TCS3430</title>
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; }
    .container { max-width: 1200px; margin: 0 auto; }
    .header { text-align: center; margin-bottom: 30px; }
    .nav-button { display: inline-block; background: rgba(255,255,255,0.2); color: white; text-decoration: none; padding: 10px 20px; border-radius: 25px; margin: 5px; transition: all 0.3s ease; }
    .nav-button:hover { background: rgba(255,255,255,0.3); transform: translateY(-2px); }
    .calibration-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-bottom: 20px; }
    .calibration-panel { background: rgba(255,255,255,0.1); border-radius: 15px; padding: 25px; backdrop-filter: blur(10px); }
    .tile-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; }
    .tile-item { background: rgba(255,255,255,0.1); padding: 15px; border-radius: 10px; text-align: center; }
    .tile-button { background: #4CAF50; color: white; border: none; padding: 8px 16px; border-radius: 6px; cursor: pointer; margin: 2px; font-size: 14px; }
    .tile-button:hover { background: #45a049; }
    .current-reading { background: rgba(255,255,255,0.1); padding: 15px; border-radius: 10px; margin: 10px 0; font-family: monospace; }
    .warning-box { background: rgba(255,193,7,0.2); border: 2px solid #FFC107; padding: 15px; border-radius: 10px; margin: 20px 0; }
    @media (max-width: 900px) { .calibration-grid { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>🎯 BCRA Tile Calibration System</h1>
      <a href="/" class="nav-button">🏠 Home</a>
      <a href="/measurement" class="nav-button">📊 Measurement</a>
      <a href="/configuration" class="nav-button">⚙️ Configuration</a>
      <a href="/diagnostics" class="nav-button">🔧 Diagnostics</a>
    </div>

    <div class="warning-box">
      <h3>⚠️ Light Source Analysis</h3>
      <p><strong>Important:</strong> Spectrophotometer reference values use D65 (6500K daylight). Your readings depend on your current light source.</p>
      <p><strong>First step:</strong> Measure the White tile to check your light source CCT and adjust accordingly.</p>
    </div>

    <div class="calibration-grid">
      <div class="calibration-panel">
        <h3>📏 Current Reading</h3>
        <div class="current-reading" id="currentReading">
          <div>Place tile under sensor and click 'Read Current' to update</div>
          <button class="tile-button" onclick="readCurrent()">🔄 Read Current</button>
        </div>

        <h4>💡 Light Source Check</h4>
        <div id="lightSourceInfo" class="current-reading">
          <div>CCT: -- K</div>
          <div>Light type: Unknown</div>
        </div>
      </div>

      <div class="calibration-panel">
        <h3>🎨 BCRA Tile References</h3>
        <div class="tile-grid">
          <div class="tile-item">
            <h4>⚪ White Tile</h4>
            <button class="tile-button" onclick="setReference('white')">📌 Set White</button>
            <div id="whiteStatus">Not calibrated</div>
          </div>
          
          <div class="tile-item">
            <h4>⚫ Black Tile</h4>
            <button class="tile-button" onclick="setReference('black')">📌 Set Black</button>
            <div id="blackStatus">Not calibrated</div>
          </div>
          
          <div class="tile-item">
            <h4>🟢 Green Tile</h4>
            <button class="tile-button" onclick="setReference('green')">📌 Set Green</button>
            <div id="greenStatus">Not calibrated</div>
          </div>
          
          <div class="tile-item">
            <h4>🔴 Red Tile</h4>
            <button class="tile-button" onclick="setReference('red')">📌 Set Red</button>
            <div id="redStatus">Not calibrated</div>
          </div>
          
          <div class="tile-item">
            <h4>🟠 Orange Tile</h4>
            <button class="tile-button" onclick="setReference('orange')">📌 Set Orange</button>
            <div id="orangeStatus">Not calibrated</div>
          </div>
          
          <div class="tile-item">
            <h4>🟡 Yellow Tile</h4>
            <button class="tile-button" onclick="setReference('yellow')">📌 Set Yellow</button>
            <div id="yellowStatus">Not calibrated</div>
          </div>
          
          <div class="tile-item">
            <h4>🩶 Mid Grey</h4>
            <button class="tile-button" onclick="setReference('midgrey')">📌 Set Grey</button>
            <div id="midgreyStatus">Not calibrated</div>
          </div>
          
          <div class="tile-item">
            <h4>🩷 Deep Pink</h4>
            <button class="tile-button" onclick="setReference('deeppink')">📌 Set Pink</button>
            <div id="deeppinkStatus">Not calibrated</div>
          </div>
        </div>
      </div>
    </div>

    <div class="calibration-panel">
      <h3>🎯 Spectrophotometer Validation</h3>
      <div id="spectroComparison" class="current-reading">
        <div>Place calibrated sample to see comparison with spectrophotometer reference values</div>
      </div>
    </div>
  </div>

<script>
  let currentData = null;

  async function readCurrent() {
    try {
      const response = await fetch('/color');
      currentData = await response.json();
      
      const reading = `
        Raw: X=${currentData.Xr}, Y=${currentData.Yr}, Z=${currentData.Zr}
        XYZ: X=${currentData.X.toFixed(3)}, Y=${currentData.Y.toFixed(3)}, Z=${currentData.Z.toFixed(3)}
        Chromaticity: x=${currentData.x.toFixed(4)}, y=${currentData.y.toFixed(4)}
        Status: ${currentData.saturated ? '⚠️ SATURATED' : '✅ OK'}
      `;
      
      document.getElementById('currentReading').innerHTML = reading.trim().split('\n').map(line => `<div>${line}</div>`).join('') + 
        '<button class="tile-button" onclick="readCurrent()">🔄 Read Current</button>';
      
      // Update light source info
      const cct = currentData.CCT > 0 ? Math.round(currentData.CCT) : 'Unknown';
      let lightType = 'Unknown';
      if (currentData.CCT > 0) {
        if (currentData.CCT < 3000) lightType = 'Warm tungsten';
        else if (currentData.CCT < 4000) lightType = 'Warm white';
        else if (currentData.CCT < 5000) lightType = 'Cool white';
        else if (currentData.CCT < 6000) lightType = 'Daylight';
        else if (currentData.CCT < 7000) lightType = 'Cool daylight';
        else lightType = 'Cool fluorescent';
      }
      
      document.getElementById('lightSourceInfo').innerHTML = `
        <div>CCT: ${cct} K</div>
        <div>Light type: ${lightType}</div>
      `;
      
      // Update spectrophotometer comparison
      if (currentData.spectro_match) {
        const match = currentData.spectro_match;
        const comparison = `
          <div><strong>Closest Match:</strong> ${match.name}</div>
          <div><strong>Target XYZ:</strong> X=${match.target_X.toFixed(2)}, Y=${match.target_Y.toFixed(2)}, Z=${match.target_Z.toFixed(2)}</div>
          <div><strong>Measured XYZ:</strong> X=${currentData.X.toFixed(2)}, Y=${currentData.Y.toFixed(2)}, Z=${currentData.Z.toFixed(2)}</div>
          <div><strong>Difference:</strong> ΔX=${match.diff_X.toFixed(2)}, ΔY=${match.diff_Y.toFixed(2)}, ΔZ=${match.diff_Z.toFixed(2)}</div>
          <div><strong>Accuracy:</strong> ${Math.abs(match.diff_X) < 0.1 && Math.abs(match.diff_Y) < 0.1 && Math.abs(match.diff_Z) < 0.1 ? '🎯 Excellent (±0.1)' : 
                                         Math.abs(match.diff_X) < 0.5 && Math.abs(match.diff_Y) < 0.5 && Math.abs(match.diff_Z) < 0.5 ? '✅ Good (±0.5)' : '⚠️ Needs calibration'}</div>
        `;
        document.getElementById('spectroComparison').innerHTML = comparison.trim().split('\\n').map(line => `<div>${line}</div>`).join('');
      } else {
        // Fallback if no spectro_match data
        document.getElementById('spectroComparison').innerHTML = `
          <div><strong>Real-time measurement:</strong> X=${currentData.X.toFixed(2)}, Y=${currentData.Y.toFixed(2)}, Z=${currentData.Z.toFixed(2)}</div>
          <div><strong>Lab values:</strong> L*=${currentData.L ? currentData.L.toFixed(1) : '--'}, a*=${currentData.a ? currentData.a.toFixed(1) : '--'}, b*=${currentData.b ? currentData.b.toFixed(1) : '--'}</div>
          <div>⚠️ Spectro matching unavailable - check JSON response</div>
        `;
      }
      
    } catch (error) {
      document.getElementById('currentReading').innerHTML = '<div>❌ Error reading sensor: ' + error.message + '</div>';
    }
  }

  async function setReference(tileName) {
    if (!currentData) {
      alert('Please read current values first');
      return;
    }
    
    if (currentData.saturated) {
      if (!confirm('Sensor is saturated! This may affect calibration accuracy. Continue?')) {
        return;
      }
    }
    
    try {
      const response = await fetch(`/set${tileName}`);
      const result = await response.text();
      
      const statusElementId = tileName + 'Status';
      console.log('Looking for status element:', statusElementId);
      const statusElement = document.getElementById(statusElementId);
      
      if (statusElement) {
        console.log('Found status element, updating...');
        statusElement.innerHTML = `✅ X=${currentData.Xr}, Y=${currentData.Yr}, Z=${currentData.Zr}`;
        statusElement.style.fontSize = '0.8em';
        statusElement.style.color = '#000000'; // Changed to black for better readability
      } else {
        console.error('Status element not found:', statusElementId);
        // Try to find all elements with IDs containing 'Status'
        const allElements = document.querySelectorAll('[id*="Status"]');
        console.log('Available status elements:', Array.from(allElements).map(el => el.id));
      }
      
      alert(`${tileName.charAt(0).toUpperCase() + tileName.slice(1)} reference set successfully!`);
      
    } catch (error) {
      alert('Failed to set reference: ' + error.message);
    }
  }

  // Auto-refresh current reading
  readCurrent();
  setInterval(readCurrent, 3000);
</script>
</body>
</html>
)rawliteral";
    r->send(200, "text/html", calibration_html);
  });
  
  // Diagnostics page
  server.on("/diagnostics", HTTP_GET, [](AsyncWebServerRequest* r){
    const char diagnostics_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Diagnostics - TCS3430</title>
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; }
    .container { max-width: 1200px; margin: 0 auto; }
    .header { text-align: center; margin-bottom: 30px; }
    .nav-button { display: inline-block; background: rgba(255,255,255,0.2); color: white; text-decoration: none; padding: 10px 20px; border-radius: 25px; margin: 5px; transition: all 0.3s ease; }
    .nav-button:hover { background: rgba(255,255,255,0.3); transform: translateY(-2px); }
    .diagnostics-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
    .diagnostic-panel { background: rgba(255,255,255,0.1); border-radius: 15px; padding: 25px; backdrop-filter: blur(10px); }
    .status-indicator { display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 8px; }
    .status-ok { background: #4CAF50; }
    .status-warning { background: #FF9800; }
    .status-error { background: #f44336; }
    .diagnostic-item { background: rgba(255,255,255,0.1); padding: 15px; border-radius: 10px; margin: 10px 0; }
    .diagnostic-value { font-weight: bold; font-size: 1.2em; }
    .test-button { background: #2196F3; color: white; border: none; padding: 10px 15px; border-radius: 8px; cursor: pointer; margin: 5px; }
    .test-button:hover { background: #1976D2; }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>🔧 System Diagnostics</h1>
      <a href="/" class="nav-button">🏠 Home</a>
      <a href="/measurement" class="nav-button">📊 Measurement</a>
      <a href="/configuration" class="nav-button">⚙️ Configuration</a>
      <a href="/calibration" class="nav-button">🎯 Calibration</a>
    </div>

    <div class="diagnostics-grid">
      <div class="diagnostic-panel">
        <h3>🔍 Sensor Validation</h3>
        <div class="diagnostic-item" id="deviceValidation">
          <div>Loading sensor validation...</div>
        </div>
        
        <div class="diagnostic-item" id="communicationTest">
          <div>I2C Communication: Testing...</div>
        </div>
        
        <button class="test-button" onclick="runSensorTest()">🧪 Run Sensor Test</button>
      </div>

      <div class="diagnostic-panel">
        <h3>📊 Performance Metrics</h3>
        <div class="diagnostic-item" id="performanceMetrics">
          <div>Loading performance data...</div>
        </div>
        
        <div class="diagnostic-item" id="calibrationAccuracy">
          <div>Calibration accuracy: Checking...</div>
        </div>
      </div>

      <div class="diagnostic-panel">
        <h3>🌐 Network Status</h3>
        <div class="diagnostic-item" id="networkStatus">
          <div>WiFi Mode: Access Point</div>
          <div>SSID: TCS3430-ColorSensor</div>
          <div>IP: 192.168.4.1</div>
        </div>
        
        <div class="diagnostic-item" id="systemResources">
          <div>System uptime: --</div>
          <div>Free memory: -- KB</div>
        </div>
      </div>

      <div class="diagnostic-panel">
        <h3>📋 Raw Debug Data</h3>
        <div class="diagnostic-item">
          <button class="test-button" onclick="showDebugData()">📄 View Debug Info</button>
          <button class="test-button" onclick="showSPIFFSInfo()">💾 SPIFFS Status</button>
        </div>
        
        <div id="debugOutput" class="diagnostic-item" style="display: none; font-family: monospace; font-size: 0.9em; max-height: 300px; overflow-y: auto;">
          <div>Debug data will appear here...</div>
        </div>
      </div>
    </div>
  </div>

<script>
  async function loadDiagnostics() {
    try {
      // Load sensor status
      const statusRes = await fetch('/status');
      const statusData = await statusRes.json();
      
      // Load current color data
      const colorRes = await fetch('/color');
      const colorData = await colorRes.json();
      
      // Update device validation
      const deviceValid = statusData.deviceValid;
      const expectedDevice = statusData.expectedDeviceId === statusData.deviceId;
      
      document.getElementById('deviceValidation').innerHTML = \`
        <div><span class="status-indicator \${deviceValid ? 'status-ok' : 'status-error'}"></span>Device ID: \${statusData.deviceId} \${expectedDevice ? '✅' : '❌'}</div>
        <div><span class="status-indicator \${statusData.revisionId === statusData.expectedRevisionId ? 'status-ok' : 'status-warning'}"></span>Revision: \${statusData.revisionId}</div>
        <div><span class="status-indicator status-ok"></span>I2C Address: \${statusData.i2cAddress}</div>
      \`;
      
      // Update communication test
      document.getElementById('communicationTest').innerHTML = \`
        <div><span class="status-indicator status-ok"></span>I2C Communication: ✅ Active</div>
        <div><span class="status-indicator \${colorData.saturated ? 'status-warning' : 'status-ok'}"></span>Sensor Status: \${colorData.saturated ? '⚠️ Saturated' : '✅ Normal'}</div>
      \`;
      
      // Update performance metrics
      const spectroMatch = colorData.spectro_match;
      let accuracyStatus = 'Unknown';
      let accuracyClass = 'status-warning';
      
      if (spectroMatch) {
        const maxDiff = Math.max(Math.abs(spectroMatch.diff_X), Math.abs(spectroMatch.diff_Y), Math.abs(spectroMatch.diff_Z));
        if (maxDiff < 0.1) {
          accuracyStatus = '🎯 Excellent (±0.08)';
          accuracyClass = 'status-ok';
        } else if (maxDiff < 0.5) {
          accuracyStatus = '✅ Good (±0.5)';
          accuracyClass = 'status-ok';
        } else {
          accuracyStatus = '⚠️ Needs calibration';
          accuracyClass = 'status-warning';
        }
      }
      
      document.getElementById('performanceMetrics').innerHTML = \`
        <div>Current gain: \${colorData.gain}x | Integration: ~\${getIntegrationTime(colorData.integration_time)}ms</div>
        <div>High gain: \${colorData.high_gain ? 'Enabled' : 'Disabled'}</div>
        <div>Data rate: ~1Hz continuous</div>
      \`;
      
      document.getElementById('calibrationAccuracy').innerHTML = \`
        <div><span class="status-indicator \${accuracyClass}"></span>Accuracy: \${accuracyStatus}</div>
        <div>Scale factors: X=\${colorData.calibration.x_scale.toFixed(4)}, Y=\${colorData.calibration.y_scale.toFixed(4)}, Z=\${colorData.calibration.z_scale.toFixed(4)}</div>
      \`;
      
    } catch (error) {
      console.error('Diagnostics load failed:', error);
      document.getElementById('deviceValidation').innerHTML = '<div><span class="status-indicator status-error"></span>❌ Failed to load diagnostics</div>';
    }
  }

  function getIntegrationTime(intTime) {
    switch(intTime) {
      case 0xF6: return '25';
      case 0xEB: return '50';
      case 0xD6: return '100';
      case 0xC0: return '180';
      default: return 'Unknown';
    }
  }

  async function runSensorTest() {
    document.getElementById('communicationTest').innerHTML = '<div>🧪 Running sensor test...</div>';
    
    try {
      // Test multiple reads
      const results = [];
      for (let i = 0; i < 5; i++) {
        const res = await fetch('/color');
        const data = await res.json();
        results.push(data);
        await new Promise(resolve => setTimeout(resolve, 200));
      }
      
      // Calculate stability
      const xValues = results.map(r => r.Xr);
      const yValues = results.map(r => r.Yr);
      const zValues = results.map(r => r.Zr);
      
      const xStd = calculateStdDev(xValues);
      const yStd = calculateStdDev(yValues);
      const zStd = calculateStdDev(zValues);
      
      const stability = Math.max(xStd, yStd, zStd) < 10 ? '✅ Stable' : '⚠️ Unstable';
      
      document.getElementById('communicationTest').innerHTML = \`
        <div><span class="status-indicator status-ok"></span>Test completed: 5 readings taken</div>
        <div><span class="status-indicator \${stability.includes('✅') ? 'status-ok' : 'status-warning'}"></span>Reading stability: \${stability}</div>
        <div>Std deviation: X=\${xStd.toFixed(1)}, Y=\${yStd.toFixed(1)}, Z=\${zStd.toFixed(1)}</div>
      \`;
      
    } catch (error) {
      document.getElementById('communicationTest').innerHTML = '<div><span class="status-indicator status-error"></span>❌ Sensor test failed</div>';
    }
  }

  function calculateStdDev(values) {
    const mean = values.reduce((sum, val) => sum + val, 0) / values.length;
    const variance = values.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / values.length;
    return Math.sqrt(variance);
  }

  async function showDebugData() {
    try {
      const response = await fetch('/debug');
      const debugText = await response.text();
      document.getElementById('debugOutput').innerHTML = '<pre>' + debugText + '</pre>';
      document.getElementById('debugOutput').style.display = 'block';
    } catch (error) {
      document.getElementById('debugOutput').innerHTML = '<div>❌ Failed to load debug data</div>';
      document.getElementById('debugOutput').style.display = 'block';
    }
  }

  async function showSPIFFSInfo() {
    try {
      const response = await fetch('/spiffs');
      const spiffsText = await response.text();
      document.getElementById('debugOutput').innerHTML = '<pre>' + spiffsText + '</pre>';
      document.getElementById('debugOutput').style.display = 'block';
    } catch (error) {
      document.getElementById('debugOutput').innerHTML = '<div>❌ Failed to load SPIFFS data</div>';
      document.getElementById('debugOutput').style.display = 'block';
    }
  }

  // Auto-refresh diagnostics
  loadDiagnostics();
  setInterval(loadDiagnostics, 10000);
</script>
</body>
</html>
)rawliteral";
    r->send(200, "text/html", diagnostics_html);
  });
  
  // Sensor status and diagnostics endpoint  
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* r){
    uint8_t deviceId = getDeviceID();
    uint8_t revisionId = getRevisionID();
    uint8_t status = getDeviceStatus();
    bool saturated = isSaturated();
    
    String j = "{";
    j += String("\"deviceId\":\"0x") + String(deviceId, HEX) + "\",";
    j += String("\"expectedDeviceId\":\"0x") + String(TCS3430_DEVICE_ID, HEX) + "\",";
    j += String("\"revisionId\":\"0x") + String(revisionId, HEX) + "\",";
    j += String("\"expectedRevisionId\":\"0x") + String(TCS3430_REVISION_ID, HEX) + "\",";
    j += String("\"status\":\"0x") + String(status, HEX) + "\",";
    j += String("\"saturated\":") + (saturated ? "true" : "false") + ",";
    j += String("\"deviceValid\":") + (deviceId == TCS3430_DEVICE_ID ? "true" : "false") + ",";
    j += String("\"integrationTime\":") + String(INTEG_TIME) + ",";
    j += String("\"gain\":") + String(GAIN) + ",";
    j += String("\"i2cAddress\":\"0x") + String(TCS3430_ADDR, HEX) + "\"";
    j += "}";
    r->send(200, "application/json", j);
  });
  
  // Measurement endpoint for API compatibility (same as /color)
  server.on("/measurement", HTTP_GET, handleColor);
  
  // Serve the CIE diagram image from SPIFFS
  server.on("/cie.jpg", HTTP_GET, [](AsyncWebServerRequest* r){
    Serial.println("CIE diagram image requested");
    if (SPIFFS.exists("/cie.jpg")) {
      r->send(SPIFFS, "/cie.jpg", "image/jpeg");
    } else {
      Serial.println("CIE image not found in SPIFFS");
      r->send(404, "text/plain", "CIE diagram image not found");
    }
  });
  
  // Debug endpoint to list SPIFFS files
  server.on("/spiffs", HTTP_GET, [](AsyncWebServerRequest* r){
    String response = "=== SPIFFS File System Status ===\n";
    
    if (!SPIFFS.begin()) {
      response += "SPIFFS failed to mount!\n";
      r->send(500, "text/plain", response);
      return;
    }
    
    response += "SPIFFS mounted successfully\n";
    response += "Total bytes: " + String(SPIFFS.totalBytes()) + "\n";
    response += "Used bytes: " + String(SPIFFS.usedBytes()) + "\n";
    response += "Free bytes: " + String(SPIFFS.totalBytes() - SPIFFS.usedBytes()) + "\n\n";
    
    response += "Files in SPIFFS:\n";
    File root = SPIFFS.open("/");
    if (root) {
      File file = root.openNextFile();
      int fileCount = 0;
      while (file) {
        fileCount++;
        response += String(fileCount) + ". " + String(file.name()) + " (" + String(file.size()) + " bytes)\n";
        file = root.openNextFile();
      }
      if (fileCount == 0) {
        response += "No files found in SPIFFS\n";
      }
    } else {
      response += "Failed to open root directory\n";
    }
    
    response += "\nSpecific check for cie.jpg:\n";
    if (SPIFFS.exists("/cie.jpg")) {
      File cieFile = SPIFFS.open("/cie.jpg", "r");
      if (cieFile) {
        response += "✓ cie.jpg found! Size: " + String(cieFile.size()) + " bytes\n";
        cieFile.close();
      } else {
        response += "✗ cie.jpg exists but cannot be opened\n";
      }
    } else {
      response += "✗ cie.jpg NOT found in SPIFFS\n";
    }
    
    r->send(200, "text/plain", response);
  }); 

  // Keep the debug endpoint for troubleshooting
  server.on("/debug", HTTP_GET, [](AsyncWebServerRequest* r){
    uint8_t samples;
    uint16_t delaySeconds;
    bool useIR2;
    parseAvgParams(r, samples, delaySeconds, useIR2);

    DEBUG_IR2_SERIAL = r->hasParam("serial");
    useIR2 = true;

    Serial.print("Reset reason: ");
    Serial.println(esp_reset_reason());

    SensorAvg s = readAveragedSensor(samples, delaySeconds, useIR2);
    uint16_t Xr = (uint16_t)s.X;
    uint16_t Yr = (uint16_t)s.Y;
    uint16_t Zr = (uint16_t)s.Z;
    uint16_t IR1r = (uint16_t)s.IR1;
    uint16_t IR2r = (uint16_t)s.IR2;
    uint8_t atime = readSingleByte(ATIME_REG);
  uint8_t cfg1  = readSingleByte(CFG1_REG);
  uint8_t cfg3  = readSingleByte(CFG3_REG);


  uint8_t again = cfg1 & 0x03;                 // AGAIN bits
  bool amux     = (cfg1 & (1 << 3)) != 0;      // AMUX bit
  bool hgain    = (cfg3 & 0x10) != 0;           // HGAIN bit
    
    String response = "=== TCS3430 Color Sensor Debug ===\n";
    Serial.println(">>> /debug called <<<");
    Serial.println("=== FIRMWARE: DEBUG SERIAL ENABLED ===");
    response += "Averaging: samples=" + String(samples) + ", delay=" + String(delaySeconds) + "s, IR2=" + String(useIR2 ? "ON" : "OFF") + "\n\n";
    //response += "Raw Readings: X=" + String(Xr) + ", Y=" + String(Yr) + ", Z=" + String(Zr) + "\n";
    //response += "Sum: " + String(Xr + Yr + Zr) + "\n\n";
    response += "AMSSensCount (register counts): X=" + String(Xr) + ", Y=" + String(Yr) + ", Z=" + String(Zr) +
               ", IR1=" + String(IR1r) + ", IR2=" + String(IR2r) + "\n";
    response += "Sum (XYZ): " + String(Xr + Yr + Zr) + "\n";
    response += "Sum (XYZ+IR1+IR2): " + String((uint32_t)Xr + Yr + Zr + IR1r + IR2r) + "\n\n";

    response += "Calibration Points:\n";
    response += "White:  X=" + String(rawWhiteX) + ", Y=" + String(rawWhiteY) + ", Z=" + String(rawWhiteZ) + "\n";
    response += "Black:  X=" + String(rawBlackX) + ", Y=" + String(rawBlackY) + ", Z=" + String(rawBlackZ) + "\n";
    response += "Green:  X=" + String(rawGreenX) + ", Y=" + String(rawGreenY) + ", Z=" + String(rawGreenZ) + "\n";
    response += "Red:    X=" + String(rawRedX) + ", Y=" + String(rawRedY) + ", Z=" + String(rawRedZ) + "\n";
    response += "Orange: X=" + String(rawOrangeX) + ", Y=" + String(rawOrangeY) + ", Z=" + String(rawOrangeZ) + "\n";
    response += "Yellow: X=" + String(rawYellowX) + ", Y=" + String(rawYellowY) + ", Z=" + String(rawYellowZ) + "\n";
    response += "Grey:   X=" + String(rawGreyX) + ", Y=" + String(rawGreyY) + ", Z=" + String(rawGreyZ) + "\n";
    response += "Pink:   X=" + String(rawPinkX) + ", Y=" + String(rawPinkY) + ", Z=" + String(rawPinkZ) + "\n\n";
    
    response += "Channel Ranges:\n";
    response += "X Range: " + String(rawWhiteX - rawBlackX) + " (current: " + String(Xr - rawBlackX) + ")\n";
    response += "Y Range: " + String(rawWhiteY - rawBlackY) + " (current: " + String(Yr - rawBlackY) + ")\n";
    response += "Z Range: " + String(rawWhiteZ - rawBlackZ) + " (current: " + String(Zr - rawBlackZ) + ")\n\n";
    
    response += "\nSensor Configuration:\n";
    response += "ATIME = 0x" + String(atime, HEX) + "\n";
    response += "CFG1  = 0x" + String(cfg1, HEX) +
              "  (GAIN=" + String(gainToStr(again)) +
              ", AMUX=" + String(amux ? "IR2" : "X") + ")\n";
    response += "CFG3  = 0x" + String(cfg3, HEX) +
              "  (HGAIN=" + String(hgain ? "ENABLED" : "DISABLED") + ")\n";

// --- Fresh CH3 reads to validate AMUX switching ---
  uint16_t xFresh_before = read16(XDATA_L);  // whatever CH3 currently is
  uint16_t ir2Fresh = 0;

  {
    uint32_t integMs = integrationMsFromATIME(INTEG_TIME);
   uint32_t waitMs  = integMs + 5;

    // Force an IR2 read using AMUX switching path
    ir2Fresh = readIR2Raw();

    // After restoring AMUX=0, take a fresh X reading
    delay(waitMs);
  ( void)read16(XDATA_L);
  }

  // response += "\n--- AMUX Switch Test (CH3) ---\n";
  // response += "CH3 (before switch) = " + String(xFresh_before) + "\n";
  // response += "IR2 (fresh)         = " + String(ir2Fresh) + "\n";

response += "\n--- AMUX Switch Test (CH3 fresh) ---\n";
uint16_t x1  = readXRawFresh();
uint16_t ir2 = readIR2RawFresh();
uint16_t x2  = readXRawFresh();

response += "X (fresh1)  = " + String(x1) + "\n";
response += "IR2 (fresh) = " + String(ir2) + "\n";
response += "X (fresh2)  = " + String(x2) + "\n";

    // Counts chromaticity (reference only: computed directly from sensor register counts)
    float sumCounts = (float)Xr + (float)Yr + (float)Zr;
    if (sumCounts <= 1e-6f) sumCounts = 1.0f;
    response += "Counts Chromaticity (ref): x=" + String(((float)Xr) / sumCounts, 4) +
                ", y=" + String(((float)Yr) / sumCounts, 4) + "\n\n";

    // ---- AMS pipeline naming ----
    // AMSSensCount: (Xr,Yr,Zr,IR1r,IR2r) are direct sensor register counts (averaged if samples>1)
    // RawXYZ: matrix output (before k-factors)
    float X_rawXYZ = 0.0f, Y_rawXYZ = 0.0f, Z_rawXYZ = 0.0f;
    transformToXYZ(Xr, Yr, Zr, X_rawXYZ, Y_rawXYZ, Z_rawXYZ);

    response += "RawXYZ (matrix output): X=" + String(X_rawXYZ, 2) +
                ", Y=" + String(Y_rawXYZ, 2) +
                ", Z=" + String(Z_rawXYZ, 2) + "\n";

    // RawXY derived ONLY from RawXYZ (matrix output)
    float sumRawXYZ = X_rawXYZ + Y_rawXYZ + Z_rawXYZ;
    float rawXYZ_x = (sumRawXYZ > 1e-6f) ? (X_rawXYZ / sumRawXYZ) : 0.0f;
    float rawXYZ_y = (sumRawXYZ > 1e-6f) ? (Y_rawXYZ / sumRawXYZ) : 0.0f;

    response += "RawXY (from RawXYZ): x=" + String(rawXYZ_x, 4) +
                ", y=" + String(rawXYZ_y, 4) + "\n";

    // CCT computed ONLY from RawXYZ chromaticity (McCamy approximation)
    float CCTraw = 0.0f;
    if (rawXYZ_x > 0.01f && rawXYZ_y > 0.01f && rawXYZ_x < 0.9f && rawXYZ_y < 0.9f && fabsf(0.1858f - rawXYZ_y) > 1e-6f) {
      float n = (rawXYZ_x - 0.3320f) / (0.1858f - rawXYZ_y);
      float n2 = n * n;
      float n3 = n2 * n;
      CCTraw = 449.0f*n3 + 3525.0f*n2 + 6823.3f*n + 5520.33f;
      if (CCTraw < 100.0f || CCTraw > 100000.0f || isnan(CCTraw) || isinf(CCTraw)) CCTraw = 0.0f;
    }

    // Lux display tied to RawXYZ Y for consistency with the RawXYZ pipeline
    float luxRaw = Y_rawXYZ;

    response += "CCT(raw from RawXYZ) = " + String(CCTraw, 0) +
                " K  Lux(rawXYZ) = " + String(luxRaw, 1) + "\n";

    // CalXYZ: AMS-style 2-point calibration in XYZ space (Black/White tiles)
    float X_calXYZ, Y_calXYZ, Z_calXYZ;
    computeCalXYZ_2Point(X_rawXYZ, Y_rawXYZ, Z_rawXYZ, X_calXYZ, Y_calXYZ, Z_calXYZ);

    response += "CalXYZ (2-point): X=" + String(X_calXYZ, 2) +
                ", Y=" + String(Y_calXYZ, 2) +
                ", Z=" + String(Z_calXYZ, 2) + "\n";

    float sumCal = X_calXYZ + Y_calXYZ + Z_calXYZ;
    float cal_x = (sumCal > 1e-6f) ? (X_calXYZ / sumCal) : 0.0f;
    float cal_y = (sumCal > 1e-6f) ? (Y_calXYZ / sumCal) : 0.0f;

    response += "CalXY (from CalXYZ): x=" + String(cal_x, 4) +
                ", y=" + String(cal_y, 4) + "\n";

    float luxCal = Y_calXYZ * 0.2f;
    response += "Lux(calXYZ) = " + String(luxCal, 1) + "\n\n";
response += "System Info:\n";
    response += "Chip ID: " + String((uint32_t)ESP.getEfuseMac(), HEX) + "\n";
    response += "Free Heap: " + String(ESP.getFreeHeap()) + " bytes\n";
    response += "WiFi Status: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected") + "\n";
    if (WiFi.status() == WL_CONNECTED) {
      response += "IP Address: " + WiFi.localIP().toString() + "\n";
      response += "Signal Strength: " + String(WiFi.RSSI()) + " dBm\n";
    }

// --- Calibration state (k-factors + 4x3 matrix) ---
response += "\n--- Calibration State ---\n";
response += "kX=" + String(kX, 6) + "  kY=" + String(kY, 6) + "  kZ=" + String(kZ, 6) + "\n";
response += "calibMatrix rows [Xraw,Yraw,Zraw,1] cols [X,Y,Z]  (row3 is constant offset)\n";
for (int rr = 0; rr < 4; rr++) {
  response += "row " + String(rr) + ": ";
  for (int cc = 0; cc < 3; cc++) {
    response += String(calibMatrix[rr][cc], 8);
    if (cc < 2) response += ", ";
  }
  response += "\n";
}
response += "\n";

// Also print to Serial so you can capture results from USB even when your PC isn't on 192.168.4.x
Serial.println(response);
    r->send(200, "text/plain", response);
  });
  
  // Calibration endpoint for setting new white/black reference points
  server.on("/setwhite", HTTP_GET, [](AsyncWebServerRequest* r){
    uint8_t samples;
    uint16_t delaySeconds;
    bool useIR2;
    parseAvgParams(r, samples, delaySeconds, useIR2);

    SensorAvg s = readAveragedSensor(samples, delaySeconds, useIR2);
    uint16_t Xr = (uint16_t)s.X;
    uint16_t Yr = (uint16_t)s.Y;
    uint16_t Zr = (uint16_t)s.Z;
    uint16_t IR1r = (uint16_t)s.IR1;
    uint16_t IR2r = (uint16_t)s.IR2;
    
    
    rawWhiteX = Xr;
    rawWhiteY = Yr;
    rawWhiteZ = Zr;
    rawWhiteIR1 = IR1r;
    rawWhiteIR2 = IR2r;
   
    saveSettings(); // Auto-save calibration
    
    //String response = "White reference set to: X=" + String(Xr) + ", Y=" + String(Yr) + ", Z=" + String(Zr) + "\n";
    String response = "White reference set to: X=" + String(Xr) + ", Y=" + String(Yr) + ", Z=" + String(Zr) +
                      ", IR1=" + String(IR1r) + ", IR2=" + String(IR2r) + "\n";
    response += "Settings automatically saved to flash memory.\n";
    Serial.printf("NEW WHITE CALIBRATION SAVED: X=%u, Y=%u, Z=%u\n", Xr, Yr, Zr);
    r->send(200, "text/plain", response);
  });
  
  server.on("/setblack", HTTP_GET, [](AsyncWebServerRequest* r){
    uint8_t samples;
    uint16_t delaySeconds;
    bool useIR2;
    parseAvgParams(r, samples, delaySeconds, useIR2);

    SensorAvg s = readAveragedSensor(samples, delaySeconds, useIR2);
    uint16_t Xr = (uint16_t)s.X;
    uint16_t Yr = (uint16_t)s.Y;
    uint16_t Zr = (uint16_t)s.Z;
    uint16_t IR1r = (uint16_t)s.IR1;
    uint16_t IR2r = (uint16_t)s.IR2;
    
    rawBlackX = Xr;
    rawBlackY = Yr;
    rawBlackZ = Zr;
    rawBlackIR1 = IR1r;
    rawBlackIR2 = IR2r;
    
    saveSettings(); // Auto-save calibration
    
    //String response = "Black reference set to: X=" + String(Xr) + ", Y=" + String(Yr) + ", Z=" + String(Zr) + "\n";
    String response = "Black reference set to: X=" + String(Xr) + ", Y=" + String(Yr) + ", Z=" + String(Zr) +
                      ", IR1=" + String(IR1r) + ", IR2=" + String(IR2r) + "\n";
    response += "Settings automatically saved to flash memory.\n";
    //Serial.printf("NEW BLACK CALIBRATION SAVED: X=%u, Y=%u, Z=%u\n", Xr, Yr, Zr);
    Serial.printf("NEW BLACK CALIBRATION SAVED: X=%u, Y=%u, Z=%u, IR1=%u, IR2=%u\n", Xr, Yr, Zr, IR1r, IR2r);
    DEBUG_IR2_SERIAL = false;
    r->send(200, "text/plain", response);
  });
  
  server.on("/setgreen", HTTP_GET, [](AsyncWebServerRequest* r){
    uint8_t samples;
    uint16_t delaySeconds;
    bool useIR2;
    parseAvgParams(r, samples, delaySeconds, useIR2);

    SensorAvg s = readAveragedSensor(samples, delaySeconds, useIR2);
    uint16_t Xr = (uint16_t)s.X;
    uint16_t Yr = (uint16_t)s.Y;
    uint16_t Zr = (uint16_t)s.Z;
    uint16_t IR1r = (uint16_t)s.IR1;
    uint16_t IR2r = (uint16_t)s.IR2;

    rawGreenX = Xr;
    rawGreenY = Yr;
    rawGreenZ = Zr;
    rawGreenIR1 = IR1r;
    rawGreenIR2 = IR2r;
    
    saveSettings(); // Auto-save calibration
    
    String response = "Green reference set to: X=" + String(Xr) + ", Y=" + String(Yr) + ", Z=" + String(Zr) +
                     ", IR1=" + String(IR1r) + ", IR2=" + String(IR2r) + "\n";
    response += "Settings automatically saved to flash memory.\n";
    Serial.printf("NEW GREEN CALIBRATION SAVED: X=%u, Y=%u, Z=%u, IR1=%u, IR2=%u\n", Xr, Yr, Zr, IR1r, IR2r);
    r->send(200, "text/plain", response);
  });
  
  // Settings management endpoints
  server.on("/settings/save", HTTP_GET, [](AsyncWebServerRequest* r){
    bool success = saveSettings();
    String response = "{\"success\":" + String(success ? "true" : "false") + ",";
    response += "\"message\":\"" + String(success ? "Settings saved successfully" : "Failed to save settings") + "\"}";
    r->send(200, "application/json", response);
  });
  
  server.on("/settings/load", HTTP_GET, [](AsyncWebServerRequest* r){
    bool success = loadSettings();
    if (success) {
      // Reapply sensor settings
      setGain(GAIN);
      Wire.beginTransmission(TCS3430_ADDR);
      Wire.write(CMD_BIT | ATIME_REG);
      Wire.write(INTEG_TIME);
      Wire.endTransmission();
      
      setHighGain(highGainEnabled);
    }
    String response = "{\"success\":" + String(success ? "true" : "false") + ",";
    response += "\"message\":\"" + String(success ? "Settings loaded and applied" : "Failed to load settings") + "\"}";
    r->send(200, "application/json", response);
  });

  server.on("/debug/ir2", HTTP_GET, [](AsyncWebServerRequest *request) {

    DEBUG_IR2_SERIAL = request->hasParam("serial");
  // Optional params
  uint8_t samples = 5;
  if (request->hasParam("avg")) samples = (uint8_t)request->getParam("avg")->value().toInt();
  if (samples < 1) samples = 1;
  if (samples > 50) samples = 50;

  uint32_t waitMs = integrationMsFromATIME(INTEG_TIME) + 5;

  // Collect multiple readings to see stability
  uint32_t sumX = 0, sumIR2 = 0;
  uint16_t minX = 65535, maxX = 0;
  uint16_t minIR2 = 65535, maxIR2 = 0;

  // Also grab Y/Z/IR1 once (normal path) so you can compare overall signal
  SensorAvg normal = readAveragedSensor(samples, 0, false);

  for (uint8_t i = 0; i < samples; i++) {
    uint16_t x  = readXRawFresh();
    uint16_t ir2 = readIR2RawFresh();

    sumX += x;
    sumIR2 += ir2;

    if (x < minX) minX = x;
    if (x > maxX) maxX = x;

    if (ir2 < minIR2) minIR2 = ir2;
    if (ir2 > maxIR2) maxIR2 = ir2;
  }

  uint16_t avgX = (uint16_t)(sumX / samples);
  uint16_t avgIR2 = (uint16_t)(sumIR2 / samples);

  String out;
  out.reserve(1200);

  out += "=== TCS3430 IR2 Debug ===\n";
  out += "INTEG_TIME (ATIME): " + String(INTEG_TIME) + "\n";
  out += "Computed integration wait (ms): " + String(waitMs) + "\n";
  out += "Samples: " + String(samples) + "\n\n";

  out += "--- Normal averaged read (AMUX=0) ---\n";
  out += "X:   " + String(normal.X) + "\n";
  out += "Y:   " + String(normal.Y) + "\n";
  out += "Z:   " + String(normal.Z) + "\n";
  out += "IR1: " + String(normal.IR1) + "\n\n";

  out += "--- Fresh CH3 reads ---\n";
  out += "X  avg:  " + String(avgX) + "   min: " + String(minX) + "   max: " + String(maxX) + "\n";
  out += "IR2 avg: " + String(avgIR2) + "   min: " + String(minIR2) + "   max: " + String(maxIR2) + "\n\n";

  // Helpful ratio info
  if (avgX > 0) {
    float ratio = (float)avgIR2 / (float)avgX;
    out += "IR2/X ratio: " + String(ratio, 4) + "\n";
  }

  out += "\nTry: /debug/ir2?avg=10\n";

  if (request->hasParam("serial")) {
    Serial.println();
    Serial.println(out);
  }

  DEBUG_IR2_SERIAL = false;
  request->send(200, "text/plain", out);
  
});
  
  server.on("/settings/reset", HTTP_GET, [](AsyncWebServerRequest* r){
    // Reset to AMS factory defaults
    kX = 0.96;
    kY = 0.96;
    kZ = 0.96;
    xOffsetAdjust = 0.0;
    yOffsetAdjust = 0.0;
    zOffsetAdjust = 0.0;
    GAIN = 1;
    INTEG_TIME = 0xF6;
    highGainEnabled = false;
    
    // Apply sensor settings
    setGain(GAIN);
    Wire.beginTransmission(TCS3430_ADDR);
    Wire.write(CMD_BIT | ATIME_REG);
    Wire.write(INTEG_TIME);
    Wire.endTransmission();
    
    setHighGain(highGainEnabled);
    
    bool success = saveSettings();
    String response = "{\"success\":" + String(success ? "true" : "false") + ",";
    response += "\"message\":\"" + String(success ? "Settings reset to defaults and saved" : "Settings reset but save failed") + "\"}";
    r->send(200, "application/json", response);
  });
  
  server.on("/settings/export", HTTP_GET, [](AsyncWebServerRequest* r){
    if (!SPIFFS.begin(true) || !SPIFFS.exists(SETTINGS_FILE)) {
      r->send(404, "text/plain", "Settings file not found");
      return;
    }
    
    File file = SPIFFS.open(SETTINGS_FILE, "r");
    if (!file) {
      r->send(500, "text/plain", "Failed to open settings file");
      return;
    }
    
    String json = file.readString();
    file.close();
    
    r->send(200, "application/json", json);
  });
  
  server.on("/setred", HTTP_GET, [](AsyncWebServerRequest* r){
    uint8_t samples;
    uint16_t delaySeconds;
    bool useIR2;
    parseAvgParams(r, samples, delaySeconds, useIR2);

    SensorAvg s = readAveragedSensor(samples, delaySeconds, useIR2);
    uint16_t Xr = (uint16_t)s.X;
    uint16_t Yr = (uint16_t)s.Y;
    uint16_t Zr = (uint16_t)s.Z;
    uint16_t IR1r = (uint16_t)s.IR1;
    uint16_t IR2r = (uint16_t)s.IR2;

    rawRedX = Xr;
    rawRedY = Yr;
    rawRedZ = Zr;
    rawRedIR1 = IR1r;
    rawRedIR2 = IR2r;
    
    saveSettings();
    
     String response = "Red reference set to: X=" + String(Xr) + ", Y=" + String(Yr) + ", Z=" + String(Zr) +
                     ", IR1=" + String(IR1r) + ", IR2=" + String(IR2r) + "\n";
    response += "Settings automatically saved to flash memory.\n";
    Serial.printf("NEW RED CALIBRATION SAVED: X=%u, Y=%u, Z=%u, IR1=%u, IR2=%u\n", Xr, Yr, Zr, IR1r, IR2r);
    r->send(200, "text/plain", response);
  });
  
  server.on("/setorange", HTTP_GET, [](AsyncWebServerRequest* r){
    uint8_t samples;
    uint16_t delaySeconds;
    bool useIR2;
    parseAvgParams(r, samples, delaySeconds, useIR2);

    SensorAvg s = readAveragedSensor(samples, delaySeconds, useIR2);
    uint16_t Xr = (uint16_t)s.X;
    uint16_t Yr = (uint16_t)s.Y;
    uint16_t Zr = (uint16_t)s.Z;
    uint16_t IR1r = (uint16_t)s.IR1;
    uint16_t IR2r = (uint16_t)s.IR2;

    rawOrangeX = Xr;
    rawOrangeY = Yr;
    rawOrangeZ = Zr;
    rawOrangeIR1 = IR1r;
    rawOrangeIR2 = IR2r;
    
    saveSettings();
    
    String response = "Orange reference set to: X=" + String(Xr) + ", Y=" + String(Yr) + ", Z=" + String(Zr) +
                     ", IR1=" + String(IR1r) + ", IR2=" + String(IR2r) + "\n";
    response += "Settings automatically saved to flash memory.\n";
    Serial.printf("NEW ORANGE CALIBRATION SAVED: X=%u, Y=%u, Z=%u, IR1=%u, IR2=%u\n", Xr, Yr, Zr, IR1r, IR2r);
    r->send(200, "text/plain", response);
  });
  
  server.on("/setyellow", HTTP_GET, [](AsyncWebServerRequest* r){
    uint8_t samples;
    uint16_t delaySeconds;
    bool useIR2;
    parseAvgParams(r, samples, delaySeconds, useIR2);

    SensorAvg s = readAveragedSensor(samples, delaySeconds, useIR2);
    uint16_t Xr = (uint16_t)s.X;
    uint16_t Yr = (uint16_t)s.Y;
    uint16_t Zr = (uint16_t)s.Z;
    uint16_t IR1r = (uint16_t)s.IR1;
    uint16_t IR2r = (uint16_t)s.IR2;
    rawYellowX = Xr;
    rawYellowY = Yr;
    rawYellowZ = Zr;
    rawYellowIR1 = IR1r;
    rawYellowIR2 = IR2r;

    saveSettings();
    
    String response = "Yellow tile reference set to: X=" + String(Xr) + ", Y=" + String(Yr) + ", Z=" + String(Zr) +
                     ", IR1=" + String(IR1r) + ", IR2=" + String(IR2r) + "\n";    
    response += "Settings automatically saved to flash memory.\n";
    Serial.printf("NEW YELLOW CALIBRATION SAVED: X=%u, Y=%u, Z=%u, IR1=%u, IR2=%u\n", Xr, Yr, Zr, IR1r, IR2r);
    r->send(200, "text/plain", response);
  });
  
  server.on("/setgrey", HTTP_GET, [](AsyncWebServerRequest* r){
    uint8_t samples;
    uint16_t delaySeconds;
    bool useIR2;
    parseAvgParams(r, samples, delaySeconds, useIR2);

    SensorAvg s = readAveragedSensor(samples, delaySeconds, useIR2);
    uint16_t Xr = (uint16_t)s.X;
    uint16_t Yr = (uint16_t)s.Y;
    uint16_t Zr = (uint16_t)s.Z;
    uint16_t IR1r = (uint16_t)s.IR1;
    uint16_t IR2r = (uint16_t)s.IR2;
    
    rawGreyX = Xr;
    rawGreyY = Yr;
    rawGreyZ = Zr;
    rawGreyIR1 = IR1r;
    rawGreyIR2 = IR2r;

    saveSettings();
    
   String response = "Grey reference set to: X=" + String(Xr) + ", Y=" + String(Yr) + ", Z=" + String(Zr) +
+                      ", IR1=" + String(IR1r) + ", IR2=" + String(IR2r) + "\n";
    response += "Settings automatically saved to flash memory.\n";
    Serial.printf("NEW GREY CALIBRATION SAVED: X=%u, Y=%u, Z=%u, IR1=%u, IR2=%u\n", Xr, Yr, Zr, IR1r, IR2r);
    
    r->send(200, "text/plain", response);
  });
  
  server.on("/setpink", HTTP_GET, [](AsyncWebServerRequest* r){
    uint8_t samples;
    uint16_t delaySeconds;
    bool useIR2;
    parseAvgParams(r, samples, delaySeconds, useIR2);

    SensorAvg s = readAveragedSensor(samples, delaySeconds, useIR2);
    uint16_t Xr = (uint16_t)s.X;
    uint16_t Yr = (uint16_t)s.Y;
    uint16_t Zr = (uint16_t)s.Z;
    uint16_t IR1r = (uint16_t)s.IR1;
    uint16_t IR2r = (uint16_t)s.IR2;
    
    rawPinkX = Xr;
    rawPinkY = Yr;
    rawPinkZ = Zr;
    rawPinkIR1 = IR1r;
    rawPinkIR2 = IR2r;

    saveSettings();
    
    String response = "Deep pink tile reference set to: X=" + String(Xr) + ", Y=" + String(Yr) + ", Z=" + String(Zr) +
                     ", IR1=" + String(IR1r) + ", IR2=" + String(IR2r) + "\n";
    response += "Settings automatically saved to flash memory.\n";
    Serial.printf("NEW PINK CALIBRATION SAVED: X=%u, Y=%u, Z=%u, IR1=%u, IR2=%u\n", Xr, Yr, Zr, IR1r, IR2r);    
    r->send(200, "text/plain", response);
  });

  // Additional endpoints for the specific tile names used by buttons
  server.on("/setdeeppink", HTTP_GET, [](AsyncWebServerRequest* r){
    uint8_t samples;
    uint16_t delaySeconds;
    bool useIR2;
    parseAvgParams(r, samples, delaySeconds, useIR2);

    SensorAvg s = readAveragedSensor(samples, delaySeconds, useIR2);
    uint16_t Xr = (uint16_t)s.X;
    uint16_t Yr = (uint16_t)s.Y;
    uint16_t Zr = (uint16_t)s.Z;
    uint16_t IR1r = (uint16_t)s.IR1;
    uint16_t IR2r = (uint16_t)s.IR2;

    rawPinkX = Xr;
    rawPinkY = Yr;
    rawPinkZ = Zr;
    rawPinkIR1 = IR1r;    
    rawPinkIR2 = IR2r;
    
    saveSettings();
    
    String response = "Deep pink tile reference set to: X=" + String(Xr) + ", Y=" + String(Yr) + ", Z=" + String(Zr) +
                     ", IR1=" + String(IR1r) + ", IR2=" + String(IR2r) + "\n";    
    response += "Settings automatically saved to flash memory.\n";
    Serial.printf("NEW DEEP PINK CALIBRATION SAVED: X=%u, Y=%u, Z=%u, IR1=%u, IR2=%u\n", Xr, Yr, Zr, IR1r, IR2r);
    r->send(200, "text/plain", response);
  });

  server.on("/setmidgrey", HTTP_GET, [](AsyncWebServerRequest* r){
    uint8_t samples;
    uint16_t delaySeconds;
    bool useIR2;
    parseAvgParams(r, samples, delaySeconds, useIR2);

    SensorAvg s = readAveragedSensor(samples, delaySeconds, useIR2);
    uint16_t Xr = (uint16_t)s.X;
    uint16_t Yr = (uint16_t)s.Y;
    uint16_t Zr = (uint16_t)s.Z;
    uint16_t IR1r = (uint16_t)s.IR1;
    uint16_t IR2r = (uint16_t)s.IR2;
    
    rawGreyX = Xr;
    rawGreyY = Yr;
    rawGreyZ = Zr;
    rawGreyIR1 = IR1r;
    rawGreyIR2 = IR2r;
    
    // Calculate what chromaticity coordinates this produces
    float X_rawXYZ, Y_rawXYZ, Z_rawXYZ;
    transformToXYZ(Xr, Yr, Zr, X_rawXYZ, Y_rawXYZ, Z_rawXYZ);
    float X_cal, Y_cal, Z_cal;
    computeCalXYZ_2Point(X_rawXYZ, Y_rawXYZ, Z_rawXYZ, X_cal, Y_cal, Z_cal);

    float sum = X_cal + Y_cal + Z_cal;
    float x_chromaticity = (sum > 1e-6f) ? X_cal / sum : 0.0f;
    float y_chromaticity = (sum > 1e-6f) ? Y_cal / sum : 0.0f;
    
    saveSettings();
    
    String response = "Mid grey tile reference set to: X=" + String(Xr) + ", Y=" + String(Yr) + ", Z=" + String(Zr) +
                     ", IR1=" + String(IR1r) + ", IR2=" + String(IR2r) + "\n";
    response += "Calibrated XYZ: X=" + String(X_cal, 2) + ", Y=" + String(Y_cal, 2) + ", Z=" + String(Z_cal, 2) + "\n";
    response += "Chromaticity: x=" + String(x_chromaticity, 4) + ", y=" + String(y_chromaticity, 4) + "\n";
    response += "Settings automatically saved to flash memory.\n";
    Serial.printf("NEW MID GREY CALIBRATION: Raw X=%u Y=%u Z=%u IR1=%u IR2=%u\n", Xr, Yr, Zr, IR1r, IR2r);    
    Serial.printf("Calibrated: X=%.2f Y=%.2f Z=%.2f\n", X_cal, Y_cal, Z_cal);
    Serial.printf("Chromaticity: x=%.4f y=%.4f\n", x_chromaticity, y_chromaticity);
    r->send(200, "text/plain", response);
  });

  server.on("/setdiffgrey", HTTP_GET, [](AsyncWebServerRequest* r){
    uint8_t samples;
    uint16_t delaySeconds;
    bool useIR2;
    parseAvgParams(r, samples, delaySeconds, useIR2);

    SensorAvg s = readAveragedSensor(samples, delaySeconds, useIR2);
    uint16_t Xr = (uint16_t)s.X;
    uint16_t Yr = (uint16_t)s.Y;
    uint16_t Zr = (uint16_t)s.Z;
    uint16_t IR1r = (uint16_t)s.IR1;
    uint16_t IR2r = (uint16_t)s.IR2;
    
    rawGreyX = Xr;  // Using same grey variable
    rawGreyY = Yr;
    rawGreyZ = Zr;
    
    
    rawGreyIR1 = IR1r;
    rawGreyIR2 = IR2r;
    
    saveSettings();
    
    String response = "Diff grey tile reference set to: X=" + String(Xr) + ", Y=" + String(Yr) + ", Z=" + String(Zr) +
                     ", IR1=" + String(IR1r) + ", IR2=" + String(IR2r) + "\n";
    response += "Settings automatically saved to flash memory.\n";
    Serial.printf("NEW DIFF GREY CALIBRATION SAVED: X=%u, Y=%u, Z=%u, IR1=%u, IR2=%u\n", Xr, Yr, Zr, IR1r, IR2r);    
    r->send(200, "text/plain", response);
  });
  
  Serial.println("Web server routes configured");
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(); 
    Serial.print("WiFi connected! IP: "); 
    Serial.println(WiFi.localIP());
    
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("WiFi Connected"));
    display.setCursor(0,10);
    display.print(F("IP: "));
    display.println(WiFi.localIP());
    display.display();
    delay(2000);
  } else {
    Serial.println("\nWiFi connection failed - continuing without it");
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("WiFi FAILED"));
    display.setCursor(0,10);
    display.println(F("Continuing..."));
    display.display();
    delay(2000);
  }
  
  // (Removed duplicate web route setup block - configured earlier)

  // OTA (ArduinoOTA) - enables wireless firmware upload when connected via STA or AP
  ArduinoOTA.setHostname("tcs3430");
  // Optional: set a password (uncomment to require it)
  // ArduinoOTA.setPassword("tcs3430");
  ArduinoOTA.onStart([](){ Serial.println("OTA: Start"); });
  ArduinoOTA.onEnd([](){ Serial.println("OTA: End"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t){ Serial.printf("OTA: %u%%\n", (p * 100) / t); });
  ArduinoOTA.onError([](ota_error_t e){ Serial.printf("OTA: Error[%u]\n", e); });
  ArduinoOTA.begin();
  Serial.println("OTA ready");

server.begin();
  Serial.println("Web server started");
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("System Ready!"));
  display.setCursor(0,10);
  display.println(F("Starting loop..."));
  display.display();
  delay(2000);
  
  Serial.println("=== Setup Complete - Entering Main Loop ===");
}

void loop() {
  ArduinoOTA.handle();  // OTA service
  static unsigned long last=0;
  static bool sensorError = false;
  static int loopCounter = 0;
  
  if (millis()-last>=1000) {
    last=millis();
    loopCounter++;
    
    Serial.printf("Loop iteration: %d\n", loopCounter);
    
    // Test if sensor is still responding
    Wire.beginTransmission(TCS3430_ADDR);
    uint8_t error = Wire.endTransmission();
    
    if (error != 0) {
      if (!sensorError) {
        Serial.println("TCS3430 communication lost!");
        sensorError = true;
      }
      
      // Update OLED with error message
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.println(F("SENSOR ERROR!"));
      display.setCursor(0,10);
      display.println(F("Check TCS3430"));
      display.setCursor(0,20);
      display.println(F("connections"));
      display.setCursor(0,40);
      display.print(F("I2C Error: "));
      display.println(error);
      display.display();
      return;
    }
    
    sensorError = false;
    
    // Check for saturation
    bool saturated = isSaturated();
    uint8_t status = getDeviceStatus();
    
    uint16_t Xr=read16(XDATA_L);
    uint16_t Yr=read16(YDATA_L);
    uint16_t Zr=read16(ZDATA_L);
    
    // Apply XYZ transform and AMS 2-point calibration (XYZ space)
    float X_rawXYZ, Y_rawXYZ, Z_rawXYZ;
    transformToXYZ(Xr, Yr, Zr, X_rawXYZ, Y_rawXYZ, Z_rawXYZ);

    // Raw chromaticity (from RawXYZ)
    float rawxyz_sum = X_rawXYZ + Y_rawXYZ + Z_rawXYZ;
    float rawxyz_x = (rawxyz_sum > 1e-6f) ? (X_rawXYZ / rawxyz_sum) : 0.0f;
    float rawxyz_y = (rawxyz_sum > 1e-6f) ? (Y_rawXYZ / rawxyz_sum) : 0.0f;

    // CCT computed ONLY from RawXYZ chromaticity (project requirement)
    float CCT = computeCCT_McCamy(rawxyz_x, rawxyz_y);

    // Calibrated XYZ using 2-point (Black/White) in XYZ space
    float X_calXYZ, Y_calXYZ, Z_calXYZ;
    computeCalXYZ_2Point(X_rawXYZ, Y_rawXYZ, Z_rawXYZ, X_calXYZ, Y_calXYZ, Z_calXYZ);

    // Calibrated chromaticity
    float cal_sum = X_calXYZ + Y_calXYZ + Z_calXYZ;
    float cal_x = (cal_sum > 1e-6f) ? (X_calXYZ / cal_sum) : 0.0f;
    float cal_y = (cal_sum > 1e-6f) ? (Y_calXYZ / cal_sum) : 0.0f;

    // Lux (relative) from calibrated Y
    float lux = Y_calXYZ * 0.2f;

    // Lab from CalXYZ, using spectro white tile XYZ as reference white
    float L, a_lab, b_lab;
    XYZtoLabRef(X_calXYZ, Y_calXYZ, Z_calXYZ,
                spectroTargets[0].X, spectroTargets[0].Y, spectroTargets[0].Z,
                L, a_lab, b_lab);

    // Update OLED display (8 lines, no overlap)
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Line 0: IP (left aligned, never overwrites)
    display.setCursor(0, 0);
    display.print(F("IP "));
    if (WiFi.getMode() == WIFI_MODE_STA && WiFi.status() == WL_CONNECTED) {
      display.print(WiFi.localIP());
    } else {
      display.print(WiFi.softAPIP());
    }

    // Saturation indicator (top right)
    if (saturated) {
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      display.setCursor(104, 0);
      display.print(F("SAT"));
      display.setTextColor(SSD1306_WHITE);
    }

    // Line 1: RawXY
    display.setCursor(0, 8);
    display.print(F("RawXY "));
    display.print(rawxyz_x, 3);
    display.print(F(" "));
    display.print(rawxyz_y, 3);

    // Line 2: CalXYZ X/Y (rounded)
    display.setCursor(0, 16);
    display.print(F("CalX "));
    display.print(X_calXYZ, 1);
    display.print(F(" Y "));
    display.print(Y_calXYZ, 1);

    // Line 3: CalZ + Lux
    display.setCursor(0, 24);
    display.print(F("CalZ "));
    display.print(Z_calXYZ, 1);
    display.print(F(" Lx "));
    display.print(lux, 0);

    // Line 4: Lab L/a
    display.setCursor(0, 32);
    display.print(F("L "));
    display.print(L, 1);
    display.print(F(" a "));
    display.print(a_lab, 1);

    // Line 5: Lab b + CCT
    display.setCursor(0, 40);
    display.print(F("b "));
    display.print(b_lab, 1);
    display.print(F(" CCT "));
    if (CCT > 0.0f) {
      display.print((int)CCT);
      display.print(F("K"));
    } else {
      display.print(F("--K"));
    }

    // Line 6: CalXY
    display.setCursor(0, 48);
    display.print(F("CalXY "));
    display.print(cal_x, 3);
    display.print(F(" "));
    display.print(cal_y, 3);

    // Line 7: Counts X/Y (quick sanity)
    display.setCursor(0, 56);
    display.print(F("Cnt "));
    display.print(Xr);
    display.print(F(" "));
    display.print(Yr);

// Debug output to match web interface (browser-toggle + throttled)
    static uint32_t lastLoopPrint = 0;
    if (loopSerialEnabled) {
      if (millis() - lastLoopPrint >= 1000) {  // once per second
        lastLoopPrint = millis();

        Serial.printf("Raw XYZ: %u, %u, %u\n", Xr, Yr, Zr);
        float sum = X_calXYZ + Y_calXYZ + Z_calXYZ;
        Serial.printf("CalXYZ: X=%.3f, Y=%.3f, Z=%.3f (sum=%.3f)", X_calXYZ, Y_calXYZ, Z_calXYZ, sum);
        Serial.printf("CalXY: x=%.4f, y=%.4f | CCT(rawXYZ): %.0fK, lux(calXYZ): %.1f", cal_x, cal_y, CCT, lux);

        if (CCT == 0.0f) {
          Serial.printf("CCT calculation failed - x=%.4f, y=%.4f (check if within valid range)\n", rawxyz_x, rawxyz_y);
        }
      }
    }
display.display();
  }
  }