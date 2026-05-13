
// rtos
#include <Arduino.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <SD.h>
#include <SPI.h>
#include <HTTPClient.h>
#include <Update.h>
#include <MD5Builder.h>

// RS485 PINS
#define RXD2              17   //RS485 serial communication (RX)
#define TXD2              16   // RS485 serial communication (TX)
#define FIRMWARE_VERSION  "1.0.2" 
#define OTA_MAX_RETRIES 5
#define OTA_CHUNK_TIMEOUT 10000  // 10 sec no data = timeout
#define BUTTON1_PIN 32  // push button
#define BUTTON2_PIN 33
#define BUTTON3_PIN 25
#define BUTTON4_PIN 26
#define BUTTON5_PIN 27
#define BUTTON6_PIN  14   
#define BUTTON7_PIN  12   
#define BUTTON8_PIN  13   
#define RS485  Serial2 
#define SLAVE_ID      0x01         // Modbus device address
#define FUNC_WRITE    0x05        // Modbus function code for writing a single coil

#define COIL_ON_H     0xFF       // Modbus coil ON value (0xFF00), split into high and low bytes
#define COIL_ON_L     0x00

#define COIL_OFF_H    0x00     // Modbus coil OFF value (0x0000), split into high and low bytes
#define COIL_OFF_L    0x00

// declaration
void otaUpdate(String firmwareUrl, String checksumUrl, String newVersion);
void saveBootLogToSD();

// default credentials CONFIG 
const char* SSID   = "Realme7";   
const char* PASS   = "12345678";
const char* BROKER = "52.77.237.146";
const char* TOKEN  = "s8o0joqrlv4hsxo46z5h";

// OBJECTS 
WiFiClient   net;     // WiFi client for MQTT , TCP socket
PubSubClient mqtt(net);    // MQTT is USING the WiFiClient (net) internally
Preferences  prefs;   // Non-volatile storage for Flash storage (EEPROM)
 
// System State 
String color = "NONE";   // Current relay color
String state = "OFF";    // Current relay state
bool relayOk = true;   // track relay response
int lastRoomId = 1;          bool saveStateFlag = false;
bool invalidCmd = false;     bool sdAvailable = false;   
bool otaFailed = false;  // flag to publish download_failed after reconnect
bool sdOk = false;      // sd card status
String otaDeviceId = ""; // store deviceId for publish after reconnect
volatile bool otaAbort = false;  // flag to abort OTA
bool otaDownloading = false; 
bool isBootRestore = false;
bool timeFromCompile = false; 
int bootRestoreCount = 0;  
int lastErrorCode = 1;
String timeApiUrl = "";
#define MAX_BOOT_LOGS 50  

struct BootSnapshot {
  String color;
  String state;
  String roomName;
  unsigned long rtcTime;
  bool captured = false;
};
BootSnapshot bootSnap;

// structure to store state of one room
struct RoomState {
  String color;
  String state;
};

#define MAX_ROOMS 8
RoomState roomState[MAX_ROOMS];

// Relay Command Queue
struct RelayCmd { 
  int roomId;   char color[10];    char state[8];
};
QueueHandle_t relayQueue;     // Queue for relay commands
SemaphoreHandle_t mqttMutex;  // Protect MQTT Mutex for MQTT client access
SemaphoreHandle_t prefsMutex;  // Mutex to protect flash

struct RoomRelay {
  uint8_t green;   uint8_t red;    uint8_t orange;   uint8_t buzzer;
};
//Rooms of relays for each room (1-8) with their respective channels for green, red, orange, and buzzer
RoomRelay relayMap[] = {
  {0, 1, 2, 3},  {4, 5, 6, 7},  {8, 9, 10, 11},  {12, 13, 14, 15},  {16, 17, 18, 19},    
  {20, 21, 22, 23},  {24, 25, 26, 27},  {28, 29, 30, 31}     
};

// Returns user-defined room name
String getRoomName(int id) {
  String name = "Room_" + String(id);   // default fallback
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
    prefs.begin("cfg", true);
    String key = "room" + String(id);
    name = prefs.getString(key.c_str(), name);
    prefs.end();
    xSemaphoreGive(prefsMutex);
  }
  return name;
}

// MQTT messages through a queue
struct MqttMsg {
  char topic[64];    
  char payload[256];   
};
QueueHandle_t publishQueue;     // Queue for MQTT messages to publish

uint16_t modbusCRC(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;     // CRC always starts with 0xFFFF (Modbus standard)
  for (int pos = 0; pos < len; pos++) {      // Process each byte one by one
    crc ^= (uint16_t)buf[pos];           // XOR byte into least sig. byte of crc
    for (int i = 0; i < 8; i++) {     // Each byte has 8 bits, so loop 8 times
      if (crc & 0x0001) {             // Check if least significant bit = 1
        crc >>= 1;           // If it is 1, shift right and XOR 0xA001 (the polynomial)
        crc ^= 0xA001;
      } else {
        crc >>= 1;        // If it is 0, just shift right
      }
    }
  }
  return crc;
}

void buildReadCmd(uint8_t channel, uint8_t *cmd) {
  cmd[0] = SLAVE_ID;
  cmd[1] = 0x01;   // READ COIL
  cmd[2] = 0x00;
  cmd[3] = channel;
  cmd[4] = 0x00;
  cmd[5] = 0x01;
  uint16_t crc = modbusCRC(cmd, 6);
  cmd[6] = crc & 0xFF;
  cmd[7] = (crc >> 8) & 0xFF;
}

void buildRelayCmd(uint8_t channel, bool state, uint8_t *cmd) {
  cmd[0] = SLAVE_ID;  cmd[1] = FUNC_WRITE;  cmd[2] = 0x00;  cmd[3] = channel;
  if (state) {
    cmd[4] = COIL_ON_H;  cmd[5] = COIL_ON_L;
  } else {
    cmd[4] = COIL_OFF_H; cmd[5] = COIL_OFF_L;  }
  uint16_t crc = modbusCRC(cmd, 6);      // Calculates error-check for first 6 bytes
  cmd[6] = crc & 0xFF;  cmd[7] = (crc >> 8) & 0xFF;
}

// RS485 SEND 
int sendCommand(uint8_t* cmd, bool printLog) {
  if (printLog) {
    Serial.print("TX: ");
    for (int i = 0; i < 8; i++) Serial.printf("%02X ", cmd[i]);
    Serial.println();
  }
  while (RS485.available()) RS485.read();
  RS485.write(cmd, 8);
  RS485.flush();
  uint8_t resp[8];
  int n = 0;
  unsigned long t = millis();

  while (millis() - t < 50) {
    if (RS485.available()) {
      resp[n++] = RS485.read();
      if (n >= 8) break;
    }
    vTaskDelay(1);
  }
  if (n == 0) return -1;          // No response
  if (n < 8) return -2;           // Incomplete
  if (resp[0] != SLAVE_ID) return -3;
  if (resp[1] & 0x80) return -4;
  if (resp[1] != FUNC_WRITE) return -5;

  uint16_t crcCalc = modbusCRC(resp, 6);
  uint16_t crcRecv = resp[6] | (resp[7] << 8);
  if (crcCalc != crcRecv) return -7;
  return 1; // success
}

int sendCommandDebug(uint8_t* cmd, uint8_t* resp, int &len) {
  while (RS485.available()) RS485.read();

  RS485.write(cmd, 8);
  RS485.flush();

  len = 0;
  unsigned long t = millis();

  while (millis() - t < 50) {
    if (RS485.available()) {
      resp[len++] = RS485.read();
      if (len >= 8) break;
    }
    vTaskDelay(1);
  }
  return len;
}

int controlRelay(uint8_t channel, bool state, bool printLog = false) {
  uint8_t cmd[8];  buildRelayCmd(channel, state, cmd);  return sendCommand(cmd, printLog);    // Sends via RS485, Waits for response , Returns result
}

String getErrorCodeString(int err) {
  switch (err) {
    case -1: return "-T"; // Timeout
    case -2: return "-I"; // Incomplete
    case -3: return "-S"; // Wrong slave ID
    case -4: return "-M"; // Modbus exception
    case -5: return "-F"; // Invalid function
    case -6: return "-L"; // Invalid length
    case -7: return "-C"; // CRC mismatch
    default: return "OK";
  }
}
  
void applyRelayState(int roomId, const String& c, const String& s) {
  int errCode = 1;    int res;           // 1 = success
  RoomRelay r = relayMap[roomId - 1];
  String cUpper = c;  cUpper.toUpperCase();
  Serial.printf("[RELAY] Room=%d color=%s state=%s\n",
                roomId, c.c_str(), s.c_str());
  // OFF condition
  if (s == "OFF") {
    res = controlRelay(r.green, false, false);     if (res < 0 && errCode > 0) errCode = res;
    res = controlRelay(r.red, false, false);       if (res < 0 && errCode > 0) errCode = res;
    res = controlRelay(r.orange, false, false);    if (res < 0 && errCode > 0) errCode = res;
    res = controlRelay(r.buzzer, false, false);    if (res < 0 && errCode > 0) errCode = res;

    relayOk = (errCode > 0);
    lastErrorCode = errCode;
    return;
  }

  // Turn OFF all first
  res = controlRelay(r.green, false, false);      if (res < 0 && errCode > 0) errCode = res;
  res = controlRelay(r.red, false, false);        if (res < 0 && errCode > 0) errCode = res;
  res = controlRelay(r.orange, false, false);     if (res < 0 && errCode > 0) errCode = res;
  res = controlRelay(r.buzzer, false, false);     if (res < 0 && errCode > 0) errCode = res;

  // Apply color
  if (cUpper == "GREEN") {
    res = controlRelay(r.green, true, true);      if (res < 0 && errCode > 0) errCode = res;
  } 
  else if (cUpper == "ORANGE") {
    res = controlRelay(r.orange, true, true);     if (res < 0 && errCode > 0) errCode = res;
  } 
  else if (cUpper == "RED") {
    res = controlRelay(r.red, true, true);        if (res < 0 && errCode > 0) errCode = res;
    res = controlRelay(r.buzzer, true, true);     if (res < 0 && errCode > 0) errCode = res;
  }
  relayOk = (errCode > 0);
  lastErrorCode = errCode;
}

// HELPERS
unsigned long long getTimestampMs() {
  return (unsigned long long)time(nullptr) * 1000ULL;         //UNIX timestamp in milliseconds
}

String getTimeString() {
  time_t now = time(nullptr);
  if (now < 1700000000) return "NOT_SYNCED";
  char buf[25];  struct tm t;   localtime_r(&now, &t);
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
  return String(buf);
}

String getDeviceId() {
  uint64_t mac = ESP.getEfuseMac();
  // Extract last 3 bytes
  uint8_t b4 = (mac >> 16) & 0xFF;       // move byte 4 to LSB
  uint8_t b5 = (mac >> 8)  & 0xFF;      // move byte 5 to LSB
  uint8_t b6 = mac & 0xFF;              // byte 6 already
  // Convert each to decimal
  int d1 = b4;   int d2 = b5;   int d3 = b6;   
  // Merge as string
  char buf[20];
  sprintf(buf, "%d%d%d", d1, d2, d3);
  Serial.printf("[DEVICE] %02X %02X %02X → %d %d %d → %s\n",
                b4, b5, b6, d1, d2, d3, buf);
  return String(buf);
}

String getJsonValue(const String& json, const String& key) {
  String search = "\"" + key + "\":";           // search pattern for key
  int i = json.indexOf(search);                 // find key in JSON
  if (i < 0) return "";
  int s = i + search.length();          // Move to value start
  while (s < json.length() && json[s] == ' ') s++;      // skip spaces  
  if (s >= json.length()) return "";

  if (json[s] == '"') {               // Check value type
    s++;
    int e = json.indexOf('"', s);
    if (e < 0) return "";
    return json.substring(s, e);
  }
  int e = s;
  while (e < json.length() && json[e] != ',' && json[e] != '}') e++;
  return json.substring(s, e);
}

void checkButton(int pin, int roomId) {
  static unsigned long lastPress[MAX_ROOMS] = {0};
  bool currentState = digitalRead(pin);
  if (currentState == LOW) {
    if (millis() - lastPress[roomId-1] < 200) return;
    lastPress[roomId-1] = millis();
    Serial.printf("[BUTTON] Room %d pressed\n", roomId);
     String c = roomState[roomId-1].color;
c.toUpperCase();
if (c == "RED" &&
        roomState[roomId-1].state == "ON") {
      Serial.printf("[ALARM] Muting Room %d\n", roomId);

      RelayCmd cmd;
      cmd.roomId = roomId;
      strncpy(cmd.color, "RED", sizeof(cmd.color)-1);
      cmd.color[sizeof(cmd.color)-1] = '\0';
      strncpy(cmd.state, "MUTE", sizeof(cmd.state)-1);
      cmd.state[sizeof(cmd.state)-1] = '\0';
      xQueueSend(relayQueue, &cmd, 0);
    } else {
      Serial.printf("[BUTTON] No alarm in Room %d\n", roomId);
    }
  }
}

// MQTT PUBLISH 
void sendTelemetry(int roomId = 0, String rawColor = "", String rawState = "", bool isBoot = false) {
  char buf[384];   String colorOut;    bool stateBool;

if (isBoot) {
  colorOut = color;     stateBool = (state == "ON");                // use stored state
} else {
  colorOut = rawColor;  stateBool = (rawState == "true" || rawState == "ON");             // use incoming data
}
  // BOOT 
  if (isBoot) {
    String ssid = "";    String pass = "";

if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
  prefs.begin("cfg", true);
  ssid = prefs.getString("ssid", "");  pass = prefs.getString("pass", "");   prefs.end();
  xSemaphoreGive(prefsMutex);
}
    String deviceId = getDeviceId();    String timeStr;   unsigned long long bootTs;

    if (bootSnap.captured && bootSnap.rtcTime > 0) {
      bootTs = (unsigned long long)time(nullptr) * 1000ULL;  // ThingsBoard ts = NOW
      time_t bt = (time_t)bootSnap.rtcTime;                  // inside json = boot time
      char tbuf[25];   struct tm t;    localtime_r(&bt, &t);
      strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &t);
      timeStr = String(tbuf);
    } else {
      bootTs  = (unsigned long long)millis();   timeStr = "NOT_SYNCED";
    }
    String roomName  = bootSnap.captured ? bootSnap.roomName : getRoomName(lastRoomId);
    String colorSnap = bootSnap.captured ? bootSnap.color    : colorOut;
    bool   stateSnap = bootSnap.captured ? (bootSnap.state == "ON") : stateBool;

    bool timeError = (timeStr == "NOT_SYNCED" || timeFromCompile);
    snprintf(buf, sizeof(buf),
      "{\"ts\":%llu,\"values\":{"
      "\"msg\":{"
      "\"msg_type\":\"boot_msg\","
      "\"timestamp\":\"%s\","
      "\"device_id\":\"%s\","
      "\"firmware_version\":\"" FIRMWARE_VERSION "\","
      "\"model\":\"bannerpnp_flasher\","
      "\"status\":\"system powered on\","
      "\"wifi\":{"
      "\"ssid\":\"%s\","
      "\"pass\":\"%s\""
      "},"
      "\"sd_health\":%s,"
      "%s"
      "\"boot_state\":{"
      "\"cr_name\":\"%s\",\"color\":\"%s\",\"state\":%s"
      "}}}}",
      bootTs,
      timeStr.c_str(),
      deviceId.c_str(),
      ssid.c_str(),        
      pass.c_str(),
      sdOk ? "success" : "failure",
      timeError ? "\"time_error\":\"api failed fallback to compile time\"," : "",
      roomName.c_str(),
      colorSnap.c_str(),
      stateSnap ? "true" : "false"
    );
  }

  // NORMAL 
  else {
    String roomName = getRoomName(roomId);
    if (invalidCmd) {
    snprintf(buf, sizeof(buf),
    "{\"ts\":%llu,\"values\":{\"msg\":{"
    "\"msg_type\":\"mqtt_sub\",\"cr_name\":\"%s\",\"color\":\"%s\",\"state\":%s,"
    "\"error\":\"invalid command\"}}}",
    getTimestampMs(),  roomName.c_str(),  colorOut.c_str(),
    stateBool ? "true" : "false"
  );
}
else if (!relayOk) {
  snprintf(buf, sizeof(buf),
    "{\"ts\":%llu,\"values\":{\"msg\":{"
    "\"msg_type\":\"mqtt_sub\",\"cr_name\":\"%s\",\"color\":\"%s\",\"state\":%s,"
    "\"error\":\"relay not responding\"}}}",
    getTimestampMs(),  roomName.c_str(),   colorOut.c_str(),
    stateBool ? "true" : "false"
  );
}
else {
  snprintf(buf, sizeof(buf),
    "{\"ts\":%llu,\"values\":{\"msg\":{"
    "\"msg_type\":\"mqtt_sub\",\"cr_name\":\"%s\",\"color\":\"%s\",\"state\":%s"
    "}"
    "}}",
    getTimestampMs(),
    roomName.c_str(),    colorOut.c_str(),   stateBool ? "true" : "false"
  );
 }
}

  // SEND
  Serial.println("[TELE] " + String(buf));
  if (!mqtt.publish("v1/devices/me/telemetry", buf)) {
    Serial.println("[ERROR] MQTT publish failed");
  }
}
    
void sendAttributes() {
  char buf[128];    bool stateBool = (state == "ON");
  snprintf(buf, sizeof(buf),
           "{\"color\":\"%s\",\"state\":%s}",
           color.c_str(),   stateBool ? "true" : "false");
  //Serial.println("[ATTR] " + String(buf));
  mqtt.publish("v1/devices/me/attributes", buf);
}

bool checkSD() {
  Serial.println("[SD] Checking...");
  if (!SD.begin()) {
    Serial.println("[SD] FAIL");      return false;
  }
  // WRITE TEST
  File f = SD.open("/sd_check.txt", FILE_WRITE);
  if (!f) {
    Serial.println("[SD] WRITE FAIL");   return false;
  }
  f.println("ok");  f.close();
  // READ TEST
  f = SD.open("/sd_check.txt");
  if (!f) {
    Serial.println("[SD] READ FAIL");    return false;
  }
  String data = f.readStringUntil('\n');
  f.close();   data.trim();
  if (data != "ok") {
    Serial.println("[SD] DATA FAIL");    return false;
  }
  Serial.println("[SD] SD Card health OK");
  return true;
}


// MQTT CALLBACK 
void handleMqttMessage(char* topic, byte* payload, unsigned int len) {
  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  Serial.println("\n[MQTT RX] " + msg);
  if (msg == "{}") {
    //Serial.println("[MQTT] Empty message ignored");
    return;
  }

  // Step 1: unwrap shared JSON (if exists)
  int si = msg.indexOf("\"shared\":");
  if (si >= 0) {
    int o = msg.indexOf('{', si);
    int c = msg.lastIndexOf('}');
    if (o >= 0 && c > o) {
      msg = msg.substring(o, c + 1);
    }
  }

  // Step 2: Parse JSON (ALWAYS runs)
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, msg);
  if (error) {
    Serial.println("[JSON] Parse failed");    return;
  }

  // Step 3: Handle nested "msg" or direct JSON
  JsonObject data;
  // (GLOBAL inside function)
   String rawColor = "";     String newColor = "";     String rawState = "";    String newState = "";
   bool isStateString = false;

  if (doc["msg"].is<JsonObject>()) {
    data = doc["msg"].as<JsonObject>();
  } else {
    data = doc.as<JsonObject>();
  }
  bool restartCmd = false;

if (data["device_restart"].is<bool>()) {
  restartCmd = data["device_restart"];
}

if (restartCmd) {
  Serial.println("[CMD] Device restart requested");
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"ts\":%llu,\"values\":{\"msg\":{"
    "\"msg_type\":\"restart\",\"status\":\"device restarting\","
    "\"timestamp\":\"%s\""
    "}"
    "}}",
    getTimestampMs(),  getTimeString().c_str()
  );
  Serial.println("[TELE] " + String(buf));

  if (mqtt.connected()) {
    mqtt.publish("v1/devices/me/telemetry", buf);
    delay(300);
    mqtt.publish("v1/devices/me/attributes",
                 "{\"device_restart\":false}");
  }
  Serial.println("[SYSTEM] Restarting now...");   delay(500);
  ESP.restart();
}

// ROOM STATUS REQUEST
if (data["room_status"].is<int>()) {
  int roomId = data["room_status"];
  if (roomId < 1 || roomId > MAX_ROOMS) {
    mqtt.publish("v1/devices/me/telemetry",
      "{\"msg\":{\"msg_type\":\"room_status\",\"error\":\"invalid room id\"}}");
    return;
  }
  String roomName = getRoomName(roomId);    RoomRelay r = relayMap[roomId - 1];

  // Read actual relay state from roomState[]
 String currentState = roomState[roomId - 1].state;
String currentColor = roomState[roomId - 1].color;
currentColor.toUpperCase();  
if (currentState != "ON") {
  currentColor = "NONE";
}

  // Derive each relay state from color+state
  bool greenOn  = (currentState == "ON" && currentColor == "GREEN");
  bool orangeOn = (currentState == "ON" && currentColor == "ORANGE");
  bool redOn    = (currentState == "ON" && currentColor == "RED");
  bool buzzerOn = (currentState == "ON" && currentColor == "RED");  // buzzer only with RED
  char buf[384];
  snprintf(buf, sizeof(buf),
    "{\"ts\":%llu,\"values\":{\"msg\":{"
    "\"msg_type\":\"room_status\",\"cr_name\":\"%s\","
    "\"room_id\":%d,\"color\":\"%s\",\"state\":\"%s\","
    "\"relays\":{\"green\":%s,\"orange\":%s,\"red\":%s,\"buzzer\":%s"
    "}"
    "}"
    "}}",
    getTimestampMs(),  roomName.c_str(),  roomId,
    currentColor.c_str(),  currentState.c_str(),
    greenOn  ? "true" : "false",      orangeOn ? "true" : "false", 
    redOn    ? "true" : "false",      buzzerOn ? "true" : "false"
  );
  Serial.println("[STATUS] " + String(buf));
  mqtt.publish("v1/devices/me/telemetry", buf);
  return;
}

if (data["sd_check"].is<bool>() && data["sd_check"] == true) {
  Serial.println("[CMD] SD check");
  sdOk = checkSD();
  char buf[128];
  snprintf(buf, sizeof(buf),
    "{\"ts\":%llu,\"values\":{\"msg\":{\"msg_type\":\"sd_check\",\"sd_health\":%s}}}",
    getTimestampMs(),     sdOk ? "success" : "failure"
  );
  mqtt.publish("v1/devices/me/telemetry", buf);
  return;   
}

// TIME API URL UPDATE
if (data["time_api"].is<String>()) {
  timeApiUrl = data["time_api"].as<String>();
  Serial.println("[TIME] API URL: " + timeApiUrl);

  if (sdAvailable) {
    File rFile = SD.open("/device_config.json");
    JsonDocument devDoc;
    if (rFile) {
      deserializeJson(devDoc, rFile);     rFile.close();
    }
    devDoc["time"]["api_url"] = timeApiUrl;
    SD.remove("/device_config.json");
    File wFile = SD.open("/device_config.json", FILE_WRITE);
    if (wFile) {
      serializeJsonPretty(devDoc, wFile);
      wFile.close();     Serial.println("[SD] Time API URL saved");
    }
  }
  mqtt.publish("v1/devices/me/telemetry",
    "{\"msg\":{\"msg_type\":\"config\",\"time_api\":\"saved\"}}");
  return;
}

// OTA ABORT
if (data["ota_abort"].is<bool>() && data["ota_abort"] == true) {
  Serial.println("[OTA] Abort requested");
  otaAbort = true;     return;
}

// DEVICE CONFIG REQUEST
if (data["device_config"].is<bool>() && data["device_config"] == true) {
  Serial.println("[CMD] Config requested");
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
    prefs.begin("cfg", true);
    String ssid     = prefs.getString("ssid", "");           String pass     = prefs.getString("pass", "");
    String broker   = prefs.getString("mqttServer", "");     String token    = prefs.getString("mqttToken", "");
    String clientId = prefs.getString("mqttClientId", "");   int ncr         = prefs.getInt("ncr", 4);

    String json = "{\"msg\":{";
    json += "\"msg_type\":\"config_req\",";      json += "\"wifi\":{";  
     json += "\"ssid\":\"" + ssid + "\",";       json += "\"pass\":\"" + pass + "\"},";
    json += "\"mqtt\":{";      json += "\"server\":\"" + broker + "\",";      json += "\"token\":\"" + token + "\",";
    json += "\"clientId\":\"" + clientId + "\"},";
    json += "\"ncr\":" + String(ncr) + ",";      json += "\"cr_names\":{";

    bool first = true;
    for (int i = 1; i <= ncr; i++) {
      String key  = "room" + String(i);
      String name = prefs.getString(key.c_str(), "Room_" + String(i));
      if (!first) json += ",";
      first = false;
      json += "\"" + String(i) + "\":\"" + name + "\"";
    }
    json += "}}}";  // close cr_names, msg, root
    prefs.end();
    xSemaphoreGive(prefsMutex);
    Serial.println("[CONFIG] " + json);
    mqtt.publish("v1/devices/me/telemetry", json.c_str());
  }
  return;
}

// OTA COMMAND
if (data["ota"].is<JsonObject>()) {
  String newVersion  = data["ota"]["version"]      | "";
  String firmwareUrl = data["ota"]["main_url"] | "";
  String checksumUrl = data["ota"]["checksum_url"] | "";

  // Check fields
  if (newVersion == "" || firmwareUrl == "" || checksumUrl == "") {
    mqtt.publish("v1/devices/me/telemetry",
      "{\"msg\":{\"msg_type\":\"ota\",\"status\":\"error\",\"error\":\"missing fields\"}}");
    return;
  }

  // Check version
  int curMajor, curMinor, curPatch;       int newMajor, newMinor, newPatch;
  sscanf(FIRMWARE_VERSION,       "%d.%d.%d", &curMajor, &curMinor, &curPatch);
  sscanf(newVersion.c_str(),     "%d.%d.%d", &newMajor, &newMinor, &newPatch);

  int curVal = curMajor * 10000 + curMinor * 100 + curPatch;
  int newVal = newMajor * 10000 + newMinor * 100 + newPatch;

  if (newVal <= curVal) {
    Serial.printf("[OTA] Ignored — %s <= %s\n", newVersion.c_str(), FIRMWARE_VERSION);
    mqtt.publish("v1/devices/me/telemetry",
      "{\"msg\":{\"msg_type\":\"ota\",\"status\":\"ignored\",\"reason\":\"version not newer\"}}");
    return;
  }
  Serial.printf("[OTA] version ok %s → %s\n", FIRMWARE_VERSION, newVersion.c_str());

  // Save URLs into existing config.json on SD
  if (!sdAvailable) {
    mqtt.publish("v1/devices/me/telemetry",
      "{\"msg\":{\"msg_type\":\"ota\",\"status\":\"error\",\"error\":\"SD not available\"}}");
    return;
  }

  // Save OTA urls to device_config.json
    File rFile = SD.open("/device_config.json");
    JsonDocument devDoc;
    if (rFile) {
      deserializeJson(devDoc, rFile);    rFile.close();
    }
    devDoc["ota"]["version"]      = newVersion;
    devDoc["ota"]["main_url"]     = firmwareUrl;
    devDoc["ota"]["checksum_url"] = checksumUrl;

    SD.remove("/device_config.json");
    File wFile = SD.open("/device_config.json", FILE_WRITE);
    if (wFile) {
      serializeJsonPretty(devDoc, wFile);
      wFile.close();      Serial.println("[OTA] URLs saved to device_config.json");
    }
  mqtt.publish("v1/devices/me/telemetry",
    "{\"msg\":{\"msg_type\":\"ota\",\"status\":\"urls_saved\"}}");

  // call OTA update
  otaUpdate(firmwareUrl, checksumUrl, newVersion);
  return;
}

  String newSSID   = data["ssid"] | "";           String newPASS   = data["password"] | "";
  String newBroker = data["mqttServer"] | "";     String newToken  = data["mqttAcesstoken"] | "";
  String newClientId = data["clientId"] | "";

  bool changed = false; 
  // Step 6: Config logs
  bool configChanged = false;
if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
  prefs.begin("cfg", false);

  // WiFi update
if (newSSID.length() > 0 && newPASS.length() > 0) {
  Serial.println("[CFG] Updating WiFi...");
  prefs.putString("ssid", newSSID);    prefs.putString("pass", newPASS);
  prefs.putBool("wifi_updated", true);

  // VERIFY WRITE (IMPORTANT)
  String verifySSID = prefs.getString("ssid", "");
  Serial.println("[CFG] Saved SSID: " + verifySSID);
  configChanged = true;
}

  // MQTT update
  if (newBroker.length() > 0 || newToken.length() > 0 || newClientId.length() > 0) {
       Serial.println("[CFG] Updating MQTT...");
  if (newBroker.length() > 0)
       prefs.putString("mqttServer", newBroker);
  if (newToken.length() > 0)
       prefs.putString("mqttToken", newToken);
  if (newClientId.length() > 0)
        prefs.putString("mqttClientId", newClientId);       prefs.putBool("mqtt_updated", true);
       configChanged = true;
     }
       prefs.end();
    xSemaphoreGive(prefsMutex);
    }
   delay(500); 

  // HANDLE NCR UPDATE
if (data["ncr"].is<int>()) {
  int newNcr = data["ncr"];
  if (newNcr > 0) {
    if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
      prefs.begin("cfg", false);    prefs.putInt("ncr", newNcr);   prefs.end();
      xSemaphoreGive(prefsMutex);
    }
    Serial.printf("[CFG] NCR updated → %d\n", newNcr);  configChanged = true;
  }
}

// ROOM NAME UPDATE
if (data["cr_names"].is<JsonObject>()) {
  JsonObject names = data["cr_names"];
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
    prefs.begin("cfg", false);
    for (JsonPair kv : names) {
      String roomIdStr = kv.key().c_str();
      int roomId = roomIdStr.toInt();

      if (roomId < 1) continue;
      String newName = kv.value().as<String>();
      String key = "room" + String(roomId);
      prefs.putString(key.c_str(), newName);
      // Serial.printf("[CFG] Room %d → %s\n", roomId, newName.c_str());
    }
    prefs.end();
    xSemaphoreGive(prefsMutex);
  }
  Serial.println("[CFG] Room names updated");
  configChanged = true;
}

// SAVE device_config.json
if (sdAvailable && configChanged) {
  JsonDocument devDoc;
  prefs.begin("cfg", true);
  devDoc["wifi"]["ssid"]    = prefs.getString("ssid", "");              devDoc["wifi"]["pass"]   = prefs.getString("pass", "");
  devDoc["mqtt"]["server"]  = prefs.getString("mqttServer", "");        devDoc["mqtt"]["token"]  = prefs.getString("mqttToken", "");
  devDoc["mqtt"]["clientId"] = prefs.getString("mqttClientId", "");     prefs.end();

  // preserve existing OTA urls if present
  File rDev = SD.open("/device_config.json");
  if (rDev) {
    JsonDocument existing;
    if (!deserializeJson(existing, rDev)) {
      if (existing["ota"].is<JsonObject>()) {
        devDoc["ota"]["version"]      = existing["ota"]["version"] | "";
        devDoc["ota"]["main_url"]     = existing["ota"]["main_url"] | "";
        devDoc["ota"]["checksum_url"] = existing["ota"]["checksum_url"] | "";
      }

      if (existing["time"].is<JsonObject>()) {
      devDoc["time"]["api_url"] = existing["time"]["api_url"] | "";
    }
    }
    rDev.close();
  }
  SD.remove("/device_config.json");
  File wDev = SD.open("/device_config.json", FILE_WRITE);
  if (wDev) {
    serializeJsonPretty(devDoc, wDev);
    wDev.close();            Serial.println("[SD] device_config.json saved");
  }
}

// SAVE flasher_config.json
if (sdAvailable && (data["ncr"].is<int>() || data["cr_names"].is<JsonObject>())) {
  JsonDocument flashDoc;
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
    prefs.begin("cfg", true);
    int ncr = prefs.getInt("ncr", 4);
    flashDoc["ncr"] = ncr;
    JsonObject names = flashDoc["cr_names"].to<JsonObject>();
    for (int i = 1; i <= ncr; i++) {
      String key  = "room" + String(i);
      String name = prefs.getString(key.c_str(), "Room_" + String(i));
      names[String(i)] = name;
    }
    prefs.end();
    xSemaphoreGive(prefsMutex);
  }
  SD.remove("/flasher_config.json");
  File wFlash = SD.open("/flasher_config.json", FILE_WRITE);
  if (wFlash) {
    serializeJsonPretty(flashDoc, wFlash);
    wFlash.close();       Serial.println("[SD] flasher_config.json saved");
  }
}


bool ncrOrNamesUpdated = false;
if (data["ncr"].is<int>()) ncrOrNamesUpdated = true;
if (data["cr_names"].is<JsonObject>()) ncrOrNamesUpdated = true;
if (ncrOrNamesUpdated) {
  JsonDocument res;
  res["msg"]["msg_type"] = "Config";
  res["msg"]["timestamp"] = getTimeString();
  if (data["ncr"].is<int>()) {
    res["msg"]["ncr"] = data["ncr"];
  }
  if (data["cr_names"].is<JsonObject>()) {
    res["msg"]["cr_names"] = data["cr_names"];
  }
  char buf[512];
  serializeJson(res, buf);
  Serial.println("[MQTT TX] " + String(buf));
  mqtt.publish("v1/devices/me/telemetry", buf);
  mqtt.loop();
  delay(300);   
}

// RESTART 
if (configChanged) {
  Serial.println("[CFG] Restarting device...");
  delay(1000);
  ESP.restart();
}

// COMMAND LOOP 
JsonObject root = data;
// GLOBAL OFF (no room key)
if (data["state"].is<bool>() && data["state"] == false) {
  Serial.println("[CMD] GLOBAL OFF received");
  RelayCmd cmd;        cmd.roomId = 0;  // global command
  strncpy(cmd.color, "NONE", sizeof(cmd.color)-1);
  cmd.color[sizeof(cmd.color)-1] = '\0';

  strncpy(cmd.state, "OFF", sizeof(cmd.state)-1);
  cmd.state[sizeof(cmd.state)-1] = '\0';
  xQueueSend(relayQueue, &cmd, 0);
  return; 
}

for (JsonPair kv : root) {
  String roomKey = kv.key().c_str();
  // SKIP CONFIG
  if (roomKey == "ncr" || roomKey == "cr_names") continue;
  int roomId = roomKey.toInt();
  JsonArray arr = kv.value().as<JsonArray>();  
  for (JsonObject obj : arr) {
     if (obj["debug_mode"].is<bool>() && obj["debug_mode"] == true) {

  RoomRelay r = relayMap[roomId - 1];

  uint8_t cmd[8];
  uint8_t resp[16];
  int len = 0;

  String g="", re="", o="", b="";

  // GREEN
  buildReadCmd(r.green, cmd);
  sendCommandDebug(cmd, resp, len);
  for (int i=0;i<len;i++){ char t[4]; sprintf(t,"%02X ",resp[i]); g+=t; }

  // RED
  buildReadCmd(r.red, cmd);
  sendCommandDebug(cmd, resp, len);
  for (int i=0;i<len;i++){ char t[4]; sprintf(t,"%02X ",resp[i]); re+=t; }

  // ORANGE
  buildReadCmd(r.orange, cmd);
  sendCommandDebug(cmd, resp, len);
  for (int i=0;i<len;i++){ char t[4]; sprintf(t,"%02X ",resp[i]); o+=t; }

  // BUZZER
  buildReadCmd(r.buzzer, cmd);
  sendCommandDebug(cmd, resp, len);
  for (int i=0;i<len;i++){ char t[4]; sprintf(t,"%02X ",resp[i]); b+=t; }

  char buf[512];
  snprintf(buf, sizeof(buf),
    "{\"ts\":%llu,\"values\":{\"msg\":{"
    "\"msg_type\":\"debug\",\"room_id\":%d,"
    "\"req1\":{\"green\":\"%s\"},"
    "\"req2\":{\"red\":\"%s\"},"
    "\"req3\":{\"orange\":\"%s\"},"
    "\"req4\":{\"buzzer\":\"%s\"}"
    "}}}",
    getTimestampMs(),
    roomId,
    g.c_str(),
    re.c_str(),
    o.c_str(),
    b.c_str()
  );

  mqtt.publish("v1/devices/me/telemetry", buf);

  continue;
}
    String rawColor = obj["color"] | "";      String newColor = rawColor;
    newColor.toUpperCase();     String rawState = "";   String newState = "";

    if (obj["state"].is<bool>()) {
      bool s = obj["state"];
      rawState = s ? "true" : "false";       newState = s ? "ON" : "OFF";
    }
    bool validColor = (newColor == "GREEN" || newColor == "RED" || newColor == "ORANGE");
    bool validState = (newState == "ON" || newState == "OFF");

if (!validColor || !validState) {
  Serial.println("[ERROR] Invalid command received");
  invalidCmd = true;  
  // send EXACT same input back
  sendTelemetry(roomId, rawColor, rawState, false);
  continue; // do not send to relay
}

invalidCmd = false;
    if (roomId >= 1 && roomId <= MAX_ROOMS) {
  roomState[roomId - 1].color = newColor;        roomState[roomId - 1].state = newState;
}
    String roomName = getRoomName(roomId);
    Serial.printf("[CMD] Room=%s color=%s state=%s\n",
              roomName.c_str(), newColor.c_str(), newState.c_str());
    RelayCmd cmd;
    cmd.roomId = roomId;

    strncpy(cmd.color, rawColor.c_str(), sizeof(cmd.color)-1);     cmd.color[sizeof(cmd.color)-1] = '\0';
    strncpy(cmd.state, newState.c_str(), sizeof(cmd.state)-1);     cmd.state[sizeof(cmd.state)-1] = '\0';

    if (xQueueSend(relayQueue, &cmd, 0) != pdTRUE) {
      Serial.println("[WARN] relayQueue full");
    }
  }
}
}


// CONNECTIONS 
void connectToWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  String savedSSID, savedPASS;
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
    prefs.begin("cfg", true);
    savedSSID = prefs.getString("ssid", SSID);   savedPASS = prefs.getString("pass", PASS);   prefs.end();
    xSemaphoreGive(prefsMutex);
  }
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 3) {
    Serial.printf("[WiFi] Attempt %d/3...\n", attempts + 1);
    // Full reset before each attempt
    WiFi.disconnect(true);   WiFi.mode(WIFI_OFF);
    vTaskDelay(pdMS_TO_TICKS(500));

    WiFi.mode(WIFI_STA);    WiFi.begin(savedSSID.c_str(), savedPASS.c_str());

    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry++ < 6) {
      vTaskDelay(pdMS_TO_TICKS(500));      Serial.print(".");
    }
    Serial.println();
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
  Serial.println("\n[WiFi] CONNECTED ");
  Serial.println("[WiFi] IP      : " + WiFi.localIP().toString());
  Serial.println("[WiFi] SSID    : " + savedSSID);   Serial.println("[WiFi] PASS    : " + savedPASS);
  mqtt.disconnect();  // keep your existing logic
} else {
    Serial.println("[WiFi] FAILED after 3 attempts — will retry");
  }
}

void syncTimeWithNTP() {
  Serial.print("[NTP] ");
  configTime(19800, 0, "pool.ntp.org", "time.google.com");  // add backup server
  int retry = 0;
  while (time(nullptr) < 1700000000 && retry++ < 10) {  
    vTaskDelay(pdMS_TO_TICKS(500));     Serial.print(".");
  }
  Serial.println(time(nullptr) > 1700000000 ? " OK" : " FAILED (will retry)");
}

void connectToMQTT() {
  String savedBroker, savedToken, savedClientId;
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {

  prefs.begin("cfg", true);
  savedBroker = prefs.getString("mqttServer", BROKER);
  savedToken  = prefs.getString("mqttToken", TOKEN);
  savedClientId = prefs.getString("mqttClientId", "");      prefs.end();
  xSemaphoreGive(prefsMutex);
}
  mqtt.setServer(savedBroker.c_str(), 1883);
  mqtt.setCallback(handleMqttMessage);

  while (!mqtt.connected()) {
    Serial.print("[MQTT] Connecting... ");
    String clientId;
    if (savedClientId.length() > 0) {
       clientId = savedClientId;     // use deviceId
      } else {
    clientId = "init-device";     // first boot only
     }
    if (mqtt.connect(clientId.c_str(), savedToken.c_str(), NULL)) {
      Serial.println("OK");
      mqtt.subscribe("v1/devices/me/attributes");
      mqtt.subscribe("v1/devices/me/attributes/response/+");
      //mqtt.publish("v1/devices/me/attributes/request/1",
                  //"{\"sharedKeys\":\"color,state\"}");
      prefs.begin("cfg", true);
      bool wifiUpdated = prefs.getBool("wifi_updated", false);     bool mqttUpdated = prefs.getBool("mqtt_updated", false);     
      bool ncrUpdated  = prefs.getBool("ncr_updated", false);      int ncr          = prefs.getInt("ncr", 4);
      String ssid      = prefs.getString("ssid", "");              String broker    = prefs.getString("mqttServer", "");
      String token     = prefs.getString("mqttToken",  "");
      String storedClientId = prefs.getString("mqttClientId", "");   prefs.end();
      if (wifiUpdated && mqttUpdated) {
  String json = "{\"msg\":{";

  json += "\"msg_type\":\"Config\"";
  json += ",\"timestamp\":\"" + getTimeString() + "\"";
  json += ",\"ssid\":\"" + ssid + "\"";
  json += ",\"mqttServer\":\"" + broker + "\"";
  json += ",\"mqttToken\":\"" + token + "\"";
  json += ",\"clientId\":\"" + storedClientId + "\"";

  // ALWAYS INCLUDE NCR
  json += ",\"ncr\":" + String(ncr);

  // ALWAYS INCLUDE ROOMS
  json += ",\"cr_names\":{";
  prefs.begin("cfg", true); 
  bool first = true;
  for (int i = 1; i <= ncr; i++) {
    String key = "room" + String(i);
    String name = prefs.getString(key.c_str(), "");

    if (name.length() > 0) {
      if (!first) json += ",";
      first = false;
      json += "\"" + String(i) + "\":\"" + name + "\"";
    }
  }
  prefs.end(); 
  json += "}";     json += ",\"status\":\"connected\"}}";
  mqtt.publish("v1/devices/me/telemetry", json.c_str());
  Serial.println("[TELE] WiFi + MQTT + NCR (forced)");
} 
   else if (ncrUpdated) {

  String json = "{\"msg\":{";
  json += "\"msg_type\":\"Config\"";
  json += ",\"timestamp\":\"" + getTimeString() + "\"";
  json += ",\"ncr\":" + String(ncr);

  // ===== cr_names =====
  json += ",\"cr_names\":{";
  prefs.begin("cfg", true);
  bool first = true;
  for (int i = 1; i <= ncr; i++) {
    String key = "room" + String(i);
    String name = prefs.getString(key.c_str(), "");
    if (name.length() > 0) {
      if (!first) json += ",";
      first = false;
      json += "\"";  json += String(i);   json += "\":\"";   json += name;  json += "\"";
    }
  }
  prefs.end();

  json += "}";
  json += ",\"status\":\"updated\"}}";
  mqtt.publish("v1/devices/me/telemetry", json.c_str());
  Serial.println("[TELE] NCR + Rooms update confirmed");
  }
  else if (wifiUpdated) {
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"msg\":{\"msg_type\":\"Config\",\"timestamp\":\"%s\",\"ssid\":\"%s\",\"status\":\"connected\"}}",
    getTimeString().c_str(),    ssid.c_str());
  mqtt.publish("v1/devices/me/telemetry", buf);
  Serial.println("[TELE] WiFi update confirmed");
      } else if (mqttUpdated) {
         char buf[256];
         snprintf(buf, sizeof(buf),
    "{\"msg\":{\"msg_type\":\"Config\",\"timestamp\":\"%s\",\"mqttServer\":\"%s\","
    "\"mqttToken\":\"%s\",\"clientId\":\"%s\",\"mqtt_status\":\"connected\"}}",
    getTimeString().c_str(),
    broker.c_str(),  token.c_str(),
    storedClientId.c_str());
        mqtt.publish("v1/devices/me/telemetry", buf);
        Serial.println("[TELE] MQTT update confirmed");
      }

      prefs.begin("cfg", false);
      prefs.putBool("wifi_updated", false);   prefs.putBool("mqtt_updated", false);
      prefs.putBool("ncr_updated", false);    prefs.end();
    } else {
      Serial.printf("FAILED rc=%d\n", mqtt.state());
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }
}

void otaUpdate(String firmwareUrl, String checksumUrl, String newVersion) {
  String deviceId = getDeviceId();
  char mqttBuf[256];
  Serial.println("[OTA] Starting...");

  // STEP 1: Download checksum
  HTTPClient httpCs;
  httpCs.begin(checksumUrl);   httpCs.setTimeout(10000);   int csCode = httpCs.GET();

  if (csCode != 200) {
    Serial.printf("[OTA] Checksum download failed: %d\n", csCode);    httpCs.end();
    snprintf(mqttBuf, sizeof(mqttBuf),
      "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"download_failed\"}}",
      deviceId.c_str());
    mqtt.publish("v1/devices/me/telemetry", mqttBuf);
    mqtt.loop(); delay(100);
    return;
  }
  String csBody = httpCs.getString();     httpCs.end();
  Serial.println("[OTA] Checksum raw: " + csBody);
  String expectedMD5 = "";       JsonDocument csDoc;
  DeserializationError csErr = deserializeJson(csDoc, csBody);

  if (!csErr) {
    if (csDoc["md5"].is<String>())           expectedMD5 = csDoc["md5"].as<String>();
    else if (csDoc["checksum"].is<String>()) expectedMD5 = csDoc["checksum"].as<String>();
  }
  if (expectedMD5 == "") {
    expectedMD5 = csBody;     expectedMD5.trim();
  }
  expectedMD5.toLowerCase();
  Serial.println("[OTA] Expected MD5: " + expectedMD5);

  if (expectedMD5.length() != 32) {
    Serial.println("[OTA] Invalid checksum format");
    snprintf(mqttBuf, sizeof(mqttBuf),
      "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"download_failed\"}}",
      deviceId.c_str());
    mqtt.publish("v1/devices/me/telemetry", mqttBuf);
    mqtt.loop(); delay(100);
    return;
  }
  // STEP 2 + 3: Download + Verify with Retry
  bool success = false;
  for (int attempt = 1; attempt <= OTA_MAX_RETRIES; attempt++) {
  // check abort
   if (otaAbort) {
       Serial.println("[OTA] Aborted by user");
       SD.remove("/firmware.bin");
       snprintf(mqttBuf, sizeof(mqttBuf),
       "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"aborted\"}}",
       deviceId.c_str());
    mqtt.publish("v1/devices/me/telemetry", mqttBuf);
    mqtt.loop();        delay(100);
    otaAbort = false;       return;
  }
  Serial.printf("[OTA] Attempt %d/%d\n", attempt, OTA_MAX_RETRIES);

  // publish retry info
  if (attempt > 1) {
    snprintf(mqttBuf, sizeof(mqttBuf),
      "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"retrying\",\"attempt\":%d}}",
      deviceId.c_str(), attempt);
    mqtt.publish("v1/devices/me/telemetry", mqttBuf);
    mqtt.loop(); delay(100);
  }
  SD.remove("/firmware.bin");

  // check WiFi before attempting — no blocking wait
if (WiFi.status() != WL_CONNECTED) {
  Serial.println("[OTA] WiFi not connected — skipping attempt");
  vTaskDelay(pdMS_TO_TICKS(3000));  // small delay then retry
  continue;
}
  HTTPClient httpFw;
  httpFw.begin(firmwareUrl);   httpFw.setTimeout(60000);     int fwCode = httpFw.GET();

  if (fwCode != 200) {
    Serial.printf("[OTA] Download failed: %d\n", fwCode);
    httpFw.end();
    vTaskDelay(pdMS_TO_TICKS(2000));  // wait before retry
    continue;
  }
  int contentLength = httpFw.getSize();
  Serial.printf("[OTA] Content length: %d\n", contentLength);
  WiFiClient* stream = httpFw.getStreamPtr();

  File fwFile = SD.open("/firmware.bin", FILE_WRITE);
  if (!fwFile) {
    Serial.println("[OTA] SD open failed");
    httpFw.end();     continue;
  }

  MD5Builder md5;
  md5.begin();
  uint8_t buf[1024];    int totalWritten = 0;
  unsigned long lastData = millis();    unsigned long lastPublishTime = 0;
  bool downloadOk = true;
  otaDownloading = true;
  while (totalWritten < contentLength) {
    // check WiFi during download 
    if (WiFi.status() != WL_CONNECTED) {
  Serial.println("[OTA] WiFi lost during download");
  downloadOk = false;      break;  // immediately break, no waiting
}

    if (!httpFw.connected()) {
      Serial.println("[OTA] HTTP disconnected");
      downloadOk = false;     break;
    }

    int available = stream->available();
    if (available) {
      int readBytes = stream->read(buf, min(available, (int)sizeof(buf)));
      fwFile.write(buf, readBytes);
      md5.add(buf, readBytes);     totalWritten += readBytes;
      lastData = millis();

       // check abort during download
      if (otaAbort) {
        Serial.println("[OTA] Aborted during download");
        downloadOk = false;     break;
      }
      // progress every 5 sec
      if (contentLength > 0 && millis() - lastPublishTime >= 5000) {
        lastPublishTime = millis();
        float percent = ((float)totalWritten * 100.0) / contentLength;
        Serial.printf("[OTA] Download: %.2f%%\n", percent);
        snprintf(mqttBuf, sizeof(mqttBuf),
          "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"downloading\",\"ota_percentage\":\"%.2f%%\"}}",
          deviceId.c_str(), percent);
        mqtt.publish("v1/devices/me/telemetry", mqttBuf);       
      }
    } else if (millis() - lastData > OTA_CHUNK_TIMEOUT) {
      Serial.println("[OTA] Chunk timeout — weak internet");
      downloadOk = false;    break;
    }

    MqttMsg msg;
  while (xQueueReceive(publishQueue, &msg, 0) == pdTRUE) {
    mqtt.publish(msg.topic, msg.payload);
    mqtt.loop(); 
    delay(1);  
  }
    mqtt.loop();
    vTaskDelay(1);
  }
  fwFile.close();    httpFw.end();
  otaDownloading = false;

  // handle abort
  if (otaAbort) {
    Serial.println("[OTA] Aborted — cleaning up");
    SD.remove("/firmware.bin");
    snprintf(mqttBuf, sizeof(mqttBuf),
      "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"aborted\"}}",
      deviceId.c_str());
    mqtt.publish("v1/devices/me/telemetry", mqttBuf);
    mqtt.loop(); delay(100);
    otaAbort = false;       return;
  }
  Serial.printf("[OTA] Written: %d / %d bytes\n", totalWritten, contentLength);

  // incomplete download
  if (!downloadOk || (contentLength > 0 && totalWritten != contentLength)) {
    Serial.println("[OTA] Incomplete — deleting and retrying");
    SD.remove("/firmware.bin");
    snprintf(mqttBuf, sizeof(mqttBuf),
      "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"download_incomplete\",\"attempt\":%d}}",
      deviceId.c_str(), attempt);
    mqtt.publish("v1/devices/me/telemetry", mqttBuf);
    mqtt.loop(); delay(100);
    vTaskDelay(pdMS_TO_TICKS(3000));  // wait before retry
    continue;
  }

  // verify MD5
  md5.calculate();
  String actualMD5 = md5.toString();
  actualMD5.toLowerCase();

  Serial.println("[OTA] Actual MD5:   " + actualMD5);     Serial.println("[OTA] Expected MD5: " + expectedMD5);

  if (actualMD5 != expectedMD5) {
    Serial.println("[OTA] Checksum mismatch — deleting and retrying");
    SD.remove("/firmware.bin");

    snprintf(mqttBuf, sizeof(mqttBuf),
      "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"checksum_mismatch\",\"attempt\":%d}}",
      deviceId.c_str(), attempt);
    mqtt.publish("v1/devices/me/telemetry", mqttBuf);
    mqtt.loop(); delay(100);
    vTaskDelay(pdMS_TO_TICKS(3000));      continue;
  }

  // all good
  Serial.println("[OTA] Download OK + Checksum OK");
  snprintf(mqttBuf, sizeof(mqttBuf),
    "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"download_finished\",\"ota_percentage\":\"100%%\"}}",
    deviceId.c_str());
  mqtt.publish("v1/devices/me/telemetry", mqttBuf);
  mqtt.loop(); delay(100);

  snprintf(mqttBuf, sizeof(mqttBuf),
    "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"checksum_match_success\"}}",
    deviceId.c_str());
  mqtt.publish("v1/devices/me/telemetry", mqttBuf);
  mqtt.loop(); delay(100);
  success = true;     break;
}

// all retries failed
if (!success) {
  Serial.println("[OTA] All retries failed — cleaning up");
  SD.remove("/firmware.bin");
  // if MQTT connected publish immediately
  if (mqtt.connected()) {
    snprintf(mqttBuf, sizeof(mqttBuf),
      "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"download_failed\"}}",
      deviceId.c_str());
    mqtt.publish("v1/devices/me/telemetry", mqttBuf);
    mqtt.loop();   delay(100);
  } else {
    // set flag — mqttTask will publish after reconnect
    otaFailed = true;    otaDeviceId = deviceId;
    Serial.println("[OTA] Will publish download_failed after reconnect");
  }
  return;
}

  // STEP 4: Flash
  File flashFile = SD.open("/firmware.bin");
  if (!flashFile) {
    Serial.println("[OTA] Cannot open firmware.bin");
    snprintf(mqttBuf, sizeof(mqttBuf),
      "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"download_failed\"}}",
      deviceId.c_str());
    mqtt.publish("v1/devices/me/telemetry", mqttBuf);
    mqtt.loop();    delay(100);
    return;
  }

  size_t flashSize = flashFile.size();
  if (!Update.begin(flashSize)) {
    Serial.println("[OTA] Not enough space");   flashFile.close();
    snprintf(mqttBuf, sizeof(mqttBuf),
      "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"download_failed\"}}",
      deviceId.c_str());
    mqtt.publish("v1/devices/me/telemetry", mqttBuf);
    mqtt.loop();   delay(100);
    return;
  }
  Serial.println("[OTA] Flashing...");
  uint8_t flashBuf[1024];     size_t flashWritten = 0;
  unsigned long lastFlashPublish = 0;

  while (flashFile.available()) {
    int readBytes = flashFile.read(flashBuf, sizeof(flashBuf));
    if (Update.write(flashBuf, readBytes) != readBytes) {
      Serial.println("[OTA] Write failed");
  return;
}  
    flashWritten += readBytes;

    if (millis() - lastFlashPublish >= 5000) {
      lastFlashPublish = millis();
      float percent = ((float)flashWritten * 100.0) / flashSize;
      Serial.printf("[OTA] Flash: %.2f%%\n", percent);
      snprintf(mqttBuf, sizeof(mqttBuf),
        "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"flashing\",\"ota_percentage\":\"%.2f%%\"}}",
        deviceId.c_str(), percent);
      mqtt.publish("v1/devices/me/telemetry", mqttBuf);
      mqtt.loop();
    }
    vTaskDelay(1);
  }
  flashFile.close();

  if (!Update.end()) {
    Serial.printf("[OTA] Flash failed: %d\n", Update.getError());
    snprintf(mqttBuf, sizeof(mqttBuf),
      "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"download_failed\"}}",
      deviceId.c_str());
    mqtt.publish("v1/devices/me/telemetry", mqttBuf);
    mqtt.loop();    delay(100);
    return;
  }

  // STEP 5: Success
  Serial.println("[OTA] Flash success! Rebooting...");
  snprintf(mqttBuf, sizeof(mqttBuf),
    "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"flash_success\"}}",
    deviceId.c_str());
  mqtt.publish("v1/devices/me/telemetry", mqttBuf);
  mqtt.loop();     delay(300);
  ESP.restart();
}

bool fetchTimeOverWiFi(time_t &currentTime) {
  if (timeApiUrl == "" || WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  http.begin(timeApiUrl);   http.setTimeout(5000);   int code = http.GET();

  if (code != 200) {
    Serial.printf("[TIME] API failed: %d\n", code);
    http.end();    return false;
  }
  String body = http.getString();
  http.end();

  JsonDocument doc;
  unsigned long unixTime = 0;
  DeserializationError err = deserializeJson(doc, body);
  if (!err) {
    if      (doc["unixtime"].is<unsigned long>())  unixTime = doc["unixtime"];
    else if (doc["unix"].is<unsigned long>())       unixTime = doc["unix"];
    else if (doc["timestamp"].is<unsigned long>())  unixTime = doc["timestamp"];
    else if (doc["epoch"].is<unsigned long>())      unixTime = doc["epoch"];
  }
  if (unixTime == 0) unixTime = body.toInt();
  if (unixTime < 1700000000) return false;
  currentTime = (time_t)unixTime;
  return true;
}

void initTime() {
  bool timeSyncOk = false;
  time_t currentTime = 0;
  bool timeInitialized = false;

  // Try HTTP API
  for (int attempt = 1; attempt <= 3 && !timeInitialized; attempt++) {
    Serial.printf("[TIME] API attempt %d/3\n", attempt);
    if (fetchTimeOverWiFi(currentTime)) {
      struct timeval tv = { .tv_sec = currentTime, .tv_usec = 0 };
      settimeofday(&tv, nullptr);    timeInitialized = true;
      Serial.println("[TIME] RTC set via API");
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // Fallback NTP
  if (!timeInitialized) {
    Serial.println("[TIME] Trying NTP...");
    configTime(19800, 0, "pool.ntp.org", "time.google.com");
    int retry = 0;
    while (time(nullptr) < 1700000000 && retry++ < 10) {
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (time(nullptr) > 1700000000) {
      currentTime = time(nullptr);
      timeInitialized = true;      // Serial.println("[TIME] RTC set via NTP");
    }
  }

  // Fallback compile time
  if (!timeInitialized) {
    Serial.println("[TIME] Fallback to compile time");
    struct tm compileTm = {};
    strptime(__DATE__ " " __TIME__, "%b %d %Y %H:%M:%S", &compileTm);     currentTime = mktime(&compileTm);
    struct timeval tv = { .tv_sec = currentTime, .tv_usec = 0 };          settimeofday(&tv, nullptr);
    timeFromCompile = true; 
  }

  struct tm finalTm;
  localtime_r(&currentTime, &finalTm);
  Serial.printf("[TIME] Final: %04d-%02d-%02d %02d:%02d:%02d\n",
    finalTm.tm_year+1900, finalTm.tm_mon+1, finalTm.tm_mday,
    finalTm.tm_hour, finalTm.tm_min, finalTm.tm_sec);
}


void relayTask(void *pv) {
  RelayCmd cmd;
  while (1) {
    if (xQueueReceive(relayQueue, &cmd, portMAX_DELAY)) {
  Serial.printf("[RTOS] RelayTask: %s %s\n", cmd.color, cmd.state);
  // GLOBAL OFF HANDLING 
  if (cmd.roomId == 0 && strcmp(cmd.state, "OFF") == 0) {
    Serial.println("[GLOBAL] Turning ALL relays OFF");

    int errCode = 1;   int res;    int failedRoom = -1;
    for (int i = 0; i < MAX_ROOMS; i++) {
      RoomRelay r = relayMap[i];

      res = controlRelay(r.green, false, false);    if (res < 0 && errCode > 0) { errCode = res; failedRoom = i + 1; }
      res = controlRelay(r.red, false, false);      if (res < 0 && errCode > 0) { errCode = res; failedRoom = i + 1; }
      res = controlRelay(r.orange, false, false);   if (res < 0 && errCode > 0) { errCode = res; failedRoom = i + 1; }
      res = controlRelay(r.buzzer, false, false);   if (res < 0 && errCode > 0) { errCode = res; failedRoom = i + 1; }
      roomState[i].state = "OFF";     roomState[i].color = "NONE";
    }
    relayOk = (errCode > 0);
    lastErrorCode = errCode;

    if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
    prefs.begin("d", false);
    for (int i = 0; i < MAX_ROOMS; i++) {
    String kc = "c" + String(i+1);         String ks = "s" + String(i+1);
    prefs.putString(kc.c_str(), "NONE");   prefs.putString(ks.c_str(), "OFF");
    }
    prefs.end();
    xSemaphoreGive(prefsMutex);
   }
   Serial.println("[STATE] all OFF saved to flash");

    // MQTT publish
    MqttMsg msg;
    char payload[256];
    String failedRoomName = (failedRoom > 0) ? getRoomName(failedRoom) : "";
    if (errCode > 0) {
      snprintf(payload, sizeof(payload),
        "{\"ts\":%llu,\"values\":{\"msg\":{\"msg_type\":\"relay_status\",\"status\":\"all_relays_off\"}}}",
        getTimestampMs()
      );
    } else {
      String errStr = getErrorCodeString(lastErrorCode); 
      snprintf(payload, sizeof(payload),
        "{\"ts\":%llu,\"values\":{\"msg\":{\"msg_type\":\"relay_status\",\"cr_name\":\"%s\",\"error_code\":%s}}}",
        getTimestampMs(),    failedRoomName.c_str(),    errStr.c_str()
      );
    }
    strncpy(msg.topic, "v1/devices/me/telemetry", sizeof(msg.topic)-1);
    msg.topic[sizeof(msg.topic)-1] = '\0';

    strncpy(msg.payload, payload, sizeof(msg.payload)-1);
    msg.payload[sizeof(msg.payload)-1] = '\0';
    xQueueSend(publishQueue, &msg, 0);
    continue;  
  }

      String roomName = getRoomName(cmd.roomId);
      if (strcmp(cmd.state, "MUTE") == 0) {
        RoomRelay r = relayMap[cmd.roomId - 1];
        Serial.printf("[RTOS] MUTE Room %d\n", cmd.roomId);
        controlRelay(r.buzzer, false, true);

        MqttMsg msg;
        char payload[256];
        snprintf(payload, sizeof(payload),
          "{\"ts\":%llu,\"values\":{\"msg\":{\"msg_type\":\"mqtt_sub\",\"cr_name\":\"%s\","
          "\"ack\":\"Alarm ack\",\"timestamp\":\"%s\"}}}",
          getTimestampMs(),  roomName.c_str(),   getTimeString().c_str()
        );
        strncpy(msg.topic, "v1/devices/me/telemetry", sizeof(msg.topic)-1);
        msg.topic[sizeof(msg.topic)-1] = '\0';
        strncpy(msg.payload, payload, sizeof(msg.payload)-1);
        msg.payload[sizeof(msg.payload)-1] = '\0';
        xQueueSend(publishQueue, &msg, 0);
      } else {
      applyRelayState(cmd.roomId, String(cmd.color), String(cmd.state));
          if (bootRestoreCount > 0) {
  bootRestoreCount--;
  continue; 
}

      String colorLower = String(cmd.color);        colorLower.toLowerCase();
roomState[cmd.roomId - 1].color = String(cmd.color);
roomState[cmd.roomId - 1].state = String(cmd.state);
color = colorLower;    state = String(cmd.state);     lastRoomId = cmd.roomId;

if (relayOk) {  // ADD — only save if relay responded
  if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
    prefs.begin("d", false);
    String keyColor = "c" + String(cmd.roomId);
    String keyState = "s" + String(cmd.roomId);
    prefs.putString(keyColor.c_str(), roomState[cmd.roomId - 1].color);
    prefs.putString(keyState.c_str(), roomState[cmd.roomId - 1].state);
    prefs.putInt("lastRoom", cmd.roomId);
    prefs.end();
    xSemaphoreGive(prefsMutex);
  }
}

//Serial.println("[STATE] Saved immediately");
        // QUEUE telemetry — never call mqtt.publish from relayTask
        {
          MqttMsg teleMsg;
          char teleBuf[256];    bool stateBool = (String(cmd.state) == "ON");
          if (!relayOk) {
            String errStr = getErrorCodeString(lastErrorCode); 
            snprintf(teleBuf, sizeof(teleBuf),
              "{\"ts\":%llu,\"values\":{\"msg\":{\"msg_type\":\"mqtt_sub\",\"cr_name\":\"%s\",\"color\":\"%s\",\"state\":%s,\"error_code\":%s}}}",
              getTimestampMs(),    roomName.c_str(),    cmd.color,
              stateBool ? "true" : "false",    errStr.c_str()
            );
          } else {
            snprintf(teleBuf, sizeof(teleBuf),
              "{\"ts\":%llu,\"values\":{\"msg\":{\"msg_type\":\"mqtt_sub\",\"cr_name\":\"%s\",\"color\":\"%s\",\"state\":%s}}}",
              getTimestampMs(),   roomName.c_str(),    cmd.color,
              stateBool ? "true" : "false"
            );
          }

          strncpy(teleMsg.topic, "v1/devices/me/telemetry", sizeof(teleMsg.topic)-1);
          teleMsg.topic[sizeof(teleMsg.topic)-1] = '\0';
          strncpy(teleMsg.payload, teleBuf, sizeof(teleMsg.payload)-1);
          teleMsg.payload[sizeof(teleMsg.payload)-1] = '\0';
          xQueueSend(publishQueue, &teleMsg, 0);
        }

        // ACK for RED — only if relay actually responded
          String cUpper = String(cmd.color);
          cUpper.toUpperCase();
          if (relayOk && cUpper == "RED" && String(cmd.state) == "ON") {
            Serial.println("[ACK] RED + BUZZER ON");
            MqttMsg msg;      char payload[256];
            snprintf(payload, sizeof(payload),
            "{\"ts\":%llu,\"values\":{\"msg\":{\"msg_type\":\"mqtt_sub\",\"cr_name\":\"%s\",\"ACK\":\"Red + Alarm ON\",\"timestamp\":\"%s\"}}}",
            getTimestampMs(),   roomName.c_str(),    getTimeString().c_str()
          );
          strncpy(msg.topic, "v1/devices/me/telemetry", sizeof(msg.topic)-1);     msg.topic[sizeof(msg.topic)-1] = '\0';
          strncpy(msg.payload, payload, sizeof(msg.payload)-1);                   msg.payload[sizeof(msg.payload)-1] = '\0';
          xQueueSend(publishQueue, &msg, 0);
        }
      }
    }
  }
}


void mqttTask(void *pv) {
  String deviceId = getDeviceId();
  Serial.println("[BOOT] Device ID: " + deviceId);
  connectToWiFi();     initTime();          // Sync time with NTP before MQTT to ensure correct timestamps in telemetry
  sdOk = checkSD();    sdAvailable = sdOk;

  prefs.begin("d", true);
int savedLastRoom = prefs.getInt("lastRoom", 1);
bootSnap.color = prefs.getString(("c" + String(savedLastRoom)).c_str(), "NONE");
bootSnap.state = prefs.getString(("s" + String(savedLastRoom)).c_str(), "OFF");
lastRoomId     = savedLastRoom;
prefs.end();
bootSnap.roomName = getRoomName(lastRoomId);
bootSnap.rtcTime  = (time(nullptr) > 1700000000) ? (unsigned long)time(nullptr) : 0;
bootSnap.captured = true;

  Serial.printf("[BOOT] Snapshot — color=%s state=%s rtc=%lu time=%s\n",
    bootSnap.color.c_str(), bootSnap.state.c_str(),     bootSnap.rtcTime, getTimeString().c_str());

  // Save to SD after time sync
  saveBootLogToSD();      connectToMQTT();                         
  static bool bootPublished = false;          // boot data sent once
  static bool ntpSynced     = false;          // time synced flag
  static bool bootRePublished  = false;      // boot data re-published after NTP sync (if needed)
  static bool bootSentWithValidTime = false;
  
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) connectToWiFi();         // reconnect if WiFi drops
    if (!mqtt.connected())              connectToMQTT();       // reconnect if MQTT drops
    mqtt.loop();   //no mutex needed — only mqttTask touches mqtt
    // NTP retry if not yet synced 
    if (!ntpSynced) {
      if (time(nullptr) > 1700000000) {
        ntpSynced = true;     Serial.println("[NTP] Synced (retry success)");
      } else {
        static unsigned long lastNtpRetry = 0;
        if (millis() - lastNtpRetry > 30000) {
          lastNtpRetry = millis();     Serial.println("[NTP] Retrying...");
          configTime(19800, 0, "pool.ntp.org", "time.google.com");
        }
      }
    }

    // Boot telemetry retry  
if (mqtt.connected() && !bootPublished) {
  bool ntpValid = (time(nullptr) > 1700000000);
  if (sdAvailable) {
    File f = SD.open("/boot_logs.json");
    if (f) {
      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, f);    f.close();

      if (!err && doc["logs"].is<JsonArray>()) {
        JsonArray logs = doc["logs"].as<JsonArray>();
        for (JsonObject entry : logs) {
          unsigned long savedBootRtc = entry["rtc"] | (unsigned long)0;  // old boot time
          bootSnap.rtcTime  = savedBootRtc;          // keep boot time in snap
          bootSnap.color    = entry["color"].as<String>();
          bootSnap.state    = entry["state"].as<String>();
          bootSnap.roomName = entry["room"].as<String>();
          bootSnap.captured = true;
          
            Serial.printf("[BOOT] Publishing — room=%s color=%s state=%s rtc=%lu\n",
            bootSnap.roomName.c_str(),   bootSnap.color.c_str(),
            bootSnap.state.c_str(),      bootSnap.rtcTime
          );
          sendTelemetry(0, "", "", true);
          vTaskDelay(pdMS_TO_TICKS(300));
        }
        SD.remove("/boot_logs.json");
        Serial.println("[BOOT] All logs published and deleted");
      }
    }

  } else {
    // No SD — just publish current boot
    sendTelemetry(0, "", "", true);
  }
  sendAttributes();    bootPublished = true;     bootSentWithValidTime = ntpValid;
}

    // Drain publish queue , Take all pending MQTT messages and send them
    MqttMsg msg;
    while (xQueueReceive(publishQueue, &msg, 0) == pdTRUE) {
      mqtt.publish(msg.topic, msg.payload);
    }

    // publish OTA failed after reconnect
  if (otaFailed && mqtt.connected()) {
     char failBuf[200];
     snprintf(failBuf, sizeof(failBuf),
        "{\"msg\":{\"device_id\":\"%s\",\"ota_progress\":\"download_failed\"}}",
        otaDeviceId.c_str());
        mqtt.publish("v1/devices/me/telemetry", failBuf);
        Serial.println("[OTA] download_failed published after reconnect");
        otaFailed = false;      otaDeviceId = "";
  }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// continuously checks button state every 50ms and sends MUTE command if RED alarm is active
void buttonTask(void *pv) {
  while (1) {
    checkButton(BUTTON1_PIN, 1);    checkButton(BUTTON2_PIN, 2);      checkButton(BUTTON3_PIN, 3);
    checkButton(BUTTON4_PIN, 4);    checkButton(BUTTON5_PIN, 5);      checkButton(BUTTON6_PIN, 6);  
    checkButton(BUTTON7_PIN, 7);    checkButton(BUTTON8_PIN, 8);  
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

bool loadConfigFromSD() {
  if (!sdAvailable) {
    Serial.println("[SD] Not available");     return false;
  }

  // ── LOAD device_config.json ──
  File devFile = SD.open("/device_config.json");
  if (!devFile) {
    Serial.println("[SD] device_config.json not found → creating default");
    File newFile = SD.open("/device_config.json", FILE_WRITE);
    if (newFile) {
      JsonDocument defDoc;
      defDoc["wifi"]["ssid"]        = "Realme7";          defDoc["wifi"]["pass"]  = "12345678";
      defDoc["mqtt"]["server"]      = "52.77.237.146";    defDoc["mqtt"]["token"]  = "s8o0joqrlv4hsxo46z5h";
      defDoc["mqtt"]["clientId"]    = "";                 defDoc["ota"]["version"]      = "";
      defDoc["ota"]["main_url"]     = "";                 defDoc["ota"]["checksum_url"] = "";
      defDoc["time"]["api_url"] = "";
      serializeJsonPretty(defDoc, newFile);
      newFile.close();              Serial.println("[SD] Default device_config.json created");
    }
  } else {
    JsonDocument devDoc;
    DeserializationError err = deserializeJson(devDoc, devFile);
    devFile.close();

    if (!err) {
      if (devDoc["wifi"].is<JsonObject>()) {
        String ssid = devDoc["wifi"]["ssid"] | "";
        String pass = devDoc["wifi"]["pass"] | "";
        if (ssid.length() > 0 && pass.length() > 0) {
          prefs.begin("cfg", false);          prefs.putString("ssid", ssid);
          prefs.putString("pass", pass);      prefs.end();
          Serial.println("[SD] WiFi loaded");
        }
      }
      if (devDoc["mqtt"].is<JsonObject>()) {
        String server   = devDoc["mqtt"]["server"]   | "";       String token    = devDoc["mqtt"]["token"]    | "";
        String clientId = devDoc["mqtt"]["clientId"] | "";       prefs.begin("cfg", false);
        if (server.length()   > 0) prefs.putString("mqttServer",   server);
        if (token.length()    > 0) prefs.putString("mqttToken",    token);
        if (clientId.length() > 0) prefs.putString("mqttClientId", clientId);
        prefs.end();          Serial.println("[SD] MQTT loaded");
      }
      if (devDoc["time"].is<JsonObject>()) {
        timeApiUrl = devDoc["time"]["api_url"] | "";
        if (timeApiUrl != "") Serial.println("[SD] Time API: " + timeApiUrl);
      }
      Serial.println("[SD] device_config.json loaded");
    } else {
      Serial.println("[SD] device_config.json parse failed");
    }
  }

  // ── LOAD flasher_config.json ──
  File flashFile = SD.open("/flasher_config.json");
  if (!flashFile) {
    Serial.println("[SD] flasher_config.json not found → creating default");
    File newFile = SD.open("/flasher_config.json", FILE_WRITE);
    if (newFile) {
      JsonDocument defDoc;
      defDoc["ncr"] = 4;
      defDoc["cr_names"]["1"] = "Room_1";        defDoc["cr_names"]["2"] = "Room_2";
      defDoc["cr_names"]["3"] = "Room_3";        defDoc["cr_names"]["4"] = "Room_4";
      serializeJsonPretty(defDoc, newFile);
      newFile.close();        Serial.println("[SD] Default flasher_config.json created");
    }
  } else {
    JsonDocument flashDoc;
    DeserializationError err = deserializeJson(flashDoc, flashFile);
    flashFile.close();

    if (!err) {
      int ncr = flashDoc["ncr"] | 4;
      if (xSemaphoreTake(prefsMutex, portMAX_DELAY)) {
        prefs.begin("cfg", false);        prefs.putInt("ncr", ncr);
        if (flashDoc["cr_names"].is<JsonObject>()) {
          JsonObject names = flashDoc["cr_names"];
          for (JsonPair kv : names) {
            String key = "room" + String(kv.key().c_str());
            prefs.putString(key.c_str(), kv.value().as<String>());
          }
        }
        prefs.end();
        xSemaphoreGive(prefsMutex);
      }
      Serial.printf("[SD] flasher_config.json loaded — NCR=%d\n", ncr);
    } else {
      Serial.println("[SD] flasher_config.json parse failed");
    }
  }
  return true;
}

void saveBootLogToSD() {
  if (!sdAvailable) {
    Serial.println("[BOOT] SD not available — log lost");
    return;
  }
  JsonDocument doc;
  File f = SD.open("/boot_logs.json");
  if (f) {
    deserializeJson(doc, f);
    f.close();
  }
  JsonArray logs;
  if (doc["logs"].is<JsonArray>()) {
    logs = doc["logs"].as<JsonArray>();
  } else {
    logs = doc["logs"].to<JsonArray>();
  }

  // No hard limit — remove only if too large (memory safety)
  while (logs.size() >= MAX_BOOT_LOGS) {
    logs.remove(0);  // remove oldest
  }
  JsonObject entry = logs.add<JsonObject>();
  entry["rtc"]       = bootSnap.rtcTime;
  entry["time"]      = getTimeString();
  entry["color"]     = bootSnap.color;
  entry["state"]     = bootSnap.state;
  entry["room"]      = bootSnap.roomName;
  entry["published"] = false;

  SD.remove("/boot_logs.json");
  File wf = SD.open("/boot_logs.json", FILE_WRITE);
  if (wf) {
    serializeJsonPretty(doc, wf);
    wf.close();           Serial.printf("[BOOT] Log saved to SD — total: %d\n", (int)logs.size());
  }
} 

// SETUP 
void setup() {
  Serial.begin(9600);
  delay(500);
  SPI.begin(18, 19, 23, 5);   // SCK, MISO, MOSI, CS
  sdAvailable = SD.begin(5);  // CS pin

  if (sdAvailable) {
    Serial.println("[SD] Ready");
  } else {
    Serial.println("[SD] Not available");
  }
  //  Direction button with internal pull-up
  pinMode(BUTTON1_PIN, INPUT_PULLUP);      pinMode(BUTTON2_PIN, INPUT_PULLUP);     pinMode(BUTTON3_PIN, INPUT_PULLUP);      
  pinMode(BUTTON4_PIN, INPUT_PULLUP);      pinMode(BUTTON5_PIN, INPUT_PULLUP);     pinMode(BUTTON6_PIN, INPUT_PULLUP);
  pinMode(BUTTON7_PIN, INPUT_PULLUP);      pinMode(BUTTON8_PIN, INPUT_PULLUP);
  RS485.begin(9600, SERIAL_8N1, RXD2, TXD2);  // Initialize RS485 serial communication (8-bit, no parity, 1 stop)
  mqtt.setBufferSize(512);

  // Create RTOS primitives first
  //mqttMutex  = xSemaphoreCreateMutex(); 
  relayQueue = xQueueCreate(15, sizeof(RelayCmd));     // Can store  commands
  prefsMutex = xSemaphoreCreateMutex();   
  if (relayQueue == NULL) {
    Serial.println("[ERROR] Queue creation failed!");
  }
  publishQueue = xQueueCreate(5, sizeof(MqttMsg));    // Can store 5 MQTT messages
  bool sdOk = loadConfigFromSD();

if (!sdOk) {
  Serial.println("[BOOT] Using prefs config");
}
prefs.begin("cfg", true);
int ncr = prefs.getInt("ncr", 4);
prefs.end();
prefs.begin("cfg", false);
for (int i = 1; i <= ncr; i++){
  String key = "room" + String(i);
  if (!prefs.isKey(key.c_str())) {
    prefs.putString(key.c_str(), "Room_" + String(i));
  }
}
prefs.end();

  if (publishQueue == NULL) {
    Serial.println("[ERROR] publishQueue creation failed!");
  }
   // Start tasks — network happens inside mqttTask
  xTaskCreatePinnedToCore(relayTask,  "RelayTask",  4096, NULL, 2, NULL, 1);   // Priority = 2  Core = 1
  xTaskCreatePinnedToCore(buttonTask, "ButtonTask",  2048, NULL, 3, NULL, 1);  // Priority = 3 (higher than relay for immediate response) Core = 1
  xTaskCreatePinnedToCore(mqttTask,   "MQTTTask",   8192, NULL, 1, NULL, 0);
  vTaskDelay(pdMS_TO_TICKS(300));

//  Load ALL rooms into roomState[] first before queuing anything
prefs.begin("d", true);
for (int i = 0; i < MAX_ROOMS; i++) {
  String kc = "c" + String(i+1);
  String ks = "s" + String(i+1);
  roomState[i].color = prefs.getString(kc.c_str(), "");
  roomState[i].state = prefs.getString(ks.c_str(), "");
}
prefs.end();

// Queue relay commands only for ON rooms
bool anyRestored = false;
for (int i = 0; i < MAX_ROOMS; i++) {
  if (roomState[i].state == "ON" &&
      roomState[i].color != "" &&
      roomState[i].color != "none" &&
      roomState[i].color != "NONE") {
    Serial.printf("[BOOT] Restoring Room=%d color=%s\n", i+1, roomState[i].color.c_str());

    RelayCmd cmd;
    cmd.roomId = i + 1;
    strncpy(cmd.color, roomState[i].color.c_str(), sizeof(cmd.color)-1);
    cmd.color[sizeof(cmd.color)-1] = '\0';
    strncpy(cmd.state, roomState[i].state.c_str(), sizeof(cmd.state)-1);
    cmd.state[sizeof(cmd.state)-1] = '\0';

    xQueueSend(relayQueue, &cmd, portMAX_DELAY);
    anyRestored = true;
    bootRestoreCount++; 
  }
}

//  Set globals to last active ON room
if (anyRestored) {
  for (int i = MAX_ROOMS - 1; i >= 0; i--) {
    if (roomState[i].state == "ON") {
      color = roomState[i].color;     state = roomState[i].state;
      lastRoomId = i + 1;             break;
    }
  }
}
Serial.println("[READY]");
}

void loop() {
  vTaskDelete(NULL);
}
  







