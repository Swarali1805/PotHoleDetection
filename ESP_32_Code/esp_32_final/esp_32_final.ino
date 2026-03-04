#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <RTClib.h>

// ================= WIFI CONFIGURATION =================
// [USER ACTION REQUIRED]: Edit the .env file in the root folder and run python update_wifi.py
#include "credentials.h"

// ================= SETTINGS =================
#define MPU_ADDR 0x69 // Common addresses are 0x68 and 0x69
#define SDA_PIN 21
#define SCL_PIN 22

// Threshold for physical detection (Optional, maintained from original code)
#define POTHOLE_THRESHOLD 20000

// ================= GLOBAL OBJECTS =================
WebServer server(80);
RTC_DS3231 rtc;

float ax_ms2, ay_ms2, az_ms2;
bool potholeDetected = false;

// ----------- MPU WAKE FUNCTION -------------
void initMPU() {
  Serial.println("Initializing MPU6050/6500...");
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // Power management register
  Wire.write(0x00);   // Wake up
  byte error = Wire.endTransmission();
  
  if(error == 0) {
    Serial.println("MPU Ready!");
  } else {
    Serial.print("MPU Error: ");
    Serial.println(error);
  }
}

// ----------- READ ACCEL --------------------
void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)6, true);

  if (Wire.available() == 6) {
    int16_t raw_ax = Wire.read() << 8 | Wire.read();
    int16_t raw_ay = Wire.read() << 8 | Wire.read();
    int16_t raw_az = Wire.read() << 8 | Wire.read();

    // The Python algorithm expects m/s^2 for severity calculation: 
    // J_MAX is 20.0 m/s^3. Default MPU scaling is +/- 2g = 16384 LSB/g. 
    // 1g = 9.81 m/s^2.
    ax_ms2 = (raw_ax / 16384.0) * 9.81;
    ay_ms2 = (raw_ay / 16384.0) * 9.81;
    az_ms2 = (raw_az / 16384.0) * 9.81;

    // Background threshold detection (legacy code)
    if (abs(raw_ax) > POTHOLE_THRESHOLD || abs(raw_ay) > POTHOLE_THRESHOLD || abs(raw_az) > POTHOLE_THRESHOLD) {
      potholeDetected = true;
    }
  } else {
    ax_ms2 = 0.0; ay_ms2 = 0.0; az_ms2 = 0.0;
  }
}

// ----------- WEB RESPONSE ------------------
void handleQuery() {
  // Read latest sensor data
  readMPU();

  // Python query looks like: http://<IP>/query?pothole_id=42
  int req_id = 0;
  if(server.hasArg("pothole_id")) {
    req_id = server.arg("pothole_id").toInt();
  }

  // Fetch RTC Date & Time safely
  String timestamp = "2000-01-01T00:00:00";
  bool rtc_ok = false;
  try {
    DateTime now = rtc.now();
    timestamp = String(now.timestamp());
    rtc_ok = true;
  } catch (...) {
  }

  // Construct JSON Payload adhering to python's json.loads() expectations
  String json = "{";
  json += "\"pothole_id\":" + String(req_id) + ",";
  json += "\"timestamp\":\"" + timestamp + "\",";
  json += "\"latitude\":0.000000,";
  json += "\"longitude\":0.000000,";
  json += "\"ax\":" + String(ax_ms2, 2) + ",";
  json += "\"ay\":" + String(ay_ms2, 2) + ",";
  json += "\"az\":" + String(az_ms2, 2) + ",";
  json += "\"mpu_ok\":true,";
  json += "\"gps_ok\":false,";
  json += "\"rtc_ok\":" + String(rtc_ok ? "true" : "false");
  json += "}";

  if (potholeDetected) {
     potholeDetected = false;
  }

  server.send(200, "application/json", json);
}

// ================= SETUP ===================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  initMPU();

  // Initialize RTC
  Serial.println("Initializing RTC...");
  if (!rtc.begin()) {
    Serial.println("RTC NOT found!");
  } else {
    Serial.println("RTC found.");
  }

  // IMPORTANT FIX: Sensor node MUST be a Station (WIFI_STA) connecting to 
  // same router as ESP32-CAM and PC. It previously hosted an isolated Access Point!
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.println("\nConnecting to WiFi...");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("ESP32 Sensor IP Address: ");
    Serial.println(WiFi.localIP()); 
  } else {
    Serial.println("\nFailed to connect. Will continue trying in loop...");
  }

  // Start Server
  server.on("/query", handleQuery);
  server.begin();
  Serial.println("HTTP Server started on port 80.");
}

// ================= LOOP ====================
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    // Reconnect logic if WiFi drops
    WiFi.reconnect();
    delay(1000);
  }
  
  // Listen for Python queries
  server.handleClient();
}