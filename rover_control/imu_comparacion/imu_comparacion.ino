// Banco IMU: compatible con Arduino ESP32 2.0.17 y 3.3.11.
// Sin PWM, Wi-Fi ni bibliotecas de sensores. RUN: 30 s; ! cancela.
#include <Arduino.h>
#include <Wire.h>
#include <esp_arduino_version.h>
#include <esp_system.h>
#include "../motion/lsm6ds3_gyro.h"

#ifndef ESP_ARDUINO_VERSION_STR
#define IMU_STRINGIFY_INNER(x) #x
#define IMU_STRINGIFY(x) IMU_STRINGIFY_INNER(x)
#define ESP_ARDUINO_VERSION_STR IMU_STRINGIFY(ESP_ARDUINO_VERSION_MAJOR) "." IMU_STRINGIFY(ESP_ARDUINO_VERSION_MINOR) "." IMU_STRINGIFY(ESP_ARDUINO_VERSION_PATCH)
#endif

Lsm6ds3Gyro<TwoWire> imu(Wire);
bool running = false;
uint32_t started = 0, lastSample = 0, lastPoll = 0, samples = 0;
float total = 0, low = 1e6F, high = -1e6F;
String command;

void finish(const char* result) {
  running = false;
  Serial.printf("RESULT %s elapsed_ms=%lu samples=%lu mean=%.4f min=%.4f max=%.4f motors=OFF\n",
                result, millis() - started, samples, samples ? total / samples : 0, low, high);
}

void setup() {
  for (uint8_t pin : {12, 14, 13, 15}) {
    digitalWrite(pin, LOW); pinMode(pin, OUTPUT);
  }
  Serial.begin(115200);
  Wire.begin(21, 22, 100000);
  Wire.setTimeOut(25);
  delay(300);
  Serial.printf("IMU BENCH core=%s idf=%s chip=%012llX SDA=21 SCL=22 hz=100000 timeout=25 motors=OFF\n",
                ESP_ARDUINO_VERSION_STR, esp_get_idf_version(), ESP.getEfuseMac());
  Serial.println("RUN: 30 s de lectura; mantener quieto. !: cancelar. INFO: version.");
}

void execute() {
  command.trim();
  if (command == "INFO") {
    Serial.printf("INFO core=%s idf=%s chip=%012llX motors=OFF\n",
                  ESP_ARDUINO_VERSION_STR, esp_get_idf_version(), ESP.getEfuseMac());
  } else if (command == "RUN" && !running) {
    samples = 0; total = 0; low = 1e6F; high = -1e6F;
    started = millis();
    if (!imu.detect()) { finish("DETECT_ERROR"); }
    else if (!imu.configure()) { finish("CONFIG_ERROR"); }
    else {
      Serial.printf("START address=0x%02X warming_ms=250 duration_ms=30000\n", imu.address());
      delay(250);
      started = lastSample = millis(); lastPoll = micros(); running = true;
    }
  }
  command = "";
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '!') { if (running) finish("CANCELLED"); command = ""; }
    else if (c == '\n') execute();
    else if (command.length() < 32) command += c;
    else command = "";
  }
  if (!running) { delay(1); return; }
  if (millis() - started >= 30000) { finish("OK"); return; }
  if (micros() - lastPoll < 2000) return;
  lastPoll = micros();
  float z = 0;
  int result = imu.sample(z);
  if (result < 0) { finish("I2C_ERROR"); return; }
  if (result == 1) {
    lastSample = millis(); ++samples; total += z;
    if (z < low) low = z;
    if (z > high) high = z;
  }
  if (millis() - lastSample > 100) finish("STALE");
}
