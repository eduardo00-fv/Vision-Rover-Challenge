/* Diagnóstico físico del CenfoBot — Arduino C++ / ESP32.
 * Sin Wi-Fi, cámara ni telemetría.
 * Monitor serial: 115200. Motores desarmados al arrancar.
 */
#include <Wire.h>
#include <FastLED.h>

// Pinout sustentado por codigos/ideaboard.py, code_4IR.py, code_ultrasonic.py
// y color_detect.py del repositorio CenfoBot.
constexpr uint8_t M1_FORWARD = 12, M1_BACKWARD = 14;
constexpr uint8_t M2_FORWARD = 13, M2_BACKWARD = 15;
constexpr uint8_t LINE_PINS[] = {36, 39, 34, 35};
constexpr uint8_t SONAR_TRIG = 25, SONAR_ECHO = 26;
constexpr uint8_t COLOR_LED_PIN = 4, COLOR_INPUT_PIN = 39;
constexpr uint32_t PWM_HZ = 1000;
constexpr uint8_t PWM_BITS = 12;
constexpr uint32_t PWM_MAX = (1U << PWM_BITS) - 1U;
constexpr int MAX_PERCENT = 35;
constexpr uint32_t MAX_MOVE_MS = 1500;
constexpr uint32_t SONAR_TIMEOUT_US = 30000;
constexpr uint32_t COLOR_SETTLE_MS = 200;

CRGB color_led[1];
bool armed = false, moving = false;
uint32_t stop_at_ms = 0;
uint8_t lsm_address = 0;

void writeWheel(uint8_t forward, uint8_t backward, int percent) {
  const uint32_t duty = static_cast<uint32_t>(abs(percent)) * PWM_MAX / 100U;
  ledcWrite(forward, percent > 0 ? duty : 0);
  ledcWrite(backward, percent < 0 ? duty : 0);
}

void stopMotors() {
  writeWheel(M1_FORWARD, M1_BACKWARD, 0);
  writeWheel(M2_FORWARD, M2_BACKWARD, 0);
  moving = false;
  stop_at_ms = 0;
}

void runMotor(int left, int right, uint32_t duration_ms) {
  if (!armed) { Serial.println("ERROR: escriba ARM antes de MOTOR"); return; }
  if (abs(left) > MAX_PERCENT || abs(right) > MAX_PERCENT || duration_ms == 0 || duration_ms > MAX_MOVE_MS) {
    Serial.printf("ERROR: PWM -%d..%d y duración 1..%lu ms\n", MAX_PERCENT, MAX_PERCENT, MAX_MOVE_MS);
    return;
  }
  writeWheel(M1_FORWARD, M1_BACKWARD, left);
  writeWheel(M2_FORWARD, M2_BACKWARD, right);
  stop_at_ms = millis() + duration_ms;
  moving = true;
  Serial.printf("MOTOR M1=%d M2=%d ms=%lu\n", left, right, duration_ms);
}

void readLines(uint16_t samples) {
  samples = constrain(samples, 1, 50);
  uint32_t total[4] = {0, 0, 0, 0};
  for (uint16_t s = 0; s < samples; ++s) {
    for (uint8_t i = 0; i < 4; ++i) total[i] += analogRead(LINE_PINS[i]);
    delay(2);
  }
  Serial.printf("LINE n=%u FL=%lu FR=%lu BL=%lu BR=%lu (ADC 0..4095)\n", samples,
                total[0] / samples, total[1] / samples, total[2] / samples, total[3] / samples);
  Serial.println("NOTA: IO39 también aparece en el ejemplo oficial de color; confirme el montaje antes de usar ambos a la vez.");
}

void readSonar(uint16_t samples) {
  samples = constrain(samples, 1, 30);
  uint16_t valid = 0;
  float total = 0.0F, minimum = 100000.0F, maximum = 0.0F;
  for (uint16_t s = 0; s < samples; ++s) {
    digitalWrite(SONAR_TRIG, LOW); delayMicroseconds(3);
    digitalWrite(SONAR_TRIG, HIGH); delayMicroseconds(10); digitalWrite(SONAR_TRIG, LOW);
    const uint32_t microseconds = pulseIn(SONAR_ECHO, HIGH, SONAR_TIMEOUT_US);
    if (microseconds != 0) {
      const float cm = microseconds / 58.0F;
      total += cm; minimum = min(minimum, cm); maximum = max(maximum, cm); ++valid;
    }
    delay(40);
  }
  if (!valid) Serial.println("SONAR: sin eco; revise jumper SELECT-Vin, TRIG/ECHO y nivel de ECHO a 3.3 V");
  else Serial.printf("SONAR n=%u/%u cm_mean=%.1f min=%.1f max=%.1f\n", valid, samples, total / valid, minimum, maximum);
}

uint16_t sampleColor(const CRGB& color, uint16_t samples) {
  color_led[0] = color; FastLED.show(); delay(COLOR_SETTLE_MS);
  uint32_t total = 0;
  for (uint16_t s = 0; s < samples; ++s) { total += analogRead(COLOR_INPUT_PIN); delay(5); }
  color_led[0] = CRGB::Black; FastLED.show();
  return total / samples;
}

void readColor(uint16_t samples) {
  samples = constrain(samples, 1, 50);
  const uint16_t red = sampleColor(CRGB::Red, samples);
  const uint16_t green = sampleColor(CRGB::Green, samples);
  const uint16_t blue = sampleColor(CRGB::Blue, samples);
  const char* detected = red <= green && red <= blue ? "RED" : (green <= blue ? "GREEN" : "BLUE");
  Serial.printf("COLOR n=%u R=%u G=%u B=%u detected=%s (menor lectura, según ejemplo oficial)\n", samples, red, green, blue, detected);
}

bool lsmRead(uint8_t reg, uint8_t* data, size_t count) {
  if (!lsm_address) return false;
  Wire.beginTransmission(lsm_address); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(static_cast<int>(lsm_address), static_cast<int>(count)) != count) return false;
  for (size_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

bool initImu() {
  // LSM6DS3TR-C: identificador de dispositivo y direcciones I2C estándar.
  constexpr uint8_t WHO_AM_I = 0x0F;
  for (uint8_t address : {0x6A, 0x6B}) {
    lsm_address = address; uint8_t identity = 0;
    if (lsmRead(WHO_AM_I, &identity, 1) && identity == 0x6A) {
      Serial.printf("IMU LSM6DS3TR-C en 0x%02X\n", address); return true;
    }
  }
  lsm_address = 0; Serial.println("IMU no encontrada; ejecute I2C y revise Qwiic"); return false;
}

void readImu(uint16_t samples) {
  if (!lsm_address && !initImu()) return;
  samples = constrain(samples, 1, 100);
  // CTRL2_G: 26 Hz, ±250 dps. OUTZ_G: 8.75 mdps/LSB.
  Wire.beginTransmission(lsm_address); Wire.write(0x11); Wire.write(0x20); Wire.endTransmission();
  float total = 0.0F, minimum = 1e6F, maximum = -1e6F;
  for (uint16_t s = 0; s < samples; ++s) {
    uint8_t raw[2];
    if (!lsmRead(0x26, raw, sizeof(raw))) { Serial.println("IMU error de lectura"); return; }
    const int16_t z = static_cast<int16_t>(raw[0] | (raw[1] << 8));
    const float dps = z * 0.00875F;
    total += dps; minimum = min(minimum, dps); maximum = max(maximum, dps); delay(20);
  }
  Serial.printf("IMU gyro_z n=%u mean=%.3f dps min=%.3f max=%.3f (quieto: use mean como drift)\n", samples, total / samples, minimum, maximum);
}

void scanI2c() {
  uint8_t found = 0; Serial.println("I2C:");
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) { Serial.printf("  0x%02X\n", address); ++found; }
  }
  if (!found) Serial.println("  sin dispositivos");
}

void printHelp() {
  Serial.println("HELP | STATUS | ARM | DISARM | STOP");
  Serial.println("MOTOR <M1 -35..35> <M2 -35..35> <ms 1..1500>");
  Serial.println("LINE [n] | SONAR [n] | COLOR [n] | IMU [n] | I2C");
  Serial.println("Este CenfoBot no tiene encoders: no hay PPR ni PI de velocidad por pulsos.");
}

void command(String line) {
  line.trim(); line.toUpperCase();
  if (line == "HELP") printHelp();
  else if (line == "STATUS") { Serial.printf("STATUS armed=%s moving=%s\n", armed ? "YES" : "NO", moving ? "YES" : "NO"); readLines(3); }
  else if (line == "ARM") { armed = true; Serial.println("ARMED: rover en área libre; las órdenes tienen corte automático"); }
  else if (line == "DISARM" || line == "STOP") { stopMotors(); armed = false; Serial.println("STOPPED + DISARMED"); }
  else if (line == "I2C") scanI2c();
  else if (line.startsWith("MOTOR ")) { int m1, m2, ms; if (sscanf(line.c_str(), "MOTOR %d %d %d", &m1, &m2, &ms) == 3) runMotor(m1, m2, ms); else Serial.println("Uso: MOTOR <M1> <M2> <ms>"); }
  else if (line.startsWith("LINE")) { int n = 10; sscanf(line.c_str(), "LINE %d", &n); readLines(n); }
  else if (line.startsWith("SONAR")) { int n = 10; sscanf(line.c_str(), "SONAR %d", &n); readSonar(n); }
  else if (line.startsWith("COLOR")) { int n = 10; sscanf(line.c_str(), "COLOR %d", &n); readColor(n); }
  else if (line.startsWith("IMU")) { int n = 30; sscanf(line.c_str(), "IMU %d", &n); readImu(n); }
  else if (line.length()) Serial.println("ERROR: HELP");
}

void setup() {
  Serial.begin(115200); delay(300);
  ledcAttach(M1_FORWARD, PWM_HZ, PWM_BITS); ledcAttach(M1_BACKWARD, PWM_HZ, PWM_BITS);
  ledcAttach(M2_FORWARD, PWM_HZ, PWM_BITS); ledcAttach(M2_BACKWARD, PWM_HZ, PWM_BITS);
  stopMotors();
  for (uint8_t pin : LINE_PINS) { pinMode(pin, INPUT); analogSetPinAttenuation(pin, ADC_11db); }
  pinMode(COLOR_INPUT_PIN, INPUT); analogSetPinAttenuation(COLOR_INPUT_PIN, ADC_11db);
  pinMode(SONAR_TRIG, OUTPUT); pinMode(SONAR_ECHO, INPUT); digitalWrite(SONAR_TRIG, LOW);
  FastLED.addLeds<WS2812, COLOR_LED_PIN, GRB>(color_led, 1); color_led[0] = CRGB::Black; FastLED.show();
  Wire.begin();
  Serial.println("CenfoBot diagnóstico C++: motores detenidos, sin Wi-Fi ni cámara. Escriba HELP.");
}

void loop() {
  if (moving && static_cast<int32_t>(millis() - stop_at_ms) >= 0) { stopMotors(); Serial.println("RESULT corte automático; motores detenidos"); }
  if (Serial.available()) command(Serial.readStringUntil('\n'));
}
