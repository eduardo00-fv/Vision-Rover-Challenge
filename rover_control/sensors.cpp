#include "sensors.h"

#include <FastLED.h>

#include "hardware_config.h"

namespace {
constexpr uint32_t LINE_PERIOD_MS = 25;
constexpr uint32_t SONAR_PERIOD_MS = 100;
constexpr uint32_t COLOR_SETTLE_MS = 25;
constexpr uint32_t SONAR_TIMEOUT_US = 12000;

CRGB color_led[1];

bool activeLine(uint16_t value) {
  return LINE_BLACK_IS_LOW ? value == LOW : value == HIGH;
}

void setColorLed(const CRGB& color) {
  color_led[0] = color;
  FastLED.show();
}
}  // namespace

void Sensors::begin() {
  for (uint8_t pin : LINE_PINS) {
    pinMode(pin, INPUT);
  }
#if VRC_ENABLE_COLOR
  pinMode(COLOR_SENSOR_PIN, INPUT);
  analogSetPinAttenuation(COLOR_SENSOR_PIN, ADC_11db);
  FastLED.addLeds<WS2812, COLOR_LED_PIN, GRB>(color_led, 1);
  setColorLed(CRGB::Black);
#endif
#if VRC_ENABLE_SONAR
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
  digitalWrite(SONAR_TRIG_PIN, LOW);
#endif
  Serial.println("[sensores] IR listos; sonar/color segun configuracion");
}

void Sensors::sampleLines() {
  for (uint8_t i = 0; i < 4; ++i) {
    snapshot_.line[i] = digitalRead(LINE_PINS[i]);
    snapshot_.line_active[i] = activeLine(snapshot_.line[i]);
  }
}

void Sensors::sampleSonar() {
  digitalWrite(SONAR_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_PIN, LOW);
  const uint32_t pulse_us = pulseIn(SONAR_ECHO_PIN, HIGH, SONAR_TIMEOUT_US);
  snapshot_.distance_valid = pulse_us != 0;
  if (snapshot_.distance_valid) snapshot_.distance_cm = pulse_us * 0.01715F;
}

void Sensors::pollColor() {
  const uint32_t now = millis();
  if (now - color_stage_ms_ < COLOR_SETTLE_MS) return;
  color_stage_ms_ = now;

  // Cada etapa toma la lectura del LED encendido en la etapa anterior y luego
  // enciende el siguiente. No introduce delay y no bloquea la red TCP.
  switch (color_stage_) {
    case 0: setColorLed(CRGB::Red); color_stage_ = 1; break;
    case 1: snapshot_.color_red = analogRead(COLOR_SENSOR_PIN);
            setColorLed(CRGB::Green); color_stage_ = 2; break;
    case 2: snapshot_.color_green = analogRead(COLOR_SENSOR_PIN);
            setColorLed(CRGB::Blue); color_stage_ = 3; break;
    default:
      snapshot_.color_blue = analogRead(COLOR_SENSOR_PIN);
      setColorLed(CRGB::Black);
      if (COLOR_REFLECTION_IS_LOW) {
        if (snapshot_.color_red <= snapshot_.color_green && snapshot_.color_red <= snapshot_.color_blue)
          snapshot_.color_name = "red";
        else if (snapshot_.color_green <= snapshot_.color_blue)
          snapshot_.color_name = "green";
        else snapshot_.color_name = "blue";
      } else {
        if (snapshot_.color_red >= snapshot_.color_green && snapshot_.color_red >= snapshot_.color_blue)
          snapshot_.color_name = "red";
        else if (snapshot_.color_green >= snapshot_.color_blue)
          snapshot_.color_name = "green";
        else snapshot_.color_name = "blue";
      }
      color_stage_ = 0;
      break;
  }
}

void Sensors::poll() {
  const uint32_t now = millis();
  if (now - last_line_ms_ >= LINE_PERIOD_MS) {
    last_line_ms_ = now;
    sampleLines();
  }
  if (VRC_ENABLE_SONAR && now - last_sonar_ms_ >= SONAR_PERIOD_MS) {
    last_sonar_ms_ = now;
    sampleSonar();
  }
  if (VRC_ENABLE_COLOR) pollColor();
}

bool Sensors::boundaryDetected() const {
  if (!LINE_STOP_ON_DETECTION) return false;
  return snapshot_.line_active[0] || snapshot_.line_active[1] ||
         snapshot_.line_active[2] || snapshot_.line_active[3];
}

bool Sensors::obstacleNear(float cm) const {
  return snapshot_.distance_valid && snapshot_.distance_cm > 0.0F &&
         snapshot_.distance_cm < cm;
}
