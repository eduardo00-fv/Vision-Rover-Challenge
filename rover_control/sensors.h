#pragma once

#include <Arduino.h>

struct SensorSnapshot {
  uint16_t line[4] = {0, 0, 0, 0};
  bool line_active[4] = {false, false, false, false};
  bool distance_valid = false;
  float distance_cm = 0.0F;
  uint16_t color_red = 0;
  uint16_t color_green = 0;
  uint16_t color_blue = 0;
  const char* color_name = "unknown";
};

class Sensors {
 public:
  void begin();
  void poll();
  const SensorSnapshot& snapshot() const { return snapshot_; }
  uint8_t lineMask() const;  // bit i = lectura HIGH de S(i+1)
  bool obstacleNear(float cm) const;

 private:
  SensorSnapshot snapshot_;
  uint32_t last_line_ms_ = 0;
  uint32_t last_sonar_ms_ = 0;
  uint32_t color_stage_ms_ = 0;
  uint8_t color_stage_ = 0;

  void sampleLines();
  void sampleSonar();
  void pollColor();
};
