#pragma once
#include <cmath>
#include <cstdint>

// Shared pure C++: relative heading PD, NOT wheel-speed or absolute-position control.
struct HeadingControl {
  float kp = 0.6F;           // PWM percentage points / degree
  float kd = 0.08F;          // PWM percentage points / (degree/s)
  float limit = 4.0F;        // maximum differential correction
  float yaw = 0, correction = 0, left = 0, right = 0;
  float base_left = 0, base_right = 0, bias = 0;
  int sign = 0;              // raw gyro sign for a physical LEFT turn
  uint32_t last_us = 0;

  bool begin(float l, float r, float offset, int direction, uint32_t now) {
    if (!std::isfinite(l) || !std::isfinite(r) || !std::isfinite(offset) ||
        !std::isfinite(kp) || !std::isfinite(kd) || !std::isfinite(limit) ||
        l < 10 || l > 25 || r < 10 || r > 25 || std::fabs(offset) > 5 ||
        (direction != 1 && direction != -1) || kp < 0 || kp > 2 ||
        kd < 0 || kd > 0.5F || limit < 0 || limit > 5) return false;
    base_left = left = l; base_right = right = r;
    bias = offset; sign = direction; last_us = now; yaw = correction = 0;
    return true;
  }

  bool update(float raw_dps, uint32_t now) {
    const uint32_t dt = now - last_us; // handles micros() rollover
    if (!dt || dt > 100000 || !std::isfinite(raw_dps)) return false;
    const float rate = sign * (raw_dps - bias);
    if (std::fabs(rate) > 150) return false;
    yaw += rate * (dt * 1e-6F);
    if (!std::isfinite(yaw) || std::fabs(yaw) > 20) return false;
    last_us = now;
    const float requested = kp * yaw + kd * rate;
    correction = requested > limit ? limit : (requested < -limit ? -limit : requested);
    // Positive yaw = left turn: accelerate left, slow right to turn back right.
    left = base_left + correction;
    right = base_right - correction;
    return true;
  }
};
