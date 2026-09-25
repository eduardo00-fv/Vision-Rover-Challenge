#pragma once
#include <cmath>
#include <cstdint>

// Experimental relative-angle controller. Never reverses or retries after cutoff.
struct TurnControl {
  static constexpr uint32_t timeoutMs(float degrees) {
    return degrees > 90 || degrees < -90 ? 7000U : 3500U;
  }
  enum class Action { RUN, CUT, FAULT };
  float start_power = 25, end_power = 22;
  float coast_left_s = 0.04F, coast_right_s = 0.05F;
  float target = 0, power = 25, progress_mark = 0;
  uint32_t progress_ms = 0;
  bool begin(float degrees, uint32_t now) {
    if (!std::isfinite(degrees) || std::fabs(degrees) < 10 || std::fabs(degrees) > 180 ||
        !std::isfinite(start_power) || !std::isfinite(end_power) ||
        !std::isfinite(coast_left_s) || !std::isfinite(coast_right_s) ||
        end_power < 10 || start_power < end_power || start_power > 35 ||
        coast_left_s < 0 || coast_left_s > 0.1F ||
        coast_right_s < 0 || coast_right_s > 0.1F) return false;
    target = degrees; power = start_power; progress_mark = 0; progress_ms = now;
    return true;
  }
  Action update(float yaw, float rate, uint32_t now) {
    if (!std::isfinite(yaw) || !std::isfinite(rate)) return Action::FAULT;
    const float direction = target > 0 ? 1.0F : -1.0F;
    const float progress = direction * yaw;
    if (progress < -5 || std::fabs(rate) > 200) return Action::FAULT;
    const float remaining = std::fabs(target) - progress;
    // Provisional coast estimate from timed tests, not a calibrated plant model.
    const float forward_rate = direction * rate > 0 ? direction * rate : 0;
    const float predicted = forward_rate * (target > 0 ? coast_left_s : coast_right_s);
    const float lead = predicted > 6 ? 6 : predicted;
    if (remaining <= 1.0F + lead) return Action::CUT;
    if (progress >= progress_mark + 1) { progress_mark = progress; progress_ms = now; }
    if (now - progress_ms > 750) return Action::FAULT;
    // 25% breaks static friction; taper toward 22% only near the target.
    power = remaining >= 15 ? start_power : end_power + (start_power - end_power) * remaining / 15;
    return Action::RUN;
  }
};
