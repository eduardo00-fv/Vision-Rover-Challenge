#pragma once
#include "heading_control.h"
#include "turn_control.h"

namespace rover_motion {
// Physical rover number, NOT the visual marker ID. Bias is measured again.
struct Profile {
  unsigned rover;
  bool initially_validated;
  int gyro_sign;
  float left, right, kp, kd, correction_limit;
  float turn_start, turn_end, coast_left_s, coast_right_s;
  void configure(HeadingControl& heading, TurnControl& turn) const {
    heading.kp = kp; heading.kd = kd; heading.limit = correction_limit;
    turn.start_power = turn_start; turn.end_power = turn_end;
    turn.coast_left_s = coast_left_s; turn.coast_right_s = coast_right_s;
  }
};
inline constexpr Profile rover1{1, true, 1, 18, 20, 0.8F, 0.08F, 4, 25, 22, 0.04F, 0.05F};
// Candidates only; wheel map and gyro sign still require physical confirmation.
inline constexpr Profile rover2{2, false, 0, 20, 20, 0.8F, 0.08F, 4, 25, 22, 0.04F, 0.05F};
} // namespace rover_motion
