#pragma once

#include <Arduino.h>

#include "world_state.h"

// Es la única autoridad que habilita movimiento. Los controladores de misión,
// navegación y motores deben consultar este módulo antes de aplicar PWM.
enum class SafetyReason : uint8_t {
  OK,
  NO_TELEMETRY,
  ROVER_NOT_VISIBLE,
  ROUND_FINISHED,
  UNKNOWN_PHASE,
};

struct SafetyDecision {
  bool motion_allowed = false;
  SafetyReason reason = SafetyReason::NO_TELEMETRY;
};

class SafetySupervisor {
 public:
  explicit SafetySupervisor(uint32_t max_world_age_ms) : max_world_age_ms_(max_world_age_ms) {}

  SafetyDecision evaluate(const WorldState& world, uint8_t rover_id) const;
  const char* reasonName(SafetyReason reason) const;

 private:
  uint32_t max_world_age_ms_;
};
