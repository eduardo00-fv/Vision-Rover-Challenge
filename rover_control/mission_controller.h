#pragma once

#include <Arduino.h>

#include "safety_supervisor.h"

enum class MissionState : uint8_t { WAIT_VISION, READY, RUNNING, FINISHED, SAFE_STOP };

// Traduce la foto oficial del mundo a un estado de misión. No controla PWM ni
// conoce sensores: eso evita que la lógica de estrategia pueda esquivar la
// parada de seguridad.
class MissionController {
 public:
  MissionState update(const WorldState& world, const SafetyDecision& safety);
  MissionState state() const { return state_; }
  const char* stateName() const;

 private:
  MissionState state_ = MissionState::WAIT_VISION;
};
