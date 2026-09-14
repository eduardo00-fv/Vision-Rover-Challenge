#include "safety_supervisor.h"

SafetyDecision SafetySupervisor::evaluate(const WorldState& world, uint8_t rover_id) const {
  if (!world.valid || millis() - world.received_at_ms > max_world_age_ms_) {
    return {false, SafetyReason::NO_TELEMETRY};
  }
  if (findRover(world, rover_id) == nullptr) {
    return {false, SafetyReason::ROVER_NOT_VISIBLE};
  }
  if (world.phase == RoundPhase::FINISHED) return {false, SafetyReason::ROUND_FINISHED};
  if (world.phase == RoundPhase::UNKNOWN) return {false, SafetyReason::UNKNOWN_PHASE};
  return {world.phase == RoundPhase::RUNNING, SafetyReason::OK};
}

const char* SafetySupervisor::reasonName(SafetyReason reason) const {
  switch (reason) {
    case SafetyReason::OK: return "OK";
    case SafetyReason::NO_TELEMETRY: return "NO_TELEMETRY";
    case SafetyReason::ROVER_NOT_VISIBLE: return "ROVER_NOT_VISIBLE";
    case SafetyReason::ROUND_FINISHED: return "ROUND_FINISHED";
    case SafetyReason::UNKNOWN_PHASE: return "UNKNOWN_PHASE";
  }
  return "UNKNOWN";
}
