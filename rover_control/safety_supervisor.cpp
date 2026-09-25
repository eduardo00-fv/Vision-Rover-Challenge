#include "safety_supervisor.h"

SafetyDecision SafetySupervisor::evaluate(const WorldState& world, uint8_t rover_id,
                                          uint32_t max_pose_age_ms) const {
  if (!world.valid || millis() - world.received_at_ms > max_world_age_ms_) {
    return {false, SafetyReason::NO_TELEMETRY};
  }
  const RoverObservation* rover = findRover(world, rover_id);
  if (rover == nullptr) {
    return {false, SafetyReason::ROVER_NOT_VISIBLE};
  }
  if (static_cast<uint64_t>(rover->age_ms) + (millis() - world.received_at_ms) > max_pose_age_ms)
    return {false, SafetyReason::ROVER_POSE_STALE};
  if (world.phase == RoundPhase::FINISHED) return {false, SafetyReason::ROUND_FINISHED};
  if (world.phase == RoundPhase::UNKNOWN) return {false, SafetyReason::UNKNOWN_PHASE};
  return {world.phase == RoundPhase::RUNNING, SafetyReason::OK};
}

const char* SafetySupervisor::reasonName(SafetyReason reason) const {
  switch (reason) {
    case SafetyReason::OK: return "OK";
    case SafetyReason::NO_TELEMETRY: return "NO_TELEMETRY";
    case SafetyReason::ROVER_NOT_VISIBLE: return "ROVER_NOT_VISIBLE";
    case SafetyReason::ROVER_POSE_STALE: return "ROVER_POSE_STALE";
    case SafetyReason::ROUND_FINISHED: return "ROUND_FINISHED";
    case SafetyReason::UNKNOWN_PHASE: return "UNKNOWN_PHASE";
  }
  return "UNKNOWN";
}
