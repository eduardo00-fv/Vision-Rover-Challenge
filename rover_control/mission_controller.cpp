#include "mission_controller.h"

MissionState MissionController::update(const WorldState& world, const SafetyDecision& safety) {
  if (safety.reason != SafetyReason::OK) {
    state_ = MissionState::SAFE_STOP;
    return state_;
  }

  switch (world.phase) {
    case RoundPhase::IDLE: state_ = MissionState::WAIT_VISION; break;
    case RoundPhase::READY: state_ = MissionState::READY; break;
    case RoundPhase::RUNNING: state_ = MissionState::RUNNING; break;
    case RoundPhase::FINISHED: state_ = MissionState::FINISHED; break;
    default: state_ = MissionState::SAFE_STOP; break;
  }
  return state_;
}

const char* MissionController::stateName() const {
  switch (state_) {
    case MissionState::WAIT_VISION: return "WAIT_VISION";
    case MissionState::READY: return "READY";
    case MissionState::RUNNING: return "RUNNING";
    case MissionState::FINISHED: return "FINISHED";
    case MissionState::SAFE_STOP: return "SAFE_STOP";
  }
  return "UNKNOWN";
}
