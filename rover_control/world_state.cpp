#include "world_state.h"

#include <string.h>

RoundPhase parsePhase(const char* value) {
  if (value == nullptr) return RoundPhase::UNKNOWN;
  if (strcmp(value, "IDLE") == 0) return RoundPhase::IDLE;
  if (strcmp(value, "READY") == 0) return RoundPhase::READY;
  if (strcmp(value, "RUNNING") == 0) return RoundPhase::RUNNING;
  if (strcmp(value, "FINISHED") == 0) return RoundPhase::FINISHED;
  return RoundPhase::UNKNOWN;
}

const char* phaseName(RoundPhase phase) {
  switch (phase) {
    case RoundPhase::IDLE: return "IDLE";
    case RoundPhase::READY: return "READY";
    case RoundPhase::RUNNING: return "RUNNING";
    case RoundPhase::FINISHED: return "FINISHED";
    default: return "UNKNOWN";
  }
}

const RoverObservation* findRover(const WorldState& world, uint8_t id) {
  for (uint8_t i = 0; i < world.rover_count; ++i) {
    if (world.rovers[i].id == id) return &world.rovers[i];
  }
  return nullptr;
}

const CubeObservation* findCube(const WorldState& world, const char* color) {
  for (uint8_t i = 0; i < world.cube_count; ++i) {
    if (world.cubes[i].color == color) return &world.cubes[i];
  }
  return nullptr;
}

const Depot* findDepot(const WorldState& world, const char* color) {
  for (uint8_t i = 0; i < world.depot_count; ++i) {
    if (world.depots[i].color == color) return &world.depots[i];
  }
  return nullptr;
}

bool isFresh(uint32_t age_ms, uint32_t max_age_ms) {
  return age_ms <= max_age_ms;
}

