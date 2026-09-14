#pragma once

#include <Arduino.h>

constexpr uint8_t MAX_ROVERS = 2;
constexpr uint8_t MAX_CUBES = 3;
constexpr uint8_t MAX_OBSTACLES = 8;

enum class RoundPhase : uint8_t { UNKNOWN, IDLE, READY, RUNNING, FINISHED };

struct PointCells {
  float col = 0.0F;
  float row = 0.0F;
};

struct RoverObservation : PointCells {
  uint8_t id = 0;
  float theta_deg = 0.0F;
  uint32_t age_ms = 0;
};

struct CubeObservation : PointCells {
  String color;
  uint32_t age_ms = 0;
};

struct ObstacleObservation : PointCells {
  uint32_t age_ms = 0;
};

struct Grid {
  uint16_t cols = 0;
  uint16_t rows = 0;
  float cell_mm = 0.0F;
};

struct RoundClock {
  uint32_t elapsed_ms = 0;
  uint32_t remaining_ms = 0;
  uint32_t total_ms = 0;
};

struct Depot : PointCells {
  String color;
};

struct DepotSize {
  float length_cells = 0.0F;
  float depth_cells = 0.0F;
};

// Solo contiene una foto completa de la telemetría oficial. El estado de misión
// (FSM, reservas y rutas) vive en módulos separados y no altera esta estructura.
struct WorldState {
  bool valid = false;
  uint8_t protocol_version = 0;
  uint32_t seq = 0;
  uint64_t captured_at_ms = 0;
  uint32_t received_at_ms = 0;
  uint32_t transport_age_ms = 0;
  uint32_t sequence_gap = 0;
  RoundPhase phase = RoundPhase::UNKNOWN;
  RoundClock clock;
  Grid grid;
  PointCells start;
  DepotSize depot_size;
  float cube_side_cells = 0.0F;

  RoverObservation rovers[MAX_ROVERS];
  uint8_t rover_count = 0;
  CubeObservation cubes[MAX_CUBES];
  uint8_t cube_count = 0;
  ObstacleObservation obstacles[MAX_OBSTACLES];
  uint8_t obstacle_count = 0;
  Depot depots[MAX_CUBES];
  uint8_t depot_count = 0;
};

RoundPhase parsePhase(const char* value);
const char* phaseName(RoundPhase phase);
const RoverObservation* findRover(const WorldState& world, uint8_t id);
const CubeObservation* findCube(const WorldState& world, const char* color);
const Depot* findDepot(const WorldState& world, const char* color);
bool isFresh(uint32_t age_ms, uint32_t max_age_ms);

