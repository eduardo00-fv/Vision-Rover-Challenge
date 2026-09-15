#pragma once

// Núcleo portable: no depende de Arduino, Wi-Fi, Webots ni JSON. El firmware y
// el controlador de simulación deben adaptarle sus propios datos de entrada.

#include <cstdint>

namespace rover_logic {

struct Pose {
  float col = 0.0F;
  float row = 0.0F;
  float theta_deg = 0.0F;
};

struct Point {
  float col = 0.0F;
  float row = 0.0F;
};

struct Grid {
  uint16_t cols = 0;
  uint16_t rows = 0;
  float cell_mm = 0.0F;
};

struct DepotSize {
  float length_cells = 0.0F;
  float depth_cells = 0.0F;
};

struct Cube {
  Point position;
  uint32_t age_ms = 0;
};

enum class Color : uint8_t { NONE = 0, GREEN = 1, BLUE = 2, RED = 3 };
struct TaskRover { uint8_t id = 0; Pose pose; uint32_t age_ms = 0; };
struct TaskCube { Color color = Color::NONE; Cube cube; };
struct TaskDepot { Color color = Color::NONE; Point position; };

enum class Mode : uint8_t {
  WAIT_TARGET,
  CUBE_DELIVERED,
  APPROACH_ALIGN,
  APPROACH,
  AT_APPROACH,
  PUSH_ALIGN,
  PUSH,
  PUSH_DONE,
};

struct DriveCommand {
  float left = 0.0F;
  float right = 0.0F;
  bool active = false;
  bool pushing = false;
  Mode mode = Mode::WAIT_TARGET;
};

struct NavigationConfig {
  uint32_t max_cube_age_ms = 1200;
  float drive_speed = 0.34F;
  float push_speed = 0.26F;
  float turn_speed = 0.28F;
  float steer_gain = 0.16F;
  float turn_in_place_deg = 20.0F;
  float goal_tolerance_mm = 35.0F;
  float slowdown_mm = 140.0F;
  float approach_standoff_mm = 115.0F;
  float approach_tolerance_mm = 45.0F;
  float push_contact_max_mm = 190.0F;
  float push_contact_offset_mm = 80.0F;
};

bool cube_delivered(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                    const Point& cube, const Point& depot);
DriveCommand navigate(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                      const NavigationConfig& config, const Pose& self, const Cube& cube,
                      const Point& depot, bool* pushing);
const char* mode_name(Mode mode);
void assign_tasks(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                  const TaskRover* rovers, uint8_t rover_count, const TaskCube* cubes,
                  uint8_t cube_count, const TaskDepot* depots, uint8_t depot_count,
                  uint32_t max_rover_age_ms, uint32_t max_cube_age_ms, Color* assigned);
bool should_yield(const Pose& self, uint8_t self_id, const Pose& other, uint8_t other_id,
                  uint32_t other_age_ms, float cell_mm, float separation_mm,
                  uint32_t max_pose_age_ms = 300);

}  // namespace rover_logic
