#include "rover_logic_core.h"

#include <cmath>

namespace rover_logic {
namespace {
constexpr float kPi = 3.14159265358979323846F;

float clamp(float value) { return value < -1.0F ? -1.0F : (value > 1.0F ? 1.0F : value); }
float distance(const Point& first, const Point& second) {
  const float dc = second.col - first.col;
  const float dr = second.row - first.row;
  return std::sqrt(dc * dc + dr * dr);
}
float wrap(float degrees) {
  while (degrees > 180.0F) degrees -= 360.0F;
  while (degrees <= -180.0F) degrees += 360.0F;
  return degrees;
}
DriveCommand drive_to(const Pose& self, const Point& target, float cell_mm,
                      const NavigationConfig& config, bool pushing) {
  DriveCommand command;
  command.pushing = pushing;
  const float dc = target.col - self.col;
  const float dr = target.row - self.row;
  const float distance_mm = std::sqrt(dc * dc + dr * dr) * cell_mm;
  if (distance_mm < config.goal_tolerance_mm) {
    command.mode = pushing ? Mode::PUSH_DONE : Mode::AT_APPROACH;
    return command;
  }
  const float desired = std::atan2(-dr, dc) * 180.0F / kPi;
  const float error = wrap(desired - self.theta_deg);
  command.active = true;
  if (std::fabs(error) > config.turn_in_place_deg) {
    const float turn = error > 0.0F ? config.turn_speed : -config.turn_speed;
    command.left = -turn;
    command.right = turn;
    command.mode = pushing ? Mode::PUSH_ALIGN : Mode::APPROACH_ALIGN;
    return command;
  }
  float speed = pushing ? config.push_speed : config.drive_speed;
  if (distance_mm < config.slowdown_mm) speed *= 0.55F;
  const float correction = clamp(error / config.turn_in_place_deg) * config.steer_gain;
  command.left = clamp(speed - correction);
  command.right = clamp(speed + correction);
  command.mode = pushing ? Mode::PUSH : Mode::APPROACH;
  return command;
}
}  // namespace

bool cube_delivered(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                    const Point& cube, const Point& depot) {
  if (grid.cols == 0 || grid.rows == 0 || depot_size.length_cells <= 0.0F ||
      depot_size.depth_cells <= 0.0F || cube_side_cells <= 0.0F) return false;
  const float nearest = std::fmin(std::fmin(depot.col, static_cast<float>(grid.cols) - depot.col),
                                  std::fmin(depot.row, static_cast<float>(grid.rows) - depot.row));
  const bool horizontal = nearest == depot.row || nearest == static_cast<float>(grid.rows) - depot.row;
  const float semi_col = (horizontal ? depot_size.length_cells : depot_size.depth_cells) / 2.0F;
  const float semi_row = (horizontal ? depot_size.depth_cells : depot_size.length_cells) / 2.0F;
  const float margin = cube_side_cells * std::sqrt(2.0F) / 2.0F;
  return std::fabs(cube.col - depot.col) <= semi_col - margin &&
         std::fabs(cube.row - depot.row) <= semi_row - margin;
}

DriveCommand navigate(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                      const NavigationConfig& config, const Pose& self, const Cube& cube,
                      const Point& depot, bool* pushing) {
  if (pushing == nullptr || grid.cell_mm <= 0.0F) return {};
  if (cube.age_ms > config.max_cube_age_ms) {
    *pushing = false;
    return {};
  }
  if (cube_delivered(grid, depot_size, cube_side_cells, cube.position, depot)) {
    *pushing = false;
    DriveCommand done;
    done.mode = Mode::CUBE_DELIVERED;
    return done;
  }
  const float depot_distance = distance(cube.position, depot);
  if (depot_distance < 0.01F) return {};
  const float unit_col = (depot.col - cube.position.col) / depot_distance;
  const float unit_row = (depot.row - cube.position.row) / depot_distance;
  const Point approach = {cube.position.col - unit_col * config.approach_standoff_mm / grid.cell_mm,
                          cube.position.row - unit_row * config.approach_standoff_mm / grid.cell_mm};
  const float rover_cube_mm = distance({self.col, self.row}, cube.position) * grid.cell_mm;
  if (*pushing && rover_cube_mm > config.push_contact_max_mm) *pushing = false;
  if (!*pushing && distance({self.col, self.row}, approach) * grid.cell_mm < config.approach_tolerance_mm) {
    *pushing = true;
  }
  const Point push_goal = {depot.col - unit_col * config.push_contact_offset_mm / grid.cell_mm,
                           depot.row - unit_row * config.push_contact_offset_mm / grid.cell_mm};
  return *pushing ? drive_to(self, push_goal, grid.cell_mm, config, true)
                  : drive_to(self, approach, grid.cell_mm, config, false);
}

const char* mode_name(Mode mode) {
  switch (mode) {
    case Mode::CUBE_DELIVERED: return "CUBE_DELIVERED";
    case Mode::APPROACH_ALIGN: return "APPROACH_ALIGN";
    case Mode::APPROACH: return "APPROACH";
    case Mode::AT_APPROACH: return "AT_APPROACH";
    case Mode::PUSH_ALIGN: return "PUSH_ALIGN";
    case Mode::PUSH: return "PUSH";
    case Mode::PUSH_DONE: return "PUSH_DONE";
    default: return "WAIT_TARGET";
  }
}

void assign_tasks(const Grid& grid, const DepotSize& depot_size, float cube_side_cells,
                  const TaskRover* rovers, uint8_t rover_count, const TaskCube* cubes,
                  uint8_t cube_count, const TaskDepot* depots, uint8_t depot_count,
                  uint32_t max_rover_age_ms, uint32_t max_cube_age_ms, Color* assigned) {
  if (assigned == nullptr) return;
  for (uint8_t r = 0; r < rover_count; ++r) assigned[r] = Color::NONE;
  bool used[3] = {false, false, false};
  for (;;) {
    int best_r = -1, best_c = -1;
    float best_cost = 0.0F;
    for (uint8_t r = 0; r < rover_count; ++r) {
      if (assigned[r] != Color::NONE || rovers[r].age_ms > max_rover_age_ms) continue;
      for (uint8_t c = 0; c < cube_count; ++c) {
        if (used[c] || cubes[c].color == Color::NONE || cubes[c].cube.age_ms > max_cube_age_ms) continue;
        const TaskDepot* depot = nullptr;
        for (uint8_t d = 0; d < depot_count; ++d) if (depots[d].color == cubes[c].color) depot = &depots[d];
        if (depot == nullptr || cube_delivered(grid, depot_size, cube_side_cells, cubes[c].cube.position, depot->position)) continue;
        const float cost = distance({rovers[r].pose.col, rovers[r].pose.row}, cubes[c].cube.position) +
                           distance(cubes[c].cube.position, depot->position);
        if (best_r < 0 || cost < best_cost || (cost == best_cost && rovers[r].id < rovers[best_r].id)) {
          best_r = r; best_c = c; best_cost = cost;
        }
      }
    }
    if (best_r < 0) return;
    assigned[best_r] = cubes[best_c].color;
    used[best_c] = true;
  }
}

bool should_yield(const Pose& self, uint8_t self_id, const Pose& other, uint8_t other_id,
                  uint32_t other_age_ms, float cell_mm, float separation_mm,
                  uint32_t max_pose_age_ms) {
  if (other_age_ms > max_pose_age_ms || cell_mm <= 0.0F || self_id <= other_id) return false;
  return distance({self.col, self.row}, {other.col, other.row}) * cell_mm < separation_mm;
}
}  // namespace rover_logic
