#include "vision_navigator.h"

#include "hardware_config.h"
#include "rover_logic_core.h"

namespace {
rover_logic::NavigationConfig navigationConfig() {
  rover_logic::NavigationConfig config;
  config.max_cube_age_ms = MAX_CUBE_AGE_MS;
  config.drive_speed = NAV_DRIVE_SPEED;
  config.push_speed = NAV_PUSH_SPEED;
  config.turn_speed = NAV_TURN_SPEED;
  config.steer_gain = NAV_STEER_GAIN;
  config.turn_in_place_deg = NAV_TURN_IN_PLACE_DEG;
  config.goal_tolerance_mm = NAV_GOAL_TOLERANCE_MM;
  config.slowdown_mm = NAV_SLOWDOWN_MM;
  config.approach_standoff_mm = NAV_APPROACH_STANDOFF_MM;
  config.approach_tolerance_mm = NAV_APPROACH_TOLERANCE_MM;
  config.push_contact_max_mm = NAV_PUSH_CONTACT_MAX_MM;
  config.push_contact_offset_mm = NAV_PUSH_CONTACT_OFFSET_MM;
  return config;
}

rover_logic::Grid toCore(const Grid& grid) {
  return {grid.cols, grid.rows, grid.cell_mm};
}

rover_logic::DepotSize toCore(const DepotSize& depot_size) {
  return {depot_size.length_cells, depot_size.depth_cells};
}

rover_logic::Pose toCore(const RoverObservation& rover) {
  return {rover.col, rover.row, rover.theta_deg};
}

rover_logic::Cube toCore(const CubeObservation& cube) {
  return {{cube.col, cube.row}, cube.age_ms};
}

rover_logic::Point toCore(const Depot& depot) {
  return {depot.col, depot.row};
}
}  // namespace

DriveCommand VisionNavigator::update(const WorldState& world, uint8_t rover_id) {
  const RoverObservation* self = findRover(world, rover_id);
  const CubeObservation* cube = findCube(world, ROVER_TARGET_COLOR);
  const Depot* depot = findDepot(world, ROVER_TARGET_COLOR);
  if (self == nullptr || cube == nullptr || depot == nullptr) {
    pushing_ = false;
    return {};
  }

  if (last_target_ != cube->color) {
    last_target_ = cube->color;
    pushing_ = false;
  }

  const rover_logic::DriveCommand core = rover_logic::navigate(
      toCore(world.grid), toCore(world.depot_size), world.cube_side_cells,
      navigationConfig(), toCore(*self), toCore(*cube), toCore(*depot), &pushing_);
  return {core.left, core.right, core.active, core.pushing, rover_logic::mode_name(core.mode)};
}
