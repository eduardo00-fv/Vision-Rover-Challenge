#include "vision_navigator.h"

#include "hardware_config.h"

#include <cstring>

// Un config.h anterior con el contacto a 80 mm deja el cubo fuera de la zona.
static_assert(NAV_PUSH_CONTACT_OFFSET_MM >= 95.0F,
              "NAV_PUSH_CONTACT_OFFSET_MM: centro rover -> centro cubo en contacto (~107 mm)");
static_assert(NAV_APPROACH_STANDOFF_MM >= NAV_PUSH_CONTACT_OFFSET_MM + 25.0F,
              "NAV_APPROACH_STANDOFF_MM debe dejar holgura para girar antes del contacto");

namespace {
constexpr uint32_t INACTIVE_AFTER_MS = 20000;
constexpr uint32_t MOVING_WITHIN_MS = 3000;
constexpr float MOVED_CELLS = 1.25F;
constexpr uint32_t MAX_ASSIGN_CUBE_AGE_MS = 10000;  // oculto no es desaparecido
constexpr uint32_t MAX_OBSTACLE_AGE_MS = 1500;
constexpr uint32_t HINT_FRESH_MS = 500;

rover_logic::NavigationConfig navigationConfig() {
  rover_logic::NavigationConfig config;
  config.max_cube_age_ms = MAX_CUBE_AGE_MS;
  config.max_push_cube_age_ms = MAX_PUSH_CUBE_AGE_MS;
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
  config.arena_margin_mm = NAV_ARENA_MARGIN_MM;
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

rover_logic::Point toCore(const PointCells& point) {
  return {point.col, point.row};
}

rover_logic::Color colorId(const String& color) {
  if (color == "red") return rover_logic::Color::RED;
  if (color == "green") return rover_logic::Color::GREEN;
  if (color == "blue") return rover_logic::Color::BLUE;
  return rover_logic::Color::NONE;
}
const char* colorName(rover_logic::Color color) {
  switch (color) {
    case rover_logic::Color::RED: return "red";
    case rover_logic::Color::GREEN: return "green";
    case rover_logic::Color::BLUE: return "blue";
    default: return "";
  }
}
uint32_t effectiveAge(uint32_t age, uint32_t elapsed) {
  const uint64_t total = static_cast<uint64_t>(age) + elapsed;
  return total > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(total);
}

DriveCommand fromCore(const rover_logic::DriveCommand& core, uint8_t target) {
  return {core.left, core.right, core.active, core.pushing, rover_logic::mode_name(core.mode),
          core.heading_error_deg, target, core.reverse, core.distance_mm};
}
}  // namespace

void VisionNavigator::trackLiveness(const WorldState& world, uint32_t now) {
  for (uint8_t i = 0; i < world.rover_count; ++i) {
    const RoverObservation& rover = world.rovers[i];
    Liveness* slot = nullptr;
    for (Liveness& entry : liveness_) if (entry.id == rover.id) slot = &entry;
    for (Liveness& entry : liveness_) if (slot == nullptr && entry.id == 0) slot = &entry;
    if (slot == nullptr) continue;
    const float dc = rover.col - slot->col, dr = rover.row - slot->row;
    if (slot->id != rover.id || world.phase != RoundPhase::RUNNING ||
        dc * dc + dr * dr > MOVED_CELLS * MOVED_CELLS) {
      *slot = {rover.id, rover.col, rover.row, now};
    }
  }
}

bool VisionNavigator::moving(uint8_t id, uint32_t now) const {
  for (const Liveness& entry : liveness_)
    if (entry.id == id) return now - entry.moved_ms < MOVING_WITHIN_MS;
  return false;
}

bool VisionNavigator::inactive(uint8_t id, uint32_t now) const {
  for (const Liveness& entry : liveness_)
    if (entry.id == id) return now - entry.moved_ms > INACTIVE_AFTER_MS;
  return false;
}

void VisionNavigator::peerHint(uint8_t id, bool can_move, uint8_t claimed, bool pushing, uint32_t now) {
  Hint* slot = nullptr;
  for (Hint& h : hints_) if (h.id == id) slot = &h;
  for (Hint& h : hints_) if (slot == nullptr && h.id == 0) slot = &h;
  if (slot != nullptr) *slot = {id, can_move, claimed, pushing, now};
}

const VisionNavigator::Hint* VisionNavigator::hint(uint8_t id, uint32_t now) const {
  for (const Hint& h : hints_)
    if (h.id == id && h.id != 0) return now - h.at_ms <= HINT_FRESH_MS ? &h : nullptr;
  return nullptr;
}

uint8_t VisionNavigator::stage() const {
  if (!decided_) return 0;
  if (strcmp(decision_.mode, "PARK") == 0) return 5;
  if (decision_.target == 0) return 0;
  switch (push_.stage) {
    case rover_logic::PushState::ALIGN: return 2;
    case rover_logic::PushState::PUSH: return 3;
    case rover_logic::PushState::RETREAT: return 4;
    default: return 1;
  }
}

DriveCommand VisionNavigator::update(const WorldState& world, uint8_t rover_id) {
  const uint32_t now = millis();
  if (world.valid) trackLiveness(world, now);
  if (!world.valid || world.phase != RoundPhase::RUNNING) { push_ = {}; decided_ = false; return {}; }
  const RoverObservation* self = findRover(world, rover_id);
  const uint32_t elapsed = now - world.received_at_ms;
  if (!self || effectiveAge(self->age_ms, elapsed) > MAX_SELF_POSE_AGE_MS) {
    push_ = {}; decided_ = false; return {};
  }
  if (world.rover_count > MAX_ROVERS || world.cube_count > MAX_CUBES || world.depot_count > MAX_CUBES) {
    push_ = {}; return {};
  }
  if (!decided_ || world.seq != decided_seq_ || world.captured_at_ms != decided_capture_ ||
      rover_id != decided_id_) {
    decision_ = decide(world, rover_id, now);
    decided_seq_ = world.seq;
    decided_capture_ = world.captured_at_ms;
    decided_id_ = rover_id;
    decided_ = true;
  }
  return decision_;
}

DriveCommand VisionNavigator::decide(const WorldState& world, uint8_t rover_id, uint32_t now) {
  const RoverObservation* self = findRover(world, rover_id);
  const uint32_t elapsed = now - world.received_at_ms;
  rover_logic::TaskRover rovers[MAX_ROVERS];
  rover_logic::TaskCube cubes[MAX_CUBES];
  rover_logic::TaskDepot depots[MAX_CUBES];
  rover_logic::Color previous[MAX_ROVERS] = {};
  rover_logic::Color assigned[MAX_ROVERS] = {};
  for (uint8_t i = 0; i < world.rover_count; ++i) {
    const RoverObservation& rover = world.rovers[i];
    const Hint* peer = rover.id != rover_id ? hint(rover.id, now) : nullptr;
    const bool excluded = rover.id != rover_id && (inactive(rover.id, now) || (peer && !peer->can_move));
    rovers[i] = {rover.id, toCore(rover), excluded ? UINT32_MAX : effectiveAge(rover.age_ms, elapsed)};
    for (uint8_t k = 0; k < MAX_ROVERS; ++k) if (previous_ids_[k] == rover.id) previous[i] = previous_[k];
    // El cubo que el compañero dice estar empujando sigue siendo suyo.
    if (peer && peer->can_move && peer->pushing && peer->claimed != 0)
      previous[i] = static_cast<rover_logic::Color>(peer->claimed);
  }
  for (uint8_t i = 0; i < world.cube_count; ++i)
    cubes[i] = {colorId(world.cubes[i].color),
                {toCore(world.cubes[i]), effectiveAge(world.cubes[i].age_ms, elapsed)}};
  for (uint8_t i = 0; i < world.depot_count; ++i)
    depots[i] = {colorId(world.depots[i].color), toCore(world.depots[i])};

  rover_logic::AssignConfig assign;
  assign.max_rover_age_ms = MAX_SELF_POSE_AGE_MS;
  assign.max_cube_age_ms = MAX_ASSIGN_CUBE_AGE_MS;
  rover_logic::plan_assignment(toCore(world.grid), toCore(world.depot_size), world.cube_side_cells,
      rovers, world.rover_count, cubes, world.cube_count, depots, world.depot_count,
      previous, assign, assigned);
  for (uint8_t i = 0; i < MAX_ROVERS; ++i) {
    previous_ids_[i] = i < world.rover_count ? world.rovers[i].id : 0;
    previous_[i] = i < world.rover_count ? assigned[i] : rover_logic::Color::NONE;
  }

  rover_logic::Color target = rover_logic::Color::NONE;
  for (uint8_t i = 0; i < world.rover_count; ++i) if (rovers[i].id == rover_id) target = assigned[i];

  const rover_logic::NavigationConfig config = navigationConfig();
  // Obstáculos: los cubos que no son el objetivo y el otro rover si se lo ve.
  rover_logic::Obstacle obstacles[MAX_CUBES + MAX_ROVERS];
  uint8_t count = 0;
  for (uint8_t i = 0; i < world.cube_count; ++i)
    if (cubes[i].color != target) obstacles[count++] = {cubes[i].cube.position, config.cube_clearance_mm, false};
  rover_logic::Point cube_points[MAX_CUBES];
  for (uint8_t i = 0; i < world.cube_count; ++i) cube_points[i] = cubes[i].cube.position;
  for (uint8_t i = 0; i < world.rover_count; ++i) {
    if (world.rovers[i].id == rover_id || effectiveAge(world.rovers[i].age_ms, elapsed) > MAX_OBSTACLE_AGE_MS)
      continue;
    const RoverObservation& other = world.rovers[i];
    const bool other_pushing = rover_logic::rover_pushing(toCore(other), cube_points, world.cube_count,
                                                          world.grid.cell_mm, config.push_contact_offset_mm);
    const bool priority = other_pushing != (push_.stage == rover_logic::PushState::PUSH) ? other_pushing : other.id < rover_id;
    const Hint* peer = hint(other.id, now);
    const bool other_moving = moving(other.id, now) && !(peer && !peer->can_move);
    // El despeje amplio es para quien tiene paso y se mueve; uno detenido
    // (averiado, estacionado) solo pide el mínimo físico.
    obstacles[count++] = {{other.col, other.row},
                          priority && other_moving ? config.rover_clearance_mm : config.rover_clearance_low_mm,
                          true, other_pushing, priority, other_moving, other.theta_deg};
  }

  const CubeObservation* cube = findCube(world, colorName(target));
  const Depot* depot = findDepot(world, colorName(target));
  if (cube != nullptr && depot != nullptr) {
    if (last_target_ != cube->color) {
      last_target_ = cube->color;
      push_ = {};
    }
    const rover_logic::Cube core_cube = {toCore(*cube), effectiveAge(cube->age_ms, elapsed)};
    const rover_logic::DriveCommand core = rover_logic::navigate(
        toCore(world.grid), toCore(world.depot_size), world.cube_side_cells, config,
        toCore(*self), core_cube, toCore(*depot), obstacles, count, &push_);
    if (core.mode != rover_logic::Mode::UNREACHABLE) return fromCore(core, static_cast<uint8_t>(target));
  }

  // Sin tarea (o sin forma de empujarla): despejar hacia una esquina libre.
  push_ = {};
  last_target_ = "";
  rover_logic::Point keep_clear[2 * MAX_CUBES];
  uint8_t keep = 0;
  for (uint8_t i = 0; i < world.cube_count; ++i) keep_clear[keep++] = toCore(world.cubes[i]);
  for (uint8_t i = 0; i < world.depot_count; ++i) keep_clear[keep++] = toCore(world.depots[i]);
  const rover_logic::Point park = rover_logic::parking_spot(toCore(world.grid), keep_clear, keep);
  return fromCore(rover_logic::go_to(toCore(world.grid), config, toCore(*self), park, obstacles, count,
                                     rover_logic::Mode::PARK), 0);
}
