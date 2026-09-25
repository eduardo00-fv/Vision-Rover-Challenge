#include "rover_logic_core.h"

// ABI pequeño para que un controlador host (Webots/Python) pueda usar el mismo
// núcleo que el firmware sin conocer C++ ni estructuras de Arduino.
extern "C" {
struct VrcGrid { unsigned short cols; unsigned short rows; float cell_mm; };
struct VrcPoint { float col; float row; };
struct VrcPose { float col; float row; float theta_deg; };
struct VrcCube { VrcPoint position; unsigned int age_ms; };
struct VrcDepotSize { float length_cells; float depth_cells; };
struct VrcDriveCommand { float left; float right; unsigned char active; unsigned char pushing; unsigned char mode; };
struct VrcTaskRover { unsigned char id; VrcPose pose; unsigned int age_ms; };
struct VrcTaskCube { unsigned char color; VrcCube cube; };
struct VrcTaskDepot { unsigned char color; VrcPoint position; };

VrcDriveCommand vrc_navigate(VrcGrid grid, VrcDepotSize depot_size, float cube_side_cells,
                             VrcPose self, VrcCube cube, VrcPoint depot, unsigned char* pushing) {
  bool core_pushing = pushing != nullptr && *pushing != 0;
  const rover_logic::DriveCommand command = rover_logic::navigate(
      {grid.cols, grid.rows, grid.cell_mm}, {depot_size.length_cells, depot_size.depth_cells},
      cube_side_cells, rover_logic::NavigationConfig(), {self.col, self.row, self.theta_deg},
      {{cube.position.col, cube.position.row}, cube.age_ms}, {depot.col, depot.row}, &core_pushing);
  if (pushing != nullptr) *pushing = core_pushing ? 1 : 0;
  return {command.left, command.right, static_cast<unsigned char>(command.active),
          static_cast<unsigned char>(command.pushing), static_cast<unsigned char>(command.mode)};
}

void vrc_assign(VrcGrid grid, VrcDepotSize depot_size, float cube_side_cells,
                const VrcTaskRover* rovers, unsigned char rover_count, const VrcTaskCube* cubes,
                unsigned char cube_count, const VrcTaskDepot* depots, unsigned char depot_count,
                unsigned char* assigned) {
  if (!assigned) return;
  if (rover_count > 2 || cube_count > 3 || depot_count > 3) return;
  rover_logic::TaskRover core_rovers[2]; rover_logic::TaskCube core_cubes[3]; rover_logic::TaskDepot core_depots[3];
  for (unsigned char i = 0; i < rover_count && i < 2; ++i) core_rovers[i] = {rovers[i].id, {rovers[i].pose.col, rovers[i].pose.row, rovers[i].pose.theta_deg}, rovers[i].age_ms};
  for (unsigned char i = 0; i < cube_count && i < 3; ++i) core_cubes[i] = {static_cast<rover_logic::Color>(cubes[i].color), {{cubes[i].cube.position.col, cubes[i].cube.position.row}, cubes[i].cube.age_ms}};
  for (unsigned char i = 0; i < depot_count && i < 3; ++i) core_depots[i] = {static_cast<rover_logic::Color>(depots[i].color), {depots[i].position.col, depots[i].position.row}};
  rover_logic::Color result[2];
  rover_logic::assign_tasks({grid.cols, grid.rows, grid.cell_mm}, {depot_size.length_cells, depot_size.depth_cells}, cube_side_cells,
                            core_rovers, rover_count, core_cubes, cube_count, core_depots, depot_count, 300, 1200, result);
  for (unsigned char i = 0; i < rover_count && i < 2; ++i) assigned[i] = static_cast<unsigned char>(result[i]);
}

unsigned char vrc_should_yield(VrcPose self, unsigned char self_id, VrcPose other,
                               unsigned char other_id, unsigned int other_age_ms,
                               float cell_mm, float separation_mm) {
  return rover_logic::should_yield({self.col, self.row, self.theta_deg}, self_id,
                                   {other.col, other.row, other.theta_deg}, other_id,
                                   other_age_ms, cell_mm, separation_mm) ? 1 : 0;
}
}
