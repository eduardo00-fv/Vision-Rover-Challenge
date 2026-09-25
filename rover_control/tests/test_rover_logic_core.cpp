#include "../rover_logic_core.h"

#include <cassert>

int main() {
  using namespace rover_logic;
  const Grid grid = {43, 43, 20.0F};
  const DepotSize depot_size = {10.0F, 7.5F};
  const Point top_depot = {21.5F, 3.75F};

  // El margen es media diagonal, igual que el contrato: un centro que parece
  // entrar por media arista pero deja una esquina afuera no se entrega.
  assert(cube_delivered(grid, depot_size, 3.0F, {21.5F, 3.75F}, top_depot));
  assert(!cube_delivered(grid, depot_size, 3.0F, {21.5F, 6.80F}, top_depot));

  NavigationConfig config;
  bool pushing = false;
  const DriveCommand turn = navigate(grid, depot_size, 3.0F, config,
                                     {10.0F, 10.0F, 180.0F}, {{15.0F, 10.0F}, 0},
                                     {39.25F, 21.5F}, &pushing);
  assert(turn.active && turn.mode == Mode::APPROACH_ALIGN);

  // Aproximando, un cubo de más de 1,2 s no se persigue.
  pushing = false;
  const DriveCommand stale = navigate(grid, depot_size, 3.0F, config,
                                      {10.0F, 10.0F, 0.0F}, {{15.0F, 10.0F}, 1201},
                                      {39.25F, 21.5F}, &pushing);
  assert(!stale.active && stale.mode == Mode::WAIT_TARGET);
  // Empujando, el propio rover lo tapa: se tolera hasta 3 s, no más.
  PushState state;
  state.stage = PushState::PUSH;
  state.ux = 1; state.uy = 0;
  state.gx = 39.25F * 20; state.gy = 10.0F * 20;
  const Pose behind = {15.0F - 107.0F / 20.0F, 10.0F, 0.0F};
  assert(navigate(grid, depot_size, 3.0F, config, behind, {{15.0F, 10.0F}, 2999},
                  {39.25F, 21.5F}, nullptr, 0, &state).active);
  state.stage = PushState::PUSH;
  assert(!navigate(grid, depot_size, 3.0F, config, behind, {{15.0F, 10.0F}, 3001},
                   {39.25F, 21.5F}, nullptr, 0, &state).active);

  // Rover entre el cubo y su zona: no avanza derecho a través del cubo.
  PushState around;
  const DriveCommand detour = navigate(grid, depot_size, 3.0F, config, {30.0F, 21.5F, 180.0F},
                                       {{20.0F, 21.5F}, 0}, {39.25F, 21.5F}, nullptr, 0, &around);
  assert(detour.active && detour.mode == Mode::AVOID);

  // Cubo dentro de la zona y pasado del centro hacia la pared: terminado.
  assert(cube_done(grid, depot_size, 3.0F, {40.5F, 21.5F}, {39.25F, 21.5F}, 12.0F));
  assert(!cube_done(grid, depot_size, 3.0F, {36.0F, 21.5F}, {39.25F, 21.5F}, 12.0F));

  // Empujando un cubo que resbala por la pared este: el rover no lo sigue
  // fuera de la superficie; se retira.
  PushState wall;
  wall.stage = PushState::PUSH;
  wall.ux = 0.2588F; wall.uy = 0.9659F; wall.gx = 961.2F; wall.gy = 789.7F;
  const DriveCommand along = navigate(grid, depot_size, 3.0F, config, {41.500F, 15.000F, -75.0F},
                                      {{42.885F, 20.168F}, 0}, {21.5F, 39.25F}, nullptr, 0, &wall);
  assert(!(along.active && !along.reverse));
  // Guarda final: una orden recta hacia afuera desde el margen se retiene.
  const DriveCommand out = go_to(grid, config, {42.0F, 21.5F, 0.0F}, {42.9F, 21.5F}, nullptr, 0, Mode::PARK);
  assert(!(out.active && !out.reverse));
}
