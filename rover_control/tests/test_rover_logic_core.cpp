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

  pushing = true;
  const DriveCommand stale = navigate(grid, depot_size, 3.0F, config,
                                      {10.0F, 10.0F, 0.0F}, {{15.0F, 10.0F}, 1201},
                                      {39.25F, 21.5F}, &pushing);
  assert(!stale.active && stale.mode == Mode::WAIT_TARGET);
  assert(!pushing);
}
