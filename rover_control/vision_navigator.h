#pragma once

#include <Arduino.h>

#include "world_state.h"

struct DriveCommand {
  float left = 0.0F;
  float right = 0.0F;
  bool active = false;
  bool pushing = false;
  const char* mode = "WAIT_TARGET";
};

// Navegación de lazo cerrado sin encoders: cada orden se decide de nuevo con
// la pose fresca que entrega la cámara. No integra distancia ni orientación.
class VisionNavigator {
 public:
  DriveCommand update(const WorldState& world, uint8_t rover_id);

 private:
  bool pushing_ = false;
  String last_target_;
};
