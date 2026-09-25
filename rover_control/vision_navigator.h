#pragma once

#include <Arduino.h>

#include "rover_logic_core.h"
#include "world_state.h"

struct DriveCommand {
  float left = 0.0F;
  float right = 0.0F;
  bool active = false;
  bool pushing = false;
  const char* mode = "WAIT_TARGET";
  float heading_error_deg = 0;
  uint8_t target = 0;
  bool reverse = false;
  float distance_mm = 0;
};

// Navegación de lazo cerrado sin encoders: cada orden se decide de nuevo con
// la pose fresca que entrega la cámara. No integra distancia ni orientación.
class VisionNavigator {
 public:
  DriveCommand update(const WorldState& world, uint8_t rover_id);
  // Estado que anuncia un compañero por ESP-NOW (opcional, fresco <500 ms).
  // Solo afina el reparto: uno que no puede moverse suelta sus cubos al
  // instante y el cubo que otro está empujando no se le disputa.
  void peerHint(uint8_t id, bool can_move, uint8_t claimed, bool pushing, uint32_t now);
  // Lo que este rover anuncia: cubo objetivo y etapa (peer_proto::Stage).
  uint8_t claimedTarget() const { return decided_ ? decision_.target : 0; }
  uint8_t stage() const;

 private:
  struct Liveness { uint8_t id = 0; float col = 0, row = 0; uint32_t moved_ms = 0; };
  struct Hint { uint8_t id = 0; bool can_move = true; uint8_t claimed = 0; bool pushing = false; uint32_t at_ms = 0; };

  rover_logic::PushState push_;
  // La decisión (asignación + A*) se calcula una vez por mensaje de visión;
  // entre mensajes el loop reutiliza la misma orden.
  uint32_t decided_seq_ = 0;
  uint64_t decided_capture_ = 0;
  uint8_t decided_id_ = 0;
  bool decided_ = false;
  DriveCommand decision_;
  String last_target_;
  // Tarea anterior de cada marcador: la asignación no cambia por centímetros.
  uint8_t previous_ids_[MAX_ROVERS] = {};
  rover_logic::Color previous_[MAX_ROVERS] = {};
  // Un rover que no se desplaza en mucho tiempo deja de contar para repartir
  // tareas (IMU enclavada, atascado), aunque la cámara lo siga viendo.
  Liveness liveness_[MAX_ROVERS];
  Hint hints_[MAX_ROVERS];
  const Hint* hint(uint8_t id, uint32_t now) const;

  void trackLiveness(const WorldState& world, uint32_t now);
  bool inactive(uint8_t id, uint32_t now) const;
  bool moving(uint8_t id, uint32_t now) const;
  DriveCommand decide(const WorldState& world, uint8_t rover_id, uint32_t now);
};
