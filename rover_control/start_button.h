#pragma once

#include <Arduino.h>

// Botón de arranque de la IdeaBoard (BOOT, IO0, activo en bajo). Con
// VRC_REQUIRE_START_BUTTON el rover se mueve solo si alguien lo presionó al
// recibir la señal del juez Y la visión está en RUNNING. Queda armado hasta
// que termina la ronda; la siguiente exige presionarlo de nuevo.
class StartButton {
 public:
  static constexpr uint32_t kDebounceMs = 50;

  // Devuelve si está armado. `pressed` es la lectura cruda del botón.
  bool update(bool pressed, bool round_over, uint32_t now) {
    if (round_over) { armed_ = false; pressed_since_ = 0; held_ = false; return false; }
    if (!pressed) { held_ = false; return armed_; }
    if (!held_) { held_ = true; pressed_since_ = now; }
    if (now - pressed_since_ >= kDebounceMs) armed_ = true;
    return armed_;
  }
  bool armed() const { return armed_; }

 private:
  bool armed_ = false, held_ = false;
  uint32_t pressed_since_ = 0;
};
