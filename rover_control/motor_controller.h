#pragma once

#include <Arduino.h>

// Driver diferencial de la IdeaBoard. El valor de cada rueda está normalizado
// entre -1.0 (reversa) y 1.0 (avance); no conoce misión ni telemetría.
class MotorController {
 public:
  void begin();
  void set(float left, float right);
  void stop();

 private:
  static constexpr uint8_t LEFT_FORWARD_PIN = 12;
  static constexpr uint8_t LEFT_BACKWARD_PIN = 14;
  static constexpr uint8_t RIGHT_FORWARD_PIN = 13;
  static constexpr uint8_t RIGHT_BACKWARD_PIN = 15;
  static constexpr uint32_t PWM_HZ = 1000;
  static constexpr uint8_t PWM_BITS = 12;

  bool ready_ = false;
  void setWheel(uint8_t forward_pin, uint8_t backward_pin, float value);
};
