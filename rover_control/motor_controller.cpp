#include "motor_controller.h"

#include <math.h>

#include "hardware_config.h"

namespace {
constexpr uint32_t MAX_DUTY = (1U << 12) - 1U;

float bounded(float value) {
  if (value > 1.0F) return 1.0F;
  if (value < -1.0F) return -1.0F;
  return value;
}
}  // namespace

void MotorController::begin() {
  ready_ = ledcAttach(LEFT_FORWARD_PIN, PWM_HZ, PWM_BITS) &&
           ledcAttach(LEFT_BACKWARD_PIN, PWM_HZ, PWM_BITS) &&
           ledcAttach(RIGHT_FORWARD_PIN, PWM_HZ, PWM_BITS) &&
           ledcAttach(RIGHT_BACKWARD_PIN, PWM_HZ, PWM_BITS);
  stop();
  Serial.printf("[motor] PWM %s\n", ready_ ? "listo" : "ERROR al inicializar");
}

void MotorController::setWheel(uint8_t forward_pin, uint8_t backward_pin, float value) {
  value = bounded(value);
  const uint32_t duty = static_cast<uint32_t>(fabsf(value) * MAX_DUTY);
  if (value >= 0.0F) {
    ledcWrite(backward_pin, 0);
    ledcWrite(forward_pin, duty);
  } else {
    ledcWrite(forward_pin, 0);
    ledcWrite(backward_pin, duty);
  }
}

void MotorController::set(float left, float right) {
  if (!ready_) return;
  if (MOTOR_LEFT_REVERSED) left = -left;
  if (MOTOR_RIGHT_REVERSED) right = -right;
  setWheel(LEFT_FORWARD_PIN, LEFT_BACKWARD_PIN, left);
  setWheel(RIGHT_FORWARD_PIN, RIGHT_BACKWARD_PIN, right);
}

void MotorController::stop() {
  ledcWrite(LEFT_FORWARD_PIN, 0);
  ledcWrite(LEFT_BACKWARD_PIN, 0);
  ledcWrite(RIGHT_FORWARD_PIN, 0);
  ledcWrite(RIGHT_BACKWARD_PIN, 0);
}
