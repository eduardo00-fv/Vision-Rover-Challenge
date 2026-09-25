// Compila el firmware SIN CAMBIOS (setup/loop reales) y lo expone al simulador.
// Se enlaza con -fvisibility=hidden: cada rover tiene sus propios globales.
#include "rover_control.ino"
#include "firmware_api.h"
#include <esp_now.h>

#define FW_API extern "C" __attribute__((visibility("default")))

namespace {
float wheel(uint8_t forward, uint8_t backward) {
  return (static_cast<float>(sim_duty[forward]) - static_cast<float>(sim_duty[backward])) / 4095.0F;
}
}  // namespace

FW_API void fw_setup() {
  sim_digital[START_BUTTON_PIN] = HIGH;  // pull-up: suelto
  setup();
}
FW_API void fw_button(int pressed) { sim_digital[START_BUTTON_PIN] = pressed ? LOW : HIGH; }
FW_API void fw_loop() { loop(); }
FW_API void fw_set_time(uint64_t us) { sim_time_us = us; }
FW_API uint64_t fw_get_time() { return sim_time_us; }  // setup() consume delay()
FW_API void fw_tcp_push(const char* data, size_t size) { WiFiClient::incoming.append(data, size); }
FW_API void fw_tcp_online(int online) {
  WiFiClient::online = online != 0;
  if (!online) { WiFiClient::session = false; WiFiClient::incoming.clear(); }
}
FW_API void fw_imu_ok(int ok) { Wire.ok = ok != 0; }
FW_API void fw_imu_sample(float z_dps) {
  const float raw = std::round(z_dps / 0.00875F);
  Wire.z = static_cast<int16_t>(raw > 32767 ? 32767 : (raw < -32768 ? -32768 : raw));
  Wire.fresh = true;
}
FW_API void fw_motors(float* left, float* right) {
  // GPIO de motor_controller.h: izquierdo 12/14, derecho 13/15.
  *left = wheel(12, 14);
  *right = wheel(13, 15);
  if (MOTOR_LEFT_REVERSED) *left = -*left;
  if (MOTOR_RIGHT_REVERSED) *right = -*right;
}
FW_API void fw_status(FwStatus* status) {
  status->mode = last_command.mode;
  status->mission = mission.stateName();
  status->motion_reason = motion.reason;
  status->motion_state = static_cast<int>(motion.state);
  status->target = last_command.target;
  status->pushing = last_command.pushing;
  status->emergency_stop = emergency_stop;
  status->heading_error_deg = last_command.heading_error_deg;
}
FW_API size_t fw_espnow_take(char* buffer, size_t capacity) {  // un paquete enviado, 0 si no hay
  if (sim_espnow_out.empty()) return 0;
  const std::string packet = sim_espnow_out.front();
  sim_espnow_out.erase(sim_espnow_out.begin());
  const size_t n = packet.size() < capacity ? packet.size() : capacity;
  packet.copy(buffer, n);
  return n;
}
FW_API void fw_espnow_deliver(const char* data, size_t size) {
  sim_espnow_deliver(reinterpret_cast<const uint8_t*>(data), static_cast<int>(size));
}
FW_API void fw_serial_input(const char* text) { Serial.input += text; }
FW_API void fw_serial_capture(int enabled) { Serial.capture = enabled != 0; }
FW_API size_t fw_serial_take(char* buffer, size_t capacity) {
  const size_t n = Serial.output.size() < capacity ? Serial.output.size() : capacity;
  Serial.output.copy(buffer, n);
  Serial.output.erase(0, n);
  return n;
}
