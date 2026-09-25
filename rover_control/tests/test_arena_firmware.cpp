#include "rover_control.ino"
#include <cassert>
#include <iostream>
static unsigned sequence = 0;
static void feed(const char* phase = "RUNNING") {
  WiFiClient::incoming = "{\"v\":2,\"seq\":" + std::to_string(++sequence) +
    ",\"ts_ms\":" + std::to_string(millis()) + ",\"phase\":\"" + phase +
    R"(","clock":{"elapsed_ms":0,"remaining_ms":1000,"total_ms":1000},"grid":{"cols":43,"rows":43,"cell_mm":20},"start":{"col":4,"row":20},"depot_size":{"length":10,"depth":7.5},"cube_side":3,"rovers":[{"id":10,"col":4,"row":20,"theta":0,"age_ms":0}],"cubes":[{"color":"blue","col":20,"row":20,"age_ms":0}],"depots":[{"color":"blue","col":39.25,"row":20}],"obstacles":[]})" "\n";
}
static bool stopped() {
  return test_duty[12] == 0 && test_duty[14] == 0 && test_duty[13] == 0 && test_duty[15] == 0;
}
static void tick(bool packet = true, const char* phase = "RUNNING") {
  test_us += 10000; if (packet) feed(phase); loop();
}
static void moving() {
  tick(); tick(); assert(!stopped());
}
int main() {
  setup(); assert(stopped());
#if VRC_NETWORK_ONLY
  for (int i = 0; i < 400; ++i) { tick(); assert(stopped()); }
  assert(telemetry.validMessages() == 400);
  assert(telemetry.world().phase == RoundPhase::RUNNING);
  std::cout << "Network-only: valid RUNNING telemetry, zero motor PWM\n";
  return 0;
#endif
  for (int i = 0; i < 225; ++i) tick(true, "READY");
  assert(motion.state == decltype(motion)::State::IDLE && stopped());
  moving(); tick(true, "FINISHED"); assert(stopped());
  moving(); WiFiClient::online = false; tick(false); assert(stopped());
  WiFiClient::online = true; moving();
  for (int i = 0; i < 31; ++i) tick(false);
  assert(stopped()); // pose age, before 350 ms maneuver deadline
  moving(); Serial.input = "!"; tick(); assert(stopped());
  tick(); assert(stopped() && emergency_stop);
  emergency_stop = false; moving();
  Wire.ok = false; tick(); assert(stopped());
  Wire.ok = true; tick(); assert(stopped()); // IMU fault stays latched
  std::cout << "Actual arena setup/loop stop checks passed\n";
}
