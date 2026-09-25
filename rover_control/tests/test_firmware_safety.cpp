#include <cassert>
#include <iostream>
#include <string>
#include "telemetry_client.h"
#include "safety_supervisor.h"
#include "vision_navigator.h"
#include "motor_controller.h"

static std::string message(unsigned seq, unsigned ts) {
  return "{\"v\":2,\"seq\":" + std::to_string(seq) + ",\"ts_ms\":" + std::to_string(ts) +
    R"(,"phase":"RUNNING","clock":{"elapsed_ms":0,"remaining_ms":1000,"total_ms":1000},"grid":{"cols":43,"rows":43,"cell_mm":20},"start":{"col":4,"row":20},"depot_size":{"length":10,"depth":7.5},"cube_side":3,"rovers":[{"id":10,"col":4,"row":20,"theta":0,"age_ms":50}],"cubes":[{"color":"blue","col":20,"row":20,"age_ms":100}],"depots":[{"color":"blue","col":39.25,"row":20}],"obstacles":[]})";
}
int main() {
  MotorController motors;
  motors.begin(); motors.set(0.2F, 0.2F);
  assert(test_duty[12] == 0 && test_duty[14] > 0); // override seen inside motor_controller.cpp
  assert(test_duty[13] > 0 && test_duty[15] == 0);
  motors.stop(); assert(test_duty[14] == 0 && test_duty[13] == 0);
  TelemetryClient client("", "", "", 2026);
  SafetySupervisor safety(1500);
  test_now = 100;
  auto feed = [&](std::string s) { WiFiClient::incoming += s; client.poll(); };
  auto m = message(10, 1000);
  feed(m.substr(0, 40)); assert(!client.world().valid);
  feed(m.substr(40) + "\n"); assert(client.world().valid);
  assert(safety.evaluate(client.world(), 10, 300).motion_allowed);
  VisionNavigator navigator;
  auto drive = navigator.update(client.world(), 10);
  assert(drive.active && drive.left == 0.11F && drive.right == 0.11F); // speed override
  auto coordinated = client.world();
  coordinated.rover_count = 2;
  coordinated.rovers[1] = coordinated.rovers[0];
  coordinated.rovers[1].id = 11;
  coordinated.rovers[1].col = 5;
  // Cada decisión corresponde a un mensaje: un mundo distinto trae otro seq.
  coordinated.seq += 1;
  // The closer rover gets the only cube. Chassis 20 mm apart overlap: the
  // idle rover holds still instead of driving through the other one.
  const auto idle = navigator.update(coordinated, 10);
  assert(!idle.active && std::string(idle.mode) == "YIELD");
  // With real clearance, the rover without a task clears the field.
  auto apart = coordinated;
  apart.rovers[1].col = 16; apart.seq += 1;
  const auto park = navigator.update(apart, 10);
  assert(park.active && std::string(park.mode) == "PARK");
  // Rover 11 has a task, but must yield within 200 mm of rover 10.
  assert(!navigator.update(coordinated, 11).active);
  coordinated.rovers[0].col = 40; coordinated.seq += 1;
  assert(navigator.update(coordinated, 11).active);
  coordinated.cubes[0].color = "green";
  coordinated.depots[0].color = "green"; coordinated.seq += 1;
  assert(navigator.update(coordinated, 11).active);
  test_now = 351;
  assert(!safety.evaluate(client.world(), 10, 300).motion_allowed);
  feed(message(11, 1000) + "\n"); // stale capture with a new publication seq
  assert(client.world().received_at_ms == 100);
  test_now = 401;
  assert(!navigator.update(client.world(), 10).active); // cube 100+301 > configured 400
  feed("{}\n"); assert(!client.world().valid);
  feed(message(12, 1200) + "\n"); assert(client.world().valid);
  auto finished = client.world(); finished.phase = RoundPhase::FINISHED;
  assert(!safety.evaluate(finished, 10, 300).motion_allowed);
  auto absent = client.world(); absent.rover_count = 0;
  assert(!safety.evaluate(absent, 10, 300).motion_allowed);
  auto bad = message(13, 1300); auto pos = bad.find("\"theta\":0");
  bad.replace(pos, 9, "\"theta\":\"bad\"");
  feed(bad + "\n"); assert(!client.world().valid);
  feed(std::string(2100, 'x') + message(14, 1400) + "\n");
  assert(!client.world().valid); // oversized line tail must not become a message
  feed(message(15, 1500) + "\n"); assert(client.world().valid);
  WiFiClient::online = false; client.poll(); assert(!client.world().valid);
  test_now = 2000; client.poll(); // new TCP session permits a restarted sequence
  feed(message(1, 2000) + "\n"); assert(client.world().valid && client.world().seq == 1);
  auto wrap = client.world(); wrap.received_at_ms = UINT32_MAX - 50; wrap.rovers[0].age_ms = 0;
  test_now = 100; assert(safety.evaluate(wrap, 10, 300).motion_allowed);
  test_now = 251; assert(!safety.evaluate(wrap, 10, 300).motion_allowed);
  std::cout << "Firmware safety/config/NDJSON checks passed\n";
}
