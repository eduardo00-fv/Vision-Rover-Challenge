#include "diagnostic_host/Wire.h"
#include "../motion/arena_motion.h"
#include <cassert>
#include <iostream>
using Controller = ArenaMotion<WireMock>;
using State = Controller::State;

void calibrate(Controller& motion, uint32_t& now) {
  motion.begin(now);
  for (int i = 0; i < 225; ++i) { now += 10000; motion.sample(now); }
  assert(motion.state == State::IDLE);
}
int main() {
  WireMock bus; bus.begin(); uint32_t now = 0;
  Controller motion(bus, rover_motion::rover1);
  calibrate(motion, now);
  motion.command(true, true, 0, 1, false, 1, now);
  now += 10000; motion.sample(now);
  assert(motion.left == .18F && motion.right == .20F);
  // Permission loss (pose/link/phase/IR/!) overrides an active maneuver.
  motion.command(false, true, 0, 1, false, 1, now);
  assert(motion.left == 0 && motion.right == 0);
  motion.command(true, true, 0, 1, false, 2, now);
  for (int i = 0; i < 35; ++i) { now += 10000; motion.sample(now); }
  assert(motion.state == State::SETTLE && motion.left == 0);
  for (int i = 0; i < 51; ++i) {
    now += 10000; motion.sample(now);
    motion.command(true, true, 0, 1, false, 2, now);
  }
  assert(motion.state == State::SETTLE); // same capture cannot restart
  motion.command(true, true, 0, 1, false, 3, now);
  assert(motion.state == State::DRIVE);
  now += 100001; motion.guard(now);
  assert(motion.state == State::FAULT && motion.left == 0);
  motion.command(true, true, 0, 1, false, 4, now);
  assert(motion.state == State::FAULT); // faults latch

  for (int sign : {1, -1}) {
    bus.z = 60; calibrate(motion, now);
    motion.command(true, true, sign * 45, 1, false, 5, now);
    bus.z = 60 + sign * 5714; // ~50 deg/s after bias
    for (int i = 0; i < 100 && motion.state == State::TURN; ++i) {
      now += 10000; motion.sample(now);
    }
    assert(motion.state == State::SETTLE && motion.left == 0);
  }
  bus.z = 60; calibrate(motion, now);
  // Un giro trabado se reintenta con otra captura; 4 seguidos enclavan.
  for (int attempt = 1; attempt <= 4; ++attempt) {
    motion.command(true, true, 90, 1, false, 100 + attempt, now);
    for (int i = 0; i < 80 && motion.state == State::TURN; ++i) { now += 10000; motion.sample(now); }
    if (attempt < 4) {
      assert(motion.state == State::SETTLE && motion.left == 0);
      for (int i = 0; i < 51; ++i) { now += 10000; motion.sample(now); motion.command(true, true, 90, 1, false, 100 + attempt, now); }
    }
  }
  assert(motion.state == State::FAULT); // stalled four times
  calibrate(motion, now);
  bus.ok = false; now += 10000; motion.sample(now);
  assert(motion.state == State::FAULT && motion.left == 0);
  bus.ok = true;
  Controller second(bus, rover_motion::rover2); second.begin(now);
  assert(second.state == State::FAULT);
  // micros wrap does not change elapsed-time turn tracking.
  now = UINT32_MAX - 2400000; bus.z = 60; calibrate(motion, now);
  motion.command(true, true, 45, 1, false, 7, now); bus.z = 5774;
  for (int i = 0; i < 100 && motion.state == State::TURN; ++i) {
    now += 10000; motion.sample(now);
  }
  assert(motion.state == State::SETTLE);
  bus.z = 60; calibrate(motion, now);
  motion.command(true, true, 0, 1, false, 8, now);
  now += 10000; motion.sample(now);
  motion.command(true, true, 0, 2, false, 9, now);
  assert(motion.state == State::SETTLE && motion.left == 0); // reassignment
  motion.begin(now);
  for (int i = 0; i < 225; ++i) {
    bus.z = i % 2 ? 0 : 500; now += 10000; motion.sample(now);
  }
  assert(motion.state == State::FAULT); // robot moved during zero
  bus.z = 60; calibrate(motion, now);
  motion.command(true, true, 180, 1, false, 10, now);
  bus.z = 60 + 2857; // 25 deg/s: progressing but cannot finish by 7 seconds
  for (int i = 0; i < 700; ++i) { now += 10000; motion.sample(now); }
  assert(motion.state == State::SETTLE && motion.left == 0); // timeout: retry, not latch
  // Reverse drive keeps the heading correction sign and never turns in place.
  bus.z = 60; calibrate(motion, now);
  motion.command(true, true, 0, 1, false, 20, now, true, 80);
  now += 10000; motion.sample(now);
  assert(motion.state == State::DRIVE && motion.left < 0 && motion.right < 0);
  // Configurable pace: 150 ms settle and a 1 s leg cap.
  Controller fast(bus, rover_motion::rover1);
  fast.timing(150, 1000);
  bus.z = 60; calibrate(fast, now);
  fast.command(true, true, 0, 1, false, 30, now);
  for (int i = 0; i < 95; ++i) { now += 10000; fast.sample(now); }
  assert(fast.state == State::DRIVE);  // 350 ms default cap no longer applies
  for (int i = 0; i < 10; ++i) { now += 10000; fast.sample(now); }
  assert(fast.state == State::SETTLE && fast.left == 0);
  for (int i = 0; i < 14; ++i) { now += 10000; fast.sample(now); fast.command(true, true, 0, 1, false, 31, now); }
  assert(fast.state == State::SETTLE);  // still coasting
  for (int i = 0; i < 2; ++i) { now += 10000; fast.sample(now); fast.command(true, true, 0, 1, false, 31, now); }
  fast.command(true, true, 0, 1, false, 32, now);
  assert(fast.state == State::DRIVE);  // fresh capture after 150 ms
  // IMU lost with the vision-only fallback: stop, then timed maneuvers.
  Controller blind(bus, rover_motion::rover1);
  blind.allowVisionOnly(true);
  blind.timing(150, 1000);
  bus.z = 60; calibrate(blind, now);
  blind.command(true, true, 0, 1, false, 40, now);
  now += 100001; blind.guard(now);
  assert(blind.state == State::SETTLE && blind.visionOnly() && blind.left == 0);
  for (int i = 0; i < 60; ++i) { now += 10000; blind.sample(now); blind.command(true, true, 30, 1, false, 40, now); }
  blind.command(true, true, 30, 1, false, 41, now);
  assert(blind.state == State::TURN && blind.right == .25F && blind.left == -.25F);  // no IMU sample needed
  now += 490000; blind.guard(now);
  assert(blind.state == State::TURN);
  now += 20000; blind.guard(now);
  assert(blind.state == State::SETTLE && blind.left == 0);  // 30 deg at 60 deg/s = 500 ms
  now += 600000; blind.command(true, true, 90, 1, false, 42, now);
  now += 760000; blind.guard(now);
  assert(blind.state == State::SETTLE);  // capped at 45 deg per maneuver
  now += 600000; blind.command(true, true, 0, 1, false, 43, now, false, 1000);
  assert(blind.state == State::DRIVE && blind.left == .18F && blind.right == .20F);
  now += 490000; blind.guard(now);
  assert(blind.state == State::DRIVE);
  now += 20000; blind.guard(now);
  assert(blind.state == State::SETTLE);  // blind legs capped at 500 ms, not 1000
  // Without the fallback the same loss still latches FAULT (checked above).
  Controller absent(bus, rover_motion::rover1);
  absent.allowVisionOnly(true);
  bus.ok = false; absent.begin(now); bus.ok = true;
  assert(absent.visionOnly() && absent.state != State::FAULT);
  std::cout << "Arena motion checks passed\n";
}
