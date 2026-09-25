#include "diagnostic_host/Arduino.h"
#include "../diagnostico/diagnostico.ino"
#include <cassert>
#include <iostream>

void send(const char* cmd) {
  Serial.input += cmd;
  while (Serial.available()) loop();
}
void advance(unsigned ms) {
  for (unsigned i = 0; i < ms; i += 10) { delay(10); loop(); }
}
void stopped() {
  assert(!moving && !armed);
  assert(!test_duty[12] && !test_duty[14] && !test_duty[13] && !test_duty[15]);
}
void calibrate() {
  Wire.ok = Wire.fresh = true; Wire.z = 60;
  send("GZERO\n"); advance(2600);
  assert(gyro_calibrated); stopped();
}
int main() {
  setup(); stopped();
  send("ARM\nSTRAIGHT 18 20 3500\n"); stopped(); // calibration required
  calibrate();
  send("ARM\nSTRAIGHT 18 20 3500\n"); stopped(); // sign required
  send("GSIGN 1\nARM\nSTRAIGHT 18 20 3500\n"); assert(moving);
  Wire.z = 1200; advance(20);
  assert(heading.yaw > 0 && heading.left > 18 && heading.right < 20);
  send("!"); stopped(); send("\n"); // no Enter needed
  Wire.z = 60;
  send("ARM\nSTRAIGHT 18 20 100\n"); assert(moving);
  send("partial"); advance(150); stopped(); send("\n");
  send("ARM\nSTRAIGHT 18 20 3500\n"); assert(moving);
  send("COLOR 50\n"); stopped(); // blocking measurement must only cancel
  send("ARM\nSTRAIGHT 18 20 3500\n"); assert(moving);
  Wire.fresh = false; advance(110); stopped(); assert(!gyro_calibrated);
  calibrate();
  send("ARM\nSTRAIGHT 18 20 3500\n"); Wire.ok = false; advance(10); stopped();
  calibrate();
  send("ARM\nSTRAIGHT nan 20 3500\n"); stopped();
  send("ARM\nSTRAIGHT 18 20 3501\n"); stopped();
  send("ARM\nSTRAIGHT 18 20 3500 junk\n"); stopped();
  advance(61000);
  send("ARM\nSTRAIGHT 18 20 3500\n"); stopped(); // expired calibration
  send("GZERO\n"); advance(400); send("!"); stopped(); assert(!gyro_calibrated); send("\n");
  send("GZERO\n"); Wire.fresh = false; advance(4100); stopped(); assert(!gyro_calibrated);
  Wire.fresh = true; Wire.z = 1000;
  send("GZERO\n"); advance(2600); stopped(); assert(!gyro_calibrated);
  calibrate(); send("GYRO 1000\n"); advance(1100); stopped();
  send("TURNTEST -25 25 1000\n"); stopped(); // ARM required
  gyro_sign = 0;
  send("ARM\nTURNTEST -25 25 1000\n"); stopped();
  send("GSIGN 1\n");
  send("ARM\nTURNTEST 25 25 1000\n"); stopped();
  send("ARM\nTURNTEST -36 25 1000\n"); stopped();
  send("ARM\nTURNTEST -25 25 3501\n"); stopped();
  send("ARM\nTURNTEST -25 25 1000 junk\n"); stopped();
  send("ARM\nTURNTEST -25 25 1000\n"); assert(moving);
  assert(test_duty[14] && test_duty[13] && !test_duty[12] && !test_duty[15]);
  Wire.z = 5203; // ~45 degrees/s after bias correction
  send("incomplete"); advance(1000); stopped();
  assert(gyro_job == GyroJob::COAST);
  assert(turn_cut_yaw > 43 && turn_cut_yaw < 46);
  advance(500); stopped();
  assert(gyro_job == GyroJob::NONE && turn_yaw > 65 && turn_yaw < 69);
  assert(Serial.output.find("TURNTEST RESULT") != std::string::npos);
  send("\n");
  Wire.z = 60;
  send("ARM\nTURNTEST 25 -25 1000\n");
  Wire.z = -5083; advance(100);
  assert(turn_yaw < -4 && turn_yaw > -5);
  send("!"); stopped(); send("\n");
  send("ARM\nTURNTEST -25 25 1000\n");
  Wire.fresh = false; advance(110); stopped(); assert(!gyro_calibrated);
  calibrate();
  send("ARM\nTURNTEST -25 25 1000\n");
  Wire.ok = false; advance(10); stopped();
  calibrate();
  send("ARM\nTURNTEST -25 25 100\n"); advance(100);
  stopped(); assert(gyro_job == GyroJob::COAST);
  send("!"); stopped(); assert(gyro_job == GyroJob::NONE); send("\n");
  send("ARM\nTURNTEST -25 25 3500\n"); Wire.z = 24000;
  advance(10); stopped(); assert(!gyro_calibrated); // rate limit
  calibrate();
  send("ARM\nTURNTEST -25 25 3500\n"); Wire.z = 17203;
  advance(1300); stopped(); assert(!gyro_calibrated); // angle limit
  calibrate();
  send("ARM\nTURNTEST -25 25 100\n"); advance(100);
  Wire.fresh = false; advance(110); stopped(); assert(!gyro_calibrated);
  assert(gyro_job == GyroJob::NONE);
  calibrate();
  send("ARM\nTURNTEST -25 25 100\n"); advance(100);
  send("ARM\n"); stopped(); // during coast this cancels, it cannot arm
  assert(gyro_job == GyroJob::NONE);
  send("ARM\nTURNTEST -25 25 100\n"); advance(100);
  Serial.output.clear(); Wire.ok = false; advance(10); stopped();
  assert(Serial.output.find("TURNTEST RESULT") == std::string::npos);
  calibrate(); advance(61000);
  send("ARM\nTURNTEST -25 25 1000\n"); stopped(); // expired calibration
  send("ARM\nTURN 45\n"); stopped(); // expired calibration
  calibrate();
  send("TURN 45\n"); stopped(); // not armed
  send("ARM\nTURN nan\n"); stopped();
  send("ARM\nTURN 181\n"); stopped();
  send("ARM\nTURN 45 junk\n"); stopped();
  gyro_sign = 0; send("ARM\nTURN 45\n"); stopped(); send("GSIGN 1\n");
  send("ARM\nTURN 45\n"); assert(moving && turn_closed);
  Wire.z = 5203; advance(1000); stopped();
  assert(gyro_job == GyroJob::COAST && turn_cut_yaw > 42 && turn_cut_yaw < 45);
  Wire.z = 60; Serial.output.clear(); advance(500); stopped();
  assert(Serial.output.find("TURN RESULT status=OK") != std::string::npos);
  send("ARM\nTURN -45\n"); assert(test_duty[12] && test_duty[15]);
  Wire.z = -5083; advance(1000); stopped();
  assert(turn_cut_yaw < -41 && turn_cut_yaw > -45);
  Wire.z = 60; advance(500); stopped();
  send("ARM\nTURN 45\n"); advance(760); stopped(); // fresh samples, but stalled
  assert(!gyro_calibrated);
  calibrate(); send("ARM\nTURN 45\n");
  Wire.z = -11369; advance(100); stopped(); // wrong direction
  calibrate(); send("ARM\nTURN 45\n");
  Wire.fresh = false; advance(110); stopped();
  calibrate(); send("ARM\nTURN 45\n");
  Wire.ok = false; advance(10); stopped();
  calibrate(); send("ARM\nTURN 45\n");
  Wire.z = 631; send("partial"); advance(3500); stopped(); // slow but progressing, timeout
  assert(gyro_job == GyroJob::COAST && turn_timed_out);
  Wire.z = 60; Serial.output.clear(); advance(500);
  assert(Serial.output.find("TURN RESULT status=TIMEOUT") != std::string::npos);
  send("\nARM\nTURN 45\n"); send("!"); stopped(); send("\n");
  send("ARM\nTURN 45\n"); Wire.z = 5203; advance(1000); stopped();
  Wire.z = 2346; // still turning ~20 dps, but below the independent overshoot limit
  Serial.output.clear(); advance(500); stopped();
  assert(Serial.output.find("status=UNSETTLED") != std::string::npos);
  Wire.z = 60;
  send("ARM\nTURN 45\n"); Wire.z = 5203; advance(1000); stopped();
  advance(100); Wire.z = 60; Serial.output.clear(); advance(500);
  assert(Serial.output.find("status=OUTSIDE_TOLERANCE") != std::string::npos);
  for (const char* cmd : {"ARM\nTURN 180\n", "ARM\nTURN -180\n"}) {
    calibrate(); send(cmd); assert(moving);
    assert(stop_at_ms - millis() == 7000);
    Wire.z = turn_control.target > 0 ? 5203 : -5083;
    advance(3500); assert(moving); // old deadline must not truncate 180 degrees
    advance(500); stopped(); assert(gyro_job == GyroJob::COAST && !turn_timed_out);
    assert(fabsf(turn_cut_yaw) > 176 && fabsf(turn_cut_yaw) < 180);
    Wire.z = 60; Serial.output.clear(); advance(500);
    assert(Serial.output.find("TURN RESULT status=OK") != std::string::npos);
  }
  calibrate(); send("ARM\nTURN 180\n"); Wire.z = 631;
  advance(7000); stopped(); assert(turn_timed_out);
  Wire.z = 60; advance(500);
  send("ARM\nMOTOR 25 -25 7000\n"); assert(!moving); send("STOP\n");
  send("ARM\nTURNTEST -25 25 7000\n"); stopped();
  send("ARM\nSTRAIGHT 18 20 7000\n"); stopped();
  send("ARM\nTURN 180\n"); send("!"); stopped(); send("\n");
  std::cout << "Diagnostic heading/turn safety checks passed\n";
}
