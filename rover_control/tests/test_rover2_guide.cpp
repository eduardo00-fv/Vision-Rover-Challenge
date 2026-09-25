#include "diagnostic_host/Arduino.h"
#include "../calibracion_rover2/calibracion_rover2.ino"
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
int main() {
  setup(); stopped();
  assert(motion_profile.rover == 2 && !motion_profile.initially_validated);
  assert(gyro_sign == 0 && !wheel_map_confirmed && guided_left == 20 && guided_right == 20);
  send("CHECK\n"); stopped();
  send("IMUBUS\n"); stopped(); assert(!gyro_calibrated);
  assert(Serial.output.find("IMUBUS FIN") != std::string::npos);
  send("M1\n"); assert(moving && test_duty[12] && !test_duty[13]);
  advance(1010); assert(moving);
  advance(2500); stopped();
  send("M2\n"); assert(moving && !test_duty[12] && test_duty[13]);
  send("!"); stopped(); send("\n");
  send("REV\n"); assert(moving && test_duty[14] && test_duty[15]);
  send("STATUS\n"); stopped();
  send("RECTO 1000\nGIRO 45\n"); stopped();
  send("CERO\n"); advance(2600); stopped(); assert(gyro_calibrated);
  send("SIGNO 1\nGIRO 45\n"); stopped(); // map still unconfirmed
  send("MAPA OK\nBASE 18 20\nGANANCIAS 0.8 0.08 4\nRECTO 1000\n");
  assert(moving && heading.base_left == 18 && heading.base_right == 20);
  send("GIRO 45\n"); stopped(); // cancels current operation, never chains
  send("GIRO 45\n"); assert(moving && turn_closed);
  send("!"); stopped(); send("\n");
  send("GIRO 181\nRECTO -1\n"); stopped();
  send("BASE nan 20\n"); assert(guided_left == 18);
  send("GANANCIAS 9 9 9\n"); assert(heading.kp == 0.8F);
  advance(61000); send("GIRO 90\n"); stopped();
  send("CERO\n"); advance(400); send("!"); advance(3000); stopped(); send("\n");
  assert(!gyro_calibrated); // no delayed motion after cancelled calibration
  std::cout << "Rover 2 guided calibration checks passed\n";
}
