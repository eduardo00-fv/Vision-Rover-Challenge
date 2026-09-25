#include "../diagnostico/heading_control.h"
#include <cassert>
#include <limits>
#include <iostream>

int main() {
  HeadingControl c;
  assert(!c.begin(18, 20, 0.53F, 0, 0));
  assert(!c.begin(26, 20, 0.53F, 1, 0));
  assert(!c.begin(18, 20, 6, 1, 0));
  assert(c.begin(18, 20, 0.53F, 1, 0));
  assert(c.update(0.53F, 10000));
  assert(c.yaw == 0 && c.left == 18 && c.right == 20);
  assert(c.update(10.53F, 20000));
  assert(c.yaw > 0 && c.left > 18 && c.right < 20);
  assert(c.begin(18, 20, 0.53F, -1, 0));
  assert(c.update(10.53F, 10000));
  assert(c.yaw < 0 && c.left < 18 && c.right > 20);
  assert(!c.update(0.53F, 110001)); // stale / scheduler stall
  assert(c.begin(18, 20, 0, 1, UINT32_MAX - 4999));
  assert(c.update(0, 5000)); // timer wrap
  assert(!c.update(std::numeric_limits<float>::quiet_NaN(), 15000));
  assert(c.begin(18, 20, 0, 1, 0));
  assert(!c.update(200, 10000));
  assert(c.begin(18, 20, 0, 1, 0));
  assert(c.update(100, 100000));
  assert(c.correction == 4 && c.left == 22 && c.right == 16);
  assert(c.update(100, 200000));
  assert(!c.update(100, 300000)); // accumulated heading safety limit
  c.kp = std::numeric_limits<float>::infinity();
  assert(!c.begin(18, 20, 0, 1, 0));
  std::cout << "Heading PD checks passed\n";
}
