#include "../diagnostico/turn_control.h"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
  TurnControl c;
  using A = TurnControl::Action;
  assert(!c.begin(0, 0) && !c.begin(181, 0) && !c.begin(-181, 0) && !c.begin(-9, 0));
  assert(TurnControl::timeoutMs(90) == 3500 && TurnControl::timeoutMs(-90) == 3500);
  assert(TurnControl::timeoutMs(180) == 7000 && TurnControl::timeoutMs(-180) == 7000);
  for (float direction : {1.0F, -1.0F}) {
    assert(c.begin(direction * 180, 0));
    for (unsigned i = 1; i <= 17; ++i)
      assert(c.update(direction * i * 10, direction * 50, i * 200) == A::RUN);
    assert(c.update(direction * 178, direction * 40, 3600) == A::CUT);
  }
  assert(!c.begin(std::numeric_limits<float>::quiet_NaN(), 0));
  assert(c.begin(45, 0));
  assert(c.update(5, 45, 100) == A::RUN && c.power == 25);
  assert(c.update(35, 40, 200) == A::RUN && c.power < 25 && c.power >= 22);
  assert(c.update(44, 40, 300) == A::CUT);
  assert(c.update(44.22F, 0, 400) == A::CUT); // physical near-target stall regression
  assert(c.begin(-45, 0));
  assert(c.update(-35, -60, 100) == A::RUN);
  assert(c.update(-42, -60, 200) == A::CUT);
  assert(c.begin(45, 0));
  assert(c.update(-6, -10, 100) == A::FAULT);
  assert(c.begin(45, 0));
  assert(c.update(0, 0, 751) == A::FAULT);
  assert(c.begin(45, UINT32_MAX - 99));
  assert(c.update(5, 30, 100) == A::RUN);
  assert(c.update(5, 0, 851) == A::FAULT);
  assert(c.begin(45, 0));
  assert(c.update(46, 0, 100) == A::CUT);
  assert(c.update(1, std::numeric_limits<float>::infinity(), 100) == A::FAULT);
  std::cout << "Turn controller checks passed\n";
}
