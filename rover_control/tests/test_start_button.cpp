#include "../start_button.h"

#include <cassert>
#include <iostream>

int main() {
  StartButton button;
  assert(!button.update(false, false, 0));
  assert(!button.update(true, false, 100));   // rebote: aún no
  assert(!button.update(false, false, 120));
  assert(!button.update(true, false, 200));
  assert(button.update(true, false, 250));    // 50 ms sostenido: armado
  assert(button.update(false, false, 300));   // soltarlo no desarma
  assert(!button.update(false, true, 400));   // fin de ronda: desarma
  assert(!button.update(true, true, 500));    // no se arma con la ronda terminada
  assert(!button.update(true, false, 600));   // la siguiente ronda exige otra pulsación completa
  assert(button.update(true, false, 650));
  std::cout << "Start button checks passed\n";
}
