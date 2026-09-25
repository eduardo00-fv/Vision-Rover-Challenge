#include "../line_guard.h"

#include <cassert>
#include <cmath>
#include <iostream>

namespace {
// Color de la casilla bajo un punto del tablero (celdas de 20 mm); afuera, blanco fijo.
uint8_t color(float x, float y, float board_mm) {
  if (x < 0 || y < 0 || x > board_mm || y > board_mm) return 0;
  return (static_cast<int>(std::floor(x / 20)) + static_cast<int>(std::floor(y / 20))) & 1;
}

// Rover que avanza en recta con un sensor 50 mm adelante (S0) y otro 50 mm atrás (S1).
// S2 y S3 no se simulan: quedan fijos, como un sensor trabado, y por eso se
// excluyen de las máscaras. Un sensor trabado en una máscara pediría retroceder
// siempre: la prueba en la arena tiene que dejarlo afuera.
struct Walk {
  LineGuard guard;
  float x, y, th;
  Walk(float x0, float y0, float th0) : x(x0), y(y0), th(th0) {
    guard.configure(60, 60, 0b0001, 0b0010);
  }
  void step(float mm, float board = 1000) {
    const float c = std::cos(th * 0.0174533F), s = std::sin(th * 0.0174533F);
    x += mm * c; y -= mm * s;
    const uint8_t m = static_cast<uint8_t>(color(x + 50 * c, y - 50 * s, board) |
                                           (color(x - 50 * c, y + 50 * s, board) << 1));
    guard.sample(m);
    guard.pose(x, y, th);
  }
};
}  // namespace

int main() {
  // Sobre el tablero, en recta y en diagonal, nunca declara borde. Con un sensor
  // puntual la racha más larga de un color es ~51 mm (cerca de 45°, rozando
  // esquinas); la única excepción es la diagonal EXACTA por las esquinas, que se
  // evita acá con el desfase de 7/3 mm y que en la cancha real rompe el ruido.
  for (float th : {0.0F, 30.0F, 45.0F, 50.0F, 90.0F, 137.0F}) {
    Walk w(507, 503, th);
    for (int i = 0; i < 100; ++i) {
      w.step(3);
      assert((w.guard.offMask() & 0b0011) == 0);
      assert(w.guard.action(true, false) == LineGuard::Action::NONE);
    }
  }

  // Hacia el borde derecho: el sensor delantero sale primero y pide retroceder.
  {
    Walk w(850, 510, 0);
    int steps = 0;
    while (w.guard.action(true, false) == LineGuard::Action::NONE && steps < 200) { w.step(3); ++steps; }
    assert(w.guard.action(true, false) == LineGuard::Action::BACK_OFF);
    assert((w.guard.offMask() & 0b0011) == 0b0001);
    const float sensor_x = w.x + 50;
    assert(sensor_x > 1000 && sensor_x < 1000 + 60 + 20);  // detectado a menos de 80 mm del borde
    // Retroceder está permitido; al volver a la cancha el sensor cambia y se limpia.
    assert(w.guard.action(true, true) == LineGuard::Action::NONE);
    for (int i = 0; i < 40; ++i) w.step(-3);
    assert((w.guard.offMask() & 0b0011) == 0);
  }

  // Quieto no acumula: sin movimiento nunca hay borde, aunque la lectura no cambie.
  {
    LineGuard g;
    g.configure(60, 60, 1, 2);
    for (int i = 0; i < 1000; ++i) { g.sample(0); g.pose(100, 100, 0); }
    assert(g.offMask() == 0);
  }

  // Quieto con el ruido real de la cámara (±2 mm, ±1,5°): el ruido no se suma.
  {
    LineGuard g;
    g.configure(60, 60, 1, 2);
    unsigned seed = 1;
    auto noise = [&seed](float amp) { seed = seed * 1103515245U + 12345U; return amp * (((seed >> 16) & 0x7FFF) / 16383.5F - 1); };
    g.sample(0);
    for (int i = 0; i < 20000; ++i) { g.pose(300 + noise(2), 300 + noise(2), 90 + noise(1.5F)); }
    assert(g.offMask() == 0);
  }

  // Un salto de pose (marcador perdido y recuperado) no cuenta como recorrido.
  {
    LineGuard g;
    g.configure(60, 60, 1, 2);
    g.sample(0); g.pose(100, 100, 0);
    g.pose(400, 100, 0);
    assert(g.offMask() == 0);
    g.losePose();
    g.pose(100, 100, 0);
    assert(g.offMask() == 0);
  }

  // Atrás afuera: retroceder se frena; los dos lados afuera frenan todo.
  {
    LineGuard g;
    g.configure(60, 60, 1, 2);
    g.sample(0); g.pose(0, 0, 0);
    for (int i = 1; i <= 30; ++i) { g.sample(1); g.pose(i * 3.0F, 0, 0); }  // S0 fijo en 1, S1 fijo en 0
    assert((g.offMask() & 0b0011) == 0b0011);
    assert(g.action(true, false) == LineGuard::Action::STOP);
  }
  {
    LineGuard g;
    g.configure(60, 60, 1, 2);
    uint8_t m = 0;
    g.sample(m); g.pose(0, 0, 0);
    for (int i = 1; i <= 30; ++i) { m ^= (i % 7 == 0) ? 1 : 0; g.sample(m); g.pose(i * 3.0F, 0, 0); }
    assert((g.offMask() & 0b0011) == 0b0010);
    assert(g.action(true, true) == LineGuard::Action::STOP);
    assert(g.action(true, false) == LineGuard::Action::NONE);
  }

  // Giro en sitio: los sensores barren arco (radio 60 mm) y eso también es recorrido.
  {
    LineGuard g;
    g.configure(60, 60, 1, 2);
    g.sample(0); g.pose(0, 0, 0);
    g.pose(0, 0, 40);  // 40° * 60 mm ≈ 42 mm
    assert(g.offMask() == 0);
    g.pose(0, 0, 80);  // ≈ 84 mm sin cambios
    assert(g.offMask() == 0b1111);
  }
  std::cout << "Line guard checks passed\n";
}
