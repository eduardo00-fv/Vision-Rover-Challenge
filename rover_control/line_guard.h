#pragma once
#include <math.h>
#include <stdint.h>

// Borde de la superficie con los IR digitales sobre un tablero de ajedrez.
//
// Negro o blanco no dice nada por sí solo: la cancha está hecha de los dos. Lo
// que sí distingue la cancha de lo de afuera es que, al moverse sobre ella, cada
// sensor cambia de color cada 20-28 mm (~51 mm en el peor caso, rozando
// esquinas). Un sensor que se alejó más de `edge_mm` del lugar donde cambió por
// última vez está sobre otra cosa (lona, mesa o vacío), sin importar qué lee.
//
// Se mide la DISTANCIA desde ese ancla, no el camino sumado: el ruido de la
// pose de la cámara (1-2 mm y ~1° por captura) sumado captura tras captura
// parecería recorrido con el rover quieto. El giro cuenta como el arco que
// barre un sensor a `sensor_radius_mm` del centro.
//
// Excepción geométrica: avanzando en direcciones que cruzan la red de esquinas
// con período corto —la diagonal (45°) y la pendiente 1/2 (26,6° y 63,4°)—, un
// sensor que pasa rozando las esquinas ve rachas de un color mucho más largas
// (en la diagonal exacta, infinitas). Cerca de esas direcciones el umbral se
// multiplica por `kDiagonalFactor`: el borde se detecta más tarde, pero sin
// falsas alarmas. Medido en el simulador (200 corridas): 36 retrocesos falsos
// con umbral fijo, 6 con solo 45° y 0 con las tres bandas. La orientación es
// relativa a los ejes de la cancha, que son los del tablero impreso.
class LineGuard {
 public:
  enum class Action : uint8_t { NONE, BACK_OFF, STOP };

  void configure(float edge_mm, float sensor_radius_mm, uint8_t front_mask, uint8_t rear_mask) {
    edge_mm_ = edge_mm; radius_mm_ = sensor_radius_mm; front_ = front_mask; rear_ = rear_mask;
  }

  // Lectura cruda de los 4 sensores (bit i = HIGH en el sensor i).
  void sample(uint8_t mask) {
    const uint8_t toggled = have_mask_ ? static_cast<uint8_t>(mask ^ mask_) : 0x0F;
    for (uint8_t i = 0; i < 4; ++i)
      if (toggled & (1U << i)) anchor(i);
    mask_ = mask; have_mask_ = true;
  }

  // Pose propia de una captura nueva. Un salto imposible (marcador confundido,
  // pose recuperada tras moverse a ciegas) reinicia las anclas: no es recorrido.
  void pose(float x_mm, float y_mm, float theta_deg) {
    // Se compara con la última pose conocida aunque se haya perdido en el medio.
    const bool jump = seen_ && (hypotf(x_mm - x_, y_mm - y_) > kMaxJumpMm ||
                                angle(theta_deg, th_) > kMaxJumpDeg);
    x_ = x_mm; y_ = y_mm; th_ = theta_deg; have_pose_ = seen_ = true;
    for (uint8_t i = 0; i < 4; ++i)
      if (jump || !anchored_[i]) anchor(i);
  }

  // Pose vencida: no se decide borde hasta tener otra.
  void losePose() { have_pose_ = false; }

  float distance(uint8_t i) const {
    if (!have_pose_ || !anchored_[i]) return 0;
    return hypotf(x_ - ax_[i], y_ - ay_[i]) + angle(th_, ath_[i]) * 0.01745329F * radius_mm_;
  }

  // Umbral para la orientación actual.
  float threshold() const {
    const float m = fmodf(fabsf(th_), 90.0F);
    const bool diagonal = fabsf(m - 45.0F) < kDiagonalBandDeg || fabsf(m - 26.57F) < kSlopeBandDeg ||
                          fabsf(m - 63.43F) < kSlopeBandDeg;
    return diagonal ? edge_mm_ * kDiagonalFactor : edge_mm_;
  }

  uint8_t offMask() const {
    const float limit = threshold();
    uint8_t off = 0;
    for (uint8_t i = 0; i < 4; ++i)
      if (distance(i) > limit) off |= static_cast<uint8_t>(1U << i);
    return off;
  }

  // Qué hacer con la orden del navegador. Adelante afuera: retroceder, que es lo
  // único que devuelve el sensor a la cancha. Atrás afuera y se va a retroceder:
  // parar. Los dos lados afuera: parar.
  Action action(bool active, bool reverse) const {
    const uint8_t off = offMask();
    const bool front = off & front_, rear = off & rear_;
    if (front && rear) return Action::STOP;
    if (front && active && !reverse) return Action::BACK_OFF;
    if (rear && active && reverse) return Action::STOP;
    return Action::NONE;
  }

 private:
  static constexpr float kMaxJumpMm = 80;
  static constexpr float kMaxJumpDeg = 60;
  static constexpr float kDiagonalBandDeg = 12;
  static constexpr float kSlopeBandDeg = 6;
  static constexpr float kDiagonalFactor = 2;

  static float angle(float a, float b) {
    float d = fabsf(a - b);
    while (d > 360) d -= 360;
    return d > 180 ? 360 - d : d;
  }
  void anchor(uint8_t i) {
    anchored_[i] = have_pose_;
    ax_[i] = x_; ay_[i] = y_; ath_[i] = th_;
  }

  float edge_mm_ = 60, radius_mm_ = 60;
  uint8_t front_ = 0, rear_ = 0, mask_ = 0;
  bool have_mask_ = false, have_pose_ = false, seen_ = false;
  float x_ = 0, y_ = 0, th_ = 0;
  bool anchored_[4] = {false, false, false, false};
  float ax_[4] = {0, 0, 0, 0}, ay_[4] = {0, 0, 0, 0}, ath_[4] = {0, 0, 0, 0};
};
