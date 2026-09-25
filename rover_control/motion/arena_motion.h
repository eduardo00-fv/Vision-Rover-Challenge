#pragma once
#include "rover_profile.h"
#include "lsm6ds3_gyro.h"

// Cooperative, stop-and-observe arena controller. Vision supplies a relative
// target at entry; gyro integrates only the bounded maneuver, never global pose.
template<class Bus> class ArenaMotion {
 public:
  // ~64 mm/s medidos a 20/20; con él se acorta el tramo cerca del objetivo.
  static constexpr float kNominalMmPerS = 60.0F;
  static constexpr uint32_t kMinDriveUs = 120000, kMaxDriveUs = 350000;
  // Un giro trabado (cubo, borde, otro rover) se reintenta con otra captura;
  // solo varios seguidos enclavan la parada.
  static constexpr unsigned kMaxConsecutiveStalls = 4;
  // Sin IMU: giro a tiempo con la tasa medida a 25 % (46–62 °/s, se toma la
  // alta para quedarse corto) y tramos cortos; la cámara corrige cada uno.
  static constexpr float kBlindTurnDps = 60.0F, kBlindTurnMaxDeg = 45.0F;
  static constexpr uint32_t kBlindTurnMinUs = 80000, kBlindMaxDriveUs = 500000;
  enum class State { CALIBRATING, IDLE, DRIVE, TURN, SETTLE, FAULT };
  explicit ArenaMotion(Bus& bus, const rover_motion::Profile& profile)
      : gyro_(bus), profile_(profile) {}
  // Pausa tras cada maniobra (inercia + una captura nueva) y tramo máximo a
  // ciegas. Menos pausa y tramos más largos = más rápido, a cambio de decidir
  // con una pose más vieja: validar en la arena antes de bajarlos.
  void timing(uint32_t settle_ms, uint32_t max_drive_ms) {
    settle_us_ = settle_ms * 1000;
    max_drive_us_ = max_drive_ms * 1000 < kMinDriveUs ? kMinDriveUs : max_drive_ms * 1000;
  }
  float left = 0, right = 0;
  State state = State::FAULT;
  const char* reason = "NOT_STARTED";
  // Con la IMU caída (o ausente al arrancar) seguir en modo solo-visión en vez
  // de enclavar FAULT. Un perfil sin validar nunca se mueve.
  void allowVisionOnly(bool allow) { allow_vision_only_ = allow; }
  bool visionOnly() const { return vision_only_; }
  const char* imuReason() const { return imu_reason_; }  // por qué pasó a solo-visión
  void begin(uint32_t now) {
    left = right = 0;
    vision_only_ = false; stalls_ = 0;
    if (!profile_.initially_validated) { fail("PROFILE"); return; }
    profile_.configure(heading_, turn_);
    if (!gyro_.configure()) { imuLost(now, "IMU_ABSENT"); return; }
    started_ = last_ = now; count_ = 0; sum_ = 0; low_ = 1000; high_ = -1000;
    state = State::CALIBRATING; reason = "KEEP_STILL";
  }
  void cancel() {
    left = right = 0;
    if (state != State::FAULT && state != State::CALIBRATING) state = State::IDLE;
  }
  void fail(const char* why) { left = right = 0; state = State::FAULT; reason = why; }
  // Call before and after any potentially slow peripheral operation.
  void guard(uint32_t now) {
    if (state == State::FAULT) return;
    if (!vision_only_ && now - last_ > 100000) imuLost(now, "IMU_STALE");
    if ((state == State::DRIVE || state == State::TURN) && now - started_ >= duration_)
      finish(now);
  }
  void sample(uint32_t now) {
    guard(now);
    if (state == State::FAULT || vision_only_) return;
    float raw = 0;
    const int result = gyro_.sample(raw);
    if (result < 0) { imuLost(now, "IMU_READ"); return; }
    if (!result) return;
    const uint32_t dt = now - last_; last_ = now;
    if (state == State::CALIBRATING) {
      if (now - started_ > 4000000) { imuLost(now, "CAL_TIMEOUT"); return; }
      if (now - started_ < 250000) return;
      sum_ += raw; if (raw < low_) low_ = raw; if (raw > high_) high_ = raw;
      if (++count_ == 200) {
        bias_ = sum_ / count_;
        if (std::fabs(bias_) > 5 || high_ - low_ > 1) { fail("CAL_MOVING"); return; }
        state = State::IDLE; reason = "READY";
      }
      return;
    }
    const float rate = profile_.gyro_sign * (raw - bias_);
    if (!std::isfinite(rate) || std::fabs(rate) > 200) { imuLost(now, "IMU_RATE"); return; }
    if (state == State::DRIVE) {
      if (!heading_.update(raw, now)) { fail("HEADING"); return; }
      if (reverse_) {  // mismas correcciones: el diferencial gira igual hacia atrás
        left = (-heading_.base_left + heading_.correction) / 100;
        right = (-heading_.base_right - heading_.correction) / 100;
      } else {
        left = heading_.left / 100; right = heading_.right / 100;
      }
    } else if (state == State::TURN) {
      yaw_ += rate * (dt * 1e-6F);
      const auto action = turn_.update(yaw_, rate, (now - started_) / 1000);
      if (action == TurnControl::Action::FAULT) { stall(now, "TURN_STALL"); return; }
      if (action == TurnControl::Action::CUT) { finish(now); return; }
      right = (turn_.target > 0 ? 1 : -1) * turn_.power / 100; left = -right;
    }
  }
  void command(bool allowed, bool active, float error, uint8_t target, bool pushing,
               uint64_t capture, uint32_t now, bool reverse = false, float distance_mm = 1e9F) {
    guard(now);
    if (!allowed || !active) { cancel(); return; }
    if (state == State::FAULT || state == State::CALIBRATING) return;
    if (state == State::SETTLE) {
      // Record captures throughout coast; demand another capture after it ends.
      if (now - started_ < settle_us_) { capture_ = capture; return; }
      if (capture == capture_) return;
      state = State::IDLE;
    }
    if (state == State::DRIVE || state == State::TURN) {
      if (target != target_ || pushing != pushing_ || reverse != reverse_) { finish(now); capture_ = capture; }
      return;
    }
    if (!std::isfinite(error) || std::fabs(error) > 180) { fail("TARGET"); return; }
    target_ = target; pushing_ = pushing; reverse_ = reverse; capture_ = capture; started_ = now; yaw_ = 0;
    if (vision_only_) { blind(error, reverse, distance_mm); return; }
    if (!reverse && std::fabs(error) > 10) {
      if (!turn_.begin(error, 0)) { fail("TURN_CONFIG"); return; }
      duration_ = TurnControl::timeoutMs(error) * 1000; state = State::TURN;
    } else {
      if (!heading_.begin(profile_.left, profile_.right, bias_, profile_.gyro_sign, now)) {
        fail("DRIVE_CONFIG"); return;
      }
      heading_.yaw = reverse ? 0 : -error;
      const float us = std::isfinite(distance_mm) ? distance_mm / kNominalMmPerS * 1e6F : max_drive_us_;
      duration_ = us < kMinDriveUs ? kMinDriveUs : (us > max_drive_us_ ? max_drive_us_ : static_cast<uint32_t>(us));
      state = State::DRIVE;
    }
    // PWM begins only on the next fresh IMU sample.
    left = right = 0;
  }
 private:
  void imuLost(uint32_t now, const char* why) {
    if (!allow_vision_only_) { fail(why); return; }
    // Detenerse y esperar una captura nueva, como tras cualquier maniobra.
    vision_only_ = true; imu_reason_ = why; reason = "VISION_ONLY";
    left = right = 0; state = State::SETTLE; started_ = now;
  }
  // Maniobra a lazo abierto: PWM del perfil durante un tiempo acotado.
  void blind(float error, bool reverse, float distance_mm) {
    if (!reverse && std::fabs(error) > 10) {
      const float degrees = std::fabs(error) < kBlindTurnMaxDeg ? std::fabs(error) : kBlindTurnMaxDeg;
      const float us = degrees / kBlindTurnDps * 1e6F;
      duration_ = us < kBlindTurnMinUs ? kBlindTurnMinUs : static_cast<uint32_t>(us);
      right = (error > 0 ? 1 : -1) * profile_.turn_start / 100; left = -right;
      state = State::TURN;
      return;
    }
    const float cap = max_drive_us_ < kBlindMaxDriveUs ? max_drive_us_ : kBlindMaxDriveUs;
    const float us = std::isfinite(distance_mm) ? distance_mm / kNominalMmPerS * 1e6F : cap;
    duration_ = us < kMinDriveUs ? kMinDriveUs : (us > cap ? static_cast<uint32_t>(cap) : static_cast<uint32_t>(us));
    const float sign = reverse ? -1.0F : 1.0F;
    left = sign * profile_.left / 100; right = sign * profile_.right / 100;
    state = State::DRIVE;
  }
  void finish(uint32_t now) {
    if (!vision_only_ && state == State::TURN && now - started_ >= duration_) { stall(now, "TURN_TIMEOUT"); return; }
    if (state == State::TURN || state == State::DRIVE) stalls_ = 0;
    left = right = 0; state = State::SETTLE; started_ = now;
  }
  void stall(uint32_t now, const char* why) {
    if (++stalls_ >= kMaxConsecutiveStalls) { fail(why); return; }
    reason = why; left = right = 0; state = State::SETTLE; started_ = now;
  }
  Lsm6ds3Gyro<Bus> gyro_;
  rover_motion::Profile profile_;
  HeadingControl heading_; TurnControl turn_;
  uint32_t started_ = 0, last_ = 0, duration_ = 0;
  uint32_t settle_us_ = 500000, max_drive_us_ = kMaxDriveUs;
  uint64_t capture_ = 0;
  unsigned count_ = 0, stalls_ = 0; uint8_t target_ = 0; bool pushing_ = false, reverse_ = false;
  float sum_ = 0, low_ = 0, high_ = 0, bias_ = 0, yaw_ = 0;
  bool allow_vision_only_ = false, vision_only_ = false;
  const char* imu_reason_ = "";
};
