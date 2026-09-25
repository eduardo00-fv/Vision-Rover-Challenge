/* Diagnóstico físico del CenfoBot — Arduino C++ / ESP32.
 * Sin Wi-Fi, cámara ni telemetría.
 * Monitor serial: 115200. Motores desarmados al arrancar.
 */
#include <Wire.h>
#include <FastLED.h>
#include "heading_control.h"
#include "turn_control.h"
#include "../motion/lsm6ds3_gyro.h"
#include "../motion/rover_profile.h"

#ifdef VRC_CALIBRATION_GUIDE
bool guidedCommand(const String& line);
#endif

// Cableado físico confirmado por el usuario. M1/M2: driver IdeaBoard.
constexpr uint8_t M1_FORWARD = 12, M1_BACKWARD = 14;
constexpr uint8_t M2_FORWARD = 13, M2_BACKWARD = 15;
constexpr uint8_t LINE_PINS[] = {4, 5, 18, 19};
constexpr uint8_t SONAR_TRIG = 27, SONAR_ECHO = 33;
constexpr uint8_t COLOR_LED_PIN = 23, COLOR_INPUT_PIN = 32;
constexpr uint32_t PWM_HZ = 1000;
constexpr uint8_t PWM_BITS = 12;
constexpr uint32_t PWM_MAX = (1U << PWM_BITS) - 1U;
constexpr int MAX_PERCENT = 35;
constexpr uint32_t MAX_MOVE_MS = 3500;
constexpr uint32_t SONAR_TIMEOUT_US = 30000;
constexpr uint32_t COLOR_SETTLE_MS = 200;

CRGB color_led[1];
bool armed = false, moving = false;
bool pwm_ready = false;
uint32_t stop_at_ms = 0;
uint8_t lsm_address = 0;
Lsm6ds3Gyro<decltype(Wire)> imu(Wire);
enum class GyroJob { NONE, ZERO, WATCH, DRIVE, TURN, COAST };
GyroJob gyro_job = GyroJob::NONE;
HeadingControl heading;
bool gyro_calibrated = false;
int gyro_sign = 0;
float gyro_bias = 0, gyro_sum = 0, gyro_min = 0, gyro_max = 0, watch_yaw = 0;
uint16_t gyro_count = 0;
uint32_t gyro_started_ms = 0, gyro_end_ms = 0, gyro_last_us = 0;
uint32_t gyro_poll_us = 0, gyro_print_ms = 0, gyro_calibrated_ms = 0;
float turn_yaw = 0, turn_cut_yaw = 0;
TurnControl turn_control;
// Diagnostic defaults use rover 1's measured profile; rover 2 overrides only
// after its guided wheel/sign checks.
#ifdef VRC_CALIBRATION_GUIDE
const rover_motion::Profile motion_profile = rover_motion::rover2;
#else
const rover_motion::Profile motion_profile = rover_motion::rover1;
#endif
bool turn_closed = false, turn_timed_out = false;
uint32_t turn_stable_since_ms = 0;

void writeWheel(uint8_t forward, uint8_t backward, float percent) {
  const uint32_t duty = static_cast<uint32_t>(fabsf(percent) * PWM_MAX / 100.0F);
  // Apagar ambas ramas antes de cambiar de sentido.
  ledcWrite(forward, 0);
  ledcWrite(backward, 0);
  if (percent > 0) ledcWrite(forward, duty);
  if (percent < 0) ledcWrite(backward, duty);
}

void stopMotors() {
  writeWheel(M1_FORWARD, M1_BACKWARD, 0);
  writeWheel(M2_FORWARD, M2_BACKWARD, 0);
  moving = false;
  stop_at_ms = 0;
  gyro_job = GyroJob::NONE;
}

void runMotor(int left, int right, uint32_t duration_ms) {
  if (!armed) { Serial.println("ERROR: escriba ARM antes de MOTOR"); return; }
  if (!pwm_ready || left < -MAX_PERCENT || left > MAX_PERCENT || right < -MAX_PERCENT || right > MAX_PERCENT || duration_ms == 0 || duration_ms > MAX_MOVE_MS) {
    Serial.printf("ERROR: PWM -%d..%d y duración 1..%lu ms\n", MAX_PERCENT, MAX_PERCENT, MAX_MOVE_MS);
    return;
  }
  writeWheel(M1_FORWARD, M1_BACKWARD, left);
  writeWheel(M2_FORWARD, M2_BACKWARD, right);
  stop_at_ms = millis() + duration_ms;
  moving = true;
  armed = false;  // ARM autoriza una sola prueba.
  Serial.printf("MOTOR M1=%d M2=%d ms=%lu\n", left, right, duration_ms);
}

void readLines(uint16_t samples) {
  samples = constrain(samples, 1, 50);
  uint32_t total[4] = {0, 0, 0, 0};
  for (uint16_t s = 0; s < samples; ++s) {
    for (uint8_t i = 0; i < 4; ++i) total[i] += digitalRead(LINE_PINS[i]) == HIGH;
    delay(2);
  }
  Serial.printf("LINE n=%u HIGH: S1=%lu S2=%lu S3=%lu S4=%lu\n", samples,
                total[0], total[1], total[2], total[3]);
  Serial.println("Conteos digitales HIGH; compare sobre claro/oscuro para identificar polaridad. GPIO19 no tiene ADC.");
}

void readSonar(uint16_t samples) {
  samples = constrain(samples, 1, 30);
  uint16_t valid = 0;
  float total = 0.0F, minimum = 100000.0F, maximum = 0.0F;
  for (uint16_t s = 0; s < samples; ++s) {
    digitalWrite(SONAR_TRIG, LOW); delayMicroseconds(3);
    digitalWrite(SONAR_TRIG, HIGH); delayMicroseconds(10); digitalWrite(SONAR_TRIG, LOW);
    const uint32_t microseconds = pulseIn(SONAR_ECHO, HIGH, SONAR_TIMEOUT_US);
    if (microseconds != 0) {
      const float cm = microseconds / 58.0F;
      total += cm; minimum = min(minimum, cm); maximum = max(maximum, cm); ++valid;
    }
    delay(60);
  }
  if (!valid) Serial.println("SONAR: sin eco; revise jumper SELECT-Vin, TRIG/ECHO y nivel de ECHO a 3.3 V");
  else Serial.printf("SONAR n=%u/%u cm_mean=%.1f min=%.1f max=%.1f\n", valid, samples, total / valid, minimum, maximum);
}

uint16_t sampleColor(const CRGB& color, uint16_t samples) {
  color_led[0] = color; FastLED.show(); delay(COLOR_SETTLE_MS);
  uint32_t total = 0;
  for (uint16_t s = 0; s < samples; ++s) { total += analogRead(COLOR_INPUT_PIN); delay(5); }
  color_led[0] = CRGB::Black; FastLED.show();
  return total / samples;
}

void readColor(uint16_t samples) {
  samples = constrain(samples, 1, 50);
  const uint16_t red = sampleColor(CRGB::Red, samples);
  const uint16_t green = sampleColor(CRGB::Green, samples);
  const uint16_t blue = sampleColor(CRGB::Blue, samples);
  const char* detected = red <= green && red <= blue ? "RED" : (green <= blue ? "GREEN" : "BLUE");
  Serial.printf("COLOR n=%u R=%u G=%u B=%u detected=%s (menor lectura, según ejemplo oficial)\n", samples, red, green, blue, detected);
}

bool lsmRead(uint8_t reg, uint8_t* data, size_t count) {
  return imu.read(reg, data, count);
}

bool initImu() {
  if (imu.detect()) {
    lsm_address = imu.address();
    Serial.printf("IMU LSM6DS3TR-C en 0x%02X\n", lsm_address);
    return true;
  }
  lsm_address = 0; Serial.println("IMU no encontrada; ejecute I2C y revise Qwiic"); return false;
}

bool gyroWrite(uint8_t reg, uint8_t value) {
  return imu.write(reg, value);
}

// LSM6DS3TR-C datasheet: 104 Hz, 250 dps, BDU + address increment.
bool configureGyro() {
  if (!lsm_address && !initImu()) return false;
  return imu.configure();
}

// -1: I2C error; 0: no NEW sample; 1: new sample. Read XYZ to release BDU.
int freshGyro(float& z) {
  return imu.sample(z);
}

bool gyroCalibrationFresh() {
  return gyro_calibrated && millis() - gyro_calibrated_ms <= 60000;
}

void startGyroZero() {
  stopMotors(); armed = false; gyro_calibrated = false;
  if (!configureGyro()) { Serial.println("ERROR IMU config; motores detenidos"); return; }
  gyro_job = GyroJob::ZERO;
  gyro_started_ms = millis(); gyro_end_ms = gyro_started_ms + 4000;
  gyro_last_us = micros(); gyro_poll_us = gyro_last_us;
  gyro_sum = 0; gyro_count = 0; gyro_min = 1e6F; gyro_max = -1e6F;
  Serial.println("GZERO: mantener QUIETO; calentamiento 250 ms + 200 muestras nuevas; ! cancela");
}

void startGyroWatch(int duration) {
  stopMotors(); armed = false;
  if (!gyroCalibrationFresh() || duration < 1000 || duration > 10000) {
    Serial.println("ERROR: GZERO reciente requerido; GYRO <1000..10000 ms>"); return;
  }
  float unused;
  if (freshGyro(unused) < 0) { gyro_calibrated = false; Serial.println("ERROR IMU"); return; }
  gyro_job = GyroJob::WATCH;
  gyro_started_ms = millis(); gyro_end_ms = gyro_started_ms + duration;
  gyro_last_us = micros(); gyro_poll_us = gyro_last_us; gyro_print_ms = millis();
  watch_yaw = 0;
  Serial.println("GYRO: motores APAGADOS; girar a mano; yaw_raw positivo/negativo sin GSIGN");
}

void startStraight(float left, float right, int duration) {
  if (!armed) { Serial.println("ERROR: ARM antes de STRAIGHT"); return; }
  armed = false;
  if (!pwm_ready || !gyroCalibrationFresh() || duration < 1 || duration > MAX_MOVE_MS ||
      !heading.begin(left, right, gyro_bias, gyro_sign, micros())) {
    Serial.println("ERROR: GZERO reciente, GSIGN confirmado; bases 10..25 y ms 1..3500"); return;
  }
  // Discard an old latched sample before starting; next sample must arrive promptly.
  float unused;
  if (freshGyro(unused) < 0) { gyro_calibrated = false; Serial.println("ERROR IMU"); return; }
  Serial.printf("STRAIGHT M1=%.1f M2=%.1f ms=%d kp=%.2f kd=%.2f limit=%.1f\n",
      left, right, duration, heading.kp, heading.kd, heading.limit);
  gyro_last_us = micros(); gyro_poll_us = gyro_last_us; heading.last_us = gyro_last_us;
  gyro_print_ms = millis(); stop_at_ms = millis() + duration;
  gyro_job = GyroJob::DRIVE; moving = true;
  writeWheel(M1_FORWARD, M1_BACKWARD, left); writeWheel(M2_FORWARD, M2_BACKWARD, right);
}

void startTurnTest(int left, int right, int duration, bool closed) {
  if (!armed) { Serial.println("ERROR: ARM antes de TURNTEST"); return; }
  armed = false;
  if (!pwm_ready || !gyroCalibrationFresh() || (gyro_sign != 1 && gyro_sign != -1) ||
      left < -MAX_PERCENT || left > MAX_PERCENT || right < -MAX_PERCENT || right > MAX_PERCENT ||
      !((left < 0 && right > 0) || (left > 0 && right < 0)) ||
      duration < 1 || static_cast<uint32_t>(duration) > (closed ? TurnControl::timeoutMs(turn_control.target) : MAX_MOVE_MS)) {
    Serial.println("ERROR: TURNTEST requiere GZERO, GSIGN, ruedas opuestas +/-35 y ms 1..3500"); return;
  }
  float unused;
  if (freshGyro(unused) < 0) { gyro_calibrated = false; Serial.println("ERROR IMU"); return; }
  if (!closed) Serial.printf("TURNTEST M1=%d M2=%d ms=%d; mide angulo, NO controla objetivo\n", left, right, duration);
  turn_yaw = turn_cut_yaw = 0;
  turn_closed = closed; turn_timed_out = false;
  gyro_last_us = micros(); gyro_poll_us = gyro_last_us; gyro_print_ms = millis();
  stop_at_ms = millis() + duration; gyro_job = GyroJob::TURN; moving = true;
  writeWheel(M1_FORWARD, M1_BACKWARD, left); writeWheel(M2_FORWARD, M2_BACKWARD, right);
}

void startTurn(float degrees) {
  if (!turn_control.begin(degrees, millis())) {
    armed = false; Serial.println("ERROR: TURN grados -180..-10 o 10..180"); return;
  }
  const uint32_t timeout_ms = TurnControl::timeoutMs(degrees);
  Serial.printf("TURN target=%.2f deg; max PWM 25, timeout %lu ms\n", degrees, timeout_ms);
  startTurnTest(degrees > 0 ? -25 : 25, degrees > 0 ? 25 : -25, timeout_ms, true);
}

void finishTurnPower(bool reached) {
  // PWM off FIRST. Continue measuring coast for 500 ms without re-arming.
  turn_cut_yaw = turn_yaw;
  turn_timed_out = turn_closed && !reached;
  stopMotors(); armed = false;
  gyro_job = GyroJob::COAST; gyro_end_ms = millis() + 500;
  turn_stable_since_ms = millis();
  Serial.printf("%s corte PWM; yaw_cut=%.2f deg; observando 500 ms sin motores\n",
      turn_closed ? "TURN" : "TURNTEST", turn_cut_yaw);
}

void gyroFault(const char* message) {
  stopMotors(); armed = false; gyro_calibrated = false;
  Serial.println(message);
}

void tickGyro() {
  if (gyro_job == GyroJob::NONE) return;
  const uint32_t now_ms = millis(), now_us = micros();
  if (gyro_job != GyroJob::DRIVE && gyro_job != GyroJob::TURN && static_cast<int32_t>(now_ms - gyro_end_ms) >= 0) {
    if (gyro_job == GyroJob::ZERO) gyroFault("ERROR GZERO: faltan muestras; repetir quieto");
    else if (gyro_job == GyroJob::COAST) {
      if (now_us - gyro_last_us > 100000) { gyroFault("ERROR IMU al finalizar TURNTEST; resultado invalido"); return; }
      stopMotors();
      if (turn_closed) {
        const float error = turn_yaw - turn_control.target;
        const char* status = turn_timed_out ? "TIMEOUT" :
            (now_ms - turn_stable_since_ms < 200 ? "UNSETTLED" : (fabsf(error) <= 3 ? "OK" : "OUTSIDE_TOLERANCE"));
        Serial.printf("TURN RESULT status=%s target=%.2f yaw=%.2f error=%.2f yaw_cut=%.2f coast=%.2f (relativo IMU)\n",
            status, turn_control.target, turn_yaw, error, turn_cut_yaw, turn_yaw - turn_cut_yaw);
      } else Serial.printf("TURNTEST RESULT yaw=%.2f deg yaw_cut=%.2f coast=%.2f (ventana +500 ms, relativo IMU)\n",
            turn_yaw, turn_cut_yaw, turn_yaw - turn_cut_yaw);
    }
    else { stopMotors(); Serial.printf("GYRO RESULT yaw_raw=%.2f deg\n", watch_yaw); }
    return;
  }
  if (gyro_job == GyroJob::ZERO && now_ms - gyro_started_ms < 250) return;
  if (now_us - gyro_poll_us < 2000) return;
  gyro_poll_us = now_us;
  float raw;
  const int result = freshGyro(raw);
  const uint32_t sampled_us = micros();
  if (result < 0) { gyroFault("ERROR IMU I2C; motores detenidos"); return; }
  if (gyro_job != GyroJob::ZERO && sampled_us - gyro_last_us > 100000) {
    gyroFault("ERROR IMU sin muestras frescas por 100 ms; motores detenidos"); return;
  }
  if (!result) return;
  if (gyro_job == GyroJob::ZERO) {
    // Discard the first latched sample after warmup.
    if (!gyro_count) { gyro_count = 1; return; }
    gyro_sum += raw; gyro_min = min(gyro_min, raw); gyro_max = max(gyro_max, raw);
    if (++gyro_count == 201) {
      gyro_bias = gyro_sum / 200;
      gyro_calibrated = fabsf(gyro_bias) <= 5 && gyro_max - gyro_min <= 1;
      gyro_calibrated_ms = millis(); stopMotors();
      Serial.printf("GZERO %s bias=%.4f dps spread=%.3f n=200; validez 60 s\n",
          gyro_calibrated ? "OK" : "RECHAZADO: movimiento/ruido", gyro_bias, gyro_max - gyro_min);
    }
    return;
  }
  if (gyro_job == GyroJob::DRIVE) {
    if (!heading.update(raw, sampled_us)) { gyroFault("ERROR rumbo/tasa/dt fuera de limite; motores detenidos"); return; }
    // An I2C operation may cross the motion deadline: never write PWM past it.
    if (static_cast<int32_t>(millis() - stop_at_ms) >= 0) {
      stopMotors(); Serial.println("RESULT corte automatico STRAIGHT"); return;
    }
    writeWheel(M1_FORWARD, M1_BACKWARD, heading.left);
    writeWheel(M2_FORWARD, M2_BACKWARD, heading.right);
  } else if (gyro_job == GyroJob::TURN || gyro_job == GyroJob::COAST) {
    const float rate = gyro_sign * (raw - gyro_bias);
    if (!std::isfinite(rate) || fabsf(rate) > 200) {
      gyroFault("ERROR TURNTEST tasa fuera de limite; resultado invalido"); return;
    }
    turn_yaw += rate * ((sampled_us - gyro_last_us) * 1e-6F);
    const float angle_limit = turn_closed ? fabsf(turn_control.target) + 15 : 180;
    if (!std::isfinite(turn_yaw) || fabsf(turn_yaw) > angle_limit) {
      gyroFault("ERROR TURNTEST angulo fuera de limite; resultado invalido"); return;
    }
    if (gyro_job == GyroJob::COAST && fabsf(rate) >= 3) turn_stable_since_ms = millis();
    if (gyro_job == GyroJob::TURN) {
      if (static_cast<int32_t>(millis() - stop_at_ms) >= 0) finishTurnPower(false);
      else if (turn_closed) {
        const auto action = turn_control.update(turn_yaw, rate, millis());
        if (action == TurnControl::Action::FAULT) {
          gyroFault("ERROR TURN: sin progreso o sentido/tasa incorrecto; motores detenidos"); return;
        }
        if (action == TurnControl::Action::CUT) finishTurnPower(true);
        else {
          const float signed_power = turn_control.target > 0 ? turn_control.power : -turn_control.power;
          writeWheel(M1_FORWARD, M1_BACKWARD, -signed_power);
          writeWheel(M2_FORWARD, M2_BACKWARD, signed_power);
        }
      }
    }
  } else {
    watch_yaw += (raw - gyro_bias) * ((sampled_us - gyro_last_us) * 1e-6F);
  }
  gyro_last_us = sampled_us;
  if (now_ms - gyro_print_ms >= 200) {
    gyro_print_ms = now_ms;
    char log[120];
    const int n = gyro_job == GyroJob::DRIVE ?
        snprintf(log, sizeof(log), "H yaw=%.2f rate=%.2f corr=%.2f L=%.2f R=%.2f\n",
          heading.yaw, gyro_sign * (raw - gyro_bias), heading.correction, heading.left, heading.right) :
        (gyro_job == GyroJob::TURN || gyro_job == GyroJob::COAST) ?
        snprintf(log, sizeof(log), "TURN yaw=%.2f rate=%.2f powered=%s\n", turn_yaw,
          gyro_sign * (raw - gyro_bias), moving ? "YES" : "NO") :
        snprintf(log, sizeof(log), "GYRO raw=%.2f corrected=%.2f yaw_raw=%.2f\n", raw, raw - gyro_bias, watch_yaw);
    // Drop telemetry instead of blocking the control loop on serial backpressure.
    if (n > 0 && n < sizeof(log) && Serial.availableForWrite() >= n) Serial.write(reinterpret_cast<const uint8_t*>(log), n);
  }
}

void readImu(uint16_t samples) {
  gyro_calibrated = false; // Legacy read changes ODR; require a new GZERO afterward.
  if (!lsm_address && !initImu()) return;
  samples = constrain(samples, 1, 100);
  // CTRL2_G: 26 Hz, ±250 dps. OUTZ_G: 8.75 mdps/LSB.
  Wire.beginTransmission(lsm_address); Wire.write(0x11); Wire.write(0x20); Wire.endTransmission();
  float total = 0.0F, minimum = 1e6F, maximum = -1e6F;
  for (uint16_t s = 0; s < samples; ++s) {
    uint8_t raw[2];
    if (!lsmRead(0x26, raw, sizeof(raw))) { Serial.println("IMU error de lectura"); return; }
    const int16_t z = static_cast<int16_t>(raw[0] | (raw[1] << 8));
    const float dps = z * 0.00875F;
    total += dps; minimum = min(minimum, dps); maximum = max(maximum, dps); delay(20);
  }
  Serial.printf("IMU gyro_z n=%u mean=%.3f dps min=%.3f max=%.3f (quieto: use mean como drift)\n", samples, total / samples, minimum, maximum);
}

void scanI2c() {
  uint8_t found = 0; Serial.println("I2C:");
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) { Serial.printf("  0x%02X\n", address); ++found; }
  }
  if (!found) Serial.println("  sin dispositivos");
}

// Bench-only bus isolation; never enabled during motion. Restore the normal
// timeout afterwards and invalidate zero because configuration is rewritten.
void diagnoseImuBus() {
  stopMotors(); armed = false; gyro_calibrated = false;
  if (!initImu()) return;
  const bool configured = configureGyro();
  Serial.printf("IMUBUS config=%s; motores OFF; prueba timeout/STOP\n", configured ? "OK" : "ERROR");
  const uint8_t regs[] = {0x0F, 0x11, 0x12, 0x1E, 0x22, 0x26};
  const uint8_t counts[] = {1, 1, 1, 1, 6, 2};
  const uint32_t original_clock = Wire.getClock();
  for (uint32_t clock_hz : {100000U, 50000U}) {
  for (int timeout : {5, 25}) {
    for (int stop : {0, 1}) {
      Wire.end(); Wire.begin(); Wire.setClock(clock_hz); Wire.setTimeOut(timeout);
      Serial.printf("IMUBUS reset bus hz=%lu timeout=%d stop=%d config=%s\n",
          clock_hz, timeout, stop, configureGyro() ? "OK" : "ERROR");
      for (unsigned r = 0; r < 6; ++r) {
        unsigned success = 0, last_tx = 0, last_rx = 0;
        uint32_t longest = 0; uint8_t values[6] = {};
        for (unsigned trial = 0; trial < 5; ++trial) {
          const uint32_t before = micros();
          Wire.beginTransmission(lsm_address); Wire.write(regs[r]);
          last_tx = Wire.endTransmission(stop != 0);
          last_rx = 0;
          if (!last_tx) {
            last_rx = Wire.requestFrom(static_cast<int>(lsm_address), static_cast<int>(counts[r]));
            for (unsigned j = 0; j < last_rx && j < 6; ++j) values[j] = Wire.read();
          }
          if (!last_tx && last_rx == counts[r]) ++success;
          const uint32_t elapsed = micros() - before;
          if (elapsed > longest) longest = elapsed;
          delay(10);
        }
        Serial.printf("BUS timeout=%d stop=%d reg=0x%02X n=%u ok=%u/5 last_tx=%u last_rx=%u max_us=%lu bytes=",
            timeout, stop, regs[r], counts[r], success, last_tx, last_rx, longest);
        for (unsigned j = 0; j < counts[r]; ++j) Serial.printf("%02X ", values[j]);
        Serial.println("");
      }
    }
  }
  }
  Wire.end(); Wire.begin(); Wire.setClock(original_clock);
  Wire.setTimeOut(5);
  Serial.println("IMUBUS FIN; timeout normal 5 ms restaurado; CERO requerido");
}

void printHelp() {
  Serial.println("HELP | STATUS | ARM | DISARM | STOP");
  Serial.printf("MOTOR <M1 -35..35> <M2 -35..35> <ms 1..%lu>\n", MAX_MOVE_MS);
  Serial.println("LINE [n] | SONAR [n] | COLOR [n] | IMU [n] | I2C | IMUBUS");
  Serial.println("GZERO | GYRO <ms 1000..10000> (sin motores) | GSIGN <-1|1> (signo al girar IZQUIERDA)");
  Serial.println("HCONFIG <kp 0..2> <kd 0..0.5> <limite 0..5> | STRAIGHT <M1 10..25> <M2 10..25> <ms 1..3500>");
  Serial.println("TURNTEST <M1 -35..35> <M2 -35..35> <ms 1..3500>: ruedas opuestas, mide giro + 500 ms sin PWM");
  Serial.println("TURN <grados -180..-10 o 10..180>: positivo izquierda, max PWM 25, timeout 3500 ms hasta 90 / 7000 ms hasta 180");
  Serial.println("Este CenfoBot no tiene encoders: no hay PPR ni PI de velocidad por pulsos.");
}

void command(String line) {
  line.trim(); line.toUpperCase();
  if (!line.length()) return;
  // Las mediciones son bloqueantes, pero se ejecutan siempre con motores apagados.
  // Durante movimiento cualquier comando cancela la prueba; no se encadenan movimientos.
  if (moving || gyro_job != GyroJob::NONE) {
    stopMotors(); armed = false;
    Serial.println("STOP: prueba cancelada. Reenvíe el comando con el rover detenido.");
    return;
  }
#ifdef VRC_CALIBRATION_GUIDE
  if (guidedCommand(line)) return;
#endif
  if (line == "HELP") printHelp();
  else if (line == "STATUS") { Serial.printf("STATUS armed=%s moving=%s\n", armed ? "YES" : "NO", moving ? "YES" : "NO");
    Serial.printf("HEADING cal=%s bias=%.4f sign=%d kp=%.2f kd=%.2f limit=%.1f\n",
      gyroCalibrationFresh() ? "YES" : "NO", gyro_bias, gyro_sign, heading.kp, heading.kd, heading.limit); readLines(3); }
  else if (line == "ARM") { armed = pwm_ready; Serial.println(armed ? "ARMED: una sola orden MOTOR" : "ERROR: PWM no disponible"); }
  else if (line == "DISARM" || line == "STOP") { stopMotors(); armed = false; Serial.println("STOPPED + DISARMED"); }
  else if (line == "I2C") scanI2c();
  else if (line == "IMUBUS") diagnoseImuBus();
  else if (line == "GZERO") startGyroZero();
  else if (line.startsWith("GYRO ")) { int ms; char extra;
    if (sscanf(line.c_str(), "GYRO %d %c", &ms, &extra) == 1) startGyroWatch(ms);
    else Serial.println("ERROR: GYRO <ms>"); }
  else if (line.startsWith("GSIGN ")) { int sign; char extra; armed = false;
    if (sscanf(line.c_str(), "GSIGN %d %c", &sign, &extra) == 1 && (sign == 1 || sign == -1)) {
      gyro_sign = sign; Serial.printf("GSIGN=%d: confirmado por operador al girar IZQUIERDA\n", sign);
    } else Serial.println("ERROR: GSIGN -1 o 1"); }
  else if (line.startsWith("HCONFIG ")) { float kp, kd, limit; char extra; armed = false;
    if (sscanf(line.c_str(), "HCONFIG %f %f %f %c", &kp, &kd, &limit, &extra) == 3 &&
        std::isfinite(kp) && std::isfinite(kd) && std::isfinite(limit) &&
        kp >= 0 && kp <= 2 && kd >= 0 && kd <= 0.5F && limit >= 0 && limit <= 5) {
      heading.kp = kp; heading.kd = kd; heading.limit = limit; Serial.println("HCONFIG OK");
    } else Serial.println("ERROR: HCONFIG kp 0..2 kd 0..0.5 limite 0..5"); }
  else if (line.startsWith("STRAIGHT ")) { float l, r; int ms; char extra;
    if (sscanf(line.c_str(), "STRAIGHT %f %f %d %c", &l, &r, &ms, &extra) == 3) startStraight(l, r, ms);
    else { armed = false; Serial.println("ERROR: STRAIGHT <M1> <M2> <ms>"); } }
  else if (line.startsWith("TURNTEST ")) { int l, r, ms; char extra;
    if (sscanf(line.c_str(), "TURNTEST %d %d %d %c", &l, &r, &ms, &extra) == 3) startTurnTest(l, r, ms, false);
    else { armed = false; Serial.println("ERROR: TURNTEST <M1> <M2> <ms>"); } }
  else if (line.startsWith("TURN ")) { float degrees; char extra;
    if (sscanf(line.c_str(), "TURN %f %c", &degrees, &extra) == 1) startTurn(degrees);
    else { armed = false; Serial.println("ERROR: TURN <grados>"); } }
  else if (line.startsWith("MOTOR ")) { int m1, m2, ms; if (sscanf(line.c_str(), "MOTOR %d %d %d", &m1, &m2, &ms) == 3) runMotor(m1, m2, ms); else Serial.println("Uso: MOTOR <M1> <M2> <ms>"); }
  else if (line.startsWith("LINE")) { int n = 10; sscanf(line.c_str(), "LINE %d", &n); readLines(n); }
  else if (line.startsWith("SONAR")) { int n = 10; sscanf(line.c_str(), "SONAR %d", &n); readSonar(n); }
  else if (line.startsWith("COLOR")) { int n = 10; sscanf(line.c_str(), "COLOR %d", &n); readColor(n); }
  else if (line.startsWith("IMU")) { int n = 30; sscanf(line.c_str(), "IMU %d", &n); readImu(n); }
  else if (line.length()) Serial.println("ERROR: HELP");
}

void setup() {
  Serial.begin(115200); delay(300);
  const bool p1 = ledcAttach(M1_FORWARD, PWM_HZ, PWM_BITS);
  const bool p2 = ledcAttach(M1_BACKWARD, PWM_HZ, PWM_BITS);
  const bool p3 = ledcAttach(M2_FORWARD, PWM_HZ, PWM_BITS);
  const bool p4 = ledcAttach(M2_BACKWARD, PWM_HZ, PWM_BITS);
  pwm_ready = p1 && p2 && p3 && p4;
  stopMotors();
  for (uint8_t pin : LINE_PINS) pinMode(pin, INPUT);
  pinMode(COLOR_INPUT_PIN, INPUT); analogSetPinAttenuation(COLOR_INPUT_PIN, ADC_11db);
  pinMode(SONAR_TRIG, OUTPUT); pinMode(SONAR_ECHO, INPUT); digitalWrite(SONAR_TRIG, LOW);
  FastLED.addLeds<WS2812, COLOR_LED_PIN, GRB>(color_led, 1); color_led[0] = CRGB::Black; FastLED.show();
  Wire.begin();
  Wire.setTimeOut(5); // Bound each I2C transaction; stop remains cooperative.
  motion_profile.configure(heading, turn_control);
  Serial.println("CenfoBot diagnóstico C++: motores detenidos, sin Wi-Fi ni cámara. Escriba HELP.");
}

void pollSerial() {
  static char buffer[96];
  static size_t length = 0;
  static bool discard = false;
  // Un byte por iteración: una línea incompleta nunca bloquea el corte.
  if (!Serial.available()) return;
  const char byte = Serial.read();
  if (byte == '!' || byte == 3) {
    stopMotors(); armed = false; length = 0; discard = true;
    Serial.println("STOPPED + DISARMED"); return;
  }
  if (byte == '\r' || byte == '\n') {
    if (!discard && length) { buffer[length] = 0; command(String(buffer)); }
    length = 0; discard = false; return;
  }
  if (discard) return;
  if (length >= sizeof(buffer) - 1) {
    stopMotors(); armed = false; length = 0; discard = true;
    Serial.println("ERROR: línea demasiado larga; motores detenidos"); return;
  }
  buffer[length++] = byte;
}

void loop() {
  if (moving && static_cast<int32_t>(millis() - stop_at_ms) >= 0) {
    if (gyro_job == GyroJob::TURN) finishTurnPower(false);
    else {
    const bool controlled = gyro_job == GyroJob::DRIVE;
    stopMotors(); Serial.println("RESULT corte automático; motores detenidos");
    if (controlled) Serial.printf("HEADING RESULT yaw=%.2f deg (relativo IMU, no posicion)\n", heading.yaw);
    }
  }
  pollSerial(); // ! is processed before another I2C transaction.
  tickGyro();
}
