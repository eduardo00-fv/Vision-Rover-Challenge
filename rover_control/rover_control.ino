#include "config.h"
#include "hardware_config.h"
#include "mission_controller.h"
#include "motor_controller.h"
#include "peer_link.h"
#include "safety_supervisor.h"
#include "sensors.h"
#include "start_button.h"
#include "telemetry_client.h"
#include "vision_navigator.h"
#include <Wire.h>
#include "motion/arena_motion.h"

constexpr uint32_t MAX_WORLD_AGE_MS = 1500;
constexpr uint32_t STATUS_PERIOD_MS = 1000;

TelemetryClient telemetry(WIFI_SSID, WIFI_PASSWORD, VISION_HOST, VISION_PORT);
SafetySupervisor safety(MAX_WORLD_AGE_MS);
MissionController mission;
MotorController motors;
Sensors sensors;
VisionNavigator navigator;
PeerLink peers;  // ESP-NOW con el compañero: solo informativo
StartButton start_button;
ArenaMotion<decltype(Wire)> motion(Wire, VRC_PHYSICAL_ROVER == 1
    ? rover_motion::rover1 : rover_motion::rover2);
bool emergency_stop = false;
DriveCommand last_command;  // última decisión, solo para diagnóstico
uint32_t last_status_ms = 0;

#if VRC_ENABLE_MOTOR_BENCH
bool motor_bench_armed = false;
uint32_t motor_bench_stop_at_ms = 0;
// Consola del banco también por Wi-Fi, para medir sin el cable USB: el rover
// llama a la laptop (VISION_HOST:BENCH_PORT), como hace con la telemetría.
// Si el enlace se corta, los motores se detienen y el banco se desarma.
WiFiClient bench_client;
bool bench_was_connected = false;
uint32_t bench_retry_ms = 0;

void benchLine(const char* text) {
  Serial.println(text);
  if (bench_client.connected()) { bench_client.print(text); bench_client.print('\n'); }
}

void benchStop() {
  motors.stop(); motor_bench_armed = false; motor_bench_stop_at_ms = 0;
}

void benchLink() {
  if (bench_client.connected()) return;
  if (bench_was_connected) {
    bench_was_connected = false;
    benchStop();
    Serial.println("[banco] enlace Wi-Fi perdido; motores detenidos");
  }
  if (WiFi.status() != WL_CONNECTED || static_cast<int32_t>(millis() - bench_retry_ms) < 0) return;
  bench_retry_ms = millis() + 1000;
  if (bench_client.connect(VISION_HOST, BENCH_PORT, 300)) {
    bench_client.setNoDelay(true);
    bench_was_connected = true;
    char hello[64];
    snprintf(hello, sizeof(hello), "[banco] escriba HELP (rover %u, Wi-Fi)", ROVER_ID);
    benchLine(hello);
  }
}

void benchCommand(String line) {
  line.trim();
  line.toUpperCase();
  if (line == "HELP") {
    benchLine("[banco] ARM | DISARM | STOP | MOTOR <izq -35..35> <der -35..35> <ms 1..1500>");
  } else if (line == "ARM") {
    motor_bench_armed = true;
    benchLine("[banco] armado; use PWM bajo y mantenga distancia de seguridad");
  } else if (line == "DISARM" || line == "STOP") {
    benchStop();
    benchLine("[banco] detenido y desarmado");
  } else if (line.startsWith("MOTOR ")) {
    int left, right, duration_ms;
    if (!motor_bench_armed) {
      benchLine("[banco] ERROR: escriba ARM antes de mover");
    } else if (sscanf(line.c_str(), "MOTOR %d %d %d", &left, &right, &duration_ms) != 3 ||
               left < -35 || left > 35 || right < -35 || right > 35 || duration_ms < 1 || duration_ms > 1500) {
      benchLine("[banco] uso: MOTOR <izq -35..35> <der -35..35> <ms 1..1500>");
    } else {
      motors.set(left / 100.0F, right / 100.0F);
      motor_bench_stop_at_ms = millis() + static_cast<uint32_t>(duration_ms);
      motor_bench_armed = false;
      char text[64];
      snprintf(text, sizeof(text), "[banco] izq=%d der=%d por %d ms", left, right, duration_ms);
      benchLine(text);
    }
  } else if (line.length() != 0) {
    benchLine("[banco] comando desconocido; escriba HELP");
  }
}

// Un lector por origen: un comando a medias por Serial no se mezcla con Wi-Fi.
struct BenchReader {
  char buffer[96];
  size_t length = 0;
  bool discard = false;
  void feed(char byte) {
    if (byte == '!' || byte == 3) {
      benchStop(); length = 0; discard = true; return;
    }
    if (byte != '\n' && byte != '\r') {
      if (discard) return;
      if (length >= sizeof(buffer) - 1) { benchStop(); length = 0; discard = true; return; }
      buffer[length++] = byte;
      return;
    }
    if (discard || length == 0) { length = 0; discard = false; return; }
    buffer[length] = 0;
    length = 0;
    benchCommand(String(buffer));
  }
};
BenchReader bench_serial, bench_wifi;

void motorBench() {
  if (motor_bench_stop_at_ms != 0 &&
      static_cast<int32_t>(millis() - motor_bench_stop_at_ms) >= 0) {
    motors.stop();
    motor_bench_stop_at_ms = 0;
    benchLine("[banco] fin de orden; motores detenidos");
  }
  benchLink();
  for (unsigned i = 0; i < 32 && Serial.available(); ++i) bench_serial.feed(Serial.read());
  for (unsigned i = 0; i < 32 && bench_client.available(); ++i) bench_wifi.feed(bench_client.read());
}
#endif

void stopMotors() {
  motors.stop();
  motion.cancel();
}

SafetyDecision updateMission(const WorldState& world) {
  const SafetyDecision decision = safety.evaluate(world, ROVER_ID, MAX_SELF_POSE_AGE_MS);
  mission.update(world, decision);
  if (!decision.motion_allowed) stopMotors();
  return decision;
}

void printStatus(const WorldState& world) {
  // Serial diagnostics must never block an active maneuver.
  if (motion.left != 0 || motion.right != 0) return;
  if (millis() - last_status_ms < STATUS_PERIOD_MS) return;
  last_status_ms = millis();
  Serial.printf("[movimiento] perfil=%d estado=%d motivo=%s stop=%d imu=%s\n",
                VRC_PHYSICAL_ROVER, static_cast<int>(motion.state), motion.reason, emergency_stop,
                motion.visionOnly() ? motion.imuReason() : "ok");
  for (uint8_t i = 0; i < world.rover_count; ++i) {
    PeerStatus peer;
    if (world.rovers[i].id == ROVER_ID) continue;
    if (peers.peer(world.rovers[i].id, millis(), peer))
      Serial.printf("[espnow] compañero=%u objetivo=%u etapa=%u puede_moverse=%d\n",
                    peer.id, peer.target, peer.stage, peer.can_move);
    else
      Serial.printf("[espnow] compañero=%u sin datos (enlace=%d)\n", world.rovers[i].id, peers.active());
  }
  if (VRC_REQUIRE_START_BUTTON) Serial.printf("[arranque] armado=%d\n", start_button.armed());
  const RoverObservation* self = findRover(world, ROVER_ID);
  const SensorSnapshot& sensor = sensors.snapshot();
  Serial.printf("[estado] wifi=%s tcp=%s fsm=%s validos=%lu invalidos=%lu saltos=%lu "
                "sonar=%s%.1fcm color=%s\n",
                WiFi.status() == WL_CONNECTED ? "ok" : "no",
                telemetry.connected() ? "ok" : "no", mission.stateName(),
                telemetry.validMessages(), telemetry.invalidMessages(),
                world.sequence_gap, sensor.distance_valid ? "" : "?",
                sensor.distance_cm, sensor.color_name);
  if (self != nullptr) {
    Serial.printf("[pose] id=%u col=%.2f row=%.2f theta=%.1f age=%lu ms phase=%s\n",
                  self->id, self->col, self->row, self->theta_deg, self->age_ms,
                  phaseName(world.phase));
  }
}

// Pasa al navegador lo que anuncian los compañeros y anuncia el estado propio.
void exchangePeers(const WorldState& world) {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < world.rover_count; ++i) {
    PeerStatus peer;
    if (world.rovers[i].id != ROVER_ID && peers.peer(world.rovers[i].id, now, peer))
      navigator.peerHint(peer.id, peer.can_move, peer.target, peer.stage == peer_proto::PUSH, now);
  }
  // Capacidad, no fase: antes de RUNNING ninguno debe quedar excluido.
  const bool can_move = !VRC_NETWORK_ONLY && !emergency_stop &&
                        motion.state != decltype(motion)::State::FAULT;
  peers.publish(navigator.claimedTarget(), navigator.stage(), can_move, now);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\nVision Rover Control | rover %u\n", ROVER_ID);
  motors.begin();
#if VRC_ENABLE_MOTOR_BENCH
  Serial.println("[banco] MODO BANCO ACTIVO: no ejecuta misión");
  Serial.println("[banco] escriba HELP");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  // consola del banco por Wi-Fi
  return;
#endif
  sensors.begin();
  if (VRC_REQUIRE_START_BUTTON) pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  telemetry.begin();
#if VRC_ENABLE_ESPNOW
  if (!peers.begin(ESPNOW_TEAM_ID, ROVER_ID)) Serial.println("[espnow] no arrancó: se sigue solo con visión");
#endif
  if (VRC_NETWORK_ONLY) {
    Serial.println("[red] SOLO TELEMETRIA: motores bloqueados incluso en RUNNING");
    return;
  }
  Wire.begin();
  Wire.setTimeOut(5);
  motion.timing(ARENA_SETTLE_MS, ARENA_MAX_DRIVE_MS);
  motion.allowVisionOnly(ARENA_VISION_ONLY_FALLBACK);
  motion.begin(micros());
  Serial.println("[arena] mantener quieto durante cero IMU; ! detiene hasta reiniciar");
}

void loop() {
#if VRC_ENABLE_MOTOR_BENCH
  motorBench();
  return;
#endif
  // Cortar antes de una reconexión TCP, que puede esperar hasta su timeout.
  if (!telemetry.connected() || !safety.evaluate(telemetry.world(), ROVER_ID, MAX_SELF_POSE_AGE_MS).motion_allowed)
    stopMotors();
  // Local emergency stop is latched. Consume a bounded amount per iteration.
  for (unsigned i = 0; i < 32 && Serial.available(); ++i) {
    if (Serial.read() == '!') emergency_stop = true;
  }
  motion.guard(micros());
  if (motion.left == 0 && motion.right == 0) motors.stop();
  if (emergency_stop || motion.state == decltype(motion)::State::FAULT) stopMotors();
  telemetry.poll();
  motion.sample(micros());
  motion.guard(micros());
  if (motion.left == 0 && motion.right == 0) motors.stop();
  sensors.poll();
  const WorldState& world = telemetry.world();
  const SafetyDecision decision = updateMission(world);

  // Ningún PWM llega al driver sin permiso de la visión y sin pasar las dos
  // guardas locales. Durante PUSH el ultrasónico está mirando el cubo objetivo,
  // por eso no se interpreta como obstáculo.
  exchangePeers(world);
  const bool was_armed = start_button.armed();
  const bool armed = !VRC_REQUIRE_START_BUTTON ||
      start_button.update(digitalRead(START_BUTTON_PIN) == LOW, mission.state() == MissionState::FINISHED, millis());
  if (VRC_REQUIRE_START_BUTTON && armed && !was_armed) Serial.println("[arranque] botón presionado: armado");
  const DriveCommand command = navigator.update(world, ROVER_ID);
  last_command = command;
  const bool allowed = !VRC_NETWORK_ONLY && !emergency_stop && armed && telemetry.connected() &&
      decision.motion_allowed && mission.state() == MissionState::RUNNING &&
      !sensors.boundaryDetected() &&
      !(sensors.obstacleNear(ULTRASONIC_STOP_CM) && !command.pushing);
  motion.command(allowed, command.active, command.heading_error_deg, command.target,
                 command.pushing, world.captured_at_ms, micros(), command.reverse,
                 command.distance_mm);
  if (!allowed) {
    stopMotors();
  } else {
    motors.set(motion.left, motion.right);
  }
  printStatus(world);
  // Sin delay(): la red, la FSM y el control de motores cooperan.
}
