#include "config.h"
#include "hardware_config.h"
#include "mission_controller.h"
#include "motor_controller.h"
#include "safety_supervisor.h"
#include "sensors.h"
#include "telemetry_client.h"
#include "vision_navigator.h"

constexpr uint32_t MAX_WORLD_AGE_MS = 1500;
constexpr uint32_t STATUS_PERIOD_MS = 1000;

TelemetryClient telemetry(WIFI_SSID, WIFI_PASSWORD, VISION_HOST, VISION_PORT);
SafetySupervisor safety(MAX_WORLD_AGE_MS);
MissionController mission;
MotorController motors;
Sensors sensors;
VisionNavigator navigator;
uint32_t last_status_ms = 0;

#if VRC_ENABLE_MOTOR_BENCH
bool motor_bench_armed = false;
uint32_t motor_bench_stop_at_ms = 0;

void motorBench() {
  if (motor_bench_stop_at_ms != 0 &&
      static_cast<int32_t>(millis() - motor_bench_stop_at_ms) >= 0) {
    motors.stop();
    motor_bench_stop_at_ms = 0;
    Serial.println("[banco] fin de orden; motores detenidos");
  }
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toUpperCase();
  if (line == "HELP") {
    Serial.println("[banco] ARM | DISARM | STOP | MOTOR <izq -35..35> <der -35..35> <ms 1..1500>");
  } else if (line == "ARM") {
    motor_bench_armed = true;
    Serial.println("[banco] armado; use PWM bajo y mantenga distancia de seguridad");
  } else if (line == "DISARM" || line == "STOP") {
    motor_bench_armed = false;
    motor_bench_stop_at_ms = 0;
    motors.stop();
    Serial.println("[banco] detenido y desarmado");
  } else if (line.startsWith("MOTOR ")) {
    int left, right, duration_ms;
    if (!motor_bench_armed) {
      Serial.println("[banco] ERROR: escriba ARM antes de mover");
    } else if (sscanf(line.c_str(), "MOTOR %d %d %d", &left, &right, &duration_ms) != 3 ||
               abs(left) > 35 || abs(right) > 35 || duration_ms < 1 || duration_ms > 1500) {
      Serial.println("[banco] uso: MOTOR <izq -35..35> <der -35..35> <ms 1..1500>");
    } else {
      motors.set(left / 100.0F, right / 100.0F);
      motor_bench_stop_at_ms = millis() + static_cast<uint32_t>(duration_ms);
      Serial.printf("[banco] izq=%d der=%d por %d ms\n", left, right, duration_ms);
    }
  } else if (line.length() != 0) {
    Serial.println("[banco] comando desconocido; escriba HELP");
  }
}
#endif

void stopMotors() {
  motors.stop();
}

SafetyDecision updateMission(const WorldState& world) {
  const SafetyDecision decision = safety.evaluate(world, ROVER_ID, MAX_SELF_POSE_AGE_MS);
  mission.update(world, decision);
  if (!decision.motion_allowed) stopMotors();
  return decision;
}

void printStatus(const WorldState& world) {
  if (millis() - last_status_ms < STATUS_PERIOD_MS) return;
  last_status_ms = millis();
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

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\nVision Rover Control | rover %u\n", ROVER_ID);
  motors.begin();
#if VRC_ENABLE_MOTOR_BENCH
  Serial.println("[banco] MODO BANCO ACTIVO: no se conecta a visión ni ejecuta misión");
  Serial.println("[banco] escriba HELP");
  return;
#endif
  sensors.begin();
  telemetry.begin();
}

void loop() {
#if VRC_ENABLE_MOTOR_BENCH
  motorBench();
  return;
#endif
  telemetry.poll();
  sensors.poll();
  const WorldState& world = telemetry.world();
  const SafetyDecision decision = updateMission(world);

  // Ningún PWM llega al driver sin permiso de la visión y sin pasar las dos
  // guardas locales. Durante PUSH el ultrasónico está mirando el cubo objetivo,
  // por eso no se interpreta como obstáculo.
  const DriveCommand command = navigator.update(world, ROVER_ID);
  if (!decision.motion_allowed || mission.state() != MissionState::RUNNING ||
      sensors.boundaryDetected() ||
      (sensors.obstacleNear(ULTRASONIC_STOP_CM) && !command.pushing)) {
    stopMotors();
  } else if (command.active) {
    motors.set(command.left, command.right);
  } else {
    stopMotors();
  }
  printStatus(world);
  // Sin delay(): la red, la FSM y el control de motores cooperan.
}
