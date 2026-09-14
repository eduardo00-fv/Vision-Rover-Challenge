#include "config.h"
#include "mission_controller.h"
#include "safety_supervisor.h"
#include "telemetry_client.h"

constexpr uint32_t MAX_WORLD_AGE_MS = 1500;
constexpr uint32_t STATUS_PERIOD_MS = 1000;

TelemetryClient telemetry(WIFI_SSID, WIFI_PASSWORD, VISION_HOST, VISION_PORT);
SafetySupervisor safety(MAX_WORLD_AGE_MS);
MissionController mission;
uint32_t last_status_ms = 0;

void stopMotors() {
  // La capa de hardware se conecta aquí después de calibrar pines y dirección.
}

void updateMission(const WorldState& world) {
  const SafetyDecision decision = safety.evaluate(world, ROVER_ID);
  mission.update(world, decision);
  if (!decision.motion_allowed) stopMotors();
}

void printStatus(const WorldState& world) {
  if (millis() - last_status_ms < STATUS_PERIOD_MS) return;
  last_status_ms = millis();
  const RoverObservation* self = findRover(world, ROVER_ID);
  Serial.printf("[estado] wifi=%s tcp=%s fsm=%s validos=%lu invalidos=%lu saltos=%lu\n",
                WiFi.status() == WL_CONNECTED ? "ok" : "no",
                telemetry.connected() ? "ok" : "no", mission.stateName(),
                telemetry.validMessages(), telemetry.invalidMessages(),
                world.sequence_gap);
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
  telemetry.begin();
}

void loop() {
  telemetry.poll();
  updateMission(telemetry.world());
  printStatus(telemetry.world());
  // Sin delay(): la red, la FSM y en el futuro el control de motores cooperan.
}
