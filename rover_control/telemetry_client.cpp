#include "telemetry_client.h"

#include <ArduinoJson.h>
#include <math.h>

namespace {
constexpr uint8_t PROTOCOL_VERSION = 2;
constexpr size_t JSON_CAPACITY = 3072;

bool required(JsonVariantConst value) { return !value.isNull(); }
bool number(JsonVariantConst value) {
  return value.is<double>() && isfinite(value.as<double>());
}

bool pointFrom(JsonObjectConst object, PointCells& point) {
  if (!number(object["col"]) || !number(object["row"])) return false;
  point.col = object["col"].as<float>();
  point.row = object["row"].as<float>();
  return true;
}
}  // namespace

TelemetryClient::TelemetryClient(const char* ssid, const char* password, const char* host,
                                 uint16_t port)
    : ssid_(ssid), password_(password), host_(host), port_(port) {}

void TelemetryClient::begin() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  ensureWifi();
}

void TelemetryClient::poll() {
  if (WiFi.status() != WL_CONNECTED || !socket_.connected()) latest_.valid = false;
  ensureWifi();
  ensureTcp();
  readSocket();
}

const WorldState& TelemetryClient::world() const { return latest_; }

bool TelemetryClient::hasFreshWorld(uint32_t max_transport_age_ms) const {
  return latest_.valid && (millis() - latest_.received_at_ms) <= max_transport_age_ms;
}

bool TelemetryClient::connected() {
  return WiFi.status() == WL_CONNECTED && socket_.connected();
}

void TelemetryClient::ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  const uint32_t now = millis();
  if (now - last_wifi_attempt_ms_ < WIFI_RETRY_MS) return;
  last_wifi_attempt_ms_ = now;
  Serial.printf("[wifi] conectando a %s\n", ssid_);
  WiFi.begin(ssid_, password_);
}

void TelemetryClient::ensureTcp() {
  if (WiFi.status() != WL_CONNECTED || socket_.connected()) return;
  const uint32_t now = millis();
  if (now - last_tcp_attempt_ms_ < TCP_RETRY_MS) return;
  last_tcp_attempt_ms_ = now;
  Serial.printf("[tcp] conectando a %s:%u\n", host_, port_);
  if (socket_.connect(host_, port_, 50)) {
    ++reconnects_;
    line_length_ = 0;
    discard_line_ = false;
    new_session_ = true;
    Serial.println("[tcp] telemetria conectada");
  } else {
    Serial.println("[tcp] conexion fallida");
  }
}

void TelemetryClient::readSocket() {
  if (!socket_.connected()) return;
  // Limitar trabajo por vuelta: un emisor continuo no debe monopolizar el control.
  size_t budget = LINE_BUFFER_SIZE * 2;
  while (budget-- && socket_.available() > 0) {
    const char byte = static_cast<char>(socket_.read());
    if (byte == '\r') continue;
    if (byte == '\n') {
      line_buffer_[line_length_] = '\0';
      if (!discard_line_) processLine();
      line_length_ = 0;
      discard_line_ = false;
      continue;
    }
    if (discard_line_) continue;
    if (line_length_ >= LINE_BUFFER_SIZE - 1) {
      ++invalid_messages_;
      line_length_ = 0;
      discard_line_ = true;
      latest_.valid = false;
      Serial.println("[telemetria] linea demasiado larga; descartada");
      continue;
    }
    line_buffer_[line_length_++] = byte;
  }
}

void TelemetryClient::processLine() {
  if (line_length_ == 0) return;
  WorldState parsed;
  if (!parseMessage(line_buffer_, parsed)) {
    ++invalid_messages_;
    latest_.valid = false;
    return;
  }
  if (!new_session_ && parsed.seq <= latest_.seq) return;
  // Una retransmisión de la misma captura no renueva su vida útil.
  if (!new_session_ && parsed.captured_at_ms <= latest_.captured_at_ms) return;
  parsed.sequence_gap = latest_.valid ? parsed.seq - latest_.seq - 1 : 0;
  latest_ = parsed;
  new_session_ = false;
  ++valid_messages_;
}

bool TelemetryClient::parseMessage(const char* line, WorldState& parsed) {
  StaticJsonDocument<JSON_CAPACITY> document;
  const DeserializationError error = deserializeJson(document, line);
  if (error) {
    Serial.printf("[telemetria] JSON invalido: %s\n", error.c_str());
    return false;
  }
  const JsonObjectConst root = document.as<JsonObjectConst>();
  if (root.isNull() || !root["v"].is<uint8_t>() || root["v"].as<uint8_t>() != PROTOCOL_VERSION) return false;

  const JsonObjectConst clock = root["clock"];
  const JsonObjectConst grid = root["grid"];
  const JsonObjectConst start = root["start"];
  const JsonObjectConst depot_size = root["depot_size"];
  if (!required(root["seq"]) || !required(root["ts_ms"]) || !required(root["phase"]) ||
      clock.isNull() || grid.isNull() || start.isNull() || depot_size.isNull() ||
      !required(root["cube_side"]) || !required(root["rovers"]) ||
      !required(root["cubes"]) || !required(root["obstacles"]) || !required(root["depots"])) {
    return false;
  }

  if (!root["seq"].is<uint32_t>() || !root["ts_ms"].is<uint64_t>() ||
      !root["phase"].is<const char*>() || !root["rovers"].is<JsonArrayConst>() ||
      !root["cubes"].is<JsonArrayConst>() || !root["depots"].is<JsonArrayConst>() ||
      !root["obstacles"].is<JsonArrayConst>() ||
      !clock["elapsed_ms"].is<uint32_t>() || !clock["remaining_ms"].is<uint32_t>() ||
      !clock["total_ms"].is<uint32_t>() || !grid["cols"].is<uint16_t>() ||
      !grid["rows"].is<uint16_t>() || !number(grid["cell_mm"]) ||
      !number(depot_size["length"]) || !number(depot_size["depth"]) ||
      !number(root["cube_side"])) return false;

  parsed.protocol_version = PROTOCOL_VERSION;
  parsed.seq = root["seq"].as<uint32_t>();
  parsed.captured_at_ms = root["ts_ms"].as<uint64_t>();
  parsed.received_at_ms = millis();
  parsed.phase = parsePhase(root["phase"]);
  if (parsed.phase == RoundPhase::UNKNOWN) return false;
  parsed.clock.elapsed_ms = clock["elapsed_ms"].as<uint32_t>();
  parsed.clock.remaining_ms = clock["remaining_ms"].as<uint32_t>();
  parsed.clock.total_ms = clock["total_ms"].as<uint32_t>();
  parsed.grid.cols = grid["cols"].as<uint16_t>();
  parsed.grid.rows = grid["rows"].as<uint16_t>();
  parsed.grid.cell_mm = grid["cell_mm"].as<float>();
  parsed.depot_size.length_cells = depot_size["length"].as<float>();
  parsed.depot_size.depth_cells = depot_size["depth"].as<float>();
  parsed.cube_side_cells = root["cube_side"].as<float>();
  if (!pointFrom(start, parsed.start) || parsed.grid.cols == 0 || parsed.grid.rows == 0 ||
      parsed.grid.cell_mm <= 0.0F || parsed.cube_side_cells <= 0.0F ||
      parsed.depot_size.length_cells <= 0.0F || parsed.depot_size.depth_cells <= 0.0F) return false;

  for (JsonObjectConst item : root["rovers"].as<JsonArrayConst>()) {
    if (parsed.rover_count >= MAX_ROVERS || !item["id"].is<uint8_t>() ||
        !number(item["theta"]) || !item["age_ms"].is<uint32_t>()) return false;
    if (findRover(parsed, item["id"].as<uint8_t>()) != nullptr) return false;
    RoverObservation& rover = parsed.rovers[parsed.rover_count];
    if (!pointFrom(item, rover)) return false;
    rover.id = item["id"].as<uint8_t>();
    rover.theta_deg = item["theta"].as<float>();
    rover.age_ms = item["age_ms"].as<uint32_t>();
    ++parsed.rover_count;
  }
  for (JsonObjectConst item : root["cubes"].as<JsonArrayConst>()) {
    if (parsed.cube_count >= MAX_CUBES || !item["color"].is<const char*>() || !item["age_ms"].is<uint32_t>()) return false;
    if (findCube(parsed, item["color"].as<const char*>()) != nullptr) return false;
    CubeObservation& cube = parsed.cubes[parsed.cube_count];
    if (!pointFrom(item, cube)) return false;
    cube.color = item["color"].as<const char*>();
    cube.age_ms = item["age_ms"].as<uint32_t>();
    ++parsed.cube_count;
  }
  for (JsonObjectConst item : root["obstacles"].as<JsonArrayConst>()) {
    if (parsed.obstacle_count >= MAX_OBSTACLES || !item["age_ms"].is<uint32_t>()) return false;
    ObstacleObservation& obstacle = parsed.obstacles[parsed.obstacle_count];
    if (!pointFrom(item, obstacle)) return false;
    obstacle.age_ms = item["age_ms"].as<uint32_t>();
    ++parsed.obstacle_count;
  }
  for (JsonObjectConst item : root["depots"].as<JsonArrayConst>()) {
    if (parsed.depot_count >= MAX_CUBES || !item["color"].is<const char*>()) return false;
    if (findDepot(parsed, item["color"].as<const char*>()) != nullptr) return false;
    Depot& depot = parsed.depots[parsed.depot_count];
    if (!pointFrom(item, depot)) return false;
    depot.color = item["color"].as<const char*>();
    ++parsed.depot_count;
  }
  parsed.valid = true;
  return true;
}
