#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include "world_state.h"

class TelemetryClient {
 public:
  TelemetryClient(const char* ssid, const char* password, const char* host, uint16_t port);

  void begin();
  void poll();
  const WorldState& world() const;
  bool hasFreshWorld(uint32_t max_transport_age_ms) const;
  bool connected() const;

  uint32_t validMessages() const { return valid_messages_; }
  uint32_t invalidMessages() const { return invalid_messages_; }
  uint32_t reconnects() const { return reconnects_; }

 private:
  static constexpr size_t LINE_BUFFER_SIZE = 2048;
  static constexpr uint32_t WIFI_RETRY_MS = 5000;
  static constexpr uint32_t TCP_RETRY_MS = 1500;

  const char* ssid_;
  const char* password_;
  const char* host_;
  uint16_t port_;
  WiFiClient socket_;
  char line_buffer_[LINE_BUFFER_SIZE];
  size_t line_length_ = 0;
  uint32_t last_wifi_attempt_ms_ = 0;
  uint32_t last_tcp_attempt_ms_ = 0;
  uint32_t valid_messages_ = 0;
  uint32_t invalid_messages_ = 0;
  uint32_t reconnects_ = 0;
  WorldState latest_;

  void ensureWifi();
  void ensureTcp();
  void readSocket();
  void processLine();
  bool parseMessage(const char* line, WorldState& parsed);
};

