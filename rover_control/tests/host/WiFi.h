#pragma once
#include "Arduino.h"
constexpr int WL_CONNECTED = 3, WIFI_STA = 1;
struct WiFiStub {
  int state = WL_CONNECTED;
  int status() const { return state; }
  void mode(int) {}
  void setSleep(bool) {}
  void begin(const char*, const char*) {}
};
inline WiFiStub WiFi;
class WiFiClient {
 public:
  inline static bool online = true;
  inline static std::string incoming;
  bool connected() const { return online; }
  bool connect(const char*, uint16_t, int) { online = true; return true; }
  int available() const { return incoming.size(); }
  int read() { char c = incoming.front(); incoming.erase(0, 1); return c; }
};
