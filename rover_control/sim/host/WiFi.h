#pragma once
#include "Arduino.h"
constexpr int WL_CONNECTED = 3, WL_DISCONNECTED = 6, WIFI_STA = 1;
struct WiFiStub {
  bool link = true;
  int status() const { return link ? WL_CONNECTED : WL_DISCONNECTED; }
  void mode(int) {}
  void setSleep(bool) {}
  void begin(const char*, const char*) {}
};
inline WiFiStub WiFi;
// El simulador escribe en `incoming`; un corte TCP descarta lo pendiente.
class WiFiClient {
 public:
  inline static bool online = true;
  inline static bool session = false;
  inline static std::string incoming;
  bool connected() const { return online && session; }
  bool connect(const char*, uint16_t, int) { session = online; return session; }
  int available() const { return static_cast<int>(incoming.size()); }
  int read() { char c = incoming.front(); incoming.erase(0, 1); return c; }
};
