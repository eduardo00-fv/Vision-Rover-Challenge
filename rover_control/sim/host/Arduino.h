#pragma once
// Periféricos simulados para compilar el firmware real como biblioteca
// compartida. Cada rover carga su propia copia: estos globales son por rover.
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
using std::min;
using std::max;
template<class T, class L, class H> T constrain(T v, L l, H h) { return v < l ? l : (v > h ? h : v); }
constexpr int HIGH = 1, LOW = 0, INPUT = 0, OUTPUT = 1, INPUT_PULLUP = 5, ADC_11db = 3;

inline uint64_t sim_time_us = 0;
inline uint32_t micros() { return static_cast<uint32_t>(sim_time_us); }
inline uint32_t millis() { return static_cast<uint32_t>(sim_time_us / 1000); }
inline void delay(uint32_t ms) { sim_time_us += static_cast<uint64_t>(ms) * 1000; }
inline void delayMicroseconds(uint32_t us) { sim_time_us += us; }

inline int sim_digital[40] = {};
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int pin) { return sim_digital[pin]; }
inline int analogRead(int) { return 0; }
inline void analogSetPinAttenuation(int, int) {}
inline uint32_t pulseIn(int, int, uint32_t) { return 0; }

inline uint32_t sim_duty[40] = {};
inline bool ledcAttach(int, int, int) { return true; }
inline void ledcWrite(int pin, uint32_t duty) { sim_duty[pin] = duty; }

class String : public std::string {
 public:
  using std::string::string;
  String() = default;
  String(const std::string& s) : std::string(s) {}
  void trim() {
    auto start = find_first_not_of(" \r\n\t");
    if (start == npos) { clear(); return; }
    auto end = find_last_not_of(" \r\n\t");
    assign(substr(start, end - start + 1));
  }
  void toUpperCase() { for (char& c : *this) c = std::toupper(static_cast<unsigned char>(c)); }
  bool startsWith(const char* prefix) const { return rfind(prefix, 0) == 0; }
};

struct SerialMock {
  std::string input, output;
  bool capture = false;  // sin captura se descarta: 600 s de estado no caben
  void begin(int) {}
  int available() { return static_cast<int>(input.size()); }
  char read() { char c = input[0]; input.erase(0, 1); return c; }
  void println(const char* line) { if (capture) { output += line; output += "\n"; } }
  template<class... Args> void printf(const char* fmt, Args... args) {
    if (!capture) return;
    char buffer[512]; std::snprintf(buffer, sizeof(buffer), fmt, args...); output += buffer;
  }
};
inline SerialMock Serial;
