#pragma once
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
inline uint32_t test_us = 0;
inline uint32_t micros() { return test_us; }
inline uint32_t millis() { return test_us / 1000; }
inline void delay(uint32_t ms) { test_us += ms * 1000; }
inline void delayMicroseconds(uint32_t us) { test_us += us; }
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return 0; }
inline int analogRead(int) { return 0; }
inline void analogSetPinAttenuation(int, int) {}
inline uint32_t pulseIn(int, int, uint32_t) { return 0; }
inline uint32_t test_duty[40] = {};
inline bool ledcAttach(int, int, int) { return true; }
inline void ledcWrite(int pin, uint32_t duty) { test_duty[pin] = duty; }
class String : public std::string {
 public:
  using std::string::string;
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
  void begin(int) {}
  int available() { return input.size(); }
  char read() { char c = input[0]; input.erase(0, 1); return c; }
  int availableForWrite() { return 256; }
  void write(const uint8_t* data, size_t size) { output.append(reinterpret_cast<const char*>(data), size); }
  void println(const char* line) { output += line; output += "\n"; }
  template<class... Args> void printf(const char* fmt, Args... args) {
    char buffer[512]; std::snprintf(buffer, sizeof(buffer), fmt, args...); output += buffer;
  }
};
inline SerialMock Serial;
