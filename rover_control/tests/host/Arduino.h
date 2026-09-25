#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
using String = std::string;
inline uint32_t test_now = 0;
inline uint32_t millis() { return test_now; }
struct SerialStub {
  template<class... T> void printf(const char*, T...) {}
  void println(const char*) {}
};
inline SerialStub Serial;
inline uint32_t test_duty[40] = {};
inline bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
inline bool ledcWrite(uint8_t pin, uint32_t duty) { test_duty[pin] = duty; return true; }
