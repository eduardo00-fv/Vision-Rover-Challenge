#pragma once
#include "Arduino.h"
struct CRGB { static const CRGB Black, Red, Green, Blue; };
inline const CRGB CRGB::Black{}, CRGB::Red{}, CRGB::Green{}, CRGB::Blue{};
struct WS2812 {};
constexpr int GRB = 0;
struct FastLEDMock {
  template<class T, int Pin, int Order> void addLeds(CRGB*, int) {}
  void show() {}
};
inline FastLEDMock FastLED;
