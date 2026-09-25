#pragma once
// LSM6DS3TR-C simulado: el simulador publica muestras a la frecuencia del ODR
// y el bit de dato nuevo se limpia al leer el bloque XYZ, como con BDU.
#include "Arduino.h"
#include <vector>
struct WireMock {
  bool ok = true, fresh = false;
  int16_t z = 0;
  uint8_t regs[256] = {}, reg = 0;
  std::vector<uint8_t> tx, rx;
  void begin() { regs[0x0F] = 0x6A; }
  void setTimeOut(int) {}
  void beginTransmission(int) { tx.clear(); }
  void write(uint8_t v) { tx.push_back(v); }
  int endTransmission(bool = true) {
    if (!ok) return 1;
    if (!tx.empty()) reg = tx[0];
    if (tx.size() == 2) regs[reg] = tx[1];
    return 0;
  }
  size_t requestFrom(int, int count) {
    rx.clear();
    if (!ok) return 0;
    regs[0x1E] = fresh ? 2 : 0;
    regs[0x26] = static_cast<uint16_t>(z) & 255;
    regs[0x27] = static_cast<uint16_t>(z) >> 8;
    for (int i = 0; i < count; ++i) rx.push_back(regs[(reg + i) & 255]);
    if (reg == 0x22) fresh = false;
    return count;
  }
  uint8_t read() { auto v = rx.front(); rx.erase(rx.begin()); return v; }
};
inline WireMock Wire;
