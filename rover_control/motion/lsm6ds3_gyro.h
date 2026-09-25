#pragma once
#include <cstddef>
#include <cstdint>

// Caller owns bus.begin(), bounded I2C timeout, scheduling and stale-data stop.
// No delay, Serial, PWM, calibration assumptions or heap allocation here.
template <class Bus>
class Lsm6ds3Gyro {
 public:
  explicit Lsm6ds3Gyro(Bus& bus) : bus_(bus) {}
  uint8_t address() const { return address_; }
  bool detect() {
    for (uint8_t candidate = 0x6A; candidate <= 0x6B; ++candidate) {
      address_ = candidate;
      uint8_t identity = 0;
      if (read(0x0F, &identity, 1) && identity == 0x6A) return true;
    }
    address_ = 0;
    return false;
  }
  bool read(uint8_t reg, uint8_t* data, size_t count) {
    if (!address_ || !data || !count || count > 6) return false;
    bus_.beginTransmission(address_); bus_.write(reg);
    if (bus_.endTransmission(false) != 0) return false;
    if (bus_.requestFrom(static_cast<int>(address_), static_cast<int>(count)) != count) return false;
    for (size_t i = 0; i < count; ++i) data[i] = bus_.read();
    return true;
  }
  bool write(uint8_t reg, uint8_t value) {
    if (!address_) return false;
    bus_.beginTransmission(address_); bus_.write(reg); bus_.write(value);
    return bus_.endTransmission() == 0;
  }
  bool configure() {
    if (!address_ && !detect()) return false;
    // LSM6DS3TR-C: 104 Hz, 250 dps, BDU + address increment.
    if (!write(0x12, 0x44) || !write(0x11, 0x40)) return false;
    uint8_t ctrl[2];
    return read(0x11, ctrl, 2) && ctrl[0] == 0x40 && ctrl[1] == 0x44;
  }
  // -1 error; 0 no fresh sample; 1 new sample. Read XYZ to release BDU.
  int sample(float& z_dps) {
    uint8_t status = 0, raw[6];
    if (!read(0x1E, &status, 1)) return -1;
    if (!(status & 2)) return 0;
    if (!read(0x22, raw, 6)) return -1;
    const uint16_t bits = static_cast<uint16_t>(raw[4]) | (static_cast<uint16_t>(raw[5]) << 8);
    const int32_t signed_raw = bits >= 32768 ? static_cast<int32_t>(bits) - 65536 : bits;
    z_dps = signed_raw * 0.00875F;
    return 1;
  }
 private:
  Bus& bus_;
  uint8_t address_ = 0;
};
