#pragma once
// Interfaz C entre el simulador y una copia del firmware real (una .so por rover).
#include <cstddef>
#include <cstdint>

extern "C" {
struct FwStatus {
  const char* mode;          // modo del núcleo de navegación
  const char* mission;       // estado de MissionController
  const char* motion_reason; // motivo de ArenaMotion (READY, IMU_STALE...)
  int motion_state;          // ArenaMotion::State
  int target;                // color objetivo (rover_logic::Color)
  int pushing;
  int emergency_stop;
  float heading_error_deg;
};

typedef void (*FwVoid)();
typedef void (*FwSetTime)(uint64_t us);
typedef uint64_t (*FwGetTime)();
typedef void (*FwTcpPush)(const char* data, size_t size);
typedef void (*FwSetFlag)(int value);
typedef void (*FwImu)(float z_dps);
typedef void (*FwMotors)(float* left, float* right);
typedef void (*FwStatusFn)(FwStatus* status);
typedef void (*FwSerialCapture)(int enabled);
typedef size_t (*FwSerialTake)(char* buffer, size_t capacity);
typedef void (*FwSerialInput)(const char* text);
}
