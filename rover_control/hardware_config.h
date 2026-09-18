#pragma once

#include <Arduino.h>

// Valores seguros por defecto para el hardware. Se pueden sobrescribir en
// config.h con #define antes de incluir cualquier módulo del rover. Mantenerlos
// acá permite actualizar el firmware sin obligar a borrar/recrear el config.h
// local, que contiene las credenciales de cada cancha.

#ifndef ROVER_TARGET_COLOR
#define ROVER_TARGET_COLOR "red"
#endif

// Si una rueda gira al revés con el comando de avance, invierta primero los
// cables de ese motor; este ajuste permite corregirlo por firmware mientras se
// calibra. Deben quedar ambos en false para el comportamiento normal.
#ifndef MOTOR_LEFT_REVERSED
#define MOTOR_LEFT_REVERSED false
#endif
#ifndef MOTOR_RIGHT_REVERSED
#define MOTOR_RIGHT_REVERSED false
#endif

#ifndef VRC_ENABLE_MOTOR_BENCH
#define VRC_ENABLE_MOTOR_BENCH 0
#endif

#ifndef LINE_THRESHOLD_1
#define LINE_THRESHOLD_1 2000
#endif
#ifndef LINE_THRESHOLD_2
#define LINE_THRESHOLD_2 2000
#endif
#ifndef LINE_THRESHOLD_3
#define LINE_THRESHOLD_3 2000
#endif
#ifndef LINE_THRESHOLD_4
#define LINE_THRESHOLD_4 2000
#endif
constexpr uint16_t LINE_THRESHOLDS[4] = {
    LINE_THRESHOLD_1, LINE_THRESHOLD_2, LINE_THRESHOLD_3, LINE_THRESHOLD_4};

#ifndef LINE_BLACK_IS_LOW
#define LINE_BLACK_IS_LOW true
#endif
#ifndef LINE_STOP_ON_DETECTION
#define LINE_STOP_ON_DETECTION false
#endif
#ifndef COLOR_REFLECTION_IS_LOW
#define COLOR_REFLECTION_IS_LOW true
#endif

#ifndef MAX_SELF_POSE_AGE_MS
#define MAX_SELF_POSE_AGE_MS 300U
#endif
#ifndef MAX_CUBE_AGE_MS
#define MAX_CUBE_AGE_MS 1200U
#endif
#ifndef NAV_DRIVE_SPEED
#define NAV_DRIVE_SPEED 0.34F
#endif
#ifndef NAV_PUSH_SPEED
#define NAV_PUSH_SPEED 0.26F
#endif
#ifndef NAV_TURN_SPEED
#define NAV_TURN_SPEED 0.28F
#endif
#ifndef NAV_STEER_GAIN
#define NAV_STEER_GAIN 0.16F
#endif
#ifndef NAV_TURN_IN_PLACE_DEG
#define NAV_TURN_IN_PLACE_DEG 20.0F
#endif
#ifndef NAV_GOAL_TOLERANCE_MM
#define NAV_GOAL_TOLERANCE_MM 35.0F
#endif
#ifndef NAV_SLOWDOWN_MM
#define NAV_SLOWDOWN_MM 140.0F
#endif
#ifndef NAV_APPROACH_STANDOFF_MM
#define NAV_APPROACH_STANDOFF_MM 115.0F
#endif
#ifndef NAV_APPROACH_TOLERANCE_MM
#define NAV_APPROACH_TOLERANCE_MM 45.0F
#endif
#ifndef NAV_PUSH_CONTACT_MAX_MM
#define NAV_PUSH_CONTACT_MAX_MM 190.0F
#endif
#ifndef NAV_PUSH_CONTACT_OFFSET_MM
#define NAV_PUSH_CONTACT_OFFSET_MM 80.0F
#endif
#ifndef ULTRASONIC_STOP_CM
#define ULTRASONIC_STOP_CM 10.0F
#endif
