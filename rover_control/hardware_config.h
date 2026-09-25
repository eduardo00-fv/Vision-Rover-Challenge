#pragma once

#include <Arduino.h>
#include "config.h"

// Receive and validate telemetry, but never authorize autonomous PWM.
#ifndef VRC_NETWORK_ONLY
#define VRC_NETWORK_ONLY 0
#endif
#if VRC_NETWORK_ONLY && VRC_ENABLE_MOTOR_BENCH
#error "La prueba de red no puede combinarse con el banco de motores"
#endif

// Physical profile must be selected explicitly, independently of marker ID.
#ifndef VRC_PHYSICAL_ROVER
#define VRC_PHYSICAL_ROVER 0
#endif
#ifndef VRC_ENABLE_SONAR
#define VRC_ENABLE_SONAR 0
#endif
#ifndef VRC_ENABLE_COLOR
#define VRC_ENABLE_COLOR 0
#endif

// Valores seguros por defecto para el hardware. Se pueden sobrescribir en
// config.h con #define antes de incluir cualquier módulo del rover. Mantenerlos
// acá permite actualizar el firmware sin obligar a borrar/recrear el config.h
// local, que contiene las credenciales de cada cancha.


// Ajustar por rover según la prueba de sentido; se aplica en cada módulo C++.
#ifndef MOTOR_LEFT_REVERSED
#define MOTOR_LEFT_REVERSED false
#endif
#ifndef MOTOR_RIGHT_REVERSED
#define MOTOR_RIGHT_REVERSED false
#endif

#ifndef VRC_ENABLE_MOTOR_BENCH
#define VRC_ENABLE_MOTOR_BENCH 0
#endif

// Cableado físico confirmado; S1..S4, sin inferir posición en el chasis.
constexpr uint8_t LINE_PINS[4] = {4, 5, 18, 19};
constexpr uint8_t SONAR_TRIG_PIN = 27;
constexpr uint8_t SONAR_ECHO_PIN = 33;
constexpr uint8_t COLOR_LED_PIN = 23;
constexpr uint8_t COLOR_SENSOR_PIN = 32;

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
// Durante el empuje el propio rover puede tapar el cubo: se tolera más edad.
#ifndef MAX_PUSH_CUBE_AGE_MS
#define MAX_PUSH_CUBE_AGE_MS 3000U
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
#define NAV_APPROACH_STANDOFF_MM 150.0F
#endif
#ifndef NAV_APPROACH_TOLERANCE_MM
#define NAV_APPROACH_TOLERANCE_MM 40.0F
#endif
#ifndef NAV_PUSH_CONTACT_MAX_MM
#define NAV_PUSH_CONTACT_MAX_MM 190.0F
#endif
#ifndef NAV_PUSH_CONTACT_OFFSET_MM
#define NAV_PUSH_CONTACT_OFFSET_MM 107.0F
#endif
// Centro del rover respecto del borde lógico: la superficie física sobra ~70 mm.
#ifndef NAV_ARENA_MARGIN_MM
#define NAV_ARENA_MARGIN_MM 60.0F
#endif
// Ritmo de ArenaMotion: pausa tras cada maniobra y tramo máximo por orden.
// El tiempo de ronda lo domina el parar-observar (ESTADO_DEL_ARTE.md §4).
// 150/1000 validado en simulación (AUDITORIA §8); 500/350 es el ritmo
// conservador anterior, para volver a él si en la arena la pose llega tarde.
#ifndef ARENA_SETTLE_MS
#define ARENA_SETTLE_MS 150U
#endif
#ifndef ARENA_MAX_DRIVE_MS
#define ARENA_MAX_DRIVE_MS 1000U
#endif
// Si la IMU falla (o no responde al arrancar), seguir en modo solo-visión:
// maniobras cortas a tiempo corregidas por la cámara. 0 = enclavar FAULT.
#ifndef ARENA_VISION_ONLY_FALLBACK
#define ARENA_VISION_ONLY_FALLBACK 1
#endif
// ESP-NOW entre rovers del equipo (latido, cubo reclamado, etapa). Si no
// llega nada, todo funciona igual. El equipo filtra rovers de otros equipos.
#ifndef VRC_ENABLE_ESPNOW
#define VRC_ENABLE_ESPNOW 1
#endif
#ifndef ESPNOW_TEAM_ID
#define ESPNOW_TEAM_ID 0x5A
#endif
// Arranque con el botón de la IdeaBoard (reglamento: "podrá utilizar"). Con 1,
// además de RUNNING hace falta presionarlo tras la señal del juez.
#ifndef VRC_REQUIRE_START_BUTTON
#define VRC_REQUIRE_START_BUTTON 0
#endif
#ifndef START_BUTTON_PIN
#define START_BUTTON_PIN 0  // BOOT: solo se lee en marcha, nunca al reiniciar
#endif
#ifndef ULTRASONIC_STOP_CM
#define ULTRASONIC_STOP_CM 10.0F
#endif
