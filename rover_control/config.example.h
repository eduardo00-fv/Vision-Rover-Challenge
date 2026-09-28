#pragma once
#include <stdint.h>

// Copiar este archivo como config.h y completar los datos del hotspot de visión.
// Cada rover usa el mismo hotspot y host, pero un ROVER_ID distinto (10 u 11).

constexpr char WIFI_SSID[] = "VISION_ROVER_HOTSPOT";
constexpr char WIFI_PASSWORD[] = "CAMBIAR_ESTO";
constexpr char VISION_HOST[] = "192.168.137.1";
constexpr uint16_t VISION_PORT = 2026;

constexpr uint8_t ROVER_ID = 10;

// Número físico, independiente del marcador ROVER_ID. Solo perfil 1 validado.
#define VRC_PHYSICAL_ROVER 1
#define VRC_ENABLE_SONAR 0
#define VRC_ENABLE_COLOR 0

// El segundo rover usa ROVER_ID=11. Los colores se asignan desde el estado
// observado por el núcleo compartido; no se fija un color por robot.
#define MOTOR_LEFT_REVERSED false
#define MOTOR_RIGHT_REVERSED false

// Banco de motores sin cámara ni Wi-Fi. Déjelo en 0 durante una ronda: al
// activarlo, el firmware solo acepta comandos por Serial y nunca navega.
// Por seguridad cada orden está limitada a ±35% PWM y 1500 ms.
#define VRC_ENABLE_MOTOR_BENCH 0

// Para una ronda oficial: 1 hace fallar la compilación si queda activo el
// banco, la prueba solo-red o el botón de arranque (hardware_config.h).
#define VRC_COMPETITION 0

// IR S1..S4: GPIO4/5/18/19. Lectura digital (GPIO19 no tiene ADC).
// La cancha es un tablero de ajedrez: negro NO es borde. El borde se detecta
// por recorrido sin cambio de color (line_guard.h). Habilitar solo con las
// máscaras que da la prueba `ir` de sesion_viernes.py, por ejemplo:
// #define LINE_STOP_ON_DETECTION true
// #define LINE_FRONT_MASK 0b0011
// #define LINE_REAR_MASK 0b1100
// #define LINE_EDGE_MM 60.0F

// Sensor de color reflectivo: NeoPixel iluminador en IO23 y salida analógica en
// IO32. Cambie a false si su fotodetector aumenta la lectura con más reflexión.
#define COLOR_REFLECTION_IS_LOW true

// Parámetros de navegación visual (sin encoders), todos en unidades físicas.
#define MAX_SELF_POSE_AGE_MS 300U
#define MAX_CUBE_AGE_MS 1200U
#define MAX_PUSH_CUBE_AGE_MS 3000U
#define NAV_DRIVE_SPEED 0.34F
#define NAV_PUSH_SPEED 0.26F
#define NAV_TURN_SPEED 0.28F
#define NAV_STEER_GAIN 0.16F
#define NAV_TURN_IN_PLACE_DEG 20.0F
#define NAV_GOAL_TOLERANCE_MM 35.0F
#define NAV_SLOWDOWN_MM 140.0F
#define NAV_APPROACH_STANDOFF_MM 150.0F
#define NAV_APPROACH_TOLERANCE_MM 40.0F
#define NAV_PUSH_CONTACT_MAX_MM 190.0F
// Centro del rover → centro del cubo apoyado en las paletas (77 + 30 mm).
// Medirlo en cancha: decide si el cubo queda dentro de la zona (±32,6 mm).
#define NAV_PUSH_CONTACT_OFFSET_MM 107.0F
#define NAV_ARENA_MARGIN_MM 60.0F
// Ritmo parar-observar (ms). Por defecto 150/1000; 500/350 = conservador.
// #define ARENA_SETTLE_MS 150U
// #define ARENA_MAX_DRIVE_MS 1000U
#define ULTRASONIC_STOP_CM 10.0F
