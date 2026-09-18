#pragma once

// Copiar este archivo como config.h y completar los datos del hotspot de visión.
// Cada rover usa el mismo hotspot y host, pero un ROVER_ID distinto (10 u 11).

constexpr char WIFI_SSID[] = "VISION_ROVER_HOTSPOT";
constexpr char WIFI_PASSWORD[] = "CAMBIAR_ESTO";
constexpr char VISION_HOST[] = "192.168.137.1";
constexpr uint16_t VISION_PORT = 2026;

constexpr uint8_t ROVER_ID = 10;

// Cada rover recibe un cubo distinto. En el segundo rover cambie ambos valores
// (por ejemplo, ROVER_ID=11 y ROVER_TARGET_COLOR="blue").
#define ROVER_TARGET_COLOR "red"
#define MOTOR_LEFT_REVERSED false
#define MOTOR_RIGHT_REVERSED false

// Banco de motores sin cámara ni Wi-Fi. Déjelo en 0 durante una ronda: al
// activarlo, el firmware solo acepta comandos por Serial y nunca navega.
// Por seguridad cada orden está limitada a ±35% PWM y 1500 ms.
#define VRC_ENABLE_MOTOR_BENCH 0

// Sensores de línea: IO36, IO39, IO34, IO35 (en ese orden). Calibrar sobre el
// piso y la línea real; los valores del ADC ESP32 van de 0 a 4095.
#define LINE_THRESHOLD_1 2000
#define LINE_THRESHOLD_2 2000
#define LINE_THRESHOLD_3 2000
#define LINE_THRESHOLD_4 2000
#define LINE_BLACK_IS_LOW true
// La cuadrícula de la competencia puede tener líneas bajo el rover. Déjelo en
// false hasta comprobar que estas líneas son un borde de seguridad, no la grilla.
#define LINE_STOP_ON_DETECTION false

// Sensor de color reflectivo: NeoPixel iluminador en IO4 y salida analógica en
// IO32. Cambie a false si su fotodetector aumenta la lectura con más reflexión.
#define COLOR_REFLECTION_IS_LOW true

// Parámetros de navegación visual (sin encoders), todos en unidades físicas.
#define MAX_SELF_POSE_AGE_MS 300U
#define MAX_CUBE_AGE_MS 1200U
#define NAV_DRIVE_SPEED 0.34F
#define NAV_PUSH_SPEED 0.26F
#define NAV_TURN_SPEED 0.28F
#define NAV_STEER_GAIN 0.16F
#define NAV_TURN_IN_PLACE_DEG 20.0F
#define NAV_GOAL_TOLERANCE_MM 35.0F
#define NAV_SLOWDOWN_MM 140.0F
#define NAV_APPROACH_STANDOFF_MM 115.0F
#define NAV_APPROACH_TOLERANCE_MM 45.0F
#define NAV_PUSH_CONTACT_MAX_MM 190.0F
#define NAV_PUSH_CONTACT_OFFSET_MM 80.0F
#define ULTRASONIC_STOP_CM 10.0F
