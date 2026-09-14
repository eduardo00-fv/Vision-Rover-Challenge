#pragma once

// Copiar este archivo como config.h y completar los datos del hotspot de visión.
// Cada rover usa el mismo hotspot y host, pero un ROVER_ID distinto (10 u 11).

constexpr char WIFI_SSID[] = "VISION_ROVER_HOTSPOT";
constexpr char WIFI_PASSWORD[] = "CAMBIAR_ESTO";
constexpr char VISION_HOST[] = "192.168.137.1";
constexpr uint16_t VISION_PORT = 2026;

constexpr uint8_t ROVER_ID = 10;

