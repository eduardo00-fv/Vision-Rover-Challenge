#pragma once
// ESP-NOW simulado para las pruebas de host (copia de sim/host/esp_now.h).
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_NOW_ETH_ALEN 6
#define ESP_NOW_KEY_LEN 16
#ifndef VRC_MOCK_WIFI_IF
#define VRC_MOCK_WIFI_IF
typedef enum { WIFI_IF_STA = 0, WIFI_IF_AP = 1 } wifi_interface_t;
#endif

typedef struct esp_now_recv_info { uint8_t* src_addr; uint8_t* des_addr; void* rx_ctrl; } esp_now_recv_info_t;
typedef struct esp_now_peer_info {
  uint8_t peer_addr[ESP_NOW_ETH_ALEN];
  uint8_t lmk[ESP_NOW_KEY_LEN];
  uint8_t channel;
  wifi_interface_t ifidx;
  bool encrypt;
  void* priv;
} esp_now_peer_info_t;
typedef void (*esp_now_recv_cb_t)(const esp_now_recv_info_t*, const uint8_t*, int);

inline bool sim_espnow_ok = true;
inline esp_now_recv_cb_t sim_espnow_cb = nullptr;
inline std::vector<std::string> sim_espnow_out;

inline esp_err_t esp_now_init() { return sim_espnow_ok ? ESP_OK : ESP_FAIL; }
inline esp_err_t esp_now_add_peer(const esp_now_peer_info_t*) { return ESP_OK; }
inline esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb) { sim_espnow_cb = cb; return ESP_OK; }
inline esp_err_t esp_now_send(const uint8_t*, const uint8_t* data, size_t len) {
  sim_espnow_out.emplace_back(reinterpret_cast<const char*>(data), len);
  return ESP_OK;
}
inline void sim_espnow_deliver(const uint8_t* data, int len) {
  esp_now_recv_info_t info{};
  if (sim_espnow_cb) sim_espnow_cb(&info, data, len);
}
