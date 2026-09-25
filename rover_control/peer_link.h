#pragma once

#include <Arduino.h>

#include <atomic>

// ESP-NOW entre los rovers del equipo: latido, cubo reclamado y etapa.
// Solo informativo: la visión sigue dando el permiso de movimiento y las
// guardas no dependen de esto. Sin paquetes, todo funciona como sin enlace.
// Los paquetes no van cifrados (difusión): uno falso solo puede empeorar el
// reparto de cubos, nunca mover un rover.
namespace peer_proto {

constexpr uint8_t kVersion = 1;
enum Stage : uint8_t { NONE = 0, GOTO = 1, ALIGN = 2, PUSH = 3, RETREAT = 4, PARK = 5 };

struct __attribute__((packed)) Packet {
  uint8_t magic[2];   // 'V' 'R'
  uint8_t version;
  uint8_t team;       // ESPNOW_TEAM_ID: ignora rovers de otros equipos
  uint8_t id;         // marcador visual del emisor
  uint8_t seq;
  uint8_t target;     // rover_logic::Color reclamado (0 = ninguno)
  uint8_t stage;      // Stage
  uint8_t can_move;   // 0 = FAULT, paro de emergencia o fuera de RUNNING
  uint8_t checksum;
};
static_assert(sizeof(Packet) == 10, "paquete ESP-NOW de 10 bytes");

Packet encode(uint8_t team, uint8_t id, uint8_t seq, uint8_t target, uint8_t stage, bool can_move);
// Valida tamaño, firma, versión, equipo y suma.
bool decode(const uint8_t* data, int len, uint8_t team, Packet& out);

}  // namespace peer_proto

struct PeerStatus {
  uint8_t id = 0;
  uint8_t target = 0;
  uint8_t stage = peer_proto::NONE;
  bool can_move = false;
};

class PeerLink {
 public:
  static constexpr uint32_t kPeriodMs = 100;
  static constexpr uint32_t kFreshMs = 500;  // 5 latidos perdidos = sin datos
  static constexpr uint8_t kMaxPeers = 2;

  // Después de WiFi.mode(WIFI_STA). Falso si ESP-NOW no arranca.
  bool begin(uint8_t team, uint8_t self_id);
  void publish(uint8_t target, uint8_t stage, bool can_move, uint32_t now);
  // Último estado fresco del compañero `id`; falso si no hay o venció.
  bool peer(uint8_t id, uint32_t now, PeerStatus& out) const;
  bool active() const { return active_; }

  // Contexto de la tarea Wi-Fi: solo copia bajo un seqlock.
  void receive(const uint8_t* data, int len, uint32_t now);

 private:
  struct Slot {
    std::atomic<uint32_t> version{0};  // impar = escribiendo
    PeerStatus status;
    uint32_t received_ms = 0;
  };
  Slot slots_[kMaxPeers];
  uint8_t team_ = 0, self_id_ = 0, seq_ = 0;
  uint32_t last_sent_ms_ = 0;
  bool active_ = false, sent_once_ = false;
};
