#include "peer_link.h"

#include <WiFi.h>
#include <esp_now.h>

#include <cstring>

namespace peer_proto {
namespace {
uint8_t sum(const Packet& p) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&p);
  uint8_t s = 0x5A;
  for (size_t i = 0; i + 1 < sizeof(Packet); ++i) s = static_cast<uint8_t>((s << 1 | s >> 7) ^ bytes[i]);
  return s;
}
}  // namespace

Packet encode(uint8_t team, uint8_t id, uint8_t seq, uint8_t target, uint8_t stage, bool can_move) {
  Packet p{{'V', 'R'}, kVersion, team, id, seq, target, stage, static_cast<uint8_t>(can_move ? 1 : 0), 0};
  p.checksum = sum(p);
  return p;
}

bool decode(const uint8_t* data, int len, uint8_t team, Packet& out) {
  if (data == nullptr || len != static_cast<int>(sizeof(Packet))) return false;
  memcpy(&out, data, sizeof(Packet));
  return out.magic[0] == 'V' && out.magic[1] == 'R' && out.version == kVersion && out.team == team &&
         out.id != 0 && out.stage <= PARK && out.can_move <= 1 && out.checksum == sum(out);
}

}  // namespace peer_proto

namespace {
PeerLink* g_link = nullptr;  // el callback de ESP-NOW no lleva contexto
const uint8_t kBroadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void onReceive(const esp_now_recv_info_t*, const uint8_t* data, int len) {
  if (g_link != nullptr) g_link->receive(data, len, millis());
}
}  // namespace

bool PeerLink::begin(uint8_t team, uint8_t self_id) {
  team_ = team;
  self_id_ = self_id;
  g_link = this;
  if (esp_now_init() != ESP_OK) return false;
  esp_now_peer_info_t info = {};
  memcpy(info.peer_addr, kBroadcast, sizeof(kBroadcast));
  info.channel = 0;  // el canal del AP al que está conectado el STA
  info.ifidx = WIFI_IF_STA;
  info.encrypt = false;
  if (esp_now_add_peer(&info) != ESP_OK || esp_now_register_recv_cb(onReceive) != ESP_OK) return false;
  active_ = true;
  return true;
}

void PeerLink::publish(uint8_t target, uint8_t stage, bool can_move, uint32_t now) {
  if (!active_ || (sent_once_ && now - last_sent_ms_ < kPeriodMs)) return;
  sent_once_ = true;
  last_sent_ms_ = now;
  const peer_proto::Packet p = peer_proto::encode(team_, self_id_, seq_++, target, stage, can_move);
  esp_now_send(kBroadcast, reinterpret_cast<const uint8_t*>(&p), sizeof(p));  // sin reintentos: es un latido
}

void PeerLink::receive(const uint8_t* data, int len, uint32_t now) {
  peer_proto::Packet p;
  if (!peer_proto::decode(data, len, team_, p) || p.id == self_id_) return;
  Slot* slot = nullptr;
  for (Slot& s : slots_) if (s.status.id == p.id) slot = &s;
  for (Slot& s : slots_) if (slot == nullptr && s.status.id == 0) slot = &s;
  if (slot == nullptr) return;  // más compañeros que los previstos: ignorar
  slot->version.fetch_add(1, std::memory_order_acq_rel);
  slot->status = {p.id, p.target, p.stage, p.can_move != 0};
  slot->received_ms = now;
  slot->version.fetch_add(1, std::memory_order_acq_rel);
}

bool PeerLink::peer(uint8_t id, uint32_t now, PeerStatus& out) const {
  for (const Slot& s : slots_) {
    for (int attempt = 0; attempt < 4; ++attempt) {
      const uint32_t before = s.version.load(std::memory_order_acquire);
      if (before & 1U) continue;
      const PeerStatus status = s.status;
      const uint32_t received = s.received_ms;
      if (s.version.load(std::memory_order_acquire) != before) continue;
      if (status.id != id) break;
      if (now - received > kFreshMs) return false;
      out = status;
      return true;
    }
  }
  return false;
}
