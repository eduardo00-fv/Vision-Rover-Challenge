#include "../peer_link.h"

#include <esp_now.h>

#include <cassert>
#include <cstring>
#include <iostream>

int main() {
  using namespace peer_proto;
  // Ida y vuelta, y rechazo de paquetes ajenos o corruptos.
  const Packet p = encode(0x5A, 11, 7, 2, PUSH, true);
  Packet out;
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&p);
  assert(decode(bytes, sizeof(p), 0x5A, out) && out.id == 11 && out.target == 2 && out.stage == PUSH && out.can_move);
  assert(!decode(bytes, sizeof(p), 0x33, out));      // otro equipo
  assert(!decode(bytes, sizeof(p) - 1, 0x5A, out));  // tamaño
  uint8_t bad[sizeof(Packet)];
  memcpy(bad, bytes, sizeof(bad));
  bad[6] ^= 1;                                        // bit cambiado
  assert(!decode(bad, sizeof(bad), 0x5A, out));

  // Enlace: guarda al compañero, ignora el eco propio y vence a los 500 ms.
  PeerLink link;
  assert(link.begin(0x5A, 10));
  link.receive(bytes, sizeof(p), 1000);
  PeerStatus peer;
  assert(link.peer(11, 1400, peer) && peer.target == 2 && peer.stage == PUSH && peer.can_move);
  assert(!link.peer(11, 1501, peer));
  const Packet echo = encode(0x5A, 10, 1, 1, GOTO, true);
  link.receive(reinterpret_cast<const uint8_t*>(&echo), sizeof(echo), 1600);
  assert(!link.peer(10, 1600, peer));

  // Publica a 10 Hz como máximo.
  sim_espnow_out.clear();
  link.publish(1, GOTO, true, 2000);
  link.publish(1, GOTO, true, 2050);
  link.publish(1, GOTO, false, 2100);
  assert(sim_espnow_out.size() == 2);
  assert(decode(reinterpret_cast<const uint8_t*>(sim_espnow_out[1].data()), sim_espnow_out[1].size(), 0x5A, out) &&
         out.id == 10 && !out.can_move);
  std::cout << "Peer link checks passed\n";
}
