#!/usr/bin/env bash
# Compila el firmware real una vez por rover (10 y 11) y el simulador.
# Uso: bash rover_control/sim/build.sh [ruta/ArduinoJson/src] [directorio_salida]
set -euo pipefail
sim_dir="$(cd "$(dirname "$0")" && pwd)"
fw_dir="$(cd "$sim_dir/.." && pwd)"
json_include="${1:-$HOME/Arduino/libraries/ArduinoJson/src}"
out_dir="${2:-$sim_dir/build}"
[[ -f "$json_include/ArduinoJson.h" ]] || { echo "No encuentro ArduinoJson.h en $json_include" >&2; exit 1; }
mkdir -p "$out_dir"
rm -f "$out_dir"/fw_*.so "$out_dir/vrc_sim"  # nunca correr binarios de una compilación anterior
# -fno-gnu-unique + visibilidad oculta: dos copias del firmware en un proceso
# sin compartir globales (WiFiClient::incoming, Wire, sim_time_us...).
# SIM_DEFINES="-DARENA_SETTLE_MS=200U ..." prueba otros parámetros sin tocar config.
read -r -a extra <<< "${SIM_DEFINES:-}"
flags=(-std=c++17 -O2 -fPIC -shared -fvisibility=hidden -fvisibility-inlines-hidden
       -fno-gnu-unique -Wl,-Bsymbolic -Wno-deprecated-declarations)
for id in 10 11; do
  stage="$out_dir/stage_$id"
  rm -rf "$stage"; mkdir -p "$stage"
  cp "$fw_dir"/*.cpp "$fw_dir"/*.h "$fw_dir/rover_control.ino" "$stage/"
  cp -R "$fw_dir/motion" "$stage/"
  # Parámetros de competencia de config.example.h; solo cambia el marcador.
  # Ambos usan el perfil físico 1 hasta calibrar el rover 2.
  sed -e "s/constexpr uint8_t ROVER_ID = 10;/constexpr uint8_t ROVER_ID = $id;/" \
      "$fw_dir/config.example.h" > "$stage/config.h"
  grep -q "ROVER_ID = $id;" "$stage/config.h"
  g++ "${flags[@]}" ${extra[@]+"${extra[@]}"} -I"$sim_dir/host" -I"$stage" -I"$sim_dir" -I"$json_include" \
      "$sim_dir/firmware_entry.cpp" "$stage"/telemetry_client.cpp "$stage"/world_state.cpp \
      "$stage"/safety_supervisor.cpp "$stage"/mission_controller.cpp "$stage"/motor_controller.cpp \
      "$stage"/sensors.cpp "$stage"/peer_link.cpp "$stage"/vision_navigator.cpp "$stage"/rover_logic_core.cpp \
      -o "$out_dir/fw_$id.so"
done
g++ -std=c++17 -O2 -Wall -Wextra -I"$sim_dir" "$sim_dir/sim.cpp" -ldl -o "$out_dir/vrc_sim"
echo "Listo: $out_dir/vrc_sim"
