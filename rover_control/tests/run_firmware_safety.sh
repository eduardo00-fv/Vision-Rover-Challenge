#!/usr/bin/env bash
set -euo pipefail
source_dir="$(cd "$(dirname "$0")/.." && pwd)"
json_include="${1:?Indique el directorio src de ArduinoJson}"
test_stage="$(mktemp -d /tmp/vrc-host-tests.XXXXXX)"
# Copia aislada para que el config.h privado nunca reemplace la configuración de prueba.
cp "$source_dir"/*.cpp "$source_dir"/*.h "$test_stage/"
cp "$source_dir/tests/host/config.h" "$test_stage/config.h"
g++ -std=c++17 -I"$source_dir/tests/host" -I"$test_stage" -I"$json_include" \
  "$source_dir/tests/test_firmware_safety.cpp" "$test_stage/telemetry_client.cpp" \
  "$test_stage/world_state.cpp" "$test_stage/safety_supervisor.cpp" \
  "$test_stage/motor_controller.cpp" \
  "$test_stage/vision_navigator.cpp" "$test_stage/rover_logic_core.cpp" \
  -o "$test_stage/test_safety"
"$test_stage/test_safety"
g++ -std=c++17 -I"$source_dir/tests/host" "$source_dir/tests/test_peer_link.cpp" "$test_stage/peer_link.cpp" \
  -o "$test_stage/test_peer_link"
"$test_stage/test_peer_link"
g++ -std=c++17 -I"$source_dir/tests/host" "$source_dir/tests/test_start_button.cpp" -o "$test_stage/test_start_button"
"$test_stage/test_start_button"
g++ -std=c++17 -Wall "$source_dir/tests/test_line_guard.cpp" -o "$test_stage/test_line_guard"
"$test_stage/test_line_guard"
# Compilación de competencia: debe rechazar banco, solo-red y botón.
competition_check() {
  echo '#include "hardware_config.h"' | g++ -std=c++17 -fsyntax-only -x c++ -I"$source_dir/tests/host" \
    -I"$test_stage" -DVRC_COMPETITION=1 -DVRC_PHYSICAL_ROVER=1 "$@" - 2>/dev/null
}
competition_check
for bad in VRC_ENABLE_MOTOR_BENCH VRC_NETWORK_ONLY VRC_REQUIRE_START_BUTTON; do
  if competition_check -D"$bad"=1; then echo "Competencia aceptó $bad=1" >&2; exit 1; fi
done
if competition_check -UVRC_PHYSICAL_ROVER -DVRC_PHYSICAL_ROVER=0; then echo "Competencia aceptó perfil 0" >&2; exit 1; fi
echo "Competition build checks passed"
