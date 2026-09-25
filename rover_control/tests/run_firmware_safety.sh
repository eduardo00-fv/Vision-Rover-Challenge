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
