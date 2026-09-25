#!/usr/bin/env bash
set -euo pipefail
source_dir="$(cd "$(dirname "$0")/.." && pwd)"
json_include="${1:?Indique el directorio src de ArduinoJson}"
test_stage="$(mktemp -d /tmp/vrc-arena-tests.XXXXXX)"
cp "$source_dir"/*.cpp "$source_dir"/*.h "$source_dir/rover_control.ino" "$test_stage/"
cp -R "$source_dir/motion" "$test_stage/"
cp "$source_dir/config.example.h" "$test_stage/config.h"
cp "$source_dir/tests/host/WiFi.h" "$test_stage/"
g++ -std=c++17 -I"$source_dir/tests/diagnostic_host" \
  "$source_dir/tests/test_arena_motion.cpp" -o "$test_stage/test_motion"
"$test_stage/test_motion"
for network_only in 0 1; do
g++ -std=c++17 -DVRC_NETWORK_ONLY="$network_only" -I"$source_dir/tests/diagnostic_host" -I"$test_stage" -I"$json_include" \
  "$source_dir/tests/test_arena_firmware.cpp" "$test_stage/telemetry_client.cpp" \
  "$test_stage/world_state.cpp" "$test_stage/safety_supervisor.cpp" \
  "$test_stage/mission_controller.cpp" "$test_stage/motor_controller.cpp" \
  "$test_stage/sensors.cpp" "$test_stage/peer_link.cpp" "$test_stage/vision_navigator.cpp" "$test_stage/rover_logic_core.cpp" \
  -o "$test_stage/test_firmware"
"$test_stage/test_firmware"
done
