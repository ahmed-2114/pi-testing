#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v "$HOME/.platformio/penv/bin" | paste -sd: -)"
hash -r

set +u
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u

PORT="${AUDIX_PORT:-/dev/ttyAMA0}"
DASHBOARD_PORT="${AUDIX_DASHBOARD_PORT:-8080}"
MOCK_IR="${AUDIX_MOCK_IR:-false}"
MOCK_GPIO="${AUDIX_MOCK_GPIO:-false}"
CAMERA_ENABLED="${AUDIX_CAMERA_ENABLED:-true}"
CAMERA_INDEX="${AUDIX_CAMERA_INDEX:-0}"
VISION_ENABLED="${AUDIX_VISION_ENABLED:-true}"
VISION_CONFIDENCE="${AUDIX_VISION_CONFIDENCE:-0.5}"
VISION_TARGET_COUNT="${AUDIX_VISION_TARGET_COUNT:-2}"
VISION_SCAN_SETTLE="${AUDIX_VISION_SCAN_SETTLE:-0.5}"
AUDIT_SIDE_1_LEVEL_1_SHELF_ID="${AUDIX_SIDE_1_LEVEL_1_SHELF_ID:-indomie}"
AUDIT_SIDE_1_LEVEL_2_SHELF_ID="${AUDIX_SIDE_1_LEVEL_2_SHELF_ID:-beans_can}"
AUDIT_SIDE_2_LEVEL_1_SHELF_ID="${AUDIX_SIDE_2_LEVEL_1_SHELF_ID:-fruit_rings_cereal}"
AUDIT_SIDE_2_LEVEL_2_SHELF_ID="${AUDIX_SIDE_2_LEVEL_2_SHELF_ID:-indomie}"

IP_ADDR="$(hostname -I 2>/dev/null | awk '{print $1}')"
if [ -z "${IP_ADDR}" ]; then
  IP_ADDR="172.20.10.2"
fi

echo "Audix robot stack starting"
echo "  UART: ${PORT}"
echo "  Dashboard: http://${IP_ADDR}:${DASHBOARD_PORT}"
echo "  Camera: enabled=${CAMERA_ENABLED} index=${CAMERA_INDEX}"
echo "  Vision: enabled=${VISION_ENABLED} confidence=${VISION_CONFIDENCE} settle=${VISION_SCAN_SETTLE}s"
echo "  Press Ctrl+C to stop"

exec ros2 launch audix_robot audix_main.launch.py \
  port:="${PORT}" \
  dashboard_port:="${DASHBOARD_PORT}" \
  mock_ir:="${MOCK_IR}" \
  mock_gpio:="${MOCK_GPIO}" \
  camera_enabled:="${CAMERA_ENABLED}" \
  camera_index:="${CAMERA_INDEX}" \
  vision_enabled:="${VISION_ENABLED}" \
  vision_confidence:="${VISION_CONFIDENCE}" \
  vision_target_count:="${VISION_TARGET_COUNT}" \
  vision_scan_settle:="${VISION_SCAN_SETTLE}" \
  audit_side_1_level_1_shelf_id:="${AUDIT_SIDE_1_LEVEL_1_SHELF_ID}" \
  audit_side_1_level_2_shelf_id:="${AUDIT_SIDE_1_LEVEL_2_SHELF_ID}" \
  audit_side_2_level_1_shelf_id:="${AUDIT_SIDE_2_LEVEL_1_SHELF_ID}" \
  audit_side_2_level_2_shelf_id:="${AUDIT_SIDE_2_LEVEL_2_SHELF_ID}"
