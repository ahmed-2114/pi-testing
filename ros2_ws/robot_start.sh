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
BAUD="${AUDIX_BAUD:-115200}"
DASHBOARD_PORT="${AUDIX_DASHBOARD_PORT:-8080}"
MOCK_IR="${AUDIX_MOCK_IR:-false}"
MOCK_GPIO="${AUDIX_MOCK_GPIO:-false}"
CAMERA_ENABLED="${AUDIX_CAMERA_ENABLED:-true}"
CAMERA_INDEX="${AUDIX_CAMERA_INDEX:-0}"
VISION_ENABLED="${AUDIX_VISION_ENABLED:-true}"
VISION_CONFIDENCE="${AUDIX_VISION_CONFIDENCE:-0.8}"
VISION_TARGET_COUNT="${AUDIX_VISION_TARGET_COUNT:-2}"
VISION_SCAN_SETTLE="${AUDIX_VISION_SCAN_SETTLE:-0.9}"
AUDIT_SIDE_1_LEVEL_1_SHELF_ID="${AUDIX_SIDE_1_LEVEL_1_SHELF_ID:-beans_can}"
AUDIT_SIDE_1_LEVEL_2_SHELF_ID="${AUDIX_SIDE_1_LEVEL_2_SHELF_ID:-indomie}"
AUDIT_SIDE_2_LEVEL_1_SHELF_ID="${AUDIX_SIDE_2_LEVEL_1_SHELF_ID:-indomie}"
AUDIT_SIDE_2_LEVEL_2_SHELF_ID="${AUDIX_SIDE_2_LEVEL_2_SHELF_ID:-fruit_rings_cereal}"
CLEAN_START="${AUDIX_CLEAN_START:-true}"

IP_ADDR="$(hostname -I 2>/dev/null | awk '{print $1}')"
if [ -z "${IP_ADDR}" ]; then
  IP_ADDR="172.20.10.2"
fi

for pkg in audix_robot audix_interfaces micro_ros_agent; do
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "ERROR: required ROS package '$pkg' is not available after sourcing install/setup.bash" >&2
    echo "Run: colcon build --symlink-install" >&2
    exit 1
  fi
done

cleanup_stale_stack() {
  if [ "${CLEAN_START}" != "true" ]; then
    return
  fi

  local patterns=(
    "ros2 launch audix_robot audix_main.launch.py"
    "micro_ros_agent.*serial"
    "install/audix_robot/lib/audix_robot/micro_ros_base_node"
    "install/audix_robot/lib/audix_robot/gpio_hardware_node"
    "install/audix_robot/lib/audix_robot/robot_manager_node"
    "install/audix_robot/lib/audix_robot/web_dashboard_node"
    "install/warehouse_vision/lib/warehouse_vision/webcam_node"
    "install/warehouse_vision/lib/warehouse_vision/vision_audit_node"
  )

  local pids=""
  local pattern
  for pattern in "${patterns[@]}"; do
    pids="${pids} $(pgrep -f "${pattern}" 2>/dev/null || true)"
  done

  pids="$(printf '%s\n' ${pids} 2>/dev/null | sort -nu | tr '\n' ' ')"
  if [ -z "${pids// }" ]; then
    return
  fi

  echo "Stopping stale Audix processes: ${pids}"
  kill -TERM ${pids} 2>/dev/null || true
  sleep 2

  local alive=""
  local pid
  for pid in ${pids}; do
    if kill -0 "${pid}" 2>/dev/null; then
      alive="${alive} ${pid}"
    fi
  done
  if [ -n "${alive// }" ]; then
    echo "Force stopping stale Audix processes:${alive}"
    kill -KILL ${alive} 2>/dev/null || true
    sleep 1
  fi
}

cleanup_stale_stack

echo "Audix robot stack starting"
echo "  UART: ${PORT}"
echo "  Baud: ${BAUD}"
echo "  Dashboard: http://${IP_ADDR}:${DASHBOARD_PORT}"
echo "  Camera: enabled=${CAMERA_ENABLED} index=${CAMERA_INDEX}"
echo "  Vision: enabled=${VISION_ENABLED} confidence=${VISION_CONFIDENCE} settle=${VISION_SCAN_SETTLE}s"
echo "  Press Ctrl+C to stop"

exec ros2 launch audix_robot audix_main.launch.py \
  port:="${PORT}" \
  baud:="${BAUD}" \
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
