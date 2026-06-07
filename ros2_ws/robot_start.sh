#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v "$HOME/.platformio/penv/bin" | paste -sd: -)"
hash -r

source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

PORT="${AUDIX_PORT:-/dev/ttyAMA0}"
DASHBOARD_PORT="${AUDIX_DASHBOARD_PORT:-8080}"
MOCK_IR="${AUDIX_MOCK_IR:-false}"
MOCK_GPIO="${AUDIX_MOCK_GPIO:-false}"

IP_ADDR="$(hostname -I 2>/dev/null | awk '{print $1}')"
if [ -z "${IP_ADDR}" ]; then
  IP_ADDR="172.20.10.2"
fi

echo "Audix robot stack starting"
echo "  UART: ${PORT}"
echo "  Dashboard: http://${IP_ADDR}:${DASHBOARD_PORT}"
echo "  Press Ctrl+C to stop"

exec ros2 launch audix_robot audix_main.launch.py \
  port:="${PORT}" \
  dashboard_port:="${DASHBOARD_PORT}" \
  mock_ir:="${MOCK_IR}" \
  mock_gpio:="${MOCK_GPIO}"
