import argparse

import paramiko


REMOTE_COMMAND = r"""
bash -lc '
set -e
cd /home/audix/audix/audix-integration/ros2_ws
ros_distro=""
for d in /opt/ros/*; do
  if [ -d "$d" ]; then ros_distro="$(basename "$d")"; break; fi
done
if [ -z "$ros_distro" ]; then
  echo "No ROS distro found under /opt/ros" >&2
  exit 1
fi
source "/opt/ros/$ros_distro/setup.bash"
colcon build --packages-select audix_robot
source install/setup.bash
PYTHONPATH=/home/audix/audix/audix-integration/ros2_ws/src/audix_robot python3 - <<PY
from audix_robot.navigation_contract import body_delta_to_world, rotation_delta_for_turn_direction
assert body_delta_to_world(10, 0, 90)[1] > 9.999
assert body_delta_to_world(10, 0, -90)[1] < -9.999
assert rotation_delta_for_turn_direction("left", 90) == 90.0
assert rotation_delta_for_turn_direction("right", 90) == -90.0
print("audix_robot rebuilt and contract import verified")
PY
'
"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", required=True)
    parser.add_argument("--password", required=True)
    args = parser.parse_args()

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        hostname=args.host,
        username=args.user,
        password=args.password,
        timeout=12,
        banner_timeout=12,
        auth_timeout=12,
        look_for_keys=False,
        allow_agent=False,
    )
    try:
        _, stdout, stderr = client.exec_command(REMOTE_COMMAND)
        out = stdout.read().decode("utf-8", errors="replace")
        err = stderr.read().decode("utf-8", errors="replace")
        status = stdout.channel.recv_exit_status()
        if out:
            print(out, end="")
        if err:
            print(err, end="")
        if status != 0:
            raise SystemExit(status)
    finally:
        client.close()


if __name__ == "__main__":
    main()
