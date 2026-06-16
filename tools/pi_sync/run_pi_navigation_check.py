import argparse

import paramiko


REMOTE_CHECK = r"""
cd /home/audix/audix/audix-integration/ros2_ws/src &&
python3 -m py_compile \
  audix_robot/audix_robot/navigation_contract.py \
  audix_robot/audix_robot/micro_ros_base_node.py \
  audix_robot/audix_robot/robot_manager_node.py &&
PYTHONPATH=/home/audix/audix/audix-integration/ros2_ws/src/audix_robot python3 - <<'PY'
from audix_robot.navigation_contract import (
    body_delta_to_world,
    forward_heading_for_world_direction,
    rotation_delta_for_turn_direction,
)

tests = [
    (0.0, (10.0, 0.0)),
    (90.0, (0.0, 10.0)),
    (-90.0, (0.0, -10.0)),
    (180.0, (-10.0, 0.0)),
]
for heading, expected in tests:
    got = body_delta_to_world(10.0, 0.0, heading)
    assert abs(got[0] - expected[0]) < 1e-6 and abs(got[1] - expected[1]) < 1e-6, (heading, got, expected)

right_strafe = body_delta_to_world(0.0, 10.0, 0.0)
assert abs(right_strafe[0]) < 1e-6 and abs(right_strafe[1] + 10.0) < 1e-6, right_strafe
assert forward_heading_for_world_direction("L") == 90.0
assert forward_heading_for_world_direction("R") == -90.0
assert rotation_delta_for_turn_direction("left", 90.0) == 90.0
assert rotation_delta_for_turn_direction("ccw", 90.0) == 90.0
assert rotation_delta_for_turn_direction("right", 90.0) == -90.0
assert rotation_delta_for_turn_direction("cw", 90.0) == -90.0
print("pi navigation contract checks passed")
PY
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
        _, stdout, stderr = client.exec_command(REMOTE_CHECK)
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
