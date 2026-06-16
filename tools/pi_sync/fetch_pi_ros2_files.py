import argparse
from pathlib import Path

import paramiko


FILES_TO_FETCH = [
    "audix_robot/audix_robot/gpio_hardware_node.py",
    "audix_robot/audix_robot/micro_ros_base_node.py",
    "audix_robot/audix_robot/robot_manager_node.py",
    "audix_robot/audix_robot/web_dashboard_node.py",
    "audix_robot/launch/audix_main.launch.py",
    "warehouse_vision/warehouse_vision/vision_audit_node.py",
]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", required=True)
    parser.add_argument("--password", required=True)
    parser.add_argument("--remote-src", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

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
        sftp = client.open_sftp()
        for rel_path in FILES_TO_FETCH:
            remote_path = args.remote_src.rstrip("/") + "/" + rel_path
            local_path = out_dir / rel_path
            local_path.parent.mkdir(parents=True, exist_ok=True)
            with sftp.open(remote_path, "rb") as remote_file:
                local_path.write_bytes(remote_file.read())
            print(f"fetched {rel_path}")
    finally:
        client.close()


if __name__ == "__main__":
    main()
