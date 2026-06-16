import argparse
import hashlib
from datetime import datetime
from pathlib import Path

import paramiko


FILES_TO_UPLOAD = [
    "audix_robot/audix_robot/navigation_contract.py",
    "audix_robot/audix_robot/micro_ros_base_node.py",
    "audix_robot/audix_robot/robot_manager_node.py",
]


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--local-src", required=True)
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", required=True)
    parser.add_argument("--password", required=True)
    parser.add_argument("--remote-src", required=True)
    args = parser.parse_args()

    local_src = Path(args.local_src)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

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
        for rel_path in FILES_TO_UPLOAD:
            local_path = local_src / rel_path
            remote_path = args.remote_src.rstrip("/") + "/" + rel_path
            backup_path = f"{remote_path}.pre_local_sync_{timestamp}"

            local_data = local_path.read_bytes()

            try:
                with sftp.open(remote_path, "rb") as remote_file:
                    remote_data = remote_file.read()
            except FileNotFoundError:
                remote_data = b""

            if remote_data:
                with sftp.open(backup_path, "wb") as backup_file:
                    backup_file.write(remote_data)

            with sftp.open(remote_path, "wb") as remote_file:
                remote_file.write(local_data)

            with sftp.open(remote_path, "rb") as remote_file:
                uploaded_data = remote_file.read()

            if sha256(local_data) != sha256(uploaded_data):
                raise RuntimeError(f"upload verification failed for {rel_path}")

            print(f"uploaded {rel_path}")
            print(f"  backup: {backup_path if remote_data else '<new file>'}")
            print(f"  sha256: {sha256(uploaded_data)}")
    finally:
        client.close()


if __name__ == "__main__":
    main()
