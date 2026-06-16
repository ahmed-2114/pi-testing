import argparse
import hashlib
from datetime import datetime
from pathlib import Path

import paramiko


FILES_TO_UPLOAD = [
    "documents/audix_navigation_contract.md",
    "documents/audix_navigation_simulation_handoff.md",
    "documents/README_AUDIX_MICRO_ROS.md",
    "documents/README_AUDIX_ROS.md",
    "documents/ros.md",
    "documents/user-manual.md",
    "documents/UART_SETUP_GUIDE_extracted.txt",
]

# ESP firmware is intentionally excluded. It is an Arduino .ino workflow and
# must be uploaded manually by the user, never by Codex automation.


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def ensure_remote_dir(sftp, remote_dir: str) -> None:
    parts = [part for part in remote_dir.split("/") if part]
    path = ""
    for part in parts:
        path += "/" + part
        try:
            sftp.stat(path)
        except FileNotFoundError:
            sftp.mkdir(path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--local-root", required=True)
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", required=True)
    parser.add_argument("--password", required=True)
    parser.add_argument("--remote-root", required=True)
    args = parser.parse_args()

    local_root = Path(args.local_root)
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
            local_path = local_root / rel_path
            remote_path = args.remote_root.rstrip("/") + "/" + rel_path.replace("\\", "/")
            remote_dir = str(Path(remote_path).parent).replace("\\", "/")
            backup_path = f"{remote_path}.pre_local_sync_{timestamp}"

            local_data = local_path.read_bytes()
            ensure_remote_dir(sftp, remote_dir)
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
