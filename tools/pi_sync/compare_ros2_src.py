import argparse
import difflib
import hashlib
import os
from pathlib import Path

import paramiko


TEXT_EXTENSIONS = {
    ".c",
    ".cc",
    ".cfg",
    ".cmake",
    ".cpp",
    ".h",
    ".hpp",
    ".launch",
    ".md",
    ".msg",
    ".py",
    ".srv",
    ".txt",
    ".xml",
    ".yaml",
    ".yml",
}

TEXT_FILENAMES = {"CMakeLists.txt", "package.xml", "setup.py", "setup.cfg", "requirements.txt"}
SKIP_DIRS = {"__pycache__", ".git", "build", "install", "log"}


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def is_text_file(path):
    name = Path(path).name
    suffix = Path(path).suffix.lower()
    return name in TEXT_FILENAMES or suffix in TEXT_EXTENSIONS


def walk_local(root):
    root = Path(root)
    files = {}
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        parts = set(path.relative_to(root).parts)
        if parts & SKIP_DIRS:
            continue
        rel = path.relative_to(root).as_posix()
        data = path.read_bytes()
        files[rel] = {
            "data": data,
            "size": len(data),
            "sha256": sha256_bytes(data),
            "text": is_text_file(rel),
        }
    return files


def sftp_walk(sftp, root):
    files = {}

    def visit(remote_dir, rel_prefix=""):
        for item in sftp.listdir_attr(remote_dir):
            name = item.filename
            if name in SKIP_DIRS:
                continue
            remote_path = remote_dir.rstrip("/") + "/" + name
            rel = f"{rel_prefix}/{name}" if rel_prefix else name
            if str(item.longname).startswith("d"):
                visit(remote_path, rel)
                continue
            with sftp.open(remote_path, "rb") as handle:
                data = handle.read()
            files[rel] = {
                "data": data,
                "size": len(data),
                "sha256": sha256_bytes(data),
                "text": is_text_file(rel),
            }

    visit(root)
    return files


def decode_text(data):
    try:
        return data.decode("utf-8").splitlines()
    except UnicodeDecodeError:
        return data.decode("latin-1").splitlines()


def entry_points(setup_py_text):
    lines = setup_py_text.splitlines()
    in_console_scripts = False
    entries = []
    for line in lines:
        stripped = line.strip()
        if "'console_scripts'" in stripped or '"console_scripts"' in stripped:
            in_console_scripts = True
            continue
        if in_console_scripts:
            if stripped.startswith("]"):
                in_console_scripts = False
                continue
            if ":" in stripped and "=" in stripped:
                entries.append(stripped.rstrip(",").strip("'\""))
    return entries


def classify(rel):
    parts = Path(rel).parts
    if len(parts) >= 3 and parts[1] in {"msg", "srv", "action"}:
        return "interface"
    if Path(rel).name == "setup.py":
        return "setup"
    if Path(rel).suffix == ".py" and len(parts) >= 3 and parts[1] == parts[0]:
        return "node_source"
    if "launch" in parts:
        return "launch"
    return "other"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--local", required=True)
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", required=True)
    parser.add_argument("--password", required=True)
    parser.add_argument("--remote", required=True)
    parser.add_argument("--report", required=True)
    args = parser.parse_args()

    local_files = walk_local(args.local)

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
        remote_files = sftp_walk(sftp, args.remote)
    finally:
        client.close()

    all_paths = sorted(set(local_files) | set(remote_files))
    only_local = [p for p in all_paths if p not in remote_files]
    only_remote = [p for p in all_paths if p not in local_files]
    changed = [
        p
        for p in all_paths
        if p in local_files and p in remote_files and local_files[p]["sha256"] != remote_files[p]["sha256"]
    ]

    report = []
    report.append("# ROS2 src comparison report")
    report.append("")
    report.append(f"Local: `{Path(args.local).resolve()}`")
    report.append(f"Remote: `{args.user}@{args.host}:{args.remote}`")
    report.append("")
    report.append("## Summary")
    report.append("")
    report.append(f"- Local files compared: {len(local_files)}")
    report.append(f"- Remote files compared: {len(remote_files)}")
    report.append(f"- Files only local: {len(only_local)}")
    report.append(f"- Files only remote: {len(only_remote)}")
    report.append(f"- Files changed: {len(changed)}")
    report.append("")

    for title, paths in (
        ("Files only local", only_local),
        ("Files only remote", only_remote),
        ("Changed files", changed),
    ):
        report.append(f"## {title}")
        report.append("")
        if not paths:
            report.append("_None_")
        else:
            for path in paths:
                kind = classify(path)
                meta = local_files.get(path) or remote_files.get(path)
                report.append(f"- `{path}` ({kind}, {meta['size']} bytes)")
        report.append("")

    interface_changes = [p for p in changed + only_local + only_remote if classify(p) == "interface"]
    node_changes = [p for p in changed + only_local + only_remote if classify(p) == "node_source"]
    setup_changes = [p for p in changed + only_local + only_remote if classify(p) == "setup"]

    report.append("## Different interfaces")
    report.append("")
    if not interface_changes:
        report.append("_None_")
    else:
        for path in sorted(interface_changes):
            report.append(f"- `{path}`")
    report.append("")

    added_or_removed_interfaces = [p for p in sorted(interface_changes) if p in only_local or p in only_remote]
    if added_or_removed_interfaces:
        report.append("## Added/removed interface contents")
        report.append("")
        for path in added_or_removed_interfaces:
            side = "local only" if path in only_local else "remote only"
            meta = local_files.get(path) or remote_files.get(path)
            report.append(f"### `{path}` ({side})")
            report.append("")
            if meta["text"]:
                report.append("```")
                report.extend(decode_text(meta["data"]))
                report.append("```")
            else:
                report.append(f"Binary/non-text interface: size {meta['size']} sha256 {meta['sha256']}")
            report.append("")

    report.append("## Different node sources")
    report.append("")
    if not node_changes:
        report.append("_None_")
    else:
        for path in sorted(node_changes):
            report.append(f"- `{path}`")
    report.append("")

    report.append("## Console script entry point differences")
    report.append("")
    setup_paths = sorted(set([p for p in all_paths if Path(p).name == "setup.py"] + setup_changes))
    if not setup_paths:
        report.append("_No setup.py files found._")
    for path in setup_paths:
        if path not in local_files or path not in remote_files:
            report.append(f"### `{path}`")
            report.append("")
            report.append("Only exists on one side.")
            report.append("")
            continue
        local_text = local_files[path]["data"].decode("utf-8", errors="replace")
        remote_text = remote_files[path]["data"].decode("utf-8", errors="replace")
        local_entries = entry_points(local_text)
        remote_entries = entry_points(remote_text)
        if local_entries == remote_entries:
            continue
        report.append(f"### `{path}`")
        report.append("")
        report.append("Local:")
        report.append("```")
        report.extend(local_entries or ["<none>"])
        report.append("```")
        report.append("Remote:")
        report.append("```")
        report.extend(remote_entries or ["<none>"])
        report.append("```")
        report.append("")

    report.append("## Line-by-line diffs")
    report.append("")
    for path in changed:
        local = local_files[path]
        remote = remote_files[path]
        report.append(f"### `{path}`")
        report.append("")
        if not (local["text"] and remote["text"]):
            report.append(
                f"Binary or non-text change: local size {local['size']} sha256 {local['sha256']}; "
                f"remote size {remote['size']} sha256 {remote['sha256']}."
            )
            report.append("")
            continue
        diff = difflib.unified_diff(
            decode_text(local["data"]),
            decode_text(remote["data"]),
            fromfile=f"local/{path}",
            tofile=f"remote/{path}",
            lineterm="",
        )
        report.append("```diff")
        report.extend(diff)
        report.append("```")
        report.append("")

    Path(args.report).write_text("\n".join(report), encoding="utf-8")


if __name__ == "__main__":
    main()
