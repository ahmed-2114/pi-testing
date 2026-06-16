# Pi Sync And Audit Tools

These scripts were created to compare and synchronize the local ROS 2 workspace
with the Audix Raspberry Pi workspace:

```text
Pi: audix@172.20.10.2
Remote workspace: /home/audix/audix/audix-integration
Remote ROS src:   /home/audix/audix/audix-integration/ros2_ws/src
```

Run these scripts from the repository root unless you pass fully qualified
paths.

## Scripts

- `compare_ros2_src.py`
  - Compares local `ros2_ws/src` against the Pi `ros2_ws/src`.
  - Skips generated folders like `__pycache__`, `.git`, `build`, `install`, and `log`.
  - Produces a Markdown line-by-line diff report.

- `fetch_pi_ros2_files.py`
  - Fetches selected active Pi ROS files into a local audit folder for inspection.
  - Use this for read-only investigation before changing code.

- `upload_selected_ros2_files.py`
  - Uploads selected ROS package files to the Pi and creates timestamped backups.
  - Current scope is the navigation contract and related ROS nodes.

- `upload_workspace_files.py`
  - Uploads selected documentation files to the Pi and creates timestamped backups.
  - ESP firmware is intentionally excluded.

- `build_pi_audix_robot.py`
  - Rebuilds only the `audix_robot` ROS package on the Pi.
  - Also runs a small navigation-contract import check.

- `run_pi_navigation_check.py`
  - Runs Pi-side Python assertions for the navigation contract.

- `run_pi_command.py`
  - Small Paramiko helper for one-off remote commands.
  - Use sparingly; prefer the purpose-built scripts above.

## Navigation Contract

The code contract lives in:

```text
ros2_ws/src/audix_robot/audix_robot/navigation_contract.py
```

The human-readable contract lives in:

```text
documents/audix_navigation_contract.md
```

Key convention:

```text
+x forward
+y left
+z up

heading 0 deg    = forward
heading 90 deg   = left
heading -90 deg  = right
heading 180 deg  = backward

left / CCW rotation  = positive yaw
right / CW rotation  = negative yaw
```

## ESP Firmware Rule

Do not use these tools to build, flash, upload, or deploy ESP firmware.

The ESP firmware is an Arduino `.ino` workflow and must be uploaded manually by
the user. Codex may edit or review the `.ino` source only when asked, then stop
and clearly tell the user to upload it manually.
