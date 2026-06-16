# Audix Navigation Update Summary

This change set makes the robot navigation frame explicit and keeps the ROS
nodes, docs, and Pi sync tooling aligned.

## What Changed

- Added `audix_robot/navigation_contract.py`.
  - Defines the shared frame contract in code.
  - Provides helpers for heading wrapping, body-to-world odometry projection,
    world-direction headings, and signed yaw deltas.

- Added `documents/audix_navigation_contract.md`.
  - Human-readable version of the same contract.
  - Documents the ESP firmware manual-upload rule.

- Updated `micro_ros_base_node.py`.
  - Uses the shared body-to-world projection.
  - Treats ESP body strafe as right-positive while Audix world strafe remains
    left-positive.

- Updated `robot_manager_node.py`.
  - Uses shared heading and yaw helpers.
  - Keeps left/CCW rotation as positive yaw and right/CW as negative yaw.
  - Keeps lateral travel as rotate-to-heading plus forward drive.

- Updated `ros2_ws/firmware/audix_esp32_microros/audix_esp32_microros.ino`.
  - The source now keeps heading-controller output in logical/right-hand-rule
    yaw and applies `rotateSign` once at wheel mixing.
  - The ESP must still be uploaded manually through the Arduino workflow.

- Updated documentation.
  - README and handoff docs now describe the same frame/yaw contract.
  - Stale relative shelf-rotation descriptions were replaced with absolute
    heading descriptions.

- Added `tools/pi_sync/`.
  - Contains comparison, upload, Pi rebuild, and Pi-side contract check helpers.
  - ESP firmware is intentionally excluded from automation.

## Navigation Contract

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

ESP firmware is Arduino `.ino` firmware. Codex/tooling must not build, flash,
upload, or deploy it. Source may be edited or reviewed, but the user uploads it
manually.

## Verification Performed

- Python syntax checks for the changed ROS nodes and helper tools.
- Local navigation-contract assertions.
- Pi-side `audix_robot` rebuild and navigation-contract checks during the work.

Restart/relaunch ROS nodes after pulling these changes onto the Pi. Upload the
ESP firmware manually if the `.ino` source change is needed on hardware.
