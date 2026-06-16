# Audix Navigation Contract

This file is the human-readable version of `audix_robot/navigation_contract.py`.
All ROS nodes, GUI controls, mission logic, and ESP command semantics should agree
with this frame.

## World Frame

```text
+x / forward_cm = forward from spawn
-x              = backward toward home
+y / strafe_cm  = left
-y / strafe_cm  = right
+z              = up
```

## Heading And Yaw

Audix uses the standard right-handed planar robotics convention:

```text
heading 0 deg    = forward / +x
heading 90 deg   = left / +y
heading -90 deg  = right / -y
heading 180 deg  = backward / -x
```

Yaw is rotation around `+z`.

```text
left / CCW rotation  = positive yaw change
right / CW rotation  = negative yaw change
```

Examples:

```text
drive forward while heading 90 deg   -> world +strafe / +y
drive forward while heading -90 deg  -> world -strafe / -y
drive forward while heading 180 deg  -> world -forward / -x
rotate left 90 deg                   -> target heading +90
rotate right 90 deg                  -> target heading -90
```

## ESP Boundary

The ESP firmware may keep calibration signs such as `rotateSign`, `strafeSign`,
and `odomYawSign` for physical motor and sensor wiring. Those signs must remain
at the hardware boundary. ROS-facing telemetry and ROS-facing commands should
use the contract above.

For position heading control, the ESP yaw controller works in displayed/logical
yaw. The wheel mixer applies `rotateSign` exactly once when converting the
logical rotate command into motor RPM targets.

## ESP Firmware Handling Rule

ESP firmware is Arduino `.ino` firmware and is uploaded manually by the user
through the Arduino workflow. Codex must not build, flash, upload, or otherwise
try to deploy ESP firmware. Codex may edit or review the `.ino` source only when
asked, then clearly tell the user that the ESP must be uploaded manually.
