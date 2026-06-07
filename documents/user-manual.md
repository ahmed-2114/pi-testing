# Audix ROS 2 User Manual

This manual is for the current real-robot ROS 2 workspace. It keeps the ESP firmware unchanged and uses ROS 2 on the Raspberry Pi to talk to the ESP and read Pi GPIO IR sensors.

## If Typing Does Nothing

Do not type movement commands into the terminal that is running `ros2 launch`. That terminal is running the launch process and often does not pass keyboard input to interactive nodes.

Use two terminals:

Terminal 1, start the bridge:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch audix_robot audix_bridge.launch.py port:=/dev/ttyAMA0
```

Terminal 2, run the move prompt:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run audix_robot terminal_move_node
```

Then type:

```text
move> F 20
move> R 15
move> stop
move> q
```

For keyboard teleop, keep Terminal 1 running and use this in Terminal 2:

```bash
ros2 run audix_robot terminal_teleop_node --ros-args -p rpm:=25.0
```

Then type one command and press Enter, for example `w`, `a`, `x`, or `i`.

The current checked-in workspace package is `audix_robot`, not `audix_bringup`. Commands that use `audix_bringup` need packages that are not present in this workspace yet.

## What Is Ready Now

Ready now in this workspace:

- ESP UART communication using the existing `esp_correct_pid_pi.ino` / `new_pid.ino` Pi protocol.
- ESP services: ping, init IMU, reset odom, reset encoders, stop, move, twist, raw command.
- IR GPIO publishing from the bridge using the same six sensor names, pins, and baseline logic as `pi_master.py`.
- Odometry topic from ESP telemetry.
- Terminal move mode via `ros2 run audix_robot terminal_move_node`.
- Terminal teleop mode via `ros2 run audix_robot terminal_teleop_node`.

Saved for later:

- Vision.
- Full RViz robot model with URDF.
- Separate `audix_bringup`, `audix_hardware`, `audix_mission`, and `audix_test_tools` packages.
- GPIO output services for buzzer, traffic LEDs, and stepper/lift.
- Path runner, IR safety node, IR train mode, and avoidance mission mode.

## Workspace Layout

```text
ros2_ws/
  AUDIX_USER_MANUAL.md
  AUDIX_MIGRATION_TODO.md
  src/
    audix_interfaces/   messages, services, actions
    audix_robot/        ESP UART bridge and terminal clients
```

## Copy To The Raspberry Pi

Copy or sync the full `ros2_ws` folder onto the Pi.

On the Pi:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If the Pi does not have dependencies:

```bash
sudo apt update
sudo apt install python3-serial python3-gpiozero python3-lgpio
```

## Main Launch File

Use this launch file for normal work:

```bash
ros2 launch audix_robot audix_bridge.launch.py port:=/dev/ttyAMA0
```

Use USB serial if the ESP32 is connected over USB:

```bash
ros2 launch audix_robot audix_bridge.launch.py port:=/dev/ttyUSB0
```

Run without real GPIO IR hardware:

```bash
ros2 launch audix_robot audix_bridge.launch.py mock_ir:=true
```

## Launch Modes

The current workspace does not have a multi-mode `audix_bringup` launch package yet. Start the bridge with `audix_robot audix_bridge.launch.py`, then run terminal clients or service calls from another terminal.

## Common Launch Examples

Terminal 1, normal bridge bringup:

```bash
ros2 launch audix_robot audix_bridge.launch.py port:=/dev/ttyAMA0
```

Terminal 1, USB serial:

```bash
ros2 launch audix_robot audix_bridge.launch.py port:=/dev/ttyUSB0
```

Terminal 1, laptop/mock GPIO:

```bash
ros2 launch audix_robot audix_bridge.launch.py mock_ir:=true
```

Terminal 2, manual move prompt:

```bash
ros2 run audix_robot terminal_move_node
```

At the move prompt:

```text
move> F 20
move> R 15
move> stop
move> q
```

Terminal 2, line-based teleop:

```bash
ros2 run audix_robot terminal_teleop_node --ros-args -p rpm:=25.0
```

At the teleop prompt, type one command and press Enter:

```text
teleop> w
teleop> a
teleop> x
teleop> i
```

Direct one-shot movement check:

```bash
ros2 service call /audix/move audix_interfaces/srv/Move "{angle_deg: 0.0, distance_m: 0.20, heading_deg: 0.0, timeout_s: 10.0, wait_for_done: true}"
```

Raw ESP status:

```bash
ros2 service call /audix/esp/raw_command audix_interfaces/srv/RawCommand "{command: 'STATUS', timeout_s: 3.0, wait_for_done: false}"
```

## Direct Run For Interactive Modes

Keep the bridge launch running in Terminal 1. Run interactive commands in Terminal 2:

```bash
ros2 run audix_robot terminal_move_node
ros2 run audix_robot terminal_teleop_node --ros-args -p rpm:=25.0
```

Typing into the bridge launch terminal will not move the robot.

## Important Topics

| Topic | Type | Meaning |
|---|---|---|
| `/audix/odom` | `nav_msgs/msg/Odometry` | Robot odometry from ESP telemetry |
| `/audix/esp/telemetry` | `audix_interfaces/msg/EspTelemetry` | Typed ESP telemetry |
| `/audix/esp/telemetry_raw` | `std_msgs/msg/String` | Raw ESP JSON |
| `/audix/ir/state` | `audix_interfaces/msg/IrState` | Six IR sensor states |

Useful checks:

```bash
ros2 topic echo /audix/esp/telemetry
ros2 topic echo /audix/odom
ros2 topic echo /audix/ir/state
```

## Important Services

ESP bridge:

```bash
ros2 service call /audix/esp/ping std_srvs/srv/Trigger {}
ros2 service call /audix/esp/init_imu std_srvs/srv/Trigger {}
ros2 service call /audix/esp/reset_odom std_srvs/srv/Trigger {}
ros2 service call /audix/esp/reset_encoders std_srvs/srv/Trigger {}
ros2 service call /audix/esp/stop std_srvs/srv/Trigger {}
```

Move:

```bash
ros2 service call /audix/move audix_interfaces/srv/Move "{angle_deg: 0.0, distance_m: 0.20, heading_deg: 0.0, timeout_s: 10.0, wait_for_done: true}"
```

Direction mapping:

| Angle | Robot movement |
|---|---|
| `0` | forward |
| `180` | backward |
| `-90` | right |
| `90` | left |

GPIO outputs for buzzer, traffic LEDs, and lift are saved for the later `audix_hardware` package. They are not available in the current `audix_robot` workspace.

## Suggested Test Order

1. Build the workspace.
2. Launch `ros2 launch audix_robot audix_bridge.launch.py port:=/dev/ttyAMA0`.
3. Test `/audix/esp/ping`.
4. Echo `/audix/esp/telemetry`.
5. Echo `/audix/odom`.
6. Test `/audix/move` manually at a short distance.
7. Echo `/audix/ir/state` and block each sensor by hand.
8. Run `ros2 run audix_robot terminal_move_node` in a second terminal and try `F 20`.
9. Run `ros2 run audix_robot terminal_teleop_node --ros-args -p rpm:=25.0` in a second terminal and try `w`, then `x`.

Do not start path/avoidance work until base movement, IR, and teleop pass.

## Notes About RViz And URDF

The bridge publishes `/audix/odom`. A later step will add TF publishing and a URDF/CAD-derived robot model for RViz.
