# Audix micro-ROS Migration

This workspace now has a micro-ROS path for the ESP32 base controller. The old
ESP UART JSON bridge is no longer the default launch path. The RX/TX serial link
is used by `micro_ros_agent`, and the ESP32 runs a micro-ROS client in its own
FreeRTOS task.

## Architecture

```text
Raspberry Pi ROS 2
  /micro_ros_agent
      serial RX/TX to ESP32

  /audix/micro_ros_base
      converts ESP telemetry JSON into audix_interfaces/msg/EspTelemetry
      exposes the existing /audix/move and /audix/esp/* services
      sends MOVE/TWIST/STOP/RESET/INIT commands through /audix/esp/move_goal

  /audix/gpio_hardware
      publishes /audix/ir/state
      exposes buzzer and lift GPIO services

ESP32 micro-ROS firmware
  /audix/esp32_controller
      subscribes /audix/esp/move_goal
      publishes /audix/odom
      publishes /audix/esp/telemetry_json
```

The ESP32 intentionally uses one ROS command subscription. Stop, twist, reset
odometry, reset encoders, IMU init, and position moves all pass through
`/audix/esp/move_goal` as command strings. This keeps the Arduino
`micro_ros_arduino` resource usage low enough for the telemetry publishers to
be created reliably.

The ESP32 PID and mecanum control loop are intentionally preserved. The firmware
still runs the original high-priority `controlTask` on core 1 at the same control
period. micro-ROS runs separately as `microRosTask` on core 0 and only updates
the same command variables that the UART bridge used before.

## Firmware

New firmware sketch:

```text
ros2_ws/firmware/audix_esp32_microros/audix_esp32_microros.ino
```

The sketch uses `micro_ros_arduino`, `geometry_msgs`, `nav_msgs`, and `std_msgs`.
It does not require the custom Audix interfaces on the ESP because the Pi-side
`micro_ros_base_node` converts the ESP JSON telemetry into `audix_interfaces`.

Important serial note: once this firmware is flashed, do not run a serial monitor
or the old `esp_uart_bridge_node` on the same port. The serial port belongs to
`micro_ros_agent`.

## Build The ROS Workspace

```bash
cd ~/pi-testing/ros2_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

Use Jazzy instead of Humble if that is what is installed:

```bash
source /opt/ros/jazzy/setup.bash
```

## Run

Launch the full robot stack with the micro-ROS Agent on the ESP serial port:

```bash
ros2 launch audix_robot audix_main.launch.py port:=/dev/ttyAMA0 baud:=115200
```

USB serial:

```bash
ros2 launch audix_robot audix_main.launch.py port:=/dev/ttyUSB0 baud:=115200
```

Base-only launch:

```bash
ros2 launch audix_robot audix_bridge.launch.py port:=/dev/ttyAMA0 baud:=115200
```

If you want to run the Agent manually:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 --baudrate 115200
```

Some installs use this baud syntax instead:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 baudrate=115200
```

## ESP Command Topic

The ESP command topic is:

```text
/audix/esp/move_goal    std_msgs/msg/String
```

The command strings intentionally match the old working ESP protocol, but they
are transported by micro-ROS instead of UART JSON:

```text
MOVE  seq=<n> angle=<deg> dist=<cm> heading=<deg> timeout=<ms>
TWIST seq=<n> forward=<rpm> strafe=<rpm> turn=<rpm> timeout=<ms>
STOP  seq=<n>
RESET_ODOM seq=<n>
RESET_ENC seq=<n>
INIT_IMU seq=<n>
```

Short forward test:

```bash
ros2 topic pub --once /audix/esp/move_goal std_msgs/msg/String "{data: 'TWIST seq=1 forward=20.0 strafe=0.0 turn=0.0 timeout=300'}"
```

Stop:

```bash
ros2 service call /audix/manager/stop std_srvs/srv/Trigger {}
```

## Position Move Service

The manager and dashboard still use the existing Audix position service:

```text
/audix/move    audix_interfaces/srv/Move
```

Forward 10 cm:

```bash
ros2 service call /audix/move audix_interfaces/srv/Move "{angle_deg: 0.0, distance_m: 0.10, heading_deg: 0.0, timeout_s: 10.0, wait_for_done: true}"
```

Strafe right 10 cm:

```bash
ros2 service call /audix/move audix_interfaces/srv/Move "{angle_deg: -90.0, distance_m: 0.10, heading_deg: 0.0, timeout_s: 10.0, wait_for_done: true}"
```

The ESP receives this as a ROS topic command on `/audix/esp/move_goal`, not as a
UART text command.

## Inspect The ROS Graph

List nodes:

```bash
ros2 node list
```

Expected important nodes:

```text
/micro_ros_agent
/audix/esp32_controller
/audix/micro_ros_base
/audix/gpio_hardware
/audix/robot_manager
/audix/web_dashboard
```

List topics with types:

```bash
ros2 topic list -t
```

List services with types:

```bash
ros2 service list -t
```

Show all publishers/subscribers for a node:

```bash
ros2 node info /audix/esp32_controller
ros2 node info /audix/micro_ros_base
ros2 node info /audix/gpio_hardware
ros2 node info /audix/robot_manager
```

Show publishers/subscribers for a topic:

```bash
ros2 topic info /audix/esp/move_goal -v
ros2 topic info /audix/odom -v
ros2 topic info /audix/esp/telemetry_json -v
ros2 topic info /audix/esp/telemetry -v
ros2 topic info /audix/ir/state -v
```

Echo live telemetry:

```bash
ros2 topic echo /audix/esp/telemetry
ros2 topic echo /audix/esp/telemetry_json
ros2 topic echo /audix/odom
ros2 topic echo /audix/ir/state
```

Show message and service definitions:

```bash
ros2 interface show std_msgs/msg/String
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show audix_interfaces/msg/EspTelemetry
ros2 interface show audix_interfaces/srv/Move
```

## Safe Test Order

1. Flash `audix_esp32_microros.ino`.
2. Start `audix_bridge.launch.py` with `mock_ir:=true` first if the IR wiring is not ready.
3. Confirm `/audix/esp32_controller` appears in `ros2 node list`.
4. Echo `/audix/esp/telemetry_json`.
5. Echo `/audix/odom`.
6. Call `/audix/esp/reset_odom`.
7. Publish a very small `TWIST` command on `/audix/esp/move_goal`.
8. Call a 10 cm `/audix/move`.
9. Launch the full `audix_main.launch.py` stack.
