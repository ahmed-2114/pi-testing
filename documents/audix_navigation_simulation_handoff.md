# Audix Navigation and Simulation Handoff

This document summarizes the current real-life Audix navigation system and the recommended approach for rebuilding a Gazebo/RViz simulation from the upgraded logic. It is intended as context for another AI chat or developer.

## Goal

Recreate the current real robot behavior in simulation by making the simulation expose the same ROS interfaces as the real hardware stack. The high-level mission and avoidance logic should stay the same; only the hardware-facing layer should be replaced by simulation nodes.

## Current Architecture

The real system is organized around a high-level manager node and a hardware/base interface.

Real robot path:

```text
robot_manager_node.py
  -> /move service
  -> micro_ros_base_node.py or esp_uart_bridge_node.py
  -> ESP32 firmware
  -> mecanum base
```

Recommended simulation path:

```text
robot_manager_node.py
  -> /move service
  -> sim_base_node.py
  -> Gazebo/RViz robot model
```

The important idea is that `robot_manager_node.py` should not need to know whether it is controlling the real robot or the simulated robot.

## ROS Interfaces To Mirror In Simulation

The simulation should implement or publish the same interfaces used by the real stack:

Services:

- `/move`
- `/twist`
- `/esp/stop`
- `/esp/reset_odom`
- `/esp/init_imu`
- `/manager/start_audit`
- `/manager/direction_move`
- `/manager/rotate`
- `/manager/go_home`
- `/manager/stop`
- `/lift/move_steps`
- `/scan_shelf`
- `/gpio/set_buzzer`

Topics:

- `/esp/telemetry`
- `/esp/telemetry_raw`
- `/ir/state`
- `/mission/event`
- `/vision/scan_result`
- `/odom`
- `/image_raw`, if camera simulation is included

## Direction Convention

The manager uses these direction angles:

```text
F  = 0 deg
FR = -45 deg
R  = -90 deg
BR = -135 deg
B  = 180 deg
BL = 135 deg
L  = 90 deg
FL = 45 deg
```

Lateral direction constants:

```text
LEFT  = -1
RIGHT = 1
```

`direction_to_angle(direction)` returns:

```text
LEFT  -> 90 deg
RIGHT -> -90 deg
```

## Reversed Heading Logic

The robot is considered reversed when:

```text
abs(wrap_degrees(current_heading)) >= 135 deg
```

Default threshold:

```text
reverse_heading_threshold_deg = 135.0
```

When heading is reversed, world/map directions are flipped before being sent as body commands:

```text
world F -> body B
world B -> body F
world L -> body R
world R -> body L
```

For avoidance lateral commands:

```text
global/map right -> body left   when heading is reversed
global/map left  -> body right  when heading is reversed
```

The log labels show this as, for example:

```text
right (body left)
left (body right)
```

## Real Robot Base Constants

From the ESP32 micro-ROS firmware:

```text
Wheel diameter:              9.7 cm
Encoder counts per rev:      4346.8
RPM target hard limit:       45 RPM
Position max RPM default:    40 RPM
Position tolerance default:  2.0 cm
Heading tolerance default:   2.0 deg
RPM filter alpha:            0.20
Command max timed run:       120000 ms
```

Odometry calibration:

```text
odomForwardSign  = 1
odomStrafeSign   = -1
odomYawSign      = -1
odomForwardScale = 0.9854
odomStrafeScale  = 0.9375
```

Yaw control constants:

```text
YAW_CONTROL_DEADBAND_DEG = 2.0
YAW_SETTLE_RATE_DPS      = 3.0
YAW_NEAR_TARGET_DEG      = 15.0
YAW_NEAR_TARGET_MAX_RPM  = 10.0
YAW_DAMP_RPM_PER_DPS     = 0.20
```

## ESP32 Position Move Behavior

The manager sends position moves as:

```text
angle_deg
distance_m
heading_deg
timeout_s
```

The ESP32 converts `angleDeg` and `distanceCm` into body-frame targets:

```text
targetForwardCm = cos(angleDeg) * distanceCm
targetStrafeCm  = -sin(angleDeg) * distanceCm
targetYawDeg    = wrapAngleDeg(headingDeg)
```

Heading and yaw use the Audix navigation contract:

```text
0 deg    = forward / +x
90 deg   = left / +y
-90 deg  = right / -y
180 deg  = backward / -x
left/CCW rotation  = positive yaw
right/CW rotation  = negative yaw
```

The ESP firmware may use calibration signs at the motor/sensor boundary, but
ROS-facing commands and telemetry must follow this contract.

The ESP32 uses cascaded control:

```text
outer body-position PI -> velocity/RPM commands -> per-wheel velocity PI
```

Position is considered done when forward, strafe, yaw error, and yaw rate are inside tolerance for the required settle ticks.

## Map Model Used By Mission Logic

The manager has a simple map:

```text
Map size:             250 cm x 200 cm
Spawn/home:           x=15,  y=165
Top travel corridor:  y=165
Audit row:            y=80
Lane 1 center:        x=50
Lane 2 center:        x=200
```

Shelf side names:

```text
Side 1: left shelf side
Side 2: right shelf side
```

Shelf-facing headings:

```text
Side 1 faces shelf at absolute heading 90 deg
Side 2 faces shelf at absolute heading -90 deg
```

Return-to-forward heading:

```text
Return to absolute heading 0 deg before the next lane
```

## Audit Mission Flow

The audit mission accepts:

```text
shelves: int32[]   # selected sides, valid values 1 and 2
level_1: bool
level_2: bool
```

Mission sequence:

1. Validate that at least one side and one level are selected.
2. Set mode to `mission`.
3. Reset internal map pose to spawn.
4. For each selected side:
   - Move to top travel corridor.
   - Move to the selected lane center.
   - Move to audit row.
   - Rotate to face the shelf.
   - Scan selected levels.
5. For non-final selected sides:
   - Rotate back forward before traveling to the next side.
6. For the final selected side:
   - Do not rotate back forward after the final scan.
   - Rotate to absolute heading `180`.
   - Start home routine.
7. After homing:
   - Rotate to absolute heading `0`.
   - Reset map pose to spawn.
   - Reset world odometry.
   - Set mode back to `manual`.

## Level Scan Logic

Selected levels are handled as follows:

```text
If level 1 selected:
  scan level 1

If level 2 selected:
  lift up by audit_lift_steps
  scan level 2
  lower by audit_lift_steps
```

Defaults:

```text
audit_lift_steps       = 500
lift_speed_sps         = 500
vision_scan_timeout_s  = 25.0
vision_scan_settle_s   = 0.5
```

Shelf IDs:

```text
side 1 level 1 -> beans_can
side 1 level 2 -> indomie
side 2 level 1 -> indomie
side 2 level 2 -> fruit_rings_cereal
```

## Backward Move Logic

Mission backward movement has special behavior.

Old idea:

```text
rotate relative 180, then move forward
```

Current behavior:

```text
face absolute heading 180, then move forward
```

This means:

- If currently facing `90`, it turns to absolute `180`.
- If currently facing `-90`, it turns to absolute `180`.
- It does not blindly rotate another relative `180`.

## Home Logic

Home uses world odometry from telemetry.

Default home parameters:

```text
home_tolerance_cm = 2.0
home_max_passes   = 8
home_settle_s     = 0.25
```

Home routine:

1. Wait for telemetry.
2. Read world forward and strafe error.
3. If forward error is outside tolerance:
   - Move world `B` if forward error is positive.
   - Move world `F` if forward error is negative.
4. Else if strafe error is outside tolerance:
   - Move world `L` if strafe error is positive.
   - Move world `R` if strafe error is negative.
5. Repeat until both errors are within tolerance or max passes are reached.
6. If still outside tolerance, raise an error.
7. Rotate to absolute heading `0`.
8. Reset map pose and world odometry.

Important mission-specific behavior:

```text
After final scan -> face absolute heading 180 -> home
```

## IR Sensors

IR sensor order:

```text
front_left
front
front_right
right
back
left
```

Front watch sensors:

```text
front
front_left
front_right
```

Side watch sensors:

```text
left
right
```

All watch sensors:

```text
front
front_left
front_right
left
right
```

## Avoidance Parameters

Defaults:

```text
front_dynamic_hold_s          = 3.0
front_strafe_search_distance  = 1.20 m
front_strafe_search_timeout   = 8.0 s
front_advance_distance        = 0.20 m
front_advance_timeout         = 4.0 s
side_follow_search_distance   = 3.00 m
side_follow_watch_front       = False
rejoin_tolerance              = 0.02 m
max_avoidance_actions         = 24
```

## Avoidance Flow

When movement is interrupted by IR:

1. Build a state dictionary from active IR sensors.
2. If exactly one front corner sensor is active and front is not active:
   - Run front-corner avoidance.
3. If any front watch sensor is active:
   - Wait `front_dynamic_hold_s`.
   - Turn buzzer on during the wait.
   - If front clears:
     - If side sensors are active, run side-path escape.
     - Else return `front_dynamic_clear`.
   - If front is still blocked:
     - Run front avoidance.
4. If only side sensors are active:
   - Run side-path escape.
5. Otherwise return ignored IR.

## Front Avoidance

Front avoidance:

1. Reset lateral reference.
2. Choose a global/map lateral direction.
3. Convert that global direction to body direction if heading is reversed.
4. Strafe in body direction.
5. Watch the matching physical body sensor.
6. Continue until the relevant diagonal front sensor falls.
7. Move forward until the relevant side sensor falls.
8. Move forward buffer.
9. Recenter using odometry.

Physical sensor pairing is preserved:

```text
body strafe right watches front_left
body strafe left  watches front_right
```

But the desired avoidance direction is map/global-aware:

```text
heading normal:
  global right -> body right
  global left  -> body left

heading reversed:
  global right -> body left
  global left  -> body right
```

## Side-Path Escape

If side sensors are active:

1. Choose lateral escape direction from active physical side sensor.
2. Convert physical/body side direction to map direction if heading is reversed.
3. Move forward buffer.
4. Strafe until back sensor falls.
5. Move backward return-to-line buffer.
6. If another IR stop happens, recurse through IR handling.

## Recenter Logic

Mission memory tracks lateral offset.

Recenter:

1. If lateral offset is within `rejoin_tolerance`, snap to center.
2. Otherwise choose map direction:
   - offset positive -> move left
   - offset negative -> move right
3. Convert map lateral direction to body lateral direction if heading is reversed.
4. Execute strafe correction.
5. Repeat up to `max_recenter_attempts`.

## What A Simulation Should Implement First

Start with a logic-accurate simulation before physics-perfect Gazebo tuning.

Create a package such as:

```text
audix_sim/
  launch/
    audix_sim.launch.py
  worlds/
    audix_store.world
  urdf/
    audix.urdf.xacro
  audix_sim/
    sim_base_node.py
    sim_ir_node.py
    sim_lift_node.py
    sim_scan_node.py
```

Minimum useful simulation nodes:

### `sim_base_node.py`

Implements:

- `/move`
- `/twist`
- `/esp/stop`
- `/esp/reset_odom`
- `/esp/init_imu`

Publishes:

- `/esp/telemetry`
- `/odom`
- TF, ideally `map -> odom -> base_link`

Behavior:

- Accept `angle_deg`, `distance_m`, `heading_deg`, `timeout_s`.
- Move an internal pose or Gazebo robot.
- Return the same response fields as the real `/move` service.
- Respect the same direction conventions and tolerances.

### `sim_ir_node.py`

Publishes:

- `/ir/state`

Behavior:

- Use simple geometric checks first.
- Later replace with Gazebo ray sensors if desired.
- Match real sensor names exactly.

### `sim_lift_node.py`

Implements:

- `/lift/move_steps`

Behavior:

- Track lift level/state.
- Optionally animate in Gazebo later.

### `sim_scan_node.py`

Implements:

- `/scan_shelf`

Behavior:

- Return expected shelf IDs and product counts.
- Later connect to camera/vision simulation.

## Suggested Development Order

1. Make a pure ROS logic simulation without Gazebo physics.
2. Run the real `robot_manager_node.py` against simulated `/move`, `/ir/state`, `/scan_shelf`, and `/lift/move_steps`.
3. Validate:
   - lane 1 only, level 1
   - lane 1 only, level 2
   - lane 1 both levels
   - lane 2 only
   - both lanes
   - final scan -> heading 180 -> home
   - obstacle at heading 0
   - obstacle at heading 180
4. Once mission logic matches reality, connect the same interface to Gazebo.
5. Add TF and RViz visualization.
6. Tune Gazebo friction/slip/delays to match real movement.

## Real Data Still Worth Measuring

To make the simulation realistic, measure these from the real robot:

- Linear speed at `40 RPM` forward.
- Linear speed at `40 RPM` strafe.
- Time to rotate `90 deg`.
- Time to rotate `180 deg`.
- Overshoot after stopping from a typical mission move.
- Strafe slip ratio on your floor.
- IR sensor range.
- IR sensor cone/field angle.
- Camera position relative to `base_link`.
- Camera field of view.
- Shelf and obstacle dimensions.
- Lift height per `500` steps.

## Key Recommendation

Do not rewrite the mission logic for simulation.

Instead, make simulation nodes mimic the real robot services and topics. Then the same `robot_manager_node.py` can run both:

```text
Real mode:
  robot_manager_node + micro_ros_base_node + gpio_hardware_node

Simulation mode:
  robot_manager_node + sim_base_node + sim_ir_node + sim_lift_node + sim_scan_node
```

This gives you a clean path toward:

- Gazebo/RViz validation.
- Real TF on the Pi.
- Better map awareness.
- Eventually replacing hand-coded mission moves with Nav2 or another planner.
