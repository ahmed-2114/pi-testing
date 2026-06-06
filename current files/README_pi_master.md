# Pi master and ESP PID workflow

This project keeps the ESP32 as the protected motion controller. The Raspberry Pi sends high-level position commands only, then waits for acknowledgements, `done` events, fresh telemetry, or IR edge events before sending the next command.

## Files

- `esp_pid.ino`: legacy/ignored for the current workflow.
- `esp_correct_pid_pi.ino`: new ESP32 sketch copied from `correct_pid.ino`, keeping the same GUI controller/PID values while adding the Pi UART JSON protocol.
- `pi_master.py`: Raspberry Pi mission, training, path, and IR debug brain.
- `pi_mecanum_path_builder.py`: Smaller interactive position/path tester.
- `pi_ps_master_teleop.py`: PS-controller manual teleop for the new ESP/Pi protocol.

## ESP command contract

The Pi master uses:

- `PING`
- `INIT_IMU`
- `RESET_ODOM`
- `RESET_ENC`
- `MOVE angle=<deg> dist=<cm> heading=<deg> timeout=<ms>`
- `TWIST forward=<rpm> strafe=<rpm> turn=<rpm> timeout=<ms>` for manual PS teleop
- `STOP`

Mission/path/training modes do not send wheel PWM, micro-motion signals, or `TWIST`; they stay on position `MOVE` commands. `TWIST` is only for the manual PS controller teleop.

For the current robot, flash `esp_correct_pid_pi.ino` when you want the proven `correct_pid.ino` controller to work with the Pi. The web GUI still exists, and the Pi talks over USB/UART JSON lines at 115200 baud.

Strafe calibration note: in `esp_correct_pid_pi.ino`, `strafeSign` stays `-1` from the working controller. The Pi-compatible sketch sets `odomStrafeSign` to `-1` so right/left position moves close their lateral odometry error and stop. If a future mechanical change makes strafes stop in the wrong direction, change only `odomStrafeSign` first.

## Pi master modes

Run help:

```bash
python3 pi_master.py --help
```

Mission mode:

```bash
python3 pi_master.py --mode mission
```

The script prompts for the goal distance in cm unless `--auto-start` or `--no-prompt-goal` is used.

Move mode:

```bash
python3 pi_master.py --mode move
```

Interactive commands:

```text
F 30
R 20
L 20
B 10
FR 25
FL 25
BR 25
BL 25
status
stop
q
```

Path mode:

```bash
python3 pi_master.py --mode path
```

Enter one waypoint per line using the same direction syntax. Submit a blank line to run the whole path sequentially.

IR monitor:

```bash
python3 pi_master.py --mode ir-monitor
```

This prints active IR states plus rising and falling edges. It does not need the ESP link.

IR training:

```bash
python3 pi_master.py --mode ir-train
```

The robot stays still until an IR rising edge is detected. Front/front-left/front-right triggers run the front obstacle sequence. Left/right triggers run the side falling-edge sequence.

## Obstacle sequence

Default mission behavior:

1. Send one forward `MOVE` toward the goal.
2. If a front IR triggers, send `STOP` and wait for the ESP stopped `done` event or fresh telemetry.
3. Hold still for `--front-dynamic-hold` seconds, with the buzzer on.
4. If the front is clear after the hold, continue the mission forward.
5. If the front is still blocked, treat it as a static obstacle.
6. Bias right first unless the right side is blocked, then switch left.
7. Strafe until the matching front diagonal sensor detects:
   - Strafe right watches `front_left`.
   - Strafe left watches `front_right`.
8. Continue the strafe until the diagonal sensor falls.
9. Move forward until the side sensor sees the obstacle and then falls.
10. Move forward 30 cm.
11. Recenter using odometry first, compensating accumulated lateral movement.
12. Continue toward the original goal distance.
13. If forward progress overshoots the goal, move backward to correct.

`front_right` is assigned to GPIO25 / physical pin 22.

## Editable parameters

Common tuning flags:

```bash
--goal-distance 1.20
--goal-distance-cm 120
--front-advance-distance 0.30
--front-dynamic-hold 3.0
--front-strafe-search-distance 1.20
--side-follow-search-distance 3.00
--preferred-first-direction right
--buzzer-pin 19
--no-buzzer
--goal-stepper-action up
--lift-steps 200
--lift-direction 1
--stepper-speed-sps 275
--goal-tolerance 0.05
--rejoin-tolerance 0.02
```

`--goal-distance` is meters. `--goal-distance-cm` is the safer option when you want to type field distances like `120`.

Raspberry Pi BCM/physical pin map:

```text
IR JST A: front-left GPIO23 pin16, front GPIO24 pin18, front-right GPIO25 pin22, GND pin20
IR JST B: right GPIO17 pin11, back GPIO27 pin13, left GPIO22 pin15
Stepper: EN GPIO5 pin29, STEP GPIO6 pin31, DIR GPIO13 pin33
Buzzer: GPIO19 pin35
Traffic light: green GPIO16 pin36, yellow GPIO20 pin38, red GPIO21 pin40
```

Mission mode uses the stepper only after the robot reaches the goal and stops. The default end action is `up`, 200 steps, direction `1`, at 275 steps/s. On this lift, `1` is up and `-1` is down. Use `--goal-stepper-action none` or `--no-lift-on-goal` to disable it.

PS controller teleop for the current master/new ESP setup:

```bash
python3 pi_ps_master_teleop.py
```

The left stick drives all 8 translation directions, the D-pad drives front/back/right/left, and the right stick controls heading/rotation. Motion is fixed at 40 RPM by default. R1 holds the lift up, L1 holds the lift down, triangle reinitializes the IMU, square runs the old stepper-home request, circle drives the buzzer, and cross runs the traffic-light dance. This teleop file uses the `TWIST` command supported by `esp_correct_pid_pi.ino`; mission mode still uses position `MOVE` commands. The ESP sketch also answers `LIMIT_STATUS pin=23` for the square/home command.

IR interpretation:

```bash
--ir-logic baseline
--ir-logic active-low
--ir-logic active-high
```

`baseline` is the default. It records the startup GPIO state and treats a changed state as detection. If that behaves badly on the real sensors, test `active-low` because the older IR test reported idle as `1` and detection as `0`.

## Logs

Each run creates:

```text
run_logs/<timestamp>_<mode>/events.jsonl
run_logs/<timestamp>_<mode>/moves.csv
```

Use `--no-log` to disable logging or `--log-dir <path>` to change the folder.

## Safe test order

1. `python3 pi_master.py --mode ir-monitor`
2. `python3 pi_master.py --mode move`
3. Test `F/R/L/B 20` to confirm mecanum direction signs.
4. `python3 pi_master.py --mode ir-train`
5. `python3 pi_master.py --mode mission`
