#!/usr/bin/env python3
"""
Single-file Raspberry Pi master for the real Audix robot.

This is the real-life version of the simple-cardinal brain. It does not need
ROS running on the Pi:

- reads the 6 IR sensors directly from Raspberry Pi GPIO
- receives odometry, IMU, encoder counts, wheel RPM, and PID telemetry from ESP32
- runs the simple-cardinal obstacle decision logic locally
- sends body velocity RPM commands to the ESP32 wheel PID controller

The matching ESP sketch is esp_pid.ino.
"""

import argparse
import importlib.util
import json
import math
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass


SYSTEM_PYTHON = "/usr/bin/python3"
REEXEC_GUARD = "PI_MASTER_SYSTEM_PYTHON"

READ_TIMEOUT_S = 0.05
PING_TIMEOUT_S = 2.5
ACK_TIMEOUT_S = 3.0
INIT_TIMEOUT_S = 8.0

IR_PINS = {
    "front_left": 23,
    "front": 24,
    "front_right": 25,
    "right": 17,
    "back": 27,
    "left": 22,
}
IR_LABELS = {
    "front_left": "FL",
    "front": "Front",
    "front_right": "FR",
    "right": "Right",
    "back": "Back",
    "left": "Left",
}

LEFT = 1
RIGHT = -1


@dataclass
class Event:
    raw: str
    data: dict | None


@dataclass
class MotionCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


def clamp(value, low, high):
    return max(low, min(high, value))


def opposite_direction(direction):
    return LEFT if direction == RIGHT else RIGHT


def module_exists(name: str) -> bool:
    try:
        return importlib.util.find_spec(name) is not None
    except ModuleNotFoundError:
        return False


def reexec_target() -> str:
    argv0 = sys.argv[0] if sys.argv else ""
    if argv0 and argv0 != "-" and os.path.exists(argv0):
        return os.path.abspath(argv0)
    return os.path.abspath(__file__)


def maybe_reexec_with_system_python() -> None:
    if (
        os.path.exists(SYSTEM_PYTHON)
        and os.environ.get(REEXEC_GUARD) != "1"
        and os.path.abspath(sys.executable) != SYSTEM_PYTHON
    ):
        os.environ[REEXEC_GUARD] = "1"
        os.execv(SYSTEM_PYTHON, [SYSTEM_PYTHON, reexec_target(), *sys.argv[1:]])


def ensure_gpiozero_runtime() -> None:
    has_gpiozero = module_exists("gpiozero")
    has_backend = any(module_exists(name) for name in ("lgpio", "RPi.GPIO", "pigpio"))
    if has_gpiozero and has_backend:
        return

    maybe_reexec_with_system_python()

    if not has_gpiozero:
        raise SystemExit(
            "gpiozero is not installed for this Python interpreter.\n"
            f"Run this script with {SYSTEM_PYTHON} or install python3-gpiozero."
        )

    raise SystemExit(
        "gpiozero is installed, but no supported GPIO backend is available.\n"
        f"Run this script with {SYSTEM_PYTHON} or install lgpio/pigpio/RPi.GPIO."
    )


def ensure_serial_runtime() -> None:
    if module_exists("serial"):
        return
    maybe_reexec_with_system_python()
    raise SystemExit("pyserial is not installed. Install python3-serial or pyserial.")


class GpioIrBank:
    def __init__(self, *, active_low: bool, pull_up: bool, mock: bool) -> None:
        self.active_low = active_low
        self.mock = mock
        self.devices = {}
        if mock:
            return

        ensure_gpiozero_runtime()
        from gpiozero import DigitalInputDevice

        self.devices = {
            name: DigitalInputDevice(pin, pull_up=pull_up, active_state=True)
            for name, pin in IR_PINS.items()
        }

    def read(self) -> dict[str, bool]:
        if self.mock:
            return {name: False for name in IR_PINS}

        state = {}
        for name, device in self.devices.items():
            raw_high = bool(device.value)
            state[name] = (not raw_high) if self.active_low else raw_high
        return state

    def close(self) -> None:
        for device in self.devices.values():
            device.close()


class EspPiControlLink:
    def __init__(self, port: str, baud: int) -> None:
        ensure_serial_runtime()
        import serial

        self.port = port
        self.baud = baud
        self.seq = 1
        self.events: queue.Queue[Event] = queue.Queue()
        self.stop_event = threading.Event()
        self.send_lock = threading.Lock()
        self.telemetry_lock = threading.Lock()
        self.last_telemetry: dict | None = None
        self.last_telemetry_time = 0.0
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=READ_TIMEOUT_S)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()

    def close(self) -> None:
        self.stop_event.set()
        self.reader.join(timeout=1.0)
        self.ser.close()

    def _reader_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                line = self.ser.readline()
            except Exception:
                return
            if not line:
                continue

            raw = line.decode("utf-8", errors="replace").strip()
            if not raw:
                continue

            data = self._parse_json_line(raw)
            if data is not None and data.get("type") == "telemetry":
                with self.telemetry_lock:
                    self.last_telemetry = data
                    self.last_telemetry_time = time.monotonic()
                continue

            self.events.put(Event(raw=raw, data=data))

    def _parse_json_line(self, raw: str) -> dict | None:
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            json_start = raw.find("{")
            if json_start < 0:
                return None
            try:
                return json.loads(raw[json_start:])
            except json.JSONDecodeError:
                return None

    def next_seq(self) -> int:
        value = self.seq
        self.seq += 1
        return value

    def send(self, line: str) -> None:
        payload = (line.strip() + "\n").encode("utf-8")
        with self.send_lock:
            self.ser.write(payload)
            self.ser.flush()

    def send_command(self, line: str) -> int:
        seq = self.next_seq()
        if " seq=" not in line:
            line = f"{line} seq={seq}"
        self.send(line)
        return seq

    def send_velocity(self, forward_rpm: float, strafe_rpm: float, rotate_rpm: float, args) -> None:
        self.send(
            "TWIST "
            f"forward={forward_rpm:.3f} "
            f"strafe={strafe_rpm:.3f} "
            f"turn={rotate_rpm:.3f} "
            f"timeout={max(100, int(args.telemetry_timeout * 1000))} "
            f"strafeSign={args.esp_strafe_sign} "
            f"rotateSign={args.esp_rotate_sign} "
            f"odomForwardSign={args.esp_odom_forward_sign} "
            f"odomStrafeSign={args.esp_odom_strafe_sign} "
            f"odomYawSign={args.esp_odom_yaw_sign} "
            f"odomForwardScale={args.esp_odom_forward_scale:.5f} "
            f"odomStrafeScale={args.esp_odom_strafe_scale:.5f}"
        )

    def wait_for(self, seq: int | None, wanted_types: set[str], timeout: float) -> dict:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                event = self.events.get(timeout=max(0.02, remaining))
            except queue.Empty:
                continue

            if event.data is None:
                continue

            data = event.data
            seq_matches = seq is None or data.get("seq") == seq
            if data.get("type") in wanted_types and seq_matches:
                return data

        raise TimeoutError(f"Timed out waiting for {wanted_types} for seq {seq}")

    def command_ack(self, line: str, timeout: float = ACK_TIMEOUT_S) -> dict:
        seq = self.send_command(line)
        ack = self.wait_for(seq, {"ack", "pong"}, timeout)
        if ack.get("type") == "ack" and not ack.get("ok", False):
            raise RuntimeError(f"{ack.get('cmd', line)} rejected: {ack.get('message', 'no message')}")
        return ack

    def latest_telemetry(self) -> tuple[dict | None, float]:
        with self.telemetry_lock:
            return self.last_telemetry, self.last_telemetry_time

    def wait_for_telemetry(self, timeout: float) -> dict:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            telemetry, stamp = self.latest_telemetry()
            if telemetry is not None and stamp > 0.0:
                return telemetry
            time.sleep(0.02)
        raise TimeoutError("Timed out waiting for ESP telemetry")


class EspPoseTracker:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_forward_m = None
        self.last_strafe_m = None

    def update(self, telemetry: dict) -> tuple[float, float, float]:
        pose = telemetry.get("pose") or telemetry.get("odometry") or {}
        imu = telemetry.get("imu") or {}
        forward_m = 0.01 * float(pose.get("forwardCm", 0.0))
        strafe_m = 0.01 * float(pose.get("strafeCm", 0.0))
        yaw_deg = pose.get("yawDeg", imu.get("yawDeg", 0.0))
        yaw = math.radians(float(yaw_deg))

        if self.last_forward_m is None:
            self.last_forward_m = forward_m
            self.last_strafe_m = strafe_m
            self.yaw = yaw
            return self.x, self.y, self.yaw

        delta_forward = forward_m - self.last_forward_m
        delta_strafe = strafe_m - self.last_strafe_m
        self.last_forward_m = forward_m
        self.last_strafe_m = strafe_m

        # Match the sim brain convention: physical forward is negative local X.
        dx_body = -delta_forward
        dy_body = -delta_strafe
        self.x += dx_body * math.cos(yaw) - dy_body * math.sin(yaw)
        self.y += dx_body * math.sin(yaw) + dy_body * math.cos(yaw)
        self.yaw = yaw
        return self.x, self.y, self.yaw


class DryRunPoseTracker(EspPoseTracker):
    def __init__(self) -> None:
        super().__init__()
        self.last_time = None

    def integrate(self, command: MotionCommand, now: float) -> tuple[float, float, float]:
        if self.last_time is None:
            self.last_time = now
            return self.x, self.y, self.yaw
        dt = max(0.0, min(0.2, now - self.last_time))
        self.last_time = now
        self.x += command.vx * dt
        self.y += command.vy * dt
        self.yaw += command.wz * dt
        return self.x, self.y, self.yaw


class SimpleCardinalRealBrain:
    def __init__(self, args) -> None:
        self.goal_distance = args.goal_distance
        self.goal_tolerance = args.goal_tolerance
        self.forward_speed = args.forward_speed
        self.lateral_speed = args.lateral_speed
        self.backoff_speed = args.backoff_speed
        self.line_kp = args.line_kp
        self.max_line_correction_speed = args.max_line_correction_speed
        self.backoff_distance = args.backoff_distance
        self.post_front_clear_lateral_distance = args.post_front_clear_lateral_distance
        self.lateral_recovery_step = args.lateral_recovery_step
        self.max_lateral_clearance_distance = args.max_lateral_clearance_distance
        self.forward_clear_distance = args.forward_clear_distance
        self.post_side_clear_forward_distance = args.post_side_clear_forward_distance
        self.shift_tolerance = args.shift_tolerance
        self.rejoin_tolerance = args.rejoin_tolerance
        self.sensor_timeout_sec = args.sensor_timeout
        self.max_shift_cycles = args.max_shift_cycles

        self.preferred_direction = RIGHT if args.preferred_first_direction != "left" else LEFT
        self.sensor_hits = {name: False for name in IR_PINS}
        self.sensor_update_sec = {name: None for name in IR_PINS}

        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.start_pose = None
        self.forward_unit = None
        self.left_unit = None
        self.enabled = False
        self.state = "WAIT_ENABLE"
        self.current_offset_target = 0.0
        self.current_shift_direction = self.preferred_direction
        self.shift_start_cross = 0.0
        self.obstacle_reference_side = opposite_direction(self.current_shift_direction)
        self.backoff_start_progress = 0.0
        self.advance_start_progress = 0.0
        self.front_clear_cross = None
        self.side_seen_during_clear = False
        self.side_clear_progress = None
        self.shift_cycle_count = 0
        self.last_trigger = "none"
        self.last_command = MotionCommand()
        self._now = time.monotonic()

    def update_pose(self, x: float, y: float, yaw: float, now: float) -> None:
        self._now = now
        self.current_x = float(x)
        self.current_y = float(y)
        self.current_yaw = float(yaw)
        self._capture_start_pose_if_ready()

    def update_ir(self, ir_state: dict[str, bool], now: float) -> None:
        self._now = now
        for name, blocked in ir_state.items():
            self.sensor_hits[name] = bool(blocked)
            self.sensor_update_sec[name] = now

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = bool(enabled)
        if not self.enabled:
            self.state = "WAIT_ENABLE"
            self.current_offset_target = self.cross_track_error() if self.start_pose is not None else 0.0
            self.front_clear_cross = None

    def _capture_start_pose_if_ready(self) -> None:
        if self.start_pose is not None or self.current_x is None or self.current_y is None:
            return
        heading = self.pose_yaw()
        self.start_pose = (self.current_x, self.current_y)
        self.forward_unit = (-math.cos(heading), -math.sin(heading))
        self.left_unit = (math.sin(heading), -math.cos(heading))

    def pose_yaw(self) -> float:
        return self.current_yaw if self.current_yaw is not None else 0.0

    def sensor_active(self, sensor_name: str) -> bool:
        stamp = self.sensor_update_sec[sensor_name]
        if stamp is None:
            return False
        if self._now - stamp > self.sensor_timeout_sec:
            return False
        return self.sensor_hits[sensor_name]

    def forward_blocked(self) -> bool:
        return self.sensor_active("front")

    def front_left_blocked(self) -> bool:
        return self.sensor_active("front_left")

    def front_right_blocked(self) -> bool:
        return self.sensor_active("front_right")

    def front_blocked_any(self) -> bool:
        return self.forward_blocked() or self.front_left_blocked() or self.front_right_blocked()

    def side_blocked(self, direction: int) -> bool:
        if direction == LEFT:
            return self.sensor_active("left") or self.sensor_active("front_left")
        return self.sensor_active("right") or self.sensor_active("front_right")

    def side_sensor_name(self, direction: int) -> str:
        return "left" if direction == LEFT else "right"

    def obstacle_side_active(self) -> bool:
        return self.sensor_active(self.side_sensor_name(self.obstacle_reference_side))

    def along_track_progress(self) -> float:
        if self.start_pose is None or self.current_x is None or self.current_y is None:
            return 0.0
        dx = self.current_x - self.start_pose[0]
        dy = self.current_y - self.start_pose[1]
        return dx * self.forward_unit[0] + dy * self.forward_unit[1]

    def cross_track_error(self) -> float:
        if self.start_pose is None or self.current_x is None or self.current_y is None:
            return 0.0
        dx = self.current_x - self.start_pose[0]
        dy = self.current_y - self.start_pose[1]
        return dx * self.left_unit[0] + dy * self.left_unit[1]

    def set_command(self, vx=0.0, vy=0.0, wz=0.0) -> MotionCommand:
        self.last_command = MotionCommand(float(vx), float(vy), float(wz))
        return self.last_command

    def choose_trigger(self) -> str | None:
        if self.forward_blocked():
            return "front"
        if self.front_left_blocked():
            return "front_left"
        if self.front_right_blocked():
            return "front_right"
        return None

    def choose_direction_for_trigger(self, trigger: str) -> int:
        if trigger == "front_left":
            return RIGHT
        if trigger == "front_right":
            return LEFT
        if self.front_left_blocked() and not self.front_right_blocked():
            return RIGHT
        if self.front_right_blocked() and not self.front_left_blocked():
            return LEFT
        return self.preferred_direction

    def reset_clearance_tracking(self) -> None:
        self.side_seen_during_clear = False
        self.side_clear_progress = None

    def reset_shift_tracking(self) -> None:
        self.front_clear_cross = None

    def begin_shift(self, trigger: str) -> None:
        direction = self.choose_direction_for_trigger(trigger)
        if self.side_blocked(direction) and not self.side_blocked(opposite_direction(direction)):
            direction = opposite_direction(direction)

        self.shift_start_cross = self.cross_track_error()
        self.current_offset_target = self.shift_start_cross
        self.current_shift_direction = direction
        self.obstacle_reference_side = opposite_direction(direction)
        self.reset_shift_tracking()
        self.reset_clearance_tracking()
        self.shift_cycle_count += 1
        self.last_trigger = trigger

        if self.shift_cycle_count > self.max_shift_cycles:
            print("warn | max shift cycles reached; stopping to avoid oscillation", flush=True)
            self.state = "DONE"
            return

        self.state = "SHIFT_OUT"

    def switch_direction(self) -> None:
        self.current_shift_direction *= -1
        self.preferred_direction = self.current_shift_direction
        self.obstacle_reference_side = opposite_direction(self.current_shift_direction)
        self.current_offset_target = self.cross_track_error()
        self.shift_start_cross = self.current_offset_target
        self.reset_shift_tracking()
        self.reset_clearance_tracking()
        self.state = "SHIFT_OUT"

    def run_move_to_goal(self) -> MotionCommand:
        remaining = self.goal_distance - self.along_track_progress()
        if remaining <= self.goal_tolerance:
            self.state = "DONE"
            return self.set_command(0.0, 0.0, 0.0)

        trigger = self.choose_trigger()
        if trigger is not None:
            self.begin_shift(trigger)
            return self.set_command(0.0, 0.0, 0.0)

        line_error = self.cross_track_error()
        vy = clamp(-self.line_kp * line_error, -self.max_line_correction_speed, self.max_line_correction_speed)
        return self.set_command(-self.forward_speed, vy, 0.0)

    def run_backoff(self) -> MotionCommand:
        backed_off = self.backoff_start_progress - self.along_track_progress()
        if backed_off >= self.backoff_distance:
            self.state = "SHIFT_OUT"
            return self.set_command(0.0, 0.0, 0.0)
        return self.set_command(self.backoff_speed, 0.0, 0.0)

    def run_shift_out(self) -> MotionCommand:
        line_error = self.cross_track_error()
        if self.side_blocked(self.current_shift_direction) and self.front_blocked_any():
            self.switch_direction()
            return self.set_command(0.0, 0.0, 0.0)

        shifted_distance = abs(line_error - self.shift_start_cross)
        if shifted_distance >= self.max_lateral_clearance_distance:
            self.switch_direction()
            return self.set_command(0.0, 0.0, 0.0)

        if self.front_blocked_any():
            self.front_clear_cross = None
            self.current_offset_target = line_error
            vy = float(self.current_shift_direction) * self.lateral_speed
            return self.set_command(0.0, vy, 0.0)

        if self.front_clear_cross is None:
            self.front_clear_cross = line_error
            self.current_offset_target = (
                self.front_clear_cross
                + self.current_shift_direction * self.post_front_clear_lateral_distance
            )

        offset_error = self.current_offset_target - line_error
        vy = clamp(offset_error * 1.8, -self.lateral_speed, self.lateral_speed)
        cmd = self.set_command(0.0, vy, 0.0)

        if abs(offset_error) <= self.shift_tolerance:
            self.advance_start_progress = self.along_track_progress()
            self.reset_clearance_tracking()
            self.state = "ADVANCE_CLEAR"
        return cmd

    def run_advance_clear(self) -> MotionCommand:
        if self.forward_blocked():
            self.shift_start_cross = self.cross_track_error()
            self.current_offset_target = self.shift_start_cross
            self.reset_shift_tracking()
            self.reset_clearance_tracking()
            self.state = "SHIFT_OUT"
            return self.set_command(0.0, 0.0, 0.0)

        progress = self.along_track_progress()
        if self.obstacle_side_active():
            self.side_seen_during_clear = True
            self.side_clear_progress = None
        elif self.side_seen_during_clear and self.side_clear_progress is None:
            self.side_clear_progress = progress

        cmd = self.set_command(-self.forward_speed, 0.0, 0.0)
        if self.side_clear_progress is not None:
            if progress - self.side_clear_progress >= self.forward_clear_distance:
                self.state = "RETURN_TO_PATH"
            return cmd

        if (not self.side_seen_during_clear) and (
            progress - self.advance_start_progress >= self.post_side_clear_forward_distance
        ):
            self.side_clear_progress = progress
        return cmd

    def run_return_to_path(self) -> MotionCommand:
        line_error = self.cross_track_error()
        if abs(line_error) <= self.rejoin_tolerance:
            self.current_offset_target = 0.0
            self.state = "MOVE_TO_GOAL"
            return self.set_command(0.0, 0.0, 0.0)

        return_direction = RIGHT if line_error > 0.0 else LEFT
        if self.side_blocked(return_direction):
            self.advance_start_progress = self.along_track_progress()
            self.side_clear_progress = self.along_track_progress()
            self.state = "ADVANCE_CLEAR"
            return self.set_command(0.0, 0.0, 0.0)

        vy = clamp(-self.line_kp * line_error, -self.lateral_speed, self.lateral_speed)
        return self.set_command(0.0, vy, 0.0)

    def step(self, now: float) -> MotionCommand:
        self._now = now

        if self.start_pose is None:
            return self.set_command(0.0, 0.0, 0.0)

        if not self.enabled:
            self.state = "WAIT_ENABLE"
            return self.set_command(0.0, 0.0, 0.0)

        if self.state == "WAIT_ENABLE":
            self.state = "MOVE_TO_GOAL"

        if self.state == "MOVE_TO_GOAL":
            return self.run_move_to_goal()
        if self.state == "BACKOFF":
            return self.run_backoff()
        if self.state == "SHIFT_OUT":
            return self.run_shift_out()
        if self.state == "ADVANCE_CLEAR":
            return self.run_advance_clear()
        if self.state == "RETURN_TO_PATH":
            return self.run_return_to_path()

        return self.set_command(0.0, 0.0, 0.0)


def command_to_esp_rpm(command: MotionCommand, args) -> tuple[float, float, float]:
    wheel_circumference_m = math.pi * args.wheel_diameter_m
    linear_to_rpm = 60.0 / wheel_circumference_m
    angular_to_rpm = ((args.wheel_base_half_m + args.track_width_half_m) / args.wheel_radius_m) * 60.0 / (2.0 * math.pi)

    forward_rpm = args.forward_command_sign * (-command.vx * linear_to_rpm)
    strafe_rpm = args.strafe_command_sign * (command.vy * linear_to_rpm)
    rotate_rpm = args.rotate_command_sign * (command.wz * angular_to_rpm)

    if args.normalize_to_rpm_limit:
        max_abs = max(abs(forward_rpm), abs(strafe_rpm), abs(rotate_rpm))
        if max_abs > 1e-6:
            scale = args.rpm_limit / max_abs
            forward_rpm *= scale
            strafe_rpm *= scale
            rotate_rpm *= scale

    return (
        clamp(forward_rpm, -args.rpm_limit, args.rpm_limit),
        clamp(strafe_rpm, -args.rpm_limit, args.rpm_limit),
        clamp(rotate_rpm, -args.rpm_limit, args.rpm_limit),
    )


def ir_summary(ir_state: dict[str, bool]) -> str:
    return " ".join(f"{IR_LABELS[name]}={1 if ir_state.get(name, False) else 0}" for name in IR_PINS)


def short_list(values, digits=1) -> str:
    if not values:
        return "[]"
    return "[" + ",".join(f"{float(value):.{digits}f}" for value in values[:4]) + "]"


def parse_args():
    parser = argparse.ArgumentParser(description="Pi master: simple-cardinal brain + GPIO IR + ESP UART PID link.")

    parser.add_argument("--port", default=os.environ.get("PI_UART_PORT", "/dev/ttyAMA0"))
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--status-period", type=float, default=0.5)
    parser.add_argument("--telemetry-timeout", type=float, default=0.5)
    parser.add_argument("--auto-start", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--mock-ir", action="store_true")
    parser.add_argument("--active-low", action="store_true")
    parser.add_argument("--pull-up", action="store_true")
    parser.add_argument("--reset-odom-on-start", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--reset-encoders-on-start", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--init-imu-on-start", action=argparse.BooleanOptionalAction, default=True)

    parser.add_argument("--goal-distance", type=float, default=3.55)
    parser.add_argument("--goal-tolerance", type=float, default=0.05)
    parser.add_argument("--forward-speed", type=float, default=None)
    parser.add_argument("--lateral-speed", type=float, default=None)
    parser.add_argument("--backoff-speed", type=float, default=None)
    parser.add_argument("--line-kp", type=float, default=1.6)
    parser.add_argument("--max-line-correction-speed", type=float, default=None)
    parser.add_argument("--backoff-distance", type=float, default=0.0)
    parser.add_argument("--post-front-clear-lateral-distance", type=float, default=0.25)
    parser.add_argument("--lateral-recovery-step", type=float, default=0.05)
    parser.add_argument("--max-lateral-clearance-distance", type=float, default=0.42)
    parser.add_argument("--forward-clear-distance", type=float, default=0.252)
    parser.add_argument("--post-side-clear-forward-distance", type=float, default=0.25)
    parser.add_argument("--shift-tolerance", type=float, default=0.015)
    parser.add_argument("--rejoin-tolerance", type=float, default=0.02)
    parser.add_argument("--sensor-timeout", type=float, default=0.5)
    parser.add_argument("--preferred-first-direction", choices=("left", "right"), default="right")
    parser.add_argument("--max-shift-cycles", type=int, default=12)

    parser.add_argument("--wheel-diameter-m", type=float, default=0.097)
    parser.add_argument("--wheel-radius-m", type=float, default=0.0485)
    parser.add_argument("--wheel-base-half-m", type=float, default=0.09)
    parser.add_argument("--track-width-half-m", type=float, default=0.1574)
    parser.add_argument("--rpm-limit", type=float, default=60.0)
    parser.add_argument("--normalize-to-rpm-limit", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--forward-command-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--strafe-command-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--rotate-command-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--esp-strafe-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--esp-rotate-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--esp-odom-forward-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--esp-odom-strafe-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--esp-odom-yaw-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--esp-odom-forward-scale", type=float, default=0.9854)
    parser.add_argument("--esp-odom-strafe-scale", type=float, default=0.9375)

    args = parser.parse_args()
    max_linear_speed = math.pi * args.wheel_diameter_m * args.rpm_limit / 60.0
    if args.forward_speed is None:
        args.forward_speed = max_linear_speed
    if args.lateral_speed is None:
        args.lateral_speed = max_linear_speed
    if args.backoff_speed is None:
        args.backoff_speed = max_linear_speed
    if args.max_line_correction_speed is None:
        args.max_line_correction_speed = max_linear_speed
    if args.period <= 0.0:
        parser.error("--period must be greater than zero")
    if args.status_period <= 0.0:
        parser.error("--status-period must be greater than zero")
    return args


def initialize_link(link: EspPiControlLink, args) -> None:
    print(f"opened {args.port} @ {args.baud}", flush=True)
    try:
        pong = link.command_ack("PING", timeout=PING_TIMEOUT_S)
        print(f"link ok | pong seq={pong.get('seq')}", flush=True)
    except Exception:
        print("link wait | no PING yet, waiting for passive telemetry", flush=True)
        link.wait_for_telemetry(PING_TIMEOUT_S)

    if args.init_imu_on_start:
        print("init | IMU calibrate + zero", flush=True)
        link.command_ack("INIT_IMU", timeout=INIT_TIMEOUT_S)
    if args.reset_encoders_on_start:
        print("init | reset encoders", flush=True)
        link.command_ack("RESET_ENC")
    if args.reset_odom_on_start:
        print("init | reset odometry", flush=True)
        link.command_ack("RESET_ODOM")


def main() -> None:
    args = parse_args()
    ir_bank = GpioIrBank(active_low=args.active_low, pull_up=args.pull_up, mock=args.mock_ir or args.dry_run)
    brain = SimpleCardinalRealBrain(args)
    link = None
    pose_tracker = DryRunPoseTracker() if args.dry_run else EspPoseTracker()
    command = MotionCommand()

    try:
        if not args.dry_run:
            link = EspPiControlLink(args.port, args.baud)
            initialize_link(link, args)

        if args.auto_start:
            enabled = True
        else:
            print("Ready. Put the robot in a clear area, then press Enter to start. Ctrl-C stops.", flush=True)
            input()
            enabled = True

        print("Pi brain running. ESP is low-level PID; Pi is high-level decision maker.", flush=True)
        next_tick = time.monotonic()
        last_status = 0.0
        last_error = 0.0

        while True:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(next_tick - now)
            next_tick += args.period
            now = time.monotonic()

            try:
                if args.dry_run:
                    x, y, yaw = pose_tracker.integrate(command, now)
                    telemetry = {"pose": {"forwardCm": -x * 100.0, "strafeCm": -y * 100.0, "yawDeg": math.degrees(yaw)}}
                    telemetry_stamp = now
                else:
                    telemetry, telemetry_stamp = link.latest_telemetry()
                    if telemetry is None or now - telemetry_stamp > args.telemetry_timeout:
                        raise TimeoutError("ESP telemetry is stale")
                    x, y, yaw = pose_tracker.update(telemetry)

                ir_state = ir_bank.read()
                brain.update_pose(x, y, yaw, now)
                brain.update_ir(ir_state, now)
                brain.set_enabled(enabled)
                command = brain.step(now)
                forward_rpm, strafe_rpm, rotate_rpm = command_to_esp_rpm(command, args)

                if not args.dry_run:
                    link.send_velocity(forward_rpm, strafe_rpm, rotate_rpm, args)

                if now - last_status >= args.status_period:
                    pose = telemetry.get("pose") or telemetry.get("odometry") or {}
                    imu = telemetry.get("imu") or {}
                    print(
                        "state={state} progress={progress:.3f}/{goal:.3f} cross={cross:.3f} "
                        "cmd_rpm(f={f:.1f},s={s:.1f},r={r:.1f}) "
                        "pose_cm(f={pf:.1f},s={ps:.1f},yaw={py:.1f}) "
                        "imu_yaw={iy:.1f} rpm={rpm} enc={enc} ir[{ir}]".format(
                            state=brain.state,
                            progress=brain.along_track_progress(),
                            goal=brain.goal_distance,
                            cross=brain.cross_track_error(),
                            f=forward_rpm,
                            s=strafe_rpm,
                            r=rotate_rpm,
                            pf=float(pose.get("forwardCm", 0.0)),
                            ps=float(pose.get("strafeCm", 0.0)),
                            py=float(pose.get("yawDeg", imu.get("yawDeg", 0.0))),
                            iy=float(imu.get("yawDeg", 0.0)),
                            rpm=short_list(telemetry.get("rpm", [])),
                            enc=telemetry.get("signedEncoderCounts", telemetry.get("encoderCounts", [])),
                            ir=ir_summary(ir_state),
                        ),
                        flush=True,
                    )
                    last_status = now

                if brain.state == "DONE":
                    if link is not None:
                        link.command_ack("STOP")
                    print("Goal reached; ESP stopped.", flush=True)
                    break

            except Exception as exc:
                if now - last_error >= 1.0:
                    print(f"error | {exc}", flush=True)
                    last_error = now
                if link is not None:
                    try:
                        link.send("STOP")
                    except Exception:
                        pass
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping.", flush=True)
    finally:
        if link is not None:
            try:
                link.send("STOP")
            except Exception:
                pass
            link.close()
        ir_bank.close()


if __name__ == "__main__":
    main()
