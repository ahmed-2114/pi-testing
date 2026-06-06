#!/usr/bin/env python3
"""
Single-file Raspberry Pi master for the real Audix robot.

This is the real-life version of the simple-cardinal brain. It does not need
ROS running on the Pi:

- reads the 6 IR sensors directly from Raspberry Pi GPIO
- receives odometry, IMU, encoder counts, and PID telemetry from ESP32
- runs the simple-cardinal obstacle decision logic locally
- sends position commands to the ESP32 position PID controller

The matching ESP sketch is esp_correct_pid_pi.ino.
"""

import argparse
import csv
import datetime as dt
import importlib.util
import json
import math
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path


SYSTEM_PYTHON = "/usr/bin/python3"
REEXEC_GUARD = "PI_MASTER_SYSTEM_PYTHON"

READ_TIMEOUT_S = 0.05
PING_TIMEOUT_S = 2.5
ACK_TIMEOUT_S = 3.0
INIT_TIMEOUT_S = 8.0

# These defaults mirror esp_correct_pid_pi.ino. Keep this block aligned with the
# sketch's serial command layer and odometry configuration.
ESP_DEFAULT_BAUD = 115200
ESP_SERIAL_TELEMETRY_INTERVAL_S = 0.05
ESP_SERIAL_COMMAND_TIMEOUT_S = 0.5
ESP_POSITION_MOVE_TIMEOUT_S = 180.0

# Raspberry Pi BCM GPIO outputs.
# Physical pins:
#   IR JST A: GPIO23 pin16, GPIO24 pin18, GPIO25 pin22, GND pin20
#   IR JST B: GPIO17 pin11, GPIO27 pin13, GPIO22 pin15
#   Stepper: EN GPIO5 pin29, STEP GPIO6 pin31, DIR GPIO13 pin33
#   Buzzer: GPIO19 pin35
#   Traffic: green GPIO16 pin36, yellow GPIO20 pin38, red GPIO21 pin40
STEPPER_STEP_PIN = 6
STEPPER_DIR_PIN = 13
STEPPER_EN_PIN = 5
STEPPER_STEP_HIGH_US = 10
STEPPER_DIR_SETUP_S = 0.010
STEPPER_SPEED_SPS = 275.0
STEPPER_DEFAULT_STEPS = 200
STEPPER_UP_DIR = 1
STEPPER_DOWN_DIR = -1
DEFAULT_BUZZER_PIN = 19
TRAFFIC_YELLOW_LED_PIN = 20
TRAFFIC_GREEN_LED_PIN = 16
TRAFFIC_RED_LED_PIN = 21

HARDWARE_OUTPUT_PINS = {
    "stepper_step": STEPPER_STEP_PIN,
    "stepper_dir": STEPPER_DIR_PIN,
    "stepper_enable": STEPPER_EN_PIN,
    "buzzer": DEFAULT_BUZZER_PIN,
    "traffic_yellow": TRAFFIC_YELLOW_LED_PIN,
    "traffic_green": TRAFFIC_GREEN_LED_PIN,
    "traffic_red": TRAFFIC_RED_LED_PIN,
}

IR_SENSOR_ORDER = ("front_left", "front", "front_right", "right", "back", "left")
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

LEFT = -1
RIGHT = 1

DIRECTION_ORDER = ("F", "R", "L", "B", "FR", "FL", "BR", "BL")
DIRECTION_ANGLES_DEG = {
    "F": 0.0,
    "FR": -45.0,
    "R": -90.0,
    "BR": -135.0,
    "B": 180.0,
    "BL": 135.0,
    "L": 90.0,
    "FL": 45.0,
}


@dataclass
class Event:
    raw: str
    data: dict | None


@dataclass
class MotionCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class RunLogger:
    def __init__(self, mode: str, root: str = "run_logs") -> None:
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dir = Path(root) / f"{stamp}_{mode}"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.events_path = self.dir / "events.jsonl"
        self.moves_path = self.dir / "moves.csv"
        self._event_file = self.events_path.open("a", encoding="utf-8")
        self._move_file = self.moves_path.open("a", newline="", encoding="utf-8")
        self._move_writer = csv.DictWriter(
            self._move_file,
            fieldnames=[
                "time",
                "label",
                "angle_deg",
                "distance_m",
                "watch",
                "result",
                "forward_cm",
                "strafe_cm",
                "heading_deg",
                "ir",
            ],
        )
        self._move_writer.writeheader()
        self._move_file.flush()

    def close(self) -> None:
        self._event_file.close()
        self._move_file.close()

    def event(self, kind: str, **data) -> None:
        payload = {
            "time": dt.datetime.now().isoformat(timespec="milliseconds"),
            "kind": kind,
            **data,
        }
        self._event_file.write(json.dumps(payload, sort_keys=True) + "\n")
        self._event_file.flush()

    def move(self, label: str, angle_deg: float, distance_m: float, watch_sensors: set[str], done: dict) -> None:
        row = {
            "time": dt.datetime.now().isoformat(timespec="milliseconds"),
            "label": label,
            "angle_deg": f"{angle_deg:.3f}",
            "distance_m": f"{distance_m:.4f}",
            "watch": " ".join(sorted(watch_sensors)),
            "result": done.get("result", ""),
            "forward_cm": f"{float(done.get('forwardCm', 0.0)):.2f}",
            "strafe_cm": f"{float(done.get('strafeCm', 0.0)):.2f}",
            "heading_deg": f"{float(done.get('headingDeg', 0.0)):.2f}",
            "ir": " ".join(done.get("ir", [])),
        }
        self._move_writer.writerow(row)
        self._move_file.flush()


class NullLogger:
    dir = None

    def close(self) -> None:
        pass

    def event(self, kind: str, **data) -> None:
        pass

    def move(self, label: str, angle_deg: float, distance_m: float, watch_sensors: set[str], done: dict) -> None:
        pass


class IrEdgeTracker:
    def __init__(self) -> None:
        self.previous: dict[str, bool] | None = None

    def update(self, state: dict[str, bool]) -> tuple[list[str], list[str]]:
        if self.previous is None:
            self.previous = dict(state)
            return [], []
        rising = sorted(name for name, active in state.items() if active and not self.previous.get(name, False))
        falling = sorted(name for name, active in state.items() if not active and self.previous.get(name, False))
        self.previous = dict(state)
        return rising, falling


def log_event(args, kind: str, **data) -> None:
    logger = getattr(args, "logger", None)
    if logger is not None:
        logger.event(kind, **data)


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
    def __init__(self, *, active_low: bool, pull_up: bool, mock: bool, logic: str = "baseline") -> None:
        self.active_low = active_low
        self.mock = mock
        self.logic = logic
        self.devices = {}
        self.baseline_raw = {}
        if mock:
            return
        ensure_gpiozero_runtime()
        from gpiozero import DigitalInputDevice

        try:
            self.devices = {name: DigitalInputDevice(pin, pull_up=pull_up) for name, pin in IR_PINS.items()}
            time.sleep(0.05)
            for name, device in self.devices.items():
                self.baseline_raw[name] = bool(device.value)
            print(
                f"gpio | IR logic={self.logic} baseline "
                + " ".join(f"{IR_LABELS[name]}={1 if value else 0}" for name, value in self.baseline_raw.items())
                + " FR=NA",
                flush=True,
            )
        except Exception as e:
            # If the GPIO pins cannot be claimed (e.g. 'GPIO busy'), fall back to mock
            # mode so the Pi can still run mission logic and talk to the ESP.
            print(f"warn | GPIO initialization failed, falling back to mock IR: {e}", flush=True)
            # Clean up any partially-created devices
            try:
                for device in self.devices.values():
                    try:
                        device.close()
                    except Exception:
                        pass
            except Exception:
                pass
            self.devices = {}
            self.baseline_raw = {name: False for name in IR_PINS}
            self.mock = True

    def read(self) -> dict[str, bool]:
        if self.mock:
            return {name: False for name in IR_SENSOR_ORDER}

        state = {name: False for name in IR_SENSOR_ORDER}
        for name, device in self.devices.items():
            raw_high = bool(device.value)
            if self.logic == "active-low":
                state[name] = not raw_high
            elif self.logic == "active-high":
                state[name] = raw_high
            else:
                state[name] = raw_high != self.baseline_raw.get(name, raw_high)
        return state

    def close(self) -> None:
        for device in self.devices.values():
            device.close()


class GpioBuzzer:
    def __init__(self, *, pin: int | None, active_high: bool, enabled: bool, mock: bool) -> None:
        self.pin = pin
        self.device = None
        self.enabled = bool(enabled and pin is not None and not mock)
        if not self.enabled:
            return

        try:
            ensure_gpiozero_runtime()
            from gpiozero import OutputDevice

            self.device = OutputDevice(pin, active_high=active_high, initial_value=False)
            print(
                f"gpio | buzzer GPIO{pin} active_high={1 if active_high else 0}",
                flush=True,
            )
        except SystemExit as exc:
            print(f"warn | buzzer GPIO{pin} unavailable; buzzer disabled: {exc}", flush=True)
            self.device = None
            self.enabled = False
        except Exception as exc:
            print(f"warn | buzzer GPIO{pin} unavailable; buzzer disabled: {exc}", flush=True)
            self.device = None
            self.enabled = False

    def on(self) -> None:
        if self.device is not None:
            self.device.on()

    def off(self) -> None:
        if self.device is not None:
            self.device.off()

    def close(self) -> None:
        try:
            self.off()
        finally:
            if self.device is not None:
                self.device.close()


class GpioStepperLift:
    def __init__(
        self,
        *,
        step_pin: int,
        dir_pin: int,
        en_pin: int,
        speed_sps: float,
        step_high_us: int,
        enabled: bool,
        mock: bool,
    ) -> None:
        self.step_device = None
        self.dir_device = None
        self.en_device = None
        self.lock = threading.Lock()
        self.speed_sps = max(1.0, float(speed_sps))
        self.step_high_us = max(1, int(step_high_us))
        self.enabled = bool(enabled and not mock)
        if not self.enabled:
            return

        try:
            ensure_gpiozero_runtime()
            from gpiozero import DigitalOutputDevice

            self.step_device = DigitalOutputDevice(step_pin, initial_value=False)
            self.dir_device = DigitalOutputDevice(dir_pin, initial_value=False)
            self.en_device = DigitalOutputDevice(en_pin, initial_value=True)
            print(
                f"gpio | stepper STEP=GPIO{step_pin} DIR=GPIO{dir_pin} EN=GPIO{en_pin}",
                flush=True,
            )
        except SystemExit as exc:
            print(f"warn | stepper unavailable; lift disabled: {exc}", flush=True)
            self.close()
            self.enabled = False
        except Exception as exc:
            print(f"warn | stepper unavailable; lift disabled: {exc}", flush=True)
            self.close()
            self.enabled = False

    def status_text(self, steps: int, direction: int) -> str:
        state = "enabled" if self.enabled else "disabled"
        return f"stepper | {state} steps={steps} direction={direction} speed={self.speed_sps:.1f} steps/s"

    def run_steps(self, steps: int, direction: int, *, label: str = "lift") -> bool:
        steps = max(0, int(steps))
        direction_sign = 1 if direction >= 0 else -1
        if steps <= 0:
            print(f"stepper | {label} skipped because steps=0", flush=True)
            return True
        if not self.enabled:
            print(f"stepper | {label} skipped because stepper is disabled", flush=True)
            return True
        if self.step_device is None or self.dir_device is None or self.en_device is None:
            print(f"stepper | {label} skipped because GPIO is unavailable", flush=True)
            return False

        interval_us = int(1_000_000.0 / self.speed_sps)
        interval_us = max(interval_us, self.step_high_us + 50)

        print(
            f"stepper | {label} start steps={steps} direction={direction_sign} speed={self.speed_sps:.1f}",
            flush=True,
        )
        with self.lock:
            if direction_sign > 0:
                self.dir_device.on()
            else:
                self.dir_device.off()
            self.en_device.off()
            time.sleep(STEPPER_DIR_SETUP_S)
            try:
                for _ in range(steps):
                    self.step_device.on()
                    time.sleep(self.step_high_us / 1_000_000.0)
                    self.step_device.off()
                    time.sleep(max((interval_us - self.step_high_us) / 1_000_000.0, 0.0))
            finally:
                self.step_device.off()
                self.en_device.on()
        print(f"stepper | {label} completed steps={steps} direction={direction_sign}", flush=True)
        return True

    def run_until_released(self, direction: int, stop_event: threading.Event, *, label: str = "lift hold") -> bool:
        direction_sign = 1 if direction >= 0 else -1
        if not self.enabled:
            print(f"stepper | {label} skipped because stepper is disabled", flush=True)
            return True
        if self.step_device is None or self.dir_device is None or self.en_device is None:
            print(f"stepper | {label} skipped because GPIO is unavailable", flush=True)
            return False

        interval_us = int(1_000_000.0 / self.speed_sps)
        interval_us = max(interval_us, self.step_high_us + 50)

        print(
            f"stepper | {label} start direction={direction_sign} speed={self.speed_sps:.1f}",
            flush=True,
        )
        with self.lock:
            if direction_sign > 0:
                self.dir_device.on()
            else:
                self.dir_device.off()
            self.en_device.off()
            time.sleep(STEPPER_DIR_SETUP_S)
            try:
                while not stop_event.is_set():
                    self.step_device.on()
                    time.sleep(self.step_high_us / 1_000_000.0)
                    self.step_device.off()
                    time.sleep(max((interval_us - self.step_high_us) / 1_000_000.0, 0.0))
            finally:
                self.step_device.off()
                self.en_device.on()
        print(f"stepper | {label} stopped direction={direction_sign}", flush=True)
        return True

    def close(self) -> None:
        for device, off_first in (
            (self.step_device, True),
            (self.dir_device, True),
            (self.en_device, False),
        ):
            if device is None:
                continue
            try:
                if off_first:
                    device.off()
                else:
                    device.on()
            except Exception:
                pass
            try:
                device.close()
            except Exception:
                pass
        self.step_device = None
        self.dir_device = None
        self.en_device = None


class EspPiControlLink:
    def __init__(self, port: str, baud: int, *, verbose_serial: bool = False) -> None:
        ensure_serial_runtime()
        import serial

        self.port = port
        self.baud = baud
        self.verbose_serial = verbose_serial
        self.seq = 1
        self.events: queue.Queue[Event] = queue.Queue()
        self.pending_events: list[Event] = []
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

            if self.verbose_serial:
                try:
                    print(f"rcv_raw | {raw}", flush=True)
                except Exception:
                    pass

            data = self._parse_json_line(raw)
            if data is None:
                # Non-JSON line received; keep it visible for debugging.
                if self.verbose_serial:
                    try:
                        print(f"parse_fail | raw={raw}", flush=True)
                    except Exception:
                        pass
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

    def send_position_move(self, angle_deg: float, distance_m: float, heading_deg: float, timeout_s: float) -> int:
        distance_cm = max(0.0, distance_m * 100.0)
        return self.send_command(
            "MOVE "
            f"angle={angle_deg:.3f} "
            f"dist={distance_cm:.3f} "
            f"heading={heading_deg:.3f} "
            f"timeout={int(timeout_s * 1000)}"
        )

    def next_event(self, timeout: float) -> Event | None:
        if self.pending_events:
            return self.pending_events.pop(0)
        try:
            return self.events.get(timeout=timeout)
        except queue.Empty:
            return None

    def drain_events(self) -> None:
        self.pending_events.clear()
        while True:
            try:
                self.events.get_nowait()
            except queue.Empty:
                return

    def wait_for(self, seq: int | None, wanted_types: set[str], timeout: float) -> dict:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for index, event in enumerate(list(self.pending_events)):
                if event.data is None:
                    continue
                data = event.data
                seq_matches = seq is None or data.get("seq") == seq
                if data.get("type") in wanted_types and seq_matches:
                    self.pending_events.pop(index)
                    return data

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
            self.pending_events.append(event)

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


class PoseAccumulator:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw

    def apply_body_delta(self, forward_m: float, strafe_m: float, yaw_deg: float) -> tuple[float, float, float]:
        yaw = math.radians(float(yaw_deg))
        # Match the sim brain convention: physical forward is negative local X.
        dx_body = -float(forward_m)
        dy_body = -float(strafe_m)
        self.x += dx_body * math.cos(yaw) - dy_body * math.sin(yaw)
        self.y += dx_body * math.sin(yaw) + dy_body * math.cos(yaw)
        self.yaw = yaw
        return self.pose()

    def apply_done(self, done: dict) -> tuple[float, float, float]:
        forward_m = 0.01 * float(done.get("forwardCm", 0.0))
        strafe_m = 0.01 * float(done.get("strafeCm", 0.0))
        yaw_deg = float(done.get("headingDeg", math.degrees(self.yaw)))
        return self.apply_body_delta(forward_m, strafe_m, yaw_deg)

    def set_from_telemetry(self, telemetry: dict) -> tuple[float, float, float]:
        pose = telemetry.get("pose") or telemetry.get("odometry") or {}
        imu = telemetry.get("imu") or {}
        forward_m = 0.01 * float(pose.get("forwardCm", 0.0))
        strafe_m = 0.01 * float(pose.get("strafeCm", 0.0))
        yaw_deg = float(pose.get("yawDeg", imu.get("yawDeg", math.degrees(self.yaw))))
        self.x = -forward_m
        self.y = -strafe_m
        self.yaw = math.radians(yaw_deg)
        return self.pose()


@dataclass
class MissionMemory:
    forward_m: float = 0.0
    lateral_m: float = 0.0

    def record_completed_move(self, angle_deg: float, distance_m: float, done: dict) -> None:
        if done.get("result") == "ir_stop":
            return

        angle_rad = math.radians(angle_deg)
        command_forward_m = math.cos(angle_rad) * distance_m
        command_lateral_m = math.sin(angle_rad) * distance_m

        if abs(command_forward_m) >= abs(command_lateral_m):
            reported_forward_m = 0.01 * float(done.get("forwardCm", command_forward_m * 100.0))
            if abs(reported_forward_m) < 1e-6:
                reported_forward_m = command_forward_m
            self.forward_m = max(0.0, self.forward_m + reported_forward_m)
        else:
            self.lateral_m += command_lateral_m

    def sync_forward_from_brain(self, brain: "SimpleCardinalRealBrain") -> None:
        self.forward_m = max(0.0, brain.along_track_progress())

    def sync_from_brain(self, brain: "SimpleCardinalRealBrain") -> None:
        self.forward_m = max(0.0, brain.along_track_progress())
        self.lateral_m = brain.cross_track_error()

    def snap_center_if_close(self, tolerance_m: float) -> None:
        if abs(self.lateral_m) <= tolerance_m:
            self.lateral_m = 0.0


class SimpleCardinalRealBrain:
    def __init__(self, args) -> None:
        self.goal_distance = args.goal_distance
        self.goal_tolerance = args.goal_tolerance
        self.forward_weight = args.forward_weight
        self.lateral_weight = args.lateral_weight
        self.backoff_weight = args.backoff_weight
        self.line_kp = args.line_kp
        self.max_line_correction_weight = args.max_line_correction_weight
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
        self.sensor_hits = {name: False for name in IR_SENSOR_ORDER}
        self.sensor_update_sec = {name: None for name in IR_SENSOR_ORDER}

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
        # Gracefully handle sensors that may be absent from the mapping
        if sensor_name not in self.sensor_update_sec:
            return False
        stamp = self.sensor_update_sec[sensor_name]
        if stamp is None:
            return False
        if self._now - stamp > self.sensor_timeout_sec:
            return False
        return bool(self.sensor_hits.get(sensor_name, False))

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
        if self.front_left_blocked():
            return RIGHT
        if self.front_right_blocked():
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
        vy = clamp(-self.line_kp * line_error, -self.max_line_correction_weight, self.max_line_correction_weight)
        return self.set_command(-self.forward_weight, vy, 0.0)

    def run_backoff(self) -> MotionCommand:
        backed_off = self.backoff_start_progress - self.along_track_progress()
        if backed_off >= self.backoff_distance:
            self.state = "SHIFT_OUT"
            return self.set_command(0.0, 0.0, 0.0)
        return self.set_command(self.backoff_weight, 0.0, 0.0)

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
            vy = float(self.current_shift_direction) * self.lateral_weight
            return self.set_command(0.0, vy, 0.0)

        if self.front_clear_cross is None:
            self.front_clear_cross = line_error
            self.current_offset_target = (
                self.front_clear_cross
                + self.current_shift_direction * self.post_front_clear_lateral_distance
            )

        offset_error = self.current_offset_target - line_error
        vy = clamp(offset_error * 1.8, -self.lateral_weight, self.lateral_weight)
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

        cmd = self.set_command(-self.forward_weight, 0.0, 0.0)
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

        return_direction = LEFT if line_error > 0.0 else RIGHT
        if self.side_blocked(return_direction):
            self.advance_start_progress = self.along_track_progress()
            self.side_clear_progress = self.along_track_progress()
            self.state = "ADVANCE_CLEAR"
            return self.set_command(0.0, 0.0, 0.0)

        vy = clamp(-self.line_kp * line_error, -self.lateral_weight, self.lateral_weight)
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


def command_to_position_step(command: MotionCommand, brain: SimpleCardinalRealBrain, args) -> tuple[float, float] | None:
    forward_component = -command.vx
    strafe_component = command.vy
    magnitude = math.hypot(forward_component, strafe_component)
    if magnitude <= 1e-6:
        return None

    angle_deg = math.degrees(math.atan2(strafe_component, forward_component))
    distance_m = args.position_step

    forward_fraction = forward_component / magnitude
    if brain.state == "MOVE_TO_GOAL" and forward_fraction > 0.001:
        remaining = max(0.0, brain.goal_distance - brain.along_track_progress())
        distance_m = min(distance_m, remaining / forward_fraction)

    if distance_m < args.min_position_step:
        return None
    return angle_deg, distance_m


def simulated_done(angle_deg: float, distance_m: float, heading_deg: float) -> dict:
    angle_rad = math.radians(angle_deg)
    return {
        "type": "done",
        "result": "completed",
        "forwardCm": math.cos(angle_rad) * distance_m * 100.0,
        "strafeCm": math.sin(angle_rad) * distance_m * 100.0,
        "headingDeg": heading_deg,
    }


def done_from_latest_telemetry(link: EspPiControlLink, seq: int, result: str, ir: list[str] | None = None) -> dict:
    telemetry, _stamp = link.latest_telemetry()
    pose = (telemetry or {}).get("pose") or {}
    imu = (telemetry or {}).get("imu") or {}
    return {
        "type": "done",
        "seq": seq,
        "result": result,
        "ir": ir or [],
        "forwardCm": float(pose.get("forwardCm", 0.0)),
        "strafeCm": float(pose.get("strafeCm", 0.0)),
        "headingDeg": float(pose.get("yawDeg", imu.get("yawDeg", 0.0))),
    }


def stop_active_move(
    link: EspPiControlLink,
    active_seq: int,
    *,
    result: str,
    ir: list[str] | None = None,
    done_timeout_s: float = 1.5,
) -> dict:
    stop_seq = link.send_command("STOP")
    try:
        link.wait_for(stop_seq, {"ack"}, timeout=ACK_TIMEOUT_S)
    except TimeoutError as exc:
        print(f"warn | STOP ack timeout: {exc}", flush=True)

    try:
        done = link.wait_for(active_seq, {"done"}, timeout=done_timeout_s)
    except TimeoutError:
        done = done_from_latest_telemetry(link, active_seq, result, ir)
    else:
        done = dict(done)
        done["result"] = result
        done["ir"] = ir or done.get("ir", [])
    return done


def wait_for_move_done_or_ir(
    link: EspPiControlLink,
    seq: int,
    ir_bank: GpioIrBank,
    timeout_s: float,
    watch_sensors: set[str],
    *,
    require_fresh_edge: bool = False,
    timeout_returns_done: bool = False,
) -> dict:
    deadline = time.monotonic() + timeout_s
    armed = {name: True for name in watch_sensors}
    if require_fresh_edge and watch_sensors:
        initial_state = ir_bank.read()
        armed = {name: not initial_state.get(name, False) for name in watch_sensors}
        initially_active = sorted(name for name in watch_sensors if initial_state.get(name, False))
        if initially_active:
            print(f"watch | waiting for {initially_active} to clear before accepting edge", flush=True)

    while time.monotonic() < deadline:
        ir_state = ir_bank.read()
        if require_fresh_edge:
            active = []
            for name in watch_sensors:
                is_active = ir_state.get(name, False)
                if not is_active:
                    armed[name] = True
                elif armed.get(name, True):
                    active.append(name)
        else:
            active = sorted(name for name in watch_sensors if ir_state.get(name, False))
        if active:
            print(f"ir_stop | active={active} state={ir_summary(ir_state)}", flush=True)
            return stop_active_move(link, seq, result="ir_stop", ir=active)

        event = link.next_event(0.02)
        if event is None or event.data is None:
            continue

        data = event.data
        if data.get("type") == "done" and data.get("seq") == seq:
            return data

    stopped = stop_active_move(link, seq, result="timeout_stop", ir=[])
    if timeout_returns_done:
        print(f"timeout_stop | seq={seq} after {timeout_s:.2f}s", flush=True)
        return stopped
    raise TimeoutError(f"Timed out waiting for MOVE seq={seq}")


def execute_position_step(
    link: EspPiControlLink | None,
    pose_tracker: PoseAccumulator,
    ir_bank: GpioIrBank,
    angle_deg: float,
    distance_m: float,
    args,
    watch_front_ir: bool,
    watch_sensors: set[str] | None = None,
    require_fresh_edge: bool = False,
    move_timeout_s: float | None = None,
    timeout_returns_done: bool = False,
) -> dict:
    if args.dry_run:
        done = simulated_done(angle_deg, distance_m, args.heading)
        pose_tracker.apply_done(done)
        time.sleep(args.dry_run_step_delay)
        return done

    if watch_sensors is None:
        watch_sensors = {"front", "front_left", "front_right"} if watch_front_ir else set()
    timeout_s = args.move_timeout if move_timeout_s is None else max(0.05, float(move_timeout_s))
    link.drain_events()

    print(
        f"send_move | angle={angle_deg:.3f} dist={distance_m:.3f}m heading={args.heading:.3f} "
        f"watch={sorted(watch_sensors)} timeout={timeout_s:.2f}s",
        flush=True,
    )

    seq = link.send_position_move(angle_deg, distance_m, args.heading, timeout_s)
    print(f"sent seq={seq}", flush=True)

    ack = link.wait_for(seq, {"ack"}, timeout=ACK_TIMEOUT_S)
    print(f"ack | {ack}", flush=True)
    if not ack.get("ok", False):
        raise RuntimeError(f"MOVE rejected: {ack.get('message', 'no message')}")

    done = wait_for_move_done_or_ir(
        link,
        seq,
        ir_bank,
        timeout_s,
        watch_sensors,
        require_fresh_edge=require_fresh_edge,
        timeout_returns_done=timeout_returns_done,
    )
    print(f"done | {done}", flush=True)
    return done


FRONT_WATCH_SENSORS = {"front", "front_left", "front_right"}
SIDE_WATCH_SENSORS = {"left", "right"}
ALL_WATCH_SENSORS = FRONT_WATCH_SENSORS | SIDE_WATCH_SENSORS


def is_connected_sensor(sensor_name: str) -> bool:
    return sensor_name in IR_PINS


def connected_sensors(sensor_names) -> set[str]:
    return {name for name in sensor_names if is_connected_sensor(name)}


def direction_to_angle(direction: int) -> float:
    return 90.0 if direction == LEFT else -90.0


def direction_name(direction: int) -> str:
    return "left" if direction == LEFT else "right"


def side_sensor_for_direction(direction: int) -> str:
    return "left" if direction == LEFT else "right"


def front_corner_sensor_after_strafe(direction: int) -> str:
    return "front_right" if direction == LEFT else "front_left"


def side_sensor_after_front_avoidance(direction: int) -> str:
    return side_sensor_for_direction(opposite_direction(direction))


def refresh_pose_from_telemetry(
    link: EspPiControlLink | None,
    pose_tracker: PoseAccumulator,
    timeout_s: float,
    min_stamp: float = 0.0,
) -> bool:
    if link is None:
        return False

    deadline = time.monotonic() + max(0.0, timeout_s)
    while time.monotonic() < deadline:
        telemetry, stamp = link.latest_telemetry()
        if telemetry is not None and stamp >= min_stamp:
            pose_tracker.set_from_telemetry(telemetry)
            return True
        time.sleep(0.02)
    return False


def update_brain_from_pose(
    brain: SimpleCardinalRealBrain,
    pose_tracker: PoseAccumulator,
    ir_bank: GpioIrBank,
    enabled: bool,
) -> dict[str, bool]:
    now = time.monotonic()
    x, y, yaw = pose_tracker.pose()
    ir_state = ir_bank.read()
    brain.update_pose(x, y, yaw, now)
    brain.update_ir(ir_state, now)
    brain.set_enabled(enabled)
    return ir_state


def print_mission_status(
    brain: SimpleCardinalRealBrain,
    pose_tracker: PoseAccumulator,
    ir_state: dict[str, bool],
    last_done: dict,
    mission: MissionMemory | None = None,
) -> None:
    x, y, yaw = pose_tracker.pose()
    progress = mission.forward_m if mission is not None else brain.along_track_progress()
    cross = mission.lateral_m if mission is not None else brain.cross_track_error()
    print(
        "state={state} progress={progress:.3f}/{goal:.3f} cross={cross:.3f} "
        "last_move={result} d_cm(f={df:.1f},s={ds:.1f},yaw={dy:.1f}) "
        "pose_xy=({x:.3f},{y:.3f}) ir[{ir}]".format(
            state=brain.state,
            progress=progress,
            goal=brain.goal_distance,
            cross=cross,
            result=last_done.get("result", "none"),
            df=float(last_done.get("forwardCm", 0.0)),
            ds=float(last_done.get("strafeCm", 0.0)),
            dy=float(last_done.get("headingDeg", math.degrees(yaw))),
            x=x,
            y=y,
            ir=ir_summary(ir_state),
        ),
        flush=True,
    )


def choose_front_avoidance(ir_state: dict[str, bool], brain: SimpleCardinalRealBrain, args) -> tuple[int, float, str]:
    front = bool(ir_state.get("front", False))
    front_left = bool(ir_state.get("front_left", False))
    front_right = bool(ir_state.get("front_right", False))

    if front:
        if front_left and not front_right:
            return RIGHT, args.front_strafe_distance, "front+front_left"
        if front_right and not front_left:
            return LEFT, args.front_strafe_distance, "front+front_right"
        return RIGHT, args.front_strafe_distance, "front"

    if front_left:
        return RIGHT, args.front_corner_strafe_distance, "front_left"
    if front_right:
        return LEFT, args.front_corner_strafe_distance, "front_right"
    return brain.preferred_direction, args.front_strafe_distance, "front"


def wait_for_dynamic_front_clear(
    ir_bank: GpioIrBank,
    brain: SimpleCardinalRealBrain,
    pose_tracker: PoseAccumulator,
    args,
    active_sensors: list[str],
) -> tuple[bool, dict[str, bool]]:
    hold_s = max(0.0, float(args.front_dynamic_hold))
    print(
        f"dynamic | front obstacle {active_sensors}; stop+buzzer hold {hold_s:.1f}s",
        flush=True,
    )
    log_event(args, "front_dynamic_hold", ir=active_sensors, hold_s=hold_s)

    buzzer = getattr(args, "buzzer", None)
    try:
        if buzzer is not None:
            buzzer.on()
        deadline = time.monotonic() + hold_s
        while time.monotonic() < deadline:
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
    finally:
        if buzzer is not None:
            buzzer.off()

    ir_state = update_brain_from_pose(brain, pose_tracker, ir_bank, True)
    still_front = any(ir_state.get(name, False) for name in FRONT_WATCH_SENSORS)
    if still_front:
        still_active = sorted(name for name in FRONT_WATCH_SENSORS if ir_state.get(name, False))
        print(f"dynamic | obstacle still present after {hold_s:.1f}s: {still_active}; running static avoidance", flush=True)
        log_event(args, "front_static_confirmed", ir=still_active)
    else:
        print(f"dynamic | front clear after {hold_s:.1f}s; continuing mission", flush=True)
        log_event(args, "front_dynamic_clear", ir=ir_summary(ir_state))
    return still_front, ir_state


def side_escape_direction(active_sensors: list[str]) -> int:
    if "left" in active_sensors:
        return RIGHT
    return LEFT


def execute_segment(
    link: EspPiControlLink | None,
    pose_tracker: PoseAccumulator,
    ir_bank: GpioIrBank,
    brain: SimpleCardinalRealBrain,
    args,
    label: str,
    angle_deg: float,
    distance_m: float,
    watch_sensors: set[str],
    mission: MissionMemory | None = None,
    require_fresh_edge: bool = False,
    move_timeout_s: float | None = None,
    timeout_returns_done: bool = False,
) -> dict:
    distance_m = max(0.0, float(distance_m))
    print(f"path | {label} angle={angle_deg:.1f} dist_cm={distance_m * 100.0:.1f}", flush=True)
    move_started = time.monotonic()
    done = execute_position_step(
        link,
        pose_tracker,
        ir_bank,
        angle_deg,
        distance_m,
        args,
        False,
        watch_sensors,
        require_fresh_edge=require_fresh_edge,
        move_timeout_s=move_timeout_s,
        timeout_returns_done=timeout_returns_done,
    )
    fresh_pose = refresh_pose_from_telemetry(link, pose_tracker, args.telemetry_timeout, move_started)
    if link is not None and not fresh_pose:
        pose_tracker.set_from_telemetry({"pose": done, "imu": {"yawDeg": done.get("headingDeg", args.heading)}})
    update_brain_from_pose(brain, pose_tracker, ir_bank, True)
    if mission is not None:
        if done.get("result") == "ir_stop":
            mission.sync_from_brain(brain)
        else:
            mission.record_completed_move(angle_deg, distance_m, done)
            mission.sync_from_brain(brain)
            mission.snap_center_if_close(args.rejoin_tolerance)
    logger = getattr(args, "logger", None)
    if logger is not None:
        logger.move(label, angle_deg, distance_m, watch_sensors, done)
    return done


def execute_front_search_strafe(
    link: EspPiControlLink | None,
    pose_tracker: PoseAccumulator,
    ir_bank: GpioIrBank,
    brain: SimpleCardinalRealBrain,
    args,
    direction: int,
    mission: MissionMemory,
) -> tuple[dict, int, str]:
    for attempt in range(2):
        corner_sensor = front_corner_sensor_after_strafe(direction)
        side_block_sensor = side_sensor_for_direction(direction)
        corner_watch = connected_sensors({corner_sensor})
        side_block_watch = connected_sensors({side_block_sensor})

        if not corner_watch:
            print(
                f"warn | {corner_sensor} is not connected; using short fallback strafe "
                f"{args.front_corner_strafe_distance * 100.0:.1f}cm",
                flush=True,
            )
            done = execute_segment(
                link,
                pose_tracker,
                ir_bank,
                brain,
                args,
                f"front avoidance {direction_name(direction)} fallback strafe",
                direction_to_angle(direction),
                args.front_corner_strafe_distance,
                side_block_watch,
                mission,
                move_timeout_s=args.front_corner_buffer_timeout,
                timeout_returns_done=True,
            )
            return done, direction, corner_sensor

        watch = corner_watch | side_block_watch
        done = execute_segment(
            link,
            pose_tracker,
            ir_bank,
            brain,
            args,
            f"front avoidance search strafe {direction_name(direction)}",
            direction_to_angle(direction),
            args.front_strafe_search_distance,
            watch,
            mission,
            move_timeout_s=args.front_strafe_search_timeout,
        )

        active = set(done.get("ir", []))
        if done.get("result") == "ir_stop" and side_block_sensor in active:
            next_direction = opposite_direction(direction)
            print(
                f"avoid | {side_block_sensor} blocked while strafing {direction_name(direction)}; "
                f"switching {direction_name(next_direction)}",
                flush=True,
            )
            direction = next_direction
            continue
        if done.get("result") == "ir_stop" and corner_sensor in active:
            return done, direction, corner_sensor
        return done, direction, corner_sensor

    raise RuntimeError("Both strafe directions were blocked during front avoidance")


def execute_strafe_until_corner_falling(
    link: EspPiControlLink | None,
    pose_tracker: PoseAccumulator,
    ir_bank: GpioIrBank,
    brain: SimpleCardinalRealBrain,
    args,
    direction: int,
    corner_sensor: str,
    action_budget: list[int],
    mission: MissionMemory,
) -> dict:
    if not is_connected_sensor(corner_sensor):
        print(f"warn | {corner_sensor} falling edge unavailable; skipping diagonal falling wait", flush=True)
        return {"result": "corner_falling_unavailable", "ir": [corner_sensor], "forwardCm": 0.0, "strafeCm": 0.0}

    initial_state = ir_bank.read()
    if not initial_state.get(corner_sensor, False):
        print(f"edge | {corner_sensor}=0 already clear", flush=True)
        return {"result": "corner_already_clear", "ir": [corner_sensor], "forwardCm": 0.0, "strafeCm": 0.0}

    if args.dry_run:
        done = execute_segment(
            link,
            pose_tracker,
            ir_bank,
            brain,
            args,
            f"strafe until {corner_sensor} falling",
            direction_to_angle(direction),
            args.front_corner_buffer_distance,
            set(),
            mission,
        )
        done["result"] = "corner_falling"
        done["ir"] = [corner_sensor]
        return done

    side_block_sensor = side_sensor_for_direction(direction)
    print(
        f"path | strafe {direction_name(direction)} until {corner_sensor} falling edge",
        flush=True,
    )
    move_started = time.monotonic()
    seq = link.send_position_move(
        direction_to_angle(direction),
        args.front_strafe_search_distance,
        args.heading,
        args.front_strafe_search_timeout,
    )
    print(f"sent seq={seq}", flush=True)
    ack = link.wait_for(seq, {"ack"}, timeout=ACK_TIMEOUT_S)
    print(f"ack | {ack}", flush=True)
    if not ack.get("ok", False):
        raise RuntimeError(f"MOVE rejected: {ack.get('message', 'no message')}")

    deadline = time.monotonic() + args.front_strafe_search_timeout
    previous_active = True
    while time.monotonic() < deadline:
        ir_state = ir_bank.read()
        if ir_state.get(side_block_sensor, False):
            done = stop_active_move(link, seq, result="ir_stop", ir=[side_block_sensor])
            fresh_pose = refresh_pose_from_telemetry(link, pose_tracker, args.telemetry_timeout, move_started)
            if not fresh_pose:
                pose_tracker.set_from_telemetry({"pose": done, "imu": {"yawDeg": done.get("headingDeg", args.heading)}})
            update_brain_from_pose(brain, pose_tracker, ir_bank, True)
            mission.sync_from_brain(brain)
            logger = getattr(args, "logger", None)
            if logger is not None:
                logger.move(f"strafe until {corner_sensor} falling", direction_to_angle(direction), args.front_strafe_search_distance, {side_block_sensor, corner_sensor}, done)
            return done

        active = bool(ir_state.get(corner_sensor, False))
        if previous_active and not active:
            done = stop_active_move(link, seq, result="corner_falling", ir=[corner_sensor])
            fresh_pose = refresh_pose_from_telemetry(link, pose_tracker, args.telemetry_timeout, move_started)
            if not fresh_pose:
                pose_tracker.set_from_telemetry({"pose": done, "imu": {"yawDeg": done.get("headingDeg", args.heading)}})
            update_brain_from_pose(brain, pose_tracker, ir_bank, True)
            mission.sync_from_brain(brain)
            print(f"edge | {corner_sensor}=0 falling edge", flush=True)
            logger = getattr(args, "logger", None)
            if logger is not None:
                logger.move(f"strafe until {corner_sensor} falling", direction_to_angle(direction), args.front_strafe_search_distance, {side_block_sensor, corner_sensor}, done)
            return done
        previous_active = active

        event = link.next_event(0.02)
        if event is None or event.data is None:
            continue
        data = event.data
        if data.get("type") == "done" and data.get("seq") == seq:
            fresh_pose = refresh_pose_from_telemetry(link, pose_tracker, args.telemetry_timeout, move_started)
            if not fresh_pose:
                pose_tracker.set_from_telemetry({"pose": data, "imu": {"yawDeg": data.get("headingDeg", args.heading)}})
            update_brain_from_pose(brain, pose_tracker, ir_bank, True)
            mission.sync_from_brain(brain)
            raise RuntimeError(f"{corner_sensor} did not produce a falling edge before the strafe search limit")

    stop_active_move(link, seq, result="timeout_stop", ir=[])
    raise TimeoutError(f"Timed out waiting for {corner_sensor} falling edge")


def execute_forward_until_side_falling(
    link: EspPiControlLink | None,
    pose_tracker: PoseAccumulator,
    ir_bank: GpioIrBank,
    brain: SimpleCardinalRealBrain,
    args,
    side_sensor: str,
    action_budget: list[int],
    mission: MissionMemory,
) -> dict:
    if args.dry_run:
        done = execute_segment(
            link,
            pose_tracker,
            ir_bank,
            brain,
            args,
            f"forward until {side_sensor} clears",
            0.0,
            args.side_follow_dry_distance,
            set(),
            mission,
        )
        done["result"] = "side_falling"
        done["ir"] = [side_sensor]
        return done

    print(
        f"path | forward until {side_sensor} rising+falling edge "
        f"max_cm={args.side_follow_search_distance * 100.0:.1f}",
        flush=True,
    )
    move_started = time.monotonic()
    seq = link.send_position_move(0.0, args.side_follow_search_distance, args.heading, args.move_timeout)
    print(f"sent seq={seq}", flush=True)
    ack = link.wait_for(seq, {"ack"}, timeout=ACK_TIMEOUT_S)
    print(f"ack | {ack}", flush=True)
    if not ack.get("ok", False):
        raise RuntimeError(f"MOVE rejected: {ack.get('message', 'no message')}")

    deadline = time.monotonic() + args.move_timeout
    initial_state = ir_bank.read()
    seen_active = bool(initial_state.get(side_sensor, False))
    previous_active = seen_active
    if seen_active:
        print(f"edge | {side_sensor}=1 at forward start; waiting for falling edge", flush=True)

    while time.monotonic() < deadline:
        ir_state = ir_bank.read()
        front_watch = FRONT_WATCH_SENSORS if args.side_follow_watch_front else set()
        front_hits = sorted(name for name in front_watch if ir_state.get(name, False))
        if front_hits:
            done = stop_active_move(link, seq, result="ir_stop", ir=front_hits)
            fresh_pose = refresh_pose_from_telemetry(link, pose_tracker, args.telemetry_timeout, move_started)
            if not fresh_pose:
                pose_tracker.set_from_telemetry({"pose": done, "imu": {"yawDeg": done.get("headingDeg", args.heading)}})
            update_brain_from_pose(brain, pose_tracker, ir_bank, True)
            mission.sync_from_brain(brain)
            return done

        active = bool(ir_state.get(side_sensor, False))
        if active and not seen_active:
            seen_active = True
            print(f"edge | {side_sensor}=1 rising edge", flush=True)
        if seen_active and previous_active and not active:
            done = stop_active_move(link, seq, result="side_falling", ir=[side_sensor])
            fresh_pose = refresh_pose_from_telemetry(link, pose_tracker, args.telemetry_timeout, move_started)
            if not fresh_pose:
                pose_tracker.set_from_telemetry({"pose": done, "imu": {"yawDeg": done.get("headingDeg", args.heading)}})
            update_brain_from_pose(brain, pose_tracker, ir_bank, True)
            mission.sync_from_brain(brain)
            print(f"edge | {side_sensor}=0 falling edge", flush=True)
            return done
        previous_active = active

        event = link.next_event(0.02)
        if event is None or event.data is None:
            continue
        data = event.data
        if data.get("type") == "done" and data.get("seq") == seq:
            fresh_pose = refresh_pose_from_telemetry(link, pose_tracker, args.telemetry_timeout, move_started)
            if not fresh_pose:
                pose_tracker.set_from_telemetry({"pose": data, "imu": {"yawDeg": data.get("headingDeg", args.heading)}})
            update_brain_from_pose(brain, pose_tracker, ir_bank, True)
            mission.sync_from_brain(brain)
            raise RuntimeError(f"{side_sensor} IR did not produce a falling edge before the forward search limit")

    stop_active_move(link, seq, result="timeout_stop", ir=[])
    raise TimeoutError(f"Timed out waiting for {side_sensor} falling edge")


def execute_side_escape(
    link: EspPiControlLink | None,
    pose_tracker: PoseAccumulator,
    ir_bank: GpioIrBank,
    brain: SimpleCardinalRealBrain,
    args,
    active_sensors: list[str],
    action_budget: list[int],
    mission: MissionMemory,
) -> dict:
    action_budget[0] -= 1
    if action_budget[0] < 0:
        raise RuntimeError("Too many avoidance actions; stopping mission")

    direction = side_escape_direction(active_sensors)
    angle = direction_to_angle(direction)
    print(
        f"avoid | side={active_sensors} escape={direction_name(direction)} "
        f"{args.side_escape_distance * 100.0:.1f}cm",
        flush=True,
    )
    done = execute_segment(
        link,
        pose_tracker,
        ir_bank,
        brain,
        args,
        "side escape strafe",
        angle,
        args.side_escape_distance,
        set(),
        mission,
    )
    if done.get("result") == "ir_stop":
        return done

    done = execute_segment(
        link,
        pose_tracker,
        ir_bank,
        brain,
        args,
        "side escape forward",
        0.0,
        args.side_escape_forward_distance,
        ALL_WATCH_SENSORS,
        mission,
    )
    return done


def execute_recenter(
    link: EspPiControlLink | None,
    pose_tracker: PoseAccumulator,
    ir_bank: GpioIrBank,
    brain: SimpleCardinalRealBrain,
    args,
    action_budget: list[int],
    mission: MissionMemory,
) -> dict:
    last_done = {"result": "none", "forwardCm": 0.0, "strafeCm": 0.0, "headingDeg": args.heading}
    for _ in range(args.max_recenter_attempts):
        update_brain_from_pose(brain, pose_tracker, ir_bank, True)
        offset = mission.lateral_m
        if abs(offset) <= args.rejoin_tolerance:
            mission.lateral_m = 0.0
            print("path | centered on original line", flush=True)
            return last_done

        direction = LEFT if offset > 0.0 else RIGHT
        done = execute_segment(
            link,
            pose_tracker,
            ir_bank,
            brain,
            args,
            "return to center",
            direction_to_angle(direction),
            abs(offset),
            {side_sensor_for_direction(direction)},
            mission,
        )
        last_done = done
        if done.get("result") == "ir_stop":
            side_done = execute_side_escape(
                link,
                pose_tracker,
                ir_bank,
                brain,
                args,
                done.get("ir", []),
                action_budget,
                mission,
            )
            last_done = side_done
            if side_done.get("result") == "ir_stop":
                continue
    raise RuntimeError("Could not return to the original center line")


def execute_front_avoidance(
    link: EspPiControlLink | None,
    pose_tracker: PoseAccumulator,
    ir_bank: GpioIrBank,
    brain: SimpleCardinalRealBrain,
    args,
    ir_state: dict[str, bool],
    action_budget: list[int],
    mission: MissionMemory,
) -> dict:
    action_budget[0] -= 1
    if action_budget[0] < 0:
        raise RuntimeError("Too many avoidance actions; stopping mission")

    direction, _strafe_distance, reason = choose_front_avoidance(ir_state, brain, args)
    if ir_state.get(side_sensor_for_direction(direction), False):
        direction = opposite_direction(direction)
        print(f"avoid | preferred side blocked; switching {direction_name(direction)}", flush=True)
    corner_sensor = front_corner_sensor_after_strafe(direction)
    side_sensor = side_sensor_after_front_avoidance(direction)
    print(
        f"avoid | {reason}: strafe {direction_name(direction)} until {corner_sensor} rising/falling, "
        f"then follow {side_sensor} falling edge and advance {args.front_advance_distance * 100.0:.1f}cm",
        flush=True,
    )

    done, direction, corner_sensor = execute_front_search_strafe(
        link,
        pose_tracker,
        ir_bank,
        brain,
        args,
        direction,
        mission,
    )
    side_sensor = side_sensor_after_front_avoidance(direction)
    if done.get("result") == "ir_stop" and corner_sensor not in set(done.get("ir", [])):
        return handle_ir_stop(link, pose_tracker, ir_bank, brain, args, done.get("ir", []), action_budget, mission)
    if done.get("result") not in ("ir_stop", "timeout_stop", "completed"):
        raise RuntimeError("Front avoidance corner IR was not detected before strafe search limit")

    done = execute_strafe_until_corner_falling(
        link,
        pose_tracker,
        ir_bank,
        brain,
        args,
        direction,
        corner_sensor,
        action_budget,
        mission,
    )
    if done.get("result") == "ir_stop":
        active = done.get("ir", [])
        if side_sensor_for_direction(direction) in active:
            return execute_front_avoidance(
                link,
                pose_tracker,
                ir_bank,
                brain,
                args,
                {"front": True, side_sensor_for_direction(direction): True},
                action_budget,
                mission,
            )
        return handle_ir_stop(link, pose_tracker, ir_bank, brain, args, active, action_budget, mission)

    done = execute_forward_until_side_falling(
        link,
        pose_tracker,
        ir_bank,
        brain,
        args,
        side_sensor,
        action_budget,
        mission,
    )
    if done.get("result") == "ir_stop":
        return handle_ir_stop(link, pose_tracker, ir_bank, brain, args, done.get("ir", []), action_budget, mission)

    done = execute_segment(
        link,
        pose_tracker,
        ir_bank,
        brain,
        args,
        "side clear forward buffer",
        0.0,
        args.front_advance_distance,
        ALL_WATCH_SENSORS,
        mission,
        move_timeout_s=args.front_advance_timeout,
        timeout_returns_done=True,
    )
    if done.get("result") == "ir_stop":
        return handle_ir_stop(link, pose_tracker, ir_bank, brain, args, done.get("ir", []), action_budget, mission)

    return execute_recenter(link, pose_tracker, ir_bank, brain, args, action_budget, mission)


def handle_ir_stop(
    link: EspPiControlLink | None,
    pose_tracker: PoseAccumulator,
    ir_bank: GpioIrBank,
    brain: SimpleCardinalRealBrain,
    args,
    active_sensors: list[str],
    action_budget: list[int],
    mission: MissionMemory,
) -> dict:
    ir_state = {name: False for name in IR_SENSOR_ORDER}
    for name in active_sensors:
        ir_state[name] = True

    if any(ir_state.get(name, False) for name in FRONT_WATCH_SENSORS):
        still_front, ir_state = wait_for_dynamic_front_clear(
            ir_bank,
            brain,
            pose_tracker,
            args,
            sorted(name for name in FRONT_WATCH_SENSORS if ir_state.get(name, False)),
        )
        if not still_front:
            side_hits = sorted(name for name in SIDE_WATCH_SENSORS if ir_state.get(name, False))
            if side_hits:
                done = execute_side_escape(link, pose_tracker, ir_bank, brain, args, side_hits, action_budget, mission)
                if done.get("result") == "ir_stop":
                    return handle_ir_stop(link, pose_tracker, ir_bank, brain, args, done.get("ir", []), action_budget, mission)
                return done
            return {
                "result": "front_dynamic_clear",
                "ir": [],
                "forwardCm": 0.0,
                "strafeCm": 0.0,
                "headingDeg": args.heading,
            }
        return execute_front_avoidance(link, pose_tracker, ir_bank, brain, args, ir_state, action_budget, mission)
    if any(ir_state.get(name, False) for name in SIDE_WATCH_SENSORS):
        done = execute_side_escape(link, pose_tracker, ir_bank, brain, args, active_sensors, action_budget, mission)
        if done.get("result") == "ir_stop":
            return handle_ir_stop(link, pose_tracker, ir_bank, brain, args, done.get("ir", []), action_budget, mission)
        return done
    return {"result": "ignored_ir", "ir": active_sensors}


def execute_goal_correction(
    link: EspPiControlLink | None,
    pose_tracker: PoseAccumulator,
    ir_bank: GpioIrBank,
    brain: SimpleCardinalRealBrain,
    args,
    action_budget: list[int],
    mission: MissionMemory,
) -> dict:
    last_done = {"result": "none", "forwardCm": 0.0, "strafeCm": 0.0, "headingDeg": args.heading}
    for _ in range(args.max_goal_correction_attempts):
        overshoot = mission.forward_m - brain.goal_distance
        if overshoot <= args.goal_tolerance:
            return last_done
        print(f"path | back to 120cm mark dist_cm={overshoot * 100.0:.1f}", flush=True)
        last_done = execute_segment(
            link,
            pose_tracker,
            ir_bank,
            brain,
            args,
            "back to goal mark",
            180.0,
            overshoot,
            {"back"},
            mission,
        )
        if last_done.get("result") == "ir_stop":
            return handle_ir_stop(
                link,
                pose_tracker,
                ir_bank,
                brain,
                args,
                last_done.get("ir", []),
                action_budget,
                mission,
            )
    raise RuntimeError("Could not correct back to the 120cm goal mark")


def run_goal_lift(args) -> None:
    action = args.goal_stepper_action
    if not args.lift_on_goal:
        action = "none"
    if action == "none":
        print("stepper | goal action none", flush=True)
        return

    stepper = getattr(args, "stepper_lift", None)
    if stepper is None:
        print("stepper | goal lift unavailable", flush=True)
        return

    steps = int(args.lift_steps)
    if args.lift_direction is None:
        direction = STEPPER_UP_DIR if action == "up" else STEPPER_DOWN_DIR
    else:
        direction = int(args.lift_direction)
    label = f"goal stepper {action}"
    print(stepper.status_text(steps, direction), flush=True)
    log_event(args, "goal_stepper_start", action=action, steps=steps, direction=direction)
    ok = stepper.run_steps(steps, direction, label=label)
    log_event(args, "goal_stepper_done", action=action, steps=steps, direction=direction, ok=ok)


def run_segment_mission(
    link: EspPiControlLink | None,
    pose_tracker: PoseAccumulator,
    ir_bank: GpioIrBank,
    brain: SimpleCardinalRealBrain,
    args,
) -> None:
    last_status = 0.0
    last_done = {"result": "none", "forwardCm": 0.0, "strafeCm": 0.0, "headingDeg": args.heading}
    action_budget = [args.max_avoidance_actions]
    mission = MissionMemory()

    update_brain_from_pose(brain, pose_tracker, ir_bank, True)
    brain.state = "MOVE_TO_GOAL"

    # Send one initial full MOVE toward the goal (counts as the mission's
    # first commanded motion). This ensures the first command is a single
    # MOVE covering the full `goal_distance` (e.g., 1.20 m / 120 cm).
    try:
        print(f"path | initial mission forward dist_cm={brain.goal_distance * 100.0:.1f}", flush=True)
        initial_done = execute_segment(
            link,
            pose_tracker,
            ir_bank,
            brain,
            args,
            "initial mission forward",
            0.0,
            brain.goal_distance,
            FRONT_WATCH_SENSORS,
            mission,
        )
        if initial_done.get("result") == "ir_stop":
            # Handle the obstacle and continue the mission loop
            initial_done = handle_ir_stop(
                link,
                pose_tracker,
                ir_bank,
                brain,
                args,
                initial_done.get("ir", []),
                action_budget,
                mission,
            )
        last_done = initial_done
    except Exception as exc:
        print(f"error | initial move failed: {exc}", flush=True)
        raise

    while True:
        ir_state = update_brain_from_pose(brain, pose_tracker, ir_bank, True)
        now = time.monotonic()
        if now - last_status >= args.status_period:
            print_mission_status(brain, pose_tracker, ir_state, last_done, mission)
            last_status = now

        remaining = brain.goal_distance - mission.forward_m
        if remaining <= 0.0:
            last_done = execute_goal_correction(link, pose_tracker, ir_bank, brain, args, action_budget, mission)
            execute_recenter(link, pose_tracker, ir_bank, brain, args, action_budget, mission)
            update_brain_from_pose(brain, pose_tracker, ir_bank, True)
            if abs(mission.forward_m - brain.goal_distance) <= args.goal_tolerance and abs(mission.lateral_m) <= args.rejoin_tolerance:
                brain.state = "DONE"
                if link is not None:
                    link.command_ack("STOP")
                print("Goal reached; ESP stopped.", flush=True)
                run_goal_lift(args)
                return

        move_distance = max(0.0, remaining)
        if move_distance < args.min_position_step:
            move_distance = args.min_position_step

        last_done = execute_segment(
            link,
            pose_tracker,
            ir_bank,
            brain,
            args,
            "mission forward",
            0.0,
            move_distance,
            FRONT_WATCH_SENSORS,
            mission,
        )
        if last_done.get("result") == "ir_stop":
            last_done = handle_ir_stop(
                link,
                pose_tracker,
                ir_bank,
                brain,
                args,
                last_done.get("ir", []),
                action_budget,
                mission,
            )


def parse_manual_waypoint(raw: str, default_heading_deg: float) -> tuple[str, float, float, float]:
    parts = raw.split()
    if len(parts) not in (2, 3):
        raise ValueError("use: <direction> <distance_cm> [heading_deg]")
    direction = parts[0].upper()
    if direction not in DIRECTION_ANGLES_DEG:
        raise ValueError(f"direction must be one of: {' '.join(DIRECTION_ORDER)}")
    distance_cm = float(parts[1])
    if distance_cm <= 0.0:
        raise ValueError("distance must be greater than zero")
    heading = float(parts[2]) if len(parts) == 3 else default_heading_deg
    return direction, DIRECTION_ANGLES_DEG[direction], distance_cm / 100.0, heading


def run_move_mode(
    link: EspPiControlLink | None,
    pose_tracker: PoseAccumulator,
    ir_bank: GpioIrBank,
    brain: SimpleCardinalRealBrain,
    args,
) -> None:
    print(f"Move mode. Directions: {' '.join(DIRECTION_ORDER)}", flush=True)
    print("Commands: <direction> <distance_cm> [heading_deg], status, stop, q", flush=True)
    while True:
        raw = input("move> ").strip()
        if not raw:
            continue
        low = raw.lower()
        if low in ("q", "quit", "exit"):
            if link is not None:
                link.command_ack("STOP")
            return
        if low == "stop":
            if link is not None:
                link.command_ack("STOP")
            continue
        if low == "status":
            update_brain_from_pose(brain, pose_tracker, ir_bank, True)
            telemetry = None
            if link is not None:
                telemetry, _stamp = link.latest_telemetry()
            log_event(args, "status", telemetry=telemetry, ir=ir_bank.read())
            print_mission_status(brain, pose_tracker, ir_bank.read(), {"result": "status"})
            continue
        try:
            direction, angle_deg, distance_m, heading = parse_manual_waypoint(raw, args.heading)
        except ValueError as exc:
            print(f"input error | {exc}", flush=True)
            continue
        args.heading = heading
        watch = ALL_WATCH_SENSORS if args.manual_watch_ir else set()
        execute_segment(
            link,
            pose_tracker,
            ir_bank,
            brain,
            args,
            f"manual {direction}",
            angle_deg,
            distance_m,
            watch,
        )


def run_path_mode(
    link: EspPiControlLink | None,
    pose_tracker: PoseAccumulator,
    ir_bank: GpioIrBank,
    brain: SimpleCardinalRealBrain,
    args,
) -> None:
    print(f"Path mode. Directions: {' '.join(DIRECTION_ORDER)}", flush=True)
    print("Enter waypoints one per line. Blank line runs the path. q quits.", flush=True)
    waypoints: list[tuple[str, float, float, float]] = []
    while True:
        raw = input(f"wp {len(waypoints) + 1}> ").strip()
        if raw.lower() in ("q", "quit", "exit"):
            if link is not None:
                link.command_ack("STOP")
            return
        if not raw:
            break
        try:
            waypoints.append(parse_manual_waypoint(raw, args.heading))
        except ValueError as exc:
            print(f"input error | {exc}", flush=True)

    watch = ALL_WATCH_SENSORS if args.manual_watch_ir else set()
    for index, (direction, angle_deg, distance_m, heading) in enumerate(waypoints, start=1):
        args.heading = heading
        done = execute_segment(
            link,
            pose_tracker,
            ir_bank,
            brain,
            args,
            f"path {index}/{len(waypoints)} {direction}",
            angle_deg,
            distance_m,
            watch,
        )
        if done.get("result") != "completed":
            print(f"path stopped at waypoint {index}: result={done.get('result')}", flush=True)
            return


def run_ir_monitor(ir_bank: GpioIrBank, args) -> None:
    print("IR monitor mode. Ctrl-C exits.", flush=True)
    tracker = IrEdgeTracker()
    last_print = 0.0
    while True:
        state = ir_bank.read()
        rising, falling = tracker.update(state)
        now = time.monotonic()
        if rising or falling or now - last_print >= args.status_period:
            print(f"ir | {ir_summary(state)} rising={rising} falling={falling}", flush=True)
            log_event(args, "ir", state=state, rising=rising, falling=falling)
            last_print = now
        time.sleep(args.period)


def run_ir_train_mode(
    link: EspPiControlLink | None,
    pose_tracker: PoseAccumulator,
    ir_bank: GpioIrBank,
    brain: SimpleCardinalRealBrain,
    args,
) -> None:
    print("IR train mode. Robot waits still, then runs the obstacle state machine on rising edges.", flush=True)
    print("Front/FL trigger full front avoidance. Left/right trigger forward-until-falling-edge training. Ctrl-C exits.", flush=True)
    tracker = IrEdgeTracker()
    mission = MissionMemory()
    action_budget = [args.max_avoidance_actions]
    update_brain_from_pose(brain, pose_tracker, ir_bank, True)

    while True:
        state = ir_bank.read()
        rising, falling = tracker.update(state)
        if rising or falling:
            print(f"train_ir | {ir_summary(state)} rising={rising} falling={falling}", flush=True)
            log_event(args, "train_ir", state=state, rising=rising, falling=falling)

        front_hits = [name for name in ("front", "front_left", "front_right") if name in rising]
        side_hits = [name for name in ("left", "right") if name in rising]

        if front_hits:
            ir_state = {name: False for name in IR_SENSOR_ORDER}
            for name in front_hits:
                ir_state[name] = True
            print(f"train | front sequence from {front_hits}", flush=True)
            still_front, ir_state = wait_for_dynamic_front_clear(ir_bank, brain, pose_tracker, args, front_hits)
            if not still_front:
                print("train | dynamic obstacle cleared; no avoidance needed", flush=True)
                tracker = IrEdgeTracker()
                continue
            done = execute_front_avoidance(link, pose_tracker, ir_bank, brain, args, ir_state, action_budget, mission)
            print(f"train | sequence done result={done.get('result')}", flush=True)
            tracker = IrEdgeTracker()
            continue

        if side_hits:
            side_sensor = side_hits[0]
            print(f"train | side falling-edge sequence for {side_sensor}", flush=True)
            done = execute_forward_until_side_falling(
                link,
                pose_tracker,
                ir_bank,
                brain,
                args,
                side_sensor,
                action_budget,
                mission,
            )
            if done.get("result") != "ir_stop":
                execute_segment(
                    link,
                    pose_tracker,
                    ir_bank,
                    brain,
                    args,
                    "training side clear forward buffer",
                    0.0,
                    args.front_advance_distance,
                    ALL_WATCH_SENSORS,
                    mission,
                    move_timeout_s=args.front_advance_timeout,
                    timeout_returns_done=True,
                )
                execute_recenter(link, pose_tracker, ir_bank, brain, args, action_budget, mission)
            tracker = IrEdgeTracker()
            continue

        time.sleep(args.period)


def ir_summary(ir_state: dict[str, bool]) -> str:
    parts = []
    for name in IR_SENSOR_ORDER:
        if name not in IR_PINS:
            parts.append(f"{IR_LABELS[name]}=NA")
        else:
            parts.append(f"{IR_LABELS[name]}={1 if ir_state.get(name, False) else 0}")
    return " ".join(parts)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pi master: simple-cardinal brain + GPIO IR + ESP UART PID link.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Modes:
  mission     Prompt/run goal-distance mission with obstacle avoidance.
  move        Interactive single MOVE commands: <direction> <distance_cm> [heading_deg].
  path        Enter several waypoints, blank line runs them sequentially.
  ir-monitor  Print IR states plus rising/falling edges. Does not need ESP.
  ir-train    Stand still, then run avoidance/falling-edge training on IR triggers.

Directions for move/path:
  {' '.join(DIRECTION_ORDER)}

ESP commands used by this Pi file:
  PING, INIT_IMU, RESET_ODOM, RESET_ENC, MOVE, STOP.
  The Pi does not send wheel PWM or TWIST commands in these modes.

Logs:
  Each run writes events.jsonl and moves.csv under --log-dir unless --no-log is used.
""",
    )

    parser.add_argument("--mode", choices=("mission", "path", "move", "ir-monitor", "ir-train"), default="mission")
    parser.add_argument("--port", default=os.environ.get("PI_UART_PORT", "/dev/ttyAMA0"))
    parser.add_argument("--baud", type=int, default=ESP_DEFAULT_BAUD)
    parser.add_argument("--period", type=float, default=ESP_SERIAL_TELEMETRY_INTERVAL_S)
    parser.add_argument("--status-period", type=float, default=0.5)
    parser.add_argument("--telemetry-timeout", type=float, default=ESP_SERIAL_COMMAND_TIMEOUT_S)
    parser.add_argument("--move-timeout", type=float, default=ESP_POSITION_MOVE_TIMEOUT_S)
    parser.add_argument("--position-step", type=float, default=0.05)
    parser.add_argument("--min-position-step", type=float, default=0.005)
    parser.add_argument("--heading", type=float, default=0.0)
    parser.add_argument("--dry-run-step-delay", type=float, default=0.02)
    parser.add_argument("--auto-start", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--mock-ir", action="store_true")
    parser.add_argument("--ir-logic", choices=("baseline", "active-low", "active-high"), default="baseline")
    parser.add_argument("--active-low", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--pull-up", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--manual-watch-ir", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--buzzer", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--buzzer-pin", type=int, default=DEFAULT_BUZZER_PIN, help=f"Buzzer BCM GPIO pin; default {DEFAULT_BUZZER_PIN}.")
    parser.add_argument("--buzzer-active-high", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--lift-on-goal", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--goal-stepper-action", choices=("up", "down", "none"), default="up")
    parser.add_argument("--lift-steps", type=int, default=STEPPER_DEFAULT_STEPS)
    parser.add_argument("--lift-direction", type=int, choices=(STEPPER_DOWN_DIR, STEPPER_UP_DIR), default=None, help="Low-level override: 1 is up, -1 is down.")
    parser.add_argument("--stepper-speed-sps", type=float, default=STEPPER_SPEED_SPS)
    parser.add_argument("--prompt-goal", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--log-dir", default="run_logs")
    parser.add_argument("--log", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--verbose-serial", action="store_true")
    parser.add_argument("--reset-odom-on-start", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--reset-encoders-on-start", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--init-imu-on-start", action=argparse.BooleanOptionalAction, default=True)

    parser.add_argument("--goal-distance", type=float, default=1.20, help="Mission goal distance in meters.")
    parser.add_argument("--goal-distance-cm", type=float, default=None, help="Mission goal distance in centimeters; overrides --goal-distance.")
    parser.add_argument("--goal-tolerance", type=float, default=0.05)
    parser.add_argument("--line-kp", type=float, default=1.6)
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
    parser.add_argument("--front-strafe-distance", type=float, default=0.20)
    parser.add_argument("--front-corner-strafe-distance", type=float, default=0.15)
    parser.add_argument("--front-advance-distance", type=float, default=0.30)
    parser.add_argument("--front-strafe-search-distance", type=float, default=1.20)
    parser.add_argument("--front-corner-buffer-distance", type=float, default=0.05)
    parser.add_argument("--front-dynamic-hold", type=float, default=3.0)
    parser.add_argument("--front-strafe-search-timeout", type=float, default=8.0)
    parser.add_argument("--front-corner-buffer-timeout", type=float, default=1.25)
    parser.add_argument("--front-advance-timeout", type=float, default=4.0)
    parser.add_argument("--side-follow-search-distance", type=float, default=3.00)
    parser.add_argument("--side-follow-dry-distance", type=float, default=0.30)
    parser.add_argument("--side-follow-watch-front", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--side-escape-distance", type=float, default=0.10)
    parser.add_argument("--side-escape-forward-distance", type=float, default=0.20)
    parser.add_argument("--max-recenter-attempts", type=int, default=8)
    parser.add_argument("--max-goal-correction-attempts", type=int, default=4)
    parser.add_argument("--max-avoidance-actions", type=int, default=24)

    args = parser.parse_args()
    if args.goal_distance_cm is not None:
        args.goal_distance = args.goal_distance_cm / 100.0
    args.forward_weight = 1.0
    args.lateral_weight = 1.0
    args.backoff_weight = 1.0
    args.max_line_correction_weight = 1.0
    if args.goal_distance <= 0.0:
        parser.error("--goal-distance must be greater than zero")
    if args.period <= 0.0:
        parser.error("--period must be greater than zero")
    if args.status_period <= 0.0:
        parser.error("--status-period must be greater than zero")
    if args.telemetry_timeout <= 0.0:
        parser.error("--telemetry-timeout must be greater than zero")
    if args.move_timeout <= 0.0:
        parser.error("--move-timeout must be greater than zero")
    if args.position_step <= 0.0:
        parser.error("--position-step must be greater than zero")
    if args.min_position_step <= 0.0:
        parser.error("--min-position-step must be greater than zero")
    if args.lift_steps < 0:
        parser.error("--lift-steps cannot be negative")
    if args.stepper_speed_sps <= 0.0:
        parser.error("--stepper-speed-sps must be greater than zero")
    if args.front_strafe_distance <= 0.0:
        parser.error("--front-strafe-distance must be greater than zero")
    if args.front_corner_strafe_distance <= 0.0:
        parser.error("--front-corner-strafe-distance must be greater than zero")
    if args.front_advance_distance <= 0.0:
        parser.error("--front-advance-distance must be greater than zero")
    if args.front_strafe_search_distance <= 0.0:
        parser.error("--front-strafe-search-distance must be greater than zero")
    if args.front_corner_buffer_distance <= 0.0:
        parser.error("--front-corner-buffer-distance must be greater than zero")
    if args.front_dynamic_hold < 0.0:
        parser.error("--front-dynamic-hold cannot be negative")
    if args.front_strafe_search_timeout <= 0.0:
        parser.error("--front-strafe-search-timeout must be greater than zero")
    if args.front_corner_buffer_timeout <= 0.0:
        parser.error("--front-corner-buffer-timeout must be greater than zero")
    if args.front_advance_timeout <= 0.0:
        parser.error("--front-advance-timeout must be greater than zero")
    if args.side_follow_search_distance <= 0.0:
        parser.error("--side-follow-search-distance must be greater than zero")
    if args.side_follow_dry_distance <= 0.0:
        parser.error("--side-follow-dry-distance must be greater than zero")
    if args.side_escape_distance <= 0.0:
        parser.error("--side-escape-distance must be greater than zero")
    if args.side_escape_forward_distance <= 0.0:
        parser.error("--side-escape-forward-distance must be greater than zero")
    if args.max_recenter_attempts <= 0:
        parser.error("--max-recenter-attempts must be greater than zero")
    if args.max_goal_correction_attempts <= 0:
        parser.error("--max-goal-correction-attempts must be greater than zero")
    if args.max_avoidance_actions <= 0:
        parser.error("--max-avoidance-actions must be greater than zero")
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


def prompt_goal_if_needed(args) -> None:
    if args.mode != "mission" or args.auto_start or not args.prompt_goal:
        return
    raw = input(f"Goal distance cm [{args.goal_distance * 100.0:.1f}]: ").strip()
    if not raw:
        pass
    else:
        value_cm = float(raw)
        if value_cm <= 0.0:
            raise ValueError("Goal distance must be greater than zero")
        args.goal_distance = value_cm / 100.0

    raw = input(f"End stepper action up/down/none [{args.goal_stepper_action}]: ").strip().lower()
    if raw:
        if raw not in {"up", "down", "none"}:
            raise ValueError("End stepper action must be up, down, or none")
        args.goal_stepper_action = raw

    if args.goal_stepper_action != "none":
        raw = input(f"End stepper steps [{args.lift_steps}]: ").strip()
        if raw:
            steps = int(raw)
            if steps < 0:
                raise ValueError("End stepper steps cannot be negative")
            args.lift_steps = steps


def main() -> None:
    args = parse_args()
    args.logger = RunLogger(args.mode, args.log_dir) if args.log else NullLogger()
    if getattr(args.logger, "dir", None) is not None:
        print(f"log | {args.logger.dir}", flush=True)

    ir_bank = GpioIrBank(
        active_low=args.active_low,
        pull_up=args.pull_up,
        mock=args.mock_ir or args.dry_run,
        logic=args.ir_logic,
    )
    args.buzzer = GpioBuzzer(
        pin=args.buzzer_pin,
        active_high=args.buzzer_active_high,
        enabled=args.buzzer,
        mock=args.dry_run,
    )
    args.stepper_lift = GpioStepperLift(
        step_pin=STEPPER_STEP_PIN,
        dir_pin=STEPPER_DIR_PIN,
        en_pin=STEPPER_EN_PIN,
        speed_sps=args.stepper_speed_sps,
        step_high_us=STEPPER_STEP_HIGH_US,
        enabled=args.mode == "mission" and args.lift_on_goal,
        mock=args.dry_run,
    )
    brain = SimpleCardinalRealBrain(args)
    link = None
    pose_tracker = PoseAccumulator()

    try:
        needs_link = args.mode != "ir-monitor"
        if not args.dry_run and needs_link:
            link = EspPiControlLink(args.port, args.baud, verbose_serial=args.verbose_serial)
            initialize_link(link, args)

        if args.mode == "ir-monitor":
            run_ir_monitor(ir_bank, args)
            return

        if args.mode == "move":
            run_move_mode(link, pose_tracker, ir_bank, brain, args)
            return

        if args.mode == "path":
            run_path_mode(link, pose_tracker, ir_bank, brain, args)
            return

        if args.mode == "ir-train":
            if not args.auto_start:
                print("Ready for IR training. Press Enter to arm. Ctrl-C stops.", flush=True)
                input()
            run_ir_train_mode(link, pose_tracker, ir_bank, brain, args)
            return

        prompt_goal_if_needed(args)
        if not args.auto_start:
            print("Ready. Put the robot in a clear area, then press Enter to start. Ctrl-C stops.", flush=True)
            input()

        print(
            f"Pi brain running to {args.goal_distance * 100.0:.1f}cm. "
            "ESP owns position PID; Pi sends planned full MOVE segments.",
            flush=True,
        )
        run_segment_mission(link, pose_tracker, ir_bank, brain, args)

    except KeyboardInterrupt:
        print("\nStopping.", flush=True)
    except Exception as exc:
        print(f"error | {exc}", flush=True)
        if link is not None:
            try:
                link.send("STOP")
            except Exception:
                pass
    finally:
        if link is not None:
            try:
                link.send("STOP")
            except Exception:
                pass
            link.close()
        args.stepper_lift.close()
        args.buzzer.close()
        ir_bank.close()
        args.logger.close()


if __name__ == "__main__":
    main()
