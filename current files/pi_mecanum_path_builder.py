#!/usr/bin/env python3
"""
Interactive Raspberry Pi position commander for esp_pid.ino.

Use pi_master.py for the IR avoidance brain. This file is for direct position
tests and hand-entered paths; it sends only MOVE, TURN, STATUS, and STOP.
"""

import argparse
import importlib.util
import json
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass


SYSTEM_PYTHON = "/usr/bin/python3"
REEXEC_GUARD = "PI_PATH_SYSTEM_PYTHON"

ESP_DEFAULT_PORT = "/dev/ttyAMA0"
ESP_DEFAULT_BAUD = 115200
ESP_READ_TIMEOUT_S = 0.05
ESP_ACK_TIMEOUT_S = 3.0
ESP_PING_TIMEOUT_S = 2.5
ESP_INIT_TIMEOUT_S = 8.0
ESP_MOVE_DONE_TIMEOUT_S = 190.0

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


@dataclass(frozen=True)
class Waypoint:
    direction: str
    angle_deg: float
    distance_cm: float
    heading_deg: float


@dataclass
class Event:
    raw: str
    data: dict | None


def module_exists(name: str) -> bool:
    try:
        return importlib.util.find_spec(name) is not None
    except ModuleNotFoundError:
        return False


def maybe_reexec_with_system_python() -> None:
    if (
        os.path.exists(SYSTEM_PYTHON)
        and os.environ.get(REEXEC_GUARD) != "1"
        and os.path.abspath(sys.executable) != SYSTEM_PYTHON
    ):
        os.environ[REEXEC_GUARD] = "1"
        target = os.path.abspath(sys.argv[0]) if sys.argv and os.path.exists(sys.argv[0]) else os.path.abspath(__file__)
        os.execv(SYSTEM_PYTHON, [SYSTEM_PYTHON, target, *sys.argv[1:]])


def ensure_serial_runtime() -> None:
    if module_exists("serial"):
        return
    maybe_reexec_with_system_python()
    raise SystemExit("pyserial is not installed. Install python3-serial or pyserial.")


class EspPiControlLink:
    def __init__(self, port: str, baud: int, *, dry_run: bool = False) -> None:
        self.port = port
        self.baud = baud
        self.dry_run = dry_run
        self.seq = 1
        self.events: queue.Queue[Event] = queue.Queue()
        self.stop_event = threading.Event()
        self.send_lock = threading.Lock()
        self.telemetry_lock = threading.Lock()
        self.last_telemetry: dict | None = None
        self.last_telemetry_time = 0.0
        self.ser = None
        self.reader = None

        if dry_run:
            return

        ensure_serial_runtime()
        import serial

        self.ser = serial.Serial(port=port, baudrate=baud, timeout=ESP_READ_TIMEOUT_S)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()

    def close(self) -> None:
        self.stop_event.set()
        if self.reader is not None:
            self.reader.join(timeout=1.0)
        if self.ser is not None:
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
        clean = line.strip()
        if self.dry_run:
            print(f"dry-send | {clean}")
            return
        payload = (clean + "\n").encode("utf-8")
        with self.send_lock:
            self.ser.write(payload)
            self.ser.flush()

    def send_command(self, line: str) -> int:
        seq = self.next_seq()
        if " seq=" not in line:
            line = f"{line} seq={seq}"
        self.send(line)
        return seq

    def wait_for(self, seq: int | None, wanted_types: set[str], timeout: float) -> dict:
        if self.dry_run:
            kind = next(iter(wanted_types))
            return {"type": kind, "seq": seq, "ok": True, "result": "completed", "message": "dry_run"}

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
            if data.get("type") in wanted_types and (seq is None or data.get("seq") == seq):
                return data

        raise TimeoutError(f"Timed out waiting for {wanted_types} seq={seq}")

    def command_ack(self, line: str, timeout: float = ESP_ACK_TIMEOUT_S) -> dict:
        seq = self.send_command(line)
        ack = self.wait_for(seq, {"ack", "pong"}, timeout)
        if ack.get("type") == "ack" and not ack.get("ok", False):
            raise RuntimeError(f"{ack.get('cmd', line)} rejected: {ack.get('message', 'no message')}")
        return ack

    def wait_for_done(self, seq: int, timeout: float = ESP_MOVE_DONE_TIMEOUT_S) -> dict:
        return self.wait_for(seq, {"done"}, timeout)

    def latest_telemetry(self) -> tuple[dict | None, float]:
        if self.dry_run:
            return {
                "type": "telemetry",
                "mode": "dry-run",
                "pose": {"forwardCm": 0.0, "strafeCm": 0.0, "yawDeg": 0.0},
                "imu": {"ok": True, "yawDeg": 0.0},
                "signedEncoderCounts": [],
            }, time.monotonic()
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


def telemetry_summary(data: dict) -> str:
    pose = data.get("pose") or data.get("odometry") or {}
    imu = data.get("imu") or {}
    enc = data.get("signedEncoderCounts", data.get("encoderCounts", []))
    return (
        "telemetry | "
        f"mode={data.get('mode', 'unknown')} "
        f"pose_cm(f={float(pose.get('forwardCm', 0.0)):.1f},"
        f"s={float(pose.get('strafeCm', 0.0)):.1f},"
        f"yaw={float(pose.get('yawDeg', imu.get('yawDeg', 0.0))):.1f}) "
        f"imu_ok={imu.get('ok', '?')} "
        f"enc={enc}"
    )


def initialize_robot(link: EspPiControlLink, args) -> None:
    print(f"opened {args.port} @ {args.baud}")
    try:
        pong = link.command_ack("PING", timeout=ESP_PING_TIMEOUT_S)
        print(f"link ok | pong seq={pong.get('seq')}")
    except Exception:
        print("link wait | no PING yet, waiting for passive telemetry")
        link.wait_for_telemetry(ESP_PING_TIMEOUT_S)

    if args.init_imu:
        print("init | IMU calibrate + zero")
        link.command_ack("INIT_IMU", timeout=ESP_INIT_TIMEOUT_S)
    if args.reset_encoders:
        print("init | reset encoders")
        link.command_ack("RESET_ENC")
    if args.reset_odom:
        print("init | reset odometry")
        link.command_ack("RESET_ODOM")


def request_status(link: EspPiControlLink) -> dict:
    link.command_ack("STATUS")
    data = link.wait_for_telemetry(ESP_ACK_TIMEOUT_S)
    print(telemetry_summary(data))
    return data


def send_stop(link: EspPiControlLink) -> None:
    ack = link.command_ack("STOP")
    print(f"stop | {ack.get('message', 'sent')}")


def move_command(angle_deg: float, distance_cm: float, heading_deg: float, timeout_s: float) -> str:
    return (
        f"MOVE angle={angle_deg:.3f} "
        f"dist={distance_cm:.3f} "
        f"heading={heading_deg:.3f} "
        f"timeout={int(timeout_s * 1000)}"
    )


def turn_command(angle_deg: float, timeout_s: float) -> str:
    return f"TURN angle={angle_deg:.3f} timeout={int(timeout_s * 1000)}"


def execute_position_line(link: EspPiControlLink, line: str, timeout_s: float) -> dict:
    print(f"send | {line}")
    seq = link.send_command(line)
    ack = link.wait_for(seq, {"ack"}, timeout=ESP_ACK_TIMEOUT_S)
    if not ack.get("ok", False):
        raise RuntimeError(f"{ack.get('cmd', line)} rejected: {ack.get('message', 'no message')}")
    print(f"ack  | {ack.get('message', 'accepted')}")
    done = link.wait_for_done(seq, timeout=timeout_s)
    print(
        "done | "
        f"result={done.get('result')} "
        f"forward={float(done.get('forwardCm', 0.0)):.1f}cm "
        f"strafe={float(done.get('strafeCm', 0.0)):.1f}cm "
        f"heading={float(done.get('headingDeg', 0.0)):.1f}deg"
    )
    return done


def parse_waypoint(raw: str, default_heading_deg: float) -> Waypoint:
    parts = raw.split()
    if len(parts) not in (2, 3):
        raise ValueError("use: <direction> <distance_cm> [heading_deg]")

    direction = parts[0].upper()
    if direction not in DIRECTION_ANGLES_DEG:
        raise ValueError(f"direction must be one of: {' '.join(DIRECTION_ORDER)}")

    distance_cm = float(parts[1])
    heading_deg = float(parts[2]) if len(parts) == 3 else default_heading_deg
    if distance_cm <= 0.0:
        raise ValueError("distance must be greater than 0 cm")

    return Waypoint(direction, DIRECTION_ANGLES_DEG[direction], distance_cm, heading_deg)


def prompt_float(prompt: str, default: float | None = None) -> float:
    suffix = f" [{default}]" if default is not None else ""
    while True:
        raw = input(f"{prompt}{suffix}: ").strip()
        if not raw and default is not None:
            return float(default)
        try:
            return float(raw)
        except ValueError:
            print("input error | enter a number")


def prompt_int(prompt: str) -> int:
    while True:
        raw = input(f"{prompt}: ").strip()
        try:
            value = int(raw)
        except ValueError:
            print("input error | enter a whole number")
            continue
        if value > 0:
            return value
        print("input error | value must be greater than 0")


def prompt_waypoints(args) -> list[Waypoint]:
    count = prompt_int("Number of waypoints")
    print(f"Directions: {' '.join(DIRECTION_ORDER)}")
    print("Waypoint format: <direction> <distance_cm> [heading_deg]")
    waypoints = []
    for index in range(1, count + 1):
        while True:
            raw = input(f"Waypoint {index}/{count}: ").strip()
            try:
                waypoints.append(parse_waypoint(raw, args.heading))
                break
            except ValueError as exc:
                print(f"input error | {exc}")
    return waypoints


def run_path(link: EspPiControlLink, args) -> None:
    waypoints = prompt_waypoints(args)
    for index, waypoint in enumerate(waypoints, start=1):
        print(
            f"path {index}/{len(waypoints)} | {waypoint.direction} "
            f"{waypoint.distance_cm:.1f}cm heading={waypoint.heading_deg:.1f}"
        )
        line = move_command(
            waypoint.angle_deg,
            waypoint.distance_cm,
            waypoint.heading_deg,
            args.move_timeout,
        )
        done = execute_position_line(link, line, args.move_timeout + 5.0)
        if done.get("result") != "completed":
            raise RuntimeError(f"path stopped at waypoint {index}: result={done.get('result')}")


def run_single_move(link: EspPiControlLink, args) -> None:
    print(f"Directions: {' '.join(DIRECTION_ORDER)}")
    raw = input("Move command <direction> <distance_cm> [heading_deg]: ").strip()
    waypoint = parse_waypoint(raw, args.heading)
    line = move_command(waypoint.angle_deg, waypoint.distance_cm, waypoint.heading_deg, args.move_timeout)
    execute_position_line(link, line, args.move_timeout + 5.0)


def run_turn(link: EspPiControlLink, args) -> None:
    angle = prompt_float("Heading target deg", args.heading)
    line = turn_command(angle, args.move_timeout)
    execute_position_line(link, line, args.move_timeout + 5.0)


def menu_loop(link: EspPiControlLink, args) -> None:
    while True:
        print()
        print("1 path   2 move   3 turn   4 status   5 stop   q quit")
        choice = input("> ").strip().lower()
        if choice in ("q", "quit", "exit"):
            send_stop(link)
            return
        if choice in ("1", "path"):
            run_path(link, args)
        elif choice in ("2", "move"):
            run_single_move(link, args)
        elif choice in ("3", "turn"):
            run_turn(link, args)
        elif choice in ("4", "status"):
            request_status(link)
        elif choice in ("5", "stop"):
            send_stop(link)
        else:
            print("input error | choose 1, 2, 3, 4, 5, or q")


def parse_args():
    parser = argparse.ArgumentParser(description="Position-only path commander for esp_pid.ino.")
    parser.add_argument("--port", default=os.environ.get("PI_UART_PORT", ESP_DEFAULT_PORT))
    parser.add_argument("--baud", type=int, default=ESP_DEFAULT_BAUD)
    parser.add_argument("--heading", type=float, default=0.0)
    parser.add_argument("--move-timeout", type=float, default=ESP_MOVE_DONE_TIMEOUT_S)
    parser.add_argument("--init-imu", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--reset-odom", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--reset-encoders", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if args.move_timeout <= 0.0:
        parser.error("--move-timeout must be greater than zero")
    return args


def main() -> None:
    args = parse_args()
    link: EspPiControlLink | None = None
    try:
        link = EspPiControlLink(args.port, args.baud, dry_run=args.dry_run)
        initialize_robot(link, args)
        request_status(link)
        menu_loop(link, args)
    except KeyboardInterrupt:
        print("\ninterrupted | sending STOP")
        if link is not None:
            try:
                send_stop(link)
            except Exception as exc:
                print(f"stop warning | {exc}")
    except (RuntimeError, TimeoutError) as exc:
        print(f"error | {exc}")
        if link is not None:
            try:
                send_stop(link)
            except Exception as stop_exc:
                print(f"stop warning | {stop_exc}")
    finally:
        if link is not None:
            link.close()


if __name__ == "__main__":
    main()
