#!/usr/bin/env python3

import importlib.util
import json
import os
import queue
import threading
import time
from dataclasses import dataclass

import serial


SYSTEM_PYTHON = "/usr/bin/python3"
REEXEC_GUARD = "PI_TESTING_SYSTEM_PYTHON"


def module_exists(name: str) -> bool:
    try:
        return importlib.util.find_spec(name) is not None
    except ModuleNotFoundError:
        return False


def reexec_target() -> str:
    argv0 = os.sys.argv[0] if os.sys.argv else ""
    if argv0 and argv0 != "-" and os.path.exists(argv0):
        return os.path.abspath(argv0)
    return os.path.abspath(__file__)


def ensure_gpiozero_runtime() -> None:
    has_gpiozero = module_exists("gpiozero")
    has_backend = any(module_exists(name) for name in ("lgpio", "RPi.GPIO", "pigpio"))
    if has_gpiozero and has_backend:
        return

    if (
        os.path.exists(SYSTEM_PYTHON)
        and os.environ.get(REEXEC_GUARD) != "1"
        and os.path.abspath(os.sys.executable) != SYSTEM_PYTHON
    ):
        os.environ[REEXEC_GUARD] = "1"
        os.execv(SYSTEM_PYTHON, [SYSTEM_PYTHON, reexec_target(), *os.sys.argv[1:]])

    if not has_gpiozero:
        raise SystemExit(
            "gpiozero is not installed for this Python interpreter.\n"
            f"Run this script with {SYSTEM_PYTHON} or install python3-gpiozero."
        )

    raise SystemExit(
        "gpiozero is installed, but no supported GPIO pin backend is available for this Python interpreter.\n"
        f"Run this script with {SYSTEM_PYTHON} or install lgpio/pigpio/RPi.GPIO for {os.sys.executable}."
    )


ensure_gpiozero_runtime()

from gpiozero import DigitalInputDevice, DigitalOutputDevice


PORT = os.environ.get("PI_UART_PORT", "/dev/ttyAMA0")
BAUD = 115200
READ_TIMEOUT = 0.1

DEFAULT_TURN_SPEED_RPM = 12.0
PING_TIMEOUT_S = 2.5
INIT_TIMEOUT_S = 8.0
ACK_TIMEOUT_S = 3.0
MOVE_DONE_TIMEOUT_S = 180.0
TURN_DONE_TIMEOUT_S = 120.0
PROGRESS_PRINT_INTERVAL_S = 0.35

IR_PINS = [23, 24, 25, 17, 27, 22]
IR_POLL_INTERVAL_S = 0.05

STEP_PIN = 6
DIR_PIN = 13
EN_PIN = 5
BUZZER_PIN = 19
YELLOW_LED_PIN = 20
GREEN_LED_PIN = 16
RED_LED_PIN = 21
STEP_HIGH_US = 10
STEPPER_SPEED_SPS = 275.0
STEPPER_DEFAULT_STEPS = 200
STEPPER_DEFAULT_DIRECTION = 1  # +1 is lift up; -1 is lift down.
STEPPER_DIR_SETUP_S = 0.010


@dataclass
class Event:
    raw: str
    data: dict | None


class EspPiControlLink:
    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.seq = 1
        self.events: queue.Queue[Event] = queue.Queue()
        self.stop_event = threading.Event()
        self.last_telemetry: dict | None = None
        self.send_lock = threading.Lock()
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=READ_TIMEOUT)
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
            except serial.SerialException:
                return

            if not line:
                continue

            raw = line.decode("utf-8", errors="replace").strip()
            if not raw:
                continue

            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                json_start = raw.find("{")
                if json_start >= 0:
                    candidate = raw[json_start:]
                    try:
                        data = json.loads(candidate)
                    except json.JSONDecodeError:
                        self.events.put(Event(raw=raw, data=None))
                    else:
                        self.events.put(Event(raw=candidate, data=data))
                    continue
                self.events.put(Event(raw=raw, data=None))
                continue

            self.events.put(Event(raw=raw, data=data))

    def next_seq(self) -> int:
        value = self.seq
        self.seq += 1
        return value

    def drain(self, seconds: float = 0.8) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            try:
                event = self.events.get(timeout=0.1)
            except queue.Empty:
                return

            if event.data and event.data.get("type") == "telemetry":
                self.last_telemetry = event.data

    def send(self, line: str) -> None:
        payload = (line.strip() + "\n").encode("utf-8")
        self.ser.write(payload)
        self.ser.flush()

    def send_command(self, line: str) -> int:
        with self.send_lock:
            seq = self.next_seq()
            if " seq=" not in line:
                line = f"{line} seq={seq}"
            self.send(line)
            return seq

    def wait_for(self, seq: int | None, wanted_types: set[str], timeout: float, *, match_seq: bool = True) -> dict:
        deadline = time.time() + timeout
        while time.time() < deadline:
            remaining = deadline - time.time()
            try:
                event = self.events.get(timeout=max(0.05, remaining))
            except queue.Empty:
                continue

            if event.data is None:
                continue

            data = event.data
            event_type = data.get("type")
            if event_type == "telemetry":
                self.last_telemetry = data

            seq_matches = (seq is None) or (data.get("seq") == seq)
            if event_type in wanted_types and (seq_matches or not match_seq):
                return data

        raise TimeoutError(f"Timed out waiting for {wanted_types} for seq {seq}")

    def wait_for_done(self, seq: int, timeout: float) -> dict:
        deadline = time.time() + timeout
        last_progress_print = 0.0

        while time.time() < deadline:
            remaining = deadline - time.time()
            try:
                event = self.events.get(timeout=max(0.05, remaining))
            except queue.Empty:
                continue

            if event.data is None:
                continue

            data = event.data
            event_type = data.get("type")

            if event_type == "telemetry":
                self.last_telemetry = data
                now = time.time()
                if now - last_progress_print >= PROGRESS_PRINT_INTERVAL_S:
                    print(progress_summary(data))
                    last_progress_print = now
                continue

            if event_type == "done" and data.get("seq") == seq:
                return data

        raise TimeoutError(f"Timed out waiting for done for seq {seq}")


class StepperController:
    def __init__(
        self,
        step_pin: int,
        dir_pin: int,
        en_pin: int,
        *,
        on_motion_start=None,
        on_motion_end=None,
    ) -> None:
        self.step_pin = DigitalOutputDevice(step_pin, initial_value=False)
        self.dir_pin = DigitalOutputDevice(dir_pin, initial_value=False)
        self.en_pin = DigitalOutputDevice(en_pin, initial_value=True)
        self.steps = STEPPER_DEFAULT_STEPS
        self.direction = STEPPER_DEFAULT_DIRECTION
        self.lock = threading.Lock()
        self.on_motion_start = on_motion_start
        self.on_motion_end = on_motion_end

    def close(self) -> None:
        self.step_pin.off()
        self.dir_pin.off()
        self.en_pin.on()

    def set_steps(self, steps: int) -> None:
        with self.lock:
            self.steps = max(0, steps)

    def set_direction(self, direction: int) -> None:
        with self.lock:
            self.direction = 1 if direction >= 0 else -1

    def snapshot(self) -> tuple[int, int]:
        with self.lock:
            return self.steps, self.direction

    def status_text(self) -> str:
        steps, direction = self.snapshot()
        return f"stepper | steps={steps} direction={direction} speed={STEPPER_SPEED_SPS:.1f} steps/s"

    def _begin_motion(self, direction_sign: int) -> None:
        with self.lock:
            if direction_sign > 0:
                self.dir_pin.on()
            else:
                self.dir_pin.off()
            self.en_pin.off()
            time.sleep(STEPPER_DIR_SETUP_S)

        if self.on_motion_start is not None:
            self.on_motion_start()

    def _end_motion(self) -> None:
        self.step_pin.off()
        self.en_pin.on()
        if self.on_motion_end is not None:
            self.on_motion_end()

    def run_configured(self, abort_event: threading.Event) -> bool:
        steps, direction = self.snapshot()
        return self.run_steps(steps, direction, abort_event)

    def run_steps(self, steps: int, direction: int, abort_event: threading.Event, *, announce: bool = True) -> bool:
        if steps <= 0:
            if announce:
                print("stepper | skipped because configured steps=0")
            return True

        direction_sign = 1 if direction >= 0 else -1
        interval_us = int(1_000_000.0 / STEPPER_SPEED_SPS)
        interval_us = max(interval_us, STEP_HIGH_US + 50)

        self._begin_motion(direction_sign)

        try:
            for step_index in range(steps):
                if abort_event.is_set():
                    print(f"stepper | aborted at step {step_index} because IR stop latched")
                    return False

                self.step_pin.on()
                time.sleep(STEP_HIGH_US / 1_000_000.0)
                self.step_pin.off()
                time.sleep(max((interval_us - STEP_HIGH_US) / 1_000_000.0, 0.0))
        finally:
            self._end_motion()

        if announce:
            print(f"stepper | completed steps={steps} direction={direction_sign}")
        return True

    def run_until_released(
        self,
        direction: int,
        stop_event: threading.Event,
        abort_event: threading.Event,
    ) -> bool:
        direction_sign = 1 if direction >= 0 else -1
        interval_us = int(1_000_000.0 / STEPPER_SPEED_SPS)
        interval_us = max(interval_us, STEP_HIGH_US + 50)

        self._begin_motion(direction_sign)

        try:
            step_index = 0
            while not stop_event.is_set():
                if abort_event.is_set():
                    print(f"stepper | aborted at step {step_index} because IR stop latched")
                    return False

                self.step_pin.on()
                time.sleep(STEP_HIGH_US / 1_000_000.0)
                self.step_pin.off()
                time.sleep(max((interval_us - STEP_HIGH_US) / 1_000_000.0, 0.0))
                step_index += 1
        finally:
            self._end_motion()

        return True


class PiIndicators:
    def __init__(self, yellow_pin: int, red_pin: int, green_pin: int, buzzer_pin: int) -> None:
        self.yellow = DigitalOutputDevice(yellow_pin, initial_value=False)
        self.red = DigitalOutputDevice(red_pin, initial_value=False)
        self.green = DigitalOutputDevice(green_pin, initial_value=False)
        self.buzzer = DigitalOutputDevice(buzzer_pin, initial_value=False)
        self.lock = threading.Lock()

    def close(self) -> None:
        with self.lock:
            self.yellow.off()
            self.red.off()
            self.green.off()
            self.buzzer.off()

    def set_lights(self, *, yellow: bool | None = None, red: bool | None = None, green: bool | None = None) -> None:
        with self.lock:
            if yellow is not None:
                if yellow:
                    self.yellow.on()
                else:
                    self.yellow.off()
            if red is not None:
                if red:
                    self.red.on()
                else:
                    self.red.off()
            if green is not None:
                if green:
                    self.green.on()
                else:
                    self.green.off()

    def set_mecanum_active(self, active: bool) -> None:
        self.set_lights(yellow=active)

    def set_stepper_active(self, active: bool) -> None:
        self.set_lights(red=active)

    def set_green(self, active: bool) -> None:
        self.set_lights(green=active)

    def set_buzzer(self, active: bool) -> None:
        with self.lock:
            if active:
                self.buzzer.on()
            else:
                self.buzzer.off()

    def status_text(self) -> str:
        with self.lock:
            yellow = 1 if self.yellow.value else 0
            red = 1 if self.red.value else 0
            green = 1 if self.green.value else 0
            buzzer = 1 if self.buzzer.value else 0
        return f"indicators | yellow={yellow} red={red} green={green} buzzer={buzzer}"


class DisabledIRSafetyMonitor:
    def close(self) -> None:
        return

    def clear_latch(self) -> bool:
        return True

    def snapshot(self) -> tuple[list[int], list[int], bool]:
        return [], [], False

    def active_sensors(self) -> list[int]:
        return []

    def status_text(self) -> str:
        return "ir | disabled"


class IRSafetyMonitor:
    def __init__(self, pins: list[int], on_trigger) -> None:
        self.pins = pins
        self.on_trigger = on_trigger
        self.sensors = [DigitalInputDevice(pin, pull_up=False) for pin in pins]
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.baseline = [1 if sensor.value else 0 for sensor in self.sensors]
        self.current = self.baseline[:]
        self.active = []
        self.latched = False
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=1.0)

    def _loop(self) -> None:
        while not self.stop_event.is_set():
            readings = [1 if sensor.value else 0 for sensor in self.sensors]
            active = [index + 1 for index, value in enumerate(readings) if value != self.baseline[index]]

            callback_needed = False
            callback_active: list[int] = []
            with self.lock:
                self.current = readings
                self.active = active
                if active and not self.latched:
                    self.latched = True
                    callback_needed = True
                    callback_active = active[:]

            if callback_needed:
                self.on_trigger(callback_active)

            time.sleep(IR_POLL_INTERVAL_S)

    def clear_latch(self) -> bool:
        with self.lock:
            if self.active:
                return False
            self.latched = False
            return True

    def snapshot(self) -> tuple[list[int], list[int], bool]:
        with self.lock:
            return self.baseline[:], self.current[:], self.latched

    def active_sensors(self) -> list[int]:
        with self.lock:
            return self.active[:]

    def status_text(self) -> str:
        baseline, current, latched = self.snapshot()
        active = self.active_sensors()
        return (
            "ir | "
            f"baseline={baseline} "
            f"current={current} "
            f"active={active if active else 'none'} "
            f"latched={'YES' if latched else 'NO'}"
        )


class MecanumIrStepperSupervisor:
    def __init__(self, link: EspPiControlLink, *, ir_enabled: bool = True) -> None:
        self.link = link
        self.ir_enabled = ir_enabled
        self.ir_stop_latched = threading.Event()
        self.indicators = PiIndicators(YELLOW_LED_PIN, RED_LED_PIN, GREEN_LED_PIN, BUZZER_PIN)
        self.stepper = StepperController(
            STEP_PIN,
            DIR_PIN,
            EN_PIN,
            on_motion_start=self._on_stepper_start,
            on_motion_end=self._on_stepper_end,
        )
        if ir_enabled:
            self.ir_monitor = IRSafetyMonitor(IR_PINS, self._on_ir_trigger)
        else:
            self.ir_monitor = DisabledIRSafetyMonitor()

    def close(self) -> None:
        self.ir_monitor.close()
        self.stepper.close()
        self.indicators.close()

    def _on_stepper_start(self) -> None:
        self.indicators.set_stepper_active(True)

    def _on_stepper_end(self) -> None:
        self.indicators.set_stepper_active(False)

    def set_mecanum_active(self, active: bool) -> None:
        self.indicators.set_mecanum_active(active)

    def _on_ir_trigger(self, active_sensors: list[int]) -> None:
        self.ir_stop_latched.set()
        self.indicators.set_buzzer(True)
        print(f"\nIR STOP | sensors={active_sensors} changed from startup baseline")
        try:
            stop_seq = self.link.send_command("STOP")
            print(f"IR STOP | sent STOP seq={stop_seq}")
        except Exception as exc:
            print(f"IR STOP | failed to send STOP: {exc}")

    def motion_allowed(self) -> bool:
        return not self.ir_stop_latched.is_set()

    def clear_latch(self) -> None:
        if self.ir_monitor.clear_latch():
            self.ir_stop_latched.clear()
            self.indicators.set_buzzer(False)
            print("clear | IR stop latch cleared")
        else:
            print("clear | cannot clear because one or more IR sensors are still active")

    def ensure_motion_allowed(self) -> None:
        if self.ir_stop_latched.is_set():
            raise RuntimeError("IR stop is latched. Type 'clear' after the sensors are back to baseline.")

    def run_stepper_after_move(self) -> None:
        print(self.stepper.status_text())
        self.stepper.run_configured(self.ir_stop_latched)

    def safety_summary(self) -> str:
        return (
            f"{self.ir_monitor.status_text()} | "
            f"{self.stepper.status_text()} | "
            f"{self.indicators.status_text()}"
        )


def require_ok_ack(data: dict) -> None:
    if data.get("ok", False):
        return

    cmd = data.get("cmd", "UNKNOWN")
    message = data.get("message", "command rejected")
    raise RuntimeError(f"{cmd} rejected: {message}")


def try_ack(link: EspPiControlLink, line: str, timeout: float) -> dict | None:
    seq = link.send_command(line)
    try:
        return link.wait_for(seq, {"ack"}, timeout=timeout)
    except TimeoutError:
        return None


def move_command(direction: str, heading_deg: float, distance_cm: float, speed_rpm: float) -> str:
    direction_key = direction.strip().lower()
    if direction_key in {"forward", "fwd", "f"}:
        angle_deg = 0.0
    elif direction_key in {"back", "backward", "b"}:
        angle_deg = 180.0
    else:
        raise ValueError("Direction must be forward or back")

    return f"MOVE angle={angle_deg} dist={abs(distance_cm)} speed={speed_rpm} heading={heading_deg}"


def turn_command(heading_deg: float, speed_rpm: float) -> str:
    return f"TURN heading={heading_deg} speed={speed_rpm}"


def telemetry_summary(data: dict) -> str:
    pose = data.get("pose", {})
    imu = data.get("imu", {})
    move = data.get("move", {})
    return (
        "status | "
        f"mode={data.get('mode')} "
        f"yaw={imu.get('yawDeg', 0):.2f}deg "
        f"fwd={pose.get('forwardCm', 0):.2f}cm "
        f"str={pose.get('strafeCm', 0):.2f}cm "
        f"prog={pose.get('progressCm', 0):.2f}cm "
        f"rem={pose.get('remainingCm', 0):.2f}cm "
        f"herr={move.get('headingErrorDeg', 0):.2f}deg"
    )


def progress_summary(data: dict) -> str:
    pose = data.get("pose", {})
    imu = data.get("imu", {})
    move = data.get("move", {})
    rpm = data.get("rpm", [])
    return (
        "progress | "
        f"mode={data.get('mode')} "
        f"prog={pose.get('progressCm', 0):.2f}cm "
        f"rem={pose.get('remainingCm', 0):.2f}cm "
        f"yaw={imu.get('yawDeg', 0):.2f}deg "
        f"herr={move.get('headingErrorDeg', 0):.2f}deg "
        f"rpm={rpm}"
    )


def print_done(done: dict) -> None:
    print(
        "done | "
        f"result={done.get('result')} "
        f"heading={done.get('headingDeg', 0):.2f}deg "
        f"forward={done.get('forwardCm', 0):.2f}cm "
        f"strafe={done.get('strafeCm', 0):.2f}cm "
        f"progress={done.get('progressCm', 0):.2f}cm"
    )


def initialize_robot(link: EspPiControlLink) -> None:
    print("checking UART link")
    link.drain(1.0)

    if link.last_telemetry is None:
        seq = link.send_command("PING")
        try:
            pong = link.wait_for(seq, {"pong"}, timeout=PING_TIMEOUT_S)
        except TimeoutError as exc:
            raise RuntimeError(
                "No UART response from ESP. "
                "The Pi opened /dev/ttyAMA0 but the ESP did not answer PING. "
                "Check power, reset the ESP, confirm TX0/RX0 wiring, and make sure the flashed sketch is running."
            ) from exc
        print(f"link ok | pong seq={pong.get('seq')}")
    else:
        print("link ok | passive telemetry received")

    print("initializing IMU: calibrate bias + zero yaw")
    ack = try_ack(link, "INIT_IMU", INIT_TIMEOUT_S)
    if ack is not None:
        require_ok_ack(ack)
        print(f"init | {ack.get('message')}")
        return

    print("init fallback | INIT_IMU did not answer, trying CAL_IMU then ZERO_IMU")
    ack = try_ack(link, "CAL_IMU", INIT_TIMEOUT_S)
    if ack is None:
        raise RuntimeError(
            "ESP answered PING earlier, but did not answer INIT_IMU or CAL_IMU. "
            "Flash esp_correct_pid_pi.ino, then reset the ESP and retry."
        )

    require_ok_ack(ack)
    print(f"init | {ack.get('message')}")

    ack = try_ack(link, "ZERO_IMU", ACK_TIMEOUT_S)
    if ack is None:
        raise RuntimeError("IMU bias calibration finished, but ZERO_IMU did not answer.")
    require_ok_ack(ack)
    print(f"init | {ack.get('message')}")


def request_status(link: EspPiControlLink) -> dict:
    seq = link.send_command("STATUS")
    ack = link.wait_for(seq, {"ack"}, timeout=ACK_TIMEOUT_S)
    require_ok_ack(ack)
    return link.wait_for(None, {"telemetry"}, timeout=ACK_TIMEOUT_S, match_seq=False)


def send_stop(link: EspPiControlLink) -> None:
    seq = link.send_command("STOP")
    ack = link.wait_for(seq, {"ack"}, timeout=ACK_TIMEOUT_S)
    require_ok_ack(ack)
    print(f"stop | {ack.get('message')}")


def execute_move(
    link: EspPiControlLink,
    supervisor: MecanumIrStepperSupervisor,
    direction: str,
    heading_deg: float,
    distance_cm: float,
    speed_rpm: float,
) -> None:
    supervisor.ensure_motion_allowed()
    line = move_command(direction, heading_deg, distance_cm, speed_rpm)
    print(f"send | {line}")
    supervisor.set_mecanum_active(True)
    try:
        seq = link.send_command(line)
        ack = link.wait_for(seq, {"ack"}, timeout=ACK_TIMEOUT_S)
        require_ok_ack(ack)
        print(f"ack  | {ack.get('message')}")
        done = link.wait_for_done(seq, timeout=MOVE_DONE_TIMEOUT_S)
        print_done(done)

        if done.get("result") == "completed" and not supervisor.ir_stop_latched.is_set():
            supervisor.run_stepper_after_move()
        else:
            print("stepper | skipped because move did not complete cleanly")
    finally:
        supervisor.set_mecanum_active(False)


def execute_turn(
    link: EspPiControlLink,
    supervisor: MecanumIrStepperSupervisor,
    heading_deg: float,
    speed_rpm: float,
) -> None:
    supervisor.ensure_motion_allowed()
    line = turn_command(heading_deg, speed_rpm)
    print(f"send | {line}")
    supervisor.set_mecanum_active(True)
    try:
        seq = link.send_command(line)
        ack = link.wait_for(seq, {"ack"}, timeout=ACK_TIMEOUT_S)
        require_ok_ack(ack)
        print(f"ack  | {ack.get('message')}")
        done = link.wait_for_done(seq, timeout=TURN_DONE_TIMEOUT_S)
        print_done(done)
    finally:
        supervisor.set_mecanum_active(False)


def print_help() -> None:
    print("Commands:")
    print("  move forward <heading_deg> <distance_cm> <speed_rpm>")
    print("  move back <heading_deg> <distance_cm> <speed_rpm>")
    print("  turn <heading_deg> [speed_rpm]")
    print("  status")
    print("  stop")
    print("  clear")
    print("  reinit")
    print("  ir status")
    print("  stepper steps <count>")
    print("  stepper dir <1|-1>")
    print("  stepper status")
    print("  stepper run")
    print("  help")
    print("  quit")


def interactive_shell(link: EspPiControlLink, supervisor: MecanumIrStepperSupervisor) -> None:
    print_help()
    print("IR note | startup baseline was captured when the script started, so begin with the path clear.")

    while True:
        try:
            raw = input("> ").strip()
        except EOFError:
            return

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0].lower()

        try:
            if cmd in {"quit", "exit"}:
                return

            if cmd == "help":
                print_help()
                continue

            if cmd == "status":
                data = request_status(link)
                print(telemetry_summary(data))
                print(supervisor.safety_summary())
                continue

            if cmd == "reinit":
                initialize_robot(link)
                data = request_status(link)
                print(telemetry_summary(data))
                continue

            if cmd == "stop":
                send_stop(link)
                continue

            if cmd == "clear":
                supervisor.clear_latch()
                continue

            if cmd == "ir" and len(parts) == 2 and parts[1].lower() == "status":
                print(supervisor.ir_monitor.status_text())
                continue

            if cmd == "stepper":
                if len(parts) < 2:
                    print("usage: stepper <steps|dir|status|run> ...")
                    continue

                subcmd = parts[1].lower()
                if subcmd == "steps" and len(parts) == 3:
                    supervisor.stepper.set_steps(int(parts[2]))
                    print(supervisor.stepper.status_text())
                    continue
                if subcmd == "dir" and len(parts) == 3:
                    direction = int(parts[2])
                    if direction == 0:
                        raise ValueError("Stepper direction must be 1 or -1")
                    supervisor.stepper.set_direction(direction)
                    print(supervisor.stepper.status_text())
                    continue
                if subcmd == "status" and len(parts) == 2:
                    print(supervisor.stepper.status_text())
                    continue
                if subcmd == "run" and len(parts) == 2:
                    supervisor.stepper.run_configured(supervisor.ir_stop_latched)
                    continue

                print("usage: stepper steps <count> | stepper dir <1|-1> | stepper status | stepper run")
                continue

            if cmd == "turn":
                if len(parts) not in {2, 3}:
                    print("usage: turn <heading_deg> [speed_rpm]")
                    continue

                heading_deg = float(parts[1])
                speed_rpm = float(parts[2]) if len(parts) == 3 else DEFAULT_TURN_SPEED_RPM
                execute_turn(link, supervisor, heading_deg, speed_rpm)
                continue

            if cmd == "move":
                if len(parts) != 5:
                    print("usage: move <forward|back> <heading_deg> <distance_cm> <speed_rpm>")
                    continue

                direction = parts[1]
                heading_deg = float(parts[2])
                distance_cm = float(parts[3])
                speed_rpm = float(parts[4])
                execute_move(link, supervisor, direction, heading_deg, distance_cm, speed_rpm)
                continue

            print("unknown command")
        except ValueError as exc:
            print(f"input error | {exc}")
        except TimeoutError as exc:
            print(f"timeout | {exc}")
        except RuntimeError as exc:
            print(f"error | {exc}")


def main() -> None:
    link = EspPiControlLink(PORT, BAUD)
    supervisor: MecanumIrStepperSupervisor | None = None
    try:
        print(f"opened {PORT} @ {BAUD}")
        link.drain(1.2)
        initialize_robot(link)
        data = request_status(link)
        print(telemetry_summary(data))
        supervisor = MecanumIrStepperSupervisor(link)
        print(supervisor.ir_monitor.status_text())
        print(supervisor.stepper.status_text())
        interactive_shell(link, supervisor)
    finally:
        if supervisor is not None:
            supervisor.close()
        link.close()


if __name__ == "__main__":
    main()
