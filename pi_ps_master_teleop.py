#!/usr/bin/env python3
"""
PlayStation controller teleop for pi_master.py + esp_correct_pid_pi.ino.

Controls:
  left stick  : 8-way translate
  d-pad       : front/back/right/left translate override
  right stick : heading/rotate
  R1 / L1     : hold scissor lift up / down
  triangle    : reinitialize IMU
  square      : home stepper using ESP GPIO 23 limit command
  circle      : buzzer while held
  cross       : traffic-light dance
"""

from __future__ import annotations

import argparse
import array
import glob
import math
import os
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
CURRENT_FILES_DIR = SCRIPT_DIR / "current files"
if CURRENT_FILES_DIR.exists():
    sys.path.insert(0, str(CURRENT_FILES_DIR))

from pi_master import (
    ACK_TIMEOUT_S,
    ESP_DEFAULT_BAUD,
    EspPiControlLink,
    GpioBuzzer,
    GpioStepperLift,
    INIT_TIMEOUT_S,
    STEPPER_DIR_PIN,
    STEPPER_EN_PIN,
    STEPPER_STEP_HIGH_US,
    STEPPER_STEP_PIN,
    STEPPER_UP_DIR,
    STEPPER_DOWN_DIR,
    STEPPER_SPEED_SPS,
    TRAFFIC_GREEN_LED_PIN,
    TRAFFIC_RED_LED_PIN,
    TRAFFIC_YELLOW_LED_PIN,
    DEFAULT_BUZZER_PIN,
    ensure_gpiozero_runtime,
)


MIN_RPM = 8.0
MAX_RPM = 40.0
DEFAULT_RPM = 40.0

STICK_DEADZONE = 0.18
DPAD_AXIS_THRESHOLD = 0.5
TWIST_SEND_INTERVAL_S = 0.08
TWIST_TIMEOUT_MS = 300
TELEOP_ACK_TIMEOUT_S = 0.25
BUZZER_HONK_MAX_S = 2.0

HOMING_LIMIT_PIN = 23
HOMING_CHUNK_STEPS = 25
HOMING_MAX_STEPS = 20000

AXIS_LEFT_X = int(os.environ.get("PS_AXIS_LEFT_X", "0"))
AXIS_LEFT_Y = int(os.environ.get("PS_AXIS_LEFT_Y", "1"))
AXIS_RIGHT_X = int(os.environ.get("PS_AXIS_RIGHT_X", "2"))
AXIS_DPAD_X = int(os.environ.get("PS_AXIS_DPAD_X", "6"))
AXIS_DPAD_Y = int(os.environ.get("PS_AXIS_DPAD_Y", "7"))

BTN_CROSS = int(os.environ.get("PS_BTN_CROSS", "0"))
BTN_CIRCLE = int(os.environ.get("PS_BTN_CIRCLE", "1"))
BTN_SQUARE = int(os.environ.get("PS_BTN_SQUARE", "2"))
BTN_TRIANGLE = int(os.environ.get("PS_BTN_TRIANGLE", "3"))
BTN_L1 = int(os.environ.get("PS_BTN_L1", "9"))
BTN_R1 = int(os.environ.get("PS_BTN_R1", "10"))

LINUX_DUALSENSE_AXIS_RIGHT_X = int(os.environ.get("PS_LINUX_AXIS_RIGHT_X", "3"))
LINUX_DUALSENSE_AXIS_DPAD_X = int(os.environ.get("PS_LINUX_AXIS_DPAD_X", "6"))
LINUX_DUALSENSE_AXIS_DPAD_Y = int(os.environ.get("PS_LINUX_AXIS_DPAD_Y", "7"))
LINUX_DUALSENSE_BTN_L1 = int(os.environ.get("PS_LINUX_BTN_L1", "4"))
LINUX_DUALSENSE_BTN_R1 = int(os.environ.get("PS_LINUX_BTN_R1", "5"))

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12


@dataclass(frozen=True)
class Twist:
    forward_rpm: float
    strafe_rpm: float
    turn_rpm: float

    def active(self) -> bool:
        return (
            abs(self.forward_rpm) > 0.01
            or abs(self.strafe_rpm) > 0.01
            or abs(self.turn_rpm) > 0.01
        )


@dataclass(frozen=True)
class ControllerMapping:
    left_x: int
    left_y: int
    right_x: int
    dpad_x: int
    dpad_y: int
    button_cross: int
    button_circle: int
    button_square: int
    button_triangle: int
    button_l1: int
    button_r1: int


class ControllerState:
    def __init__(self, joystick, mapping: ControllerMapping) -> None:
        self.joystick = joystick
        self.mapping = mapping
        self.axis_centers = {
            mapping.left_x: get_axis(joystick, mapping.left_x),
            mapping.left_y: get_axis(joystick, mapping.left_y),
            mapping.right_x: get_axis(joystick, mapping.right_x),
        }

    def centered_axis(self, axis_index: int) -> float:
        raw_value = get_axis(self.joystick, axis_index)
        center = self.axis_centers.get(axis_index, 0.0)
        return clamp(raw_value - center, -1.0, 1.0)


class ButtonEdges:
    def __init__(self) -> None:
        self.previous: dict[int, bool] = {}

    def pressed(self, joystick, button_index: int) -> bool:
        now = get_button(joystick, button_index)
        was = self.previous.get(button_index, False)
        self.previous[button_index] = now
        return now and not was


class LinuxJoystick:
    uses_pygame = False

    def __init__(self, device_path: str) -> None:
        import fcntl

        self.device_path = device_path
        self.name = os.path.basename(device_path)
        self.file = open(device_path, "rb", buffering=0)
        self.axis_count = self._read_u8_ioctl(fcntl, JSIOCGAXES)
        self.button_count = self._read_u8_ioctl(fcntl, JSIOCGBUTTONS)
        self.axes = [0.0] * self.axis_count
        self.buttons = [False] * self.button_count
        self.stop_event = threading.Event()
        self.reader = threading.Thread(target=self._reader_loop, name="linux-joystick", daemon=True)
        self.reader.start()

    def _read_u8_ioctl(self, fcntl_module, request: int) -> int:
        buf = array.array("B", [0])
        fcntl_module.ioctl(self.file.fileno(), request, buf, True)
        return int(buf[0])

    def _reader_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                event = self.file.read(8)
            except OSError:
                return
            if len(event) != 8:
                return

            _, value, event_type, number = struct.unpack("IhBB", event)
            event_type &= ~JS_EVENT_INIT
            if event_type == JS_EVENT_AXIS and number < self.axis_count:
                self.axes[number] = max(-1.0, min(1.0, value / 32767.0))
            elif event_type == JS_EVENT_BUTTON and number < self.button_count:
                self.buttons[number] = bool(value)

    def init(self) -> None:
        pass

    def close(self) -> None:
        self.stop_event.set()
        try:
            self.file.close()
        except OSError:
            pass

    def get_name(self) -> str:
        return self.name

    def get_numaxes(self) -> int:
        return self.axis_count

    def get_numbuttons(self) -> int:
        return self.button_count

    def get_numhats(self) -> int:
        return 0

    def get_axis(self, axis_index: int) -> float:
        if axis_index < 0 or axis_index >= self.axis_count:
            return 0.0
        return self.axes[axis_index]

    def get_button(self, button_index: int) -> bool:
        if button_index < 0 or button_index >= self.button_count:
            return False
        return self.buttons[button_index]


class GpioTrafficLight:
    def __init__(self, *, enabled: bool, mock: bool) -> None:
        self.enabled = bool(enabled and not mock)
        self.green = None
        self.yellow = None
        self.red = None
        self.lock = threading.Lock()
        if not self.enabled:
            return
        try:
            ensure_gpiozero_runtime()
            from gpiozero import DigitalOutputDevice

            self.green = DigitalOutputDevice(TRAFFIC_GREEN_LED_PIN, initial_value=False)
            self.yellow = DigitalOutputDevice(TRAFFIC_YELLOW_LED_PIN, initial_value=False)
            self.red = DigitalOutputDevice(TRAFFIC_RED_LED_PIN, initial_value=False)
            print(
                f"gpio | traffic green=GPIO{TRAFFIC_GREEN_LED_PIN} "
                f"yellow=GPIO{TRAFFIC_YELLOW_LED_PIN} red=GPIO{TRAFFIC_RED_LED_PIN}",
                flush=True,
            )
        except Exception as exc:
            print(f"warn | traffic light unavailable; disabled: {exc}", flush=True)
            self.close()
            self.enabled = False

    def set_lights(self, *, green: bool = False, yellow: bool = False, red: bool = False) -> None:
        with self.lock:
            for device, active in ((self.green, green), (self.yellow, yellow), (self.red, red)):
                if device is None:
                    continue
                if active:
                    device.on()
                else:
                    device.off()

    def close(self) -> None:
        try:
            self.set_lights()
        except Exception:
            pass
        for device in (self.green, self.yellow, self.red):
            if device is not None:
                try:
                    device.close()
                except Exception:
                    pass
        self.green = None
        self.yellow = None
        self.red = None


class BackgroundActions:
    def __init__(self, link: EspPiControlLink, stepper: GpioStepperLift, traffic: GpioTrafficLight) -> None:
        self.link = link
        self.stepper = stepper
        self.traffic = traffic
        self.action_lock = threading.Lock()
        self.stepper_hold_thread: threading.Thread | None = None
        self.stepper_hold_stop: threading.Event | None = None
        self.stepper_hold_direction: int | None = None

    def close(self) -> None:
        self.set_stepper_hold(None)

    def set_stepper_hold(self, direction: int | None) -> None:
        if (
            direction == self.stepper_hold_direction
            and self.stepper_hold_thread is not None
            and self.stepper_hold_thread.is_alive()
        ):
            return

        self._stop_stepper_hold()
        if direction is None:
            return

        stop_event = threading.Event()
        self.stepper_hold_stop = stop_event
        self.stepper_hold_direction = direction

        def worker() -> None:
            text = "up" if direction > 0 else "down"
            try:
                self.stepper.run_until_released(direction, stop_event, label=f"manual {text}")
            finally:
                self.stepper_hold_direction = None
                self.stepper_hold_stop = None
                self.stepper_hold_thread = None

        thread = threading.Thread(target=worker, name="stepper_hold", daemon=True)
        self.stepper_hold_thread = thread
        thread.start()

    def _stop_stepper_hold(self) -> None:
        stop_event = self.stepper_hold_stop
        thread = self.stepper_hold_thread
        if stop_event is None or thread is None:
            self.stepper_hold_direction = None
            return
        stop_event.set()
        thread.join(timeout=0.5)
        self.stepper_hold_direction = None

    def home_stepper(self) -> None:
        self.set_stepper_hold(None)
        if not self.action_lock.acquire(blocking=False):
            print("stepper_home | busy", flush=True)
            return

        def worker() -> None:
            try:
                self._home_stepper_worker()
            finally:
                self.action_lock.release()

        threading.Thread(target=worker, name="stepper_home", daemon=True).start()

    def _home_stepper_worker(self) -> None:
        print(f"home | checking ESP limit switch on GPIO {HOMING_LIMIT_PIN}", flush=True)
        try:
            if self._limit_pressed():
                print("home | already on limit", flush=True)
                return
        except RuntimeError as exc:
            print(f"home | unavailable: {exc}", flush=True)
            return

        moved = 0
        while moved < HOMING_MAX_STEPS:
            self.stepper.run_steps(HOMING_CHUNK_STEPS, STEPPER_DOWN_DIR, label="home")
            moved += HOMING_CHUNK_STEPS

            try:
                if self._limit_pressed():
                    print(f"home | limit reached after {moved} steps", flush=True)
                    return
            except RuntimeError as exc:
                print(f"home | unavailable after moving {moved} steps: {exc}", flush=True)
                return

        print("home | stopped before limit was reached", flush=True)

    def _limit_pressed(self) -> bool:
        seq = self.link.send_command(f"LIMIT_STATUS pin={HOMING_LIMIT_PIN}")
        try:
            data = self.link.wait_for(seq, {"ack", "limit"}, timeout=ACK_TIMEOUT_S)
        except TimeoutError as exc:
            raise RuntimeError(
                "LIMIT_STATUS timed out. Check UART wiring/power and confirm the ESP sketch is running."
            ) from exc

        if data.get("type") == "limit":
            return bool(data.get("pressed", False))

        if data.get("cmd") == "LIMIT_STATUS" and data.get("ok", False):
            return bool(data.get("pressed", False))

        message = data.get("message", "ESP sketch does not support LIMIT_STATUS")
        raise RuntimeError(message)

    def traffic_dance(self) -> None:
        if not self.action_lock.acquire(blocking=False):
            print("traffic | busy", flush=True)
            return

        def worker() -> None:
            try:
                pattern = [
                    (True, False, False, 0.12),
                    (False, True, False, 0.12),
                    (False, False, True, 0.12),
                    (True, True, True, 0.12),
                    (False, False, False, 0.10),
                ]
                print("traffic | dance", flush=True)
                for _ in range(5):
                    for green, yellow, red, duration_s in pattern:
                        self.traffic.set_lights(green=green, yellow=yellow, red=red)
                        time.sleep(duration_s)
                self.traffic.set_lights()
            finally:
                self.action_lock.release()

        threading.Thread(target=worker, name="traffic_dance", daemon=True).start()


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def apply_deadzone(value: float, deadzone: float = STICK_DEADZONE) -> float:
    if abs(value) <= deadzone:
        return 0.0
    scaled = (abs(value) - deadzone) / (1.0 - deadzone)
    return math.copysign(clamp(scaled, 0.0, 1.0), value)


def axis_to_cardinal(value: float) -> int:
    if value >= DPAD_AXIS_THRESHOLD:
        return 1
    if value <= -DPAD_AXIS_THRESHOLD:
        return -1
    return 0


def get_axis(joystick, axis_index: int) -> float:
    if axis_index < 0 or axis_index >= joystick.get_numaxes():
        return 0.0
    return float(joystick.get_axis(axis_index))


def get_button(joystick, button_index: int) -> bool:
    if button_index < 0 or button_index >= joystick.get_numbuttons():
        return False
    return bool(joystick.get_button(button_index))


def default_mapping() -> ControllerMapping:
    return ControllerMapping(
        left_x=AXIS_LEFT_X,
        left_y=AXIS_LEFT_Y,
        right_x=AXIS_RIGHT_X,
        dpad_x=AXIS_DPAD_X,
        dpad_y=AXIS_DPAD_Y,
        button_cross=BTN_CROSS,
        button_circle=BTN_CIRCLE,
        button_square=BTN_SQUARE,
        button_triangle=BTN_TRIANGLE,
        button_l1=BTN_L1,
        button_r1=BTN_R1,
    )


def controller_mapping_for(joystick) -> ControllerMapping:
    name = joystick.get_name().lower()
    mapping = default_mapping()
    if "dualsense" in name or "wireless controller" in name:
        return ControllerMapping(
            left_x=mapping.left_x,
            left_y=mapping.left_y,
            right_x=LINUX_DUALSENSE_AXIS_RIGHT_X,
            dpad_x=LINUX_DUALSENSE_AXIS_DPAD_X,
            dpad_y=LINUX_DUALSENSE_AXIS_DPAD_Y,
            button_cross=mapping.button_cross,
            button_circle=mapping.button_circle,
            button_square=mapping.button_square,
            button_triangle=mapping.button_triangle,
            button_l1=LINUX_DUALSENSE_BTN_L1,
            button_r1=LINUX_DUALSENSE_BTN_R1,
        )
    return mapping


def select_controller(pygame):
    count = pygame.joystick.get_count()
    linux_devices = sorted(glob.glob("/dev/input/js*"))
    if count <= 0 and linux_devices:
        if len(linux_devices) == 1:
            joystick = LinuxJoystick(linux_devices[0])
            print(f"controller | {joystick.get_name()} via linux js fallback", flush=True)
            return joystick
        for index, path in enumerate(linux_devices):
            print(f"{index}: {path}", flush=True)
        choice = int(input("controller index> ").strip())
        joystick = LinuxJoystick(linux_devices[choice])
        print(f"controller | {joystick.get_name()} via linux js fallback", flush=True)
        return joystick
    if count <= 0:
        raise SystemExit("No controller found. Pair/connect the PS controller and retry.")
    if count == 1:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"controller | {joystick.get_name()}", flush=True)
        return joystick

    for index in range(count):
        joystick = pygame.joystick.Joystick(index)
        joystick.init()
        print(f"{index}: {joystick.get_name()}", flush=True)
    choice = int(input("controller index> ").strip())
    joystick = pygame.joystick.Joystick(choice)
    joystick.init()
    return joystick


def list_controllers(pygame) -> int:
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    linux_devices = sorted(glob.glob("/dev/input/js*"))
    print(f"pygame_controller_count={count}", flush=True)
    for index in range(count):
        joystick = pygame.joystick.Joystick(index)
        joystick.init()
        mapping = controller_mapping_for(joystick)
        print(
            f"{index}: {joystick.get_name()} "
            f"axes={joystick.get_numaxes()} buttons={joystick.get_numbuttons()} hats={joystick.get_numhats()} "
            f"mapping(lx={mapping.left_x},ly={mapping.left_y},rx={mapping.right_x},"
            f"dpadx={mapping.dpad_x},dpady={mapping.dpad_y},"
            f"cross={mapping.button_cross},circle={mapping.button_circle},"
            f"square={mapping.button_square},triangle={mapping.button_triangle},"
            f"l1={mapping.button_l1},r1={mapping.button_r1})",
            flush=True,
        )
    print(f"linux_js_count={len(linux_devices)}", flush=True)
    for index, path in enumerate(linux_devices):
        try:
            joystick = LinuxJoystick(path)
            mapping = controller_mapping_for(joystick)
            print(
                f"linux {index}: {path} axes={joystick.get_numaxes()} buttons={joystick.get_numbuttons()} "
                f"mapping(lx={mapping.left_x},ly={mapping.left_y},rx={mapping.right_x},"
                f"dpadx={mapping.dpad_x},dpady={mapping.dpad_y},"
                f"cross={mapping.button_cross},circle={mapping.button_circle},"
                f"square={mapping.button_square},triangle={mapping.button_triangle},"
                f"l1={mapping.button_l1},r1={mapping.button_r1})",
                flush=True,
            )
            joystick.close()
        except Exception as exc:
            print(f"linux {index}: {path} unavailable: {exc}", flush=True)
    return count + len(linux_devices)


def quantized_translation(left_x: float, left_y: float, speed_rpm: float) -> tuple[float, float]:
    x = apply_deadzone(left_x)
    y = apply_deadzone(-left_y)
    magnitude = clamp(math.hypot(x, y), 0.0, 1.0)
    if magnitude <= 0.0:
        return 0.0, 0.0

    angle_deg = math.degrees(math.atan2(x, y))
    quantized_deg = round(angle_deg / 45.0) * 45.0
    angle_rad = math.radians(quantized_deg)
    forward_unit = math.cos(angle_rad)
    strafe_unit = math.sin(angle_rad)
    wheel_mix_max = max(
        abs(forward_unit - strafe_unit),
        abs(forward_unit + strafe_unit),
        0.001,
    )
    output_rpm = MIN_RPM + magnitude * (speed_rpm - MIN_RPM)
    scaled_rpm = output_rpm / wheel_mix_max
    return forward_unit * scaled_rpm, strafe_unit * scaled_rpm


def limit_combined_twist(forward: float, strafe: float, turn: float, max_rpm: float) -> tuple[float, float, float]:
    wheel_targets = [
        forward - strafe - turn,
        forward + strafe - turn,
        forward + strafe + turn,
        forward - strafe + turn,
    ]
    largest = max(abs(value) for value in wheel_targets)
    if largest <= max_rpm or largest <= 0.001:
        return forward, strafe, turn
    scale = max_rpm / largest
    return forward * scale, strafe * scale, turn * scale


def joystick_to_twist(controller: ControllerState, speed_rpm: float) -> Twist:
    forward, strafe = quantized_translation(
        controller.centered_axis(controller.mapping.left_x),
        controller.centered_axis(controller.mapping.left_y),
        speed_rpm,
    )
    turn_axis = apply_deadzone(controller.centered_axis(controller.mapping.right_x))
    turn = 0.0
    if turn_axis != 0.0:
        turn = math.copysign(MIN_RPM + abs(turn_axis) * (speed_rpm - MIN_RPM), turn_axis)
    forward, strafe, turn = limit_combined_twist(forward, strafe, turn, speed_rpm)
    return Twist(forward, strafe, turn)


def dpad_to_twist(controller: ControllerState, speed_rpm: float) -> Twist | None:
    joystick = controller.joystick
    if hasattr(joystick, "get_numhats") and joystick.get_numhats() > 0:
        dpad_x, dpad_y = joystick.get_hat(0)
    else:
        dpad_x = axis_to_cardinal(get_axis(joystick, controller.mapping.dpad_x))
        dpad_y = -axis_to_cardinal(get_axis(joystick, controller.mapping.dpad_y))

    if dpad_y > 0:
        return Twist(speed_rpm, 0.0, 0.0)
    if dpad_y < 0:
        return Twist(-speed_rpm, 0.0, 0.0)
    if dpad_x > 0:
        return Twist(0.0, speed_rpm, 0.0)
    if dpad_x < 0:
        return Twist(0.0, -speed_rpm, 0.0)
    return None


def send_command_ack(link: EspPiControlLink, line: str, timeout: float = ACK_TIMEOUT_S) -> dict:
    seq = link.send_command(line)
    ack = link.wait_for(seq, {"ack", "pong"}, timeout=timeout)
    if ack.get("type") == "ack" and not ack.get("ok", False):
        raise RuntimeError(f"{ack.get('cmd', line)} rejected: {ack.get('message', 'no message')}")
    return ack


def send_twist(link: EspPiControlLink, twist: Twist) -> None:
    seq = link.send_command(
        f"TWIST forward={twist.forward_rpm:.3f} "
        f"strafe={twist.strafe_rpm:.3f} "
        f"turn={twist.turn_rpm:.3f} "
        f"timeout={TWIST_TIMEOUT_MS}"
    )
    ack = link.wait_for(seq, {"ack"}, timeout=TELEOP_ACK_TIMEOUT_S)
    if not ack.get("ok", False):
        raise RuntimeError(f"TWIST rejected: {ack.get('message', 'no message')}")


def stop_robot(link: EspPiControlLink) -> None:
    try:
        send_command_ack(link, "STOP")
    except Exception as exc:
        print(f"stop warning | {exc}", flush=True)


def print_controls() -> None:
    print("controls", flush=True)
    print("  left stick  | 8-way translate", flush=True)
    print("  d-pad       | F/B/R/L translate override", flush=True)
    print("  right stick | heading/rotate", flush=True)
    print("  speed       | fixed 40 rpm by default", flush=True)
    print("  R1/L1       | hold lift up/down", flush=True)
    print("  triangle    | INIT_IMU", flush=True)
    print("  square      | home stepper using ESP GPIO 23 limit command", flush=True)
    print("  circle      | buzzer while held", flush=True)
    print("  cross       | traffic light dance", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(description="PS controller teleop for esp_correct_pid_pi.ino")
    parser.add_argument("--port", default=os.environ.get("PI_UART_PORT", "/dev/ttyAMA0"))
    parser.add_argument("--baud", type=int, default=ESP_DEFAULT_BAUD)
    parser.add_argument("--speed-rpm", type=float, default=DEFAULT_RPM, help="fixed teleop speed, default 40 rpm")
    parser.add_argument("--stepper-speed-sps", type=float, default=STEPPER_SPEED_SPS)
    parser.add_argument("--no-init-imu", action="store_true")
    parser.add_argument("--mock-gpio", action="store_true")
    parser.add_argument("--list-controllers", action="store_true")
    parser.add_argument("--stepper", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--buzzer", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--traffic", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    try:
        import pygame
    except ImportError as exc:
        raise SystemExit("pygame is required. Install it on the Pi with: sudo apt install python3-pygame") from exc

    pygame.init()
    if args.list_controllers:
        count = list_controllers(pygame)
        pygame.quit()
        if count <= 0:
            raise SystemExit("No controller found by pygame or /dev/input/js*.")
        return

    joystick = select_controller(pygame)
    mapping = controller_mapping_for(joystick)
    controller = ControllerState(joystick, mapping)
    edges = ButtonEdges()

    print_controls()
    print(
        "controller | mapping "
        f"lx={mapping.left_x} ly={mapping.left_y} rx={mapping.right_x} "
        f"dpadx={mapping.dpad_x} dpady={mapping.dpad_y} "
        f"cross={mapping.button_cross} circle={mapping.button_circle} "
        f"square={mapping.button_square} triangle={mapping.button_triangle} "
        f"l1={mapping.button_l1} r1={mapping.button_r1}",
        flush=True,
    )

    link = EspPiControlLink(args.port, args.baud, verbose_serial=False)
    stepper = GpioStepperLift(
        step_pin=STEPPER_STEP_PIN,
        dir_pin=STEPPER_DIR_PIN,
        en_pin=STEPPER_EN_PIN,
        speed_sps=args.stepper_speed_sps,
        step_high_us=STEPPER_STEP_HIGH_US,
        enabled=args.stepper,
        mock=args.mock_gpio,
    )
    buzzer = GpioBuzzer(pin=DEFAULT_BUZZER_PIN, active_high=True, enabled=args.buzzer, mock=args.mock_gpio)
    traffic = GpioTrafficLight(enabled=args.traffic, mock=args.mock_gpio)
    actions = BackgroundActions(link, stepper, traffic)

    speed_rpm = clamp(float(args.speed_rpm), MIN_RPM, MAX_RPM)
    last_send_s = 0.0
    was_moving = False
    honk_started_s: float | None = None

    try:
        print(f"opened {args.port} @ {args.baud}", flush=True)
        try:
            pong = send_command_ack(link, "PING", timeout=2.5)
            print(f"link ok | pong seq={pong.get('seq')}", flush=True)
        except Exception as exc:
            print(f"link warning | {exc}", flush=True)
        if not args.no_init_imu:
            print("init | IMU", flush=True)
            send_command_ack(link, "INIT_IMU", timeout=INIT_TIMEOUT_S)
        print(f"speed | fixed {speed_rpm:.1f} rpm", flush=True)

        while True:
            if getattr(joystick, "uses_pygame", True):
                pygame.event.pump()
            now_s = time.time()

            if edges.pressed(joystick, mapping.button_triangle):
                print("imu | INIT_IMU", flush=True)
                send_command_ack(link, "INIT_IMU", timeout=INIT_TIMEOUT_S)

            if edges.pressed(joystick, mapping.button_square):
                actions.home_stepper()

            if edges.pressed(joystick, mapping.button_cross):
                actions.traffic_dance()

            l1_held = get_button(joystick, mapping.button_l1)
            r1_held = get_button(joystick, mapping.button_r1)
            if r1_held and not l1_held:
                actions.set_stepper_hold(STEPPER_UP_DIR)
            elif l1_held and not r1_held:
                actions.set_stepper_hold(STEPPER_DOWN_DIR)
            else:
                actions.set_stepper_hold(None)

            honking = get_button(joystick, mapping.button_circle)
            if honking and honk_started_s is None:
                honk_started_s = now_s
            if not honking:
                honk_started_s = None
            honk_allowed = honk_started_s is not None and now_s - honk_started_s <= BUZZER_HONK_MAX_S
            if honk_allowed:
                buzzer.on()
            else:
                buzzer.off()

            twist = dpad_to_twist(controller, speed_rpm)
            if twist is None:
                twist = joystick_to_twist(controller, speed_rpm)

            if twist.active():
                if now_s - last_send_s >= TWIST_SEND_INTERVAL_S:
                    send_twist(link, twist)
                    last_send_s = now_s
                    was_moving = True
            elif was_moving:
                stop_robot(link)
                was_moving = False

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping.", flush=True)
    finally:
        actions.close()
        buzzer.off()
        traffic.close()
        stepper.close()
        stop_robot(link)
        link.close()
        if hasattr(joystick, "close"):
            joystick.close()
        pygame.quit()


if __name__ == "__main__":
    main()
