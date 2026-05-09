#!/usr/bin/env python3

import array
import fcntl
import glob
import math
import os
from pathlib import Path
import struct
import threading
import time
from dataclasses import dataclass

from pi_mecanum_ir_stepper_control import (
    ACK_TIMEOUT_S,
    BAUD,
    PORT,
    EspPiControlLink,
    MecanumIrStepperSupervisor,
    initialize_robot,
    request_status,
    require_ok_ack,
    telemetry_summary,
)


MIN_RPM = 8.0
MAX_RPM = 60.0
DEFAULT_RPM = 25.0
SPEED_CHANGE_RPM_PER_S = 24.0

STICK_DEADZONE = 0.18
TRIGGER_DEADZONE = 0.08
DPAD_AXIS_THRESHOLD = 0.5
TWIST_SEND_INTERVAL_S = 0.08
TWIST_TIMEOUT_MS = 300
TELEOP_ACK_TIMEOUT_S = 0.20
BUZZER_HONK_MAX_S = 2.0

STEPPER_JOG_STEPS = 100
STEPPER_UP_DIR = -1
STEPPER_DOWN_DIR = 1

HOMING_LIMIT_PIN = 23
HOMING_CHUNK_STEPS = 25
HOMING_MAX_STEPS = 20000

AXIS_LEFT_X = int(os.environ.get("PS_AXIS_LEFT_X", "0"))
AXIS_LEFT_Y = int(os.environ.get("PS_AXIS_LEFT_Y", "1"))
AXIS_RIGHT_X = int(os.environ.get("PS_AXIS_RIGHT_X", "2"))
AXIS_LEFT_TRIGGER = int(os.environ.get("PS_AXIS_L2", "4"))
AXIS_RIGHT_TRIGGER = int(os.environ.get("PS_AXIS_R2", "5"))
AXIS_DPAD_X = int(os.environ.get("PS_AXIS_DPAD_X", "6"))
AXIS_DPAD_Y = int(os.environ.get("PS_AXIS_DPAD_Y", "7"))

BTN_CROSS = int(os.environ.get("PS_BTN_CROSS", "0"))
BTN_CIRCLE = int(os.environ.get("PS_BTN_CIRCLE", "1"))
BTN_SQUARE = int(os.environ.get("PS_BTN_SQUARE", "2"))
BTN_TRIANGLE = int(os.environ.get("PS_BTN_TRIANGLE", "3"))
BTN_L1 = int(os.environ.get("PS_BTN_L1", "9"))
BTN_R1 = int(os.environ.get("PS_BTN_R1", "10"))

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12

LINUX_DUALSENSE_AXIS_RIGHT_X = int(os.environ.get("PS_LINUX_AXIS_RIGHT_X", "3"))
LINUX_DUALSENSE_AXIS_LEFT_TRIGGER = int(os.environ.get("PS_LINUX_AXIS_L2", "2"))
LINUX_DUALSENSE_AXIS_RIGHT_TRIGGER = int(os.environ.get("PS_LINUX_AXIS_R2", "5"))
LINUX_DUALSENSE_AXIS_DPAD_X = int(os.environ.get("PS_LINUX_AXIS_DPAD_X", "6"))
LINUX_DUALSENSE_AXIS_DPAD_Y = int(os.environ.get("PS_LINUX_AXIS_DPAD_Y", "7"))
LINUX_DUALSENSE_BTN_L1 = int(os.environ.get("PS_LINUX_BTN_L1", "4"))
LINUX_DUALSENSE_BTN_R1 = int(os.environ.get("PS_LINUX_BTN_R1", "5"))


def jsiocgname(length: int) -> int:
    return 0x80006A13 + (length << 16)


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
    left_trigger: int
    right_trigger: int
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


class TriggerAxis:
    def __init__(self, idle_value: float) -> None:
        self.idle_value = idle_value

    def value(self, raw_value: float) -> float:
        if self.idle_value <= 0.5:
            denom = max(1.0 - self.idle_value, 0.001)
            value = clamp((raw_value - self.idle_value) / denom, 0.0, 1.0)
        else:
            denom = max(self.idle_value + 1.0, 0.001)
            value = clamp((self.idle_value - raw_value) / denom, 0.0, 1.0)

        if value <= TRIGGER_DEADZONE:
            return 0.0

        return clamp((value - TRIGGER_DEADZONE) / (1.0 - TRIGGER_DEADZONE), 0.0, 1.0)


class LinuxJoystick:
    uses_pygame = False

    def __init__(self, device_path: str, name: str) -> None:
        self.device_path = device_path
        self.name = name
        self.file = open(device_path, "rb", buffering=0)
        self.axis_count = self._read_u8_ioctl(JSIOCGAXES)
        self.button_count = self._read_u8_ioctl(JSIOCGBUTTONS)
        self.axes = [0.0] * self.axis_count
        self.buttons = [False] * self.button_count
        self.stop_event = threading.Event()
        self.reader = threading.Thread(target=self._reader_loop, name="linux-joystick", daemon=True)
        self.reader.start()

    def _read_u8_ioctl(self, request: int) -> int:
        buf = array.array("B", [0])
        fcntl.ioctl(self.file.fileno(), request, buf, True)
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

    def get_axis(self, axis_index: int) -> float:
        if axis_index < 0 or axis_index >= self.axis_count:
            return 0.0
        return self.axes[axis_index]

    def get_button(self, button_index: int) -> bool:
        if button_index < 0 or button_index >= self.button_count:
            return False
        return self.buttons[button_index]


class BackgroundActions:
    def __init__(self, link: EspPiControlLink, supervisor: MecanumIrStepperSupervisor) -> None:
        self.link = link
        self.supervisor = supervisor
        self.lock = threading.Lock()
        self.stepper_hold_thread: threading.Thread | None = None
        self.stepper_hold_stop: threading.Event | None = None
        self.stepper_hold_direction: int | None = None

    def close(self) -> None:
        self.set_stepper_hold(None)

    def set_stepper_hold(self, direction: int | None) -> None:
        if direction == self.stepper_hold_direction and self.stepper_hold_thread is not None and self.stepper_hold_thread.is_alive():
            return

        self._stop_stepper_hold()
        if direction is None:
            return

        if not self.lock.acquire(blocking=False):
            return

        stop_event = threading.Event()
        self.stepper_hold_stop = stop_event
        self.stepper_hold_direction = direction

        def wrapped() -> None:
            try:
                self._stepper_hold_worker(direction, stop_event)
            finally:
                self.stepper_hold_direction = None
                self.stepper_hold_stop = None
                self.stepper_hold_thread = None
                self.lock.release()

        thread = threading.Thread(target=wrapped, name="stepper_hold", daemon=True)
        self.stepper_hold_thread = thread
        thread.start()

    def home_stepper(self) -> None:
        self.set_stepper_hold(None)
        self._start_thread("stepper_home", self._home_stepper_worker)

    def traffic_dance(self) -> None:
        self._start_thread("traffic_dance", self._traffic_dance_worker)

    def _stop_stepper_hold(self) -> None:
        stop_event = self.stepper_hold_stop
        thread = self.stepper_hold_thread
        if stop_event is None or thread is None:
            self.stepper_hold_direction = None
            return

        stop_event.set()
        thread.join(timeout=0.5)
        self.stepper_hold_direction = None

    def _start_thread(self, name: str, target, *args) -> None:
        if not self.lock.acquire(blocking=False):
            print(f"{name} | busy")
            return

        def wrapped() -> None:
            try:
                target(*args)
            finally:
                self.lock.release()

        thread = threading.Thread(target=wrapped, name=name, daemon=True)
        thread.start()

    def _stepper_hold_worker(self, direction: int, stop_event: threading.Event) -> None:
        text = "up" if direction < 0 else "down"
        print(f"stepper | hold {text}")
        self.supervisor.stepper.run_until_released(
            direction,
            stop_event,
            self.supervisor.ir_stop_latched,
        )
        print(f"stepper | hold {text} stopped")

    def _home_stepper_worker(self) -> None:
        print(f"home | checking ESP limit switch on GPIO {HOMING_LIMIT_PIN}")
        try:
            if self._limit_pressed():
                print("home | already on limit")
                return
        except RuntimeError as exc:
            print(f"home | unavailable: {exc}")
            return

        moved = 0
        while moved < HOMING_MAX_STEPS and not self.supervisor.ir_stop_latched.is_set():
            self.supervisor.stepper.run_steps(
                HOMING_CHUNK_STEPS,
                STEPPER_DOWN_DIR,
                self.supervisor.ir_stop_latched,
            )
            moved += HOMING_CHUNK_STEPS

            if self._limit_pressed():
                print(f"home | limit reached after {moved} steps")
                return

        print("home | stopped before limit was reached")

    def _limit_pressed(self) -> bool:
        seq = self.link.send_command(f"LIMIT_STATUS pin={HOMING_LIMIT_PIN}")
        try:
            data = self.link.wait_for(seq, {"ack", "limit"}, timeout=ACK_TIMEOUT_S)
        except TimeoutError as exc:
            raise RuntimeError("LIMIT_STATUS timed out. Check UART wiring/power and confirm the ESP sketch is running.") from exc

        if data.get("type") == "limit":
            return bool(data.get("pressed", False))

        if data.get("cmd") == "LIMIT_STATUS" and data.get("ok", False):
            return bool(data.get("pressed", False))

        message = data.get("message", "ESP sketch does not support LIMIT_STATUS")
        raise RuntimeError(message)

    def _traffic_dance_worker(self) -> None:
        print("traffic | dance")
        pattern = [
            (False, False, True, 0.12),
            (False, True, False, 0.12),
            (True, False, False, 0.12),
            (True, True, True, 0.12),
            (False, False, False, 0.10),
        ]
        for _ in range(5):
            for yellow, red, green, duration_s in pattern:
                self.supervisor.indicators.set_lights(yellow=yellow, red=red, green=green)
                time.sleep(duration_s)

        self.supervisor.indicators.set_lights(yellow=False, red=False, green=False)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def apply_deadzone(value: float, deadzone: float = STICK_DEADZONE) -> float:
    if abs(value) <= deadzone:
        return 0.0

    sign = 1.0 if value > 0.0 else -1.0
    return sign * clamp((abs(value) - deadzone) / (1.0 - deadzone), 0.0, 1.0)


def get_axis(joystick, axis_index: int) -> float:
    if axis_index < 0 or axis_index >= joystick.get_numaxes():
        return 0.0
    return float(joystick.get_axis(axis_index))


def get_button(joystick, button_index: int) -> bool:
    if button_index < 0 or button_index >= joystick.get_numbuttons():
        return False
    return bool(joystick.get_button(button_index))


def prompt_ir_enabled() -> bool:
    while True:
        raw = input("IR sensors included? (y/n): ").strip().lower()
        if raw in {"y", "yes"}:
            return True
        if raw in {"n", "no"}:
            return False
        print("input error | enter y or n")


def axis_to_cardinal(value: float) -> int:
    if value >= DPAD_AXIS_THRESHOLD:
        return 1
    if value <= -DPAD_AXIS_THRESHOLD:
        return -1
    return 0


def controller_mapping_for(joystick) -> ControllerMapping:
    mapping = ControllerMapping(
        left_x=AXIS_LEFT_X,
        left_y=AXIS_LEFT_Y,
        right_x=AXIS_RIGHT_X,
        left_trigger=AXIS_LEFT_TRIGGER,
        right_trigger=AXIS_RIGHT_TRIGGER,
        dpad_x=AXIS_DPAD_X,
        dpad_y=AXIS_DPAD_Y,
        button_cross=BTN_CROSS,
        button_circle=BTN_CIRCLE,
        button_square=BTN_SQUARE,
        button_triangle=BTN_TRIANGLE,
        button_l1=BTN_L1,
        button_r1=BTN_R1,
    )

    if isinstance(joystick, LinuxJoystick) and score_controller_name(joystick.get_name()) <= 1:
        return ControllerMapping(
            left_x=AXIS_LEFT_X,
            left_y=AXIS_LEFT_Y,
            right_x=LINUX_DUALSENSE_AXIS_RIGHT_X,
            left_trigger=LINUX_DUALSENSE_AXIS_LEFT_TRIGGER,
            right_trigger=LINUX_DUALSENSE_AXIS_RIGHT_TRIGGER,
            dpad_x=LINUX_DUALSENSE_AXIS_DPAD_X,
            dpad_y=LINUX_DUALSENSE_AXIS_DPAD_Y,
            button_cross=BTN_CROSS,
            button_circle=BTN_CIRCLE,
            button_square=BTN_SQUARE,
            button_triangle=BTN_TRIANGLE,
            button_l1=LINUX_DUALSENSE_BTN_L1,
            button_r1=LINUX_DUALSENSE_BTN_R1,
        )

    return mapping


def quantized_translation(left_x: float, left_y: float, speed_rpm: float) -> tuple[float, float]:
    x = apply_deadzone(-left_x)
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


def joystick_to_twist(controller: ControllerState, speed_rpm: float) -> Twist:
    forward, strafe = quantized_translation(
        controller.centered_axis(controller.mapping.left_x),
        controller.centered_axis(controller.mapping.left_y),
        speed_rpm,
    )

    turn_axis = apply_deadzone(controller.centered_axis(controller.mapping.right_x))
    if turn_axis == 0.0:
        turn = 0.0
    else:
        turn = math.copysign(MIN_RPM + abs(turn_axis) * (speed_rpm - MIN_RPM), turn_axis)

    forward, strafe, turn = limit_combined_twist(forward, strafe, turn, speed_rpm)
    return Twist(forward, strafe, turn)


def dpad_to_twist(controller: ControllerState, speed_rpm: float) -> Twist | None:
    joystick = controller.joystick
    if getattr(joystick, "uses_pygame", True) and hasattr(joystick, "get_numhats") and joystick.get_numhats() > 0:
        dpad_x, dpad_y = joystick.get_hat(0)
    else:
        dpad_x = axis_to_cardinal(get_axis(joystick, controller.mapping.dpad_x))
        dpad_y = -axis_to_cardinal(get_axis(joystick, controller.mapping.dpad_y))

    if dpad_y > 0:
        return Twist(speed_rpm, 0.0, 0.0)
    if dpad_y < 0:
        return Twist(-speed_rpm, 0.0, 0.0)
    if dpad_x < 0:
        return Twist(0.0, speed_rpm, 0.0)
    if dpad_x > 0:
        return Twist(0.0, -speed_rpm, 0.0)
    return None


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


def send_twist(link: EspPiControlLink, twist: Twist) -> None:
    seq = link.send_command(
        f"TWIST forward={twist.forward_rpm:.3f} "
        f"strafe={twist.strafe_rpm:.3f} "
        f"turn={twist.turn_rpm:.3f} "
        f"timeout={TWIST_TIMEOUT_MS}"
    )
    ack = link.wait_for(seq, {"ack"}, timeout=TELEOP_ACK_TIMEOUT_S)
    require_ok_ack(ack)


def send_stop_no_throw(link: EspPiControlLink) -> None:
    try:
        link.send_command("STOP")
    except Exception as exc:
        print(f"stop warning | {exc}")


def score_controller_name(name: str) -> int:
    lower = name.lower()
    if "dualsense" in lower or "ps5" in lower:
        return 0
    if "dualshock" in lower or "ps4" in lower or "wireless controller" in lower:
        return 1
    if "motion sensor" in lower or "touchpad" in lower:
        return 3
    return 2


def find_linux_joystick() -> LinuxJoystick | None:
    candidates: list[tuple[int, str, str]] = []
    for device_path in sorted(glob.glob("/dev/input/js*")):
        sys_name_path = Path(f"/sys/class/input/{Path(device_path).name}/device/name")
        try:
            name = sys_name_path.read_text(encoding="utf-8").strip()
        except OSError:
            name = Path(device_path).name
        candidates.append((score_controller_name(name), device_path, name))

    if not candidates:
        return None

    _, device_path, name = sorted(candidates, key=lambda item: (item[0], item[1]))[0]
    return LinuxJoystick(device_path, name)


def select_controller(pygame):
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    if count <= 0:
        joystick = find_linux_joystick()
        if joystick is None:
            raise RuntimeError("no PS controller detected")
        print(f"controller | using {joystick.get_name()} via {joystick.device_path}")
        print(f"controller | axes={joystick.get_numaxes()} buttons={joystick.get_numbuttons()}")
        return joystick

    candidates = []
    for index in range(count):
        joystick = pygame.joystick.Joystick(index)
        joystick.init()
        name = joystick.get_name()
        score = score_controller_name(name)
        candidates.append((score, index, joystick, name))

    candidates.sort(key=lambda item: (item[0], item[1]))
    _, _, joystick, name = candidates[0]
    print(f"controller | using {name}")
    print(f"controller | axes={joystick.get_numaxes()} buttons={joystick.get_numbuttons()}")
    return joystick


def update_speed(speed_rpm: float, l2: TriggerAxis, r2: TriggerAxis, controller: ControllerState, dt: float) -> float:
    decrease = l2.value(get_axis(controller.joystick, controller.mapping.left_trigger))
    increase = r2.value(get_axis(controller.joystick, controller.mapping.right_trigger))
    delta = (increase - decrease) * SPEED_CHANGE_RPM_PER_S * dt
    return clamp(speed_rpm + delta, MIN_RPM, MAX_RPM)


def print_controls() -> None:
    print("controls")
    print("  left stick  | 8-way translate")
    print("  d-pad       | cardinal translate override")
    print("  right stick | rotate")
    print("  R2/L2       | increase/decrease speed")
    print("  L1/R1       | hold for stepper up/down, release to stop")
    print("  triangle    | reinit IMU")
    print("  circle      | honk")
    print("  square      | home stepper using ESP GPIO 23 limit command")
    print("  cross       | traffic light dance")


def main() -> None:
    try:
        import pygame
    except ImportError as exc:
        raise SystemExit("pygame is required. Install it on the Pi with: sudo apt install python3-pygame") from exc

    pygame.init()
    ir_enabled = prompt_ir_enabled()
    joystick = select_controller(pygame)
    mapping = controller_mapping_for(joystick)
    controller = ControllerState(joystick, mapping)
    print_controls()
    print(f"IR mode | {'enabled' if ir_enabled else 'disabled'}")
    print(
        "controller | mapping "
        f"lx={mapping.left_x} ly={mapping.left_y} "
        f"rx={mapping.right_x} l2={mapping.left_trigger} r2={mapping.right_trigger} dpadx={mapping.dpad_x} dpady={mapping.dpad_y} "
        f"cross={mapping.button_cross} circle={mapping.button_circle} square={mapping.button_square} "
        f"triangle={mapping.button_triangle} l1={mapping.button_l1} r1={mapping.button_r1}"
    )

    l2 = TriggerAxis(get_axis(joystick, mapping.left_trigger))
    r2 = TriggerAxis(get_axis(joystick, mapping.right_trigger))
    edges = ButtonEdges()

    link = EspPiControlLink(PORT, BAUD)
    supervisor: MecanumIrStepperSupervisor | None = None
    actions: BackgroundActions | None = None
    speed_rpm = DEFAULT_RPM
    last_send_s = 0.0
    last_speed_print_s = 0.0
    was_moving = False
    honk_started_s: float | None = None

    try:
        print(f"opened {PORT} @ {BAUD}")
        link.drain(1.2)
        initialize_robot(link)
        data = request_status(link)
        print(telemetry_summary(data))

        supervisor = MecanumIrStepperSupervisor(link, ir_enabled=ir_enabled)
        actions = BackgroundActions(link, supervisor)
        print(supervisor.ir_monitor.status_text())
        print(supervisor.stepper.status_text())
        if ir_enabled:
            print("IR note | startup baseline was captured when the script started, so begin with the path clear.")
        else:
            print("IR note | disabled for this run")

        last_loop_s = time.time()
        while True:
            if getattr(joystick, "uses_pygame", True):
                pygame.event.pump()
            now_s = time.time()
            dt = max(now_s - last_loop_s, 0.001)
            last_loop_s = now_s

            speed_rpm = update_speed(speed_rpm, l2, r2, controller, dt)
            if now_s - last_speed_print_s >= 1.0:
                print(f"speed | {speed_rpm:.1f} rpm")
                last_speed_print_s = now_s

            if edges.pressed(joystick, mapping.button_triangle):
                print("imu | reinit")
                initialize_robot(link)

            l1_held = get_button(joystick, mapping.button_l1)
            r1_held = get_button(joystick, mapping.button_r1)
            if l1_held and not r1_held:
                actions.set_stepper_hold(STEPPER_UP_DIR)
            elif r1_held and not l1_held:
                actions.set_stepper_hold(STEPPER_DOWN_DIR)
            else:
                actions.set_stepper_hold(None)

            if edges.pressed(joystick, mapping.button_square):
                actions.home_stepper()

            if edges.pressed(joystick, mapping.button_cross):
                actions.traffic_dance()

            honking = get_button(joystick, mapping.button_circle)
            if honking and honk_started_s is None:
                honk_started_s = now_s
            if not honking:
                honk_started_s = None

            if ir_enabled and supervisor.ir_stop_latched.is_set():
                if was_moving:
                    send_stop_no_throw(link)
                    was_moving = False
                if not supervisor.ir_monitor.active_sensors():
                    supervisor.clear_latch()
                time.sleep(0.03)
                continue

            honk_allowed = honk_started_s is not None and now_s - honk_started_s <= BUZZER_HONK_MAX_S
            supervisor.indicators.set_buzzer(honk_allowed)

            twist = dpad_to_twist(controller, speed_rpm)
            if twist is None:
                twist = joystick_to_twist(controller, speed_rpm)
            if twist.active():
                if now_s - last_send_s >= TWIST_SEND_INTERVAL_S:
                    send_twist(link, twist)
                    supervisor.set_mecanum_active(True)
                    last_send_s = now_s
                    was_moving = True
            elif was_moving:
                send_stop_no_throw(link)
                supervisor.set_mecanum_active(False)
                was_moving = False

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nteleop | interrupted")
    finally:
        if actions is not None:
            actions.close()
        send_stop_no_throw(link)
        if supervisor is not None:
            supervisor.close()
        link.close()
        if hasattr(joystick, "close"):
            joystick.close()
        pygame.quit()


if __name__ == "__main__":
    main()
