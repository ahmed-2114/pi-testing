#!/usr/bin/env python3
"""GPIO output services for Audix buzzer and scissor-lift stepper."""

from __future__ import annotations

import threading
import time
from typing import Any

import rclpy
from audix_interfaces.msg import IrState
from audix_interfaces.srv import LiftMoveSteps
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool


STEPPER_STEP_PIN = 6
STEPPER_DIR_PIN = 13
STEPPER_EN_PIN = 5
STEPPER_STEP_HIGH_US = 10
STEPPER_DIR_SETUP_S = 0.010
STEPPER_SPEED_SPS = 500.0
STEPPER_UP_DIR = 1
STEPPER_DOWN_DIR = -1
DEFAULT_BUZZER_PIN = 19
IR_SENSOR_ORDER = ("front_left", "front", "front_right", "right", "back", "left")
IR_PINS = {
    "front_left": 23,
    "front": 24,
    "front_right": 25,
    "right": 17,
    "back": 27,
    "left": 22,
}


class GpioIrBank:
    """GPIO IR reader using the same sensor pins and logic as pi_master.py."""

    def __init__(self, *, active_low: bool, pull_up: bool, mock: bool, logic: str, node: Node) -> None:
        self.active_low = active_low
        self.mock = mock
        self.logic = logic
        self.node = node
        self.devices: dict[str, Any] = {}
        self.baseline_raw: dict[str, bool] = {}

        if self.mock:
            return

        from gpiozero import DigitalInputDevice

        self.devices = {name: DigitalInputDevice(pin, pull_up=pull_up) for name, pin in IR_PINS.items()}
        time.sleep(0.05)
        self.baseline_raw = {name: bool(device.value) for name, device in self.devices.items()}
        baseline = " ".join(f"{name}={1 if value else 0}" for name, value in self.baseline_raw.items())
        self.node.get_logger().info(f"IR GPIO ready logic={self.logic} baseline {baseline}")

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
        self.devices = {}


class GpioHardware(Node):
    def __init__(self) -> None:
        super().__init__("gpio_hardware")
        self.callback_group = ReentrantCallbackGroup()
        self.mock_gpio = bool(self.declare_parameter("mock_gpio", False).value)
        self.ir_enabled = bool(self.declare_parameter("ir_enabled", True).value)
        self.mock_ir = bool(self.declare_parameter("mock_ir", False).value)
        self.ir_logic = str(self.declare_parameter("ir_logic", "baseline").value)
        self.ir_active_low = bool(self.declare_parameter("ir_active_low", False).value)
        self.ir_pull_up = bool(self.declare_parameter("ir_pull_up", False).value)
        self.ir_poll_period_s = float(self.declare_parameter("ir_poll_period_s", 0.05).value)
        self.base_frame_id = str(self.declare_parameter("base_frame_id", "base_link").value)
        self.buzzer_pin = int(self.declare_parameter("buzzer_pin", DEFAULT_BUZZER_PIN).value)
        self.buzzer_active_high = bool(self.declare_parameter("buzzer_active_high", True).value)
        self.step_pin = int(self.declare_parameter("step_pin", STEPPER_STEP_PIN).value)
        self.dir_pin = int(self.declare_parameter("dir_pin", STEPPER_DIR_PIN).value)
        self.en_pin = int(self.declare_parameter("en_pin", STEPPER_EN_PIN).value)
        self.default_speed_sps = float(self.declare_parameter("stepper_speed_sps", STEPPER_SPEED_SPS).value)
        self.step_high_us = int(self.declare_parameter("step_high_us", STEPPER_STEP_HIGH_US).value)

        self.buzzer: Any | None = None
        self.step_device: Any | None = None
        self.dir_device: Any | None = None
        self.en_device: Any | None = None
        self.ir_bank: GpioIrBank | None = None
        self.stepper_lock = threading.Lock()

        if self.mock_gpio:
            self.get_logger().warning("GPIO hardware node running in mock mode")
        else:
            self._open_gpio()

        self.create_service(SetBool, "gpio/set_buzzer", self._handle_set_buzzer, callback_group=self.callback_group)
        self.create_service(LiftMoveSteps, "lift/move_steps", self._handle_lift_move_steps, callback_group=self.callback_group)
        if self.ir_enabled:
            self.ir_bank = GpioIrBank(
                active_low=self.ir_active_low,
                pull_up=self.ir_pull_up,
                mock=self.mock_ir,
                logic=self.ir_logic,
                node=self,
            )
            self.ir_pub = self.create_publisher(IrState, "ir/state", 10)
            self.create_timer(max(0.01, self.ir_poll_period_s), self._publish_ir, callback_group=self.callback_group)
        self.get_logger().info("GPIO hardware services ready")

    def _open_gpio(self) -> None:
        from gpiozero import DigitalOutputDevice, OutputDevice

        self.buzzer = OutputDevice(
            self.buzzer_pin,
            active_high=self.buzzer_active_high,
            initial_value=False,
        )
        self.step_device = DigitalOutputDevice(self.step_pin, initial_value=False)
        self.dir_device = DigitalOutputDevice(self.dir_pin, initial_value=False)
        self.en_device = DigitalOutputDevice(self.en_pin, initial_value=True)
        self.get_logger().info(
            f"GPIO ready buzzer=GPIO{self.buzzer_pin} "
            f"STEP=GPIO{self.step_pin} DIR=GPIO{self.dir_pin} EN=GPIO{self.en_pin}"
        )

    def _handle_set_buzzer(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if self.mock_gpio:
            response.success = True
            response.message = f"mock buzzer {'on' if request.data else 'off'}"
            return response
        if self.buzzer is None:
            response.success = False
            response.message = "buzzer GPIO unavailable"
            return response
        if request.data:
            self.buzzer.on()
        else:
            self.buzzer.off()
        response.success = True
        response.message = "buzzer on" if request.data else "buzzer off"
        return response

    def _handle_lift_move_steps(
        self,
        request: LiftMoveSteps.Request,
        response: LiftMoveSteps.Response,
    ) -> LiftMoveSteps.Response:
        steps = abs(int(request.steps))
        logical_direction = 1 if int(request.direction) >= 0 else -1
        pin_direction = STEPPER_UP_DIR if logical_direction >= 0 else STEPPER_DOWN_DIR
        speed_sps = float(request.speed_sps) if request.speed_sps > 0.0 else self.default_speed_sps

        if steps <= 0:
            response.ok = True
            response.message = "steps=0"
            return response
        if self.mock_gpio:
            response.ok = True
            response.message = self._lift_message(steps, logical_direction, speed_sps, mock=True)
            return response
        if self.step_device is None or self.dir_device is None or self.en_device is None:
            response.ok = False
            response.message = "stepper GPIO unavailable"
            return response

        try:
            self._run_steps(steps, pin_direction, speed_sps)
        except Exception as exc:
            response.ok = False
            response.message = str(exc)
            return response

        response.ok = True
        response.message = self._lift_message(steps, logical_direction, speed_sps)
        return response

    @staticmethod
    def _lift_message(steps: int, logical_direction: int, speed_sps: float, *, mock: bool = False) -> str:
        signed_steps = steps if logical_direction >= 0 else -steps
        direction_name = "up" if logical_direction >= 0 else "down"
        prefix = "mock lift" if mock else "lift moved"
        return f"{prefix} jog={signed_steps} direction={direction_name} speed={speed_sps:.1f}"

    def _run_steps(self, steps: int, direction: int, speed_sps: float) -> None:
        interval_us = int(1_000_000.0 / max(1.0, speed_sps))
        high_us = max(1, self.step_high_us)
        interval_us = max(interval_us, high_us + 50)

        with self.stepper_lock:
            if direction >= 0:
                self.dir_device.on()
            else:
                self.dir_device.off()
            self.en_device.off()
            time.sleep(STEPPER_DIR_SETUP_S)
            try:
                for _ in range(steps):
                    self.step_device.on()
                    time.sleep(high_us / 1_000_000.0)
                    self.step_device.off()
                    time.sleep(max((interval_us - high_us) / 1_000_000.0, 0.0))
            finally:
                self.step_device.off()
                self.en_device.on()

    def _publish_ir(self) -> None:
        if self.ir_bank is None:
            return
        state = self.ir_bank.read()
        msg = IrState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame_id
        msg.front_left = state["front_left"]
        msg.front = state["front"]
        msg.front_right = state["front_right"]
        msg.right = state["right"]
        msg.back = state["back"]
        msg.left = state["left"]
        msg.active = [name for name in IR_SENSOR_ORDER if state.get(name, False)]
        self.ir_pub.publish(msg)

    def destroy_node(self) -> bool:
        for device, off_first in (
            (self.buzzer, True),
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
        if self.ir_bank is not None:
            self.ir_bank.close()
            self.ir_bank = None
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = GpioHardware()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
