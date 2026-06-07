#!/usr/bin/env python3
"""GPIO output services for Audix buzzer and scissor-lift stepper."""

from __future__ import annotations

import threading
import time
from typing import Any

import rclpy
from audix_interfaces.srv import LiftMoveSteps
from rclpy.node import Node
from std_srvs.srv import SetBool


STEPPER_STEP_PIN = 6
STEPPER_DIR_PIN = 13
STEPPER_EN_PIN = 5
STEPPER_STEP_HIGH_US = 10
STEPPER_DIR_SETUP_S = 0.010
STEPPER_SPEED_SPS = 275.0
STEPPER_UP_DIR = 1
STEPPER_DOWN_DIR = -1
DEFAULT_BUZZER_PIN = 19


class GpioHardware(Node):
    def __init__(self) -> None:
        super().__init__("gpio_hardware")
        self.mock_gpio = bool(self.declare_parameter("mock_gpio", False).value)
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
        self.stepper_lock = threading.Lock()

        if self.mock_gpio:
            self.get_logger().warning("GPIO hardware node running in mock mode")
        else:
            self._open_gpio()

        self.create_service(SetBool, "gpio/set_buzzer", self._handle_set_buzzer)
        self.create_service(LiftMoveSteps, "lift/move_steps", self._handle_lift_move_steps)
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
        direction = STEPPER_UP_DIR if int(request.direction) >= 0 else STEPPER_DOWN_DIR
        speed_sps = float(request.speed_sps) if request.speed_sps > 0.0 else self.default_speed_sps

        if steps <= 0:
            response.ok = True
            response.message = "steps=0"
            return response
        if self.mock_gpio:
            response.ok = True
            response.message = f"mock lift steps={steps} direction={direction} speed={speed_sps:.1f}"
            return response
        if self.step_device is None or self.dir_device is None or self.en_device is None:
            response.ok = False
            response.message = "stepper GPIO unavailable"
            return response

        try:
            self._run_steps(steps, direction, speed_sps)
        except Exception as exc:
            response.ok = False
            response.message = str(exc)
            return response

        response.ok = True
        response.message = f"lift moved steps={steps} direction={direction}"
        return response

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
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = GpioHardware()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
