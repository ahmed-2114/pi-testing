#!/usr/bin/env python3
"""Small terminal MOVE client for the Audix ROS bridge."""

from __future__ import annotations

import shlex

import rclpy
from audix_interfaces.srv import DirectionCommand, RotateCommand
from rclpy.node import Node
from std_srvs.srv import Trigger


DIRECTION_ALIASES = {
    "F": "F",
    "FORWARD": "F",
    "B": "B",
    "BACK": "B",
    "BACKWARD": "B",
    "R": "R",
    "RIGHT": "R",
    "L": "L",
    "LEFT": "L",
    "FR": "FR",
    "FORWARD_RIGHT": "FR",
    "FL": "FL",
    "FORWARD_LEFT": "FL",
    "BR": "BR",
    "BACK_RIGHT": "BR",
    "BL": "BL",
    "BACK_LEFT": "BL",
}


class TerminalMove(Node):
    def __init__(self) -> None:
        super().__init__("terminal_move")
        self.move_client = self.create_client(DirectionCommand, "/audix/manager/direction_move")
        self.rotate_client = self.create_client(RotateCommand, "/audix/manager/rotate")
        self.stop_client = self.create_client(Trigger, "/audix/manager/stop")
        self.reset_client = self.create_client(Trigger, "/audix/esp/reset_odom")
        self.init_imu_client = self.create_client(Trigger, "/audix/esp/init_imu")

    def wait_for_services(self) -> None:
        for name, client in (
            ("/audix/manager/direction_move", self.move_client),
            ("/audix/manager/rotate", self.rotate_client),
            ("/audix/manager/stop", self.stop_client),
            ("/audix/esp/reset_odom", self.reset_client),
            ("/audix/esp/init_imu", self.init_imu_client),
        ):
            while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
                print(f"waiting for {name} ...", flush=True)

    def call_trigger(self, client, label: str) -> None:
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            print(f"{label} failed: no response", flush=True)
            return
        print(f"{label}: {'ok' if result.success else 'failed'} | {result.message}", flush=True)

    def call_move(self, direction: str, distance_cm: float, timeout_s: float) -> None:
        request = DirectionCommand.Request()
        request.direction = direction
        request.distance_cm = max(0.0, float(distance_cm))
        request.timeout_s = float(timeout_s)

        print(f"move: {direction} dist={distance_cm:.1f}cm", flush=True)
        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            print("move failed: no response", flush=True)
            return
        print(
            f"done: ok={result.ok} result={result.result} "
            f"forward={result.forward_cm:.1f}cm strafe={result.strafe_cm:.1f}cm "
            f"heading={result.heading_deg:.1f}deg",
            flush=True,
        )
        if result.message and result.message != result.result:
            print(f"message: {result.message}", flush=True)

    def call_rotate(self, direction: str, degrees: float) -> None:
        request = RotateCommand.Request()
        request.direction = direction
        request.degrees = abs(float(degrees))
        request.timeout_s = 10.0
        future = self.rotate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            print("rotate failed: no response", flush=True)
            return
        print(
            f"rotate: ok={result.ok} result={result.result} heading={result.heading_deg:.1f}deg",
            flush=True,
        )


def print_help() -> None:
    print(
        """
Commands:
  F 20              move forward 20 cm
  B 10              move backward 10 cm
  R 15              strafe right 15 cm
  L 15              strafe left 15 cm
  FR 20             diagonal forward-right 20 cm
  FL 20             diagonal forward-left 20 cm
  BR 20             diagonal back-right 20 cm
  BL 20             diagonal back-left 20 cm
  rotl 90           rotate left in place 90 deg
  rotr 90           rotate right in place 90 deg
  stop              stop the robot
  reset             reset odometry
  init              calibrate/zero IMU
  help              show this help
  q                 quit
""".strip(),
        flush=True,
    )


def main() -> None:
    rclpy.init()
    node = TerminalMove()
    try:
        node.wait_for_services()
        print_help()
        while rclpy.ok():
            try:
                raw = input("move> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not raw:
                continue

            try:
                parts = shlex.split(raw)
            except ValueError as exc:
                print(f"parse error: {exc}", flush=True)
                continue
            if not parts:
                continue

            cmd = parts[0].upper()
            if cmd in ("Q", "QUIT", "EXIT"):
                break
            if cmd in ("H", "HELP", "?"):
                print_help()
                continue
            if cmd == "STOP":
                node.call_trigger(node.stop_client, "stop")
                continue
            if cmd in ("RESET", "RESET_ODOM"):
                node.call_trigger(node.reset_client, "reset odom")
                continue
            if cmd in ("INIT", "INIT_IMU"):
                node.call_trigger(node.init_imu_client, "init imu")
                continue
            if cmd in ("ROTL", "ROTATE_LEFT"):
                if len(parts) < 2:
                    print("usage: rotl <degrees>", flush=True)
                    continue
                node.call_rotate("left", float(parts[1]))
                continue
            if cmd in ("ROTR", "ROTATE_RIGHT"):
                if len(parts) < 2:
                    print("usage: rotr <degrees>", flush=True)
                    continue
                node.call_rotate("right", float(parts[1]))
                continue
            if cmd not in DIRECTION_ALIASES:
                print("unknown command; type help", flush=True)
                continue
            if len(parts) < 2:
                print(f"usage: {parts[0]} <distance_cm>", flush=True)
                continue

            distance_cm = float(parts[1])
            timeout_s = max(5.0, abs(distance_cm) * 0.5)
            node.call_move(DIRECTION_ALIASES[cmd], distance_cm, timeout_s)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
