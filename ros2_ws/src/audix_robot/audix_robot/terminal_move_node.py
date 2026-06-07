#!/usr/bin/env python3
"""Small terminal MOVE client for the Audix ROS bridge."""

from __future__ import annotations

import shlex

import rclpy
from audix_interfaces.srv import Move
from rclpy.node import Node
from std_srvs.srv import Trigger


DIRECTION_DEG = {
    "F": 0.0,
    "FORWARD": 0.0,
    "B": 180.0,
    "BACK": 180.0,
    "BACKWARD": 180.0,
    "R": -90.0,
    "RIGHT": -90.0,
    "L": 90.0,
    "LEFT": 90.0,
}


class TerminalMove(Node):
    def __init__(self) -> None:
        super().__init__("terminal_move")
        self.move_client = self.create_client(Move, "/audix/move")
        self.stop_client = self.create_client(Trigger, "/audix/esp/stop")
        self.reset_client = self.create_client(Trigger, "/audix/esp/reset_odom")
        self.init_imu_client = self.create_client(Trigger, "/audix/esp/init_imu")

    def wait_for_services(self) -> None:
        for name, client in (
            ("/audix/move", self.move_client),
            ("/audix/esp/stop", self.stop_client),
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

    def call_move(self, angle_deg: float, distance_cm: float, heading_deg: float, timeout_s: float) -> None:
        request = Move.Request()
        request.angle_deg = float(angle_deg)
        request.distance_m = max(0.0, float(distance_cm)) / 100.0
        request.heading_deg = float(heading_deg)
        request.timeout_s = float(timeout_s)
        request.wait_for_done = True

        print(
            f"move: angle={request.angle_deg:.1f} dist={distance_cm:.1f}cm heading={heading_deg:.1f}",
            flush=True,
        )
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


def print_help() -> None:
    print(
        """
Commands:
  F 20              move forward 20 cm
  B 10              move backward 10 cm
  R 15              strafe right 15 cm
  L 15              strafe left 15 cm
  F 20 0            move with explicit heading target
  turn 90           turn to heading 90 deg
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
            if cmd in ("TURN", "T"):
                if len(parts) < 2:
                    print("usage: turn <heading_deg>", flush=True)
                    continue
                node.call_move(0.0, 0.0, float(parts[1]), 10.0)
                continue
            if cmd not in DIRECTION_DEG:
                print("unknown command; type help", flush=True)
                continue
            if len(parts) < 2:
                print(f"usage: {parts[0]} <distance_cm> [heading_deg]", flush=True)
                continue

            heading_deg = float(parts[2]) if len(parts) >= 3 else 0.0
            distance_cm = float(parts[1])
            timeout_s = max(5.0, abs(distance_cm) * 0.5)
            node.call_move(DIRECTION_DEG[cmd], distance_cm, heading_deg, timeout_s)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
