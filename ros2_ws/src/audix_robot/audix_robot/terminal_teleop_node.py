#!/usr/bin/env python3
"""Line-based terminal teleop client for the Audix ROS bridge."""

from __future__ import annotations

import rclpy
from audix_interfaces.srv import TwistCommand
from rclpy.node import Node
from std_srvs.srv import Trigger


class TerminalTeleop(Node):
    def __init__(self) -> None:
        super().__init__("terminal_teleop")
        self.declare_parameter("rpm", 25.0)
        self.declare_parameter("timeout_s", 0.3)
        self.twist_client = self.create_client(TwistCommand, "/audix/twist")
        self.stop_client = self.create_client(Trigger, "/audix/manager/stop")
        self.reset_client = self.create_client(Trigger, "/audix/esp/reset_odom")
        self.init_imu_client = self.create_client(Trigger, "/audix/esp/init_imu")

    @property
    def rpm(self) -> float:
        return float(self.get_parameter("rpm").value)

    @property
    def timeout_s(self) -> float:
        return float(self.get_parameter("timeout_s").value)

    def wait_for_services(self) -> None:
        for name, client in (
            ("/audix/twist", self.twist_client),
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

    def send_twist(self, forward: float, strafe: float, turn: float) -> None:
        request = TwistCommand.Request()
        request.forward_rpm = float(forward)
        request.strafe_rpm = float(strafe)
        request.turn_rpm = float(turn)
        request.timeout_s = self.timeout_s
        future = self.twist_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None or not result.ok:
            message = "no response" if result is None else result.message
            print(f"twist failed: {message}", flush=True)
            return
        print(
            f"twist: f={forward:.1f}rpm s={strafe:.1f}rpm t={turn:.1f}rpm",
            flush=True,
        )


def print_help(rpm: float) -> None:
    print(
        f"""
Commands use {rpm:.1f} RPM. Press Enter after each command:
  w/s   forward/back
  a/d   strafe left/right
  q/e   rotate left/right
  x     stop
  r     reset odometry
  i     calibrate/zero IMU
  help  show this help
  exit  quit
""".strip(),
        flush=True,
    )


def main() -> None:
    rclpy.init()
    node = TerminalTeleop()
    try:
        node.wait_for_services()
        print_help(node.rpm)
        while rclpy.ok():
            try:
                raw = input("teleop> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not raw:
                continue
            cmd = raw.split()[0]
            rpm = node.rpm
            if cmd in ("exit", "quit"):
                break
            if cmd in ("help", "h", "?"):
                print_help(rpm)
            elif cmd == "w":
                node.send_twist(rpm, 0.0, 0.0)
            elif cmd == "s":
                node.send_twist(-rpm, 0.0, 0.0)
            elif cmd == "a":
                node.send_twist(0.0, rpm, 0.0)
            elif cmd == "d":
                node.send_twist(0.0, -rpm, 0.0)
            elif cmd == "q":
                node.send_twist(0.0, 0.0, rpm)
            elif cmd == "e":
                node.send_twist(0.0, 0.0, -rpm)
            elif cmd == "x":
                node.call_trigger(node.stop_client, "stop")
            elif cmd == "r":
                node.call_trigger(node.reset_client, "reset odom")
            elif cmd == "i":
                node.call_trigger(node.init_imu_client, "init imu")
            else:
                print("unknown command; type help", flush=True)
    finally:
        try:
            node.call_trigger(node.stop_client, "stop")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
