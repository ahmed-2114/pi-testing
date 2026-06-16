#!/usr/bin/env python3
"""ROS-facing base services for the Audix ESP32 micro-ROS firmware.

This node replaces the old serial JSON bridge. The serial RX/TX link now belongs
to micro_ros_agent; this node only translates between existing Audix services and
the ESP32 micro-ROS topics.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any

import rclpy
from audix_robot.navigation_contract import body_delta_to_world
from audix_interfaces.msg import EspTelemetry
from audix_interfaces.srv import Move, RawCommand, TwistCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


DEFAULT_MOVE_TIMEOUT_S = 180.0


class WorldOdomAccumulator:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.forward_cm = 0.0
        self.strafe_cm = 0.0
        self.last_body_forward_cm: float | None = None
        self.last_body_strafe_cm: float | None = None

    def update(self, body_forward_cm: float, body_strafe_cm: float, yaw_deg: float) -> tuple[float, float]:
        body_forward_cm = float(body_forward_cm)
        body_strafe_cm = float(body_strafe_cm)

        if self.last_body_forward_cm is None or self.last_body_strafe_cm is None:
            self.last_body_forward_cm = body_forward_cm
            self.last_body_strafe_cm = body_strafe_cm
            return self.forward_cm, self.strafe_cm

        if (
            abs(body_forward_cm) <= 0.5
            and abs(body_strafe_cm) <= 0.5
            and (
                abs(self.last_body_forward_cm) > 5.0
                or abs(self.last_body_strafe_cm) > 5.0
            )
        ):
            self.reset()
            self.last_body_forward_cm = body_forward_cm
            self.last_body_strafe_cm = body_strafe_cm
            return self.forward_cm, self.strafe_cm

        delta_forward_cm = body_forward_cm - self.last_body_forward_cm
        delta_strafe_cm = body_strafe_cm - self.last_body_strafe_cm
        self.last_body_forward_cm = body_forward_cm
        self.last_body_strafe_cm = body_strafe_cm

        world_forward_cm, world_strafe_cm = body_delta_to_world(delta_forward_cm, delta_strafe_cm, yaw_deg)
        self.forward_cm += world_forward_cm
        self.strafe_cm += world_strafe_cm
        return self.forward_cm, self.strafe_cm


class AudixMicroRosBase(Node):
    def __init__(self) -> None:
        super().__init__("audix_micro_ros_base")
        self.callback_group = ReentrantCallbackGroup()

        self.frame_id = str(self.declare_parameter("frame_id", "odom").value)
        self.base_frame_id = str(self.declare_parameter("base_frame_id", "base_link").value)
        self.move_timeout_s = float(self.declare_parameter("move_timeout_s", DEFAULT_MOVE_TIMEOUT_S).value)
        self.init_imu_on_start = bool(self.declare_parameter("init_imu_on_start", True).value)
        self.reset_odom_on_start = bool(self.declare_parameter("reset_odom_on_start", True).value)

        self.seq = 1
        self.latest_data: dict[str, Any] | None = None
        self.latest_raw = ""
        self.latest_telemetry_time_s = 0.0
        self.completed_moves: dict[int, dict[str, Any]] = {}
        self.active_move_seqs: set[int] = set()
        self.cancelled_moves: set[int] = set()
        self.telemetry_condition = threading.Condition()
        self.world_odom = WorldOdomAccumulator()

        self.telemetry_pub = self.create_publisher(EspTelemetry, "esp/telemetry", 10)
        self.telemetry_raw_pub = self.create_publisher(String, "esp/telemetry_raw", 10)

        esp_command_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        esp_telemetry_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.move_goal_pub = self.create_publisher(String, "esp/move_goal", esp_command_qos)
        self.create_subscription(
            String,
            "esp/telemetry_json",
            self._on_telemetry_json,
            esp_telemetry_qos,
            callback_group=self.callback_group,
        )

        self.create_service(Trigger, "esp/ping", self._handle_ping, callback_group=self.callback_group)
        self.create_service(Trigger, "esp/init_imu", self._handle_init_imu, callback_group=self.callback_group)
        self.create_service(Trigger, "esp/reset_odom", self._handle_reset_odom, callback_group=self.callback_group)
        self.create_service(
            Trigger,
            "esp/reset_encoders",
            self._handle_reset_encoders,
            callback_group=self.callback_group,
        )
        self.create_service(Trigger, "esp/stop", self._handle_stop, callback_group=self.callback_group)
        self.create_service(Move, "move", self._handle_move, callback_group=self.callback_group)
        self.create_service(TwistCommand, "twist", self._handle_twist, callback_group=self.callback_group)
        self.create_service(RawCommand, "esp/raw_command", self._handle_raw_command, callback_group=self.callback_group)

        self.startup_timer = self.create_timer(1.0, self._startup_once, callback_group=self.callback_group)
        self.get_logger().info("Audix micro-ROS base services ready")

    def _startup_once(self) -> None:
        self.startup_timer.cancel()
        if self.init_imu_on_start:
            self._publish_esp_command("INIT_IMU")
        if self.reset_odom_on_start:
            self.world_odom.reset()
            self._publish_esp_command("RESET_ODOM")

    def _next_seq(self) -> int:
        value = self.seq
        self.seq += 1
        if self.seq > 2_000_000_000:
            self.seq = 1
        return value

    def _on_telemetry_json(self, msg: String) -> None:
        raw = msg.data
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            self.get_logger().warning("ignored malformed ESP telemetry JSON")
            return
        if not isinstance(data, dict):
            return

        self._update_world_pose(data)
        now = time.monotonic()
        with self.telemetry_condition:
            self.latest_data = data
            self.latest_raw = raw
            self.latest_telemetry_time_s = now
            seq = int(data.get("seq", 0))
            move = data.get("move", {})
            if seq and (bool(move.get("done", False)) or data.get("type") == "done"):
                self._remember_completed_move(seq, data)
            self.telemetry_condition.notify_all()

        self._publish_telemetry(data, raw)

    def _update_world_pose(self, data: dict[str, Any]) -> None:
        pose = data.get("pose", {})
        imu = data.get("imu", {})
        body_forward_cm = float(pose.get("forwardCm", data.get("forwardCm", 0.0)))
        body_strafe_cm = float(pose.get("strafeCm", data.get("strafeCm", 0.0)))
        yaw_deg = float(imu.get("yawDeg", pose.get("yawDeg", data.get("yawDeg", 0.0))))
        forward_cm, strafe_cm = self.world_odom.update(body_forward_cm, body_strafe_cm, yaw_deg)
        data["_audix_world_pose"] = {
            "forwardCm": forward_cm,
            "strafeCm": strafe_cm,
            "bodyForwardCm": body_forward_cm,
            "bodyStrafeCm": body_strafe_cm,
        }

    def _remember_completed_move(self, seq: int, data: dict[str, Any]) -> None:
        self.completed_moves[seq] = data
        if len(self.completed_moves) > 20:
            oldest = sorted(self.completed_moves)[:-20]
            for key in oldest:
                self.completed_moves.pop(key, None)

    def _publish_telemetry(self, data: dict[str, Any], raw: str) -> None:
        now = self.get_clock().now().to_msg()
        msg = EspTelemetry()
        msg.header.stamp = now
        msg.header.frame_id = self.base_frame_id
        msg.mode = str(data.get("mode", ""))
        msg.seq = int(data.get("seq", 0))

        imu = data.get("imu", {})
        pose = data.get("pose", {})
        move = data.get("move", {})
        msg.imu_ok = bool(imu.get("ok", False))
        msg.yaw_deg = float(imu.get("yawDeg", pose.get("yawDeg", 0.0)))
        msg.raw_yaw_deg = float(imu.get("rawYawDeg", 0.0))
        msg.gyro_dps = float(imu.get("gyroDps", 0.0))
        msg.gyro_filt_dps = float(imu.get("gyroFiltDps", 0.0))

        world_pose = data.get("_audix_world_pose", {})
        msg.forward_cm = float(world_pose.get("forwardCm", pose.get("forwardCm", 0.0)))
        msg.strafe_cm = float(world_pose.get("strafeCm", pose.get("strafeCm", 0.0)))
        msg.progress_cm = float(pose.get("progressCm", 0.0))
        msg.remaining_cm = float(pose.get("remainingCm", 0.0))
        msg.local_forward_cm = float(pose.get("localForwardCm", 0.0))
        msg.local_strafe_cm = float(pose.get("localStrafeCm", 0.0))

        msg.move_phase = int(move.get("phase", 0))
        msg.move_angle_deg = float(move.get("angleDeg", 0.0))
        msg.move_distance_cm = float(move.get("distanceCm", 0.0))
        msg.heading_target_deg = float(move.get("headingTargetDeg", 0.0))
        msg.heading_error_deg = float(move.get("headingErrorDeg", 0.0))
        msg.move_done = bool(move.get("done", False))

        msg.rpm = self._float4(data.get("rpm", []))
        msg.target_rpm = self._float4(data.get("targetRpm", []))
        msg.pwm = self._int4(data.get("pwm", []))
        msg.raw_encoder_counts = self._int4(data.get("rawEncoderCounts", []))
        msg.signed_encoder_counts = self._int4(data.get("signedEncoderCounts", []))
        msg.raw_json = raw
        self.telemetry_pub.publish(msg)

        raw_msg = String()
        raw_msg.data = raw
        self.telemetry_raw_pub.publish(raw_msg)

    @staticmethod
    def _float4(values: Any) -> list[float]:
        output = [0.0, 0.0, 0.0, 0.0]
        if isinstance(values, list):
            for index, value in enumerate(values[:4]):
                output[index] = float(value)
        return output

    @staticmethod
    def _int4(values: Any) -> list[int]:
        output = [0, 0, 0, 0]
        if isinstance(values, list):
            for index, value in enumerate(values[:4]):
                output[index] = int(value)
        return output

    def _fresh_telemetry_age_s(self) -> float | None:
        with self.telemetry_condition:
            if self.latest_data is None:
                return None
            return max(0.0, time.monotonic() - self.latest_telemetry_time_s)

    def _publish_esp_command(self, command: str) -> None:
        msg = String()
        msg.data = f"{command} seq={self._next_seq()}"
        self.move_goal_pub.publish(msg)

    def _publish_command_line(self, command: str) -> None:
        msg = String()
        msg.data = command.strip()
        self.move_goal_pub.publish(msg)

    def _handle_ping(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        age = self._fresh_telemetry_age_s()
        response.success = age is not None and age < 1.0
        response.message = "fresh telemetry" if response.success else "waiting for micro-ROS ESP telemetry"
        return response

    def _handle_init_imu(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._publish_esp_command("INIT_IMU")
        response.success = True
        response.message = "init IMU command published"
        return response

    def _handle_reset_odom(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.world_odom.reset()
        self._publish_esp_command("RESET_ODOM")
        response.success = True
        response.message = "reset odom command published"
        return response

    def _handle_reset_encoders(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.world_odom.reset()
        self._publish_esp_command("RESET_ENC")
        response.success = True
        response.message = "reset encoders command published"
        return response

    def _handle_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._publish_esp_command("STOP")
        with self.telemetry_condition:
            self.cancelled_moves.update(self.active_move_seqs)
            self.telemetry_condition.notify_all()
        response.success = True
        response.message = "stop command published"
        return response

    def _handle_move(self, request: Move.Request, response: Move.Response) -> Move.Response:
        timeout_s = float(request.timeout_s) if request.timeout_s > 0.0 else self.move_timeout_s
        seq = self._next_seq()
        distance_cm = max(0.0, float(request.distance_m) * 100.0)
        command = (
            f"MOVE seq={seq} "
            f"angle={float(request.angle_deg):.3f} "
            f"dist={distance_cm:.3f} "
            f"heading={float(request.heading_deg):.3f} "
            f"timeout={int(max(0.0, timeout_s) * 1000.0)}"
        )
        if self.move_goal_pub.get_subscription_count() <= 0:
            response.ok = False
            response.result = "esp_not_connected"
            response.message = "ESP micro-ROS node is not subscribed to esp/move_goal"
            response.heading_deg = float(request.heading_deg)
            return response

        msg = String()
        msg.data = command
        sent_s = time.monotonic()
        with self.telemetry_condition:
            self.active_move_seqs.add(seq)
        self.move_goal_pub.publish(msg)

        if not request.wait_for_done:
            response.ok = True
            response.result = "accepted"
            response.message = "accepted"
            response.heading_deg = float(request.heading_deg)
            with self.telemetry_condition:
                self.active_move_seqs.discard(seq)
            return response

        deadline = time.monotonic() + timeout_s
        last_data: dict[str, Any] | None = None
        try:
            with self.telemetry_condition:
                while time.monotonic() < deadline:
                    now_s = time.monotonic()
                    if seq in self.cancelled_moves:
                        self.cancelled_moves.discard(seq)
                        if last_data is not None:
                            self._fill_move_response(response, last_data, result="stopped")
                        response.ok = False
                        response.result = "stopped"
                        response.message = "stopped by /audix/esp/stop"
                        return response

                    completed = self.completed_moves.pop(seq, None)
                    if completed is not None:
                        self._fill_move_response(response, completed, result="completed")
                        response.ok = True
                        return response

                    remaining = max(0.0, deadline - now_s)
                    self.telemetry_condition.wait(timeout=min(0.1, remaining))
                    data = self.latest_data
                    latest_time_s = self.latest_telemetry_time_s
                    if not data:
                        continue
                    data_seq = int(data.get("seq", 0))
                    move = data.get("move", {})
                    if data_seq != seq:
                        if (
                            last_data is not None
                            and data_seq == 0
                            and latest_time_s >= sent_s
                            and str(data.get("mode", "")).lower() == "idle"
                            and bool(move.get("done", False))
                        ):
                            self._fill_move_response(response, data, result="completed")
                            response.ok = True
                            return response
                        continue
                    last_data = data
                    if bool(move.get("done", False)):
                        self._fill_move_response(response, data, result="completed")
                        response.ok = True
                        return response
        finally:
            with self.telemetry_condition:
                self.active_move_seqs.discard(seq)
                self.cancelled_moves.discard(seq)

        self._publish_esp_command("STOP")
        if last_data is not None:
            self._fill_move_response(response, last_data, result="timeout_stop")
        response.ok = False
        response.result = "timeout_stop"
        response.message = f"timed out waiting for move seq={seq}"
        return response

    def _fill_move_response(self, response: Move.Response, data: dict[str, Any], *, result: str) -> None:
        pose = data.get("pose", {})
        world_pose = data.get("_audix_world_pose", {})
        response.result = result
        response.message = result
        response.forward_cm = float(world_pose.get("forwardCm", pose.get("forwardCm", data.get("forwardCm", 0.0))))
        response.strafe_cm = float(world_pose.get("strafeCm", pose.get("strafeCm", data.get("strafeCm", 0.0))))
        response.heading_deg = float(pose.get("yawDeg", data.get("headingDeg", 0.0)))
        response.raw_json = json.dumps(data, separators=(",", ":"))

    def _handle_twist(self, request: TwistCommand.Request, response: TwistCommand.Response) -> TwistCommand.Response:
        timeout_s = float(request.timeout_s) if request.timeout_s > 0.0 else 0.3
        command = (
            f"TWIST seq={self._next_seq()} "
            f"forward={float(request.forward_rpm):.3f} "
            f"strafe={float(request.strafe_rpm):.3f} "
            f"turn={float(request.turn_rpm):.3f} "
            f"timeout={int(max(0.0, timeout_s) * 1000.0)}"
        )
        self._publish_command_line(command)
        response.ok = True
        response.message = "twist command published"
        response.raw_json = command
        return response

    def _handle_raw_command(self, request: RawCommand.Request, response: RawCommand.Response) -> RawCommand.Response:
        command = request.command.strip()
        if not command:
            response.ok = False
            response.type = "error"
            response.message = "empty command"
            return response
        tokens = command.split()
        verb = tokens[0].upper() if tokens else ""
        if verb in {"RESET_ODOM", "RESET_ENC"}:
            self.world_odom.reset()
        if not any(part.startswith("seq=") for part in tokens[1:]):
            command = f"{command} seq={self._next_seq()}"
        self._publish_command_line(command)
        response.ok = True
        response.type = "published"
        response.message = "custom ESP command published over micro-ROS"
        response.raw_json = command
        return response


def main() -> None:
    rclpy.init()
    node = AudixMicroRosBase()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
