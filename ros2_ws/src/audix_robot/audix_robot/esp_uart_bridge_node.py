#!/usr/bin/env python3
"""ROS 2 bridge for the final Audix ESP UART controller.

The ESP32 keeps running esp_correct_pid_pi.ino. This node translates ROS 2
services/topics to the proven JSON-over-UART command contract.
"""

from __future__ import annotations

import json
import math
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any

import rclpy
from audix_interfaces.msg import EspTelemetry, IrState
from audix_interfaces.srv import Move, RawCommand, TwistCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


READ_TIMEOUT_S = 0.05
DEFAULT_BAUD = 115200
DEFAULT_ACK_TIMEOUT_S = 3.0
DEFAULT_MOVE_TIMEOUT_S = 180.0
DEFAULT_PING_TIMEOUT_S = 2.5

IR_SENSOR_ORDER = ("front_left", "front", "front_right", "right", "back", "left")
IR_PINS = {
    "front_left": 23,
    "front": 24,
    "front_right": 25,
    "right": 17,
    "back": 27,
    "left": 22,
}


@dataclass
class Event:
    raw: str
    data: dict[str, Any] | None


class SerialJsonLink:
    """Threaded JSON-line serial transport for esp_correct_pid_pi.ino."""

    def __init__(self, port: str, baud: int, *, read_timeout_s: float, verbose: bool, node: Node) -> None:
        import serial

        self.node = node
        self.verbose = verbose
        self.seq = 1
        self.telemetry: queue.Queue[Event] = queue.Queue()
        self.response_events: list[Event] = []
        self.response_condition = threading.Condition()
        self.stop_event = threading.Event()
        self.send_lock = threading.Lock()

        self.ser = serial.Serial(port=port, baudrate=baud, timeout=read_timeout_s)
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
            except Exception as exc:
                self.node.get_logger().error(f"serial read failed: {exc}")
                return
            if not line:
                continue

            raw = line.decode("utf-8", errors="replace").strip()
            if not raw:
                continue
            if self.verbose:
                self.node.get_logger().debug(f"esp raw: {raw}")

            data = self._parse_json_line(raw)
            event = Event(raw=raw, data=data)
            if data is not None and data.get("type") == "telemetry":
                self.telemetry.put(event)
            else:
                with self.response_condition:
                    self.response_events.append(event)
                    self.response_condition.notify_all()

    @staticmethod
    def _parse_json_line(raw: str) -> dict[str, Any] | None:
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError:
            json_start = raw.find("{")
            if json_start < 0:
                return None
            try:
                parsed = json.loads(raw[json_start:])
            except json.JSONDecodeError:
                return None
        return parsed if isinstance(parsed, dict) else None

    def next_seq(self) -> int:
        value = self.seq
        self.seq += 1
        return value

    def send(self, line: str) -> None:
        payload = (line.strip() + "\n").encode("utf-8")
        with self.send_lock:
            self.ser.write(payload)
            self.ser.flush()

    def send_command(self, line: str) -> int:
        seq = self.next_seq()
        if " seq=" not in line:
            line = f"{line} seq={seq}"
        self.send(line)
        return seq

    def send_position_move(self, angle_deg: float, distance_m: float, heading_deg: float, timeout_s: float) -> int:
        distance_cm = max(0.0, distance_m * 100.0)
        return self.send_command(
            "MOVE "
            f"angle={angle_deg:.3f} "
            f"dist={distance_cm:.3f} "
            f"heading={heading_deg:.3f} "
            f"timeout={int(max(0.0, timeout_s) * 1000.0)}"
        )

    def send_twist(self, forward_rpm: float, strafe_rpm: float, turn_rpm: float, timeout_s: float) -> int:
        return self.send_command(
            "TWIST "
            f"forward={forward_rpm:.3f} "
            f"strafe={strafe_rpm:.3f} "
            f"turn={turn_rpm:.3f} "
            f"timeout={int(max(0.0, timeout_s) * 1000.0)}"
        )

    def wait_for(self, seq: int | None, wanted_types: set[str], timeout_s: float) -> Event:
        deadline = time.monotonic() + timeout_s
        with self.response_condition:
            while time.monotonic() < deadline:
                for index, event in enumerate(list(self.response_events)):
                    data = event.data
                    if data is None:
                        continue
                    seq_matches = seq is None or data.get("seq") == seq
                    if seq_matches and data.get("type") in wanted_types:
                        return self.response_events.pop(index)

                remaining = deadline - time.monotonic()
                self.response_condition.wait(timeout=max(0.0, remaining))

            raise TimeoutError(f"timed out waiting for {sorted(wanted_types)} seq={seq}")


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


class AudixEspUartBridge(Node):
    """ROS 2 node that exposes the final Audix ESP UART firmware."""

    def __init__(self) -> None:
        super().__init__("audix_esp_uart_bridge")
        self.callback_group = ReentrantCallbackGroup()

        self.port = self.declare_parameter("port", "/dev/ttyAMA0").value
        self.baud = int(self.declare_parameter("baud", DEFAULT_BAUD).value)
        self.verbose_serial = bool(self.declare_parameter("verbose_serial", False).value)
        self.ack_timeout_s = float(self.declare_parameter("ack_timeout_s", DEFAULT_ACK_TIMEOUT_S).value)
        self.ping_timeout_s = float(self.declare_parameter("ping_timeout_s", DEFAULT_PING_TIMEOUT_S).value)
        self.move_timeout_s = float(self.declare_parameter("move_timeout_s", DEFAULT_MOVE_TIMEOUT_S).value)
        self.telemetry_period_s = float(self.declare_parameter("telemetry_period_s", 0.02).value)
        self.ir_poll_period_s = float(self.declare_parameter("ir_poll_period_s", 0.05).value)
        self.frame_id = self.declare_parameter("frame_id", "odom").value
        self.base_frame_id = self.declare_parameter("base_frame_id", "base_link").value
        self.odom_forward_sign = float(self.declare_parameter("odom_forward_sign", 1.0).value)
        self.odom_strafe_sign = float(self.declare_parameter("odom_strafe_sign", 1.0).value)
        self.init_imu_on_start = bool(self.declare_parameter("init_imu_on_start", True).value)
        self.reset_odom_on_start = bool(self.declare_parameter("reset_odom_on_start", True).value)
        self.reset_encoders_on_start = bool(self.declare_parameter("reset_encoders_on_start", False).value)
        self.ir_enabled = bool(self.declare_parameter("ir_enabled", True).value)
        self.mock_ir = bool(self.declare_parameter("mock_ir", False).value)
        self.ir_logic = self.declare_parameter("ir_logic", "baseline").value
        self.ir_active_low = bool(self.declare_parameter("ir_active_low", True).value)
        self.ir_pull_up = bool(self.declare_parameter("ir_pull_up", True).value)

        self.command_lock = threading.Lock()
        self.link = SerialJsonLink(
            self.port,
            self.baud,
            read_timeout_s=READ_TIMEOUT_S,
            verbose=self.verbose_serial,
            node=self,
        )
        self.ir_bank: GpioIrBank | None = None
        if self.ir_enabled:
            self.ir_bank = GpioIrBank(
                active_low=self.ir_active_low,
                pull_up=self.ir_pull_up,
                mock=self.mock_ir,
                logic=self.ir_logic,
                node=self,
            )

        self.telemetry_pub = self.create_publisher(EspTelemetry, "esp/telemetry", 10)
        self.telemetry_raw_pub = self.create_publisher(String, "esp/telemetry_raw", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.ir_pub = self.create_publisher(IrState, "ir/state", 10)

        self.create_service(Trigger, "esp/ping", self._handle_ping, callback_group=self.callback_group)
        self.create_service(Trigger, "esp/init_imu", self._handle_init_imu, callback_group=self.callback_group)
        self.create_service(Trigger, "esp/reset_odom", self._handle_reset_odom, callback_group=self.callback_group)
        self.create_service(Trigger, "esp/reset_encoders", self._handle_reset_encoders, callback_group=self.callback_group)
        self.create_service(Trigger, "esp/stop", self._handle_stop, callback_group=self.callback_group)
        self.create_service(Move, "move", self._handle_move, callback_group=self.callback_group)
        self.create_service(TwistCommand, "twist", self._handle_twist, callback_group=self.callback_group)
        self.create_service(RawCommand, "esp/raw_command", self._handle_raw_command, callback_group=self.callback_group)

        self.create_timer(max(0.005, self.telemetry_period_s), self._drain_telemetry, callback_group=self.callback_group)
        if self.ir_enabled:
            self.create_timer(max(0.01, self.ir_poll_period_s), self._publish_ir, callback_group=self.callback_group)

        self._startup_handshake()
        self.get_logger().info(f"Audix ESP UART bridge ready on {self.port} @ {self.baud}")

    def _startup_handshake(self) -> None:
        with self.command_lock:
            try:
                pong = self._send_and_wait("PING", {"pong"}, self.ping_timeout_s)
                self.get_logger().info(f"ESP ping ok seq={pong.data.get('seq') if pong.data else 'unknown'}")
            except Exception as exc:
                self.get_logger().warning(f"ESP ping failed, continuing: {exc}")

            if self.init_imu_on_start:
                self._command_ack_locked("INIT_IMU", self.ack_timeout_s * 3.0)
            if self.reset_encoders_on_start:
                self._command_ack_locked("RESET_ENC", self.ack_timeout_s)
            if self.reset_odom_on_start:
                self._command_ack_locked("RESET_ODOM", self.ack_timeout_s)

    def _send_and_wait(self, command: str, wanted: set[str], timeout_s: float) -> Event:
        seq = self.link.send_command(command)
        return self.link.wait_for(seq, wanted, timeout_s)

    def _command_ack_locked(self, command: str, timeout_s: float) -> Event:
        event = self._send_and_wait(command, {"ack"}, timeout_s)
        data = event.data or {}
        if not data.get("ok", False):
            raise RuntimeError(f"{command} rejected: {data.get('message', 'no message')}")
        return event

    def _handle_ping(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        with self.command_lock:
            try:
                event = self._send_and_wait("PING", {"pong"}, self.ping_timeout_s)
                response.success = bool((event.data or {}).get("ok", True))
                response.message = event.raw
            except Exception as exc:
                response.success = False
                response.message = str(exc)
        return response

    def _handle_init_imu(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._trigger_ack("INIT_IMU", response, timeout_s=self.ack_timeout_s * 3.0)

    def _handle_reset_odom(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._trigger_ack("RESET_ODOM", response, timeout_s=self.ack_timeout_s)

    def _handle_reset_encoders(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._trigger_ack("RESET_ENC", response, timeout_s=self.ack_timeout_s)

    def _handle_stop(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._trigger_ack("STOP", response, timeout_s=self.ack_timeout_s)

    def _trigger_ack(self, command: str, response: Trigger.Response, *, timeout_s: float) -> Trigger.Response:
        with self.command_lock:
            try:
                event = self._command_ack_locked(command, timeout_s)
                response.success = True
                response.message = event.raw
            except Exception as exc:
                response.success = False
                response.message = str(exc)
        return response

    def _handle_move(self, request: Move.Request, response: Move.Response) -> Move.Response:
        timeout_s = float(request.timeout_s) if request.timeout_s > 0.0 else self.move_timeout_s
        try:
            with self.command_lock:
                seq = self.link.send_position_move(request.angle_deg, request.distance_m, request.heading_deg, timeout_s)
                ack = self.link.wait_for(seq, {"ack"}, self.ack_timeout_s)
            ack_data = ack.data or {}
            if not ack_data.get("ok", False):
                response.ok = False
                response.message = ack_data.get("message", "MOVE rejected")
                response.raw_json = ack.raw
                return response

            if request.wait_for_done:
                done = self.link.wait_for(seq, {"done"}, timeout_s + self.ack_timeout_s)
                self._fill_move_response_from_event(response, done)
            else:
                response.ok = True
                response.result = "accepted"
                response.message = ack_data.get("message", "accepted")
                response.raw_json = ack.raw
        except Exception as exc:
            response.ok = False
            response.message = str(exc)
        return response

    @staticmethod
    def _fill_move_response_from_event(response: Move.Response, event: Event) -> None:
        data = event.data or {}
        response.result = str(data.get("result", data.get("type", "")))
        response.ok = response.result == "completed"
        response.message = response.result
        response.forward_cm = float(data.get("forwardCm", 0.0))
        response.strafe_cm = float(data.get("strafeCm", 0.0))
        response.heading_deg = float(data.get("headingDeg", 0.0))
        response.raw_json = event.raw

    def _handle_twist(self, request: TwistCommand.Request, response: TwistCommand.Response) -> TwistCommand.Response:
        timeout_s = float(request.timeout_s) if request.timeout_s > 0.0 else 0.3
        with self.command_lock:
            try:
                seq = self.link.send_twist(request.forward_rpm, request.strafe_rpm, request.turn_rpm, timeout_s)
                ack = self.link.wait_for(seq, {"ack"}, self.ack_timeout_s)
                data = ack.data or {}
                response.ok = bool(data.get("ok", False))
                response.message = str(data.get("message", ""))
                response.raw_json = ack.raw
            except Exception as exc:
                response.ok = False
                response.message = str(exc)
        return response

    def _handle_raw_command(self, request: RawCommand.Request, response: RawCommand.Response) -> RawCommand.Response:
        timeout_s = float(request.timeout_s) if request.timeout_s > 0.0 else self.ack_timeout_s
        try:
            with self.command_lock:
                seq = self.link.send_command(request.command)
                first = self.link.wait_for(seq, {"ack", "pong", "limit", "done"}, timeout_s)
            event = first
            if request.wait_for_done and (first.data or {}).get("type") == "ack":
                event = self.link.wait_for(seq, {"done"}, timeout_s)
            data = event.data or {}
            response.ok = bool(data.get("ok", True))
            response.type = str(data.get("type", ""))
            response.result = str(data.get("result", ""))
            response.message = str(data.get("message", event.raw))
            response.raw_json = event.raw
        except Exception as exc:
            response.ok = False
            response.message = str(exc)
        return response

    def _drain_telemetry(self) -> None:
        while True:
            try:
                event = self.link.telemetry.get_nowait()
            except queue.Empty:
                return
            if event.data is None:
                continue
            self._publish_telemetry(event)

    def _publish_telemetry(self, event: Event) -> None:
        data = event.data or {}
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

        msg.forward_cm = float(pose.get("forwardCm", 0.0))
        msg.strafe_cm = float(pose.get("strafeCm", 0.0))
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
        msg.raw_json = event.raw
        self.telemetry_pub.publish(msg)

        raw_msg = String()
        raw_msg.data = event.raw
        self.telemetry_raw_pub.publish(raw_msg)

        self.odom_pub.publish(self._odom_from_telemetry(now, msg))

    def _odom_from_telemetry(self, stamp: Any, telemetry: EspTelemetry) -> Odometry:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.odom_forward_sign * telemetry.forward_cm / 100.0
        odom.pose.pose.position.y = self.odom_strafe_sign * telemetry.strafe_cm / 100.0
        yaw_rad = math.radians(telemetry.yaw_deg)
        odom.pose.pose.orientation.z = math.sin(yaw_rad * 0.5)
        odom.pose.pose.orientation.w = math.cos(yaw_rad * 0.5)
        odom.twist.twist.angular.z = math.radians(telemetry.gyro_filt_dps)
        return odom

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
        try:
            self.link.send("STOP")
        except Exception:
            pass
        try:
            self.link.close()
        except Exception:
            pass
        if self.ir_bank is not None:
            self.ir_bank.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = AudixEspUartBridge()
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
