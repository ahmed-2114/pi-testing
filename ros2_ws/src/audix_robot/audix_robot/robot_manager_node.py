#!/usr/bin/env python3
"""Safe high-level command gate and mission/avoidance manager for Audix."""

from __future__ import annotations

import math
import json
import threading
import time
from dataclasses import dataclass
from typing import Callable

import rclpy
from audix_interfaces.msg import EspTelemetry, IrState
from audix_interfaces.srv import (
    AuditMission,
    DirectionCommand,
    LiftMoveSteps,
    Move,
    RotateCommand,
    SetRobotMode,
    ShelfScan,
)
from audix_robot.navigation_contract import (
    DIRECTION_ANGLES_DEG,
    HEADING_BACKWARD_DEG,
    HEADING_FORWARD_DEG,
    HEADING_LEFT_DEG,
    HEADING_RIGHT_DEG,
    forward_heading_for_world_direction,
    rotation_delta_for_turn_direction,
    wrap_degrees,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


LEFT = -1
RIGHT = 1
IR_SENSOR_ORDER = ("front_left", "front", "front_right", "right", "back", "left")
FRONT_CORNER_SENSORS = {"front_left", "front_right"}
FRONT_WATCH_SENSORS = {"front", "front_left", "front_right"}
SIDE_WATCH_SENSORS = {"left", "right"}
ALL_WATCH_SENSORS = FRONT_WATCH_SENSORS | SIDE_WATCH_SENSORS

MAP_MOVE_OK_RESULTS = {"completed", "front_dynamic_clear", "side_falling", "corner_falling", "back_falling", "none"}


@dataclass(frozen=True)
class MapPoint:
    x_cm: float
    y_cm: float


class AudixStoreMap:
    WIDTH_CM = 250.0
    HEIGHT_CM = 200.0
    ROBOT_WIDTH_CM = 30.0
    ROBOT_LENGTH_CM = 40.0
    SPAWN = MapPoint(15.0, 165.0)
    TOP_TRAVEL_Y_CM = 165.0
    AUDIT_Y_CM = 80.0
    LANE_CENTER_X_CM = {1: 50.0, 2: 200.0}
    SIDE_NAME = {1: "left shelf side", 2: "right shelf side"}
    SCAN_HEADING_DEG = {1: HEADING_LEFT_DEG, 2: HEADING_RIGHT_DEG}
    FRONT_AVOIDANCE_BIAS = {1: LEFT, 2: RIGHT}
    SHELF_X_MIN_CM = 105.0
    SHELF_X_MAX_CM = 145.0
    SHELF_Y_MIN_CM = 75.0
    SHELF_Y_MAX_CM = 125.0


@dataclass
class MoveDone:
    result: str = "none"
    message: str = ""
    forward_cm: float = 0.0
    strafe_cm: float = 0.0
    heading_deg: float = 0.0
    ir: list[str] | None = None

    def as_dict(self) -> dict:
        return {
            "result": self.result,
            "message": self.message,
            "forwardCm": self.forward_cm,
            "strafeCm": self.strafe_cm,
            "headingDeg": self.heading_deg,
            "ir": self.ir or [],
        }


@dataclass
class MissionArgs:
    goal_distance: float = 1.20
    goal_tolerance: float = 0.05
    position_step: float = 0.05
    min_position_step: float = 0.005
    heading: float = 0.0
    status_period: float = 0.5
    telemetry_timeout: float = 0.5
    move_timeout: float = 180.0
    front_dynamic_hold: float = 3.0
    front_strafe_distance: float = 0.20
    front_corner_strafe_distance: float = 0.15
    front_advance_distance: float = 0.20
    front_strafe_search_distance: float = 1.20
    front_corner_buffer_distance: float = 0.05
    front_strafe_search_timeout: float = 8.0
    front_corner_buffer_timeout: float = 1.25
    front_advance_timeout: float = 4.0
    side_follow_search_distance: float = 3.00
    side_follow_dry_distance: float = 0.30
    side_follow_watch_front: bool = False
    side_escape_distance: float = 0.10
    side_escape_forward_distance: float = 0.20
    rejoin_tolerance: float = 0.02
    max_recenter_attempts: int = 8
    max_goal_correction_attempts: int = 4
    max_avoidance_actions: int = 24
    reverse_heading_threshold_deg: float = 135.0


def opposite_direction(direction: int) -> int:
    return LEFT if direction == RIGHT else RIGHT


def direction_to_angle(direction: int) -> float:
    return HEADING_LEFT_DEG if direction == LEFT else HEADING_RIGHT_DEG


def direction_name(direction: int) -> str:
    return "left" if direction == LEFT else "right"


def side_sensor_for_direction(direction: int) -> str:
    return "left" if direction == LEFT else "right"


def front_corner_sensor_after_strafe(direction: int) -> str:
    return "front_right" if direction == LEFT else "front_left"


def side_sensor_after_front_avoidance(direction: int) -> str:
    return side_sensor_for_direction(opposite_direction(direction))


class PoseAccumulator:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.world_forward_cm = 0.0
        self.world_strafe_cm = 0.0

    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw

    def set_from_telemetry(self, telemetry: EspTelemetry) -> tuple[float, float, float]:
        self.world_forward_cm = float(telemetry.forward_cm)
        self.world_strafe_cm = float(telemetry.strafe_cm)
        self.yaw = math.radians(float(telemetry.yaw_deg))
        self.x = -self.world_forward_cm / 100.0
        self.y = -self.world_strafe_cm / 100.0
        return self.pose()

    def reset_world(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.world_forward_cm = 0.0
        self.world_strafe_cm = 0.0


@dataclass
class MissionMemory:
    forward_m: float = 0.0
    lateral_m: float = 0.0
    lateral_reference_m: float = 0.0

    def record_completed_move(self, angle_deg: float, distance_m: float, done: dict) -> None:
        if done.get("result") == "ir_stop":
            return
        angle_rad = math.radians(angle_deg)
        command_forward_m = math.cos(angle_rad) * distance_m
        command_lateral_m = math.sin(angle_rad) * distance_m
        if abs(command_forward_m) >= abs(command_lateral_m):
            reported_forward_m = 0.01 * float(done.get("forwardCm", command_forward_m * 100.0))
            if abs(reported_forward_m) < 1e-6:
                reported_forward_m = command_forward_m
            self.forward_m = max(0.0, self.forward_m + reported_forward_m)
        else:
            self.lateral_m += command_lateral_m

    def sync_from_pose(self, pose_tracker: PoseAccumulator) -> None:
        self.forward_m = max(0.0, -pose_tracker.x)
        self.lateral_m = -pose_tracker.y - self.lateral_reference_m

    def reset_lateral_reference(self, pose_tracker: PoseAccumulator) -> None:
        self.lateral_reference_m = -pose_tracker.y
        self.lateral_m = 0.0

    def snap_center_if_close(self, tolerance_m: float) -> None:
        if abs(self.lateral_m) <= tolerance_m:
            self.lateral_m = 0.0


class RobotManager(Node):
    def __init__(self) -> None:
        super().__init__("robot_manager")
        self.callback_group = ReentrantCallbackGroup()
        self.mode = "manual"
        self.mode_lock = threading.Lock()
        self.motion_lock = threading.Lock()
        self.latest_ir = {name: False for name in IR_SENSOR_ORDER}
        self.latest_telemetry: EspTelemetry | None = None
        self.latest_telemetry_time_s = 0.0
        self.pose = PoseAccumulator()
        self.last_event = "ready"
        self.manual_buzzer_until = 0.0
        self.manual_stop_last_s = 0.0
        self.mission_running = False
        self.cancel_mission = threading.Event()
        self.map_pose = AudixStoreMap.SPAWN
        self.current_audit_side: int | None = None
        self.front_avoidance_bias_override: int | None = None

        self.args = MissionArgs(
            goal_distance=float(self.declare_parameter("goal_distance_m", 1.20).value),
            front_dynamic_hold=float(self.declare_parameter("front_dynamic_hold_s", 3.0).value),
            reverse_heading_threshold_deg=float(
                self.declare_parameter("reverse_heading_threshold_deg", 135.0).value
            ),
        )
        self.allow_placeholder_audit = bool(self.declare_parameter("allow_placeholder_audit", False).value)
        self.buzzer_hold_s = float(self.declare_parameter("manual_buzzer_hold_s", 1.5).value)
        self.lift_steps = int(self.declare_parameter("audit_lift_steps", 500).value)
        self.lift_speed_sps = float(self.declare_parameter("lift_speed_sps", 500.0).value)
        self.scan_timeout_s = float(self.declare_parameter("vision_scan_timeout_s", 25.0).value)
        self.scan_settle_s = float(self.declare_parameter("vision_scan_settle_s", 0.5).value)
        self.home_tolerance_cm = float(self.declare_parameter("home_tolerance_cm", 2.0).value)
        self.home_max_passes = int(self.declare_parameter("home_max_passes", 8).value)
        self.home_settle_s = float(self.declare_parameter("home_settle_s", 0.25).value)
        self.audit_shelf_ids = {
            (1, 1): str(self.declare_parameter("audit_side_1_level_1_shelf_id", "beans_can").value),
            (1, 2): str(self.declare_parameter("audit_side_1_level_2_shelf_id", "indomie").value),
            (2, 1): str(self.declare_parameter("audit_side_2_level_1_shelf_id", "indomie").value),
            (2, 2): str(self.declare_parameter("audit_side_2_level_2_shelf_id", "fruit_rings_cereal").value),
        }

        self.move_client = self.create_client(Move, "move", callback_group=self.callback_group)
        self.stop_client = self.create_client(Trigger, "esp/stop", callback_group=self.callback_group)
        self.buzzer_client = self.create_client(SetBool, "gpio/set_buzzer", callback_group=self.callback_group)
        self.lift_client = self.create_client(LiftMoveSteps, "lift/move_steps", callback_group=self.callback_group)
        self.scan_client = self.create_client(ShelfScan, "scan_shelf", callback_group=self.callback_group)

        self.create_subscription(IrState, "ir/state", self._on_ir, 10, callback_group=self.callback_group)
        self.create_subscription(EspTelemetry, "esp/telemetry", self._on_telemetry, 10, callback_group=self.callback_group)
        self.event_pub = self.create_publisher(String, "mission/event", 20)
        self.scan_result_pub = self.create_publisher(String, "vision/scan_result", 10)

        self.create_service(SetRobotMode, "manager/set_mode", self._handle_set_mode, callback_group=self.callback_group)
        self.create_service(DirectionCommand, "manager/direction_move", self._handle_direction_move, callback_group=self.callback_group)
        self.create_service(RotateCommand, "manager/rotate", self._handle_rotate, callback_group=self.callback_group)
        self.create_service(AuditMission, "manager/start_audit", self._handle_start_audit, callback_group=self.callback_group)
        self.create_service(Trigger, "manager/go_home", self._handle_go_home, callback_group=self.callback_group)
        self.create_service(Trigger, "manager/stop", self._handle_manager_stop, callback_group=self.callback_group)
        self.create_timer(0.05, self._manual_safety_tick, callback_group=self.callback_group)
        self.get_logger().info("Robot manager ready")

    def _on_ir(self, msg: IrState) -> None:
        self.latest_ir = {
            "front_left": bool(msg.front_left),
            "front": bool(msg.front),
            "front_right": bool(msg.front_right),
            "right": bool(msg.right),
            "back": bool(msg.back),
            "left": bool(msg.left),
        }

    def _on_telemetry(self, msg: EspTelemetry) -> None:
        self.latest_telemetry = msg
        self.latest_telemetry_time_s = time.monotonic()
        self.pose.set_from_telemetry(msg)
        self.args.heading = float(msg.yaw_deg)

    def _publish_event(self, text: str) -> None:
        self.last_event = text
        msg = String()
        msg.data = text
        self.event_pub.publish(msg)
        self.get_logger().info(text)

    def _active_ir(self, sensors: set[str] | None = None) -> list[str]:
        names = sensors if sensors is not None else set(IR_SENSOR_ORDER)
        return sorted(name for name in names if self.latest_ir.get(name, False))

    def _call_sync(self, client, request, timeout_s: float):
        if not client.wait_for_service(timeout_sec=max(0.1, float(timeout_s))):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        event = threading.Event()
        holder = {}
        future = client.call_async(request)
        future.add_done_callback(lambda done: (holder.setdefault("future", done), event.set()))
        if not event.wait(timeout_s):
            raise TimeoutError(f"timed out waiting for {client.srv_name}")
        return holder["future"].result()

    def _set_buzzer(self, enabled: bool) -> None:
        try:
            req = SetBool.Request()
            req.data = bool(enabled)
            self._call_sync(self.buzzer_client, req, 1.0)
        except Exception as exc:
            self.get_logger().warning(f"buzzer request failed: {exc}")

    def _stop_robot(self) -> None:
        try:
            self._call_sync(self.stop_client, Trigger.Request(), 2.0)
        except Exception as exc:
            self.get_logger().warning(f"STOP request failed: {exc}")

    def _manual_safety_tick(self) -> None:
        with self.mode_lock:
            mode = self.mode
        if mode != "manual":
            return
        active = self._active_ir()
        now = time.monotonic()
        if active:
            if now - self.manual_stop_last_s >= 0.5:
                self._stop_robot()
                self.manual_stop_last_s = now
                self._publish_event(f"manual obstacle stop: {active}")
            self.manual_buzzer_until = now + self.buzzer_hold_s
            self._set_buzzer(True)
        elif self.manual_buzzer_until and now >= self.manual_buzzer_until:
            self.manual_buzzer_until = 0.0
            self._set_buzzer(False)

    def _send_move_future(self, angle_deg: float, distance_m: float, heading_deg: float, timeout_s: float):
        if not self.move_client.wait_for_service(timeout_sec=max(0.1, min(float(timeout_s), 3.0))):
            raise RuntimeError(f"service unavailable: {self.move_client.srv_name}")
        request = Move.Request()
        request.angle_deg = float(angle_deg)
        request.distance_m = max(0.0, float(distance_m))
        request.heading_deg = float(heading_deg)
        request.timeout_s = max(0.05, float(timeout_s))
        request.wait_for_done = True
        return self.move_client.call_async(request)

    def _move_done_from_response(self, response: Move.Response, result_override: str | None = None, ir: list[str] | None = None) -> dict:
        result = result_override or str(response.result)
        heading_deg = float(response.heading_deg)
        self.args.heading = heading_deg
        return MoveDone(
            result=result,
            message=str(response.message),
            forward_cm=float(response.forward_cm),
            strafe_cm=float(response.strafe_cm),
            heading_deg=heading_deg,
            ir=ir or [],
        ).as_dict()

    def _current_heading_deg(self) -> float:
        if self.latest_telemetry is not None:
            return float(self.latest_telemetry.yaw_deg)
        return math.degrees(self.pose.yaw)

    def _execute_move_watch(
        self,
        angle_deg: float,
        distance_m: float,
        watch_sensors: set[str],
        timeout_s: float,
        *,
        label: str,
        stop_predicate: Callable[[], tuple[str, list[str]] | None] | None = None,
    ) -> dict:
        self._publish_event(f"move {label}: angle={angle_deg:.1f} dist_cm={distance_m * 100.0:.1f}")
        future = self._send_move_future(angle_deg, distance_m, self.args.heading, timeout_s)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if future.done():
                response = future.result()
                return self._move_done_from_response(response)

            if stop_predicate is not None:
                stop = stop_predicate()
                if stop is not None:
                    result, active = stop
                    self._stop_robot()
                    response = self._wait_future_response(future, 1.5)
                    if response is not None:
                        return self._move_done_from_response(response, result_override=result, ir=active)
                    return MoveDone(result=result, heading_deg=self._current_heading_deg(), ir=active).as_dict()

            active = self._active_ir(watch_sensors)
            if active:
                self._publish_event(f"ir_stop active={active}")
                self._stop_robot()
                response = self._wait_future_response(future, 1.5)
                if response is not None:
                    return self._move_done_from_response(response, result_override="ir_stop", ir=active)
                return MoveDone(result="ir_stop", heading_deg=self._current_heading_deg(), ir=active).as_dict()

            time.sleep(0.02)

        self._stop_robot()
        response = self._wait_future_response(future, 1.5)
        if response is not None:
            return self._move_done_from_response(response, result_override="timeout_stop")
        raise TimeoutError(f"timed out waiting for move {label}")

    @staticmethod
    def _wait_future_response(future, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.02)
        return None

    def _execute_segment(
        self,
        angle_deg: float,
        distance_m: float,
        watch_sensors: set[str],
        mission: MissionMemory | None,
        *,
        label: str,
        move_timeout_s: float | None = None,
        timeout_returns_done: bool = False,
    ) -> dict:
        timeout_s = move_timeout_s if move_timeout_s is not None else self.args.move_timeout
        done = self._execute_move_watch(angle_deg, max(0.0, distance_m), watch_sensors, timeout_s, label=label)
        if done.get("result") == "timeout_stop" and not timeout_returns_done:
            raise TimeoutError(f"Timed out during {label}")
        if mission is not None:
            if done.get("result") == "ir_stop":
                mission.sync_from_pose(self.pose)
            else:
                mission.record_completed_move(angle_deg, distance_m, done)
                mission.sync_from_pose(self.pose)
                mission.snap_center_if_close(self.args.rejoin_tolerance)
        return done

    def _wait_for_dynamic_front_clear(self, active_sensors: list[str]) -> tuple[bool, dict[str, bool]]:
        hold_s = max(0.0, self.args.front_dynamic_hold)
        self._publish_event(f"dynamic front hold {hold_s:.1f}s active={active_sensors}")
        self._set_buzzer(True)
        try:
            time.sleep(hold_s)
        finally:
            self._set_buzzer(False)
        ir_state = dict(self.latest_ir)
        still_front = any(ir_state.get(name, False) for name in FRONT_WATCH_SENSORS)
        return still_front, ir_state

    def _active_avoidance_side(self) -> int:
        if self.current_audit_side in AudixStoreMap.LANE_CENTER_X_CM:
            return int(self.current_audit_side)
        return min(
            AudixStoreMap.LANE_CENTER_X_CM,
            key=lambda side: abs(AudixStoreMap.LANE_CENTER_X_CM[side] - self.map_pose.x_cm),
        )

    def _heading_is_reversed(self) -> bool:
        threshold = min(179.0, max(90.0, float(self.args.reverse_heading_threshold_deg)))
        return abs(wrap_degrees(self.args.heading)) >= threshold

    def _front_avoidance_bias(self) -> tuple[int, int]:
        side = self._active_avoidance_side()
        if self.front_avoidance_bias_override is not None:
            return side, self.front_avoidance_bias_override
        bias = AudixStoreMap.FRONT_AVOIDANCE_BIAS.get(side, LEFT)
        return side, bias

    def _map_direction_to_body_direction(self, direction: str) -> str:
        direction = direction.upper()
        if not self._heading_is_reversed():
            return direction
        return {
            "F": "B",
            "B": "F",
            "L": "R",
            "R": "L",
        }.get(direction, direction)

    def _map_lateral_to_body_direction(self, direction: int) -> int:
        if not self._heading_is_reversed():
            return direction
        return opposite_direction(direction)

    def _body_lateral_to_map_direction(self, direction: int) -> int:
        if not self._heading_is_reversed():
            return direction
        return opposite_direction(direction)

    def _avoidance_direction_label(self, direction: int) -> str:
        body_direction = self._map_lateral_to_body_direction(direction)
        label = direction_name(direction)
        if body_direction != direction:
            label += f" (body {direction_name(body_direction)})"
        return label

    def _choose_front_avoidance(self, ir_state: dict[str, bool]) -> tuple[int, str]:
        side, bias = self._front_avoidance_bias()
        front = bool(ir_state.get("front", False))
        front_left = bool(ir_state.get("front_left", False))
        front_right = bool(ir_state.get("front_right", False))
        if front:
            if front_left and not front_right:
                return self._body_lateral_to_map_direction(RIGHT), "front+front_left"
            if front_right and not front_left:
                return self._body_lateral_to_map_direction(LEFT), "front+front_right"
            return bias, f"front lane{side} bias {direction_name(bias)}"
        if front_left:
            return self._body_lateral_to_map_direction(RIGHT), "front_left"
        if front_right:
            return self._body_lateral_to_map_direction(LEFT), "front_right"
        return bias, f"front lane{side} bias {direction_name(bias)}"

    def _execute_front_search_strafe(self, direction: int, mission: MissionMemory) -> tuple[dict, int, str]:
        for _attempt in range(2):
            body_direction = self._map_lateral_to_body_direction(direction)
            corner_sensor = front_corner_sensor_after_strafe(body_direction)
            side_block_sensor = side_sensor_for_direction(body_direction)
            watch = {corner_sensor, side_block_sensor}
            done = self._execute_segment(
                direction_to_angle(body_direction),
                self.args.front_strafe_search_distance,
                watch,
                mission,
                label=f"front search strafe {self._avoidance_direction_label(direction)}",
                move_timeout_s=self.args.front_strafe_search_timeout,
                timeout_returns_done=True,
            )
            active = set(done.get("ir", []))
            if done.get("result") == "ir_stop" and side_block_sensor in active:
                direction = opposite_direction(direction)
                continue
            return done, direction, corner_sensor
        raise RuntimeError("both strafe directions blocked during front avoidance")

    def _execute_strafe_until_corner_falling(self, direction: int, corner_sensor: str, mission: MissionMemory) -> dict:
        previous_active = bool(self.latest_ir.get(corner_sensor, False))

        def stop_predicate():
            body_direction = self._map_lateral_to_body_direction(direction)
            side_block_sensor = side_sensor_for_direction(body_direction)
            if self.latest_ir.get(side_block_sensor, False):
                return "ir_stop", [side_block_sensor]
            nonlocal previous_active
            active = bool(self.latest_ir.get(corner_sensor, False))
            if previous_active and not active:
                return "corner_falling", [corner_sensor]
            previous_active = active
            return None

        body_direction = self._map_lateral_to_body_direction(direction)
        done = self._execute_move_watch(
            direction_to_angle(body_direction),
            self.args.front_strafe_search_distance,
            set(),
            self.args.front_strafe_search_timeout,
            label=f"strafe {self._avoidance_direction_label(direction)} until {corner_sensor} falling",
            stop_predicate=stop_predicate,
        )
        mission.sync_from_pose(self.pose)
        return done

    def _execute_forward_until_side_falling(self, side_sensor: str, mission: MissionMemory) -> dict:
        seen_active = bool(self.latest_ir.get(side_sensor, False))
        previous_active = seen_active

        def stop_predicate():
            nonlocal seen_active, previous_active
            front_watch = FRONT_WATCH_SENSORS if self.args.side_follow_watch_front else set()
            front_hits = self._active_ir(front_watch)
            if front_hits:
                return "ir_stop", front_hits
            active = bool(self.latest_ir.get(side_sensor, False))
            if active and not seen_active:
                seen_active = True
            if seen_active and previous_active and not active:
                return "side_falling", [side_sensor]
            previous_active = active
            return None

        done = self._execute_move_watch(
            0.0,
            self.args.side_follow_search_distance,
            set(),
            self.args.move_timeout,
            label=f"forward until {side_sensor} falling",
            stop_predicate=stop_predicate,
        )
        mission.sync_from_pose(self.pose)
        return done

    def _execute_lateral_until_back_falling(self, direction: int, mission: MissionMemory) -> dict:
        seen_active = bool(self.latest_ir.get("back", False))
        previous_active = seen_active

        def stop_predicate():
            nonlocal seen_active, previous_active
            front_hits = self._active_ir(FRONT_WATCH_SENSORS)
            if front_hits:
                return "ir_stop", front_hits
            active = bool(self.latest_ir.get("back", False))
            if active and not seen_active:
                seen_active = True
            if seen_active and previous_active and not active:
                return "back_falling", ["back"]
            previous_active = active
            return None

        body_direction = self._map_lateral_to_body_direction(direction)
        done = self._execute_move_watch(
            direction_to_angle(body_direction),
            self.args.side_follow_search_distance,
            set(),
            self.args.move_timeout,
            label=f"strafe {self._avoidance_direction_label(direction)} until back falling",
            stop_predicate=stop_predicate,
        )
        mission.sync_from_pose(self.pose)
        return done

    def _side_path_direction(self, active_sensors: list[str]) -> int:
        active = set(active_sensors)
        if "right" in active and "left" not in active:
            return self._body_lateral_to_map_direction(RIGHT)
        if "left" in active and "right" not in active:
            return self._body_lateral_to_map_direction(LEFT)
        return self._front_avoidance_bias()[1]

    def _execute_side_path_escape(self, active_sensors: list[str], action_budget: list[int], mission: MissionMemory) -> dict:
        action_budget[0] -= 1
        if action_budget[0] < 0:
            raise RuntimeError("too many avoidance actions")
        direction = self._side_path_direction(active_sensors)
        self._publish_event(
            f"side path escape {active_sensors}: forward then {self._avoidance_direction_label(direction)}"
        )

        done = self._execute_segment(
            0.0,
            self.args.front_advance_distance,
            FRONT_WATCH_SENSORS,
            mission,
            label="side path forward buffer",
            move_timeout_s=self.args.front_advance_timeout,
            timeout_returns_done=True,
        )
        if done.get("result") == "ir_stop":
            return self._handle_ir_stop(done.get("ir", []), action_budget, mission)

        done = self._execute_lateral_until_back_falling(direction, mission)
        if done.get("result") == "ir_stop":
            return self._handle_ir_stop(done.get("ir", []), action_budget, mission)

        done = self._execute_segment(
            DIRECTION_ANGLES_DEG["B"],
            self.args.front_advance_distance,
            ALL_WATCH_SENSORS,
            mission,
            label="side path backward return to original line",
            move_timeout_s=self.args.front_advance_timeout,
            timeout_returns_done=True,
        )
        if done.get("result") == "ir_stop":
            return self._handle_ir_stop(done.get("ir", []), action_budget, mission)
        return done

    def _execute_recenter(self, action_budget: list[int], mission: MissionMemory) -> dict:
        last_done = MoveDone(heading_deg=self._current_heading_deg()).as_dict()
        for _ in range(self.args.max_recenter_attempts):
            offset = mission.lateral_m
            if abs(offset) <= self.args.rejoin_tolerance:
                mission.lateral_m = 0.0
                return last_done
            direction = LEFT if offset > 0.0 else RIGHT
            body_direction = self._map_lateral_to_body_direction(direction)
            done = self._execute_segment(
                direction_to_angle(body_direction),
                abs(offset),
                {side_sensor_for_direction(body_direction)},
                mission,
                label=f"return to center {self._avoidance_direction_label(direction)}",
            )
            last_done = done
            if done.get("result") == "ir_stop":
                last_done = self._execute_side_path_escape(done.get("ir", []), action_budget, mission)
        raise RuntimeError("could not return to center")

    def _finish_front_avoidance_after_corner(
        self,
        direction: int,
        corner_sensor: str,
        action_budget: list[int],
        mission: MissionMemory,
    ) -> dict:
        done = self._execute_strafe_until_corner_falling(direction, corner_sensor, mission)
        if done.get("result") == "ir_stop":
            return self._handle_ir_stop(done.get("ir", []), action_budget, mission)
        body_direction = self._map_lateral_to_body_direction(direction)
        side_sensor = side_sensor_after_front_avoidance(body_direction)
        done = self._execute_forward_until_side_falling(side_sensor, mission)
        if done.get("result") == "ir_stop":
            return self._handle_ir_stop(done.get("ir", []), action_budget, mission)
        done = self._execute_segment(
            0.0,
            self.args.front_advance_distance,
            ALL_WATCH_SENSORS,
            mission,
            label="side clear forward buffer",
            move_timeout_s=self.args.front_advance_timeout,
            timeout_returns_done=True,
        )
        if done.get("result") == "ir_stop":
            return self._handle_ir_stop(done.get("ir", []), action_budget, mission)
        return self._execute_recenter(action_budget, mission)

    def _execute_front_corner_avoidance(
        self,
        ir_state: dict[str, bool],
        action_budget: list[int],
        mission: MissionMemory,
    ) -> dict:
        action_budget[0] -= 1
        if action_budget[0] < 0:
            raise RuntimeError("too many avoidance actions")
        mission.reset_lateral_reference(self.pose)
        self._publish_event("front corner avoidance lateral reference set")
        direction, reason = self._choose_front_avoidance(ir_state)
        body_direction = self._map_lateral_to_body_direction(direction)
        if ir_state.get(side_sensor_for_direction(body_direction), False):
            direction = opposite_direction(direction)

        if ir_state.get("front_left", False) and not ir_state.get("front_right", False):
            corner_sensor = "front_left"
        elif ir_state.get("front_right", False) and not ir_state.get("front_left", False):
            corner_sensor = "front_right"
        else:
            return self._execute_front_avoidance(ir_state, action_budget, mission)

        self._publish_event(
            f"front corner avoidance {reason}: strafe {self._avoidance_direction_label(direction)} until {corner_sensor} clears"
        )
        return self._finish_front_avoidance_after_corner(direction, corner_sensor, action_budget, mission)

    def _execute_front_avoidance(
        self,
        ir_state: dict[str, bool],
        action_budget: list[int],
        mission: MissionMemory,
        *,
        reset_lateral_reference: bool = True,
    ) -> dict:
        action_budget[0] -= 1
        if action_budget[0] < 0:
            raise RuntimeError("too many avoidance actions")
        if reset_lateral_reference:
            mission.reset_lateral_reference(self.pose)
            self._publish_event("front avoidance lateral reference set")
        direction, reason = self._choose_front_avoidance(ir_state)
        body_direction = self._map_lateral_to_body_direction(direction)
        if ir_state.get(side_sensor_for_direction(body_direction), False):
            direction = opposite_direction(direction)
        self._publish_event(f"front avoidance {reason}: strafe {self._avoidance_direction_label(direction)}")
        done, direction, corner_sensor = self._execute_front_search_strafe(direction, mission)
        if done.get("result") == "ir_stop" and corner_sensor not in set(done.get("ir", [])):
            return self._handle_ir_stop(done.get("ir", []), action_budget, mission)
        return self._finish_front_avoidance_after_corner(direction, corner_sensor, action_budget, mission)

    def _handle_ir_stop(self, active_sensors: list[str], action_budget: list[int], mission: MissionMemory) -> dict:
        ir_state = {name: False for name in IR_SENSOR_ORDER}
        for name in active_sensors:
            ir_state[name] = True
        corner_hits = [name for name in FRONT_CORNER_SENSORS if ir_state.get(name, False)]
        if not ir_state.get("front", False) and len(corner_hits) == 1:
            return self._execute_front_corner_avoidance(ir_state, action_budget, mission)
        if any(ir_state.get(name, False) for name in FRONT_WATCH_SENSORS):
            still_front, ir_state = self._wait_for_dynamic_front_clear(
                sorted(name for name in FRONT_WATCH_SENSORS if ir_state.get(name, False))
            )
            if not still_front:
                side_hits = sorted(name for name in SIDE_WATCH_SENSORS if ir_state.get(name, False))
                if side_hits:
                    done = self._execute_side_path_escape(side_hits, action_budget, mission)
                    if done.get("result") == "ir_stop":
                        return self._handle_ir_stop(done.get("ir", []), action_budget, mission)
                    return done
                return MoveDone(result="front_dynamic_clear", heading_deg=self._current_heading_deg()).as_dict()
            return self._execute_front_avoidance(ir_state, action_budget, mission)
        if any(ir_state.get(name, False) for name in SIDE_WATCH_SENSORS):
            done = self._execute_side_path_escape(active_sensors, action_budget, mission)
            if done.get("result") == "ir_stop":
                return self._handle_ir_stop(done.get("ir", []), action_budget, mission)
            return done
        return MoveDone(result="ignored_ir", heading_deg=self._current_heading_deg(), ir=active_sensors).as_dict()

    def _execute_turnaround_forward(
        self,
        distance_m: float,
        watch_sensors: set[str],
        mission: MissionMemory | None,
        *,
        label: str,
        move_timeout_s: float | None = None,
        timeout_returns_done: bool = False,
    ) -> dict:
        self._publish_event(f"{label}: face heading 180 then move forward")
        self._execute_rotation_to_heading(HEADING_BACKWARD_DEG, f"{label} turn around")
        return self._execute_segment(
            HEADING_FORWARD_DEG,
            distance_m,
            watch_sensors,
            mission,
            label=f"{label} forward",
            move_timeout_s=move_timeout_s,
            timeout_returns_done=timeout_returns_done,
        )

    def _mission_direction_move(self, direction: str, distance_cm: float, timeout_s: float) -> dict:
        direction = direction.upper()
        mission = MissionMemory()
        action_budget = [self.args.max_avoidance_actions]
        move_timeout_s = timeout_s if timeout_s > 0.0 else self.args.move_timeout
        distance_m = max(0.0, distance_cm) / 100.0
        if direction == "B":
            done = self._execute_turnaround_forward(
                distance_m,
                ALL_WATCH_SENSORS,
                mission,
                label="mission B",
                move_timeout_s=move_timeout_s,
                timeout_returns_done=True,
            )
        else:
            angle = DIRECTION_ANGLES_DEG[direction]
            done = self._execute_segment(
                angle,
                distance_m,
                ALL_WATCH_SENSORS,
                mission,
                label=f"mission {direction}",
                move_timeout_s=move_timeout_s,
                timeout_returns_done=True,
            )
        if done.get("result") == "ir_stop":
            done = self._handle_ir_stop(done.get("ir", []), action_budget, mission)
        return done

    def _mission_world_direction_move(self, direction: str, distance_cm: float, timeout_s: float) -> dict:
        direction = direction.upper()
        body_direction = self._map_direction_to_body_direction(direction)
        if body_direction != direction:
            self._publish_event(
                f"world move heading {self.args.heading:.1f}deg converts {direction} to body {body_direction}"
            )
        return self._mission_direction_move(body_direction, distance_cm, timeout_s)

    @staticmethod
    def _lateral_forward_heading_and_bias(direction: str) -> tuple[float, int]:
        direction = direction.upper()
        if direction == "R":
            return forward_heading_for_world_direction("R"), RIGHT
        if direction == "L":
            return forward_heading_for_world_direction("L"), LEFT
        raise RuntimeError(f"lateral forward move requires L/R, got {direction}")

    def _execute_lateral_as_forward(
        self,
        direction: str,
        distance_cm: float,
        *,
        label: str,
        bias_note: str,
    ) -> dict:
        heading_deg, bias = self._lateral_forward_heading_and_bias(direction)
        distance_cm = abs(float(distance_cm))
        self._publish_event(
            f"{label}: face {heading_deg:.0f}deg, forward {distance_cm:.1f}cm; "
            f"front avoidance bias {direction_name(bias)} {bias_note}"
        )
        previous_bias = self.front_avoidance_bias_override
        self.front_avoidance_bias_override = bias
        try:
            self._execute_rotation_to_heading(heading_deg, f"{label} face lateral travel")
            return self._mission_direction_move("F", distance_cm, 0.0)
        finally:
            self.front_avoidance_bias_override = previous_bias

    def _reset_map_pose(self) -> None:
        self.map_pose = AudixStoreMap.SPAWN
        self._publish_event(
            f"map pose reset x={self.map_pose.x_cm:.1f} y={self.map_pose.y_cm:.1f}"
        )

    def _raise_if_cancelled(self) -> None:
        if self.cancel_mission.is_set():
            raise RuntimeError("mission cancelled")

    def _execute_mapped_move(self, direction: str, distance_cm: float, target: MapPoint, label: str) -> None:
        self._raise_if_cancelled()
        distance_cm = abs(float(distance_cm))
        if distance_cm <= 0.25:
            self.map_pose = target
            return

        self._publish_event(
            f"map move {label}: {direction} {distance_cm:.1f}cm -> x={target.x_cm:.1f} y={target.y_cm:.1f}"
        )
        done = self._mission_world_direction_move(direction, distance_cm, 0.0)
        result = str(done.get("result", ""))
        if result not in MAP_MOVE_OK_RESULTS:
            raise RuntimeError(f"map move {label} failed: {result or done.get('message', 'unknown')}")
        self.map_pose = target

    def _move_map_x(self, target_x_cm: float, label: str) -> None:
        dx = float(target_x_cm) - self.map_pose.x_cm
        if abs(dx) <= 0.25:
            self.map_pose = MapPoint(float(target_x_cm), self.map_pose.y_cm)
            return

        direction = "R" if dx > 0.0 else "L"
        distance_cm = abs(dx)
        target = MapPoint(float(target_x_cm), self.map_pose.y_cm)
        self._publish_event(
            "map lane shift "
            f"{label}: {direction} {distance_cm:.1f}cm "
            f"-> x={target.x_cm:.1f} y={target.y_cm:.1f}"
        )
        done = self._execute_lateral_as_forward(
            direction,
            distance_cm,
            label=f"map lane shift {label}",
            bias_note="toward shelf",
        )

        result = str(done.get("result", ""))
        if result not in MAP_MOVE_OK_RESULTS:
            raise RuntimeError(f"map lane shift {label} failed: {result or done.get('message', 'unknown')}")
        self.map_pose = target

    def _move_map_y(self, target_y_cm: float, label: str) -> None:
        dy = float(target_y_cm) - self.map_pose.y_cm
        if abs(dy) <= 0.25:
            self.map_pose = MapPoint(self.map_pose.x_cm, float(target_y_cm))
            return
        direction = "B" if dy > 0.0 else "F"
        self._execute_mapped_move(
            direction,
            abs(dy),
            MapPoint(self.map_pose.x_cm, float(target_y_cm)),
            label,
        )

    def _go_to_audit_side(self, side: int) -> None:
        self.current_audit_side = int(side)
        lane_x = AudixStoreMap.LANE_CENTER_X_CM[side]
        side_name = AudixStoreMap.SIDE_NAME[side]
        self._publish_event(
            f"target {side_name}: lane_x={lane_x:.1f} audit_y={AudixStoreMap.AUDIT_Y_CM:.1f}"
        )
        self._execute_rotation_to_heading(HEADING_FORWARD_DEG, f"{side_name} lane travel face forward")
        self._move_map_y(AudixStoreMap.TOP_TRAVEL_Y_CM, "top clear corridor")
        self._move_map_x(lane_x, f"{side_name} lane center")
        self._execute_rotation_to_heading(HEADING_FORWARD_DEG, f"{side_name} audit row face forward")
        self._move_map_y(AudixStoreMap.AUDIT_Y_CM, f"{side_name} audit row")

    def _execute_rotation_in_place(self, direction: str, degrees: float, label: str) -> None:
        self._raise_if_cancelled()
        direction = direction.lower()
        target = self.args.heading + rotation_delta_for_turn_direction(direction, degrees)
        timeout_s = max(10.0, abs(float(degrees)) / 18.0 + 5.0)
        self._publish_event(f"rotate {label}: {direction} {degrees:.1f}deg")
        future = self._send_move_future(0.0, 0.0, target, timeout_s)
        result = self._wait_future_response(future, timeout_s)
        if result is None:
            self._stop_robot()
            raise TimeoutError(f"timed out rotating {label}")
        done = self._move_done_from_response(result)
        if done.get("result") != "completed":
            raise RuntimeError(f"rotation {label} failed: {done.get('message', done.get('result', 'unknown'))}")
        self.args.heading = float(done.get("headingDeg", target))

    def _execute_rotation_to_heading(self, target_heading_deg: float, label: str) -> None:
        self._raise_if_cancelled()
        target = wrap_degrees(float(target_heading_deg))
        current = wrap_degrees(self.args.heading)
        error = wrap_degrees(target - current)
        if abs(error) <= 2.0:
            self._publish_event(f"rotate {label}: already at target {target:.1f}deg")
            return

        timeout_s = max(10.0, abs(error) / 18.0 + 5.0)
        self._publish_event(f"rotate {label}: target={target:.1f}deg error={error:.1f}deg")
        future = self._send_move_future(0.0, 0.0, target, timeout_s)
        result = self._wait_future_response(future, timeout_s)
        if result is None:
            self._stop_robot()
            raise TimeoutError(f"timed out rotating {label}")
        done = self._move_done_from_response(result)
        if done.get("result") != "completed":
            raise RuntimeError(f"rotation {label} failed: {done.get('message', done.get('result', 'unknown'))}")
        self.args.heading = float(done.get("headingDeg", target))

    def _capture_level_placeholder(self, side: int, level: int) -> None:
        self._raise_if_cancelled()
        self._publish_event(f"audit {AudixStoreMap.SIDE_NAME[side]} level {level}: vision placeholder")
        time.sleep(2.0)

    def _scan_shelf_level(self, side: int, level: int) -> None:
        self._raise_if_cancelled()
        shelf_id = self.audit_shelf_ids.get((side, level), "")
        if not shelf_id:
            raise RuntimeError(f"no shelf id configured for side {side} level {level}")

        self._publish_event(
            f"ready facing {AudixStoreMap.SIDE_NAME[side]} level {level}: trigger scan {shelf_id}"
        )
        if self.scan_settle_s > 0.0:
            time.sleep(self.scan_settle_s)
        req = ShelfScan.Request()
        req.shelf_id = shelf_id

        try:
            result = self._call_sync(self.scan_client, req, self.scan_timeout_s)
        except Exception as exc:
            if self.allow_placeholder_audit:
                self._publish_event(
                    f"vision unavailable for {shelf_id}: {exc}; using placeholder wait"
                )
                self._capture_level_placeholder(side, level)
                return
            raise

        if not result.success:
            self._publish_scan_result(side, level, result)
            if self.allow_placeholder_audit:
                self._publish_event(
                    f"vision scan failed for {shelf_id}: {result.message}; using placeholder wait"
                )
                self._capture_level_placeholder(side, level)
                return
            raise RuntimeError(f"vision scan failed for {shelf_id}: {result.message}")

        self._publish_event(
            "vision "
            f"{shelf_id}: {result.status} "
            f"expected={result.expected_product} "
            f"count={result.detected_count}/{result.expected_count} "
            f"confidence={float(result.confidence):.2f} "
            f"wrong={list(result.wrong_products)}"
        )
        self._publish_scan_result(side, level, result)

    def _publish_scan_result(self, side: int, level: int, result) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "success": bool(result.success),
                "side": int(side),
                "level": int(level),
                "shelf_id": str(result.shelf_id),
                "expected_product": str(result.expected_product),
                "expected_count": int(result.expected_count),
                "detected_count": int(result.detected_count),
                "detected_products": list(result.detected_products),
                "wrong_products": list(result.wrong_products),
                "confidence": float(result.confidence),
                "status": str(result.status),
                "message": str(result.message),
                "image_path": str(result.image_path),
            }
        )
        self.scan_result_pub.publish(msg)

    def _audit_side_levels(self, side: int, levels: list[int]) -> None:
        if 1 in levels:
            self._scan_shelf_level(side, 1)
        if 2 in levels:
            self._run_lift(self.lift_steps, 1)
            try:
                self._scan_shelf_level(side, 2)
            finally:
                self._run_lift(self.lift_steps, -1)

    def _run_map_audit(self, sides: list[int], levels: list[int]) -> None:
        try:
            with self.mode_lock:
                self.mode = "mission"
            self.current_audit_side = None
            self._reset_map_pose()
            self._publish_event(
                "map mission start "
                f"size={AudixStoreMap.WIDTH_CM:.0f}x{AudixStoreMap.HEIGHT_CM:.0f}cm "
                f"sides={sides} levels={levels}"
            )

            for index, side in enumerate(sides):
                self._raise_if_cancelled()
                final_side = index == len(sides) - 1
                self._go_to_audit_side(side)
                scan_heading = AudixStoreMap.SCAN_HEADING_DEG[side]
                self._execute_rotation_to_heading(scan_heading, f"face {AudixStoreMap.SIDE_NAME[side]}")
                side_scans_complete = False
                try:
                    self._audit_side_levels(side, levels)
                    side_scans_complete = True
                finally:
                    if not self.cancel_mission.is_set():
                        if final_side and side_scans_complete:
                            self._execute_rotation_to_heading(HEADING_BACKWARD_DEG, "final scan face lane start")
                        else:
                            self._execute_rotation_to_heading(HEADING_FORWARD_DEG, "return forward before next lane")

            self._execute_rotation_to_heading(HEADING_BACKWARD_DEG, "mission complete face home")
            self._go_home_to_odom_zero("mission complete home")
            self._publish_event(
                f"audit mission complete x={self.map_pose.x_cm:.1f} y={self.map_pose.y_cm:.1f}"
            )
        except Exception as exc:
            self._stop_robot()
            self._set_buzzer(False)
            self._publish_event(f"audit mission stopped: {exc}")
        finally:
            self.mission_running = False
            self.current_audit_side = None
            with self.mode_lock:
                self.mode = "manual"

    def _handle_set_mode(self, request: SetRobotMode.Request, response: SetRobotMode.Response) -> SetRobotMode.Response:
        mode = request.mode.strip().lower()
        if mode not in {"manual", "mission", "idle"}:
            response.ok = False
            response.message = "mode must be manual, mission, or idle"
            response.active_mode = self.mode
            return response
        with self.mode_lock:
            self.mode = mode
        if mode != "mission":
            self.cancel_mission.set()
        response.ok = True
        response.message = f"mode set to {mode}"
        response.active_mode = mode
        self._publish_event(response.message)
        return response

    def _handle_direction_move(
        self,
        request: DirectionCommand.Request,
        response: DirectionCommand.Response,
    ) -> DirectionCommand.Response:
        direction = request.direction.strip().upper()
        if direction not in DIRECTION_ANGLES_DEG:
            response.ok = False
            response.result = "invalid_direction"
            response.message = "direction must be F, B, R, L, FR, FL, BR, or BL"
            return response

        with self.motion_lock:
            try:
                with self.mode_lock:
                    mode = self.mode
                if mode == "manual":
                    active = self._active_ir()
                    if active:
                        self._stop_robot()
                        self._set_buzzer(True)
                        self.manual_buzzer_until = time.monotonic() + self.buzzer_hold_s
                        response.ok = False
                        response.result = "manual_ir_stop"
                        response.message = f"manual obstacle stop: {active}"
                        return response
                    angle = DIRECTION_ANGLES_DEG[direction]
                    done = self._execute_segment(
                        angle,
                        max(0.0, float(request.distance_cm)) / 100.0,
                        set(),
                        None,
                        label=f"manual {direction}",
                        move_timeout_s=float(request.timeout_s) if request.timeout_s > 0.0 else self.args.move_timeout,
                        timeout_returns_done=True,
                    )
                else:
                    done = self._mission_direction_move(direction, float(request.distance_cm), float(request.timeout_s))
            except Exception as exc:
                self._stop_robot()
                self.get_logger().error(f"direction move failed: {exc}")
                response.ok = False
                response.result = "error"
                response.message = str(exc)
                response.forward_cm = 0.0
                response.strafe_cm = 0.0
                response.heading_deg = self._current_heading_deg()
                return response

        response.ok = done.get("result") in {"completed", "front_dynamic_clear", "side_falling", "corner_falling", "none"}
        response.result = str(done.get("result", ""))
        response.message = str(done.get("message", response.result))
        response.forward_cm = float(done.get("forwardCm", 0.0))
        response.strafe_cm = float(done.get("strafeCm", 0.0))
        response.heading_deg = float(done.get("headingDeg", self._current_heading_deg()))
        return response

    def _handle_rotate(self, request: RotateCommand.Request, response: RotateCommand.Response) -> RotateCommand.Response:
        direction = request.direction.strip().lower()
        if direction not in {"left", "right", "ccw", "cw"}:
            response.ok = False
            response.result = "invalid_direction"
            response.message = "direction must be left/right or ccw/cw"
            response.heading_deg = self._current_heading_deg()
            return response
        target = self._current_heading_deg() + rotation_delta_for_turn_direction(direction, request.degrees)

        with self.motion_lock:
            try:
                active = self._active_ir() if self.mode == "manual" else []
                if active:
                    self._stop_robot()
                    self._set_buzzer(True)
                    self.manual_buzzer_until = time.monotonic() + self.buzzer_hold_s
                    response.ok = False
                    response.result = "manual_ir_stop"
                    response.message = f"manual obstacle stop: {active}"
                    response.heading_deg = self._current_heading_deg()
                    return response
                self._publish_event(f"rotate {direction} {request.degrees:.1f}deg target={target:.1f}")
                future = self._send_move_future(0.0, 0.0, target, float(request.timeout_s) if request.timeout_s > 0.0 else 10.0)
                result = self._wait_future_response(future, float(request.timeout_s) if request.timeout_s > 0.0 else 10.0)
                if result is not None:
                    done = self._move_done_from_response(result)
                else:
                    self._stop_robot()
                    done = MoveDone(result="timeout_stop", heading_deg=self._current_heading_deg()).as_dict()
            except Exception as exc:
                self._stop_robot()
                self.get_logger().error(f"rotate failed: {exc}")
                response.ok = False
                response.result = "error"
                response.message = str(exc)
                response.heading_deg = self._current_heading_deg()
                return response

        response.ok = done.get("result") == "completed"
        response.result = str(done.get("result", ""))
        response.message = str(done.get("message", response.result))
        response.heading_deg = float(done.get("headingDeg", self._current_heading_deg()))
        return response

    def _handle_start_audit(self, request: AuditMission.Request, response: AuditMission.Response) -> AuditMission.Response:
        sides = [int(side) for side in request.shelves if 1 <= int(side) <= 2]
        levels = []
        if request.level_1:
            levels.append(1)
        if request.level_2:
            levels.append(2)
        if not sides:
            response.accepted = False
            response.message = "select at least one shelf side"
            return response
        if not levels:
            response.accepted = False
            response.message = "select at least one level"
            return response

        if self.mission_running:
            response.accepted = False
            response.message = "mission already running"
            return response
        self.cancel_mission.clear()
        self.mission_running = True
        threading.Thread(target=self._run_map_audit, args=(sides, levels), daemon=True).start()
        response.accepted = True
        response.message = f"audit mission started sides={sides} levels={levels}"
        return response

    def _wait_for_telemetry(
        self,
        *,
        newer_than_s: float | None = None,
        timeout_s: float = 2.0,
    ) -> EspTelemetry:
        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() <= deadline:
            telemetry = self.latest_telemetry
            if telemetry is not None and (
                newer_than_s is None or self.latest_telemetry_time_s > newer_than_s
            ):
                return telemetry
            time.sleep(0.02)
        if newer_than_s is None and self.latest_telemetry is not None:
            return self.latest_telemetry
        if newer_than_s is None:
            raise RuntimeError("cannot home before telemetry is available")
        raise RuntimeError("timed out waiting for fresh odom telemetry")

    def _go_home_to_odom_zero(self, label: str) -> None:
        self._raise_if_cancelled()
        tolerance_cm = max(0.5, self.home_tolerance_cm)
        max_passes = max(1, self.home_max_passes)
        self._wait_for_telemetry(timeout_s=2.0)
        forward_cm = float(self.pose.world_forward_cm)
        strafe_cm = float(self.pose.world_strafe_cm)
        self._publish_event(
            f"{label}: world odom forward={forward_cm:.1f}cm strafe={strafe_cm:.1f}cm"
        )

        for home_pass in range(1, max_passes + 1):
            self._raise_if_cancelled()
            self._wait_for_telemetry(timeout_s=1.0)
            forward_cm = float(self.pose.world_forward_cm)
            strafe_cm = float(self.pose.world_strafe_cm)
            if abs(forward_cm) <= tolerance_cm and abs(strafe_cm) <= tolerance_cm:
                break

            if abs(forward_cm) > tolerance_cm:
                direction = "B" if forward_cm > 0.0 else "F"
                distance_cm = abs(forward_cm)
                self._publish_event(
                    f"{label}: home pass {home_pass} world forward {direction} {distance_cm:.1f}cm"
                )
                done = self._mission_world_direction_move(direction, distance_cm, 0.0)
                result = str(done.get("result", ""))
                if result not in MAP_MOVE_OK_RESULTS:
                    raise RuntimeError(
                        f"home forward correction failed: {result or done.get('message', 'unknown')}"
                    )
                completed_s = time.monotonic()
                if self.home_settle_s > 0.0:
                    time.sleep(self.home_settle_s)
                self._wait_for_telemetry(newer_than_s=completed_s, timeout_s=3.0)
                continue

            direction = "R" if strafe_cm > 0.0 else "L"
            distance_cm = abs(strafe_cm)
            self._publish_event(
                f"{label}: home pass {home_pass} world lateral {direction} {distance_cm:.1f}cm"
            )
            done = self._execute_lateral_as_forward(
                direction,
                distance_cm,
                label=f"{label}: home pass {home_pass} lateral {direction}",
                bias_note="during home return",
            )
            result = str(done.get("result", ""))
            if result not in MAP_MOVE_OK_RESULTS:
                raise RuntimeError(
                    f"home lateral correction failed: {result or done.get('message', 'unknown')}"
                )
            completed_s = time.monotonic()
            if self.home_settle_s > 0.0:
                time.sleep(self.home_settle_s)
            self._wait_for_telemetry(newer_than_s=completed_s, timeout_s=3.0)

        if self.home_settle_s > 0.0:
            time.sleep(self.home_settle_s)
        self._wait_for_telemetry(timeout_s=1.0)
        forward_cm = float(self.pose.world_forward_cm)
        strafe_cm = float(self.pose.world_strafe_cm)
        if abs(forward_cm) > tolerance_cm or abs(strafe_cm) > tolerance_cm:
            raise RuntimeError(
                f"home residual world odom forward={forward_cm:.1f}cm strafe={strafe_cm:.1f}cm"
            )

        self._execute_rotation_to_heading(HEADING_FORWARD_DEG, f"{label} face forward")

        self.map_pose = AudixStoreMap.SPAWN
        self.pose.reset_world()
        self._publish_event(
            f"home reached world odom zero forward={forward_cm:.1f}cm strafe={strafe_cm:.1f}cm"
        )

    def _run_lift(self, steps: int, direction: int) -> None:
        req = LiftMoveSteps.Request()
        abs_steps = abs(int(steps))
        req.steps = abs_steps
        req.direction = 1 if direction >= 0 else -1
        req.speed_sps = self.lift_speed_sps
        self._call_sync(self.lift_client, req, max(5.0, abs_steps / max(1.0, self.lift_speed_sps) + 2.0))

    def _handle_go_home(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self.mission_running:
            response.success = False
            response.message = "cannot home while audit mission is running"
            return response
        with self.motion_lock:
            try:
                with self.mode_lock:
                    previous_mode = self.mode
                    self.mode = "mission"
                self.cancel_mission.clear()
                self._go_home_to_odom_zero("manual home")
                response.success = True
                response.message = "home reached odom zero"
            except Exception as exc:
                self._stop_robot()
                response.success = False
                response.message = str(exc)
            finally:
                with self.mode_lock:
                    self.mode = previous_mode if "previous_mode" in locals() else "manual"
        return response

    def _handle_manager_stop(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.cancel_mission.set()
        self._stop_robot()
        self._set_buzzer(False)
        with self.mode_lock:
            self.mode = "manual"
        response.success = True
        response.message = "robot stopped and manager set to manual"
        self._publish_event(response.message)
        return response


def main() -> None:
    rclpy.init()
    node = RobotManager()
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
