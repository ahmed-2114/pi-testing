"""Shared Audix navigation frame contract.

World frame:
  +x / forward_cm: forward from spawn.
  -x: backward toward spawn/home.
  +y / strafe_cm: left.
  -y / strafe_cm: right.

Heading frame:
  0 deg: forward (+x).
  90 deg: left (+y).
  -90 deg: right (-y).
  180 deg: backward (-x).

This is the standard right-handed robotics convention for planar motion:
+z points up, and positive yaw is counter-clockwise from +x toward +y.
Therefore, rotating left/CCW increases yaw and rotating right/CW decreases yaw.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


HEADING_FORWARD_DEG = 0.0
HEADING_LEFT_DEG = 90.0
HEADING_RIGHT_DEG = -90.0
HEADING_BACKWARD_DEG = 180.0
TURN_LEFT_SIGN = 1.0
TURN_RIGHT_SIGN = -1.0

WORLD_FORWARD_AXIS = "+x"
WORLD_BACKWARD_AXIS = "-x"
WORLD_LEFT_AXIS = "+y"
WORLD_RIGHT_AXIS = "-y"

DIRECTION_ANGLES_DEG = {
    "F": HEADING_FORWARD_DEG,
    "FR": -45.0,
    "R": HEADING_RIGHT_DEG,
    "BR": -135.0,
    "B": HEADING_BACKWARD_DEG,
    "BL": 135.0,
    "L": HEADING_LEFT_DEG,
    "FL": 45.0,
}


@dataclass(frozen=True)
class NavigationContract:
    forward_heading_deg: float = HEADING_FORWARD_DEG
    left_heading_deg: float = HEADING_LEFT_DEG
    right_heading_deg: float = HEADING_RIGHT_DEG
    backward_heading_deg: float = HEADING_BACKWARD_DEG
    left_turn_sign: float = TURN_LEFT_SIGN
    right_turn_sign: float = TURN_RIGHT_SIGN
    forward_axis: str = WORLD_FORWARD_AXIS
    backward_axis: str = WORLD_BACKWARD_AXIS
    left_axis: str = WORLD_LEFT_AXIS
    right_axis: str = WORLD_RIGHT_AXIS


NAVIGATION_CONTRACT = NavigationContract()


def wrap_degrees(angle_deg: float) -> float:
    """Wrap an angle to [-180, 180], keeping 180 instead of -180."""
    wrapped = (float(angle_deg) + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def body_delta_to_world(
    delta_forward_cm: float,
    delta_strafe_cm: float,
    heading_deg: float,
    *,
    body_strafe_right_positive: bool = True,
) -> tuple[float, float]:
    """Project a body-frame delta into the Audix world frame.

    The ESP body odometry reports forward as positive forward and strafe as
    positive right. The Audix world frame stores strafe as positive left, so the
    default conversion flips body strafe before applying the standard yaw
    rotation.
    """
    body_forward_cm = float(delta_forward_cm)
    body_left_cm = -float(delta_strafe_cm) if body_strafe_right_positive else float(delta_strafe_cm)
    yaw_rad = math.radians(wrap_degrees(heading_deg))
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    world_forward_cm = cos_yaw * body_forward_cm - sin_yaw * body_left_cm
    world_strafe_cm = sin_yaw * body_forward_cm + cos_yaw * body_left_cm
    return world_forward_cm, world_strafe_cm


def forward_heading_for_world_direction(direction: str) -> float:
    """Return the heading to face before driving forward in a world direction."""
    key = direction.upper()
    if key not in DIRECTION_ANGLES_DEG:
        raise ValueError(f"unknown world direction {direction!r}")
    return DIRECTION_ANGLES_DEG[key]


def rotation_delta_for_turn_direction(direction: str, degrees: float) -> float:
    """Return signed yaw delta for an in-place turn under right-hand-rule yaw."""
    key = direction.lower()
    if key in {"left", "ccw"}:
        return TURN_LEFT_SIGN * abs(float(degrees))
    if key in {"right", "cw"}:
        return TURN_RIGHT_SIGN * abs(float(degrees))
    raise ValueError(f"unknown turn direction {direction!r}")


def world_strafe_sign_for_heading_forward(heading_deg: float) -> int:
    """Return +1 for left/+y, -1 for right/-y, or 0 for non-lateral headings."""
    heading = wrap_degrees(heading_deg)
    if abs(wrap_degrees(heading - HEADING_LEFT_DEG)) <= 1e-6:
        return 1
    if abs(wrap_degrees(heading - HEADING_RIGHT_DEG)) <= 1e-6:
        return -1
    return 0
