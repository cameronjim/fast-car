"""Pure conversion logic for the gym<->ROS bridge.

Deliberately free of ROS (rclpy, message types) and f1tenth_gym imports --
only stdlib math and numpy -- so this module is testable as an ordinary L1
unit test (claude-docs/12-testing.md) with no ROS or gym installation.
`bridge_node.py` is the only module in this package that touches rclpy,
ROS message types, or f1tenth_gym; it plugs these pure functions' outputs
into the actual message objects.

SI units and REP-103 frames throughout, per claude-docs/06-vehicle-params.md
and claude-docs/04-architecture.md: metres, seconds, radians; z-up,
right-handed.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np


def drive_cmd_to_action(steering_angle: float, speed: float) -> np.ndarray:
    """Build the (1, 2) f1tenth_gym action array for one ego agent.

    f1tenth_gym's per-agent action row is ``[steering_angle_rad, speed_mps]``
    regardless of the env's ``control_input`` config order. Verified
    empirically against the pinned commit
    (5a301bd0ae1ceaf7dec653e7549c8d099db58a6b): the default
    ``control_input`` is ``["speed", "steering_angle"]``, but
    ``CarAction.act()`` always reads ``action[0]`` as the steer command and
    ``action[1]`` as the speed command -- the config list only selects
    *which* longitudinal/steering controller types are used, not the
    action array's column order.
    """
    return np.array([[float(steering_angle), float(speed)]], dtype=np.float64)


def yaw_to_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    """Planar yaw (REP-103, z-up right-handed) -> (x, y, z, w) quaternion."""
    half = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Inverse of `yaw_to_quaternion` for a planar (x=y=0) quaternion.

    Used only by tests to check the round trip; the bridge itself never
    needs to go from quaternion back to yaw.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass(frozen=True)
class ScanFields:
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: list[float]


def build_scan_fields(
    ranges: Sequence[float],
    fov_rad: float,
    range_min: float,
    range_max: float,
) -> ScanFields:
    """Compute sensor_msgs/LaserScan field values from a raw gym scan.

    ``fov_rad``/``range_max`` should come from the gym's own
    ``ScanSimulator2D`` at runtime (``num_beams``, ``fov``, ``max_range``) --
    never hand-write a second copy of these. They are the LiDAR-equivalent
    of a physical constant; task 0.7's vehicle_params.yaml does not cover
    the sim LiDAR model, so the caller reads the gym's live config instead
    (see CLAUDE.md hard invariant 2).
    """
    ranges_list = [float(r) for r in ranges]
    num_beams = len(ranges_list)
    if num_beams < 2:
        raise ValueError("scan must have at least 2 beams to compute angle_increment")
    angle_increment = fov_rad / (num_beams - 1)
    return ScanFields(
        angle_min=-fov_rad / 2.0,
        angle_max=fov_rad / 2.0,
        angle_increment=angle_increment,
        range_min=float(range_min),
        range_max=float(range_max),
        ranges=ranges_list,
    )


@dataclass(frozen=True)
class OdomFields:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    linear: tuple[float, float, float]
    angular: tuple[float, float, float]


def build_odom_fields(
    pose_x: float,
    pose_y: float,
    yaw_rad: float,
    vx: float,
    vy: float,
    yaw_rate: float,
) -> OdomFields:
    """Compute nav_msgs/Odometry field values from gym ground-truth state.

    ``vx``/``vy`` are already body-frame (child_frame_id=base_link)
    longitudinal/lateral velocity components, matching f1tenth_gym's
    ``FeaturesObservation`` (``vx = v * cos(beta)``, ``vy = v * sin(beta)``);
    no extra frame rotation is needed here.
    """
    return OdomFields(
        position=(float(pose_x), float(pose_y), 0.0),
        orientation=yaw_to_quaternion(yaw_rad),
        linear=(float(vx), float(vy), 0.0),
        angular=(0.0, 0.0, float(yaw_rate)),
    )
