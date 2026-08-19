"""racer_tools.raceline_path_builder: pure geometry helpers for turning loaded raceline
points into the plain (x, y, quaternion-z, quaternion-w) fields a nav_msgs/Path needs
(roadmap milestone 3).

ROS-free (no geometry_msgs/nav_msgs import) so this is L1 unit-testable with no ROS
install, same split as sim/bridge/racer_gym_bridge/racer_gym_bridge/conversions.py (pure
"fields" builders, tested at L1) vs. that package's bridge_node.py (assembles the actual
ROS message from those fields, verified at L3 instead) -- racer_tools.raceline_publisher_node
is the thin ROS-plumbing side of that same split here.
"""

from __future__ import annotations

import math
from typing import NamedTuple

from racer_tools.raceline_loader import RacelinePoint


class PathPointFields(NamedTuple):
    x_m: float
    y_m: float
    orientation_z: float
    orientation_w: float


def yaw_to_quaternion_zw(yaw_rad: float) -> tuple[float, float]:
    """Planar (yaw-only) quaternion (qx=qy=0), returned as (qz, qw)."""
    return math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def raceline_path_fields(points: list[RacelinePoint]) -> list[PathPointFields]:
    """One PathPointFields per raceline point, in order, PLUS a repeated first point at the
    end so the published path visually closes the loop -- the raceline itself is a closed
    loop (racer_control/include/racer_control/raceline.hpp treats it as one, wrapping
    `advance_to_lookahead`/lap-progress arithmetic in tests/l5_tracker_lap around the same
    closing segment), but the raw CSV never repeats its first row, so a plain point-by-point
    Path would otherwise render with a visible gap at the start/finish line.

    Returns an empty list for an empty input (no closing point to add either)."""
    fields = []
    for p in points:
        qz, qw = yaw_to_quaternion_zw(p.heading_rad)
        fields.append(PathPointFields(p.x_m, p.y_m, qz, qw))
    if points:
        first = points[0]
        qz, qw = yaw_to_quaternion_zw(first.heading_rad)
        fields.append(PathPointFields(first.x_m, first.y_m, qz, qw))
    return fields
