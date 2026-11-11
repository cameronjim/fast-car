"""Lap detection from a pose track. Pure numpy/math, no ROS.

The start line is a segment through the car's position at the moment driving starts,
perpendicular to its heading, `line_length_m` long and centred on the car. A lap completes when
the pose track crosses that segment in the forward direction (the car's heading within
`max_heading_error_rad` of the start heading) after at least `min_lap_length_m` of path since the
last crossing, and only after the car has been at least `min_excursion_m` away from the line
(default a quarter of the minimum lap length). The second condition is what keeps pose noise from
making laps: a parked car with 2 cm of jitter integrates metres of path in a minute, but never
leaves the line. Path length is integrated from the pose samples, never from wheel odometry. The
crossing time is interpolated linearly between the two pose samples around the crossing.

A gap in the pose (the caller simply does not call `update`) neither creates nor loses a lap: the
next sample is tested against the previous one, so a crossing inside the gap is still found, and
the straight line across the gap counts toward the path length.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class LapCrossing:
    """One completed lap."""

    lap_index: int  # 1 for the first completed lap
    t_cross_s: float  # interpolated time of the crossing
    lap_time_s: float
    length_m: float  # path length from the previous crossing (or the start) to this one


def wrap_angle(a: float) -> float:
    """Wrap to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class LapDetector:
    def __init__(
        self,
        min_lap_length_m: float = 3.0,
        line_length_m: float = 1.5,
        max_heading_error_rad: float = math.radians(60.0),
        min_excursion_m: float | None = None,
    ) -> None:
        if min_lap_length_m <= 0.0 or line_length_m <= 0.0 or max_heading_error_rad <= 0.0:
            raise ValueError("lap detector limits must be positive")
        self.min_lap_length_m = float(min_lap_length_m)
        self.line_length_m = float(line_length_m)
        self.max_heading_error_rad = float(max_heading_error_rad)
        self.min_excursion_m = (
            0.25 * self.min_lap_length_m if min_excursion_m is None else float(min_excursion_m)
        )
        if self.min_excursion_m < 0.0:
            raise ValueError("min_excursion_m must not be negative")
        self._max_dist_m = 0.0
        self._started = False
        self._centre: NDArray[np.float64] = np.zeros(2)
        self._heading = 0.0
        self._fwd: NDArray[np.float64] = np.array([1.0, 0.0])
        self._left: NDArray[np.float64] = np.array([0.0, 1.0])
        self._laps = 0
        self._t_lap_start = 0.0
        self._path_m = 0.0
        self._last: tuple[float, float, float, float] | None = None  # t, x, y, yaw

    @property
    def started(self) -> bool:
        return self._started

    @property
    def laps_completed(self) -> int:
        return self._laps

    @property
    def path_since_crossing_m(self) -> float:
        """Path length driven since the start or the last crossing."""
        return self._path_m

    @property
    def start_line(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """The two end points of the start line (valid once started)."""
        half = 0.5 * self.line_length_m * self._left
        a = self._centre + half
        b = self._centre - half
        return (float(a[0]), float(a[1])), (float(b[0]), float(b[1]))

    def set_start(self, t_s: float, x: float, y: float, yaw: float) -> None:
        """Put the start line through this pose and begin lap 1. Resets everything."""
        self._started = True
        self._centre = np.array([x, y], dtype=np.float64)
        self._heading = float(yaw)
        self._fwd = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float64)
        self._left = np.array([-math.sin(yaw), math.cos(yaw)], dtype=np.float64)
        self._laps = 0
        self._t_lap_start = float(t_s)
        self._path_m = 0.0
        self._max_dist_m = 0.0
        self._last = (float(t_s), float(x), float(y), float(yaw))

    def update(self, t_s: float, x: float, y: float, yaw: float) -> LapCrossing | None:
        """Feed one pose. Returns the lap that this sample completed, if any."""
        if not self._started:
            raise RuntimeError("set_start must be called before update")
        assert self._last is not None
        t0, x0, y0, yaw0 = self._last
        if t_s <= t0:
            return None  # not newer: ignore, keep state
        seg = math.hypot(x - x0, y - y0)
        self._max_dist_m = max(
            self._max_dist_m, math.hypot(x - self._centre[0], y - self._centre[1])
        )
        crossing = self._try_cross(t0, x0, y0, yaw0, t_s, x, y, yaw, seg)
        self._last = (t_s, x, y, yaw)
        if crossing is None:
            self._path_m += seg
        return crossing

    def _try_cross(
        self,
        t0: float,
        x0: float,
        y0: float,
        yaw0: float,
        t1: float,
        x1: float,
        y1: float,
        yaw1: float,
        seg: float,
    ) -> LapCrossing | None:
        d0 = (x0 - self._centre[0]) * self._fwd[0] + (y0 - self._centre[1]) * self._fwd[1]
        d1 = (x1 - self._centre[0]) * self._fwd[0] + (y1 - self._centre[1]) * self._fwd[1]
        # Forward crossing of the line's plane: strictly behind, then on or ahead.
        if not (d0 < 0.0 <= d1):
            return None
        f = -d0 / (d1 - d0)  # fraction of the segment at the crossing, in (0, 1]
        cx = x0 + f * (x1 - x0)
        cy = y0 + f * (y1 - y0)
        lateral = (cx - self._centre[0]) * self._left[0] + (cy - self._centre[1]) * self._left[1]
        if abs(lateral) > 0.5 * self.line_length_m:
            return None
        yaw_c = yaw0 + f * wrap_angle(yaw1 - yaw0)
        if abs(wrap_angle(yaw_c - self._heading)) > self.max_heading_error_rad:
            return None
        length = self._path_m + f * seg
        if length < self.min_lap_length_m or self._max_dist_m < self.min_excursion_m:
            return None
        t_cross = t0 + f * (t1 - t0)
        self._laps += 1
        crossing = LapCrossing(
            lap_index=self._laps,
            t_cross_s=t_cross,
            lap_time_s=t_cross - self._t_lap_start,
            length_m=length,
        )
        self._t_lap_start = t_cross
        self._path_m = (1.0 - f) * seg
        self._max_dist_m = 0.0
        return crossing
