"""The oracle track: a ground truth `LocalTrack` from the simulator's own world (measurement M-A2,
pre-registration v0.4 section 3.5, brief 40). Pure numpy, no ROS: `oracle_track_node.py` wraps it.

TEST ONLY. This lives under tests/ and is never installed: the car has no ground truth, and
nothing in ros_ws, racer_bringup or pnp.launch.py may start it (test_oracle_guard.py).

What it computes, in the message's own terms (`racer_pnp_msgs/msg/LocalTrack`, 13-plug-and-play.md
4.1): 16 stations 0.20 m apart along the ground truth midline of the free lane, station 0 the
midline point nearest to the base_link origin and the others further in the direction of travel
(the midline direction within 90 degrees of the heading), in the base_link frame; half widths the
free distance from each station along the midline normal to the first wall (obstacle boxes
included) on the left and right of the travel direction, capped at 2.0 m, read from the occupancy
grid the simulator ray casts, i.e. what its LiDAR can see; every station valid; sigma, boundary
residual and intrusion at their best value (0).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

STATIONS = 16
SPACING_M = 0.20
HALF_WIDTH_CAP_M = 2.0
#: Marching step along a normal when looking for the first wall (m); a fifth of a 0.05 m cell.
MARCH_STEP_M = 0.01
#: The occupancy grid marks a wall with 0 and free floor with 255 (f1tenth_gym's layout).
_OCCUPIED_BELOW = 128.0
#: Best values of the confidence fields (a perfect track): see the message.
BEST_SIGMA_M = 0.0
BEST_BOUNDARY_RESIDUAL_M = 0.0
BEST_INTRUSION_FRACTION = 0.0
MODEL_ID = "oracle-ground-truth"


@dataclass(frozen=True)
class OracleTrackData:
    """The message's arrays, base_link frame."""

    x_m: np.ndarray
    y_m: np.ndarray
    half_width_left_m: np.ndarray
    half_width_right_m: np.ndarray
    sigma_m: np.ndarray
    valid_count: int
    boundary_residual_m: float
    intrusion_fraction: float
    travel_direction: int  # +1 along the stored centreline, -1 against it


class GroundTruthTrack:
    """A closed ground truth midline and the occupancy grid of the same world."""

    def __init__(
        self,
        centreline_x: np.ndarray,
        centreline_y: np.ndarray,
        occupancy: np.ndarray,
        origin_xy: tuple[float, float],
        resolution_m: float,
    ) -> None:
        self._x = np.asarray(centreline_x, dtype=float)
        self._y = np.asarray(centreline_y, dtype=float)
        if len(self._x) < 8 or len(self._x) != len(self._y):
            raise ValueError("the centreline needs at least 8 points")
        step = np.hypot(np.diff(np.r_[self._x, self._x[0]]), np.diff(np.r_[self._y, self._y[0]]))
        self._s = np.r_[0.0, np.cumsum(step[:-1])]
        self.length_m = float(step.sum())
        self._ss = np.r_[self._s, self.length_m]
        self._sx = np.r_[self._x, self._x[0]]
        self._sy = np.r_[self._y, self._y[0]]
        self._occupancy = np.asarray(occupancy)
        self._origin = (float(origin_xy[0]), float(origin_xy[1]))
        self._res = float(resolution_m)

    @classmethod
    def load(cls, path: str | Path) -> GroundTruthTrack:
        """From the .npz the runner writes (`write_ground_truth`)."""
        with np.load(path) as f:
            return cls(
                f["centreline_x"],
                f["centreline_y"],
                f["occupancy"],
                (float(f["origin_x"]), float(f["origin_y"])),
                float(f["resolution_m"]),
            )

    # -- midline ----------------------------------------------------------------------------

    def point_at(self, arc_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Midline x and y at arc positions (periodic)."""
        q = np.asarray(arc_m, dtype=float) % self.length_m
        return np.interp(q, self._ss, self._sx), np.interp(q, self._ss, self._sy)

    def tangent_at(self, arc_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Unit tangent of the stored midline (direction of increasing arc)."""
        q = np.asarray(arc_m, dtype=float)
        x1, y1 = self.point_at(q + 0.05)
        x0, y0 = self.point_at(q - 0.05)
        dx, dy = x1 - x0, y1 - y0
        n = np.hypot(dx, dy)
        return dx / n, dy / n

    def nearest_arc(self, x: float, y: float) -> float:
        """Arc position of the midline point nearest to (x, y), projected onto the two segments
        around the nearest vertex."""
        i = int(np.argmin(np.hypot(self._x - x, self._y - y)))
        best_arc, best_d = float(self._s[i]), math.inf
        n = len(self._x)
        for a, b in (((i - 1) % n, i), (i, (i + 1) % n)):
            ax, ay, bx, by = self._x[a], self._y[a], self._x[b], self._y[b]
            vx, vy = bx - ax, by - ay
            seg2 = vx * vx + vy * vy
            t = 0.0 if seg2 == 0.0 else min(max(((x - ax) * vx + (y - ay) * vy) / seg2, 0.0), 1.0)
            d = math.hypot(x - (ax + t * vx), y - (ay + t * vy))
            if d < best_d:
                best_d, best_arc = d, float(self._s[a] + t * math.sqrt(seg2))
        return best_arc % self.length_m

    # -- walls ------------------------------------------------------------------------------

    def free_distance(
        self, x: np.ndarray, y: np.ndarray, dx: np.ndarray, dy: np.ndarray
    ) -> np.ndarray:
        """Distance along unit directions (dx, dy) from points (x, y) to the first occupied cell
        of the grid, capped at HALF_WIDTH_CAP_M. A point already inside a wall gives 0."""
        d = np.arange(0.0, HALF_WIDTH_CAP_M + 1e-9, MARCH_STEP_M)
        px = x[:, None] + dx[:, None] * d[None, :]
        py = y[:, None] + dy[:, None] * d[None, :]
        col = np.floor((px - self._origin[0]) / self._res).astype(int)
        row = np.floor((py - self._origin[1]) / self._res).astype(int)
        rows, cols = self._occupancy.shape
        inside = (row >= 0) & (row < rows) & (col >= 0) & (col < cols)
        value = np.full(px.shape, 255.0)
        value[inside] = self._occupancy[row[inside], col[inside]]
        hit = value < _OCCUPIED_BELOW
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
        out = np.full(len(x), HALF_WIDTH_CAP_M)
        found = first >= 0
        out[found] = np.maximum(first[found] * MARCH_STEP_M - MARCH_STEP_M / 2.0, 0.0)
        return np.minimum(out, HALF_WIDTH_CAP_M)

    # -- the message ------------------------------------------------------------------------

    def local_track(self, base_x: float, base_y: float, base_yaw: float) -> OracleTrackData:
        """The 16 stations for a car whose base_link origin is at (base_x, base_y) with heading
        base_yaw in the world frame."""
        s0 = self.nearest_arc(base_x, base_y)
        tx, ty = self.tangent_at(np.array([s0]))
        direction = 1 if (tx[0] * math.cos(base_yaw) + ty[0] * math.sin(base_yaw)) >= 0.0 else -1
        arcs = s0 + direction * SPACING_M * np.arange(STATIONS)
        px, py = self.point_at(arcs)
        tx, ty = self.tangent_at(arcs)
        tx, ty = direction * tx, direction * ty  # travel direction
        lx, ly = -ty, tx  # left of travel
        left = self.free_distance(px, py, lx, ly)
        right = self.free_distance(px, py, -lx, -ly)
        dx, dy = px - base_x, py - base_y
        c, s = math.cos(base_yaw), math.sin(base_yaw)
        return OracleTrackData(
            x_m=c * dx + s * dy,
            y_m=-s * dx + c * dy,
            half_width_left_m=left,
            half_width_right_m=right,
            sigma_m=np.full(STATIONS, BEST_SIGMA_M),
            valid_count=STATIONS,
            boundary_residual_m=BEST_BOUNDARY_RESIDUAL_M,
            intrusion_fraction=BEST_INTRUSION_FRACTION,
            travel_direction=direction,
        )


def write_ground_truth(
    path: str | Path,
    centreline_x: np.ndarray,
    centreline_y: np.ndarray,
    occupancy: np.ndarray,
    origin_xy: tuple[float, float],
    resolution_m: float,
) -> None:
    """The file `oracle_track_node` reads: the midline in the driving direction and the grid."""
    np.savez(
        path,
        centreline_x=np.asarray(centreline_x, dtype=float),
        centreline_y=np.asarray(centreline_y, dtype=float),
        occupancy=np.asarray(occupancy, dtype=np.uint8),
        origin_x=float(origin_xy[0]),
        origin_y=float(origin_xy[1]),
        resolution_m=float(resolution_m),
    )


def base_pose_from_cg(
    cg_x: float, cg_y: float, yaw: float, cg_to_rear_axle_m: float
) -> tuple[float, float]:
    """base_link (rear axle centre) from the simulator's ground truth pose, which is the CG."""
    return cg_x - cg_to_rear_axle_m * math.cos(yaw), cg_y - cg_to_rear_axle_m * math.sin(yaw)


def interpolate_pose(
    samples: list[tuple[float, float, float, float]], t: float
) -> tuple[float, float, float] | None:
    """(x, y, yaw) at time t from time ordered (t, x, y, yaw) samples: linear between the two
    around t, the first sample for a t before all of them, None when t is after the last (the
    odometry of that step has not arrived)."""
    if not samples or t > samples[-1][0]:
        return None
    if t <= samples[0][0]:
        return samples[0][1:]
    lo, hi = 0, len(samples) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if samples[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    (t0, x0, y0, a0), (t1, x1, y1, a1) = samples[lo], samples[hi]
    w = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    da = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
    return x0 + w * (x1 - x0), y0 + w * (y1 - y0), a0 + w * da
