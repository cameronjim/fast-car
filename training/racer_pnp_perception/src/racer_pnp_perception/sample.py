"""One training sample: a car pose in a lane, the scan it would see, and the label.

The label is the contract's output tensor [16, 5] computed from the clean geometry: station 0 is
the centreline point nearest base_link, station k lies k * 0.20 m along the travel direction (the
centreline direction within 90 degrees of the heading), half widths are the free distances to the
walls along the centreline normal, capped at 2.0 m. Columns: x, y (base_link), half width left,
half width right, log_sigma (unused in the label, zero).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import NDArray
from racer_track_perception.track import HALF_WIDTH_CAP_M, K_STATIONS, STATION_SPACING_M
from racer_track_perception.vehicle import LidarMount

from racer_pnp_perception import nuisance as nz
from racer_pnp_perception.raycast import ScanGeometry, cast_rays, lidar_scan
from racer_pnp_perception.tracks import World

LATERAL_MARGIN_M = 0.2
HEADING_SIGMA_RAD = 0.3
HEADING_CLIP_RAD = 1.0


class LaneGeometry:
    """Periodic arc-length queries on a world's centreline."""

    def __init__(self, centreline: NDArray[np.float64]) -> None:
        pts = np.asarray(centreline, dtype=np.float64)
        closed = np.vstack([pts, pts[:1]])
        seg = np.hypot(*np.diff(closed, axis=0).T)
        self.pts = pts
        self.closed = closed
        self.s = np.concatenate([[0.0], np.cumsum(seg)])
        self.length = float(self.s[-1])
        tangent = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
        tangent /= np.maximum(np.hypot(tangent[:, 0], tangent[:, 1]), 1e-12)[:, None]
        self.tangent = np.vstack([tangent, tangent[:1]])
        # Signed curvature (positive = turning left going counter-clockwise): the heading change
        # over the neighbouring points (wrapped), lightly smoothed round the loop.
        heading = np.arctan2(tangent[:, 1], tangent[:, 0])
        dh = np.mod(np.roll(heading, -1) - np.roll(heading, 1) + math.pi, 2.0 * math.pi) - math.pi
        ds = np.hypot(*(np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)).T)
        kappa = dh / np.maximum(ds, 1e-9)
        smooth = sum(np.roll(kappa, k) for k in range(-2, 3)) / 5.0
        self.kappa = np.concatenate([smooth, smooth[:1]])

    def at(
        self, s: NDArray[np.float64]
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        """(positions [K, 2], unit tangents [K, 2] counter-clockwise, curvature [K]) at arc
        lengths `s` (wrapped)."""
        sw = np.mod(np.asarray(s, dtype=np.float64), self.length)
        pos = np.stack([np.interp(sw, self.s, self.closed[:, i]) for i in (0, 1)], axis=1)
        tan = np.stack([np.interp(sw, self.s, self.tangent[:, i]) for i in (0, 1)], axis=1)
        tan /= np.maximum(np.hypot(tan[:, 0], tan[:, 1]), 1e-12)[:, None]
        return pos, tan, np.interp(sw, self.s, self.kappa)

    def nearest_s(self, p: NDArray[np.float64]) -> float:
        """Arc length of the centreline point nearest to `p`."""
        d = np.hypot(self.pts[:, 0] - p[0], self.pts[:, 1] - p[1])
        i = int(np.argmin(d))
        best_s, best_d = float(self.s[i]), float(d[i])
        m = len(self.pts)
        for a, b, s_a in (((i - 1) % m, i, self.s[(i - 1) % m]), (i, (i + 1) % m, self.s[i])):
            p0, p1 = self.pts[a], self.pts[b]
            e = p1 - p0
            seg_len = float(np.hypot(e[0], e[1]))
            if seg_len < 1e-12:
                continue
            t = float(np.clip(np.dot(p - p0, e) / seg_len**2, 0.0, 1.0))
            dist = float(np.hypot(*(p0 + t * e - p)))
            if dist < best_d:
                best_d, best_s = dist, float(s_a + t * seg_len) % self.length
        return best_s


def label_for_pose(
    lane: LaneGeometry,
    walls: NDArray[np.float64],
    pose: tuple[float, float, float],
) -> NDArray[np.float32]:
    """The label [16, 5] for a base_link pose (x, y, yaw) in the world, from the clean walls."""
    x0, y0, yaw = pose
    s0 = lane.nearest_s(np.array([x0, y0]))
    _, tan0, _ = lane.at(np.array([s0]))
    heading = np.array([math.cos(yaw), math.sin(yaw)])
    direction = 1.0 if float(np.dot(tan0[0], heading)) >= 0.0 else -1.0
    s = s0 + direction * STATION_SPACING_M * np.arange(K_STATIONS)
    pos, tan, _ = lane.at(s)
    tan = tan * direction
    normal = np.stack([-tan[:, 1], tan[:, 0]], axis=1)
    half_left = _free_width(pos, normal, walls)
    half_right = _free_width(pos, -normal, walls)
    c, sn = math.cos(yaw), math.sin(yaw)
    dx, dy = pos[:, 0] - x0, pos[:, 1] - y0
    label = np.zeros((K_STATIONS, 5), dtype=np.float32)
    label[:, 0] = c * dx + sn * dy
    label[:, 1] = -sn * dx + c * dy
    label[:, 2] = half_left
    label[:, 3] = half_right
    return label


def _free_width(
    pos: NDArray[np.float64], direction: NDArray[np.float64], walls: NDArray[np.float64]
) -> NDArray[np.float64]:
    angle = np.arctan2(direction[:, 1], direction[:, 0])
    d = cast_rays(pos[:, 0], pos[:, 1], angle, walls, HALF_WIDTH_CAP_M)
    return np.minimum(d, HALF_WIDTH_CAP_M)


@dataclass
class Sample:
    ranges: NDArray[np.float64]  # [720], NaN / inf for no return
    label: NDArray[np.float32]  # [16, 5]
    pose: tuple[float, float, float]
    meta: dict[str, Any] = field(default_factory=dict)


def draw_pose(
    lane: LaneGeometry, lane_width_m: float, rng: np.random.Generator
) -> tuple[float, float, float]:
    """A base_link pose in the lane: arc length uniform, lateral offset up to half the width
    minus 0.2 m, heading error normal(0.3) clipped to 1.0 rad, either direction of travel."""
    s = float(rng.uniform(0.0, lane.length))
    pos, tan, _ = lane.at(np.array([s]))
    reach = max(lane_width_m / 2.0 - LATERAL_MARGIN_M, 0.0)
    offset = float(rng.uniform(-reach, reach))
    left = np.array([-tan[0, 1], tan[0, 0]])
    p = pos[0] + offset * left
    base = math.atan2(tan[0, 1], tan[0, 0]) + (math.pi if rng.uniform() < 0.5 else 0.0)
    err = float(np.clip(rng.normal(0.0, HEADING_SIGMA_RAD), -HEADING_CLIP_RAD, HEADING_CLIP_RAD))
    return float(p[0]), float(p[1]), base + err


def make_sample(
    world: World,
    lane: LaneGeometry,
    rng: np.random.Generator,
    mount: LidarMount,
    geometry: ScanGeometry,
    clean: bool = False,
    pose: tuple[float, float, float] | None = None,
) -> Sample:
    """One sample. `clean=True` gives an exact scan: no noise, dropout, gaps, room or motion."""
    pose = pose or draw_pose(lane, world.lane_width_m, rng)
    label = label_for_pose(lane, world.walls, pose)
    nuisance = nz.draw_nuisance(rng, clean=clean)
    walls = nz.perturb_world(world.walls, lane.pts, world.lane_width_m, nuisance, rng)
    s0 = lane.nearest_s(np.array(pose[:2]))
    _, tan0, kappa0 = lane.at(np.array([s0]))
    forward = 1.0 if float(np.dot(tan0[0], [math.cos(pose[2]), math.sin(pose[2])])) >= 0 else -1.0
    motion = nz.motion_for(nuisance, forward * float(kappa0[0]))
    ranges = lidar_scan(pose, mount, geometry, walls, motion)
    ranges = nz.perturb_ranges(ranges, geometry.laser_bearings(), mount.yaw_rad, nuisance, rng)
    return Sample(
        ranges=ranges,
        label=label,
        pose=pose,
        meta={"world": world.name, "speed_mps": nuisance.speed_mps, "dropout": nuisance.dropout},
    )
