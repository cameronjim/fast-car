"""Training worlds: closed lanes with a centreline and wall segments.

Two families, both counter-clockwise (a sample reverses the direction of travel itself):

* island tracks: `island_track` from the sim bridge's track_loader (the same builder the L5
  track-family runner uses) with continuous parameters;
* random Fourier loops: centreline radius R0 * (1 + sum_k a_k cos(k theta + phi_k)), k up to 4,
  the walls the centreline offset by half the lane width on each side.

`HELD_OUT_ISLANDS` are never trained on: `is_held_out` is checked in the generator and any island
within 0.05 m of one of them in every parameter is rejected.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from racer_track_perception.vehicle import find_repo_root

LANE_WIDTH_RANGE_M = (0.7, 1.6)
HELD_OUT_TOLERANCE_M = 0.05
FOURIER_MAX_K = 4
CENTRELINE_DS_M = 0.05
CURVATURE_MARGIN_M = 0.15
# Two parts of a Fourier loop further apart than this along the loop must be at least a lane
# width plus CLEARANCE_M apart in space, or the lanes would touch.
LOOP_SEPARATION_ARC_M = 1.5
CLEARANCE_M = 0.25


def _track_loader():  # type: ignore[no-untyped-def]
    bridge = find_repo_root() / "sim" / "bridge" / "racer_gym_bridge"
    if str(bridge) not in sys.path:
        sys.path.insert(0, str(bridge))
    from racer_gym_bridge import track_loader

    return track_loader


@dataclass(frozen=True)
class IslandParams:
    """island_track arguments: lane width, island length (x), island height (y), corner radius."""

    lane_width_m: float
    island_length_m: float
    island_height_m: float
    island_radius_m: float

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (
            self.lane_width_m,
            self.island_length_m,
            self.island_height_m,
            self.island_radius_m,
        )


def stadium_params(lane_width_m: float, straight_length_m: float, radius_m: float) -> IslandParams:
    """The island_track parameters of build_family_track("stadium", ...)."""
    return IslandParams(lane_width_m, straight_length_m + 2.0 * radius_m, 2.0 * radius_m, radius_m)


# The three held-out family tracks (never trained on, scored in eval_sim).
HELD_OUT_ISLANDS: dict[str, IslandParams] = {
    "stadium_l1.0_s2.0_r0.4": stadium_params(1.0, 2.0, 0.4),
    "rounded_rect_l1.0_3.0x2.0_r0.4": IslandParams(1.0, 3.0, 2.0, 0.4),
    "rounded_rect_l1.2_2.0x1.6_r0.8": IslandParams(1.2, 2.0, 1.6, 0.8),
}


def is_held_out(p: IslandParams, tol_m: float = HELD_OUT_TOLERANCE_M) -> bool:
    """True when `p` is within `tol_m` of a held-out island in every parameter."""
    for h in HELD_OUT_ISLANDS.values():
        if all(abs(a - b) <= tol_m for a, b in zip(p.as_tuple(), h.as_tuple(), strict=True)):
            return True
    return False


@dataclass(frozen=True)
class World:
    """A closed lane. `centreline` is [M, 2], counter-clockwise, spaced by about 0.05 m, the first
    point not repeated; `walls` is [S, 4] segments (x0, y0, x1, y1)."""

    name: str
    centreline: NDArray[np.float64]
    walls: NDArray[np.float64]
    lane_width_m: float

    def bounds(self) -> tuple[float, float, float, float]:
        xs = self.walls[:, [0, 2]]
        ys = self.walls[:, [1, 3]]
        return float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max())


def resample_closed(points: NDArray[np.float64], ds_m: float) -> NDArray[np.float64]:
    """Resample a closed polyline to equal arc-length spacing (about ds_m, closing exactly)."""
    closed = np.vstack([points, points[:1]])
    seg = np.hypot(*np.diff(closed, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(round(s[-1] / ds_m), 8)
    t = np.linspace(0.0, s[-1], n, endpoint=False)
    return np.stack([np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])], axis=1)


def polyline_normals(points: NDArray[np.float64]) -> NDArray[np.float64]:
    """Left unit normals of a closed counter-clockwise polyline (central differences)."""
    tangent = np.roll(points, -1, axis=0) - np.roll(points, 1, axis=0)
    tangent /= np.maximum(np.hypot(tangent[:, 0], tangent[:, 1]), 1e-12)[:, None]
    return np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)


def closed_segments(points: NDArray[np.float64]) -> NDArray[np.float64]:
    return np.hstack([points, np.roll(points, -1, axis=0)])


def island_world(p: IslandParams, name: str = "island") -> World:
    """The island lane of parameters `p` as a World (uses the sim's own builder)."""
    track = _track_loader().island_track(
        name, p.lane_width_m, p.island_length_m, p.island_height_m, p.island_radius_m
    )
    centre = np.stack([track.centreline_x, track.centreline_y], axis=1)
    return World(name, centre, np.asarray(track.walls, dtype=np.float64), p.lane_width_m)


def sample_island_params(rng: np.random.Generator) -> IslandParams:
    """Continuous island parameters, never within tolerance of a held-out track."""
    while True:
        lane = float(rng.uniform(*LANE_WIDTH_RANGE_M))
        length = float(rng.uniform(1.0, 3.6))
        height = float(rng.uniform(0.6, 2.4))
        r_max = min(length, height) / 2.0
        radius = float(rng.uniform(0.0, r_max)) if rng.uniform() < 0.85 else r_max
        p = IslandParams(lane, length, height, radius)
        if not is_held_out(p):
            return p


@dataclass(frozen=True)
class FourierParams:
    r0_m: float
    amplitudes: tuple[float, ...]
    phases: tuple[float, ...]
    lane_width_m: float


def fourier_centreline(
    r0_m: float,
    amplitudes: tuple[float, ...],
    phases: tuple[float, ...],
    n: int = 2400,
) -> NDArray[np.float64]:
    """Polar loop r(theta) = r0 * (1 + sum_k a_k cos(k theta + phi_k)), k = 1..len(amplitudes),
    counter-clockwise, resampled to CENTRELINE_DS_M."""
    theta = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    scale = np.ones_like(theta)
    for k, (a, ph) in enumerate(zip(amplitudes, phases, strict=True), start=1):
        scale = scale + a * np.cos(k * theta + ph)
    r = r0_m * scale
    pts = np.stack([r * np.cos(theta), r * np.sin(theta)], axis=1)
    return resample_closed(pts, CENTRELINE_DS_M)


def min_curvature_radius(points: NDArray[np.float64]) -> float:
    """Smallest radius of curvature of a closed polyline (inf for a straight one)."""
    p0, p1, p2 = np.roll(points, 1, axis=0), points, np.roll(points, -1, axis=0)
    a = np.hypot(*(p1 - p0).T)
    b = np.hypot(*(p2 - p1).T)
    c = np.hypot(*(p2 - p0).T)
    cross = np.abs(
        (p1[:, 0] - p0[:, 0]) * (p2[:, 1] - p0[:, 1])
        - (p1[:, 1] - p0[:, 1]) * (p2[:, 0] - p0[:, 0])
    )
    with np.errstate(divide="ignore"):
        radius = a * b * c / (2.0 * np.maximum(cross, 1e-12))
    return float(radius.min())


def lanes_clear(points: NDArray[np.float64], lane_width_m: float) -> bool:
    """False when two parts of the loop that are far apart along it come closer than a lane width
    plus the clearance."""
    sub = points[:: max(len(points) // 240, 1)]
    seg = np.hypot(*(np.roll(sub, -1, axis=0) - sub).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])[:-1]
    total = float(seg.sum())
    arc = np.abs(s[:, None] - s[None, :])
    arc = np.minimum(arc, total - arc)
    dist = np.hypot(sub[:, None, 0] - sub[None, :, 0], sub[:, None, 1] - sub[None, :, 1])
    far = arc > LOOP_SEPARATION_ARC_M
    return bool(np.all(dist[far] > lane_width_m + CLEARANCE_M)) if far.any() else True


def fourier_world(fp: FourierParams, name: str = "fourier") -> World | None:
    """The Fourier loop lane, or None when it is rejected (local curvature radius under half the
    width plus CURVATURE_MARGIN_M, or the lanes touch)."""
    centre = fourier_centreline(fp.r0_m, fp.amplitudes, fp.phases)
    half = fp.lane_width_m / 2.0
    if min_curvature_radius(centre) < half + CURVATURE_MARGIN_M:
        return None
    if not lanes_clear(centre, fp.lane_width_m):
        return None
    normal = polyline_normals(centre)
    left = centre + half * normal
    right = centre - half * normal
    walls = np.vstack([closed_segments(left), closed_segments(right)])
    return World(name, centre, walls, fp.lane_width_m)


def sample_fourier_world(rng: np.random.Generator) -> World:
    """A random Fourier loop lane, resampled until one passes the rejection tests."""
    while True:
        lane = float(rng.uniform(*LANE_WIDTH_RANGE_M))
        r0 = float(rng.uniform(0.8, 1.9))
        # Amplitudes fall with k so the loop stays a loop; the curvature test does the rest.
        amps = tuple(float(rng.uniform(0.0, 0.35 / k)) for k in range(1, FOURIER_MAX_K + 1))
        phases = tuple(float(rng.uniform(0.0, 2.0 * math.pi)) for _ in range(FOURIER_MAX_K))
        world = fourier_world(FourierParams(r0, amps, phases, lane), "fourier")
        if world is not None:
            return world


def sample_world(rng: np.random.Generator, fourier_share: float = 0.5) -> World:
    """One training world: a Fourier loop with probability `fourier_share`, else an island."""
    if rng.uniform() < fourier_share:
        return sample_fourier_world(rng)
    return island_world(sample_island_params(rng))


def held_out_worlds() -> dict[str, World]:
    return {name: island_world(p, name) for name, p in HELD_OUT_ISLANDS.items()}


def repo_root() -> Path:
    return find_repo_root()
