"""Procedural, walled, small-scale tracks for training and evaluating the residual (v0).

The committed raceline tracks (`config/tracks/gym_oval`, `oschersleben`) are racing-scale:
turn radius 3 m, speeds to 20 m/s, and -- because `Track.from_refline` builds a wall-free
occupancy map -- no walls, so the S.3 env could neither crash nor see anything on LiDAR. The
real car runs on ~1.0 m wide lanes of foam board on a living-room floor at about 2 m/s, with
U-turns whose inner radius is anywhere from 0 to 0.8 m and a layout that is rebuilt every
session. This module generates that family at the gym level:

  * `stadium`      two straights joined by two half circles (an oval),
  * `rounded_rect` four straights joined by four quarter circles,
  * `blob`         a smooth random closed curve (HELD OUT: never drawn for training),
  * `large`        a stadium at racing-room scale (large radii, long straights).

Each shape is traversed CCW or CW. The reference line is NOT the lane centreline: with a
1 m lane and an inner radius under about 0.7 m the car physically cannot follow the
centreline (minimum turning radius about 0.60 m), so `plan_reference_line` swings the line
outward through tight turns (offset capped well inside the walls), then
the committed `tools/raceline` pipeline (`build_raceline_from_centerline`: resample, smooth,
curvature/friction speed profile) turns it into a speed-annotated raceline. That pipeline is
imported, not re-implemented, so the speed profile is the same code that produced the
committed racelines.

The walled occupancy map comes from the bridge's `track_loader.build_corridor_occupancy`
(imported by path; sim/bridge is owned by another change and is never edited here).

Everything is deterministic in (family config, seed): the same `TrackSpec` always builds the
same track.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np

from racer_train.raceline import Raceline, RacelinePoint

_REPO_ROOT = Path(__file__).resolve().parents[4]

SHAPES = ("stadium", "rounded_rect", "blob", "large")


class TrackGenerationError(RuntimeError):
    """A track could not be generated (rejection sampling exhausted, tools missing, ...)."""


# -- imports of the two repo helpers this module reuses -------------------------------------


def _import_raceline_pipeline(repo_root: Path = _REPO_ROOT) -> ModuleType:
    tools_dir = repo_root / "tools"
    if not (tools_dir / "raceline" / "raceline.py").is_file():
        raise TrackGenerationError(f"tools/raceline not found under {repo_root}")
    if str(tools_dir) not in sys.path:
        sys.path.insert(0, str(tools_dir))
    import raceline.raceline as pipeline

    return pipeline


def _import_track_loader(repo_root: Path = _REPO_ROOT) -> ModuleType:
    path = (
        repo_root / "sim" / "bridge" / "racer_gym_bridge" / "racer_gym_bridge" / "track_loader.py"
    )
    if not path.is_file():
        raise TrackGenerationError(f"bridge track_loader.py not found at {path}")
    name = "racer_train._bridge_track_loader"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:  # pragma: no cover - defensive
        raise TrackGenerationError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


# -- config ----------------------------------------------------------------------------------


@dataclass(frozen=True)
class TrackFamilyConfig:
    """Ranges for the procedural family. Inner radius is the radius of the INSIDE wall at a
    U-turn; the lane centreline radius there is `inner_radius + lane_width / 2`."""

    lane_width_m: float = 1.0
    # Shapes drawn for training / default eval, with relative weights.
    shapes: tuple[str, ...] = ("stadium", "rounded_rect")
    shape_weights: tuple[float, ...] = (0.5, 0.5)
    inner_radius_m: tuple[float, float] = (0.0, 0.8)
    stadium_straight_m: tuple[float, float] = (1.0, 4.0)
    rect_side_m: tuple[float, float] = (1.8, 4.5)
    large_straight_m: tuple[float, float] = (4.0, 7.0)
    large_radius_m: tuple[float, float] = (1.5, 3.0)
    # Reference line planning.
    max_offset_m: float = 0.25  # keeps the car body (half width ~0.155 m) off the walls
    # Speed profile (tools/raceline.speed_profile). The tuned values come from the base
    # controller sweep recorded in the v0 note; vehicle top speed is the real 2.08 m/s.
    v_max_mps: float = 2.0
    a_lat_max_mps2: float = 2.5
    a_long_max_mps2: float = 1.5
    resample_ds_m: float = 0.1
    smooth_window: int = 5

    def __post_init__(self) -> None:
        if len(self.shapes) != len(self.shape_weights) or not self.shapes:
            raise ValueError("shapes and shape_weights must be the same non-zero length")
        for shape in self.shapes:
            if shape not in SHAPES:
                raise ValueError(f"unknown track shape {shape!r}; expected one of {SHAPES}")
        if self.lane_width_m <= 0.0:
            raise ValueError("lane_width_m must be > 0")
        if not 0.0 < self.max_offset_m < self.lane_width_m / 2.0:
            raise ValueError("max_offset_m must be inside the lane half width")


@dataclass(frozen=True)
class TrackSpec:
    """Fully determines one track. `params` holds the shape-specific numbers."""

    shape: str
    direction: int  # +1 counter-clockwise, -1 clockwise
    params: tuple[tuple[str, float], ...]
    seed: int = 0

    def param(self, name: str) -> float:
        return dict(self.params)[name]

    @property
    def label(self) -> str:
        body = ",".join(f"{k}={v:.2f}" for k, v in self.params)
        return f"{self.shape}[{'ccw' if self.direction > 0 else 'cw'};{body}]"


@dataclass
class TrackBundle:
    """A built track: the controller's raceline + the walled gym `Track` it races on."""

    spec: TrackSpec
    raceline: Raceline
    gym_track: Any
    lane_half_width_m: float
    perimeter_m: float
    min_ref_radius_m: float
    meta: dict[str, float] = field(default_factory=dict)


# -- centreline generators (CCW, closed, no duplicated end point) ---------------------------


def _arc(
    cx: float, cy: float, r: float, a0: float, a1: float, ds: float
) -> tuple[np.ndarray, np.ndarray]:
    n = max(math.ceil(abs(a1 - a0) * r / ds), 4)
    t = np.linspace(a0, a1, n, endpoint=False)
    return cx + r * np.cos(t), cy + r * np.sin(t)


def _line(x0: float, y0: float, x1: float, y1: float, ds: float) -> tuple[np.ndarray, np.ndarray]:
    n = max(math.ceil(math.hypot(x1 - x0, y1 - y0) / ds), 2)
    t = np.linspace(0.0, 1.0, n, endpoint=False)
    return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t


def stadium_centreline(straight_m: float, radius_m: float, ds: float = 0.05):
    half = straight_m / 2.0
    parts = [
        _line(-half, -radius_m, half, -radius_m, ds),
        _arc(half, 0.0, radius_m, -math.pi / 2, math.pi / 2, ds),
        _line(half, radius_m, -half, radius_m, ds),
        _arc(-half, 0.0, radius_m, math.pi / 2, 3 * math.pi / 2, ds),
    ]
    return np.concatenate([p[0] for p in parts]), np.concatenate([p[1] for p in parts])


def rounded_rect_centreline(width_m: float, height_m: float, radius_m: float, ds: float = 0.05):
    if radius_m * 2.0 > min(width_m, height_m) + 1e-9:
        raise ValueError("corner radius too large for the rectangle")
    hw, hh, r = width_m / 2.0, height_m / 2.0, radius_m
    if r < 1e-6:
        corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
        parts = [_line(*corners[i], *corners[(i + 1) % 4], ds) for i in range(4)]
    else:
        parts = [
            _line(-hw + r, -hh, hw - r, -hh, ds),
            _arc(hw - r, -hh + r, r, -math.pi / 2, 0.0, ds),
            _line(hw, -hh + r, hw, hh - r, ds),
            _arc(hw - r, hh - r, r, 0.0, math.pi / 2, ds),
            _line(hw - r, hh, -hw + r, hh, ds),
            _arc(-hw + r, hh - r, r, math.pi / 2, math.pi, ds),
            _line(-hw, hh - r, -hw, -hh + r, ds),
            _arc(-hw + r, -hh + r, r, math.pi, 3 * math.pi / 2, ds),
        ]
    return np.concatenate([p[0] for p in parts]), np.concatenate([p[1] for p in parts])


def blob_centreline(scale_m: float, coeffs: tuple[tuple[float, float], ...], n: int = 720):
    """r(theta) = scale * (1 + sum_k a_k cos(k theta + phi_k)), k = 2, 3, ..."""
    theta = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    r = np.ones_like(theta)
    for k, (amp, phase) in enumerate(coeffs, start=2):
        r += amp * np.cos(k * theta + phase)
    r *= scale_m
    return r * np.cos(theta), r * np.sin(theta)


def _polyline_min_radius(x: np.ndarray, y: np.ndarray) -> float:
    """Smallest radius of curvature (m) of a closed, roughly uniformly sampled polyline."""
    _import_raceline_pipeline()
    from raceline.geometry import curvature_from_points_closed, resample_closed_uniform

    xu, yu = resample_closed_uniform(x, y, 0.05)
    k = np.abs(curvature_from_points_closed(xu, yu))
    return float(1.0 / max(float(k.max()), 1e-9))


def _min_nonlocal_separation(x: np.ndarray, y: np.ndarray, min_arc_gap_m: float = 1.5) -> float:
    """Smallest distance between two points of a closed curve that are more than
    `min_arc_gap_m` apart ALONG the curve (self-proximity: lanes must not overlap)."""
    seg = np.hypot(np.diff(np.append(x, x[0])), np.diff(np.append(y, y[0])))
    s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
    total = float(seg.sum())
    step = max(len(x) // 400, 1)
    xs, ys, ss = x[::step], y[::step], s[::step]
    d = np.hypot(xs[:, None] - xs[None, :], ys[:, None] - ys[None, :])
    gap = np.abs(ss[:, None] - ss[None, :])
    gap = np.minimum(gap, total - gap)
    d = np.where(gap > min_arc_gap_m, d, np.inf)
    return float(d.min())


# -- spec sampling and building ---------------------------------------------------------------


def sample_spec(rng: np.random.Generator, family: TrackFamilyConfig, seed: int = 0) -> TrackSpec:
    w = np.asarray(family.shape_weights, dtype=float)
    shape = family.shapes[int(rng.choice(len(family.shapes), p=w / w.sum()))]
    direction = 1 if rng.random() < 0.5 else -1
    half_lane = family.lane_width_m / 2.0
    if shape == "stadium":
        inner = float(rng.uniform(*family.inner_radius_m))
        params = (
            ("straight_m", float(rng.uniform(*family.stadium_straight_m))),
            ("radius_m", inner + half_lane),
            ("inner_radius_m", inner),
        )
    elif shape == "rounded_rect":
        inner = float(rng.uniform(*family.inner_radius_m))
        radius = inner + half_lane
        lo = max(family.rect_side_m[0], 2.0 * radius + 0.4)
        a = float(rng.uniform(lo, max(family.rect_side_m[1], lo)))
        b = float(rng.uniform(lo, max(family.rect_side_m[1], lo)))
        params = (("width_m", a), ("height_m", b), ("radius_m", radius), ("inner_radius_m", inner))
    elif shape == "large":
        radius = float(rng.uniform(*family.large_radius_m))
        params = (
            ("straight_m", float(rng.uniform(*family.large_straight_m))),
            ("radius_m", radius),
            ("inner_radius_m", radius - half_lane),
        )
    else:  # blob
        scale = float(rng.uniform(1.6, 2.4))
        coeffs = []
        for k in (2, 3, 4):
            amp = float(rng.uniform(0.0, 0.22 / (k - 1)))
            coeffs.append((amp, float(rng.uniform(0.0, 2.0 * math.pi))))
        params = (("scale_m", scale),) + tuple(
            item
            for k, (amp, ph) in enumerate(coeffs, start=2)
            for item in ((f"a{k}", amp), (f"p{k}", ph))
        )
    return TrackSpec(shape=shape, direction=direction, params=tuple(params), seed=seed)


def _centreline_for(spec: TrackSpec) -> tuple[np.ndarray, np.ndarray]:
    if spec.shape in ("stadium", "large"):
        x, y = stadium_centreline(spec.param("straight_m"), spec.param("radius_m"))
    elif spec.shape == "rounded_rect":
        x, y = rounded_rect_centreline(
            spec.param("width_m"), spec.param("height_m"), spec.param("radius_m")
        )
    elif spec.shape == "blob":
        coeffs = tuple((spec.param(f"a{k}"), spec.param(f"p{k}")) for k in (2, 3, 4))
        x, y = blob_centreline(spec.param("scale_m"), coeffs)
    else:
        raise ValueError(f"unknown shape {spec.shape!r}")
    if spec.direction < 0:
        x, y = x[::-1].copy(), y[::-1].copy()
    return x, y


def plan_reference_line(
    x: np.ndarray,
    y: np.ndarray,
    max_offset_m: float,
    widen_start_curvature_1pm: float = 0.8,
    widen_gain_m_per_1pm: float = 0.2,
    smooth_length_m: float = 0.8,
    ds: float = 0.05,
) -> tuple[np.ndarray, np.ndarray]:
    """Widen tight turns: shift the reference line OUTWARD where the centreline is tighter
    than the car can drive.

    A lane centreline with radius under about 0.7 m is not drivable (minimum turning radius
    about 0.60 m at the rear axle), so the line must swing wide through a tight U-turn. Where
    the centreline curvature |k| exceeds `widen_start_curvature_1pm` the line moves outward
    (away from the turn's inside) by `widen_gain_m_per_1pm * (|k| - start)`, capped at
    `max_offset_m` so the car body stays off the walls; the offset profile is then smoothed
    over `smooth_length_m` so the line eases in and out. On a straight or a gentle curve
    nothing moves. This is a documented simplification, not a minimum-curvature solution --
    the same stance `tools/raceline` takes for the committed racelines (it only smooths the
    centreline). The speed profile downstream still slows the car for whatever curvature
    remains.
    """
    _import_raceline_pipeline()
    from raceline.geometry import curvature_from_points_closed, resample_closed_uniform

    cx, cy = resample_closed_uniform(x, y, ds)
    n = len(cx)
    kappa = curvature_from_points_closed(cx, cy)
    # Light smoothing of the curvature itself first: the discrete estimate spikes at the
    # straight-to-arc junctions.
    k_win = max(round(0.3 / ds) | 1, 3)
    kernel = np.ones(k_win) / k_win
    kappa_s = np.convolve(np.concatenate([kappa[-k_win:], kappa, kappa[:k_win]]), kernel, "same")[
        k_win:-k_win
    ]
    mag = np.clip(widen_gain_m_per_1pm * (np.abs(kappa_s) - widen_start_curvature_1pm), 0.0, None)
    outward = -np.sign(kappa_s) * np.minimum(mag, max_offset_m)  # along the LEFT normal
    o_win = max(round(smooth_length_m / ds) | 1, 3)
    o_kernel = np.ones(o_win) / o_win
    padded = np.concatenate([outward[-o_win:], outward, outward[:o_win]])
    offset = np.convolve(padded, o_kernel, "same")[o_win:-o_win]
    offset = np.clip(offset, -max_offset_m, max_offset_m)

    tx = np.roll(cx, -1) - np.roll(cx, 1)
    ty = np.roll(cy, -1) - np.roll(cy, 1)
    norm = np.maximum(np.hypot(tx, ty), 1e-12)
    nx, ny = -ty / norm, tx / norm  # left normal
    assert len(offset) == n
    return cx + offset * nx, cy + offset * ny


def raceline_from_arrays(r: Any) -> Raceline:
    points = tuple(
        RacelinePoint(
            s_m=float(r.s_m[i]),
            x_m=float(r.x_m[i]),
            y_m=float(r.y_m[i]),
            heading_rad=float(r.heading_rad[i]),
            curvature_1pm=float(r.curvature_1pm[i]),
            target_speed_mps=float(r.target_speed_mps[i]),
        )
        for i in range(len(r.s_m))
    )
    return Raceline(points)


def build_track(spec: TrackSpec, family: TrackFamilyConfig) -> TrackBundle:
    """Build the walled gym track and the speed-annotated reference line for `spec`."""
    from f1tenth_gym.envs.track import Track

    pipeline = _import_raceline_pipeline()
    loader = _import_track_loader()

    cx, cy = _centreline_for(spec)
    if spec.shape == "blob":
        if _polyline_min_radius(cx, cy) < 0.6:
            raise TrackGenerationError("blob centreline too tight")
        if _min_nonlocal_separation(cx, cy) < family.lane_width_m + 0.3:
            raise TrackGenerationError("blob lanes overlap")
    rx, ry = plan_reference_line(cx, cy, family.max_offset_m)
    reference = pipeline.build_raceline_from_centerline(
        rx,
        ry,
        a_lat_max_mps2=family.a_lat_max_mps2,
        a_max_mps2=family.a_long_max_mps2,
        v_max_mps=family.v_max_mps,
        resample_ds_m=family.resample_ds_m,
        smooth_window=family.smooth_window,
    )
    raceline = raceline_from_arrays(reference)

    half_lane = family.lane_width_m / 2.0
    # f1tenth_gym's Track is built from the (unsmoothed) centreline so the map origin and
    # the lap gate are centred on the lane, then its map is replaced by the walled corridor.
    track = Track.from_refline(x=cx, y=cy, velx=np.full_like(cx, 1.0))
    occupancy, origin = loader.build_corridor_occupancy(
        cx, cy, half_lane, track.spec.resolution, border_m=1.0
    )
    track.occupancy_map = occupancy
    track.spec = dataclasses.replace(track.spec, origin=origin)

    from raceline.geometry import curvature_from_points_closed

    kappa = np.abs(curvature_from_points_closed(reference.x_m, reference.y_m))
    return TrackBundle(
        spec=spec,
        raceline=raceline,
        gym_track=track,
        lane_half_width_m=half_lane,
        perimeter_m=raceline.length_m,
        min_ref_radius_m=float(1.0 / max(float(kappa.max()), 1e-9)),
        meta={
            "mean_target_speed_mps": float(np.mean(reference.target_speed_mps)),
            "min_target_speed_mps": float(np.min(reference.target_speed_mps)),
        },
    )


def build_pool(
    family: TrackFamilyConfig, n: int, seed_base: int, max_attempts: int = 50
) -> list[TrackBundle]:
    """`n` tracks drawn with per-track seeds `seed_base + i` (disjoint seed ranges give
    disjoint train/eval pools)."""
    bundles: list[TrackBundle] = []
    for i in range(n):
        seed = seed_base + i
        for attempt in range(max_attempts):
            rng = np.random.default_rng([seed, attempt])
            spec = sample_spec(rng, family, seed=seed)
            try:
                bundles.append(build_track(spec, family))
                break
            except (TrackGenerationError, ValueError):
                continue
        else:
            raise TrackGenerationError(
                f"could not build track for seed {seed} in {max_attempts} tries"
            )
    return bundles


def start_pose(bundle: TrackBundle, index: int, lateral_m: float = 0.0, heading_rad: float = 0.0):
    """(x, y, yaw) on the reference line at point `index`, optionally perturbed."""
    p = bundle.raceline.at(index)
    nx, ny = -math.sin(p.heading_rad), math.cos(p.heading_rad)
    return (p.x_m + lateral_m * nx, p.y_m + lateral_m * ny, p.heading_rad + heading_rad)
