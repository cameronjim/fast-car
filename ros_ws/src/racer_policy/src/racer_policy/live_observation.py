"""The live observation builder for the "v0" observation profile.

The policy is trained on `racer_train.observation` / `racer_train.env.ResidualRacerEnv._sense`
and must be fed EXACTLY the same vector on the car, or it sees inputs it never trained on.
This module is the deploy-side twin of that code, kept ROS-free so it is unit-testable, and
`training/racer_train/tests/test_live_observation_parity.py` runs both on the same inputs and
requires identical vectors (and ties every constant below to training's own).

Differences that are NOT differences of definition, only of sensor geometry:

* Training sees a 1080-beam scan over 4.7 rad, min-pooled by 30 into 36 beams of 4.7/36 rad.
  The car's RPLidar C1 is a 360 degree, about 500-beam scan whose head is mounted rotated by
  `sensors.lidar.mount_yaw_rad`. `pool_scan` bins every live beam by its VEHICLE bearing into
  the same 36 angular bins and takes the minimum in each, which is the same operation (and
  gives the identical vector on a training-geometry scan).
* A beam that is not a finite number, or is below the sensor's minimum range, is a "no
  return" and reads as the clip distance, like a sim beam that hit nothing.

Nothing here is a physical constant: the wheelbase and the lidar mount come in as arguments
from the generated vehicle_params binding (CLAUDE.md invariant 2).
"""

from __future__ import annotations

import csv
import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt

Array = npt.NDArray[Any]

PROFILE = "v0"

# (name, dtype, units) in contract order. Tied to racer_train.observation.LOW_DIM_FIELDS_V0
# by the parity test.
LIVE_FIELDS_V0: tuple[tuple[str, str, str], ...] = (
    ("velocity_mps", "float32", "m/s"),
    ("yaw_rate_rad_s", "float32", "rad/s"),
    ("slip_proxy", "float32", "dimensionless"),
    ("raceline_lateral_error_m", "float32", "m"),
    ("raceline_heading_error_rad", "float32", "rad"),
    ("base_steering_rad", "float32", "rad"),
    ("base_speed_mps", "float32", "m/s"),
    ("raceline_curvature_near_1pm", "float32", "1/m"),
    ("raceline_curvature_far_1pm", "float32", "1/m"),
    ("prev_steering_cmd_rad", "float32", "rad"),
    ("prev_speed_cmd_mps", "float32", "m/s"),
)
OOD_STATE_DIM = 5

# The LiDAR block the v0 policy was trained against (training/configs/v0, v1: 1080 raw beams
# over 4.7 rad, downsample factor 30 -> 36 beams). The live schema reports THESE, not the
# contract's, so `verify_against_environment` compares two independent statements.
LIDAR_BEAM_COUNT = 36
LIDAR_FOV_RAD = 4.7
LIDAR_DOWNSAMPLE_FACTOR = 30
LIDAR_CLIP_M = 5.0

CURVATURE_NEAR_M = 0.4
CURVATURE_FAR_M = 1.0
SLIP_PROXY_SPEED_FLOOR_MPS = 0.3
SLIP_PROXY_LIMIT = 2.0

_COLUMNS = ("s_m", "x_m", "y_m", "heading_rad", "curvature_1pm", "target_speed_mps")


class RacelineError(Exception):
    """A raceline CSV could not be read. No fallback to a degenerate line."""


class Raceline:
    """The closed reference line, read from the same CSV the tracker follows.

    Mirrors `racer_train.raceline.Raceline` for the three operations the observation needs
    (nearest point, point `d` metres ahead, closed length)."""

    def __init__(
        self,
        x_m: Array,
        y_m: Array,
        heading_rad: Array,
        curvature_1pm: Array,
        s_m: Array,
    ) -> None:
        if len(x_m) == 0:
            raise RacelineError("refusing to build a raceline from zero points")
        self.x = np.asarray(x_m, dtype=np.float64)
        self.y = np.asarray(y_m, dtype=np.float64)
        self.heading = np.asarray(heading_rad, dtype=np.float64)
        self.curvature = np.asarray(curvature_1pm, dtype=np.float64)
        closing = math.hypot(self.x[0] - self.x[-1], self.y[0] - self.y[-1])
        self.length_m = float(s_m[-1]) + closing

    def __len__(self) -> int:
        return len(self.x)

    @classmethod
    def from_csv(cls, path: Path | str) -> Raceline:
        path = Path(path)
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except OSError as exc:
            raise RacelineError(f"could not open raceline '{path}': {exc}") from exc
        header = False
        rows: list[tuple[float, ...]] = []
        for number, raw in enumerate(lines, start=1):
            line = raw.removesuffix("\r")
            if not line or line[0] == "#":
                continue
            fields = next(csv.reader([line]))
            if not header:
                if tuple(fields) != _COLUMNS:
                    raise RacelineError(f"{path}: expected header {','.join(_COLUMNS)}, got {line}")
                header = True
                continue
            if len(fields) != 6:
                raise RacelineError(f"{path}:{number}: expected 6 columns, got {len(fields)}")
            try:
                rows.append(tuple(float(f) for f in fields))
            except ValueError as exc:
                raise RacelineError(f"{path}:{number}: {exc}") from exc
        if not rows:
            raise RacelineError(f"{path}: no raceline rows")
        arr = np.asarray(rows, dtype=np.float64)
        return cls(arr[:, 1], arr[:, 2], arr[:, 3], arr[:, 4], arr[:, 0])

    def nearest_index(self, x: float, y: float) -> int:
        dx = self.x - x
        dy = self.y - y
        return int(np.argmin(dx * dx + dy * dy))

    def index_ahead(self, from_index: int, distance_m: float) -> int:
        n = len(self.x)
        mean_spacing_m = self.length_m / n
        return (from_index + round(distance_m / mean_spacing_m)) % n


def compose_pose(
    frame_from_odom: tuple[float, float, float], pose_in_odom: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Pose of the vehicle in the frame the raceline lives in: T(frame <- odom) applied to the
    odometry pose (the same composition tracker_node does with `pose_frame` set)."""
    fx, fy, fyaw = frame_from_odom
    px, py, pyaw = pose_in_odom
    c, s = math.cos(fyaw), math.sin(fyaw)
    return (fx + c * px - s * py, fy + s * px + c * py, wrap_to_pi(fyaw + pyaw))


def wrap_to_pi(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def slip_proxy(
    steering_cmd_rad: float, yaw_rate_rad_s: float, velocity_mps: float, wheelbase_m: float
) -> float:
    """Kinematic understeer indicator tan(delta) - L * yaw_rate / max(|v|, floor), clipped."""
    v = max(abs(velocity_mps), SLIP_PROXY_SPEED_FLOOR_MPS)
    value = math.tan(steering_cmd_rad) - wheelbase_m * yaw_rate_rad_s / v
    return max(-SLIP_PROXY_LIMIT, min(SLIP_PROXY_LIMIT, value))


def raceline_relative_pose(
    raceline: Raceline, x_m: float, y_m: float, yaw_rad: float
) -> tuple[float, float, int]:
    """(lateral error, heading error, nearest index): the vehicle-frame y offset of the
    nearest raceline point (left positive) and the wrapped heading difference."""
    i = raceline.nearest_index(x_m, y_m)
    dx = float(raceline.x[i]) - x_m
    dy = float(raceline.y[i]) - y_m
    cos_neg = math.cos(-yaw_rad)
    sin_neg = math.sin(-yaw_rad)
    lateral = sin_neg * dx + cos_neg * dy
    heading = wrap_to_pi(float(raceline.heading[i]) - yaw_rad)
    return lateral, heading, i


def pool_scan(
    ranges: Sequence[float] | Array,
    angle_min_rad: float,
    angle_increment_rad: float,
    mount_yaw_rad: float,
    range_min_m: float,
    beam_count: int = LIDAR_BEAM_COUNT,
    fov_rad: float = LIDAR_FOV_RAD,
    clip_m: float = LIDAR_CLIP_M,
) -> Array:
    """Min-pool a live scan into `beam_count` bins of `fov_rad / beam_count` about the
    vehicle's forward axis. Returns float32, every entry in (0, clip_m]."""
    r = np.asarray(ranges, dtype=np.float64)
    angles = angle_min_rad + angle_increment_rad * np.arange(len(r)) + mount_yaw_rad
    # Vehicle bearing in [-pi, pi), left positive.
    bearing = (angles + math.pi) % (2.0 * math.pi) - math.pi
    valid = np.isfinite(r) & (r >= range_min_m)
    r = np.where(valid, np.minimum(r, clip_m), clip_m)
    width = fov_rad / beam_count
    idx = np.floor((bearing + fov_rad / 2.0) / width).astype(np.int64)
    # The +fov/2 edge itself belongs to the last bin (a training-geometry scan has a beam
    # exactly there).
    idx = np.where((idx == beam_count) & (bearing <= fov_rad / 2.0 + 1e-9), beam_count - 1, idx)
    inside = (idx >= 0) & (idx < beam_count)
    out = np.full(beam_count, clip_m, dtype=np.float64)
    np.minimum.at(out, idx[inside], r[inside])
    return out.astype(np.float32)


@dataclass(frozen=True)
class LiveInputs:
    """One decision's worth of measurements, all SI.

    `x_m, y_m, yaw_rad` are the pose the tracker also follows (the localised pose on the car,
    ground truth in a sim canary). `beams` is the already-pooled scan from `pool_scan`."""

    velocity_mps: float
    yaw_rate_rad_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    base_steering_rad: float
    base_speed_mps: float
    prev_steering_cmd_rad: float
    prev_speed_cmd_mps: float
    beams: Array


class LiveObservationBuilder:
    """Assembles the flat float32 observation: 11 low-dim fields then 36 beams."""

    def __init__(self, raceline: Raceline, wheelbase_m: float) -> None:
        self._raceline = raceline
        self._wheelbase_m = float(wheelbase_m)

    @property
    def raceline(self) -> Raceline:
        return self._raceline

    @property
    def dim(self) -> int:
        return len(LIVE_FIELDS_V0) + LIDAR_BEAM_COUNT

    def build(self, inputs: LiveInputs) -> Array:
        raceline = self._raceline
        lateral, heading, nearest = raceline_relative_pose(
            raceline, inputs.x_m, inputs.y_m, inputs.yaw_rad
        )
        slip = slip_proxy(
            inputs.prev_steering_cmd_rad,
            inputs.yaw_rate_rad_s,
            inputs.velocity_mps,
            self._wheelbase_m,
        )
        k_near = float(raceline.curvature[raceline.index_ahead(nearest, CURVATURE_NEAR_M)])
        k_far = float(raceline.curvature[raceline.index_ahead(nearest, CURVATURE_FAR_M)])
        low = np.array(
            [
                inputs.velocity_mps,
                inputs.yaw_rate_rad_s,
                slip,
                lateral,
                heading,
                inputs.base_steering_rad,
                inputs.base_speed_mps,
                k_near,
                k_far,
                inputs.prev_steering_cmd_rad,
                inputs.prev_speed_cmd_mps,
            ],
            dtype=np.float32,
        )
        beams = np.minimum(np.asarray(inputs.beams, dtype=np.float32), np.float32(LIDAR_CLIP_M))
        if beams.shape != (LIDAR_BEAM_COUNT,):
            raise ValueError(f"expected {LIDAR_BEAM_COUNT} pooled beams, got shape {beams.shape}")
        return np.concatenate([low, beams])
