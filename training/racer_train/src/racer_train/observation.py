"""Observation schema for the S.3 training env.

claude-docs/08-learning.md "Boring choices, committed": "Observations: low-dimensional state
(from EKF/localizer: velocity, yaw rate, slip proxy, raceline-relative pose) + downsampled
LiDAR. Exact schema lives in the contract." The field names/order/dtypes/units defined here
ARE that exact schema: `racer_train.env.ResidualRacerEnv` builds observation vectors with
this module, and `racer_train.contract_export` reads these same constants when it fills in
the deployment contract's `observation_schema` section -- one source of truth, per
claude-docs/02-repo-layout.md's "no file may duplicate ..." rule applied to schema, not just
physical constants.

Field names/units here match ros_ws/src/racer_policy/tests/conftest.py's `_TEMPLATE`
fixture, which was written against this exact schema ahead of S.3 landing it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# (name, units) pairs, IN ORDER. Order is part of the contract (claude-docs/08-learning.md):
# a live observation vector with the same fields in a different order is a hard refusal.
LOW_DIM_FIELDS: tuple[tuple[str, str], ...] = (
    ("velocity_mps", "m/s"),
    ("yaw_rate_rad_s", "rad/s"),
    ("slip_proxy", "dimensionless"),
    ("raceline_lateral_error_m", "m"),
    ("raceline_heading_error_rad", "rad"),
)
OBS_DTYPE = "float32"


@dataclass(frozen=True)
class LidarConfig:
    """LiDAR configuration as seen by the policy, AFTER downsampling. `raw_beam_count` and
    `fov_rad` must come from the live scan simulator at runtime (mirrors
    sim/bridge/racer_gym_bridge/racer_gym_bridge/conversions.py's rule: "never hand-write a
    second copy" of a sim sensor config), never hand-typed constants."""

    raw_beam_count: int
    fov_rad: float
    downsample_factor: int

    def __post_init__(self) -> None:
        if self.downsample_factor < 1:
            raise ValueError(f"downsample_factor must be >= 1, got {self.downsample_factor}")
        if self.beam_count < 1:
            raise ValueError(
                f"raw_beam_count={self.raw_beam_count} // downsample_factor="
                f"{self.downsample_factor} produces 0 beams"
            )

    @property
    def beam_count(self) -> int:
        return self.raw_beam_count // self.downsample_factor


def downsample_scan(ranges: np.ndarray, downsample_factor: int) -> np.ndarray:
    """Min-pool the raw scan into `downsample_factor`-wide bins.

    Min, not mean: a single close obstacle inside a bin must not be diluted by farther beams
    sharing that bin -- this observation feeds a policy that must not learn to steer into
    something a coarser average would have hidden. Any trailing beams that do not fill a
    whole bin are dropped (raw_beam_count is not required to be an exact multiple of
    downsample_factor).
    """
    n_bins = len(ranges) // downsample_factor
    usable = np.asarray(ranges, dtype=np.float32)[: n_bins * downsample_factor]
    return usable.reshape(n_bins, downsample_factor).min(axis=1)


def wrap_to_pi(angle_rad: float) -> float:
    """Wrap an angle to (-pi, pi]. Used for raceline heading error, which is a difference of
    two REP-103 yaw angles and must not blow up across the +-pi seam."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def raceline_relative_pose(raceline, x_m: float, y_m: float, yaw_rad: float) -> tuple[float, float]:
    """(lateral_error_m, heading_error_rad) of (x_m, y_m, yaw_rad) against the nearest
    raceline point.

    `lateral_error_m` is the vehicle-frame y offset of the nearest raceline point (REP-103:
    left positive) -- the same world-to-body rotation
    ros_ws/src/racer_control/src/pure_pursuit.cpp uses for its lookahead point, applied here
    to the nearest point instead.
    """
    nearest_index = raceline.nearest_index(x_m, y_m)
    point = raceline.at(nearest_index)
    dx = point.x_m - x_m
    dy = point.y_m - y_m
    cos_neg_yaw = math.cos(-yaw_rad)
    sin_neg_yaw = math.sin(-yaw_rad)
    lateral_error_m = sin_neg_yaw * dx + cos_neg_yaw * dy
    heading_error_rad = wrap_to_pi(point.heading_rad - yaw_rad)
    return lateral_error_m, heading_error_rad


def build_observation(
    *,
    velocity_mps: float,
    yaw_rate_rad_s: float,
    slip_proxy: float,
    lateral_error_m: float,
    heading_error_rad: float,
    downsampled_scan: np.ndarray,
) -> np.ndarray:
    """Assemble the full flat observation vector: low-dimensional fields (LOW_DIM_FIELDS
    order) followed by the downsampled LiDAR beams -- matches the contract's `normalization`
    section layout (claude-docs/08-learning.md: "one entry per flattened observation
    element ... the low-dimensional fields, in order, followed by the downsampled LiDAR
    beams", ros_ws/src/racer_policy/contract.schema.json)."""
    low_dim = np.array(
        [velocity_mps, yaw_rate_rad_s, slip_proxy, lateral_error_m, heading_error_rad],
        dtype=np.float32,
    )
    return np.concatenate([low_dim, np.asarray(downsampled_scan, dtype=np.float32)])
