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

# v0 profile (2026-11): the S.3 five plus what a RESIDUAL needs to be a function of. A residual
# acts on the base controller's command, so it must see that command (and the command it just
# sent, because actuator lag and transport delay make the history matter), plus a short
# curvature preview of the raceline ahead (the base controller is the only other thing that
# knows the corner is coming). Every field is derivable on the car from the tracker's own
# internals, /odom and the localiser; none needs a sensor the car lacks. `slip_proxy` is the
# kinematic understeer indicator tan(delta_cmd) - L*yaw_rate/max(v, v_floor): zero when the
# car turns as its steering commands say, nonzero when it slides; it replaces the S.3
# definition (the sim's true slip angle, which no sensor on the car can measure).
LOW_DIM_FIELDS_V0: tuple[tuple[str, str], ...] = (
    ("velocity_mps", "m/s"),
    ("yaw_rate_rad_s", "rad/s"),
    ("slip_proxy", "dimensionless"),
    ("raceline_lateral_error_m", "m"),
    ("raceline_heading_error_rad", "rad"),
    ("base_steering_rad", "rad"),
    ("base_speed_mps", "m/s"),
    ("raceline_curvature_near_1pm", "1/m"),
    ("raceline_curvature_far_1pm", "1/m"),
    ("prev_steering_cmd_rad", "rad"),
    ("prev_speed_cmd_mps", "m/s"),
)
# The envelope's OOD scorer looks only at the first five (the "where is the car" state).
OOD_STATE_DIM = 5

LOW_DIM_PROFILES: dict[str, tuple[tuple[str, str], ...]] = {
    "s3": LOW_DIM_FIELDS,
    "v0": LOW_DIM_FIELDS_V0,
}
OBS_DTYPE = "float32"

# Beyond this the LiDAR return carries no information a 1 m lane policy can use, and the real
# sensor's 12 m limit differs from the sim's: clip so the two agree.
LIDAR_CLIP_M = 5.0
SLIP_PROXY_SPEED_FLOOR_MPS = 0.3
SLIP_PROXY_LIMIT = 2.0


def slip_proxy_from(
    steering_cmd_rad: float, yaw_rate_rad_s: float, velocity_mps: float, wheelbase_m: float
) -> float:
    """Kinematic understeer indicator, dimensionless: tan(delta) - L * yaw_rate / v.

    `wheelbase_m` comes from vehicle_params at the call site (CLAUDE.md invariant 2). The
    speed floor keeps the ratio finite at standstill; the result is clipped to +-2.
    """
    v = max(abs(velocity_mps), SLIP_PROXY_SPEED_FLOOR_MPS)
    value = math.tan(steering_cmd_rad) - wheelbase_m * yaw_rate_rad_s / v
    return max(-SLIP_PROXY_LIMIT, min(SLIP_PROXY_LIMIT, value))


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


def build_observation_vector(
    low_dim: tuple[float, ...], downsampled_scan: np.ndarray
) -> np.ndarray:
    """Flat float32 observation for any profile: `low_dim` (profile order) then the beams."""
    return np.concatenate(
        [np.asarray(low_dim, dtype=np.float32), np.asarray(downsampled_scan, dtype=np.float32)]
    )
