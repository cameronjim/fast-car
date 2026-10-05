# scripted gap-following opponent, ported from reactive_control/reactive_control/gap_logic.py

from __future__ import annotations

from dataclasses import dataclass, fields, replace

import numpy as np

STEER_MAX_RAD = 0.4189


@dataclass(frozen=True)
class GapFollowerConfig:
    """tunables for the scripted opponent; defaults are reactive_control's tuned gap_follow params."""

    clip_max_range_m: float = 3.5
    disparity_threshold_m: float = 1.0
    vehicle_half_width_m: float = 0.5
    free_space_threshold_m: float = 1.2
    corner_min_clearance_m: float = 0.2
    cone_left_fraction: float = 0.25
    cone_right_fraction: float = 0.75
    steer_gain: float = 3.0
    steer_damping_sec: float = 0.02
    steer_max_rad: float = STEER_MAX_RAD
    speed_cap_mps: float = 3.5
    speed_min_mps: float = 0.4
    target_distance_m: float = 0.6

    def __post_init__(self) -> None:
        if self.clip_max_range_m <= self.target_distance_m:
            raise ValueError(
                f"clip_max_range_m ({self.clip_max_range_m}) must exceed "
                f"target_distance_m ({self.target_distance_m})"
            )
        if self.speed_cap_mps <= self.speed_min_mps:
            raise ValueError(
                f"speed_cap_mps ({self.speed_cap_mps}) must exceed "
                f"speed_min_mps ({self.speed_min_mps})"
            )
        if not 0.0 <= self.cone_left_fraction < self.cone_right_fraction <= 1.0:
            raise ValueError(
                f"cone fractions must satisfy 0 <= left < right <= 1, got "
                f"{self.cone_left_fraction} and {self.cone_right_fraction}"
            )
        if self.steer_max_rad <= 0.0 or self.vehicle_half_width_m <= 0.0:
            raise ValueError("steer_max_rad and vehicle_half_width_m must both be > 0")

    @classmethod
    def from_dict(cls, blob: dict | None) -> "GapFollowerConfig":
        """build from a config mapping, rejecting keys that would otherwise be ignored."""
        blob = dict(blob or {})
        unknown = sorted(set(blob) - {field.name for field in fields(cls)})
        if unknown:
            raise ValueError(f"unknown opponent config keys: {unknown}")
        return cls(**blob)

    def to_dict(self) -> dict:
        return {field.name: getattr(self, field.name) for field in fields(self)}

    def with_updates(self, **changes) -> "GapFollowerConfig":
        return replace(self, **changes)


def extend_disparities(ranges, disparity_threshold_m, half_width_m, angle_increment):
    """shrink the rays behind each disparity edge so a chosen gap still fits the car."""
    safe_ranges = np.array(ranges, dtype=float)
    raw_ranges = np.asarray(ranges, dtype=float)

    edges = np.flatnonzero(np.abs(np.diff(raw_ranges)) > disparity_threshold_m)

    for i in edges:
        if raw_ranges[i] < raw_ranges[i + 1]:
            near = raw_ranges[i]
            direction = 1
            start = i + 1
        else:
            near = raw_ranges[i + 1]
            direction = -1
            start = i

        # half-width subtended at the near edge, converted to a ray count
        danger_rays = int(np.arctan2(half_width_m, near) / angle_increment)

        for step in range(danger_rays):
            k = start + direction * step
            if 0 <= k < len(safe_ranges) and safe_ranges[k] > near:
                safe_ranges[k] = near

    return safe_ranges


def select_target_ray(ranges, free_space_threshold_m, cone_left_fraction, cone_right_fraction):
    """center ray of the widest free gap in the forward cone, tie-broken toward straight."""
    ranges = np.asarray(ranges)
    num_rays = len(ranges)
    center = num_rays // 2

    # cone bounds are fractions of the scan so any ray count works
    left = int(num_rays * cone_left_fraction)
    right = int(num_rays * cone_right_fraction)
    cone = ranges[left:right]

    free = cone > free_space_threshold_m
    if not np.any(free):
        return center

    best_start = best_end = None
    best_len = -1
    best_center_dist = float("inf")

    i = 0
    while i < len(free):
        if not free[i]:
            i += 1
            continue
        start = i
        while i < len(free) and free[i]:
            i += 1
        end = i - 1

        gap_len = end - start + 1
        gap_center = (start + end) // 2
        dist_to_center = abs((gap_center + left) - center)

        if (gap_len > best_len) or (gap_len == best_len and dist_to_center < best_center_dist):
            best_len = gap_len
            best_start, best_end = start, end
            best_center_dist = dist_to_center

    return int((best_start + best_end) // 2 + left)


def corner_blocked(ranges, steering_angle, min_clearance_m):
    """true when the whole sector the car is turning into is inside min_clearance_m."""
    ranges = np.asarray(ranges)
    num_rays = len(ranges)
    if steering_angle < 0:
        return bool(np.all(ranges[int(num_rays * 5 / 6) :] < min_clearance_m))
    return bool(np.all(ranges[: int(num_rays / 6)] < min_clearance_m))


def target_bearing_rad(target_ray, angle_min_rad, angle_increment):
    """bearing of a ray from the scan's own geometry, not from an index-center assumption."""
    return float(angle_min_rad) + int(target_ray) * float(angle_increment)


def aim_clearance_m(ranges, target_ray, half_width_m, angle_increment):
    """closest range inside the cone the car's half width subtends at the ray it aims down."""
    ranges = np.asarray(ranges, dtype=float)
    num_rays = len(ranges)
    at_target = max(float(ranges[target_ray]), 1e-3)
    danger_rays = int(np.arctan2(half_width_m, at_target) / angle_increment)
    lower = max(target_ray - danger_rays, 0)
    upper = min(target_ray + danger_rays + 1, num_rays)
    # a zero-width cone leaves nothing to reduce over, so the aimed ray is the whole answer
    if lower >= upper:
        return at_target
    return float(np.min(ranges[lower:upper]))


class GapFollowerOpponent:
    """drives toward the widest lidar gap; the scripted car the rl ego races against."""

    def __init__(
        self,
        angle_min_rad: float,
        angle_increment_rad: float,
        control_period_sec: float,
        config: GapFollowerConfig | None = None,
    ):
        if angle_increment_rad <= 0.0:
            raise ValueError(f"angle_increment_rad must be > 0, got {angle_increment_rad}")
        if control_period_sec <= 0.0:
            raise ValueError(f"control_period_sec must be > 0, got {control_period_sec}")
        self.config = config or GapFollowerConfig()
        self.angle_min_rad = float(angle_min_rad)
        self.angle_increment_rad = float(angle_increment_rad)
        self.control_period_sec = float(control_period_sec)
        self._prev_angle_rad = 0.0

    def reset(self) -> None:
        """clear the derivative term so a new episode does not inherit the last one's error."""
        self._prev_angle_rad = 0.0

    def set_speed_cap(self, speed_cap_mps: float) -> None:
        self.config = self.config.with_updates(speed_cap_mps=float(speed_cap_mps))

    def plan(self, scan) -> tuple[float, float]:
        """steering angle in rad and speed command in m/s from one lidar sweep."""
        cfg = self.config
        ranges = np.asarray(scan, dtype=float)
        # a dropped or infinite beam reads as max range, the same reading the deploy nodes take
        ranges = np.nan_to_num(
            ranges, nan=cfg.clip_max_range_m, posinf=cfg.clip_max_range_m, neginf=0.0
        )
        ranges = np.clip(ranges, 0.0, cfg.clip_max_range_m)

        safe_ranges = extend_disparities(
            ranges, cfg.disparity_threshold_m, cfg.vehicle_half_width_m, self.angle_increment_rad
        )
        target_ray = select_target_ray(
            safe_ranges,
            cfg.free_space_threshold_m,
            cfg.cone_left_fraction,
            cfg.cone_right_fraction,
        )
        angle_rad = target_bearing_rad(target_ray, self.angle_min_rad, self.angle_increment_rad)
        if corner_blocked(safe_ranges, angle_rad, cfg.corner_min_clearance_m):
            angle_rad = 0.0

        rate = (angle_rad - self._prev_angle_rad) / self.control_period_sec
        steering = cfg.steer_gain * angle_rad + cfg.steer_damping_sec * rate
        self._prev_angle_rad = angle_rad
        steering = float(np.clip(steering, -cfg.steer_max_rad, cfg.steer_max_rad))
        return steering, self._speed_mps(safe_ranges, target_ray)

    def _speed_mps(self, safe_ranges, target_ray: int) -> float:
        """min-to-cap interpolation on how far the clearance the car aims into runs."""
        cfg = self.config
        clearance = aim_clearance_m(
            safe_ranges, target_ray, cfg.vehicle_half_width_m, self.angle_increment_rad
        )
        span = cfg.clip_max_range_m - cfg.target_distance_m
        fraction = float(np.clip((clearance - cfg.target_distance_m) / span, 0.0, 1.0))
        return float(cfg.speed_min_mps + fraction * (cfg.speed_cap_mps - cfg.speed_min_mps))
