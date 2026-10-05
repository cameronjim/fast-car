# pure pursuit steering on the raceline, with speed read from the raceline profile

from __future__ import annotations

import math
from dataclasses import dataclass, fields

from ..track import RacelineIndex

STEER_MAX_RAD = 0.4189
WHEELBASE_M = 0.3302
COG_TO_REAR_AXLE_M = 0.17145
VEHICLE_SPEED_MAX_MPS = 20.0


@dataclass(frozen=True)
class PurePursuitConfig:
    """tunables for raceline pure pursuit."""

    # tuned on spielberg: longer lookaheads understeer, and under 0.5 m clips walls on monza
    lookahead_gain_sec: float = 0.05
    lookahead_min_m: float = 0.5
    lookahead_max_m: float = 4.0
    speed_scale: float = 1.2
    speed_lookahead_m: float = 0.0
    fallback_speed_mps: float = 4.0
    speed_min_mps: float = 0.5
    speed_max_mps: float = VEHICLE_SPEED_MAX_MPS
    wheelbase_m: float = WHEELBASE_M
    steer_max_rad: float = STEER_MAX_RAD
    cog_to_rear_axle_m: float = COG_TO_REAR_AXLE_M

    def __post_init__(self) -> None:
        if self.lookahead_min_m <= 0.0:
            raise ValueError(f"lookahead_min_m must be > 0, got {self.lookahead_min_m}")
        if self.lookahead_max_m < self.lookahead_min_m:
            raise ValueError(
                f"lookahead_max_m ({self.lookahead_max_m}) must be >= "
                f"lookahead_min_m ({self.lookahead_min_m})"
            )
        if self.lookahead_gain_sec < 0.0:
            raise ValueError(f"lookahead_gain_sec must be >= 0, got {self.lookahead_gain_sec}")
        if self.speed_scale <= 0.0:
            raise ValueError(f"speed_scale must be > 0, got {self.speed_scale}")
        if self.speed_max_mps <= self.speed_min_mps:
            raise ValueError(
                f"speed_max_mps ({self.speed_max_mps}) must exceed "
                f"speed_min_mps ({self.speed_min_mps})"
            )
        if self.steer_max_rad <= 0.0 or self.wheelbase_m <= 0.0:
            raise ValueError("steer_max_rad and wheelbase_m must both be > 0")

    @classmethod
    def from_dict(cls, blob: dict | None) -> "PurePursuitConfig":
        """build from a config mapping, rejecting keys that would otherwise be ignored."""
        blob = dict(blob or {})
        unknown = sorted(set(blob) - {field.name for field in fields(cls)})
        if unknown:
            raise ValueError(f"unknown pure pursuit config keys: {unknown}")
        return cls(**blob)

    def to_dict(self) -> dict:
        return {field.name: getattr(self, field.name) for field in fields(self)}


class PurePursuitPlanner:
    """steers at a raceline point one velocity-scaled lookahead ahead, at whatever rate it is called."""

    def __init__(self, track, config: PurePursuitConfig | None = None, use_centerline: bool = False):
        self.config = config or PurePursuitConfig()
        self.line = (
            track
            if isinstance(track, RacelineIndex)
            else RacelineIndex.from_track(track, use_centerline=use_centerline)
        )

    @property
    def has_speed_profile(self) -> bool:
        return self.line.has_speed_profile

    def lookahead_m(self, speed_mps: float) -> float:
        """velocity-scaled lookahead distance, clamped to the configured band."""
        cfg = self.config
        raw = cfg.lookahead_gain_sec * max(float(speed_mps), 0.0) + cfg.lookahead_min_m
        return float(min(max(raw, cfg.lookahead_min_m), cfg.lookahead_max_m))

    def plan(self, x: float, y: float, yaw: float, speed_mps: float) -> tuple[float, float]:
        """steering angle in rad and speed command in m/s for the current pose."""
        # the single-track model reports the pose at the cog, pure pursuit geometry is rear-axle
        offset = self.config.cog_to_rear_axle_m
        rear_x = float(x) - offset * math.cos(yaw)
        rear_y = float(y) - offset * math.sin(yaw)
        s, _ = self.line.project(rear_x, rear_y)
        goal_x, goal_y = self.line.point_at(s + self.lookahead_m(speed_mps))
        return self.steering_to(rear_x, rear_y, yaw, goal_x, goal_y), self.speed_at(s)

    def steering_to(self, x: float, y: float, yaw: float, goal_x: float, goal_y: float) -> float:
        """pure pursuit steering angle onto a goal point, clipped to the steering limit."""
        cfg = self.config
        dx, dy = goal_x - x, goal_y - y
        ahead = math.cos(yaw) * dx + math.sin(yaw) * dy
        left = -math.sin(yaw) * dx + math.cos(yaw) * dy
        chord_sq = ahead * ahead + left * left
        # a goal on top of the car has no defined arc, so hold the wheel straight
        if chord_sq < 1e-9:
            return 0.0
        steering = math.atan(cfg.wheelbase_m * 2.0 * left / chord_sq)
        return float(min(max(steering, -cfg.steer_max_rad), cfg.steer_max_rad))

    def speed_at(self, s: float) -> float:
        """raceline speed at arc-length s, scaled and clamped to the commandable band."""
        cfg = self.config
        # speed_scale trims the raceline profile; the fallback is already an absolute command
        if not self.line.has_speed_profile:
            target = cfg.fallback_speed_mps
        else:
            target = cfg.speed_scale * self.line.speed_at(s + cfg.speed_lookahead_m)
        return float(min(max(target, cfg.speed_min_mps), cfg.speed_max_mps))
