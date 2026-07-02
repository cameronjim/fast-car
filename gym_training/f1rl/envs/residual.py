# residual rl: pure pursuit steers every physics step and the policy adds bounded deltas

from __future__ import annotations

import math
from dataclasses import dataclass, fields

import numpy as np

from ..planners import PurePursuitPlanner
from .f110_wrapper import F110RLWrapper
from .obs import ActionBounds, ObsConfig

# the reference-line block the residual policy sees, in this order, before prev_action
CONTEXT_FEATURES = (
    "ref_lateral_error",
    "ref_heading_error",
    "ref_curvature",
    "ref_speed",
    "ref_steer",
)

# standard_state is [x, y, steering, speed, yaw, yaw_rate, beta]
STATE_X = 0
STATE_Y = 1
STATE_SPEED = 3
STATE_YAW = 4


@dataclass(frozen=True)
class ResidualBounds:
    """how far one policy action may move the planner's steering and speed command."""

    dsteer_max_rad: float = 0.15
    dspeed_max_mps: float = 1.5

    def __post_init__(self) -> None:
        if self.dsteer_max_rad <= 0.0:
            raise ValueError(f"dsteer_max_rad must be > 0, got {self.dsteer_max_rad}")
        if self.dspeed_max_mps <= 0.0:
            raise ValueError(f"dspeed_max_mps must be > 0, got {self.dspeed_max_mps}")

    @classmethod
    def from_dict(cls, blob: dict | None) -> "ResidualBounds":
        """build from a config mapping, rejecting keys that would otherwise be ignored."""
        blob = dict(blob or {})
        unknown = sorted(set(blob) - {field.name for field in fields(cls)})
        if unknown:
            raise ValueError(f"unknown residual bounds keys: {unknown}")
        return cls(**blob)

    def to_dict(self) -> dict:
        return {field.name: getattr(self, field.name) for field in fields(self)}


def wrap_to_pi(angle: float) -> float:
    """angle folded into [-pi, pi), so heading errors never jump a full turn."""
    return float((float(angle) + math.pi) % (2.0 * math.pi) - math.pi)


def compose_command(
    steer_pp: float,
    speed_pp: float,
    unit,
    bounds: ResidualBounds,
    action_bounds: ActionBounds,
) -> np.ndarray:
    """planner command plus the policy's bounded delta, clipped at the vehicle and config limits."""
    delta = np.clip(np.asarray(unit, dtype=np.float64).reshape(2), -1.0, 1.0)
    steering = float(steer_pp) + delta[0] * bounds.dsteer_max_rad
    speed = float(speed_pp) + delta[1] * bounds.dspeed_max_mps
    return np.array(
        [
            np.clip(steering, -action_bounds.steer_max_rad, action_bounds.steer_max_rad),
            np.clip(speed, action_bounds.speed_min_mps, action_bounds.speed_cap_mps),
        ],
        dtype=np.float32,
    )


def raceline_context(
    planner: PurePursuitPlanner,
    obs_cfg: ObsConfig,
    x: float,
    y: float,
    yaw: float,
    speed_mps: float,
) -> np.ndarray:
    """unnormalized reference-line context, in the order obs_cfg declares it."""
    line = planner.line
    # errors are measured at the cog, which is the pose the policy is being asked to correct
    s, lateral_m = line.project(float(x), float(y))
    steer_pp, speed_pp = planner.plan(x, y, yaw, speed_mps)
    horizons = np.asarray(obs_cfg.curvature_horizons_m, dtype=np.float64)
    values = {
        "ref_lateral_error": np.array([lateral_m]),
        "ref_heading_error": np.array([wrap_to_pi(float(yaw) - line.heading_at(s))]),
        "ref_curvature": np.asarray(line.curvature_at(s + horizons)).reshape(-1),
        "ref_speed": np.array([speed_pp]),
        "ref_steer": np.array([steer_pp]),
    }
    return np.concatenate([values[name] for name in obs_cfg.context_features])


class ResidualPPWrapper(F110RLWrapper):
    """pure pursuit replans every physics step; the policy adds a delta at the control rate."""

    def __init__(
        self,
        env,
        obs_cfg: ObsConfig,
        action_bounds: ActionBounds,
        reward,
        planner: PurePursuitPlanner,
        residual_bounds: ResidualBounds,
        action_repeat: int = 2,
        wrong_way_steps: int = 0,
    ):
        unknown = sorted(set(obs_cfg.context_features) - set(CONTEXT_FEATURES))
        if unknown:
            raise ValueError(f"residual mode cannot build these context features: {unknown}")
        if not obs_cfg.context_features:
            raise ValueError("residual mode needs reference-line context features in ObsConfig")
        super().__init__(
            env,
            obs_cfg=obs_cfg,
            action_bounds=action_bounds,
            reward=reward,
            action_repeat=action_repeat,
            wrong_way_steps=wrong_way_steps,
        )
        self.planner = planner
        self.residual_bounds = residual_bounds

    def pose(self) -> tuple[float, float, float, float]:
        """x, y, yaw, and forward speed read straight off the simulator state."""
        state = self.env.unwrapped.sim.state.standard_state[0]
        return (
            float(state[STATE_X]),
            float(state[STATE_Y]),
            float(state[STATE_YAW]),
            float(state[STATE_SPEED]),
        )

    def command_for_substep(self, unit) -> np.ndarray:
        steer_pp, speed_pp = self.planner.plan(*self.pose())
        return compose_command(
            steer_pp, speed_pp, unit, self.residual_bounds, self.action_bounds
        )

    def context_vector(self) -> np.ndarray:
        return raceline_context(self.planner, self.obs_cfg, *self.pose())
