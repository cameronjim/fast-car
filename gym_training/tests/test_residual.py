# residual action composition and reference-line context, against a synthetic circle

import math

import gymnasium as gym
import numpy as np
import pytest

from f1rl.envs.obs import ActionBounds, ObsConfig
from f1rl.envs.residual import (
    CONTEXT_FEATURES,
    ResidualBounds,
    ResidualPPWrapper,
    compose_command,
    raceline_context,
    wrap_to_pi,
)
from f1rl.envs.reward import ProgressReward
from f1rl.planners import PurePursuitConfig, PurePursuitPlanner
from f1rl.track import RacelineIndex

CIRCLE_RADIUS_M = 10.0
CIRCLE_POINTS = 720
HORIZONS_M = (5.0, 10.0, 15.0, 20.0, 30.0)
REF_SPEED_MPS = 4.0


def circle_planner(speed_scale: float = 1.0) -> PurePursuitPlanner:
    """counter-clockwise circle of radius 10 m, pose taken at the rear axle."""
    angles = np.linspace(0.0, 2.0 * np.pi, CIRCLE_POINTS, endpoint=False)
    line = RacelineIndex(
        CIRCLE_RADIUS_M * np.cos(angles),
        CIRCLE_RADIUS_M * np.sin(angles),
        speeds=np.full(CIRCLE_POINTS, REF_SPEED_MPS),
    )
    config = PurePursuitConfig(cog_to_rear_axle_m=0.0, speed_scale=speed_scale)
    return PurePursuitPlanner(line, config)


def context_cfg(**changes) -> ObsConfig:
    base = dict(
        features=("scan", "linear_vel_x"),
        num_beams=4,
        context_features=CONTEXT_FEATURES,
        curvature_horizons_m=HORIZONS_M,
        speed_norm_mps=12.0,
    )
    base.update(changes)
    return ObsConfig(**base)


def test_bounds_reject_non_positive_and_unknown_keys():
    with pytest.raises(ValueError):
        ResidualBounds(dsteer_max_rad=0.0)
    with pytest.raises(ValueError):
        ResidualBounds(dspeed_max_mps=-1.0)
    with pytest.raises(ValueError, match="unknown residual bounds keys"):
        ResidualBounds.from_dict({"dsteer_max": 0.2})


def test_zero_action_passes_the_planner_command_through():
    bounds = ResidualBounds()
    action = ActionBounds(speed_min_mps=0.5, speed_cap_mps=12.0)
    assert compose_command(0.12, 7.5, [0.0, 0.0], bounds, action) == pytest.approx([0.12, 7.5])


def test_full_action_moves_by_exactly_the_delta_limit():
    bounds = ResidualBounds(dsteer_max_rad=0.15, dspeed_max_mps=1.5)
    action = ActionBounds(speed_min_mps=0.5, speed_cap_mps=12.0)
    assert compose_command(0.1, 7.0, [1.0, 1.0], bounds, action) == pytest.approx([0.25, 8.5])
    assert compose_command(0.1, 7.0, [-1.0, -1.0], bounds, action) == pytest.approx([-0.05, 5.5])
    # a delta outside the unit box is clipped before it is scaled, not after
    assert compose_command(0.1, 7.0, [9.0, -9.0], bounds, action) == pytest.approx([0.25, 5.5])


def test_composed_command_is_clipped_at_the_vehicle_and_config_limits():
    bounds = ResidualBounds(dsteer_max_rad=0.15, dspeed_max_mps=1.5)
    action = ActionBounds(steer_max_rad=0.4189, speed_min_mps=0.5, speed_cap_mps=12.0)
    assert compose_command(0.4189, 11.5, [1.0, 1.0], bounds, action) == pytest.approx(
        [0.4189, 12.0]
    )
    assert compose_command(-0.4189, 1.0, [-1.0, -1.0], bounds, action) == pytest.approx(
        [-0.4189, 0.5]
    )


def test_wrap_to_pi_folds_a_full_turn():
    assert wrap_to_pi(0.3) == pytest.approx(0.3)
    assert wrap_to_pi(0.3 + 2.0 * math.pi) == pytest.approx(0.3)
    assert wrap_to_pi(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)


def test_curvature_on_a_circle_is_one_over_the_radius_at_every_horizon():
    line = circle_planner().line
    curvatures = line.curvature_at(np.array(HORIZONS_M))
    assert curvatures == pytest.approx(np.full(len(HORIZONS_M), 1.0 / CIRCLE_RADIUS_M), rel=1e-3)


def test_curvature_lookup_wraps_around_the_lap():
    line = circle_planner().line
    assert float(line.curvature_at(-3.0)) == pytest.approx(
        float(line.curvature_at(line.length - 3.0))
    )
    assert float(line.curvature_at(line.length + 3.0)) == pytest.approx(
        float(line.curvature_at(3.0))
    )
    # three laps ahead is still the same point on the line
    assert float(line.curvature_at(3.0 * line.length + 7.0)) == pytest.approx(
        float(line.curvature_at(7.0))
    )


def test_heading_matches_the_circle_tangent_and_wraps():
    line = circle_planner().line
    assert line.heading_at(0.0) == pytest.approx(math.pi / 2.0, abs=1e-3)
    assert line.heading_at(0.5 * line.length) == pytest.approx(-math.pi / 2.0, abs=1e-3)
    assert line.heading_at(line.length + 1.0) == pytest.approx(line.heading_at(1.0), abs=1e-9)


def test_lateral_error_is_positive_left_of_the_line():
    planner, cfg = circle_planner(), context_cfg()
    # driving counter-clockwise at (10, 0) heading +y, inside the circle is the car's left
    inside = raceline_context(planner, cfg, CIRCLE_RADIUS_M - 1.0, 0.0, math.pi / 2.0, 4.0)
    outside = raceline_context(planner, cfg, CIRCLE_RADIUS_M + 1.0, 0.0, math.pi / 2.0, 4.0)
    lateral = cfg.context_slices["ref_lateral_error"]
    assert float(inside[lateral][0]) == pytest.approx(1.0, abs=0.01)
    assert float(outside[lateral][0]) == pytest.approx(-1.0, abs=0.01)


def test_heading_error_is_positive_when_the_car_points_left_of_the_line():
    planner, cfg = circle_planner(), context_cfg()
    heading = cfg.context_slices["ref_heading_error"]
    aligned = raceline_context(planner, cfg, CIRCLE_RADIUS_M, 0.0, math.pi / 2.0, 4.0)
    assert float(aligned[heading][0]) == pytest.approx(0.0, abs=1e-3)
    turned = raceline_context(planner, cfg, CIRCLE_RADIUS_M, 0.0, math.pi / 2.0 + 0.2, 4.0)
    assert float(turned[heading][0]) == pytest.approx(0.2, abs=1e-3)
    # the error must fold rather than read as most of a turn when the car faces backwards
    reversed_ = raceline_context(planner, cfg, CIRCLE_RADIUS_M, 0.0, -math.pi / 2.0 + 0.1, 4.0)
    assert float(reversed_[heading][0]) == pytest.approx(-math.pi + 0.1, abs=1e-3)


def test_context_reports_the_planner_speed_and_steering():
    planner, cfg = circle_planner(speed_scale=1.5), context_cfg()
    raw = raceline_context(planner, cfg, CIRCLE_RADIUS_M, 0.0, math.pi / 2.0, 4.0)
    steer_pp, speed_pp = planner.plan(CIRCLE_RADIUS_M, 0.0, math.pi / 2.0, 4.0)
    assert float(raw[cfg.context_slices["ref_speed"]][0]) == pytest.approx(speed_pp)
    assert float(raw[cfg.context_slices["ref_steer"]][0]) == pytest.approx(steer_pp)
    assert speed_pp == pytest.approx(1.5 * REF_SPEED_MPS)


def test_context_layout_and_normalization():
    cfg = context_cfg()
    assert cfg.context_dim == 9
    assert cfg.raw_dim == 5
    assert cfg.obs_dim == 5 + 9 + 2
    assert cfg.context_slices["ref_curvature"] == slice(2, 7)
    raw = np.zeros(cfg.context_dim, dtype=np.float32)
    raw[cfg.context_slices["ref_lateral_error"]] = 1.0
    raw[cfg.context_slices["ref_heading_error"]] = np.pi
    raw[cfg.context_slices["ref_curvature"]] = 0.25
    raw[cfg.context_slices["ref_speed"]] = 6.0
    raw[cfg.context_slices["ref_steer"]] = 0.4189
    obs = cfg.normalize(np.zeros(cfg.raw_dim, np.float32), [0.1, -0.2], raw)
    assert obs[cfg.raw_dim] == pytest.approx(0.5)
    assert obs[cfg.raw_dim + 1] == pytest.approx(1.0)
    assert obs[cfg.raw_dim + 2 : cfg.raw_dim + 7] == pytest.approx(np.full(5, 0.5))
    assert obs[cfg.raw_dim + 7] == pytest.approx(0.5)
    assert obs[cfg.raw_dim + 8] == pytest.approx(1.0)
    assert obs[-2:] == pytest.approx([0.1, -0.2])


def test_context_is_clipped_into_the_unit_box():
    cfg = context_cfg()
    raw = np.zeros(cfg.context_dim, dtype=np.float32)
    raw[cfg.context_slices["ref_lateral_error"]] = -40.0
    raw[cfg.context_slices["ref_curvature"]] = 9.0
    obs = cfg.normalize(np.zeros(cfg.raw_dim, np.float32), None, raw)
    assert np.all(np.abs(obs) <= 1.0)
    assert obs[cfg.raw_dim] == pytest.approx(-1.0)


def test_context_features_are_marked_undeployable():
    cfg = context_cfg()
    blob = cfg.to_json()
    deployable = {feature["name"]: feature["deployable"] for feature in blob["features"]}
    assert deployable["scan"] is True
    for name in CONTEXT_FEATURES:
        assert deployable[name] is False
    assert set(cfg.undeployable_features()) >= set(CONTEXT_FEATURES)


def test_context_shape_is_enforced():
    cfg = context_cfg()
    with pytest.raises(ValueError, match="context of shape"):
        cfg.normalize(np.zeros(cfg.raw_dim, np.float32), None, np.zeros(3, np.float32))
    with pytest.raises(ValueError, match="context dims"):
        cfg.normalize(np.zeros(cfg.raw_dim, np.float32), None, None)


class CircleSimEnv(gym.Env):
    """the slice of the f1tenth env the residual wrapper touches, with the car pinned in place."""

    def __init__(self, obs_cfg: ObsConfig):
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (obs_cfg.raw_dim,), np.float32)
        self.action_space = gym.spaces.Box(
            np.array([-0.4189, -5.0], np.float32), np.array([0.4189, 20.0], np.float32)
        )
        self.sim = self
        self.state = self
        self.standard_state = np.zeros((1, 7), dtype=np.float32)
        self.standard_state[0, 0] = CIRCLE_RADIUS_M
        self.standard_state[0, 4] = math.pi / 2.0
        self.commands: list[np.ndarray] = []
        self._raw = np.zeros(obs_cfg.raw_dim, dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        self.commands.clear()
        return self._raw, {}

    def step(self, action):
        self.commands.append(np.asarray(action, dtype=np.float64).copy())
        info = {
            "progress": np.array([0.1]),
            "lap_counts": np.array([0]),
            "lap_times": np.array([0.0]),
            "collisions": np.array([0.0]),
        }
        return self._raw, 0.0, False, False, info


def residual_env(action_repeat: int = 2) -> ResidualPPWrapper:
    obs_cfg = context_cfg(features=("scan",), num_beams=6)
    return ResidualPPWrapper(
        CircleSimEnv(obs_cfg),
        obs_cfg=obs_cfg,
        action_bounds=ActionBounds(speed_min_mps=0.5, speed_cap_mps=12.0),
        reward=ProgressReward(),
        planner=circle_planner(),
        residual_bounds=ResidualBounds(),
        action_repeat=action_repeat,
    )


def test_wrapper_replans_once_per_physics_step():
    env = residual_env(action_repeat=3)
    env.reset()
    env.step(np.array([0.0, 1.0], np.float32))
    assert len(env.unwrapped.commands) == 3
    steer_pp, speed_pp = env.planner.plan(CIRCLE_RADIUS_M, 0.0, math.pi / 2.0, 0.0)
    for command in env.unwrapped.commands:
        assert command == pytest.approx([steer_pp, speed_pp + 1.5], abs=1e-6)


def test_prev_action_stores_the_clipped_unit_deltas():
    env = residual_env()
    obs, _ = env.reset()
    assert obs.shape == (env.obs_cfg.obs_dim,)
    assert obs[-2:] == pytest.approx([0.0, 0.0])
    obs, _, _, _, _ = env.step(np.array([0.4, -0.6], np.float32))
    assert obs[-2:] == pytest.approx([0.4, -0.6], abs=1e-6)
    # out-of-box actions are stored as the clipped units the wrapper actually applied
    obs, _, _, _, _ = env.step(np.array([3.0, -3.0], np.float32))
    assert obs[-2:] == pytest.approx([1.0, -1.0])
    obs, _ = env.reset()
    assert obs[-2:] == pytest.approx([0.0, 0.0])


def test_wrapper_rejects_a_context_free_obs_config():
    obs_cfg = ObsConfig(features=("scan",), num_beams=6)
    with pytest.raises(ValueError, match="reference-line context"):
        ResidualPPWrapper(
            CircleSimEnv(obs_cfg),
            obs_cfg=obs_cfg,
            action_bounds=ActionBounds(),
            reward=ProgressReward(),
            planner=circle_planner(),
            residual_bounds=ResidualBounds(),
        )
