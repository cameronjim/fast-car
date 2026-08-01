"""`ResidualRacerEnv`: the S.3 gymnasium training environment.

claude-docs/08-learning.md: the policy learns a bounded RESIDUAL on the base (classical)
controller's command, never a full command, and the layer-4 envelope
(training/envelope, claude-docs/05-safety.md) is enforced INSIDE this env -- the SAME
`envelope.apply()` the on-vehicle deploy node uses -- so the policy can never learn a
behavior deployment would clip ("train/deploy envelope divergence is a correctness bug").

Per step:
  1. Read the true pose from the sim (poses_x/y/theta -- this env trains against ground
     truth, not an estimated pose; state estimation is a separate, hardware-track concern).
  2. Compute the base command from `racer_train.raceline.PurePursuitController` (the Python
     port of racer_control's C++ core).
  3. Scale the policy's raw [-1, 1]^2 action into a physical residual `Command`.
  4. Run `envelope.apply(base, residual, ...)` -- this is the ONLY place a command is
     assembled; nothing bypasses it (mirrors CLAUDE.md invariant 1's shape for layer 4).
  5. Step the underlying `racer_gym` sim with the enveloped command.
  6. Build the next observation (racer_train.observation) and reward
     (racer_train.reward: progress - crash - envelope violation, nothing else).

This module stays torch-free (see package pyproject.toml).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, ClassVar

import gymnasium as gym
import numpy as np
import racer_gym
from envelope import Command, EnvelopeState
from envelope import apply as envelope_apply
from f1tenth_gym.envs.track import Track
from racer_gym.params import DynParams, build_dyn_params, load_vehicle_params

from racer_train.config import ExperimentConfig
from racer_train.envelope_config import build_envelope_config
from racer_train.observation import (
    LOW_DIM_FIELDS,
    OBS_DTYPE,
    LidarConfig,
    build_observation,
    downsample_scan,
    raceline_relative_pose,
)
from racer_train.raceline import PurePursuitConfig, PurePursuitController, Raceline
from racer_train.reward import RewardTerms, compute_reward

REPO_ROOT = Path(__file__).resolve().parents[4]


def _track_from_raceline(raceline: Raceline) -> Track:
    """Builds the f1tenth_gym `Track` the sim races on directly from the SAME committed
    raceline points the base controller tracks (mirrors
    sim/bridge/racer_gym_bridge/racer_gym_bridge/track_loader.py's approach for the L5
    tracker-lap canary): the training env's map and its base controller's reference are, by
    construction, exactly the same line."""
    n = len(raceline)
    x = np.array([raceline.at(i).x_m for i in range(n)], dtype=np.float64)
    y = np.array([raceline.at(i).y_m for i in range(n)], dtype=np.float64)
    v = np.array([raceline.at(i).target_speed_mps for i in range(n)], dtype=np.float64)
    return Track.from_refline(x=x, y=y, velx=v)


class ResidualRacerEnv(gym.Env):
    """Gymnasium env: action = bounded residual on the pure-pursuit base command."""

    metadata: ClassVar[dict[str, list[str]]] = {"render_modes": []}

    def __init__(
        self,
        config: ExperimentConfig,
        repo_root: Path | str = REPO_ROOT,
        vehicle_params: Any | None = None,
        render_mode: str | None = None,
    ) -> None:
        super().__init__()
        self.config = config
        repo_root = Path(repo_root)
        self._render_mode = render_mode

        if vehicle_params is None:
            vehicle_params = load_vehicle_params()
        self.vehicle_params = vehicle_params

        self.raceline = Raceline.load_from_csv(repo_root / config.env.track_csv)
        track = _track_from_raceline(self.raceline)

        self._gym_env = racer_gym.build_env(
            config={
                "map": track,
                "num_agents": 1,
                "seed": config.seed,
                "timestep": config.env.timestep_s,
                "observation_config": {"type": "original"},
            },
            vehicle_params=vehicle_params,
        )

        pp = config.pure_pursuit
        self._controller = PurePursuitController(
            PurePursuitConfig(
                wheelbase_m=vehicle_params.chassis.wheelbase_m,
                lookahead_min_m=pp.lookahead_min_m,
                lookahead_max_m=pp.lookahead_max_m,
                lookahead_curvature_ref_1pm=pp.lookahead_curvature_ref_1pm,
                max_steering_angle_rad=vehicle_params.steering.max_angle_rad,
            )
        )

        self._envelope_config = build_envelope_config(
            config.envelope, vehicle_params, config.env.timestep_s
        )

        scan_sim = self._gym_env.unwrapped.sim.agents[0].scan_simulator
        self.lidar_config = LidarConfig(
            raw_beam_count=scan_sim.num_beams,
            fov_rad=scan_sim.fov,
            downsample_factor=config.env.lidar_downsample_factor,
        )

        self.dyn_params: DynParams = build_dyn_params(
            vehicle_params, config.env.timestep_s
        ).dyn_params

        # Contract-facing schema, filled in once here so train.py/contract_export.py never
        # re-derive it independently of what this env actually produces (claude-docs/
        # 02-repo-layout.md: one source of truth).
        self.observation_fields: tuple[tuple[str, str, str], ...] = tuple(
            (name, OBS_DTYPE, units) for name, units in LOW_DIM_FIELDS
        )
        self.action_fields: tuple[tuple[str, float, float, str], ...] = (
            (
                "steering_residual_rad",
                -config.env.action.steering_scale_rad,
                config.env.action.steering_scale_rad,
                "rad",
            ),
            (
                "speed_residual_mps",
                -config.env.action.speed_scale_mps,
                config.env.action.speed_scale_mps,
                "m/s",
            ),
        )
        self.residual_limits = (
            config.envelope.residual_fraction_steering,
            config.envelope.residual_fraction_speed,
        )
        self.actuator_assumptions = {
            "rate_limit_steering_rad_s": float(vehicle_params.steering.max_rate_rad_per_s),
            "rate_limit_speed_mps_s": float(vehicle_params.actuation.max_acceleration_mps2),
            "command_delay_s": float(self.dyn_params.delay_steps * config.env.timestep_s),
        }

        obs_dim = len(LOW_DIM_FIELDS) + self.lidar_config.beam_count
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )
        # Raw policy action: a bounded residual in [-1, 1] per channel, scaled by
        # config.env.action to a physical (rad, m/s) residual BEFORE the envelope further
        # tightens it -- see this env's module docstring, step 3.
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self._envelope_state = EnvelopeState(last_output=Command(steering_rad=0.0, speed_mps=0.0))
        self._prev_s_m = 0.0
        self._step_count = 0

    # -- gymnasium API -------------------------------------------------------------------

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        gym_obs, info = self._gym_env.reset(seed=seed)
        self._envelope_state = EnvelopeState(last_output=Command(steering_rad=0.0, speed_mps=0.0))
        self._step_count = 0

        x, y, yaw = self._pose_from_obs(gym_obs)
        self._prev_s_m = self.raceline.at(self.raceline.nearest_index(x, y)).s_m

        obs = self._build_observation(gym_obs, x, y, yaw)
        return obs, info

    def step(self, action: np.ndarray):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        residual_steering_rad = float(action[0]) * self.config.env.action.steering_scale_rad
        residual_speed_mps = float(action[1]) * self.config.env.action.speed_scale_mps

        gym_obs = self._last_gym_obs
        x, y, yaw = self._pose_from_obs(gym_obs)
        pp_command = self._controller.compute_command(self.raceline, x, y, yaw)
        base_command = Command(
            steering_rad=pp_command.steering_angle_rad, speed_mps=pp_command.speed_mps
        )
        residual_command = Command(steering_rad=residual_steering_rad, speed_mps=residual_speed_mps)

        low_dim_now = self._low_dim_features(gym_obs, x, y, yaw)
        result = envelope_apply(
            self._envelope_config,
            self._envelope_state,
            base_command,
            residual_command,
            observed_state=low_dim_now,
        )
        self._envelope_state = result.next_state

        gym_action = np.array(
            [[result.command.steering_rad, result.command.speed_mps]], dtype=np.float64
        )
        next_gym_obs, _upstream_reward, terminated, truncated, info = self._gym_env.step(gym_action)
        self._last_gym_obs = next_gym_obs

        next_x, next_y, next_yaw = self._pose_from_obs(next_gym_obs)
        crashed = bool(np.asarray(next_gym_obs["collisions"])[0])
        s_new_m = self.raceline.at(self.raceline.nearest_index(next_x, next_y)).s_m

        envelope_intervened = result.residual_clipped or result.rate_limited or result.ood_triggered
        reward_terms: RewardTerms = compute_reward(
            s_prev_m=self._prev_s_m,
            s_new_m=s_new_m,
            track_length_m=self.raceline.length_m,
            crashed=crashed,
            envelope_intervened=envelope_intervened,
            weights=self.config.reward,
        )
        self._prev_s_m = s_new_m

        self._step_count += 1
        terminated = bool(terminated) or crashed
        truncated = bool(truncated) or self._step_count >= self.config.env.max_episode_steps

        obs = self._build_observation(next_gym_obs, next_x, next_y, next_yaw)
        info = dict(info)
        info["envelope"] = {
            "ood_triggered": result.ood_triggered,
            "residual_clipped": result.residual_clipped,
            "rate_limited": result.rate_limited,
            # The actual enveloped command sent to the sim this step -- exposed so tests
            # (and any training-side logger) can verify it against a direct
            # `envelope.apply()` call for the same inputs without reaching into private
            # state (claude-docs/05-safety.md: "an unlogged intervention is a bug").
            "command": {
                "steering_rad": result.command.steering_rad,
                "speed_mps": result.command.speed_mps,
            },
            "base_command": {
                "steering_rad": base_command.steering_rad,
                "speed_mps": base_command.speed_mps,
            },
            "residual_command": {
                "steering_rad": residual_command.steering_rad,
                "speed_mps": residual_command.speed_mps,
            },
        }
        info["reward"] = {
            "progress": reward_terms.progress,
            "crash": reward_terms.crash,
            "envelope_violation": reward_terms.envelope_violation,
        }
        return obs, reward_terms.total, terminated, truncated, info

    def close(self) -> None:
        self._gym_env.close()

    @property
    def envelope_config(self):
        """The `envelope.EnvelopeConfig` this env enforces every step -- exposed (read-only)
        so callers (tests, training-side loggers) can run a direct `envelope.apply()` call
        with the exact same config claude-docs/12-testing.md's L5 "envelope-in-env test"
        divergence check compares against, without reaching into a private attribute."""
        return self._envelope_config

    @property
    def envelope_state(self) -> EnvelopeState:
        """The `envelope.EnvelopeState` that will be threaded into the NEXT `step()` call
        (i.e. the state as of the last completed `step()`/`reset()`)."""
        return self._envelope_state

    # -- helpers --------------------------------------------------------------------------

    @staticmethod
    def _pose_from_obs(gym_obs: dict) -> tuple[float, float, float]:
        agent = gym_obs.get("agent_0")
        if agent is not None:
            return float(agent["pose_x"]), float(agent["pose_y"]), float(agent["pose_theta"])
        return (
            float(np.asarray(gym_obs["poses_x"])[0]),
            float(np.asarray(gym_obs["poses_y"])[0]),
            float(np.asarray(gym_obs["poses_theta"])[0]),
        )

    def _low_dim_features(self, gym_obs: dict, x: float, y: float, yaw: float) -> tuple[float, ...]:
        velocity_mps = float(np.asarray(gym_obs["linear_vels_x"])[0])
        yaw_rate_rad_s = float(np.asarray(gym_obs["ang_vels_z"])[0])
        # f1tenth_gym's "original" observation type does not expose the ST model's slip
        # angle (beta) directly -- it is index 6 of the raw agent state vector (see
        # sim/racer_gym/racer_gym/dynamics/model.py's state-vector docstring). Reading it
        # straight off the sim's ground-truth agent state is the same thing
        # sim/racer_gym/tests/test_determinism.py does for the same reason.
        slip_proxy = float(self._gym_env.unwrapped.sim.agents[0].state[6])
        lateral_error_m, heading_error_rad = raceline_relative_pose(self.raceline, x, y, yaw)
        return (velocity_mps, yaw_rate_rad_s, slip_proxy, lateral_error_m, heading_error_rad)

    def _build_observation(self, gym_obs: dict, x: float, y: float, yaw: float) -> np.ndarray:
        self._last_gym_obs = gym_obs
        velocity_mps, yaw_rate_rad_s, slip_proxy, lateral_error_m, heading_error_rad = (
            self._low_dim_features(gym_obs, x, y, yaw)
        )
        raw_scan = np.asarray(gym_obs["scans"])[0]
        downsampled = downsample_scan(raw_scan, self.lidar_config.downsample_factor)
        return build_observation(
            velocity_mps=velocity_mps,
            yaw_rate_rad_s=yaw_rate_rad_s,
            slip_proxy=slip_proxy,
            lateral_error_m=lateral_error_m,
            heading_error_rad=heading_error_rad,
            downsampled_scan=downsampled,
        )
