"""`ResidualRacerEnv`: the S.3 gymnasium training environment (extended for the v0 run).

claude-docs/08-learning.md: the policy learns a bounded RESIDUAL on the base (classical)
controller's command, never a full command, and the layer-4 envelope
(training/envelope, claude-docs/05-safety.md) is enforced INSIDE this env -- the SAME
`envelope.apply()` the on-vehicle deploy node uses -- so the policy can never learn a
behavior deployment would clip ("train/deploy envelope divergence is a correctness bug").

Per step:
  1. Sense: the true pose from the sim, corrupted by the episode's localiser-error process,
     is what the base controller and the observation see (the controller never gets ground
     truth when randomisation is on).
  2. Compute the base command from `racer_train.raceline.PurePursuitController` (the Python
     port of racer_control's C++ core).
  3. Scale the policy's raw [-1, 1]^2 action into a physical residual `Command`.
  4. Run `envelope.apply(base, residual, ...)` -- this is the ONLY place a command is
     assembled; nothing bypasses it (mirrors CLAUDE.md invariant 1's shape for layer 4).
  5. Step the underlying `racer_gym` sim with the enveloped command (after the per-episode
     steering gain/bias and speed-scale errors, which are properties of the CAR, not of the
     command the safety layers see).
  6. Build the next observation (racer_train.observation) and reward
     (racer_train.reward: progress - crash - envelope violation, nothing else), both from
     TRUE state.

Two modes share this class. The S.3 mode (no `env.tracks`, randomisation off, profile `s3`)
is byte-for-byte what S.3 shipped: one committed raceline, wall-free map, ground-truth
sensing. The v0 mode (`env.tracks` set, `randomization.enabled`, profile `v0`) draws a
procedural walled track and a domain-randomisation draw at every reset. For a PAIRED
evaluation, `reset(options=...)` accepts an explicit `track_index`, `episode_seed`, `draw`,
`start`, `laps` and `residual_enabled`, so the tracker-only arm and the tracker+residual arm
can be given identical worlds (see `racer_train.evaluate`).

This module stays torch-free (see package pyproject.toml).
"""

from __future__ import annotations

import math
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from typing import Any, ClassVar

import gymnasium as gym
import numpy as np
import racer_gym
from envelope import Command, EnvelopeState
from envelope import apply as envelope_apply
from f1tenth_gym.envs.track import Track
from racer_gym.dynamics.model import RacerCarAction, RacerSingleTrackModel
from racer_gym.params import DynParams, build_dyn_params, load_vehicle_params

from racer_train.config import ExperimentConfig
from racer_train.envelope_config import build_envelope_config
from racer_train.observation import (
    LIDAR_CLIP_M,
    LOW_DIM_PROFILES,
    OBS_DTYPE,
    OOD_STATE_DIM,
    LidarConfig,
    build_observation_vector,
    downsample_scan,
    raceline_relative_pose,
    slip_proxy_from,
)
from racer_train.raceline import PurePursuitConfig, PurePursuitController, Raceline
from racer_train.randomization import (
    EpisodeDraw,
    SensorNoise,
    apply_draw_to_dyn_params,
    sample_draw,
)
from racer_train.reward import RewardTerms, compute_reward, progress_reward
from racer_train.tracks import build_pool, start_pose
from racer_train.vehicle_overrides import apply_vehicle_overrides

REPO_ROOT = Path(__file__).resolve().parents[4]

# Curvature-preview distances (m of arc length ahead of the nearest raceline point).
CURVATURE_NEAR_M = 0.4
CURVATURE_FAR_M = 1.0
# Forward half angle of the TTC proxy sector (rad); see `_ttc_proxy`.
TTC_SECTOR_HALF_ANGLE_RAD = 0.3


def _track_from_raceline(raceline: Raceline) -> Track:
    """Builds the f1tenth_gym `Track` the sim races on directly from the SAME committed
    raceline points the base controller tracks (mirrors
    sim/bridge/racer_gym_bridge/racer_gym_bridge/track_loader.py's approach for the L5
    tracker-lap canary): the training env's map and its base controller's reference are, by
    construction, exactly the same line. S.3 mode only: this map has no walls."""
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
        track_pool: list | None = None,
    ) -> None:
        super().__init__()
        self.config = config
        repo_root = Path(repo_root)
        self._render_mode = render_mode
        self._dt = config.env.timestep_s
        self._profile = config.env.observation_profile
        if self._profile not in LOW_DIM_PROFILES:
            raise ValueError(
                f"unknown observation_profile {self._profile!r}; "
                f"expected one of {sorted(LOW_DIM_PROFILES)}"
            )
        self._low_dim_fields = LOW_DIM_PROFILES[self._profile]
        self._randomized = bool(config.randomization.enabled)

        if vehicle_params is None:
            vehicle_params = load_vehicle_params()
        # The overrides are applied to a copy; `self.vehicle_params` is what the whole env
        # (sim, envelope, contract) consumes, `self.committed_vehicle_params` is what the
        # contract's `vehicle_params` reference must name (the deploy node compares against
        # the LIVE yaml, not against a training-time override).
        self.committed_vehicle_params = vehicle_params
        vehicle_params = apply_vehicle_overrides(vehicle_params, config.vehicle_overrides)
        self.vehicle_params = vehicle_params

        if config.env.tracks is not None:
            tracks_cfg = config.env.tracks
            self.track_pool = (
                track_pool
                if track_pool is not None
                else build_pool(
                    tracks_cfg.family, tracks_cfg.train_pool_size, tracks_cfg.train_seed_base
                )
            )
        else:
            raceline = Raceline.load_from_csv(repo_root / config.env.track_csv)
            self.track_pool = [
                SimpleNamespace(
                    raceline=raceline,
                    gym_track=_track_from_raceline(raceline),
                    spec=None,
                    lane_half_width_m=None,
                )
            ]
        self._track_index = 0
        self.raceline: Raceline = self.track_pool[0].raceline

        self._gym_env = racer_gym.build_env(
            config={
                "map": self.track_pool[0].gym_track,
                "num_agents": 1,
                "seed": config.seed,
                "timestep": self._dt,
                "observation_config": {"type": "original"},
            },
            vehicle_params=vehicle_params,
        )
        self._agent = self._gym_env.unwrapped.sim.agents[0]

        pp = config.pure_pursuit
        self._wheelbase_m = float(vehicle_params.chassis.wheelbase_m)
        self._controller = PurePursuitController(
            PurePursuitConfig(
                wheelbase_m=self._wheelbase_m,
                lookahead_min_m=pp.lookahead_min_m,
                lookahead_max_m=pp.lookahead_max_m,
                lookahead_curvature_ref_1pm=pp.lookahead_curvature_ref_1pm,
                max_steering_angle_rad=vehicle_params.steering.max_angle_rad,
            )
        )

        self._envelope_config = build_envelope_config(
            config.envelope, vehicle_params, config.env.timestep_s
        )

        scan_sim = self._agent.scan_simulator
        self.lidar_config = LidarConfig(
            raw_beam_count=scan_sim.num_beams,
            fov_rad=scan_sim.fov,
            downsample_factor=config.env.lidar_downsample_factor,
        )

        build_result = build_dyn_params(vehicle_params, config.env.timestep_s)
        self.dyn_params: DynParams = build_result.dyn_params
        self._base_dyn_params: DynParams = build_result.dyn_params

        # TTC proxy constants, from vehicle_params (CLAUDE.md invariant 2).
        self._ttc_brake_s = float(vehicle_params.limits.ttc_brake_s)
        self._min_forward_clearance_m = float(vehicle_params.limits.min_forward_clearance_m)
        self._front_extent_m = float(
            vehicle_params.chassis.cg_to_front_axle_m + vehicle_params.chassis.front_overhang_m
        )
        angles = np.asarray(self._agent.scan_angles)
        self._ttc_beams = np.abs(angles) <= TTC_SECTOR_HALF_ANGLE_RAD

        # Contract-facing schema, filled in once here so train.py/contract_export.py never
        # re-derive it independently of what this env actually produces (claude-docs/
        # 02-repo-layout.md: one source of truth).
        self.observation_fields: tuple[tuple[str, str, str], ...] = tuple(
            (name, OBS_DTYPE, units) for name, units in self._low_dim_fields
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
        # Under randomisation the command delay is drawn per episode; the contract records the
        # centre of the range as the assumed value.
        if self._randomized:
            lo, hi = config.randomization.command_delay_s
            assumed_delay_s = 0.5 * (lo + hi)
        else:
            assumed_delay_s = float(self.dyn_params.delay_steps * config.env.timestep_s)
        self.actuator_assumptions = {
            "rate_limit_steering_rad_s": float(vehicle_params.steering.max_rate_rad_per_s),
            "rate_limit_speed_mps_s": float(vehicle_params.actuation.max_acceleration_mps2),
            "command_delay_s": assumed_delay_s,
        }

        obs_dim = len(self._low_dim_fields) + self.lidar_config.beam_count
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
        self._draw = EpisodeDraw()
        self._dynamics_overridden = False
        self._noise: SensorNoise | None = None
        self._residual_enabled = True
        self._laps_target = 0.0
        self._s_total_m = 0.0
        self._ttc_engaged = False
        self._ttc_engagements = 0
        self._scan_hold: np.ndarray | None = None
        self._sensor_fifo: deque = deque()
        self._prev_cmd = (0.0, 0.0)
        self._cur_base = Command(0.0, 0.0)
        self._cur_ood_state: tuple[float, ...] = (0.0,) * OOD_STATE_DIM
        self._last_gym_obs: dict = {}

    # -- gymnasium API -------------------------------------------------------------------

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        options = dict(options or {})
        episode_seed = options.get("episode_seed")
        if episode_seed is None:
            episode_seed = int(self.np_random.integers(0, 2**31 - 1))
        self._episode_seed = int(episode_seed)
        rng = np.random.default_rng([self._episode_seed, 104729])

        # Track.
        n_tracks = len(self.track_pool)
        track_index = options.get("track_index")
        track_index = int(rng.integers(0, n_tracks)) if track_index is None else int(track_index)
        self._select_track(track_index)

        # Randomisation draw (always consume the same rng calls, enabled or not).
        draw_rng_seed = int(rng.integers(0, 2**31 - 1))
        if "draw" in options and options["draw"] is not None:
            self._draw = options["draw"]
            self._dynamics_overridden = True
        elif self._randomized:
            self._draw = sample_draw(
                np.random.default_rng(draw_rng_seed), self.config.randomization, self._dt
            )
            self._dynamics_overridden = True
        else:
            self._draw = EpisodeDraw()
            self._dynamics_overridden = False
        self._apply_dynamics()

        self._residual_enabled = bool(options.get("residual_enabled", True))
        self._laps_target = float(options.get("laps", 0.0))

        # Start pose.
        start_mode = options.get("start", self.config.env.episode.start)
        ep = self.config.env.episode
        raceline = self.raceline
        n = len(raceline)
        if start_mode == "standing":
            idx, lat, head, speed_frac = 0, 0.0, 0.0, 0.0
        else:
            idx = int(options.get("start_index", rng.integers(0, n)))
            lat = float(rng.uniform(-ep.start_lateral_m, ep.start_lateral_m))
            head = float(rng.uniform(-ep.start_heading_rad, ep.start_heading_rad))
            speed_frac = float(rng.uniform(0.0, 1.0))
        x0, y0, yaw0 = start_pose(self.track_pool[track_index], idx, lat, head)
        v0 = speed_frac * raceline.at(idx).target_speed_mps

        gym_obs, info = self._gym_env.reset(
            seed=None, options={"poses": np.array([[x0, y0, yaw0]], dtype=np.float64)}
        )
        self._prime_actuators(v0)
        self._agent.state[3] = v0
        # `gym_env.reset` ran one zero-command step before the start speed was applied;
        # re-observe so the first observation matches the state the episode really starts in.
        gym_obs = self._gym_env.unwrapped.observation_type.observe()

        self._envelope_state = EnvelopeState(last_output=Command(steering_rad=0.0, speed_mps=v0))
        self._prev_cmd = (0.0, v0)
        self._step_count = 0
        self._s_total_m = 0.0
        self._ttc_engaged = False
        self._ttc_engagements = 0
        self._scan_hold = None
        self._sensor_fifo = deque()
        self._noise = SensorNoise(
            self._draw, self._episode_seed, self._dt, self.lidar_config.beam_count
        )

        x, y, _yaw = self._pose_from_obs(gym_obs)
        self._prev_s_m = raceline.at(raceline.nearest_index(x, y)).s_m
        obs = self._sense(gym_obs)
        info = dict(info)
        info["track_label"] = (
            self.track_pool[track_index].spec.label
            if getattr(self.track_pool[track_index], "spec", None) is not None
            else "committed"
        )
        info["episode_seed"] = self._episode_seed
        info["track_index"] = track_index
        return obs, info

    def step(self, action: np.ndarray):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        residual_steering_rad = float(action[0]) * self.config.env.action.steering_scale_rad
        residual_speed_mps = float(action[1]) * self.config.env.action.speed_scale_mps
        if not self._residual_enabled:
            residual_steering_rad = 0.0
            residual_speed_mps = 0.0

        base_command = self._cur_base
        residual_command = Command(steering_rad=residual_steering_rad, speed_mps=residual_speed_mps)
        result = envelope_apply(
            self._envelope_config,
            self._envelope_state,
            base_command,
            residual_command,
            observed_state=self._cur_ood_state,
        )
        self._envelope_state = result.next_state
        self._prev_cmd = (result.command.steering_rad, result.command.speed_mps)

        draw = self._draw
        if self._dynamics_overridden:
            steer_to_sim = draw.steering_gain * result.command.steering_rad + draw.steering_bias_rad
            speed_to_sim = draw.speed_scale * result.command.speed_mps
        else:
            steer_to_sim = result.command.steering_rad
            speed_to_sim = result.command.speed_mps
        gym_action = np.array([[steer_to_sim, speed_to_sim]], dtype=np.float64)
        next_gym_obs, _upstream_reward, terminated, truncated, info = self._gym_env.step(gym_action)

        next_x, next_y, _next_yaw = self._pose_from_obs(next_gym_obs)
        crashed = bool(np.asarray(next_gym_obs["collisions"])[0])
        nearest = self.raceline.nearest_index(next_x, next_y)
        s_new_m = self.raceline.at(nearest).s_m

        envelope_intervened = result.residual_clipped or result.rate_limited or result.ood_triggered
        reward_terms: RewardTerms = compute_reward(
            s_prev_m=self._prev_s_m,
            s_new_m=s_new_m,
            track_length_m=self.raceline.length_m,
            crashed=crashed,
            envelope_intervened=envelope_intervened,
            weights=self.config.reward,
        )
        self._s_total_m += progress_reward(self._prev_s_m, s_new_m, self.raceline.length_m)
        self._prev_s_m = s_new_m

        self._step_count += 1
        laps_done = self._laps_target > 0.0 and (
            self._s_total_m >= self._laps_target * self.raceline.length_m
        )
        # The upstream `terminated` also fires when f1tenth_gym's own start-gate logic decides a
        # lap is done (about one lap after a standing start); the real termination conditions
        # here are a wall contact or this episode's own lap target. Treating the gate as
        # terminal would truncate training episodes AND wrongly zero the value bootstrap.
        del terminated
        terminated = crashed or laps_done
        truncated = bool(truncated) or self._step_count >= self.config.env.max_episode_steps

        ttc_now, clearance_m = self._ttc_proxy(next_gym_obs)
        if ttc_now and not self._ttc_engaged:
            self._ttc_engagements += 1
        self._ttc_engaged = ttc_now

        obs = self._sense(next_gym_obs)
        info = dict(info)
        info["envelope"] = {
            "ood_triggered": result.ood_triggered,
            "residual_clipped": result.residual_clipped,
            "rate_limited": result.rate_limited,
            # The actual enveloped command sent to the sim env this step -- exposed so tests
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
        info["truth"] = {
            "cross_track_error_m": self._true_cross_track_m(next_gym_obs),
            "speed_mps": float(np.asarray(next_gym_obs["linear_vels_x"])[0]),
            "s_total_m": self._s_total_m,
            "laps": self._s_total_m / self.raceline.length_m,
            "crashed": crashed,
            "success": bool(laps_done and not crashed),
            "ttc_engaged": ttc_now,
            "ttc_engagements": self._ttc_engagements,
            "forward_clearance_m": clearance_m,
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

    @property
    def episode_draw(self) -> EpisodeDraw:
        return self._draw

    @property
    def current_track(self):
        return self.track_pool[self._track_index]

    # -- episode setup helpers -----------------------------------------------------------

    def _select_track(self, index: int) -> None:
        if index != self._track_index or not hasattr(self, "_map_set"):
            self._gym_env.unwrapped.sim.set_map(self.track_pool[index].gym_track)
            self._map_set = True
        self._track_index = index
        self.raceline = self.track_pool[index].raceline

    def _apply_dynamics(self) -> None:
        if self._dynamics_overridden:
            dyn = apply_draw_to_dyn_params(self._base_dyn_params, self._draw, self._dt)
        else:
            dyn = self._base_dyn_params
        self.dyn_params = dyn
        self._agent.model = RacerSingleTrackModel(dyn)
        self._agent.action_type = RacerCarAction(dyn, self._dt)

    def _prime_actuators(self, v0: float) -> None:
        action_type = self._agent.action_type
        action_type._throttle_delay.reset(v0)
        action_type._throttle_actuator.reset(v0)

    # -- sensing -------------------------------------------------------------------------

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

    def _true_cross_track_m(self, gym_obs: dict) -> float:
        x, y, _yaw = self._pose_from_obs(gym_obs)
        point = self.raceline.at(self.raceline.nearest_index(x, y))
        dx, dy = x - point.x_m, y - point.y_m
        return -dx * math.sin(point.heading_rad) + dy * math.cos(point.heading_rad)

    def _ttc_proxy(self, gym_obs: dict) -> tuple[bool, float]:
        """A stand-in for layer 3's TTC gate (racer_safety is C++ and not in this loop):
        forward LiDAR clearance minus the body's front extent, divided by speed. The real gate
        has swept-corridor and per-source logic this does not; the count is a RELATIVE
        indicator between the two arms of a paired evaluation, not a prediction of how many
        times the real safety node would fire."""
        scan = np.asarray(gym_obs["scans"])[0]
        clearance = float(scan[self._ttc_beams].min()) - self._front_extent_m
        speed = float(np.asarray(gym_obs["linear_vels_x"])[0])
        if speed < 0.2:
            return False, clearance
        engaged = clearance < self._min_forward_clearance_m or (
            clearance / speed < self._ttc_brake_s
        )
        return engaged, clearance

    def _sense(self, gym_obs: dict) -> np.ndarray:
        """Build the observation for the NEXT decision and stash the base command and OOD
        state `step()` will use. Everything the policy sees goes through here."""
        self._last_gym_obs = gym_obs
        x, y, yaw = self._pose_from_obs(gym_obs)
        raceline = self.raceline

        if self._dynamics_overridden or self._randomized:
            noise = self._noise.step()
            ex, ey, eyaw = (
                x + noise["pose_error"][0],
                y + noise["pose_error"][1],
                yaw + noise["pose_error"][2],
            )
        else:
            noise = None
            ex, ey, eyaw = x, y, yaw

        pp_command = self._controller.compute_command(raceline, ex, ey, eyaw)
        base_command = Command(
            steering_rad=pp_command.steering_angle_rad, speed_mps=pp_command.speed_mps
        )
        self._cur_base = base_command

        true_v = float(np.asarray(gym_obs["linear_vels_x"])[0])
        true_yaw_rate = float(np.asarray(gym_obs["ang_vels_z"])[0])
        lateral_error_m, heading_error_rad = raceline_relative_pose(raceline, ex, ey, eyaw)

        raw_scan = np.asarray(gym_obs["scans"])[0]
        scan_ds = downsample_scan(raw_scan, self.lidar_config.downsample_factor)

        if noise is not None:
            velocity = true_v + noise["velocity_noise_mps"]
            yaw_rate = true_yaw_rate + noise["yaw_rate_noise_rad_s"]
            if noise["lidar_refresh"] or self._scan_hold is None:
                held = scan_ds.astype(np.float64) + noise["lidar_noise_m"]
                held = np.where(noise["lidar_dropout"], LIDAR_CLIP_M, held)
                self._scan_hold = np.clip(held, 0.0, LIDAR_CLIP_M).astype(np.float32)
            scan_meas = self._scan_hold
        else:
            velocity, yaw_rate = true_v, true_yaw_rate
            scan_meas = scan_ds
        if self._profile == "v0":
            scan_meas = np.minimum(scan_meas, LIDAR_CLIP_M)

        # Observation latency: delay the SENSED quantities (not the controller's own state).
        sensed = (velocity, yaw_rate, lateral_error_m, heading_error_rad, scan_meas)
        self._sensor_fifo.append(sensed)
        latency = self._draw.obs_latency_steps if self._dynamics_overridden else 0
        while len(self._sensor_fifo) > latency + 1:
            self._sensor_fifo.popleft()
        velocity, yaw_rate, lateral_error_m, heading_error_rad, scan_meas = self._sensor_fifo[0]

        if self._profile == "v0":
            slip = slip_proxy_from(self._prev_cmd[0], yaw_rate, velocity, self._wheelbase_m)
            nearest = raceline.nearest_index(ex, ey)
            k_near = raceline.at(raceline.index_ahead(nearest, CURVATURE_NEAR_M)).curvature_1pm
            k_far = raceline.at(raceline.index_ahead(nearest, CURVATURE_FAR_M)).curvature_1pm
            low_dim: tuple[float, ...] = (
                velocity,
                yaw_rate,
                slip,
                lateral_error_m,
                heading_error_rad,
                base_command.steering_rad,
                base_command.speed_mps,
                k_near,
                k_far,
                self._prev_cmd[0],
                self._prev_cmd[1],
            )
        else:
            # S.3 definition: the sim's true slip angle (index 6 of the raw agent state).
            slip = float(self._agent.state[6])
            low_dim = (velocity, yaw_rate, slip, lateral_error_m, heading_error_rad)

        self._cur_ood_state = tuple(float(v) for v in low_dim[:OOD_STATE_DIM])
        return build_observation_vector(low_dim, scan_meas)
