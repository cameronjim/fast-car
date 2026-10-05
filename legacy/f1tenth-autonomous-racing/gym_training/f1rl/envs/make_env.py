# builds the training env stack from a yaml config

from __future__ import annotations

from pathlib import Path

import gymnasium as gym
import numpy as np
import yaml
from f1tenth_gym.envs.dynamic_models import F1TENTH_VEHICLE_PARAMETERS
from f1tenth_gym.envs.env_config import (
    DomainRandomizationConfig,
    EnvConfig,
    ObservationConfig,
    ResetConfig,
    SimulationConfig,
    TerminationConfig,
)
from f1tenth_gym.envs.lidar import LiDARConfig
from f1tenth_gym.envs.observation import ObservationType
from f1tenth_gym.envs.reset import ReferenceLine, ResetStrategy
from f1tenth_gym.envs.wrappers import SingleAgentWrapper
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

from ..planners import PurePursuitConfig, PurePursuitPlanner
from ..track import RacelineIndex, generated_raceline_path, raceline_index_from_csv
from .f110_wrapper import F110RLWrapper
from .obs import DEFAULT_CURVATURE_HORIZONS_M, ActionBounds, ObsConfig
from .opponent import GapFollowerConfig
from .residual import CONTEXT_FEATURES, ResidualBounds, ResidualPPWrapper
from .reward import ProgressReward
from .versus import OvertakeBonus, VersusConfig, VersusEgoWrapper

SIM_TIMESTEP_SEC = 0.01

DEFAULT_FEATURES = ("scan", "linear_vel_x", "ang_vel_z", "delta", "frenet_pose")

RESIDUAL_KEYS = frozenset(
    {
        "enabled",
        "reference",
        "raceline_csv",
        "dsteer_max_rad",
        "dspeed_max_mps",
        "curvature_horizons_m",
        "pure_pursuit",
    }
)


def load_config(path) -> dict:
    """read a training yaml, failing loudly with the path when it is missing."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"config not found: {path.resolve()}")
    cfg = yaml.safe_load(path.read_text())
    for section in ("env", "algo", "run"):
        if section not in cfg:
            raise ValueError(f"config {path} is missing the '{section}' section")
    return cfg


def _map_for_index(env_cfg: dict, index: int) -> str:
    maps = env_cfg.get("maps") or ["Spielberg"]
    return str(maps[index % len(maps)])


def _domain_randomization(env_cfg: dict) -> DomainRandomizationConfig:
    """per-episode vehicle-parameter spread, given as absolute [low, high] pairs per field."""
    dr_cfg = dict(env_cfg.get("domain_randomization") or {})
    if not dr_cfg.pop("enabled", False):
        return DomainRandomizationConfig()
    base = F1TENTH_VEHICLE_PARAMETERS
    low: dict[str, float] = {}
    high: dict[str, float] = {}
    for name, span in dr_cfg.items():
        if not hasattr(base, name):
            raise ValueError(f"{name!r} is not a vehicle parameter, so it cannot be randomized")
        if len(span) != 2:
            raise ValueError(f"domain_randomization.{name} must be a [low, high] pair, got {span}")
        low[name], high[name] = float(span[0]), float(span[1])
    if not low:
        raise ValueError("domain_randomization is enabled but names no parameter to vary")
    return DomainRandomizationConfig(
        enabled=True, low=base.with_updates(**low), high=base.with_updates(**high)
    )


def _env_config(
    env_cfg: dict, seed: int, map_name: str, render: bool, num_agents: int = 1
) -> EnvConfig:
    action_repeat = int(env_cfg.get("action_repeat", 1))
    max_control_steps = env_cfg.get("max_episode_steps")
    lidar = LiDARConfig(
        num_beams=int(env_cfg.get("num_beams", 108)),
        noise_std=float(env_cfg.get("lidar_noise_std", 0.01)),
        dropout_prob=float(env_cfg.get("lidar_dropout_prob", 0.0)),
        range_bias_std=float(env_cfg.get("lidar_range_bias_std", 0.0)),
        range_max=float(env_cfg.get("scan_range_max_m", 30.0)),
    )
    reset = ResetConfig(
        strategy=ResetStrategy[env_cfg.get("reset_strategy", "RL_GRID_STATIC")],
        reference_line=ReferenceLine[env_cfg.get("reference_line", "RACELINE")],
    )
    simulation = SimulationConfig(
        timestep=SIM_TIMESTEP_SEC,
        max_laps=None if env_cfg.get("continuous_laps", True) else 1,
    )
    termination = TerminationConfig(
        max_episode_steps=None if max_control_steps is None else int(max_control_steps) * action_repeat,
        terminate_on_collision=True,
        # ego only: a scripted opponent crashing on its own must not end the ego's episode
        collision_agents="ego",
    )
    return EnvConfig(
        seed=seed,
        map_name=map_name,
        num_agents=num_agents,
        observation_config=ObservationConfig(
            type=ObservationType.FEATURES,
            features=tuple(env_cfg.get("features", DEFAULT_FEATURES)),
        ),
        lidar_config=lidar,
        reset_config=reset,
        simulation_config=simulation,
        termination_config=termination,
        domain_randomization_config=_domain_randomization(env_cfg),
        render_enabled=render,
    )


def _residual_config(env_cfg: dict) -> dict | None:
    """the residual block when it is switched on, with unknown keys rejected loudly."""
    residual_cfg = dict(env_cfg.get("residual") or {})
    unknown = sorted(set(residual_cfg) - RESIDUAL_KEYS)
    if unknown:
        raise ValueError(f"unknown residual config keys: {unknown}")
    return residual_cfg if residual_cfg.pop("enabled", False) else None


def _versus_config(env_cfg: dict) -> tuple[VersusConfig, GapFollowerConfig] | None:
    """the head-to-head block when it is switched on, as a race config plus an opponent config."""
    versus_cfg = dict(env_cfg.get("versus") or {})
    if not versus_cfg.pop("enabled", False):
        return None
    opponent_cfg = versus_cfg.pop("opponent", None)
    return VersusConfig.from_dict(versus_cfg), GapFollowerConfig.from_dict(opponent_cfg)


def _reference_line(track, residual_cfg: dict, map_name: str) -> RacelineIndex:
    """the line the embedded planner follows: shipped, generated, the centerline, or a named csv."""
    source = str(residual_cfg.get("reference", "shipped")).lower()
    if source == "shipped":
        return RacelineIndex.from_track(track)
    if source == "centerline":
        return RacelineIndex.from_track(track, use_centerline=True)
    if source == "generated":
        path = generated_raceline_path(track)
        if not path.is_file():
            raise FileNotFoundError(
                f"{map_name} has no generated raceline at {path}; run f1rl.track.generate_raceline"
            )
        return raceline_index_from_csv(path)
    if source == "csv":
        csv = residual_cfg.get("raceline_csv")
        if isinstance(csv, dict):
            csv = csv.get(map_name)
        if not csv:
            raise ValueError(f"residual.reference is 'csv' but no raceline_csv covers {map_name}")
        return raceline_index_from_csv(Path(csv))
    raise ValueError(
        f"unknown residual reference {source!r}, expected shipped, generated, centerline, or csv"
    )


def build_env(
    cfg: dict,
    seed_offset: int = 0,
    map_name: str | None = None,
    render_mode: str | None = None,
) -> gym.Env:
    """one fully wrapped env driving a single rl agent, seeded off the run seed."""
    env_cfg = cfg["env"]
    seed = int(cfg["run"].get("seed", 0)) + int(seed_offset)
    map_name = map_name or _map_for_index(env_cfg, seed_offset)
    action_repeat = int(env_cfg.get("action_repeat", 1))
    residual_cfg = _residual_config(env_cfg)
    versus_cfg = _versus_config(env_cfg)

    inner = gym.make(
        "f1tenth_gym:f1tenth-v0",
        config=_env_config(
            env_cfg,
            seed,
            map_name,
            render=render_mode is not None,
            num_agents=1 if versus_cfg is None else 2,
        ),
        render_mode=render_mode,
    )
    track_length_m = float(inner.unwrapped.track.centerline.spline.s_frame_max)
    ego = (
        SingleAgentWrapper(inner)
        if versus_cfg is None
        else VersusEgoWrapper(inner, versus_cfg[0], versus_cfg[1])
    )
    flat = gym.wrappers.FlattenObservation(ego)

    norm = env_cfg.get("obs_norm", {})
    obs_cfg = ObsConfig(
        features=tuple(env_cfg.get("features", DEFAULT_FEATURES)),
        num_beams=int(env_cfg.get("num_beams", 108)),
        scan_range_max_m=float(env_cfg.get("scan_range_max_m", 30.0)),
        speed_norm_mps=float(norm.get("speed_mps", 8.0)),
        yaw_rate_norm_rps=float(norm.get("yaw_rate_rps", 5.0)),
        steer_norm_rad=float(norm.get("steer_rad", 0.4189)),
        ey_norm_m=float(norm.get("ey_m", 5.0)),
        track_length_m=track_length_m,
        control_hz=1.0 / (SIM_TIMESTEP_SEC * action_repeat),
        context_features=CONTEXT_FEATURES if residual_cfg is not None else (),
        ref_lateral_norm_m=float(norm.get("ref_lateral_m", 2.0)),
        curvature_norm_radpm=float(norm.get("curvature_radpm", 0.5)),
        curvature_horizons_m=tuple(
            (residual_cfg or {}).get("curvature_horizons_m", DEFAULT_CURVATURE_HORIZONS_M)
        ),
    )
    if obs_cfg.raw_dim != int(np.prod(flat.observation_space.shape)):
        raise ValueError(
            f"obs layout mismatch: ObsConfig says {obs_cfg.raw_dim} dims, env gives "
            f"{flat.observation_space.shape}"
        )
    action_bounds = ActionBounds(
        steer_max_rad=float(env_cfg.get("steer_max_rad", 0.4189)),
        speed_min_mps=float(env_cfg.get("speed_min_mps", 0.5)),
        speed_cap_mps=float(env_cfg.get("speed_cap_mps", 3.0)),
    )
    shared = dict(
        obs_cfg=obs_cfg,
        action_bounds=action_bounds,
        reward=ProgressReward.from_config(env_cfg.get("reward")),
        action_repeat=action_repeat,
        wrong_way_steps=int(env_cfg.get("wrong_way_steps", 0)),
    )
    if residual_cfg is None:
        env = F110RLWrapper(flat, **shared)
    else:
        planner = PurePursuitPlanner(
            _reference_line(inner.unwrapped.track, residual_cfg, map_name),
            PurePursuitConfig.from_dict(residual_cfg.get("pure_pursuit")),
        )
        env = ResidualPPWrapper(
            flat,
            planner=planner,
            residual_bounds=ResidualBounds.from_dict(
                {
                    key: residual_cfg[key]
                    for key in ("dsteer_max_rad", "dspeed_max_mps")
                    if key in residual_cfg
                }
            ),
            **shared,
        )
    if versus_cfg is not None:
        env = OvertakeBonus(env, versus_cfg[0].overtake_bonus, ego.overtake_margin_m)
    env.action_space.seed(seed)
    env.observation_space.seed(seed)
    return env


def _monitored(cfg: dict, seed_offset: int, map_name: str | None = None):
    def _init():
        return Monitor(build_env(cfg, seed_offset=seed_offset, map_name=map_name))

    return _init


def build_vec_env(cfg: dict, n_envs: int | None = None, seed_offset: int = 0):
    """subproc vec env with a distinct seed and map per worker."""
    env_cfg = cfg["env"]
    n_envs = int(n_envs if n_envs is not None else cfg["run"].get("n_envs", 8))
    # a shared EnvConfig.seed makes every worker replay the same rollout, so index it here
    makers = [
        _monitored(cfg, seed_offset + i, _map_for_index(env_cfg, seed_offset + i))
        for i in range(n_envs)
    ]
    if n_envs == 1:
        return DummyVecEnv(makers)
    return SubprocVecEnv(makers, start_method=cfg["run"].get("start_method"))


def build_eval_env(cfg: dict, seed_offset: int = 1000, n_envs: int | None = None):
    """eval env seeded away from the training workers, one worker per configured map."""
    env_cfg = cfg["env"]
    n_envs = int(n_envs if n_envs is not None else cfg["run"].get("eval_n_envs", 1))
    makers = [
        _monitored(cfg, seed_offset + i, _map_for_index(env_cfg, seed_offset + i))
        for i in range(n_envs)
    ]
    if n_envs == 1:
        return DummyVecEnv(makers)
    return SubprocVecEnv(makers, start_method=cfg["run"].get("start_method"))
