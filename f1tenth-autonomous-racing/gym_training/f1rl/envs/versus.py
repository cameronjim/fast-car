# head-to-head racing: the rl policy on agent_0, a scripted gap follower on agent_1

from __future__ import annotations

from dataclasses import dataclass, fields

import gymnasium as gym
import numpy as np

from .opponent import GapFollowerConfig, GapFollowerOpponent

EGO_INDEX = 0
OPPONENT_INDEX = 1

# the margin defaults to this much clear air past the car's own length
OVERTAKE_CLEARANCE_M = 1.0

# a car spawned inside the collision margin is halted where it stands and never drives out,
# so a draw that lands one there is redrawn rather than raced
SPAWN_CLEARANCE_SLACK_M = 0.05
MAX_SPAWN_DRAWS = 10


@dataclass(frozen=True)
class VersusConfig:
    """how the two cars are spawned, how fast the opponent may go, and what a pass is worth."""

    gap_min_m: float = 5.0
    gap_max_m: float = 15.0
    lateral_jitter_m: float = 0.4
    # the line both cars spawn on; the raceline is what the env's own random resets use
    spawn_line: str = "raceline"
    opponent_speed_min_mps: float = 3.0
    opponent_speed_max_mps: float = 4.5
    overtake_bonus: float = 20.0
    # None reads the car's length off the sim and adds OVERTAKE_CLEARANCE_M
    overtake_margin_m: float | None = None

    def __post_init__(self) -> None:
        if not 0.0 < self.gap_min_m <= self.gap_max_m:
            raise ValueError(
                f"spawn gap must satisfy 0 < gap_min_m <= gap_max_m, got "
                f"{self.gap_min_m} and {self.gap_max_m}"
            )
        if self.lateral_jitter_m < 0.0:
            raise ValueError(f"lateral_jitter_m must be >= 0, got {self.lateral_jitter_m}")
        if self.spawn_line not in ("raceline", "centerline"):
            raise ValueError(
                f"spawn_line must be 'raceline' or 'centerline', got {self.spawn_line!r}"
            )
        if not 0.0 < self.opponent_speed_min_mps <= self.opponent_speed_max_mps:
            raise ValueError(
                f"opponent speed range must satisfy 0 < min <= max, got "
                f"{self.opponent_speed_min_mps} and {self.opponent_speed_max_mps}"
            )
        if self.overtake_bonus < 0.0:
            raise ValueError(f"overtake_bonus must be >= 0, got {self.overtake_bonus}")
        if self.overtake_margin_m is not None and self.overtake_margin_m <= 0.0:
            raise ValueError(f"overtake_margin_m must be > 0, got {self.overtake_margin_m}")

    @classmethod
    def from_dict(cls, blob: dict | None) -> "VersusConfig":
        """build from a config mapping, rejecting keys that would otherwise be ignored."""
        blob = dict(blob or {})
        unknown = sorted(set(blob) - {field.name for field in fields(cls)})
        if unknown:
            raise ValueError(f"unknown versus config keys: {unknown}")
        return cls(**blob)

    def to_dict(self) -> dict:
        return {field.name: getattr(self, field.name) for field in fields(self)}


def advance_s(prev_s_m: float, s_m: float, track_length_m: float) -> float:
    """forward centerline arclength between two s samples, wraparound-safe."""
    delta = float(s_m) - float(prev_s_m)
    half = 0.5 * float(track_length_m)
    if delta < -half:
        delta += float(track_length_m)
    elif delta > half:
        delta -= float(track_length_m)
    return delta


def spawn_poses(
    track,
    opponent_s_m: float,
    gap_m: float,
    ego_ey_m: float,
    opponent_ey_m: float,
    use_raceline: bool = True,
):
    """ego one gap behind the opponent along the spawn line, both facing the track direction."""
    opponent = track.frenet_to_cartesian(
        float(opponent_s_m), float(opponent_ey_m), 0.0, use_raceline=use_raceline
    )
    ego = track.frenet_to_cartesian(
        float(opponent_s_m) - float(gap_m), float(ego_ey_m), 0.0, use_raceline=use_raceline
    )
    return np.array([ego, opponent], dtype=np.float64)


def spawn_is_clear(scans, clearance_m: float) -> bool:
    """true when every car was placed with room to drive away from where it stands."""
    return all(float(np.min(np.asarray(scan))) >= float(clearance_m) for scan in scans)


class VersusEgoWrapper(gym.Wrapper):
    """ego-only view of a two-agent env, with agent_1 driven by the scripted gap follower."""

    def __init__(self, env: gym.Env, config: VersusConfig, opponent_cfg: GapFollowerConfig):
        super().__init__(env)
        inner = env.unwrapped
        if inner.num_agents != 2:
            raise ValueError(f"VersusEgoWrapper needs num_agents == 2, got {inner.num_agents}")
        if inner.ego_idx != EGO_INDEX:
            raise ValueError(f"VersusEgoWrapper needs ego_index 0, got {inner.ego_idx}")
        # an opponent that crashes alone must not end the ego's episode
        if inner.terminate_on_collision and inner.collision_agents != "ego":
            raise ValueError(
                f"versus mode needs collision_agents 'ego', got {inner.collision_agents!r}"
            )

        self.config = config
        self._ego_id, self._opponent_id = inner.agent_ids[EGO_INDEX], inner.agent_ids[OPPONENT_INDEX]
        self.observation_space = env.observation_space[self._ego_id]
        act = env.action_space
        self.action_space = gym.spaces.Box(
            low=np.asarray(act.low[EGO_INDEX]), high=np.asarray(act.high[EGO_INDEX]), dtype=act.dtype
        )

        self.opponent = GapFollowerOpponent(
            angle_min_rad=inner.lidar_cfg.angle_min,
            angle_increment_rad=inner.lidar_cfg.angle_increment,
            control_period_sec=float(inner.timestep),
            config=opponent_cfg,
        )
        self.overtake_margin_m = (
            float(config.overtake_margin_m)
            if config.overtake_margin_m is not None
            else float(inner.sim.vehicle_params.length) + OVERTAKE_CLEARANCE_M
        )
        self.min_spawn_clearance_m = (
            0.5 * float(inner.sim.vehicle_params.width) + SPAWN_CLEARANCE_SLACK_M
        )
        self.track_length_m = float(inner.track.centerline.spline.s_frame_max)
        self._use_raceline = config.spawn_line == "raceline"
        spawn_spline = (inner.track.raceline if self._use_raceline else inner.track.centerline).spline
        self._spawn_length_m = float(spawn_spline.s_frame_max)

        # sb3 resets without a seed after the first episode, so the spawn stream starts from the config seed
        self._rng = np.random.default_rng(inner.seed)
        self._opponent_scan = None
        self._prev_s = np.zeros(2)
        self._gap_m = 0.0
        self._opponent_distance_m = 0.0
        self._opponent_speed_cap_mps = 0.0

    def _sample_poses(self) -> np.ndarray:
        cfg = self.config
        jitter = cfg.lateral_jitter_m
        return spawn_poses(
            self.env.unwrapped.track,
            opponent_s_m=self._rng.uniform(0.0, self._spawn_length_m),
            gap_m=self._rng.uniform(cfg.gap_min_m, cfg.gap_max_m),
            ego_ey_m=self._rng.uniform(-jitter, jitter),
            opponent_ey_m=self._rng.uniform(-jitter, jitter),
            use_raceline=self._use_raceline,
        )

    def _reset_on_clear_spawn(self, seed, options: dict):
        """reset, redrawing the spawn while either car lands inside the collision margin."""
        if "poses" in options:
            return self.env.reset(seed=seed, options=options)
        for _ in range(MAX_SPAWN_DRAWS):
            obs, info = self.env.reset(
                seed=seed, options={**options, "poses": self._sample_poses()}
            )
            scans = [obs[agent_id]["scan"] for agent_id in (self._ego_id, self._opponent_id)]
            if spawn_is_clear(scans, self.min_spawn_clearance_m):
                return obs, info
        raise RuntimeError(
            f"no clear two-car spawn in {MAX_SPAWN_DRAWS} draws; the jitter or the gap range "
            f"does not fit this track"
        )

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        options = dict(options or {})
        obs, info = self._reset_on_clear_spawn(seed, options)

        cfg = self.config
        self._opponent_speed_cap_mps = float(
            self._rng.uniform(cfg.opponent_speed_min_mps, cfg.opponent_speed_max_mps)
        )
        self.opponent.reset()
        self.opponent.set_speed_cap(self._opponent_speed_cap_mps)

        frenet = np.asarray(self.env.unwrapped.sim.state.frenet)
        self._prev_s = frenet[:, 0].astype(float).copy()
        # measured from the spawn poses rather than the sampled gap, so the jitter is included
        self._gap_m = advance_s(self._prev_s[OPPONENT_INDEX], self._prev_s[EGO_INDEX], self.track_length_m)
        poses = np.asarray(self.env.unwrapped.sim.state.poses)
        self._opponent_distance_m = float(
            np.linalg.norm(poses[EGO_INDEX][:2] - poses[OPPONENT_INDEX][:2])
        )
        self._opponent_scan = np.asarray(obs[self._opponent_id]["scan"], dtype=float)
        return obs[self._ego_id], self._race_info(info, opponent_collision=False)

    def step(self, action):
        steering, speed = self.opponent.plan(self._opponent_scan)
        ego = np.asarray(action, dtype=np.float32).reshape(2)
        combined = np.array([ego, [steering, speed]], dtype=np.float32)

        obs, reward, terminated, truncated, info = self.env.step(combined)

        self._opponent_scan = np.asarray(obs[self._opponent_id]["scan"], dtype=float)
        frenet = np.asarray(self.env.unwrapped.sim.state.frenet)[:, 0].astype(float)
        self._gap_m += advance_s(
            self._prev_s[EGO_INDEX], frenet[EGO_INDEX], self.track_length_m
        ) - advance_s(self._prev_s[OPPONENT_INDEX], frenet[OPPONENT_INDEX], self.track_length_m)
        self._prev_s = frenet.copy()

        opponent_collision = bool(np.asarray(info["collisions"]).reshape(-1)[OPPONENT_INDEX] > 0)
        poses = np.asarray(self.env.unwrapped.sim.state.poses)
        self._opponent_distance_m = float(
            np.linalg.norm(poses[EGO_INDEX][:2] - poses[OPPONENT_INDEX][:2])
        )
        return (
            obs[self._ego_id],
            reward,
            terminated,
            truncated,
            self._race_info(info, opponent_collision),
        )

    def _race_info(self, info: dict, opponent_collision: bool) -> dict:
        info = dict(info)
        info.update(
            {
                # signed unwrapped centerline lead of the ego over the opponent
                "gap_m": self._gap_m,
                "opponent_distance_m": self._opponent_distance_m,
                "opponent_collision": opponent_collision,
                "opponent_speed_cap_mps": self._opponent_speed_cap_mps,
            }
        )
        return info


class OvertakeBonus(gym.Wrapper):
    """one-shot reward the first time the ego leads the opponent by the overtake margin."""

    def __init__(self, env: gym.Env, bonus: float, margin_m: float):
        super().__init__(env)
        if margin_m <= 0.0:
            raise ValueError(f"margin_m must be > 0, got {margin_m}")
        self.bonus = float(bonus)
        self.margin_m = float(margin_m)
        self._overtaken = False

    @property
    def action_bounds(self):
        return self.env.action_bounds

    def set_speed_cap(self, speed_cap_mps: float) -> None:
        self.env.set_speed_cap(speed_cap_mps)

    def reset(self, *, seed=None, options=None):
        obs, info = self.env.reset(seed=seed, options=options)
        self._overtaken = False
        info["overtaken"] = False
        return obs, info

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        # a level test, not an edge: an action repeat that skips the crossing step still fires
        if not self._overtaken and float(info["gap_m"]) >= self.margin_m:
            self._overtaken = True
            reward += self.bonus
            info["overtake_time_sec"] = float(info["sim_time"])
        info["overtaken"] = self._overtaken
        return obs, reward, terminated, truncated, info
