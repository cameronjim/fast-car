# eval-gated speed-cap curriculum and the racing-metric eval callback that feeds it

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback, EvalCallback

# overtakes can move a score by at most this, so they rank above lap time and well below
# the collision-free rate
OVERTAKE_WEIGHT = 20.0


@dataclass
class EvalRound:
    """what one evaluation round says about the policy at the cap it ran at."""

    speed_cap_mps: float
    episodes: int
    collision_free_rate: float
    laps: int
    lap_times_sec: list[float] = field(default_factory=list)
    # zero outside head-to-head runs, where no episode reports an overtake
    overtake_rate: float = 0.0

    @property
    def best_lap_time_sec(self) -> float | None:
        return min(self.lap_times_sec) if self.lap_times_sec else None

    @property
    def mean_lap_time_sec(self) -> float | None:
        return float(np.mean(self.lap_times_sec)) if self.lap_times_sec else None


@dataclass
class SpeedCapSchedule:
    """decides when the speed cap has earned a promotion, from eval rounds alone."""

    start_cap_mps: float = 4.5
    max_cap_mps: float = 8.0
    step_mps: float = 0.5
    promote_rate: float = 0.9
    window_rounds: int = 2
    # cooldown >= window keeps every round in the deciding window at the current cap
    cooldown_rounds: int = 2

    cap_mps: float = field(init=False)
    rates: list[float] = field(init=False, default_factory=list)
    rounds_since_promotion: int = field(init=False, default=0)

    def __post_init__(self) -> None:
        if self.step_mps <= 0.0:
            raise ValueError(f"step_mps must be > 0, got {self.step_mps}")
        if self.max_cap_mps < self.start_cap_mps:
            raise ValueError(
                f"max_cap_mps ({self.max_cap_mps}) is below start_cap_mps ({self.start_cap_mps})"
            )
        if not 0.0 <= self.promote_rate <= 1.0:
            raise ValueError(f"promote_rate must be in [0, 1], got {self.promote_rate}")
        if self.window_rounds < 1:
            raise ValueError(f"window_rounds must be >= 1, got {self.window_rounds}")
        if self.cooldown_rounds < 0:
            raise ValueError(f"cooldown_rounds must be >= 0, got {self.cooldown_rounds}")
        self.cap_mps = float(self.start_cap_mps)

    @property
    def at_target(self) -> bool:
        return self.cap_mps >= self.max_cap_mps - 1e-9

    def record(self, collision_free_rate: float) -> float | None:
        """log one eval round, returning the new cap when the round earns a promotion."""
        self.rates.append(float(collision_free_rate))
        self.rounds_since_promotion += 1
        if self.at_target:
            return None
        if self.rounds_since_promotion < self.cooldown_rounds:
            return None
        window = self.rates[-self.window_rounds :]
        if len(window) < self.window_rounds or min(window) < self.promote_rate:
            return None
        self.cap_mps = min(self.cap_mps + self.step_mps, self.max_cap_mps)
        self.rounds_since_promotion = 0
        return self.cap_mps

    @classmethod
    def from_config(cls, cfg: dict | None) -> "SpeedCapSchedule":
        cfg = cfg or {}
        tunable = {name for name, spec in cls.__dataclass_fields__.items() if spec.init}
        unknown = set(cfg) - tunable
        if unknown:
            raise ValueError(f"unknown curriculum keys in config: {sorted(unknown)}")
        return cls(**cfg)


def racing_score(round_: EvalRound) -> float:
    """orders eval rounds by reliability first, then overtakes, then how fast the clean laps were."""
    score = round_.collision_free_rate * 100.0 + round_.overtake_rate * OVERTAKE_WEIGHT
    if round_.best_lap_time_sec is None:
        return score
    # the lap term stays under 10, so no lap time can outrank a single lost episode
    return score + 300.0 / round_.best_lap_time_sec


class RacingEvalCallback(EvalCallback):
    """eval callback that scores rounds on collision-free rate and lap time, not reward."""

    def __init__(
        self,
        *args,
        racing_best_path: str | None = None,
        speed_cap_mps: float = 0.0,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.racing_best_path = racing_best_path
        self.speed_cap_mps = float(speed_cap_mps)
        self.rounds: list[EvalRound] = []
        self.best_racing_score = -np.inf
        self._lap_times: dict[int, list[float]] = {}
        self._episode_clean: list[bool] = []
        self._episode_laps: list[int] = []
        self._episode_overtakes: list[bool] = []
        self._round_lap_times: list[float] = []

    def _log_success_callback(self, locals_, globals_) -> None:
        super()._log_success_callback(locals_, globals_)
        index = int(locals_["i"])
        info = locals_["info"]
        if "lap_time_sec" in info:
            self._lap_times.setdefault(index, []).append(float(info["lap_time_sec"]))
        if not locals_["done"]:
            return
        self._episode_clean.append(not bool(info.get("is_collision", False)))
        self._episode_laps.append(int(info.get("lap_count", 0)))
        self._episode_overtakes.append(bool(info.get("overtaken", False)))
        self._round_lap_times.extend(self._lap_times.pop(index, []))

    def _on_step(self) -> bool:
        evaluating = self.eval_freq > 0 and self.n_calls % self.eval_freq == 0
        if evaluating:
            self._lap_times.clear()
            self._episode_clean.clear()
            self._episode_laps.clear()
            self._episode_overtakes.clear()
            self._round_lap_times.clear()
        keep_training = super()._on_step()
        # with a child attached the round is closed in _on_event, before the child reads it
        if evaluating and self.callback is None:
            self._finish_round()
            self.logger.dump(self.num_timesteps)
        return keep_training

    def _on_event(self) -> bool:
        self._finish_round()
        return super()._on_event()

    def _finish_round(self) -> None:
        episodes = len(self._episode_clean)
        if episodes == 0:
            return
        finished = EvalRound(
            speed_cap_mps=self.speed_cap_mps,
            episodes=episodes,
            collision_free_rate=float(np.mean(self._episode_clean)),
            laps=int(np.sum(self._episode_laps)),
            lap_times_sec=list(self._round_lap_times),
            overtake_rate=float(np.mean(self._episode_overtakes)),
        )
        self.rounds.append(finished)
        score = racing_score(finished)
        self.logger.record("eval/collision_free_rate", finished.collision_free_rate)
        self.logger.record("eval/laps_per_episode", finished.laps / episodes)
        self.logger.record("eval/racing_score", score)
        if finished.overtake_rate:
            self.logger.record("eval/overtake_rate", finished.overtake_rate)
        if finished.best_lap_time_sec is not None:
            self.logger.record("eval/best_lap_time_sec", finished.best_lap_time_sec)
            self.logger.record("eval/mean_lap_time_sec", finished.mean_lap_time_sec)
        lap = (
            f"best lap {finished.best_lap_time_sec:.2f}s"
            if finished.best_lap_time_sec is not None
            else "no lap"
        )
        overtakes = f", overtake {finished.overtake_rate:.0%}" if finished.overtake_rate else ""
        print(
            f"eval at {self.num_timesteps} steps, cap {finished.speed_cap_mps:.2f} m/s: "
            f"clean {finished.collision_free_rate:.0%} over {episodes} episodes, "
            f"{finished.laps} laps, {lap}{overtakes}"
        )
        if self.racing_best_path is not None and score > self.best_racing_score:
            self.best_racing_score = score
            self.model.save(self.racing_best_path)
            print(f"new best racing policy at score {score:.2f}, saved to {self.racing_best_path}")


class SpeedCapCurriculum(BaseCallback):
    """applies the schedule's cap to every training sub-env and to the eval env."""

    def __init__(self, schedule: SpeedCapSchedule, eval_env=None, verbose: int = 1):
        super().__init__(verbose=verbose)
        self.schedule = schedule
        self.eval_env = eval_env

    def _init_callback(self) -> None:
        if self.eval_env is None and self.parent is not None:
            self.eval_env = self.parent.eval_env
        self._apply(self.schedule.cap_mps)
        print(f"curriculum starting at a {self.schedule.cap_mps:.2f} m/s speed cap")

    def _apply(self, cap_mps: float) -> None:
        self.training_env.env_method("set_speed_cap", float(cap_mps))
        if self.eval_env is not None:
            self.eval_env.env_method("set_speed_cap", float(cap_mps))
        if isinstance(self.parent, RacingEvalCallback):
            self.parent.speed_cap_mps = float(cap_mps)

    def _on_step(self) -> bool:
        # the parent eval callback fires this once per evaluation round, after closing it
        rounds = getattr(self.parent, "rounds", None)
        if not rounds:
            return True
        promoted = self.schedule.record(rounds[-1].collision_free_rate)
        if promoted is not None:
            self._apply(promoted)
            print(f"speed cap promoted to {promoted:.2f} m/s at {self.num_timesteps} steps")
        self.logger.record("curriculum/speed_cap_mps", self.schedule.cap_mps)
        self.logger.dump(self.num_timesteps)
        return True
