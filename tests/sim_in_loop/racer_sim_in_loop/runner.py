"""Headless, seeded scenario runner (claude-docs/12-testing.md L5).

Deliberately gym-free: :func:`run_scenario` is written against the small
:class:`EnvLike` protocol below (whatever ``gymnasium.Env`` subset
f1tenth_gym's ``reset``/``step``/``close`` actually need), not against
f1tenth_gym directly. That keeps this module's own plumbing testable with a
trivial in-repo fake env, with no gym install and no rendering stack, while
still being exactly what a real ``gym.make("f1tenth_gym:f1tenth-v0", ...)``
env satisfies (see ``tests/test_runner_gym.py`` for that proof against the
pinned commit).

No reference trajectories and no real controller live here (roadmap task
0.9 is scaffold only): the controller passed in is any
``Callable[[observation], action]``, and callers own building both the env
and the controller. S.2 (tracker lap test) and S.6 (dynamics regression
battery) are expected to import this module and supply their own env
factory / controller / reference data.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, Protocol


class EnvLike(Protocol):
    """The subset of the gymnasium ``Env`` API this runner needs."""

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[Any, dict[str, Any]]: ...

    def step(self, action: Any) -> tuple[Any, float, bool, bool, dict[str, Any]]: ...

    def close(self) -> None: ...


ControllerLike = Callable[[Any], Any]
EnvFactory = Callable[[], EnvLike]


@dataclass(frozen=True)
class TrajectoryStep:
    index: int
    obs: Any
    action: Any
    reward: float
    terminated: bool
    truncated: bool
    info: dict[str, Any]


@dataclass(frozen=True)
class TrajectoryRecord:
    """The full recorded output of one :func:`run_scenario` call."""

    seed: int | None
    steps: list[TrajectoryStep] = field(default_factory=list)
    terminated: bool = False
    truncated: bool = False

    @property
    def step_count(self) -> int:
        return len(self.steps)

    def observations(self) -> list[Any]:
        return [s.obs for s in self.steps]

    def infos(self) -> list[dict[str, Any]]:
        return [s.info for s in self.steps]


def run_scenario(
    env_factory: EnvFactory,
    controller: ControllerLike,
    *,
    seed: int | None,
    max_steps: int,
    reset_options: dict[str, Any] | None = None,
) -> TrajectoryRecord:
    """Run one headless, seeded episode and record every step.

    ``env_factory`` is called exactly once and the resulting env is always
    ``close()``-d, even if the controller or a step raises -- a leaked
    render context/process is exactly the kind of thing that makes a CI
    sim-in-loop job flaky.
    """
    if max_steps <= 0:
        raise ValueError(f"max_steps must be positive, got {max_steps}")

    env = env_factory()
    try:
        obs, _reset_info = env.reset(seed=seed, options=reset_options)
        steps: list[TrajectoryStep] = []
        terminated = False
        truncated = False
        for i in range(max_steps):
            action = controller(obs)
            obs, reward, terminated, truncated, info = env.step(action)
            steps.append(
                TrajectoryStep(
                    index=i,
                    obs=obs,
                    action=action,
                    reward=reward,
                    terminated=terminated,
                    truncated=truncated,
                    info=info,
                )
            )
            if terminated or truncated:
                break
        return TrajectoryRecord(seed=seed, steps=steps, terminated=terminated, truncated=truncated)
    finally:
        env.close()


def constant_command_controller(action: Any) -> ControllerLike:
    """A trivial controller that always returns the same fixed action.

    Used only to prove the runner's plumbing end to end (this task's
    brief: "a trivial constant-command controller in the runner's own test
    proves the plumbing end to end"). Not a real tracker -- S.2 replaces
    this with the actual raceline-tracking controller.
    """

    def _controller(_obs: Any) -> Any:
        return action

    return _controller
