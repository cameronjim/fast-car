"""Low-level open-loop env stepping shared by every maneuver (claude-docs/12-testing.md L5,
roadmap S.6).

Deliberately gym/racer_gym-specific -- unlike tests/sim_in_loop/racer_sim_in_loop/runner.py's
gym-free ``EnvLike`` protocol -- because every maneuver here needs the raw 7-element
dynamics state (``env.unwrapped.sim.agents[0].state``), not the wrapped observation dict
that runner records. This module is the one place that knows how to build a racer_gym env
on the shared battery track (track.py) and step it with an open-loop command sequence.

State vector layout (sim/racer_gym/racer_gym/dynamics/model.py's module docstring):
``[x, y, delta, v, yaw, yaw_rate, slip_angle]`` -- the same 7-element state upstream
f1tenth_gym's ``DynamicModel.ST`` uses.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
import racer_gym
from racer_gym.params import DynParamsResult

from .track import build_battery_track

STATE_X = 0
STATE_Y = 1
STATE_DELTA = 2
STATE_V = 3
STATE_YAW = 4
STATE_YAW_RATE = 5
STATE_SLIP = 6

Record = dict[str, float]
# step index (0-based) -> (steer_cmd_rad, speed_cmd_mps)
CommandFn = Callable[[int], tuple[float, float]]


@dataclass(frozen=True)
class OpenLoopRun:
    dt_s: float
    seed: int
    states: np.ndarray  # shape (num_steps, 7); row i is the state AFTER step i
    dyn_params_result: DynParamsResult


def run_open_loop(
    *,
    seed: int,
    num_steps: int,
    command_fn: CommandFn,
    vehicle_params=None,
) -> OpenLoopRun:
    """Step a fresh racer_gym env ``num_steps`` times with ``command_fn``, recording every
    post-step state.

    No termination handling: the battery track (track.py) is deliberately long, straight,
    and wide enough that none of this module's maneuvers should ever trigger a wall
    collision or lap-progress truncation within their step budgets. A termination flag here
    is therefore a maneuver-design bug (a command sequence that ran the car off the track),
    not a normal outcome to swallow -- it raises loudly instead of silently returning a
    truncated golden trajectory that would look like a shorter, unrelated maneuver.
    """
    if num_steps <= 0:
        raise ValueError(f"num_steps must be positive, got {num_steps}")

    env = racer_gym.build_env(
        config={
            "map": build_battery_track(),
            "num_agents": 1,
            "seed": seed,
            "observation_config": {"type": "dynamic_state"},
        },
        vehicle_params=vehicle_params,
    )
    try:
        env.reset(seed=seed)
        dt_s = float(env.unwrapped.timestep)
        states = np.empty((num_steps, 7), dtype=np.float64)
        for i in range(num_steps):
            steer_cmd, speed_cmd = command_fn(i)
            action = np.array([[steer_cmd, speed_cmd]], dtype=np.float64)
            _obs, _reward, terminated, truncated, info = env.step(action)
            states[i] = np.asarray(env.unwrapped.sim.agents[0].state, dtype=np.float64)
            if terminated or truncated:
                raise RuntimeError(
                    f"battery maneuver terminated/truncated at step {i} "
                    f"(terminated={terminated}, truncated={truncated}, info={info!r}); "
                    "the shared battery track (track.py) is sized to make this impossible "
                    "for every maneuver in this package -- widen the track or shorten/retune "
                    "the offending maneuver rather than swallowing this."
                )
        return OpenLoopRun(
            dt_s=dt_s, seed=seed, states=states, dyn_params_result=env.racer_dyn_params_result
        )
    finally:
        env.close()


def sample_trajectory(run: OpenLoopRun, *, stride: int) -> list[Record]:
    """Decimate a run's full state history into a compact list of golden-comparable
    records, one every ``stride`` steps plus always the final step.

    ``t_s`` is ``(step_index + 1) * dt_s`` -- plain double multiplication of an exact
    literal, bit-identical on every IEEE-754-conforming platform (see tolerances.py).
    """
    if stride <= 0:
        raise ValueError(f"stride must be positive, got {stride}")

    def _record(i: int) -> Record:
        x, y, delta, v, yaw, yaw_rate, slip = run.states[i]
        return {
            "t_s": (i + 1) * run.dt_s,
            "x_m": float(x),
            "y_m": float(y),
            "yaw_rad": float(yaw),
            "speed_mps": float(v),
            "yaw_rate_radps": float(yaw_rate),
            "slip_angle_rad": float(slip),
            "steer_angle_rad": float(delta),
        }

    num_steps = run.states.shape[0]
    indices = list(range(0, num_steps, stride))
    last = num_steps - 1
    if indices[-1] != last:
        indices.append(last)
    return [_record(i) for i in indices]
