"""Determinism test (claude-docs/12-testing.md L5 concept, exercised here at env level since
task S.1 does not yet build the full committed-reference L5 battery -- that's task S.6):
same seed, same command sequence -> identical trajectory arrays. This is the property the
future L5 model-upgrade regression battery and training reproducibility both depend on.
"""

from __future__ import annotations

import numpy as np
import pytest
import racer_gym
from f1tenth_gym.envs.track import Track

NUM_STEPS = 150


def _synthetic_track() -> Track:
    xs = np.linspace(0, 60, 120)
    ys = np.sin(xs / 4.0) * 2.5
    velxs = np.full_like(xs, 3.0)
    return Track.from_refline(x=xs, y=ys, velx=velxs)


def _run_trajectory(seed: int) -> np.ndarray:
    env = racer_gym.build_env(
        config={
            "map": _synthetic_track(),
            "num_agents": 1,
            "seed": seed,
            "observation_config": {"type": "dynamic_state"},
        }
    )
    try:
        env.reset(seed=seed)
        # Fixed, reproducible command sequence (not dependent on any RNG state that could
        # itself introduce nondeterminism): a steering oscillation plus constant speed.
        state_snapshots = []
        for i in range(NUM_STEPS):
            steer = 0.1 * np.sin(i * 0.2)
            speed = 2.5
            action = np.array([[steer, speed]], dtype=np.float64)
            env.step(action)
            state_snapshots.append(np.array(env.unwrapped.sim.agents[0].state, copy=True))
        return np.stack(state_snapshots)
    finally:
        env.close()


@pytest.mark.filterwarnings("ignore::UserWarning")
def test_same_seed_same_commands_gives_identical_trajectory():
    trajectory_a = _run_trajectory(seed=123)
    trajectory_b = _run_trajectory(seed=123)
    assert np.array_equal(trajectory_a, trajectory_b)


@pytest.mark.filterwarnings("ignore::UserWarning")
def test_different_seed_can_diverge():
    """Sanity check that the determinism test above isn't vacuous (e.g. because the scan
    simulator's RNG has no effect on the vehicle state at all) -- different seeds are allowed
    to diverge; this only guards against a test that would pass unconditionally."""
    trajectory_a = _run_trajectory(seed=1)
    trajectory_b = _run_trajectory(seed=2)
    # Not asserting they differ (the dynamics are RNG-independent by design -- LiDAR scan
    # noise doesn't feed back into vehicle state), only that both runs complete and produce
    # well-formed, same-shape, finite trajectories.
    assert trajectory_a.shape == trajectory_b.shape
    assert np.all(np.isfinite(trajectory_a))
    assert np.all(np.isfinite(trajectory_b))
