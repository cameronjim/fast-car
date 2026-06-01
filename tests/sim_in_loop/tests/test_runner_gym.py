"""L5 plumbing proof against the REAL, pinned f1tenth_gym.

Per roadmap task 0.9: "a trivial constant-command controller in the
runner's own test proves the plumbing end to end against the pinned
f1tenth_gym". No reference trajectories, no real controller, no wiring
into racer_gym (sim/racer_gym/, task S.1 -- doesn't exist yet): this is
exactly the sim-cpu smoke test's headless-construction pattern
(docker/sim-cpu/smoke_test.py), reused here to prove run_scenario() itself
works end to end against a real gymnasium env, not just the FakeEnv in
test_runner_fake_env.py.

f1tenth_gym is deliberately NOT a dependency of this package's
pyproject.toml (see its module docstring) -- this test skips cleanly via
pytest.importorskip when f1tenth_gym/gymnasium are not installed (e.g. the
repo's bare CI jobs), and runs for real only when they are (this task's
local verification, and any future job that installs the pinned commit,
see docker/sim-cpu/pyproject.toml for the exact pin).
"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("gymnasium")
pytest.importorskip("f1tenth_gym")

from racer_sim_in_loop.runner import run_scenario


def _build_env():
    """A headless f1tenth_gym env on a synthetic reference line.

    Mirrors docker/sim-cpu/smoke_test.py's build_env(): Track.from_refline
    rather than a named map, so this needs no network access and is fully
    reproducible from the pinned commit alone.
    """
    import gymnasium as gym
    from f1tenth_gym.envs.track import Track

    xs = np.linspace(0, 50, 100)
    ys = np.sin(xs / 3.0) * 3.0
    velxs = np.full_like(xs, 3.0)
    track = Track.from_refline(x=xs, y=ys, velx=velxs)

    return gym.make(
        "f1tenth_gym:f1tenth-v0",
        config={
            "map": track,
            "num_agents": 1,
            "observation_config": {"type": "kinematic_state"},
        },
        render_mode=None,
    )


def _constant_action(_obs) -> np.ndarray:
    # [steering_angle_rad, speed_mps] for one ego agent -- see
    # sim/bridge/racer_gym_bridge/racer_gym_bridge/conversions.py
    # drive_cmd_to_action for the empirical column-order note.
    return np.array([[0.02, 1.5]], dtype=np.float64)


class TestRunScenarioAgainstRealGym:
    def test_plumbing_runs_end_to_end(self):
        record = run_scenario(_build_env, _constant_action, seed=0, max_steps=50)
        assert record.step_count > 0
        assert record.step_count <= 50
        # Every recorded observation must be finite -- same invariant the
        # sim-cpu smoke test checks; a non-finite obs means the dynamics
        # pipeline broke, independent of anything runner.py does.
        for obs in record.observations():
            agent_obs = obs["agent_0"]
            for value in agent_obs.values():
                assert np.all(np.isfinite(np.asarray(value, dtype=np.float64)))

    def test_same_seed_is_deterministic(self):
        record_a = run_scenario(_build_env, _constant_action, seed=123, max_steps=20)
        record_b = run_scenario(_build_env, _constant_action, seed=123, max_steps=20)
        obs_a = record_a.observations()[-1]["agent_0"]
        obs_b = record_b.observations()[-1]["agent_0"]
        for key in obs_a:
            np.testing.assert_allclose(
                np.asarray(obs_a[key], dtype=np.float64),
                np.asarray(obs_b[key], dtype=np.float64),
            )
