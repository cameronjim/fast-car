"""L1 unit tests for racer_sim_in_loop.runner, against a trivial fake env.

No gym, no rendering, no ROS -- proves the runner's own plumbing (env
lifecycle, seeding pass-through, step recording, early termination,
guaranteed close()) independent of any real simulator. See
test_runner_gym.py for the same plumbing exercised against the real,
pinned f1tenth_gym.
"""

from __future__ import annotations

import pytest
from racer_sim_in_loop.runner import (
    TrajectoryRecord,
    constant_command_controller,
    run_scenario,
)


class FakeEnv:
    """A minimal EnvLike: moves in a straight line and stops after N steps."""

    def __init__(self, terminate_after: int | None = None):
        self._terminate_after = terminate_after
        self._step_count = 0
        self.reset_calls: list[dict] = []
        self.closed = False

    def reset(self, *, seed=None, options=None):
        self._step_count = 0
        self.reset_calls.append({"seed": seed, "options": options})
        return {"x": 0.0, "y": 0.0}, {"reset_seed": seed}

    def step(self, action):
        self._step_count += 1
        obs = {"x": float(self._step_count), "y": 0.0}
        terminated = self._terminate_after is not None and self._step_count >= self._terminate_after
        info = {"step_count": self._step_count}
        return obs, 1.0, terminated, False, info

    def close(self):
        self.closed = True


class RaisingEnv:
    """An env whose step() always raises, to test close() is still called."""

    def __init__(self):
        self.closed = False

    def reset(self, *, seed=None, options=None):
        return {}, {}

    def step(self, action):
        raise RuntimeError("boom")

    def close(self):
        self.closed = True


class TestRunScenario:
    def test_runs_to_max_steps_when_never_terminated(self):
        env = FakeEnv()
        record = run_scenario(lambda: env, constant_command_controller("go"), seed=42, max_steps=5)
        assert isinstance(record, TrajectoryRecord)
        assert record.step_count == 5
        assert not record.terminated
        assert not record.truncated

    def test_stops_early_on_termination(self):
        env = FakeEnv(terminate_after=3)
        record = run_scenario(lambda: env, constant_command_controller("go"), seed=0, max_steps=100)
        assert record.step_count == 3
        assert record.terminated

    def test_seed_is_passed_to_reset(self):
        env = FakeEnv()
        run_scenario(lambda: env, constant_command_controller(0), seed=7, max_steps=1)
        assert env.reset_calls == [{"seed": 7, "options": None}]

    def test_reset_options_are_passed_through(self):
        env = FakeEnv()
        run_scenario(
            lambda: env,
            constant_command_controller(0),
            seed=1,
            max_steps=1,
            reset_options={"track": "gym_a"},
        )
        assert env.reset_calls[0]["options"] == {"track": "gym_a"}

    def test_records_action_and_reward_per_step(self):
        env = FakeEnv()
        record = run_scenario(
            lambda: env, constant_command_controller("cmd"), seed=None, max_steps=2
        )
        assert [s.action for s in record.steps] == ["cmd", "cmd"]
        assert [s.reward for s in record.steps] == [1.0, 1.0]
        assert [s.index for s in record.steps] == [0, 1]

    def test_env_is_always_closed(self):
        env = FakeEnv()
        run_scenario(lambda: env, constant_command_controller(0), seed=0, max_steps=1)
        assert env.closed

    def test_env_is_closed_even_if_step_raises(self):
        env = RaisingEnv()
        with pytest.raises(RuntimeError):
            run_scenario(lambda: env, constant_command_controller(0), seed=0, max_steps=1)
        assert env.closed

    def test_rejects_nonpositive_max_steps(self):
        env = FakeEnv()
        with pytest.raises(ValueError):
            run_scenario(lambda: env, constant_command_controller(0), seed=0, max_steps=0)

    def test_observations_and_infos_helpers(self):
        env = FakeEnv()
        record = run_scenario(lambda: env, constant_command_controller(0), seed=0, max_steps=2)
        assert record.observations() == [{"x": 1.0, "y": 0.0}, {"x": 2.0, "y": 0.0}]
        assert record.infos() == [{"step_count": 1}, {"step_count": 2}]


class TestConstantCommandController:
    def test_returns_the_same_action_regardless_of_observation(self):
        controller = constant_command_controller((0.1, 2.0))
        assert controller({"anything": 1}) == (0.1, 2.0)
        assert controller(None) == (0.1, 2.0)
