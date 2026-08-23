"""Unit tests for scenarios.sample_trajectory's decimation logic -- pure, no gym/racer_gym
needed (a hand-built OpenLoopRun stands in for a real one)."""

from __future__ import annotations

import numpy as np
import pytest
from racer_sim_regression.scenarios import OpenLoopRun, sample_trajectory


def _run(num_steps: int, dt_s: float = 0.01) -> OpenLoopRun:
    # State i is just [i, i, i, i, i, i, i] * 1.0 so each field is trivially checkable.
    states = np.tile(np.arange(num_steps, dtype=np.float64).reshape(-1, 1), (1, 7))
    return OpenLoopRun(dt_s=dt_s, seed=0, states=states, dyn_params_result=None)


class TestSampleTrajectory:
    def test_rejects_non_positive_stride(self):
        with pytest.raises(ValueError, match="stride"):
            sample_trajectory(_run(10), stride=0)

    def test_every_record_has_the_expected_fields(self):
        records = sample_trajectory(_run(5), stride=2)
        expected_fields = {
            "t_s",
            "x_m",
            "y_m",
            "yaw_rad",
            "speed_mps",
            "yaw_rate_radps",
            "slip_angle_rad",
            "steer_angle_rad",
        }
        for record in records:
            assert set(record) == expected_fields

    def test_stride_selects_expected_indices_plus_final(self):
        run = _run(10)
        records = sample_trajectory(run, stride=3)
        # indices 0, 3, 6, 9 (9 is both a stride hit and the final index)
        sampled_x = [r["x_m"] for r in records]
        assert sampled_x == [0.0, 3.0, 6.0, 9.0]

    def test_final_step_always_included_even_off_stride(self):
        run = _run(8)  # last index = 7, not a multiple of stride=3 (0, 3, 6, then +7)
        records = sample_trajectory(run, stride=3)
        sampled_x = [r["x_m"] for r in records]
        assert sampled_x == [0.0, 3.0, 6.0, 7.0]

    def test_t_s_is_one_indexed_step_times_dt(self):
        run = _run(3, dt_s=0.01)
        records = sample_trajectory(run, stride=1)
        assert [r["t_s"] for r in records] == [
            pytest.approx(0.01),
            pytest.approx(0.02),
            pytest.approx(0.03),
        ]

    def test_single_step_run(self):
        records = sample_trajectory(_run(1), stride=5)
        assert len(records) == 1
        assert records[0]["x_m"] == 0.0
