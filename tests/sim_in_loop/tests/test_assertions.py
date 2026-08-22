"""L1 unit tests for racer_sim_in_loop.assertions, against synthetic trajectories."""

from __future__ import annotations

import pytest
from racer_sim_in_loop.assertions import (
    assert_lap_completed,
    assert_lap_time_in_band,
    assert_no_wall_contact,
    assert_trajectory_matches_reference,
    lap_time_seconds,
)
from racer_sim_in_loop.runner import TrajectoryRecord, TrajectoryStep


def _record(n: int, *, terminated: bool = False, infos: list[dict] | None = None):
    infos = infos or [{} for _ in range(n)]
    steps = [
        TrajectoryStep(
            index=i,
            obs=None,
            action=None,
            reward=0.0,
            terminated=False,
            truncated=False,
            info=infos[i],
        )
        for i in range(n)
    ]
    return TrajectoryRecord(seed=0, steps=steps, terminated=terminated, truncated=False)


class TestAssertLapCompleted:
    def test_passes_when_progress_meets_target(self):
        record = _record(10)
        assert_lap_completed(record, progress_fn=lambda r: 1.0, target_progress=1.0)

    def test_raises_when_progress_below_target(self):
        record = _record(10)
        with pytest.raises(AssertionError, match="lap not completed"):
            assert_lap_completed(record, progress_fn=lambda r: 0.5, target_progress=1.0)

    def test_progress_fn_receives_the_record(self):
        record = _record(3)
        seen = []

        def progress_fn(r):
            seen.append(r)
            return r.step_count

        assert_lap_completed(record, progress_fn=progress_fn, target_progress=3)
        assert seen == [record]


class TestAssertNoWallContact:
    def test_passes_when_no_step_collides(self):
        record = _record(5)
        assert_no_wall_contact(record, collision_predicate=lambda step: False)

    def test_raises_with_step_index_on_collision(self):
        infos = [{}, {}, {"collision": True}, {}]
        record = _record(4, infos=infos)
        with pytest.raises(AssertionError, match="step 2"):
            assert_no_wall_contact(
                record, collision_predicate=lambda step: bool(step.info.get("collision"))
            )

    def test_empty_trajectory_never_raises(self):
        record = _record(0)
        assert_no_wall_contact(record, collision_predicate=lambda step: True)


class TestAssertLapTimeInBand:
    def test_within_band_passes(self):
        assert_lap_time_in_band(10.0, low_s=9.0, high_s=11.0)

    def test_at_low_boundary_passes(self):
        assert_lap_time_in_band(9.0, low_s=9.0, high_s=11.0)

    def test_at_high_boundary_passes(self):
        assert_lap_time_in_band(11.0, low_s=9.0, high_s=11.0)

    def test_below_band_raises(self):
        with pytest.raises(AssertionError):
            assert_lap_time_in_band(8.999, low_s=9.0, high_s=11.0)

    def test_above_band_raises(self):
        with pytest.raises(AssertionError):
            assert_lap_time_in_band(11.001, low_s=9.0, high_s=11.0)

    def test_invalid_band_raises_value_error(self):
        with pytest.raises(ValueError):
            assert_lap_time_in_band(10.0, low_s=11.0, high_s=9.0)


class TestLapTimeSeconds:
    def test_multiplies_step_count_by_dt(self):
        record = _record(50)
        assert lap_time_seconds(record, dt_s=0.02) == pytest.approx(1.0)


class TestAssertTrajectoryMatchesReference:
    def test_identical_trajectories_pass(self):
        traj = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        assert_trajectory_matches_reference(traj, traj, atol_m=0.0)

    def test_within_tolerance_passes(self):
        actual = [(0.0, 0.0), (1.01, 0.0)]
        reference = [(0.0, 0.0), (1.0, 0.0)]
        assert_trajectory_matches_reference(actual, reference, atol_m=0.02)

    def test_out_of_tolerance_raises_with_index(self):
        actual = [(0.0, 0.0), (5.0, 0.0)]
        reference = [(0.0, 0.0), (1.0, 0.0)]
        with pytest.raises(AssertionError, match="index 1"):
            assert_trajectory_matches_reference(actual, reference, atol_m=0.1)

    def test_length_mismatch_raises(self):
        with pytest.raises(AssertionError, match="length mismatch"):
            assert_trajectory_matches_reference([(0.0, 0.0)], [(0.0, 0.0), (1.0, 0.0)], atol_m=1.0)

    def test_empty_trajectories_pass(self):
        assert_trajectory_matches_reference([], [], atol_m=0.0)
