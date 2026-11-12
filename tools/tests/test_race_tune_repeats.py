"""L1 tests for repeated evaluations in the race harness: the pooled variant, the lap ratio against
the MEAN of the baseline's repeats, and the brakes-per-lap interval."""

from __future__ import annotations

import math

import pytest
from race_tuning import report, selection


def _rec(track="t", direction="ccw", lap=20.0, laps=2, brakes=0, completed=True):
    return selection.RunRecord(track, direction, 2, laps, lap, brakes, 0, 0, 0.1, completed)


def test_pool_merges_repeats_by_name_and_keeps_order():
    a, b, c = [_rec(lap=10.0)], [_rec(lap=11.0)], [_rec(lap=12.0)]
    pooled = report.pool({"base": a, "race": b, "base_r2": c, "race_r2": a, "race_r10": b})
    assert list(pooled) == ["base", "race"]
    assert len(pooled["base"]) == 2 and len(pooled["race"]) == 3
    # A name that merely contains _r2 in the middle is not a repeat.
    assert list(report.pool({"x_r2_y": a})) == ["x_r2_y"]


def test_baseline_times_are_the_mean_of_the_completed_repeats_per_track_and_direction():
    base = [
        _rec("a", "ccw", 20.0),
        _rec("a", "ccw", 22.0),
        _rec("a", "cw", 30.0),
        _rec("a", "cw", math.nan, completed=False),  # a DNF is not a time
    ]
    times = selection.baseline_times(base)
    assert times == {("a", "ccw"): 21.0, ("a", "cw"): 30.0}


def test_aggregate_ratio_is_against_the_mean_of_the_baseline_repeats():
    base = [_rec("a", "ccw", 20.0), _rec("a", "ccw", 22.0)]
    agg = selection.aggregate([_rec("a", "ccw", 10.5)], base)
    assert agg.mean_ratio == pytest.approx(0.5)


def test_summary_of_a_pooled_variant_uses_all_repeats():
    base = [_rec("a", "ccw", 20.0, brakes=2), _rec("a", "ccw", 20.0, brakes=4)]
    race = [_rec("a", "ccw", 10.0, brakes=1), _rec("a", "ccw", 12.0, brakes=3)]
    s = report.summarise_variant(race, base)
    assert s["runs"] == 2 and s["completed"] == 2
    assert s["brakes_per_lap"] == pytest.approx(4 / 4)
    mean, low, high = s["brakes_ci"]  # type: ignore[misc]
    assert mean == pytest.approx(1.0) and low <= mean <= high
    ratio = s["ratio"][0]  # type: ignore[index]
    assert ratio == pytest.approx((0.5 + 0.6) / 2)


def test_comparison_table_has_a_brakes_interval_column():
    base = [_rec("a", "ccw", 20.0, brakes=2), _rec("b", "ccw", 20.0, brakes=0)]
    table = report.comparison_table({"baseline": base}, "baseline")
    assert "Brakes per lap (95% CI)" in table
    assert "0.50 [0.00, 1.00]" in table or "0.50 [" in table


def test_intervals_resample_cells_so_repeats_do_not_shrink_them():
    cells = [("a", "ccw", 10.0), ("a", "cw", 14.0), ("b", "ccw", 20.0), ("b", "cw", 30.0)]
    once = [_rec(t, d, lap) for t, d, lap in cells]
    thrice = once * 3  # the same four cells, three times each
    base = [_rec(t, d, 20.0) for t, d, _ in cells]
    s1 = report.summarise_variant(once, base)
    s3 = report.summarise_variant(thrice, base * 3)
    assert s1["lap_time"] == s3["lap_time"]  # identical repeats change nothing
    assert s1["ratio"] == pytest.approx(s3["ratio"])
    mean, low, high = s3["lap_time"]  # type: ignore[misc]
    assert mean == pytest.approx(18.5) and high - low > 3.0  # four cells, not twelve runs
