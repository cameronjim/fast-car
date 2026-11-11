"""The reported numbers come from tested code or they are not reported (12-testing.md): the
paired-evaluation statistics against hand-computed answers and synthetic data with a known
effect size."""

from __future__ import annotations

import numpy as np
import pytest
from racer_train.calibrate import ood_score_percentiles
from racer_train.evaluate import (
    EpisodeResult,
    paired_bootstrap_ci,
    summarize_pair,
    wilson_interval,
)


def test_wilson_matches_known_values():
    # Standard published values for n=10.
    lo, hi = wilson_interval(5, 10)
    assert lo == pytest.approx(0.2366, abs=2e-4)
    assert hi == pytest.approx(0.7634, abs=2e-4)
    lo, hi = wilson_interval(0, 10)
    assert lo == pytest.approx(0.0, abs=1e-12)
    assert hi == pytest.approx(0.2775, abs=2e-4)
    lo, hi = wilson_interval(10, 10)
    assert lo == pytest.approx(0.7225, abs=2e-4)
    assert hi == pytest.approx(1.0, abs=1e-12)


def test_wilson_validates_arguments():
    with pytest.raises(ValueError):
        wilson_interval(1, 0)
    with pytest.raises(ValueError):
        wilson_interval(11, 10)


def test_bootstrap_ci_of_a_constant_is_that_constant():
    mean, lo, hi = paired_bootstrap_ci([-0.5] * 12)
    assert mean == lo == hi == -0.5


def test_bootstrap_recovers_a_known_effect_with_ci_excluding_zero():
    rng = np.random.default_rng(0)
    true_effect = -0.30
    diffs = true_effect + 0.2 * rng.standard_normal(200)
    mean, lo, hi = paired_bootstrap_ci(diffs, seed=1)
    assert lo < true_effect < hi
    assert hi < 0.0
    assert mean == pytest.approx(true_effect, abs=0.05)


def test_bootstrap_ci_straddles_zero_when_there_is_no_effect():
    rng = np.random.default_rng(3)
    diffs = 0.2 * rng.standard_normal(60)
    _, lo, hi = paired_bootstrap_ci(diffs, seed=2)
    assert lo < 0.0 < hi


def test_bootstrap_rejects_empty_input():
    with pytest.raises(ValueError):
        paired_bootstrap_ci([])


def _result(success: bool, lap_time: float | None, crashed: bool = False, residual: bool = False):
    return EpisodeResult(
        track_label="t",
        track_index=0,
        episode_seed=0,
        residual=residual,
        success=success,
        crashed=crashed,
        timed_out=False,
        steps=100,
        time_s=2.0 * (lap_time or 5.0),
        laps=2.0 if success else 0.5,
        lap_time_s=lap_time,
        mean_speed_mps=1.5,
        mean_abs_cte_m=0.05,
        max_abs_cte_m=0.2,
        ttc_engagements=2,
        clipped_fraction=0.0,
        rate_limited_fraction=0.1,
        ood_fraction=0.0,
        mean_abs_residual_steering_rad=0.01,
        mean_abs_residual_speed_mps=0.05,
    )


def test_summarize_pair_handles_dnf_without_imputation():
    base = [_result(True, 10.0), _result(True, 10.0), _result(False, None, crashed=True)]
    learned = [
        _result(True, 9.0, residual=True),
        _result(False, None, crashed=True, residual=True),
        _result(False, None, crashed=True, residual=True),
    ]
    s = summarize_pair(base, learned)
    assert s["tracker"]["completed"] == 2 and s["tracker_plus_residual"]["completed"] == 1
    assert s["tracker"]["completion_rate"] == pytest.approx(2 / 3)
    assert s["tracker"]["lap_time_mean_s"] == 10.0
    assert s["tracker_plus_residual"]["lap_time_mean_s"] == 9.0
    # Only the one episode BOTH arms completed enters the paired difference.
    assert s["paired"]["n_both_completed"] == 1
    assert s["paired"]["lap_time_diff_mean_s"] == pytest.approx(-1.0)
    assert s["paired"]["lap_time_diff_mean_pct"] == pytest.approx(-10.0)


def test_summarize_pair_when_nobody_completes():
    base = [_result(False, None, crashed=True)]
    learned = [_result(False, None, crashed=True, residual=True)]
    s = summarize_pair(base, learned)
    assert s["paired"] == {"n_both_completed": 0}
    assert s["tracker"]["lap_time_mean_s"] is None


def test_summarize_pair_validates_shapes():
    with pytest.raises(ValueError):
        summarize_pair([], [])
    with pytest.raises(ValueError):
        summarize_pair([_result(True, 1.0)], [])


def test_ood_percentiles_hand_computed():
    states = np.array([[0.0, 0.0], [3.0, 4.0]])  # scores 0 and 5 with unit scale and zero ref
    pct = ood_score_percentiles(states, (0.0, 0.0), (1.0, 1.0), percentiles=(0.0, 100.0))
    assert pct[0.0] == 0.0
    assert pct[100.0] == pytest.approx(5.0)
