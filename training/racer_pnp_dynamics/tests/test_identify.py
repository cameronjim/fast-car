"""L1 for identify.py: the identification verdicts and the steering lag cross-check."""

from __future__ import annotations

import numpy as np
import pytest
from racer_pnp_dynamics import identify
from racer_pnp_dynamics.dataset import make_segments
from synth import make_run
from test_fit import TRUTH

GRID = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30]


def test_a_sharp_minimum_in_the_middle_is_identified():
    errs = [0.20, 0.12, 0.05, 0.12, 0.20, 0.30, 0.40]
    v = identify.assess(0.10, (0.0, 0.30), GRID, errs)
    assert v.identified and v.reasons == ()


def test_a_value_within_half_a_percent_of_a_bound_is_not_identified():
    errs = [0.05, 0.12, 0.20, 0.30, 0.40, 0.50, 0.60]
    low = identify.assess(0.0, (0.0, 0.30), GRID, errs)
    assert not low.identified and "lower bound" in low.text()
    high = identify.assess(0.299, (0.0, 0.30), GRID, errs[::-1])
    assert not high.identified and "upper bound" in high.text()
    near = identify.assess(0.0014, (0.0, 0.30), GRID, errs)  # 0.47 percent of the range
    assert not near.identified
    away = identify.assess(0.0016, (0.0, 0.30), GRID, errs)  # 0.53 percent of the range
    assert away.identified


def test_a_flat_profile_over_more_than_half_the_range_is_not_identified():
    flat = [0.100, 0.101, 0.102, 0.101, 0.100, 0.101, 0.150]
    v = identify.assess(0.10, (0.0, 0.30), GRID, flat)
    assert not v.identified and "flat" in v.text()
    # 2 percent exactly is still flat, 3 percent is not
    assert identify.flat_share(GRID, [0.1, 0.102, 0.1, 0.1, 0.1, 0.1, 0.1]) == pytest.approx(1.0)
    assert identify.flat_share(GRID, [0.1, 0.103, 0.1, 0.1, 0.1, 0.1, 0.1]) < 0.7


def test_flat_over_exactly_half_the_range_is_not_more_than_half():
    errs = [0.1, 0.1, 0.1, 0.1, 0.5, 0.5, 0.5]  # flat on 0.0 to 0.15 of 0.30
    assert identify.flat_share(GRID, errs) == pytest.approx(0.5)
    assert identify.assess(0.05, (0.0, 0.3), GRID, errs).identified


def test_time_constants_are_measured_on_a_log_scale():
    grid = [0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0]
    errs = [1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 3.0, 4.0]  # flat on 0.005 to 0.1: ln 20 of ln 200
    assert identify.flat_share(grid, errs, log_scale=True) == pytest.approx(0.565, abs=0.01)
    assert identify.flat_share(grid, errs, log_scale=False) < 0.15  # linear measure disagrees


def test_non_finite_errors_do_not_crash_the_verdict():
    v = identify.assess(0.1, (0.0, 0.3), GRID, [np.nan, 0.2, 0.1, 0.2, 0.3, 0.4, 0.5])
    assert v.identified
    assert identify.flat_share([0.0, 1.0], [np.nan, np.nan]) == 0.0


def test_profile_table_reports_the_excess_over_the_minimum():
    rows = identify.profile_table([1.0, 2.0, 3.0], [0.2, 0.1, 0.15])
    assert [r["over_min"] for r in rows] == pytest.approx([1.0, 0.0, 0.5])
    assert [r["value"] for r in rows] == [1.0, 2.0, 3.0]


def test_steering_lag_recovers_a_known_delay_plus_time_constant(toy_vehicle):
    truth = TRUTH.replace(command_delay_s=0.10, steer_tau_s=0.10)
    run, motion = make_run(truth, toy_vehicle, seconds=600.0, seed=31, yaw_noise=0.001)
    lag = identify.steering_lag_s(make_segments(run, motion))
    assert lag is not None and lag.n_intervals > 1000
    assert lag.peak_correlation > 0.5
    # delay + tau is the mean lag of the response; the ICP interval is a 0.1 s average
    assert lag.lag_s == pytest.approx(0.10 + 0.10, abs=0.06)


def test_steering_lag_is_none_without_motion_or_enough_intervals(toy_vehicle):
    run, _ = make_run(TRUTH, toy_vehicle, seconds=60.0, seed=32)
    assert identify.steering_lag_s(make_segments(run, None)) is None
    assert identify.steering_lag_s([]) is None
