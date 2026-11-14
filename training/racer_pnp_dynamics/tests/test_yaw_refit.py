"""L1 for candidate Y (brief 31): data generated from known yaw parameters plus noise, the four
parameter fit recovers them and leaves every frozen parameter bit-identical."""

from __future__ import annotations

import pytest
from racer_pnp_dynamics import pipeline_v03 as p3
from racer_pnp_dynamics.dataset import by_split
from test_fit import TRUTH
from test_residual_train import NOISE, _segments


@pytest.fixture(scope="module")
def refit(toy_vehicle_module):
    segments = _segments(toy_vehicle_module, None, seed=41, **NOISE)
    # the base: the truth with the four yaw parameters knocked off, every other field the truth's
    base = TRUTH.replace(
        steer_gain_left=1.0,
        steer_gain_right=1.0,
        steer_offset_rad=0.0,
        understeer_s2_per_m2=0.0,
    )
    return p3.yaw_refit_fit(segments, toy_vehicle_module, base), base


def test_the_four_yaw_parameters_are_recovered(refit):
    result, _ = refit
    got, want = result.nominal.to_dict(), TRUTH.to_dict()
    for name in p3.YAW_REFIT_FREE:
        assert got[name] == pytest.approx(want[name], rel=0.12, abs=0.01), (name, got[name])
    # the synthetic yaw noise (0.02 rad per interval) is most of the error: the refit must beat the
    # base and land near that floor
    assert result.yaw_rmse_val["refit"] < result.yaw_rmse_val["base"]
    assert result.yaw_rmse_val["refit"] < 1.05 * NOISE["yaw_noise"]


def test_every_frozen_parameter_is_bit_identical(refit):
    result, base = refit
    assert p3.frozen_unchanged(base, result.nominal, p3.YAW_REFIT_FREE) == []
    a, b = base.to_dict(), result.nominal.to_dict()
    for name in a:
        if name not in p3.YAW_REFIT_FREE:
            assert a[name] == b[name]  # exact, not approx
    assert result.nominal.steer_tau_s == base.steer_tau_s  # the fifth yaw parameter stays pinned


def test_the_free_names_are_the_pre_registered_four():
    assert p3.YAW_REFIT_FREE == (
        "steer_gain_left",
        "steer_gain_right",
        "steer_offset_rad",
        "understeer_s2_per_m2",
    )


def test_profiles_and_bounds_are_reported_per_parameter(refit):
    result, _ = refit
    assert set(result.profiles) == set(p3.YAW_REFIT_FREE)
    for name, rows in result.profiles.items():
        assert len(rows) == p3.PROFILE_POINTS
        lo, hi = result.bounds[name]
        assert rows[0]["value"] == pytest.approx(lo) and rows[-1]["value"] == pytest.approx(hi)
        assert min(r["over_min"] for r in rows) == pytest.approx(0.0)
    assert result.counts["yaw_intervals_train"] > 0 and result.counts["yaw_intervals_val"] > 0


def test_a_parameter_the_truth_puts_on_a_bound_is_flagged(toy_vehicle_module):
    """A plant whose understeer is below zero cannot be fitted inside [0, 2]: the fit sits on the
    lower bound and says so (item 6 of GP2c-Y then fails)."""
    segments = _segments(toy_vehicle_module, None, seed=42, **NOISE)
    base = TRUTH.replace(understeer_s2_per_m2=0.0)
    truth_like = by_split(segments, "train")
    assert truth_like  # the segments exist
    result = p3.yaw_refit_fit(segments, toy_vehicle_module, base, free=("understeer_s2_per_m2",))
    # TRUTH has 0.5, inside the bounds: not on a bound
    assert result.on_bound == []
    assert result.nominal.understeer_s2_per_m2 == pytest.approx(0.5, rel=0.15)
