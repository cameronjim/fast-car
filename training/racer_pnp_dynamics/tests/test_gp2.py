"""L1/L2 for gp2.py: every decision item at, just inside and just outside its threshold, and the
stability sweep catching a deliberately unstable residual."""

from __future__ import annotations

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_pnp_dynamics import gp2
from racer_pnp_dynamics.model import MlpWeights, Residual, zero_weights
from test_fit import TRUTH

EPS = 1e-9
OK_SWEEP = gp2.SweepResult(1000, 30, 0, 0, 0, 1.2, 0.3)


def test_thresholds_are_the_pre_registered_numbers():
    assert (gp2.REFIT_P1_MAX_RATIO, gp2.REFIT_S1_MAX_RATIO, gp2.REFIT_Y1_MAX_RATIO) == (
        1.0,
        1.05,
        1.05,
    )
    assert (gp2.RESIDUAL_P1_MAX_RATIO, gp2.RESIDUAL_P1_CI_HI_BELOW) == (0.95, 1.00)
    assert (gp2.RESIDUAL_Y1_MAX_RATIO, gp2.RESIDUAL_S1_MAX_RATIO) == (1.00, 1.02)
    assert (gp2.RESIDUAL_SEED_P1_MAX_RATIO, gp2.RESIDUAL_STRATUM_MAX_RATIO) == (1.00, 1.05)
    assert (gp2.SWEEP_SAMPLES, gp2.SWEEP_STEPS, gp2.SWEEP_V_FACTOR) == (100_000, 30, 1.5)


def _refit(p, y, s):
    base = {"P1": 1.0, "Y1": 1.0, "S1": 1.0}
    return gp2.decide_refit({"P1": p, "Y1": y, "S1": s}, base)


@pytest.mark.parametrize(
    "which, limit",
    [(0, 1.00), (1, 1.05), (2, 1.05)],
)
def test_refit_items_at_just_inside_and_just_outside(which, limit):
    def at(x):
        vals = [1.0, 1.0, 1.0]  # P, Y, S
        vals[{0: 0, 1: 2, 2: 1}[which]] = x
        return _refit(*vals)

    assert at(limit).passed  # at the threshold passes (at most)
    assert at(limit - EPS).passed
    assert not at(limit + 1e-6).passed
    d = at(limit + 0.01)
    assert [i.passed for i in d.items].count(False) == 1  # only that item fails
    assert len(d.items) == 3 and all("ratio" in i.name for i in d.items)


def test_refit_with_no_number_does_not_pass():
    assert not _refit(float("nan"), 1.0, 1.0).passed


def _res(p=0.90, y=0.99, s=1.0, hi=0.97, seeds=None, strata=None, sweep=OK_SWEEP):
    nom = {"P1": 1.0, "Y1": 1.0, "S1": 1.0}
    seeds = seeds or {0: 0.92, 1: 0.90, 2: 0.95}
    strata = strata or {"cw:0.25-0.6": 1.0, "ccw:1.25+": 1.0}
    return gp2.decide_residual(
        nom,
        {"P1": p, "Y1": y, "S1": s},
        hi,
        seeds,
        strata,
        {k: v * 0.95 for k, v in strata.items()},
        sweep,
    )


def test_a_residual_that_meets_every_item_ships():
    d = _res()
    assert d.passed and d.gate.startswith("3.2")
    names = [i.name for i in d.items]
    assert any(n.startswith("1a") for n in names) and any(n.startswith("5") for n in names)
    assert sum(n.startswith("3 seed") for n in names) == 3
    assert sum(n.startswith("4 stratum") for n in names) == 2


def _failing(d):
    return [i.name[:2] for i in d.items if not i.passed]


def test_item_1_p1_ratio_at_just_inside_just_outside():
    assert _res(p=0.95).passed and _res(p=0.95 - EPS).passed
    assert _failing(_res(p=0.95 + 1e-6)) == ["1a"]


def test_item_1_bootstrap_upper_end_must_be_strictly_below_one():
    assert _res(hi=1.0 - EPS).passed
    assert _failing(_res(hi=1.0)) == ["1b"]
    assert _failing(_res(hi=1.02)) == ["1b"]


def test_item_2_yaw_and_speed_ratios():
    assert _res(y=1.00).passed and _failing(_res(y=1.00 + 1e-6)) == ["2a"]
    assert _res(s=1.02).passed and _failing(_res(s=1.02 + 1e-6)) == ["2b"]


def test_item_3_every_seed_must_be_at_most_the_nominal():
    assert _res(seeds={0: 1.0, 1: 0.9, 2: 0.9}).passed
    d = _res(seeds={0: 1.0 + 1e-6, 1: 0.9, 2: 0.9})
    assert _failing(d) == ["3 "] and not d.passed


def test_item_4_every_stratum_at_most_1_05():
    nominal = {"a": 1.0, "b": 2.0}

    def with_ratio(r):
        return gp2.decide_residual(
            {"P1": 1.0, "Y1": 1.0, "S1": 1.0},
            {"P1": 0.9, "Y1": 1.0, "S1": 1.0},
            0.9,
            {0: 0.9},
            nominal,
            {"a": 1.0, "b": 2.0 * r},
            OK_SWEEP,
        )

    assert with_ratio(1.05).passed and with_ratio(1.05 - EPS).passed
    assert _failing(with_ratio(1.05 + 1e-6)) == ["4 "]


def test_item_5_the_sweep_must_be_clean():
    for bad in (
        gp2.SweepResult(10, 30, 1, 0, 0, 1.0, 0.3),
        gp2.SweepResult(10, 30, 0, 1, 0, 1.0, 0.5),
        gp2.SweepResult(10, 30, 0, 0, 1, 9.0, 0.3),
    ):
        assert not bad.passed
        assert _failing(_res(sweep=bad)) == ["5 "]
    assert OK_SWEEP.passed and OK_SWEEP.to_dict()["passed"]


def test_decision_serialises_every_item_with_value_threshold_and_result():
    d = _res(p=0.99).to_dict()
    assert d["passed"] is False
    row = next(i for i in d["items"] if i["name"].startswith("1a"))
    assert row == {
        "name": row["name"],
        "value": pytest.approx(0.99),
        "comparison": "<=",
        "threshold": 0.95,
        "passed": False,
        "note": "",
    }


# ---- the random sweep ------------------------------------------------------------------------


def _residual_with(w3_scale, b3=(0.0, 0.0), out=(1.0, 1.0)):
    w = zero_weights()
    w = MlpWeights(w.w1, w.b1 + 1.0, w.w2, w.b2 + 1.0, w.w3 + w3_scale, np.array(b3))
    return Residual(True, w, np.zeros(4), np.ones(4), np.array(out))


def test_the_sweep_passes_for_the_nominal_alone_and_for_a_gentle_residual(toy_vehicle):
    r = gp2.stability_sweep(TRUTH, None, toy_vehicle, 1.0, samples=2000, steps=30)
    assert r.passed and r.samples == 2000 and r.max_speed_mps <= 1.5
    gentle = _residual_with(0.001, out=(0.1, 0.05))
    assert gp2.stability_sweep(TRUTH, gentle, toy_vehicle, 1.0, samples=2000).passed


def test_the_sweep_catches_a_deliberately_unstable_residual(toy_vehicle):
    # +5 m/s2 of extra acceleration whatever the state: the speed leaves 1.5 x v_max
    runaway = _residual_with(0.0, b3=(5.0, 0.0))
    r = gp2.stability_sweep(TRUTH, runaway, toy_vehicle, 1.0, samples=500, steps=30)
    assert not r.passed and r.speed_violations > 0
    # a residual whose output is not finite
    nan = _residual_with(0.0, b3=(float("nan"), 0.0))
    r = gp2.stability_sweep(TRUTH, nan, toy_vehicle, 1.0, samples=100, steps=5)
    assert not r.passed and r.nonfinite == 100


def test_the_sweep_flags_a_steering_state_beyond_the_state_bounds(toy_vehicle):
    ok = gp2.stability_sweep(TRUTH, None, toy_vehicle, 1.0, samples=300, steps=10)
    assert ok.passed and ok.max_abs_delta_rad <= toy_vehicle.steer_max_rad + 1e-9
    # the same rollouts checked against bounds narrower than the box they started in
    r = gp2.stability_sweep(
        TRUTH, None, toy_vehicle, 1.0, samples=300, steps=10, delta_limits=(-0.05, 0.05)
    )
    assert not r.passed and r.steering_violations > 0 and r.speed_violations == 0


@settings(max_examples=30, deadline=None)
@given(
    p=st.floats(0.5, 1.5),
    hi=st.floats(0.5, 1.5),
    y=st.floats(0.9, 1.1),
)
def test_decision_passes_exactly_when_every_item_does(p, hi, y):
    d = _res(p=p, hi=hi, y=y)
    assert d.passed == all(i.passed for i in d.items)
    expect = p <= 0.95 and hi < 1.0 and y <= 1.0
    assert d.passed == expect
