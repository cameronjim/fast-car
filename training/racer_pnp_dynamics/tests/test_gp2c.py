"""L1 for the GP2c decisions (pre-registration v0.3 section 2.3): one table-driven test per item,
each with a pass, exactly at the threshold, an epsilon over and NaN."""

from __future__ import annotations

import math

import pytest
from racer_pnp_dynamics import gp2

EPS = 1e-9
SWEEP_OK = gp2.SweepResult(1000, 30, 0, 0, 0, 1.0, 0.4)
SWEEP_BAD = gp2.SweepResult(1000, 30, 3, 0, 0, 1.0, 0.4)


def _refit(**over):
    args = {
        "nominal": {"P1": 1.0, "Y1": 1.0, "S1": 0.04},
        "candidate": {"P1": 0.9, "Y1": 0.9, "S1": 0.04},
        "ratio_ci_hi": 0.95,
        "stratum_nominal_pos": {"ccw:0.25-0.6": 1.0, "cw:0.85+": 2.0},
        "stratum_candidate_pos": {"ccw:0.25-0.6": 1.0, "cw:0.85+": 2.0},
        "yp_nominal": 0.03,
        "yp_candidate": 0.027,
        "on_bound": [],
    }
    args.update(over)
    return gp2.decide_yaw_refit(**args)


def _resid(**over):
    args = {
        "nominal": {"P1": 1.0, "Y1": 1.0, "S1": 0.04},
        "residual": {"P1": 0.9, "Y1": 0.9, "S1": 0.04},
        "ratio_ci_hi": 0.95,
        "seed_p1": {0: 0.9, 1: 0.9, 2: 0.95},
        "stratum_nominal_pos": {"ccw:0.25-0.6": 1.0, "cw:0.85+": 2.0},
        "stratum_residual_pos": {"ccw:0.25-0.6": 1.0, "cw:0.85+": 2.0},
        "sweep": SWEEP_OK,
        "yp_nominal": 0.03,
        "yp_residual": 0.027,
    }
    args.update(over)
    return gp2.decide_yaw_residual(**args)


def _by_prefix(decision, prefix):
    return [i for i in decision.items if i.name.startswith(prefix)]


def test_a_clean_candidate_passes_every_item():
    assert _refit().passed and _resid().passed
    assert {i.name[0] for i in _refit().items} == set("123456")
    assert {i.name[0] for i in _resid().items} == set("123456")


# (label, builder kwargs, item prefix, expected pass)
REFIT_CASES = [
    ("1a at threshold", {"candidate": {"P1": 1.0, "Y1": 0.9, "S1": 0.04}}, "1a", True),
    ("1a over", {"candidate": {"P1": 1.0 + EPS, "Y1": 0.9, "S1": 0.04}}, "1a", False),
    ("1a nan", {"candidate": {"P1": math.nan, "Y1": 0.9, "S1": 0.04}}, "1a", False),
    ("1b below", {"ratio_ci_hi": 1.0 - EPS}, "1b", True),
    ("1b at one (strict)", {"ratio_ci_hi": 1.0}, "1b", False),
    ("1b nan", {"ratio_ci_hi": math.nan}, "1b", False),
    ("2 at threshold", {"candidate": {"P1": 0.9, "Y1": 1.0, "S1": 0.04}}, "2", True),
    ("2 over", {"candidate": {"P1": 0.9, "Y1": 1.0 + EPS, "S1": 0.04}}, "2", False),
    ("2 nan", {"candidate": {"P1": 0.9, "Y1": math.nan, "S1": 0.04}}, "2", False),
    ("3 at 1.05", {"stratum_candidate_pos": {"ccw:0.25-0.6": 1.05, "cw:0.85+": 2.0}}, "3", True),
    (
        "3 over",
        {"stratum_candidate_pos": {"ccw:0.25-0.6": 1.0, "cw:0.85+": 2.0 * (1.05 + EPS)}},
        "3",
        False,
    ),
    ("3 nan", {"stratum_candidate_pos": {"ccw:0.25-0.6": math.nan, "cw:0.85+": 2.0}}, "3", False),
    ("4 at threshold", {"yp_candidate": 0.03}, "4", True),
    ("4 over", {"yp_candidate": 0.03 * (1.0 + 1e-6)}, "4", False),
    ("4 nan", {"yp_candidate": math.nan}, "4", False),
    ("4 nominal nan", {"yp_nominal": math.nan}, "4", False),
    ("5 equal", {"candidate": {"P1": 0.9, "Y1": 0.9, "S1": 0.04 + 1e-7}}, "5", True),
    (
        "5 exactly at tolerance",
        {
            "nominal": {"P1": 1.0, "Y1": 1.0, "S1": 0.0},
            "candidate": {"P1": 0.9, "Y1": 0.9, "S1": 1e-6},
        },
        "5",
        True,
    ),
    ("5 1e-5 apart", {"candidate": {"P1": 0.9, "Y1": 0.9, "S1": 0.04 + 1e-5}}, "5", False),
    ("5 other way", {"candidate": {"P1": 0.9, "Y1": 0.9, "S1": 0.04 - 1e-5}}, "5", False),
    ("5 nan", {"candidate": {"P1": 0.9, "Y1": 0.9, "S1": math.nan}}, "5", False),
    ("6 none", {"on_bound": []}, "6", True),
    ("6 one on a bound", {"on_bound": ["understeer_s2_per_m2"]}, "6", False),
]


@pytest.mark.parametrize(("label", "over", "prefix", "expected"), REFIT_CASES)
def test_yaw_refit_items(label, over, prefix, expected):
    d = _refit(**over)
    items = _by_prefix(d, prefix)
    assert items, label
    assert all(i.passed for i in items) == expected, (label, [i.to_dict() for i in items])
    assert d.passed == expected or not expected  # a failing item fails the gate


RESID_CASES = [
    ("1a at threshold", {"residual": {"P1": 0.95, "Y1": 0.9, "S1": 0.04}}, "1a", True),
    ("1a over", {"residual": {"P1": 0.95 + EPS, "Y1": 0.9, "S1": 0.04}}, "1a", False),
    ("1a nan", {"residual": {"P1": math.nan, "Y1": 0.9, "S1": 0.04}}, "1a", False),
    ("1b below", {"ratio_ci_hi": 1.0 - EPS}, "1b", True),
    ("1b at one", {"ratio_ci_hi": 1.0}, "1b", False),
    ("1b nan", {"ratio_ci_hi": math.nan}, "1b", False),
    ("2a at threshold", {"residual": {"P1": 0.9, "Y1": 1.0, "S1": 0.04}}, "2a", True),
    ("2a over", {"residual": {"P1": 0.9, "Y1": 1.0 + EPS, "S1": 0.04}}, "2a", False),
    ("2a nan", {"residual": {"P1": 0.9, "Y1": math.nan, "S1": 0.04}}, "2a", False),
    ("2b equal", {"residual": {"P1": 0.9, "Y1": 0.9, "S1": 0.04}}, "2b", True),
    ("2b 1e-5 apart", {"residual": {"P1": 0.9, "Y1": 0.9, "S1": 0.04 + 1e-5}}, "2b", False),
    ("2b nan", {"residual": {"P1": 0.9, "Y1": 0.9, "S1": math.nan}}, "2b", False),
    ("3 seed at one", {"seed_p1": {0: 1.0, 1: 0.9, 2: 0.9}}, "3", True),
    ("3 a seed over", {"seed_p1": {0: 1.0 + EPS, 1: 0.9, 2: 0.9}}, "3", False),
    ("3 seed nan", {"seed_p1": {0: math.nan, 1: 0.9, 2: 0.9}}, "3", False),
    ("4 at 1.05", {"stratum_residual_pos": {"ccw:0.25-0.6": 1.05, "cw:0.85+": 2.0}}, "4", True),
    (
        "4 over",
        {"stratum_residual_pos": {"ccw:0.25-0.6": 1.0, "cw:0.85+": 2.0 * (1.05 + EPS)}},
        "4",
        False,
    ),
    ("4 nan", {"stratum_residual_pos": {"ccw:0.25-0.6": math.nan, "cw:0.85+": 2.0}}, "4", False),
    ("5 sweep ok", {"sweep": SWEEP_OK}, "5", True),
    ("5 sweep bad", {"sweep": SWEEP_BAD}, "5", False),
    ("6 at threshold", {"yp_residual": 0.03}, "6", True),
    ("6 over", {"yp_residual": 0.03 * (1.0 + 1e-6)}, "6", False),
    ("6 nan", {"yp_residual": math.nan}, "6", False),
]


@pytest.mark.parametrize(("label", "over", "prefix", "expected"), RESID_CASES)
def test_yaw_residual_items(label, over, prefix, expected):
    d = _resid(**over)
    items = _by_prefix(d, prefix)
    assert items, label
    assert all(i.passed for i in items) == expected, (label, [i.to_dict() for i in items])
    if not expected:
        assert not d.passed


def test_a_failing_refit_item_fails_the_gate():
    assert not _refit(on_bound=["steer_gain_left"]).passed
    assert not _refit(candidate={"P1": 0.9, "Y1": 0.9, "S1": 0.04 + 1e-5}).passed


def test_yp_that_cannot_be_applied_is_stated_not_substituted():
    item = _by_prefix(_refit(yp_candidate=math.nan, yp_nominal=math.nan), "4")[0]
    assert not item.passed and "cannot be applied" in item.note
    assert math.isnan(item.value)


def test_thresholds_are_the_pre_registered_numbers():
    assert gp2.YAW_REFIT_STRATUM_MAX_RATIO == 1.05 and gp2.YAW_RESIDUAL_STRATUM_MAX_RATIO == 1.05
    assert gp2.YAW_RESIDUAL_P1_MAX_RATIO == 0.95 and gp2.YAW_REFIT_P1_MAX_RATIO == 1.0
    assert gp2.S1_EQUAL_TOLERANCE_MPS == 1e-6
    assert gp2.YAW_REFIT_MAX_ON_BOUND == 0
