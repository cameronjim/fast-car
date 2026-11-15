"""L1 for the GP2d decisions (pre-registration v0.4 section 2.4): one table-driven test per item,
each with a pass, exactly at the threshold, an epsilon over and NaN; the heading outcome; the
validity rule with its "cannot be applied" case."""

from __future__ import annotations

import dataclasses
import math

import pytest
import yaml
from racer_pnp_dynamics import gp2
from racer_pnp_dynamics.vehicle import repo_root

EPS = 1e-9
SWEEP_OK = gp2.SweepResult(1000, 30, 0, 0, 0, 1.0, 0.4)
SWEEP_BAD = gp2.SweepResult(1000, 30, 3, 0, 0, 1.0, 0.4)
STRATA = {"ccw:0.25-0.6": 1.0, "cw:0.85+": 2.0}


def _nominal(**over):
    args = {
        "v01": {"P1": 1.0, "Y1": 1.0, "S1": 0.04},
        "n4": {"P1": 0.9, "Y1": 0.9, "S1": 0.04},
        "ratio_ci_hi": 0.95,
        "stratum_v01_pos": STRATA,
        "stratum_n4_pos": dict(STRATA),
        "on_bound": [],
        "yp_v01": 0.03,
        "yp_n4": 0.027,
    }
    args.update(over)
    return gp2.decide_gp2d_nominal(**args)


def _resid(**over):
    args = {
        "nominal": {"P1": 1.0, "Y1": 1.0, "S1": 0.04},
        "residual": {"P1": 0.9, "Y1": 0.9, "S1": 0.04},
        "ratio_ci_hi": 0.95,
        "seed_p1": {0: 0.9, 1: 0.9, 2: 0.95},
        "stratum_nominal_pos": STRATA,
        "stratum_residual_pos": dict(STRATA),
        "sweep": SWEEP_OK,
        "yp_nominal": 0.03,
        "yp_residual": 0.027,
    }
    args.update(over)
    return gp2.decide_gp2d_residual(**args)


def _heading(**over):
    args = {
        "nominal": {"P1": 1.0, "Y1": 1.0},
        "candidate": {"P1": 0.95, "Y1": 0.7},
        "lateral_ratio": 0.8,
        "lateral_ci_hi": 0.9,
        "stratum_nominal_pos": STRATA,
        "stratum_candidate_pos": dict(STRATA),
    }
    args.update(over)
    return gp2.heading_outcome(**args)


def _by_prefix(decision, prefix):
    return [i for i in decision.items if i.name.startswith(prefix)]


def test_clean_candidates_pass_every_item():
    assert _nominal().passed and _resid().passed
    assert {i.name[0] for i in _nominal().items} == set("12345")
    assert {i.name[0] for i in _resid().items} == set("123456")
    assert _nominal().gate.startswith("GP2d-N") and _resid().gate.startswith("GP2d-R")


NOMINAL_CASES = [
    ("1a at threshold", {"n4": {"P1": 1.0, "Y1": 0.9, "S1": 0.04}}, "1a", True),
    ("1a over", {"n4": {"P1": 1.0 + EPS, "Y1": 0.9, "S1": 0.04}}, "1a", False),
    ("1a nan", {"n4": {"P1": math.nan, "Y1": 0.9, "S1": 0.04}}, "1a", False),
    ("1b below", {"ratio_ci_hi": 1.0 - EPS}, "1b", True),
    ("1b at threshold fails (strict)", {"ratio_ci_hi": 1.0}, "1b", False),
    ("1b nan", {"ratio_ci_hi": math.nan}, "1b", False),
    ("2a at threshold", {"n4": {"P1": 0.9, "Y1": 1.0, "S1": 0.04}}, "2a", True),
    ("2a over", {"n4": {"P1": 0.9, "Y1": 1.0 + EPS, "S1": 0.04}}, "2a", False),
    ("2a nan", {"n4": {"P1": 0.9, "Y1": math.nan, "S1": 0.04}}, "2a", False),
    ("2b at threshold", {"n4": {"P1": 0.9, "Y1": 0.9, "S1": 0.04}}, "2b", True),
    (
        "2b over (no 1.02 slack here)",
        {"n4": {"P1": 0.9, "Y1": 0.9, "S1": 0.04 * (1 + 1e-6)}},
        "2b",
        False,
    ),
    ("2b nan", {"n4": {"P1": 0.9, "Y1": 0.9, "S1": math.nan}}, "2b", False),
    (
        "3 at 1.05",
        {"stratum_n4_pos": {"ccw:0.25-0.6": 1.05, "cw:0.85+": 2.0}},
        "3 stratum ccw",
        True,
    ),
    (
        "3 over 1.05",
        {"stratum_n4_pos": {"ccw:0.25-0.6": 1.05 + 1e-6, "cw:0.85+": 2.0}},
        "3 stratum ccw",
        False,
    ),
    (
        "3 nan",
        {"stratum_n4_pos": {"ccw:0.25-0.6": math.nan, "cw:0.85+": 2.0}},
        "3 stratum ccw",
        False,
    ),
    ("4 none on a bound", {"on_bound": []}, "4", True),
    ("4 one on a bound", {"on_bound": ["steer_tau_s"]}, "4", False),
    ("5 at threshold", {"yp_n4": 0.03}, "5", True),
    ("5 over", {"yp_n4": 0.03 * (1 + 1e-6)}, "5", False),
    ("5 nan", {"yp_n4": math.nan}, "5", False),
    ("5 nominal nan", {"yp_v01": math.nan}, "5", False),
]


@pytest.mark.parametrize(("label", "over", "prefix", "expected"), NOMINAL_CASES)
def test_gp2d_n_items(label, over, prefix, expected):
    decision = _nominal(**over)
    items = _by_prefix(decision, prefix)
    assert items, label
    assert items[0].passed is expected, label
    assert decision.passed is (expected and all(i.passed for i in decision.items))
    if not expected:
        assert not decision.passed


def test_gp2d_n_yp_nan_says_it_cannot_be_applied():
    item = _by_prefix(_nominal(yp_n4=math.nan), "5")[0]
    assert "cannot be applied" in item.note


def test_gp2d_n_thresholds_are_the_pre_registered_ones():
    assert (gp2.NOMINAL_P1_MAX_RATIO, gp2.NOMINAL_Y1_MAX_RATIO, gp2.NOMINAL_S1_MAX_RATIO) == (
        1.0,
        1.0,
        1.0,
    )
    assert gp2.NOMINAL_STRATUM_MAX_RATIO == 1.05 and gp2.NOMINAL_MAX_ON_BOUND == 0
    assert gp2.NOMINAL_YP_MAX_RATIO == 1.0


RESIDUAL_CASES = [
    ("1a at 0.95", {"residual": {"P1": 0.95, "Y1": 0.9, "S1": 0.04}}, "1a", True),
    ("1a over 0.95", {"residual": {"P1": 0.95 + EPS, "Y1": 0.9, "S1": 0.04}}, "1a", False),
    ("1a nan", {"residual": {"P1": math.nan, "Y1": 0.9, "S1": 0.04}}, "1a", False),
    ("1b below", {"ratio_ci_hi": 1.0 - EPS}, "1b", True),
    ("1b at 1.0", {"ratio_ci_hi": 1.0}, "1b", False),
    ("2a at 1.0", {"residual": {"P1": 0.9, "Y1": 1.0, "S1": 0.04}}, "2a", True),
    ("2a over", {"residual": {"P1": 0.9, "Y1": 1.0 + EPS, "S1": 0.04}}, "2a", False),
    ("2b equal within 1e-6", {"residual": {"P1": 0.9, "Y1": 0.9, "S1": 0.04 + 5e-7}}, "2b", True),
    ("2b over 1e-6", {"residual": {"P1": 0.9, "Y1": 0.9, "S1": 0.04 + 2e-6}}, "2b", False),
    ("2b nan", {"residual": {"P1": 0.9, "Y1": 0.9, "S1": math.nan}}, "2b", False),
    ("3 seed at 1.0", {"seed_p1": {0: 1.0, 1: 0.9}}, "3 seed 0", True),
    ("3 seed over", {"seed_p1": {0: 1.0 + EPS, 1: 0.9}}, "3 seed 0", False),
    ("3 seed nan", {"seed_p1": {0: math.nan, 1: 0.9}}, "3 seed 0", False),
    (
        "4 at 1.05",
        {"stratum_residual_pos": {"ccw:0.25-0.6": 1.05, "cw:0.85+": 2.0}},
        "4 stratum ccw",
        True,
    ),
    (
        "4 over",
        {"stratum_residual_pos": {"ccw:0.25-0.6": 1.05 + 1e-6, "cw:0.85+": 2.0}},
        "4 stratum ccw",
        False,
    ),
    ("5 sweep ok", {"sweep": SWEEP_OK}, "5", True),
    ("5 sweep bad", {"sweep": SWEEP_BAD}, "5", False),
    ("6 Yp at 1.0", {"yp_residual": 0.03}, "6", True),
    ("6 Yp over", {"yp_residual": 0.03 * (1 + 1e-6)}, "6", False),
    ("6 Yp nan", {"yp_residual": math.nan}, "6", False),
]


@pytest.mark.parametrize(("label", "over", "prefix", "expected"), RESIDUAL_CASES)
def test_gp2d_r_items(label, over, prefix, expected):
    items = _by_prefix(_resid(**over), prefix)
    assert items and items[0].passed is expected, label


def test_gp2d_r_keeps_the_gp2c_r_thresholds():
    assert gp2.YAW_RESIDUAL_P1_MAX_RATIO == 0.95 and gp2.YAW_RESIDUAL_Y1_MAX_RATIO == 1.0
    assert gp2.S1_EQUAL_TOLERANCE_MPS == 1e-6 and gp2.YAW_RESIDUAL_STRATUM_MAX_RATIO == 1.05
    assert gp2.YAW_RESIDUAL_YP_MAX_RATIO == 1.0 and gp2.SWEEP_SAMPLES == 100_000


HEADING_CASES = [
    ("Y1 at 0.80", {"candidate": {"P1": 0.95, "Y1": 0.80}}, True),
    ("Y1 over 0.80", {"candidate": {"P1": 0.95, "Y1": 0.80 + EPS}}, False),
    ("Y1 nan", {"candidate": {"P1": 0.95, "Y1": math.nan}}, False),
    ("lateral at 0.90", {"lateral_ratio": 0.90}, True),
    ("lateral over 0.90", {"lateral_ratio": 0.90 + EPS}, False),
    ("lateral nan", {"lateral_ratio": math.nan}, False),
    ("lateral ci below 1", {"lateral_ci_hi": 1.0 - EPS}, True),
    ("lateral ci at 1", {"lateral_ci_hi": 1.0}, False),
    ("lateral ci nan", {"lateral_ci_hi": math.nan}, False),
    ("P1 at 1.00", {"candidate": {"P1": 1.0, "Y1": 0.7}}, True),
    ("P1 over 1.00", {"candidate": {"P1": 1.0 + EPS, "Y1": 0.7}}, False),
    ("P1 nan", {"candidate": {"P1": math.nan, "Y1": 0.7}}, False),
    ("stratum at 1.05", {"stratum_candidate_pos": {"ccw:0.25-0.6": 1.05, "cw:0.85+": 2.0}}, True),
    (
        "stratum over",
        {"stratum_candidate_pos": {"ccw:0.25-0.6": 1.05 + 1e-6, "cw:0.85+": 2.0}},
        False,
    ),
    ("stratum nan", {"stratum_candidate_pos": {"ccw:0.25-0.6": math.nan, "cw:0.85+": 2.0}}, False),
]


@pytest.mark.parametrize(("label", "over", "expected"), HEADING_CASES)
def test_heading_outcome(label, over, expected):
    out = _heading(**over)
    assert (out.label == gp2.HEADING_LABEL) is expected, label
    assert out.label in (gp2.HEADING_LABEL, gp2.NO_NAMED_OUTCOME)


def test_heading_outcome_is_a_label_and_never_a_deploy_flag():
    out = _heading()
    assert out.label == "improves heading, not position"
    names = {f.name for f in dataclasses.fields(out)} | {k for k in out.to_dict()}
    assert not any("deploy" in n or "ship" in n or "pass" in n for n in names)
    assert not hasattr(out, "passed") and not hasattr(out, "deploy")


def test_the_constants_match_the_frozen_yaml():
    path = repo_root() / "evaluation" / "prereg" / "pnp-v0.4.yaml"
    if not path.is_file():
        pytest.skip("evaluation/prereg/pnp-v0.4.yaml is not in this checkout (PR 186)")
    d = yaml.safe_load(path.read_text(encoding="utf-8"))["dynamics"]
    assert d["heading_outcome"]["max_y1_ratio"] == gp2.HEADING_Y1_MAX_RATIO
    assert d["heading_outcome"]["max_lateral_ratio"] == gp2.HEADING_LATERAL_MAX_RATIO
    assert d["heading_outcome"]["max_p1_ratio"] == gp2.HEADING_P1_MAX_RATIO
    assert d["fast_item"]["min_start_speed_mps"] == gp2.FAST_MIN_START_SPEED_MPS
    assert d["fast_item"]["min_windows_per_direction"] == gp2.FAST_MIN_WINDOWS_PER_DIRECTION
    assert d["validity_cap_mps"] == gp2.VALIDITY_CAP_MPS
    assert d["budget_measurement"]["unreachable_if_perfect_yaw_ratio_above"] == 0.95


# ---- validity -------------------------------------------------------------------------------


def _fast(ccw=(40, 0.9, 1.0), cw=(40, 0.9, 1.0)):
    return [
        gp2.FastWindows("ccw", ccw[0], ccw[1], ccw[2]),
        gp2.FastWindows("cw", cw[0], cw[1], cw[2]),
    ]


VALIDITY_CASES = [
    # label, rule, gate, fast windows, expected v_max
    ("holds both directions, capped at 1.5", 1.8, True, _fast(), 1.5),
    ("holds, rule below the cap", 1.4, True, _fast(), 1.4),
    ("ratio exactly 1.00 holds", 1.8, True, _fast((40, 1.0, 1.0), (40, 1.0, 1.0)), 1.5),
    ("ratio epsilon over fails", 1.8, True, _fast((40, 1.0 + EPS, 1.0)), 1.3),
    ("ratio nan fails", 1.8, True, _fast((40, math.nan, 1.0)), 1.3),
    ("exactly 30 windows is enough", 1.8, True, _fast((30, 0.9, 1.0), (30, 0.9, 1.0)), 1.5),
    ("29 windows cannot be applied", 1.8, True, _fast((29, 0.9, 1.0)), 1.3),
    ("cw too few windows", 1.8, True, _fast(cw=(0, math.nan, math.nan)), 1.3),
    ("gate failed", 1.8, False, _fast(), 1.3),
    ("rule below 1.3 stays the rule even when held", 1.1, True, _fast((10, 0.9, 1.0)), 1.1),
    ("rule below 1.3 and holds", 1.1, True, _fast(), 1.1),
]


@pytest.mark.parametrize(("label", "rule", "gate", "fast", "expected"), VALIDITY_CASES)
def test_validity_rule(label, rule, gate, fast, expected):
    d = gp2.decide_validity(rule, gate, fast)
    assert d.v_max_mps == pytest.approx(expected), label
    assert d.exceeds_base is (expected > gp2.BASE_VALIDITY_MPS), label


def test_validity_cannot_be_applied_is_said_and_holds_1_3():
    d = gp2.decide_validity(1.8, True, _fast((29, 0.9, 1.0)))
    assert d.v_max_mps == 1.3 and not d.applied and "cannot be applied" in d.note
    assert any(i.note == gp2.FAST_CANNOT_BE_APPLIED for i in d.items)
    assert gp2.decide_validity(1.8, True, []).v_max_mps == 1.3  # no windows at all


def test_validity_rule_that_is_not_a_number_gives_no_validity():
    d = gp2.decide_validity(math.nan, True, _fast())
    assert math.isnan(d.v_max_mps) and not d.exceeds_base


def test_validity_to_dict_round_trips_the_decision():
    d = gp2.decide_validity(1.8, True, _fast())
    out = d.to_dict()
    assert out["v_max_mps"] == 1.5 and out["exceeds_base"] is True and out["applied"] is True
    assert gp2.HeadingOutcome("x", ()).to_dict() == {"label": "x", "items": []}
