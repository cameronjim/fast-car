"""L1 for the GP2e decisions (addendum 1 section 2.6): one table-driven test per item, each with a
pass, exactly at the threshold, an epsilon over and NaN; GP2e-L not evaluated when GP2e-T failed;
the outcome names."""

from __future__ import annotations

import math

import pytest
import yaml
from racer_pnp_dynamics import gp2
from racer_pnp_dynamics.ground_sweep import SweepV2
from racer_pnp_dynamics.vehicle import repo_root

EPS = 1e-9
STRATA = {"ccw:0.25-0.6": 1.0, "cw:0.85+": 2.0}
SWEEP_OK = SweepV2(1000, 0, 0, 0, 0)
SWEEP_BAD = SweepV2(1000, 0, 2, 0, 0)
M = {"P1": 1.0, "A1": 1.0, "C1": 1.0, "Y1": 0.5, "S1": 0.04}


def _t(**over):
    args = {
        "nominal": dict(M),
        "table": {**M, "P1": 0.9},
        "ratio_ci_hi": 0.95,
        "stratum_nominal_pos": STRATA,
        "stratum_table_pos": dict(STRATA),
        "on_bound": [],
        "p1_alt_nominal": 1.0,
        "p1_alt_table": 0.9,
        "pp_nominal": 0.3,
        "pp_table": 0.27,
    }
    args.update(over)
    return gp2.decide_gp2e_table(**args)


def _l(**over):
    args = {
        "table_passed": True,
        "t4": dict(M),
        "tl4": {**M, "P1": 0.9},
        "ratio_ci_hi": 0.95,
        "seed_p1": {0: 0.9, 1: 0.92, 2: 0.95},
        "stratum_t4_pos": STRATA,
        "stratum_tl4_pos": dict(STRATA),
        "sweep": SWEEP_OK,
        "p1_alt_t4": 1.0,
        "p1_alt_tl4": 0.9,
        "pp_t4": 0.3,
        "pp_tl4": 0.27,
        "yp_t4": 0.03,
        "yp_tl4": 0.027,
    }
    args.update(over)
    return gp2.decide_gp2e_learned(**args)


def _by(d, prefix):
    return [i for i in d.items if i.name.startswith(prefix)]


def test_clean_candidates_pass_every_item():
    assert _t().passed and _l().passed
    assert {i.name[0] for i in _t().items} == set("1234567")
    assert {i.name[0] for i in _l().items} == set("1234567")


def _m(**over):
    return {**M, "P1": 0.9, **over}


TABLE_CASES = [
    ("1a at 0.95", {"table": _m(P1=0.95)}, "1a", True),
    ("1a over", {"table": _m(P1=0.95 + EPS)}, "1a", False),
    ("1a nan", {"table": _m(P1=math.nan)}, "1a", False),
    ("1b below", {"ratio_ci_hi": 1.0 - EPS}, "1b", True),
    ("1b at 1.0 (strict)", {"ratio_ci_hi": 1.0}, "1b", False),
    ("1b nan", {"ratio_ci_hi": math.nan}, "1b", False),
    ("2a at 1.0", {"table": _m(A1=1.0)}, "2a", True),
    ("2a over", {"table": _m(A1=1.0 + EPS)}, "2a", False),
    ("2a nan", {"table": _m(A1=math.nan)}, "2a", False),
    ("2b at 1.0", {"table": _m(C1=1.0)}, "2b", True),
    ("2b over", {"table": _m(C1=1.0 + EPS)}, "2b", False),
    ("2b nan", {"table": _m(C1=math.nan)}, "2b", False),
    ("3a equal within 1e-6", {"table": _m(Y1=0.5 + 5e-7)}, "3a", True),
    ("3a over 1e-6", {"table": _m(Y1=0.5 + 2e-6)}, "3a", False),
    ("3a nan", {"table": _m(Y1=math.nan)}, "3a", False),
    ("3b equal within 1e-6", {"table": _m(S1=0.04 - 5e-7)}, "3b", True),
    ("3b over 1e-6", {"table": _m(S1=0.04 + 2e-6)}, "3b", False),
    ("3b nan", {"table": _m(S1=math.nan)}, "3b", False),
    (
        "4 at 1.05",
        {"stratum_table_pos": {"ccw:0.25-0.6": 1.05, "cw:0.85+": 2.0}},
        "4 stratum ccw",
        True,
    ),
    (
        "4 over",
        {"stratum_table_pos": {"ccw:0.25-0.6": 1.05 + 1e-6, "cw:0.85+": 2.0}},
        "4 stratum ccw",
        False,
    ),
    (
        "4 nan",
        {"stratum_table_pos": {"ccw:0.25-0.6": math.nan, "cw:0.85+": 2.0}},
        "4 stratum ccw",
        False,
    ),
    ("5 none", {"on_bound": []}, "5", True),
    ("5 one", {"on_bound": ["s+1.0000/v0.85"]}, "5", False),
    ("6 at 1.0", {"p1_alt_table": 1.0}, "6", True),
    ("6 over", {"p1_alt_table": 1.0 + EPS}, "6", False),
    ("6 nan", {"p1_alt_table": math.nan}, "6", False),
    ("7 at 1.0", {"pp_table": 0.3}, "7", True),
    ("7 over", {"pp_table": 0.3 * (1 + 1e-6)}, "7", False),
    ("7 nan", {"pp_table": math.nan}, "7", False),
    ("7 nominal nan", {"pp_nominal": math.nan}, "7", False),
]


@pytest.mark.parametrize(("label", "over", "prefix", "expected"), TABLE_CASES)
def test_gp2e_t_items(label, over, prefix, expected):
    d = _t(**over)
    items = _by(d, prefix)
    assert items and items[0].passed is expected, label
    if not expected:
        assert not d.passed


def test_gp2e_t_pp_nan_says_it_cannot_be_applied():
    assert "cannot be applied" in _by(_t(pp_table=math.nan), "7")[0].note


def _lm(**over):
    return {**M, "P1": 0.9, **over}


LEARNED_CASES = [
    ("1a at 0.95", {"tl4": _lm(P1=0.95)}, "1a", True),
    ("1a over", {"tl4": _lm(P1=0.95 + EPS)}, "1a", False),
    ("1a nan", {"tl4": _lm(P1=math.nan)}, "1a", False),
    ("1b below", {"ratio_ci_hi": 1.0 - EPS}, "1b", True),
    ("1b at 1.0", {"ratio_ci_hi": 1.0}, "1b", False),
    ("2a at 1.0", {"tl4": _lm(Y1=0.5)}, "2a", True),
    ("2a over", {"tl4": _lm(Y1=0.5 * (1 + 1e-9) + EPS)}, "2a", False),
    ("2a nan", {"tl4": _lm(Y1=math.nan)}, "2a", False),
    ("2b equal", {"tl4": _lm(S1=0.04 + 5e-7)}, "2b", True),
    ("2b over", {"tl4": _lm(S1=0.04 + 2e-6)}, "2b", False),
    ("2b nan", {"tl4": _lm(S1=math.nan)}, "2b", False),
    ("3 at 1.0", {"seed_p1": {0: 1.0, 1: 0.9}}, "3 seed 0", True),
    ("3 over", {"seed_p1": {0: 1.0 + EPS, 1: 0.9}}, "3 seed 0", False),
    ("3 nan", {"seed_p1": {0: math.nan, 1: 0.9}}, "3 seed 0", False),
    (
        "4 at 1.05",
        {"stratum_tl4_pos": {"ccw:0.25-0.6": 1.05, "cw:0.85+": 2.0}},
        "4 stratum ccw",
        True,
    ),
    (
        "4 over",
        {"stratum_tl4_pos": {"ccw:0.25-0.6": 1.05 + 1e-6, "cw:0.85+": 2.0}},
        "4 stratum ccw",
        False,
    ),
    (
        "4 nan",
        {"stratum_tl4_pos": {"ccw:0.25-0.6": math.nan, "cw:0.85+": 2.0}},
        "4 stratum ccw",
        False,
    ),
    ("5 ok", {"sweep": SWEEP_OK}, "5", True),
    ("5 bad", {"sweep": SWEEP_BAD}, "5", False),
    ("6 at 1.0", {"p1_alt_tl4": 1.0}, "6", True),
    ("6 over", {"p1_alt_tl4": 1.0 + EPS}, "6", False),
    ("6 nan", {"p1_alt_tl4": math.nan}, "6", False),
    ("7a at 1.0", {"pp_tl4": 0.3}, "7a", True),
    ("7a over", {"pp_tl4": 0.3 * (1 + 1e-6)}, "7a", False),
    ("7a nan", {"pp_tl4": math.nan}, "7a", False),
    ("7b at 1.0", {"yp_tl4": 0.03}, "7b", True),
    ("7b over", {"yp_tl4": 0.03 * (1 + 1e-6)}, "7b", False),
    ("7b nan", {"yp_tl4": math.nan}, "7b", False),
]


@pytest.mark.parametrize(("label", "over", "prefix", "expected"), LEARNED_CASES)
def test_gp2e_l_items(label, over, prefix, expected):
    items = _by(_l(**over), prefix)
    assert items and items[0].passed is expected, label


def test_gp2e_l_is_not_evaluated_when_the_table_failed_and_that_is_never_a_pass():
    d = gp2.decide_gp2e_learned(False)  # nothing else has to be computed
    assert not d.evaluated and not d.passed and d.items == ()
    assert d.to_dict()["evaluated"] is False and d.to_dict()["passed"] is False
    assert "evaluated" not in _l().to_dict()  # an evaluated decision keeps the old shape


def test_outcome_names():
    ok_t, bad_t = _t(), _t(ratio_ci_hi=1.0)
    ok_l, bad_l = _l(), _l(sweep=SWEEP_BAD)
    skipped = gp2.decide_gp2e_learned(False)
    assert gp2.gp2e_outcome(bad_t, ok_l) == gp2.ONE_SPEED_STAYS == "the one-speed contract stays"
    assert gp2.gp2e_outcome(bad_t, skipped) == gp2.ONE_SPEED_STAYS
    assert gp2.gp2e_outcome(ok_t, bad_l) == gp2.TABLE_WINS
    assert gp2.gp2e_outcome(ok_t, skipped) == gp2.TABLE_WINS
    assert gp2.gp2e_outcome(ok_t, ok_l) == gp2.TL4_CANDIDATE == "TL4 is the candidate"


def test_the_constants_match_the_frozen_yaml():
    path = repo_root() / "evaluation" / "prereg" / "pnp-v0.4-addendum-1.yaml"
    if not path.is_file():
        pytest.skip("the addendum yaml is not in this checkout (PR 189)")
    g = yaml.safe_load(path.read_text(encoding="utf-8"))["ground_motion"]
    t, lr = g["gate_table"], g["gate_learned"]
    assert t["max_p1_ratio"] == gp2.TABLE_P1_MAX_RATIO
    assert t["max_p1_ratio_bootstrap_upper"] == gp2.TABLE_P1_CI_HI_BELOW
    assert t["max_along_ratio"] == gp2.TABLE_ALONG_MAX_RATIO
    assert t["max_cross_ratio"] == gp2.TABLE_CROSS_MAX_RATIO
    assert t["channel_equal_tolerance"] == gp2.CHANNEL_EQUAL_TOLERANCE
    assert t["max_stratum_ratio"] == gp2.TABLE_STRATUM_MAX_RATIO
    assert t["max_p1_ratio_alt_truth"] == gp2.TABLE_ALT_TRUTH_MAX_RATIO
    assert t["max_pose_position_ratio"] == gp2.TABLE_PP_MAX_RATIO
    assert lr["max_p1_ratio"] == gp2.LEARNED_P1_MAX_RATIO
    assert lr["max_p1_ratio_bootstrap_upper"] == gp2.LEARNED_P1_CI_HI_BELOW
    assert lr["max_y1_ratio"] == gp2.LEARNED_Y1_MAX_RATIO
    assert lr["speed_equal_tolerance"] == gp2.CHANNEL_EQUAL_TOLERANCE
    assert lr["max_seed_p1_ratio"] == gp2.LEARNED_SEED_P1_MAX_RATIO
    assert lr["max_stratum_ratio"] == gp2.LEARNED_STRATUM_MAX_RATIO
    assert lr["max_p1_ratio_alt_truth"] == gp2.LEARNED_ALT_TRUTH_MAX_RATIO
    assert lr["max_pose_position_ratio"] == gp2.LEARNED_PP_MAX_RATIO
    assert lr["max_pose_yaw_ratio"] == gp2.LEARNED_YP_MAX_RATIO
    assert lr["sweep_samples"] == gp2.SWEEP_SAMPLES
    assert g["table"]["min_windows_per_node"] == 30
