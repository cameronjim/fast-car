"""L1 for the GP2f decisions (gp2f.py): a table-driven test per item of every gate (pass, at the
threshold, epsilon over, NaN, cannot be applied), GP2f-L not evaluated when GP2f-T failed,
GP2f-V staying at 1.3 on a missing item, the outcome names, and the thresholds against the yaml."""

from __future__ import annotations

import copy
from dataclasses import replace

import numpy as np
import pytest
from racer_pnp_dynamics import gp2, gp2f
from test_step_v21 import C5

NAN = float("nan")
EPS = 1e-9
G = C5.doc


def metrics(scale: float = 1.0, **over) -> gp2f.Metrics:
    """A model whose every error is `scale` times the reference's (1.0 everywhere)."""
    base = gp2f.Metrics(
        macro={"P1": scale, "A1": scale, "C1": scale, "Y1": scale, "S1": 0.03},
        stratum_pos={"ccw:0.25-0.6": scale, "cw:0.85+": scale},
        direction={"ccw": {"P1": scale, "Y1": scale}, "cw": {"P1": scale, "Y1": scale}},
        yaw_bias={"ccw": 0.01 * scale, "cw": -0.02 * scale},
        near_limit={"left": (40, scale), "right": (50, scale)},
        transition=(60, scale),
        p1_alt=scale,
        pp=scale,
        yp=scale,
        fast={
            1.15: {"ccw": (40, scale), "cw": (35, scale)},
            1.3: {"ccw": (31, scale), "cw": (30, scale)},
        },
    )
    return replace(base, **over)


REF = metrics(1.0)


def item(decision: gp2.Decision, prefix: str) -> gp2.Item:
    found = [i for i in decision.items if i.name.startswith(prefix)]
    assert len(found) == 1, (prefix, [i.name for i in decision.items])
    return found[0]


def table(cand: gp2f.Metrics, support_passed: bool = True, ci_hi: float = 0.8, ref=REF):
    return gp2f.decide_gp2f_table(C5, support_passed, ref, cand, ci_hi)


# ---- the thresholds are the yaml's --------------------------------------------------------------


def test_the_table_gate_reads_its_numbers_from_the_registered_yaml():
    d = table(metrics(0.5))
    assert d.passed
    g = G["gate_table"]
    assert item(d, "1a").threshold == g["max_p1_ratio"] == 0.90
    assert item(d, "1b").threshold == g["max_p1_ratio_bootstrap_upper"] == 0.95
    assert item(d, "3a").threshold == g["max_y1_ratio"] == 0.90
    assert item(d, "4 stratum ccw:0.25-0.6").threshold == g["max_stratum_ratio"] == 1.05
    assert item(d, "5 ccw P1").threshold == g["per_direction"]["max_p1_ratio"] == 0.95
    assert item(d, "5 ccw Y1").threshold == g["per_direction"]["max_y1_ratio"] == 1.00
    assert item(d, "9").threshold == g["max_p1_ratio_alt_truth"] == 1.00


# ---- GP2f-T, item by item -------------------------------------------------------------------------


def with_macro(**kv) -> gp2f.Metrics:
    return metrics(0.5, macro={**metrics(0.5).macro, **kv})


CASES_T = [
    # (item prefix, builder of the candidate, threshold exact value, pass at, over)
    ("1a", lambda x: with_macro(P1=x), 0.90),
    ("2a", lambda x: with_macro(A1=x), 1.00),
    ("2b", lambda x: with_macro(C1=x), 1.00),
    ("3a", lambda x: with_macro(Y1=x), 0.90),
    ("9", lambda x: replace(metrics(0.5), p1_alt=x), 1.00),
    ("10a", lambda x: replace(metrics(0.5), pp=x), 1.00),
    ("10b", lambda x: replace(metrics(0.5), yp=x), 1.00),
    (
        "4 stratum cw:0.85+",
        lambda x: replace(metrics(0.5), stratum_pos={"ccw:0.25-0.6": 0.5, "cw:0.85+": x}),
        1.05,
    ),
    (
        "5 cw P1",
        lambda x: replace(
            metrics(0.5), direction={"ccw": {"P1": 0.5, "Y1": 0.5}, "cw": {"P1": x, "Y1": 0.5}}
        ),
        0.95,
    ),
    (
        "5 ccw Y1",
        lambda x: replace(
            metrics(0.5), direction={"ccw": {"P1": 0.5, "Y1": x}, "cw": {"P1": 0.5, "Y1": 0.5}}
        ),
        1.00,
    ),
    (
        "7 near-limit left",
        lambda x: replace(metrics(0.5), near_limit={"left": (40, x), "right": (50, 0.5)}),
        1.00,
    ),
    (
        "7 near-limit right",
        lambda x: replace(metrics(0.5), near_limit={"left": (40, 0.5), "right": (50, x)}),
        1.00,
    ),
    ("8", lambda x: replace(metrics(0.5), transition=(60, x)), 1.00),
]


@pytest.mark.parametrize(("prefix", "build", "limit"), CASES_T)
def test_every_ratio_item_of_gp2f_t_passes_at_its_threshold_and_fails_just_over_or_on_nan(
    prefix, build, limit
):
    at = table(build(limit))  # a reference of 1.0, so the ratio is the value itself
    assert item(at, prefix).passed, item(at, prefix)
    over = table(build(limit + EPS))
    assert not item(over, prefix).passed and not over.passed
    nan = table(build(NAN))
    assert not item(nan, prefix).passed and not nan.passed
    assert item(table(build(limit / 2)), prefix).passed


def test_the_bootstrap_item_is_strict_and_the_s1_item_is_an_equality():
    d = table(metrics(0.5), ci_hi=0.95)
    assert not item(d, "1b").passed  # strictly below 0.95
    assert table(metrics(0.5), ci_hi=0.95 - EPS).passed
    assert not table(metrics(0.5), ci_hi=NAN).passed
    tol = G["gate_table"]["speed_equal_tolerance"]
    ok = table(with_macro(S1=0.03 + 0.5 * tol))
    assert item(ok, "3b").passed
    assert not table(with_macro(S1=0.03 + 2 * tol)).passed
    assert not table(with_macro(S1=NAN)).passed


def test_gp2f_t_needs_gp2f_s_and_still_prints_every_item():
    d = table(metrics(0.5), support_passed=False)
    assert not d.passed and not item(d, "0").passed
    assert len(d.items) >= 20 and all(i.passed for i in d.items if not i.name.startswith("0"))


def test_yaw_bias_passes_under_the_absolute_limit_or_at_half_of_the_reference():
    ref = metrics(1.0, yaw_bias={"ccw": 0.20, "cw": -0.01})
    absolute = G["gate_table"]["yaw_bias"]["max_abs_radps"]

    def bias(ccw, cw):
        return replace(metrics(0.5), yaw_bias={"ccw": ccw, "cw": cw})

    assert item(table(bias(absolute, -0.03), ref=ref), "6 ccw").passed  # at the absolute limit
    assert not item(table(bias(absolute + 1e-6, 0.0), ref=ref), "6 ccw").value <= absolute
    assert item(table(bias(0.10, 0.0), ref=ref), "6 ccw").passed  # half of the 0.20 reference
    assert not item(table(bias(0.10 + 1e-6, 0.0), ref=ref), "6 ccw").passed
    # cw: reference 0.01: half is 0.005, below the absolute limit, which therefore decides
    assert item(table(bias(0.0, -0.03), ref=ref), "6 cw").passed
    assert not item(table(bias(0.0, 0.03 + 1e-6), ref=ref), "6 cw").passed
    assert not item(table(bias(0.0, NAN), ref=ref), "6 cw").passed


@pytest.mark.parametrize("side", ["left", "right"])
def test_near_limit_windows_below_the_minimum_are_cannot_be_applied_not_a_failure(side):
    other = "right" if side == "left" else "left"
    thin = replace(metrics(0.5), near_limit={side: (29, 9.0), other: (50, 0.5)})
    d = table(thin)
    it = item(d, f"7 near-limit {side}")
    assert it.comparison == gp2f.NA and it.passed and "cannot be applied: 29" in it.note
    assert d.passed
    at_min = replace(metrics(0.5), near_limit={side: (30, 9.0), other: (50, 0.5)})
    assert not table(at_min).passed  # 30 windows: applied, and 9.0 against 1.0 fails


def test_transition_windows_below_the_minimum_cannot_be_applied():
    d = table(replace(metrics(0.5), transition=(29, 9.0)))
    assert item(d, "8").comparison == gp2f.NA and d.passed
    assert not table(replace(metrics(0.5), transition=(30, 9.0))).passed


def test_a_missing_direction_cannot_pass_the_items_that_can_always_be_applied():
    cand = replace(metrics(0.5), direction={"ccw": {"P1": 0.5, "Y1": 0.5}})
    d = table(cand)
    assert not item(d, "5 cw P1").passed and not item(d, "5 cw Y1").passed and not d.passed
    cand = replace(metrics(0.5), yaw_bias={"ccw": 0.0})
    assert not item(table(cand), "6 cw").passed


def test_the_reference_stratum_set_decides_which_strata_are_checked():
    cand = replace(metrics(0.5), stratum_pos={"ccw:0.25-0.6": 0.5})  # a stratum missing
    d = table(cand)
    assert not item(d, "4 stratum cw:0.85+").passed


# ---- GP2f-S ---------------------------------------------------------------------------------------


def support(on_bound=(), a=0.0, g=0.0, share=1.0):
    return gp2f.decide_gp2f_support(C5, list(on_bound), a, g, share)


def test_gp2f_s_items_pass_at_threshold_and_fail_over_or_on_nan():
    s = G["gate_support"]
    assert support().passed
    assert item(support(on_bound=["s+0.5000/v1.5"]), "1").passed is False
    assert item(support(on_bound=["s+0.5000/v1.5"]), "1").note == "s+0.5000/v1.5"
    a, g = s["max_split_half_difference_alpha"], s["max_split_half_difference_gamma"]
    assert (a, g) == (0.05, 0.08)
    assert support(a=a, g=g).passed
    assert not support(a=a + EPS).passed and not support(g=g + EPS).passed
    assert not support(a=NAN).passed and not support(g=NAN).passed
    share = s["min_heldout_step_share_in_supported_cells"]
    assert share == 0.95
    assert support(share=share).passed
    assert not support(share=share - EPS).passed
    assert not support(share=NAN).passed
    assert item(support(), "3").comparison == ">="


# ---- GP2f-L ---------------------------------------------------------------------------------------


def learned(cand: gp2f.Metrics, ref: gp2f.Metrics = REF, ci=0.9, seeds=None, sweep=0, sat=0.0):
    return gp2f.decide_gp2f_learned(
        C5,
        True,
        ref,
        cand,
        ci,
        seeds if seeds is not None else {0: 0.9, 1: 0.9, 2: 0.9},
        sweep,
        100000,
        sat,
    )


def test_gp2f_l_is_not_evaluated_when_gp2f_t_failed_and_is_never_a_pass():
    d = gp2f.decide_gp2f_learned(C5, False)
    assert d.evaluated is False and not d.passed and d.items == ()
    assert d.to_dict()["evaluated"] is False
    # every other argument is ignored, so none of them has to exist
    assert gp2f.decide_gp2f_learned(C5, False, None, None, NAN, None, None).passed is False


CASES_L = [
    ("1a", lambda x: with_macro(P1=x), 0.95),
    ("2a", lambda x: with_macro(Y1=x), 1.00),
    ("8", lambda x: replace(metrics(0.5), p1_alt=x), 1.00),
    ("9a", lambda x: replace(metrics(0.5), pp=x), 1.00),
    ("9b", lambda x: replace(metrics(0.5), yp=x), 1.00),
    (
        "4 stratum ccw",
        lambda x: replace(metrics(0.5), stratum_pos={"ccw:0.25-0.6": x, "cw:0.85+": 0.5}),
        1.05,
    ),
    (
        "4 cw P1",
        lambda x: replace(
            metrics(0.5), direction={"ccw": {"P1": 0.5, "Y1": 0.5}, "cw": {"P1": x, "Y1": 0.5}}
        ),
        1.00,
    ),
    (
        "5 near-limit left",
        lambda x: replace(metrics(0.5), near_limit={"left": (40, x), "right": (50, 0.5)}),
        1.05,
    ),
    ("5 transition", lambda x: replace(metrics(0.5), transition=(60, x)), 1.05),
]


@pytest.mark.parametrize(("prefix", "build", "limit"), CASES_L)
def test_every_ratio_item_of_gp2f_l_passes_at_its_threshold_and_fails_just_over_or_on_nan(
    prefix, build, limit
):
    assert item(learned(build(limit)), prefix).passed
    assert not item(learned(build(limit + EPS)), prefix).passed
    assert not item(learned(build(NAN)), prefix).passed


def test_gp2f_l_seed_sweep_saturation_and_bootstrap_items():
    cand = metrics(0.5)
    assert learned(cand).passed
    assert item(learned(cand, ci=1.0), "1b").passed is False  # strictly below 1.00
    assert item(learned(cand, ci=1.0 - EPS), "1b").passed
    assert item(learned(cand, seeds={0: 1.0, 1: 0.5}), "3 seed 0").passed  # at 1.00
    assert not item(learned(cand, seeds={0: 1.0 + EPS}), "3 seed 0").passed
    assert not item(learned(cand, seeds={0: NAN}), "3 seed 0").passed
    assert not learned(cand, sweep=1).passed and item(learned(cand, sweep=0), "6").passed
    assert item(learned(cand), "6").note == "100000 samples"
    sat = G["gate_learned"]["saturation"]["max_heldout_step_share"]
    assert sat == 0.01
    assert item(learned(cand, sat=sat), "7").passed
    assert not item(learned(cand, sat=sat + EPS), "7").passed
    assert not item(learned(cand, sat=NAN), "7").passed
    s1 = G["gate_learned"]["speed_equal_tolerance"]
    assert item(learned(with_macro(S1=0.03 + 0.5 * s1)), "2b").passed
    assert not item(learned(with_macro(S1=0.03 + 2 * s1)), "2b").passed
    not_run = gp2f.decide_gp2f_learned(C5, True, REF, cand, 0.9, {0: 0.9}, None, 0, 0.0)
    assert not item(not_run, "6").passed and item(not_run, "6").note == "not run"


def test_gp2f_l_cannot_be_applied_items():
    d = learned(
        replace(
            metrics(0.5), near_limit={"left": (3, 9.0), "right": (50, 0.5)}, transition=(0, 9.0)
        )
    )
    assert item(d, "5 near-limit left").comparison == gp2f.NA
    assert item(d, "5 transition").comparison == gp2f.NA
    assert d.passed


# ---- GP2f-V ---------------------------------------------------------------------------------------


FAST = {
    1.15: {"ccw": (40, 1.0, 1.0), "cw": (35, 0.9, 1.0)},
    1.30: {"ccw": (31, 0.8, 1.0), "cw": (30, 1.0, 1.0)},
}


def validity(gate=True, rule=1.5, top=1.5, fast=None):
    return gp2f.decide_gp2f_validity(C5, gate, rule, top, FAST if fast is None else fast)


def test_validity_is_1_3_unless_every_item_holds_and_then_the_smallest_of_three():
    v = validity()
    assert v.v_max_mps == 1.5 and v.exceeds_base and v.applied and "every item holds" in v.note
    assert validity(rule=1.42).v_max_mps == 1.42
    assert validity(top=1.3).v_max_mps == 1.3 and not validity(top=1.3).exceeds_base
    assert validity(top=1.1).v_max_mps == 1.1  # the literal rule: smaller than 1.3 if support ends
    nan = validity(rule=NAN)
    assert np.isnan(nan.v_max_mps) and not nan.exceeds_base


def test_validity_stays_at_1_3_when_a_gate_or_an_item_fails_or_is_missing():
    assert validity(gate=False).v_max_mps == 1.3
    assert "failed its gate" in validity(gate=False).note
    over = {**FAST, 1.15: {**FAST[1.15], "ccw": (40, 1.0 + EPS, 1.0)}}
    assert validity(fast=over).v_max_mps == 1.3 and "fast item failed" in validity(fast=over).note
    at = {**FAST, 1.15: {**FAST[1.15], "ccw": (40, 1.0, 1.0)}}
    assert validity(fast=at).v_max_mps == 1.5  # at 1.00 holds
    for speed in FAST:
        for direction in gp2f.DIRECTIONS:
            thin = copy.deepcopy(FAST)
            thin[speed][direction] = (29, 0.1, 1.0)  # one window short: cannot be applied
            v = validity(fast=thin)
            assert v.v_max_mps == 1.3 and not v.applied
            assert gp2.FAST_CANNOT_BE_APPLIED in v.note
            missing = copy.deepcopy(FAST)
            del missing[speed][direction]  # the item does not exist at all
            assert validity(fast=missing).v_max_mps == 1.3
    nan = {**FAST, 1.30: {**FAST[1.30], "cw": (30, NAN, 1.0)}}
    assert validity(fast=nan).v_max_mps == 1.3
    assert validity(fast={}).v_max_mps == 1.3
    assert validity(top=None).v_max_mps == 1.3 and "no top supported" in validity(top=None).note


def test_validity_reads_the_registered_numbers():
    g = G["gate_validity"]
    assert (g["cap_mps"], g["default_mps"]) == (1.5, 1.3)
    assert [f["min_start_speed_mps"] for f in g["fast_items"]] == [1.15, 1.30]
    assert all(
        f["min_windows_per_direction"] == 30 and f["max_ratio"] == 1.0 for f in g["fast_items"]
    )


def grid(true_cells: set[tuple[int, int]]) -> np.ndarray:
    out = np.zeros((5, 6), dtype=bool)
    for i, j in true_cells:
        out[i, j] = True
    return out


def full(top_col: int) -> np.ndarray:
    """Supported at share +-0.5 and +-1 for the speed columns 1 .. top_col (0.6 up)."""
    return grid({(i, j) for i in (0, 1, 3, 4) for j in range(1, top_col + 1)})


def test_top_supported_speed_needs_both_tables_both_sides_and_every_lower_node():
    assert gp2f.top_supported_speed(C5, full(5), full(5)) == 1.5
    assert gp2f.top_supported_speed(C5, full(4), full(5)) == 1.3  # one table stops at 1.3
    assert gp2f.top_supported_speed(C5, full(2), full(2)) == 0.85
    assert gp2f.top_supported_speed(C5, full(0), full(0)) is None  # nothing from 0.6 up
    # a hole at 1.1 (column 3) in one table: the top is the node below it, whatever lies above
    holes = full(5)
    holes[[0, 1, 3, 4], 3] = False
    assert gp2f.top_supported_speed(C5, holes, full(5)) == 0.85
    # the 0.35 column does not count
    only_low = grid({(i, 0) for i in (0, 1, 3, 4)})
    assert gp2f.top_supported_speed(C5, only_low, only_low) is None
    # one side missing at a speed ends the chain; the centre column never counts
    one_side = full(5)
    one_side[[0, 1], 3] = False  # no right-hand node at 1.1
    assert gp2f.top_supported_speed(C5, one_side, full(5)) == 0.85
    centre_only = grid({(2, j) for j in range(1, 6)})
    assert gp2f.top_supported_speed(C5, centre_only, centre_only) is None
    # either of the two turning shares is enough on a side
    half_only = grid({(i, j) for i in (1, 3) for j in range(1, 6)})
    assert gp2f.top_supported_speed(C5, half_only, half_only) == 1.5
    lock_only = grid({(i, j) for i in (0, 4) for j in range(1, 6)})
    assert gp2f.top_supported_speed(C5, lock_only, lock_only) == 1.5


# ---- outcome --------------------------------------------------------------------------------------


def dec(passed: bool, evaluated: bool = True) -> gp2.Decision:
    items = (gp2.Item("x", 0.0, "<=", 1.0, passed),) if evaluated else ()
    return gp2.Decision("g", items, evaluated)


def test_outcome_names():
    assert gp2f.gp2f_outcome(dec(False), dec(True), dec(True)) == gp2.ONE_SPEED_STAYS
    assert gp2f.gp2f_outcome(dec(True), dec(False), dec(False, False)) == gp2.ONE_SPEED_STAYS
    assert gp2f.gp2f_outcome(dec(True), dec(True), dec(False)) == gp2f.T5_CANDIDATE
    assert gp2f.gp2f_outcome(dec(True), dec(True), dec(False, evaluated=False)) == gp2f.T5_CANDIDATE
    assert gp2f.gp2f_outcome(dec(True), dec(True), dec(True)) == gp2f.TL5_CANDIDATE
