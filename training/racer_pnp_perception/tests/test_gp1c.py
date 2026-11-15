"""L1: `decide_gp1c`, items 1 to 8 of pre-registration v0.4 section 3.2, table driven: for each item
a pass, a value at the threshold, a value over it, and too few labelled scans where that applies."""

from __future__ import annotations

import math

import numpy as np
from racer_pnp_perception import gp1, gp1c
from racer_pnp_perception.gp1 import BACKEND_GEOMETRIC as GEO
from racer_pnp_perception.gp1 import BACKEND_NET as NET
from racer_pnp_perception.gp1 import Cell, LabelFree, Labelled
from racer_pnp_perception.gp1c import CanaryCase, DevResult


def free(intrusion=0.005, residual=0.05, deficit=0.02, c95=0.9, scans=3000) -> LabelFree:
    return LabelFree(scans, intrusion, residual, deficit, c95, scans)


def lab(e_c=0.05, v=16.0, scans=800, c95=0.9) -> Labelled:
    return Labelled(scans, e_c, e_c * 1.4, v, c95)


def inputs() -> dict:
    return {
        "h14": {NET: lab(0.06), GEO: lab(0.136)},
        "a14": {NET: free(), GEO: free(0.003, 0.03, 0.02, 0.9)},
        "ratio_ci_hi": 0.6,
        "cells": [Cell("run", "ccw", "0.85+", 100, 0.06, 0.136)],
        "dev": DevResult(True, [], "pass"),
        "latency_p99_ms": 6.0,
        "canary": [CanaryCase("stadium-ccw", True, 0, 10.2, 10.0)],
    }


def decide(**over):
    d = {**inputs(), **over}
    return gp1c.decide_gp1c(
        d["h14"],
        d["a14"],
        d["ratio_ci_hi"],
        d["cells"],
        d["dev"],
        d["latency_p99_ms"],
        d["canary"],
    )


def item(items, n):
    return next(i for i in items if i.number == n)


def test_everything_passing_is_a_pass_with_eight_items() -> None:
    items, outcome, failed = decide()
    assert failed == [] and outcome == gp1c.OUTCOME_PASS
    assert [i.number for i in items] == list(range(1, 9))


def test_items_3_and_5_carry_the_by_construction_flag() -> None:
    items, _o, _f = decide()
    for n in (3, 5):
        assert "by construction" in item(items, n).note
    for n in (1, 2, 4, 6, 7, 8):
        assert "by construction" not in item(items, n).note


# ---- item 1 ---------------------------------------------------------------------------------


def test_item1_too_few_labelled_scans_is_undecided() -> None:
    h = {NET: lab(0.06, scans=299), GEO: lab(0.136, scans=299)}
    _items, outcome, failed = decide(h14=h)
    assert 1 in failed and outcome == gp1c.OUTCOME_UNDECIDED
    h = {NET: lab(0.06, scans=300), GEO: lab(0.136, scans=300)}
    assert item(decide(h14=h)[0], 1).passed  # at the threshold


# ---- item 2 ---------------------------------------------------------------------------------


def test_item2_ratio_at_threshold_over_and_ci() -> None:
    geo = 0.136
    h = {NET: lab(geo * 0.9), GEO: lab(geo)}
    assert item(decide(h14=h)[0], 2).passed  # at 0.90
    h = {NET: lab(geo * 0.9 + 1e-6), GEO: lab(geo)}
    __items, outcome, failed = decide(h14=h)
    assert 2 in failed and outcome == gp1c.OUTCOME_NO_BETTER
    assert 2 in decide(ratio_ci_hi=1.0)[2]  # the interval must be strictly below 1.00
    assert 2 not in decide(ratio_ci_hi=0.999)[2]


# ---- item 3 ---------------------------------------------------------------------------------


def test_item3_range_at_threshold_and_over() -> None:
    h = {NET: lab(0.06, v=16.0), GEO: lab(0.136, v=16.0)}
    assert item(decide(h14=h)[0], 3).passed
    h = {NET: lab(0.06, v=15.99), GEO: lab(0.136, v=16.0)}
    __items, outcome, failed = decide(h14=h)
    assert 3 in failed and outcome == gp1c.OUTCOME_NO_BETTER


# ---- item 4 ---------------------------------------------------------------------------------


def test_item4_cells_at_threshold_over_and_too_few() -> None:
    def cell(n, net, geo):
        return Cell("run", "cw", "[0.2, 0.6)", n, net, geo)

    assert item(decide(cells=[cell(40, 0.11, 0.1)])[0], 4).passed  # ratio 1.10 at threshold
    __items, outcome, failed = decide(cells=[cell(40, 0.1101, 0.1)])
    assert 4 in failed and outcome == gp1c.OUTCOME_NOT_ELSEWHERE
    items, _o, failed = decide(cells=[cell(39, 0.5, 0.1)])  # too few labelled scans: not judged
    assert 4 not in failed and item(items, 4).value["cells_not_judged"] == 1
    items, _o, failed = decide(cells=[])
    assert 4 not in failed and "nothing judged" in item(items, 4).note


# ---- item 5 ---------------------------------------------------------------------------------


def test_item5_each_condition_at_threshold_and_over() -> None:
    geo = free(0.003, 0.03, 0.02, 0.9)
    a = {GEO: geo}
    cases = {
        "intrusion": ({"intrusion": max(0.02, 0.003 + 0.005)}, {"intrusion": 0.0200001}),
        "residual": ({"residual": 0.10}, {"residual": 0.1001}),
        "deficit": ({"deficit": 0.05}, {"deficit": 0.0501}),
        "c95": ({"c95": 0.88}, {"c95": 0.879}),
    }
    for ok, bad in cases.values():
        assert item(decide(a14={**a, NET: free(**ok)})[0], 5).passed, ok
        __items, outcome, failed = decide(a14={**a, NET: free(**bad)})
        assert 5 in failed and outcome == gp1c.OUTCOME_NOT_ELSEWHERE, bad


def test_item5_intrusion_limit_follows_the_geometric_value() -> None:
    geo = free(0.03, 0.03, 0.02, 0.9)  # 0.03 + 0.005 = 0.035 above the 0.02 floor
    assert item(decide(a14={GEO: geo, NET: free(0.03 + 0.005)})[0], 5).passed
    assert not item(decide(a14={GEO: geo, NET: free(0.0351)})[0], 5).passed


# ---- item 6 ---------------------------------------------------------------------------------


def test_item6_follows_the_development_rule_and_is_flagged_development() -> None:
    items, _o, failed = decide(dev=DevResult(False, [3], "x", "fold F1 range"))
    assert 6 in failed
    assert "development" in item(items, 6).note
    assert item(decide(dev=DevResult(True, [], "pass"))[0], 6).passed


def test_item6_not_computed_does_not_pass() -> None:
    items, outcome, failed = decide(dev=None)
    assert 6 in failed and "not measured" in item(items, 6).note
    assert outcome == gp1c.OUTCOME_NOT_ELSEWHERE


def test_dev_result_from_gp1b_items() -> None:
    it = [gp1.Item(1, "a", {}, {}, True), gp1.Item(2, "b", {}, {}, False, "note")]
    r = gp1c.dev_result_from_gp1b(it, [2])
    assert not r.passed and r.failed_items == [2] and "item 2: note" in r.note
    assert gp1c.dev_result_from_gp1b(it[:1], []).passed


# ---- item 7 ---------------------------------------------------------------------------------


def test_item7_latency_under_at_and_over_ten_ms() -> None:
    assert item(decide(latency_p99_ms=9.999)[0], 7).passed
    assert not item(decide(latency_p99_ms=10.0)[0], 7).passed  # "under 10 ms": 10 fails
    __items, outcome, failed = decide(latency_p99_ms=12.0)
    assert 7 in failed and outcome == gp1c.OUTCOME_NOT_ELSEWHERE
    items, _o, failed = decide(latency_p99_ms=None)
    assert 7 in failed and "not measured" in item(items, 7).note


# ---- item 8 ---------------------------------------------------------------------------------


def test_item8_canary_at_threshold_over_contact_and_incomplete() -> None:
    def case(fused=10.3, geo=10.0, contacts=0, done=True, name="c"):
        return CanaryCase(name, done, contacts, fused, geo)

    assert item(decide(canary=[case(10.3)])[0], 8).passed  # 1.03 at threshold
    assert not item(decide(canary=[case(10.31)])[0], 8).passed
    assert not item(decide(canary=[case(contacts=1)])[0], 8).passed
    assert not item(decide(canary=[case(done=False)])[0], 8).passed
    assert not item(decide(canary=[case(), case(10.5, name="d")])[0], 8).passed  # every case
    assert not item(decide(canary=[case(geo=0.0)])[0], 8).passed
    items, _o, failed = decide(canary=None)
    assert 8 in failed and "not measured" in item(items, 8).note


def test_canary_ratio_and_median_lap() -> None:
    assert math.isclose(CanaryCase("x", True, 0, 11.0, 10.0).ratio, 1.1)
    assert gp1c.median_lap_s([5.0, 7.0, 6.0]) == 6.0
    assert math.isnan(gp1c.median_lap_s([]))


# ---- outcome naming -------------------------------------------------------------------------


def test_outcome_precedence() -> None:
    bad_h = {NET: lab(0.136), GEO: lab(0.136)}  # item 2 fails
    __i, outcome, failed = decide(h14=bad_h, latency_p99_ms=50.0)
    assert 2 in failed and 7 in failed and outcome == gp1c.OUTCOME_NO_BETTER
    __i, outcome, failed = decide(latency_p99_ms=50.0)
    assert failed == [7] and outcome == gp1c.OUTCOME_NOT_ELSEWHERE
    few = {NET: lab(0.136, scans=10), GEO: lab(0.136, scans=10)}
    assert decide(h14=few)[1] == gp1c.OUTCOME_UNDECIDED


def synthetic_rows() -> list[gp1.ScanRow]:
    rows = []
    rng = np.random.default_rng(0)
    for k in range(400):
        r = gp1.ScanRow(
            bag="b",
            index=k,
            set=gp1.SET_RUN,
            segment="K1",
            segment_from_s=0.0,
            t_s=k * 0.1,
            speed_mps=0.9,
            direction="ccw",
            path=None,
            label=np.zeros((16, 5), dtype=np.float32),
            label_valid=16,
        )
        for b, e in (
            ("geometric", 0.14),
            (gp1c.FUSED, 0.06),
            ("net", 0.07),
            ("net_unrefined", 0.05),
        ):
            r.e_c[b] = e + 0.01 * rng.random()
            r.e_far[b] = r.e_c[b]
            r.valid[b] = 16
            r.intrusion[b] = 0.0
            r.residual[b] = 0.03
            r.deficit[b] = np.array([0.0, 0.01])
            r.containment[b] = 1.0
        rows.append(r)
    return rows


def test_the_report_is_strict_json_with_the_four_backends_side_by_side() -> None:
    out = gp1c.gp1c_report(synthetic_rows(), DevResult(True, [], "pass"), 5.0, None)
    import json

    json.dumps(out, allow_nan=False)
    assert set(out["M-A1"]["H14"]) == {"geometric", "fused", "net", "net_unrefined"}
    assert set(out["M-A1"]["A14"]) == set(out["M-A1"]["H14"])
    assert out["failed_items"] == [8]  # no canary given: not measured, not a pass
    assert out["sets"] == {"H14_labelled": 400, "A14_moving": 400}
    assert "GP1c outcome" in gp1c.format_report(out)
