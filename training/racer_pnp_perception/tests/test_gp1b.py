"""L1: `decide_gp1b`, items 1 to 5 of pre-registration v0.3 section 3.3, table driven: for each item
a pass, a value at the threshold, a value over it, and too few labelled scans where that applies."""

from __future__ import annotations

import copy
import math

import pytest
from racer_pnp_perception import gp1
from racer_pnp_perception.gp1 import (
    BACKEND_GEOMETRIC as GEO,
)
from racer_pnp_perception.gp1 import (
    BACKEND_NET as NET,
)
from racer_pnp_perception.gp1 import FoldResult, LabelFree, Labelled, Numbers


def free(intrusion=0.005, residual=0.05, deficit=0.02, c95=0.9, scans=1000) -> LabelFree:
    return LabelFree(scans, intrusion, residual, deficit, c95, scans)


def lab(e_c=0.05, v=16.0, scans=2000, c95=0.9) -> Labelled:
    return Labelled(scans, e_c, e_c * 1.4, v, c95)


def passing_full() -> Numbers:
    return Numbers(
        h_run={NET: lab(0.05), GEO: lab(0.136)},
        h_layout={NET: lab(0.1, 15.9, 1000), GEO: lab(0.15, 15.95, 1000)},
        a_run={NET: free(), GEO: free(0.003, 0.03, 0.02, 0.9)},
        a_layout={NET: free(), GEO: free(0.006, 0.03, 0.02, 0.9)},
        ratio_ci_hi=0.4,
        cells=[],
    )


def passing_fold(scans: int = 500) -> FoldResult:
    return FoldResult(
        {NET: lab(0.1, 16.0, scans), GEO: lab(0.15, 16.0, scans)},
        {NET: free(), GEO: free(0.005, 0.03, 0.02, 0.9)},
    )


def decide(full=None, folds=None):
    full = full or passing_full()
    folds = folds if folds is not None else {"F1": passing_fold(), "F2": passing_fold()}
    return gp1.decide_gp1b(full, folds)


def result_of(items, n):
    return next(i for i in items if i.number == n)


def test_everything_passing_is_a_pass() -> None:
    items, outcome, failed = decide()
    assert failed == [] and outcome == gp1.OUTCOME_GP1B_PASS
    assert [i.number for i in items] == [1, 2, 3, 4, 5]


# ---- item 1: v0.2 items 1, 2, 3, 6 on H-run, full model -------------------------------------


def test_item1_accuracy_at_threshold_over_and_ci() -> None:
    f = passing_full()
    f.h_run[NET] = lab(0.9 * 0.136 * 0.999)  # just under 0.90 times the tracer
    f.ratio_ci_hi = 0.999
    assert result_of(decide(f)[0], 1).passed
    f.h_run[NET] = lab(0.136 * 0.9 + 1e-6)  # over
    __items, outcome, failed = decide(f)
    assert 1 in failed and outcome == gp1.OUTCOME_GP1B_NO_BETTER
    f = passing_full()
    f.ratio_ci_hi = 1.0  # the interval must be strictly below 1.00
    assert 1 in decide(f)[2]


def test_item1_range_and_too_few_labelled_scans() -> None:
    f = passing_full()
    f.h_run[NET] = lab(0.05, 15.99)
    assert 1 in decide(f)[2]
    f = passing_full()
    f.h_run[NET] = lab(0.05, 16.0, scans=299)  # under 300 labelled scans
    f.h_run[GEO] = lab(0.136, 16.0, scans=299)
    assert 1 in decide(f)[2]
    f.h_run[NET] = lab(0.05, 16.0, scans=300)
    f.h_run[GEO] = lab(0.136, 16.0, scans=300)
    assert 1 not in decide(f)[2]


def test_item1_per_cell() -> None:
    f = passing_full()
    f.cells = [gp1.Cell("run", "ccw", "[0.2, 0.6)", 40, 1.10, 1.0)]
    assert 1 not in decide(f)[2]  # at the threshold
    f.cells = [gp1.Cell("run", "ccw", "[0.2, 0.6)", 40, 1.1001, 1.0)]
    assert 1 in decide(f)[2]
    f.cells = [gp1.Cell("run", "ccw", "[0.2, 0.6)", 39, 2.0, 1.0)]  # under 40, not judged
    assert 1 not in decide(f)[2]


# ---- item 2 and the label-free rule (items 2, 4, 5) ------------------------------------------

LABEL_FREE_CASES = [
    ("intrusion", {"intrusion": 0.02}, True),
    ("intrusion", {"intrusion": 0.0201}, False),
    ("residual", {"residual": 0.10}, True),
    ("residual", {"residual": 0.1001}, False),
    ("deficit", {"deficit": 0.05}, True),
    ("deficit", {"deficit": 0.0501}, False),
    ("c95", {"c95": 0.88}, True),
    ("c95", {"c95": 0.8799}, False),
]


@pytest.mark.parametrize(("name", "kw", "ok"), LABEL_FREE_CASES)
def test_item2_label_free_on_a_run(name: str, kw: dict, ok: bool) -> None:
    f = passing_full()
    f.a_run[NET] = free(**{"intrusion": 0.0, **kw})
    items, outcome, _failed = decide(f)
    assert result_of(items, 2).passed is ok
    if not ok:
        assert outcome == gp1.OUTCOME_GP1B_NO_BETTER


def test_item2_intrusion_limit_follows_the_tracer_above_the_floor() -> None:
    f = passing_full()
    f.a_run[GEO] = free(0.05)
    f.a_run[NET] = free(0.055)  # geometric + 0.005
    assert result_of(decide(f)[0], 2).passed
    f.a_run[NET] = free(0.0551)
    assert not result_of(decide(f)[0], 2).passed


@pytest.mark.parametrize(("name", "kw", "ok"), LABEL_FREE_CASES)
def test_item4_label_free_in_each_fold(name: str, kw: dict, ok: bool) -> None:
    folds = {"F1": passing_fold(), "F2": passing_fold()}
    folds["F2"].free[NET] = free(**{"intrusion": 0.0, **kw})
    items, outcome, failed = decide(folds=folds)
    assert result_of(items, 4).passed is ok
    if not ok:
        assert outcome == gp1.OUTCOME_GP1B_NOT_ELSEWHERE and failed == [4]


@pytest.mark.parametrize(("name", "kw", "ok"), LABEL_FREE_CASES)
def test_item5_label_free_on_the_2026_11_12_layout(name: str, kw: dict, ok: bool) -> None:
    f = passing_full()
    f.a_layout[NET] = free(**{"intrusion": 0.0, **kw})
    items, outcome, failed = decide(f)
    assert result_of(items, 5).passed is ok
    assert "development data" in result_of(items, 5).note
    if not ok:
        assert outcome == gp1.OUTCOME_GP1B_NOT_ELSEWHERE and failed == [5]


# ---- item 3: folds with at least 100 labelled scans ------------------------------------------


def test_item3_accuracy_pass_at_threshold_and_over() -> None:
    folds = {"F1": passing_fold(), "F2": passing_fold()}
    folds["F1"].labelled[NET] = lab(0.15)  # ratio exactly 1.00
    assert result_of(decide(folds=folds)[0], 3).passed
    folds["F1"].labelled[NET] = lab(0.1501)
    _items, outcome, failed = decide(folds=folds)
    assert failed == [3] and outcome == gp1.OUTCOME_GP1B_NOT_ELSEWHERE


def test_item3_range_at_threshold_and_over() -> None:
    folds = {"F1": passing_fold(), "F2": passing_fold()}
    folds["F2"].labelled[NET] = lab(0.1, 15.5, 500)  # V(geometric) 16.0 minus 0.5
    assert result_of(decide(folds=folds)[0], 3).passed
    folds["F2"].labelled[NET] = lab(0.1, 15.49, 500)
    assert 3 in decide(folds=folds)[2]


def test_item3_a_fold_with_too_few_labelled_scans_is_not_judged_and_says_so() -> None:
    folds = {"F1": passing_fold(99), "F2": passing_fold()}
    folds["F1"].labelled[NET] = lab(5.0, 0.0, 99)  # terrible, but under 100 scans
    items, _o, failed = decide(folds=folds)
    it = result_of(items, 3)
    assert it.passed and 3 not in failed
    assert "F1" in it.note and it.value["F1"]["judged"] is False
    folds["F1"] = passing_fold(100)
    folds["F1"].labelled[NET] = lab(5.0, 0.0, 100)  # exactly 100: judged
    assert 3 in decide(folds=folds)[2]


def test_item3_nan_fails() -> None:
    folds = {"F1": passing_fold(), "F2": passing_fold()}
    folds["F1"].labelled[NET] = lab(math.nan)
    assert 3 in decide(folds=folds)[2]


def test_items_1_and_2_failing_beat_3_to_5_in_the_outcome() -> None:
    f = passing_full()
    f.a_run[NET] = free(0.5)
    folds = {"F1": passing_fold(), "F2": passing_fold()}
    folds["F1"].free[NET] = free(0.5)
    _items, outcome, failed = decide(f, folds)
    assert failed == [2, 4] and outcome == gp1.OUTCOME_GP1B_NO_BETTER


def test_thresholds_are_the_v02_constants_not_retyped() -> None:
    assert gp1.ITEM5_INTRUSION_FLOOR == 0.02 and gp1.ITEM4_MAX_RATIO == 1.00
    assert gp1.ITEM4_V_MARGIN == 0.5 and gp1.MIN_H_LAYOUT_LABELS == 100
    assert copy.deepcopy(gp1.GP1B_ITEM1_V02_ITEMS) == (1, 2, 3, 6)


def test_deficit_by_construction_flag_adds_the_note_and_changes_no_decision() -> None:
    plain = decide()
    flagged = gp1.decide_gp1b(
        passing_full(), {"F1": passing_fold(), "F2": passing_fold()}, deficit_by_construction=True
    )
    assert plain[1:] == flagged[1:]
    for i in flagged[0]:
        has = gp1.DEFICIT_BY_CONSTRUCTION_NOTE in i.note
        assert has is (i.number in (2, 4, 5))
    assert all(gp1.DEFICIT_BY_CONSTRUCTION_NOTE not in i.note for i in plain[0])
