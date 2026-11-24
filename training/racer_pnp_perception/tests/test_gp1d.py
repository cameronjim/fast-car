"""L1: `decide_gp1d`, `decide_gp1d_k`, the global items and the outcome rule of the pre-registration
pnp-v0.6-perception-fused: every item at pass, at threshold and over; too few labelled scans; one set
passing and one failing; thresholds read from the yaml and compared with GP1c's constants; the
data-role guards."""

from __future__ import annotations

import hashlib
import shutil
from pathlib import Path

import pytest
import yaml
from racer_pnp_perception import gp1, gp1c, gp1d, split_v04
from racer_pnp_perception.gp1 import BACKEND_GEOMETRIC as GEO
from racer_pnp_perception.gp1 import BACKEND_NET as NET
from racer_pnp_perception.gp1 import Cell, LabelFree, Labelled
from racer_pnp_perception.gp1c import CanaryCase
from racer_track_perception.vehicle import find_repo_root

YAML = find_repo_root() / gp1d.PREREG_RELATIVE_PATH


def free(intrusion=0.005, residual=0.05, deficit=0.02, c95=0.9, scans=3000) -> LabelFree:
    return LabelFree(scans, intrusion, residual, deficit, c95, scans)


def lab(e_c=0.05, v=16.0, scans=800, c95=0.9) -> Labelled:
    return Labelled(scans, e_c, e_c * 1.4, v, c95)


def decide(label="22", **over):
    d = {
        "h": {NET: lab(0.06), GEO: lab(0.136)},
        "a": {NET: free(), GEO: free(0.003, 0.03, 0.02, 0.9)},
        "ci": 0.6,
        "cells": [Cell("run", "ccw", "0.85+", 100, 0.06, 0.136)],
    }
    d.update(over)
    return gp1d.decide_gp1d(label, d["h"], d["a"], d["ci"], d["cells"])


def item(items, n):
    return next(i for i in items if i.number == n)


def test_pass_has_five_items_named_for_the_set() -> None:
    items, failed = decide("22")
    assert failed == [] and [i.number for i in items] == [1, 2, 3, 4, 5]
    assert "H22" in items[0].name and "A22" in items[4].name


def test_item1_at_299_300_and_the_set_is_undecided() -> None:
    assert 1 in decide(h={NET: lab(0.06, scans=299), GEO: lab(0.136, scans=299)})[1]
    assert 1 not in decide(h={NET: lab(0.06, scans=300), GEO: lab(0.136, scans=300)})[1]


@pytest.mark.parametrize(
    ("ratio", "ci", "ok"),
    [(0.90, 0.99, True), (0.9001, 0.5, False), (0.5, 1.0, False), (0.5, 0.999, True)],
)
def test_item2_ratio_and_bootstrap_bound(ratio, ci, ok) -> None:
    h = {NET: lab(0.1 * ratio), GEO: lab(0.1)}
    assert (2 not in decide(h=h, ci=ci)[1]) is ok


def test_item3_v_at_and_below_geometric() -> None:
    assert 3 not in decide(h={NET: lab(0.06, v=15.0), GEO: lab(0.136, v=15.0)})[1]
    assert 3 in decide(h={NET: lab(0.06, v=14.99), GEO: lab(0.136, v=15.0)})[1]


def test_item4_cells_at_threshold_over_and_too_few() -> None:
    at = Cell("run", "ccw", "0.85+", 40, 0.11, 0.10)
    over = Cell("run", "ccw", "0.85+", 40, 0.1101, 0.10)
    few = Cell("run", "cw", "0.85+", 39, 0.5, 0.10)
    assert 4 not in decide(cells=[at])[1]
    assert 4 in decide(cells=[over])[1]
    assert 4 not in decide(cells=[few, at])[1]
    assert 4 not in decide(cells=[few])[1]  # nothing judged, said in the note


def test_item5_each_condition_at_threshold_and_over() -> None:
    geo = free(0.003, 0.03, 0.02, 0.9)
    for kw, ok in (
        ({"intrusion": 0.02}, True),
        ({"intrusion": 0.0201}, False),
        ({"residual": 0.10}, True),
        ({"residual": 0.1001}, False),
        ({"deficit": 0.05}, True),
        ({"deficit": 0.0501}, False),
        ({"c95": 0.88}, True),
        ({"c95": 0.879}, False),
    ):
        assert (5 not in decide(a={NET: free(**kw), GEO: geo})[1]) is ok, kw


def test_the_intrusion_limit_follows_a_bad_geometric_value() -> None:
    a = {NET: free(intrusion=0.0349), GEO: free(0.03)}
    assert 5 not in decide(a=a)[1]
    assert 5 in decide(a={NET: free(intrusion=0.0351), GEO: free(0.03)})[1]


def test_global_items_not_measured_do_not_pass() -> None:
    items, failed = gp1d.decide_global(None, "", None, None)
    assert failed == [6, 7, 8]
    assert all(i.note == gp1c.NOT_MEASURED for i in items)


def test_global_items_pass_and_latency_and_canary_thresholds() -> None:
    good = [CanaryCase("c", True, 0, 10.3, 10.0)]  # 1.03 at the threshold
    assert gp1d.decide_global(True, "", 9.99, good)[1] == []
    assert gp1d.decide_global(True, "", 10.0, good)[1] == [7]
    assert gp1d.decide_global(True, "", 5.0, [CanaryCase("c", True, 1, 10, 10)])[1] == [8]
    assert gp1d.decide_global(True, "", 5.0, [CanaryCase("c", False, 0, 10, 10)])[1] == [8]
    assert gp1d.decide_global(True, "", 5.0, [CanaryCase("c", True, 0, 10.31, 10)])[1] == [8]
    assert gp1d.decide_global(False, "", 5.0, good)[1] == [6]


def test_k_at_threshold_over_and_empty() -> None:
    n = 200
    at = [0.0] * 198 + [0.05, 0.05]
    items, ok = gp1d.decide_gp1d_k("22", [0.01] * n, [0.01] * n)
    assert ok and "A22" in items[0].name
    assert not gp1d.decide_gp1d_k("22", [0.0101] * n, [0.0] * n)[1]
    assert not gp1d.decide_gp1d_k("22", [0.0] * n, [0.0] * 150 + [0.06] * 50)[1]
    assert not gp1d.decide_gp1d_k("22", [], [])[1]
    assert gp1d.decide_gp1d_k("14", [0.0] * n, at)[1] is True


def test_outcome_order() -> None:
    both_ok = {"14": [], "22": []}
    k_ok = {"14": True, "22": True}
    assert gp1d.combine_outcomes(both_ok, [], k_ok) == gp1d.OUTCOME_PASS
    assert gp1d.combine_outcomes({"14": [1], "22": []}, [], k_ok) == gp1d.OUTCOME_UNDECIDED
    assert gp1d.combine_outcomes({"14": [2], "22": [3]}, [], k_ok) == gp1d.OUTCOME_NO_BETTER
    assert gp1d.combine_outcomes({"14": [], "22": [3]}, [], k_ok) == gp1d.OUTCOME_NO_BETTER
    assert (
        gp1d.combine_outcomes(both_ok, [], {"14": True, "22": False}) == gp1d.OUTCOME_K_NOT_KEPT
    )
    # one set passes everything, the other fails an item other than 2 and 3
    assert gp1d.combine_outcomes({"14": [], "22": [5]}, [], k_ok) == gp1d.OUTCOME_ONE_SET
    assert gp1d.combine_outcomes({"14": [], "22": [4]}, [], k_ok) == gp1d.OUTCOME_ONE_SET
    # both fail another item
    assert gp1d.combine_outcomes({"14": [4], "22": [5]}, [], k_ok) == gp1d.OUTCOME_NOT_ELSEWHERE
    # a global item failing counts against both
    assert gp1d.combine_outcomes(both_ok, [7], k_ok) == gp1d.OUTCOME_NOT_ELSEWHERE
    assert gp1d.combine_outcomes(both_ok, [8], k_ok) == gp1d.OUTCOME_NOT_ELSEWHERE


def test_benefit_rule() -> None:
    assert gp1d.meets_benefit({"a": 0.95, "b": 0.5})
    assert not gp1d.meets_benefit({"a": 0.9501})
    assert not gp1d.meets_benefit({"a": float("nan")})
    assert not gp1d.meets_benefit({})


def test_thresholds_are_read_from_the_yaml_and_match_gp1c() -> None:
    doc = gp1d.load_prereg()
    assert doc["gate_k"]["max_mean_loss"] == gp1c.K_MAX_MEAN
    assert doc["gate_k"]["max_p99_loss"] == gp1c.K_MAX_P99
    assert doc["gate"]["item_2"]["max_ec_ratio"] == gp1.ITEM2_MAX_RATIO
    assert doc["gate"]["item_8_canary"]["max_median_lap_ratio_to_geometric"] == gp1c.CANARY_MAX_LAP_RATIO


def test_load_prereg_refuses_another_digest_and_other_thresholds(tmp_path: Path) -> None:
    other = tmp_path / "p.yaml"
    other.write_text(YAML.read_text() + "\n# x\n")
    with pytest.raises(gp1d.PreregMismatch, match="SHA-256"):
        gp1d.load_prereg(other)
    doc = yaml.safe_load(YAML.read_text())
    doc["gate"]["item_2"]["max_ec_ratio"] = 0.95
    with pytest.raises(gp1d.PreregMismatch, match="item_2"):
        gp1d.check_thresholds(doc)
    doc = yaml.safe_load(YAML.read_text())
    doc["gate_k"]["max_p99_loss"] = 0.06
    with pytest.raises(gp1d.PreregMismatch, match="p99"):
        gp1d.check_thresholds(doc)
    shutil.copy(YAML, tmp_path / "same.yaml")
    assert hashlib.sha256((tmp_path / "same.yaml").read_bytes()).hexdigest() == gp1d.PREREG_SHA256
    assert gp1d.load_prereg(tmp_path / "same.yaml")["prereg_id"] == "pnp-v0.6-perception-fused"


def test_the_role_guards() -> None:
    dev = gp1d.assert_bag_allowed
    for bag in gp1d.DEV_2026_11_16:
        dev(bag, "development")
    dev("2026-11-13T22-00-40_car_teleop", "development")
    for held in ("2026-11-14T21-37-23_car_teleop", gp1d.HELDOUT_22_BAG):
        with pytest.raises(gp1d.DataRoleRefused, match="held-out"):
            dev(held, "development")
        dev(held, "heldout")
    for later in ("2026-11-24T00-00-00_car_teleop", "2027-01-01T00-00-00_car_teleop"):
        for purpose in ("development", "heldout"):
            with pytest.raises(gp1d.DataRoleRefused, match="after 2026-11-22"):
                dev(later, purpose)
    with pytest.raises(gp1d.DataRoleRefused, match="not one of the two"):
        dev("2026-11-13T22-00-40_car_teleop", "heldout")
    with pytest.raises(gp1d.DataRoleRefused):
        gp1d.assert_role_allowed("reserve")
    gp1d.assert_role_allowed("heldout")


def test_the_reserve_loader_refuses() -> None:
    with pytest.raises(split_v04.ReserveSegmentRefused):
        split_v04.perception_split(roles=("heldout", "reserve"))
