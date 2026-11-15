"""Brief 41 analysis: GP4-V, the variant form, the step 8 table and the cap above 1.3 rule
(pre-registration v0.4 sections 6.2, 6.4, 6.5)."""

from __future__ import annotations

import copy
import json
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml
from racer_eval import cap_step, gates, gp4v, same_session, variant_form
from racer_eval.cap_step import CapRun, decide_cap_step
from racer_eval.cli import cmd_session
from racer_eval.curves import RunLaps
from racer_eval.gp4v import VariantNotDocumented, decide_gp4v
from racer_eval.recording import BagData
from racer_eval.same_session import DrivenRun, SameSessionRefused, compare
from racer_eval.session import SessionError, analyse_session, bundle_validity_mps, load_session
from racer_eval.variant_form import (
    FormInvalid,
    check_form,
    load_checked,
    parse_form,
    problems_of,
    write_receipt,
)
from test_cli_session import make_bag

LIDAR_Z = 0.20  # a test value standing in for the vehicle binding's mount_z_m


# ------------------------------------------------------------------------------ variant form


def good_doc() -> dict[str, Any]:
    def obj(i: int, wall: str, d: float, prot: float = 0.20, lane: float = 0.95) -> dict[str, Any]:
        return {
            "id": f"o{i}",
            "height_m": 0.30,
            "size_m": [0.30, 0.20],
            "wall": wall,
            "distance_along_outer_wall_m": d,
            "protrusion_m": prot,
            "lane_width_m": lane,
            "against_wall": True,
            "heavy": True,
            "reflective_or_glass": False,
        }

    return {
        "variant_id": "variant-2",
        "photo": "variant2.jpg",
        "seed": 4321,
        "driven_before_first_protocol_run": False,
        "objects": [obj(1, "outer", 1.0), obj(2, "inner", 3.0)],
    }


def form_with(**changes: Any) -> variant_form.VariantForm:
    return replace(parse_form(good_doc()), **changes)


def test_a_good_form_checks() -> None:
    checked = check_form(parse_form(good_doc()), LIDAR_Z)
    assert checked.form.variant_id == "variant-2" and checked.lidar_mount_z_m == LIDAR_Z


def test_too_few_objects() -> None:
    doc = good_doc()
    doc["objects"] = doc["objects"][:1]
    problems = problems_of(parse_form(doc), LIDAR_Z)
    assert any("at least 2 are needed" in p for p in problems)
    assert any("lane changes" in p for p in problems)


def test_lane_change_needs_two_places_of_at_least_015() -> None:
    doc = good_doc()
    doc["objects"][1]["protrusion_m"] = 0.14999
    assert any("lane changes" in p for p in problems_of(parse_form(doc), LIDAR_Z))
    doc["objects"][1]["protrusion_m"] = 0.15
    assert problems_of(parse_form(doc), LIDAR_Z) == []


@pytest.mark.parametrize(("lane", "ok"), [(0.79999, False), (0.80, True), (1.1, True)])
def test_remaining_lane_at_least_080(lane: float, ok: bool) -> None:
    doc = good_doc()
    doc["objects"][0]["lane_width_m"] = lane
    assert (problems_of(parse_form(doc), LIDAR_Z) == []) is ok


def test_a_reused_position_is_refused_against_earlier_forms_and_within_the_form() -> None:
    form = parse_form(good_doc())
    assert any("used before" in p for p in problems_of(form, LIDAR_Z, [("outer", 1.1)]))
    assert problems_of(form, LIDAR_Z, [("inner", 1.0)]) == []  # other wall: another position
    assert problems_of(form, LIDAR_Z, [("outer", 1.4)]) == []  # beyond the tolerance
    doc = good_doc()
    doc["objects"][1].update(wall="outer", distance_along_outer_wall_m=1.1)
    assert any("same position" in p for p in problems_of(parse_form(doc), LIDAR_Z))


def test_an_object_lower_than_the_lidar_plane_is_refused() -> None:
    doc = good_doc()
    doc["objects"][0]["height_m"] = LIDAR_Z + variant_form.LIDAR_HEIGHT_MARGIN_M - 0.001
    assert any("under the LiDAR plane" in p for p in problems_of(parse_form(doc), LIDAR_Z))
    doc["objects"][0]["height_m"] = LIDAR_Z + variant_form.LIDAR_HEIGHT_MARGIN_M
    assert problems_of(parse_form(doc), LIDAR_Z) == []


@pytest.mark.parametrize(
    ("key", "value", "text"),
    [
        ("against_wall", False, "not against a wall"),
        ("heavy", False, "not heavy"),
        ("reflective_or_glass", True, "reflective or glass"),
        ("wall", "middle", "wall must be"),
        ("protrusion_m", 0.0, "protrusion"),
        ("size_m", [0.0, 0.2], "size"),
    ],
)
def test_object_rules(key: str, value: Any, text: str) -> None:
    doc = good_doc()
    doc["objects"][0][key] = value
    assert any(text in p for p in problems_of(parse_form(doc), LIDAR_Z))


def test_form_level_rules() -> None:
    assert any("no photo" in p for p in problems_of(form_with(photo="  "), LIDAR_Z))
    assert any(
        "driven before" in p
        for p in problems_of(form_with(driven_before_first_protocol_run=True), LIDAR_Z)
    )
    doc = good_doc()
    doc["objects"][1]["id"] = "o1"
    assert any("not unique" in p for p in problems_of(parse_form(doc), LIDAR_Z))
    with pytest.raises(FormInvalid) as exc:
        check_form(form_with(photo=""), LIDAR_Z)
    assert exc.value.problems


def test_parse_errors_list_every_missing_or_mistyped_field() -> None:
    with pytest.raises(FormInvalid) as exc:
        parse_form({"objects": "no"})
    text = " ".join(exc.value.problems)
    assert "variant_id" in text and "must be a list" in text
    doc = good_doc()
    doc["seed"] = "abc"
    doc["driven_before_first_protocol_run"] = "no"
    del doc["objects"][0]["height_m"]
    doc["objects"][1]["height_m"] = float("nan")
    doc["objects"].append("junk")
    with pytest.raises(FormInvalid) as exc:
        parse_form(doc)
    text = " ".join(exc.value.problems)
    for needle in ("seed", "driven_before", "missing", "wrong type", "not a mapping"):
        assert needle in text
    doc = good_doc()
    doc["seed"] = True
    with pytest.raises(FormInvalid):
        parse_form(doc)


def test_receipt_roundtrip_and_edit_detection(tmp_path: Path) -> None:
    path = tmp_path / "variant2_form.yaml"
    path.write_text(yaml.safe_dump(good_doc()))
    with pytest.raises(FormInvalid, match="no receipt"):
        load_checked(path)
    checked = check_form(variant_form.load_form(path), LIDAR_Z)
    rp = write_receipt(path, checked, [("outer", 5.0)], "test")
    assert rp.name == "variant2_form.checked.json"
    assert load_checked(path).form.seed == 4321
    path.write_text(yaml.safe_dump({**good_doc(), "seed": 1}))
    with pytest.raises(FormInvalid, match="edited"):
        load_checked(path)
    path.write_text("- not a mapping\n")
    with pytest.raises(FormInvalid, match="not a mapping"):
        variant_form.load_form(path)


def test_interleaved_order_uses_the_seed_of_the_form() -> None:
    checked = check_form(parse_form(good_doc()), LIDAR_Z)
    a = variant_form.interleaved_order(checked, ["d_on", "d_off"], 5)
    assert a == variant_form.interleaved_order(checked, ["d_on", "d_off"], 5)
    assert sorted(a) == ["d_off"] * 5 + ["d_on"] * 5
    other = check_form(form_with(seed=1), LIDAR_Z)
    assert variant_form.interleaved_order(other, ["d_on", "d_off"], 5) != a


# ------------------------------------------------------------------------------------- GP4-V


def laps(first: float = 10.0, late: float = 7.0, n: int = 10) -> tuple[float, ...]:
    """T1 = first, laps 8 to 10 = late, a straight ramp between."""
    ramp = np.linspace(first, late, 8)
    return tuple(float(x) for x in [*ramp[:7], late, late, late][:n])


def run(rid: str, cond: str, **kw: Any) -> RunLaps:
    return RunLaps(rid, cond, kw.pop("lap_times", laps()), **kw)


def on_off(n: int = 5, **on_kw: Any) -> tuple[list[RunLaps], list[RunLaps]]:
    on = [run(f"on{i}", "on", **on_kw) for i in range(n)]
    off = [run(f"off{i}", "off", lap_times=(10.0,) * 10) for i in range(n)]
    return on, off


CHECKED = check_form(parse_form(good_doc()), LIDAR_Z)


def test_gp4v_passes_and_carries_its_own_label() -> None:
    on, off = on_off()
    res = decide_gp4v(on, off, CHECKED)
    assert res.passed and res.outcome == "pass"
    assert res.name == "gp4v" and res.name != "gp4"
    assert res.reported["label"] == "GP4-V, lap learning on an obstacle variant of a known track"
    assert res.reported["variant_id"] == "variant-2" and res.preregistered


def test_gp4v_items_equal_the_gp4_items() -> None:
    on, off = on_off()
    v, g = decide_gp4v(on, off, CHECKED), gates.gp4_decision(on, off)
    assert [i for i in v.items] == [i for i in g.items]


@pytest.mark.parametrize(
    ("item", "on_kw", "n"),
    [
        ("5.2-0a", {}, 4),  # only 4 runs
        ("5.2-2c", {"lap_times": laps(10.0, 8.5)}, 5),  # median r 0.85
        ("5.2-2a", {"lap_times": laps(10.0, 10.0)}, 5),  # delta not positive
    ],
)
def test_gp4v_each_item_failing_alone(item: str, on_kw: dict[str, Any], n: int) -> None:
    on, off = on_off(n, **on_kw)
    res = decide_gp4v(on, off, CHECKED)
    failed = {i.item_id for i in res.items if not i.passed}
    assert item in failed and not res.passed and res.outcome == "fail"


def test_gp4v_contacts_item_fails_alone() -> None:
    on, off = on_off()
    on[0] = run("on0", "on", contacts=1)
    res = decide_gp4v(on, off, CHECKED)
    assert {i.item_id for i in res.items if not i.passed} == {"5.2-3"}


def test_gp4v_a_dnf_fails_item_one_and_is_named() -> None:
    on, off = on_off()
    on[2] = run("on2", "on", lap_times=laps()[:6], dnf_after=6, dnf_cause="stuck at object o1")
    res = decide_gp4v(on, off, CHECKED)
    item = res.item("5.2-1")
    assert not item.passed and "stuck at object o1" in item.note
    assert res.reported["dnf"] == [("on2", "stuck at object o1")]


def test_gp4v_refuses_runs_without_a_checked_form_or_with_a_driven_variant() -> None:
    on, off = on_off()
    with pytest.raises(VariantNotDocumented, match="not documented"):
        decide_gp4v(on, off, None)
    with pytest.raises(VariantNotDocumented):
        decide_gp4v(on, off, parse_form(good_doc()))  # type: ignore[arg-type]  # unchecked form
    with pytest.raises(VariantNotDocumented, match="driven in an earlier step"):
        decide_gp4v(on, off, CHECKED, driven_variants=["variant-2"])
    assert decide_gp4v(on, off, CHECKED, driven_variants=["variant-1"]).passed


def test_gp4v_thresholds_are_the_gp4_constants_by_identity() -> None:
    assert gp4v.GP4_MIN_RUNS is gates.GP4_MIN_RUNS
    assert gp4v.GP4_MAX_MEDIAN_R is gates.GP4_MAX_MEDIAN_R
    assert gp4v.GP4_LEVEL is gates.GP4_LEVEL
    assert gp4v.gp4_decision is gates.gp4_decision
    assert not hasattr(gp4v, "decide_gp4")


# ---------------------------------------------------------------------------- same_session

BAG_A = "2026-11-20T10-00-00_car_teleop"
BAG_B = "2026-11-20T10-30-00_car_teleop"


def driven(rid: str, driver: str, t: float = 5.6, n: int = 5, **kw: Any) -> DrivenRun:
    base: dict[str, Any] = {
        "bag": BAG_A,
        "direction": "ccw",
        "lap_times": tuple(t + 0.01 * k for k in range(n)),
        "wheel_distance_m": 5.5 * n,
        "start_voltage_v": 8.0,
    }
    base.update(kw)
    return DrivenRun(rid, driver, **base)


def test_same_session_table() -> None:
    table = compare([driven("t1", "tracker", 5.6), driven("p1", "pnp", 5.9, bag=BAG_B)])
    assert table["session"] == "2026-11-20" and table["direction"] == "ccw"
    assert table["comparison_claim_allowed"] and table["reasons"] == []
    row = table["rows"][0]
    assert row["laps"] == 5 and row["best_lap_s"] == pytest.approx(5.6)
    assert row["mean_lap_s"] == pytest.approx(5.62)
    assert row["wheel_distance_per_lap_m"] == pytest.approx(5.5)
    assert table["per_driver"]["pnp"]["mean_lap_s"] == pytest.approx(5.92)
    assert table["battery_window"].startswith("held")
    assert table["note"] == "a comparison table, no gate"


@pytest.mark.parametrize(
    ("runs", "text"),
    [
        ([], "no runs"),
        (
            [driven("t", "tracker"), driven("p", "pnp", bag="2026-11-21T10-00-00_car_teleop")],
            "sessions",
        ),
        ([driven("t", "tracker"), driven("p", "pnp", direction="cw")], "directions"),
        ([driven("t", "tracker")], "both the tracker and PnP"),
        ([driven("t", "tracker"), driven("p", "pnp", bag="nodate")], "no date prefix"),
        ([driven("t", "tracker"), replace(driven("p", "pnp"), driver="robot")], "driver must be"),
    ],
)
def test_same_session_refusals(runs: list[DrivenRun], text: str) -> None:
    with pytest.raises(SameSessionRefused, match=text):
        compare(runs)


def test_same_session_claim_needs_five_laps_each_and_the_battery_window() -> None:
    few = compare([driven("t", "tracker", n=4), driven("p", "pnp")])
    assert not few["comparison_claim_allowed"]
    assert any("tracker: 4 laps" in r for r in few["reasons"])
    two_runs = compare(
        [driven("t", "tracker", n=3), driven("t2", "tracker", n=2), driven("p", "pnp")]
    )
    assert two_runs["comparison_claim_allowed"]  # 3 + 2 laps pooled is 5
    wide = compare(
        [driven("t", "tracker", start_voltage_v=8.5), driven("p", "pnp", start_voltage_v=7.69)]
    )
    assert not wide["comparison_claim_allowed"] and wide["battery_window"].startswith("broken")
    edge = compare(
        [driven("t", "tracker", start_voltage_v=8.5), driven("p", "pnp", start_voltage_v=7.7)]
    )
    assert edge["comparison_claim_allowed"]  # exactly 0.8 V is inside the window
    unknown = compare([driven("t", "tracker", start_voltage_v=None), driven("p", "pnp")])
    assert (
        not unknown["comparison_claim_allowed"] and "cannot be checked" in unknown["battery_window"]
    )


def test_session_of_bag() -> None:
    assert same_session.session_of_bag(BAG_A) == "2026-11-20"


# ----------------------------------------------------------------------------------- cap_step


def cap_run(rid: str, cap: float, direction: str, speed: float, **kw: Any) -> CapRun:
    base: dict[str, Any] = {
        "bag": BAG_A,
        "lap_memory_off": True,
        "brakes": 0,
        "contacts": 0,
        "mean_wheel_speed_mps": speed,
    }
    base.update(kw)
    return CapRun(rid, cap_mps=cap, direction=direction, **base)


def cap_runs(new_speed: float = 1.45) -> tuple[list[CapRun], list[CapRun]]:
    new = [cap_run(f"n{i}{d}", 1.5, d, new_speed) for i in range(2) for d in ("ccw", "cw")]
    ref = [cap_run(f"r{d}", 1.3, d, 1.25) for d in ("ccw", "cw")]
    return new, ref


def failed(res: gates.GateResult) -> set[str]:
    return {i.item_id for i in res.items if not i.passed}


def test_cap_step_passes() -> None:
    new, ref = cap_runs()
    res = decide_cap_step(new, ref, 1.5)
    assert res.passed and res.outcome == "pass" and res.section == "v0.4 6.4"
    assert cap_step.REFERENCE_CAP_MPS == 1.3 and cap_step.RUNS_PER_DIRECTION == 2


def test_cap_step_cap_must_exceed_13_and_fit_the_validity() -> None:
    new, ref = cap_runs()
    assert failed(decide_cap_step(new, ref, 1.3)) == {"6.4-2"}
    at_edge = [replace(r, cap_mps=1.3) for r in new]
    assert "6.4-1" in failed(decide_cap_step(at_edge, ref, 1.5))
    assert "6.4-1" in failed(decide_cap_step([], ref, 1.5))


def test_cap_step_needs_two_runs_per_direction_and_a_reference() -> None:
    new, ref = cap_runs()
    assert failed(decide_cap_step(new[:3], ref, 1.5)) == {"6.4-3"}
    assert failed(decide_cap_step(new, ref[:1], 1.5)) == {"6.4-4", "6.4-9"}


def test_cap_step_session_lap_memory_brakes_and_contacts() -> None:
    new, ref = cap_runs()
    other = [replace(ref[0], bag="2026-11-21T10-00-00_car_teleop"), ref[1]]
    assert failed(decide_cap_step(new, other, 1.5)) == {"6.4-5"}
    mem = [replace(new[0], lap_memory_off=False), *new[1:]]
    assert failed(decide_cap_step(mem, ref, 1.5)) == {"6.4-6"}
    brake = [replace(new[0], brakes=1), *new[1:]]
    assert failed(decide_cap_step(brake, ref, 1.5)) == {"6.4-7"}
    hit = [replace(new[0], contacts=1), *new[1:]]
    assert failed(decide_cap_step(hit, ref, 1.5)) == {"6.4-8"}


def test_cap_step_mean_speed_not_lower_per_direction() -> None:
    new, ref = cap_runs(new_speed=1.25)
    assert decide_cap_step(new, ref, 1.5).passed  # equal is not lower
    new, ref = cap_runs(new_speed=1.2499)
    res = decide_cap_step(new, ref, 1.5)
    assert failed(res) == {"6.4-9"}
    assert res.reported["mean_wheel_speed"][0]["reference_mean_mps"] == pytest.approx(1.25)


# --------------------------------------------------------------------------- session blocks

B1 = "2026-11-20T10-00-00_car_teleop"
B2 = "2026-11-20T10-30-00_car_teleop"


def write_form(tmp_path: Path) -> Path:
    path = tmp_path / "variant2_form.yaml"
    path.write_text(yaml.safe_dump(good_doc()))
    write_receipt(path, check_form(variant_form.load_form(path), LIDAR_Z), [], "test")
    return path


def test_session_gp4v_block_end_to_end(tmp_path: Path) -> None:
    form = write_form(tmp_path)
    doc = {
        "gp4v": {"on": "d_on", "off": "d_off", "form": str(form)},
        "runs": [
            {"id": f"{c}{i}", "bag": f"b{c}{i}", "at_s": 50.0, "condition": c, "cap_mps": 0.5}
            for c in ("d_on", "d_off")
            for i in range(5)
        ],
    }
    bags = {f"b{c}{i}": make_bag(f"b{c}{i}") for c in ("d_on", "d_off") for i in range(5)}
    result = analyse_session(doc, bags)
    assert result.decisions["gp4v"].name == "gp4v"
    assert result.decisions["gp4v"].reported["label"].startswith("GP4-V")
    bad = copy.deepcopy(doc)
    del bad["gp4v"]["form"]
    with pytest.raises(SessionError, match="variant not documented"):
        analyse_session(bad, bags)
    bad = copy.deepcopy(doc)
    bad["gp4v"]["off"] = "nothing"
    with pytest.raises(SessionError, match="gp4v.off"):
        analyse_session(bad, bags)
    form.write_text(yaml.safe_dump({**good_doc(), "seed": 9}))  # edited after the receipt
    with pytest.raises(FormInvalid, match="edited"):
        analyse_session(doc, bags)


def test_session_same_session_block_and_cli_outputs(tmp_path: Path) -> None:
    runs = [
        {"id": "t1", "bag": B1, "at_s": 50.0, "condition": "tracker", "cap_mps": 1.0,
         "driver": "tracker", "direction": "ccw", "start_voltage_v": 8.1},
        {"id": "p1", "bag": B2, "at_s": 50.0, "condition": "pnp", "cap_mps": 1.0,
         "driver": "pnp", "direction": "ccw", "start_voltage_v": 8.0},
    ]  # fmt: skip
    doc = {"n_laps": 10, "same_session": {}, "runs": runs}
    path = tmp_path / "s.yaml"
    path.write_text(yaml.safe_dump(doc))
    bags = {B1: make_bag(B1), B2: make_bag(B2)}
    result = analyse_session(load_session(path), bags)
    table = result.tables["same_session"]
    # make_bag drives at 0.5 m/s for 129 s: 64.5 m over 10 laps
    assert table["rows"][0]["wheel_distance_per_lap_m"] == pytest.approx(6.45, rel=0.02)
    assert table["comparison_claim_allowed"]
    import argparse

    out = tmp_path / "out"
    ns = argparse.Namespace(
        bags=[str(tmp_path / B1), str(tmp_path / B2)], conditions=str(path), out=str(out)
    )
    assert cmd_session(ns, read=lambda p: bags[p.name]) == 0
    tables = json.loads((out / "tables.json").read_text())
    assert tables["same_session"]["note"] == "a comparison table, no gate"
    assert "same_session" in (out / "summary.md").read_text()
    runs[1].pop("direction")
    path.write_text(yaml.safe_dump({**doc, "runs": runs}))
    with pytest.raises(SessionError, match="needs `direction`"):
        analyse_session(load_session(path), bags)
    runs[1]["direction"] = "cw"
    path.write_text(yaml.safe_dump({**doc, "runs": runs}))
    with pytest.raises(SameSessionRefused, match="directions"):
        analyse_session(load_session(path), bags)
    ns.conditions = str(tmp_path / "s2.yaml")
    runs[1]["direction"] = "ccw"
    runs[1]["driver"] = "tracker"  # two tracker runs: no PnP to compare with
    (tmp_path / "s2.yaml").write_text(yaml.safe_dump({**doc, "runs": runs}))
    with pytest.raises(SameSessionRefused):
        analyse_session(load_session(tmp_path / "s2.yaml"), bags)
    # a run without a driver key is left out of the table
    runs[1].pop("driver")
    (tmp_path / "s3.yaml").write_text(yaml.safe_dump({**doc, "runs": runs}))
    with pytest.raises(SameSessionRefused):
        analyse_session(load_session(tmp_path / "s3.yaml"), bags)


def manifest(tmp_path: Path, v_max: float = 1.5) -> Path:
    p = tmp_path / "manifest.yaml"
    p.write_text(yaml.safe_dump({"io": {"validity": {"v_min_mps": 0.0, "v_max_mps": v_max}}}))
    return p


def test_session_cap_step_block(tmp_path: Path) -> None:
    def r(i: str, cond: str, cap: float, direction: str, **kw: Any) -> dict[str, Any]:
        return {"id": i, "bag": B1, "at_s": 50.0, "condition": cond, "cap_mps": cap,
                "direction": direction, "lap_memory": "off", **kw}  # fmt: skip

    runs = [r(f"n{i}{d}", "cap15", 1.5, d) for i in range(2) for d in ("ccw", "cw")]
    runs += [r(f"r{d}", "cap13", 1.3, d) for d in ("ccw", "cw")]
    doc = {
        "cap_step": {
            "new": "cap15",
            "reference": "cap13",
            "bundle_manifest": str(manifest(tmp_path)),
        },
        "runs": runs,
    }
    bags = {B1: make_bag(B1)}
    result = analyse_session(doc, bags)
    res = result.decisions["cap_step"]
    # the same synthetic bag for every run: equal mean speeds, and the bag's ttc event is inside
    # the counting window of the run, so item 7 sees it.
    assert res.item("6.4-9").passed and res.item("6.4-2").passed
    assert res.item("6.4-7").measured == 4  # one `ttc` ENGAGE per run in make_bag
    bad = copy.deepcopy(doc)
    bad["runs"][0].pop("lap_memory")
    with pytest.raises(SessionError, match="needs `direction` and `lap_memory`"):
        analyse_session(bad, bags)
    bad = copy.deepcopy(doc)
    bad["cap_step"]["new"] = "nothing"
    with pytest.raises(SessionError, match="no runs"):
        analyse_session(bad, bags)
    assert bundle_validity_mps(manifest(tmp_path, 1.3)) == 1.3
    empty = tmp_path / "empty.yaml"
    empty.write_text("io: {}\n")
    with pytest.raises(SessionError, match="validity"):
        bundle_validity_mps(empty)


def test_wheel_stats_of_a_run_without_odometry() -> None:
    from racer_eval.runs import Run
    from racer_eval.session import wheel_stats

    bag = BagData(name="x")
    assert wheel_stats(bag, Run("R", "pnp", 0.0, 1.0, 5.0)) == (0.0, 0.0)
