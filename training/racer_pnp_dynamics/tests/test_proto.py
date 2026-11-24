"""L1: the development split of the prototype bundle (proto.py) and its table helpers. L2: the
residual of the prototype's configs is bounded by its weights for any input (hypothesis)."""

from __future__ import annotations

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_pnp_dynamics import proto
from racer_pnp_dynamics.model import Residual
from racer_pnp_dynamics.residual_train import _init_weights, cap_weights

ROW = {
    "run_id": "R0",
    "start_s": 100.0,
    "end_s": 200.0,
    "duration_s": 100.0,
    "wheel_distance_m": 50.0,
}
NEW_BAG = "2026-11-24T04-36-54_car_teleop"
PROTECTED = "2026-11-23T03-50-32_car_teleop"


def test_a_run_is_train_val_guard_dev_in_time_order() -> None:
    parts = proto.parts_of_run(NEW_BAG, ROW)
    assert [p.role for p in parts] == ["train", "val", "dev"]
    train, val, dev = parts
    assert dev.t_from_s == pytest.approx(175.0) and dev.t_to_s == 200.0
    assert val.t_to_s == pytest.approx(173.0)  # 2.0 s guard before the dev part
    assert train.t_from_s == 100.0 and train.t_to_s == pytest.approx(val.t_from_s)
    assert val.duration_s == pytest.approx(0.15 * 73.0)
    proto.check_roles(parts)


def test_the_gp2g_heldout_bag_gives_only_its_registered_train_share() -> None:
    assert PROTECTED in proto.PROTECTED_BAGS
    parts = proto.parts_of_run(PROTECTED, ROW)
    assert [p.role for p in parts] == ["train", "val"]
    assert parts[-1].t_to_s == pytest.approx(100.0 + 0.55 * 100.0)
    with pytest.raises(proto.RoleRefused):
        proto.check_roles([*parts, proto.RunPart(PROTECTED, "R0", "dev", 160.0, 200.0)])


@pytest.mark.parametrize(
    "row",
    [
        {**ROW, "duration_s": 14.9, "end_s": 114.9},
        {**ROW, "wheel_distance_m": 0.9},
    ],
)
def test_short_or_undriven_runs_are_unused(row) -> None:
    assert proto.parts_of_run(NEW_BAG, row) == []


def test_overlapping_parts_and_unknown_roles_are_refused() -> None:
    a = proto.RunPart(NEW_BAG, "R0", "train", 0.0, 10.0)
    with pytest.raises(ValueError, match="overlap"):
        proto.check_roles([a, proto.RunPart(NEW_BAG, "R0", "dev", 9.0, 12.0)])
    with pytest.raises(ValueError, match="role"):
        proto.check_roles([proto.RunPart(NEW_BAG, "R0", "test", 0.0, 1.0)])


def test_plan_parts_and_the_split_document_are_stable() -> None:
    runs = {
        NEW_BAG: [ROW, {**ROW, "run_id": "R1", "start_s": 300.0, "end_s": 400.0}],
        PROTECTED: [ROW],
    }
    parts = proto.plan_parts(runs)
    assert len(parts) == 3 + 3 + 2
    first = {NEW_BAG: 1.0, PROTECTED: 2.0}
    text = proto.split_document(parts, first)
    assert text == proto.split_document(proto.plan_parts(runs), first)
    assert text.startswith(f"model_id: {proto.MODEL_ID}\n")
    assert "role: dev" in text and PROTECTED in text
    assert text.count("role: dev") == 2  # none from the protected bag


def test_summarise_gives_ratios_to_the_baseline() -> None:
    n = 40
    lab = np.array(["cw:a"] * 20 + ["ccw:a"] * 20)
    base = {f: np.full(n, 0.2) for f in ("pos", "yaw", "speed")}
    cand = {f: np.full(n, 0.1) for f in ("pos", "yaw", "speed")}
    errs = {"base": {1.0: base}, "cand": {1.0: cand}}
    rows = proto.summarise(errs, lab, {"all": np.ones(n, bool), "cw": lab == "cw:a"}, "base")
    by = {(r["group"], r["model"]): r for r in rows}
    assert by[("all", "cand")]["P1_ratio"] == pytest.approx(0.5)
    assert by[("cw", "base")]["windows"] == 20 and by[("cw", "base")]["S1_ratio"] == 1.0
    assert "| all | cand | 40 |" in proto.render_rows(rows)


CAP = np.array([2.0, 0.5])
SCALE = np.array([0.7, 0.3])
WEIGHTS = cap_weights(_init_weights(np.random.default_rng(3), out_scale=5.0), CAP / SCALE)
RES = Residual(True, WEIGHTS, np.zeros(4), np.ones(4), SCALE)


@settings(max_examples=300, deadline=None)
@given(
    st.lists(st.floats(-1e3, 1e3, allow_nan=False, allow_infinity=False), min_size=4, max_size=4)
)
def test_capped_residual_output_never_leaves_its_bound(x) -> None:
    out = RES.forward(np.asarray(x, dtype=float))
    assert np.all(np.isfinite(out))
    assert np.all(np.abs(out) <= CAP + 1e-9)
