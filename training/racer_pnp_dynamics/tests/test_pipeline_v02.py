"""L1 for the report side of pipeline_v02: named spans, the per run table (R0 to R8), the legacy
report, the dead zone and lag reports, and the split-aware CLI helpers."""

from __future__ import annotations

import numpy as np
import pytest
from racer_pnp_dynamics import cli, pipeline_v02
from racer_pnp_dynamics.dataset import by_split, make_segments
from racer_pnp_dynamics.fit import FitResult
from racer_pnp_dynamics.model import PLACEHOLDER_NOMINAL, Residual
from racer_pnp_dynamics.split import BagEntry, Split
from synth import make_run, make_split
from test_fit import TRUTH

SEGS = [
    ("R0", 5.0, 120.0, "train", "cw"),
    ("R3", 125.0, 240.0, "heldout", "ccw"),
    ("R7a", 250.0, 330.0, "train", "cw"),
    ("R7b", 332.0, 410.0, "heldout", "cw"),
    ("G9", 420.0, 470.0, "heldout", "cw"),  # not an R run
]


@pytest.fixture(scope="module")
def segs(toy_vehicle_module):
    run, motion = make_run(
        TRUTH, toy_vehicle_module, seconds=480.0, seed=2, name="bag", v_hi=1.0, speed_noise=0.001
    )
    return make_segments(run, motion, make_split("bag", SEGS), t_ref_s=run.t_s[0])


def test_named_spans_join_the_train_and_val_pieces_of_one_segment(segs):
    spans = pipeline_v02.named_spans(segs)
    assert set(spans) == {"R0", "R3", "R7a", "R7b", "G9"}
    r0 = spans["R0"][0]
    pieces = [s for s in segs if s.name == "R0"]
    assert r0.i0 == min(p.i0 for p in pieces) and r0.i1 == max(p.i1 for p in pieces)
    assert r0.split == "train" and spans["R3"][0].split == "heldout"
    assert {p.split for p in pieces} == {"train", "val"}


def test_run_table_has_one_row_per_run_with_r7_pooled_and_the_role_of_each(
    segs, toy_vehicle_module
):
    variants = {
        "truth": (TRUTH, None),
        "placeholder": (PLACEHOLDER_NOMINAL, None),
    }
    table = pipeline_v02.run_table(segs, "bag", variants, toy_vehicle_module)
    assert list(table) == ["R0", "R3", "R7"]  # G9 is not an R run
    assert table["R0"]["role"] == "train" and table["R3"]["role"] == "heldout"
    assert table["R7"]["role"] == "heldout+train"  # R7a trains, R7b is held out
    for row in table.values():
        for name in variants:
            e = row["variants"][name]
            assert e["windows"] > 20 and set(e["errors"]) == {0.5, 1.0, 1.5}
    r3 = table["R3"]["variants"]
    assert (
        r3["truth"]["errors"][1.0]["speed_rmse_mps"]
        < r3["placeholder"]["errors"][1.0]["speed_rmse_mps"]
    )
    other = pipeline_v02.run_table(segs, "no-such-bag", variants, toy_vehicle_module)
    assert other == {}


def test_legacy_report_is_the_v01_pooled_report_on_the_legacy_part(toy_vehicle_module):
    run, motion = make_run(TRUTH, toy_vehicle_module, seconds=400.0, seed=4, name="old")
    split = Split("t", "0" * 64, {"old": BagEntry("old", "current", None, (), "legacy_70_30")})
    segs = make_segments(run, motion, split)
    legacy = by_split(segs, "legacy_heldout")
    out = pipeline_v02.legacy_report(
        legacy, toy_vehicle_module, {"truth": (TRUTH, None), "bad": (PLACEHOLDER_NOMINAL, None)}
    )
    assert out["segments"] == len(legacy) > 0
    assert (
        out["errors"]["truth"][1.0]["speed_rmse_mps"] < out["errors"]["bad"][1.0]["speed_rmse_mps"]
    )
    assert pipeline_v02.legacy_report([], toy_vehicle_module, {"truth": (TRUTH, None)}) == {
        "segments": 0,
        "errors": {},
    }


def test_dead_zone_and_lag_reports_read_only_the_segments_they_are_given(segs, toy_vehicle_module):
    train_val = [s for s in segs if s.split in ("train", "val")]
    result = FitResult(nominal=TRUTH)
    dz = pipeline_v02.dead_zone_report(train_val, result, toy_vehicle_module)
    assert dz["fitted_mps"] == TRUTH.speed_deadband_mps
    assert set(dz) >= {"launches_from_rest", "steady_state", "launch_list", "choice"}
    lag = pipeline_v02.lag_report(train_val, TRUTH)
    assert lag["available"] and "peak_lag_s" in lag
    assert lag["fitted_delay_plus_steer_tau_s"] == pytest.approx(
        TRUTH.command_delay_s + TRUTH.steer_tau_s
    )
    assert pipeline_v02.lag_report([], TRUTH) == {"available": False}


def test_split_bags_lists_the_bags_with_something_for_dynamics():
    def whole(role):
        return BagEntry(role, "current", None, (), role)

    split = Split(
        "t",
        "0" * 64,
        {
            "seg": make_split("seg", SEGS).bags["seg"],
            "legacy_70_30": whole("legacy_70_30"),
            "unused": whole("unused"),
            "heldout_layout": whole("heldout_layout"),
        },
    )
    assert cli.split_bags(split) == {"seg", "legacy_70_30"}


def test_the_residual_golden_rollouts_are_part_of_the_manifest_only_when_enabled(
    tmp_path, toy_vehicle_module
):
    from racer_pnp_dynamics import export

    for enabled, n in ((False, 12), (True, 17)):
        export.write_bundle(
            tmp_path / str(enabled), model_id="m", created_utc="2026-11-13T00:00:00Z",
            git_sha="0" * 40, training_config_sha256="0" * 64, vehicle=toy_vehicle_module,
            nominal=TRUTH, residual=Residual.disabled(), v_max_mps=1.0, data=[], metrics={},
            golden_residual_rollouts=enabled,
        )  # fmt: skip
        assert len(export.read_golden(tmp_path / str(enabled) / "golden_rollouts.csv")) == n
    assert np.isfinite(pipeline_v02.V_MAX_CAP_MPS)
