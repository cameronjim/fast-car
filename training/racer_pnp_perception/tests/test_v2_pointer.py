"""L1: the committed pointer of trackperc-v2 (brief 32): the verdict line first, the by-construction
note for the clearance deficit, the refinement declared in the manifest, never the weights."""

from __future__ import annotations

import json
from pathlib import Path

import yaml
from racer_pnp_perception import export, gp1
from racer_track_perception.vehicle import find_repo_root

POINTER = find_repo_root() / "training" / "pnp_models" / "trackperc-v2-2026-11-14"


def test_the_pointer_starts_with_the_verdict_and_holds_no_weights() -> None:
    metrics = json.loads((POINTER / "metrics.json").read_text())
    readme = (POINTER / "README.md").read_text()
    verdict = metrics["gp1b"]["refined"]["outcome"]
    assert readme.splitlines()[0] == f"GP1b verdict: {verdict}"
    assert verdict in (
        gp1.OUTCOME_GP1B_PASS,
        gp1.OUTCOME_GP1B_NO_BETTER,
        gp1.OUTCOME_GP1B_NOT_ELSEWHERE,
    )
    assert "BY CONSTRUCTION" in readme
    assert not list(POINTER.glob("*.npz")) and not list(POINTER.glob("*.pt"))
    assert sum(f.stat().st_size for f in POINTER.iterdir()) < 200_000


def test_the_pointer_reports_the_three_variants_side_by_side() -> None:
    g = json.loads((POINTER / "metrics.json").read_text())["gp1b"]
    assert {"refined", "refined_no_cap", "unrefined"} <= set(g)
    readme = (POINTER / "README.md").read_text()
    for title in ("refined (candidate)", "without the clearance cap", "unrefined"):
        assert title in readme
    flagged = [i for i in g["refined"]["items"] if gp1.DEFICIT_BY_CONSTRUCTION_NOTE in i["note"]]
    assert [i["number"] for i in flagged] == [2, 4, 5]
    assert all(
        gp1.DEFICIT_BY_CONSTRUCTION_NOTE not in i["note"] for i in g["refined_no_cap"]["items"]
    )


def test_the_manifest_declares_the_refinement_and_the_recipe() -> None:
    manifest = yaml.safe_load((POINTER / "manifest.yaml").read_text())
    assert manifest["model_id"] == "trackperc-v2-2026-11-14"
    assert manifest["io"]["arch"]["type"] == "circ_conv1d_v0_refine_v1"
    data = " ".join(manifest["data"])
    assert "c66ce014dabac8340239252f70128d7992e17c51" in data and "device mps" in data
    assert Path(POINTER / "README.md").read_text().count("—") == 0  # no em dashes


def test_export_v2_ids_and_lines() -> None:
    assert export.default_model_id_v2().startswith("trackperc-v2-")
    lines = export.data_lines_v2(
        {
            "train": "t.npz",
            "val": "v.npz",
            "epochs": 6,
            "batch_size": 256,
            "seed": 1,
            "real_fraction": 0.75,
            "scale_range": [0.75, 1.4],
            "scale_probability": 1.0,
            "device": "mps",
            "torch": "x",
        },
        None,
        "abc",
    )
    assert any("2026-10" in line for line in lines) and any("device mps" in line for line in lines)
