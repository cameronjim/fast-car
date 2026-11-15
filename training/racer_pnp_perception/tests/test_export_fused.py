"""L1: the fused bundle export (`trackperc-v2f-*`) and its pointer README."""

from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml
from racer_pnp_perception import export_fused, gp1c
from racer_track_perception import fuse
from racer_track_perception.bundle import load_bundle
from racer_track_perception.vehicle import LiveVehicle
from test_fused_perceivers import RELEASE
from test_gp1c import synthetic_rows

SHA = "0123456789abcdef0123456789abcdef01234567"


def test_export_writes_a_loadable_fused_bundle_with_the_same_weights(
    tmp_path: Path, live: LiveVehicle
) -> None:
    cfg = fuse.FuseConfig(reach_m=0.3)
    out = export_fused.export_fused(RELEASE, "trackperc-v2f-test", cfg, SHA, {}, tmp_path)
    assert (out / "weights.npz").read_bytes() == (RELEASE / "weights.npz").read_bytes()
    b = load_bundle(out, "track_perception", live)
    assert b.fuse_config == cfg
    m = yaml.safe_load((out / "manifest.yaml").read_text())
    assert m["io"]["arch"]["type"] == "circ_conv1d_v0_fused_v1"
    assert m["git_sha"] == SHA and m["model_id"] == "trackperc-v2f-test"
    assert "no retraining" in " ".join(m["data"])


def test_export_refuses_a_bad_sha_or_config(tmp_path: Path) -> None:
    with pytest.raises(RuntimeError, match="sha"):
        export_fused.export_fused(RELEASE, "x", fuse.FuseConfig(), "abc", {}, tmp_path)
    with pytest.raises(ValueError, match="blend"):
        export_fused.export_fused(RELEASE, "x", fuse.FuseConfig(blend=2.0), SHA, {}, tmp_path)


def test_the_pointer_readme_starts_with_the_verdict_and_flags_the_construction(
    tmp_path: Path,
) -> None:
    report = gp1c.gp1c_report(synthetic_rows(), gp1c.DevResult(True, [], "pass"), 5.0, None)
    report.update({"frozen_sha": SHA, "prereg_sha256": "ab" * 32, "fuse_config": {"reach_m": 0.35}})
    out = export_fused.export_fused(
        RELEASE, "trackperc-v2f-test", fuse.FuseConfig(), SHA, {"gp1c": report}, tmp_path
    )
    metrics = yaml.safe_load((out / "manifest.yaml").read_text())["metrics"]
    pointer = export_fused.write_pointer(tmp_path / "repo", out, metrics)
    text = (pointer / "README.md").read_text()
    assert text.splitlines()[0].startswith("GP1c verdict: ")
    assert "CONSTRUCTION" not in text.upper()
    assert "GP1c-K: " in text
    assert text.count("| H14 |") == 4 and text.count("| A14 |") == 4
    assert not (pointer / "weights.npz").exists()
    json.loads((pointer / "metrics.json").read_text())
