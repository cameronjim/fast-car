"""L1: the commands of GP1d after the freeze: the set report from rows, the held-out 22 run list,
the fused-v2 export and pointer, and the one-pass guards."""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import numpy as np
import pytest
import yaml
from racer_pnp_perception import export_fused_v2, gp1, gp1c, gp1d, gp1d_score, heldout22
from racer_pnp_perception.eval_bags import build_perceivers
from racer_track_perception import fuse_v2
from racer_track_perception.core import BACKEND_NET
from racer_track_perception.vehicle import LiveVehicle
from test_fused_perceivers import RELEASE
from test_gp1c import synthetic_rows

SHA = "0123456789abcdef0123456789abcdef01234567"


def v2_copy(tmp_path: Path) -> Path:
    target = tmp_path / "v2"
    shutil.copytree(RELEASE, target)
    m = yaml.safe_load((target / "manifest.yaml").read_text())
    m["io"]["arch"] = {
        "type": "circ_conv1d_v0_fused_v2",
        "fuse": fuse_v2.FuseV2Config(edge_end_extension_m=0.1).as_dict(),
    }
    (target / "manifest.yaml").write_text(yaml.safe_dump(m, sort_keys=False))
    return target


def test_set_report_is_strict_json_and_named_for_the_set() -> None:
    report, failed, k_ok = gp1d_score.set_report("22", synthetic_rows())
    json.dumps(gp1._clean(report), allow_nan=False)
    assert failed == [] and k_ok is False  # no coverage on the synthetic rows: K cannot pass
    assert "H22" in report["items"][0]["name"]
    assert set(report["side_by_side"]["H14"]) == {"geometric", "fused", "net", "net_unrefined"}
    assert report["counts"] == {"labelled": 400, "moving": 400}


def test_set_report_with_coverage_rows_passes_k() -> None:
    rows = synthetic_rows()
    for r in rows:
        r.coverage = {"loss": 0.001, "excess": 0.002, "extra": 0, "geo_area_m2": 1.0}
    _report, failed, k_ok = gp1d_score.set_report("14", rows)
    assert failed == [] and k_ok is True


def test_format_report_says_what_it_must(tmp_path: Path) -> None:
    rows = synthetic_rows()
    s, failed, k = gp1d_score.set_report("14", rows)
    g_items, g_failed = gp1d_score.global_items(None, None, None)
    result = gp1._clean(
        {
            "outcome": gp1d.combine_outcomes({"14": failed}, g_failed, {"14": k}),
            "sets": {"14": s},
            "global_items": [vars(i) for i in g_items],
        }
    )
    text = gp1d_score.format_report(result)
    assert "GP1d outcome" in text and "3 percent of lap time" in text


def test_run_windows_from_launch_logs(tmp_path: Path) -> None:
    for k, (_n, log_name, _d) in enumerate(heldout22.RUNS):
        base = 1795406000.0 + 100 * k
        (tmp_path / log_name).write_text(
            f"[INFO] [{base + 1.5:.9f}] [x]: a\n[WARN] [{base + 50.25:.9f}] [y]: b\n"
        )
    t0 = 1795405990.0
    win = heldout22.run_windows(tmp_path, t0)
    assert [w["name"] for w in win] == ["R0", "R1", "R2", "R3", "R4"]
    assert win[0]["from"] == pytest.approx(11.5) and win[0]["to"] == pytest.approx(60.25)
    assert [w["direction"] for w in win] == ["ccw", "ccw", "ccw", "cw", "cw"]
    split = heldout22.split_22(tmp_path, t0)
    seg = split.locate(gp1d.HELDOUT_22_BAG, 20.0, 30.0)
    assert seg is not None and seg.name == "R0" and seg.role == "heldout"
    assert split.locate(gp1d.HELDOUT_22_BAG, 70.0, 80.0) is None  # between runs: unused
    assert split.locate(gp1d.HELDOUT_22_BAG, 50.0, 120.0) is None  # a window across two runs
    (tmp_path / "pnp-observe-205702.log").write_text("no stamps here")
    with pytest.raises(ValueError, match="no ROS time stamp"):
        heldout22.run_windows(tmp_path, t0)


def test_build_perceivers_runs_a_v2_bundle_as_fused_v2(tmp_path: Path, live: LiveVehicle) -> None:
    p = build_perceivers(live, v2_copy(tmp_path))
    cfg = p[BACKEND_NET].fuse_cfg
    assert isinstance(cfg, fuse_v2.FuseV2Config) and cfg.edge_end_extension_m == 0.1
    assert p[BACKEND_NET].refine_net is True


def test_perceivers_from_net_take_a_v2_config(live: LiveVehicle) -> None:
    from racer_track_perception.runtime import CircConv1dV0

    net = CircConv1dV0.from_npz(RELEASE / "weights.npz")
    p = gp1c.perceivers_from_net(net, live, "x", fuse_v2.FuseV2Config())
    assert isinstance(p[gp1c.FUSED].fuse_cfg, fuse_v2.FuseV2Config)
    assert p[BACKEND_NET].fuse_cfg is None  # the v2 refined backend is not fused


def test_export_writes_a_loadable_v2_bundle_with_the_same_weights(
    tmp_path: Path, live: LiveVehicle
) -> None:
    from racer_track_perception.bundle import load_bundle

    cfg = fuse_v2.FuseV2Config(reach_m=0.3, edge_end_extension_m=0.1)
    out = export_fused_v2.export_fused_v2(RELEASE, "trackperc-v2g-test", cfg, SHA, {}, tmp_path)
    assert (out / "weights.npz").read_bytes() == (RELEASE / "weights.npz").read_bytes()
    assert load_bundle(out, "track_perception", live).fuse_config == cfg
    m = yaml.safe_load((out / "manifest.yaml").read_text())
    assert m["io"]["arch"]["type"] == "circ_conv1d_v0_fused_v2" and m["git_sha"] == SHA


def test_export_refuses_a_bad_sha_or_config(tmp_path: Path) -> None:
    with pytest.raises(RuntimeError, match="sha"):
        export_fused_v2.export_fused_v2(RELEASE, "x", fuse_v2.FuseV2Config(), "abc", {}, tmp_path)
    with pytest.raises(ValueError, match="edge_end_extension_m"):
        export_fused_v2.export_fused_v2(
            RELEASE, "x", fuse_v2.FuseV2Config(edge_end_extension_m=0.5), SHA, {}, tmp_path
        )


def test_pointer_readme_starts_with_the_verdict_and_carries_the_sentence(tmp_path: Path) -> None:
    rows = synthetic_rows()
    s, failed, k = gp1d_score.set_report("14", rows)
    g_items, g_failed = gp1d_score.global_items(None, None, None)
    gate = gp1._clean(
        {
            "outcome": gp1d.combine_outcomes({"14": failed}, g_failed, {"14": k}),
            "sets": {"14": s},
            "global_items": [vars(i) for i in g_items],
            "frozen_sha": SHA,
            "prereg_sha256": gp1d.PREREG_SHA256,
            "fuse_config": {"reach_m": 0.35},
        }
    )
    out = export_fused_v2.export_fused_v2(
        RELEASE, "trackperc-v2g-test", fuse_v2.FuseV2Config(), SHA, {"gp1d": gate}, tmp_path
    )
    metrics = yaml.safe_load((out / "manifest.yaml").read_text())["metrics"]
    pointer = export_fused_v2.write_pointer(tmp_path / "repo", out, metrics)
    text = (pointer / "README.md").read_text()
    assert text.splitlines()[0].startswith("GP1d verdict:")
    assert "3 percent of lap time" in text and not (pointer / "weights.npz").exists()


def test_score_refuses_to_overwrite_and_a_dirty_or_wrong_head(tmp_path: Path) -> None:
    out = tmp_path / "gp1d.json"
    out.write_text("{}")
    args = type("A", (), {"out": out, "frozen_sha": SHA})()
    with pytest.raises(SystemExit, match="refusing to overwrite"):
        gp1d_score.score_main(args)  # type: ignore[arg-type]
    out.unlink()
    with pytest.raises(SystemExit, match="HEAD is"):
        gp1d_score.score_main(args)  # type: ignore[arg-type]


def test_a_real_numpy_import_is_available() -> None:
    assert np.__version__
