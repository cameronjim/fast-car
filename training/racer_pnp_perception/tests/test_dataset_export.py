from __future__ import annotations

import datetime as dt
import json
from pathlib import Path

import numpy as np
import pytest
import yaml
from racer_pnp_perception import dataset, export
from racer_track_perception.bundle import load_bundle
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle


def test_quantise_round_trip() -> None:
    rng = np.random.default_rng(0)
    t = np.zeros((5, 2, 360), dtype=np.float32)
    t[:, 0] = rng.uniform(size=(5, 360))
    t[:, 0, ::7] = 1.0
    t[:, 1] = (rng.uniform(size=(5, 360)) < 0.5).astype(np.float32)
    q0, q1 = dataset.quantise(t)
    back = dataset.dequantise(q0, q1)
    assert np.abs(back - t).max() < 1e-4 / 6  # well under a millimetre of range
    assert np.array_equal(back[:, 1], t[:, 1]) and (back[:, 0, ::7] == 1.0).all()


def test_generate_is_deterministic_and_well_formed() -> None:
    a = dataset.generate(24, 5, workers=1, keep_ranges=True)
    b = dataset.generate(24, 5, workers=1, keep_ranges=True)
    assert np.array_equal(a["tensor"], b["tensor"]) and np.array_equal(a["label"], b["label"])
    assert a["tensor"].shape == (24, 2, 360) and a["label"].shape == (24, 16, 5)
    assert a["ranges"].shape == (24, 720)
    assert np.isfinite(a["label"]).all()
    assert a["tensor"].min() >= 0.0 and a["tensor"].max() <= 1.0
    c = dataset.generate(24, 6, workers=1)
    assert not np.array_equal(a["tensor"], c["tensor"])


def test_generate_chunks_do_not_depend_on_the_worker_count() -> None:
    a = dataset.generate(dataset.CHUNK + 30, 3, workers=1)
    assert a["tensor"].shape[0] == dataset.CHUNK + 30


def test_clean_and_held_out_sets() -> None:
    clean = dataset.generate(8, 1, workers=1, clean=True, keep_ranges=True)
    assert not np.isnan(clean["ranges"]).any()  # no dropout in a clean scan
    held = dataset.generate(8, 1, workers=1, held_out="stadium_l1.0_s2.0_r0.4")
    assert np.isfinite(held["label"]).all()
    # A stadium of lane 1.0: half widths about 0.5 on the straights.
    assert 0.4 < np.median(held["label"][:, :, 2]) < 0.6


def test_save_and_load(tmp_path: Path) -> None:
    data = dataset.generate(10, 2, workers=1)
    dataset.save(tmp_path / "d.npz", data)
    x, y = dataset.load(tmp_path / "d.npz")
    assert np.abs(x - data["tensor"]).max() < 1e-4 and np.array_equal(y, data["label"])


def random_arrays(seed: int = 0) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    ch = (2, 4, 4, 4, 4)
    out: dict[str, np.ndarray] = {"n_conv": np.array(4), "n_linear": np.array(2)}
    for i in range(4):
        out[f"conv{i}_weight"] = rng.normal(size=(ch[i + 1], ch[i], 5)).astype(np.float32) * 0.2
        out[f"conv{i}_bias"] = np.zeros(ch[i + 1], dtype=np.float32)
        out[f"conv{i}_stride"] = np.array(2)
        out[f"conv{i}_pad"] = np.array(2)
    out["linear0_weight"] = rng.normal(size=(8, 4 * 23)).astype(np.float32) * 0.1
    out["linear0_bias"] = np.zeros(8, dtype=np.float32)
    out["linear1_weight"] = rng.normal(size=(80, 8)).astype(np.float32) * 0.1
    out["linear1_bias"] = np.zeros(80, dtype=np.float32)
    return out


def test_write_bundle_dir_loads_back(tmp_path: Path, live: LiveVehicle) -> None:
    out = export.write_bundle_dir(
        tmp_path / "trackperc-v0-2026-11-11",
        random_arrays(),
        live,
        "trackperc-v0-2026-11-11",
        "a" * 64,
        "b" * 40,
        ["synthetic"],
        {"x": 1.0},
    )
    bundle = load_bundle(out, "track_perception", live)
    assert bundle.model_id == "trackperc-v0-2026-11-11"
    net = CircConv1dV0.from_npz(bundle.payload_path("weights.npz"))
    assert net.forward(np.zeros((1, 2, 360), dtype=np.float32)).shape == (16, 5)


def test_finalize_writes_metrics_pointer_and_fixture(tmp_path: Path, live: LiveVehicle) -> None:
    bundle = export.write_bundle_dir(
        tmp_path / "models" / "trackperc-v0-2026-11-11",
        random_arrays(),
        live,
        "trackperc-v0-2026-11-11",
        "a" * 64,
        "b" * 40,
        ["synthetic"],
        {},
    )
    repo = tmp_path / "repo"
    metrics = {
        "verdict": "net fails, use backend:=geometric tonight",
        "median": float("nan"),
        "a": 1.5,
    }
    export.finalize(bundle, metrics, repo, copy_fixture=True, live=live)
    pointer = repo / "training" / "pnp_models" / "trackperc-v0-2026-11-11"
    assert (pointer / "manifest.yaml").is_file() and not (pointer / "weights.npz").exists()
    stored = json.loads((pointer / "metrics.json").read_text())
    assert stored["verdict"].startswith("net fails") and stored["median"] is None
    readme = (pointer / "README.md").read_text()
    assert "net fails, use backend:=geometric tonight" in readme and chr(0x2014) not in readme
    manifest = yaml.safe_load((bundle / "manifest.yaml").read_text())
    assert manifest["metrics"]["a"] == 1.5
    release = repo / "ros_ws/src/racer_track_perception/test/fixtures/bundle_release"
    assert (release / "weights.npz").is_file() and (release / "manifest.yaml").is_file()
    load_bundle(release, "track_perception", live)
    export.finalize(bundle, {"verdict": "net passes"}, repo, copy_fixture=False, live=live)
    assert "net passes" in (pointer / "README.md").read_text()


def test_compose_metrics_and_readme_tables() -> None:
    net = {
        "centreline_err_median_stations_0_7_m": 0.01,
        "centreline_err_median_stations_8_15_m": 0.03,
        "half_width_err_median_m": 0.01,
    }
    sim = {
        "in_distribution": {"net_raw": net, "geometric_valid": net, "samples": 5},
        "held_out": {"t1": {"net_raw": net, "geometric_valid": net, "samples": 5}},
        "held_out_acceptance": {"all_ok": True},
    }
    row = {
        "boundary_residual_median_m": 0.2,
        "boundary_residual_p90_m": 0.3,
        "intrusion_fraction_mean": float("nan"),
        "share_valid_count_under_6": 0.0,
    }
    bags = {"verdict": "net fails, use backend:=geometric tonight", "gate_pooled": {"net": row}}
    metrics = export.compose_metrics(sim, bags, [{"epoch": 3}])
    assert metrics["verdict"].startswith("net fails") and metrics["final_val"] == {"epoch": 3}
    assert set(metrics["sim_held_out"]["t1"]) == {"net_raw", "geometric_valid"}
    text = export.readme_text("trackperc-v0-x", export._clean(metrics))
    assert "| net | 0.200 | 0.300 | n/a | 0.000 |" in text
    assert "| t1 | 0.010 | 0.030 | 0.010 |" in text
    assert export.compose_metrics(sim, {}, None)["verdict"] == "no real-bag verdict"


def test_default_model_id_and_clean() -> None:
    assert export.default_model_id(dt.date(2026, 11, 12)) == "trackperc-v0-2026-11-12"
    assert export._clean({"a": [float("inf"), 1.0, np.float32("nan")]}) == {"a": [None, 1.0, None]}
    assert len(export.git_sha(Path(__file__).parent)) == 40


def test_git_sha_rejects_garbage(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    class Out:
        stdout = "not-a-sha\n"

    monkeypatch.setattr(export.subprocess, "run", lambda *a, **k: Out())
    with pytest.raises(RuntimeError):
        export.git_sha(tmp_path)
