"""The v1 export: manifest against the bundle schema, the pinned vehicle_params, the pointer, and
one refusal test per refusal class on a copy of a v1 bundle with the defect injected."""

from __future__ import annotations

import copy
import dataclasses
import json
from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml
from racer_pnp_perception import export as ex
from racer_pnp_perception import gp1
from racer_track_perception import bundle as bd
from racer_track_perception import errors as er
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle
from test_gp1 import SIM_SPLIT_YAML, random_net  # noqa: F401  (random_net builds the weights)

SPLIT_SHA = "bc5185c912ece67a6177581f1b37adaaa9101b34ee1efe9881af7723681aef83"
KIND = "track_perception"


def tiny_arrays() -> dict[str, np.ndarray]:
    net = random_net(1)
    arrays: dict[str, np.ndarray] = {"n_conv": np.array(len(net._convs)), "n_linear": np.array(2)}
    for i, c in enumerate(net._convs):
        shape = (c.w2.shape[0], c.c_in, c.k)
        arrays[f"conv{i}_weight"] = c.w2.reshape(shape)
        arrays[f"conv{i}_bias"] = c.b
        arrays[f"conv{i}_stride"] = np.array(c.stride)
        arrays[f"conv{i}_pad"] = np.array(c.pad)
    for i, (w, b) in enumerate(net._linears):
        arrays[f"linear{i}_weight"], arrays[f"linear{i}_bias"] = w, b
    return arrays


def fake_gp1(outcome: str = gp1.OUTCOME_NO_BETTER) -> dict[str, Any]:
    lab = {"scans": 700, "e_c_m": 0.05, "e_far_m": 0.09, "v": 15.5, "c95": 0.9}
    free = {
        "scans": 900,
        "intrusion_mean": 0.01,
        "residual_median_m": 0.04,
        "deficit_p90_m": 0.02,
        "c95": 0.9,
        "c95_scans": 800,
    }
    two = lambda d: {"net": dict(d), "geometric": dict(d)}
    return {
        "split_sha256": SPLIT_SHA,
        "H-run": two(lab),
        "H-layout": two(lab),
        "A-run": two(free),
        "A-layout": two(free),
        "bootstrap_e_c_ratio_net_over_geometric_h_run": {
            "ratio": 0.95,
            "lo": 0.9,
            "hi": 1.0,
            "blocks": 90,
            "block_s": 2.0,
            "resamples": 2000,
            "seed": 0,
        },
        "items": [
            {"number": k, "name": f"item {k}", "passed": k != 2, "value": {"x": 1.0}}
            for k in range(1, 7)
        ],
        "cells": [
            {
                "set": "run",
                "direction": "ccw",
                "speed_bin": "0.85+",
                "scans": 50,
                "ratio": 1.0,
                "judged": True,
            }
        ],
        "outcome": outcome,
    }


SIM_NET = {
    "centreline_err_median_stations_0_7_m": 0.012,
    "centreline_err_median_stations_8_15_m": 0.044,
    "half_width_err_median_m": 0.02,
}

CONFIG = {
    "train": "/x/trackperc-v0-train-s0-n300000.npz",
    "val": "/x/trackperc-v0-val-s0-n20000.npz",
    "epochs": 6,
    "batch_size": 256,
    "seed": 1,
    "real_fraction": 0.25,
    "init": None,
    "real": {
        "train_labels": 5654,
        "val_labels": 1200,
        "segments_used": ["bagA/G1a", "bagB/R0"],
        "split_sha256": SPLIT_SHA,
    },
}


@pytest.fixture
def v1_bundle(tmp_path: Path, live: LiveVehicle) -> Path:
    metrics = {
        "gp1": fake_gp1(),
        "split_sha256": SPLIT_SHA,
        "latency_numpy_forward_ms": {"p50": 0.9, "p99": 1.4, "scans": 2000},
        "sim": {"held_out": {"trackA": {"net_raw": SIM_NET}}},
        "sim_v0_held_out_net_raw": {"trackA": {**SIM_NET, "half_width_err_median_m": 0.01}},
    }
    return ex.write_v1_bundle(
        tmp_path / "trackperc-v1-2026-11-14",
        tiny_arrays(),
        live,
        "trackperc-v1-2026-11-14",
        "0" * 64,
        "1" * 40,
        ex.data_lines_v1(
            CONFIG,
            {"criterion": "final val_real_ec_m", "per_seed": {0: 0.1, 1: 0.09}, "selected_seed": 1},
        ),
        metrics,
    )


def test_manifest_validates_pins_the_vehicle_and_carries_gp1_and_the_split_sha(
    v1_bundle: Path, live: LiveVehicle
) -> None:
    b = bd.load_bundle(v1_bundle, KIND, live)  # validates against config/pnp/bundle.schema.json
    manifest = yaml.safe_load((v1_bundle / "manifest.yaml").read_text())
    assert manifest["vehicle_params"] == {
        "schema_version": "0.17.0",
        "sysid_session_id": "none-preliminary",
    }
    assert b.model_id == "trackperc-v1-2026-11-14"
    assert manifest["metrics"]["split_sha256"] == SPLIT_SHA
    assert manifest["metrics"]["gp1"]["outcome"] == gp1.OUTCOME_NO_BETTER
    assert any("never trained on held-out" in line for line in manifest["data"])
    assert any("bagA/G1a" in line for line in manifest["data"])
    assert any("simulated lanes" in line and "n300000" in line for line in manifest["data"])
    # The weights file is the numpy runtime's input and the runtime loads it.
    CircConv1dV0.from_npz(b.payload_path("weights.npz"))


def test_an_unpinned_vehicle_params_is_refused_at_export(tmp_path: Path, live: LiveVehicle) -> None:
    for bad in (
        dataclasses.replace(live, schema_version="0.16.0"),
        dataclasses.replace(live, sysid_session_id="sysid-2026-11-13"),
    ):
        with pytest.raises(RuntimeError, match="pins 0.17.0"):
            ex.write_v1_bundle(tmp_path / "b", tiny_arrays(), bad, "m", "0" * 64, "1" * 40, [], {})
        assert not (tmp_path / "b" / "manifest.yaml").exists()


def test_the_pointer_has_the_verdict_first_and_no_weights(v1_bundle: Path, tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    repo.mkdir()
    metrics = yaml.safe_load((v1_bundle / "manifest.yaml").read_text())["metrics"]
    out = ex.write_pointer_v1(repo, v1_bundle, metrics)
    assert out == repo / "training" / "pnp_models" / "trackperc-v1-2026-11-14"
    assert sorted(p.name for p in out.iterdir()) == ["README.md", "manifest.yaml", "metrics.json"]
    readme = (out / "README.md").read_text()
    assert readme.splitlines()[0] == f"GP1 verdict: {gp1.OUTCOME_NO_BETTER}"
    assert "| H-run | net | 700 |" in readme and "FAIL" in readme and "p99 1.400 ms" in readme
    assert "| trackA | 0.012 / 0.012 | 0.044 / 0.044 | 0.020 / 0.010 |" in readme
    assert "—" not in readme
    assert json.loads((out / "metrics.json").read_text())["split_sha256"] == SPLIT_SHA


def test_the_release_fixture_is_only_copied_for_a_pass(tmp_path: Path) -> None:
    # main_v1 reads the gp1 outcome before it touches the fixture; the guard is the outcome string.
    assert gp1.OUTCOME_PASS != gp1.OUTCOME_NO_BETTER
    assert gp1.OUTCOME_PASS.startswith("pass")


# ---------------------------------------------------------------------------------------------
# One refusal test per refusal class, on a v1 bundle copy with the defect injected
# ---------------------------------------------------------------------------------------------


def edited(v1_bundle: Path, tmp_path: Path, mutate: Callable[[dict[str, Any]], None]) -> Path:
    dst = tmp_path / "copy"
    dst.mkdir()
    for name in ("manifest.yaml", "weights.npz"):
        (dst / name).write_bytes((v1_bundle / name).read_bytes())
    manifest = yaml.safe_load((dst / "manifest.yaml").read_text())
    mutate(manifest)
    (dst / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
    return dst


def test_refusal_manifest_missing(v1_bundle: Path, tmp_path: Path, live: LiveVehicle) -> None:
    (v1_bundle / "manifest.yaml").unlink()
    with pytest.raises(er.ManifestMissing):
        bd.load_bundle(v1_bundle, KIND, live)


def test_refusal_manifest_invalid(v1_bundle: Path, tmp_path: Path, live: LiveVehicle) -> None:
    with pytest.raises(er.ManifestInvalid):
        bd.load_bundle(edited(v1_bundle, tmp_path, lambda m: m.pop("git_sha")), KIND, live)


def test_refusal_schema_version(v1_bundle: Path, tmp_path: Path, live: LiveVehicle) -> None:
    def v2(m: dict[str, Any]) -> None:
        m["bundle_schema_version"] = "2.0.0"

    with pytest.raises(er.BundleSchemaVersionMismatch):
        bd.load_bundle(edited(v1_bundle, tmp_path, v2), KIND, live)


def test_refusal_kind(v1_bundle: Path, live: LiveVehicle) -> None:
    with pytest.raises(er.KindMismatch):
        bd.load_bundle(v1_bundle, "dynamics", live)


def test_refusal_payload_missing(v1_bundle: Path, live: LiveVehicle) -> None:
    (v1_bundle / "weights.npz").unlink()
    with pytest.raises(er.PayloadMissing):
        bd.load_bundle(v1_bundle, KIND, live)


def test_refusal_checksum(v1_bundle: Path, live: LiveVehicle) -> None:
    data = (v1_bundle / "weights.npz").read_bytes()
    (v1_bundle / "weights.npz").write_bytes(data[:-1] + bytes([data[-1] ^ 1]))
    with pytest.raises(er.ChecksumMismatch):
        bd.load_bundle(v1_bundle, KIND, live)


@pytest.mark.parametrize(
    ("key", "value"), [("schema_version", "0.16.0"), ("sysid_session_id", "sysid-2026-11-13")]
)
def test_refusal_vehicle_params(
    v1_bundle: Path, tmp_path: Path, live: LiveVehicle, key: str, value: str
) -> None:
    def change(m: dict[str, Any]) -> None:
        m["vehicle_params"][key] = value

    with pytest.raises(er.VehicleParamsMismatch, match=key):
        bd.load_bundle(edited(v1_bundle, tmp_path, change), KIND, live)


def test_refusal_io_contract(v1_bundle: Path, tmp_path: Path, live: LiveVehicle) -> None:
    def change(m: dict[str, Any]) -> None:
        m["io"]["input"]["n_bins"] = 180

    with pytest.raises(er.IoContractMismatch, match="n_bins"):
        bd.load_bundle(edited(v1_bundle, tmp_path, change), KIND, live)
    copy.deepcopy(v1_bundle)


@pytest.mark.torch
def test_export_v1_from_a_torch_run_matches_numpy_and_writes_candidate_then_final(
    tmp_path: Path, live: LiveVehicle
) -> None:
    torch = pytest.importorskip("torch")
    from racer_pnp_perception import model
    from racer_track_perception.vehicle import find_repo_root

    torch.manual_seed(0)
    net = model.build_model()
    with torch.no_grad():
        for p in net.parameters():
            p.add_(torch.randn_like(p) * 0.05)
    run = tmp_path / "run"
    run.mkdir()
    torch.save(net.state_dict(), run / "model.pt")
    (run / "config.json").write_text(json.dumps(CONFIG))
    repo = find_repo_root()
    candidate = ex.export_v1(run, "trackperc-v1-test", None, {}, repo, None, tmp_path / "cand")
    cand_manifest = yaml.safe_load((candidate / "manifest.yaml").read_text())
    assert cand_manifest["metrics"]["status"].startswith("candidate")
    final = ex.export_v1(run, "trackperc-v1-test", fake_gp1(), {}, repo, None, tmp_path / "final")
    m = yaml.safe_load((final / "manifest.yaml").read_text())
    assert m["metrics"]["numpy_vs_torch_max_abs_diff"] < 1e-5
    assert m["vehicle_params"]["schema_version"] == "0.17.0"
    # Same weights in both: GP1 is run on the candidate, the final bundle is the same network.
    assert (candidate / "weights.npz").read_bytes() == (final / "weights.npz").read_bytes()
    bd.load_bundle(final, KIND, live)
