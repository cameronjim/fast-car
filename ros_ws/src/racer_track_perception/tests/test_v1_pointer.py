"""The committed pointer of trackperc-v1 (training/pnp_models/trackperc-v1-*/: manifest, metrics,
README, never the weights) is consistent, and, where the dev machine's bundle is present, the
bundle itself loads with this package's loader and gives a track on a synthetic lane."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
import yaml
from racer_track_perception import bundle as bd
from racer_track_perception import core
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle, find_repo_root
from scan_fixture import SCAN, corridor_ranges

REPO = find_repo_root()
POINTERS = sorted((REPO / "training" / "pnp_models").glob("trackperc-v1-*"))
DEV_BUNDLES = Path("/Users/cameronjim/code/car-data/pnp/models")
OUTCOMES = (
    "pass",
    "undecided",
    "no better than geometric in the layout",
    "better where it trained, not shown elsewhere",
)


@pytest.mark.skipif(not POINTERS, reason="no v1 pointer in this checkout")
@pytest.mark.parametrize("pointer", POINTERS, ids=lambda p: p.name)
def test_pointer_manifest_validates_and_pins_the_vehicle(pointer: Path) -> None:
    manifest = yaml.safe_load((pointer / "manifest.yaml").read_text())
    schema = bd._schema(None)
    validator = bd.jsonschema.validators.validator_for(schema)(schema)
    errors = sorted(validator.iter_errors(manifest), key=lambda e: list(e.path))
    assert not errors, errors[0].message
    assert manifest["model_id"] == pointer.name and manifest["kind"] == "track_perception"
    assert manifest["vehicle_params"] == {
        "schema_version": "0.17.0",
        "sysid_session_id": "none-preliminary",
    }
    assert manifest["metrics"]["split_sha256"] == manifest["metrics"]["gp1"]["split_sha256"]
    assert len(manifest["metrics"]["split_sha256"]) == 64
    assert manifest["metrics"]["numpy_vs_torch_max_abs_diff"] < 1e-5


@pytest.mark.skipif(not POINTERS, reason="no v1 pointer in this checkout")
@pytest.mark.parametrize("pointer", POINTERS, ids=lambda p: p.name)
def test_pointer_has_no_weights_and_leads_with_the_verdict(pointer: Path) -> None:
    assert sorted(p.name for p in pointer.iterdir()) == [
        "README.md",
        "manifest.yaml",
        "metrics.json",
    ]
    manifest = yaml.safe_load((pointer / "manifest.yaml").read_text())
    metrics = json.loads((pointer / "metrics.json").read_text())
    assert metrics["gp1"] == manifest["metrics"]["gp1"]
    first = (pointer / "README.md").read_text().splitlines()[0]
    assert first == f"GP1 verdict: {metrics['gp1']['outcome']}"
    assert any(o in first for o in OUTCOMES)
    items = metrics["gp1"]["items"]
    assert [i["number"] for i in items] == [1, 2, 3, 4, 5, 6]
    assert {k for k in ("H-run", "H-layout", "A-run", "A-layout")} <= set(metrics["gp1"])


@pytest.mark.parametrize("pointer", POINTERS, ids=lambda p: p.name)
def test_dev_bundle_loads_and_gives_a_track(pointer: Path, live: LiveVehicle) -> None:
    bundle_dir = DEV_BUNDLES / pointer.name
    if not (bundle_dir / "weights.npz").is_file():
        pytest.skip("the weights are not on this machine")
    b = bd.load_bundle(bundle_dir, "track_perception", live)
    assert b.manifest["vehicle_params"]["schema_version"] == live.schema_version == "0.17.0"
    net = CircConv1dV0.from_npz(b.payload_path("weights.npz"))
    p = core.TrackPerceiver(core.BACKEND_NET, live, net=net, model_id=b.model_id)
    t = p.process(corridor_ranges(live), **SCAN)
    assert t.arrays_ok() and np.isfinite(t.x_m).all() and 0 <= t.valid_count <= 16
