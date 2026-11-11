"""A tiny valid track_perception bundle, built on the fly from the live vehicle_params.

The committed fixture (tests/fixtures/bundle_tiny/) holds the seeded random weights; the manifest
is rebuilt here from the live vehicle_params so a vehicle_params version bump does not make the
fixture stale. `write_bundle` takes mutators that break exactly one thing for the refusal tests.
"""

from __future__ import annotations

import copy
import hashlib
import itertools
import shutil
from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from racer_track_perception.bundle import expected_io
from racer_track_perception.vehicle import LiveVehicle

FIXTURE_DIR = Path(__file__).parent / "fixtures" / "bundle_tiny"
CHANNELS = (2, 4, 4, 4, 4)


def tiny_weights(seed: int = 0) -> dict[str, np.ndarray]:
    """Small seeded weights of a valid circ_conv1d_v0 (4 channels, 16 hidden units)."""
    rng = np.random.default_rng(seed)
    out: dict[str, np.ndarray] = {
        "n_conv": np.array(len(CHANNELS) - 1),
        "n_linear": np.array(2),
    }
    length = 360
    for i, (c_in, c_out) in enumerate(itertools.pairwise(CHANNELS)):
        out[f"conv{i}_weight"] = (rng.normal(size=(c_out, c_in, 5)) * 0.3).astype(np.float32)
        out[f"conv{i}_bias"] = (rng.normal(size=c_out) * 0.1).astype(np.float32)
        out[f"conv{i}_stride"] = np.array(2)
        out[f"conv{i}_pad"] = np.array(2)
        length = (length + 4 - 5) // 2 + 1
    features = CHANNELS[-1] * length
    out["linear0_weight"] = (rng.normal(size=(16, features)) * 0.1).astype(np.float32)
    out["linear0_bias"] = (rng.normal(size=16) * 0.1).astype(np.float32)
    out["linear1_weight"] = (rng.normal(size=(80, 16)) * 0.1).astype(np.float32)
    out["linear1_bias"] = np.zeros(80, dtype=np.float32)
    # A usable sigma at every station so the net's output is a valid track.
    out["linear1_bias"][4::5] = np.log(0.05)
    return out


def manifest_for(live: LiveVehicle, weights_sha256: str) -> dict[str, Any]:
    return {
        "bundle_schema_version": "1.0.0",
        "kind": "track_perception",
        "model_id": "trackperc-v0-2026-11-12-tiny",
        "created_utc": "2026-11-12T03:00:00Z",
        "git_sha": "0123456789abcdef0123456789abcdef01234567",
        "training_config_sha256": "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
        "vehicle_params": {
            "schema_version": live.schema_version,
            "sysid_session_id": live.sysid_session_id,
        },
        "payload": [{"file": "weights.npz", "sha256": weights_sha256}],
        "io": expected_io(live),
        "data": ["test fixture, random weights"],
        "metrics": {},
    }


def write_bundle(
    directory: Path,
    live: LiveVehicle,
    mutate: Callable[[dict[str, Any]], None] | None = None,
    weights: Path | None = None,
) -> Path:
    """Write a bundle into `directory` (weights copied from the committed fixture), apply
    `mutate` to the manifest dict, and return the directory."""
    directory.mkdir(parents=True, exist_ok=True)
    src = weights or FIXTURE_DIR / "weights.npz"
    shutil.copyfile(src, directory / "weights.npz")
    sha = hashlib.sha256((directory / "weights.npz").read_bytes()).hexdigest()
    manifest = copy.deepcopy(manifest_for(live, sha))
    if mutate is not None:
        mutate(manifest)
    (directory / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))
    return directory


if __name__ == "__main__":  # regenerate the committed fixture: python tests/bundle_fixture.py
    from racer_track_perception.vehicle import load_live_vehicle

    FIXTURE_DIR.mkdir(parents=True, exist_ok=True)
    np.savez(FIXTURE_DIR / "weights.npz", **tiny_weights())
    sha_ = hashlib.sha256((FIXTURE_DIR / "weights.npz").read_bytes()).hexdigest()
    man = manifest_for(load_live_vehicle(), sha_)
    (FIXTURE_DIR / "manifest.yaml").write_text(yaml.safe_dump(man, sort_keys=False))
    print(f"wrote {FIXTURE_DIR}")
