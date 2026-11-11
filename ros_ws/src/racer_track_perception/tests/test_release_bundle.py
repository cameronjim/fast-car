"""The released bundle committed as a fixture (test/fixtures/bundle_release/, for the sim canary)
is intact and runs. Its vehicle_params identity is the one it was exported with, not the live
file's, so a later vehicle_params bump does not fail this test: the canary's own loader refuses
a stale bundle, which is the point of the refusal; this test is about the files."""

from __future__ import annotations

import dataclasses
from pathlib import Path

import numpy as np
import yaml
from racer_track_perception import bundle as bd
from racer_track_perception import core
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle
from scan_fixture import SCAN, corridor_ranges

RELEASE = Path(__file__).parents[1] / "test" / "fixtures" / "bundle_release"


def exported_vehicle(live: LiveVehicle) -> LiveVehicle:
    manifest = yaml.safe_load((RELEASE / "manifest.yaml").read_text())
    vp = manifest["vehicle_params"]
    return dataclasses.replace(
        live, schema_version=vp["schema_version"], sysid_session_id=vp["sysid_session_id"]
    )


def test_release_bundle_is_small_and_loads(live: LiveVehicle) -> None:
    total = sum(f.stat().st_size for f in RELEASE.iterdir())
    assert total < 500_000
    b = bd.load_bundle(RELEASE, "track_perception", exported_vehicle(live))
    assert b.model_id.startswith("trackperc-v0-")
    assert "verdict" in b.manifest["metrics"]


def test_release_bundle_gives_a_track_on_a_synthetic_lane(live: LiveVehicle) -> None:
    b = bd.load_bundle(RELEASE, "track_perception", exported_vehicle(live))
    net = CircConv1dV0.from_npz(b.payload_path("weights.npz"))
    p = core.TrackPerceiver(core.BACKEND_NET, live, net=net, model_id=b.model_id)
    t = p.process(corridor_ranges(live), **SCAN)
    assert t.arrays_ok() and t.valid_count >= 12
    # A smoke check on an idealised straight 1 m lane (nothing like the training scans, which
    # always have a closed track and a room): the centreline is near y = 0, half widths near 0.5 m.
    assert np.abs(t.y_m[: t.valid_count]).max() < 0.25
    assert np.abs(t.half_width_left_m[: t.valid_count] - 0.5).max() < 0.2
