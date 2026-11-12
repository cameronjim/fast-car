"""The self-label CLI end to end on a simulated drive, with the bag read replaced by the scan cache
the module writes (a bag is never opened)."""

from __future__ import annotations

import json

import numpy as np
from racer_pnp_perception import selflabel as sl
from racer_pnp_perception.tracks import IslandParams, island_world
from racer_track_perception.vehicle import LiveVehicle
from test_selflabel import SPEED_MPS, drive_poses, scans_from_poses, true_motion


def write_cache(out, name: str, scans: sl.ScanSet, speed: float) -> None:
    cache = out / "cache"
    cache.mkdir(parents=True, exist_ok=True)
    np.savez(
        cache / f"{name}.scans.npz",
        stamp_s=scans.stamp_s,
        ranges=scans.ranges,
        angle_min=scans.angle_min,
        angle_increment=scans.angle_increment,
        range_min=scans.range_min,
        range_max=scans.range_max,
        speed_t=scans.stamp_s,
        speed_mps=np.full(len(scans), speed),
    )


def test_cli_writes_labels_and_a_manifest(tmp_path, live: LiveVehicle) -> None:
    world = island_world(IslandParams(1.0, 3.0, 2.0, 0.4))
    poses = drive_poses(world, 64)
    scans = scans_from_poses(world, live, poses)
    out = tmp_path / "labels"
    write_cache(out, "simbag", scans, SPEED_MPS)
    sl.save_motion(out / "cache" / "simbag.motion.npz", true_motion(poses))
    assert sl.main(["--bags", str(tmp_path / "simbag"), "--out", str(out)]) == 0
    manifest = json.loads((out / "manifest.json").read_text())
    entry = manifest["bags"]["simbag"]
    assert manifest["git_sha"]
    assert entry["moving_scans"] == 64 and entry["kept"] > 10
    assert set(entry["dropped"]) == set(sl.REASONS)
    assert entry["kept"] + sum(entry["dropped"].values()) == entry["moving_scans"]
    with np.load(out / "simbag.npz") as z:
        assert len(z["index"]) == entry["kept"]
        assert z["tensor"].shape == (entry["kept"], 1, 2, 360) and z["tensor"].dtype == np.float32
        assert z["label"].shape == (entry["kept"], 16, 5)
    # A second bag is added to the manifest, not written over it.
    write_cache(out, "stopped", scans, 0.0)
    sl.save_motion(out / "cache" / "stopped.motion.npz", true_motion(poses))
    assert sl.main(["--bags", str(tmp_path / "stopped"), "--out", str(out)]) == 0
    manifest = json.loads((out / "manifest.json").read_text())
    assert set(manifest["bags"]) == {"simbag", "stopped"}
    assert (
        manifest["bags"]["stopped"]["moving_scans"] == 0
        and manifest["bags"]["stopped"]["kept"] == 0
    )
