from __future__ import annotations

import json
import math

import numpy as np
import pytest
from racer_pnp_perception import eval_selflabels as ev
from racer_pnp_perception import selflabel as sl
from racer_pnp_perception.tracks import IslandParams, island_world
from racer_track_perception import core
from racer_track_perception.vehicle import LiveVehicle
from test_selflabel import drive_poses, scans_from_poses, true_motion


def test_summarise_hand_case() -> None:
    nan = math.nan
    error = np.full((4, 16), nan)
    error[0, :3] = [0.1, 0.2, 0.3]
    error[1, :2] = [0.3, 0.4]
    error[2, :1] = [0.2]
    out = ev.summarise(error, np.array([3.0, 2.0, 1.0, 0.0]), np.array([1.0, 0.5, nan, nan]))
    assert out["scans"] == 4
    assert abs(out["centreline_error_mean_m"] - np.mean([0.1, 0.2, 0.3, 0.3, 0.4, 0.2])) < 1e-12
    st = out["centreline_error_per_station"]
    assert st["0"]["n"] == 3 and abs(st["0"]["mean_m"] - 0.2) < 1e-12
    assert st["2"]["n"] == 1 and abs(st["2"]["p90_m"] - 0.3) < 1e-12
    assert st["5"]["n"] == 0 and math.isnan(st["5"]["mean_m"])
    assert out["valid_count_mean"] == 1.5
    # A scan with no valid stations counts as containment 0.
    assert abs(out["path_containment_mean"] - 0.375) < 1e-12
    assert out["path_containment_share_at_least_0_95"] == 0.25
    empty = ev.summarise(np.zeros((0, 16)), np.zeros(0), np.zeros(0))
    assert empty["scans"] == 0 and math.isnan(empty["centreline_error_mean_m"])


@pytest.fixture(scope="module")
def oval_labels(live: LiveVehicle):
    world = island_world(IslandParams(1.0, 3.0, 2.0, 0.4))
    poses = drive_poses(world, 64)
    scans = scans_from_poses(world, live, poses)
    cand = np.zeros(len(scans), dtype=bool)
    cand[8:34] = True
    return scans, sl.make_labels(scans, true_motion(poses), live, cand)


def test_score_backend_on_the_oval(live: LiveVehicle, oval_labels) -> None:
    scans, labels = oval_labels
    assert len(labels) > 10
    geo = core.TrackPerceiver(core.BACKEND_GEOMETRIC, live)
    s = ev.score_backend(
        geo,
        scans,
        labels.index,
        labels.label,
        labels.valid_count,
        labels.path_xy,
        live.body.half_width_m,
    )
    assert s["error"].shape == (len(labels), 16)
    assert np.nanmean(s["error"]) < 0.1  # the same tracer on one scan and on the submap
    assert np.nanmean(s["containment"]) > 0.8
    assert (s["valid"] >= 6).all()


def test_evaluate_bag_and_main_read_the_labels_and_the_scan_cache(
    tmp_path, live: LiveVehicle, oval_labels, capsys
) -> None:
    scans, labels = oval_labels
    cache = tmp_path / "cache"
    cache.mkdir()
    np.savez(
        cache / "fakebag.scans.npz",
        stamp_s=scans.stamp_s,
        ranges=scans.ranges,
        angle_min=scans.angle_min,
        angle_increment=scans.angle_increment,
        range_min=scans.range_min,
        range_max=scans.range_max,
        speed_t=np.zeros(1),
        speed_mps=np.zeros(1),
    )
    labels.save(tmp_path / "fakebag.npz")
    perceivers = {core.BACKEND_GEOMETRIC: core.TrackPerceiver(core.BACKEND_GEOMETRIC, live)}
    bag = tmp_path / "fakebag"
    result = ev.evaluate_bag(bag, tmp_path, perceivers, live)
    assert result["labels"] == len(labels)
    assert result["label_path_containment_mean"] >= sl.MIN_PATH_CONTAINMENT
    assert result["backends"]["geometric"]["scans"] == len(labels)
    out_json = tmp_path / "out.json"
    assert ev.main(["--labels", str(tmp_path), "--bags", str(bag), "--out", str(out_json)]) == 0
    table = capsys.readouterr().out
    assert "fakebag: " in table and "geometric" in table and "per station mean" in table
    assert json.loads(out_json.read_text())["bags"]["fakebag"]["labels"] == len(labels)


def test_evaluate_bag_without_labels(tmp_path, live: LiveVehicle, oval_labels) -> None:
    scans, _ = oval_labels
    cache = tmp_path / "cache"
    cache.mkdir()
    np.savez(
        cache / "nobag.scans.npz",
        stamp_s=scans.stamp_s,
        ranges=scans.ranges,
        angle_min=scans.angle_min,
        angle_increment=scans.angle_increment,
        range_min=scans.range_min,
        range_max=scans.range_max,
        speed_t=np.zeros(1),
        speed_mps=np.zeros(1),
    )
    none = sl.make_labels(scans, true_motion(np.zeros((len(scans), 3))), live, np.zeros(64, bool))
    assert len(none) == 0 and none.tensor.shape == (0, 1, 2, 360)
    none.save(tmp_path / "nobag.npz")
    perceivers = {core.BACKEND_GEOMETRIC: core.TrackPerceiver(core.BACKEND_GEOMETRIC, live)}
    result = ev.evaluate_bag(tmp_path / "nobag", tmp_path, perceivers, live)
    assert result["labels"] == 0 and result["backends"] == {}
    assert "nobag: 0 labels" in ev.format_table({"bags": {"nobag": result}})
