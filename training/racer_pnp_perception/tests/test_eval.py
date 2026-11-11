from __future__ import annotations

import math

import numpy as np
from racer_pnp_perception import eval_bags as eb
from racer_pnp_perception import eval_sim as es
from racer_pnp_perception.raycast import ScanGeometry
from racer_track_perception import core
from racer_track_perception import track as tr
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle


def test_moving_mask_uses_the_latest_speed_sample() -> None:
    scan_t = np.array([0.0, 1.0, 2.0, 3.0])
    speed_t = np.array([0.5, 1.5, 2.5])
    speed = np.array([0.0, 0.5, -0.4])
    mask = eb.moving_mask(scan_t, speed_t, speed)
    # t=0: no sample yet; t=1: 0.0; t=2: 0.5; t=3: |-0.4| (reverse counts as moving).
    assert mask.tolist() == [False, False, True, True]
    assert eb.moving_mask(scan_t, np.array([]), np.array([])).tolist() == [False] * 4
    assert eb.moving_mask(np.array([]), speed_t, speed).shape == (0,)


def make_track(valid: int, offset: float = 0.0) -> tr.LocalTrackData:
    return tr.assemble(
        0.2 * np.arange(16),
        np.full(16, offset),
        np.full(16, 0.5),
        np.full(16, 0.5),
        np.concatenate([np.full(valid, 0.05), np.full(16 - valid, 0.9)]),
        0.25,
    )


def test_centreline_distance() -> None:
    assert abs(eb.centreline_distance(make_track(16), make_track(16, 0.1)) - 0.1) < 1e-6
    assert math.isnan(eb.centreline_distance(make_track(16), make_track(5)))


def test_summarise() -> None:
    res = np.array([0.05, 0.1, math.nan, 0.2])
    out = eb.summarise(
        res,
        np.array([0.0, 0.02, math.nan, 0.04]),
        np.array([16.0, 3.0, 0.0, 8.0]),
        np.array([1.0, 2.0]),
    )
    assert out["moving_scans"] == 4 and out["residual_scans"] == 3
    assert abs(out["boundary_residual_median_m"] - 0.1) < 1e-12
    assert abs(out["intrusion_fraction_mean"] - 0.02) < 1e-12
    assert out["share_valid_count_under_6"] == 0.5
    assert out["latency_ms"]["max"] == 2.0
    empty = eb.summarise(np.array([]), np.array([]), np.array([]))
    assert math.isnan(empty["boundary_residual_median_m"]) and "latency_ms" not in empty


def test_gate_verdict_thresholds() -> None:
    good = {
        "boundary_residual_median_m": 0.10,
        "intrusion_fraction_mean": 0.02,
        "share_valid_count_under_6": 0.05,
    }
    verdict, checks = eb.gate_verdict(good)
    assert verdict == eb.VERDICT_PASS and all(checks.values())
    for key, bad in (
        ("boundary_residual_median_m", 0.1001),
        ("intrusion_fraction_mean", 0.0201),
        ("share_valid_count_under_6", 0.0501),
        ("boundary_residual_median_m", math.nan),
    ):
        verdict, checks = eb.gate_verdict({**good, key: bad})
        assert verdict == eb.VERDICT_FAIL and not all(checks.values())
    assert eb.VERDICT_FAIL == "net fails, use backend:=geometric tonight"
    assert eb.VERDICT_PASS == "net passes"


def corridor_scan(live: LiveVehicle) -> eb.ScanRec:
    # The ROS package's synthetic lane is a test helper there; rebuild a lane scan here.
    n = 720
    inc = 2 * math.pi / n
    bearings = -math.pi + np.arange(n) * inc + live.mount.yaw_rad
    r = np.full(n, np.inf)
    s = np.sin(bearings)
    for wall_y in (0.5, -0.5):
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (wall_y - live.mount.y_m) / s
        r[t > 0] = np.minimum(r[t > 0], t[t > 0])
    r[r > 8.0] = np.inf
    return eb.ScanRec(0.0, r, -math.pi, inc, 0.05, 12.0)


def test_score_scans_two_backends(live: LiveVehicle) -> None:
    net = CircConv1dV0(tiny_arrays())
    perceivers = {
        core.BACKEND_GEOMETRIC: core.TrackPerceiver(core.BACKEND_GEOMETRIC, live),
        core.BACKEND_NET: core.TrackPerceiver(core.BACKEND_NET, live, net=net, model_id="t"),
    }
    scans = [corridor_scan(live)] * 3
    arrays, tracks, dist = eb.score_scans(perceivers, scans)
    assert set(arrays) == set(perceivers) and len(tracks) == 3 and len(dist) == 3
    assert arrays[core.BACKEND_GEOMETRIC]["valid"].tolist() == [16.0] * 3
    assert arrays[core.BACKEND_GEOMETRIC]["residual"].max() < 0.05
    pooled = eb.pool([arrays], core.BACKEND_GEOMETRIC)
    assert pooled["moving_scans"] == 3
    only_geo = eb.score_scans({core.BACKEND_GEOMETRIC: perceivers[core.BACKEND_GEOMETRIC]}, scans)
    assert all(math.isnan(d) for d in only_geo[2])


def tiny_arrays() -> dict[str, np.ndarray]:
    from test_dataset_export import random_arrays

    arrays = random_arrays()
    arrays["linear1_bias"][4::5] = math.log(0.05)
    return arrays


def test_error_summary_masks() -> None:
    pos = np.zeros((2, 16))
    pos[:, :8] = 0.02
    pos[:, 8:] = 0.2
    wid = np.full((2, 16, 2), 0.03)
    valid = np.ones((2, 16), dtype=bool)
    out = es.error_summary(pos, wid, valid)
    assert abs(out["centreline_err_median_stations_0_7_m"] - 0.02) < 1e-12
    assert abs(out["centreline_err_median_stations_8_15_m"] - 0.2) < 1e-12
    assert abs(out["half_width_err_median_m"] - 0.03) < 1e-12
    valid[:, 8:] = False
    out = es.error_summary(pos, wid, valid)
    assert math.isnan(out["centreline_err_median_stations_8_15_m"])
    assert out["valid_share"] == 0.5


def test_acceptance_flags() -> None:
    def net(near: float, far: float, width: float) -> dict:
        return {
            "net_raw": {
                "centreline_err_median_stations_0_7_m": near,
                "centreline_err_median_stations_8_15_m": far,
                "half_width_err_median_m": width,
            }
        }

    ok = es.acceptance({"a": net(0.05, 0.10, 0.07)})
    assert ok["all_ok"] and ok["a"] == {
        "centreline_0_7_ok": True,
        "centreline_8_15_ok": True,
        "half_width_ok": True,
    }
    bad = es.acceptance({"a": net(0.05, 0.10, 0.07), "b": net(0.06, 0.10, 0.07)})
    assert not bad["all_ok"] and not bad["b"]["centreline_0_7_ok"]


def test_evaluate_set_geometric_only_and_with_a_net(live: LiveVehicle) -> None:
    from racer_pnp_perception import dataset

    data = dataset.generate(12, 4, workers=1, clean=True, keep_ranges=True)
    geometry = ScanGeometry()
    out = es.evaluate_set(data, live, None, geometry)
    assert out["samples"] == 12 and "net_raw" not in out
    assert out["geometric_valid"]["valid_share"] > 0.3
    # On clean scans the geometric tracer's centreline error is small where it traces.
    assert out["geometric_valid"]["centreline_err_median_stations_0_7_m"] < 0.15
    net = CircConv1dV0(tiny_arrays())
    both = es.evaluate_set(data, live, net, geometry)
    assert {"net_raw", "net_valid", "net_sigma_median_m"} <= set(both)
