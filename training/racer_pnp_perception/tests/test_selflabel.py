"""Self-labels without a map: submap chaining, window drops, the driven-path check and the whole
pipeline against the simulator's true centreline."""

from __future__ import annotations

import math

import numpy as np
import pytest
from racer_pnp_perception import selflabel as sl
from racer_pnp_perception.raycast import ScanGeometry, lidar_scan
from racer_pnp_perception.sample import LaneGeometry, label_for_pose
from racer_pnp_perception.tracks import IslandParams, island_world
from racer_track_perception import preprocess
from racer_track_perception import track as tr
from racer_track_perception.vehicle import LiveVehicle

GEOMETRY = ScanGeometry()
SPEED_MPS = 1.5
DT_S = 0.1


@pytest.fixture(scope="module")
def world():
    return island_world(IslandParams(1.0, 3.0, 2.0, 0.4))


def drive_poses(world, n: int, start_s: float = 0.5) -> np.ndarray:
    """Poses (x, y, yaw) of a car driving the centreline counter-clockwise at SPEED_MPS."""
    lane = LaneGeometry(world.centreline)
    s = start_s + SPEED_MPS * DT_S * np.arange(n)
    pos, tan, _ = lane.at(s)
    return np.column_stack([pos[:, 0], pos[:, 1], np.arctan2(tan[:, 1], tan[:, 0])])


def scans_from_poses(world, live: LiveVehicle, poses: np.ndarray) -> sl.ScanSet:
    ranges = np.array(
        [lidar_scan(tuple(p), live.mount, GEOMETRY, world.walls) for p in poses], dtype=np.float32
    )
    ranges[~np.isfinite(ranges)] = np.inf
    return sl.ScanSet(
        DT_S * np.arange(len(poses)),
        ranges,
        GEOMETRY.angle_min_rad,
        GEOMETRY.angle_increment_rad,
        GEOMETRY.range_min_m,
        GEOMETRY.range_max_m,
    )


def true_motion(poses: np.ndarray) -> sl.MotionSeries:
    """The per interval motion in the frame at the interval start, from known poses."""
    n = len(poses) - 1
    dx, dy, dyaw = np.zeros(n), np.zeros(n), np.zeros(n)
    for i in range(n):
        c, s = math.cos(poses[i, 2]), math.sin(poses[i, 2])
        ex, ey = poses[i + 1, 0] - poses[i, 0], poses[i + 1, 1] - poses[i, 1]
        dx[i], dy[i] = c * ex + s * ey, -s * ex + c * ey
        dyaw[i] = (poses[i + 1, 2] - poses[i, 2] + math.pi) % (2.0 * math.pi) - math.pi
    return sl.MotionSeries(dx, dy, dyaw, np.zeros(n), np.ones(n), np.ones(n, dtype=bool))


def distance_to_segments(points: np.ndarray, segments: np.ndarray) -> np.ndarray:
    a, b = segments[:, :2], segments[:, 2:]
    ab = b - a
    t = np.clip(
        ((points[:, None, :] - a[None]) * ab[None]).sum(2)
        / np.maximum((ab**2).sum(1), 1e-12)[None],
        0.0,
        1.0,
    )
    closest = a[None] + t[..., None] * ab[None]
    return np.sqrt(((points[:, None, :] - closest) ** 2).sum(2)).min(axis=1)


def straight_track(half_width: float = 0.5, valid: int = 16) -> tr.LocalTrackData:
    return tr.assemble(
        0.2 * np.arange(16),
        np.zeros(16),
        np.full(16, half_width),
        np.full(16, half_width),
        np.concatenate([np.full(valid, 0.05), np.full(16 - valid, 0.9)]),
        0.25,
    )


def test_submap_from_known_poses_recovers_the_walls(world, live: LiveVehicle) -> None:
    poses = drive_poses(world, 60)
    scans = scans_from_poses(world, live, poses)
    chain = sl.chain_motion(true_motion(poses))
    points = sl.scan_points_list(scans, live, np.ones(len(scans), dtype=bool))
    i = 20
    lo, hi = sl.window_bounds(chain, i)
    assert sl.window_status(chain, i, lo, hi) is None
    assert hi - lo > 20  # a window of about 5 m of travel
    rel = sl.relative_poses(chain, i, lo, hi)
    assert np.allclose(rel[i - lo], 0.0, atol=1e-9)
    sub = sl.build_submap(points[lo : hi + 1], rel)
    # The submap is in the frame of scan i: move the world walls there.
    x, y, yaw = poses[i]
    c, s = math.cos(yaw), math.sin(yaw)

    def to_frame(p: np.ndarray) -> np.ndarray:
        d = p - np.array([x, y])
        return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], axis=1)

    seg = np.hstack([to_frame(world.walls[:, :2]), to_frame(world.walls[:, 2:])])
    dist = distance_to_segments(sub, seg)
    assert len(sub) > 300
    assert dist.max() < 0.03, dist.max()


def test_window_bounds_follow_the_travelled_distance(world, live: LiveVehicle) -> None:
    poses = drive_poses(world, 120)
    chain = sl.chain_motion(true_motion(poses))
    lo, hi = sl.window_bounds(chain, 40)
    # 1.0 m behind at 0.15 m a scan is 6 scans, 4.0 m ahead is 26.
    assert (40 - lo, hi - 40) == (6, 26)
    assert sl.window_bounds(chain, 0)[0] == 0
    assert sl.window_bounds(chain, 119)[1] == 119
    # A stopped car: the window is capped at 80 scans.
    still = sl.MotionSeries(
        np.zeros(200), np.zeros(200), np.zeros(200), np.zeros(200), np.ones(200), np.ones(200, bool)
    )
    lo, hi = sl.window_bounds(sl.chain_motion(still), 100)
    assert hi - lo + 1 == sl.MAX_WINDOW_SCANS


def test_window_with_a_failed_interval_is_dropped(world, live: LiveVehicle) -> None:
    poses = drive_poses(world, 60)
    scans = scans_from_poses(world, live, poses)
    series = true_motion(poses)
    points = sl.scan_points_list(scans, live, np.ones(len(scans), dtype=bool))
    ok = series.ok.copy()
    ok[30] = False  # one interval inside the window of scan 20
    broken = sl.MotionSeries(
        series.dx_m, series.dy_m, series.dyaw_rad, series.rms_m, series.inlier_share, ok
    )
    chain = sl.chain_motion(broken)
    assert sl.label_scan(chain, points, 20, live).reason == "icp_failed"
    # A scan whose window ends before the failure is not affected by it.
    lo, hi = sl.window_bounds(chain, 0)
    assert hi < 30 and sl.window_status(chain, 0, lo, hi) is None
    assert sl.label_scan(chain, points, 0, live).reason != "icp_failed"


def test_window_with_chained_yaw_over_two_pi_is_dropped() -> None:
    n = 40
    series = sl.MotionSeries(
        np.full(n, 0.15),
        np.zeros(n),
        np.full(n, 0.4),  # 0.4 rad per scan: 16 rad over the window
        np.zeros(n),
        np.ones(n),
        np.ones(n, dtype=bool),
    )
    chain = sl.chain_motion(series)
    lo, hi = sl.window_bounds(chain, 10)
    assert sl.window_status(chain, 10, lo, hi) == "yaw_wrap"


def test_a_short_drive_has_no_driven_path(world, live: LiveVehicle) -> None:
    poses = drive_poses(world, 60)
    scans = scans_from_poses(world, live, poses)
    chain = sl.chain_motion(true_motion(poses))
    points = sl.scan_points_list(scans, live, np.ones(len(scans), dtype=bool))
    assert sl.label_scan(chain, points, 58, live).reason == "path_short"


def test_driven_path_is_resampled_over_two_metres() -> None:
    n = 60
    series = sl.MotionSeries(
        np.full(n, 0.15), np.zeros(n), np.zeros(n), np.zeros(n), np.ones(n), np.ones(n, dtype=bool)
    )
    chain = sl.chain_motion(series)
    path = sl.driven_path(chain, 5, 40)
    assert path is not None and path.shape == (sl.PATH_POINTS, 2)
    assert np.allclose(path[:, 0], np.linspace(0.0, 2.0, sl.PATH_POINTS), atol=1e-9)
    assert np.allclose(path[:, 1], 0.0)
    assert sl.driven_path(chain, 5, 10) is None  # only 0.75 m


def test_path_containment_passes_a_centred_path_and_fails_one_leaving_the_corridor() -> None:
    t = straight_track(0.5)
    body_hw = 0.155
    x = np.linspace(0.0, 2.0, sl.PATH_POINTS)
    centred = np.stack([x, np.zeros_like(x)], axis=1)
    assert sl.path_containment(t, centred, body_hw) == 1.0
    # Drifting to the left wall: inside while |y| <= 0.5 - 0.155 = 0.345.
    drift = np.stack([x, 0.5 * x], axis=1)
    share = sl.path_containment(t, drift, body_hw)
    assert abs(share - np.mean(0.5 * x <= 0.345 + 1e-12)) < 1e-12 and share < 0.95
    # Hugging the right wall by less than the half body width fails everywhere.
    assert sl.path_containment(t, np.stack([x, np.full_like(x, -0.4)], axis=1), body_hw) == 0.0
    # Beyond the valid stations the path is outside.
    short = straight_track(0.5, valid=6)  # stations 0 to 1.0 m
    assert sl.path_containment(short, centred, body_hw) < 0.95
    assert math.isnan(sl.path_containment(tr.no_track(), centred, body_hw))
    assert math.isnan(sl.path_containment(t, np.zeros((0, 2)), body_hw))


def test_voxel_thin_keeps_one_point_per_cell() -> None:
    pts = np.array([[0.001, 0.001], [0.002, 0.002], [0.5, 0.5], [0.501, 0.499]])
    out = sl.voxel_thin(pts)
    assert len(out) == 2
    assert np.allclose(sorted(out[:, 0]), [0.0015, 0.5005], atol=1e-9)
    assert len(sl.voxel_thin(np.zeros((0, 2)))) == 0


def test_chain_motion_and_relative_poses() -> None:
    series = sl.MotionSeries(
        np.array([1.0, 1.0, 1.0]),
        np.zeros(3),
        np.array([math.pi / 2, 0.0, 0.0]),
        np.zeros(3),
        np.ones(3),
        np.array([True, True, True]),
    )
    chain = sl.chain_motion(series)
    assert np.allclose(chain.poses[1], [1.0, 0.0, math.pi / 2])
    assert np.allclose(chain.poses[2], [1.0, 1.0, math.pi / 2])  # the next metre is to the left
    assert np.allclose(chain.travel_m, [0, 1, 2, 3])
    rel = sl.relative_poses(chain, 2, 1, 3)
    assert np.allclose(rel[0], [-1.0, 0.0, 0.0])
    assert np.allclose(rel[2], [1.0, 0.0, 0.0])


def test_contract_and_label_layout() -> None:
    t = straight_track()
    assert sl.contract_ok(t)
    label = sl.label_array(t)
    assert label.shape == (16, 5) and label.dtype == np.float32
    assert np.allclose(label[:, 0], t.x_m) and np.allclose(label[:6, 4], math.log(0.05), atol=1e-6)
    # Travelling backwards (direction against the heading) is refused.
    back = tr.assemble(
        -0.2 * np.arange(16),
        np.zeros(16),
        np.full(16, 0.5),
        np.full(16, 0.5),
        np.full(16, 0.05),
        0.25,
    )
    assert not sl.contract_ok(back)
    # Station 0 behind the nearest centreline point is refused.
    shifted = tr.assemble(
        -0.5 + 0.2 * np.arange(16),
        np.zeros(16),
        np.full(16, 0.5),
        np.full(16, 0.5),
        np.full(16, 0.05),
        0.25,
    )
    assert not sl.contract_ok(shifted)
    assert not sl.contract_ok(tr.no_track())


@pytest.fixture(scope="module")
def oval_run(world, live: LiveVehicle):
    """64 scans of a drive round the oval, the ICP motion between them (about 12 s of ICP)."""
    poses = drive_poses(world, 64)
    scans = scans_from_poses(world, live, poses)
    return poses, scans, sl.compute_motion(scans, live)


def test_labels_agree_with_the_true_centreline_on_the_oval(
    world, live: LiveVehicle, oval_run
) -> None:
    """The pipeline, with ICP motion, against the simulator's own centreline."""
    poses, scans, series = oval_run
    assert series.ok.mean() > 0.95
    candidates = np.zeros(len(scans), dtype=bool)
    candidates[8:34] = True  # the window ahead (4 m at 1.5 m/s) and the driven path fit
    labels = sl.make_labels(scans, series, live, candidates)
    assert len(labels) >= 0.5 * candidates.sum(), labels.dropped  # the yield on the oval
    lane = LaneGeometry(world.centreline)
    err = np.full((len(labels), 16), np.nan)
    for k, i in enumerate(labels.index):
        truth = label_for_pose(lane, world.walls, tuple(poses[i]))
        n = labels.valid_count[k]
        err[k, :n] = np.hypot(
            labels.label[k, :n, 0] - truth[:n, 0], labels.label[k, :n, 1] - truth[:n, 1]
        )
    mean_by_station = np.nanmean(err, axis=0)
    assert np.all(np.isfinite(mean_by_station[:12]))
    assert np.nanmax(mean_by_station[:12]) < 0.05, mean_by_station
    # The gate numbers of a kept label are within the gates.
    assert (labels.intrusion <= sl.MAX_INTRUSION).all()
    assert (labels.deficit_p90_m <= sl.MAX_DEFICIT_P90_M).all()
    assert (labels.path_containment >= sl.MIN_PATH_CONTAINMENT).all()
    # The scan tensor is exactly what the net gets.
    i0 = int(labels.index[0])
    want = preprocess.scan_to_tensor(
        scans.ranges[i0],
        scans.angle_min,
        scans.angle_increment,
        scans.range_min,
        scans.range_max,
        live.mount,
        live.body,
    )
    assert np.array_equal(labels.tensor[0], want) and labels.tensor.shape[1:] == (1, 2, 360)


def test_icp_chain_follows_the_true_poses_over_a_window(oval_run) -> None:
    """Over the length of a submap window (30 scans, 4.5 m at 1.5 m/s) the chained ICP motion drifts
    by under 0.12 m and 0.07 rad (measured: 0.087 m, 0.053 rad in the rounded rectangle lane)."""
    poses, _, series = oval_run
    chain = sl.chain_motion(series)
    truth = sl.chain_motion(true_motion(poses))
    worst_xy, worst_yaw = 0.0, 0.0
    for i in range(0, len(poses) - 30, 5):
        got = sl.relative_poses(chain, i, i, i + 30)[-1]
        want = sl.relative_poses(truth, i, i, i + 30)[-1]
        worst_xy = max(worst_xy, float(np.hypot(*(got[:2] - want[:2]))))
        worst_yaw = max(worst_yaw, abs(float(got[2] - want[2])))
    assert worst_xy < 0.12 and worst_yaw < 0.07, (worst_xy, worst_yaw)


def test_labels_save_and_motion_roundtrip(tmp_path, world, live: LiveVehicle) -> None:
    poses = drive_poses(world, 60)
    scans = scans_from_poses(world, live, poses)
    series = true_motion(poses)
    sl.save_motion(tmp_path / "m.npz", series)
    back = sl.load_motion(tmp_path / "m.npz")
    assert np.array_equal(back.ok, series.ok) and np.allclose(back.dx_m, series.dx_m)
    cand = np.zeros(len(scans), dtype=bool)
    cand[8:20] = True
    labels = sl.make_labels(scans, series, live, cand)
    labels.save(tmp_path / "l.npz")
    with np.load(tmp_path / "l.npz") as z:
        assert z["label"].shape == (len(labels), 16, 5) and z["tensor"].shape[1:] == (1, 2, 360)
        assert z["path_xy"].shape[1:] == (sl.PATH_POINTS, 2)
    assert sum(labels.dropped.values()) + len(labels) == labels.candidates
    assert set(labels.dropped) == set(sl.REASONS)


def test_intervals_to_match_cover_the_windows_only() -> None:
    cand = np.zeros(300, dtype=bool)
    cand[150] = True
    want = sl.intervals_to_match(cand)
    assert want.sum() == sl.MAX_BEHIND_SCANS + sl.MAX_AHEAD_SCANS
    assert not want[:100].any() and not want[250:].any()


def test_bag_units_by_date() -> None:
    assert not sl.bag_in_current_units("2026-11-11T04-20-19_car_teleop_v3")
    assert sl.bag_in_current_units("2026-11-12T03-04-56_car_teleop")


def test_compute_motion_flags_a_bad_interval(world, live: LiveVehicle) -> None:
    poses = drive_poses(world, 6)
    scans = scans_from_poses(world, live, poses)
    bad = sl.ScanSet(
        scans.stamp_s,
        scans.ranges.copy(),
        scans.angle_min,
        scans.angle_increment,
        scans.range_min,
        scans.range_max,
    )
    bad.ranges[3] = np.inf  # no returns: too few points for ICP
    series = sl.compute_motion(bad, live)
    assert series.ok[:4].tolist() == [True, True, False, False]
    assert np.isnan(series.dx_m[2:4]).all()
    # After a failure the next interval starts again from the identity (the corner it is in may
    # then fail the quality flags, which is the flag working, so only the numbers are checked).
    assert np.isfinite(series.dx_m[4])
    want = np.array([True, False, False, False, False])
    only = sl.compute_motion(scans, live, want)
    assert only.ok.tolist() == [True, False, False, False, False]
