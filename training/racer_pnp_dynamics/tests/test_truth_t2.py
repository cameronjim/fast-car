"""L1 for T2, the wheel-seeded ICP truth (brief 31): the wheel seed against hand values, the matcher
started from it on a scene where the identity start under-reads, the cache keys, the yaw gain
table of the vehicle."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
from follower_replay.bag import ScanRecord
from racer_pnp_dynamics import icp_motion as im
from racer_pnp_dynamics import wheel_seed
from racer_pnp_dynamics.bags import GRID_DT_S, Run

N_BEAMS = 720
INC = 2 * np.pi / N_BEAMS
TABLE = ((0.10, 1.12), (0.20, 0.96), (0.30, 0.92), (0.42, 0.86))


def _run(speed: float, steer: float, seconds: float = 2.0, wheel: float | None = None) -> Run:
    t = 100.0 + np.arange(int(seconds / GRID_DT_S)) * GRID_DT_S
    return Run(
        "r",
        t,
        np.full_like(t, speed),
        np.full_like(t, steer),
        np.full_like(t, speed if wheel is None else wheel),
        [],
    )


def test_seed_straight_is_all_path_no_turn(toy_vehicle):
    dyaw, path = wheel_seed.interval_seed(_run(1.0, 0.0), toy_vehicle, [100.5], [100.6])
    assert dyaw[0] == pytest.approx(0.0, abs=1e-12)
    assert path[0] == pytest.approx(0.1)


def test_seed_left_arc_hand_value(toy_vehicle):
    # v tan(delta) / L * dt = 1.0 * tan(0.2) / 0.5 * 0.1
    dyaw, path = wheel_seed.interval_seed(_run(1.0, 0.2), toy_vehicle, [100.5], [100.6])
    assert dyaw[0] == pytest.approx(np.tan(0.2) / 0.5 * 0.1, rel=1e-9)
    assert path[0] == pytest.approx(0.1)


def test_seed_right_arc_is_the_mirror(toy_vehicle):
    left, _ = wheel_seed.interval_seed(_run(0.8, 0.25), toy_vehicle, [100.5], [100.6])
    right, _ = wheel_seed.interval_seed(_run(0.8, -0.25), toy_vehicle, [100.5], [100.6])
    assert right[0] == pytest.approx(-left[0], rel=1e-12)
    assert right[0] < 0.0


def test_seed_uses_the_yaw_gain_table_and_the_wheelbase(toy_vehicle):
    veh = replace(toy_vehicle, yaw_gain_table=TABLE)
    dyaw, _ = wheel_seed.interval_seed(_run(1.0, 0.2), veh, [100.5], [100.6])
    assert dyaw[0] == pytest.approx(0.96 * np.tan(0.2) / 0.5 * 0.1, rel=1e-9)  # knot at 0.2
    long_car = replace(veh, wheelbase_m=1.0)
    dyaw2, _ = wheel_seed.interval_seed(_run(1.0, 0.2), long_car, [100.5], [100.6])
    assert dyaw2[0] == pytest.approx(dyaw[0] / 2.0, rel=1e-9)


def test_seed_is_in_the_rear_axle_frame_whatever_the_lidar_mount(toy_vehicle):
    """The seed is the rear axle's motion; the LiDAR mount (yaw pi on the car) is applied by the
    matcher's scan_points, so changing the mount must not change the seed."""
    other = replace(toy_vehicle, lidar_yaw_rad=0.0, lidar_x_m=0.0)
    a = wheel_seed.interval_seed(_run(1.0, 0.2), toy_vehicle, [100.5], [100.6])
    b = wheel_seed.interval_seed(_run(1.0, 0.2), other, [100.5], [100.6])
    np.testing.assert_array_equal(a[0], b[0])
    np.testing.assert_array_equal(a[1], b[1])


def test_seed_missing_inputs_give_nan(toy_vehicle):
    run = _run(1.0, 0.2)
    run.steering_cmd_rad[25:30] = np.nan  # a stale /drive inside the first interval
    dyaw, path = wheel_seed.interval_seed(run, toy_vehicle, [100.5, 101.0], [100.6, 101.1])
    assert np.isnan(dyaw[0]) and np.isfinite(path[0])  # the path needs only the speed
    assert np.isfinite(dyaw[1])
    run.wheel_speed_mps[:] = np.nan
    dyaw, path = wheel_seed.interval_seed(run, toy_vehicle, [101.0], [101.1])
    assert np.isnan(dyaw[0]) and np.isnan(path[0])


def test_seed_steering_is_held_between_messages(toy_vehicle):
    """Zero order hold: a command that arrives at 100.54 counts from there on, six of the ten
    midpoints of the interval 100.5 to 100.6 (100.545 and later), not from the next message."""
    run = _run(1.0, 0.0)
    k = round((100.54 - run.t_s[0]) / GRID_DT_S)
    run.steering_cmd_rad[k:] = 0.2
    dyaw, _ = wheel_seed.interval_seed(run, toy_vehicle, [100.5], [100.6])
    assert dyaw[0] == pytest.approx(0.6 * np.tan(0.2) / 0.5 * 0.1, rel=1e-9)


# ---- the vehicle's yaw gain table ----------------------------------------------------------


def test_yaw_gain_is_piecewise_linear_and_held_outside(toy_vehicle):
    veh = replace(toy_vehicle, yaw_gain_table=TABLE)
    np.testing.assert_allclose(veh.yaw_gain([0.0, 0.05, 0.10]), [1.12, 1.12, 1.12])  # held below
    assert veh.yaw_gain(0.15) == pytest.approx(1.04)  # half way between 1.12 and 0.96
    assert veh.yaw_gain(-0.15) == pytest.approx(1.04)  # |delta|
    assert veh.yaw_gain(0.9) == pytest.approx(0.86)  # held above
    np.testing.assert_array_equal(toy_vehicle.yaw_gain([0.0, 0.3, -0.5]), np.ones(3))  # no table


def test_the_live_vehicle_carries_the_measured_table():
    from racer_pnp_dynamics.vehicle import load_vehicle

    veh = load_vehicle()
    assert len(veh.yaw_gain_table) >= 2
    gains = [g for _, g in veh.yaw_gain_table]
    assert all(0.5 < g < 1.5 for g in gains)
    knots = [a for a, _ in veh.yaw_gain_table]
    assert knots == sorted(knots)


# ---- T2 on a scene where the identity start under-reads ------------------------------------


def _cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def _round_room(radius: float = 1.6, n: int = 180) -> np.ndarray:
    """A round room with one post: turning about the centre is hidden from every wall point (the
    wall maps onto itself), only the post shows it. A start at the identity slides along the wall
    and reads almost no turn although the match looks clean."""
    ang = np.linspace(0.0, 2.0 * np.pi, n + 1)
    pts = np.stack([radius * np.cos(ang), radius * np.sin(ang)], axis=1)
    segs = [(*pts[i], *pts[i + 1]) for i in range(n)]
    cx, cy, w = 1.25 * np.cos(0.6), 1.25 * np.sin(0.6), 0.12
    segs += [
        (cx - w, cy - w, cx + w, cy - w),
        (cx + w, cy - w, cx + w, cy + w),
        (cx + w, cy + w, cx - w, cy + w),
        (cx - w, cy + w, cx - w, cy - w),
    ]
    return np.array(segs, dtype=float)


def _scan(room: np.ndarray, pose, mount, stamp: float) -> ScanRecord:
    """Ray cast from the lidar of a car whose rear axle is at `pose` (x, y, yaw)."""
    x, y, yaw = pose
    origin = np.array(
        [
            x + mount.x_m * np.cos(yaw) - mount.y_m * np.sin(yaw),
            y + mount.x_m * np.sin(yaw) + mount.y_m * np.cos(yaw),
        ]
    )
    world = yaw + (-np.pi + np.arange(N_BEAMS) * INC) + mount.yaw_rad
    d = np.stack([np.cos(world), np.sin(world)], axis=1)
    ranges = np.full(N_BEAMS, np.inf)
    for x1, y1, x2, y2 in room:
        p1 = np.array([x1, y1])
        e = np.array([x2 - x1, y2 - y1])
        denom = _cross(d, e)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = _cross(p1 - origin, e) / denom
            u = _cross(p1 - origin, d) / denom
        hit = (t > 0) & (u >= 0) & (u <= 1)
        ranges[hit] = np.minimum(ranges[hit], t[hit])
    return ScanRecord(stamp, -np.pi, INC, 0.05, 12.0, tuple(float(r) for r in ranges))


def _turning_scene(toy_vehicle, turn: float = 0.2, path: float = 0.1):
    """Two scans of the round room 0.1 s apart (the car turns by `turn` rad along `path` m) and the
    run whose wheel speed and steering say exactly that."""
    mount = im.mount_of(toy_vehicle)
    room = _round_room()
    half = turn / 2.0
    poses = [(0.0, 0.0, 0.0), (path * np.cos(half), path * np.sin(half), turn)]
    scans = [_scan(room, p, mount, 100.0 + 0.1 * i) for i, p in enumerate(poses)]
    # steering that gives `turn` over 0.1 s at path / 0.1 m/s: v tan(delta) / L * 0.1 = turn
    v = path / 0.1
    delta = float(np.arctan(turn * toy_vehicle.wheelbase_m / (v * 0.1)))
    t = 99.0 + np.arange(200) * GRID_DT_S
    run = Run("room", t, np.full_like(t, v), np.full_like(t, delta), np.full_like(t, v), scans)
    return scans, run


def test_wheel_seed_recovers_the_turn_where_the_identity_start_under_reads(toy_vehicle):
    scans, run = _turning_scene(toy_vehicle, turn=0.2, path=0.1)
    t1 = im.compute_motion(scans, toy_vehicle, ignore=frozenset())
    t2 = im.compute_motion(scans, toy_vehicle, ignore=frozenset(), seed=im.SEED_WHEEL, run=run)
    assert t2.ok[0] and t2.seeded is not None and t2.seeded[0]
    assert t2.dyaw_rad[0] == pytest.approx(0.2, abs=0.01)
    assert t1.dyaw_rad[0] < 0.1  # the identity start reads under half of the turn ...
    assert t1.ok[0]  # ... and the quality flags do not notice
    assert t2.ground_speed_mps()[0] == pytest.approx(1.0, abs=0.1)


def test_t1_does_not_carry_a_seed_flag(toy_vehicle):
    scans, _ = _turning_scene(toy_vehicle)
    assert im.compute_motion(scans, toy_vehicle, ignore=frozenset()).seeded is None


def test_an_interval_without_a_seed_is_matched_from_the_identity(toy_vehicle):
    scans, run = _turning_scene(toy_vehicle)
    run.wheel_speed_mps[:] = np.nan
    t2 = im.compute_motion(scans, toy_vehicle, ignore=frozenset(), seed="wheel", run=run)
    t1 = im.compute_motion(scans, toy_vehicle, ignore=frozenset())
    assert not t2.seeded[0]
    assert t2.dyaw_rad[0] == pytest.approx(t1.dyaw_rad[0])


def test_wheel_seed_needs_the_run_and_the_seed_name_is_checked(toy_vehicle):
    scans, _ = _turning_scene(toy_vehicle)
    with pytest.raises(ValueError, match="needs the run"):
        im.compute_motion(scans, toy_vehicle, ignore=frozenset(), seed="wheel")
    with pytest.raises(ValueError, match="seed must be one of"):
        im.compute_motion(scans, toy_vehicle, ignore=frozenset(), seed="gps")


def test_unwanted_intervals_get_no_seed_flag(toy_vehicle):
    scans, run = _turning_scene(toy_vehicle)
    m = im.compute_motion(
        scans, toy_vehicle, want=np.array([False]), ignore=frozenset(), seed="wheel", run=run
    )
    assert not m.ok[0] and not m.seeded[0]


# ---- caches --------------------------------------------------------------------------------


def test_cache_keys_differ_by_seed(tmp_path):
    a = im.motion_cache_path(tmp_path, "bag", "identity")
    b = im.motion_cache_path(tmp_path, "bag", "wheel")
    assert a != b
    assert a.name == "bag.motion.npz"  # the v0.2 name, so the T1 caches already on disk are found
    assert "wheel" in b.name
    assert im.motion_cache_path(tmp_path, "other", "wheel") != b
    with pytest.raises(ValueError):
        im.motion_cache_path(tmp_path, "bag", "gps")


def test_seeded_flag_round_trips_and_an_old_cache_loads(tmp_path):
    n = np.arange(3.0)
    m = im.MotionSeries(
        *(n,) * 7, ok=np.array([True, False, True]), seeded=np.array([1, 0, 1], bool)
    )
    im.save_motion(tmp_path / "w.npz", m)
    back = im.load_motion(tmp_path / "w.npz")
    np.testing.assert_array_equal(back.seeded, m.seeded)
    old = im.MotionSeries(*(n,) * 7, ok=np.array([True, False, True]))
    im.save_motion(tmp_path / "t1.npz", old)
    assert im.load_motion(tmp_path / "t1.npz").seeded is None
