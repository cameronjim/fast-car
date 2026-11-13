"""L1 for dataset.py: the split is by time, no sample is in two splits, nested bags are dropped,
every bag's held-out tail is after its training part."""

from __future__ import annotations

from itertools import pairwise

import numpy as np
from racer_pnp_dynamics import dataset
from racer_pnp_dynamics.bags import Run
from racer_pnp_dynamics.model import PLACEHOLDER_NOMINAL
from synth import make_run


def _run(name, t0, seconds, speed=0.4, odom=True):
    t = t0 + np.arange(int(seconds * 50)) / 50.0
    wheel = np.full_like(t, speed) if odom else np.full_like(t, np.nan)
    return Run(name, t, np.full_like(t, speed), np.zeros_like(t), wheel, [])


def test_split_is_70_30_of_the_moving_data_and_has_no_validation_part():
    run = _run("a", 100.0, 100.0)
    b1, b2 = dataset.split_times(run)
    assert b1 == b2 and b1 == run.t_s[70 * 50]
    which = dataset.split_of_time(run, run.t_s)
    assert 0.69 < np.mean(which == 0) < 0.71 and np.mean(which == 1) == 0.0
    assert 0.29 < np.mean(which == 2) < 0.31 and (np.diff(which) >= 0).all()


def test_split_counts_moving_samples_not_the_time_span():
    run = _run("a", 0.0, 100.0)
    run.speed_cmd_mps[:2500] = 0.0  # the first half of the bag stands still
    run.wheel_speed_mps[:2500] = 0.0
    b1, _ = dataset.split_times(run)
    moving = np.flatnonzero(dataset.driving_mask(run))
    assert len(moving) == 2500
    assert b1 == run.t_s[moving[int(np.ceil(0.7 * 2500))]]
    assert 0.84 < (b1 - run.t_s[0]) / 100.0 < 0.86  # 50 s idle + 70 percent of 50 s moving


def test_no_sample_is_in_two_splits_and_segments_cover_each_split_once():
    run = _run("a", 0.0, 100.0)
    segs = dataset.make_segments(run)
    spans = sorted((s.i0, s.i1) for s in segs)
    assert all(a1 <= b0 for (_, a1), (b0, _) in pairwise(spans))
    assert {s.split for s in segs} == {"train", "heldout"}
    train = [s for s in segs if s.split == "train"]
    heldout = [s for s in segs if s.split == "heldout"]
    assert max(s.t1_s for s in train) < min(s.t0_s for s in heldout)
    assert dataset.seconds(segs) > 95.0


def test_every_bag_has_a_held_out_tail_and_none_is_held_out_whole():
    for name in ("2026-11-11T04-20-19_car_teleop_v3", "2026-11-12T03-04-56_car_teleop"):
        segs = dataset.make_segments(_run(name, 0.0, 60.0))
        assert {s.split for s in segs} == {"train", "heldout"}
        assert dataset.by_split(segs, "val") == []


def test_by_split_odom_filter():
    segs = dataset.make_segments(_run("a", 0.0, 60.0)) + dataset.make_segments(
        _run("old", 0.0, 60.0, odom=False)
    )
    assert all(s.run.has_odom for s in dataset.by_split(segs, "train", odom_only=True))
    assert any(not s.run.has_odom for s in dataset.by_split(segs, "train"))


def test_nested_copies_are_dropped_longest_kept():
    long = _run("long", 0.0, 100.0)
    prefix = _run("prefix", 0.0, 40.0)
    other = _run("other", 500.0, 30.0)
    kept = {r.name for r in dataset.drop_nested([prefix, long, other])}
    assert kept == {"long", "other"}
    twin_a, twin_b = _run("a", 0.0, 10.0), _run("b", 0.0, 10.0)
    assert len(dataset.drop_nested([twin_a, twin_b])) == 1


def test_driving_mask_bridges_short_stops_and_splits_at_gaps():
    run = _run("a", 0.0, 30.0)
    run.speed_cmd_mps[500:540] = 0.0  # 0.8 s stop: bridged
    run.wheel_speed_mps[500:540] = 0.0
    run.speed_cmd_mps[900:1100] = 0.0  # 4 s stop: a split
    run.wheel_speed_mps[900:1100] = 0.0
    run.speed_cmd_mps[1200:1210] = np.nan  # command gap: not valid, not driving
    mask = dataset.driving_mask(run)
    assert mask[510] and not mask[1000] and not mask[1205]
    segs = dataset.make_segments(run)
    assert not any(s.i0 < 1000 < s.i1 for s in segs)


def test_short_segments_are_dropped():
    run = _run("a", 0.0, 100.0)
    run.speed_cmd_mps[:] = 0.0
    run.wheel_speed_mps[:] = 0.0
    run.speed_cmd_mps[100:200] = 0.4  # 2 s of driving
    run.wheel_speed_mps[100:200] = 0.4
    assert dataset.make_segments(run) == []


def test_synthetic_run_segments_have_motion(toy_vehicle):
    run, motion = make_run(PLACEHOLDER_NOMINAL, toy_vehicle, seconds=200.0)
    segs = dataset.make_segments(run, motion)
    assert all(s.motion is motion for s in segs) and dataset.seconds(segs) > 100.0


def test_yaw_data_ground_speed_is_the_icp_speed_of_the_interval(toy_vehicle):
    from racer_pnp_dynamics.bags import GRID_DT_S, Run
    from racer_pnp_dynamics.icp_motion import MotionSeries
    from racer_pnp_dynamics.windows import yaw_data

    n = 500
    t = 1000.0 + np.arange(n) * GRID_DT_S
    run = Run("hand", t, np.full(n, 0.5), np.zeros(n), np.full(n, 0.5), [])
    t_scan = np.array([t[100], t[105], t[110], t[115]])
    motion = MotionSeries(
        t_s=t_scan,
        dt_s=np.array([0.1, 0.1, 0.2, 0.1]),
        dx_m=np.array([0.03, 0.0, 0.08, 0.05]),
        dy_m=np.array([0.04, 0.0, 0.06, 0.0]),
        dyaw_rad=np.zeros(4),
        rms_m=np.zeros(4),
        inlier_share=np.ones(4),
        ok=np.array([True, True, True, False]),
    )
    seg = dataset.Segment(run=run, motion=motion, i0=0, i1=n, split="train")
    data = yaw_data([seg], 0.0)
    assert data.ground_mps.shape == (len(data),) == data.dyaw_rad.shape
    # interval 1 is standing still (wheel speed 0.5 here, so only the ICP speed matters): 0 m/s
    np.testing.assert_allclose(sorted(data.ground_mps), [0.0, 0.5, 0.5], atol=1e-12)
    assert len(yaw_data([], 0.0).ground_mps) == 0
