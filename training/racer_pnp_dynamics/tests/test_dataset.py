"""L1 for dataset.py: the split is by time, no sample is in two splits, nested bags are dropped,
the held-out bag never appears in training."""

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


def test_split_boundaries_are_70_10_20_of_the_span():
    run = _run("a", 100.0, 100.0)
    b1, _ = dataset.split_times(run.t_s[0], run.t_s[-1])
    assert b1 == run.t_s[0] + 0.7 * (run.t_s[-1] - run.t_s[0])
    which = dataset.split_of_time(run, run.t_s)
    assert np.mean(which == 0) == np.float64(np.mean(run.t_s < b1))
    assert 0.69 < np.mean(which == 0) < 0.71 and 0.09 < np.mean(which == 1) < 0.11
    assert 0.19 < np.mean(which == 2) < 0.21 and (np.diff(which) >= 0).all()


def test_no_sample_is_in_two_splits_and_segments_cover_each_split_once():
    run = _run("a", 0.0, 100.0)
    segs = dataset.make_segments(run)
    spans = sorted((s.i0, s.i1) for s in segs)
    assert all(a1 <= b0 for (_, a1), (b0, _) in pairwise(spans))
    assert {s.split for s in segs} == {"train", "val", "heldout"}
    train = [s for s in segs if s.split == "train"]
    heldout = [s for s in segs if s.split == "heldout"]
    assert max(s.t1_s for s in train) < min(s.t0_s for s in heldout)
    assert dataset.seconds(segs) > 95.0


def test_held_out_bag_is_entirely_held_out():
    run = _run(dataset.HELD_OUT_BAG, 0.0, 60.0)
    segs = dataset.make_segments(run)
    assert segs and {s.split for s in segs} == {"heldout"}
    assert dataset.by_split(segs, "train") == [] and dataset.by_split(segs, "val") == []


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
