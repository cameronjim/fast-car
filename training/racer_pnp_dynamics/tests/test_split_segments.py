"""L1 for dataset.make_segments with the pre-registered split: the segments are exactly those
listed, the validation tail is the last 15 percent of each train segment, held-out seconds are
never in train or val, and a window never reaches over a segment edge."""

from __future__ import annotations

import numpy as np
import pytest
from follower_replay.bag import ScanRecord
from racer_pnp_dynamics import dataset, pipeline_v02
from racer_pnp_dynamics.bags import Run
from racer_pnp_dynamics.model import PLACEHOLDER_NOMINAL
from racer_pnp_dynamics.split import BagEntry, Split
from racer_pnp_dynamics.windows import speed_windows, yaw_data
from synth import make_run, make_split

SEGS = [
    ("A", 5.0, 105.0, "train", "ccw"),
    ("B", 107.0, 167.0, "heldout", "ccw"),
    ("C", 170.0, 250.0, "train", "cw"),
    ("D", 252.0, 300.0, "heldout", "cw"),
    ("E", 305.0, 330.0, "unused", "cw"),
]


def _plain_run(name, t0, seconds, speed=0.4):
    t = t0 + np.arange(int(seconds * 50)) / 50.0
    return Run(name, t, np.full_like(t, speed), np.zeros_like(t), np.full_like(t, speed), [])


def _split_segments(toy_vehicle, seconds=340.0):
    run, motion = make_run(PLACEHOLDER_NOMINAL, toy_vehicle, seconds=seconds, seed=3, name="bag-x")
    split = make_split("bag-x", SEGS)
    return run, motion, split, dataset.make_segments(run, motion, split, t_ref_s=run.t_s[0])


def test_segments_are_exactly_those_listed_and_train_ends_in_validation(toy_vehicle):
    run, _, split, segs = _split_segments(toy_vehicle)
    t0 = run.t_s[0]
    assert {s.split for s in segs} == {"train", "val", "heldout"}
    for s in segs:
        want = split.role_of("bag-x", s.t0_s - t0, s.t1_s - t0)
        assert want == ("heldout" if s.split == "heldout" else "train"), s
        assert s.name in {"A", "B", "C", "D"} and s.direction in {"ccw", "cw"}
    by_name = {n: [s for s in segs if s.name == n] for n in "ABCD"}
    assert {s.split for s in by_name["B"]} == {"heldout"}  # never carved
    a_train = [s for s in by_name["A"] if s.split == "train"]
    a_val = [s for s in by_name["A"] if s.split == "val"]
    assert a_train and a_val
    # validation is the last 15 percent in time of the train segment
    assert min(s.t0_s for s in a_val) - t0 == pytest.approx(5.0 + 0.85 * 100.0, abs=0.05)
    assert max(s.t1_s for s in a_train) < min(s.t0_s for s in a_val)
    assert not any(s.t1_s - t0 > 300.0 for s in segs)  # nothing from the unused segment E
    for lo, hi in ((105.0, 107.0), (167.0, 170.0), (250.0, 252.0)):  # the guards
        assert not any(s.t0_s - t0 < hi and s.t1_s - t0 > lo for s in segs)


def test_no_heldout_sample_is_in_train_or_val_by_construction_and_by_check(toy_vehicle):
    run, _, split, segs = _split_segments(toy_vehicle)
    held = [(s.i0, s.i1) for s in segs if s.split == "heldout"]
    rest = [(s.i0, s.i1) for s in segs if s.split != "heldout"]
    assert not any(a0 < b1 and b0 < a1 for a0, a1 in held for b0, b1 in rest)
    run.scans.append(ScanRecord(float(run.t_s[0]), 0.0, 0.01, 0.1, 10.0, (1.0,), float(run.t_s[0])))
    assert pipeline_v02.check_segments_against_split(segs, split) == len(segs)
    bad = [dataset.Segment(run, None, segs[0].i0, segs[0].i1 + 400, "heldout")]
    with pytest.raises(ValueError, match="split file says"):
        pipeline_v02.check_segments_against_split(bad, split)
    other = [dataset.Segment(_plain_run("other", 0.0, 10.0), None, 0, 100, "train")]
    with pytest.raises(ValueError, match="not in the split"):
        pipeline_v02.check_segments_against_split(other, split)


def test_fit_windows_never_reach_over_a_segment_edge(toy_vehicle):
    """Every command and wheel sample a speed window or a yaw interval reads, including the
    delayed command before the window's start, lies inside its segment."""
    _, _, _, segs = _split_segments(toy_vehicle)
    delay = 0.2
    for seg in segs:
        w = speed_windows([seg], delay, horizon_s=1.0)
        assert len(w) > 0 or seg.duration_s < 3.0
        starts = np.arange(seg.t0_s + delay, seg.t1_s - 1.0, 0.25)
        assert starts[0] - delay >= seg.t0_s - 1e-9
        assert starts[-1] + 1.0 <= seg.t1_s + 1e-9
        y = yaw_data([seg], delay)
        if len(y):
            assert y.t_s.min() - 0.5 - delay >= seg.t0_s - 1e-9  # warm up history, delayed
            assert y.t_s.max() + 0.1 <= seg.t1_s + 1e-9


def test_a_driving_stop_inside_a_segment_never_extends_it(toy_vehicle):
    run, motion, split, _ = _split_segments(toy_vehicle)
    k0 = 120 * 50
    run.speed_cmd_mps[k0 : k0 + 200] = 0.0  # 4 s stop in held-out B
    run.wheel_speed_mps[k0 : k0 + 200] = 0.0
    segs = dataset.make_segments(run, motion, split, t_ref_s=run.t_s[0])
    held = [s for s in segs if s.name == "B"]
    assert len(held) >= 2 and not any(s.i0 <= k0 + 100 < s.i1 for s in held)
    assert min(s.t0_s for s in held) - run.t_s[0] >= 107.0
    assert max(s.t1_s for s in held) - run.t_s[0] <= 167.0


def test_make_segments_needs_a_reference_time_and_checks_the_first_scan():
    run = _plain_run("bag-x", 1000.0, 200.0)
    split = make_split("bag-x", SEGS[:1])
    with pytest.raises(ValueError, match="no /scan"):
        dataset.make_segments(run, None, split)
    scan = ScanRecord(1000.0, 0.0, 0.01, 0.1, 10.0, (1.0,), log_time_s=1000.0)
    run2 = Run(
        run.name, run.t_s, run.speed_cmd_mps, run.steering_cmd_rad, run.wheel_speed_mps, [scan]
    )
    assert dataset.make_segments(run2, None, split)  # the first scan's log time is the origin
    entry = split.entry("bag-x")
    wrong = Split("t", "0" * 64, {"bag-x": BagEntry("bag-x", "c", 5.0, entry.segments)})
    with pytest.raises(ValueError, match="not the split file"):
        dataset.make_segments(run2, None, wrong)


def test_legacy_bags_keep_the_70_30_rule_with_a_validation_tail_and_unused_bags_are_dropped():
    def whole(role):
        return Split("t", "0" * 64, {"old": BagEntry("old", "current", None, (), role)})

    run = _plain_run("old", 0.0, 200.0)
    segs = dataset.make_segments(run, None, whole("legacy_70_30"))
    assert {s.split for s in segs} == {"train", "val", "legacy_heldout"}
    assert all(s.direction == "legacy" for s in segs)
    train = [s for s in segs if s.split == "train"]
    val = [s for s in segs if s.split == "val"]
    held = [s for s in segs if s.split == "legacy_heldout"]
    assert max(s.t1_s for s in train + val) < min(s.t0_s for s in held)
    assert dataset.seconds(val) == pytest.approx(0.15 * 140.0, abs=1.0)  # 15 percent of the 70
    assert dataset.make_segments(run, None, whole("unused")) == []
    assert dataset.make_segments(run, None, Split("t", "0" * 64, {})) == []
    assert dataset.make_segments(run, None, whole("heldout_layout")) == []
    assert {s.split for s in dataset.make_segments(run, None, whole("train"))} == {"train"}


def test_data_table_counts_seconds_per_role_bag_direction_and_bin(toy_vehicle):
    _, _, _, segs = _split_segments(toy_vehicle)
    table = pipeline_v02.data_table(segs)
    bag = table["seconds_by_bag"]["bag-x"]
    assert set(bag) == {"train", "val", "heldout"}
    assert bag["train"] + bag["val"] == pytest.approx(
        sum(s.duration_s for s in segs if s.split != "heldout"), abs=0.2
    )
    cells = table["seconds_by_role_direction_speed_bin"]
    assert set(cells["heldout"]) == {"ccw", "cw"}
    assert sum(cells["heldout"]["ccw"].values()) == pytest.approx(
        sum(s.duration_s for s in segs if s.split == "heldout" and s.direction == "ccw"), abs=1.0
    )
