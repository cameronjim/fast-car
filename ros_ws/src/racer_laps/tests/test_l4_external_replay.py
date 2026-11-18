"""L4: the lap source rule on two slices of the car runs of 2026-11-16 (brief 51), development bags.

* `car_2026_11_16_22_38_47_laps_1_6.npz`: GP3-R1, laps 1 to 6 of the 20 of 20 run (camera peaks 0.92
  to 0.95, the reference taken standing). Six laps, all confirmed, the cap rises one step per lap.
* `car_2026_11_16_02_18_54_events_1_7.npz`: the run that counted 10 of 20 (the reference taken at
  0.8 m/s). The ready rule fails it; with ready assumed (the replay's development switch) the laps
  of 2 and 4 lap lengths are doubtful by length and heading and the single laps are confirmed.

Cut by `tools/pnp_session/cut_lap_src_fixture.py`. The pinned lap times and lengths are the first
record of these slices under the source rule and are never refreshed to get a test green.

Fault injection on the first slice, one test each: nothing crashes and each gives the verdict the
rule states."""

from __future__ import annotations

import copy
import math
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("yaml")
pytest.importorskip("jsonschema")

from racer_laps.external_replay import (
    ReplayResult,
    ReplayStreams,
    external_config,
    replay_external,
)
from racer_laps.gate import GateConfig
from racer_laps.vehicle import load_live_vehicle

FIXTURES = Path(__file__).parent / "fixtures_camera"
GOOD = "car_2026_11_16_22_38_47_laps_1_6.npz"
MARGINAL = "car_2026_11_16_02_18_54_events_1_7.npz"

GOLDEN_LAP_TIMES_S = [6.3391, 5.5978, 5.6994, 5.7592, 5.6674, 5.6445]
GOLDEN_LENGTHS_M = [5.499, 5.2618, 5.2777, 5.2899, 5.2327, 5.3017]
CAPS_AFTER = [0.6, 0.7, 0.8, 0.9, 1.0, 1.0]


def odom_model() -> GateConfig:
    v = load_live_vehicle()
    return GateConfig(wheelbase_m=v.wheelbase_m, yaw_gain_table=v.yaw_gain_table)


def replay(
    streams: ReplayStreams, assume_ready: bool = False, renumber: bool = False
) -> ReplayResult:
    return replay_external(
        streams,
        external_config(odom_model(), "camera", "v0", 0.5, 1.0),
        assume_ready=assume_ready,
        renumber_events=renumber,
    )


def load(name: str) -> ReplayStreams:
    return ReplayStreams.load(FIXTURES / name)


# -- the clean run ---------------------------------------------------------------------------------


def test_the_good_run_gives_six_confirmed_laps_with_golden_times_and_lengths() -> None:
    s = load(GOOD)
    r = replay(s)
    assert r.source_ready and r.source_not_ready_reason == ""
    assert len(r.verdicts) == 6 and r.confirmed == 6 and r.doubtful == 0
    assert r.rejected_total() == 0
    for v, t, length in zip(r.verdicts, GOLDEN_LAP_TIMES_S, GOLDEN_LENGTHS_M, strict=True):
        assert v.lap_time_s == pytest.approx(t, abs=0.02)
        assert v.length_m == pytest.approx(length, abs=0.02)
    # the crossings are the camera's, the lengths agree with the camera's within 0.10 m
    for k, v in enumerate(r.verdicts):
        assert v.t_cross_s == s.event_cross[k]
        assert abs(v.length_m - s.event_distance[k]) <= 0.10
    # laps 2 to 6: lap time within 0.02 s of the camera's
    for k in range(1, 6):
        assert abs(r.verdicts[k].lap_time_s - s.event_lap_time[k]) <= 0.02
    assert [v.next_cap_mps for v in r.verdicts] == pytest.approx(CAPS_AFTER)
    assert all(v.clean for v in r.verdicts)


def test_the_good_run_hands_out_no_lap_one_cap_after_the_first_lap() -> None:
    r = replay(load(GOOD))
    after = [c for t, c in r.cap_series if t > r.verdicts[0].t_cross_s + 1.0]
    assert min(after) >= 0.6 - 1e-9


# -- the marginal run -------------------------------------------------------------------------------


def test_the_marginal_run_is_not_ready() -> None:
    r = replay(load(MARGINAL))
    assert r.source_ready is False
    assert "no source status" in r.source_not_ready_reason
    assert len(r.verdicts) == 7 and r.confirmed == 0
    assert all("ready" in v.reasons for v in r.verdicts)
    assert all(v.next_cap_mps == 0.5 for v in r.verdicts)  # the cap never leaves lap 1's
    assert min(c for _t, c in r.cap_series) == 0.5 and max(c for _t, c in r.cap_series) == 0.5


def test_the_marginal_run_with_ready_assumed_has_the_registered_verdicts() -> None:
    r = replay(load(MARGINAL), assume_ready=True)
    by_event = {v.event_lap: v for v in r.verdicts}
    assert not by_event[1].confirmed and by_event[1].reasons == ("heading",)  # lap 1, reported
    for k in (2, 3, 5):
        assert by_event[k].confirmed, k
    for k in (4, 6, 7):
        assert not by_event[k].confirmed and by_event[k].reasons == ("length", "heading"), k
        assert by_event[k].length_m == pytest.approx(10.3, abs=0.2)  # two lap lengths
    # the what-if cap never rises at a doubtful lap
    for v in r.verdicts:
        if not v.confirmed:
            assert v.next_cap_mps <= v.cap_mps
    assert r.assumed_ready is True


# -- fault injection on the good run -----------------------------------------------------------------


def edited(s: ReplayStreams, **changes: np.ndarray) -> ReplayStreams:
    out = copy.deepcopy(s)
    for name, value in changes.items():
        setattr(out, name, value)
    return out


def drop_event(s: ReplayStreams, index: int, renumber: bool = True) -> ReplayStreams:
    keep = np.arange(len(s.event_arrival)) != index
    out = copy.deepcopy(s)
    for name in (
        "event_arrival",
        "event_lap",
        "event_cross",
        "event_lap_time",
        "event_distance",
        "event_peak",
    ):
        setattr(out, name, getattr(s, name)[keep])
    if renumber:  # a camera that does not know it missed one numbers on consecutively
        out.event_lap = np.arange(1, len(out.event_lap) + 1, dtype=np.int64)
    return out


def never_rises_at_a_doubtful_lap(r: ReplayResult) -> None:
    for v in r.verdicts:
        if not v.confirmed:
            assert v.next_cap_mps <= v.cap_mps, v


def test_fault_an_event_dropped_gives_one_doubtful_merged_lap() -> None:
    r = replay(drop_event(load(GOOD), 2))
    assert len(r.verdicts) == 5
    merged = r.verdicts[2]
    assert not merged.confirmed and set(merged.reasons) == {"length", "heading"}
    assert merged.length_m == pytest.approx(10.5, abs=0.3)
    assert [v.confirmed for v in r.verdicts] == [True, True, False, True, True]
    never_rises_at_a_doubtful_lap(r)
    assert r.verdicts[2].next_cap_mps == pytest.approx(0.6)  # one step down from 0.7
    # the next lap is confirmed again but, needing two clean laps, does not raise the cap
    assert r.verdicts[3].next_cap_mps == pytest.approx(0.6)


def test_fault_an_event_dropped_with_the_numbers_skipping_is_also_a_sequence_miss() -> None:
    r = replay(drop_event(load(GOOD), 2, renumber=False))
    assert set(r.verdicts[2].reasons) == {"sequence", "length", "heading"}


def test_fault_an_event_duplicated_is_rejected_and_changes_nothing() -> None:
    s = load(GOOD)
    k = 2
    dup = copy.deepcopy(s)
    for name in (
        "event_arrival",
        "event_lap",
        "event_cross",
        "event_lap_time",
        "event_distance",
        "event_peak",
    ):
        arr = getattr(s, name)
        extra = arr[k] + (0.2 if name == "event_arrival" else 0)
        setattr(dup, name, np.insert(arr, k + 1, extra))
    r = replay(dup)
    base = replay(s)
    assert len(r.verdicts) == 6 and r.rejected_total() == 1
    assert r.rejected["not_monotone"] == 1  # the same crossing time
    assert [v.t_cross_s for v in r.verdicts] == [v.t_cross_s for v in base.verdicts]
    assert [v.confirmed for v in r.verdicts] == [True] * 6
    assert [v.next_cap_mps for v in r.verdicts] == [v.next_cap_mps for v in base.verdicts]


def test_fault_an_event_delayed_3_s_is_rejected_and_the_next_lap_is_doubtful() -> None:
    s = load(GOOD)
    arrival = s.event_arrival.copy()
    arrival[2] += 3.0
    r = replay(edited(s, event_arrival=arrival))
    assert r.rejected["latency"] == 1
    assert len(r.verdicts) == 5
    after = r.verdicts[2]  # the event after the lost one
    assert not after.confirmed and {"sequence", "length", "heading"} <= set(after.reasons)
    never_rises_at_a_doubtful_lap(r)


def test_fault_a_crossing_time_jumped_back_is_rejected() -> None:
    s = load(GOOD)
    cross = s.event_cross.copy()
    arrival = s.event_arrival.copy()
    cross[2] = cross[1] - 0.5  # a stamp from before the previous crossing ...
    arrival[2] = cross[1] + 0.3  # ... arriving 0.8 s after it (inside the latency limit)
    r = replay(edited(s, event_cross=cross, event_arrival=arrival))
    assert r.rejected["not_monotone"] == 1 and len(r.verdicts) == 5
    assert not r.verdicts[2].confirmed and "sequence" in r.verdicts[2].reasons
    never_rises_at_a_doubtful_lap(r)


def test_fault_a_crossing_time_jumped_forward_is_rejected() -> None:
    s = load(GOOD)
    cross = s.event_cross.copy()
    cross[2] = s.event_arrival[2] + 1.0
    r = replay(edited(s, event_cross=cross))
    assert r.rejected["future"] == 1 and len(r.verdicts) == 5
    never_rises_at_a_doubtful_lap(r)


def test_fault_a_nan_crossing_time_is_rejected_as_not_finite() -> None:
    s = load(GOOD)
    cross = s.event_cross.copy()
    cross[2] = math.nan
    r = replay(edited(s, event_cross=cross))
    assert r.rejected["not_finite"] == 1 and len(r.verdicts) == 5
    never_rises_at_a_doubtful_lap(r)


def test_fault_a_frozen_status_makes_the_later_laps_doubtful_and_drops_the_cap() -> None:
    s = load(GOOD)
    t_freeze = float(s.event_cross[1])  # the camera's status stops after lap 2
    keep = s.status_arrival <= t_freeze
    frozen = edited(
        s,
        status_arrival=s.status_arrival[keep],
        status_state=s.status_state[keep],
        status_laps=s.status_laps[keep],
        status_sim=s.status_sim[keep],
    )
    r = replay(frozen)
    assert [v.confirmed for v in r.verdicts[:2]] == [True, True]
    assert all(not v.confirmed and "alive" in v.reasons for v in r.verdicts[3:])
    never_rises_at_a_doubtful_lap(r)
    # a silent source: the lap-1 cap while it is silent
    t_after = t_freeze + 3.5
    assert r.min_cap_between(t_after, float(s.event_cross[3])) == pytest.approx(0.5)


def test_fault_the_odometry_frozen_for_1_s_over_a_crossing_makes_that_lap_doubtful() -> None:
    s = load(GOOD)
    t_cross = float(s.event_cross[3])
    keep = ~((s.odom_arrival > t_cross - 0.5) & (s.odom_arrival < t_cross + 0.5))
    frozen = edited(
        s,
        odom_arrival=s.odom_arrival[keep],
        odom_stamp=s.odom_stamp[keep],
        odom_v=s.odom_v[keep],
    )
    r = replay(frozen)
    assert len(r.verdicts) == 6
    bad = r.verdicts[3]
    assert not bad.confirmed and "odometry" in bad.reasons
    never_rises_at_a_doubtful_lap(r)
    # the odometry came back: the laps after it are judged on their own
    assert r.verdicts[5].confirmed


def test_every_fault_leaves_the_replay_runnable_with_no_events_at_all() -> None:
    s = load(GOOD)
    none = edited(
        s,
        event_arrival=s.event_arrival[:0],
        event_lap=s.event_lap[:0],
        event_cross=s.event_cross[:0],
        event_lap_time=s.event_lap_time[:0],
        event_distance=s.event_distance[:0],
        event_peak=s.event_peak[:0],
    )
    r = replay(none)
    # without a lap there is no reference length, so nothing is overdue; the source stayed alive
    assert r.verdicts == [] and r.status["station_uncertain"] is False


def test_the_slices_are_small_and_window_and_save_work(tmp_path: Path) -> None:
    for name in (GOOD, MARGINAL):
        assert (FIXTURES / name).stat().st_size < 2_000_000
    s = load(GOOD)
    w = s.window(float(s.event_arrival[1]) - 1.0, float(s.event_arrival[3]) + 0.1)
    assert 0 < len(w.event_arrival) <= 3 and len(w.odom_stamp) < len(s.odom_stamp)
    assert s.truncated_events(3).event_arrival.shape[0] == 3
    assert s.truncated_events(99) is s
    assert s.truncated_events(0).event_arrival.shape[0] == 0
    s.save(tmp_path / "x.npz")
    again = ReplayStreams.load(tmp_path / "x.npz")
    assert np.array_equal(again.odom_stamp, s.odom_stamp)
    assert list(again.status_state) == list(s.status_state)
