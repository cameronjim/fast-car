"""L1 tests of track_family_runner's lap memory reporting (`lap_results`): ground-truth lap times
with lap 1 counted from the instant the planner first reported DRIVING, the interventions per
ground-truth lap by source, the system's events and the guidance cap per lap. No ROS."""

from __future__ import annotations

import math

import numpy as np
import pytest
import track_family_runner as runner


class Tracker:
    def __init__(self, crossings: list[float], lap_times: list[float]) -> None:
        self._crossings = crossings
        self.lap_times_s = lap_times


def event(t: float, source: str, phase: int = 0, severity: int = 2) -> tuple:
    return (t, source, phase, severity)


def test_lap_1_is_counted_from_the_first_driving_report_and_the_rest_are_unchanged() -> None:
    tracker = Tracker([110.0, 130.0, 150.5], [11.0, 20.0, 20.5])  # the run's clock started at 99.0
    out = runner.lap_results(tracker, [], [], [], {}, driving_first_s=100.0)
    assert out["gt_lap_times_s"] == [10.0, 20.0, 20.5]
    assert out["gt_lap_crossings_s"] == [110.0, 130.0, 150.5]
    assert out["gt_lap1_start_s"] == 100.0
    # without a DRIVING report the tracker's own numbers stand
    plain = runner.lap_results(tracker, [], [], [], {}, driving_first_s=None)
    assert plain["gt_lap_times_s"] == [11.0, 20.0, 20.5]


def test_lap_1_starts_at_the_first_scan_after_driving_when_there_is_one() -> None:
    tracker = Tracker([110.0, 130.0], [11.0, 20.0])
    out = runner.lap_results(tracker, [], [], [], {}, driving_first_s=100.0, lap1_start_s=100.4)
    assert out["gt_lap_times_s"] == [9.6, 20.0]
    assert out["gt_lap1_start_s"] == 100.4 and out["driving_first_s"] == 100.0


def test_interventions_are_counted_per_ground_truth_lap_by_source() -> None:
    tracker = Tracker([110.0, 130.0], [10.0, 20.0])
    events = [
        event(95.0, "ttc"),  # before DRIVING: in no lap
        event(105.0, "ttc"),  # lap 1
        event(108.0, "watchdog", severity=2),  # lap 1
        event(109.0, "ttc", severity=0),  # lap 1, advisory only: engage but not a brake
        event(110.0, "ttc_reverse"),  # exactly at the crossing: the end of lap 1
        event(115.0, "rate_limit"),  # lap 2
        event(120.0, "ttc", phase=1),  # a RELEASE record is never counted
        event(135.0, "ttc"),  # after the last crossing: in no lap
    ]
    out = runner.lap_results(tracker, events, [], [], {}, driving_first_s=100.0)
    assert out["engage_per_lap"] == [
        {"ttc": 2, "watchdog": 1, "ttc_reverse": 1},
        {"rate_limit": 1},
    ]
    assert out["brake_per_lap"] == [{"ttc": 1, "watchdog": 1, "ttc_reverse": 1}, {"rate_limit": 1}]


def test_system_events_guidance_caps_and_status_are_passed_through() -> None:
    tracker = Tracker([110.0], [10.0])
    lap_events = [{"lap_index": 1, "lap_time_s": 10.1}]
    guidance = [
        (100.0, 1, 0.5, 0),
        (105.0, 1, 0.5, 0),
        (111.0, 2, 0.6, 61),
        (115.0, 2, 0.7, 61),
    ]
    out = runner.lap_results(tracker, [], lap_events, guidance, {"phase": "running"}, 100.0)
    assert out["system_laps"] == 1 and out["system_lap_events"] == lap_events
    assert out["guidance_caps_by_lap"]["1"] == {
        "first_cap_mps": 0.5,
        "max_cap_mps": 0.5,
        "max_stations": 0,
        "last_cap_mps": 0.5,
    }
    assert out["guidance_caps_by_lap"]["2"]["max_cap_mps"] == 0.7
    assert out["guidance_caps_by_lap"]["2"]["first_cap_mps"] == 0.6
    assert out["guidance_caps_by_lap"]["2"]["max_stations"] == 61
    assert out["lap_status"] == {"phase": "running"}


def test_a_tracker_with_no_laps_gives_empty_lists() -> None:
    out = runner.lap_results(Tracker([], []), [event(1.0, "ttc")], [], [], {}, None)
    assert out["gt_lap_times_s"] == [] and out["engage_per_lap"] == []
    assert out["system_laps"] == 0 and out["guidance_caps_by_lap"] == {}


def test_count_by_source() -> None:
    assert runner._count_by_source([event(1.0, "a"), event(2.0, "a"), event(3.0, "b")]) == {
        "a": 2,
        "b": 1,
    }
    assert runner._count_by_source([]) == {}


# -- ground truth counted from the system's own start line ---------------------------------------


def circle_odom(laps: float, period_s: float, t0: float = 100.0, hz: float = 50.0):
    """Ground-truth odometry (t, x, y, yaw, v) of a car on a circle of radius 1 m that stands for
    1 s, then goes round once per `period_s`."""
    n = int((1.0 + laps * period_s) * hz)
    out = []
    for i in range(n):
        t = i / hz
        a = 0.0 if t < 1.0 else 2 * math.pi * (t - 1.0) / period_s
        out.append((t0 + t, math.cos(a), math.sin(a), a + math.pi / 2, 0.0))
    return out


def circle_centreline():
    a = np.linspace(0, 2 * math.pi, 400, endpoint=False)
    return np.cos(a), np.sin(a)


def test_the_truth_is_recounted_from_the_pose_of_the_systems_start_scan() -> None:
    xs, ys = circle_centreline()
    odom = circle_odom(3.2, 10.0)
    start_stamp = 103.0  # the start scan: 2 s into the first lap
    first = {"lap_index": 1, "t_cross_s": 111.0 + 0.05, "lap_time_s": 111.0 - start_stamp}
    rescored = runner.rescore_from_start_scan(odom, xs, ys, [first], sweep_s=0.1)
    assert rescored is not None
    times, crossings, start = rescored
    assert start == pytest.approx(start_stamp, abs=1e-9)
    # counted from the pose reached at 103.0: the first crossing is a whole 10 s later, then
    # every 10 s
    assert times[0] == pytest.approx(10.0, abs=0.1)
    assert all(t == pytest.approx(10.0, abs=0.1) for t in times[1:])
    assert crossings[0] == pytest.approx(113.0, abs=0.1)


def test_no_lap_events_means_no_rescoring() -> None:
    xs, ys = circle_centreline()
    assert runner.rescore_from_start_scan(circle_odom(1.5, 10.0), xs, ys, []) is None


def test_lap_results_prefers_the_rescored_truth_and_keeps_the_first_odometry_numbers() -> None:
    tracker = Tracker([110.0, 130.0], [11.0, 20.0])
    out = runner.lap_results(
        tracker,
        [],
        [],
        [],
        {},
        driving_first_s=100.0,
        lap1_start_s=100.4,
        rescored=([9.9, 20.1], [110.3, 130.4], 100.4),
    )
    assert out["gt_lap_times_s"] == [9.9, 20.1]
    assert out["gt_lap_crossings_s"] == [110.3, 130.4]
    assert out["gt_lap_times_from_first_odom_s"] == [11.0, 20.0]
    assert out["gt_rescored_from_start_scan"] is True
