"""L1: the per-lap measures of battery B11 (brief 53), `dv1_measures`, on built series."""

from __future__ import annotations

import math

import dv1_measures as m


def test_clearance_by_lap_takes_the_lowest_per_truth_lap() -> None:
    odom = [(t * 0.5, t, 0.0, 0.0, 1.0) for t in range(12)]  # t 0 .. 5.5 s, x = 2 t
    clear = {x: 0.3 - 0.01 * x for x in range(12)}
    out = m.clearance_by_lap(odom, 0.0, [2.0, 4.0], lambda x, y, yaw: clear[int(x)])
    # lap 1 is (0, 2]: samples t 0.5 .. 2.0 (x 1 .. 4); lap 2 is (2, 4]: x 5 .. 8
    assert out == [round(0.3 - 0.04, 4), round(0.3 - 0.08, 4)]
    assert m.clearance_by_lap(odom, 10.0, [11.0], lambda *a: 0.1) == [None]


def test_max_command_by_lap_and_overall() -> None:
    drive = [(0.5, 0.9, 0.0), (1.5, 0.4, 0.0), (2.5, 0.7, 0.0), (5.0, 1.2, 0.0)]
    by_lap, top = m.max_command_by_lap(drive, 1.0, [2.0, 3.0])
    assert by_lap == [0.4, 0.7]  # the command before the start does not count
    assert top == 1.2  # after the last crossing still counts for the ceiling


def test_longest_stop_after_first_motion() -> None:
    odom = [
        (t, 0.0, 0.0, 0.0, v)
        for t, v in [(0, 0), (1, 0), (2, 0.5), (3, 0.0), (4, 0.01), (6, 0.4), (7, 0.0), (8, 0.0)]
    ]
    # the stand before moving is not a stop; 3 to 6 is 3 s; 7 to 8 (the end) 1 s
    assert m.longest_stop_s(odom, -1.0, None) == 3.0
    assert m.longest_stop_s(odom, -1.0, 7.5) == 3.0
    assert m.longest_stop_s([(0, 0, 0, 0, 0.0)], None, None) == 0.0


def test_bound_log_and_the_comparison_after_a_doubtful_lap() -> None:
    log = m.BoundLog()
    log.add(2, 1.0, [0.0, 0.1, 0.2], [0.8, 1.2, 0.9])  # capped by v_cap
    log.add(2, 0.9, [0.1], [1.0])
    log.add(3, 0.5, [], [])  # cap only
    log.add(4, 1.0, [0.0, 0.1], [0.8, 1.1])
    js = log.to_json()
    assert js["2"]["bins"] == {"0": 0.8, "1": 1.0, "2": 0.9} and js["2"]["cap_only_max"] is None
    assert js["3"] == {"cap_only_max": 0.5, "bins": {}}
    # lap 4 after lap 2: bin 0 equal, bin 1 equal (1.0 vs 1.0) -> no violation
    assert m.bounds_above_after(js["2"], js["4"]) == []
    log.add(4, 1.2, [0.2], [0.95])
    above = m.bounds_above_after(js["2"], log.to_json()["4"])
    assert [(b, round(v, 3)) for b, v, _ in above] == [(2, 0.95)]
    # after a cap-only lap every station is bounded by that cap
    above = m.bounds_above_after(js["3"], js["4"])
    assert {b for b, _, _ in above} == {0, 1}
    # a lap with neither bins nor cap compares as nothing: every bin of the next lap is above
    above = m.bounds_above_after({"cap_only_max": None, "bins": {}}, js["4"])
    assert all(math.isnan(ref) for _, _, ref in above) and above
    # NaN and negative stations are skipped
    log.add(5, 1.0, [math.nan, -0.1, 50.0], [1.0, 1.0, 1.0])
    assert log.to_json()["5"]["bins"] == {}
