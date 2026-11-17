"""L1, F tests of gate v3 (brief 50): written first, red on gate v2.1 (main at 71ae56d).

F1: a ring (island 0.40 m, lane 1.1 m, 0.5 m/s, exact odometry): v2.1 closes lap 1 at 92 percent of
a lap. F late odometry: the line passage of lap 3 is lost to the 0.2 s odometry guard; v2.1 loses
the lap."""

from __future__ import annotations

from v3_support import RING, RING_SPEED, STADIUM, lap_s, run, run_late


def test_f1_on_a_ring_lap_1_does_not_close_before_95_percent_of_a_lap() -> None:
    """Red on gate v2.1: lap 1 closes at 92 percent of a lap (every scan on a ring is a STRONG
    candidate as soon as the heading slack lets candidates in)."""
    laps = run(RING, 1.3, RING_SPEED)[1]
    assert laps, "no lap closed in 1.3 laps of driving"
    assert laps[0].lap_time_s >= 0.95 * lap_s(RING, RING_SPEED)


def test_f_late_odometry_over_the_line_passage_of_lap_3_is_closed_by_the_anchor() -> None:
    """Every scan within 0.6 m of the line on lap 3 has odometry 0.35 s stale (the guard skips
    them). v2.1 loses the lap (the next closure is flagged missed); v3 places it from the anchor."""
    _, clean = run_late(None)
    truth_lap = STADIUM.length_m
    window = (3.0 * truth_lap - 0.9, 3.0 * truth_lap + 0.6)
    mem, laps = run_late(window)
    assert [r.lap_index for r in laps[:4]] == [1, 2, 3, 4], [r.lap_index for r in laps]
    lap3, ref3 = laps[2], clean[2]
    assert lap3.line_source == "anchor"
    assert not lap3.missed_gate
    assert abs(lap3.t_cross_s - ref3.t_cross_s) < 0.05  # speed 1 m/s: 0.05 m
    assert not any(r.missed_gate for r in laps)
    assert mem.status()["stale_odometry_scans"] > 0
    assert mem.status()["lap_line_sources"][:3] == ["start_scan", "start_scan", "anchor"]
    assert laps[3].line_source == "start_scan"
