"""L1: the reading of B4's stored numbers (brief 48 item 7). The golden: from the stored numbers of
the four runs of lap-1 errors of 2.3 s, the registered origin gives lap 1 of the truth as 31.94,
32.67, 35.81 and 32.26 s, within 0.08 s of the system's. Nothing here re-scores B4."""

from __future__ import annotations

from pathlib import Path

import b4_reading as reading
import pytest

#: run -> (gt_lap_crossings_s[0], first LapEvent t_cross_s, its lap_time_s, golden lap 1 of the truth)
STORED = {
    "d_N3_cw_23": (1794709651.493, 1794709651.6079745, 32.00471496582031, 31.94),
    "e_N1_cw_23": (1794709886.698, 1794709886.8244374, 32.74699020385742, 32.67),
    "e_N2_cw_23": (1794710207.451, 1794710207.5687027, 35.87742614746094, 35.81),
    "e_N3_ccw_22": (1794710357.017, 1794710357.1067688, 32.300270080566406, 32.26),
}
B4_DIR = Path("/Users/cameronjim/code/car-data/pnp/battery/v0.3-b4")


@pytest.mark.parametrize("run", sorted(STORED))
def test_the_registered_origin_gives_the_golden_lap_1_of_the_truth(run: str) -> None:
    gt0, t_cross, lap_time, golden = STORED[run]
    event = {"t_cross_s": t_cross, "lap_time_s": lap_time}
    truth_lap1 = reading.lap1_by_registered_origin(gt0, event)
    assert truth_lap1 == pytest.approx(golden, abs=0.006)
    assert abs(lap_time - truth_lap1) <= 0.08  # the launch from standstill is counted by both


def test_the_registered_origin_is_the_stamp_of_the_start_scan() -> None:
    event = {"t_cross_s": 1794709651.6079745, "lap_time_s": 32.00471496582031}
    # B4's own record of that stamp for the same run (gt_lap1_start_s)
    assert reading.registered_origin_s(event) == pytest.approx(1794709619.5532596, abs=1e-4)


@pytest.mark.skipif(not B4_DIR.exists(), reason="B4 results are outside git")
def test_the_evidence_table_from_the_stored_files_is_the_brief_s_table() -> None:
    rows = {r["run"]: r for r in reading.lap1_rows(reading.load(B4_DIR))}
    want = {  # tool's truth, first truth odometry after DRIVING, lap-1 wheel length, its median
        "d_N3_cw_23": (29.65, 2.36, 12.43, 11.92),
        "e_N1_cw_23": (30.38, 2.30, 14.73, 13.80),
        "e_N2_cw_23": (33.52, 2.36, 13.37, 12.94),
        "e_N3_ccw_22": (30.07, 2.29, 12.60, 12.11),
    }
    for run, (tool, after, length, median) in want.items():
        r = rows[run]
        assert r["tool_truth_s"] == pytest.approx(tool, abs=0.006)
        assert r["first_truth_after_driving_s"] == pytest.approx(after, abs=0.006)
        assert r["lap1_length_m"] == pytest.approx(length, abs=0.006)
        assert r["lap1_length_median_m"] == pytest.approx(median, abs=0.006)
        assert r["registered_s"] == pytest.approx(STORED[run][3], abs=0.006)


@pytest.mark.skipif(not B4_DIR.exists(), reason="B4 results are outside git")
def test_the_lap_3_crossings_of_the_three_runs_are_early_by_0_3_to_0_46_s() -> None:
    rows = {r["run"]: r for r in reading.lap3_rows(reading.load(B4_DIR))}
    assert rows["c_N1_ccw_21"]["lap3_offset_s"] == pytest.approx(-0.418, abs=0.001)
    assert rows["c_N1_ccw_22"]["lap3_offset_s"] == pytest.approx(-0.325, abs=0.001)
    assert rows["c_N2_cw_22"]["lap3_offset_s"] == pytest.approx(-0.458, abs=0.001)
    for r in rows.values():
        assert 0.05 < r["others_median_offset_s"] < 0.13


def test_the_note_says_in_its_first_line_that_d3_s0_remains_fail() -> None:
    results = {
        run: {
            "system_lap_events": [
                {"t_cross_s": 100.0 + k, "lap_time_s": 10.0, "length_m": 5.0 + k} for k in range(6)
            ],
            "gt_lap_crossings_s": [100.0 + k for k in range(6)],
            "gt_lap_times_s": [10.0] * 6,
            "gt_lap_times_from_first_odom_s": [10.0] * 6,
            "gt_lap_times_by_crossing_s": [10.0] * 6,
            "driving_first_s": 90.0,
        }
        for run in (*reading.LAP1_RUNS, *reading.LAP3_RUNS)
    }
    note = reading.format_note(results)
    assert note.splitlines()[0] == "# B4 reading, 2026-11: D3-S0 remains FAIL"
    assert "edits and re-scores nothing" in note
