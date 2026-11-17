"""Reading note for B4 (brief 48 item 7): the evidence table of the lap-1 and lap-3 findings,
re-derived from B4's stored result JSONs (`car-data/pnp/battery/v0.3-b4/<run>.json`). Nothing in B4
is edited, re-scored or relabelled: D3-S0 remains FAIL. This only reads.

    python3 tests/l5_pnp_laps/b4_reading.py <b4 results dir> > docs/notes/pnp-lap-battery-b4-reading-2026-11.md

The registered origin of lap 1 is the stamp of the first /scan after the planner first reported
DRIVING. B4's scorer did not keep it, but the system's own lap 1 started at the effective time of
that same scan (the stamp plus half a sweep): the first LapEvent's crossing stamp minus its lap
time minus half the simulator's sweep (0.05 s) is that stamp. It is used HERE only to re-read B4
(the new runner computes its truth without reading the system's laps).
"""

from __future__ import annotations

import json
import statistics
import sys
from pathlib import Path

HALF_SWEEP_S = 0.05
#: The four runs of lap-1 errors of 2.23 to 2.37 s, and the three of the lap-3 crossing.
LAP1_RUNS = ("d_N3_cw_23", "e_N1_cw_23", "e_N2_cw_23", "e_N3_ccw_22")
LAP3_RUNS = ("c_N1_ccw_21", "c_N1_ccw_22", "c_N2_cw_22")


def registered_origin_s(first_event: dict) -> float:
    """The stamp of the first /scan after DRIVING, read from the system's first LapEvent."""
    return first_event["t_cross_s"] - first_event["lap_time_s"] - HALF_SWEEP_S


def lap1_by_registered_origin(gt_crossing_0_s: float, first_event: dict) -> float:
    """Ground-truth crossing 1 minus the registered origin: lap 1 of the truth by the rule."""
    return gt_crossing_0_s - registered_origin_s(first_event)


def first_truth_after_driving_s(result: dict) -> float:
    """When the B4 scorer's truth recording began, after the receipt of the first DRIVING report:
    the first ground-truth odometry sample is crossing 1 minus the tracker's own lap 1
    (`gt_lap_times_from_first_odom_s`, which in these runs counts from that sample)."""
    first_sample = result["gt_lap_crossings_s"][0] - result["gt_lap_times_from_first_odom_s"][0]
    return first_sample - result["driving_first_s"]


def load(results_dir: Path) -> dict[str, dict]:
    return {p.stem: json.loads(p.read_text()) for p in sorted(results_dir.glob("*_*_*_*.json"))}


def lap1_rows(results: dict[str, dict]) -> list[dict]:
    rows = []
    for run in LAP1_RUNS:
        d = results[run]
        first = d["system_lap_events"][0]
        _cond, case, direction, _seed = run.split("_")
        lengths = [
            r["system_lap_events"][0]["length_m"]
            for name, r in results.items()
            if name.split("_")[1:3] == [case, direction] and r.get("system_lap_events")
        ]
        rows.append(
            {
                "run": run,
                "system_lap1_s": first["lap_time_s"],
                "tool_truth_s": d["gt_lap_times_by_crossing_s"][0],
                "registered_s": lap1_by_registered_origin(d["gt_lap_crossings_s"][0], first),
                "first_truth_after_driving_s": first_truth_after_driving_s(d),
                "lap1_length_m": first["length_m"],
                "lap1_length_median_m": statistics.median(lengths),
            }
        )
    return rows


def crossing_offsets(d: dict) -> list[float]:
    """System crossing stamp minus ground-truth crossing stamp, per crossing."""
    return [
        e["t_cross_s"] - t
        for e, t in zip(d["system_lap_events"], d["gt_lap_crossings_s"], strict=False)
    ]


def lap3_rows(results: dict[str, dict]) -> list[dict]:
    rows = []
    for run in LAP3_RUNS:
        d = results[run]
        off = crossing_offsets(d)
        lengths = [e["length_m"] for e in d["system_lap_events"]]
        rows.append(
            {
                "run": run,
                "offsets_s": off[:6],
                "lap_lengths_m": lengths[:6],
                "lap3_offset_s": off[2],
                "others_median_offset_s": statistics.median(off[:2] + off[3:6]),
            }
        )
    return rows


INTRO = (
    "This note re-derives, from B4's stored result files, the evidence of brief 48. It edits "
    "and re-scores nothing: B4's decisions (D3-S0 FAIL, D3-S1 PASS, D3-S2(d) FAIL, D3-S2(e) "
    "PASS) stand, and B4's cases and seeds are development data from now on. Produced by "
    "`tests/l5_pnp_laps/b4_reading.py`."
)
LAP1_HEADER = (
    "| run | system lap 1 s | tool's truth s | truth by the registered origin s | "
    "first truth odometry after DRIVING s | lap-1 wheel length m (median of case and "
    "direction) |"
)
LAP1_TEXT = (
    "The registered origin is the stamp of the first /scan after the first DRIVING report: the "
    "system's own lap 1 starts there (first LapEvent crossing minus lap time minus half the "
    "0.1 s sweep). In these four runs the scorer's truth recording began 2.3 s after it (the "
    "startup reset finished late), so the tool's lap 1 was 2.3 s short; by the registered "
    "origin truth and system agree to within 0.08 s."
)
LAP3_TEXT = (
    "System crossing stamp minus ground-truth crossing stamp, crossings 1 to 6 (s), and the "
    "system's wheel length of laps 1 to 6 (m):"
)


def format_note(results: dict[str, dict]) -> str:
    lines = [
        "# B4 reading, 2026-11: D3-S0 remains FAIL",
        "",
        INTRO,
        "",
        "## Lap 1 of the four runs 2.3 s off (S0b(d) and S0b(e))",
        "",
        LAP1_HEADER,
        "|---|---|---|---|---|---|",
    ]
    for r in lap1_rows(results):
        lines.append(
            f"| {r['run']} | {r['system_lap1_s']:.2f} | {r['tool_truth_s']:.2f} | "
            f"{r['registered_s']:.2f} | +{r['first_truth_after_driving_s']:.2f} | "
            f"{r['lap1_length_m']:.2f} ({r['lap1_length_median_m']:.2f}) |"
        )
    lines += [
        "",
        LAP1_TEXT,
        "",
        "## Lap 3 of the three condition c runs (S0b(c))",
        "",
        LAP3_TEXT,
        "",
        "| run | offsets s | lap lengths m | crossing 3 | median of the others |",
        "|---|---|---|---|---|",
    ]
    for r in lap3_rows(results):
        lines.append(
            f"| {r['run']} | {' '.join(f'{x:+.3f}' for x in r['offsets_s'])} | "
            f"{' '.join(f'{x:.2f}' for x in r['lap_lengths_m'])} | {r['lap3_offset_s']:+.3f} | "
            f"{r['others_median_offset_s']:+.3f} |"
        )
    return "\n".join(lines) + "\n"


def main(argv: list[str]) -> int:
    sys.stdout.write(format_note(load(Path(argv[1]))))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
