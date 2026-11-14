"""D3-C0 (pre-registration PnP v0.3 section 1.5): the lap memory's lap events against the camera
lap counter's, per run.

    "with the lap memory in observe mode ... over at least two runs of 10 laps, one per
    direction: the lap count equals the camera lap counter's in both runs and at least 18 of the
    20 lap times agree with it within 0.3 s. If the owner does not accept the camera counter as
    the reference, the phone video of the same runs is the reference and that is said."

Laps of the two sources are paired by their order inside the run (racer_eval.laps.lap_table does
the pairing and the 0.3 s flag). The bag cannot say which direction a run was driven, so the
"one per direction" part is the procedure's, not this report's; the report needs at least two runs.

Verdicts: PASS, FAIL, NO REFERENCE (no camera or video lap in any run: nothing to compare with, so
never a PASS) and INSUFFICIENT (fewer than two runs, or fewer than 20 compared laps: the rule
cannot be applied, nothing is decided).
"""

from __future__ import annotations

import csv
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from racer_eval.laps import in_run_mask, lap_table

from pnp_session.bagread import SessionBag
from pnp_session.constants import (
    D3C0_MIN_RUNS,
    D3C0_MIN_WITHIN,
    D3C0_OF_LAPS,
    D3C0_TOLERANCE_S,
)
from pnp_session.timing import find_pnp_runs


@dataclass(frozen=True)
class RunLapInput:
    run_id: str
    #: Lap times of the lap memory (/pnp/lap/event), in order.
    pnp_times: tuple[float, ...]
    #: Lap times of the reference (camera counter or video), in order; empty when there is none.
    reference_times: tuple[float, ...]


def read_video_times(path: Path) -> dict[str, tuple[float, ...]]:
    """Hand-timed laps: a CSV with the header `run,lap,lap_time_s`; laps in `lap` order."""
    rows: dict[str, list[tuple[int, float]]] = {}
    with Path(path).open(newline="") as f:
        reader = csv.DictReader(f)
        needed = {"run", "lap", "lap_time_s"}
        if reader.fieldnames is None or not needed <= set(reader.fieldnames):
            raise ValueError(f"{path}: the CSV needs the columns run, lap, lap_time_s")
        for row in reader:
            rows.setdefault(row["run"].strip(), []).append(
                (int(row["lap"]), float(row["lap_time_s"]))
            )
    return {run: tuple(t for _, t in sorted(laps)) for run, laps in rows.items()}


def inputs_from_bag(
    bag: SessionBag, video: dict[str, tuple[float, ...]] | None = None
) -> list[RunLapInput]:
    out = []
    for run in find_pnp_runs(bag):
        cam = in_run_mask(bag.cam_t, run.t_start, run.t_end)
        pnp = in_run_mask(bag.pnp_t, run.t_start, run.t_end)
        if video is None:
            reference = tuple(float(x) for x in bag.cam_lap_time_s[cam])
        else:
            reference = video.get(run.run_id, ())
        out.append(RunLapInput(run.run_id, tuple(float(x) for x in bag.pnp_lap_time_s[pnp]), reference))
    return out


def _run_row(inp: RunLapInput) -> dict[str, Any]:
    ref = np.asarray(inp.reference_times, dtype=np.float64)
    pnp = np.asarray(inp.pnp_times, dtype=np.float64)
    # lap_table pairs by order and flags |camera - pnp| above LAP_SOURCE_TOLERANCE_S (0.3 s, the
    # same number as D3-C0's); the within-tolerance count below uses D3C0_TOLERANCE_S itself.
    rows = lap_table(
        inp.run_id,
        np.arange(len(ref), dtype=np.float64),
        ref,
        np.zeros(len(ref)),
        np.arange(len(pnp), dtype=np.float64),
        pnp,
        [],
        [],
    )
    laps = []
    within = 0
    compared = 0
    for r in rows:
        if r.diff_s is None:
            laps.append({"lap": r.lap, "reference_s": r.camera_time_s, "pnp_s": r.pnp_time_s, "diff_s": None, "within": None})
            continue
        ok = abs(r.diff_s) <= D3C0_TOLERANCE_S + 1e-9  # float noise of a difference of two times
        compared += 1
        within += int(ok)
        laps.append(
            {
                "lap": r.lap,
                "reference_s": r.camera_time_s,
                "pnp_s": r.pnp_time_s,
                "diff_s": r.diff_s,
                "within": ok,
            }
        )
    return {
        "run_id": inp.run_id,
        "n_reference": len(ref),
        "n_pnp": len(pnp),
        "count_equal": len(ref) == len(pnp),
        "laps_compared": compared,
        "laps_within": within,
        "laps": laps,
    }


def d3c0(
    runs: Sequence[RunLapInput], reference_name: str = "camera"
) -> dict[str, Any]:
    """The D3-C0 verdict over the given runs, with every per-lap difference."""
    rows = [_run_row(r) for r in runs]
    n_ref = sum(r["n_reference"] for r in rows)
    compared = sum(r["laps_compared"] for r in rows)
    within = sum(r["laps_within"] for r in rows)
    counts_equal = all(r["count_equal"] for r in rows) if rows else False
    reasons: list[str] = []
    if n_ref == 0:
        verdict = "NO REFERENCE"
        reasons.append(f"no {reference_name} lap event in any run: nothing to compare the lap memory with")
    elif len(rows) < D3C0_MIN_RUNS:
        verdict = "INSUFFICIENT"
        reasons.append(f"{len(rows)} run(s), the rule needs at least {D3C0_MIN_RUNS}")
    elif compared < D3C0_OF_LAPS and counts_equal:
        verdict = "INSUFFICIENT"
        reasons.append(f"{compared} laps compared, the rule is stated over {D3C0_OF_LAPS}")
    else:
        # The pre-registered share is 18 of 20; over a different number of compared laps the
        # same share (0.9) is applied, rounded up, as racer_eval's planner acceptance does for
        # its starts.
        need = -(-compared * D3C0_MIN_WITHIN // D3C0_OF_LAPS)
        if not counts_equal:
            verdict = "FAIL"
            reasons.append("the lap count differs from the reference in at least one run")
        elif within < need:
            verdict = "FAIL"
            reasons.append(f"{within} of {compared} lap times within {D3C0_TOLERANCE_S} s, need {need}")
        else:
            verdict = "PASS"
    need_report = -(-compared * D3C0_MIN_WITHIN // D3C0_OF_LAPS) if compared else None
    return {
        "section": "v0.3 1.5 D3-C0",
        "reference": reference_name,
        "verdict": verdict,
        "reasons": reasons,
        "tolerance_s": D3C0_TOLERANCE_S,
        "rule": f"count equal in every run and at least {D3C0_MIN_WITHIN} of {D3C0_OF_LAPS} lap times within {D3C0_TOLERANCE_S} s",
        "runs": rows,
        "laps_compared": compared,
        "laps_within": within,
        "laps_within_needed": need_report,
        "share_within": (within / compared) if compared else None,
        "count_equal_all_runs": counts_equal,
        "laps_off_by_more_than_tolerance": [
            {"run_id": r["run_id"], "lap": lap["lap"], "diff_s": lap["diff_s"]}
            for r in rows
            for lap in r["laps"]
            if lap["within"] is False
        ],
    }
