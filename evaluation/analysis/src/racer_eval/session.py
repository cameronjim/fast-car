"""One car session from bags to the tables and decisions of the pre-registration.

The session file (YAML) names the runs, which bag and time each is in, its condition, cap, wall
contacts (people or video decide, section 4.3) and DNF cause:

    n_laps: 10
    gp4: {on: d_on, off: d_off_cap}            # optional: GP4 over these two conditions
    acceptance: {conditions: [pnp], wall_contacts: 0}   # optional: planner acceptance (4.2)
    runs:
      - {id: on1, bag: 2026-11-14T10-00-00_car_teleop, at_s: 120.0, condition: d_on,
         cap_mps: 0.5, contacts: 0, dnf_cause: null}

`at_s` is any bag second inside the run. The runs of a bag are found by runs.find_runs (the status
stretch of a PnP run, else the non-zero /drive_raw stretch). The thresholds of the decisions are
the pre-registered constants; nothing here passes one.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from racer_eval.curves import N_LAPS, LapCurve, RunLaps, lap_curve
from racer_eval.gates import GateResult, gp4_decision, planner_car_acceptance
from racer_eval.interventions import RunInterventions, count_interventions
from racer_eval.laps import LapRow, in_run_mask, lap_table
from racer_eval.recording import BagData
from racer_eval.runs import Run, find_runs


class SessionError(ValueError):
    """The session file is not valid or does not match the bags."""


@dataclass(frozen=True)
class SessionRun:
    run_id: str
    bag: str
    at_s: float
    condition: str
    cap_mps: float
    contacts: int
    dnf_cause: str | None


def load_session(path: Path) -> dict[str, Any]:
    doc = yaml.safe_load(Path(path).read_text())
    if not isinstance(doc, dict) or not isinstance(doc.get("runs"), list) or not doc["runs"]:
        raise SessionError(f"{path}: needs a non-empty `runs` list")
    return doc


def session_runs(doc: dict[str, Any]) -> list[SessionRun]:
    out = []
    seen: set[str] = set()
    for raw in doc["runs"]:
        for key in ("id", "bag", "at_s", "condition", "cap_mps"):
            if key not in raw:
                raise SessionError(f"run {raw!r}: missing {key!r}")
        run_id = str(raw["id"])
        if run_id in seen:
            raise SessionError(f"run id {run_id!r} used twice")
        seen.add(run_id)
        out.append(
            SessionRun(
                run_id,
                str(raw["bag"]),
                float(raw["at_s"]),
                str(raw["condition"]),
                float(raw["cap_mps"]),
                int(raw.get("contacts", 0)),
                raw.get("dnf_cause"),
            )
        )
    return out


def locate_run(bag: BagData, at_s: float) -> Run:
    """The run of `bag` that contains bag second `at_s` (PnP status runs first)."""
    candidates = find_runs(bag.draw_t, bag.draw_speed, bag.st_t if len(bag.st_t) else None)
    candidates += find_runs(bag.draw_t, bag.draw_speed, None, prefix="D")
    for run in candidates:
        if run.t_start <= at_s <= run.t_end:
            return run
    raise SessionError(f"{bag.name}: no run contains {at_s} s")


@dataclass
class SessionResult:
    laps: list[tuple[SessionRun, LapRow]] = field(default_factory=list)
    interventions: list[tuple[SessionRun, RunInterventions]] = field(default_factory=list)
    run_laps: list[RunLaps] = field(default_factory=list)
    curves: dict[str, LapCurve] = field(default_factory=dict)
    decisions: dict[str, GateResult] = field(default_factory=dict)
    lap_sources: dict[str, str] = field(default_factory=dict)


def analyse_session(doc: dict[str, Any], bags: dict[str, BagData]) -> SessionResult:
    n_laps = int(doc.get("n_laps", N_LAPS))
    result = SessionResult()
    for sr in session_runs(doc):
        if sr.bag not in bags:
            raise SessionError(f"run {sr.run_id}: bag {sr.bag} was not given")
        bag = bags[sr.bag]
        run = locate_run(bag, sr.at_s)
        result.interventions.append(
            (sr, count_interventions(run, bag.ev_t, bag.ev_phase, bag.ev_source))
        )
        cam = in_run_mask(bag.cam_t, run.t_start, run.t_end)
        pnp = in_run_mask(bag.pnp_t, run.t_start, run.t_end)
        rows = lap_table(
            sr.run_id,
            bag.cam_t[cam],
            bag.cam_lap_time_s[cam],
            bag.cam_distance_m[cam],
            bag.pnp_t[pnp],
            bag.pnp_lap_time_s[pnp],
            bag.odom_t,
            bag.odom_v,
        )
        result.laps.extend((sr, row) for row in rows)
        # The camera counter is primary; D's own times only where there is no camera event.
        source = "camera" if cam.any() else "pnp"
        times = (bag.cam_lap_time_s[cam] if cam.any() else bag.pnp_lap_time_s[pnp]).tolist()
        result.lap_sources[sr.run_id] = source if times else "none"
        result.run_laps.append(
            RunLaps(
                run_id=sr.run_id,
                condition=sr.condition,
                lap_times=tuple(times),
                contacts=sr.contacts,
                dnf_after=len(times) if sr.dnf_cause else None,
                dnf_cause=sr.dnf_cause,
                n_target=n_laps,
            )
        )
    by_condition: dict[str, list[RunLaps]] = defaultdict(list)
    for r in result.run_laps:
        by_condition[r.condition].append(r)
    result.curves = {c: lap_curve(rs) for c, rs in by_condition.items()}
    gp4 = doc.get("gp4")
    if gp4:
        for key in ("on", "off"):
            if gp4.get(key) not in by_condition:
                raise SessionError(f"gp4.{key}: no runs of condition {gp4.get(key)!r}")
        result.decisions["gp4"] = gp4_decision(by_condition[gp4["on"]], by_condition[gp4["off"]])
    acc = doc.get("acceptance")
    if acc:
        runs = [(sr, iv) for sr, iv in result.interventions if sr.condition in acc["conditions"]]
        if not runs:
            raise SessionError("acceptance: no runs of the named conditions")
        result.decisions["planner_car_acceptance"] = planner_car_acceptance(
            [iv for _, iv in runs],
            [sr.cap_mps for sr, _ in runs],
            int(acc.get("wall_contacts", 0)),
        )
    return result
