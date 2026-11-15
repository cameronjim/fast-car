"""The tracker against PnP table of step 8 (pre-registration v0.4 section 6.5).

    "A statement that the mapless driver is faster than the mapped tracker needs both driven in
    the same session on the same layout, same direction, at least 5 laps each."

There is no gate. The table lists, per run, its laps, mean and best lap time, wheel distance per
lap, direction and the battery voltage at the start. Runs of different sessions or directions are
refused (the table would be a comparison the pre-registration does not allow). Whether the table
may be read as a comparison is `comparison_claim_allowed`, with the reasons it is not.
"""

from __future__ import annotations

import math
import re
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

#: v0.4 6.5: "at least 5 laps each".
MIN_LAPS_EACH = 5
#: v0.2 5.2: the VESC input voltage at the start of every run inside a 0.8 V window.
BATTERY_WINDOW_V = 0.8
DRIVERS = ("tracker", "pnp")


class SameSessionRefused(ValueError):
    """The runs are not from one session and one direction."""


@dataclass(frozen=True)
class DrivenRun:
    run_id: str
    driver: str  # "tracker" or "pnp"
    bag: str
    direction: str  # "ccw" or "cw"
    #: Completed lap times in order, seconds.
    lap_times: tuple[float, ...]
    #: Wheel distance of the whole run (integral of /odom/wheel), metres.
    wheel_distance_m: float
    #: VESC input voltage at the start (5 s mean at rest), declared by the person; None if unknown.
    start_voltage_v: float | None = None


def session_of_bag(bag_name: str) -> str:
    """The session of a bag: its date, the `YYYY-MM-DD` prefix of the recorder's name."""
    m = re.match(r"(\d{4}-\d{2}-\d{2})", bag_name)
    if m is None:
        raise SameSessionRefused(f"bag {bag_name!r} has no date prefix: its session is unknown")
    return m.group(1)


def _row(r: DrivenRun) -> dict[str, Any]:
    laps = np.asarray(r.lap_times, dtype=np.float64)
    n = len(laps)
    return {
        "run_id": r.run_id,
        "driver": r.driver,
        "bag": r.bag,
        "direction": r.direction,
        "laps": n,
        "mean_lap_s": float(laps.mean()) if n else None,
        "best_lap_s": float(laps.min()) if n else None,
        "wheel_distance_per_lap_m": r.wheel_distance_m / n if n else None,
        "start_voltage_v": r.start_voltage_v,
    }


def compare(runs: Sequence[DrivenRun]) -> dict[str, Any]:
    """The step 8 table. Raises SameSessionRefused for runs of different sessions or directions,
    an unknown driver, or runs of only one driver."""
    if not runs:
        raise SameSessionRefused("no runs")
    for r in runs:
        if r.driver not in DRIVERS:
            raise SameSessionRefused(f"run {r.run_id}: driver must be one of {list(DRIVERS)}")
    sessions = {session_of_bag(r.bag) for r in runs}
    if len(sessions) > 1:
        raise SameSessionRefused(f"runs of different sessions: {sorted(sessions)}")
    directions = {r.direction for r in runs}
    if len(directions) > 1:
        raise SameSessionRefused(f"runs of different directions: {sorted(directions)}")
    if {r.driver for r in runs} != set(DRIVERS):
        raise SameSessionRefused("a comparison needs runs of both the tracker and PnP")
    rows = [_row(r) for r in runs]
    per_driver: dict[str, dict[str, Any]] = {}
    reasons: list[str] = []
    for d in DRIVERS:
        laps = np.asarray([t for r in runs if r.driver == d for t in r.lap_times], dtype=np.float64)
        per_driver[d] = {
            "runs": sum(r.driver == d for r in runs),
            "laps": len(laps),
            "mean_lap_s": float(laps.mean()) if len(laps) else None,
            "best_lap_s": float(laps.min()) if len(laps) else None,
        }
        if len(laps) < MIN_LAPS_EACH:
            reasons.append(f"{d}: {len(laps)} laps, v0.4 6.5 needs at least {MIN_LAPS_EACH}")
    volts = [r.start_voltage_v for r in runs]
    known = [v for v in volts if v is not None and math.isfinite(v)]
    spread = (max(known) - min(known)) if known else None
    if spread is None or len(known) < len(runs):
        window = "cannot be checked (a start voltage is missing)"
        reasons.append("battery window cannot be checked: a start voltage is missing")
    elif spread > BATTERY_WINDOW_V:
        window = f"broken: spread {spread:.2f} V is over {BATTERY_WINDOW_V} V"
        reasons.append(window)
    else:
        window = f"held: spread {spread:.2f} V is within {BATTERY_WINDOW_V} V"
    return {
        "section": "v0.4 6.5",
        "session": sessions.pop(),
        "direction": directions.pop(),
        "rows": rows,
        "per_driver": per_driver,
        "battery_window": window,
        "battery_spread_v": spread,
        "comparison_claim_allowed": not reasons,
        "reasons": reasons,
        "note": "a comparison table, no gate",
    }
