"""The cap above 1.3 (pre-registration v0.4 section 6.4).

"A cap above 1.3 m/s is driven only with a bundle whose validity allows it by section 2.4,
and first as two runs per direction at that cap with the lap memory off, compared with the
runs at 1.3 of the same session: no TTC or obstacle brake, no contact, mean wheel speed not
lower."
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, replace

import numpy as np

from racer_eval.gates import GateResult, _item
from racer_eval.same_session import session_of_bag

SECTION = "v0.4 6.4"
#: "compared with the runs at 1.3 of the same session".
REFERENCE_CAP_MPS = 1.3
#: "two runs per direction at that cap".
RUNS_PER_DIRECTION = 2
DIRECTIONS = ("ccw", "cw")


@dataclass(frozen=True)
class CapRun:
    run_id: str
    bag: str
    cap_mps: float
    direction: str
    lap_memory_off: bool
    #: TTC plus obstacle brakes in the run (racer_eval.interventions: `ttc`, `ttc_reverse`, ...).
    brakes: int
    contacts: int
    mean_wheel_speed_mps: float


def _mean(runs: Sequence[CapRun]) -> float | None:
    return float(np.mean([r.mean_wheel_speed_mps for r in runs])) if runs else None


def decide_cap_step(
    new_runs: Sequence[CapRun],
    reference_runs: Sequence[CapRun],
    bundle_validity_mps: float,
) -> GateResult:
    """Every item of section 6.4 on the runs at the new cap against the session's 1.3 runs.
    `bundle_validity_mps` is the active bundle's `v_max_mps` (the validity of 2.4)."""
    caps = {r.cap_mps for r in new_runs}
    new_cap = max(caps) if caps else None
    per_dir = {d: [r for r in new_runs if r.direction == d] for d in DIRECTIONS}
    ref_dir = {
        d: [
            r
            for r in reference_runs
            if r.direction == d and math.isclose(r.cap_mps, REFERENCE_CAP_MPS)
        ]
        for d in DIRECTIONS
    }
    sessions = {session_of_bag(r.bag) for r in [*new_runs, *reference_runs]}
    speed_rows = []
    not_lower = 0
    for d in DIRECTIONS:
        a, b = _mean(per_dir[d]), _mean(ref_dir[d])
        speed_rows.append({"direction": d, "new_mean_mps": a, "reference_mean_mps": b})
        not_lower += int(a is not None and b is not None and a >= b)
    items = (
        _item(
            "6.4-1",
            SECTION,
            "one cap above 1.3 m/s over the new runs (largest cap if several)",
            new_cap,
            ">",
            REFERENCE_CAP_MPS,
            note=f"caps seen {sorted(caps)}" if len(caps) > 1 else "",
        ),
        _item(
            "6.4-2",
            SECTION,
            "the cap is within the active bundle's validity (2.4)",
            new_cap,
            "<=",
            bundle_validity_mps,
        ),
        _item(
            "6.4-3",
            SECTION,
            "runs per direction at the new cap, fewest of the two directions",
            min(len(per_dir[d]) for d in DIRECTIONS),
            ">=",
            RUNS_PER_DIRECTION,
        ),
        _item(
            "6.4-4",
            SECTION,
            "reference runs at 1.3 per direction, fewest of the two directions",
            min(len(ref_dir[d]) for d in DIRECTIONS),
            ">=",
            1,
        ),
        _item(
            "6.4-5",
            SECTION,
            "all runs from one session (reference runs are of the same session)",
            len(sessions),
            "==",
            1,
            note=f"sessions {sorted(sessions)}" if len(sessions) != 1 else "",
        ),
        _item(
            "6.4-6",
            SECTION,
            "runs with the lap memory on (new and reference)",
            sum(not r.lap_memory_off for r in [*new_runs, *reference_runs]),
            "==",
            0,
        ),
        _item(
            "6.4-7",
            SECTION,
            "TTC or obstacle brakes in the runs at the new cap",
            sum(r.brakes for r in new_runs),
            "==",
            0,
        ),
        _item(
            "6.4-8",
            SECTION,
            "wall contacts in the runs at the new cap",
            sum(r.contacts for r in new_runs),
            "==",
            0,
        ),
        _item(
            "6.4-9",
            SECTION,
            "directions where the mean wheel speed at the new cap is not lower than at 1.3",
            not_lower,
            "==",
            len(DIRECTIONS),
        ),
    )
    result = GateResult(
        name="cap_step",
        section=SECTION,
        items=items,
        preregistered=True,
        reported={"mean_wheel_speed": speed_rows, "bundle_validity_mps": bundle_validity_mps},
    )
    return replace(result, outcome="pass" if result.passed else "fail")
