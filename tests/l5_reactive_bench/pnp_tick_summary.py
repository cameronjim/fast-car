"""Per run diagnostics from the planner's 50 Hz status, for the oracle measurements M-A2x and the
obstacle battery B5 (brief 40 and 40a). Pure functions, no ROS: the runner collects the ticks.

Reported without any threshold. The cost terms of the planner's final sequence and its per term
speed limiter are NOT in PlannerStatus (and the PlannerTiming message of brief 42 is not on main
when this was written), so what is available here is the confidence the planner gave the track,
whether the command was held at the effective speed cap, and the smallest planned clearance.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

STATE_DRIVING = 1
#: A tick is "cap limited" when the commanded speed is within this of the effective speed cap
#: (m/s). A reporting aid, not a pre-registered threshold.
CAP_LIMITED_WITHIN_MPS = 0.05
#: An obstacle is being passed while the car's CG is within this of the box centre (m), plus half
#: the box length.
PASS_RADIUS_M = 0.8


@dataclass(frozen=True)
class Tick:
    """One /pnp/planner/status message."""

    stamp_s: float
    state: int
    cmd_speed_mps: float
    v_cap_mps: float
    track_confidence: float
    plan_min_clearance_m: float
    block_reason: int = 0  # PlannerStatus.block_reason; 0 is BLOCK_NONE


def tick_summary(ticks: list[Tick]) -> dict:
    """Mean track confidence, the share of ticks held at the speed cap, and the smallest planned
    clearance, over the DRIVING ticks."""
    driving = [t for t in ticks if t.state == STATE_DRIVING]
    if not driving:
        return {"driving_ticks": 0}
    conf = [t.track_confidence for t in driving if math.isfinite(t.track_confidence)]
    clear = [t.plan_min_clearance_m for t in driving if math.isfinite(t.plan_min_clearance_m)]
    cap_limited = sum(
        1
        for t in driving
        if t.v_cap_mps > 0.0 and t.cmd_speed_mps >= t.v_cap_mps - CAP_LIMITED_WITHIN_MPS
    )
    n = len(driving)
    blocked: dict[str, int] = {}
    for t in ticks:
        if t.block_reason:
            blocked[str(t.block_reason)] = blocked.get(str(t.block_reason), 0) + 1
    return {
        "blocked_ticks_by_reason": blocked,
        "driving_ticks": n,
        "mean_track_confidence": float(np.mean(conf)) if conf else math.nan,
        "share_cap_limited": cap_limited / n,
        "share_below_cap": 1.0 - cap_limited / n,
        "min_plan_clearance_m": float(min(clear)) if clear else math.nan,
        "cost_terms": "not available: PlannerStatus carries no cost terms",
    }


def obstacle_passes(
    ticks: list[Tick],
    odom: list[tuple],
    obstacles: list[tuple[float, float, float]],
) -> list[dict]:
    """For each obstacle (centre x, centre y, length) the smallest planned clearance over the
    DRIVING ticks while the car's CG was within PASS_RADIUS_M plus half its length of the box
    centre, and how many ticks that was. `odom` rows are (stamp, x, y, ...), time ordered."""
    out = []
    if not odom:
        return [
            {"index": k, "ticks": 0, "min_plan_clearance_m": math.nan}
            for k in range(len(obstacles))
        ]
    stamps = np.array([o[0] for o in odom])
    xy = np.array([[o[1], o[2]] for o in odom])
    driving = [t for t in ticks if t.state == STATE_DRIVING]
    idx = np.clip(np.searchsorted(stamps, [t.stamp_s for t in driving]), 0, len(stamps) - 1)
    for k, (cx, cy, length) in enumerate(obstacles):
        near = np.hypot(xy[idx, 0] - cx, xy[idx, 1] - cy) <= PASS_RADIUS_M + length / 2.0
        clear = [
            t.plan_min_clearance_m
            for t, n in zip(driving, near, strict=True)
            if n and math.isfinite(t.plan_min_clearance_m)
        ]
        out.append(
            {
                "index": k,
                "ticks": int(near.sum()),
                "min_plan_clearance_m": float(min(clear)) if clear else math.nan,
            }
        )
    return out
