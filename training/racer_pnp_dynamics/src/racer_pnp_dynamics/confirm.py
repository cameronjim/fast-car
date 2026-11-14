"""The fresh confirmation of pre-registration v0.3 section 2.4.

At the next car session the first PnP runs are driven with the deployed model. On at least 120 s of
that driving, both directions, scored with the same code under T2, the candidate must have P1 at
most 1.00 times and Y1 at most 1.00 times the deployed model's. Only then is it switched on.

    python -m racer_pnp_dynamics.cli confirm --bag <dir> --candidate <bundle> --deployed <bundle>

`decide_confirmation` is the rule; `run_confirmation` finds the PnP runs of the bag from
`/pnp/planner/status`, builds the T2 truth, scores both bundles on the same windows and prints P1
and Y1 ratios with PASS or FAIL (or CANNOT DECIDE when the data is too thin).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

# evaluation/prereg/pnp-v0.3.yaml dynamics.fresh_confirmation_min_driving_s
MIN_DRIVING_S = 120.0
# 2.4 "the candidate must have P1 at most 1.00 times and Y1 at most 1.00 times the deployed model's"
P1_MAX_RATIO = 1.00
Y1_MAX_RATIO = 1.00
DIRECTIONS = ("ccw", "cw")


@dataclass(frozen=True)
class Confirmation:
    driving_s: float
    direction_s: dict[str, float]
    p1_ratio: float
    y1_ratio: float
    verdict: str  # PASS, FAIL or CANNOT DECIDE
    reasons: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {**self.__dict__}


def direction_of(net_yaw_rad: float) -> str:
    """ccw for a net left turn over a run (positive yaw), cw otherwise."""
    return "ccw" if net_yaw_rad >= 0.0 else "cw"


def decide_confirmation(
    driving_s: float, direction_s: dict[str, float], p1_ratio: float, y1_ratio: float
) -> Confirmation:
    """The rule of 2.4. Too little driving or a missing direction is CANNOT DECIDE (the candidate
    is not confirmed, nothing is substituted); a ratio that is not a number is a FAIL."""
    reasons: list[str] = []
    if driving_s < MIN_DRIVING_S:
        reasons.append(f"{driving_s:.1f} s of driving, at least {MIN_DRIVING_S:g} s needed")
    for d in DIRECTIONS:
        if direction_s.get(d, 0.0) <= 0.0:
            reasons.append(f"no driving in direction {d}")
    if reasons:
        return Confirmation(driving_s, direction_s, p1_ratio, y1_ratio, "CANNOT DECIDE", reasons)
    ok_p1 = bool(np.isfinite(p1_ratio) and p1_ratio <= P1_MAX_RATIO)
    ok_y1 = bool(np.isfinite(y1_ratio) and y1_ratio <= Y1_MAX_RATIO)
    if not ok_p1:
        reasons.append(f"P1 ratio {p1_ratio:.4f} is above {P1_MAX_RATIO:g}")
    if not ok_y1:
        reasons.append(f"Y1 ratio {y1_ratio:.4f} is above {Y1_MAX_RATIO:g}")
    return Confirmation(
        driving_s, direction_s, p1_ratio, y1_ratio, "PASS" if ok_p1 and ok_y1 else "FAIL", reasons
    )


def format_confirmation(c: Confirmation, candidate: str, deployed: str) -> str:
    lines = [
        f"candidate {candidate} against deployed {deployed}, truth T2",
        f"driving {c.driving_s:.1f} s ("
        + ", ".join(f"{d} {s:.1f} s" for d, s in c.direction_s.items())
        + ")",
        f"P1 ratio {c.p1_ratio:.4f} (limit {P1_MAX_RATIO:g})",
        f"Y1 ratio {c.y1_ratio:.4f} (limit {Y1_MAX_RATIO:g})",
        *[f"  - {r}" for r in c.reasons],
        c.verdict,
    ]
    return "\n".join(lines)


def run_confirmation(  # pragma: no cover - real data
    bag: Path,
    candidate_dir: Path,
    deployed_dir: Path,
    cache: Path,
    workers: int = 8,
    units: str = "current",
) -> Confirmation:
    """Score `candidate_dir` and `deployed_dir` (model bundles) on the PnP runs of a session bag."""
    import tempfile

    from pnp_replay import bag as pnp_bag  # tools/ is on sys.path through .bags
    from pnp_replay import segments as pnp_segments

    from . import export
    from .bags import Run, load_run
    from .dataset import Segment
    from .evaluate import eval_set, window_errors
    from .icp_motion import SEED_WHEEL, compute_motion, driving_intervals
    from .pipeline import fit_vehicle_of
    from .pipeline_v02 import macro_triplet
    from .recordings import recorded_for
    from .strata import assign_strata
    from .vehicle import load_vehicle

    vehicle = load_vehicle()
    fvehicle = fit_vehicle_of(vehicle)
    rec = pnp_bag.load_recording(bag, Path(tempfile.mkdtemp(dir=cache)) / "rec.npz")
    runs_found = [
        s
        for s in pnp_segments.find_segments(rec["st_t"], rec["odom_t"], rec["odom_v"])
        if s.kind == "pnp"
    ]
    run: Run = load_run(bag, None if units == "current" else recorded_for(bag.name))
    t_ref = float(run.scans[0].log_time_s)
    motion = compute_motion(
        run.scans, vehicle, driving_intervals(run), workers=workers, seed=SEED_WHEEL, run=run
    )
    segs: list[Segment] = []
    direction_s = {d: 0.0 for d in DIRECTIONS}
    for s in runs_found:
        i0 = int(np.searchsorted(run.t_s, t_ref + s.t_from))
        i1 = int(np.searchsorted(run.t_s, t_ref + s.t_to))
        if i1 - i0 < 10:
            continue
        sel = motion.ok & (motion.t_s >= run.t_s[i0]) & (motion.t_s <= run.t_s[i1 - 1])
        direction = direction_of(float(np.sum(motion.dyaw_rad[sel])))
        direction_s[direction] += s.duration_s
        segs.append(Segment(run, motion, i0, i1, "heldout", s.name, direction))
    driving_s = float(sum(direction_s.values()))
    es = eval_set(segs)
    bundles = {
        "deployed": export.read_bundle(deployed_dir),
        "candidate": export.read_bundle(candidate_dir),
    }
    if len(es) == 0:
        return decide_confirmation(driving_s, direction_s, float("nan"), float("nan"))
    strata = assign_strata(es.direction, es.start_speed)
    macro = {
        k: macro_triplet(
            es, strata, {1.0: window_errors(es, b.nominal, fvehicle, b.residual, (1.0,))[1.0]}
        )[1.0]
        for k, b in bundles.items()
    }
    return decide_confirmation(
        driving_s,
        direction_s,
        macro["candidate"]["P"] / macro["deployed"]["P"],
        macro["candidate"]["Y"] / macro["deployed"]["Y"],
    )
