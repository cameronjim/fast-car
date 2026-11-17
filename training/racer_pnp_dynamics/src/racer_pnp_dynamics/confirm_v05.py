"""The fresh confirmation of pre-registration v0.5 section 7 (GP2f-C item 2), the v0.5 form of
`confirm`.

At the first car session after the fit, the first PnP runs are driven with the deployed model. On
at least 120 s of them, at least 40 s in each direction (the direction is the measured heading, not
a run name), the candidate (a contract 2.1 bundle) must have P1 at most 1.00 times and Y1 at most
1.00 times the deployed model's, overall AND in each direction, and at least 95 percent of the
steps must fall in supported cells. The numbers are read from the registered yaml
(`gate_confirmation.fresh_driving`).

    python -m racer_pnp_dynamics.cli_v05 confirm --bag <dir> --candidate <bundle> \\
        --deployed <bundle> [--run R0 --run R1]

`--run` names PnP runs of the bag (the names `tools/pnp_data.py runs-v05` prints); without it every
PnP run is used. `decide_confirmation_v05` is the rule; the glue finds the runs, builds T2, scores
both bundles on the same windows and prints the table with PASS, FAIL or CANNOT DECIDE.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from .dataset import Segment
from .ground_motion_v21 import ConstantsV05
from .session_v05 import HEADING_BLOCK_S

DIRECTIONS = ("ccw", "cw")
SCOPES = ("overall", "ccw", "cw")


@dataclass(frozen=True)
class ConfirmationV05:
    driving_s: float
    direction_s: dict[str, float]
    ratios: dict[
        str, dict[str, float]
    ]  # scope (overall, ccw, cw) -> {"P1", "Y1"} candidate / deployed
    step_share: float  # share of the candidate's rollout steps in supported cells
    verdict: str  # PASS, FAIL or CANNOT DECIDE
    reasons: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {**self.__dict__}


def direction_seconds(
    segments: Sequence[Segment], block_s: float = HEADING_BLOCK_S
) -> dict[str, float]:
    """Driving time per direction: the sum of the durations of the good T2 intervals inside the
    segments, each labelled by the net T2 yaw of the `block_s` block (counted from the start of its
    segment) it lies in (`ccw` for a net left turn, else `cw`). A block of the segment is whole or
    cut by the segment edge, so the time is the time in the segment."""
    out = {d: 0.0 for d in DIRECTIONS}
    for seg in segments:
        m = seg.motion
        if m is None:
            continue
        inside = m.ok & (m.t_s >= seg.t0_s - 1e-9) & (m.t_s + m.dt_s <= seg.t1_s + 1e-9)
        if not inside.any():
            continue
        block = np.floor((m.t_s - seg.t0_s) / block_s + 1e-9).astype(int)
        yaw = np.where(inside & np.isfinite(m.dyaw_rad), m.dyaw_rad, 0.0)
        for b in np.unique(block[inside]):
            sel = inside & (block == b)
            label = "ccw" if float(np.sum(yaw[block == b])) > 0.0 else "cw"
            out[label] += float(np.sum(m.dt_s[sel]))
    return out


def decide_confirmation_v05(
    c: ConstantsV05,
    driving_s: float,
    direction_s: Mapping[str, float],
    ratios: Mapping[str, Mapping[str, float]],
    step_share: float,
) -> ConfirmationV05:
    """The rule of GP2f-C item 2. Too little driving, in total or in a direction, is CANNOT DECIDE
    (the candidate is not confirmed, nothing is substituted); a ratio or a share that is not a
    number is a FAIL."""
    g = c.doc["gate_confirmation"]["fresh_driving"]
    total_min, dir_min = float(g["min_total_s"]), float(g["min_per_direction_s"])
    p_max, y_max = float(g["max_p1_ratio"]), float(g["max_y1_ratio"])
    share_min = float(g["min_step_share_in_supported_cells"])
    dirs = {d: float(direction_s.get(d, 0.0)) for d in DIRECTIONS}
    ratios_out = {s: dict(ratios.get(s, {})) for s in SCOPES}
    reasons: list[str] = []
    if driving_s < total_min:
        reasons.append(f"{driving_s:.1f} s of driving, at least {total_min:g} s needed")
    for d, s in dirs.items():
        if s < dir_min:
            reasons.append(f"{s:.1f} s of driving in direction {d}, at least {dir_min:g} s needed")
    if reasons:
        return ConfirmationV05(driving_s, dirs, ratios_out, step_share, "CANNOT DECIDE", reasons)
    for scope in SCOPES:
        for key, limit in (("P1", p_max), ("Y1", y_max)):
            value = ratios_out[scope].get(key, float("nan"))
            if not (np.isfinite(value) and value <= limit):
                reasons.append(f"{scope} {key} ratio {value:.4f} is above {limit:g}")
    if not (np.isfinite(step_share) and step_share >= share_min):
        reasons.append(f"{step_share:.4f} of the steps in supported cells, at least {share_min:g}")
    return ConfirmationV05(
        driving_s, dirs, ratios_out, step_share, "FAIL" if reasons else "PASS", reasons
    )


def format_confirmation_v05(
    c: ConfirmationV05, config: ConstantsV05, candidate: str, deployed: str
) -> str:
    g = config.doc["gate_confirmation"]["fresh_driving"]
    lines = [
        f"candidate {candidate} against deployed {deployed}, truth T2, direction by measured heading",
        f"driving {c.driving_s:.1f} s ("
        + ", ".join(f"{d} {s:.1f} s" for d, s in c.direction_s.items())
        + f"); at least {g['min_total_s']:g} s and {g['min_per_direction_s']:g} s per direction needed",
    ]
    for scope in SCOPES:
        r = c.ratios.get(scope, {})
        lines.append(
            f"{scope}: P1 ratio {r.get('P1', float('nan')):.4f} (limit {g['max_p1_ratio']:g}), "
            f"Y1 ratio {r.get('Y1', float('nan')):.4f} (limit {g['max_y1_ratio']:g})"
        )
    lines.append(
        f"steps in supported cells {c.step_share:.4f} "
        f"(at least {g['min_step_share_in_supported_cells']:g})"
    )
    lines += [f"  - {r}" for r in c.reasons]
    lines.append(c.verdict)
    return "\n".join(lines)


def run_confirmation_v05(  # pragma: no cover - real data
    bag: Path,
    candidate_dir: Path,
    deployed_dir: Path,
    cache: Path,
    runs: Sequence[str] = (),
    workers: int = 8,
) -> tuple[ConfirmationV05, ConstantsV05]:
    """Score the contract 2.1 bundle `candidate_dir` and the deployed bundle `deployed_dir` on the
    PnP runs (or the named ones) of a session bag."""
    import tempfile

    from pnp_replay import bag as pnp_bag  # tools/ is on sys.path through .bags
    from pnp_replay import segments as pnp_segments

    from . import export, export_v21, scoring_v05
    from .bags import load_run
    from .evaluate import eval_set
    from .icp_motion import SEED_WHEEL, compute_motion, driving_intervals
    from .pass_v05 import step_share_in_supported
    from .pipeline import fit_vehicle_of
    from .recordings import recorded_for
    from .session_v05 import with_heading_direction
    from .stages_v05 import constants
    from .strata import assign_strata
    from .vehicle import load_vehicle

    c = constants()
    vehicle = load_vehicle()
    fvehicle = fit_vehicle_of(vehicle)
    rec = pnp_bag.load_recording(bag, Path(tempfile.mkdtemp(dir=cache)) / "rec.npz")
    found = [
        s
        for s in pnp_segments.find_segments(rec["st_t"], rec["odom_t"], rec["odom_v"])
        if s.kind == "pnp" and (not runs or s.name in runs)
    ]
    run = load_run(bag, recorded_for(bag.name))
    t_ref = float(run.scans[0].log_time_s)
    motion = compute_motion(
        run.scans, vehicle, driving_intervals(run), workers=workers, seed=SEED_WHEEL, run=run
    )
    segs: list[Segment] = []
    for s in found:
        i0 = int(np.searchsorted(run.t_s, t_ref + s.t_from))
        i1 = int(np.searchsorted(run.t_s, t_ref + s.t_to))
        if i1 - i0 >= 10:
            segs.append(Segment(run, motion, i0, i1, "heldout", s.name, "by_heading"))
    cand = export_v21.read_bundle_v21(candidate_dir, c)
    dep = export.read_bundle(deployed_dir)
    es = with_heading_direction(eval_set(segs), segs)
    seconds = direction_seconds(segs)
    if len(es) == 0:
        return decide_confirmation_v05(c, sum(seconds.values()), seconds, {}, float("nan")), c
    strata = assign_strata(es.direction, es.start_speed)
    specs = {
        "candidate": scoring_v05.Spec(cand.nominal, ground21=cand.ground),
        "deployed": scoring_v05.Spec(dep.nominal, dep.residual),
    }
    errs = {n: s.errors(es, fvehicle)[1.0] for n, s in specs.items()}
    ratios: dict[str, dict[str, float]] = {}
    macro = {n: scoring_v05.macro_at(e, strata) for n, e in errs.items()}
    ratios["overall"] = {k: macro["candidate"][k] / macro["deployed"][k] for k in ("P1", "Y1")}
    for d in DIRECTIONS:
        dm = {n: scoring_v05.direction_macro(e, strata, d) for n, e in errs.items()}
        ratios[d] = {k: dm["candidate"][k] / dm["deployed"][k] for k in ("P1", "Y1")}
    share = step_share_in_supported(
        c, cand.ground, es, cand.nominal, fvehicle, np.asarray(cand.ground.support, bool)
    )
    return decide_confirmation_v05(c, sum(seconds.values()), seconds, ratios, share), c
