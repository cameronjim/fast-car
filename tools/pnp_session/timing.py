"""The timing report (pre-registration PnP v0.3 section 5, M-T): what the planner, `/drive_raw` and
safety_node did inside the counting window of every PnP run.

Run finding, the window and the 4.2 counting rule are racer_eval's (runs.find_runs and
interventions.count_interventions); this module selects the in-window messages with the same
inclusive edges and adds the statistics. Nothing here decides anything: the 4.2 acceptance is
racer_eval.gates.planner_car_acceptance, printed with every item.
"""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Sequence
from typing import Any

import numpy as np
from numpy.typing import NDArray
from racer_eval import gates, interventions
from racer_eval.runs import Run, find_runs

from pnp_session import detail as detail_mod
from pnp_session import gaps
from pnp_session.bagread import SessionBag
from pnp_session.constants import (
    BUNCHED_BELOW_S,
    CURRENT_WATCHDOG_LIMIT_S,
    GAP_LIMITS_S,
    STATE_FAULT,
    STATE_NAMES,
    STATE_STARTING,
    WATCHDOG_MATCH_TOL_S,
)

UNPARSED_SHOWN = 5


def find_pnp_runs(bag: SessionBag) -> list[Run]:
    """PnP runs of the bag (status stretches); empty when the bag has no planner status."""
    if len(bag.st_t) == 0:
        return []
    return find_runs(bag.draw_t, bag.draw_speed, bag.st_t)


def in_window(t: NDArray[np.float64], window: tuple[float, float]) -> NDArray[np.bool_]:
    """Both edges included, as the counting rule of section 4.2 has it."""
    return (t >= window[0]) & (t <= window[1])


def _summary(values: NDArray[np.float64]) -> dict[str, Any]:
    if len(values) == 0:
        return {"n": 0, "min": None, "p50": None, "p99": None, "max": None}
    p50, p99 = np.percentile(values, [50.0, 99.0], method="linear")
    return {
        "n": len(values),
        "min": float(np.min(values)),
        "p50": float(p50),
        "p99": float(p99),
        "max": float(np.max(values)),
    }


@dataclasses.dataclass
class _RunPieces:
    """Arrays of one run kept for pooling."""

    run: Run
    draw_recv_gaps: NDArray[np.float64]
    draw_stamp_gaps: NDArray[np.float64]
    st_recv_gaps: NDArray[np.float64]
    st_stamp_gaps: NDArray[np.float64]
    solve_s: NDArray[np.float64]
    n_draw: int
    n_status: int
    states: NDArray[np.int64]
    engage_details: dict[str, list[str]]
    watchdog_gap_hits: list[tuple[float, float]]
    iv: interventions.RunInterventions
    iv_start: interventions.RunInterventions
    starting: dict[str, Any]


def _engage_details(bag: SessionBag, window: tuple[float, float]) -> dict[str, list[str]]:
    """Source and detail of the ENGAGE records inside the window, by source. Same predicate as
    the counting rule; run_report checks the total against count_interventions."""
    sel = (bag.ev_phase == interventions.PHASE_ENGAGE) & in_window(bag.ev_t, window)
    out: dict[str, list[str]] = {}
    for src, det in zip(bag.ev_source[sel], bag.ev_detail[sel], strict=True):
        out.setdefault(str(src), []).append(str(det))
    return out


def _detail_block(engage: dict[str, list[str]]) -> dict[str, Any]:
    rate = engage.get("rate_limit", [])
    dts = [(s, detail_mod.parse_rate_limit_dt(s)) for s in rate]
    parsed_dt = np.array([v for _, v in dts if v is not None], dtype=np.float64)
    wd = engage.get("watchdog", [])
    wds = [(s, detail_mod.parse_watchdog(s)) for s in wd]
    ages = np.array([w.age_s for _, w in wds if w is not None], dtype=np.float64)
    timeouts = sorted({w.timeout_s for _, w in wds if w is not None})
    return {
        "rate_limit": {
            "dt_s": _summary(parsed_dt),
            "dt_below_half_tick": int(gaps.count_below(parsed_dt, BUNCHED_BELOW_S)),
            "n_unparsed": sum(1 for _, v in dts if v is None),
            "unparsed_shown": [s for s, v in dts if v is None][:UNPARSED_SHOWN],
        },
        "watchdog": {
            "age_s": _summary(ages),
            "timeouts_s": timeouts,
            "n_unparsed": sum(1 for _, w in wds if w is None),
            "unparsed_shown": [s for s, w in wds if w is None][:UNPARSED_SHOWN],
        },
    }


def _starting(bag: SessionBag, run: Run) -> dict[str, Any]:
    """STARTING at the beginning of one run: leading ticks in state STARTING and the time from the
    first status to the first status that is not STARTING (header stamps)."""
    sel = in_window(bag.st_t, (run.t_start, run.t_end))
    states = bag.st_state[sel]
    stamps = bag.st_stamp[sel]
    n = 0
    while n < len(states) and states[n] == STATE_STARTING:
        n += 1
    duration = float(stamps[n] - stamps[0]) if 0 < n < len(states) else None
    return {"ticks": n, "duration_s": duration}


def _watchdog_gap_hits(recv: NDArray[np.float64], limit_s: float) -> list[tuple[float, float]]:
    """Receive gaps above the limit as (start, end) pairs."""
    g = gaps.gaps_of(recv)
    idx = np.nonzero(g - limit_s > gaps.ABOVE_EPS_S)[0]
    return [(float(recv[i]), float(recv[i + 1])) for i in idx]


def _run_pieces(bag: SessionBag, run: Run) -> _RunPieces:
    window = run.counting_window
    assert window is not None
    d_sel = in_window(bag.draw_t, window)
    s_sel = in_window(bag.st_t, window)
    iv = interventions.count_interventions(run, bag.ev_t, bag.ev_phase, bag.ev_source)
    # The first 2.0 s of the start: the same rule on a run that ends 2.0 s after its first command.
    start_run = dataclasses.replace(
        run, t_end=min(run.t_end, window[0] + interventions.START_WINDOW_S)
    )
    iv_start = interventions.count_interventions(start_run, bag.ev_t, bag.ev_phase, bag.ev_source)
    engage = _engage_details(bag, window)
    if sum(len(v) for v in engage.values()) != sum(iv.counts.values()):
        raise ValueError(
            f"{run.run_id}: the ENGAGE records selected for their detail strings "
            f"({sum(len(v) for v in engage.values())}) differ from the 4.2 count "
            f"({sum(iv.counts.values())}); the counting rule has changed under this report"
        )
    return _RunPieces(
        run=run,
        draw_recv_gaps=gaps.gaps_of(bag.draw_t[d_sel]),
        draw_stamp_gaps=gaps.gaps_of(bag.draw_stamp[d_sel]),
        st_recv_gaps=gaps.gaps_of(bag.st_t[s_sel]),
        st_stamp_gaps=gaps.gaps_of(bag.st_stamp[s_sel]),
        solve_s=bag.st_solve_s[s_sel],
        n_draw=int(np.count_nonzero(d_sel)),
        n_status=int(np.count_nonzero(s_sel)),
        states=bag.st_state[s_sel],
        engage_details=engage,
        watchdog_gap_hits=_watchdog_gap_hits(bag.draw_t[d_sel], CURRENT_WATCHDOG_LIMIT_S),
        iv=iv,
        iv_start=iv_start,
        starting=_starting(bag, run),
    )


def _state_counts(states: NDArray[np.int64]) -> dict[str, int]:
    return {
        STATE_NAMES.get(int(s), f"STATE_{int(s)}"): int(np.count_nonzero(states == s))
        for s in np.unique(states)
    }


def _planner_block(
    solve: NDArray[np.float64],
    st_stamp_gaps: NDArray[np.float64],
    st_recv_gaps: NDArray[np.float64],
    n_status: int,
    states: NDArray[np.int64],
) -> dict[str, Any]:
    nonzero = solve[solve > 0.0]
    return {
        "ticks": n_status,
        "solve_time_s": _summary(solve),
        "solve_time_nonzero_s": _summary(nonzero),
        "ticks_with_zero_solve_time": int(len(solve) - len(nonzero)),
        "tick_gap_stamp": gaps.gap_stats(st_stamp_gaps, n_status).to_dict(),
        "tick_gap_recv": gaps.gap_stats(st_recv_gaps, n_status).to_dict(),
        "bunched_ticks": gaps.count_below(st_stamp_gaps, BUNCHED_BELOW_S),
        "bunched_ticks_recv": gaps.count_below(st_recv_gaps, BUNCHED_BELOW_S),
        "plan_age_s": None,
        "plan_age_note": "PlannerStatus carries no plan age field; not measurable from the bag",
        "fault_ticks": int(np.count_nonzero(states == STATE_FAULT)),
        "state_ticks": _state_counts(states),
    }


def _safety_block(
    iv: interventions.RunInterventions,
    iv_start: interventions.RunInterventions,
    engage: dict[str, list[str]],
) -> dict[str, Any]:
    per_min = {k: (None if math.isnan(v) else v) for k, v in iv.per_minute.items()}
    return {
        "engage_counts": dict(sorted(iv.counts.items())),
        "engage_per_minute": dict(sorted(per_min.items())),
        "first_2s_counts": dict(sorted(iv_start.counts.items())),
        "start_event": iv.start_event,
        "details": _detail_block(engage),
    }


def _run_entry(p: _RunPieces) -> dict[str, Any]:
    window = p.run.counting_window
    assert window is not None
    return {
        "run_id": p.run.run_id,
        "commanded": True,
        "window_s": [window[0], window[1]],
        "duration_s": p.run.duration_s,
        "starting": p.starting,
        "drive_raw": {
            "recv": gaps.gap_stats(p.draw_recv_gaps, p.n_draw).to_dict(),
            "stamp": gaps.gap_stats(p.draw_stamp_gaps, p.n_draw).to_dict(),
        },
        "planner": _planner_block(p.solve_s, p.st_stamp_gaps, p.st_recv_gaps, p.n_status, p.states),
        "safety": _safety_block(p.iv, p.iv_start, p.engage_details),
    }


def _pooled_entry(pieces: Sequence[_RunPieces]) -> dict[str, Any]:
    n_draw = sum(p.n_draw for p in pieces)
    n_status = sum(p.n_status for p in pieces)
    states = np.concatenate([p.states for p in pieces]) if pieces else np.zeros(0, np.int64)
    solve = np.concatenate([p.solve_s for p in pieces]) if pieces else np.zeros(0)
    engage: dict[str, list[str]] = {}
    for p in pieces:
        for k, v in p.engage_details.items():
            engage.setdefault(k, []).extend(v)
    tot = interventions.total([p.iv for p in pieces])
    tot_start = interventions.total([p.iv_start for p in pieces])
    with_event, n_starts = interventions.starts_with_event([p.iv for p in pieces])
    safety = _safety_block(tot, tot_start, engage)
    safety["start_event"] = None
    safety["starts_with_event"] = with_event
    safety["starts"] = n_starts
    return {
        "run_id": "pooled",
        "n_runs": len(pieces),
        "duration_s": tot.duration_s,
        "drive_raw": {
            "recv": gaps.pooled_stats([p.draw_recv_gaps for p in pieces], n_draw).to_dict(),
            "stamp": gaps.pooled_stats([p.draw_stamp_gaps for p in pieces], n_draw).to_dict(),
        },
        "planner": _planner_block(
            solve,
            np.concatenate([p.st_stamp_gaps for p in pieces]) if pieces else np.zeros(0),
            np.concatenate([p.st_recv_gaps for p in pieces]) if pieces else np.zeros(0),
            n_status,
            states,
        ),
        "safety": safety,
    }


def watchdog_whatif(
    pieces: Sequence[_RunPieces],
    watchdog_events: Sequence[tuple[float, float]],
    limits_s: Sequence[float] = GAP_LIMITS_S,
) -> dict[str, Any]:
    """For each limit, the number of `/drive_raw` receive gaps above it (in-window). Next to it,
    how well the row of the current limit agrees with the recorded watchdog engagements.

    `watchdog_events` are (cycle stamp, receive time) of the recorded in-window watchdog ENGAGE
    records; the cycle stamp is the one compared with the gaps."""
    table = {
        gaps.limit_key(lim): sum(gaps.count_above(p.draw_recv_gaps, lim) for p in pieces)
        for lim in limits_s
    }
    hits = [h for p in pieces for h in p.watchdog_gap_hits]
    events = [t for t, _ in watchdog_events]
    matched_events = 0
    used: set[int] = set()
    for t in events:
        for i, (a, b) in enumerate(hits):
            lo = a + CURRENT_WATCHDOG_LIMIT_S - WATCHDOG_MATCH_TOL_S
            if i not in used and lo <= t <= b + WATCHDOG_MATCH_TOL_S:
                matched_events += 1
                used.add(i)
                break
    return {
        "gaps_above_limit": table,
        "current_limit_s": CURRENT_WATCHDOG_LIMIT_S,
        "recorded_watchdog_engagements": len(events),
        "gaps_above_current_limit": len(hits),
        "matched": matched_events,
        "recorded_without_gap": len(events) - matched_events,
        "gaps_without_recorded_event": len(hits) - len(used),
        "match_tolerance_s": WATCHDOG_MATCH_TOL_S,
        "note": (
            "Receive time at the recorder is not safety_node's own clock: safety_node measures "
            "the age of /drive_raw on its steady clock from its own receive time. A gap seen "
            "here can be absent there and the other way round."
        ),
    }


def watchdog_event_stamps(
    bag: SessionBag, pieces: Sequence[_RunPieces]
) -> list[tuple[float, float]]:
    """(event stamp, event receive time) of the recorded in-window watchdog ENGAGE records."""
    out: list[tuple[float, float]] = []
    for p in pieces:
        window = p.run.counting_window
        assert window is not None
        sel = (
            (bag.ev_phase == interventions.PHASE_ENGAGE)
            & in_window(bag.ev_t, window)
            & (bag.ev_source == "watchdog")
        )
        out.extend(
            (float(s), float(r)) for s, r in zip(bag.ev_stamp[sel], bag.ev_t[sel], strict=True)
        )
    return out


def acceptance_4_2(
    pieces: Sequence[_RunPieces], caps_mps: Sequence[float] | None, wall_contacts: int
) -> dict[str, Any]:
    if caps_mps is None:
        return {
            "status": "not computed",
            "reason": "the cap of each run is not in the bag; pass --caps or --caps-from-split",
        }
    if len(caps_mps) != len(pieces):
        return {
            "status": "not computed",
            "reason": f"{len(caps_mps)} caps given for {len(pieces)} runs",
        }
    res = gates.planner_car_acceptance([p.iv for p in pieces], list(caps_mps), wall_contacts)
    out = res.to_dict()
    out["status"] = "computed"
    out["wall_contacts_given"] = wall_contacts
    out["caps_mps"] = list(caps_mps)
    return out


@dataclasses.dataclass
class TimingAnalysis:
    pieces: list[_RunPieces]
    skipped_runs: list[str]
    report: dict[str, Any]


def analyse_timing(
    bag: SessionBag,
    caps_mps: Sequence[float] | None = None,
    wall_contacts: int = 0,
) -> TimingAnalysis:
    """The runs, pooled numbers, watchdog what-if and 4.2 acceptance of one bag. The rate limiter
    what-if is added by pnp_session.whatif (it needs the tool binary)."""
    runs = find_pnp_runs(bag)
    pieces: list[_RunPieces] = []
    skipped: list[str] = []
    for run in runs:
        if run.counting_window is None:
            skipped.append(run.run_id)
        else:
            pieces.append(_run_pieces(bag, run))
    run_entries = [_run_entry(p) for p in pieces]
    run_entries += [{"run_id": r, "commanded": False} for r in skipped]
    wd_events = watchdog_event_stamps(bag, pieces)
    report = {
        "runs": run_entries,
        "pooled": _pooled_entry(pieces),
        "watchdog_whatif": watchdog_whatif(pieces, wd_events),
        "acceptance_4_2": acceptance_4_2(pieces, caps_mps, wall_contacts),
    }
    return TimingAnalysis(pieces, skipped, report)
