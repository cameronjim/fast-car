"""The session report: the checks an analyst did by hand after car session 1 (2026-11-22), as code.

    cd tools && uv run --project . --extra session python -m pnp_session session \\
        --bag BAG [--bag BAG] [--out report.json]

Read only, from a recorded bag (a bag that was not finalised is read as far as it goes). One block
per run of the bag (a run is a stretch of `/pnp/planner/status` with no gap over 5 s):

* the lap table: per lap the camera's peak similarity and distance, the lap node's length, the
  ratio of that length to the lap node's reference length, the cap, brakes, clearance, the clean
  flag and why a lap is not clean (brake, clearance, the lap node's doubtful reasons, a planner
  blip inside the lap, a state other than DRIVING inside the lap);
* the peak table: min, median and max camera peak of the run and the peaks under 0.88;
* missed crossings: a camera event whose distance is about two (or more) reference laps, and the
  laps the wheels drove against the camera events counted;
* blips: a short BLOCKED stretch between DRIVING ticks with the car moving (state, reason,
  duration, speed lost);
* planner samples (histogram over DRIVING ticks) and solve times;
* safety timing: every `watchdog` and `rate_limit` ENGAGE with the leg at fault (the safety cycle
  ran late, or `/drive_raw` had a publish gap), whether it came at the planner's first command,
  and the latch repeats and skips of `/safety/cycle`;
* the start: seconds from the first DRIVING command to the first wheel motion.

Nothing here drives a car, writes a parameter or judges a gate. The thresholds below are
conventions of this report (marked so), not registered numbers. Times are seconds on the bag axis
(after the log time of the first `/scan`), like the other `pnp_session` commands.
"""

from __future__ import annotations

import json
import math
import statistics
from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

SCHEMA = "pnp-session-checks/1"

TOPIC_PLANNER = "/pnp/planner/status"
TOPIC_PNP_EVENT = "/pnp/lap/event"
TOPIC_PNP_STATUS = "/pnp/lap/status"
TOPIC_CAM_EVENT = "/lap/event"
TOPIC_SAFETY_EVENTS = "/safety/events"
TOPIC_SAFETY_CYCLE = "/safety/cycle"
TOPIC_ODOM = "/odom/wheel"
TOPIC_DRIVE_RAW = "/drive_raw"
TOPICS = (
    TOPIC_PLANNER,
    TOPIC_PNP_EVENT,
    TOPIC_PNP_STATUS,
    TOPIC_CAM_EVENT,
    TOPIC_SAFETY_EVENTS,
    TOPIC_SAFETY_CYCLE,
    TOPIC_ODOM,
    TOPIC_DRIVE_RAW,
)

STATE_DRIVING = 1
STATE_BLOCKED = 5
#: PlannerStatus.block_reason values.
BLOCK_NAMES = {
    0: "none",
    1: "all_collide",
    2: "below_speed_floor",
    3: "wheel_stalled",
    4: "gate_refusing",
    5: "recovery_exhausted",
}
PHASE_ENGAGE = 0

#: Convention: planner status further apart than this starts a new run (the analyst's rule).
RUN_GAP_S = 5.0
#: Convention: a BLOCKED stretch of at most this long, between DRIVING ticks, with the car
#: moving, is a blip; a longer one is a block.
BLIP_MAX_S = 0.5
#: Convention: the car counts as moving above this wheel speed (m/s).
MOVING_MPS = 0.3
#: Convention: the speed before a blip is the highest estimate in this window before it, and the
#: speed after it the lowest in the window after it (seconds).
BLIP_BEFORE_S = 0.3
BLIP_AFTER_S = 0.5
#: Convention: a camera event whose distance is at least this many reference laps missed one.
MISSED_RATIO = 1.6
#: Convention: the D10-C0 peak floor, to show the peaks under it.
PEAK_FLOOR = 0.88
#: Convention: a gate cycle longer than this is late (nominal 20 ms), a /drive_raw receive gap
#: longer than this is a publish gap (the watchdog's own limit is 60 ms).
LATE_CYCLE_S = 0.045
COMMAND_GAP_S = 0.050
#: Convention: look this far before an engagement's record, and a little after, for its cause.
CAUSE_BEFORE_S = 0.15
CAUSE_AFTER_S = 0.03
#: Convention: an engagement within this long after the first command of a run is "at start".
START_WINDOW_S = 0.5
#: Convention: wheels still after this long since the first DRIVING command: the car did not answer.
UNANSWERED_S = 3.0
#: Convention: the planner's full sample count (the session of 2026-11-22 ran the planner at 512).
PLANNER_FULL_SAMPLES = 512
#: Convention: a solve over this is a late solve (the sample budget's own halving limit).
LATE_SOLVE_S = 0.015
#: Convention: clean limit of the lap node (racer_laps.engine.LapEngineConfig.min_clearance_clean_m).
CLEAN_CLEARANCE_M = 0.08
#: A wheel odometry gap longer than this adds no distance (the lap gate's own rule).
MAX_ODOM_DT_S = 1.0


@dataclass
class SessionData:
    """The columns of one bag the checks read, all on the bag axis."""

    name: str
    t0_log_s: float = 0.0
    # planner: arrival, stamp, state, cmd speed, est speed, cap, clearance, solve, samples, reason
    st_t: list[float] = field(default_factory=list)
    st_stamp: list[float] = field(default_factory=list)
    st_state: list[int] = field(default_factory=list)
    st_cmd: list[float] = field(default_factory=list)
    st_est: list[float] = field(default_factory=list)
    st_cap: list[float] = field(default_factory=list)
    st_clear: list[float] = field(default_factory=list)
    st_solve: list[float] = field(default_factory=list)
    st_samples: list[int] = field(default_factory=list)
    st_reason: list[int] = field(default_factory=list)
    # the lap node: events (arrival, record) and status JSON (arrival, dict)
    pnp_events: list[tuple[float, dict[str, Any]]] = field(default_factory=list)
    pnp_status: list[tuple[float, dict[str, Any]]] = field(default_factory=list)
    # the camera's events (arrival, dict, with crossing_s on the bag axis)
    cam_events: list[tuple[float, dict[str, Any]]] = field(default_factory=list)
    # /safety/events: arrival, severity, phase, source, detail
    ev: list[tuple[float, int, int, str, str]] = field(default_factory=list)
    # /safety/cycle: arrival, has_command, raw age, raw count, dt
    cy_t: list[float] = field(default_factory=list)
    cy_has: list[bool] = field(default_factory=list)
    cy_age: list[float] = field(default_factory=list)
    cy_count: list[int] = field(default_factory=list)
    cy_dt: list[float] = field(default_factory=list)
    # wheel odometry: arrival, header stamp (or arrival), speed
    od_t: list[float] = field(default_factory=list)
    od_stamp: list[float] = field(default_factory=list)
    od_v: list[float] = field(default_factory=list)
    # /drive_raw: arrival, header stamp
    dr_t: list[float] = field(default_factory=list)
    dr_stamp: list[float] = field(default_factory=list)
    reports: list[Any] = field(default_factory=list)


def _stamp_s(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _json(data: str) -> dict[str, Any] | None:
    try:
        d = json.loads(data)
    except ValueError:
        return None
    return d if isinstance(d, dict) else None


def data_from_messages(
    name: str, t0_ns: int, messages: Sequence[tuple[int, str, Any]]
) -> SessionData:
    """Columns from decoded messages (log time ns, topic, message)."""
    out = SessionData(name=name, t0_log_s=t0_ns / 1e9)
    for t_ns, topic, m in messages:
        t = (t_ns - t0_ns) / 1e9
        if topic == TOPIC_PLANNER:
            out.st_t.append(t)
            out.st_stamp.append(_stamp_s(m.header.stamp) - out.t0_log_s)
            out.st_state.append(int(m.state))
            out.st_cmd.append(float(m.cmd_speed_mps))
            out.st_est.append(float(m.est_speed_mps))
            out.st_cap.append(float(m.v_cap_mps))
            out.st_clear.append(float(m.plan_min_clearance_m))
            out.st_solve.append(float(m.solve_time_s))
            out.st_samples.append(int(m.samples))
            out.st_reason.append(int(m.block_reason))
        elif topic == TOPIC_PNP_EVENT:
            out.pnp_events.append(
                (
                    t,
                    {
                        "lap": int(m.lap_index),
                        "lap_time_s": float(m.lap_time_s),
                        "length_m": float(m.lap_length_m),
                        "cap_mps": float(m.v_cap_mps),
                        "brakes": int(m.safety_brakes),
                        "min_clearance_m": float(m.min_clearance_m),
                        "clean": bool(m.clean),
                    },
                )
            )
        elif topic == TOPIC_PNP_STATUS:
            d = _json(m.data)
            if d is not None:
                out.pnp_status.append((t, d))
        elif topic == TOPIC_CAM_EVENT:
            d = _json(m.data)
            if d is None or "lap" not in d or "crossing_time_s" not in d:
                continue
            out.cam_events.append(
                (
                    t,
                    {
                        "lap": int(d["lap"]),
                        "peak": _f(d.get("peak_similarity")),
                        "distance_m": _f(d.get("distance_m")),
                        "lap_time_s": _f(d.get("lap_time_s")),
                        "crossing_s": float(d["crossing_time_s"]) - out.t0_log_s,
                    },
                )
            )
        elif topic == TOPIC_SAFETY_EVENTS:
            out.ev.append((t, int(m.severity), int(m.phase), str(m.source), str(m.detail)))
        elif topic == TOPIC_SAFETY_CYCLE:
            out.cy_t.append(t)
            out.cy_has.append(bool(m.has_command))
            out.cy_age.append(float(m.drive_raw_age_s))
            out.cy_count.append(int(m.drive_raw_count))
            out.cy_dt.append(float(m.cycle_dt_s))
        elif topic == TOPIC_ODOM:
            stamp = _stamp_s(m.header.stamp)
            out.od_t.append(t)
            out.od_stamp.append(stamp - out.t0_log_s if stamp > 0.0 else t)
            out.od_v.append(float(m.twist.twist.linear.x))
        elif topic == TOPIC_DRIVE_RAW:
            stamp = _stamp_s(m.header.stamp)
            out.dr_t.append(t)
            out.dr_stamp.append(stamp - out.t0_log_s if stamp > 0.0 else t)
    return out


def _f(x: Any) -> float:
    return float(x) if isinstance(x, (int, float)) else math.nan


def read_bag(path: Path) -> SessionData:
    """A bag read with the lenient stream reader."""
    from pnp_session import streamread

    t0_ns, messages, reports = streamread.read_topics(Path(path), TOPICS)
    data = data_from_messages(Path(path).name, t0_ns, messages)
    data.reports = reports
    return data


# -- small helpers ---------------------------------------------------------------------------------


def _pct(values: Sequence[float], q: float) -> float | None:
    a = np.asarray([v for v in values if math.isfinite(v)], dtype=float)
    return float(np.percentile(a, q)) if len(a) else None


def _rng(t: Sequence[float], a: float, b: float) -> slice:
    """Index range of the sorted `t` inside [a, b]."""
    arr = np.asarray(t, dtype=float)
    return slice(int(np.searchsorted(arr, a, "left")), int(np.searchsorted(arr, b, "right")))


def _wheel_distance(t: Sequence[float], v: Sequence[float]) -> float:
    if len(t) < 2:
        return 0.0
    tt = np.asarray(t, dtype=float)
    speed = np.maximum(np.asarray(v, dtype=float), 0.0)
    dt = np.diff(tt)
    step = np.where((dt > 0.0) & (dt <= MAX_ODOM_DT_S), 0.5 * (speed[1:] + speed[:-1]) * dt, 0.0)
    return float(np.sum(step))


def find_runs(d: SessionData) -> list[tuple[float, float]]:
    """(first, last) planner status arrival of each run: gaps above RUN_GAP_S split."""
    runs: list[tuple[float, float]] = []
    if not d.st_t:
        return runs
    start = prev = d.st_t[0]
    for t in d.st_t[1:]:
        if t - prev > RUN_GAP_S:
            runs.append((start, prev))
            start = t
        prev = t
    runs.append((start, prev))
    return runs


# -- the pieces ------------------------------------------------------------------------------------


def blips(d: SessionData, a: float, b: float) -> list[dict[str, Any]]:
    """BLOCKED stretches inside the run, between DRIVING ticks, short, with the car moving."""
    sl = _rng(d.st_t, a, b)
    idx = range(sl.start, sl.stop)
    t = d.st_t
    out: list[dict[str, Any]] = []
    i = sl.start
    while i < sl.stop:
        if d.st_state[i] != STATE_BLOCKED:
            i += 1
            continue
        j = i
        while j + 1 < sl.stop and d.st_state[j + 1] == STATE_BLOCKED:
            j += 1
        before_ok = i > sl.start and d.st_state[i - 1] == STATE_DRIVING
        after_ok = j + 1 < sl.stop and d.st_state[j + 1] == STATE_DRIVING
        duration = t[j + 1] - t[i] if after_ok else t[j] - t[i]
        pre = [d.st_est[k] for k in idx if t[i] - BLIP_BEFORE_S <= t[k] < t[i]]
        post = [d.st_est[k] for k in idx if t[i] <= t[k] <= t[i] + BLIP_AFTER_S]
        v_before = max(pre) if pre else math.nan
        v_min = min(post) if post else math.nan
        is_blip = before_ok and after_ok and duration <= BLIP_MAX_S and v_before > MOVING_MPS
        out.append(
            {
                "kind": "blip" if is_blip else "block",
                "t_s": t[i],
                "ticks": j - i + 1,
                "duration_s": duration,
                "reason": BLOCK_NAMES.get(d.st_reason[i], str(d.st_reason[i])),
                "cmd_min_mps": min(d.st_cmd[i : j + 1]),
                "speed_before_mps": v_before,
                "speed_min_after_mps": v_min,
                "speed_lost_mps": v_before - v_min if math.isfinite(v_before + v_min) else None,
            }
        )
        i = j + 1
    return out


def samples_block(d: SessionData, a: float, b: float) -> dict[str, Any]:
    """Planner samples over DRIVING ticks and the solve times of every tick of the run."""
    sl = _rng(d.st_t, a, b)
    drv = [k for k in range(sl.start, sl.stop) if d.st_state[k] == STATE_DRIVING]
    hist = Counter(d.st_samples[k] for k in drv)
    solve_ms = [d.st_solve[k] * 1e3 for k in range(sl.start, sl.stop)]
    top = max(hist) if hist else None
    return {
        "driving_ticks": len(drv),
        "histogram": {int(k): int(v) for k, v in sorted(hist.items())},
        "max_samples": top,
        "share_at_max": (hist[top] / len(drv)) if top is not None and drv else None,
        "share_at_full": (
            sum(v for k, v in hist.items() if k >= PLANNER_FULL_SAMPLES) / len(drv) if drv else None
        ),
        "median": float(statistics.median(d.st_samples[k] for k in drv)) if drv else None,
        "solve_ms": {
            "p50": _pct(solve_ms, 50),
            "p95": _pct(solve_ms, 95),
            "p99": _pct(solve_ms, 99),
            "max": max(solve_ms) if solve_ms else None,
            "late_over_15ms": int(sum(1 for s in solve_ms if s > LATE_SOLVE_S * 1e3)),
        },
    }


def _first_command_s(d: SessionData, a: float, b: float) -> float | None:
    sl = _rng(d.st_t, a, b)
    for k in range(sl.start, sl.stop):
        if d.st_state[k] == STATE_DRIVING and d.st_cmd[k] > 0.1:
            return d.st_t[k]
    return None


def start_block(d: SessionData, a: float, b: float) -> dict[str, Any]:
    """Seconds from the first DRIVING command above 0.1 m/s to the first wheel speed above it."""
    t_cmd = _first_command_s(d, a, b)
    if t_cmd is None:
        return {"first_command_s": None, "wheel_response_s": None, "unanswered": False}
    sl = _rng(d.od_t, t_cmd, b)
    t_wheel = next((d.od_t[k] for k in range(sl.start, sl.stop) if d.od_v[k] > 0.1), None)
    resp = None if t_wheel is None else t_wheel - t_cmd
    return {
        "first_command_s": t_cmd,
        "wheel_response_s": resp,
        "unanswered": resp is None or resp > UNANSWERED_S,
    }


def _cause(d: SessionData, t: float) -> dict[str, Any]:
    """The leg at fault of an engagement recorded at `t`: the longest gate cycle and the longest
    /drive_raw receive gap (log time) in the window around it."""
    lo, hi = t - CAUSE_BEFORE_S, t + CAUSE_AFTER_S
    cs = _rng(d.cy_t, lo, hi)
    dt_max = max(d.cy_dt[cs], default=math.nan) if cs.stop > cs.start else math.nan
    age_max = max(d.cy_age[cs], default=math.nan) if cs.stop > cs.start else math.nan
    ds = _rng(d.dr_t, lo, hi)
    # receive gaps (log time: what the gate waited for); the message before the window counts only
    # when it is recent, so the first command after a silence is not a gap
    first = (
        ds.start - 1 if ds.start > 0 and lo - d.dr_t[ds.start - 1] <= CAUSE_BEFORE_S else ds.start
    )
    arrivals = d.dr_t[first : ds.stop]
    gap_max = float(np.max(np.diff(arrivals))) if len(arrivals) > 1 else math.nan
    late = math.isfinite(dt_max) and dt_max > LATE_CYCLE_S
    gap = math.isfinite(gap_max) and gap_max > COMMAND_GAP_S
    leg = (
        "both"
        if late and gap
        else "safety_cycle_late"
        if late
        else "command_gap"
        if gap
        else "undetermined"
    )
    return {
        "leg": leg,
        "cycle_dt_max_s": None if not math.isfinite(dt_max) else float(dt_max),
        "drive_raw_age_max_s": None if not math.isfinite(age_max) else float(age_max),
        "drive_raw_gap_max_s": None if not math.isfinite(gap_max) else float(gap_max),
    }


def engagements(d: SessionData, a: float, b: float) -> list[dict[str, Any]]:
    """Every watchdog and rate_limit ENGAGE of the run, with the leg at fault."""
    sl = _rng(d.dr_t, a - 1.0, b)
    first = d.dr_t[sl.start] if sl.stop > sl.start else None  # the planner's first /drive_raw
    out = []
    for t, severity, phase, source, detail in d.ev:
        if phase != PHASE_ENGAGE or not a <= t <= b or source not in ("watchdog", "rate_limit"):
            continue
        out.append(
            {
                "t_s": t,
                "source": source,
                "severity": severity,
                "at_first_command": first is not None and -0.05 <= t - first <= START_WINDOW_S,
                "detail": detail,
                **_cause(d, t),
            }
        )
    return out


def latch_block(d: SessionData, a: float, b: float) -> dict[str, Any]:
    """Gate cycles of the run: how often the latched command was the same as the cycle before (a
    repeat) or newer by more than one (a skip), the cycle time and the age of the command."""
    sl = _rng(d.cy_t, a, b)
    count = d.cy_count[sl]
    dt = [x * 1e3 for x in d.cy_dt[sl]]
    age = [x * 1e3 for x in d.cy_age[sl] if x >= 0.0]
    diffs = (
        np.diff(np.asarray(count, dtype=np.int64))
        if len(count) > 1
        else np.array([], dtype=np.int64)
    )
    return {
        "cycles": len(count),
        "repeated": int(np.sum(diffs == 0)),
        "skipped": int(np.sum(diffs > 1)),
        "dt_ms": {
            "p50": _pct(dt, 50),
            "p99": _pct(dt, 99),
            "max": max(dt) if dt else None,
            "over_45ms": int(sum(1 for x in dt if x > LATE_CYCLE_S * 1e3)),
        },
        "command_age_ms_p50": _pct(age, 50),
    }


def lap_table(
    d: SessionData, a: float, b: float, blip_list: list[dict[str, Any]]
) -> dict[str, Any]:
    """Per lap of the lap node: the camera's peak and distance, length and its ratio to the
    reference, cap, clean and why not; then the peak summary and the missed crossings."""
    ev = [(t, e) for t, e in d.pnp_events if a - 1.0 <= t <= b + 1.0]
    cam = [(t, e) for t, e in d.cam_events if a - 1.0 <= t <= b + 1.0]
    status = [(t, s) for t, s in d.pnp_status if a - 1.0 <= t <= b + 1.0]
    last = status[-1][1] if status else {}
    doubtful = {int(x["lap"]): list(x.get("reasons", [])) for x in last.get("doubtful_laps", [])}
    missed_gate = {int(k) for k in last.get("missed_gate_laps", [])}
    used: set[int] = set()
    rows: list[dict[str, Any]] = []
    prev_t = a
    for t, e in ev:
        # the camera event the lap node got this one from: the latest not used, just before it
        match = None
        for k in range(len(cam) - 1, -1, -1):
            if k not in used and cam[k][0] <= t + 0.1 and t - cam[k][0] <= 1.0:
                match = k
                break
        if match is not None:
            used.add(match)
        c = cam[match][1] if match is not None else {}
        ref = next((s.get("l_ref_m") for tt, s in reversed(status) if tt <= t), None)
        ref = ref if isinstance(ref, (int, float)) and ref > 0 else None
        nb = [x for x in blip_list if x["kind"] == "blip" and prev_t <= x["t_s"] <= t]
        sl = _rng(d.st_t, prev_t, t)
        not_driving = sum(1 for k in range(sl.start, sl.stop) if d.st_state[k] != STATE_DRIVING)
        why: list[str] = []
        if e["brakes"] > 0:
            why.append(f"brakes {e['brakes']}")
        if e["min_clearance_m"] < CLEAN_CLEARANCE_M:
            why.append(f"clearance {e['min_clearance_m']:.2f}")
        if e["lap"] in doubtful:
            why.append("doubtful " + "+".join(doubtful[e["lap"]]))
        if e["lap"] in missed_gate:
            why.append("gate missed")
        if nb:
            why.append(f"blip x{len(nb)}")
        if not_driving:
            why.append(f"not DRIVING x{not_driving}")
        rows.append(
            {
                "lap": e["lap"],
                "arrival_s": t,
                "peak": c.get("peak"),
                "camera_distance_m": c.get("distance_m"),
                "length_m": e["length_m"],
                "ref_m": ref,
                "ratio": e["length_m"] / ref if ref is not None else None,
                "lap_time_s": e["lap_time_s"],
                "cap_mps": e["cap_mps"],
                "brakes": e["brakes"],
                "min_clearance_m": e["min_clearance_m"],
                "clean": e["clean"],
                "doubtful": doubtful.get(e["lap"], []),
                "blips": len(nb),
                "why_not_clean": [] if e["clean"] else why,
                "other_flags": why if e["clean"] else [],
            }
        )
        prev_t = t
    peaks = [
        c["peak"] for _, c in cam if isinstance(c.get("peak"), float) and math.isfinite(c["peak"])
    ]
    cam_len = [c["distance_m"] for _, c in cam[1:] if math.isfinite(c["distance_m"])]
    lens = [r["ref_m"] for r in rows if r["ref_m"] is not None] or cam_len
    ref_m = statistics.median(lens) if lens else None
    missed = []
    if ref_m:
        for t, c in cam:
            ratio = c["distance_m"] / ref_m if math.isfinite(c["distance_m"]) else 0.0
            if ratio >= MISSED_RATIO:
                missed.append(
                    {
                        "camera_lap": c["lap"],
                        "arrival_s": t,
                        "distance_m": c["distance_m"],
                        "laps_in_it": round(ratio),
                        "missed": max(round(ratio) - 1, 1),
                    }
                )
    odom = _rng(d.od_stamp, a, b)
    wheel_m = _wheel_distance(d.od_stamp[odom], d.od_v[odom])
    return {
        "rows": rows,
        "camera_events": len(cam),
        "lap_node_events": len(ev),
        "peaks": {
            "n": len(peaks),
            "min": min(peaks, default=None),
            "median": statistics.median(peaks) if peaks else None,
            "max": max(peaks, default=None),
            "under_floor": sorted(round(p, 3) for p in peaks if p < PEAK_FLOOR),
            "floor": PEAK_FLOOR,
        },
        "reference_m": ref_m,
        "wheel_distance_m": wheel_m,
        "laps_driven": wheel_m / ref_m if ref_m else None,
        "missed_crossings": missed,
        "mode": last.get("lap_mode"),
        "source": last.get("lap_source"),
    }


def run_block(d: SessionData, index: int, a: float, b: float) -> dict[str, Any]:
    bl = blips(d, a, b)
    return {
        "run": index,
        "start_s": a,
        "end_s": b,
        "duration_s": b - a,
        "start": start_block(d, a, b),
        "laps": lap_table(d, a, b, bl),
        "blips": [x for x in bl if x["kind"] == "blip"],
        "blocks": [x for x in bl if x["kind"] == "block"],
        "planner": samples_block(d, a, b),
        "engagements": engagements(d, a, b),
        "latch": latch_block(d, a, b),
    }


def analyse(d: SessionData) -> dict[str, Any]:
    runs = [run_block(d, i + 1, a, b) for i, (a, b) in enumerate(find_runs(d))]
    return {"schema": SCHEMA, "bag": d.name, "t0_log_s": d.t0_log_s, "runs": runs}


# -- text ------------------------------------------------------------------------------------------


def _n(x: Any, fmt: str = ".2f") -> str:
    return "-" if x is None or (isinstance(x, float) and math.isnan(x)) else format(x, fmt)


def markdown(doc: dict[str, Any]) -> str:
    lines = [f"# Session report: {doc['bag']}", ""]
    if not doc["runs"]:
        return "\n".join([*lines, "No planner status in the bag: no run found.", ""])
    for r in doc["runs"]:
        laps = r["laps"]
        lines += [
            (
                f"## Run {r['run']}: {r['start_s']:.1f} to {r['end_s']:.1f} s "
                f"({r['duration_s']:.0f} s), log time {doc['t0_log_s'] + r['start_s']:.3f} to "
                f"{doc['t0_log_s'] + r['end_s']:.3f} (for tools/lap_replay.py --t-from, --t-to)"
            ),
            "",
            (
                f"Lap source {laps['source']}, mode {laps['mode']}. Camera events "
                f"{laps['camera_events']}, lap node events {laps['lap_node_events']}, wheels drove "
                f"{_n(laps['wheel_distance_m'], '.1f')} m = {_n(laps['laps_driven'], '.1f')} laps "
                f"of {_n(laps['reference_m'])} m."
            ),
        ]
        st = r["start"]
        if st["first_command_s"] is not None:
            lines.append(
                f"Start: first command at {st['first_command_s']:.2f} s, wheels moved "
                f"{_n(st['wheel_response_s'])} s later"
                + (
                    f". CAR DID NOT ANSWER (over {UNANSWERED_S:.0f} s)."
                    if st["unanswered"]
                    else "."
                )
            )
        lines += [
            "",
            "| lap | peak | cam m | len m | ref m | ratio | time s | cap | brakes | clear m | clean | why not |",
            "|---|---|---|---|---|---|---|---|---|---|---|---|",
        ]
        for x in laps["rows"]:
            why = ", ".join(x["why_not_clean"]) if not x["clean"] else ", ".join(x["other_flags"])
            lines.append(
                f"| {x['lap']} | {_n(x['peak'], '.3f')} | {_n(x['camera_distance_m'])} | "
                f"{_n(x['length_m'])} | {_n(x['ref_m'])} | {_n(x['ratio'])} | {_n(x['lap_time_s'])} | "
                f"{_n(x['cap_mps'])} | {x['brakes']} | {_n(x['min_clearance_m'], '.3f')} | "
                f"{'yes' if x['clean'] else 'NO'} | {why} |"
            )
        p = laps["peaks"]
        lines += [
            "",
            (
                f"Peaks: n {p['n']}, min {_n(p['min'], '.3f')}, median {_n(p['median'], '.3f')}, "
                f"max {_n(p['max'], '.3f')}; under {p['floor']}: {p['under_floor'] or 'none'}."
            ),
        ]
        if laps["missed_crossings"]:
            lines.append(
                "Missed crossings: "
                + "; ".join(
                    f"camera lap {m['camera_lap']} ({m['distance_m']:.2f} m, about {m['laps_in_it']} laps, "
                    f"{m['missed']} missed)"
                    for m in laps["missed_crossings"]
                )
            )
        else:
            lines.append("Missed crossings: none.")
        lines.append("")
        if r["blips"]:
            lines.append("Blips (short BLOCKED while driving):")
            lines += [
                f"- {x['t_s']:.2f} s: {x['reason']}, {x['ticks']} ticks, {x['duration_s'] * 1e3:.0f} ms, "
                f"commanded min {x['cmd_min_mps']:.2f}, speed {_n(x['speed_before_mps'])} to "
                f"{_n(x['speed_min_after_mps'])} m/s (lost {_n(x['speed_lost_mps'])})"
                for x in r["blips"]
            ]
        else:
            lines.append("Blips: none.")
        if r["blocks"]:
            lines.append("Blocks (long, or the car was not moving):")
            lines += [
                f"- {x['t_s']:.2f} s: {x['reason']}, {x['duration_s']:.2f} s, {x['ticks']} ticks"
                for x in r["blocks"]
            ]
        pl = r["planner"]
        s = pl["solve_ms"]
        share = pl["share_at_max"]
        lines += [
            "",
            (
                f"Planner samples over {pl['driving_ticks']} DRIVING ticks: {pl['histogram']}; "
                f"median {_n(pl['median'], '.0f')}, at {PLANNER_FULL_SAMPLES}: "
                f"{_n(None if pl['share_at_full'] is None else 100 * pl['share_at_full'], '.1f')} percent, "
                f"at the most seen ({pl['max_samples']}): {_n(None if share is None else 100 * share, '.0f')} percent. "
                f"Solve ms p50 {_n(s['p50'], '.1f')}, p95 {_n(s['p95'], '.1f')}, p99 {_n(s['p99'], '.1f')}, "
                f"max {_n(s['max'], '.1f')}; over 15 ms: {s['late_over_15ms']}."
            ),
        ]
        lt = r["latch"]
        lines.append(
            f"Safety cycles {lt['cycles']}: dt p50 {_n(lt['dt_ms']['p50'], '.1f')} ms, p99 "
            f"{_n(lt['dt_ms']['p99'], '.1f')}, max {_n(lt['dt_ms']['max'], '.1f')}, over 45 ms "
            f"{lt['dt_ms']['over_45ms']}; latch repeated {lt['repeated']}, skipped {lt['skipped']}; "
            f"command age p50 {_n(lt['command_age_ms_p50'], '.1f')} ms."
        )
        if r["engagements"]:
            lines.append("Watchdog and rate_limit engagements:")
            lines += [
                f"- {x['t_s']:.2f} s {x['source']}: leg {x['leg']} (cycle dt max "
                f"{_n(None if x['cycle_dt_max_s'] is None else x['cycle_dt_max_s'] * 1e3, '.0f')} ms, "
                f"/drive_raw gap max "
                f"{_n(None if x['drive_raw_gap_max_s'] is None else x['drive_raw_gap_max_s'] * 1e3, '.0f')} ms)"
                + (", at the first command" if x["at_first_command"] else "")
                for x in r["engagements"]
            ]
        else:
            lines.append("Watchdog and rate_limit engagements: none.")
        lines.append("")
    return "\n".join(lines)


def run(bags: Sequence[Path]) -> tuple[dict[str, Any], str]:
    """(report, text) of one or more bags."""
    docs = []
    text = []
    for b in bags:
        d = read_bag(Path(b))
        for rep in d.reports:
            if not rep.complete:
                text.append(f"NOTE: {rep.say()}")
        doc = analyse(d)
        docs.append(doc)
        text.append(markdown(doc))
    return {"schema": SCHEMA, "bags": docs}, "\n".join(text)
