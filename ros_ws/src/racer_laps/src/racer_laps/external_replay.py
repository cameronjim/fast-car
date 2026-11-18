"""Replay an external lap source through the lap engine (brief 51). Pure numpy, no ROS, no bag.

Arrays of what a lap node would have received (stamped wheel odometry, the gated steering, planner
states, safety events, the camera's events with their arrival times and the camera's statuses) go
in; the laps with their verdicts, the what-if caps and the final status come out. Used by the L4
tests, by D9-B0 and by `pnp_session lap-src`.

Every array carries two clocks where the node does: odometry has its header stamp (the time the
engine integrates on) and the time it arrived (the order of delivery); everything else only has
the time it arrived. All on one axis, in seconds. The engine is fed in arrival order. The camera's
`crossing_time_s` is on the camera node's clock, the same clock the odometry stamps and the arrival
times are on (the node's ROS clock), so no conversion is made.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from racer_laps.engine import LapConfig, LapMemory, LapResult
from racer_laps.external_gate import EXTERNAL_SOURCES, ExternalConfig, ExternalLapGate
from racer_laps.gate import GateConfig
from racer_laps.lap_record import STATE_DRIVING

FloatArray = NDArray[np.float64]

#: The order of delivery at equal arrival times: inputs before the event that may depend on them.
_ORDER = {"drive": 0, "planner": 1, "safety": 2, "status": 3, "odom": 4, "event": 5}


def _f() -> FloatArray:
    return np.zeros(0, dtype=np.float64)


def _i() -> NDArray[np.int64]:
    return np.zeros(0, dtype=np.int64)


def _s() -> NDArray[np.str_]:
    return np.zeros(0, dtype=str)


@dataclass
class ReplayStreams:
    """The inputs of a replay. Times in seconds on one axis."""

    #: /odom/wheel: arrival, header stamp, forward speed.
    odom_arrival: FloatArray = field(default_factory=_f)
    odom_stamp: FloatArray = field(default_factory=_f)
    odom_v: FloatArray = field(default_factory=_f)
    #: /drive (the gated steering): arrival, steering angle.
    drive_arrival: FloatArray = field(default_factory=_f)
    drive_steer: FloatArray = field(default_factory=_f)
    #: /pnp/planner/status: arrival, state, plan clearance.
    planner_arrival: FloatArray = field(default_factory=_f)
    planner_state: NDArray[np.int64] = field(default_factory=_i)
    planner_clearance: FloatArray = field(default_factory=_f)
    #: /safety/events: arrival, phase, severity, source.
    safety_arrival: FloatArray = field(default_factory=_f)
    safety_phase: NDArray[np.int64] = field(default_factory=_i)
    safety_severity: NDArray[np.int64] = field(default_factory=_i)
    safety_source: NDArray[np.str_] = field(default_factory=_s)
    #: /lap/event: arrival, lap number, crossing_time_s, and for reporting lap_time_s,
    #: distance_m and peak_similarity.
    event_arrival: FloatArray = field(default_factory=_f)
    event_lap: NDArray[np.int64] = field(default_factory=_i)
    event_cross: FloatArray = field(default_factory=_f)
    event_lap_time: FloatArray = field(default_factory=_f)
    event_distance: FloatArray = field(default_factory=_f)
    event_peak: FloatArray = field(default_factory=_f)
    #: /lap/status: arrival, state, laps, similarity (NaN for null).
    status_arrival: FloatArray = field(default_factory=_f)
    status_state: NDArray[np.str_] = field(default_factory=_s)
    status_laps: NDArray[np.int64] = field(default_factory=_i)
    status_sim: FloatArray = field(default_factory=_f)

    def save(self, path: Path) -> None:
        np.savez_compressed(path, **{f.name: getattr(self, f.name) for f in fields(self)})

    @classmethod
    def load(cls, path: Path) -> ReplayStreams:
        out = cls()
        with np.load(path, allow_pickle=False) as data:
            for f in fields(cls):
                if f.name in data.files:
                    setattr(out, f.name, data[f.name])
        return out

    def window(self, t0: float, t1: float) -> ReplayStreams:
        """The samples that arrived in [t0, t1]."""
        out = ReplayStreams()
        groups = {
            "odom_": self.odom_arrival,
            "drive_": self.drive_arrival,
            "planner_": self.planner_arrival,
            "safety_": self.safety_arrival,
            "event_": self.event_arrival,
            "status_": self.status_arrival,
        }
        for f in fields(self):
            for prefix, arrival in groups.items():
                if f.name.startswith(prefix):
                    keep = (arrival >= t0) & (arrival <= t1)
                    setattr(out, f.name, getattr(self, f.name)[keep])
                    break
        return out

    def truncated_events(self, n: int) -> ReplayStreams:
        """The streams up to the arrival of the first `n` camera events (plus the 0.5 s after it)."""
        if n >= len(self.event_arrival):
            return self
        t1 = float(self.event_arrival[n - 1]) + 0.5 if n > 0 else -math.inf
        return self.window(-math.inf, t1)


@dataclass(frozen=True)
class LapVerdict:
    """One lap the engine closed, as the status of the lap node would say it."""

    lap: int  # the gate's lap index (1 for the first event it accepted)
    event_lap: int  # the number of the source event
    confirmed: bool
    reasons: tuple[str, ...]
    t_cross_s: float
    lap_time_s: float
    length_m: float
    heading_rad: float
    cap_mps: float  # the cap the lap was driven under
    next_cap_mps: float  # the what-if cap after it
    clean: bool
    stored: bool


@dataclass
class ReplayResult:
    laps: list[LapResult] = field(default_factory=list)
    verdicts: list[LapVerdict] = field(default_factory=list)
    #: The status of the lap node at the end of the replay (its JSON keys).
    status: dict[str, Any] = field(default_factory=dict)
    #: (arrival time, guidance cap) at every odometry sample after the start.
    cap_series: list[tuple[float, float]] = field(default_factory=list)
    start_t: float = math.nan
    source_ready: bool = False
    source_not_ready_reason: str = ""
    events_in: int = 0
    #: The lowest guidance cap handed out between two times (see `min_cap_between`).
    assumed_ready: bool = False

    @property
    def confirmed(self) -> int:
        return sum(1 for v in self.verdicts if v.confirmed)

    @property
    def doubtful(self) -> int:
        return sum(1 for v in self.verdicts if not v.confirmed)

    @property
    def rejected(self) -> dict[str, int]:
        return dict(self.status.get("rejected_events", {}))  # type: ignore[call-overload]

    def rejected_total(self) -> int:
        return int(sum(self.rejected.values()))

    def min_cap_between(self, t0: float, t1: float) -> float:
        caps = [c for t, c in self.cap_series if t0 <= t <= t1]
        return min(caps) if caps else math.nan


def external_config(
    odom: GateConfig,
    source: str = "camera",
    mode: str = "v0",
    v_cap_start_mps: float = 0.5,
    v_cap_ceiling_mps: float = 1.0,
    ext: ExternalConfig | None = None,
) -> LapConfig:
    """A `LapConfig` for a replay: lap-1 cap, ceiling, the external rule and the odometry model."""
    if source not in EXTERNAL_SOURCES:
        raise ValueError(f"source must be one of {EXTERNAL_SOURCES}")
    return LapConfig(
        lap_source=source,
        lap_mode=mode,
        gate=odom,
        external=ext,
        v_cap_start_mps=v_cap_start_mps,
        v_cap_ceiling_mps=v_cap_ceiling_mps,
    )


def replay_external(
    streams: ReplayStreams,
    cfg: LapConfig,
    assume_ready: bool = False,
    renumber_events: bool = False,
) -> ReplayResult:
    """The streams through `LapMemory` on the external source of `cfg`, in arrival order.

    `assume_ready` is the replay's development switch (it is not, and must never become, a node
    parameter): just before the start it feeds one status (`running`, 0 laps, similarity 1.0), so a
    run that was not started for this purpose is judged by the rest of the rule. It is used where
    the ready rule is judged elsewhere (D9-B0 item iii, the E3 check).
    `renumber_events` makes the first camera event of the streams number 1, as a lap node started
    with this run would see a camera that was reset at its start."""
    if cfg.lap_source not in EXTERNAL_SOURCES:
        raise ValueError("replay_external needs lap_source camera or truth")
    mem = LapMemory(cfg)
    gate = mem.gate
    assert isinstance(gate, ExternalLapGate)
    s = streams
    items: list[tuple[float, int, str, int]] = []
    for kind, arrival in (
        ("odom", s.odom_arrival),
        ("drive", s.drive_arrival),
        ("planner", s.planner_arrival),
        ("safety", s.safety_arrival),
        ("event", s.event_arrival),
        ("status", s.status_arrival),
    ):
        items.extend((float(t), _ORDER[kind], kind, i) for i, t in enumerate(arrival))
    items.sort(key=lambda x: (x[0], x[1], x[3]))
    first_no = int(s.event_lap[0]) if len(s.event_lap) else 1
    out = ReplayResult(assumed_ready=assume_ready, events_in=len(s.event_arrival))
    driving_seen = False
    started = False
    for t, _o, kind, i in items:
        mem.on_clock(t)
        if kind == "drive":
            mem.on_drive(float(s.drive_steer[i]))
        elif kind == "planner":
            state = int(s.planner_state[i])
            driving_seen = driving_seen or state == STATE_DRIVING
            mem.on_planner_status(state, float(s.planner_clearance[i]))
        elif kind == "safety":
            mem.on_safety_event(
                int(s.safety_phase[i]), int(s.safety_severity[i]), str(s.safety_source[i])
            )
        elif kind == "status":
            sim = float(s.status_sim[i])
            mem.on_lap_source_status(
                t,
                str(s.status_state[i]),
                int(s.status_laps[i]),
                sim if math.isfinite(sim) else None,
            )
        elif kind == "odom":
            if assume_ready and driving_seen and not started:
                mem.on_lap_source_status(t, "running", 0, 1.0)
            mem.on_wheel_odom(float(s.odom_stamp[i]), float(s.odom_v[i]))
            if not started and mem.lap_in_progress > 0:
                started = True
                out.start_t = float(s.odom_stamp[i])
            if started:
                out.cap_series.append((t, mem.guidance().v_cap_mps))
        else:  # event
            no = int(s.event_lap[i]) - (first_no - 1 if renumber_events else 0)
            result = mem.on_lap_source_event(t, no, float(s.event_cross[i]))
            if result is not None:
                out.laps.append(result)
                out.verdicts.append(
                    _verdict(mem, result, int(s.event_lap[i]) if not renumber_events else no)
                )
    out.status = mem.status()
    out.source_ready = bool(gate.source_ready)
    out.source_not_ready_reason = gate.source_not_ready_reason
    return out


def _verdict(mem: LapMemory, r: LapResult, event_lap: int) -> LapVerdict:
    gate = mem.gate
    assert isinstance(gate, ExternalLapGate)
    reasons: tuple[str, ...] = ()
    if r.missed_gate:
        last = gate.doubtful[-1]
        reasons = tuple(str(x) for x in last["reasons"])  # type: ignore[attr-defined]
    return LapVerdict(
        lap=r.lap_index,
        event_lap=event_lap,
        confirmed=not r.missed_gate,
        reasons=reasons,
        t_cross_s=r.t_cross_s,
        lap_time_s=r.lap_time_s,
        length_m=r.length_m,
        heading_rad=r.heading_rad,
        cap_mps=r.cap_mps,
        next_cap_mps=r.next_cap_mps,
        clean=r.clean,
        stored=r.stored,
    )
