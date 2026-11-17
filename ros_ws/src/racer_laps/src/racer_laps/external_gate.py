"""The lap source rule of brief 51: D takes its laps from an outside lap instrument (the camera lap
counter on the car, a truth oracle in the simulator) and says when it does not believe one.

`ExternalLapGate` stands where `racer_laps.gate.StartScanGate` stands inside `LapMemory` when
`lap_source` is `camera` or `truth`. It answers what the engine asks of a gate (`started`,
`laps_completed`, `l_ref_m`, `l_ref_trusted`, `suspect_laps`, `station_m`, `armed`, `on_odom`,
`on_steering`, `poll_uncertain`, `status`). It keeps its own wheel odometry (the integrator of
`StartScanGate.on_odom`: forward distance only, trapezoid on speed, heading from the gated steering
through the yaw gain) and 30 s of (stamp, wheel total, heading).

The rule (design 13 section 21.2, pre-registration `preregistration-pnp-v0.5-laps-camera.md`,
constants in `evaluation/prereg/pnp-v0.5-laps-camera.yaml` `source_rule`, frozen with it):

* The lap ends at the event's `crossing_time_s` (the peak frame), never at the event's arrival and
  never from the 1 Hz status. The wheel total and the heading at the crossing are interpolated
  from the odometry history, so everything driven after the crossing belongs to the next lap.
* An event is rejected (counted by reason, nothing changes) before the start, when a value is not
  finite, when its crossing is later than its arrival, when it arrives too long after its
  crossing, when its crossing is not after the previous crossing, or when its number repeats.
* Otherwise it closes a lap. The lap is confirmed only if the run started ready, the event
  numbers are consecutive, the odometry covers the crossing, the source stayed alive, and the wheel
  length and the heading fit. Else it is doubtful, with every failed reason listed. A doubtful lap
  is returned as a `GateCrossing` with `missed=True`: the engine's existing missed-gate path (not
  clean, not stored, one cap step down, two clean laps to rise). It never enters the reference
  length.
* With no crossing past the reference length plus its band, or with a silent source, the station is
  uncertain until the next crossing and the engine hands out the lap-1 cap. Nothing invents a lap
  and nothing closes a lap by odometry alone.

Numpy and the standard library only.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from racer_laps.gate import (
    BAND_ABS_M,
    BAND_REL,
    MAX_ODOM_DT_S,
    GateConfig,
    GateCrossing,
    yaw_gain,
)

TWO_PI = 2.0 * math.pi
#: The sources this gate serves (`GateCrossing.line_source` and `LapResult.source` carry the name).
EXTERNAL_SOURCES = ("camera", "truth")
#: Odometry history kept (seconds). A crossing is placed back in it by its time; an event is
#: rejected after `max_event_latency_s` (2.5 s), so 30 s is generous.
HISTORY_S = 30.0
#: Tolerance of "within" and "at least" against the float noise of a difference of two lengths
#: (convention, as `pnp_session.lap_c0.EPS`): far below anything a wheel can measure.
EPS = 1e-9

#: Reasons an event is rejected (`rejected_events` of the status). `malformed` is the node's: a
#: JSON message that did not parse or lacked a field.
REJECT_REASONS = (
    "before_start",
    "not_finite",
    "future",
    "latency",
    "not_monotone",
    "duplicate",
    "malformed",
)
#: Reasons a lap is doubtful.
DOUBTFUL_REASONS = ("ready", "sequence", "odometry", "alive", "length", "heading")


@dataclass(frozen=True)
class ExternalConfig:
    """The `source_rule` block of `evaluation/prereg/pnp-v0.5-laps-camera.yaml`. Fixed by the
    pre-registration; a test pins every default to the yaml."""

    #: Arrival minus crossing time above this: the event is rejected.
    max_event_latency_s: float = 2.5
    #: A crossing time later than its arrival by more than this: rejected.
    future_tol_s: float = 0.05
    #: The camera node's `min_lap_distance_m`; the node passes its `min_lap_length_m` parameter.
    min_lap_length_m: float = 3.0
    #: A lap's heading must be within this of 2 pi in magnitude.
    heading_slack_rad: float = 1.2
    #: The lap length band, the scan gate's `BAND_ABS_M` and `BAND_REL`.
    band_abs_m: float = BAND_ABS_M
    band_rel: float = BAND_REL
    #: The reference length is the median of the last this many confirmed laps.
    length_median_laps: int = 5
    #: While fewer than 2 confirmed laps: overdue after this many reference lengths (the scan
    #: gate's `missed_factor`).
    overdue_factor: float = 1.25
    #: No source status for this long: the source is not alive.
    source_stale_s: float = 3.0
    #: The standing car must still see its reference at the start.
    ready_similarity_min: float = 0.90
    #: A crossing newer than the last odometry sample by more than this is doubtful.
    odom_extrapolation_s: float = 0.2

    def __post_init__(self) -> None:
        for name in (
            "max_event_latency_s",
            "future_tol_s",
            "min_lap_length_m",
            "heading_slack_rad",
            "band_abs_m",
            "band_rel",
            "overdue_factor",
            "source_stale_s",
            "ready_similarity_min",
            "odom_extrapolation_s",
        ):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f"ExternalConfig.{name} must be finite")
        if self.length_median_laps < 1:
            raise ValueError("length_median_laps must be at least 1")
        if self.overdue_factor <= 1.0:
            raise ValueError("overdue_factor must exceed 1")


@dataclass
class _Sample:
    t: float
    total_m: float
    heading: float  # integrated, unwrapped


class ExternalLapGate:
    """See the module docstring. `odom` is only read for the wheelbase and the yaw gain (the
    odometry model of the start-scan gate)."""

    def __init__(self, cfg: ExternalConfig, odom: GateConfig, source: str = "camera") -> None:
        if source not in EXTERNAL_SOURCES:
            raise ValueError(f"source must be one of {EXTERNAL_SOURCES}, got {source!r}")
        self.cfg = cfg
        self.odom = odom
        self._source_name = source
        self._hist: deque[_Sample] = deque()
        self._steer = 0.0
        self._speed = 0.0
        self._rate = 0.0  # heading rate of the last odometry step (rad/s)
        self._odom_t: float | None = None
        self._now = -math.inf  # latest time seen on any input (odometry, status, event, clock)
        self._started = False
        self._laps = 0
        self._origin_m = 0.0
        self._heading_origin = 0.0
        self._t_lap_start = math.nan
        self._lengths: list[float] = []  # confirmed laps, in order
        self._prev_no = 0  # event number of the last accepted event
        self._uncertain_lap = False
        # the source
        self._status_t = math.nan  # arrival of the last source status
        self._status_state = ""
        self._status_laps = -1
        self._status_sim = math.nan
        self._dead_lap = False  # a gap above source_stale_s during the lap in progress
        self._ready = False
        self._ready_reason = "no source status yet"
        # bookkeeping for the status
        self.confirmed = 0
        self.doubtful: list[dict[str, object]] = []
        self.rejected: dict[str, int] = {k: 0 for k in REJECT_REASONS}
        self.missed_laps: list[int] = []
        self.latency_last = math.nan
        self.latency_max = math.nan
        self.events_seen = 0

    # -- state -----------------------------------------------------------------------------

    @property
    def started(self) -> bool:
        return self._started

    @property
    def laps_completed(self) -> int:
        return self._laps

    @property
    def total_m(self) -> float:
        return self._hist[-1].total_m if self._hist else 0.0

    @property
    def station_m(self) -> float:
        """Wheel distance since the last closure (since the start before the first)."""
        return self.total_m - self._origin_m

    @property
    def heading_since_closure_rad(self) -> float:
        return (self._hist[-1].heading if self._hist else 0.0) - self._heading_origin

    @property
    def l_ref_m(self) -> float:
        """Median of the last `length_median_laps` confirmed laps (NaN before the first)."""
        if not self._lengths:
            return math.nan
        return float(np.median(self._lengths[-self.cfg.length_median_laps :]))

    @property
    def l_ref_trusted(self) -> bool:
        """At least two confirmed laps and the last two within the band of the reference."""
        if len(self._lengths) < 2:
            return False
        return abs(self._lengths[-1] - self._lengths[-2]) <= self.band_m() + EPS

    @property
    def suspect_laps(self) -> list[int]:
        """Always empty: a lap is judged when it closes and never revised."""
        return []

    @property
    def armed(self) -> bool:
        """A closure can come: started and at least `min_lap_length_m` of wheel since the last."""
        return self._started and self.station_m >= self.cfg.min_lap_length_m

    def band_m(self) -> float:
        value = self.l_ref_m
        if not math.isfinite(value):
            return self.cfg.band_abs_m
        return max(self.cfg.band_abs_m, self.cfg.band_rel * value)

    # -- source health ----------------------------------------------------------------------

    def status_age_s(self, now_s: float | None = None) -> float:
        """Seconds since the last source status (inf if none)."""
        now = self._now if now_s is None else now_s
        if not math.isfinite(self._status_t):
            return math.inf
        return max(0.0, now - self._status_t)

    @property
    def source_alive(self) -> bool:
        return self.status_age_s() <= self.cfg.source_stale_s

    def _ready_check(self) -> tuple[bool, str]:
        c = self.cfg
        age = self.status_age_s()
        if not math.isfinite(age):
            return False, "no source status yet"
        if age > c.source_stale_s:
            return False, f"source status {age:.1f} s old"
        if self._status_state != "running":
            return False, f"source state {self._status_state!r}, not 'running'"
        if self._status_laps != 0:
            return False, f"source has counted {self._status_laps} laps already, not 0"
        sim = self._status_sim
        if not math.isfinite(sim) or sim < c.ready_similarity_min - EPS:
            return False, f"similarity {sim:.2f} below {c.ready_similarity_min:.2f}"
        return True, ""

    @property
    def source_ready_now(self) -> bool:
        """What `source_ready` would be if the run started now."""
        return self._ready_check()[0]

    @property
    def source_ready(self) -> bool:
        """Decided once, at the start; before the start it is False."""
        return self._started and self._ready

    @property
    def source_not_ready_reason(self) -> str:
        if self._started:
            return "" if self._ready else self._ready_reason
        return self._ready_check()[1]

    def on_clock(self, t_s: float) -> None:
        """Advance the gate's notion of now (the node's clock tick)."""
        if math.isfinite(t_s) and t_s > self._now:
            self._now = float(t_s)
        self._note_dead()

    def on_source_status(
        self, t_arrival_s: float, state: str, laps: int, similarity: float | None
    ) -> None:
        """A source status (the camera's 1 Hz `/lap/status`). Its content is only used for the
        ready decision and its arrival for liveness: never to place a crossing or count a lap."""
        if not math.isfinite(t_arrival_s):
            return
        if self._started and math.isfinite(self._status_t):
            if t_arrival_s - self._status_t > self.cfg.source_stale_s:
                self._dead_lap = True
        self._now = max(self._now, float(t_arrival_s))
        self._note_dead()
        self._status_t = float(t_arrival_s)
        self._status_state = str(state)
        self._status_laps = int(laps)
        self._status_sim = (
            float(similarity) if similarity is not None and math.isfinite(similarity) else math.nan
        )

    def _note_dead(self) -> None:
        if self._started and self.status_age_s() > self.cfg.source_stale_s:
            self._dead_lap = True

    # -- odometry ---------------------------------------------------------------------------

    def on_steering(self, steer_rad: float) -> None:
        """The gated steering angle (zero-order hold until the next one)."""
        if math.isfinite(steer_rad):
            self._steer = float(steer_rad)

    def on_odom(self, t_s: float, speed_mps: float) -> None:
        """One wheel odometry sample. Forward distance only: a negative speed (recovery) adds
        nothing to the station and nothing to the heading."""
        if not math.isfinite(speed_mps) or not math.isfinite(t_s):
            return
        v = max(0.0, float(speed_mps))
        last = self._hist[-1] if self._hist else None
        if last is None or self._odom_t is None:
            self._hist.append(_Sample(t_s, 0.0, 0.0))
            self._odom_t = t_s
            self._speed = v
            self._now = max(self._now, t_s)
            return
        dt = t_s - last.t
        if dt <= 0.0:
            return
        dt = min(dt, MAX_ODOM_DT_S)
        v_mid = 0.5 * (self._speed + v)
        delta = self._steer
        gain = yaw_gain(self.odom.yaw_gain_table, abs(delta))
        if delta < 0.0:
            gain *= self.odom.yaw_gain_right_scale
        self._rate = gain * v_mid * math.tan(delta) / self.odom.wheelbase_m
        self._hist.append(_Sample(t_s, last.total_m + v_mid * dt, last.heading + self._rate * dt))
        self._speed = v
        self._odom_t = t_s
        self._now = max(self._now, t_s)
        while len(self._hist) > 2 and self._hist[0].t < t_s - HISTORY_S:
            self._hist.popleft()
        self._note_dead()

    def _at(self, t_s: float) -> tuple[_Sample, bool]:
        """(total, heading) at time `t_s` from the history, and whether the odometry covers it:
        inside the history, or newer than the last sample by at most `odom_extrapolation_s`
        (extrapolated at the last speed and rate). A time before the history is clamped."""
        h = self._hist
        if not h:
            return _Sample(t_s, 0.0, 0.0), False
        first, last = h[0], h[-1]
        if t_s < first.t:
            return _Sample(t_s, first.total_m, first.heading), False
        if t_s >= last.t:
            dt = t_s - last.t
            covered = dt <= self.cfg.odom_extrapolation_s + EPS
            return (
                _Sample(t_s, last.total_m + self._speed * dt, last.heading + self._rate * dt),
                covered,
            )
        lo, hi = 0, len(h) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if h[mid].t <= t_s:
                lo = mid
            else:
                hi = mid
        a, b = h[lo], h[hi]
        w = (t_s - a.t) / (b.t - a.t) if b.t > a.t else 0.0
        return _Sample(
            t_s, a.total_m + w * (b.total_m - a.total_m), a.heading + w * (b.heading - a.heading)
        ), True

    # -- the start --------------------------------------------------------------------------

    def set_start(self, t_s: float) -> bool:
        """Lap 1 starts at the odometry sample at `t_s` (the first at or after the planner's first
        DRIVING report). The ready decision is taken here, once. False when there is no odometry
        yet or the gate already started."""
        if self._started or not self._hist or not math.isfinite(t_s):
            return False
        self._now = max(self._now, t_s)
        self._ready, self._ready_reason = self._ready_check()
        last = self._hist[-1]
        self._origin_m = last.total_m
        self._heading_origin = last.heading
        self._t_lap_start = last.t
        self._started = True
        self._dead_lap = False
        return True

    # -- the event --------------------------------------------------------------------------

    def note_malformed(self) -> None:
        """A source message that did not parse or lacked a field (counted by the node)."""
        self.rejected["malformed"] += 1

    def _reject(self, reason: str) -> None:
        self.rejected[reason] += 1

    def on_event(self, t_arrival_s: float, lap_no: int, t_cross_s: float) -> GateCrossing | None:
        """One source event: its arrival on this node's clock, its number and its crossing time
        (the peak frame). Returns the lap it closed (`missed` set when doubtful), or None when
        the event was rejected."""
        c = self.cfg
        finite = math.isfinite(t_arrival_s) and math.isfinite(t_cross_s)
        if finite:
            self.events_seen += 1
            latency = t_arrival_s - t_cross_s
            self.latency_last = latency
            self.latency_max = (
                latency if math.isnan(self.latency_max) else max(self.latency_max, latency)
            )
            self._now = max(self._now, t_arrival_s)
        if not self._started:
            self._reject("before_start")
            return None
        if not finite:
            self._reject("not_finite")
            return None
        if t_cross_s > t_arrival_s + c.future_tol_s + EPS:
            self._reject("future")
            return None
        if t_arrival_s - t_cross_s > c.max_event_latency_s + EPS:
            self._reject("latency")
            return None
        if t_cross_s <= self._t_lap_start:
            self._reject("not_monotone")
            return None
        if lap_no == self._prev_no and self._laps > 0:
            self._reject("duplicate")
            return None
        # the source is alive if it spoke within source_stale_s of this event
        if self.status_age_s(t_arrival_s) > c.source_stale_s:
            self._dead_lap = True
        return self._close(t_cross_s, lap_no)

    def _close(self, t_cross_s: float, lap_no: int) -> GateCrossing:
        c = self.cfg
        at, covered = self._at(t_cross_s)
        length = at.total_m - self._origin_m
        heading = at.heading - self._heading_origin
        reasons: list[str] = []
        if not self._ready:
            reasons.append("ready")
        if lap_no != self._prev_no + 1:
            reasons.append("sequence")
        if not covered:
            reasons.append("odometry")
        if self._dead_lap:
            reasons.append("alive")
        ref = self.l_ref_m
        too_short = length < c.min_lap_length_m - EPS
        off_band = math.isfinite(ref) and abs(length - ref) > self.band_m() + EPS
        if too_short or off_band or not math.isfinite(length):
            reasons.append("length")
        if not (abs(abs(heading) - TWO_PI) <= c.heading_slack_rad + EPS):
            reasons.append("heading")
        doubtful = bool(reasons)
        self._laps += 1
        crossing = GateCrossing(
            lap_index=self._laps,
            t_cross_s=float(t_cross_s),
            lap_time_s=float(t_cross_s - self._t_lap_start),
            length_m=float(length),
            missed=doubtful,
            lateral_m=0.0,
            yaw_rad=0.0,
            heading_rad=float(heading),
            n_candidates=0,
            spread_m=0.0,
            line_source=self._source_name,
        )
        if doubtful:
            self.missed_laps.append(self._laps)
            self.doubtful.append({"lap": self._laps, "event_lap": int(lap_no), "reasons": reasons})
        else:
            self.confirmed += 1
            self._lengths.append(float(length))
        self._prev_no = int(lap_no)
        self._origin_m = at.total_m
        self._heading_origin = at.heading
        self._t_lap_start = float(t_cross_s)
        self._uncertain_lap = False
        self._dead_lap = False
        return crossing

    # -- uncertainty ------------------------------------------------------------------------

    def _uncertain_after_m(self) -> float:
        ref = self.l_ref_m
        if not math.isfinite(ref):
            return math.inf  # never before a reference exists
        if len(self._lengths) >= 2:
            return ref + self.band_m()
        return self.cfg.overdue_factor * ref

    @property
    def station_uncertain(self) -> bool:
        """Overdue (the station is past the reference length plus its band, or past
        `overdue_factor` reference lengths with one confirmed lap) or the source is not alive."""
        if not self._started:
            return False
        if self.status_age_s() > self.cfg.source_stale_s:
            return True
        return self.station_m > self._uncertain_after_m()

    def poll_uncertain(self) -> bool:
        """Latch the uncertain condition (the engine calls it on every odometry sample and when it
        hands out a cap) and return it. The latch holds until the next crossing: a status that
        returns does not clear it."""
        if self.station_uncertain:
            self._uncertain_lap = True
        return self._uncertain_lap

    # -- status -----------------------------------------------------------------------------

    def status(self) -> dict[str, object]:
        return {
            "gate_armed": self.armed,
            "gate_state": "external",
            "station_m": self.station_m,
            "heading_since_closure_rad": self.heading_since_closure_rad,
            "l_ref_m": self.l_ref_m,
            "l_ref_trusted": self.l_ref_trusted,
            "suspect_laps": [],
            "station_uncertain": self.poll_uncertain(),
            "missed_gate_laps": list(self.missed_laps),
            "line_source": self._source_name,
            "source_ready": self.source_ready,
            "source_ready_now": self.source_ready_now,
            "source_not_ready_reason": self.source_not_ready_reason,
            "source_alive": self.source_alive,
            "source_status_age_s": self.status_age_s(),
            "confirmed_laps": self.confirmed,
            "doubtful_laps": [dict(d) for d in self.doubtful],
            "rejected_events": dict(self.rejected),
            "event_latency_s_last": self.latency_last,
            "event_latency_s_max": self.latency_max,
        }
