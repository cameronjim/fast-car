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

import json
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
DOUBTFUL_REASONS = (
    "ready",
    "start_unanswered",
    "sequence",
    "odometry",
    "alive",
    "length",
    "heading",
)


#: Owner decision 24: a doubtful lap whose reasons all lie in this set is a camera fact, not a
#: driving fact; with a clean drive it holds the speed cap (see `CapSchedule.finish_lap`).
CAMERA_ONLY_REASONS = frozenset({"length", "heading", "sequence"})


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
    #: Owner decision 21 (2026-11-22, not registered): the length check as a ratio of the
    #: reference, from `length_ratio_min` to `length_ratio_max` times it, in place of the band.
    #: A faster lap takes a shorter line (car 2026-11-22: 5.3 m at 0.9 m/s, 6.2 m at 0.6 m/s),
    #: which the band rejected; a missed crossing (a double lap, 2x) is still outside 1.5x.
    #: Both 0.0 (the default): the registered band, unchanged.
    length_ratio_min: float = 0.0
    length_ratio_max: float = 0.0

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
            "length_ratio_min",
            "length_ratio_max",
        ):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f"ExternalConfig.{name} must be finite")
        if self.length_median_laps < 1:
            raise ValueError("length_median_laps must be at least 1")
        if self.overdue_factor <= 1.0:
            raise ValueError("overdue_factor must exceed 1")
        if (self.length_ratio_min, self.length_ratio_max) != (0.0, 0.0) and not (
            0.0 < self.length_ratio_min < 1.0 < self.length_ratio_max < 2.0
        ):
            raise ValueError(
                "length_ratio_min and length_ratio_max: both 0 (the band), or "
                "0 < min < 1 < max < 2 (a double lap must stay outside)"
            )

    @property
    def length_ratio(self) -> bool:
        """The length check is the ratio (owner decision 21), not the registered band."""
        return self.length_ratio_max > 0.0


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
        self.start_unanswered = False
        # outside review of brief 51c: what the source says of its own camera (a counter that
        # republishes `running` on a timer says nothing of whether images arrive). Keys of the last
        # status only; none for a source that does not say (an older counter, a bag).
        self._cam: dict[str, object] = {}
        self._seq_prev: int | None = None
        # bookkeeping for the status
        self.confirmed = 0
        self.doubtful: list[dict[str, object]] = []
        self.rejected: dict[str, int] = {k: 0 for k in REJECT_REASONS}
        self.missed_laps: list[int] = []
        self.latency_last = math.nan
        self.latency_max = math.nan
        # brief 51b: a diagnostic only, it feeds no verdict, reason, cap or reference
        self.reference_suspect = False
        self.reference_suspect_lap: int | None = None
        self.events_seen = 0
        # brief 51c: three status keys that read state and feed no verdict, reason, cap or
        # reference (a test pins that): where the first source status stood against the start
        self._statuses_before_start = 0
        self._status_age_at_start = math.nan
        self._status_gap_max = math.nan

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
    def camera_stale(self) -> bool:
        """The source itself says its camera frames are stale (never true for a source that does
        not say)."""
        return self._cam.get("camera_fresh") is False

    @property
    def source_alive(self) -> bool:
        return self.status_age_s() <= self.cfg.source_stale_s and not self.camera_stale

    def _camera_check(self) -> tuple[bool, str]:
        """The source's own freshness: frames recent, wheel odometry recent, and the frame sequence
        advanced since the status before. Nothing is asked of a status that carries none of it."""
        cam = self._cam
        if cam.get("camera_fresh") is False:
            age = cam.get("frame_age_s")
            when = f" (last frame {age:.1f} s ago)" if isinstance(age, float) else ""
            return False, f"camera frames stale{when}"
        if cam.get("odom_fresh") is False:
            return False, "the lap counter's wheel odometry is stale"
        seq = cam.get("frame_seq")
        if isinstance(seq, int):
            if self._seq_prev is None:
                return False, "no camera frame progress seen yet (one status)"
            if seq <= self._seq_prev:
                return False, f"camera frame count not advancing ({seq} after {self._seq_prev})"
        return True, ""

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
        return self._camera_check()

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
        self,
        t_arrival_s: float,
        state: str,
        laps: int,
        similarity: float | None,
        camera: dict[str, object] | None = None,
    ) -> None:
        """A source status (the camera's 1 Hz `/lap/status`). Its content is only used for the
        ready decision and its arrival for liveness: never to place a crossing or count a lap."""
        if not math.isfinite(t_arrival_s):
            return
        self._trace_status(float(t_arrival_s))
        if (
            self._started
            and math.isfinite(self._status_t)
            and t_arrival_s - self._status_t > self.cfg.source_stale_s
        ):
            self._dead_lap = True
        self._now = max(self._now, float(t_arrival_s))
        previous = self._cam.get("frame_seq")
        self._seq_prev = previous if isinstance(previous, int) else None
        self._cam = dict(camera or {})
        self._note_dead()
        self._status_t = float(t_arrival_s)
        self._status_state = str(state)
        self._status_laps = int(laps)
        self._status_sim = (
            float(similarity) if similarity is not None and math.isfinite(similarity) else math.nan
        )

    def _note_dead(self) -> None:
        if self._started and (self.status_age_s() > self.cfg.source_stale_s or self.camera_stale):
            self._dead_lap = True

    def _trace_status(self, t_arrival_s: float) -> None:
        """Brief 51c bookkeeping, called before a source status is taken in: the count of statuses
        before the start and the largest gap between two statuses since the start. Reads state,
        writes only its own three fields."""
        if not self._started:
            self._statuses_before_start += 1
            return
        if math.isfinite(self._status_t):
            gap = t_arrival_s - self._status_t
            if math.isnan(self._status_gap_max) or gap > self._status_gap_max:
                self._status_gap_max = gap

    def _trace_start(self) -> None:
        """Brief 51c bookkeeping, called at the start before the ready decision: the age of the
        last source status at that instant (infinite if none came)."""
        self._status_age_at_start = self.status_age_s()

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
        self._trace_start()
        self._ready, self._ready_reason = self._ready_check()
        last = self._hist[-1]
        self._origin_m = last.total_m
        self._heading_origin = last.heading
        self._t_lap_start = last.t
        self._started = True
        self._dead_lap = False
        self.reference_suspect = False
        self.reference_suspect_lap = None
        return True

    # -- the event --------------------------------------------------------------------------

    def note_start_unanswered(self) -> None:
        """The car did not move after the start while commands were sent (the engine decides).
        Latched for the run: every lap of it is doubtful, as a start that was not ready."""
        self.start_unanswered = True

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
        if self.status_age_s(t_arrival_s) > c.source_stale_s or self.camera_stale:
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
        if self.start_unanswered:
            reasons.append("start_unanswered")
        if lap_no != self._prev_no + 1:
            reasons.append("sequence")
        if not covered:
            reasons.append("odometry")
        if self._dead_lap:
            reasons.append("alive")
        ref = self.l_ref_m
        too_short = length < c.min_lap_length_m - EPS
        if c.length_ratio:
            off_band = math.isfinite(ref) and not (
                c.length_ratio_min * ref - EPS <= length <= c.length_ratio_max * ref + EPS
            )
        else:
            off_band = math.isfinite(ref) and abs(length - ref) > self.band_m() + EPS
        if too_short or off_band or not math.isfinite(length):
            reasons.append("length")
        if not (abs(abs(heading) - TWO_PI) <= c.heading_slack_rad + EPS):
            reasons.append("heading")
        doubtful = bool(reasons)
        self._laps += 1
        self._note_suspect(reasons, length)
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
            reasons=tuple(reasons),
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

    def _note_suspect(self, reasons: list[str], length: float) -> None:
        """Brief 51b: the reference rests on exactly one confirmed lap and this lap fails only
        `length`: lap 1 was probably not a whole lap. Latched until the next start. A diagnostic:
        it reads the verdict and changes nothing."""
        if self.reference_suspect or len(self._lengths) != 1 or reasons != ["length"]:
            return
        self.reference_suspect = True
        self.reference_suspect_lap = self._laps
        self.suspect_lap_length_m = float(length)

    suspect_lap_length_m = math.nan

    # -- uncertainty ------------------------------------------------------------------------

    def _uncertain_after_m(self) -> float:
        ref = self.l_ref_m
        if not math.isfinite(ref):
            return math.inf  # never before a reference exists
        if len(self._lengths) >= 2:
            if self.cfg.length_ratio:
                return self.cfg.length_ratio_max * ref
            return ref + self.band_m()
        return self.cfg.overdue_factor * ref

    @property
    def station_uncertain(self) -> bool:
        """Overdue (the station is past the reference length plus its band, or past
        `overdue_factor` reference lengths with one confirmed lap) or the source is not alive."""
        if not self._started:
            return False
        if self.status_age_s() > self.cfg.source_stale_s or self.camera_stale:
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
            "start_unanswered": self.start_unanswered,
            "source_ready_now": self.source_ready_now,
            "source_not_ready_reason": self.source_not_ready_reason,
            "source_alive": self.source_alive,
            "source_status_age_s": self.status_age_s(),
            "confirmed_laps": self.confirmed,
            "doubtful_laps": [dict(d) for d in self.doubtful],
            "rejected_events": dict(self.rejected),
            "event_latency_s_last": self.latency_last,
            "event_latency_s_max": self.latency_max,
            "reference_suspect": self.reference_suspect,
            "reference_suspect_lap": self.reference_suspect_lap,
            "source_status_age_at_start_s": self._status_age_at_start,
            "source_statuses_before_start": self._statuses_before_start,
            "source_status_gap_max_s": self._status_gap_max,
            "source_camera_fresh": self._cam.get("camera_fresh"),
            "source_frame_seq": self._cam.get("frame_seq"),
            "source_reset_id": self._cam.get("reset_id"),
            "source_odom_fresh": self._cam.get("odom_fresh"),
            "source_frame_age_s": self._cam.get("frame_age_s"),
        }


def _number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def parse_event_json(data: str) -> tuple[int, float] | None:
    """`(lap, crossing_time_s)` of a source event (the camera's `/lap/event` JSON), or None when it
    does not parse or lacks a field. A NaN crossing time parses: the gate rejects it."""
    try:
        d = json.loads(data)
    except (ValueError, TypeError):
        return None
    if not isinstance(d, dict):
        return None
    lap, cross = _number(d.get("lap")), _number(d.get("crossing_time_s"))
    if lap is None or cross is None or not math.isfinite(lap) or lap != int(lap):
        return None
    return int(lap), cross


def _count(value: object) -> int | None:
    number = _number(value)
    if number is None or not math.isfinite(number) or number != int(number):
        return None
    return int(number)


def parse_status_camera(data: str) -> dict[str, object] | None:
    """The freshness keys of a camera status (outside review of brief 51c): `camera_fresh` and
    `odom_fresh` (booleans), `frame_seq`, `reset_id` and `frames_since_reset` (counts),
    `frame_age_s` (a finite age). Keys that are missing or of the wrong type are left out, so a
    status from an older counter gives `{}` and is read as before. None when it does not parse."""
    try:
        d = json.loads(data)
    except (ValueError, TypeError):
        return None
    if not isinstance(d, dict):
        return None
    out: dict[str, object] = {}
    for key in ("camera_fresh", "odom_fresh"):
        if isinstance(d.get(key), bool):
            out[key] = d[key]
    for key in ("frame_seq", "reset_id", "frames_since_reset"):
        count = _count(d.get(key))
        if count is not None:
            out[key] = count
    age = _number(d.get("frame_age_s"))
    if age is not None and math.isfinite(age):
        out["frame_age_s"] = age
    return out


def parse_status_json(data: str) -> tuple[str, int, float | None] | None:
    """`(state, laps, similarity)` of a source status (the camera's `/lap/status` JSON; similarity
    is null while it has no reference), or None when it does not parse or lacks a field."""
    try:
        d = json.loads(data)
    except (ValueError, TypeError):
        return None
    if not isinstance(d, dict) or not {"state", "laps", "similarity"} <= d.keys():
        return None
    laps, sim = _number(d["laps"]), d["similarity"]
    if not isinstance(d["state"], str) or laps is None or not math.isfinite(laps):
        return None
    if laps != int(laps):
        return None
    if sim is None:
        return d["state"], int(laps), None
    similarity = _number(sim)
    if similarity is None:
        return None
    return d["state"], int(laps), similarity
