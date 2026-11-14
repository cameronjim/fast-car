"""The lap memory decision logic with no ROS in it: the node feeds it messages, scans and (for
the map-pose source) TF poses and publishes what it returns. Numpy only.

Two lap sources, one bookkeeping:

* `start_scan` (the default on the car): the planner first reports DRIVING -> the next scan is
  stored as the start scan (`gate.py`) -> wheel odometry and every scan go to the gate -> a
  crossing closes the lap. Station is wheel distance since the last closure. No map is needed
  after lap 1; with `mapping:=true` the pose detector also runs on lap 1 as a cross-check
  (`agree` or `disagree` in the status, the start-scan result always wins).
* `map_pose`: the pre-2026-11-13 behaviour, kept for sim comparisons: the start line is a
  segment through the first map pose, the detector (`lap_detector.py`) closes laps from poses.

A closed lap is built into a record, judged clean or not, clean laps go into the safe set
(`map_pose`) or the station set (`start_scan`), the cap schedule moves (only here, so never mid
lap) and the next lap starts.

What a clean lap is (all sources): no obstacle-gate brake (`/safety/events` ENGAGE at BRAKE
severity from the sources `ttc` or `ttc_reverse`), the planner stayed in DRIVING, the minimum plan
clearance was at least `min_clearance_clean_m`, and the gate was not missed. `watchdog` and
`rate_limit` engagements are TIMING events: they count in the lap's `safety_brakes` (every
BRAKE-severity engagement) and in the status, but do not by themselves make a lap unclean.

D v0 is the cap schedule with empty guidance arrays. D v1 (`lap_mode: v1`, start-scan source only)
adds the station set: the guidance carries a per-station speed bound and cost to go learned from
the car's own clean laps, plus caution zones before every brake, BLOCKED or RECOVERING state and
low-clearance event. Nothing here talks to the car.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from racer_laps.cap_schedule import CapSchedule
from racer_laps.gate import GateConfig, GateCrossing, StartScanGate
from racer_laps.lap_detector import LapDetector
from racer_laps.lap_record import STATE_DRIVING, LapRecord, LapRecorder
from racer_laps.safe_set import SafeSet
from racer_laps.station_set import StationSet

#: racer_msgs/SafetyEvent constants.
SEVERITY_BRAKE = 2
PHASE_ENGAGE = 0
#: SafetyEvent.source strings of the obstacle gates (racer_safety gate_logic_formatting.cpp).
OBSTACLE_SOURCES = frozenset({"ttc", "ttc_reverse"})
#: PlannerStatus states that put a caution zone before them.
STATE_BLOCKED = 5
STATE_RECOVERING = 7
CAUTION_STATES = frozenset({STATE_BLOCKED, STATE_RECOVERING})

LAP_SOURCES = ("start_scan", "map_pose")
LAP_MODES = ("v0", "v1")
GUIDANCE_SPACING_M = 0.10
#: A sample of the lap is kept at most this often (the odometry runs at 50 Hz).
SAMPLE_PERIOD_S = 0.04
RECENT_SAMPLES = 400
PLANNER_MAX_ARRAY = 128


@dataclass(frozen=True)
class LapConfig:
    v_cap_start_mps: float = 0.5
    v_cap_step_mps: float = 0.1
    v_cap_ceiling_mps: float = 1.0
    min_clearance_clean_m: float = 0.08
    min_lap_length_m: float = 3.0
    start_line_length_m: float = 1.5
    start_heading_tolerance_rad: float = math.radians(60.0)
    guidance_horizon_m: float = 3.0
    lap_source: str = "map_pose"
    lap_mode: str = "v0"
    explore_dv_mps: float = 0.2
    caution_before_m: float = 1.0
    caution_laps: int = 2
    min_filter_half_m: float = 0.4
    #: Gate settings of the start-scan source (required for it).
    gate: GateConfig | None = None
    #: Lap-1 cross-check: both sources closing within this many seconds is `agree`.
    crosscheck_window_s: float = 0.5

    def __post_init__(self) -> None:
        if self.lap_source not in LAP_SOURCES:
            raise ValueError(f"lap_source must be one of {LAP_SOURCES}, got {self.lap_source!r}")
        if self.lap_mode not in LAP_MODES:
            raise ValueError(f"lap_mode must be one of {LAP_MODES}, got {self.lap_mode!r}")
        if self.lap_source == "start_scan" and self.gate is None:
            raise ValueError(
                "lap_source start_scan needs a GateConfig (wheelbase from vehicle_params)"
            )
        if self.lap_mode == "v1":
            if self.lap_source != "start_scan":
                raise ValueError(
                    "lap_mode v1 needs lap_source start_scan (it is indexed by station)"
                )
            n = round(self.guidance_horizon_m / GUIDANCE_SPACING_M) + 1
            if not 2 <= n <= PLANNER_MAX_ARRAY:
                raise ValueError(
                    f"guidance_horizon_m {self.guidance_horizon_m} gives {n} stations; "
                    f"the planner accepts 2 to {PLANNER_MAX_ARRAY}"
                )


@dataclass(frozen=True)
class LapResult:
    """What the node publishes as a LapEvent, plus whether the lap entered the safe set."""

    lap_index: int
    t_cross_s: float
    lap_time_s: float
    length_m: float
    cap_mps: float  # the cap this lap was driven under
    safety_brakes: int  # every BRAKE-severity engagement
    min_clearance_m: float
    clean: bool
    stored: bool
    next_cap_mps: float
    source: str = "map_pose"
    missed_gate: bool = False
    obstacle_brakes: int = -1
    timing_brakes: int = 0
    heading_rad: float = math.nan  # start-scan source: heading integrated over the lap


@dataclass(frozen=True)
class Guidance:
    lap_index: int  # the lap being driven (1 for the first), 0 before the start line exists
    v_cap_mps: float
    s_m: NDArray[np.float64] = field(default_factory=lambda: np.zeros(0))
    v_ref_mps: NDArray[np.float64] = field(default_factory=lambda: np.zeros(0))
    cost_to_go_s: NDArray[np.float64] = field(default_factory=lambda: np.zeros(0))


class LapMemory:
    def __init__(self, cfg: LapConfig) -> None:
        self.cfg = cfg
        self.schedule = CapSchedule(cfg.v_cap_start_mps, cfg.v_cap_step_mps, cfg.v_cap_ceiling_mps)
        self.detector = LapDetector(
            cfg.min_lap_length_m, cfg.start_line_length_m, cfg.start_heading_tolerance_rad
        )
        self.safe_set = SafeSet()
        self.station_set = StationSet(
            min_filter_half_m=cfg.min_filter_half_m,
            caution_before_m=cfg.caution_before_m,
            caution_laps=cfg.caution_laps,
        )
        self.gate: StartScanGate | None = (
            StartScanGate(cfg.gate) if cfg.lap_source == "start_scan" and cfg.gate else None
        )
        self.records: list[LapRecord] = []
        self._recorder = LapRecorder(cfg.min_clearance_clean_m)
        self._driving_seen = False
        self._pose_ok = False
        self._last_pose_t = -math.inf
        self._wheel_speed = math.nan
        self._cmd_speed = math.nan
        self._cmd_steer = math.nan
        self._state = -1
        self._clearance = math.nan
        self._cap_this_lap = self.schedule.cap_mps
        self._lap_t0 = math.nan
        self._last_sample_t = -math.inf
        self._recent: deque[tuple[float, float]] = deque(maxlen=RECENT_SAMPLES)  # station, speed
        self._low_clearance = False
        self._pending_zones: list[tuple[float, float, str]] = []
        self._timing_events = 0
        self._zone_count = 0
        self._gate_t1 = math.nan  # lap-1 closure time of the start-scan gate
        self._pose_t1 = math.nan  # lap-1 closure time of the pose detector
        self._pose_feeding = True

    # -- inputs ----------------------------------------------------------------------------

    @property
    def _started(self) -> bool:
        if self.gate is not None:
            return self.gate.started
        return self.detector.started

    def on_planner_status(self, state: int, clearance_m: float) -> None:
        prev_state = self._state
        self._state = int(state)
        self._clearance = float(clearance_m)
        if state == STATE_DRIVING:
            self._driving_seen = True
        if self._started:
            self._recorder.note_planner(int(state), float(clearance_m))
            if self.gate is not None:
                if int(state) in CAUTION_STATES and prev_state not in CAUTION_STATES:
                    self._note_zone("blocked" if state == STATE_BLOCKED else "recovering")
                low = math.isfinite(clearance_m) and clearance_m < self.cfg.min_clearance_clean_m
                if low and not self._low_clearance:
                    self._note_zone("clearance")
                self._low_clearance = low

    def on_safety_event(self, phase: int, severity: int, source: str = "ttc") -> None:
        """A /safety/events record. `source` defaults to an obstacle gate so a caller that does
        not know it is conservative."""
        if not (self._started and phase == PHASE_ENGAGE and severity == SEVERITY_BRAKE):
            return
        obstacle = source in OBSTACLE_SOURCES
        self._recorder.note_brake(obstacle)
        if obstacle:
            self._note_zone("brake")
        else:
            self._timing_events += 1

    def on_wheel_speed(self, v_mps: float) -> None:
        self._wheel_speed = float(v_mps)

    def on_wheel_odom(self, t_s: float, v_mps: float) -> None:
        """Wheel odometry with its stamp (start-scan source): station, heading and lap samples."""
        self._wheel_speed = float(v_mps)
        gate = self.gate
        if gate is None:
            return
        gate.on_odom(t_s, v_mps)
        if not gate.started:
            return
        gate.poll_uncertain()
        station = gate.station_m
        self._recent.append((station, max(0.0, float(v_mps))))
        if t_s - self._last_sample_t >= SAMPLE_PERIOD_S:
            self._last_sample_t = t_s
            self._recorder.add_sample(
                t_s,
                math.nan,
                math.nan,
                math.nan,
                self._wheel_speed,
                self._cmd_speed,
                self._cmd_steer,
                self._state,
                self._clearance,
                station,
            )

    def on_drive(self, steering_rad: float) -> None:
        """The gated steering angle from /drive (the heading integral of the gate)."""
        if self.gate is not None:
            self.gate.on_steering(steering_rad)

    def on_drive_raw(self, speed_mps: float, steering_rad: float) -> None:
        self._cmd_speed = float(speed_mps)
        self._cmd_steer = float(steering_rad)

    def on_pose_missing(self) -> None:
        """TF is missing or stale: hold state, never invent a lap."""
        self._pose_ok = False

    # -- the start-scan source -------------------------------------------------------------

    def on_scan(self, t_eff_s: float, points: NDArray[np.float64]) -> LapResult | None:
        """One scan as base_link points (body filtered) at its effective time (header stamp plus
        half the sweep). Returns the lap it closed, if any."""
        gate = self.gate
        if gate is None:
            return None
        if not gate.started:
            if not self._driving_seen:
                return None
            if gate.set_start(t_eff_s, points):
                self._begin_lap(t_eff_s)
            return None
        crossing = gate.on_scan(t_eff_s, points)
        if crossing is None:
            return None
        return self._close_station_lap(crossing)

    def _begin_lap(self, t_s: float) -> None:
        self._recorder = LapRecorder(self.cfg.min_clearance_clean_m)
        self._cap_this_lap = self._driven_cap()
        self._lap_t0 = t_s
        self._recent.clear()
        # The planner may already have left DRIVING between the report and this scan.
        self._recorder.note_planner(self._state, self._clearance)

    def _note_zone(self, reason: str) -> None:
        """A caution zone before the car's current station, speed from the last metre."""
        gate = self.gate
        if gate is None or not gate.started:
            return
        s = gate.station_m
        lo = s - self.cfg.caution_before_m
        speeds = [v for st, v in self._recent if lo <= st <= s]
        speed = float(np.mean(speeds)) if speeds else max(0.0, self._wheel_speed)
        if not math.isfinite(speed):
            return
        self._zone_count += 1
        if self.station_set.add_zone(s, speed, self.lap_in_progress, reason) is None:
            self._pending_zones.append((s, speed, reason))

    def _close_station_lap(self, crossing: GateCrossing) -> LapResult:
        assert self.gate is not None
        record = self._recorder.finish(
            crossing.lap_index,
            crossing.lap_time_s,
            crossing.length_m,
            self._cap_this_lap,
            missed_gate=crossing.missed,
        )
        # Samples are stamped in the odometry clock; make them lap relative for the station set.
        t_rel = record.t_s - self._lap_t0
        self.records.append(record)
        if crossing.lap_index == 1 and not crossing.missed:
            self._gate_t1 = crossing.t_cross_s
            if math.isfinite(self.gate.l_ref_m):
                self.station_set.set_reference_length(self.gate.l_ref_m)
            for s, v, why in self._pending_zones:
                self.station_set.add_zone(s, v, 1, why)
            self._pending_zones = []
        stored = False
        if record.clean and math.isfinite(self.station_set.l_ref_m):
            stored = self.station_set.add_lap(
                crossing.lap_index,
                record.station_m,
                t_rel,
                record.wheel_speed_mps,
                crossing.lap_time_s,
                crossing.length_m,
                self._cap_this_lap,
            )
        cap = self._cap_this_lap
        next_cap = self.schedule.finish_lap(record.clean)
        self.station_set.expire_zones(crossing.lap_index + 1)
        self._lap_t0 = crossing.t_cross_s
        self._recent.clear()
        self._cap_this_lap = self._driven_cap()
        # The new lap starts with the state the planner is in now.
        self._recorder.note_planner(self._state, self._clearance)
        self._low_clearance = False
        if self.cfg.lap_mode == "v1":
            next_cap = self._cap_this_lap
        return LapResult(
            lap_index=crossing.lap_index,
            t_cross_s=crossing.t_cross_s,
            lap_time_s=crossing.lap_time_s,
            length_m=crossing.length_m,
            cap_mps=cap,
            safety_brakes=record.brake_count,
            min_clearance_m=record.min_clearance_m,
            clean=record.clean,
            stored=stored,
            next_cap_mps=next_cap,
            source="start_scan",
            missed_gate=crossing.missed,
            obstacle_brakes=record.obstacle_brake_count,
            timing_brakes=record.brake_count - record.obstacle_brake_count,
            heading_rad=crossing.heading_rad,
        )

    def _driven_cap(self) -> float:
        """The cap handed out now: the lap-1 cap until a clean lap is stored, then (D v1) the
        highest speed bound of the lap capped at the ceiling; D v0 follows the schedule."""
        if self.cfg.lap_mode == "v1" and self.gate is not None:
            if self.station_set.n_laps >= 1 and not self._uncertain():
                top = self.station_set.max_bound(self.lap_in_progress, self.cfg.explore_dv_mps)
                if math.isfinite(top):
                    return float(min(self.cfg.v_cap_ceiling_mps, max(top, 0.0)))
            return self.cfg.v_cap_start_mps
        if self._uncertain():
            return self.cfg.v_cap_start_mps
        return self.schedule.cap_mps

    def _uncertain(self) -> bool:
        return self.gate is not None and self.gate.poll_uncertain()

    # -- the map-pose source ---------------------------------------------------------------

    def on_pose(self, t_s: float, x: float, y: float, yaw: float) -> LapResult | None:
        """One pose sample in `map`. With `map_pose` returns the lap it completed, if any. With
        `start_scan` it only feeds the lap-1 cross-check and returns None."""
        self._pose_ok = True
        if t_s <= self._last_pose_t:
            return None
        self._last_pose_t = t_s
        if self.gate is not None:
            self._crosscheck_pose(t_s, x, y, yaw)
            return None
        result: LapResult | None = None
        if not self.detector.started:
            if not self._driving_seen:
                return None
            self.detector.set_start(t_s, x, y, yaw)
            self._recorder = LapRecorder(self.cfg.min_clearance_clean_m)
            self._cap_this_lap = self.schedule.cap_mps
            # The planner may already have left DRIVING between the report and this pose.
            self._recorder.note_planner(self._state, self._clearance)
        else:
            crossing = self.detector.update(t_s, x, y, yaw)
            if crossing is not None:
                result = self._close_lap(
                    crossing.lap_index, crossing.t_cross_s, crossing.lap_time_s, crossing.length_m
                )
        self._recorder.add_sample(
            t_s,
            x,
            y,
            yaw,
            self._wheel_speed,
            self._cmd_speed,
            self._cmd_steer,
            self._state,
            self._clearance,
        )
        return result

    def _crosscheck_pose(self, t_s: float, x: float, y: float, yaw: float) -> None:
        """Lap 1 only: run the pose detector next to the gate, for the agree/disagree flag."""
        assert self.gate is not None
        if not self._pose_feeding:
            return
        if not self.gate.started:
            return
        if not self.detector.started:
            self.detector.set_start(t_s, x, y, yaw)
            return
        crossing = self.detector.update(t_s, x, y, yaw)
        if crossing is not None and crossing.lap_index == 1:
            self._pose_t1 = crossing.t_cross_s
        if math.isfinite(self._pose_t1) or (
            math.isfinite(self._gate_t1) and t_s > self._gate_t1 + self.cfg.crosscheck_window_s
        ):
            self._pose_feeding = False  # after lap 1 the pose is only logged

    @property
    def crosscheck(self) -> str:
        """`no_pose`, `pending`, `agree` or `disagree` for the lap-1 closure (start scan versus
        map pose)."""
        if self.gate is None:
            return "off"
        if not self.detector.started and not math.isfinite(self._pose_t1):
            return "no_pose"
        if math.isfinite(self._gate_t1) and math.isfinite(self._pose_t1):
            ok = abs(self._gate_t1 - self._pose_t1) <= self.cfg.crosscheck_window_s
            return "agree" if ok else "disagree"
        if math.isfinite(self._gate_t1) and not self._pose_feeding:
            return "disagree"
        return "pending"

    def _close_lap(
        self, lap_index: int, t_cross_s: float, lap_time_s: float, length_m: float
    ) -> LapResult:
        cap = self._cap_this_lap
        record = self._recorder.finish(lap_index, lap_time_s, length_m, cap)
        self.records.append(record)
        stored = self.safe_set.add_lap(record)
        next_cap = self.schedule.finish_lap(record.clean)
        self._cap_this_lap = next_cap
        # The new lap starts with the state the planner is in now.
        self._recorder.note_planner(self._state, self._clearance)
        return LapResult(
            lap_index=lap_index,
            t_cross_s=t_cross_s,
            lap_time_s=lap_time_s,
            length_m=length_m,
            cap_mps=cap,
            safety_brakes=record.brake_count,
            min_clearance_m=record.min_clearance_m,
            clean=record.clean,
            stored=stored,
            next_cap_mps=next_cap,
            source="map_pose",
            obstacle_brakes=record.obstacle_brake_count,
            timing_brakes=record.brake_count - record.obstacle_brake_count,
        )

    # -- outputs ---------------------------------------------------------------------------

    @property
    def laps_completed(self) -> int:
        if self.gate is not None:
            return self.gate.laps_completed
        return self.detector.laps_completed

    @property
    def lap_in_progress(self) -> int:
        """Index of the lap being driven: 0 before the start line, then 1, 2, ..."""
        return self.laps_completed + 1 if self._started else 0

    @property
    def station_m(self) -> float:
        """Wheel distance since the last closure (NaN for the map-pose source)."""
        return self.gate.station_m if self.gate is not None and self.gate.started else math.nan

    def guidance(self) -> Guidance:
        empty = Guidance(self.lap_in_progress, self._cap_for_guidance())
        if self.gate is not None:
            if (
                self.cfg.lap_mode != "v1"
                or not self.gate.started
                or self.station_set.n_laps < 1
                or self._uncertain()
            ):
                return empty
            s, v, q = self.station_set.arrays(
                self.gate.station_m,
                self.cfg.guidance_horizon_m,
                self.lap_in_progress,
                self.cfg.explore_dv_mps,
                GUIDANCE_SPACING_M,
            )
            if s.shape[0] < 2:
                return empty
            return Guidance(self.lap_in_progress, self._cap_for_guidance(), s, v, q)
        cap = self._cap_this_lap
        if not self.detector.started or self.detector.laps_completed < 1:
            return Guidance(self.lap_in_progress, cap)
        s, v, c = self.safe_set.horizon(
            self.detector.path_since_crossing_m, self.cfg.guidance_horizon_m
        )
        return Guidance(self.lap_in_progress, cap, s, v, c)

    def _cap_for_guidance(self) -> float:
        if self.gate is None:
            return self._cap_this_lap
        # Recomputed live: zones appear mid lap and a missed gate drops it to the lap-1 cap.
        return self._driven_cap()

    def status(self) -> dict[str, object]:
        if not self._started:
            if not self._driving_seen:
                phase = "waiting_for_driving"
            else:
                phase = "waiting_for_scan" if self.gate is not None else "waiting_for_pose"
        else:
            phase = "running"
        out: dict[str, object] = {
            "phase": phase,
            "lap_source": self.cfg.lap_source,
            "lap_mode": self.cfg.lap_mode,
            "pose_ok": self._pose_ok,
            "lap_in_progress": self.lap_in_progress,
            "laps_completed": self.laps_completed,
            "v_cap_mps": self._cap_for_guidance(),
            "safe_set_laps": self.station_set.n_laps if self.gate else self.safe_set.n_laps,
            "best_lap_time_s": (
                self.station_set.best_lap_time_s if self.gate else self.safe_set.best_lap_time_s
            ),
            "path_since_crossing_m": self.detector.path_since_crossing_m,
        }
        if self.gate is not None:
            out.update(self.gate.status())
            out["lap_source_check"] = self.crosscheck
            out["timing_events"] = self._timing_events
            out["caution_zones"] = len(self.station_set.zones)
            out["zones_created"] = self._zone_count
            out["why_no_lap"] = self._why_no_lap()
        return out

    def _why_no_lap(self) -> str:
        """One word for the status: why no lap has closed yet (empty once one has)."""
        gate = self.gate
        if gate is None or gate.laps_completed >= 1:
            return ""
        if not self._driving_seen:
            return "waiting_for_driving"
        if not gate.started:
            return "waiting_for_scan"
        if not gate.armed:
            return "gate_not_armed"
        return "armed_no_crossing_yet"
