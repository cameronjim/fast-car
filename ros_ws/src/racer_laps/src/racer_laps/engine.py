"""The lap memory decision logic with no ROS in it: the node feeds it messages and TF poses and
publishes what it returns. Numpy only.

Sequence: the planner first reports DRIVING -> the next valid pose sets the start line and starts
lap 1 -> every pose goes to the lap detector and the recorder -> a crossing closes the lap: the
record is built, clean or not is decided, clean laps go into the safe set, the cap schedule moves
(only here, so never mid lap) and the next lap starts. Nothing here talks to the car.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from racer_laps.cap_schedule import CapSchedule
from racer_laps.lap_detector import LapDetector
from racer_laps.lap_record import STATE_DRIVING, LapRecord, LapRecorder
from racer_laps.safe_set import SafeSet

#: racer_msgs/SafetyEvent constants.
SEVERITY_BRAKE = 2
PHASE_ENGAGE = 0


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


@dataclass(frozen=True)
class LapResult:
    """What the node publishes as a LapEvent, plus whether the lap entered the safe set."""

    lap_index: int
    t_cross_s: float
    lap_time_s: float
    length_m: float
    cap_mps: float  # the cap this lap was driven under
    safety_brakes: int
    min_clearance_m: float
    clean: bool
    stored: bool
    next_cap_mps: float


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

    # -- inputs ----------------------------------------------------------------------------

    def on_planner_status(self, state: int, clearance_m: float) -> None:
        self._state = int(state)
        self._clearance = float(clearance_m)
        if state == STATE_DRIVING:
            self._driving_seen = True
        if self.detector.started:
            self._recorder.note_planner(int(state), float(clearance_m))

    def on_safety_event(self, phase: int, severity: int) -> None:
        if self.detector.started and phase == PHASE_ENGAGE and severity == SEVERITY_BRAKE:
            self._recorder.note_brake()

    def on_wheel_speed(self, v_mps: float) -> None:
        self._wheel_speed = float(v_mps)

    def on_drive_raw(self, speed_mps: float, steering_rad: float) -> None:
        self._cmd_speed = float(speed_mps)
        self._cmd_steer = float(steering_rad)

    def on_pose_missing(self) -> None:
        """TF is missing or stale: hold state, never invent a lap."""
        self._pose_ok = False

    def on_pose(self, t_s: float, x: float, y: float, yaw: float) -> LapResult | None:
        """One pose sample in `map`. Returns the lap it completed, if any."""
        self._pose_ok = True
        if t_s <= self._last_pose_t:
            return None
        self._last_pose_t = t_s
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
        )

    # -- outputs ---------------------------------------------------------------------------

    @property
    def lap_in_progress(self) -> int:
        """Index of the lap being driven: 0 before the start line, then 1, 2, ..."""
        return self.detector.laps_completed + 1 if self.detector.started else 0

    def guidance(self) -> Guidance:
        cap = self._cap_this_lap
        if not self.detector.started or self.detector.laps_completed < 1:
            return Guidance(self.lap_in_progress, cap)
        s, v, c = self.safe_set.horizon(
            self.detector.path_since_crossing_m, self.cfg.guidance_horizon_m
        )
        return Guidance(self.lap_in_progress, cap, s, v, c)

    def status(self) -> dict[str, object]:
        if not self.detector.started:
            phase = "waiting_for_pose" if self._driving_seen else "waiting_for_driving"
        else:
            phase = "running"
        return {
            "phase": phase,
            "pose_ok": self._pose_ok,
            "lap_in_progress": self.lap_in_progress,
            "laps_completed": self.detector.laps_completed,
            "v_cap_mps": self._cap_this_lap,
            "safe_set_laps": self.safe_set.n_laps,
            "best_lap_time_s": self.safe_set.best_lap_time_s,
            "path_since_crossing_m": self.detector.path_since_crossing_m,
        }
