"""Per-lap record and the recorder that builds it. Pure numpy, no ROS.

A lap record holds the samples of one lap: time, pose in `map`, measured wheel speed, commanded
speed and steering, planner state and plan clearance, plus the lap's summary (time, length, cap,
brake count, clean flag). Samples are taken by the caller at the pose rate (20 Hz); the last
known wheel speed, command and planner values are attached to each pose sample.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from racer_laps.cap_schedule import lap_is_clean

FloatArray = NDArray[np.float64]

#: PlannerStatus.STATE_DRIVING (racer_pnp_msgs/msg/PlannerStatus.msg).
STATE_DRIVING = 1


@dataclass(frozen=True)
class LapRecord:
    lap_index: int
    lap_time_s: float
    length_m: float
    cap_mps: float
    brake_count: int
    non_driving_seen: bool
    min_clearance_m: float  # NaN when the planner reported no clearance during the lap
    clean: bool
    t_s: FloatArray
    x_m: FloatArray
    y_m: FloatArray
    yaw_rad: FloatArray
    wheel_speed_mps: FloatArray
    cmd_speed_mps: FloatArray
    cmd_steering_rad: FloatArray
    planner_state: FloatArray
    plan_clearance_m: FloatArray


class LapRecorder:
    """Accumulates the samples of the lap in progress."""

    def __init__(self, min_clearance_clean_m: float) -> None:
        self.min_clearance_clean_m = float(min_clearance_clean_m)
        self._rows: list[tuple[float, ...]] = []
        self._brakes = 0
        self._non_driving = False
        self._min_clearance = math.inf
        self._clearance_seen = False

    def note_brake(self) -> None:
        """A safety event with phase ENGAGE and severity BRAKE happened during this lap."""
        self._brakes += 1

    def note_planner(self, state: int, clearance_m: float) -> None:
        """Every planner status message (50 Hz), not only the ones at pose samples."""
        if state != STATE_DRIVING:
            self._non_driving = True
        if math.isfinite(clearance_m):
            self._clearance_seen = True
            self._min_clearance = min(self._min_clearance, clearance_m)

    def add_sample(
        self,
        t_s: float,
        x: float,
        y: float,
        yaw: float,
        wheel_speed: float,
        cmd_speed: float,
        cmd_steering: float,
        planner_state: int,
        clearance_m: float,
    ) -> None:
        self._rows.append(
            (
                t_s,
                x,
                y,
                yaw,
                wheel_speed,
                cmd_speed,
                cmd_steering,
                float(planner_state),
                clearance_m,
            )
        )

    @property
    def n_samples(self) -> int:
        return len(self._rows)

    @property
    def brake_count(self) -> int:
        return self._brakes

    def finish(
        self, lap_index: int, lap_time_s: float, length_m: float, cap_mps: float
    ) -> LapRecord:
        """Close the lap, return its record and start an empty one."""
        a = np.array(self._rows, dtype=float).reshape(-1, 9)
        min_clearance = self._min_clearance if self._clearance_seen else math.nan
        clean = lap_is_clean(
            self._brakes, self._non_driving, min_clearance, self.min_clearance_clean_m
        )
        record = LapRecord(
            lap_index=lap_index,
            lap_time_s=float(lap_time_s),
            length_m=float(length_m),
            cap_mps=float(cap_mps),
            brake_count=self._brakes,
            non_driving_seen=self._non_driving,
            min_clearance_m=float(min_clearance),
            clean=clean,
            t_s=a[:, 0],
            x_m=a[:, 1],
            y_m=a[:, 2],
            yaw_rad=a[:, 3],
            wheel_speed_mps=a[:, 4],
            cmd_speed_mps=a[:, 5],
            cmd_steering_rad=a[:, 6],
            planner_state=a[:, 7],
            plan_clearance_m=a[:, 8],
        )
        self._rows = []
        self._brakes = 0
        self._non_driving = False
        self._min_clearance = math.inf
        self._clearance_seen = False
        return record
