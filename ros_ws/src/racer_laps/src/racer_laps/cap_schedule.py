"""The speed cap rule (D v0). Pure Python, no ROS.

Lap 1 runs at `v_cap_start_mps`. After a clean lap the cap rises one step, up to the ceiling.
After an unclean lap it drops one step (not below the start value), and the next rise needs two
clean laps in a row. The cap only changes in `finish_lap`, so it never changes mid lap.

The cap is `min(ceiling, start + level * step)` with an integer `level`, so repeated steps do not
accumulate float error and a ceiling that is not on the step grid is still reached exactly.
"""

from __future__ import annotations

import math


def lap_is_clean(
    brake_count: int,
    non_driving_seen: bool,
    min_clearance_m: float,
    min_clearance_clean_m: float,
) -> bool:
    """A lap is clean when there was no obstacle-gate brake (`brake_count` is the number of
    /safety/events ENGAGE records at BRAKE severity from the sources ttc and ttc_reverse: the
    watchdog and rate_limit gates are timing events and are not passed here), the planner stayed
    in DRIVING the whole lap, and the smallest plan clearance was at least
    `min_clearance_clean_m`.

    A lap with no clearance data (NaN) is not clean: it cannot be verified.
    """
    if brake_count > 0 or non_driving_seen:
        return False
    if math.isnan(min_clearance_m):
        return False
    return min_clearance_m >= min_clearance_clean_m


class CapSchedule:
    def __init__(self, start_mps: float, step_mps: float, ceiling_mps: float) -> None:
        if not (start_mps > 0.0 and step_mps > 0.0):
            raise ValueError("v_cap_start_mps and v_cap_step_mps must be positive")
        if ceiling_mps < start_mps:
            raise ValueError(
                f"v_cap_ceiling_mps {ceiling_mps} is below v_cap_start_mps {start_mps}"
            )
        self.start_mps = float(start_mps)
        self.step_mps = float(step_mps)
        self.ceiling_mps = float(ceiling_mps)
        self._max_level = math.ceil((self.ceiling_mps - self.start_mps) / self.step_mps - 1e-9)
        self._level = 0
        self._clean_streak = 0
        self._needs_two = False

    @property
    def cap_mps(self) -> float:
        return min(self.ceiling_mps, self.start_mps + self._level * self.step_mps)

    def finish_lap(self, clean: bool) -> float:
        """Record the outcome of the lap just driven. Returns the cap for the next lap."""
        if clean:
            self._clean_streak += 1
            if (not self._needs_two) or self._clean_streak >= 2:
                self._level = min(self._max_level, self._level + 1)
                self._clean_streak = 0
                self._needs_two = False
        else:
            self._level = max(0, self._level - 1)
            self._clean_streak = 0
            self._needs_two = True
        return self.cap_mps
