"""Find the runs of a bag: the stretch in which a driver was commanding.

Same rule as tools/pnp_replay/segments.py (reimplemented, not imported): a PnP run is a maximal
stretch of /pnp/planner/status messages with no gap above 2 s. Other drivers (gap follow, the
tracker) have no status topic; their run is a maximal stretch of non-zero /drive_raw commands with
no gap above 2 s. Times are bag seconds (arrival time after the first /scan).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

#: Largest gap inside one run (pre-registration section 4.2 run definition, segments.py).
RUN_GAP_S = 2.0

#: racer_pnp_msgs/PlannerStatus state values used by the gates.
STATE_BLOCKED = 5
STATE_RECOVERING = 7


@dataclass(frozen=True)
class Run:
    run_id: str
    #: "pnp" (status stretch) or "drive_raw" (non-zero command stretch).
    kind: str
    #: First status message (pnp) or first non-zero command (drive_raw).
    t_start: float
    #: First non-zero /drive_raw command inside the run, None when it never moved a command.
    t_first_cmd: float | None
    #: Last status message (pnp) or last non-zero command (drive_raw): end of the counting window.
    t_end: float

    @property
    def counting_window(self) -> tuple[float, float] | None:
        """[first non-zero command, end] of pre-registration section 4.2, None without a command."""
        if self.t_first_cmd is None:
            return None
        return (self.t_first_cmd, self.t_end)

    @property
    def duration_s(self) -> float:
        """Length of the counting window (0 when the run never commanded)."""
        window = self.counting_window
        return 0.0 if window is None else window[1] - window[0]


def _stretches(times: NDArray[np.float64], max_gap_s: float) -> list[tuple[float, float]]:
    if len(times) == 0:
        return []
    t = np.sort(times)
    breaks = np.nonzero(np.diff(t) > max_gap_s)[0]
    starts = np.r_[0, breaks + 1]
    ends = np.r_[breaks, len(t) - 1]
    return [(float(t[a]), float(t[b])) for a, b in zip(starts, ends, strict=True)]


def find_runs(
    draw_t: ArrayLike,
    draw_speed: ArrayLike,
    status_t: ArrayLike | None = None,
    prefix: str = "R",
) -> list[Run]:
    """The runs of a bag, named `prefix` + index in time order.

    With `status_t` (a PnP bag) a run is a status stretch; its first command is the first
    non-zero /drive_raw inside it and it ends at its last status. Without, a run is a stretch
    of non-zero /drive_raw commands; it starts at the first and ends at the last.
    """
    t = np.asarray(draw_t, dtype=np.float64)
    v = np.asarray(draw_speed, dtype=np.float64)
    if t.shape != v.shape:
        raise ValueError("draw_t and draw_speed must have the same length")
    moving = t[v != 0.0]
    if status_t is None:
        return [
            Run(f"{prefix}{i}", "drive_raw", a, a, b)
            for i, (a, b) in enumerate(_stretches(moving, RUN_GAP_S))
        ]
    out = []
    for i, (a, b) in enumerate(_stretches(np.asarray(status_t, dtype=np.float64), RUN_GAP_S)):
        inside = moving[(moving >= a) & (moving <= b)]
        first = float(inside.min()) if len(inside) else None
        out.append(Run(f"{prefix}{i}", "pnp", a, first, b))
    return out


def longest_state_hold(
    status_t: ArrayLike, status_state: ArrayLike, state: int = STATE_BLOCKED
) -> float:
    """Longest time the planner status stayed in `state`: from the first status message of a
    stretch in that state to the first message that is not (or the last message)."""
    t = np.asarray(status_t, dtype=np.float64)
    s = np.asarray(status_state)
    longest, start = 0.0, None
    for i in range(len(t)):
        if s[i] == state:
            if start is None:
                start = t[i]
            longest = max(longest, float(t[i] - start))
        elif start is not None:
            longest = max(longest, float(t[i] - start))
            start = None
    return longest


def count_state_entries(status_state: ArrayLike, state: int = STATE_RECOVERING) -> int:
    """How many separate times the status entered `state` (a recovery maneuver each)."""
    s = np.asarray(status_state) == state
    if len(s) == 0:
        return 0
    return int(s[0]) + int(np.count_nonzero(s[1:] & ~s[:-1]))
