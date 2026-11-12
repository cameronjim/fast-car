"""Find the stretches of a bag worth replaying.

PnP runs ("R0", "R1", ...): stretches of /pnp/planner/status messages with no gap above 2 s.
Fast laps ("L" is the longest, the others are "L1", "L2", ... in time order): stretches where the wheel speed is above 0.5 m/s for more than
10 s, outside every PnP run (on the 2026-11-12 bag that is another driver's gap follow laps).
Fast samples less than 15 s apart belong to one stretch: slow corners and a short stop inside a
session do not split it. A window ("W") is any --from/--to in bag seconds.
All times are bag seconds, as in bag.py.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

PNP_GAP_S = 2.0
FAST_SPEED_MPS = 0.5
FAST_GAP_S = 15.0
FAST_MIN_DURATION_S = 10.0


@dataclass(frozen=True)
class Segment:
    name: str
    kind: str  # "pnp", "fast" or "window"
    t_from: float
    t_to: float

    @property
    def duration_s(self) -> float:
        return self.t_to - self.t_from

    @property
    def has_planner_status(self) -> bool:
        return self.kind == "pnp"


def _runs(times: NDArray[np.float64], max_gap_s: float) -> list[tuple[float, float]]:
    """Maximal stretches of sorted `times` whose consecutive gaps are at most max_gap_s."""
    if len(times) == 0:
        return []
    t = np.sort(times)
    breaks = np.nonzero(np.diff(t) > max_gap_s)[0]
    starts = np.r_[0, breaks + 1]
    ends = np.r_[breaks, len(t) - 1]
    return [(float(t[a]), float(t[b])) for a, b in zip(starts, ends, strict=True)]


def find_segments(
    status_t: NDArray[np.float64],
    odom_t: NDArray[np.float64],
    odom_v: NDArray[np.float64],
) -> list[Segment]:
    """PnP runs (R0, R1, ...) then the fast laps (in time order, the longest named L)."""
    out = [Segment(f"R{i}", "pnp", a, b) for i, (a, b) in enumerate(_runs(status_t, PNP_GAP_S))]
    fast = np.asarray(odom_v) > FAST_SPEED_MPS
    laps = [
        (a, b)
        for a, b in _runs(np.asarray(odom_t)[fast], FAST_GAP_S)
        if b - a > FAST_MIN_DURATION_S
        and not any(a <= r.t_to and b >= r.t_from for r in out if r.kind == "pnp")
    ]
    longest = max(range(len(laps)), key=lambda i: laps[i][1] - laps[i][0], default=None)
    n = 0
    for i, (a, b) in enumerate(laps):
        if i == longest:
            out.append(Segment("L", "fast", a, b))
        else:
            n += 1
            out.append(Segment(f"L{n}", "fast", a, b))
    return out


def select(
    segments: list[Segment],
    names: list[str] | None,
    t_from: float | None = None,
    t_to: float | None = None,
) -> list[Segment]:
    """The segments called for by --segment names, or one window by --from/--to. Raises
    ValueError naming what exists when a name is unknown."""
    if t_from is not None or t_to is not None:
        if t_from is None or t_to is None or t_to <= t_from:
            raise ValueError("--from and --to both, with --to after --from (bag seconds)")
        return [Segment("W", "window", float(t_from), float(t_to))]
    if not names:
        return list(segments)
    by_name = {s.name: s for s in segments}
    missing = [n for n in names if n not in by_name]
    if missing:
        have = ", ".join(f"{s.name} {s.t_from:.0f}-{s.t_to:.0f} s" for s in segments) or "none"
        raise ValueError(f"no segment {', '.join(missing)}; the bag has: {have}")
    return [by_name[n] for n in names]
