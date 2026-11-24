"""Per-lap measures battery B11 (brief 53) reads that the runner did not record before: the lowest
body to wall clearance and the highest speed command of every ground-truth lap, the longest stop
after the car first moved, and the speed bound the lap memory published per station and lap.
Pure functions, no ROS; the runner calls them at the end of a run with a lap memory.

Times are the runner's sim clock (the odometry, /drive_raw header stamps and the node clock agree
under --sim-time). Ground-truth lap k (1-based) is (crossing k-1, crossing k], crossing 0 being the
registered start.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence

#: A wheel speed under this is a stop (the lap gate's standstill threshold is the same order).
STOP_MPS = 0.05
#: Station bins of the published bound, metres (the guidance spacing).
BIN_M = 0.10
#: Stations past this are not binned (no lap of a battery case is longer).
MAX_STATION_M = 40.0


def _lap_edges(crossing0_s: float | None, crossings_s: Sequence[float]) -> list[float]:
    start = -math.inf if crossing0_s is None else float(crossing0_s)
    return [start, *(float(c) for c in crossings_s)]


def _lap_of(t: float, edges: list[float]) -> int:
    """1-based truth lap of time t, 0 before the start, len(edges) after the last crossing."""
    if t <= edges[0]:
        return 0
    for k in range(1, len(edges)):
        if t <= edges[k]:
            return k
    return len(edges)


def clearance_by_lap(
    odom: Sequence[tuple],
    crossing0_s: float | None,
    crossings_s: Sequence[float],
    clearance_of: Callable[[float, float, float], float],
    stride: int = 1,
) -> list[float | None]:
    """Lowest body to wall clearance per truth lap (None for a lap with no sample). `odom` rows
    are (t, x, y, yaw, v); every `stride`-th sample is checked."""
    edges = _lap_edges(crossing0_s, crossings_s)
    out: list[float | None] = [None] * len(crossings_s)
    for i in range(0, len(odom), max(1, stride)):
        t, x, y, yaw = odom[i][0], odom[i][1], odom[i][2], odom[i][3]
        k = _lap_of(float(t), edges)
        if 1 <= k <= len(crossings_s):
            c = float(clearance_of(float(x), float(y), float(yaw)))
            prev = out[k - 1]
            out[k - 1] = c if prev is None else min(prev, c)
    return [None if c is None else round(c, 4) for c in out]


def max_command_by_lap(
    drive_raw: Sequence[tuple], crossing0_s: float | None, crossings_s: Sequence[float]
) -> tuple[list[float | None], float | None]:
    """(highest /drive_raw speed per truth lap, highest after the start). Rows are (stamp, speed,
    steering)."""
    edges = _lap_edges(crossing0_s, crossings_s)
    out: list[float | None] = [None] * len(crossings_s)
    top: float | None = None
    for row in drive_raw:
        t, v = float(row[0]), float(row[1])
        k = _lap_of(t, edges)
        if k == 0:
            continue
        top = v if top is None else max(top, v)
        if k <= len(crossings_s):
            prev = out[k - 1]
            out[k - 1] = v if prev is None else max(prev, v)
    return out, top


def longest_stop_s(odom: Sequence[tuple], crossing0_s: float | None, end_s: float | None) -> float:
    """Longest interval with |wheel speed| under STOP_MPS between the first sample at or above it
    after the start and `end_s` (the last truth crossing judged, or the end of the recording)."""
    start = -math.inf if crossing0_s is None else float(crossing0_s)
    end = math.inf if end_s is None else float(end_s)
    moved = False
    stop_from: float | None = None
    longest = 0.0
    last_t: float | None = None
    for row in odom:
        t, v = float(row[0]), float(row[4])
        if t < start:
            continue
        if t > end:
            break
        last_t = t
        if abs(v) >= STOP_MPS:
            if stop_from is not None:
                longest = max(longest, t - stop_from)
                stop_from = None
            moved = True
        elif moved and stop_from is None:
            stop_from = t
    if stop_from is not None and last_t is not None:
        longest = max(longest, last_t - stop_from)
    return round(longest, 3)


class BoundLog:
    """The speed bound the lap memory published, per system lap and station bin: the highest of
    min(v_cap, v_ref(s)) over the messages of that lap, and the highest cap of the messages with
    no arrays (a cap alone bounds every station)."""

    def __init__(self) -> None:
        self.laps: dict[int, dict] = {}

    def add(
        self, lap_index: int, v_cap: float, s_m: Sequence[float], v_ref: Sequence[float]
    ) -> None:
        entry = self.laps.setdefault(int(lap_index), {"cap_only_max": None, "bins": {}})
        if len(s_m) == 0:
            prev = entry["cap_only_max"]
            entry["cap_only_max"] = float(v_cap) if prev is None else max(prev, float(v_cap))
            return
        bins = entry["bins"]
        for s, v in zip(s_m, v_ref, strict=False):
            s, v = float(s), float(v)
            if not (math.isfinite(s) and math.isfinite(v)) or s < 0.0 or s >= MAX_STATION_M:
                continue
            b = math.floor(s / BIN_M + 1e-6)
            eff = min(float(v_cap), v)
            bins[b] = max(bins.get(b, -math.inf), eff)

    def to_json(self) -> dict[str, dict]:
        return {
            str(k): {
                "cap_only_max": e["cap_only_max"],
                "bins": {str(b): round(v, 4) for b, v in sorted(e["bins"].items())},
            }
            for k, e in sorted(self.laps.items())
        }


def effective_bounds(entry: dict, bins: set[int]) -> dict[int, float]:
    """Per bin of `bins`: the bound of one lap of `BoundLog.to_json` (bin value or cap alone,
    the higher; a bin with neither is left out)."""
    cap = entry.get("cap_only_max")
    stored = {int(b): float(v) for b, v in (entry.get("bins") or {}).items()}
    out = {}
    for b in bins:
        vals = [x for x in (stored.get(b), cap) if x is not None]
        if vals:
            out[b] = max(vals)
    return out


def bounds_above_after(
    doubtful: dict, after: dict, tol: float = 1e-6
) -> list[tuple[int, float, float]]:
    """(bin, bound after, bound in the doubtful lap) for every station bin where the lap after a
    doubtful lap published a higher bound than the doubtful lap did. Bins are those either lap
    has; a bin only the lap after has is compared with the doubtful lap's cap alone, and with
    nothing (a violation) when the doubtful lap published neither."""
    bins = {int(b) for b in (doubtful.get("bins") or {})} | {
        int(b) for b in (after.get("bins") or {})
    }
    if after.get("cap_only_max") is not None and not bins:
        bins = {0}
    a = effective_bounds(doubtful, bins)
    b = effective_bounds(after, bins)
    out = []
    for k in sorted(b):
        ref = a.get(k)
        if ref is None or b[k] > ref + tol:
            out.append((k, b[k], math.nan if ref is None else ref))
    return out
