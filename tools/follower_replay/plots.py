"""PNG plots for a follower replay. matplotlib is an optional extra (`uv sync --extra plots`).

select_scan_indices is pure and unit-tested; the drawing functions import matplotlib when
called, so the rest of the tool runs without it.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from follower_replay.bag import PHASE_RELEASE, EventRecord, ScanRecord

PLOT_RANGE_M = 3.0  # half extent of the scan panels
ARC_LENGTH_M = 1.0  # length of the drawn steering arcs
TTC_SOURCES = ("ttc", "ttc_reverse")


def select_scan_indices(
    table: Sequence[dict[str, Any]],
    count: int,
    at_s: Sequence[float] = (),
    flagged_only: bool = False,
) -> list[int]:
    """Which scans to draw: the ones nearest the times in at_s, else `count` evenly spaced.

    at_s are seconds from the first scan. flagged_only restricts the evenly spaced pick to
    scans where the swept-path clamp or the corner override fired.
    """
    if not table:
        return []
    if at_s:
        picked = {min(range(len(table)), key=lambda i, t=t: abs(table[i]["t_s"] - t)) for t in at_s}
        return sorted(picked)
    pool = (
        [i for i, r in enumerate(table) if r["clamped"] or r["corner_blocked"]]
        if (flagged_only)
        else list(range(len(table)))
    )
    if not pool or count <= 0:
        return []
    if count >= len(pool):
        return pool
    if count == 1:
        return [pool[len(pool) // 2]]
    step = (len(pool) - 1) / (count - 1)
    return sorted({pool[round(k * step)] for k in range(count)})


def vehicle_points(scan: ScanRecord, geometry: dict[str, float]) -> tuple[list[float], list[float]]:
    """Valid returns in the rear-axle frame (x forward, y left), as the follower sees them.

    vehicle bearing = laser bearing + laser yaw (the follower's laser bearing = vehicle bearing
    - yaw); then shifted by the LiDAR mount.
    """
    xs: list[float] = []
    ys: list[float] = []
    for i, r in enumerate(scan.ranges):
        if not (math.isfinite(r) and scan.range_min <= r <= scan.range_max and r > 0.0):
            continue
        bearing = scan.angle_min + i * scan.angle_increment + geometry["laser_yaw_rad"]
        xs.append(r * math.cos(bearing) + geometry["lidar_mount_x_m"])
        ys.append(r * math.sin(bearing) + geometry["lidar_mount_y_m"])
    return xs, ys


def arc_points(
    steering_rad: float, wheelbase_m: float, length_m: float = ARC_LENGTH_M, n: int = 30
):
    """The rear axle's path for a constant steering angle (bicycle model), x forward, y left."""
    k = math.tan(steering_rad) / wheelbase_m
    pts = []
    for j in range(n + 1):
        s = length_m * j / n
        if abs(k) < 1e-9:
            pts.append((s, 0.0))
        else:
            pts.append((math.sin(k * s) / k, (1 - math.cos(k * s)) / k))
    return pts


def _pyplot():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def plot_scans(
    scans: Sequence[ScanRecord],
    table: Sequence[dict[str, Any]],
    indices: Sequence[int],
    geometry: dict[str, float],
    path: Path,
) -> None:
    """The chosen scans in the vehicle frame with the target and the steering arcs.

    The horizontal axis is y drawn right to left, so the car's left is on the left of the
    picture; the car points up.
    Blue arc: replay output before the low-pass, dashed: what the follower wanted before the
    clamps, green: the recorded /drive_raw steering. The star is the chosen target.
    """
    plt = _pyplot()
    n = max(len(indices), 1)
    cols = min(n, 3)
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(4.5 * cols, 4.5 * rows), squeeze=False)
    half_w = geometry["half_width_m"]
    for ax in axes.flat:
        ax.set_visible(False)
    for ax, i in zip(axes.flat, indices, strict=False):
        ax.set_visible(True)
        scan, row = scans[i], table[i]
        xs, ys = vehicle_points(scan, geometry)
        ax.scatter(ys, xs, s=4, color="0.35", label="returns")
        ax.add_patch(
            plt.Rectangle(
                (-half_w, 0.0), 2 * half_w, geometry["body_front_x_m"], fill=False, color="k"
            )
        )
        ax.plot([geometry["lidar_mount_y_m"]], [geometry["lidar_mount_x_m"]], "k+", ms=8)
        wheelbase = geometry["wheelbase_m"]
        for key, style, colour, label in (
            ("wanted_steering_rad", "--", "tab:blue", "wanted"),
            ("steering_rad", "-", "tab:blue", "replay"),
            ("recorded_drive_raw_rad", "-", "tab:green", "recorded /drive_raw"),
        ):
            value = row[key]
            if value is None or (isinstance(value, float) and math.isnan(value)):
                continue
            pts = arc_points(value, wheelbase)
            ax.plot([p[1] for p in pts], [p[0] for p in pts], style, color=colour, label=label)
        bearing = row["target_bearing_rad"]
        if row["valid"] and bearing is not None:
            tx = row["target_range_m"] * math.cos(bearing) + geometry["lidar_mount_x_m"]
            ty = row["target_range_m"] * math.sin(bearing) + geometry["lidar_mount_y_m"]
            ax.plot(
                [geometry["lidar_mount_y_m"], ty],
                [geometry["lidar_mount_x_m"], tx],
                ":",
                color="tab:red",
            )
            ax.plot([ty], [tx], "*", color="tab:red", ms=14, label="target")
        flags = [
            name
            for name, on in (
                ("clamped", row["clamped"]),
                ("corner override", row["corner_blocked"]),
                ("no gap", row["valid"] and not row["gap_found"]),
            )
            if on
        ]
        ax.set_title(
            f"t = {row['t_s']:.2f} s" + (f"  [{', '.join(flags)}]" if flags else ""), fontsize=9
        )
        ax.set_xlim(PLOT_RANGE_M, -PLOT_RANGE_M)
        ax.set_ylim(-0.5, PLOT_RANGE_M * 1.5)
        ax.set_aspect("equal")
        ax.set_xlabel("y (m), left positive")
        ax.set_ylabel("x forward (m)")
        ax.grid(alpha=0.3)
    first = next((a for a in axes.flat if a.get_visible()), None)
    if first is not None:
        first.legend(fontsize=7, loc="lower right")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def latch_spans(events: Sequence[EventRecord], t0_s: float) -> list[tuple[float, float, str]]:
    """(start, end, source) in seconds from t0 for every released TTC latch."""
    spans = []
    for e in events:
        if e.phase == PHASE_RELEASE and e.source in TTC_SOURCES:
            spans.append((e.stamp_s - e.duration_s - t0_s, e.stamp_s - t0_s, e.source))
    return spans


def plot_steering_timeline(
    table: Sequence[dict[str, Any]],
    events: Sequence[EventRecord],
    path: Path,
) -> None:
    """Replay steering against the recorded /drive_raw, with flags and the TTC latches."""
    plt = _pyplot()
    t = [r["t_s"] for r in table]

    def column(key: str) -> list[float]:
        return [float("nan") if r[key] is None else r[key] for r in table]

    fig, ax = plt.subplots(figsize=(12, 4.5))
    ax.plot(
        t, column("recorded_drive_raw_rad"), color="tab:green", lw=1.5, label="recorded /drive_raw"
    )
    ax.plot(
        t, column("filtered_steering_rad"), color="tab:blue", lw=1.5, label="replay (low-passed)"
    )
    ax.plot(t, column("steering_rad"), color="tab:blue", lw=0.8, ls=":", label="replay (raw)")
    ax.plot(
        t,
        column("wanted_steering_rad"),
        color="0.5",
        lw=0.8,
        ls="--",
        label="wanted (before clamps)",
    )
    for key, colour, label, level in (
        ("clamped", "tab:orange", "swept-path clamp", -0.5),
        ("corner_blocked", "tab:red", "corner override", -0.55),
    ):
        hits = [r["t_s"] for r in table if r[key]]
        ax.plot(hits, [level] * len(hits), "|", color=colour, ms=8, label=label)
    if table:
        t0 = table[0]["stamp_s"]
        for k, (start, end, source) in enumerate(latch_spans(events, t0)):
            ax.axvspan(
                start, end, color="tab:purple", alpha=0.15, label="TTC latch" if k == 0 else None
            )
            _ = source
    ax.axhline(0.0, color="k", lw=0.5)
    ax.set_xlabel("time since first scan (s)")
    ax.set_ylabel("steering (rad, left positive)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncol=3, loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
