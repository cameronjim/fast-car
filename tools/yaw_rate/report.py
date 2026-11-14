"""Tables, JSON and one PNG per segment (reference, ICP and wheel yaw rate against time)."""

from __future__ import annotations

import json
import logging
import math
from pathlib import Path
from typing import Any

import numpy as np

from .pipeline import BIN_LABELS, SegmentResult

log = logging.getLogger(__name__)


def _clean(value: Any) -> Any:
    """JSON-safe copy: NaN and inf become null, numpy scalars become Python numbers."""
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if isinstance(value, (np.floating, float)):
        return None if not math.isfinite(float(value)) else float(value)
    if isinstance(value, (np.integer, np.bool_)):
        return value.item()
    return value


def write_json(summary: dict[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_clean(summary), indent=2, sort_keys=True) + "\n")


def _f(value: float, digits: int = 3) -> str:
    return "n/a" if value is None or not math.isfinite(value) else f"{value:.{digits}f}"


def _lag(d: dict[str, float]) -> str:
    if not math.isfinite(d["lag_s"]):
        return "n/a"
    return f"{d['lag_s']:+.3f} (r {d['peak_corr']:.2f}, drop {d['drop_at_0p1']:.2f}, n {d['n']})"


def markdown_tables(summary: dict[str, Any]) -> str:
    """One table per signal; rows are direction x speed bin."""
    lines: list[str] = []
    for name, per_dir in summary["signals"].items():
        lines.append(f"### Signal: {name}")
        lines.append("")
        lines.append(
            "| direction | speed bin m/s | intervals | rejected | bias rad/s | RMS rad/s | "
            "closure RMS rad/s | longest rejected run s | delay vs pose s | "
            "delay vs steering model s |"
        )
        lines.append("|---|---|---|---|---|---|---|---|---|---|")
        for d, per_bin in per_dir.items():
            for label in BIN_LABELS:
                row = per_bin[label]
                if row["intervals"] == 0:
                    continue
                closure = _f(row.get("closure_rms_rad_per_s", float("nan")))
                run = _f(row.get("longest_rejection_run_s", float("nan")), 1)
                lines.append(
                    f"| {d} | {label} | {row['intervals']} | {_f(row['rejected_share'] * 100, 1)}% "
                    f"| {_f(row['bias'])} | {_f(row['rms'])} | {closure} | {run} "
                    f"| {_lag(row['delay_vs_pose'])} | {_lag(row['delay_vs_steering_model'])} |"
                )
        lines.append("")
    lines.append("### Drops (pose intervals)")
    lines.append("")
    lines.append(
        "| segment | intervals | longer than 1.5 s | pose jump | kept | median pose dt s |"
    )
    lines.append("|---|---|---|---|---|---|")
    for seg, d in summary["drops"].items():
        lines.append(
            f"| {seg} | {d['pose_intervals']} | {d['too_long']} | {d['jumped']} | {d['kept']} "
            f"| {_f(d['pose_dt_median_s'], 3)} |"
        )
    lines.append("")
    lines.append("### Verdicts (0.85 m/s and above, directions pooled)")
    lines.append("")
    for name, v in summary["verdicts"].items():
        lines.append(f"- {name}: {v['label']} ({v['checks']})")
    return "\n".join(lines) + "\n"


def plot_segment(res: SegmentResult, path: Path) -> bool:
    """Reference, ICP and wheel yaw rate against time, whole segment on top and a 20 s zoom of
    the fastest stretch below. Returns False (and logs) when matplotlib is not installed."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:  # pragma: no cover - plots are an optional extra
        log.warning("matplotlib is not installed; no plot for %s", res.name)
        return False
    iv = res.iv
    mid = 0.5 * (iv.t0 + iv.t1)
    wheel = res.sig["wheel"]
    icp = res.sig.get("icp")
    fig, axes = plt.subplots(2, 1, figsize=(12, 7))
    zoom_lo, zoom_hi = None, None
    if len(mid):
        k = int(np.nanargmax(res.speed))
        zoom_lo, zoom_hi = float(mid[k]) - 10.0, float(mid[k]) + 10.0
    for ax, (lo, hi) in zip(axes, [(None, None), (zoom_lo, zoom_hi)], strict=True):
        sel = np.ones(len(mid), bool) if lo is None else (mid >= lo) & (mid <= hi)
        ax.plot(wheel.t, wheel.w, color="tab:green", lw=0.8, label="wheel model")
        if icp is not None:
            t_mid = 0.5 * (icp.t0 + icp.t1)
            ax.plot(t_mid, icp.v / (icp.t1 - icp.t0), ".", ms=2, color="tab:blue", label="ICP step")
        ax.plot(
            mid[sel],
            iv.rate[sel],
            "o",
            ms=3 if lo is not None else 2,
            color="tab:red",
            label="pose interval mean (reference)",
        )
        if lo is not None:
            ax.set_xlim(lo, hi)
        elif len(mid):
            ax.set_xlim(float(mid.min()), float(mid.max()))
        ax.set_ylabel("yaw rate rad/s")
        ax.grid(alpha=0.3)
    axes[1].set_xlabel("stamp, s after the first scan of the bag")
    axes[0].legend(loc="upper right", ncol=3)
    axes[0].set_title(f"{res.name} ({res.direction}), pose median dt {res.pose_dt_median_s:.2f} s")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return True
