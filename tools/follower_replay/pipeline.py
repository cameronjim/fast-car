"""bag -> scan file -> CLI -> table, summary, CSV and plots: the whole replay in one call."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from follower_replay import analysis, bag, plots, replay


@dataclass
class PlotOptions:
    enabled: bool = False
    count: int = 6
    at_s: list[float] = field(default_factory=list)
    flagged_only: bool = False


@dataclass
class ReplayResult:
    summary: dict[str, Any]
    table: list[dict[str, Any]]
    files: dict[str, Path]


def replay_bag(
    bag_path: Path,
    out_dir: Path,
    profile: str,
    overrides: dict[str, replay.ParamValue],
    cli: Path,
    plot: PlotOptions | None = None,
    straight_threshold_rad: float = analysis.DEFAULT_STRAIGHT_THRESHOLD_RAD,
) -> ReplayResult:
    """Replay a bag's /scan messages through the follower and write everything to out_dir."""
    data = bag.read_bag(bag_path)
    if not data.scans:
        raise replay.ReplayError(
            f"no /scan messages in {bag_path} (topics seen: "
            f"{', '.join(sorted(data.topic_counts)) or 'none'}); was the LiDAR running "
            "(lidar:=true)?"
        )
    out_dir.mkdir(parents=True, exist_ok=True)
    params = replay.resolve_params(profile, overrides)
    # The bag's own LiDAR limits, unless the profile or --param already pinned them.
    cli_params = {**replay.lidar_range_limits(data.scans), **params}
    files = {
        "scans": out_dir / "scans.txt",
        "params": out_dir / "params.json",
        "csv": out_dir / "per_scan.csv",
        "summary": out_dir / "summary.json",
    }
    replay.write_scan_file(data.scans, files["scans"])
    replay.write_params_file(cli_params, files["params"])
    rows = replay.run_cli(cli, files["scans"], files["params"])
    table = analysis.build_table(data.scans, rows, data.drive_raw, data.drive)
    analysis.write_csv(table, files["csv"])
    geometry = replay.describe_cli(cli, files["params"])
    summary = analysis.summarise(
        data,
        table,
        profile,
        params,
        straight_threshold_rad,
        max_steering_rad=geometry.get("max_steering_rad"),
    )
    files["summary"].write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    if plot is not None and plot.enabled:
        indices = plots.select_scan_indices(table, plot.count, plot.at_s, plot.flagged_only)
        files["plot_scans"] = out_dir / "scans.png"
        files["plot_steering"] = out_dir / "steering.png"
        plots.plot_scans(data.scans, table, indices, geometry, files["plot_scans"])
        plots.plot_steering_timeline(table, data.events, files["plot_steering"])
    return ReplayResult(summary=summary, table=table, files=files)
