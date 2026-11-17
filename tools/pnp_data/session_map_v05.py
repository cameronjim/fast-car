"""Which run of the 2026-11-15 session is which block of the excitation plan (brief 47 step 4).

The runs are those of `pnp_data.runs.run_rows_all` (bag seconds after the first /scan). A launch
log of the car (one file per block, `<block>.launch.log`) carries ROS time (epoch seconds) in
brackets on most lines; a block's interval is the first to the last such time of its log. A run
belongs to the block whose interval it overlaps most; a run no interval touches is `unlogged`
(an aborted launch has no log). The mapping is a table for the reader, committed before any fit;
nothing about the split depends on it (the split rule is one rule for every run).
"""

from __future__ import annotations

import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

from pnp_data.runs import RunRow

_ROS_TIME = re.compile(r"\[(1[0-9]{9}\.[0-9]+)\]")
LOG_SUFFIX = ".launch.log"
UNLOGGED = "unlogged"


def launch_interval(text: str) -> tuple[float, float] | None:
    """(first, last) ROS time in the text, None when it carries none."""
    times = [float(m) for m in _ROS_TIME.findall(text)]
    return (min(times), max(times)) if times else None


def read_blocks(log_dir: Path) -> dict[str, tuple[float, float]]:
    """Block name (the file name without `.launch.log`) -> (first, last) ROS time."""
    out: dict[str, tuple[float, float]] = {}
    for path in sorted(Path(log_dir).glob(f"*{LOG_SUFFIX}")):
        interval = launch_interval(path.read_text(errors="replace"))
        if interval is not None:
            out[path.name[: -len(LOG_SUFFIX)]] = interval
    return out


def overlap_s(a: tuple[float, float], b: tuple[float, float]) -> float:
    return max(0.0, min(a[1], b[1]) - max(a[0], b[0]))


def block_of(run_epoch: tuple[float, float], blocks: Mapping[str, tuple[float, float]]) -> str:
    """The block whose launch interval overlaps the run most (the earlier name on a tie), else
    `unlogged`."""
    best, best_s = UNLOGGED, 0.0
    for name, interval in sorted(blocks.items()):
        s = overlap_s(run_epoch, interval)
        if s > best_s + 1e-9:
            best, best_s = name, s
    return best


@dataclass(frozen=True)
class MapRow:
    bag: str
    run: str
    kind: str
    start_s: float
    end_s: float
    epoch_start: float
    epoch_end: float
    duration_s: float
    wheel_distance_m: float
    command_direction: str
    block: str


def map_runs(
    bag: str,
    rows: Sequence[RunRow],
    first_scan_log_s: float,
    blocks: Mapping[str, tuple[float, float]],
) -> list[MapRow]:
    out = []
    for r in rows:
        epoch = (first_scan_log_s + r.start_s, first_scan_log_s + r.end_s)
        out.append(
            MapRow(
                bag, r.run_id, r.kind, r.start_s, r.end_s, epoch[0], epoch[1], r.duration_s,
                r.wheel_distance_m, r.direction, block_of(epoch, blocks),
            )
        )  # fmt: skip
    return out


def render_table(rows: Sequence[MapRow]) -> str:
    lines = [
        (
            "| bag | run | kind | bag s from | to | duration s | wheel distance m | "
            "command direction | block (launch log) |"
        ),
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        lines.append(
            f"| {r.bag} | {r.run} | {r.kind} | {r.start_s:.1f} | {r.end_s:.1f} | "
            f"{r.duration_s:.1f} | {r.wheel_distance_m:.1f} | {r.command_direction} | {r.block} |"
        )
    return "\n".join(lines) + "\n"
