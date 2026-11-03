"""The safety_replay_cli stream file, and running the CLI.

Messages go in ARRIVAL order (the recorder's log time), which is what safety_node acts on: a scan
is used from the moment it arrives, about one sweep after its header stamp, and a /drive_raw's age
is measured from its arrival. Records without a log time (0.0) fall back to their stamp.
"""

from __future__ import annotations

import json
import math
import os
import shutil
import subprocess
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any

from follower_replay.bag import BagData

REPO_ROOT = Path(__file__).resolve().parents[2]
CLI_NAME = "safety_replay_cli"
CLI_ENV = "SAFETY_REPLAY_CLI"

# The rule both 2026-10 floor bags were driven with (vehicle_params 0.12.0): TTC and the floor on
# the distance from the LiDAR head, the TTC on the request alone.
LEGACY_RULE: dict[str, float | bool] = {
    "distance_from_body": False,
    "ttc_brake_s": 0.3,
    "ttc_warning_s": 0.45,
    "min_forward_clearance_m": 0.25,
    "speed_bound_deceleration_mps2": 0.0,
}


class ReplayError(RuntimeError):
    """A problem the user can fix (missing CLI, bad parameter); printed without a trace."""


def _number(value: float) -> str:
    if math.isnan(value):
        return "nan"
    if math.isinf(value):
        return "inf" if value > 0 else "-inf"
    return f"{value:.9g}"


def _arrival(record: Any) -> float:
    log_time = float(getattr(record, "log_time_s", 0.0))
    return log_time if log_time > 0.0 else float(record.stamp_s)


def stream_lines(bag: BagData) -> list[str]:
    """The bag's /scan (S), /drive_raw (D) and /odom/wheel (O) messages as CLI lines, in arrival
    order (scans first on a tie, as a scan callback and a command callback at the same instant
    leave the same state either way)."""
    events: list[tuple[float, int, str]] = []
    for s in bag.scans:
        t = _arrival(s)
        ranges = " ".join(_number(r) for r in s.ranges)
        events.append(
            (
                t,
                0,
                f"S {t:.6f} {s.angle_min:.9g} {s.angle_increment:.9g} {_number(s.range_min)} "
                f"{_number(s.range_max)} {len(s.ranges)} {ranges}".rstrip(),
            )
        )
    for d in bag.drive_raw:
        t = _arrival(d)
        events.append((t, 1, f"D {t:.6f} {d.steering_rad:.9g} {d.speed_mps:.9g}"))
    for o in bag.odom:
        t = _arrival(o)
        events.append((t, 2, f"O {t:.6f} {o.speed_mps:.9g}"))
    events.sort(key=lambda e: (e[0], e[1]))
    return [line for _, _, line in events]


def write_stream(bag: BagData, path: Path) -> int:
    lines = stream_lines(bag)
    with path.open("w", encoding="utf-8") as out:
        out.write("# safety_replay_cli stream (tools/replay_safety.py)\n")
        for line in lines:
            out.write(line + "\n")
    return len(lines)


def find_cli(explicit: str | None = None) -> Path:
    """--cli, else $SAFETY_REPLAY_CLI, else the colcon install tree, else PATH."""
    if explicit:
        if not Path(explicit).is_file():
            raise ReplayError(f"--cli {explicit}: no such file")
        return Path(explicit)
    for candidate in (
        os.environ.get(CLI_ENV, ""),
        str(REPO_ROOT / "ros_ws/install/racer_safety/lib/racer_safety" / CLI_NAME),
    ):
        if candidate and Path(candidate).is_file():
            return Path(candidate)
    found = shutil.which(CLI_NAME)
    if found:
        return Path(found)
    raise ReplayError(
        f"{CLI_NAME} not found. Build it (colcon build --packages-select racer_safety, or the "
        f"clang++ line in docs/notes/safety-floor-review-2026-11-02.md) and pass --cli, or set "
        f"${CLI_ENV}."
    )


def _format_value(value: float | bool) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return f"{value:.9g}"


def set_args(params: Mapping[str, float | bool]) -> list[str]:
    args: list[str] = []
    for key in sorted(params):
        args += ["--set", f"{key}={_format_value(params[key])}"]
    return args


def run_cli(
    cli: Path, stream: Path, params: Mapping[str, float | bool], extra: Iterable[str] = ()
) -> list[dict[str, Any]]:
    """Run the CLI on a stream file; one dict per output line."""
    proc = subprocess.run(
        [str(cli), "--stream", str(stream), *set_args(params), *extra],
        capture_output=True,
        text=True,
        check=False,
    )
    if proc.returncode != 0:
        raise ReplayError(f"{CLI_NAME} exited {proc.returncode}: {proc.stderr.strip()}")
    return [json.loads(line) for line in proc.stdout.splitlines() if line.strip()]


def describe_cli(cli: Path, params: Mapping[str, float | bool] | None = None) -> dict[str, Any]:
    proc = subprocess.run(
        [str(cli), *set_args(params or {}), "--describe"],
        capture_output=True,
        text=True,
        check=False,
    )
    if proc.returncode != 0:
        raise ReplayError(f"{CLI_NAME} exited {proc.returncode}: {proc.stderr.strip()}")
    return json.loads(proc.stdout)
