"""The planner_replay_cli stream file, and running the CLI.

Events go in ARRIVAL order, which is what mppi_planner_node acts on: a scan is used from the
moment it arrives (about one sweep after its header stamp) and carries its header stamp for the
staleness gate and the ego-motion bookkeeping; odometry is stamped with its arrival; a local
track carries the header stamp of the scan it came from. One K line (a planner tick) every
0.02 s from the segment start. An event that arrives at the same instant as a tick is applied
before it.

Tracks: `recorded` feeds the bag's /pnp/local_track at its recorded arrival, `replay` feeds the
tracks of the replayed perception at the scan's arrival plus a fixed latency, `none` feeds none
(the scan-only fault case).

Running the CLI: --cli, else $PLANNER_REPLAY_CLI, else the colcon install tree, else PATH. On a
machine where the binary is a Linux one (a Mac), $PLANNER_REPLAY_DOCKER_CLI names the Linux
binary and the CLI runs in $PLANNER_REPLAY_DOCKER_IMAGE (default ros-dev:local) with the bundle,
stream and output directories mounted at the same paths.
"""

from __future__ import annotations

import csv
import json
import math
import os
import shutil
import subprocess
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from pnp_replay.bag import Recording, scan_ranges
from pnp_replay.perception import Tracks

REPO_ROOT = Path(__file__).resolve().parents[2]
CLI_NAME = "planner_replay_cli"
CLI_ENV = "PLANNER_REPLAY_CLI"
DOCKER_CLI_ENV = "PLANNER_REPLAY_DOCKER_CLI"
DOCKER_IMAGE_ENV = "PLANNER_REPLAY_DOCKER_IMAGE"
TICK_S = 0.02
PREROLL_S = 0.5
TRACK_MODES = ("recorded", "replay", "none")


class ReplayError(RuntimeError):
    """A problem the user can fix (missing CLI, refused bundle); printed without a trace."""


def _number(value: float) -> str:
    if math.isnan(value):
        return "nan"
    if math.isinf(value):
        return "inf" if value > 0 else "-inf"
    return f"{value:.9g}"


def scan_indices(rec: Recording, t_from: float, t_to: float) -> NDArray[np.int64]:
    """Scans that arrive in [t_from, t_to]."""
    t = rec["scan_t"]
    return np.nonzero((t >= t_from) & (t <= t_to))[0].astype(np.int64)


def _track_line(
    arrival: float,
    stamp: float,
    valid: int,
    residual: float,
    intrusion: float,
    arrays: Iterable[NDArray[np.floating]],
) -> str:
    numbers = " ".join(_number(float(v)) for a in arrays for v in a)
    return (
        f"T {arrival:.6f} {stamp:.6f} {int(valid)} {_number(residual)} {_number(intrusion)} "
        f"{numbers}"
    )


def stream_lines(
    rec: Recording,
    t_from: float,
    t_to: float,
    tracks_mode: str = "recorded",
    tracks: Tracks | None = None,
    track_latency_s: float = 0.03,
    clip_speed_mps: float | None = None,
    preroll_s: float = PREROLL_S,
    tick_s: float = TICK_S,
) -> list[str]:
    """The stream of a segment [t_from, t_to] (bag seconds). `tracks` is required for mode
    replay and must cover the scans of [t_from - preroll_s, t_to]. `clip_speed_mps` feeds
    min(wheel speed, clip) as the odometry (replay only: it is not what the car measured)."""
    if tracks_mode not in TRACK_MODES:
        raise ValueError(f"tracks must be one of {TRACK_MODES}, got {tracks_mode!r}")
    if tracks_mode == "replay" and tracks is None:
        raise ValueError("tracks mode replay needs the replayed tracks")
    lo = t_from - preroll_s
    events: list[tuple[float, int, str]] = []
    for i in scan_indices(rec, lo, t_to):
        t = float(rec["scan_t"][i])
        ranges = " ".join(_number(float(r)) for r in scan_ranges(rec, int(i)))
        events.append(
            (
                t,
                0,
                (
                    f"S {t:.6f} {float(rec['scan_stamp'][i]):.6f} "
                    f"{_number(float(rec['scan_angle_min'][i]))} "
                    f"{_number(float(rec['scan_angle_increment'][i]))} "
                    f"{_number(float(rec['scan_range_min'][i]))} "
                    f"{_number(float(rec['scan_range_max'][i]))} {len(ranges.split())} {ranges}"
                ),
            )
        )
    odom = np.nonzero((rec["odom_t"] >= lo) & (rec["odom_t"] <= t_to))[0]
    for i in odom:
        v = float(rec["odom_v"][i])
        if clip_speed_mps is not None:
            v = min(v, clip_speed_mps)
        events.append((float(rec["odom_t"][i]), 0, f"O {float(rec['odom_t'][i]):.6f} {_number(v)}"))
    if tracks_mode == "recorded":
        for k in np.nonzero((rec["trk_t"] >= lo) & (rec["trk_t"] <= t_to))[0]:
            t = float(rec["trk_t"][k])
            events.append(
                (
                    t,
                    0,
                    _track_line(
                        t,
                        float(rec["trk_stamp"][k]),
                        int(rec["trk_valid"][k]),
                        float(rec["trk_residual"][k]),
                        float(rec["trk_intrusion"][k]),
                        (
                            rec["trk_x"][k],
                            rec["trk_y"][k],
                            rec["trk_hl"][k],
                            rec["trk_hr"][k],
                            rec["trk_sigma"][k],
                        ),
                    ),
                )
            )
    elif tracks_mode == "replay":
        assert tracks is not None
        for j, i in enumerate(tracks.scan_index):
            t = float(rec["scan_t"][i]) + track_latency_s
            if not (lo <= t <= t_to):
                continue
            events.append(
                (
                    t,
                    0,
                    _track_line(
                        t,
                        float(rec["scan_stamp"][i]),
                        int(tracks.valid[j]),
                        float(tracks.residual[j]),
                        float(tracks.intrusion[j]),
                        (tracks.x[j], tracks.y[j], tracks.hl[j], tracks.hr[j], tracks.sigma[j]),
                    ),
                )
            )
    n_ticks = math.floor((t_to - t_from) / tick_s + 1e-9) + 1
    for k in range(n_ticks):
        t = t_from + k * tick_s
        events.append((t, 1, f"K {t:.6f}"))
    events.sort(key=lambda e: (e[0], e[1]))
    return [line for _, _, line in events]


def write_stream(lines: Iterable[str], path: Path) -> int:
    n = 0
    with path.open("w", encoding="utf-8") as out:
        out.write("# planner_replay_cli stream (tools/pnp_replay.py)\n")
        for line in lines:
            out.write(line + "\n")
            n += 1
    return n


@dataclass(frozen=True)
class Cli:
    """How to start planner_replay_cli: a native binary, or a Linux one inside docker."""

    binary: Path
    docker_image: str | None = None

    def command(self, args: list[str]) -> list[str]:
        if self.docker_image is None:
            return [str(self.binary), *args]
        mounts = {str(self.binary.parent)}
        for a in args:
            p = Path(a)
            if a.startswith("/") and (p.exists() or p.parent.exists()):
                mounts.add(str(p if p.is_dir() else p.parent))
        flags: list[str] = []
        for path in sorted(mounts):
            flags += ["-v", f"{path}:{path}"]
        return ["docker", "run", "--rm", *flags, "--entrypoint", str(self.binary),
                self.docker_image, *args]  # fmt: skip


def find_cli(explicit: str | None = None) -> Cli:
    """--cli, else $PLANNER_REPLAY_CLI, else the colcon install tree, else PATH, else a Linux
    binary run in docker ($PLANNER_REPLAY_DOCKER_CLI)."""
    if explicit:
        if not Path(explicit).is_file():
            raise ReplayError(f"--cli {explicit}: no such file")
        return Cli(Path(explicit))
    for candidate in (
        os.environ.get(CLI_ENV, ""),
        str(REPO_ROOT / "ros_ws/install/racer_planner/lib/racer_planner" / CLI_NAME),
    ):
        if candidate and Path(candidate).is_file():
            return Cli(Path(candidate))
    found = shutil.which(CLI_NAME)
    if found:
        return Cli(Path(found))
    docker_cli = os.environ.get(DOCKER_CLI_ENV, "")
    if docker_cli and Path(docker_cli).is_file():
        return Cli(Path(docker_cli).resolve(), os.environ.get(DOCKER_IMAGE_ENV, "ros-dev:local"))
    raise ReplayError(
        f"{CLI_NAME} not found. Build it (colcon build --packages-select racer_planner in the "
        f"ros-dev image) and pass --cli, or set ${CLI_ENV}; on a Mac copy the Linux binary out "
        f"of the build and set ${DOCKER_CLI_ENV} to it."
    )


def _format_value(value: float | bool) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return f"{value:.9g}"


def set_args(params: Mapping[str, float | bool | int]) -> list[str]:
    args: list[str] = []
    for key in sorted(params):
        args += ["--set", f"{key}={_format_value(params[key])}"]
    return args


def _run(cli: Cli, args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cli.command(args), capture_output=True, text=True, check=False)


def describe_cli(
    cli: Cli, bundle: Path, v_max: float, params: Mapping[str, float | bool | int]
) -> dict[str, Any]:
    proc = _run(cli, ["--bundle", str(bundle), "--v-max", _format_value(v_max), *set_args(params),
                      "--describe"])  # fmt: skip
    if proc.returncode != 0:
        raise ReplayError(f"{CLI_NAME} --describe exited {proc.returncode}: {proc.stderr.strip()}")
    return json.loads(proc.stdout)


def read_rows(path: Path) -> dict[str, NDArray[Any]]:
    """The CLI's CSV as one array per column (state_name stays strings)."""
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = list(reader)
    out: dict[str, NDArray[Any]] = {}
    for i, name in enumerate(header):
        column = [r[i] for r in rows]
        out[name] = (
            np.array(column, dtype=str) if name == "state_name" else np.array(column, dtype=float)
        )
    out["state"] = out["state"].astype(int)
    return out


def run_cli(
    cli: Cli,
    bundle: Path,
    v_max: float,
    params: Mapping[str, float | bool | int],
    stream: Path,
    out_csv: Path,
) -> dict[str, NDArray[Any]]:
    """Run the CLI on a stream file; the rows of its CSV. A bundle refusal comes back as a
    ReplayError carrying the refusal name."""
    proc = _run(
        cli,
        ["--bundle", str(bundle), "--v-max", _format_value(v_max), *set_args(params),
         "--stream", str(stream), "--out", str(out_csv)],
    )  # fmt: skip
    if proc.returncode != 0:
        raise ReplayError(f"{CLI_NAME} exited {proc.returncode}: {proc.stderr.strip()}")
    return read_rows(out_csv)
