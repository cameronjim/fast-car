"""Rate limiter what-if: the recorded `/drive_raw` through racer_safety's UNCHANGED gate code.

The gate is not reimplemented here. `limiter_whatif_cli` (ros_ws/src/racer_session_tools) compiles
racer_safety's own sources read only, the way safety_replay_cli does, and this module feeds it one
line per recorded safety cycle:

    cycle times     the header stamps of the recorded `/drive` messages (safety_node publishes one
                    per gate cycle, stamped with the same `now` it stamps its events with)
    dt, recorded    the gap of two consecutive cycle stamps
    dt, nominal     the gate's control period (the CLI substitutes it)
    command         the latest `/drive_raw` that had arrived at or before the cycle; arrival is the
                    command's own header stamp (the recorder's receive time lags by up to 0.37 s)
    age             the cycle time minus that arrival
    segments        one per counting window, seeded from the recorded `/drive` output before it,
                    because the bag lacks `/drive_raw` messages at run starts

The header stamp is not safety_node's own receive time either (about a millisecond of DDS and
executor delay), so the command in force can differ by one message at a cycle and the watchdog
replays are not exact. The check that
matters first: with the recorded dt, the replay must raise the `rate_limit` engagements the bag
recorded. matched, missed and extra are reported; the replay has no scan, so a recorded obstacle
brake has no counterpart (see whatif_core.hpp).
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from racer_eval import interventions

from pnp_session import detail as detail_mod
from pnp_session.bagread import SessionBag
from pnp_session.constants import LIMITER_MATCH_TOL_S
from pnp_session.timing import TimingAnalysis

REPO_ROOT = Path(__file__).resolve().parents[2]
CLI_NAME = "limiter_whatif_cli"
CLI_ENV = "LIMITER_WHATIF_CLI"
DT_MODES = ("recorded", "nominal")


class LimiterUnavailable(RuntimeError):
    """The what-if tool binary is not there or failed; says exactly what blocks the item."""


def find_cli(explicit: str | None = None) -> Path:
    """--limiter-cli, else $LIMITER_WHATIF_CLI, else the colcon install tree, else PATH."""
    if explicit:
        if not Path(explicit).is_file():
            raise LimiterUnavailable(f"--limiter-cli {explicit}: no such file")
        return Path(explicit)
    for candidate in (
        os.environ.get(CLI_ENV, ""),
        str(REPO_ROOT / "ros_ws/install/racer_session_tools/lib/racer_session_tools" / CLI_NAME),
    ):
        if candidate and Path(candidate).is_file():
            return Path(candidate)
    found = shutil.which(CLI_NAME)
    if found:
        return Path(found)
    raise LimiterUnavailable(
        f"{CLI_NAME} not found. It is built from ros_ws/src/racer_session_tools, which compiles "
        f"racer_safety's own sources read only: `colcon build --packages-select "
        f"racer_session_tools` (ros-dev container), then pass --limiter-cli or set ${CLI_ENV}."
    )


def _number(v: float) -> str:
    if np.isinf(v):
        return "inf" if v > 0 else "-inf"
    return f"{v:.17g}"  # exact: a command is a float32 widened to double, 9 digits would perturb it


def cycle_table(bag: SessionBag, arrival: str = "stamp") -> tuple[NDArray[np.float64], ...]:
    """(stamp, dt, steering, speed, age) of every recorded safety cycle. The command in force at
    a cycle is the latest /drive_raw that had arrived at or before the cycle stamp; before the
    first one it is zero with an infinite age. `arrival` is the recorder's receive time ("recv")
    or the command's own header stamp ("stamp"); the recorder's receive time can lag the sender by
    hundreds of milliseconds (up to 0.37 s in 2026-11-13T22-20-18)."""
    if arrival not in ("recv", "stamp"):
        raise ValueError("arrival must be recv or stamp")
    stamp = np.asarray(bag.drv_stamp, dtype=np.float64)
    n = len(stamp)
    dt = np.zeros(n, dtype=np.float64)
    if n > 1:
        dt[1:] = np.diff(stamp)
    raw = np.asarray(bag.draw_t if arrival == "recv" else bag.draw_stamp, dtype=np.float64)
    order = np.argsort(raw, kind="stable")
    recv = raw[order]
    idx = np.searchsorted(recv, stamp, side="right") - 1
    steer = np.zeros(n, dtype=np.float64)
    speed = np.zeros(n, dtype=np.float64)
    age = np.full(n, np.inf)
    have = idx >= 0
    steer[have] = bag.draw_steer[order][idx[have]]
    speed[have] = bag.draw_speed[order][idx[have]]
    age[have] = stamp[have] - recv[idx[have]]
    return stamp, dt, steer, speed, age


def write_stream(
    bag: SessionBag,
    path: Path,
    windows: Sequence[tuple[float, float]] | None = None,
    arrival: str = "stamp",
    teacher: bool = False,
) -> int:
    """The cycle stream. With `windows` (the counting windows of the runs) each window is its own
    segment: an `R` line seeds the previous output from the recorded /drive output of the cycle
    before the window, then the window's cycles follow. The recorded /drive_raw can lack the
    messages behind the output a run starts from (the recorder joins late), so a chain running
    across the whole bag would start every run from a state the bag cannot explain."""
    stamp, dt, steer, speed, age = cycle_table(bag, arrival)
    segments = [(-np.inf, np.inf)] if windows is None else list(windows)
    n_written = 0
    with path.open("w", encoding="utf-8") as out:
        out.write("# limiter_whatif_cli stream (tools/pnp_session/whatif.py)\n")
        for lo, hi in segments:
            idx = np.nonzero((stamp >= lo) & (stamp <= hi))[0]
            if len(idx) == 0:
                continue
            first = int(idx[0])
            if windows is not None:
                seed = first - 1 if first > 0 else first
                out.write(
                    f"R {stamp[first]:.9f} {_number(bag.drv_steer[seed])} "
                    f"{_number(bag.drv_speed[seed])}\n"
                )
            for i in idx:
                extra = ""
                if teacher:
                    j = max(int(i) - 1, 0)
                    extra = f" {_number(bag.drv_steer[j])} {_number(bag.drv_speed[j])}"
                out.write(
                    f"C {stamp[i]:.9f} {dt[i]:.9f} {_number(steer[i])} {_number(speed[i])} "
                    f"{_number(age[i])}{extra}\n"
                )
                n_written += 1
    return n_written


def run_cli(cli: Path, stream: Path, dt_mode: str, teacher: bool = False) -> list[dict[str, Any]]:
    if dt_mode not in DT_MODES:
        raise ValueError(f"dt_mode must be one of {DT_MODES}")
    proc = subprocess.run(
        [
            str(cli),
            "--stream",
            str(stream),
            "--dt-mode",
            dt_mode,
            *(["--teacher"] if teacher else []),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if proc.returncode != 0:
        raise LimiterUnavailable(f"{CLI_NAME} exited {proc.returncode}: {proc.stderr.strip()}")
    return [json.loads(line) for line in proc.stdout.splitlines() if line.strip()]


def _windows(analysis: TimingAnalysis) -> list[tuple[str, tuple[float, float]]]:
    out = []
    for p in analysis.pieces:
        w = p.run.counting_window
        assert w is not None
        out.append((p.run.run_id, w))
    return out


def _in_any(t: float, windows: Sequence[tuple[str, tuple[float, float]]]) -> str | None:
    for rid, (lo, hi) in windows:
        if lo <= t <= hi:
            return rid
    return None


def _engage(lines: Sequence[dict[str, Any]], source: str) -> list[dict[str, Any]]:
    return [
        ln
        for ln in lines
        if ln["type"] == "event" and ln["phase"] == "engage" and ln["source"] == source
    ]


def compare_events(
    recorded: Sequence[tuple[float, str]],
    replay: Sequence[dict[str, Any]],
    tol_s: float = LIMITER_MATCH_TOL_S,
) -> dict[str, Any]:
    """Match recorded engagements (stamp, detail) to replayed ones by cycle stamp, one to one.
    Returns matched, missed (recorded, not replayed), extra (replayed, not recorded) and, for the
    matched pairs, the largest difference of the dt printed in the two detail strings."""
    used: set[int] = set()
    matched = 0
    dt_diffs: list[float] = []
    for t, det in recorded:
        best = None
        for i, r in enumerate(replay):
            if i in used:
                continue
            d = abs(float(r["t"]) - t)
            if d <= tol_s and (best is None or d < best[0]):
                best = (d, i)
        if best is not None:
            used.add(best[1])
            matched += 1
            a = detail_mod.parse_rate_limit_dt(det)
            b = detail_mod.parse_rate_limit_dt(str(replay[best[1]]["detail"]))
            if a is not None and b is not None:
                dt_diffs.append(abs(a - b))
    return {
        "recorded": len(recorded),
        "replayed": len(replay),
        "matched": matched,
        "missed": len(recorded) - matched,
        "extra": len(replay) - matched,
        "dt_detail_max_abs_diff_s": max(dt_diffs) if dt_diffs else None,
        "dt_detail_compared": len(dt_diffs),
        "match_tolerance_s": tol_s,
    }


def whatif_report(
    bag: SessionBag,
    analysis: TimingAnalysis,
    cli: Path,
    work_dir: Path | None = None,
    arrival: str = "stamp",
) -> dict[str, Any]:
    """Both replays, clips counted per run and pooled in the counting windows, and the
    reproduction check against the recorded rate_limit engagements."""
    windows = _windows(analysis)
    with tempfile.TemporaryDirectory(dir=work_dir) as tmp:
        stream = Path(tmp) / "cycles.txt"
        n_cycles = write_stream(bag, stream, [w for _, w in windows], arrival)
        results = {mode: run_cli(cli, stream, mode) for mode in DT_MODES}
        teacher_stream = Path(tmp) / "cycles_teacher.txt"
        write_stream(bag, teacher_stream, [w for _, w in windows], arrival, teacher=True)
        teacher_results = {
            mode: run_cli(cli, teacher_stream, mode, teacher=True) for mode in DT_MODES
        }
        teacher_lines = teacher_results["recorded"]

    def per_run(lines: Sequence[dict[str, Any]]) -> dict[str, dict[str, int]]:
        out = {
            rid: {"clip_cycles": 0, "engage_rate_limit": 0, "engage_watchdog": 0}
            for rid, _ in windows
        }
        for ln in lines:
            if ln["type"] == "clip":
                rid = _in_any(float(ln["t"]), windows)
                if rid:
                    out[rid]["clip_cycles"] += 1
            elif ln["type"] == "event" and ln["phase"] == "engage":
                rid = _in_any(float(ln["t"]), windows)
                if rid and ln["source"] in ("rate_limit", "watchdog"):
                    out[rid][f"engage_{ln['source']}"] += 1
        return out

    modes: dict[str, Any] = {}
    keys = ("clip_cycles", "engage_rate_limit", "engage_watchdog")
    for mode in DT_MODES:
        for label, lines in (("", teacher_results[mode]), ("_free_running", results[mode])):
            rows = per_run(lines)
            pooled = {k: sum(r[k] for r in rows.values()) for k in keys}
            modes[mode + label] = {"per_run": rows, "pooled": pooled}

    # Reproduction: recorded rate_limit and watchdog engagements inside the windows, by cycle stamp.
    rec_sel = bag.ev_phase == interventions.PHASE_ENGAGE
    repro: dict[str, Any] = {}
    for source in ("rate_limit", "watchdog"):
        rec = [
            (float(bag.ev_stamp[i]), str(bag.ev_detail[i]))
            for i in np.nonzero(rec_sel & (bag.ev_source == source))[0]
            if _in_any(float(bag.ev_t[i]), windows)
        ]
        rep = [ln for ln in _engage(teacher_lines, source) if _in_any(float(ln["t"]), windows)]
        repro[source] = compare_events(rec, rep)
        free = [
            ln for ln in _engage(results["recorded"], source) if _in_any(float(ln["t"]), windows)
        ]
        repro[source + "_free_running"] = compare_events(rec, free)
    return {
        "status": "run",
        "tool": CLI_NAME,
        "cycles": n_cycles,
        "arrival": arrival,
        "segments": "one per counting window, seeded from the recorded /drive output before it",
        "modes": modes,
        "reproduction_recorded_dt": repro,
        "window_basis": (
            "Replayed engagements are placed in a run's counting window by the cycle stamp; the "
            "recorded ones by recorder receive time (the 4.2 rule)."
        ),
        "not_modelled": (
            "no scan (the obstacle gate never trips), no wheel odometry; the age of /drive_raw "
            "uses the command's header stamp as its arrival, not safety_node's own receive time"
        ),
    }


def not_run(reason: str) -> dict[str, Any]:
    return {"status": "not run", "reason": reason}
