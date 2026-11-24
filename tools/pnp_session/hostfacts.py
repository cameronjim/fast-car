"""Host facts for the session log: the scheduling policy of safety_node and the planner process,
the CPU governor and the load, found with READ-ONLY commands (pre-registration PnP v0.3 section 5:
"read, not set"; brief 52 for the second process and the per-thread view).

Nothing here sets a scheduling policy, a governor or any parameter. The commands are an
allow-list; a test asserts that no entry can write (no `chrt` option other than `-p <pid>`, no
redirection, no `sudo`). Output goes to a text file (the session log) and a JSON file.

`scheduling_check` compares the facts with config/scheduling.yaml and the run's `rt_sched` value:
`as configured`, `off`, `MISMATCH` or `not recorded` (missing facts are never a pass).
"""

from __future__ import annotations

import glob
import importlib.util
import subprocess
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

PROCESS_PATTERN = "mppi_planner_node"
SAFETY_PATTERN = "safety_node"
#: node name -> pgrep pattern, in the order they are looked up and logged
PROCESSES = {"safety_node": SAFETY_PATTERN, "mppi_planner_node": PROCESS_PATTERN}
PS_COLUMNS = "tid,cls,rtprio,ni,psr,comm"
SCHED_RT_RUNTIME = "/proc/sys/kernel/sched_rt_runtime_us"
CPU_ONLINE = "/sys/devices/system/cpu/online"
_SCHEDULING_PY = Path(__file__).resolve().parents[2] / "config" / "scheduling.py"
GOVERNOR_GLOB = "/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor"

Runner = Callable[[Sequence[str]], "subprocess.CompletedProcess[str]"]


def default_runner(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(list(command), capture_output=True, text=True, check=False, timeout=10)


def planner_pids(runner: Runner, pattern: str = PROCESS_PATTERN) -> list[int]:
    """Process ids whose command line contains `pattern` (pgrep, read only)."""
    done = runner(["pgrep", "-f", pattern])
    return [int(w) for w in done.stdout.split() if w.isdigit()]


def read_only_commands(
    pids: Sequence[int],
    governor_files: Sequence[str],
    safety_pids: Sequence[int] = (),
) -> list[list[str]]:
    """Every command the facts step runs, spelled out."""
    commands: list[list[str]] = [["pgrep", "-f", PROCESS_PATTERN], ["pgrep", "-f", SAFETY_PATTERN]]
    for pid in [*pids, *safety_pids]:
        commands += [
            ["chrt", "-p", str(pid)],
            ["ps", "-L", "-o", PS_COLUMNS, "-p", str(pid)],
            ["cat", f"/proc/{pid}/limits"],
        ]
    commands += [["cat", SCHED_RT_RUNTIME], ["cat", CPU_ONLINE], ["nvpmodel", "-q"]]
    if governor_files:
        commands.append(["cat", *governor_files])
    commands.append(["uptime"])
    return commands


def parse_ps_threads(text: str) -> list[dict[str, Any]]:
    """Threads from `ps -L -o tid,cls,rtprio,ni,psr,comm -p <pid>` output (header line first).
    `cls` is ps's class code (TS = SCHED_OTHER, FF = FIFO, RR); rtprio is None for `-`."""
    threads = []
    for line in text.splitlines()[1:]:
        parts = line.split(None, 5)
        if len(parts) < 6 or not parts[0].isdigit():
            continue
        tid, cls, rtprio, ni, psr, comm = parts
        threads.append(
            {
                "tid": int(tid),
                "cls": cls,
                "rtprio": int(rtprio) if rtprio.lstrip("-").isdigit() and rtprio != "-" else None,
                "ni": ni,
                "psr": psr,
                "comm": comm,
            }
        )
    return threads


def scheduling_summary(entries: Sequence[dict[str, Any]], pids: dict[str, list[int]]) -> dict:
    """node -> {pids, processes: {pid: threads}} from the `ps -L` entries."""
    out: dict[str, Any] = {}
    for node, node_pids in pids.items():
        processes = {}
        for pid in node_pids:
            for e in entries:
                if e["command"][:2] == ["ps", "-L"] and e["command"][-1] == str(pid):
                    processes[str(pid)] = parse_ps_threads(e["stdout"] or "")
        out[node] = {"pids": list(node_pids), "processes": processes}
    return out


def collect(
    runner: Runner = default_runner,
    governor_files: Sequence[str] | None = None,
) -> dict[str, Any]:
    """Run the commands and return what they printed, one entry per command."""
    pids = planner_pids(runner)
    safety_pids = planner_pids(runner, SAFETY_PATTERN)
    files = sorted(glob.glob(GOVERNOR_GLOB)) if governor_files is None else list(governor_files)
    entries = []
    for command in read_only_commands(pids, files, safety_pids):
        try:
            done = runner(command)
            entries.append(
                {
                    "command": command,
                    "returncode": done.returncode,
                    "stdout": done.stdout,
                    "stderr": done.stderr,
                }
            )
        except (OSError, subprocess.SubprocessError) as exc:
            entries.append(
                {"command": command, "returncode": None, "stdout": "", "stderr": str(exc)}
            )
    return {
        "planner_pids": pids,
        "safety_pids": safety_pids,
        "scheduling": scheduling_summary(
            entries, {"safety_node": safety_pids, "mppi_planner_node": pids}
        ),
        "note": (
            "no planner process found on this host (the planner runs in a container: run this on "
            "the Jetson host, where its pid is visible)"
            if not pids
            else ""
        ),
        "entries": entries,
    }


def format_text(facts: dict[str, Any]) -> str:
    lines = ["# host facts (read only, nothing was set)"]
    for e in facts["entries"]:
        lines.append(f"$ {' '.join(e['command'])}")
        out = (e["stdout"] or "").rstrip()
        err = (e["stderr"] or "").rstrip()
        if out:
            lines.append(out)
        if err:
            lines.append(f"[stderr] {err}")
        lines.append(f"[exit {e['returncode']}]")
    if facts["note"]:
        lines.append(f"NOTE: {facts['note']}")
    return "\n".join(lines) + "\n"


# -- the session check ---------------------------------------------------------------------------

FIFO_CLASS = "FF"


def load_scheduling(path: Path | str | None = None):
    """config/scheduling.py's Scheduling, loaded by path (it is plain Python and PyYAML)."""
    spec = importlib.util.spec_from_file_location("racer_scheduling_config_tools", _SCHEDULING_PY)
    if spec is None or spec.loader is None:  # pragma: no cover - defensive
        raise RuntimeError(f"cannot load {_SCHEDULING_PY}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.load(path)


def scheduling_check(facts: dict[str, Any] | None, scheduling: Any, rt_sched: str) -> dict:
    """Compare host facts with config/scheduling.yaml and the run's `rt_sched` value.

    verdict: `as configured` (on, and every thread of safety_node and the planner is FIFO at the
    configured priority), `off` (off, and no thread of either is real-time), `MISMATCH` (anything
    else that was recorded), `not recorded` (no facts, or a process or its threads missing: never
    a pass). `problems` lists what differs.
    """
    if rt_sched not in ("on", "off"):
        raise ValueError(f"rt_sched must be on or off, got {rt_sched!r}")
    summary = (facts or {}).get("scheduling")
    if not summary:
        return {"rt_sched": rt_sched, "verdict": "not recorded", "problems": ["no host facts"]}
    problems: list[str] = []
    missing: list[str] = []
    for node in PROCESSES:
        entry = summary.get(node) or {}
        threads = [t for ts in (entry.get("processes") or {}).values() for t in ts]
        if not threads:
            missing.append(f"{node}: no process or no threads recorded")
            continue
        expected = scheduling.priority(node)
        for t in threads:
            fifo = t["cls"] == FIFO_CLASS
            if rt_sched == "on" and not (fifo and t["rtprio"] == expected):
                problems.append(
                    f"{node} thread {t['tid']} is {t['cls']}/{t['rtprio']}, expected FF/{expected}"
                )
            if rt_sched == "off" and fifo:
                problems.append(f"{node} thread {t['tid']} is FF/{t['rtprio']} with rt_sched off")
    if problems:
        return {"rt_sched": rt_sched, "verdict": "MISMATCH", "problems": problems + missing}
    if missing:
        return {"rt_sched": rt_sched, "verdict": "not recorded", "problems": missing}
    return {
        "rt_sched": rt_sched,
        "verdict": "as configured" if rt_sched == "on" else "off",
        "problems": [],
    }


def format_check(check: dict[str, Any]) -> str:
    flag = "FLAG " if check["verdict"] == "MISMATCH" else ""
    lines = [f"{flag}Scheduling check (rt_sched {check['rt_sched']}): {check['verdict']}"]
    lines += [f"  - {p}" for p in check["problems"]]
    return "\n".join(lines) + "\n"
