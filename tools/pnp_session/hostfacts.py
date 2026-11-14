"""Host facts for the session log: the planner process's scheduling policy, the CPU governor and
the load, found with READ-ONLY commands (pre-registration PnP v0.3 section 5: "read, not set").

Nothing here sets a scheduling policy, a governor or any parameter. The commands are an
allow-list; a test asserts that no entry can write (no `chrt` option other than `-p <pid>`, no
redirection, no `sudo`). Output goes to a text file (the session log) and a JSON file.
"""

from __future__ import annotations

import glob
import subprocess
from collections.abc import Callable, Sequence
from typing import Any

PROCESS_PATTERN = "mppi_planner_node"
GOVERNOR_GLOB = "/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor"

Runner = Callable[[Sequence[str]], "subprocess.CompletedProcess[str]"]


def default_runner(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(list(command), capture_output=True, text=True, check=False, timeout=10)


def planner_pids(runner: Runner, pattern: str = PROCESS_PATTERN) -> list[int]:
    """Process ids whose command line contains `pattern` (pgrep, read only)."""
    done = runner(["pgrep", "-f", pattern])
    return [int(w) for w in done.stdout.split() if w.isdigit()]


def read_only_commands(pids: Sequence[int], governor_files: Sequence[str]) -> list[list[str]]:
    """Every command the facts step runs, spelled out."""
    commands: list[list[str]] = [["pgrep", "-f", PROCESS_PATTERN]]
    commands += [["chrt", "-p", str(pid)] for pid in pids]
    if governor_files:
        commands.append(["cat", *governor_files])
    commands.append(["uptime"])
    return commands


def collect(
    runner: Runner = default_runner,
    governor_files: Sequence[str] | None = None,
) -> dict[str, Any]:
    """Run the commands and return what they printed, one entry per command."""
    pids = planner_pids(runner)
    files = sorted(glob.glob(GOVERNOR_GLOB)) if governor_files is None else list(governor_files)
    entries = []
    for command in read_only_commands(pids, files):
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
