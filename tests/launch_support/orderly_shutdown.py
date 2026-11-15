"""Orderly end of a launch_testing stack: one participant leaves the DDS domain at a time.

WHY. launch_testing ends a test by sending SIGINT to every process it launched at the same moment.
With three to ten participants in one domain (bridge_node, safety_node, a controller, ekf_node,
amcl, ...) plus the test's own rclpy node, Humble's Fast-DDS 2.6 (libfastrtps 2.6.12, the newest
Humble ships) loses a race between two teardowns of the same endpoint:

  * a process deleting its own subscription or reader on its main thread
    (rclpy Subscription.destroy -> rmw_destroy_subscription), and
  * the same process's receive thread handling the dispose of a PEER's writer, which the peer
    sends from its own exit (EDPSimplePUBListener::onNewCacheChangeAdded ->
    PDP::removeWriterProxyData -> StatefulReader::matched_writer_remove).

Both walk the reader's matched-writer list; the loser dereferences freed state and the process dies
of SIGSEGV (exit -11) AFTER every assertion of the test has passed. Seen in CI as bridge_node (the
L5 park canary, faulthandler trace ending in Subscription.destroy), tracker_node, wheel_twist_node,
ekf_node and gap_follow_node. The crash is in the middleware, not in our nodes (the nodes' own
shutdown code is the orderly kind in every case), and nothing in Humble's apt feed fixes it.
`test_gap_follow_node_params_launch.py` has the original core dump backtrace.

WHAT THIS DOES. `stop_stack()` runs at the end of the test's last tearDownClass, after the test's
own rclpy context is shut down. It sends SIGINT to the processes launch_testing started, one at a
time, newest first, waits until each has really exited, then waits `settle_s` so the survivors
have finished processing that participant's departure before the next one starts. The oldest
process (the sim bridge in every canary) is left for launch_testing's own shutdown, which now has
exactly one process to stop. That last one is deliberate: if every process had exited before the
test thread returned, launch_testing would take "all processes died" for a failed launch.

WHAT IT DOES NOT DO. It does not look at exit codes, kill anything or tolerate anything: the
post-shutdown assertExitCodes still sees the code each process really exited with, so a node that
crashes for its own reasons still fails the test. A process that ignores SIGINT is left alone for
launch_testing's escalation (SIGTERM, SIGKILL).

Linux only (/proc), like the ros-dev image the canaries run in.
"""

from __future__ import annotations

import os
import signal
import time
from collections.abc import Callable
from dataclasses import dataclass

#: Pause after each process exit, so the survivors have handled the departure. A participant's
#: departure is one burst of UDP datagrams that a peer's receive thread works through in
#: milliseconds; half a second is a wide margin for a loaded 2-CPU runner (docs/notes/
#: shutdown-segv-2026-11-14.md). It costs 0.5 s per process at the end of a canary.
SETTLE_S = 0.5
#: How long one process gets to exit on SIGINT before it is left to launch_testing's escalation.
EXIT_TIMEOUT_S = 15.0


@dataclass(frozen=True)
class Child:
    pid: int
    start_ticks: int
    cmdline: str


def _read_stat(pid: int) -> tuple[str, int, int] | None:
    """(state, ppid, start_ticks) of a process, or None when it is gone."""
    try:
        with open(f"/proc/{pid}/stat", "rb") as handle:
            raw = handle.read().decode("utf-8", "replace")
    except OSError:
        return None
    # The command name sits in parentheses and may itself contain spaces or parentheses.
    fields = raw[raw.rindex(")") + 2 :].split()
    return fields[0], int(fields[1]), int(fields[19])


def _cmdline(pid: int) -> str:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as handle:
            return handle.read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
    except OSError:
        return ""


def children_of(parent_pid: int) -> list[Child]:
    """Live (not zombie) direct children of `parent_pid`, oldest first."""
    found = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        stat = _read_stat(int(entry))
        if stat is None or stat[1] != parent_pid or stat[0] in ("Z", "X"):
            continue
        found.append(Child(int(entry), stat[2], _cmdline(int(entry))))
    return sorted(found, key=lambda c: (c.start_ticks, c.pid))


def _has_exited(pid: int) -> bool:
    stat = _read_stat(pid)
    # A zombie has exited; it stays listed until launch's child watcher reaps it.
    return stat is None or stat[0] in ("Z", "X")


def stop_stack(
    *,
    settle_s: float = SETTLE_S,
    exit_timeout_s: float = EXIT_TIMEOUT_S,
    parent_pid: int | None = None,
    log: Callable[[str], None] = print,
) -> list[int]:
    """SIGINT the launched processes newest first, one at a time; returns the pids it stopped.

    Leaves the oldest process running (see the module docstring). Call it after the test's own
    rclpy.shutdown()."""
    children = children_of(os.getpid() if parent_pid is None else parent_pid)
    stopped: list[int] = []
    if len(children) < 2:
        return stopped
    time.sleep(settle_s)  # the test's own participant has just left the domain
    for child in reversed(children[1:]):
        try:
            os.kill(child.pid, signal.SIGINT)
        except ProcessLookupError:
            continue
        deadline = time.monotonic() + exit_timeout_s
        while not _has_exited(child.pid) and time.monotonic() < deadline:
            time.sleep(0.02)
        if not _has_exited(child.pid):
            log(f"[orderly_shutdown] pid {child.pid} ({child.cmdline[:80]}) ignored SIGINT")
            continue
        stopped.append(child.pid)
        time.sleep(settle_s)
    return stopped
