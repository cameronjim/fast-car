"""Real-time scheduling helpers for car_teleop.launch.py and pnp.launch.py (brief 52).

Launch level only: nothing in racer_safety or racer_planner changes. With `rt_sched:=on` the
launch files start `safety_node` and `mppi_planner_node` behind `chrt -f <priority>` (values from
config/scheduling.yaml, read by config/scheduling.py). If `chrt` cannot set the policy it exits
non-zero, the node never starts, and the launch shuts down with one ERROR line that starts with
config/scheduling.py's REFUSAL_PREFIX. There is no fallback to SCHED_OTHER: running without real
time is `rt_sched:=off`, the default.

After start, a read-only check looks at /proc/<pid>/task/*/stat of the process and logs one line
(policy and priority of every thread). With `on`, any thread that is not FIFO at the configured
priority is an ERROR and shuts the launch down; with `off` the same line is logged at INFO so every
launch log says what it ran with. Nothing here publishes, serves or writes anything.

This file is loaded by path from the two launch files (they are not importable as a package).
"""

from __future__ import annotations

import importlib.util
import sys
import time
from pathlib import Path

from launch import logging as launch_logging
from launch.actions import EmitEvent, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration

#: launch/ -> racer_bringup/ -> src/ -> ros_ws/ -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[4]
_SCHEDULING_PY = _REPO_ROOT / "config" / "scheduling.py"

RT_SCHED_CHOICES = ["off", "on"]
RT_SCHED_DEFAULT = "off"
#: seconds after a process start before its threads are read (DDS and executor threads exist by then)
CHECK_DELAY_S = 5.0
#: a non-zero exit this soon after start is read as `chrt` refusing
EARLY_EXIT_S = 10.0
#: kernel policy numbers (include/uapi/linux/sched.h) as ps prints them
POLICY_NAMES = {0: "OTHER", 1: "FIFO", 2: "RR", 3: "BATCH", 5: "IDLE", 6: "DEADLINE"}
FIFO = 1


def scheduling_module():
    """config/scheduling.py, loaded by path (cached)."""
    name = "racer_scheduling_config"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, _SCHEDULING_PY)
    if spec is None or spec.loader is None:  # pragma: no cover - defensive
        raise RuntimeError(f"rt_sched: cannot load {_SCHEDULING_PY}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def load_scheduling():
    return scheduling_module().load()


def refusal_prefix() -> str:
    return scheduling_module().REFUSAL_PREFIX


def check_mode(text: str) -> str:
    """`on` or `off`; anything else is refused."""
    value = text.strip()
    if value not in RT_SCHED_CHOICES:
        raise RuntimeError(f"rt_sched must be one of {RT_SCHED_CHOICES}, got {text!r}")
    return value


def mode_of(context) -> str:
    return check_mode(LaunchConfiguration("rt_sched", default=RT_SCHED_DEFAULT).perform(context))


class RtPrefix(Substitution):
    """The launch prefix of one node: empty with rt_sched off, ` chrt -f <priority>` with on."""

    def __init__(self, node: str) -> None:
        super().__init__()
        self._node = node

    def describe(self) -> str:
        return f"RtPrefix({self._node})"

    def perform(self, context) -> str:
        if mode_of(context) == "off":
            return ""
        return " " + load_scheduling().chrt_prefix(self._node)


def prefix_for(node: str) -> list:
    """A Node `prefix` that keeps the global `launch-prefix` and adds the RT prefix when on."""
    return [LaunchConfiguration("launch-prefix", default=""), RtPrefix(node)]


# -- /proc reading -----------------------------------------------------------------------------


def parse_stat(text: str) -> tuple[int, int, int, str]:
    """(tid, policy, rt_priority, comm) of one /proc/<pid>/task/<tid>/stat line.

    comm may contain spaces and parentheses, so the fields after it are counted from the LAST
    closing parenthesis: field 3 is the state, 40 is rt_priority and 41 is policy (proc(5)).
    """
    head, _, tail = text.rpartition(")")
    tid_text, _, comm = head.partition("(")
    fields = tail.split()  # fields[0] is field 3
    return int(tid_text), int(fields[41 - 3]), int(fields[40 - 3]), comm


def read_threads(pid: int, proc_root: str = "/proc") -> list[tuple[int, int, int, str]] | None:
    """Every thread of `pid` as (tid, policy, rt_priority, comm); None when the process is gone."""
    task_dir = Path(proc_root) / str(pid) / "task"
    try:
        tids = sorted(int(p.name) for p in task_dir.iterdir() if p.name.isdigit())
    except OSError:
        return None
    threads = []
    for tid in tids:
        try:
            threads.append(parse_stat((task_dir / str(tid) / "stat").read_text()))
        except (OSError, ValueError, IndexError):
            continue  # that thread ended between the listing and the read
    return threads or None


def _name(policy: int) -> str:
    return POLICY_NAMES.get(policy, str(policy))


def evaluate(
    label: str,
    pid: int,
    threads: list[tuple[int, int, int, str]] | None,
    expected_priority: int,
    mode: str,
) -> tuple[bool, str]:
    """(ok, one log line). `ok` is False only for mode `on` with a thread that is not FIFO at the
    expected priority, or no readable thread at all."""
    if threads is None:
        return mode != "on", f"rt_sched {label}: pid {pid} is gone, its threads cannot be read"
    listing = ", ".join(
        f"{tid}:{_name(policy)}" + (f"/{prio}" if policy in (1, 2) else "")
        for tid, policy, prio, _comm in threads
    )
    line = f"rt_sched {mode}: {label} pid {pid}, {len(threads)} threads: {listing}"
    if mode != "on":
        return True, line
    bad = [t for t in threads if t[1] != FIFO or t[2] != expected_priority]
    if not bad:
        return True, f"{line} (all FIFO {expected_priority})"
    wrong = ", ".join(f"{tid}:{_name(policy)}/{prio}" for tid, policy, prio, _comm in bad)
    return False, f"{line}; not FIFO {expected_priority}: {wrong}"


# -- launch actions ----------------------------------------------------------------------------


def _log():
    return launch_logging.get_logger("racer_bringup.rt_sched")


def check_actions(
    node, label: str, node_key: str, started: dict[str, float], proc_root: str = "/proc"
) -> list:
    """Handler: read the process's threads CHECK_DELAY_S after it starts and log them; with `on`,
    a bad thread is an ERROR and a Shutdown. `started` records the start time for exit_actions."""

    def on_start(event, context):
        mode = mode_of(context)
        pid = event.pid
        started["at"] = time.monotonic()

        def check(ctx):
            threads = read_threads(pid, proc_root)
            expected = load_scheduling().priority(node_key)
            ok, line = evaluate(label, pid, threads, expected, mode)
            if ok:
                _log().info(line)
                return []
            _log().error(line)
            return [EmitEvent(event=Shutdown(reason=line))]

        return [TimerAction(period=CHECK_DELAY_S, actions=[OpaqueFunction(function=check)])]

    return [RegisterEventHandler(OnProcessStart(target_action=node, on_start=on_start))]


def exit_callback(label: str, started: dict[str, float], emit_shutdown: bool = True):
    """The OnProcessExit callback of exit_actions, separate so a test can call it."""

    def on_exit(event, context):
        if mode_of(context) != "on":
            return []
        code = event.returncode
        at = started.get("at")
        refused = code != 0 and (at is None or time.monotonic() - at < EARLY_EXIT_S)
        if refused:
            reason = (
                f"{refusal_prefix()} {label} exited with code {code} within {EARLY_EXIT_S:.0f} s "
                "of its start (most likely chrt refused: is the container started with "
                "--ulimit rtprio?)"
            )
        else:
            reason = f"{label} exited"
        _log().error(reason)
        if refused:
            # an exception, not a Shutdown: `ros2 launch` then ends with a non-zero exit code
            raise RuntimeError(reason)
        return [EmitEvent(event=Shutdown(reason=reason))] if emit_shutdown else []

    return on_exit


def exit_actions(node, label: str, started: dict[str, float], emit_shutdown: bool = True) -> list:
    """With `on`: a node that exits is logged as an ERROR. A non-zero exit soon after the start is
    `chrt` refusing: the REFUSAL_PREFIX line, then an exception so the launch exits non-zero. Any
    other exit shuts the launch down unless `emit_shutdown` is False (the caller already does)."""
    return [
        RegisterEventHandler(
            OnProcessExit(target_action=node, on_exit=exit_callback(label, started, emit_shutdown))
        )
    ]


def all_actions(node, label: str, node_key: str, emit_shutdown: bool = True) -> list:
    started: dict[str, float] = {}
    return [
        *check_actions(node, label, node_key, started),
        *exit_actions(node, label, started, emit_shutdown),
    ]


def actions_if_on(node, label: str, node_key: str, emit_shutdown: bool = True) -> OpaqueFunction:
    """For a launch description built before its arguments are known (car_teleop): one entity
    that adds the handlers only with `rt_sched:=on`. Put it BEFORE `node` so the handlers exist
    when the node's start event fires."""

    def expand(context):
        if mode_of(context) != "on":
            return []
        return all_actions(node, label, node_key, emit_shutdown)

    return OpaqueFunction(function=expand)
