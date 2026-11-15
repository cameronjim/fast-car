"""L1: orderly_shutdown.stop_stack stops the children one at a time, newest first, and leaves the
oldest alone; it never changes anyone's exit status and never kills."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import textwrap
import time

import orderly_shutdown
import pytest

pytestmark = pytest.mark.skipif(not sys.platform.startswith("linux"), reason="needs /proc")

# A child that records when it got SIGINT and when it finished leaving, then exits with `code`.
_CHILD = textwrap.dedent(
    """
    import os, signal, sys, time
    name, log, code, leave_s, ignore = sys.argv[1], sys.argv[2], int(sys.argv[3]), float(sys.argv[4]), sys.argv[5]
    def on_int(*_):
        if ignore == "1":
            return
        with open(log, "a") as f:
            f.write(f"{name} int {time.monotonic():.3f}\\n")
        time.sleep(leave_s)
        with open(log, "a") as f:
            f.write(f"{name} gone {time.monotonic():.3f}\\n")
        os._exit(code)
    signal.signal(signal.SIGINT, on_int)
    open(log + "." + name + ".ready", "w").close()
    while True:
        time.sleep(0.05)
    """
)


def _start(tmp_path, name, *, code=0, leave_s=0.05, ignore=False):
    log = tmp_path / "events.log"
    proc = subprocess.Popen(
        [
            sys.executable,
            "-c",
            _CHILD,
            name,
            str(log),
            str(code),
            str(leave_s),
            "1" if ignore else "0",
        ]
    )
    ready = tmp_path / f"events.log.{name}.ready"
    deadline = time.monotonic() + 10
    while not ready.exists() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert ready.exists(), f"{name} did not start"
    time.sleep(0.05)  # distinct start times, whatever the clock tick
    return proc


def _events(tmp_path):
    log = tmp_path / "events.log"
    rows = [line.split() for line in log.read_text().splitlines()] if log.exists() else []
    return [(name, what, float(t)) for name, what, t in rows]


@pytest.fixture
def children(tmp_path):
    started = []

    def start(*args, **kwargs):
        proc = _start(tmp_path, *args, **kwargs)
        started.append(proc)
        return proc

    yield start
    for proc in started:
        if proc.poll() is None:
            proc.kill()
        proc.wait()


def test_newest_first_one_at_a_time_oldest_left_running(tmp_path, children):
    oldest = children("a")
    middle = children("b")
    newest = children("c")
    stopped = orderly_shutdown.stop_stack(settle_s=0.3, exit_timeout_s=10.0, log=lambda m: None)
    assert stopped == [newest.pid, middle.pid]
    assert oldest.poll() is None, "the oldest process is left to launch_testing"
    events = _events(tmp_path)
    assert [(n, w) for n, w, _ in events] == [
        ("c", "int"),
        ("c", "gone"),
        ("b", "int"),
        ("b", "gone"),
    ]
    gone_c = events[1][2]
    int_b = events[2][2]
    assert int_b - gone_c >= 0.29, "the next SIGINT waits for the settle time after the exit"


def test_exit_status_is_left_alone(tmp_path, children):
    children("a")
    crashing = children("b", code=139)
    orderly_shutdown.stop_stack(settle_s=0.05, exit_timeout_s=10.0, log=lambda m: None)
    assert crashing.wait(timeout=5) == 139, "a process's own exit status is never changed or hidden"


def test_a_process_that_ignores_sigint_is_not_killed(tmp_path, children):
    children("a")
    stubborn = children("b", ignore=True)
    newest = children("c")
    messages: list[str] = []
    stopped = orderly_shutdown.stop_stack(settle_s=0.05, exit_timeout_s=0.5, log=messages.append)
    assert stopped == [newest.pid]
    assert stubborn.poll() is None, "never SIGKILL: launch_testing's escalation is the right place"
    assert any("ignored SIGINT" in m for m in messages)


def test_one_child_or_none_is_a_no_op(tmp_path, children):
    assert orderly_shutdown.stop_stack(settle_s=0.0, log=lambda m: None) == []
    only = children("a")
    assert orderly_shutdown.stop_stack(settle_s=0.0, log=lambda m: None) == []
    assert only.poll() is None


def test_zombies_do_not_count_as_children(tmp_path, children):
    # `a` has exited but nobody has waited for it yet: a zombie, which must not be signalled.
    children("a")
    dead = children("b", code=3, leave_s=0.0)
    dead.send_signal(signal.SIGINT)
    time.sleep(0.3)
    assert orderly_shutdown.children_of(os.getpid())  # `a` is still there
    assert all(c.pid != dead.pid for c in orderly_shutdown.children_of(os.getpid()))
