"""L2 (hypothesis): properties of the start rule of brief 51c. Left out where hypothesis is absent
(conftest.py).

1. For any status sequence, `start` is returned only if the last two statuses taken in before it
   both said ready and were at most 2.5 s apart.
2. Once the wait is over, the verdict is `start` or `refuse`, never `wait`.
3. No sequence gives both: a verdict never changes once it is not `wait`."""

from __future__ import annotations

import math
from typing import Any

from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from racer_laps.start_ready import StartGateConfig, StartReadyGate

SETTINGS = settings(
    max_examples=200,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large],
)

KINDS = ("ready", "not_ready", "no_key", "wrong_type", "running", "start_scan", "nan_time")


def make_status(kind: str) -> dict[str, Any]:
    out: dict[str, Any] = {
        "lap_source": "camera",
        "phase": "waiting_for_driving",
        "source_ready_now": True,
        "source_alive": True,
        "source_not_ready_reason": "r",
    }
    if kind == "not_ready":
        out["source_ready_now"] = False
    elif kind == "no_key":
        del out["source_alive"]
    elif kind == "wrong_type":
        out["source_ready_now"] = 1
    elif kind == "running":
        out["phase"] = "running"
    elif kind == "start_scan":
        out["lap_source"] = "start_scan"
    return out


steps = st.lists(st.tuples(st.floats(0.0, 6.0), st.sampled_from(KINDS)), min_size=0, max_size=40)


def says_ready(kind: str) -> bool:
    return kind == "ready"


@SETTINGS
@given(plan=steps, wait_s=st.floats(1.0, 40.0))
def test_start_only_after_two_fresh_ready_statuses(
    plan: list[tuple[float, str]], wait_s: float
) -> None:
    cfg = StartGateConfig(wait_s=wait_s)
    t0 = 50.0
    gate = StartReadyGate(cfg, t0)
    t = t0
    taken: list[tuple[float, str]] = []  # the statuses the gate took in: (time, kind)
    for dt, kind in plan:
        t += dt
        stamp = math.nan if kind == "nan_time" else t
        gate.on_status(stamp, make_status("ready" if kind == "nan_time" else kind))
        if kind != "nan_time":
            taken.append((t, kind))
        before = gate.decide(t)[0]
        if before == "start":
            last_two = [x for x in taken if x[1] != "nan_time"][-2:]
            assert len(last_two) == 2
            assert all(says_ready(k) for _, k in last_two), last_two
            assert last_two[1][0] - last_two[0][0] <= cfg.status_gap_max_s + 1e-9, last_two
            break


@SETTINGS
@given(plan=steps, wait_s=st.floats(1.0, 40.0), extra=st.floats(0.001, 10.0))
def test_after_the_wait_the_verdict_is_start_or_refuse(
    plan: list[tuple[float, str]], wait_s: float, extra: float
) -> None:
    gate = StartReadyGate(StartGateConfig(wait_s=wait_s), 50.0)
    t = 50.0
    for dt, kind in plan:
        t += dt
        gate.on_status(math.nan if kind == "nan_time" else t, make_status(kind))
        gate.decide(t)
    verdict, reason = gate.decide(max(t, 50.0 + wait_s) + extra)
    assert verdict in ("start", "refuse")
    assert (reason == "") == (verdict == "start")


@SETTINGS
@given(plan=steps, wait_s=st.floats(1.0, 40.0), decide_after=st.lists(st.floats(0, 8), max_size=10))
def test_a_verdict_never_changes_once_it_is_not_wait(
    plan: list[tuple[float, str]], wait_s: float, decide_after: list[float]
) -> None:
    gate = StartReadyGate(StartGateConfig(wait_s=wait_s), 50.0)
    t = 50.0
    final: tuple[str, str] | None = None
    for dt, kind in plan:
        t += dt
        gate.on_status(math.nan if kind == "nan_time" else t, make_status(kind))
        verdict = gate.decide(t)
        if final is not None:
            assert verdict == final
        elif verdict[0] != "wait":
            final = verdict
    for extra in decide_after:
        t += extra
        verdict = gate.decide(t)
        if final is not None:
            assert verdict == final
        elif verdict[0] != "wait":
            final = verdict


@SETTINGS
@given(plan=steps, wait_s=st.floats(1.0, 40.0))
def test_start_only_if_the_second_ready_status_arrived_inside_the_wait(
    plan: list[tuple[float, str]], wait_s: float
) -> None:
    gate = StartReadyGate(StartGateConfig(wait_s=wait_s), 50.0)
    t = 50.0
    for dt, kind in plan:
        t += dt
        gate.on_status(math.nan if kind == "nan_time" else t, make_status(kind))
        if gate.decide(t)[0] == "start":
            assert t <= 50.0 + wait_s + 1e-9, (t, wait_s)
            break
