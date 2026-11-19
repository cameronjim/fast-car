"""L1: the waiter ties readiness to the counter's reset (outside review of brief 51c, finding 2).

Required order: the reset is acknowledged, then fresh frames and odometry after it, then the source
is ready, then the planner starts. A status that was queued before the reset, or published by a lap
node from the counter's earlier life, must not satisfy the gate: the waiter is given the reset id the
counter acknowledged and ignores every status that carries an older one."""

from __future__ import annotations

import json

from racer_laps.lap_source_ready_wait import EXIT_REFUSE, EXIT_START, ReadyWaiter
from racer_laps.start_ready import StartGateConfig


def body(reset_id, **changes) -> str:
    out = {
        "lap_source": "camera",
        "phase": "waiting_for_driving",
        "source_ready_now": True,
        "source_alive": True,
        "source_not_ready_reason": "",
    }
    if reset_id is not None:
        out["source_reset_id"] = reset_id
    out.update(changes)
    return json.dumps(out)


def waiter(min_reset_id, wait_s=30.0):
    lines: list[str] = []
    return ReadyWaiter(StartGateConfig(wait_s=wait_s), 0.0, lines.append, min_reset_id), lines


def test_statuses_of_an_earlier_reset_never_start() -> None:
    w, lines = waiter(3)
    for t in (1.0, 2.0, 3.0):
        w.on_message(t, body(2))  # ready, but of the counter's reset before ours
        assert w.step(t) is None
    assert "reset" in lines[-1] and "3" in lines[-1]


def test_statuses_of_the_acknowledged_reset_start() -> None:
    w, _ = waiter(3)
    w.on_message(1.0, body(3))
    w.on_message(2.0, body(3))
    assert w.step(2.0) == EXIT_START


def test_a_later_reset_is_fine() -> None:
    w, _ = waiter(3)
    w.on_message(1.0, body(4))
    w.on_message(2.0, body(4))
    assert w.step(2.0) == EXIT_START


def test_a_queued_status_then_a_fresh_one_needs_two_fresh_ones() -> None:
    w, _ = waiter(2)
    w.on_message(1.0, body(1))  # queued from before the reset
    w.on_message(2.0, body(2))  # the first status of the new reset
    assert w.step(2.0) is None
    w.on_message(3.0, body(2))
    assert w.step(3.0) == EXIT_START


def test_a_status_without_a_reset_id_does_not_satisfy_a_gate_that_was_told_one() -> None:
    w, _ = waiter(1)
    w.on_message(1.0, body(None))
    w.on_message(2.0, body(None))
    assert w.step(2.0) is None
    assert w.step(31.0) == EXIT_REFUSE


def test_the_refusal_after_the_wait_names_the_reset_mismatch() -> None:
    w, lines = waiter(5, wait_s=5.0)
    w.on_message(1.0, body(4))
    assert w.step(6.0) == EXIT_REFUSE
    assert "reset" in lines[-1]


def test_no_reset_id_given_means_no_condition_as_before() -> None:
    w, _ = waiter(None)
    w.on_message(1.0, body(None))
    w.on_message(2.0, body(7))
    assert w.step(2.0) == EXIT_START
