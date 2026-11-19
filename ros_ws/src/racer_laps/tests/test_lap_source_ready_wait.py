"""L1: the core of `lap_source_ready_wait` (brief 51c item 4): the status reader around
`StartReadyGate` that the car's launch runs before it starts the planner. No ROS. The node around
it is `test_lap_source_ready_wait_ros.py`."""

from __future__ import annotations

import json

import pytest
from racer_laps.lap_source_ready_wait import (
    EXIT_REFUSE,
    EXIT_START,
    LAST_LINE_PREFIX,
    ReadyWaiter,
    reason_of_last_line,
)
from racer_laps.start_ready import StartGateConfig


def body(**changes) -> str:
    out = {
        "lap_source": "camera",
        "phase": "waiting_for_driving",
        "source_ready_now": True,
        "source_alive": True,
        "source_not_ready_reason": "",
    }
    out.update(changes)
    return json.dumps(out)


def waiter(wait_s: float = 30.0, t0: float = 0.0):
    lines: list[str] = []
    return ReadyWaiter(StartGateConfig(wait_s=wait_s), t0, lines.append), lines


def test_the_exit_codes_are_the_registered_ones() -> None:
    assert EXIT_START == 0 and EXIT_REFUSE == 3


def test_two_ready_statuses_start_with_exit_0() -> None:
    w, lines = waiter()
    w.on_message(1.0, body())
    assert w.step(1.0) is None
    w.on_message(2.0, body())
    assert w.step(2.0) == EXIT_START
    assert "ready" in lines[-1] and "starting the planner" in lines[-1]
    assert w.step(500.0) == EXIT_START  # final


def test_no_status_refuses_with_exit_3_and_the_reason_is_the_last_line() -> None:
    w, lines = waiter(wait_s=5.0)
    assert w.step(5.0) is None
    assert w.step(5.1) == EXIT_REFUSE
    assert lines[-1] == f"{LAST_LINE_PREFIX}no lap node status"
    assert reason_of_last_line(lines[-1]) == "no lap node status"


def test_a_not_ready_status_refuses_with_the_lap_nodes_own_reason() -> None:
    w, lines = waiter(wait_s=5.0)
    reason = "similarity 0.50 below 0.90"
    w.on_message(1.0, body(source_ready_now=False, source_not_ready_reason=reason))
    assert w.step(6.0) == EXIT_REFUSE
    assert reason_of_last_line(lines[-1]) == reason


def test_one_line_per_change_of_reason() -> None:
    w, lines = waiter()
    w.step(0.5)
    w.step(0.6)  # still no status: no new line
    w.on_message(1.0, body(source_ready_now=False, source_not_ready_reason="A"))
    w.step(1.0)
    w.on_message(2.0, body(source_ready_now=False, source_not_ready_reason="A"))
    w.step(2.0)
    w.on_message(3.0, body(source_ready_now=False, source_not_ready_reason="B"))
    w.step(3.0)
    assert [x for x in lines if "waiting" in x] == [
        "waiting for the lap node: no lap node status",
        "waiting for the lap node: A",
        "waiting for the lap node: B",
    ]


def test_a_running_lap_node_refuses_at_once() -> None:
    w, lines = waiter()
    w.on_message(1.0, body(phase="running"))
    assert w.step(1.0) == EXIT_REFUSE
    assert reason_of_last_line(lines[-1]) == "lap node already running"


def test_malformed_json_is_counted_and_never_says_ready() -> None:
    w, _ = waiter(wait_s=5.0)
    w.on_message(1.0, "{not json")
    w.on_message(2.0, "[1, 2]")
    w.on_message(3.0, json.dumps({"phase": "x"}))
    assert w.malformed == 2  # the two that did not parse into a mapping
    assert w.step(6.0) == EXIT_REFUSE


def test_a_lap_node_that_died_after_one_good_status_refuses() -> None:
    w, lines = waiter(wait_s=10.0)
    w.on_message(1.0, body())
    assert w.step(1.0) is None
    assert w.step(20.0) == EXIT_REFUSE
    assert LAST_LINE_PREFIX in lines[-1]


def test_reason_of_a_line_without_the_prefix_is_the_line() -> None:
    assert reason_of_last_line("something else") == "something else"
    assert reason_of_last_line(f"[node] {LAST_LINE_PREFIX}no lap node status\n") == (
        "no lap node status"
    )


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_non_finite_times_do_not_decide(bad: float) -> None:
    w, _ = waiter()
    w.on_message(bad, body())
    assert w.step(bad) is None
