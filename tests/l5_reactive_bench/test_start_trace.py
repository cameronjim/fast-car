"""L1 tests (brief 51c, item 1): the runner's `start_trace` and the test hook that holds the
oracle's status back. No ROS."""

from __future__ import annotations

import math

import pytest
import track_family_runner as runner

TRACE_KEYS = {
    "lap_node_first_status_s",
    "ready_first_s",
    "gate_start_s",
    "planner_spawn_s",
    "driving_first_s",
    "lap1_start_s",
    "oracle_first_status_s",
    "gate_verdict",
    "gate_reason",
}


def test_the_trace_has_exactly_the_registered_keys_and_none_for_what_never_happened() -> None:
    out = runner.start_trace()
    assert set(out) == TRACE_KEYS
    assert all(v is None for v in out.values())


def test_the_trace_keeps_what_it_is_given() -> None:
    out = runner.start_trace(
        lap_node_first_status_s=10.5,
        ready_first_s=11.5,
        gate_start_s=12.5,
        planner_spawn_s=12.6,
        driving_first_s=15.0,
        lap1_start_s=15.02,
        oracle_first_status_s=9.75,
        gate_verdict="start",
        gate_reason="",
    )
    assert out["lap_node_first_status_s"] == 10.5
    assert out["gate_start_s"] == 12.5 and out["planner_spawn_s"] == 12.6
    assert out["driving_first_s"] == 15.0 and out["lap1_start_s"] == 15.02
    assert out["oracle_first_status_s"] == 9.75
    assert out["gate_verdict"] == "start" and out["gate_reason"] == ""


def test_a_non_finite_time_is_written_as_none_so_the_json_stays_valid() -> None:
    out = runner.start_trace(driving_first_s=math.nan, lap1_start_s=math.inf)
    assert out["driving_first_s"] is None and out["lap1_start_s"] is None


@pytest.mark.parametrize(
    ("hold", "driving", "now", "held"),
    [
        (None, None, 5.0, False),  # no hook: never held
        (1.0, None, 5.0, True),  # no DRIVING report yet: held
        (1.0, 10.0, 10.5, True),  # inside the hold after the report
        (1.0, 10.0, 11.0, False),  # exactly at the end of the hold: released
        (1.0, 10.0, 11.5, False),
    ],
)
def test_the_hold_after_driving_hook(
    hold: float | None, driving: float | None, now: float, held: bool
) -> None:
    assert runner.oracle_status_held(now, driving, hold) is held


def test_the_hook_is_read_from_the_environment_only(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(runner.ORACLE_HOLD_ENV, raising=False)
    assert runner.oracle_hold_after_driving_s() is None
    monkeypatch.setenv(runner.ORACLE_HOLD_ENV, "1.0")
    assert runner.oracle_hold_after_driving_s() == 1.0
    monkeypatch.setenv(runner.ORACLE_HOLD_ENV, "not a number")
    with pytest.raises(ValueError):
        runner.oracle_hold_after_driving_s()
