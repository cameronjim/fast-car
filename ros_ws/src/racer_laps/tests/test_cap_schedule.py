"""L1: the cap rule, table driven, and the clean-lap criteria one at a time."""

from __future__ import annotations

import math

import pytest
from racer_laps.cap_schedule import CapSchedule, lap_is_clean

C, U = True, False  # clean, unclean

# (start, step, ceiling), outcomes in order, caps expected for each lap 1..n+1
CASES = [
    pytest.param((0.5, 0.1, 1.0), [], [0.5], id="lap1-at-start"),
    pytest.param((0.5, 0.1, 1.0), [C], [0.5, 0.6], id="rise"),
    pytest.param((0.5, 0.1, 1.0), [C, C, C], [0.5, 0.6, 0.7, 0.8], id="keeps-rising"),
    pytest.param((0.5, 0.1, 1.0), [C] * 7, [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.0, 1.0], id="ceiling"),
    pytest.param((0.5, 0.1, 0.75), [C, C, C], [0.5, 0.6, 0.7, 0.75], id="off-grid-ceiling"),
    pytest.param((0.5, 0.1, 1.0), [U], [0.5, 0.5], id="floor-on-first-lap"),
    pytest.param((0.5, 0.1, 1.0), [U, U, U], [0.5, 0.5, 0.5, 0.5], id="floor-holds"),
    pytest.param((0.5, 0.1, 1.0), [C, C, U], [0.5, 0.6, 0.7, 0.6], id="drop"),
    pytest.param(
        (0.5, 0.1, 1.0), [C, C, U, C], [0.5, 0.6, 0.7, 0.6, 0.6], id="one-clean-after-drop-holds"
    ),
    pytest.param(
        (0.5, 0.1, 1.0),
        [C, C, U, C, C],
        [0.5, 0.6, 0.7, 0.6, 0.6, 0.7],
        id="two-clean-after-drop-rise",
    ),
    pytest.param(
        (0.5, 0.1, 1.0),
        [C, C, U, C, U, C, C],
        [0.5, 0.6, 0.7, 0.6, 0.6, 0.5, 0.5, 0.6],
        id="unclean-resets-the-streak",
    ),
    pytest.param(
        (0.5, 0.1, 1.0),
        [C, C, U, C, C, C],
        [0.5, 0.6, 0.7, 0.6, 0.6, 0.7, 0.8],
        id="two-clean-rule-applies-once",
    ),
    pytest.param(
        (0.5, 0.1, 1.0),
        [U, C, C],
        [0.5, 0.5, 0.5, 0.6],
        id="floor-then-needs-two",
    ),
    pytest.param(
        (0.5, 0.1, 1.0),
        [C] * 5 + [U, C, C, C],
        [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 0.9, 0.9, 1.0, 1.0],
        id="drop-from-ceiling",
    ),
]


@pytest.mark.parametrize(("params", "outcomes", "caps"), CASES)
def test_cap_sequences(
    params: tuple[float, float, float], outcomes: list[bool], caps: list[float]
) -> None:
    sched = CapSchedule(*params)
    seen = [sched.cap_mps]
    for clean in outcomes:
        seen.append(sched.finish_lap(clean))
        assert sched.cap_mps == seen[-1]
    assert seen == pytest.approx(caps)


def test_cap_does_not_change_between_finish_calls() -> None:
    sched = CapSchedule(0.5, 0.1, 1.0)
    sched.finish_lap(True)
    assert {sched.cap_mps for _ in range(100)} == {sched.cap_mps}


def test_no_float_drift_after_many_laps() -> None:
    sched = CapSchedule(0.5, 0.1, 5.0)
    for _ in range(45):
        sched.finish_lap(True)
    assert sched.cap_mps == pytest.approx(5.0, abs=0.0)


@pytest.mark.parametrize(
    ("start", "step", "ceiling"), [(0.0, 0.1, 1.0), (0.5, 0.0, 1.0), (0.5, 0.1, 0.4)]
)
def test_bad_parameters_are_refused(start: float, step: float, ceiling: float) -> None:
    with pytest.raises(ValueError):
        CapSchedule(start, step, ceiling)


# Each clean criterion failing on its own (all the others satisfied).
@pytest.mark.parametrize(
    ("brakes", "non_driving", "clearance", "expected"),
    [
        (0, False, 0.30, True),
        (0, False, 0.08, True),  # at the threshold counts
        (1, False, 0.30, False),  # a safety brake
        (3, False, 0.30, False),
        (0, True, 0.30, False),  # planner left DRIVING
        (0, False, 0.0799, False),  # too close to a wall
        (0, False, -0.01, False),  # plan through a wall
        (0, False, math.nan, False),  # no clearance data: cannot verify
    ],
)
def test_clean_criteria(brakes: int, non_driving: bool, clearance: float, expected: bool) -> None:
    assert lap_is_clean(brakes, non_driving, clearance, 0.08) is expected
