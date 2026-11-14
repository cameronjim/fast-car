"""L1 tests of run finding and the intervention counting rule (pre-registration section 4.2),
on hand-built event lists and on the real shapes of run R3 of bag 2026-11-13T22-20-18."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from racer_eval.interventions import (
    count_interventions,
    starts_with_event,
    total,
)
from racer_eval.recording import BagData
from racer_eval.runs import (
    Run,
    count_state_entries,
    find_runs,
    longest_state_hold,
)

FIXTURE = Path(__file__).parent / "fixtures" / "r3_22-20-18.npz"


def test_find_pnp_runs_splits_on_a_gap_above_two_seconds() -> None:
    status_t = [0.0, 0.5, 1.0, 1.5, 3.5, 3.6, 10.0, 10.5]  # 2.0 s gap stays, 6.4 s gap splits
    draw_t = [0.0, 0.5, 1.0, 1.5, 3.5, 10.2, 10.5]
    draw_v = [0.0, 0.0, 0.3, 0.3, 0.3, 0.2, 0.2]
    runs = find_runs(draw_t, draw_v, status_t)
    assert [(r.run_id, r.t_start, r.t_first_cmd, r.t_end) for r in runs] == [
        ("R0", 0.0, 1.0, 3.6),
        ("R1", 10.0, 10.2, 10.5),
    ]
    assert runs[0].counting_window == (1.0, 3.6)
    assert runs[0].duration_s == pytest.approx(2.6)
    assert runs[1].kind == "pnp"


def test_a_pnp_run_without_a_non_zero_command_has_no_window() -> None:
    (run,) = find_runs([0.0, 0.5], [0.0, 0.0], [0.0, 0.5])
    assert run.t_first_cmd is None
    assert run.counting_window is None
    assert run.duration_s == 0.0


def test_negative_commands_count_as_non_zero() -> None:
    (run,) = find_runs([0.0, 0.5, 1.0], [0.0, -0.3, 0.0], [0.0, 0.5, 1.0])
    assert run.t_first_cmd == 0.5


def test_find_drive_raw_runs_for_other_drivers() -> None:
    draw_t = [5.0, 5.5, 6.0, 20.0, 20.5]
    draw_v = [0.3, 0.3, 0.0, 0.2, 0.2]
    runs = find_runs(draw_t, draw_v, prefix="D")
    assert [(r.run_id, r.kind, r.t_first_cmd, r.t_end) for r in runs] == [
        ("D0", "drive_raw", 5.0, 5.5),
        ("D1", "drive_raw", 20.0, 20.5),
    ]


def test_find_runs_empty_and_shape_error() -> None:
    assert find_runs([], [], None) == []
    assert find_runs([], [], []) == []
    with pytest.raises(ValueError):
        find_runs([0.0, 1.0], [0.0], None)


def test_longest_state_hold_and_recovery_entries() -> None:
    t = [0, 1, 2, 3, 4, 5]
    assert longest_state_hold(t, [0, 5, 5, 5, 0, 5]) == 3.0
    assert longest_state_hold(t, [0, 0, 0, 0, 0, 0]) == 0.0
    assert longest_state_hold(t, [5, 5, 5, 5, 5, 5]) == 5.0  # held to the last message
    assert longest_state_hold([], []) == 0.0
    assert count_state_entries([7, 7, 0, 7, 0, 0]) == 2
    assert count_state_entries([0, 7, 7]) == 1
    assert count_state_entries([]) == 0


RUN = Run("R", "pnp", 0.0, 10.0, 50.0)


def test_counting_rule_edges_and_sources() -> None:
    """Window [10, 50], both edges included. Excluded: before the first command (5.0), after the
    last status (50.01, the stop-mechanism watchdog), and every RELEASE record."""
    ev = [
        (5.0, 0, "watchdog"),  # before the first command
        (10.0, 0, "watchdog"),  # exactly on the first command
        (11.9, 0, "rate_limit"),
        (20.0, 1, "watchdog"),  # RELEASE, never counted
        (30.0, 0, "ttc"),
        (31.0, 0, "ttc_reverse"),
        (32.0, 0, "command_sanity"),
        (50.0, 0, "watchdog"),  # exactly on the last status
        (50.01, 0, "watchdog"),  # after the last status
    ]
    t, phase, src = (list(x) for x in zip(*ev, strict=True))
    r = count_interventions(RUN, t, phase, src)
    assert r.counts["watchdog"] == 2
    assert r.counts["rate_limit"] == 1
    assert r.count("ttc", "ttc_reverse") == r.obstacle == 2
    assert r.counts["command_sanity"] == 1
    assert r.duration_s == 40.0
    assert r.per_minute["watchdog"] == pytest.approx(2 / (40.0 / 60.0))
    assert r.start_event is True  # the watchdog exactly on the first command


def test_start_event_window_is_two_seconds_inclusive() -> None:
    inside = count_interventions(RUN, [12.0], [0], ["rate_limit"])
    outside = count_interventions(RUN, [12.01], [0], ["rate_limit"])
    other = count_interventions(RUN, [10.5], [0], ["ttc"])
    released = count_interventions(RUN, [10.5], [1], ["watchdog"])
    assert inside.start_event and not outside.start_event
    assert not other.start_event  # only watchdog and rate_limit count for the start rule
    assert not released.start_event


def test_start_window_is_clipped_to_the_run() -> None:
    short = Run("S", "pnp", 0.0, 10.0, 11.0)
    r = count_interventions(short, [11.5], [0], ["watchdog"])
    assert r.counts["watchdog"] == 0 and not r.start_event


def test_run_without_a_command_counts_nothing() -> None:
    r = count_interventions(Run("N", "pnp", 0.0, None, 5.0), [1.0], [0], ["watchdog"])
    assert not r.commanded and r.duration_s == 0.0 and r.counts == {}
    assert starts_with_event([r]) == (0, 0)


def test_zero_duration_rate_is_nan_and_totals_add_up() -> None:
    zero = count_interventions(Run("Z", "pnp", 0.0, 3.0, 3.0), [3.0], [0], ["watchdog"])
    assert np.isnan(zero.per_minute["watchdog"])
    a = count_interventions(RUN, [11.0, 12.5], [0, 0], ["watchdog", "rate_limit"])
    b = count_interventions(RUN, [], [], [])
    tot = total([a, b])
    assert tot.run_id == "total" and tot.duration_s == 80.0
    assert tot.count("watchdog") == 1 and tot.count("rate_limit") == 1
    assert starts_with_event([a, b]) == (1, 2)


def test_fixture_is_small_and_has_the_real_shapes() -> None:
    assert FIXTURE.stat().st_size < 400_000
    data = BagData.load(FIXTURE)
    assert data.name.endswith("R3")
    assert len(data.st_t) > 1000 and len(data.ev_t) == 16


def test_counting_rule_on_run_r3_of_the_real_bag() -> None:
    """R3 (2729.7 to 2786.3 s) has 16 safety records in the fixture. Before the first command:
    a rate_limit engage and release at 2728.9 s (not counted). In the window: watchdog engages at
    2730.38, 2730.46 and 2786.28 (0.005 s before the last status, so counted), rate_limit
    engages at 2736.19 and 2753.96. After the last status: a rate_limit engage 0.007 s later and
    a watchdog engage 0.105 s later (the stop mechanism, not counted)."""
    data = BagData.load(FIXTURE)
    runs = find_runs(data.draw_t, data.draw_speed, data.st_t)
    assert len(runs) == 1
    r = count_interventions(runs[0], data.ev_t, data.ev_phase, data.ev_source)
    assert r.counts["watchdog"] == 3
    assert r.counts["rate_limit"] == 2
    assert r.obstacle == 0
    assert r.start_event is True  # the watchdog at 2730.38 s is 0.68 s after the first command
    assert r.duration_s == pytest.approx(56.6, abs=0.1)
