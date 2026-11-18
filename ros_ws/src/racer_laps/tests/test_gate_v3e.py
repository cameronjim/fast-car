"""L1, brief 50e: the alias verdict is a majority over the life of the reference, held and not for
good; proof is the plain majority; nothing closes by the start scan while the majority is
ambiguous."""

from __future__ import annotations

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_laps.gate import GateConfig
from racer_laps.ring import is_aliased, is_proven
from synth import WHEELBASE_M
from test_gate import result
from test_gate_v3_ring import run_flagged

CFG = GateConfig(wheelbase_m=WHEELBASE_M)


def aliased(u: int, a: int) -> bool:
    return is_aliased(u, a, CFG.alias_min_scans, CFG.alias_max_share)


def test_the_constants_are_the_ones_written_in_the_brief() -> None:
    assert (CFG.alias_min_scans, CFG.alias_max_share) == (6, 0.75)
    assert CFG.alias_release_share == 0.5  # type: ignore[attr-defined]
    assert (CFG.proof_min_unique, CFG.alias_gap, CFG.unique_gap) == (4, 0.025, 0.0325)


@pytest.mark.parametrize(
    ("u", "a", "is_al", "is_pr"),
    [
        (5, 3, False, True),  # a placed track at speed
        (1, 3, False, False),
        (0, 5, False, False),
        (0, 6, True, False),
        (2, 6, False, False),  # 0.75 is not over 0.75
        (2, 7, True, False),
        (4, 2, False, True),  # 50d held it
        (4, 4, False, False),
        (3, 0, False, False),
    ],
)
def test_the_verdict_table(u: int, a: int, is_al: bool, is_pr: bool) -> None:
    assert aliased(u, a) is is_al
    assert is_proven(u, a, CFG.proof_min_unique, 0.0, CFG.unique_shift_m) is is_pr  # type: ignore[call-arg]


def test_a_stadium_at_speed_with_5_unique_and_3_ambiguous_is_never_aliased(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Finding 3 of round 4, scripted in the order of the L3 run (L1 cannot reproduce the node's
    scan timing): three very low gaps among the first matches, then unique ones."""
    closed, gate = run_flagged(monkeypatch, [True, False, True, False, True, False, True, True])
    assert gate.start_verdict == "proven" and not gate.start_aliased
    assert closed is not None and closed.line_source == "start_scan"  # type: ignore[attr-defined]
    assert closed.length_m == pytest.approx(5.2, abs=0.05)  # type: ignore[attr-defined]


# -- release -------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("n_unique", "verdict"),
    [(0, "aliased"), (5, "aliased"), (6, "unproven"), (7, "proven")],
)
def test_an_aliased_start_scan_is_released_when_unique_matches_catch_up(
    monkeypatch: pytest.MonkeyPatch, n_unique: int, verdict: str
) -> None:
    closed, gate = run_flagged(monkeypatch, [False] * 6 + [True] * n_unique, step=0.02)
    assert gate.start_verdict == verdict
    if verdict == "aliased":
        assert closed is None
        assert gate.aliased_released_at_scan is None  # type: ignore[attr-defined]
    else:
        assert gate.aliased_entered_at_scan == 6  # type: ignore[attr-defined]
        assert gate.aliased_released_at_scan == 12  # type: ignore[attr-defined]
        assert gate.aliased_entries == 1
        assert gate.aliased_releases == 1
    if verdict == "proven":
        assert closed is not None and closed.line_source == "start_scan"  # type: ignore[attr-defined]


def test_one_unique_match_in_ten_never_releases_an_aliased_start_scan(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    flags = [False] * 6 + ([False] * 9 + [True]) * 20
    closed, gate = run_flagged(monkeypatch, flags, step=0.003)
    assert gate.start_aliased and closed is None
    assert gate.aliased_released_at_scan is None  # type: ignore[attr-defined]


# -- the hold --------------------------------------------------------------------------------------


def scripted_approach(mp: pytest.MonkeyPatch, flags: list[bool]):  # type: ignore[no-untyped-def]
    """Matches 0.1 m apart ending 0.1 m before the line, each called unique or ambiguous in turn;
    then the scan 0.41 m after the line (the closing one)."""
    from scripted_car import Car
    from test_gate_v3_ring import AMBIGUOUS, UNIQUE

    car = Car(mp, unique_shift_m=0.4)
    it = iter(flags)
    mp.setattr(car.gate, "_classify", lambda *a, **k: UNIQUE if next(it) else AMBIGUOUS)
    length = 5.2
    car.steer_for(length)
    for i in range(len(flags)):
        x = -0.1 * (len(flags) - i)
        car.move_to(length + x)
        car.scan(result(x))
    car.move_to(length + 0.41)
    return car, length


def test_nothing_closes_by_the_start_scan_while_the_majority_is_ambiguous(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """(2, 3) with two candidates: held, and the hold does not run out."""
    car, length = scripted_approach(monkeypatch, [True, False, True, False, False])
    assert car.scan(None) is None and not car.gate.start_aliased and len(car.gate._cands) == 2
    car.move_to(length + 0.41 + 1.6)  # more than candidate_ttl_s later
    assert car.scan(None) is None
    car.move_to(length + 0.41 + 6.0)
    assert car.scan(None) is None and len(car.gate._cands) == 2


def test_a_start_scan_with_a_majority_unique_is_closed_by_v21_after_the_ttl(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """(2, 1): unproven (two unique), not a majority ambiguous: v2.1's rule after the ttl."""
    car, length = scripted_approach(monkeypatch, [True, False, True])
    assert car.scan(None) is None
    car.move_to(length + 0.41 + 1.6)
    closed = car.scan(None)
    assert closed is not None and closed.line_source == "start_scan"


# -- property --------------------------------------------------------------------------------------


@settings(max_examples=300, deadline=None)
@given(st.lists(st.booleans(), max_size=120))
def test_the_state_changes_only_at_a_counted_match_and_aliased_means_a_majority_ambiguous(
    seq: list[bool],
) -> None:
    from racer_laps.ring import is_released

    u = a = 0
    state = "none"  # "aliased" or "free"
    for unique in seq:
        u, a = (u + 1, a) if unique else (u, a + 1)
        before = state
        if state == "aliased":
            if is_released(u, a, CFG.alias_release_share):  # type: ignore[attr-defined]
                state = "free"
        elif aliased(u, a):
            state = "aliased"
        if state == "aliased":
            assert a > u
        assert not (before == "aliased" and state == "free" and is_proven(u, a, 4, 0.0, 0.4)), (
            u,
            a,
        )
