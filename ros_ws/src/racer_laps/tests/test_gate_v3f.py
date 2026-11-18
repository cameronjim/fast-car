"""L1, brief 50f: proof by span (a place stayed distinct over a displacement of its own size) and
the lap 1 of a stadium at 3 m/s on time."""

from __future__ import annotations

import pytest
from racer_laps.gate import GateConfig
from racer_laps.ring import is_proven
from synth import WHEELBASE_M, Stadium, feed
from test_gate_v3c import BOX_RING, L3_STADIUM, lap_s
from v3_support import RING, make_mem

CFG = GateConfig(wheelbase_m=WHEELBASE_M)


@pytest.mark.parametrize(
    ("u", "a", "span", "proven"),
    [
        (3, 0, 0.6, True),  # three looks 0.6 m apart
        (3, 0, 0.3, False),
        (3, 1, 0.6, False),  # one ambiguous match: the count rule only
        (2, 0, 0.9, False),
        (4, 0, 0.1, True),  # four matches, as before
        (4, 3, 2.0, True),
        (4, 4, 2.0, False),
        (3, 0, 0.4, True),  # the span is at least unique_shift_m
    ],
)
def test_proof_is_four_unique_or_three_unique_none_ambiguous_over_0_4_m(
    u: int, a: int, span: float, proven: bool
) -> None:
    assert is_proven(u, a, CFG.proof_min_unique, span, CFG.unique_shift_m) is proven  # type: ignore[call-arg]


@pytest.mark.parametrize("speed", [3.12, 3.22])
def test_lap_1_of_a_stadium_at_3_m_per_s_closes_without_the_hold(speed: float) -> None:
    """Seed 7 (the builder's reproduction of the L3 flake): three unique matches on the first
    approach, 0.6 m apart, and lap 1's event came 1.4 s after the crossing."""
    closed_at: list[float] = []

    def watch(mem: object, tick: object) -> None:
        if mem.laps_completed > len(closed_at):  # type: ignore[attr-defined]
            closed_at.append(tick.t)  # type: ignore[attr-defined]

    mem = make_mem()
    laps = feed(mem, L3_STADIUM, 2.4, speed_mps=speed, t0=0.0, seed=7, on_tick=watch)
    truth = lap_s(L3_STADIUM, speed)
    assert len(laps) >= 1 and closed_at
    assert abs(laps[0].lap_time_s - truth) < 0.05  # type: ignore[attr-defined]
    assert closed_at[0] - truth < 0.3, closed_at[0] - truth


@pytest.mark.parametrize(
    ("label", "stadium", "speed", "clockwise"),
    [
        ("plain ring 0.5 m/s", RING, 0.5, False),
        ("plain ring 3 m/s", RING, 3.0, False),
        ("box ring ccw", BOX_RING, 0.5, False),
    ],
)
def test_a_ring_is_never_proven_at_any_scan(
    label: str, stadium: Stadium, speed: float, clockwise: bool
) -> None:
    seen: list[bool] = []

    def watch(mem: object, tick: object) -> None:
        seen.append(bool(mem.gate.start_proven))  # type: ignore[attr-defined]

    mem = make_mem()
    feed(mem, stadium, 3.4, speed_mps=speed, t0=0.0, clockwise=clockwise, on_tick=watch)
    assert seen and not any(seen), label
