"""L1, brief 50d: the dead band of the alias verdict (`alias_gap`) and the share rule of proof."""

from __future__ import annotations

import pytest
import racer_laps.gate as gate_module
from racer_laps.gate import AMBIGUOUS, UNIQUE, UNKNOWN, GateConfig
from racer_laps.scan_match import MatchResult
from synth import WHEELBASE_M, Stadium
from test_gate_v2 import ISLAND_R_M, LANE_M, LAPS, feed, make_mem, truth_lap_times
from test_gate_v3 import feed_gate
from test_gate_v3_ring import run_flagged
from test_gate_v3c import classified, same
from v3_support import RING

# -- classification: three classes ----------------------------------------------------------------


@pytest.mark.parametrize(
    ("label", "violation", "expected"),
    [
        ("gap 0.0249 is ambiguous", 0.0249, AMBIGUOUS),
        ("gap 0.025 is in the band", 0.025, UNKNOWN),
        ("gap 0.029 (the smallest of the 5 m stadium)", 0.029, UNKNOWN),
        ("gap 0.031 (the largest alias of RD)", 0.031, UNKNOWN),
        ("gap 0.0324", 0.0324, UNKNOWN),
        ("gap 0.0325 is unique", 0.0325, UNIQUE),
    ],
)
def test_the_dead_band_between_alias_gap_and_unique_gap_is_unknown(
    monkeypatch: pytest.MonkeyPatch, label: str, violation: float, expected: str
) -> None:
    assert classified(monkeypatch, same(), 0.0, violation) == expected, label


def test_the_band_is_reported_apart_from_the_two_classes() -> None:
    gate, _ = feed_gate(RING, 1.2, gate_kw={"ring_enabled": True})
    status = gate._v3_status()
    assert hasattr(gate, "gaps_unknown")
    assert status["unknown_gap_count"] == len(gate.gaps_unknown)  # type: ignore[attr-defined]


def test_alias_gap_is_a_checked_constant() -> None:
    cfg = GateConfig(wheelbase_m=WHEELBASE_M)
    assert cfg.alias_gap == 0.025  # type: ignore[attr-defined]
    for bad in (-0.001, 0.0, cfg.unique_gap + 0.001):
        with pytest.raises(ValueError):
            GateConfig(wheelbase_m=WHEELBASE_M, alias_gap=bad)  # type: ignore[call-arg]
    GateConfig(wheelbase_m=WHEELBASE_M, alias_gap=cfg.unique_gap)  # type: ignore[call-arg]


@pytest.mark.parametrize(
    ("violation", "expected"),
    [(0.0249, AMBIGUOUS), (0.029, AMBIGUOUS), (0.0324, AMBIGUOUS), (0.0325, UNIQUE)],
)
def test_alias_gap_equal_to_unique_gap_is_round_3(
    monkeypatch: pytest.MonkeyPatch, violation: float, expected: str
) -> None:
    gate, _ = feed_gate(RING, 1.2, gate_kw={"ring_enabled": False, "alias_gap": 0.0325})
    assert gate.cfg.alias_gap == gate.cfg.unique_gap  # type: ignore[attr-defined]
    res = MatchResult(0.1, 0.0, 0.0, 0.02, 0.9, 200, 0.0)
    monkeypatch.setattr(gate_module, "match_scans", lambda *a, **k: same())
    monkeypatch.setattr(gate, "_freespace", lambda *a, **k: violation)
    assert gate._classify(res, gate._start, gate.total_m) == expected


# -- the stadium with a 5 m straight, through the v3 defaults -----------------------------------


@pytest.mark.parametrize("scale", [0.95, 1.0, 1.05])
@pytest.mark.parametrize("clockwise", [False, True])
@pytest.mark.parametrize("rising", [False, True], ids=["0.5mps", "rising-to-1.2mps"])
def test_the_5_m_stadium_is_never_aliased_and_closes_every_lap_by_start_scan(
    clockwise: bool, rising: bool, scale: float
) -> None:
    stadium = Stadium(straight_m=5.0, lane_m=LANE_M, island_r_m=ISLAND_R_M)
    step = 0.1 if rising else 0.0
    mem = make_mem(gain=1.1)
    laps = feed(
        mem,
        stadium,
        laps=LAPS + 0.2,
        speed_mps=0.5,
        seed=50 + int(clockwise),
        clockwise=clockwise,
        odom_scale=scale,
        speed_step_per_lap_mps=step,
        wobble_m=0.1,
    )
    gate = mem.gate
    assert gate is not None
    assert gate.status()["aliased_entries"] == 0  # never aliased at any scan
    n = len(gate.gaps_unique) + len(gate.gaps_ambiguous)
    assert len(gate.gaps_ambiguous) <= 0.25 * max(n, 1)
    assert [r.lap_index for r in laps] == list(range(1, LAPS + 1))  # type: ignore[attr-defined]
    assert set(gate.line_sources) == {"start_scan"}


def test_the_5_m_stadium_with_unique_gap_0_035_is_not_latched() -> None:
    stadium = Stadium(straight_m=5.0, lane_m=LANE_M, island_r_m=ISLAND_R_M)
    mem = make_mem(gain=1.1, unique_gap=0.035)
    laps = feed(
        mem,
        stadium,
        laps=LAPS + 0.2,
        speed_mps=0.5,
        seed=51,
        clockwise=True,
        odom_scale=1.05,
        speed_step_per_lap_mps=0.1,
        wobble_m=0.1,
    )
    gate = mem.gate
    assert gate is not None and not gate.start_aliased
    assert [r.lap_index for r in laps] == list(range(1, LAPS + 1))  # type: ignore[attr-defined]
    truth = truth_lap_times(stadium, 0.5, 0.1)
    for r, t in zip(laps, truth, strict=True):
        assert abs(r.lap_time_s - t) < 0.3, (r.lap_index, r.lap_time_s, t)  # type: ignore[attr-defined]


# -- proof needs the share too ------------------------------------------------------------------


@pytest.mark.parametrize(
    ("n_unique", "n_ambiguous", "proven"),
    [(4, 2, False), (4, 1, True), (4, 0, True), (8, 2, True), (3, 0, False), (9, 3, True)],
)
def test_proof_is_unique_matches_and_a_quarter_at_most_ambiguous(
    n_unique: int, n_ambiguous: int, proven: bool
) -> None:
    from racer_laps.ring import is_proven

    assert is_proven(n_unique, n_ambiguous, 4, 0.25) is proven


def test_a_start_scan_with_4_unique_and_2_ambiguous_holds_its_candidates(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    closed, gate = run_flagged(monkeypatch, [True] * 4 + [False] * 2)
    assert gate.start_verdict == "unproven"
    assert closed is None
    assert not gate.start_aliased
    assert len(gate._cands) == 4
