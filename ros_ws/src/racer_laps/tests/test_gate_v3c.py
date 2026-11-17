"""L1, brief 50c: the margin of the uniqueness test, the choice of the displaced scan, speed, and
the box ring."""

from __future__ import annotations

from collections import deque

import numpy as np
import pytest
import racer_laps.gate as gate_module
from racer_laps.gate import AMBIGUOUS, UNIQUE, UNKNOWN, GateConfig, StartScanGate, _ScanRec
from racer_laps.scan_match import MatchResult
from synth import WHEELBASE_M, Stadium
from test_gate_v3 import feed_gate
from v3_support import RING, STADIUM, lap_s, run

RING_GATE_KW = {"ring_enabled": False}


def classified(
    monkeypatch: pytest.MonkeyPatch,
    displaced: MatchResult | None,
    own: float,
    violation: float,
    displaced_available: bool = True,
) -> str:
    """`_classify` of a match with free-space violation `own`, the displaced scan matching as
    `displaced` says (None: the ICP gives nothing) with violation `violation`."""
    gate, _pts = feed_gate(RING, 1.2, gate_kw=RING_GATE_KW)
    res = MatchResult(0.1, 0.0, 0.0, 0.02, 0.9, 200, own)
    monkeypatch.setattr(gate_module, "match_scans", lambda *a, **k: displaced)
    monkeypatch.setattr(gate, "_freespace", lambda *a, **k: violation)
    if not displaced_available:
        gate._scans = deque()
    return gate._classify(res, gate._start, gate.total_m)


def same(dx: float = 0.0, **kw: float) -> MatchResult:
    return MatchResult(0.1 + dx, 0.0, 0.0, kw.get("rms", 0.02), kw.get("share", 0.9), 200)


@pytest.mark.parametrize(
    ("label", "displaced", "own", "violation", "expected"),
    [
        ("displaced not accepted (rms)", same(rms=0.05), 0.0, 0.3, UNIQUE),
        ("displaced not accepted (inliers)", same(share=0.3), 0.0, 0.3, UNIQUE),
        ("displaced gives nothing", None, 0.0, 0.3, UNIQUE),
        ("accepted 0.25 m along", same(dx=0.25), 0.0, 0.0, UNIQUE),
        ("accepted just inside unique_same_m, no gap", same(dx=0.19), 0.0, 0.0, AMBIGUOUS),
        ("gap 0.032", same(), 0.010, 0.042, AMBIGUOUS),
        ("gap 0.0325", same(), 0.010, 0.0425, UNIQUE),
        ("gap 0.033", same(), 0.010, 0.043, UNIQUE),
        ("gap 0.010, displaced not STRONG (the RD case)", same(), 0.015, 0.025, AMBIGUOUS),
        ("gap 0.031, the largest alias of RD", same(), 0.000, 0.031, AMBIGUOUS),
        ("gap 0.040, the smallest real place", same(), 0.000, 0.040, UNIQUE),
        ("displaced STRONG and gap negative", same(), 0.010, 0.0, AMBIGUOUS),
        ("a stadium: gap 0.05 and more", same(), 0.0, 0.05, UNIQUE),
    ],
)
def test_the_classification_table(
    monkeypatch: pytest.MonkeyPatch,
    label: str,
    displaced: MatchResult | None,
    own: float,
    violation: float,
    expected: str,
) -> None:
    assert classified(monkeypatch, displaced, own, violation) == expected, label


def test_no_displaced_scan_is_unknown(monkeypatch: pytest.MonkeyPatch) -> None:
    assert classified(monkeypatch, same(), 0.0, 0.1, displaced_available=False) == UNKNOWN


def test_unique_gap_is_a_checked_constant() -> None:
    assert GateConfig(wheelbase_m=WHEELBASE_M).unique_gap == 0.0325
    with pytest.raises(ValueError):
        GateConfig(wheelbase_m=WHEELBASE_M, unique_gap=-0.001)


def test_the_gaps_of_the_start_scan_matches_are_reported() -> None:
    gate, _ = feed_gate(RING, 1.2, gate_kw={"ring_enabled": True})
    gate.gaps_ambiguous.clear()
    gate.gaps_unique.clear()
    res = MatchResult(0.0, 0.0, 0.0, 0.02, 0.9, 200, 0.0)
    gate._classify(res, gate._start, gate.total_m)
    assert len(gate.gaps_ambiguous) == 1 and gate.gaps_unique == []
    assert gate.gaps_ambiguous[0] < 0.0325  # a ring: the displaced scan fits as well as the match
    status = gate._v3_status()
    assert status["ambiguous_gap_max"] == pytest.approx(gate.gaps_ambiguous[0])
    assert np.isnan(status["unique_gap_min"])  # type: ignore[arg-type]


# -- the displaced scan -------------------------------------------------------------------------------


def gate_with_scans(spacing: float, n: int = 40) -> tuple[StartScanGate, float]:
    gate = StartScanGate(GateConfig(wheelbase_m=WHEELBASE_M, **RING_GATE_KW))  # type: ignore[arg-type]
    pts = np.zeros((20, 2))
    totals = [spacing * k for k in range(n)]
    gate._scans = deque(_ScanRec(t, pts, pts, pts) for t in totals if totals[-1] - t <= 1.0)
    return gate, totals[-1]


@pytest.mark.parametrize(
    ("spacing", "behind"),
    [(0.05, 0.40), (0.13, 0.39), (0.25, 0.50), (0.30, 0.60), (0.10, 0.40), (0.20, 0.40)],
)
def test_the_displaced_scan_is_the_nearest_to_0_4_m_back_among_those_more_than_0_3_back(
    spacing: float, behind: float
) -> None:
    gate, now = gate_with_scans(spacing)
    rec = gate._displaced(now)
    assert rec is not None
    assert now - rec.total_m == pytest.approx(behind, abs=1e-9)


def test_under_0_3_m_of_travel_there_is_no_displaced_scan() -> None:
    gate, now = gate_with_scans(0.05, n=6)  # 0.25 m of travel in the buffer
    assert gate._displaced(now) is None
    gate, now = gate_with_scans(0.05, n=8)  # 0.35 m
    assert gate._displaced(now) is not None


# -- speed ----------------------------------------------------------------------------------------------


@pytest.mark.parametrize("speed", [2.5, 3.0])
def test_a_ring_above_2_m_per_s_is_wound_every_lap(speed: float) -> None:
    mem, laps = run(RING, 3.4, speed)
    assert [r.lap_index for r in laps] == [1, 2, 3]
    assert {r.line_source for r in laps} == {"winding"}
    truth = lap_s(RING, speed)
    for r in laps:
        assert abs(r.lap_time_s - truth) <= max(0.3, 0.04 * truth), r.lap_time_s
    assert mem.status()["start_verdict"] == "aliased"


@pytest.mark.parametrize("speed", [2.5, 3.0])
def test_a_stadium_above_2_m_per_s_is_proven_and_never_wound(speed: float) -> None:
    mem, laps = run(STADIUM, 3.4, speed)
    assert {r.line_source for r in laps} == {"start_scan"}
    assert mem.status()["start_verdict"] == "proven"
    assert mem.status()["start_alias_share_last_lap"] == 0.0


# -- the box ring ---------------------------------------------------------------------------------------

#: RD's geometry: a ring with one box 0.20 m into a 1.2 m lane, at 0.35 of the lap. Counter-clockwise
#: the start scan sees it at grazing range, clockwise it is out of sight.
BOX_RING = Stadium(straight_m=0.0, lane_m=1.2, island_r_m=0.50, box=(0.35, 0.20, 0.30))


@pytest.mark.parametrize("clockwise", [False, True])
def test_a_ring_with_a_box_the_start_scan_may_see_is_latched_aliased_before_any_closure(
    clockwise: bool,
) -> None:
    mem, laps = run(BOX_RING, 3.4, 0.5, clockwise=clockwise)
    assert [r.line_source for r in laps] == ["winding"] * 3
    status = mem.status()
    assert status["lap_start_verdicts"][0] == "aliased"  # type: ignore[index]
    assert status["start_verdict"] == "aliased" and status["missed_gate_laps"] == []
    truth = BOX_RING.length_m / 0.5
    for r in laps:
        assert abs(r.lap_time_s - truth) <= max(0.3, 0.04 * truth), r.lap_time_s
