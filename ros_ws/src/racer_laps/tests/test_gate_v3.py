"""L1: lap gate v3 (brief 50, design 13-plug-and-play.md section 20) on synthetic scans: a ring
(a stadium with no straight, where a scan does not say where along the lap the car is), a ring
with one box, and stadiums. Seeded, no ROS.

Premise (F1): on a ring every scan matches the start scan STRONG, so gate v2.1 closes lap 1 as soon
as its heading slack lets a candidate in, at 86 percent of a lap or earlier. The F1 test below is
written against `GateConfig` defaults only: on main (v2.1) it fails.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest
from racer_laps.engine import LapResult
from racer_laps.gate import GateConfig, StartScanGate
from racer_laps.scan_match import match_scans, scan_to_points
from synth import (
    BODY,
    MOUNT,
    RANGE_MAX_M,
    WHEELBASE_M,
    Stadium,
    drive_stadium,
    scan_at,
)
from v3_support import (
    OFF,
    RING,
    RING_SPEED,
    STADIUM,
    deliver,
    make_mem,
    run,
)

# -- stadium: v3 changes no crossing -------------------------------------------------------------

#: Crossings of two stadium runs of gate v2.1 (main at 71ae56d): (t_cross_s, lap_time_s, length_m).
GOLDEN_V21 = {
    "A": (
        STADIUM,
        1.0,
        False,
        [
            (11.99962312904487, 11.959623129044871, 11.99962312904487),
            (23.974429124419565, 11.974805995374695, 11.974805995374695),
            (35.949855782778926, 11.97542665835936, 11.97542665835936),
            (47.893968025254196, 11.94411224247527, 11.94411224247527),
        ],
    ),
    "B": (
        Stadium(straight_m=2.0, lane_m=1.0, island_r_m=0.4),
        0.8,
        True,
        [
            (12.103082185973069, 12.06308218597307, 9.682465748778576),
            (24.189711638904782, 12.086629452931714, 9.669303562345217),
            (36.250689938479155, 12.060978299574373, 9.64878263965879),
            (48.30012602728681, 12.049436088807653, 9.639548871047165),
        ],
    ),
}


@pytest.mark.parametrize("name", sorted(GOLDEN_V21))
@pytest.mark.parametrize("kw", [{}, OFF], ids=["defaults_on", "ring_and_uniqueness_off"])
def test_stadium_crossings_equal_gate_v21s(name: str, kw: dict) -> None:
    """With the defaults on the crossings are v2.1's (0.02 s); with the ring and the uniqueness
    test off they are v2.1's to rounding."""
    stadium, speed, clockwise, golden = GOLDEN_V21[name]
    mem, laps = run(stadium, 4.3, speed, kw, clockwise=clockwise)
    assert len(laps) == len(golden)
    tol = 0.02 if not kw else 1e-9
    for r, (t, lap_time, length) in zip(laps, golden, strict=True):
        assert r.t_cross_s == pytest.approx(t, abs=tol)
        assert r.lap_time_s == pytest.approx(lap_time, abs=tol)
        assert r.length_m == pytest.approx(length, abs=max(tol, 0.02) if not kw else 1e-9)
        assert r.line_source == "start_scan" and not r.missed_gate
    assert mem.status()["phase_observable"] is True
    assert mem.status()["ring_disagreements"] == 0


def test_with_the_ring_off_the_status_has_no_ring_block() -> None:
    mem, _ = run(STADIUM, 1.2, 1.0, OFF)
    status = mem.status()
    assert "ring" not in status and status["lap_line_sources"] == ["start_scan"]
    mem_on, _ = run(STADIUM, 1.2, 1.0)
    ring = mem_on.status()["ring"]
    assert isinstance(ring, dict) and ring["keyframes"] > 5 and ring["generation"] == 0


# -- the displaced-scan test -----------------------------------------------------------------------


@dataclass
class Setup:
    gate: StartScanGate
    ref_pts: np.ndarray
    pose_xyz: tuple[float, float, float]


def feed_gate(
    stadium: Stadium,
    travel_m: float,
    speed: float = 0.5,
    gate_kw: dict | None = None,
    start_s: float = 0.0,
) -> tuple[StartScanGate, np.ndarray]:
    """A gate that was started at the first scan and driven `travel_m` of the stadium; returns it
    and the last scan's points."""
    gate = StartScanGate(
        GateConfig(
            wheelbase_m=WHEELBASE_M, lidar_x_m=MOUNT.x_m, lidar_y_m=MOUNT.y_m, **(gate_kw or {})
        )
    )
    rng = np.random.default_rng(7)
    walls = stadium.walls()
    last = np.zeros((0, 2))
    n = 0
    for tick in drive_stadium(stadium, 5.0, speed, start_s=start_s, t0=0.0):
        if tick.kind == "drive":
            gate.on_steering(tick.payload[0])
        elif tick.kind == "odom":
            gate.on_odom(tick.t, tick.payload[0])
        else:
            x, y, yaw, stamp = tick.payload
            ranges, a0, da = scan_at(walls, x, y, yaw, rng)
            last = scan_to_points(ranges, a0, da, 0.05, RANGE_MAX_M, MOUNT, BODY)
            if not gate.started:
                assert gate.set_start(stamp + 0.04, last)
            else:
                gate.on_scan(stamp + 0.04, last)
            n += 1
        if gate.started and gate.total_m >= travel_m:
            break
    return gate, last


def match_to_start(gate: StartScanGate, points: np.ndarray, seed_x: float) -> object:
    from racer_laps.scan_match import DENSE_MAX_POINTS, DENSE_VOXEL_M, thin

    res = match_scans(
        thin(points),
        gate._start.pts,
        (seed_x, 0.0, 0.0),
        score_source=thin(points, DENSE_VOXEL_M, DENSE_MAX_POINTS),
        score_target=gate._start.dense,
    )
    assert res is not None
    return res


def test_on_a_ring_the_displaced_scan_makes_a_match_ambiguous() -> None:
    gate, pts = feed_gate(RING, 1.2, gate_kw={"ring_enabled": False})
    res = match_to_start(gate, pts, 0.0)  # the ring: the scan fits the start scan anywhere
    gate._source_dense = gate._scans[-1].dense
    res = _with_violation(gate, res, pts)
    assert not gate._is_unique(res, gate._start, gate.total_m)


def test_on_a_stadium_straight_with_its_ends_in_view_a_match_is_unique() -> None:
    """Near the line of a stadium with a short straight both bends are in view: the scan taken
    0.4 m earlier does not fit the start scan at the pose of this one."""
    gate, pts = feed_gate(STADIUM, 1.2, gate_kw={"ring_enabled": False})
    res = match_to_start(gate, pts, 1.2)
    gate._source_dense = gate._scans[-1].dense
    res = _with_violation(gate, res, pts)
    assert gate._is_unique(res, gate._start, gate.total_m)


def test_with_no_displaced_scan_a_match_is_not_unique() -> None:
    gate, pts = feed_gate(RING, 0.2, gate_kw={"ring_enabled": False})  # inside the first 0.4 m
    res = match_to_start(gate, pts, 0.0)
    res = _with_violation(gate, res, pts)
    assert gate._displaced(gate.total_m) is None
    assert not gate._is_unique(res, gate._start, gate.total_m)


def test_a_displaced_scan_that_is_accepted_but_not_strong_leaves_the_match_unique(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    gate, pts = feed_gate(RING, 1.2, gate_kw={"ring_enabled": False})
    res = match_to_start(gate, pts, 0.0)
    res = _with_violation(gate, res, pts)
    assert not gate._is_unique(res, gate._start, gate.total_m)  # ambiguous as it stands
    monkeypatch.setattr(gate, "_freespace", lambda *a, **k: 0.5)  # the displaced one sees through
    assert gate._is_unique(res, gate._start, gate.total_m)


def _with_violation(gate: StartScanGate, res: object, pts: np.ndarray) -> object:
    from dataclasses import replace

    return replace(res, freespace_violation=gate._freespace(res, pts))  # type: ignore[type-var]


def test_the_scan_buffer_keeps_one_metre_and_survives_standing_still() -> None:
    gate, _ = feed_gate(RING, 2.5, gate_kw={"ring_enabled": False})
    assert 8 <= len(gate._scans) <= 31
    assert gate._scans[-1].total_m - gate._scans[0].total_m <= 1.0 + 1e-9
    n = len(gate._scans)
    pts = gate._scans[-1].points
    t = gate._hist[-1].t
    for k in range(1, 6):  # the car stops
        gate.on_odom(t + 0.02 * k, 0.0)
    t += 0.1
    for _k in range(40):  # standing: the wheel total does not move, nothing is pushed
        for _j in range(5):
            t += 0.02
            gate.on_odom(t, 0.0)
        gate.on_scan(t, pts)
    assert len(gate._scans) in (n, n + 1)  # at most the scan that arrived as the car stopped


# -- ring with one box ---------------------------------------------------------------------------

BOX_RING = Stadium(straight_m=0.0, lane_m=1.2, island_r_m=0.55, box=(0.5, 0.20, 0.30))


def test_a_ring_with_one_box_winds_lap_1_then_closes_by_the_anchor() -> None:
    """The box is what tells places apart: lap 1 winds, from lap 2 the keyframes that see the box
    give fixes and the anchor places the line. Seeded with the odometry's yaw prediction
    (`kf_seed_yaw`, see GateConfig): the registered zero-yaw seed does worse here, see the next
    test."""
    mem, laps = run(BOX_RING, 6.2, RING_SPEED, {"kf_seed_yaw": True})
    truth = BOX_RING.length_m
    assert len(laps) == 6
    assert laps[0].line_source == "winding"
    assert [r.line_source for r in laps[1:]] == ["anchor"] * 5
    for r in laps[1:]:
        assert r.length_m == pytest.approx(truth, rel=0.005), r
        assert not r.missed_gate
    status = mem.status()
    assert status["phase_observable"] is True
    assert status["ring"]["fixes_last_lap"] >= 1  # type: ignore[index]
    assert status["ring_disagreements"] == 0
    assert status["missed_gate_laps"] == []


def test_the_registered_zero_yaw_seed_on_the_box_ring_places_anchors_about_2_percent_short() -> (
    None
):
    """The registered keyframe seed has zero yaw. On a curved ring the ICP then pulls the along
    offset toward zero (a displacement d is read as about d / 3), so the fixes, and the anchor
    laps, are early by 0.1 to 0.2 m and later laps lose the keyframes (the station is shifted) and
    fall back to winding. Every lap is still within 3 percent; this is the number to know."""
    mem, laps = run(BOX_RING, 6.2, RING_SPEED)
    truth = BOX_RING.length_m
    assert len(laps) == 6
    assert [r.line_source for r in laps[:2]] == ["winding", "anchor"]
    assert laps[1].length_m == pytest.approx(truth, rel=0.03)
    assert laps[1].length_m < truth * 0.99  # early, not on the truth
    for r in laps:
        assert r.length_m == pytest.approx(truth, rel=0.03), r
        assert not r.missed_gate
    assert mem.status()["missed_gate_laps"] == []


def test_with_the_ring_off_the_same_lap_is_lost_and_nothing_is_invented() -> None:
    """v2.1 behaviour: a missed gate, never a lap made up (winding does not fire on a stadium,
    where the start scan is not aliased, and the anchor is off)."""
    truth_lap = STADIUM.length_m
    window = (3.0 * truth_lap - 0.9, 3.0 * truth_lap + 0.6)
    mem = make_mem(**OFF)
    rng = np.random.default_rng(3)
    walls = STADIUM.walls()
    events = []
    for tick in drive_stadium(STADIUM, 4.2, speed_mps=1.0, t0=0.0):
        late = 0.35 if tick.kind == "odom" and window[0] <= tick.t <= window[1] else 0.0
        events.append((tick.t + late, 0 if tick.kind != "scan" else 1, tick))
    events.sort(key=lambda e: (e[0], e[1]))
    out: list[LapResult] = []
    for _t, _k, tick in events:
        deliver(mem, tick, walls, rng, out)
    assert [r.lap_index for r in out] == [1, 2, 3]  # lap 3 is the one that spans two true laps
    assert out[2].missed_gate
    assert all(r.line_source == "start_scan" for r in out)
    assert out[2].length_m > 1.2 * truth_lap  # not a lap: it spans the lost line
