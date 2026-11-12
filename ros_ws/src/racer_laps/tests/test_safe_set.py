"""L1: resampling by arc length, v_ref and cost_to_go, clean filtering, save and load."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from racer_laps.lap_record import LapRecord
from racer_laps.safe_set import SPACING_M, SafeSet, resample_lap


def straight_lap(
    speed: float,
    length: float = 6.0,
    clean: bool = True,
    rate_hz: float = 20.0,
    lap_index: int = 1,
    dwell_s: float = 0.0,
) -> LapRecord:
    """A lap along +x at constant `speed`, optionally standing still for `dwell_s` first."""
    n_dwell = int(dwell_s * rate_hz)
    n_move = int(length / speed * rate_hz)
    t = np.arange(n_dwell + n_move + 1) / rate_hz
    x = np.concatenate([np.zeros(n_dwell), np.arange(n_move + 1) * speed / rate_hz])
    lap_time = float(t[-1] + 0.5 / rate_hz)
    n = t.shape[0]
    return LapRecord(
        lap_index=lap_index,
        lap_time_s=lap_time,
        length_m=float(x[-1]) + 0.01,
        cap_mps=speed,
        brake_count=0 if clean else 1,
        non_driving_seen=False,
        min_clearance_m=0.3,
        clean=clean,
        t_s=t,
        x_m=x,
        y_m=np.zeros(n),
        yaw_rad=np.zeros(n),
        wheel_speed_mps=np.full(n, speed),
        cmd_speed_mps=np.full(n, speed),
        cmd_steering_rad=np.zeros(n),
        planner_state=np.ones(n),
        plan_clearance_m=np.full(n, 0.3),
    )


def test_resample_stations_are_every_ten_centimetres() -> None:
    lap = resample_lap(straight_lap(1.0, length=5.0))
    assert lap is not None
    assert SPACING_M == 0.10
    n = lap.n_stations
    assert n == math.floor(lap.length_m / 0.10 + 1e-9) + 1
    assert lap.x_m[0] == pytest.approx(0.0)
    # stations are 0.10 m apart along the path (x is arc length on a straight); the pose path
    # is scaled to the recorded lap length, which is 1 cm longer here
    assert np.diff(lap.x_m) == pytest.approx(np.full(n - 1, 0.10), abs=3e-4)
    assert lap.v_mps == pytest.approx(np.full(n, 1.0))


def test_cost_to_go_decreases_and_equals_lap_time_at_zero() -> None:
    ss = SafeSet()
    rec = straight_lap(1.0)
    assert ss.add_lap(rec)
    assert ss.cost_to_go(0.0) == pytest.approx(rec.lap_time_s, abs=1e-9)
    s = np.arange(0.0, ss.length_m, 0.1)
    c = np.array([ss.cost_to_go(v) for v in s])
    assert np.all(np.diff(c) < 0.0)
    assert c[-1] == pytest.approx(0.0, abs=0.15)  # within one station of the finish
    assert ss.best_lap_time_s == pytest.approx(rec.lap_time_s)


def test_standing_start_gives_nonincreasing_cost_to_go() -> None:
    ss = SafeSet()
    assert ss.add_lap(straight_lap(1.0, dwell_s=1.0))
    c = np.array([ss.cost_to_go(v) for v in np.arange(0.0, ss.length_m, 0.1)])
    assert np.all(np.diff(c) <= 1e-12)


def test_v_ref_is_the_maximum_over_stored_laps() -> None:
    ss = SafeSet()
    ss.add_lap(straight_lap(0.5, length=6.0, lap_index=1))
    assert ss.v_ref(2.0) == pytest.approx(0.5)
    ss.add_lap(straight_lap(0.8, length=6.0, lap_index=2))
    ss.add_lap(straight_lap(0.6, length=6.0, lap_index=3))
    assert ss.v_ref(2.0) == pytest.approx(0.8)
    assert ss.n_laps == 3
    # cost to go is the smallest over laps: the 0.8 m/s lap is the fastest one
    fastest = min(lap.lap_time_s for lap in ss.laps)
    assert ss.cost_to_go(0.0) == pytest.approx(fastest)


def test_laps_of_different_length_cover_the_longer_one() -> None:
    ss = SafeSet()
    ss.add_lap(straight_lap(0.5, length=4.0, lap_index=1))
    ss.add_lap(straight_lap(0.5, length=6.0, lap_index=2))
    assert ss.length_m == pytest.approx(6.0, abs=0.11)
    assert ss.v_ref(5.5) == pytest.approx(0.5)  # only the longer lap reaches it


def test_unclean_laps_are_not_stored() -> None:
    ss = SafeSet()
    assert not ss.add_lap(straight_lap(1.0, clean=False))
    assert ss.n_laps == 0
    assert math.isnan(ss.v_ref(1.0))
    assert math.isnan(ss.best_lap_time_s)
    assert ss.length_m == 0.0


def test_laps_without_motion_are_not_stored() -> None:
    rec = straight_lap(1.0)
    still = LapRecord(**{**rec.__dict__, "x_m": np.zeros_like(rec.x_m)})
    assert resample_lap(still) is None
    one = LapRecord(
        **{
            **rec.__dict__,
            "t_s": rec.t_s[:1],
            "x_m": rec.x_m[:1],
            "y_m": rec.y_m[:1],
            "wheel_speed_mps": rec.wheel_speed_mps[:1],
        }
    )
    assert resample_lap(one) is None
    flat_time = LapRecord(**{**rec.__dict__, "t_s": np.zeros_like(rec.t_s)})
    assert resample_lap(flat_time) is None


def test_queries_outside_the_stored_range_are_nan() -> None:
    ss = SafeSet()
    ss.add_lap(straight_lap(1.0, length=3.0))
    assert math.isnan(ss.v_ref(-0.1))
    assert math.isnan(ss.cost_to_go(ss.length_m + 0.5))
    s, v, c = ss.horizon(ss.length_m + 1.0, 3.0)
    assert s.shape == v.shape == c.shape == (0,)


def test_horizon_returns_stations_ahead() -> None:
    ss = SafeSet()
    ss.add_lap(straight_lap(1.0, length=6.0))
    s, v, c = ss.horizon(1.23, 1.0)
    assert s[0] == pytest.approx(1.3)
    assert s[-1] == pytest.approx(2.2)
    assert np.all(np.diff(c) < 0.0)
    assert v == pytest.approx(np.full(s.shape, 1.0))
    # near the end the horizon is clipped to the stored range
    s2, _, _ = ss.horizon(ss.length_m - 0.5, 3.0)
    assert s2[-1] == pytest.approx(ss.length_m)
    assert SafeSet().horizon(0.0, 3.0)[0].shape == (0,)


def test_save_and_load_round_trip(tmp_path: Path) -> None:
    ss = SafeSet()
    ss.add_lap(straight_lap(0.5, lap_index=1))
    ss.add_lap(straight_lap(0.8, lap_index=2))
    path = tmp_path / "sub" / "safe_set_test.npz"
    ss.save(path)
    assert path.exists() and not (tmp_path / "sub" / "safe_set_test.npz.tmp").exists()
    back = SafeSet.load(path)
    assert back.n_laps == 2
    assert back.spacing_m == ss.spacing_m
    assert back.length_m == ss.length_m
    for s in (0.0, 0.7, 2.5, ss.length_m):
        assert back.v_ref(s) == ss.v_ref(s)
        assert back.cost_to_go(s) == ss.cost_to_go(s)
    for a, b in zip(ss.laps, back.laps, strict=True):
        assert (a.lap_index, a.lap_time_s, a.length_m, a.cap_mps) == (
            b.lap_index,
            b.lap_time_s,
            b.length_m,
            b.cap_mps,
        )
        assert np.array_equal(a.x_m, b.x_m)


def test_empty_set_round_trips(tmp_path: Path) -> None:
    SafeSet().save(tmp_path / "e.npz")
    back = SafeSet.load(tmp_path / "e.npz")
    assert back.n_laps == 0 and back.length_m == 0.0


def test_load_refuses_an_unknown_format(tmp_path: Path) -> None:
    np.savez(tmp_path / "bad.npz", format_version=np.array([99.0]))
    with pytest.raises(ValueError):
        SafeSet.load(tmp_path / "bad.npz")
