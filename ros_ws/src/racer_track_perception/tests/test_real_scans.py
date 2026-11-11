"""L4: 60 real scans (cut from 2026-11-11T03-54-11_car_teleop) through the geometric backend,
against a committed golden output, plus fault injection."""

from __future__ import annotations

import math

import numpy as np
import pytest
from racer_track_perception import core
from racer_track_perception.vehicle import LiveVehicle
from regen_golden import GOLDEN, SCANS, run_geometric

TOLERANCE_M = 0.02


@pytest.fixture(scope="module")
def scans() -> tuple[np.ndarray, tuple[float, float, float, float]]:
    with np.load(SCANS) as d:
        meta = (
            float(d["angle_min"]),
            float(d["angle_increment"]),
            float(d["range_min"]),
            float(d["range_max"]),
        )
        return d["ranges"], meta


def test_fixture_is_small_and_float32(scans) -> None:
    ranges, _ = scans
    assert ranges.shape == (60, 720) and ranges.dtype == np.float32
    assert SCANS.stat().st_size < 250_000
    assert GOLDEN.stat().st_size < 100_000


def test_geometric_matches_the_golden_output(live: LiveVehicle) -> None:
    got = run_geometric(live)
    with np.load(GOLDEN) as g:
        want = {k: g[k] for k in g.files}
    assert np.array_equal(got["valid"], want["valid"])
    for key in ("x", "y", "hl", "hr"):
        for i, n in enumerate(want["valid"]):
            diff = np.abs(got[key][i, :n] - want[key][i, :n])
            assert diff.size == 0 or diff.max() <= TOLERANCE_M, (key, i, diff.max())


def test_real_scans_mostly_give_a_track(live: LiveVehicle) -> None:
    with np.load(GOLDEN) as g:
        valid = g["valid"]
    assert (valid >= 6).mean() > 0.9  # the golden is not a file of empty tracks


def test_fault_injection(live: LiveVehicle, scans) -> None:
    ranges, meta = scans
    rng = np.random.default_rng(0)
    p = core.TrackPerceiver(core.BACKEND_GEOMETRIC, live)
    for r in ranges[::3]:
        nan_burst = r.copy()
        start = int(rng.integers(0, 600))
        nan_burst[start : start + 100] = math.nan
        half_dropped = r.copy()
        half_dropped[rng.random(720) < 0.5] = math.nan
        inf_all = np.full(720, math.inf, dtype=np.float32)
        for fault in (nan_burst, half_dropped, inf_all, np.zeros(720, dtype=np.float32)):
            t = p.process(fault, *meta)
            assert len(t.x_m) == 16 and t.arrays_ok()
            n = t.valid_count
            for a in (t.x_m, t.y_m, t.half_width_left_m, t.half_width_right_m, t.sigma_m):
                assert np.isfinite(a).all()
            assert 0 <= n <= 16
