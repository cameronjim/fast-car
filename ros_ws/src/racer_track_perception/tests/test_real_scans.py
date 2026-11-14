"""L4: 60 real scans (cut from 2026-11-11T03-54-11_car_teleop) through the geometric backend,
against a committed golden output, plus fault injection."""

from __future__ import annotations

import math

import numpy as np
import pytest
from make_real_scans_2026_11_13 import GOLDEN as GOLDEN_1113
from make_real_scans_2026_11_13 import SCANS as SCANS_1113
from make_real_scans_2026_11_13 import dequantise
from make_real_scans_2026_11_13 import run_geometric as run_geometric_1113
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


# ---------------------------------------------------------------------------------------------
# The 2026-11-13 fixture (P1.6): 45 scans of the TRAIN segments, straight, corner entry and mid
# corner in both directions, and three scans with a blanked sector. See
# make_real_scans_2026_11_13.py for how they were picked. Its golden is separate from the
# 2026-11-11 one above, which is untouched.
# ---------------------------------------------------------------------------------------------

TOLERANCE_1113_M = 0.02
TRAIN_SEGMENTS_1113 = {"G1a", "G2", "R0", "R1"}


@pytest.fixture(scope="module")
def scans_1113() -> dict[str, np.ndarray]:
    with np.load(SCANS_1113) as d:
        return {k: d[k] for k in d.files}


def test_1113_fixture_is_small_and_cut_from_train_segments(scans_1113) -> None:
    n = len(scans_1113["ranges_q"])
    assert scans_1113["ranges_q"].shape == (n, 720) and 0 < n <= 60
    assert SCANS_1113.stat().st_size + GOLDEN_1113.stat().st_size < 100_000
    assert set(scans_1113["segment"].tolist()) <= TRAIN_SEGMENTS_1113
    kinds = set(scans_1113["kind"].tolist())
    assert kinds == {"straight", "corner entry", "mid corner"}
    for direction in ("ccw", "cw"):
        for kind in kinds:
            sel = (scans_1113["direction"] == direction) & (scans_1113["kind"] == kind)
            assert (sel & ~scans_1113["dropped_sector"]).sum() >= 5, (direction, kind)
    assert scans_1113["dropped_sector"].sum() == 3


def longest_invalid_run(bad: np.ndarray) -> int:
    """Longest circular run of True."""
    best = cur = 0
    for v in np.concatenate([bad, bad]):
        cur = cur + 1 if v else 0
        best = max(best, cur)
    return min(best, len(bad))


def test_1113_dropped_sector_scans_have_a_blank_sector(scans_1113) -> None:
    ranges = dequantise(scans_1113["ranges_q"])
    dropped = scans_1113["dropped_sector"]
    for k in range(len(ranges)):
        run = longest_invalid_run(~np.isfinite(ranges[k]))
        # 40 degrees of 720 beams over 360 degrees is 80 beams in one run; a real scan of this
        # loop never has a gap of that size.
        assert (run >= 75) == bool(dropped[k]), (k, run)


def test_1113_geometric_matches_the_golden_output(live: LiveVehicle) -> None:
    got = run_geometric_1113()
    with np.load(GOLDEN_1113) as g:
        want = {k: g[k] for k in g.files}
    assert np.array_equal(got["valid"], want["valid"])
    for key in ("x", "y", "hl", "hr"):
        for i, n in enumerate(want["valid"]):
            diff = np.abs(got[key][i, :n] - want[key][i, :n])
            assert diff.size == 0 or diff.max() <= TOLERANCE_1113_M, (key, i, diff.max())


def test_1113_real_scans_mostly_give_a_track(scans_1113) -> None:
    with np.load(GOLDEN_1113) as g:
        valid = g["valid"]
    assert (valid >= 6).mean() > 0.9
    plain = ~scans_1113["dropped_sector"]
    assert (valid[plain] >= 12).mean() > 0.9  # the golden is not a file of empty tracks


def test_1113_dropped_sector_scans_still_give_a_well_formed_track(
    live: LiveVehicle, scans_1113
) -> None:
    p = core.TrackPerceiver(core.BACKEND_GEOMETRIC, live)
    ranges = dequantise(scans_1113["ranges_q"])
    meta = (
        float(scans_1113["angle_min"]),
        float(scans_1113["angle_increment"]),
        float(scans_1113["range_min"]),
        float(scans_1113["range_max"]),
    )
    for k in np.flatnonzero(scans_1113["dropped_sector"]):
        t = p.process(ranges[k], *meta)
        assert t.arrays_ok() and 0 <= t.valid_count <= 16
