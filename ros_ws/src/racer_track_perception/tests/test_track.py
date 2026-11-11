from __future__ import annotations

import math

import numpy as np
from racer_track_perception import track as tr


def arrays(n_ok: int = 16, sigma: float = 0.05):
    k = np.arange(16, dtype=np.float32)
    s = np.full(16, sigma, dtype=np.float32)
    s[n_ok:] = 0.9
    hl = np.full(16, 0.5, np.float32)
    hr = np.full(16, 0.6, np.float32)
    return k * 0.2, np.zeros(16, np.float32), hl, hr, s


def test_valid_count_from_sigma_is_the_leading_run() -> None:
    s = np.array([0.05, 0.1, 0.3, 0.05], dtype=np.float32)
    assert tr.valid_count_from_sigma(s, 0.25) == 2
    assert tr.valid_count_from_sigma(np.full(16, 0.1), 0.25) == 16
    assert tr.valid_count_from_sigma(np.array([math.nan, 0.1]), 0.25) == 0
    assert tr.valid_count_from_sigma(np.array([0.1, math.inf]), 0.25) == 1
    assert tr.valid_count_from_sigma(np.array([]), 0.25) == 0


def test_assemble_full_and_partial() -> None:
    x, y, hl, hr, s = arrays()
    t = tr.assemble(x, y, hl, hr, s, 0.25)
    assert t.valid_count == 16 and t.arrays_ok()
    x, y, hl, hr, s = arrays(n_ok=7)
    t = tr.assemble(x, y, hl, hr, s, 0.25)
    assert t.valid_count == 7 and t.arrays_ok()
    assert np.all(t.sigma_m[7:] == tr.INVALID_SIGMA_M)
    assert np.all(t.x_m[7:] == t.x_m[6])  # padding repeats the last valid station
    assert abs(t.x_m[6] - 6 * 0.2) < 1e-6


def test_assemble_cap_and_clip() -> None:
    x, y, hl, hr, s = arrays()
    hl[:] = 5.0
    hr[:] = -1.0
    t = tr.assemble(x, y, hl, hr, s, 0.25, valid_cap=4)
    assert t.valid_count == 4
    assert np.all(t.half_width_left_m == 2.0) and np.all(t.half_width_right_m == 0.0)


def test_assemble_nonfinite_stops_the_valid_run() -> None:
    x, y, hl, hr, s = arrays()
    y[5] = math.nan
    t = tr.assemble(x, y, hl, hr, s, 0.25)
    assert t.valid_count == 5
    for a in (t.x_m, t.y_m, t.half_width_left_m, t.half_width_right_m, t.sigma_m):
        assert np.isfinite(a).all()


def test_assemble_wrong_length_or_empty_is_no_track() -> None:
    x, y, hl, hr, s = arrays()
    assert tr.assemble(x[:15], y, hl, hr, s, 0.25).valid_count == 0
    x, y, hl, hr, s = arrays(n_ok=0)
    t = tr.assemble(x, y, hl, hr, s, 0.25)
    assert t.valid_count == 0 and t.arrays_ok()
    assert len(t.x_m) == 16 and np.isfinite(t.x_m).all()


def test_arrays_ok_rules() -> None:
    t = tr.no_track()
    assert t.arrays_ok() and t.valid_count == 0
    assert math.isnan(t.boundary_residual_m) and math.isnan(t.intrusion_fraction)
    t.valid_count = 3
    t.x_m[1] = math.nan
    assert not t.arrays_ok()
    t.x_m[1] = 0.0
    assert t.arrays_ok()
    t.valid_count = 17
    assert not t.arrays_ok()
    t.valid_count = 2
    t.y_m = np.zeros(15, dtype=np.float32)
    assert not t.arrays_ok()
