"""L2 (hypothesis): `refine_track` is well formed for any net output and any scan (finite, 16
stations, the spacing of 4.1, empty, NaN and inf points included), and it does not put more scan
points inside the corridor than the input did (by more than 0.01 of the near points, on scans of
at least 50 points) when the net is wrong by an amount the refinement can reach.

Why the second property is not stated for literally any net: a net whose centreline runs into a
wall further than `max_shift_m` from the lane cannot be repaired by a bounded move of each station,
and a net that claims a half width under 0.125 m has a corridor narrower than the 0.05 m intrusion
margin allows to guard between two stations 0.20 m apart (hw - sqrt(hw^2 - 0.01) is under 0.05
only from hw = 0.125 m), so it has no intrusion to be compared with. The strategies below draw
nets from a lane (straight or bending, half width 0.3 to 0.9 m) with a centreline error of up to
0.3 m (under the 0.35 m reach) and any claimed half widths from 0.15 m.
"""

from __future__ import annotations

import math

import numpy as np
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from racer_track_perception import refine, selfcheck
from racer_track_perception import track as tr

SETTINGS = settings(
    max_examples=300,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large],
)
INTRUSION_SLACK = 0.01
MIN_POINTS = 50
MIN_CLAIMED_HALF_WIDTH_M = 0.15
MAX_CENTRELINE_ERROR_M = 0.3


def _well_formed(net: tr.LocalTrackData, pts: np.ndarray) -> tr.LocalTrackData:
    out = refine.refine_track(net, pts)
    assert len(out.x_m) == tr.K_STATIONS and len(out.sigma_m) == tr.K_STATIONS
    assert out.arrays_ok()
    n = out.valid_count
    for a in (out.x_m, out.y_m, out.half_width_left_m, out.half_width_right_m, out.sigma_m):
        assert len(a) == tr.K_STATIONS and np.isfinite(a).all()
    assert 0 <= n <= tr.K_STATIONS
    if n >= 2:
        chord = np.hypot(np.diff(out.x_m[:n].astype(float)), np.diff(out.y_m[:n].astype(float)))
        # Resampled stations are 0.20 m of arc apart (a chord of at most that); an output that is
        # the input unchanged (no scan support) keeps the net's own spacing.
        unchanged = np.array_equal(out.x_m, net.x_m) and np.array_equal(out.y_m, net.y_m)
        assert unchanged or chord.max() <= tr.STATION_SPACING_M + 1e-5
    return out


@st.composite
def any_net_outputs(draw) -> tr.LocalTrackData:
    """A polyline of 16 stations that bends, with half widths anywhere in [0, 2], and a valid run
    of any length."""
    heading = draw(st.floats(-1.0, 1.0))
    bend = draw(st.floats(-0.4, 0.4))
    x, y = draw(st.floats(-0.3, 0.3)), draw(st.floats(-0.4, 0.4))
    step = draw(st.floats(0.15, 0.25))
    xs, ys = [], []
    for _ in range(tr.K_STATIONS):
        xs.append(x)
        ys.append(y)
        heading += bend * draw(st.floats(0.0, 1.0))
        x += step * math.cos(heading)
        y += step * math.sin(heading)
    hl = draw(st.lists(st.floats(0.0, 2.0), min_size=16, max_size=16))
    hr = draw(st.lists(st.floats(0.0, 2.0), min_size=16, max_size=16))
    n = draw(st.integers(0, 16))
    return tr.assemble(xs, ys, hl, hr, np.full(16, 0.05), 0.25, valid_cap=n)


@st.composite
def any_scans(draw) -> np.ndarray:
    """Clutter, walls with noise, both, bad values (NaN, inf) and nothing."""
    kind = draw(st.sampled_from(["walls", "clutter", "mixed", "bad", "empty"]))
    rng = np.random.default_rng(draw(st.integers(0, 2**31 - 1)))
    if kind == "empty":
        return np.zeros((0, 2))
    n = draw(st.integers(0, 400))
    if kind == "clutter":
        pts = rng.uniform(-2.0, 4.0, size=(n, 2))
    else:
        half = draw(st.floats(0.3, 0.9))
        xs = rng.uniform(-0.5, 4.0, size=n)
        side = rng.choice([-1.0, 1.0], size=n)
        pts = np.stack([xs, side * half + rng.normal(0, 0.02, size=n)], 1)
        if kind == "mixed":
            pts = np.concatenate([pts, rng.uniform(-2.0, 4.0, size=(draw(st.integers(0, 80)), 2))])
    if kind == "bad" and len(pts):
        bad = rng.random(len(pts)) < 0.2
        pts[bad, 0] = rng.choice([math.nan, math.inf, -math.inf], size=int(bad.sum()))
    return pts


@SETTINGS
@given(net=any_net_outputs(), pts=any_scans())
def test_refine_is_well_formed_for_any_net_and_any_scan(net, pts) -> None:
    _well_formed(net, pts)


@st.composite
def lane_scenes(draw) -> tuple[tr.LocalTrackData, np.ndarray]:
    """A lane (straight or bending), its wall points with noise and a few bad values, and a net
    output that follows the lane with an error of up to 0.3 m and any claimed half widths."""
    half = draw(st.floats(0.3, 0.9))
    curvature = draw(st.floats(-1.2, 1.2))  # 1 / radius, left positive
    rng = np.random.default_rng(draw(st.integers(0, 2**31 - 1)))

    def centreline(s: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if abs(curvature) < 1e-6:
            return s, np.zeros_like(s), np.zeros_like(s)
        th = curvature * s
        return np.sin(th) / curvature, (1.0 - np.cos(th)) / curvature, th

    s = np.arange(-0.5, 4.0, 0.015)
    cx, cy, th = centreline(s)
    nx, ny = -np.sin(th), np.cos(th)
    left = np.stack([cx + half * nx, cy + half * ny], 1)
    right = np.stack([cx - half * nx, cy - half * ny], 1)
    pts = np.concatenate([left, right]) + rng.normal(0, 0.01, size=(2 * len(s), 2))
    keep = rng.random(len(pts)) < draw(st.floats(0.3, 1.0))
    pts = pts[keep]
    if draw(st.booleans()) and len(pts):
        bad = rng.random(len(pts)) < 0.1
        pts[bad, 1] = rng.choice([math.nan, math.inf], size=int(bad.sum()))
    ns = 0.2 * np.arange(tr.K_STATIONS)
    nx_, ny_, nth = centreline(ns)
    offset = draw(st.floats(-MAX_CENTRELINE_ERROR_M, MAX_CENTRELINE_ERROR_M))
    wobble = rng.uniform(-0.05, 0.05, size=tr.K_STATIONS)
    err = np.clip(offset + wobble, -MAX_CENTRELINE_ERROR_M, MAX_CENTRELINE_ERROR_M)
    x = nx_ - np.sin(nth) * err
    y = ny_ + np.cos(nth) * err
    hl = draw(st.lists(st.floats(MIN_CLAIMED_HALF_WIDTH_M, 2.0), min_size=16, max_size=16))
    hr = draw(st.lists(st.floats(MIN_CLAIMED_HALF_WIDTH_M, 2.0), min_size=16, max_size=16))
    net = tr.assemble(x, y, hl, hr, np.full(16, 0.05), 0.25, valid_cap=draw(st.integers(2, 16)))
    return net, pts


@SETTINGS
@given(scene=lane_scenes())
def test_refine_does_not_add_intrusion_on_a_lane(scene) -> None:
    net, pts = scene
    out = _well_formed(net, pts)
    finite_pts = pts[np.isfinite(pts).all(axis=1)]
    if len(pts) >= MIN_POINTS and out.valid_count >= 2:
        before = selfcheck.intrusion_fraction(net, finite_pts)
        after = selfcheck.intrusion_fraction(out, finite_pts)
        if math.isfinite(before) and math.isfinite(after):
            assert after <= before + INTRUSION_SLACK, (before, after)
