"""L2 (hypothesis): for any tracer track and any net track, `fuse_track` keeps the tracer's edges
on the shared stations, never has fewer valid stations than the tracer, and is well formed (finite,
16 entries, spacing) for any input, empty and non-finite included."""

from __future__ import annotations

import math

import numpy as np
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from racer_track_perception import fuse, selfcheck
from racer_track_perception import track as tr

SETTINGS = settings(
    max_examples=300,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large],
)
EDGE_TOL = 2e-5  # float32 storage of metre-sized values on bends


@st.composite
def tracks(draw, allow_bad: bool = False) -> tr.LocalTrackData:
    n = draw(st.integers(0, 16))
    heading = draw(st.floats(-1.0, 1.0))
    bend = draw(st.floats(-0.4, 0.4))
    x, y = draw(st.floats(-0.3, 0.3)), draw(st.floats(-0.4, 0.4))
    step = draw(st.floats(0.15, 0.25))
    t = tr.LocalTrackData(valid_count=n)
    h = heading
    for i in range(16):
        t.x_m[i], t.y_m[i] = x, y
        x += step * math.cos(h)
        y += step * math.sin(h)
        h += bend * 0.2
    hw = st.floats(0.0, 2.0)
    for i in range(16):
        t.half_width_left_m[i] = draw(hw)
        t.half_width_right_m[i] = draw(hw)
    t.sigma_m[:n] = 0.05
    t.sigma_m[n:] = tr.INVALID_SIGMA_M
    if allow_bad and draw(st.booleans()):
        k = draw(st.integers(0, 15))
        which = draw(st.sampled_from(["x_m", "y_m", "half_width_left_m", "sigma_m"]))
        getattr(t, which)[k] = draw(st.sampled_from([math.nan, math.inf, -math.inf]))
    return t


def _normals(t: tr.LocalTrackData, n: int) -> np.ndarray:
    x = t.x_m[:n].astype(np.float64)
    y = t.y_m[:n].astype(np.float64)
    tan = selfcheck._tangents(x, y) if n >= 2 else np.array([[1.0, 0.0]])
    return np.stack([-tan[:, 1], tan[:, 0]], axis=1)


@SETTINGS
@given(geo=tracks(), net=tracks(allow_bad=True), append=st.booleans())
def test_fused_edges_equal_the_tracers_and_the_count_does_not_shrink(geo, net, append) -> None:
    out = fuse.fuse_track(geo, net, fuse.FuseConfig(append_far=append))
    n = geo.valid_count
    assert out.valid_count >= n
    assert out.arrays_ok()
    for a in (out.x_m, out.y_m, out.half_width_left_m, out.half_width_right_m, out.sigma_m):
        assert len(a) == tr.K_STATIONS and np.isfinite(a).all()
    if n == 0:
        assert out.valid_count == 0
        return
    # Edges of both tracks along the TRACER's normals (the fused centre moved along them).
    nrm = _normals(geo, n)
    g = np.stack([geo.x_m[:n], geo.y_m[:n]], axis=1).astype(np.float64)
    c = np.stack([out.x_m[:n], out.y_m[:n]], axis=1).astype(np.float64)
    g_left = g + geo.half_width_left_m[:n, None].astype(np.float64) * nrm
    g_right = g - geo.half_width_right_m[:n, None].astype(np.float64) * nrm
    f_left = c + out.half_width_left_m[:n, None].astype(np.float64) * nrm
    f_right = c - out.half_width_right_m[:n, None].astype(np.float64) * nrm
    assert np.abs(f_left - g_left).max() <= EDGE_TOL
    assert np.abs(f_right - g_right).max() <= EDGE_TOL


@SETTINGS
@given(geo=tracks(), net=tracks(allow_bad=True))
def test_fused_output_is_well_formed_with_the_spacing_of_the_contract(geo, net) -> None:
    cfg = fuse.FuseConfig()
    out = fuse.fuse_track(geo, net, cfg)
    n = out.valid_count
    assert 0 <= n <= tr.K_STATIONS
    assert np.isfinite(out.sigma_m).all()
    ng = geo.valid_count
    if ng >= 2:
        d_geo = np.hypot(np.diff(geo.x_m[:ng].astype(float)), np.diff(geo.y_m[:ng].astype(float)))
        d_out = np.hypot(np.diff(out.x_m[:ng].astype(float)), np.diff(out.y_m[:ng].astype(float)))
        # each chord changes by at most the two lateral moves of its end stations
        assert (d_out <= d_geo + 2.0 * cfg.reach_m + 1e-5).all()
    if n > ng >= 1:
        join = math.hypot(
            float(out.x_m[ng]) - float(out.x_m[ng - 1]),
            float(out.y_m[ng]) - float(out.y_m[ng - 1]),
        )
        assert abs(join - tr.STATION_SPACING_M) <= fuse.JOIN_TOLERANCE_M + 1e-5
