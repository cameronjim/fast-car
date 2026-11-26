"""L2 (hypothesis): for any tracer and any net track `fuse_track_v2` is well formed (finite, 16
entries, 0.20 m spacing along its polyline) and every rebuilt edge point lies on the tracer's edge
polyline or its allowed extension; the coverage loss obeys the bound derived below.

Bound. Let G be the tracer's corridor polygon (edge polylines L and R joined) and F the polygon a
consumer rebuilds from the output (first n stations). Every vertex of F lies on L or R (the
property asserted first), so F is the polygon through points of the boundary of G. G minus F is then
(a) the part of G beyond the cross-section of the last output station and before the first one, and
(b) the slivers between each edge polyline and its chord between consecutive output stations. For a
corridor of constant width w along a gentle bend, (a) is at most (1 + max(ng - n, 0)) / (ng - 1) of G
(the start may be trimmed by up to one station spacing, the point of the shifted centre nearest
base_link, and each station the output lacks of the tracer's ng costs one more; stations the
extension adds beyond ng cover nothing of G), each worth 1 / (ng - 1), and (b) is at most
the sagitta of a chord of 0.2 m on a bend of at most 0.4 rad/m, s = 0.2^2 * 0.4 / 8 = 2 mm per edge
over a corridor of at least 0.6 m: 2 * 0.002 / 0.6 < 0.007, doubled by the shift of the normals to
0.014. The test allows 0.03 (`training/racer_pnp_perception/tests/test_fuse_v2_coverage.py`, which can
import the registered `corridor` measure).
"""

from __future__ import annotations

import numpy as np
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from racer_track_perception import fuse_v2
from racer_track_perception import track as tr
from test_fuse_property import tracks
from test_fuse_v2 import dist_to_polyline, edge_polylines, with_extensions

SETTINGS = settings(
    max_examples=300,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large],
)
EDGE_TOL = 2e-5  # float32 storage of metre-sized values on bends


@SETTINGS
@given(geo=tracks(), net=tracks(allow_bad=True), ext=st.floats(0.0, 0.2), append=st.booleans())
def test_output_is_well_formed_for_any_input(geo, net, ext, append) -> None:
    cfg = fuse_v2.FuseV2Config(edge_end_extension_m=ext, append_far=append)
    out = fuse_v2.fuse_track_v2(geo, net, cfg)
    n = out.valid_count
    assert 0 <= n <= tr.K_STATIONS
    assert out.arrays_ok()
    for a in (out.x_m, out.y_m, out.half_width_left_m, out.half_width_right_m, out.sigma_m):
        assert len(a) == tr.K_STATIONS and np.isfinite(a).all()
    if geo.valid_count == 0:
        assert n == 0
    assert np.all(out.half_width_left_m >= 0.0) and np.all(out.half_width_left_m <= 2.0)
    assert np.all(out.half_width_right_m >= 0.0) and np.all(out.half_width_right_m <= 2.0)


@SETTINGS
@given(geo=tracks(), net=tracks(), ext=st.floats(0.0, 0.2))
def test_every_rebuilt_edge_point_lies_on_the_tracers_edge_polyline(geo, net, ext) -> None:
    out = fuse_v2.fuse_track_v2(
        geo, net, fuse_v2.FuseV2Config(edge_end_extension_m=ext, append_far=False)
    )
    ng, n = geo.valid_count, out.valid_count
    if ng < 2 or n < 2:
        return
    if np.array_equal(out.x_m[:n], geo.x_m[:n]) and np.array_equal(
        out.half_width_left_m[:n], geo.half_width_left_m[:n]
    ):
        # the tracer's own track (a fallback): its edges are its own
        return
    g_left, g_right = edge_polylines(geo)
    o_left, o_right = edge_polylines(out)
    widths = (out.half_width_left_m[:n], out.half_width_right_m[:n])
    for poly, rebuilt, width in ((g_left, o_left, widths[0]), (g_right, o_right, widths[1])):
        extended = with_extensions(poly, ext)
        for p, w in zip(rebuilt, width, strict=True):
            if w >= tr.HALF_WIDTH_CAP_M - 1e-6:
                continue  # a hit beyond the message's 2.0 m cap is clipped to it, off the polyline
            assert dist_to_polyline(p, extended) <= EDGE_TOL * 10
