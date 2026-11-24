"""L2 (hypothesis): the coverage loss of `fuse_track_v2` against the tracer, measured with the
registered `corridor.compare`, obeys the bound derived in
`ros_ws/src/racer_track_perception/tests/test_fuse_v2_property.py`."""

from __future__ import annotations

import math

from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from racer_pnp_perception import corridor
from racer_track_perception import fuse_v2
from racer_track_perception import track as tr

SETTINGS = settings(
    max_examples=200,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large],
)
SLIVER = 0.03


@st.composite
def smooth_pairs(draw) -> tuple[tr.LocalTrackData, tr.LocalTrackData, float]:
    """A constant width tracer on a gentle bend and a net track offset from it laterally."""
    n = draw(st.integers(4, 16))
    bend = draw(st.floats(-0.4, 0.4))
    hw = draw(st.floats(0.3, 0.8))
    off = draw(st.floats(-0.3, 0.3))
    wobble = draw(st.floats(0.0, 0.05))
    geo = tr.LocalTrackData(valid_count=n)
    net = tr.LocalTrackData(valid_count=n)
    x = y = h = 0.0
    for i in range(16):
        geo.x_m[i], geo.y_m[i] = x, y
        # the net's centre: a lateral offset along the local left normal plus a small wobble
        net.x_m[i] = x - math.sin(h) * (off + wobble * math.sin(i))
        net.y_m[i] = y + math.cos(h) * (off + wobble * math.sin(i))
        x += 0.2 * math.cos(h)
        y += 0.2 * math.sin(h)
        h += bend * 0.2
    for t in (geo, net):
        t.half_width_left_m[:] = hw
        t.half_width_right_m[:] = hw
        t.sigma_m[:n] = 0.05
        t.sigma_m[n:] = tr.INVALID_SIGMA_M
    return geo, net, draw(st.floats(0.0, 0.2))


@SETTINGS
@given(pair=smooth_pairs())
def test_coverage_loss_obeys_the_derived_bound(pair) -> None:
    geo, net, ext = pair
    cfg = fuse_v2.FuseV2Config(edge_end_extension_m=ext, append_far=False)
    out = fuse_v2.fuse_track_v2(geo, net, cfg)
    ng, n = geo.valid_count, out.valid_count
    result = corridor.compare(geo, out)
    bound = max(ng - n + 1, 0) / (ng - 1) + SLIVER
    assert result["loss"] <= bound
