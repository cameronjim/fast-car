"""L1: `fuse.fuse_track`, the tracer's corridor with the net's centre (`fused-v1`, brief 38)."""

from __future__ import annotations

import math

import numpy as np
import pytest
from racer_track_perception import fuse
from racer_track_perception import track as tr

# float32 storage of the message arrays: positions of a few metres carry 1e-7 of rounding, so
# "unchanged" is asserted to 1e-6 (the brief's 1e-9 cannot hold in float32).
EDGE_TOL = 1e-6


def straight(
    n: int = 16,
    y: float = 0.0,
    hl: float = 0.5,
    hr: float = 0.5,
    sigma: float = 0.05,
    x0: float = 0.0,
) -> tr.LocalTrackData:
    t = tr.LocalTrackData(valid_count=n)
    t.x_m[:] = x0 + 0.2 * np.arange(16)
    t.y_m[:] = y
    t.half_width_left_m[:] = hl
    t.half_width_right_m[:] = hr
    t.sigma_m[:n] = sigma
    t.sigma_m[n:] = tr.INVALID_SIGMA_M
    return t


def edges(t: tr.LocalTrackData) -> tuple[np.ndarray, np.ndarray]:
    """Left and right edge y of a track that runs along +x."""
    n = t.valid_count
    return (
        t.y_m[:n].astype(float) + t.half_width_left_m[:n],
        t.y_m[:n].astype(float) - t.half_width_right_m[:n],
    )


def test_net_equal_to_tracer_gives_the_tracer() -> None:
    geo = straight(10)
    out = fuse.fuse_track(geo, straight(10))
    for name in ("x_m", "y_m", "half_width_left_m", "half_width_right_m", "sigma_m"):
        assert np.allclose(getattr(out, name)[:10], getattr(geo, name)[:10], atol=1e-7), name
    assert out.valid_count == 10


def test_net_centre_shifted_inside_the_corridor_gives_that_centre_and_unmoved_edges() -> None:
    geo = straight(12)
    out = fuse.fuse_track(geo, straight(12, y=0.1))
    n = out.valid_count
    assert n == 12
    assert np.allclose(out.y_m[:n], 0.1, atol=1e-6)
    left, right = edges(out)
    g_left, g_right = edges(geo)
    assert np.abs(left - g_left).max() <= EDGE_TOL
    assert np.abs(right - g_right).max() <= EDGE_TOL
    assert np.allclose(out.half_width_left_m[:n], 0.4, atol=1e-6)
    assert np.allclose(out.half_width_right_m[:n], 0.6, atol=1e-6)


def test_the_net_on_the_other_side_moves_the_centre_the_other_way() -> None:
    out = fuse.fuse_track(straight(12), straight(12, y=-0.2))
    assert np.allclose(out.y_m[:12], -0.2, atol=1e-6)
    assert np.allclose(out.half_width_left_m[:12], 0.7, atol=1e-6)


def test_a_shift_beyond_the_margin_is_clamped_to_the_margin() -> None:
    geo = straight(12, hl=0.3, hr=0.3)  # margin 0.15: the centre may move at most 0.15
    out = fuse.fuse_track(geo, straight(12, y=0.3))  # in reach (0.35), outside the margin
    assert np.allclose(out.y_m[:12], 0.15, atol=1e-6)
    assert np.allclose(out.half_width_left_m[:12], 0.15, atol=1e-6)
    assert np.allclose(out.half_width_right_m[:12], 0.45, atol=1e-6)
    left, right = edges(out)
    g_left, g_right = edges(geo)
    assert np.abs(left - g_left).max() <= EDGE_TOL and np.abs(right - g_right).max() <= EDGE_TOL


def test_a_net_out_of_reach_leaves_the_station() -> None:
    geo = straight(12, hl=0.9, hr=0.9)
    out = fuse.fuse_track(geo, straight(12, y=0.5))  # 0.5 m away, reach 0.35
    assert np.allclose(out.y_m[:12], 0.0, atol=1e-7)
    assert np.allclose(out.half_width_left_m[:12], 0.9, atol=1e-6)


def test_reach_is_per_station() -> None:
    geo = straight(12, hl=0.9, hr=0.9)
    net = straight(12, y=0.0)
    net.y_m[6:] = 0.6  # the net leaves the corridor from station 6 on
    out = fuse.fuse_track(geo, net)
    assert np.allclose(out.y_m[:4], 0.0, atol=1e-6)


def test_blend_scales_the_move() -> None:
    out = fuse.fuse_track(straight(12), straight(12, y=0.2), fuse.FuseConfig(blend=0.5))
    assert np.allclose(out.y_m[:12], 0.1, atol=1e-6)


def test_a_narrow_corridor_keeps_the_tracers_centre() -> None:
    geo = straight(12, hl=0.12, hr=0.12)  # narrower than twice the 0.15 margin
    out = fuse.fuse_track(geo, straight(12, y=0.05))
    assert np.allclose(out.y_m[:12], 0.0, atol=1e-7)
    assert np.allclose(out.half_width_left_m[:12], 0.12, atol=1e-6)


def test_the_half_width_cap_is_never_exceeded() -> None:
    geo = straight(12, hl=0.6, hr=2.0)  # right edge already at the 2.0 m cap
    out = fuse.fuse_track(geo, straight(12, y=0.3))  # moving left would grow hr past the cap
    assert out.half_width_right_m[:12].max() <= tr.HALF_WIDTH_CAP_M + 1e-6
    left, right = edges(out)
    g_left, g_right = edges(geo)
    assert np.abs(left - g_left).max() <= EDGE_TOL and np.abs(right - g_right).max() <= EDGE_TOL


def test_count_spacing_and_sigma_are_the_tracers() -> None:
    geo = straight(9, sigma=0.07)
    out = fuse.fuse_track(geo, straight(16, y=0.1), fuse.FuseConfig(append_far=False))
    assert out.valid_count == 9
    assert np.allclose(out.x_m[:9], geo.x_m[:9], atol=1e-6)
    assert np.allclose(out.sigma_m[:9], 0.07, atol=1e-6)
    assert np.all(out.sigma_m[9:] == tr.INVALID_SIGMA_M)


def test_append_far_adds_the_nets_stations_beyond_the_tracers_last() -> None:
    geo = straight(9)
    net = straight(16, y=0.1)
    out = fuse.fuse_track(geo, net)
    assert out.valid_count == 16
    assert out.arrays_ok()
    # the appended stations are the net's own
    assert np.allclose(out.y_m[9:16], 0.1, atol=1e-6)
    assert np.allclose(out.x_m[9:16], net.x_m[9:16], atol=1e-6)
    d = np.hypot(np.diff(out.x_m.astype(float)), np.diff(out.y_m.astype(float)))
    assert np.allclose(d, 0.2, atol=1e-5)


def test_append_far_off_adds_nothing() -> None:
    out = fuse.fuse_track(straight(9), straight(16), fuse.FuseConfig(append_far=False))
    assert out.valid_count == 9


def test_the_net_not_reaching_past_the_tracer_appends_nothing() -> None:
    out = fuse.fuse_track(straight(9), straight(9, y=0.1))
    assert out.valid_count == 9


def test_a_join_that_breaks_the_spacing_contract_appends_nothing() -> None:
    geo = straight(9)
    net = straight(16)
    net.x_m[9:] += 0.12  # the net's far stations sit 0.32 m after the tracer's last
    out = fuse.fuse_track(geo, net)
    assert out.valid_count == 9
    net2 = straight(16)
    net2.y_m[9:] = 0.3  # a sideways jump at the join
    assert fuse.fuse_track(geo, net2).valid_count == 9


def test_a_full_tracer_track_appends_nothing() -> None:
    assert fuse.fuse_track(straight(16), straight(16)).valid_count == 16


@pytest.mark.parametrize("bad", ["empty", "nan", "short", "inf_y"])
def test_unusable_net_outputs_return_the_tracer_and_are_counted(bad: str) -> None:
    geo = straight(10)
    if bad == "empty":
        net = tr.no_track()
    elif bad == "nan":
        net = straight(10)
        net.y_m[:] = np.nan
    elif bad == "inf_y":
        net = straight(10)
        net.y_m[0] = np.inf
    else:
        net = straight(0)  # valid_count 0
    out, outcome = fuse.fuse_track_with_outcome(geo, net)
    assert outcome is fuse.FuseOutcome.NET_UNUSABLE
    for name in ("x_m", "y_m", "half_width_left_m", "half_width_right_m", "sigma_m"):
        assert np.array_equal(getattr(out, name), getattr(geo, name)), name
    assert out.valid_count == 10
    assert fuse.fuse_track(geo, net).valid_count == 10


def test_a_one_station_net_is_a_point_target() -> None:
    out = fuse.fuse_track(straight(10), straight(1, y=0.1), fuse.FuseConfig(append_far=False))
    assert out.y_m[0] == pytest.approx(0.1, abs=1e-6)  # within reach of station 0 only
    assert out.valid_count == 10


def test_an_empty_tracer_gives_no_track() -> None:
    out, outcome = fuse.fuse_track_with_outcome(tr.no_track(), straight(10))
    assert outcome is fuse.FuseOutcome.TRACER_EMPTY
    assert out.valid_count == 0


def test_inputs_are_not_modified() -> None:
    geo, net = straight(9), straight(16, y=0.1)
    g0 = geo.y_m.copy()
    n0 = net.y_m.copy()
    fuse.fuse_track(geo, net)
    assert np.array_equal(geo.y_m, g0) and np.array_equal(net.y_m, n0)


def test_a_bend_moves_along_the_normal() -> None:
    """On a bend the centre moves along the tracer's own normal, edges stay on the normal line."""
    geo = tr.LocalTrackData(valid_count=12)
    theta = 0.1 * np.arange(16)
    radius = 3.0
    geo.x_m[:] = radius * np.sin(theta)
    geo.y_m[:] = radius * (1 - np.cos(theta))
    geo.half_width_left_m[:] = 0.6
    geo.half_width_right_m[:] = 0.6
    geo.sigma_m[:12] = 0.05
    geo.sigma_m[12:] = tr.INVALID_SIGMA_M
    net = tr.LocalTrackData(valid_count=12)
    r2 = radius - 0.12  # a smaller circle: the net's centreline is 0.12 m to the left
    net.x_m[:] = r2 * np.sin(theta * radius / r2)
    net.y_m[:] = radius - r2 * np.cos(theta * radius / r2)
    net.half_width_left_m[:] = 0.5
    net.half_width_right_m[:] = 0.5
    net.sigma_m[:12] = 0.05
    net.sigma_m[12:] = tr.INVALID_SIGMA_M
    out = fuse.fuse_track(geo, net, fuse.FuseConfig(append_far=False))
    n = out.valid_count
    gx, gy = geo.x_m[:n].astype(float), geo.y_m[:n].astype(float)
    tan = np.stack([np.gradient(gx), np.gradient(gy)], 1)
    tan /= np.hypot(tan[:, 0], tan[:, 1])[:, None]
    nrm = np.stack([-tan[:, 1], tan[:, 0]], 1)
    le = np.stack([gx, gy], 1) + geo.half_width_left_m[:n, None] * nrm
    lo = np.stack([out.x_m[:n], out.y_m[:n]], 1) + out.half_width_left_m[:n, None] * nrm
    assert np.abs(le - lo).max() <= 1e-5
    shift = np.hypot(out.x_m[:n] - geo.x_m[:n], out.y_m[:n] - geo.y_m[:n])
    assert shift.max() > 0.05
    assert shift.max() <= fuse.FuseConfig().reach_m + 1e-6
    assert math.isfinite(float(out.half_width_left_m[0]))


def test_config_round_trips_and_is_validated() -> None:
    cfg = fuse.FuseConfig(reach_m=0.3, edge_margin_m=0.1, blend=0.8, append_far=False)
    assert fuse.FuseConfig.from_dict(cfg.as_dict()) == cfg
    for bad in (
        {"reach_m": 0.0},
        {"reach_m": float("nan")},
        {"edge_margin_m": -0.1},
        {"blend": 1.5},
        {"blend": True},
        {"append_far": 1},
    ):
        raw = {**fuse.FuseConfig().as_dict(), **bad}
        with pytest.raises(ValueError):
            fuse.FuseConfig.from_dict(raw)
    with pytest.raises(ValueError):
        fuse.FuseConfig.from_dict({"reach_m": 0.3})
    with pytest.raises(ValueError):
        fuse.FuseConfig.from_dict(None)
