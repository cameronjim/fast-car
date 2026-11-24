"""L1: `fuse_v2.fuse_track_v2`, the tracer's edge polylines with the net's centre and half widths by
ray cast (`fused-v2`, brief 55)."""

from __future__ import annotations

import itertools
import math

import numpy as np
import pytest
from racer_track_perception import fuse, fuse_v2, selfcheck
from racer_track_perception import track as tr
from test_fuse import straight

TOL = 1e-6
FIELDS = ("x_m", "y_m", "half_width_left_m", "half_width_right_m", "sigma_m")


def curved(n: int = 16, bend: float = 0.15, hw: float = 0.5) -> tr.LocalTrackData:
    """A tracer track on a circular arc, stations exactly 0.2 m apart along the chords."""
    t = tr.LocalTrackData(valid_count=n)
    x = y = h = 0.0
    for i in range(16):
        t.x_m[i], t.y_m[i] = x, y
        x += 0.2 * math.cos(h)
        y += 0.2 * math.sin(h)
        h += bend * 0.2
    t.half_width_left_m[:] = hw
    t.half_width_right_m[:] = hw
    t.sigma_m[:n] = 0.05
    t.sigma_m[n:] = tr.INVALID_SIGMA_M
    return t


def edge_polylines(geo: tr.LocalTrackData) -> tuple[np.ndarray, np.ndarray]:
    n = geo.valid_count
    x = geo.x_m[:n].astype(float)
    y = geo.y_m[:n].astype(float)
    tan = selfcheck._tangents(x, y)
    nx, ny = -tan[:, 1], tan[:, 0]
    hl = geo.half_width_left_m[:n].astype(float)
    hr = geo.half_width_right_m[:n].astype(float)
    return (
        np.stack([x + hl * nx, y + hl * ny], axis=1),
        np.stack([x - hr * nx, y - hr * ny], axis=1),
    )


def dist_to_polyline(p: np.ndarray, poly: np.ndarray) -> float:
    best = math.inf
    for a, b in itertools.pairwise(poly):
        e = b - a
        t = float(np.clip(np.dot(p - a, e) / max(float(np.dot(e, e)), 1e-18), 0.0, 1.0))
        best = min(best, float(np.hypot(*(p - (a + t * e)))))
    return best


def with_extensions(poly: np.ndarray, ext: float) -> np.ndarray:
    """The polyline pushed out along its two end tangents (written out here, not imported)."""
    t0 = (poly[0] - poly[1]) / np.hypot(*(poly[0] - poly[1]))
    t1 = (poly[-1] - poly[-2]) / np.hypot(*(poly[-1] - poly[-2]))
    return np.vstack([poly[0] + ext * t0, poly, poly[-1] + ext * t1])


def rebuilt_edges(out: tr.LocalTrackData) -> tuple[np.ndarray, np.ndarray]:
    return edge_polylines(out)


def test_net_equal_to_tracer_gives_the_tracer() -> None:
    for geo in (straight(10), curved(12)):
        out = fuse_v2.fuse_track_v2(geo, geo, fuse_v2.FuseV2Config())
        n = geo.valid_count
        assert out.valid_count == n
        for name in FIELDS:
            assert np.allclose(getattr(out, name)[:n], getattr(geo, name)[:n], atol=1e-5), name


def test_a_centre_shift_leaves_the_rebuilt_edges_on_the_tracers_edge_polylines() -> None:
    geo = curved(14, bend=0.3)
    net = curved(14, bend=0.3)
    net.y_m[:] += 0.1  # a 0.1 m shift, inside the corridor and the reach
    cfg = fuse_v2.FuseV2Config(edge_end_extension_m=0.2)
    out = fuse_v2.fuse_track_v2(geo, net, cfg)
    assert out.valid_count >= 12
    g_left, g_right = edge_polylines(geo)
    o_left, o_right = rebuilt_edges(out)
    for poly, rebuilt in ((g_left, o_left), (g_right, o_right)):
        ext = with_extensions(poly, 0.2)
        for p in rebuilt:
            assert dist_to_polyline(p, ext) < TOL
    # and the centre did move
    assert abs(float(np.mean(out.y_m[:5] - geo.y_m[:5]))) > 0.02


def test_a_shift_is_in_reach_only_out_of_reach_stations_keep_the_tracers_centre() -> None:
    geo = straight(12)
    out = fuse_v2.fuse_track_v2(geo, straight(12, y=0.6), fuse_v2.FuseV2Config())
    assert np.allclose(out.y_m[:12], 0.0, atol=TOL)  # 0.6 m is beyond reach 0.35


def test_narrow_corridor_keeps_the_tracers_centre() -> None:
    geo = straight(12, hl=0.1, hr=0.1)  # narrower than twice the margin
    out = fuse_v2.fuse_track_v2(geo, straight(12, y=0.05), fuse_v2.FuseV2Config())
    assert np.allclose(out.y_m[:12], 0.0, atol=TOL)
    assert np.allclose(out.half_width_left_m[:12], 0.1, atol=1e-5)


def test_straight_shift_gives_the_exact_half_widths() -> None:
    out = fuse_v2.fuse_track_v2(straight(12), straight(12, y=0.1), fuse_v2.FuseV2Config())
    n = out.valid_count
    assert n >= 11
    assert np.allclose(out.y_m[:n], 0.1, atol=TOL)
    assert np.allclose(out.half_width_left_m[:n], 0.4, atol=1e-5)
    assert np.allclose(out.half_width_right_m[:n], 0.6, atol=1e-5)


def test_first_hit_misses_a_parallel_or_receding_edge() -> None:
    ax = np.array([0.0, 1.0])
    ay = np.array([0.5, 0.5])
    bx = np.array([1.0, 2.0])
    by = np.array([0.5, 0.5])
    assert fuse_v2._first_hit(0.0, 0.0, 1.0, 0.0, ax, ay, bx, by) is None  # parallel
    assert fuse_v2._first_hit(0.0, 0.0, 0.0, -1.0, ax, ay, bx, by) is None  # away
    assert fuse_v2._first_hit(0.5, 0.0, 0.0, 1.0, ax, ay, bx, by) == pytest.approx(0.5)
    assert fuse_v2._first_hit(5.0, 0.0, 0.0, 1.0, ax, ay, bx, by) is None  # beside the segments


def test_a_ray_that_misses_ends_the_valid_stations_there(monkeypatch: pytest.MonkeyPatch) -> None:
    geo = straight(12)
    real = fuse_v2._first_hit
    calls = {"n": 0}

    def flaky(*args):  # the left ray of the 7th station (calls come in left, right pairs) misses
        calls["n"] += 1
        return None if calls["n"] == 13 else real(*args)

    monkeypatch.setattr(fuse_v2, "_first_hit", flaky)
    out = fuse_v2.fuse_track_v2(geo, straight(12, y=0.05), fuse_v2.FuseV2Config())
    assert out.valid_count == 6 and out.arrays_ok()
    assert np.all(out.sigma_m[6:] == tr.INVALID_SIGMA_M)


def test_a_miss_at_the_first_station_gives_the_tracer(monkeypatch: pytest.MonkeyPatch) -> None:
    geo = curved(10)
    monkeypatch.setattr(fuse_v2, "_first_hit", lambda *a: None)
    out = fuse_v2.fuse_track_v2(geo, curved(10), fuse_v2.FuseV2Config())
    for name in FIELDS:
        assert np.array_equal(getattr(out, name), getattr(geo, name)), name


def test_the_extension_recovers_the_end_station() -> None:
    geo = curved(10, bend=0.4)
    net = curved(10, bend=0.4)
    net.y_m[:10] += 0.05
    short = fuse_v2.fuse_track_v2(geo, net, fuse_v2.FuseV2Config(edge_end_extension_m=0.0))
    long = fuse_v2.fuse_track_v2(geo, net, fuse_v2.FuseV2Config(edge_end_extension_m=0.2))
    assert long.valid_count >= short.valid_count


@pytest.mark.parametrize("bad", ["empty", "nan", "short"])
def test_bad_net_output_returns_the_tracer_and_is_reported(bad: str) -> None:
    geo = curved(10)
    net = tr.no_track() if bad == "empty" else curved(10)
    if bad == "nan":
        net.x_m[3] = math.nan
    if bad == "short":
        net = curved(0)
    out, outcome = fuse_v2.fuse_track_v2_with_outcome(geo, net, fuse_v2.FuseV2Config())
    assert outcome is fuse.FuseOutcome.NET_UNUSABLE
    for name in FIELDS:
        assert np.array_equal(getattr(out, name), getattr(geo, name)), name
    assert out.valid_count == geo.valid_count


def test_an_empty_tracer_is_reported() -> None:
    out, outcome = fuse_v2.fuse_track_v2_with_outcome(
        tr.no_track(), curved(10), fuse_v2.FuseV2Config()
    )
    assert outcome is fuse.FuseOutcome.TRACER_EMPTY and out.valid_count == 0


def test_a_lone_tracer_station_is_returned_as_it_is() -> None:
    geo = straight(1)
    out = fuse_v2.fuse_track_v2(geo, straight(10, y=0.1), fuse_v2.FuseV2Config())
    assert out.valid_count == 1 and np.array_equal(out.x_m, geo.x_m)


def test_spacing_contract_of_the_output() -> None:
    geo = curved(16, bend=0.3)
    net = curved(16, bend=0.3)
    net.y_m[:] += 0.08
    out = fuse_v2.fuse_track_v2(
        geo, net, fuse_v2.FuseV2Config(edge_end_extension_m=0.2, append_far=False)
    )
    n = out.valid_count
    d = np.hypot(np.diff(out.x_m[:n].astype(float)), np.diff(out.y_m[:n].astype(float)))
    # stations lie on a polyline and are 0.20 m apart along it; the chord of a bend is shorter
    assert np.all(d <= 0.2 + 1e-5) and np.all(d >= 0.19)


def test_the_shift_slope_none_is_a_number_and_validates() -> None:
    cfg = fuse_v2.FuseV2Config(max_shift_slope=fuse_v2.NO_SLOPE_LIMIT)
    cfg.validate()
    out = fuse_v2.fuse_track_v2(straight(12), straight(12, y=0.1), cfg)
    assert out.valid_count >= 11


@pytest.mark.parametrize(
    "bad",
    [
        {"edge_end_extension_m": -0.01},
        {"edge_end_extension_m": 0.21},
        {"edge_end_extension_m": math.nan},
        {"edge_end_extension_m": "0.1"},
    ],
)
def test_config_validation_refuses_bad_extension(bad: dict) -> None:
    with pytest.raises(ValueError, match="edge_end_extension_m"):
        fuse_v2.FuseV2Config(**bad).validate()


def test_config_round_trip_and_exact_keys() -> None:
    cfg = fuse_v2.FuseV2Config(reach_m=0.3, edge_end_extension_m=0.1, max_shift_slope=0.15)
    assert fuse_v2.FuseV2Config.from_dict(cfg.as_dict()) == cfg
    raw = cfg.as_dict()
    del raw["edge_end_extension_m"]
    with pytest.raises(ValueError, match="exactly"):
        fuse_v2.FuseV2Config.from_dict(raw)
    with pytest.raises(ValueError, match="mapping"):
        fuse_v2.FuseV2Config.from_dict(None)
