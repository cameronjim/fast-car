"""L1/L4: the batched refinement (`refine._refine_stations`, all stations against all points at
once) gives the same output as the per-station loop it replaced (overnight 2026-11-23, W2: the
refinement was most of the net backend's per-scan CPU). The loop is frozen here as the reference,
and both run on the committed real-scan fixtures through the released v0 net and on random
corridors. Same centres, half widths, sigma and valid_count to 1e-9 m."""

from __future__ import annotations

import math

import numpy as np
import pytest
from make_real_scans_2026_11_13 import SCANS as SCANS_1113
from make_real_scans_2026_11_13 import dequantise
from make_refined_golden import RELEASE_WEIGHTS
from racer_track_perception import core, geometric, refine, selfcheck
from racer_track_perception import track as tr
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle
from regen_golden import SCANS as SCANS_1111

TOL_M = 1e-9


def _station_reference(c, t, hl_net, hr_net, pts, cfg):  # type: ignore[no-untyped-def]
    """The per-station loop body as it was before batching (git cce6bd3, refine.py)."""
    rel = pts - c
    along = rel @ t
    lat = -rel[:, 0] * t[1] + rel[:, 1] * t[0]
    dist = np.hypot(rel[:, 0], rel[:, 1])
    near = (np.abs(along) <= cfg.window_m) & (dist <= cfg.feeler_max_m)
    nl = geometric._nearest(dist, near & (lat > 0.0))
    nr = geometric._nearest(dist, near & (lat < 0.0))
    if nl is None and nr is None:
        return c, hl_net, hr_net, False
    c_new, hl, hr = refine._walls(c, t, hl_net, hr_net, pts, nl, nr, cfg)
    if not cfg.clearance_cap:
        return c_new, hl, hr, True
    rel = pts - c_new
    d_new = np.hypot(rel[:, 0], rel[:, 1])
    lat_new = -rel[:, 0] * t[1] + rel[:, 1] * t[0]
    left = d_new[lat_new >= 0.0]
    right = d_new[lat_new <= 0.0]
    if len(left):
        hl = min(hl, float(left.min()))
    if len(right):
        hr = min(hr, float(right.min()))
    return c_new, hl, hr, True


def _stations_reference(track, pts, cfg):  # type: ignore[no-untyped-def]
    n = int(track.valid_count)
    x = track.x_m[:n].astype(np.float64)
    y = track.y_m[:n].astype(np.float64)
    hl = track.half_width_left_m[:n].astype(np.float64)
    hr = track.half_width_right_m[:n].astype(np.float64)
    tan = selfcheck._tangents(x, y)
    new = np.zeros((n, 4))
    supported = False
    for i in range(n):
        c, left, right, hit = _station_reference(
            np.array([x[i], y[i]]), tan[i], float(hl[i]), float(hr[i]), pts, cfg
        )
        new[i] = (c[0], c[1], left, right)
        supported = supported or hit
    return new, supported


def _stations_batched(track, pts, cfg):  # type: ignore[no-untyped-def]
    n = int(track.valid_count)
    x = track.x_m[:n].astype(np.float64)
    y = track.y_m[:n].astype(np.float64)
    tan = selfcheck._tangents(x, y)
    return refine._refine_stations(
        np.stack([x, y], axis=1),
        tan,
        track.half_width_left_m[:n].astype(np.float64),
        track.half_width_right_m[:n].astype(np.float64),
        pts,
        cfg,
    )


def _real_cases(live: LiveVehicle):  # type: ignore[no-untyped-def]
    from racer_track_perception import preprocess

    net = CircConv1dV0.from_npz(RELEASE_WEIGHTS)
    for path, quantised in ((SCANS_1111, False), (SCANS_1113, True)):
        with np.load(path) as d:
            ranges = dequantise(d["ranges_q"]) if quantised else d["ranges"]
            meta = (
                float(d["angle_min"]),
                float(d["angle_increment"]),
                float(d["range_min"]),
                float(d["range_max"]),
            )
        for r in ranges:
            tensor, points = preprocess.scan_to_tensor_and_points(r, *meta, live.mount, live.body)
            track = core.net_output_to_track(net.forward(tensor), 0.25)
            if track.valid_count >= 2 and len(points):
                yield track, refine._clean_points(points)


@pytest.mark.parametrize("cap", [True, False])
def test_batched_stations_match_the_loop_on_real_scans(live: LiveVehicle, cap: bool) -> None:
    cfg = refine.RefineConfig(clearance_cap=cap)
    count = 0
    for track, pts in _real_cases(live):
        want, want_hit = _stations_reference(track, pts, cfg)
        got, got_hit = _stations_batched(track, pts, cfg)
        assert got_hit == want_hit
        assert np.abs(got - want).max() <= TOL_M
        count += 1
    assert count > 50


def test_refine_track_on_real_scans_is_unchanged(live: LiveVehicle) -> None:
    """The whole refinement (resample and assemble included) equals the loop's output."""
    cfg = refine.RefineConfig()
    for track, pts in _real_cases(live):
        got = refine.refine_track(track, pts, cfg)
        new, hit = _stations_reference(track, pts, cfg)
        if not hit:
            assert got.valid_count == track.valid_count
            continue
        sg = track.sigma_m[: track.valid_count].astype(np.float64)
        res = refine._resample(new[:, 0], new[:, 1], new[:, 2], new[:, 3], sg)
        assert res is not None
        stations, n_valid = res
        sigma = stations[:, 4].copy()
        sigma[n_valid:] = tr.INVALID_SIGMA_M
        want = tr.assemble(
            stations[:, 0],
            stations[:, 1],
            stations[:, 2],
            stations[:, 3],
            sigma,
            max(cfg.sigma_max_m, float(sg.max()) + 1e-6),
            valid_cap=n_valid,
        )
        if want.valid_count < 1 or not want.arrays_ok():
            continue
        assert got.valid_count == want.valid_count
        k = want.valid_count
        for a, b in (
            (got.x_m, want.x_m),
            (got.y_m, want.y_m),
            (got.half_width_left_m, want.half_width_left_m),
            (got.half_width_right_m, want.half_width_right_m),
            (got.sigma_m, want.sigma_m),
        ):
            assert np.abs(np.asarray(a[:k]) - np.asarray(b[:k])).max() <= TOL_M


def test_batched_stations_match_the_loop_on_random_corridors() -> None:
    rng = np.random.default_rng(23)
    cfg = refine.RefineConfig()
    count = 0
    for _ in range(300):
        n = int(rng.integers(2, tr.K_STATIONS + 1))
        heading = rng.uniform(-math.pi, math.pi)
        curv = rng.uniform(-1.5, 1.5)
        s = np.arange(n) * tr.STATION_SPACING_M
        th = heading + curv * s
        x = np.cumsum(np.r_[0.0, np.cos(th[:-1]) * tr.STATION_SPACING_M])
        y = np.cumsum(np.r_[0.0, np.sin(th[:-1]) * tr.STATION_SPACING_M])
        x, y = x + rng.normal(0, 0.05, n), y + rng.normal(0, 0.05, n)
        hl = rng.uniform(0.1, 0.9, n)
        hr = rng.uniform(0.1, 0.9, n)
        pad = tr.K_STATIONS - n
        sigma = np.r_[np.full(n, 0.05), np.full(pad, 1.0)]
        x, y, hl, hr = (np.r_[a, np.full(pad, a[-1])] for a in (x, y, hl, hr))
        track = tr.assemble(x, y, hl, hr, sigma, 0.25)
        assert track.valid_count == n
        pts = rng.uniform(-3, 3, (int(rng.integers(0, 400)), 2))
        if track.valid_count < 2 or len(pts) == 0:
            continue
        want, want_hit = _stations_reference(track, pts, cfg)
        got, got_hit = _stations_batched(track, pts, cfg)
        assert got_hit == want_hit
        assert np.abs(got - want).max() <= TOL_M
        count += 1
    assert count > 250
