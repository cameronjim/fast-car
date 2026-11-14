"""L1: scale augmentation of real samples (brief 32)."""

from __future__ import annotations

import math

import numpy as np
import pytest
from racer_pnp_perception import augment as au
from racer_pnp_perception import prereg
from racer_pnp_perception.raycast import ScanGeometry, lidar_scan
from racer_track_perception import preprocess
from racer_track_perception import track as tr
from racer_track_perception.vehicle import LiveVehicle

GEOMETRY = ScanGeometry()
LIDAR = au.lidar_position()
LIMITS = au.lidar_range_limits()


def corridor_walls(width: float) -> np.ndarray:
    h = width / 2.0
    return np.array([[-6.0, h, 9.0, h], [-6.0, -h, 9.0, -h]])


def corridor_tensor(width: float, live: LiveVehicle) -> np.ndarray:
    ranges = lidar_scan((0.0, 0.0, 0.0), live.mount, GEOMETRY, corridor_walls(width))
    ranges = np.where(np.isfinite(ranges), ranges, np.inf).astype(np.float32)
    return preprocess.scan_to_tensor(
        ranges,
        GEOMETRY.angle_min_rad,
        GEOMETRY.angle_increment_rad,
        GEOMETRY.range_min_m,
        GEOMETRY.range_max_m,
        live.mount,
        live.body,
    )


def straight_label(width: float, n: int = 16) -> np.ndarray:
    lab = np.zeros((16, 5), dtype=np.float32)
    lab[:, 0] = 0.2 * np.arange(16)
    lab[:, 2:4] = width / 2.0
    lab[:, 4] = math.log(0.05)
    return lab


def curved_label(radius: float = 1.5) -> np.ndarray:
    th = 0.2 * np.arange(16) / radius
    lab = np.zeros((16, 5), dtype=np.float32)
    lab[:, 0] = radius * np.sin(th)
    lab[:, 1] = radius * (1.0 - np.cos(th))
    lab[:, 2:4] = 0.5
    lab[:, 4] = math.log(0.05)
    return lab


def test_the_range_is_the_pre_registered_one() -> None:
    assert prereg.scale_augmentation_range() == (0.75, 1.4)
    k = au.draw_scales(np.random.default_rng(0), 20000)
    assert 0.75 <= k.min() < 0.76 and 1.39 < k.max() <= 1.4
    assert abs(k.mean() - 1.075) < 0.01  # uniform


@pytest.mark.parametrize("k", [0.75, 0.9, 1.25, 1.4])
def test_a_straight_corridor_scaled_by_k_is_a_corridor_k_times_as_wide(
    live: LiveVehicle, k: float
) -> None:
    w = 1.0
    scaled = au.scale_tensor(corridor_tensor(w, live), k, LIMITS)
    direct = corridor_tensor(k * w, live)
    # Clipped bins of the original are unknown below k = 1 and become no return there; compare
    # the bins that are valid and not clipped in both.
    both = (scaled[:, 1] > 0) & (direct[:, 1] > 0) & (direct[:, 0] < 0.999)
    assert both.sum() > 100
    assert np.abs(scaled[:, 0] - direct[:, 0])[both].max() < 1e-3
    # And the label: half widths k w / 2, centreline on y = 0, valid and spaced 0.20 m.
    lab, n = au.scale_label(straight_label(w), 16, k, LIDAR)
    assert np.allclose(lab[:n, 2:4], k * w / 2.0, atol=1e-6)
    assert np.abs(lab[:n, 1]).max() < 1e-6
    d = np.hypot(np.diff(lab[:n, 0]), np.diff(lab[:n, 1]))
    assert np.allclose(d, 0.2, atol=1e-5)


def test_station_zero_is_the_point_nearest_the_origin() -> None:
    lab, _n = au.scale_label(straight_label(1.0), 16, 1.4, LIDAR)
    assert abs(lab[0, 0]) < 1e-6  # the scaled line reaches behind the origin: station 0 at x = 0
    lab, _n = au.scale_label(straight_label(1.0), 16, 0.75, LIDAR)
    assert lab[0, 0] == pytest.approx(LIDAR[0] * (1 - 0.75), abs=1e-6)  # starts ahead: its start


def test_stations_off_the_scaled_label_are_masked_and_finite() -> None:
    lab, n = au.scale_label(straight_label(1.0), 16, 0.75, LIDAR)
    assert n < 16
    assert np.isfinite(lab).all()
    assert (lab[n:] == lab[n - 1]).all()
    from racer_pnp_perception.train import station_weights

    w = station_weights(np.array([n]))[0]
    assert w[:n].sum() == n and w[n:].sum() == 0
    _lab2, n2 = au.scale_label(straight_label(1.0), 16, 1.4, LIDAR)
    assert n2 == 16


@pytest.mark.parametrize("k", [0.8, 1.25, 1.4])
def test_scaling_by_k_then_by_one_over_k_returns_the_sample(live: LiveVehicle, k: float) -> None:
    t = corridor_tensor(1.0, live)
    back = au.scale_tensor(au.scale_tensor(t, k, LIMITS), 1.0 / k, LIMITS)
    unclipped = (t[:, 1] > 0) & (t[:, 0] < 0.999 / max(k, 1.0 / k))
    assert np.abs(back[:, 0] - t[:, 0])[unclipped].max() < 1e-5
    assert (back[:, 1][unclipped] == 1.0).all()
    # The label comes back onto the original polyline with the original half widths. Its first
    # station can differ: re-anchoring at the point nearest the origin drops what lay behind the
    # origin after the first scaling, which is the 4.1 definition and not invertible.
    for lab in (straight_label(1.0), curved_label()):
        out, n = au.scale_label(lab, 16, k, LIDAR)
        back_lab, nb = au.scale_label(out, n, 1.0 / k, LIDAR)
        dense_s = np.linspace(0.0, 15.0, 3001)
        dense = np.stack([np.interp(dense_s, np.arange(16.0), lab[:, c]) for c in range(4)], 1)
        for row in back_lab[:nb]:
            d = np.hypot(dense[:, 0] - row[0], dense[:, 1] - row[1])
            j = int(np.argmin(d))
            assert d[j] <= 0.01, (k, d[j])
            assert np.abs(dense[j, 2:4] - row[2:4]).max() <= 0.01


def test_curved_label_keeps_spacing_after_resampling() -> None:
    for k in (0.75, 1.0, 1.4):
        lab, n = au.scale_label(curved_label(), 16, k, LIDAR)
        d = np.hypot(np.diff(lab[:n, 0]), np.diff(lab[:n, 1]))
        assert d.max() <= tr.STATION_SPACING_M + 1e-6
        assert d.min() >= 0.19  # a chord of an arc of 0.20 m at a radius of at least 1 m


def test_invalid_bins_stay_invalid_and_clipped_bins_follow_the_rule() -> None:
    t = np.ones((1, 2, 360), dtype=np.float32)
    t[0, 1] = 0.0
    t[0, 0, 10], t[0, 1, 10] = 0.5, 1.0  # a return at 3 m
    t[0, 0, 20], t[0, 1, 20] = 1.0, 1.0  # a clipped return
    up = au.scale_tensor(t, 1.4, LIMITS)
    assert up[0, 1].sum() == 2 and up[0, 0, 10] == pytest.approx(0.7) and up[0, 0, 20] == 1.0
    down = au.scale_tensor(t, 0.75, LIMITS)
    assert down[0, 1, 10] == 1.0 and down[0, 0, 10] == pytest.approx(0.375)
    assert down[0, 1, 20] == 0.0 and down[0, 0, 20] == 1.0  # unknown range: no return
    assert (down[0, 1, np.array([0, 5, 100])] == 0).all()
    # A return pushed past the LiDAR's maximum range is no return.
    far = t.copy()
    far[0, 0, 30], far[0, 1, 30] = 0.99, 1.0
    assert au.scale_tensor(far, 1.4, (0.05, 8.0))[0, 1, 30] == 0.0


def test_scale_real_gives_copies_inside_the_contract() -> None:
    rng = np.random.default_rng(0)
    n = 32
    x = np.ones((n, 2, 360), dtype=np.float32)
    x[:, 1] = (rng.uniform(size=(n, 360)) < 0.7).astype(np.float32)
    x[:, 0] = np.where(x[:, 1] > 0, rng.uniform(0.05, 1.0, (n, 360)), 1.0)
    labels = np.stack([curved_label()] * n)
    v = np.full(n, 16)
    sx, sy, sv, k = au.scale_real(x, labels, v, rng)
    assert sx.shape == x.shape and sy.shape == labels.shape and sv.shape == (n,)
    assert np.isfinite(sx).all() and np.isfinite(sy).all()
    assert sx[:, 0].min() >= 0.0 and sx[:, 0].max() <= 1.0
    assert (k >= 0.75).all() and (k <= 1.4).all()
    assert (sv >= 1).all() and (sv <= 16).all()
    assert (sv[k < 0.95] < 16).all() and (sv[k > 1.05] == 16).all()
    assert np.array_equal(labels[0], curved_label())  # the input is not modified
    _, _, _, k0 = au.scale_real(x, labels, v, rng, probability=0.0)
    assert (k0 == 1.0).all()
