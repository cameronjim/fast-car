"""unit tests for the shared lidar scan preprocessing, no ros."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "preprocessing"))

from scan import (  # noqa: E402
    MAX_RANGE_M,
    downsample_indices,
    downsample_scan,
    normalize_scan,
)

NUM_FEATURES = 181


def test_1081_ray_scan_downsamples_to_every_sixth_ray():
    idx = downsample_indices(1081, NUM_FEATURES)

    assert np.array_equal(idx, np.arange(0, 1081, 6))


def test_1080_ray_scan_still_yields_exactly_181_in_bounds_rays():
    idx = downsample_indices(1080, NUM_FEATURES)

    assert len(idx) == NUM_FEATURES
    assert idx[0] == 0
    assert idx[-1] == 1079
    assert idx.max() < 1080
    assert np.all(np.diff(idx) > 0)


def test_downsample_picks_real_rays_and_never_pads():
    ranges = np.arange(1080, dtype=np.float32) * 0.001
    idx = downsample_indices(1080, NUM_FEATURES)

    picked = downsample_scan(ranges, NUM_FEATURES)

    assert np.allclose(picked, ranges[idx])


def test_non_finite_rays_become_max_range():
    ranges = np.full(1081, 5.0, dtype=np.float32)
    ranges[0] = np.inf
    ranges[6] = np.nan
    ranges[12] = -np.inf

    picked = downsample_scan(ranges, NUM_FEATURES)

    assert picked[0] == MAX_RANGE_M
    assert picked[1] == MAX_RANGE_M
    assert picked[2] == MAX_RANGE_M
    assert picked[3] == 5.0


def test_ranges_are_clipped_into_the_trained_window():
    ranges = np.full(1081, 5.0, dtype=np.float32)
    ranges[0] = 500.0
    ranges[6] = -3.0

    picked = downsample_scan(ranges, NUM_FEATURES)

    assert picked[0] == MAX_RANGE_M
    assert picked[1] == 0.0


def test_downsample_stays_float32_for_torch():
    ranges = np.full(1081, 5.0, dtype=np.float32)

    assert downsample_scan(ranges, NUM_FEATURES).dtype == np.float32


def test_downsample_accepts_a_plain_list_of_ranges():
    ranges = [5.0] * 1081

    picked = downsample_scan(ranges, NUM_FEATURES)

    assert len(picked) == NUM_FEATURES
    assert np.all(picked == 5.0)


def test_normalize_applies_the_exported_scaler():
    ranges_m = np.array([0.0, 5.0, 10.0], dtype=np.float32)
    lidar_scale = np.full(3, 0.1, dtype=np.float32)
    lidar_min = np.zeros(3, dtype=np.float32)

    assert np.allclose(normalize_scan(ranges_m, lidar_scale, lidar_min), [0.0, 0.5, 1.0])
