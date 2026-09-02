"""lidar scan preprocessing shared by the training pipeline and every deploy node."""

import numpy as np

MAX_RANGE_M = 10.0


def downsample_indices(num_rays, num_features):
    """evenly spaced ray indices, so 1080 and 1081 scans both give num_features real rays."""
    return np.round(np.linspace(0, num_rays - 1, num_features)).astype(int)


def downsample_scan(ranges, num_features, max_range_m=MAX_RANGE_M):
    """downsample by index, treat non-finite rays as max range, clip to [0, max_range_m]."""
    ranges = np.asarray(ranges, dtype=np.float32)
    picked = ranges[downsample_indices(len(ranges), num_features)]
    picked = np.where(np.isfinite(picked), picked, max_range_m)
    return np.clip(picked, 0.0, max_range_m)


def normalize_scan(ranges_m, lidar_scale, lidar_min):
    """apply the exported min-max scaler, turning metre ranges into the policy input."""
    return ranges_m * lidar_scale + lidar_min
