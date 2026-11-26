"""Label geometry against a map (label_map_check.py), on synthetic data."""

from __future__ import annotations

import math

import numpy as np
from racer_pnp_perception import label_map_check as lm


def test_pose_composes_map_odom_and_interpolated_odom_base() -> None:
    tf = {
        "map_odom": np.array([[0.0, 1.0, 0.0, math.pi / 2]]),
        "odom_base": np.array([[1.0, 0.0, 0.0, 0.0], [1.1, 0.2, 0.0, 0.0]]),
    }
    p = lm.pose_at(tf, 1.05)
    assert p is not None and np.allclose(p, [1.0, 0.1, math.pi / 2])
    assert lm.pose_at(tf, 0.5) is None


def test_a_centred_straight_label_has_no_centring_error_and_exact_spacing() -> None:
    x = np.arange(-1.0, 4.0, 0.05)
    walls = np.r_[np.c_[x, np.full_like(x, 0.5)], np.c_[x, np.full_like(x, -0.5)]]
    label = np.zeros((16, 5))
    label[:, 0] = np.arange(16) * 0.2
    label[:, 2:4] = 0.5
    s = lm.station_numbers(label, 16, walls)
    assert np.allclose(s["dl"], 0.5, atol=0.03) and np.allclose(s["dr"], 0.5, atol=0.03)
    assert np.allclose(s["gap"][:15], 0.2) and np.isinf(s["radius"][1:15]).all()
    summary = lm.summarise([s])
    assert summary["centring_error_mean_m"] < 0.01 and summary["wall_distance_error_mean_m"] < 0.03
    shifted = label.copy()
    shifted[:, 1] = 0.2
    assert lm.summarise([lm.station_numbers(shifted, 16, walls)])["centring_error_mean_m"] > 0.15
