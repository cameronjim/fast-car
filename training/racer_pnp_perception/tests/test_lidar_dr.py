"""L1: the LiDAR-model randomisation of trackperc-v3 (`dataset --lidar-dr`). Off by default and
then bit-identical to the earlier generator; on, it only ever loses or shifts returns and never
touches the label."""

from __future__ import annotations

import math

import numpy as np
from racer_pnp_perception import dataset
from racer_pnp_perception import nuisance as nz


def test_ideal_model_is_the_identity_and_draws_nothing() -> None:
    rng = np.random.default_rng(0)
    r = np.array([0.5, np.nan, np.inf, 3.0, 5.9])
    out = nz.apply_lidar_model(r, np.zeros(5), nz.IDEAL_LIDAR, rng)
    assert np.array_equal(out, r, equal_nan=True)
    assert rng.uniform() == np.random.default_rng(0).uniform()


def test_scale_and_bias_apply_to_finite_returns_only() -> None:
    model = nz.LidarModel(0.0, 0.0, 0.0, 1.01, 0.02, 0.0, ())
    r = np.array([1.0, np.nan, np.inf, 4.0])
    out = nz.apply_lidar_model(r, np.zeros(4), model, np.random.default_rng(0))
    assert math.isclose(out[0], 1.03) and math.isclose(out[3], 4.06)
    assert math.isnan(out[1]) and math.isinf(out[2])


def test_dark_sector_loses_only_far_returns_inside_it() -> None:
    model = nz.LidarModel(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, ((0.0, math.radians(20.0), 1.5),))
    bearings = np.radians([0.0, 5.0, 0.0, 30.0, -179.0])
    r = np.array([1.0, 2.0, 3.0, 3.0, 3.0])
    out = nz.apply_lidar_model(r, bearings, model, np.random.default_rng(0))
    assert out[0] == 1.0  # inside, near: kept
    assert math.isnan(out[1]) and math.isnan(out[2])  # inside, beyond 1.5 m: lost
    assert out[3] == 3.0 and out[4] == 3.0  # outside the sector


def test_far_dropout_grows_with_range() -> None:
    model = nz.LidarModel(0.0, 0.0, 0.0, 1.0, 0.0, 0.4, ())
    rng = np.random.default_rng(1)
    near = nz.apply_lidar_model(np.full(20000, 0.6), np.zeros(20000), model, rng)
    far = nz.apply_lidar_model(np.full(20000, 6.0), np.zeros(20000), model, rng)
    assert np.isnan(near).mean() < 0.01
    assert 0.35 < np.isnan(far).mean() < 0.45


def test_drawn_models_stay_in_their_ranges() -> None:
    rng = np.random.default_rng(2)
    for _ in range(500):
        m = nz.draw_lidar_model(rng)
        assert abs(m.yaw_error_rad) <= nz.MOUNT_YAW_ERROR_RAD
        assert abs(m.x_error_m) <= nz.MOUNT_XY_ERROR_M and abs(m.y_error_m) <= nz.MOUNT_XY_ERROR_M
        assert abs(m.range_scale - 1.0) <= nz.RANGE_SCALE_ERROR
        assert abs(m.range_bias_m) <= nz.RANGE_BIAS_M
        assert 0.0 <= m.far_dropout <= nz.FAR_DROPOUT_MAX
        assert len(m.dark_sectors) <= nz.MAX_DARK_SECTORS


def test_generator_default_is_unchanged_and_lidar_dr_changes_scans_not_labels() -> None:
    plain = dataset.generate_chunk(5, 32)
    again = dataset.generate_chunk(5, 32, lidar_dr=False)
    assert np.array_equal(plain["tensor"], again["tensor"])
    assert np.array_equal(plain["label"], again["label"])
    dr = dataset.generate_chunk(5, 32, lidar_dr=True)
    assert dr["tensor"].shape == plain["tensor"].shape
    assert np.isfinite(dr["tensor"]).all() and np.isfinite(dr["label"]).all()
    assert (dr["tensor"] >= 0.0).all() and (dr["tensor"] <= 1.0).all()
    assert not np.array_equal(dr["tensor"], plain["tensor"])
    # The first world and pose are drawn before any LiDAR-model number, so the first label is
    # the same geometry.
    assert np.array_equal(dr["label"][0], plain["label"][0])
