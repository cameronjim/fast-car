"""L1 tests for racer_gym_bridge.sensor_noise (no ROS, no gym)."""

from __future__ import annotations

import math

import numpy as np
import pytest
from racer_gym_bridge.sensor_noise import WheelSpeedNoise, degrade_scan, publish_divider


def test_publish_divider():
    assert publish_divider(100.0, 0.0) == 1
    assert publish_divider(100.0, 10.0) == 10
    assert publish_divider(100.0, 50.0) == 2
    assert publish_divider(100.0, 30.0) == 3
    assert publish_divider(100.0, 500.0) == 1
    assert publish_divider(0.0, 10.0) == 1


def test_perfect_wheel_speed_is_untouched():
    noise = WheelSpeedNoise()
    rng = np.random.default_rng(0)
    assert noise.is_perfect
    assert noise.measure(1.234, 5.0, rng) == 1.234


def test_wheel_speed_scale_and_slip():
    noise = WheelSpeedNoise(scale=1.02, slip_s=0.01)
    rng = np.random.default_rng(0)
    assert noise.measure(2.0, 3.0, rng) == pytest.approx(2.04 + 0.03)
    # Reverse keeps the sign; braking slip lowers the reading.
    assert noise.measure(-1.0, 0.0, rng) == pytest.approx(-1.02)
    assert noise.measure(1.0, -2.0, rng) == pytest.approx(1.02 - 0.02)


def test_wheel_speed_noise_statistics():
    noise = WheelSpeedNoise(noise_std_mps=0.05)
    rng = np.random.default_rng(1)
    samples = np.array([noise.measure(1.0, 0.0, rng) for _ in range(20000)])
    assert samples.mean() == pytest.approx(1.0, abs=0.003)
    assert samples.std() == pytest.approx(0.05, rel=0.05)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"scale": 0.0},
        {"scale": -1.0},
        {"scale": math.nan},
        {"noise_std_mps": -0.1},
        {"slip_s": -0.1},
        {"slip_s": math.inf},
    ],
)
def test_wheel_speed_noise_rejects_nonsense(kwargs):
    with pytest.raises(ValueError):
        WheelSpeedNoise(**kwargs)


def test_degrade_scan_defaults_copy_unchanged():
    ranges = np.array([0.5, 1.0, 20.0])
    out = degrade_scan(ranges, 0.0, 0.0, np.random.default_rng(0))
    np.testing.assert_array_equal(out, ranges)
    assert out is not ranges


def test_degrade_scan_range_max_gives_inf():
    out = degrade_scan(np.array([0.5, 11.9, 12.1, 30.0]), 0.0, 12.0, np.random.default_rng(0))
    assert out[0] == 0.5 and out[1] == 11.9
    assert math.isinf(out[2]) and math.isinf(out[3])


def test_degrade_scan_noise_is_unbiased_and_non_negative():
    rng = np.random.default_rng(2)
    out = degrade_scan(np.full(50000, 3.0), 0.02, 0.0, rng)
    assert out.mean() == pytest.approx(3.0, abs=0.001)
    assert out.std() == pytest.approx(0.02, rel=0.05)
    near = degrade_scan(np.full(1000, 0.001), 0.05, 0.0, rng)
    assert (near >= 0.0).all()


def test_degrade_scan_rejects_nonsense():
    with pytest.raises(ValueError):
        degrade_scan(np.zeros(3), -0.1, 0.0, np.random.default_rng(0))
    with pytest.raises(ValueError):
        degrade_scan(np.zeros(3), 0.0, -1.0, np.random.default_rng(0))
