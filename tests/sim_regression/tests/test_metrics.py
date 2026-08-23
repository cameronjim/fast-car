"""Unit tests for the derived-scalar helpers (racer_sim_regression/metrics.py) against
hand-computed cases -- claude-docs/12-testing.md L1: "standard pytest ... coverage of the
math against hand-computed cases". Pure numpy, no gym/racer_gym needed."""

from __future__ import annotations

import numpy as np
import pytest
from racer_sim_regression.metrics import (
    first_time_below,
    peak_overshoot_frac,
    settling_time_s,
    steady_state_mean,
)


class TestSteadyStateMean:
    def test_mean_of_last_window(self):
        values = np.array([0.0, 0.0, 10.0, 10.0, 10.0])
        assert steady_state_mean(values, window=3) == pytest.approx(10.0)

    def test_window_larger_than_array_uses_whole_array(self):
        values = np.array([1.0, 2.0, 3.0])
        assert steady_state_mean(values, window=10) == pytest.approx(2.0)

    def test_rejects_non_positive_window(self):
        with pytest.raises(ValueError, match="window"):
            steady_state_mean(np.array([1.0]), window=0)


class TestSettlingTimeS:
    def test_already_settled_returns_first_time(self):
        times = np.array([0.0, 1.0, 2.0])
        values = np.array([5.0, 5.0, 5.0])
        assert settling_time_s(times, values, band=0.01) == pytest.approx(0.0)

    def test_returns_sample_after_last_out_of_band_sample(self):
        times = np.array([0.0, 1.0, 2.0, 3.0, 4.0])
        values = np.array([0.0, 8.0, 4.9, 5.0, 5.0])  # final = 5.0
        # |0-5|=5 (out), |8-5|=3 (out), |4.9-5|=0.1 (in, band=0.2), |5-5|=0, |5-5|=0
        result = settling_time_s(times, values, band=0.2)
        assert result == pytest.approx(2.0)

    def test_never_settles_returns_last_time(self):
        times = np.array([0.0, 1.0, 2.0])
        values = np.array([0.0, 10.0, 5.0])  # final=5.0; index1 (|10-5|=5) out of band=0.01
        assert settling_time_s(times, values, band=0.01) == pytest.approx(2.0)

    def test_last_sample_itself_out_of_band_returns_last_time(self):
        # The out-of-band sample IS the final sample (last_outside + 1 == len(times)).
        times = np.array([0.0, 1.0, 2.0])
        values = np.array([5.0, 5.0, 100.0])  # final=100.0; every earlier sample is "outside"
        assert settling_time_s(times, values, band=0.01) == pytest.approx(2.0)

    def test_rejects_negative_band(self):
        with pytest.raises(ValueError, match="band"):
            settling_time_s(np.array([0.0]), np.array([1.0]), band=-1.0)


class TestPeakOvershootFrac:
    def test_positive_overshoot(self):
        values = np.array([0.0, 1.2, 1.0, 1.0])
        assert peak_overshoot_frac(values, steady_state=1.0) == pytest.approx(0.2)

    def test_zero_steady_state_returns_peak_magnitude(self):
        values = np.array([0.0, -3.0, 0.0])
        assert peak_overshoot_frac(values, steady_state=0.0) == pytest.approx(3.0)

    def test_negative_steady_state_uses_magnitude(self):
        values = np.array([0.0, -1.5, -1.0])
        assert peak_overshoot_frac(values, steady_state=-1.0) == pytest.approx(0.5)


class TestFirstTimeBelow:
    def test_finds_first_crossing(self):
        times = np.array([0.0, 1.0, 2.0, 3.0])
        values = np.array([5.0, 3.0, 0.5, 0.1])
        assert first_time_below(times, values, threshold=1.0) == pytest.approx(2.0)

    def test_never_crosses_returns_last_time(self):
        times = np.array([0.0, 1.0, 2.0])
        values = np.array([5.0, 4.0, 3.0])
        assert first_time_below(times, values, threshold=1.0) == pytest.approx(2.0)
