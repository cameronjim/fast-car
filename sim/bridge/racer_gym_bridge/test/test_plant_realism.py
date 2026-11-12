"""L1 unit tests for the bridge's plant realism options (speed dead band, steering lag, command
delay; all default off). Pure logic in racer_gym_bridge.conversions, no ROS and no f1tenth_gym.

The bridge applies them in this order each control step: the /drive command goes through the
delay FIFO, the speed through the dead band (then the gain and slew limits), the steering through
the gain, offset and limits and then the lag, and the result goes to the gym.
"""

from __future__ import annotations

import math

import pytest
from racer_gym_bridge.conversions import CommandDelay, FirstOrderLag, apply_speed_deadband

DT = 0.02  # a 50 Hz control period


class TestSpeedDeadband:
    def test_a_request_under_the_dead_band_is_zero_and_one_over_it_passes(self):
        assert apply_speed_deadband(0.1, 0.2) == 0.0
        assert apply_speed_deadband(0.3, 0.2) == pytest.approx(0.3)

    def test_the_dead_band_is_on_the_magnitude(self):
        assert apply_speed_deadband(-0.1, 0.2) == 0.0
        assert apply_speed_deadband(-0.3, 0.2) == pytest.approx(-0.3)

    def test_the_edge_passes_and_a_zero_dead_band_is_off(self):
        assert apply_speed_deadband(0.2, 0.2) == pytest.approx(0.2)
        assert apply_speed_deadband(0.01, 0.0) == pytest.approx(0.01)
        assert apply_speed_deadband(0.01, -1.0) == pytest.approx(0.01)


class TestSteeringLag:
    def test_reaches_63_percent_of_a_step_after_one_time_constant(self):
        tau = 0.1
        lag = FirstOrderLag(tau, DT)
        out = [lag.step(1.0) for _ in range(round(tau / DT))]
        assert out[-1] == pytest.approx(1.0 - math.exp(-1.0), abs=1e-12)  # 0.632
        assert 0.0 < out[0] < out[-1] < 1.0
        assert out == sorted(out)

    def test_the_first_control_period_moves_by_the_exact_fraction(self):
        assert FirstOrderLag(0.1, DT).step(1.0) == pytest.approx(1.0 - math.exp(-DT / 0.1))

    def test_settles_and_follows_a_new_target(self):
        lag = FirstOrderLag(0.1, DT)
        for _ in range(200):
            lag.step(0.3)
        assert lag.value == pytest.approx(0.3, abs=1e-9)
        assert lag.step(-0.3) < 0.3

    def test_zero_tau_is_a_pass_through_and_the_initial_value_is_kept(self):
        assert FirstOrderLag(0.0, DT).step(0.7) == pytest.approx(0.7)
        assert FirstOrderLag(0.1, DT, initial=0.5).value == 0.5

    def test_a_non_positive_period_is_refused(self):
        with pytest.raises(ValueError):
            FirstOrderLag(0.1, 0.0)


class TestCommandDelay:
    def test_a_step_comes_out_after_the_configured_time(self):
        fifo = CommandDelay(0.08, DT)
        out = [fifo.push((1.0, 0.2))[0] for _ in range(8)]
        assert fifo.steps == 4
        assert out == [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]  # 4 steps of 20 ms = 80 ms

    def test_values_keep_their_order_and_the_pair_stays_together(self):
        fifo = CommandDelay(0.04, DT)
        out = [fifo.push((float(k), -float(k))) for k in range(1, 6)]
        assert out == [(0.0, 0.0), (0.0, 0.0), (1.0, -1.0), (2.0, -2.0), (3.0, -3.0)]

    def test_no_delay_is_a_pass_through_and_a_fraction_of_a_step_rounds(self):
        assert CommandDelay(0.0, DT).push((0.5, 0.1)) == (0.5, 0.1)
        assert CommandDelay(0.009, DT).steps == 0
        assert CommandDelay(0.011, DT).steps == 1

    def test_a_non_positive_period_is_refused(self):
        with pytest.raises(ValueError):
            CommandDelay(0.1, 0.0)


def test_the_car_like_chain_gives_zero_speed_for_a_crawl_and_a_delayed_lagged_steer():
    """The three together, in the bridge's order: a 0.1 m/s request is a standstill, a 0.3 m/s
    one drives, and a steering step reaches the gym late and softened."""
    delay, lag = CommandDelay(0.06, DT), FirstOrderLag(0.1, DT)
    speeds, steers = [], []
    for k in range(40):
        request = (0.1 if k < 20 else 0.3, 0.2 if k >= 5 else 0.0)
        speed, steer = delay.push(request)
        speeds.append(apply_speed_deadband(speed, 0.2))
        steers.append(lag.step(steer))
    assert speeds[:23] == [0.0] * 23  # the 0.3 m/s request arrives 3 steps after step 20
    assert speeds[23] == pytest.approx(0.3)
    assert steers[7] == 0.0  # the steer request at step 5 arrives at step 8
    assert 0.0 < steers[8] < 0.2 * 0.2  # one period into a 0.1 s lag
    assert steers[-1] == pytest.approx(0.2, abs=0.01)
