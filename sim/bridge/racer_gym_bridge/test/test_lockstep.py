"""L1 tests of racer_gym_bridge.lockstep (GitHub issue 156): simulated time and lockstep pacing.

Pure logic, no ROS: runs everywhere. Each branch of the gate and the pacer has a case,
including the boundaries (a lag exactly at the limit, a wait exactly at the timeout).
"""

from __future__ import annotations

import pytest
from racer_gym_bridge.lockstep import (
    SIM_TIME_START_NS,
    LagGate,
    SimClock,
    WallPacer,
    parse_ack_key,
)


class TestSimClock:
    def test_starts_away_from_zero(self):
        assert SimClock(0.01).now_ns == SIM_TIME_START_NS > 0

    def test_ticks_are_exact_integers(self):
        clock = SimClock(0.01)
        for _ in range(100_000):
            clock.tick()
        # 1000 s of 10 ms steps, with no float drift at all.
        assert clock.now_ns == SIM_TIME_START_NS + 1000 * 10**9

    def test_rejects_a_non_positive_timestep(self):
        for bad in (0.0, -0.01):
            with pytest.raises(ValueError):
                SimClock(bad)


class TestWallPacer:
    def test_first_step_is_due_at_once(self):
        assert WallPacer(0.01, 1.0).due(0.0)

    def test_not_due_before_the_period_and_due_at_it(self):
        pacer = WallPacer(0.01, 1.0)
        pacer.taken(10.0)
        assert not pacer.due(10.0099)
        assert pacer.due(10.01)

    def test_rtf_scales_the_wall_period(self):
        assert WallPacer(0.01, 0.5).wall_period_s == pytest.approx(0.02)
        assert WallPacer(0.01, 2.0).wall_period_s == pytest.approx(0.005)

    def test_unpaced_is_always_due(self):
        pacer = WallPacer(0.01, 0.0)
        pacer.taken(1.0)
        assert pacer.wall_period_s == 0.0
        assert pacer.due(1.0)

    def test_a_late_poll_does_not_slow_the_average_rate(self):
        pacer = WallPacer(0.01, 1.0)
        pacer.taken(0.0)  # next due 0.01
        pacer.taken(0.013)  # 3 ms late: the deadline stays on the grid, next due 0.02
        assert not pacer.due(0.0199)
        assert pacer.due(0.02)

    def test_a_step_a_whole_period_late_restarts_the_deadline(self):
        pacer = WallPacer(0.01, 1.0)
        pacer.taken(0.0)
        pacer.taken(5.0)  # the machine stalled: no burst of catch-up steps
        assert not pacer.due(5.0099)
        assert pacer.due(5.01)


class TestLagGate:
    def gate(self, **kwargs):
        return LagGate({"/odom": 0.0, "tf:odom": 0.2}, max_lag_s=0.1, timeout_s=0.2, **kwargs)

    def test_rejects_bad_limits(self):
        with pytest.raises(ValueError):
            LagGate({}, max_lag_s=-0.1, timeout_s=0.2)
        with pytest.raises(ValueError):
            LagGate({}, max_lag_s=0.1, timeout_s=0.0)

    def test_open_while_nothing_has_acknowledged(self):
        # A node that is still starting must never hold the sim back.
        assert self.gate().open(10 * 10**9, 0.0)

    def test_open_when_every_output_is_current(self):
        gate = self.gate()
        gate.ack("/odom", 5 * 10**9)
        gate.ack("tf:odom", 5 * 10**9 + 2 * 10**8)
        assert gate.open(5 * 10**9, 0.0)

    def test_lag_exactly_at_the_limit_is_open_and_over_it_is_shut(self):
        gate = self.gate()
        gate.ack("/odom", 5 * 10**9)
        assert gate.open(5 * 10**9 + 10**8, 0.0)
        assert not gate.open(5 * 10**9 + 10**8 + 1, 0.0)

    def test_the_lead_is_subtracted_from_a_future_dated_output(self):
        gate = self.gate()
        gate.ack("tf:odom", 5 * 10**9)  # data from 4.8 s
        assert gate.behind(4_900_000_000) == []
        assert gate.behind(5_000_000_001) == ["tf:odom"]

    def test_the_slowest_output_decides(self):
        gate = self.gate()
        gate.ack("/odom", 9 * 10**9)
        gate.ack("tf:odom", 5 * 10**9)
        assert gate.behind(9 * 10**9) == ["tf:odom"]
        assert not gate.open(9 * 10**9, 0.0)

    def test_opens_again_when_the_output_catches_up(self):
        gate = self.gate()
        gate.ack("/odom", 5 * 10**9)
        assert not gate.open(6 * 10**9, 0.0)
        gate.ack("/odom", 6 * 10**9)
        assert gate.open(6 * 10**9, 0.01)

    def test_ack_never_moves_an_output_backwards(self):
        gate = self.gate()
        gate.ack("/odom", 5 * 10**9)
        gate.ack("/odom", 1 * 10**9)  # a late, old message
        assert gate.behind(5 * 10**9) == []

    def test_an_unknown_key_is_ignored(self):
        gate = self.gate()
        gate.ack("/other", 5 * 10**9)
        assert gate.open(99 * 10**9, 0.0)

    def test_a_dead_estimator_times_out_instead_of_hanging(self):
        gate = LagGate({"/odom": 0.0}, max_lag_s=0.1, timeout_s=0.25)  # 0.25: exact in binary
        gate.ack("/odom", 5 * 10**9)
        assert not gate.open(9 * 10**9, 100.0)
        assert not gate.open(9 * 10**9, 100.2499)
        assert gate.open(9 * 10**9, 100.25)  # exactly the timeout
        assert gate.timeouts == 1
        # the wait restarts: the next shut check does not open at once
        assert not gate.open(9 * 10**9, 100.5)
        assert gate.blocked_checks == 4

    def test_a_gate_that_opens_normally_resets_the_wait(self):
        gate = self.gate()
        gate.ack("/odom", 5 * 10**9)
        assert not gate.open(6 * 10**9, 50.0)
        gate.ack("/odom", 6 * 10**9)
        assert gate.open(6 * 10**9, 50.1)
        assert not gate.open(7 * 10**9, 50.15)  # a fresh wait, not 0.15 s into the old one
        assert gate.timeouts == 0

    def test_keys_lists_the_configured_outputs(self):
        assert self.gate().keys == ["/odom", "tf:odom"]


@pytest.mark.parametrize(
    ("spec", "expected"),
    [
        ("/odom", ("/odom", 0.0)),
        ("/odom:0.5", ("/odom", 0.5)),
        ("tf:odom", ("tf:odom", 0.0)),
        ("tf:odom:0.2", ("tf:odom", 0.2)),
        ("tf:base_link:0", ("tf:base_link", 0.0)),
        ("/ns/odom:notanumber", ("/ns/odom:notanumber", 0.0)),
    ],
)
def test_parse_ack_key(spec, expected):
    assert parse_ack_key(spec) == expected
