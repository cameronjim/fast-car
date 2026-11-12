"""L1 for deadzone.py: launches from rest, steady state brackets, rolling below the dead zone."""

from __future__ import annotations

import numpy as np
import pytest
from racer_pnp_dynamics import deadzone
from racer_pnp_dynamics.bags import GRID_DT_S, Run
from racer_pnp_dynamics.windows import SpeedWindows


def _run(cmd: np.ndarray, wheel: np.ndarray, name: str = "hand") -> Run:
    t = 1000.0 + np.arange(len(cmd)) * GRID_DT_S
    return Run(name, t, cmd, np.zeros(len(cmd)), wheel, [])


def _ramped_launch(threshold: float, rate: float = 0.5, rest_s: float = 3.0) -> Run:
    """Still for rest_s, then the command ramps at `rate`; the wheels turn once it reaches
    `threshold`, rising at 1 m/s2 to the command."""
    n = 10 * 50
    cmd = np.zeros(n)
    wheel = np.zeros(n)
    k0 = int(rest_s * 50)
    for k in range(k0, n):
        cmd[k] = rate * (k - k0) * GRID_DT_S
        wheel[k] = max(0.0, cmd[k] - threshold) if cmd[k] >= threshold else 0.0
    return _run(cmd, wheel)


def test_a_ramped_launch_is_found_with_a_narrow_interval():
    launches = deadzone.find_launches(_ramped_launch(0.20))
    assert len(launches) == 1
    x = launches[0]
    assert 0.20 <= x.cmd_launch_mps < 0.30  # the first sample over 0.03 m/s after the threshold
    assert x.cmd_rest_max_mps < x.cmd_launch_mps
    assert 0.0 < x.width_mps <= deadzone.RESOLVED_WIDTH_MPS + 0.05


def test_a_step_launch_has_a_wide_interval_and_is_not_resolved():
    n = 500
    cmd = np.zeros(n)
    cmd[200:] = 0.5
    wheel = np.zeros(n)
    wheel[205:] = 0.45
    (x,) = deadzone.find_launches(_run(cmd, wheel))
    assert x.cmd_launch_mps == 0.5 and x.cmd_rest_max_mps == 0.0 and x.width_mps == 0.5
    summary = deadzone.launch_summary([x])
    assert summary["all"]["n"] == 1 and summary["resolved"]["n"] == 0


def test_a_car_that_stood_still_under_a_high_command_has_no_interval():
    """Held under 0.4 m/s for two seconds and then moving: not an observation of a threshold."""
    n = 600
    cmd = np.zeros(n)
    cmd[100:] = 0.4
    wheel = np.zeros(n)
    wheel[250:] = 0.35
    (x,) = deadzone.find_launches(_run(cmd, wheel))
    assert x.width_mps == pytest.approx(0.0, abs=1e-9)
    assert deadzone.launch_summary([x])["resolved"]["n"] == 0


def test_no_launch_without_rest_or_without_odometry():
    n = 400
    moving = _run(np.full(n, 0.4), np.full(n, 0.4))
    assert deadzone.find_launches(moving) == []
    no_odom = _run(np.full(n, 0.4), np.full(n, np.nan))
    assert deadzone.find_launches(no_odom) == []
    reverse = np.zeros(n)
    reverse[100:] = -0.4
    wheel = np.zeros(n)
    wheel[110:] = 0.3
    assert deadzone.find_launches(_run(reverse, wheel)) == []  # negative command: skipped


def test_launch_summary_percentiles():
    runs = [_ramped_launch(th) for th in (0.15, 0.20, 0.25)]
    launches = [x for r in runs for x in deadzone.find_launches(r)]
    s = deadzone.launch_summary(launches)["all"]
    assert s["n"] == 3 and s["p10_mps"] < s["median_mps"] < s["p90_mps"]
    assert deadzone.launch_summary([])["all"] == {"n": 0}


def test_steady_bins_and_brackets_find_the_still_and_the_moving_side():
    n = 3000
    cmd = np.zeros(n)
    wheel = np.zeros(n)
    cmd[300:900] = 0.07  # held, wheels still
    cmd[1200:1800] = 0.30  # held, rolling
    wheel[1230:1800] = 0.28
    cmd[2100:2700] = 0.5
    wheel[2130:2700] = 0.47
    bins = deadzone.steady_bins([_run(cmd, wheel)])
    brackets = deadzone.steady_brackets(bins)
    assert brackets["largest_still_bin_hi_mps"] == pytest.approx(0.08)
    assert brackets["smallest_moving_bin_lo_mps"] == pytest.approx(0.30)
    assert brackets["consistent"]


def test_hysteresis_in_the_steady_state_is_reported_as_inconsistent():
    n = 2400
    cmd = np.zeros(n)
    wheel = np.zeros(n)
    cmd[300:900] = 0.20  # still at 0.20 ...
    cmd[1200:1800] = 0.12  # ... but rolling at 0.12
    wheel[1230:1800] = 0.1
    brackets = deadzone.steady_brackets(deadzone.steady_bins([_run(cmd, wheel)]))
    assert brackets["largest_still_bin_hi_mps"] == pytest.approx(0.21)
    assert brackets["smallest_moving_bin_lo_mps"] == pytest.approx(0.12)
    assert not brackets["consistent"]


def test_sparse_bins_do_not_set_a_bracket():
    cmd = np.zeros(2000)
    wheel = np.zeros(2000)
    cmd[100:140] = 0.2  # 40 samples held, 9 steady
    brackets = deadzone.steady_brackets(deadzone.steady_bins([_run(cmd, wheel)]))
    assert brackets["largest_still_bin_hi_mps"] is None


def test_rolling_below_the_dead_zone_counts_windows_in_the_band():
    v = np.tile(np.linspace(0.3, 0.1, 21), (4, 1))
    cmds = np.zeros((4, 20, 2))
    cmds[0, :, 0] = 0.1  # in the band
    cmds[1, :, 0] = 0.1
    cmds[2, :, 0] = 0.5  # above the dead zone
    cmds[3, :, 0] = 0.1
    v[3, 0] = 0.05  # not rolling at the start
    w = SpeedWindows(v, cmds)
    out = deadzone.rolling_below_the_dead_zone(w, 0.2, model_v_end=np.array([0.0, 0.0, 0.5, 0.0]))
    assert out["windows"] == 2
    assert out["median_end_speed_mps"] == pytest.approx(0.1)
    assert out["share_still_turning_at_end"] == 1.0
    assert out["model_median_end_speed_mps"] == 0.0
    assert deadzone.rolling_below_the_dead_zone(w, 0.05)["windows"] == 0
    empty = SpeedWindows(np.zeros((0, 21)), np.zeros((0, 20, 2)))
    assert deadzone.rolling_below_the_dead_zone(empty, 0.2) == {"windows": 0}
