"""The rate allowance count of tools/pnp_replay (roadmap P3.4), the recorded tick grid it needs,
the options that let one run cover runs with different caps, and the committed 2026-11-13 baseline.

The count: consecutive publishes whose steering step, or growth of the speed's magnitude, is above
0.45 of the vehicle's rate limit times the elapsed time between them (elapsed clamped to one tick).
Replayed on the uniform 0.02 s grid the planner never does that; replayed on the arrival times of
the recorded /pnp/planner/status (the car's bunched ticks) it does, which is what brief 25 removes.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np
import pytest
from pnp_replay import cut_fixture, metrics, planner_stream, segments, vehicle

TESTS = Path(__file__).resolve().parent
REPO_ROOT = TESTS.parents[1]
FAULT_FIXTURE = TESTS / "fixtures" / "pnp_fault_2026-11-13.npz"
BASELINE = REPO_ROOT / "tools" / "pnp_replay" / "baselines" / "2026-11-13-v0.1.json"
CAR_BUNDLE = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-car-2026-11-12"

STEER_RATE = 3.2
ACCEL = 9.51
TICK = 0.02


def allowance(now: list[float], steer: list[float], speed: list[float]) -> dict:
    return metrics.rate_allowance(
        np.array(now), np.array(steer), np.array(speed), STEER_RATE, ACCEL, TICK
    )


def test_a_step_of_exactly_the_allowance_is_not_a_violation_and_a_bit_over_is() -> None:
    step_s = 0.45 * STEER_RATE * TICK  # 0.0288 rad
    step_v = 0.45 * ACCEL * TICK  # 0.08559 m/s
    ok = allowance([0.0, 0.02], [0.0, step_s], [0.0, step_v])
    assert ok["violation_pairs"] == 0 and ok["pairs"] == 1
    over = allowance([0.0, 0.02], [0.0, step_s * 1.01], [0.0, step_v])
    assert (over["steering"], over["speed_up"], over["violation_pairs"]) == (1, 0, 1)
    over = allowance([0.0, 0.02], [0.0, 0.0], [0.5, 0.5 + step_v * 1.01])
    assert (over["steering"], over["speed_up"], over["violation_pairs"]) == (0, 1, 1)


def test_bunched_ticks_get_a_smaller_allowance_and_a_gap_does_not_get_a_larger_one() -> None:
    full = 0.45 * STEER_RATE * TICK
    bunched = allowance([0.0, 0.005], [0.0, full], [0.5, 0.5])  # 5 ms apart, a full step
    assert bunched["steering"] == 1
    assert allowance([0.0, 0.005], [0.0, full / 4], [0.5, 0.5])["steering"] == 0
    gap = allowance([0.0, 0.1], [0.0, full * 2], [0.5, 0.5])  # 100 ms apart: clamped to one tick
    assert gap["steering"] == 1
    assert allowance([0.0, 0.1], [0.0, full], [0.5, 0.5])["steering"] == 0


def test_only_the_growth_of_the_speed_is_limited_not_slowing_and_both_directions_count() -> None:
    v = 0.45 * ACCEL * TICK
    assert allowance([0.0, 0.02], [0, 0], [1.0, 0.0])["speed_up"] == 0  # braking to zero
    assert allowance([0.0, 0.02], [0, 0], [-0.3, -0.3 - 2 * v])["speed_up"] == 1  # reverse grows
    assert allowance([0.0, 0.02], [0, 0], [-0.3, -0.3 + 2 * v])["speed_up"] == 0  # reverse slows
    # a change of sign is growth from zero: -0.3 to +0.3 is a step of 0.3, not of 0
    assert allowance([0.0, 0.02], [0, 0], [-0.3, 0.3])["speed_up"] == 1
    assert allowance([0.0, 0.02], [0, 0], [-0.05, 0.05])["speed_up"] == 0


def test_non_finite_commands_and_short_series_are_skipped_not_counted() -> None:
    assert allowance([0.0], [0.0], [0.0]) == {
        "pairs": 0, "steering": 0, "speed_up": 0, "violation_pairs": 0, "fraction": 0.45,
    }  # fmt: skip
    r = allowance([0.0, 0.02, 0.04], [0.0, math.nan, 0.0], [0.5, math.nan, 0.5])
    assert r["violation_pairs"] == 0 and r["pairs"] == 2


def test_the_count_is_in_the_planner_metrics_and_the_report() -> None:
    from pnp_replay import report

    n = 50
    now = np.arange(n) * 0.02
    steer = np.where(np.arange(n) % 2 == 0, 0.0, 0.1)  # +-0.1 rad every tick: a violation each
    speed = np.full(n, 0.5)
    zeros = np.zeros(n)
    m = metrics.planner_metrics(
        now, np.ones(n, int), speed, steer, zeros, zeros, np.full(n, 0.1),
        np.arange(0.0, 2.0, 0.01), rate_limits=(STEER_RATE, ACCEL, TICK),
    )  # fmt: skip
    assert m["rate_allowance"]["violation_pairs"] == n - 1
    flat = report.flatten({"planner": m})
    assert flat["planner.rate_allowance.violation_pairs"] == n - 1
    assert "rate allowance violation pairs" in report.format_table(
        {"segments": {"R": {"planner": m, "t_from": 0.0, "t_to": 1.0, "duration_s": 1.0}}}
    )
    plain = metrics.planner_metrics(
        now, np.ones(n, int), speed, steer, zeros, zeros, np.full(n, 0.1), np.arange(0.0, 2.0, 0.01)
    )
    assert "rate_allowance" not in plain  # not asked for


def test_vehicle_rate_limits_come_from_the_generated_bindings() -> None:
    v = vehicle.load_vehicle_limits()
    assert v.schema_version == "0.17.0"
    assert v.steering_max_rate_rad_per_s > 0 and v.max_acceleration_mps2 > 0
    assert v.steering_min_rad < 0 < v.steering_max_rad


def test_recorded_tick_times_replace_the_uniform_grid() -> None:
    rec = {
        "scan_t": np.array([10.1]), "scan_stamp": np.array([10.0]),
        "scan_angle_min": np.array([-3.0]), "scan_angle_increment": np.array([1.0]),
        "scan_range_min": np.array([0.05]), "scan_range_max": np.array([12.0]),
        "scan_q": np.full((1, 4), 4000, np.uint16),
        "odom_t": np.array([10.0]), "odom_v": np.array([0.5]),
    }  # fmt: skip
    ticks = [10.12, 10.121, 10.15, 10.30, 99.0]  # bunched, a gap, and one outside the segment
    lines = planner_stream.stream_lines(rec, 10.1, 10.4, tracks_mode="none", tick_times=ticks)
    ks = [float(ln.split()[1]) for ln in lines if ln.startswith("K ")]
    assert ks == [10.12, 10.121, 10.15, 10.30]
    uniform = planner_stream.stream_lines(rec, 10.1, 10.4, tracks_mode="none")
    assert len([ln for ln in uniform if ln.startswith("K ")]) == 16


def _script():
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "pnp_replay_script", REPO_ROOT / "tools" / "pnp_replay.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_window_and_per_segment_cap_options_parse_and_refuse_nonsense() -> None:
    script = _script()
    w = script.parse_windows(["P0=16.6:113.9"])
    assert w == [segments.Segment("P0", "window", 16.6, 113.9)]
    for bad in ("P0", "P0=1", "=1:2", "P0=5:5"):
        with pytest.raises(ValueError, match="--window"):
            script.parse_windows([bad])
    assert script.parse_v_max_segments(["R2=1.0", "R5=0.7"]) == {"R2": 1.0, "R5": 0.7}
    with pytest.raises(ValueError, match="--v-max-segment"):
        script.parse_v_max_segments(["R2"])


def _cli_or_skip() -> planner_stream.Cli:
    try:
        return planner_stream.find_cli()
    except planner_stream.ReplayError as error:
        if os.environ.get("PNP_REPLAY_REQUIRE_CLI"):
            pytest.fail(f"planner_replay_cli is required here but not built: {error}")
        pytest.skip(f"planner_replay_cli not built ({error})")


def test_the_command_runs_a_window_on_the_recorded_ticks_with_its_own_cap(
    tmp_path: Path,
) -> None:
    _cli_or_skip()
    from pnp_replay import bag

    rec, a, b = cut_fixture.load_cut(FAULT_FIXTURE, "R7")
    path = tmp_path / "r7.npz"
    bag.save_recording(rec, path)
    out = tmp_path / "r7.json"
    code = _script().main(
        [str(path), "--window", f"X={a + 1.0}:{b}", "--segment", "X", "--tracks", "recorded",
         "--dynamics-bundle", str(CAR_BUNDLE), "--v-max", "0.4", "--v-max-segment", "X=1.0",
         "--ticks", "recorded", "--set", "solve_halve_over_ms=200",
         "--set", "stall_hold_s=600", "--out", str(out)]
    )  # fmt: skip
    assert code == 0
    seg = json.loads(out.read_text())["segments"]["X"]
    assert seg["v_max_mps"] == 1.0 and seg["ticks"] == "recorded"
    m = seg["planner"]
    assert m["rate_allowance"]["pairs"] == m["ticks"] - 1
    status = (rec["st_t"] >= a + 1.0) & (rec["st_t"] <= b)
    assert m["ticks"] == status.sum()  # one planner tick per recorded status message
    assert m["state_counts"]["DRIVING"] > 100


# --- the committed baseline ------------------------------------------------------------------


def test_the_2026_11_13_baseline_has_the_allowance_count_for_every_run_and_p0() -> None:
    data = json.loads(BASELINE.read_text())
    assert data["bag"].startswith("2026-11-13T22-20-18_car_teleop")
    assert data["dynamics_bundle"] == "dyn-car-2026-11-12"
    assert list(data["segments"]) == [f"R{i}" for i in range(9)] + ["P0"]
    caps = {"R0": 0.5, "R1": 0.5, "R2": 1.0, "R3": 0.5, "R4": 0.5, "R5": 0.7, "R6": 0.7,
            "R7": 1.0, "R8": 1.0, "P0": 0.55}  # fmt: skip
    for name, seg in data["segments"].items():
        assert seg["v_max_mps"] == caps[name], name
        allow = seg["planner"]["rate_allowance"]
        assert allow["pairs"] == seg["planner"]["ticks"] - 1
        either = allow["violation_pairs"]
        assert max(allow["steering"], allow["speed_up"]) <= either
        assert either <= allow["steering"] + allow["speed_up"]
        if name.startswith("R"):
            assert seg["ticks"] == "recorded"
            # bunched ticks of the car: the planner on main breaks the allowance in every run
            assert allow["violation_pairs"] > 0, name
            assert seg["fidelity"]["ok"] and seg["fidelity"]["max_centreline_diff_m"] == 0.0
        else:
            assert (
                seg["ticks"] == "uniform"
            )  # P0 is gap follow: no planner status to take ticks from
            assert allow["violation_pairs"] == 0
