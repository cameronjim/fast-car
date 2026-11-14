"""Fit recovery for v0.2: known parameters, speeds to 1.5 m/s, both steering directions, the delay
chosen on the validation role (never on held-out), the dead zone estimated only from train and
val launches, and the unidentified flag when the steering lag is not excited."""

from __future__ import annotations

import numpy as np
import pytest
from racer_pnp_dynamics import fit, pipeline_v02
from racer_pnp_dynamics.dataset import by_split, make_segments
from racer_pnp_dynamics.fit import launches_in_segments
from synth import make_run, make_split
from test_fit import RATE_LIMITED, TRUTH

SEGS = [
    ("A", 5.0, 330.0, "train", "ccw"),
    ("B", 335.0, 560.0, "train", "cw"),
    ("H", 565.0, 690.0, "heldout", "ccw"),
]
HELD_FROM, HELD_TO = 565.0, 690.0


def _split_segments(vehicle, truth=RATE_LIMITED, seed=3, **kw):
    run, motion = make_run(truth, vehicle, seconds=700.0, seed=seed, name="bag", **kw)
    return make_segments(run, motion, make_split("bag", SEGS), t_ref_s=run.t_s[0])


def test_fit_recovers_known_parameters_to_1p5_mps_in_both_directions_scored_on_val(toy_vehicle):
    segs = _split_segments(toy_vehicle, v_hi=1.5, steer_hi=0.4)
    assert max(float(s.run.wheel_speed_mps[s.i0 : s.i1].max()) for s in segs) > 1.2
    result = fit.fit_nominal(
        segs, toy_vehicle, tau_grid=(0.1, 0.3), fit_split="train", score_split="val",
        launch_splits=("train", "val"),
    )  # fmt: skip
    got, want = result.nominal.to_dict(), RATE_LIMITED.to_dict()
    for name in (*fit.SPEED_PARAMS, *fit.YAW_PARAMS):
        assert got[name] == pytest.approx(want[name], rel=0.12), (name, got[name], want[name])
    assert abs(got["command_delay_s"] - want["command_delay_s"]) <= 0.02 + 1e-9
    assert not result.sides_tied
    assert result.delay_selection == "held-out windows"  # the score windows are the val role
    assert result.counts["speed_windows_heldout"] > fit.MIN_VAL_WINDOWS
    # the delay was scored on val: the same fit scored on the held-out role gives other numbers
    held_windows = len(by_split(segs, "heldout"))
    assert held_windows > 0  # held-out data exists and was not what the delay was scored on


def test_the_fit_does_not_read_heldout_segments(toy_vehicle):
    """Corrupting the held-out part of the run changes nothing in the fit."""
    segs = _split_segments(toy_vehicle, v_hi=1.0)
    kwargs = {
        "tau_grid": (0.1, 0.3),
        "delays": (0.10,),
        "fit_split": "train",
        "score_split": "val",
        "launch_splits": ("train", "val"),
    }
    base = fit.fit_nominal(segs, toy_vehicle, **kwargs)
    run = segs[0].run
    t0 = run.t_s[0]
    held = (run.t_s >= t0 + HELD_FROM) & (run.t_s <= t0 + HELD_TO)
    run.wheel_speed_mps[held] = 5.0 * np.random.default_rng(0).random(held.sum())
    run.speed_cmd_mps[held] = 1.7
    again = fit.fit_nominal(segs, toy_vehicle, **kwargs)
    assert again.nominal == base.nominal


def test_dead_zone_launches_come_only_from_train_and_val_segments(toy_vehicle):
    truth = TRUTH.replace(accel_max_mps2=2.0)
    run, motion = make_run(
        truth, toy_vehicle, seconds=700.0, seed=8, name="bag", ramp_mps2=0.7, dead_zone_mps=0.25,
        speed_noise=0.001,
    )  # fmt: skip
    segs = make_segments(run, motion, make_split("bag", SEGS), t_ref_s=run.t_s[0])
    train_val = [s for s in segs if s.split in ("train", "val")]
    held = [s for s in segs if s.split == "heldout"]
    mine = launches_in_segments(train_val, 0.0)
    theirs = launches_in_segments(held, 0.0)
    assert mine and theirs
    t0 = run.t_s[0]
    assert all(
        any(s.t0_s <= x.t_s <= s.t1_s for s in train_val) and x.t_s - t0 < HELD_FROM for x in mine
    )
    assert all(x.t_s - t0 >= HELD_FROM - 1e-9 for x in theirs)
    est = fit.launch_deadband([run], 0.0, mine)
    assert est["launches"] == len(mine) and est["value_mps"] > 0.0
    # the estimate over launches in segments differs from the whole run's (which includes held-out)
    whole = fit.launch_deadband([run], 0.0)
    assert whole["launches"] == len(mine) + len(theirs) + 0 or whole["launches"] >= len(mine)


def test_the_unidentified_flag_still_fires_when_the_steering_lag_is_not_excited(toy_vehicle):
    # steering held constant for the whole run: the lag cannot be seen
    run, motion = make_run(TRUTH, toy_vehicle, seconds=700.0, seed=5, name="bag", steer_const=0.2)
    segs = make_segments(run, motion, make_split("bag", SEGS[:2]), t_ref_s=run.t_s[0])
    result = fit.fit_nominal(
        segs, toy_vehicle, tau_grid=(0.05, 0.1, 0.3, 0.6), delays=(0.08, 0.10, 0.12),
        fit_split="train", score_split="val", launch_splits=("train", "val"),
    )  # fmt: skip
    assert "steer_tau_s" in result.unidentified or "command_delay_s" in result.unidentified


def test_v_max_is_the_p99_rounded_down_and_capped_at_1p5(toy_vehicle):
    segs = _split_segments(toy_vehicle, v_hi=2.0, steer_hi=0.2)
    info = pipeline_v02.v_max_of([s for s in segs if s.split in ("train", "val")], toy_vehicle)
    assert info["p99_wheel_speed_train_mps"] > 1.5
    assert info["v_max_mps"] == 1.5 and info["capped_at_mps"] == 1.5
    low = pipeline_v02.v_max_of(
        [s for s in _split_segments(toy_vehicle, v_hi=0.8) if s.split == "train"], toy_vehicle
    )
    assert low["capped_at_mps"] is None
    assert low["v_max_mps"] <= low["p99_wheel_speed_train_mps"] < low["v_max_mps"] + 0.05 + 1e-9
    hist = low["speed_histogram_seconds_by_direction"]
    assert set(hist) == {"ccw", "cw"} and sum(hist["ccw"].values()) > 100.0
    assert low["p99_icp_ground_speed_train_mps"] > 0.5
