"""L1 for M-B3 and M-B2 (budget.py): a synthetic plant whose only model error is known."""

from __future__ import annotations

import numpy as np
import pytest
from racer_pnp_dynamics import budget
from racer_pnp_dynamics.dataset import by_split, make_segments
from racer_pnp_dynamics.evaluate import eval_set
from synth import make_run, make_split
from test_fit import TRUTH

SEGS = [("T1", 5.0, 200.0, "train", "ccw"), ("T2", 205.0, 400.0, "train", "cw")]
QUIET = {"yaw_noise": 0.0005, "speed_noise": 0.0005}


def _windows(vehicle, truth, seed=3):
    run, motion = make_run(
        truth, vehicle, seconds=420.0, seed=seed, name="bag", v_hi=1.0, exact_model_dt=True, **QUIET
    )
    segs = make_segments(run, motion, make_split("bag", SEGS), t_ref_s=run.t_s[0])
    train = by_split(segs, "train", odom_only=True)
    return train, eval_set(train)


@pytest.fixture(scope="module")
def yaw_wrong(toy_vehicle_module):
    """The plant steers 15 percent more than the model: the model's only error is the yaw gain."""
    truth = TRUTH.replace(steer_gain_left=1.15, steer_gain_right=1.15, steer_offset_rad=0.0)
    model = truth.replace(steer_gain_left=1.0, steer_gain_right=1.0)
    segs, es = _windows(toy_vehicle_module, truth)
    return segs, es, model, budget.error_budget(es, model, toy_vehicle_module)


@pytest.fixture(scope="module")
def speed_wrong(toy_vehicle_module):
    """The plant is 20 percent faster than the model: the model's only error is the speed gain."""
    truth = TRUTH.replace(speed_gain=1.0, steer_offset_rad=0.0)
    model = truth.replace(speed_gain=0.8)
    segs, es = _windows(toy_vehicle_module, truth, seed=4)
    return segs, es, model, budget.error_budget(es, model, toy_vehicle_module)


def test_a_yaw_only_error_is_removed_by_the_perfect_yaw_oracle_and_not_by_perfect_speed(yaw_wrong):
    _, es, _, b = yaw_wrong
    t = b.table()["macro"]
    assert len(es) > 200
    assert t["ratio_perfect_yaw"] < 0.15
    assert t["ratio_perfect_speed"] > 0.9


def test_a_speed_only_error_is_removed_by_the_perfect_speed_oracle_and_not_by_perfect_yaw(
    speed_wrong,
):
    t = speed_wrong[3].table()["macro"]
    assert t["ratio_perfect_speed"] < 0.15
    assert t["ratio_perfect_yaw"] > 0.9


def test_along_and_across_add_in_quadrature_per_stratum_and_per_window(yaw_wrong):
    b = yaw_wrong[3]
    assert np.allclose(b.pos**2, b.along**2 + b.across**2)
    for name, row in b.table().items():
        if name != "macro":
            assert row["pos"] ** 2 == pytest.approx(row["along"] ** 2 + row["across"] ** 2)


def test_a_yaw_only_error_is_mostly_across(yaw_wrong):
    row = yaw_wrong[3].table()["macro"]
    assert row["across"] > 2.0 * row["along"]


def test_a_perfect_model_has_a_tiny_budget(toy_vehicle_module):
    truth = TRUTH.replace(steer_offset_rad=0.0)
    _, es = _windows(toy_vehicle_module, truth, seed=5)
    b = budget.error_budget(es, truth, toy_vehicle_module)
    assert b.table()["macro"]["pos"] < 0.02


def test_the_mb3_text_has_the_unreachable_sentence_only_above_the_threshold(yaw_wrong, speed_wrong):
    low = budget.render_mb3(yaw_wrong[3], "m", ["note"])
    assert budget.UNREACHABLE_SENTENCE not in low and "not above 0.95" in low
    assert "| macro |" in low and "- note" in low
    # a plant with a speed only error: perfect yaw buys nothing, so the sentence is written
    high = budget.render_mb3(speed_wrong[3], "m", [])
    assert yaw_wrong[3].perfect_yaw_ratio() < 0.95 < speed_wrong[3].perfect_yaw_ratio()
    assert budget.UNREACHABLE_SENTENCE in high


def test_empty_windows_are_refused(toy_vehicle):
    from racer_pnp_dynamics.evaluate import _empty_set

    with pytest.raises(ValueError, match="no windows"):
        budget.error_budget(_empty_set(40, 10), TRUTH, toy_vehicle)


def test_the_true_yaw_is_interpolated_between_scans(yaw_wrong):
    _, es, _, _ = yaw_wrong
    y = budget._true_yaw_steps(es, 20)
    assert y.shape == (len(es), 21)
    assert np.allclose(y[:, 0], es.poses[:, 0, 2]) and np.allclose(y[:, 20], es.poses[:, 10, 2])
    assert np.allclose(y[:, 1], 0.5 * (es.poses[:, 0, 2] + es.poses[:, 1, 2]))


def test_asymmetry_table_reports_limit_share_and_yaw_rate_bias(yaw_wrong, toy_vehicle_module):
    segs, es, model, _ = yaw_wrong
    rows = budget.asymmetry_table(segs, es, model, toy_vehicle_module)
    assert {r["direction"] for r in rows} == {"ccw", "cw"}
    assert all(0.0 <= r["steer_at_limit_share"] <= 1.0 for r in rows if r["samples_s"] > 0)
    rated = [r for r in rows if r["windows"] > 20]
    assert rated
    assert all(abs(r["yaw_rate_rms"]) > 0 for r in rated)
    text = budget.render_mb2(rows, [budget.lap_radius(s) for s in segs], "m", ["note"])
    assert "| ccw |" in text and "Lap radius" in text and "- note" in text


def test_lap_radius_of_a_circle(toy_vehicle_module):
    """Constant steering at a moving car: radius = wheelbase / tan(delta)."""
    truth = TRUTH.replace(
        steer_gain_left=1.0, steer_offset_rad=0.0, understeer_s2_per_m2=0.0, speed_tau_s=0.05
    )
    run, motion = make_run(
        truth, toy_vehicle_module, seconds=120.0, seed=6, name="bag", steer_const=0.25,
        v_hi=0.9, exact_model_dt=True, **QUIET,
    )  # fmt: skip
    segs = make_segments(
        run, motion, make_split("bag", [("C", 5.0, 110.0, "train", "ccw")]), t_ref_s=run.t_s[0]
    )
    rows = [budget.lap_radius(s) for s in by_split(segs, "train", odom_only=True)]
    assert rows
    expect = toy_vehicle_module.wheelbase_m / np.tan(0.25)
    assert rows[0]["radius_m"] == pytest.approx(expect, rel=0.05)


def test_lap_radius_without_odometry_is_nan(toy_vehicle_module):
    run, motion = make_run(TRUTH, toy_vehicle_module, seconds=60.0, odom=False, name="bag")
    segs = make_segments(
        run, motion, make_split("bag", [("C", 5.0, 50.0, "train", "ccw")]), t_ref_s=run.t_s[0]
    )
    assert segs == [] or np.isnan(budget.lap_radius(segs[0])["radius_m"])
