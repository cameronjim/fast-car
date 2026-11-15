"""L1 for N4 (refit_v04.py) on synthetic plants. Synthetic data only: no bag, no held-out."""

from __future__ import annotations

import numpy as np
import pytest
from racer_pnp_dynamics import fit, refit_v04
from racer_pnp_dynamics.dataset import by_split, make_segments
from racer_pnp_dynamics.evaluate import eval_set
from racer_pnp_dynamics.icp_motion import MotionSeries
from synth import make_run, make_split
from test_fit import TRUTH

SEGS = [
    ("T1", 5.0, 330.0, "train", "ccw"),
    ("T2", 335.0, 660.0, "train", "cw"),
]
PLANT = TRUTH.replace(
    steer_gain_left=1.15, steer_gain_right=0.95, steer_offset_rad=0.01, understeer_s2_per_m2=0.4
)
YAW_NAMES = ("steer_gain_left", "steer_gain_right", "steer_offset_rad")


def _data(vehicle, ground_noise_m=0.0, seed=11):
    run, motion = make_run(
        PLANT, vehicle, seconds=670.0, seed=seed, name="bag", v_hi=1.0, exact_model_dt=True,
        yaw_noise=0.003, speed_noise=0.01,
    )  # fmt: skip
    if ground_noise_m:
        rng = np.random.default_rng(seed + 1)
        n = len(motion.t_s)
        motion = MotionSeries(
            motion.t_s, motion.dt_s,
            motion.dx_m + rng.normal(0.0, ground_noise_m, n),
            motion.dy_m + rng.normal(0.0, ground_noise_m, n),
            motion.dyaw_rad, motion.rms_m, motion.inlier_share, motion.ok,
        )  # fmt: skip
    segs = make_segments(run, motion, make_split("bag", SEGS), t_ref_s=run.t_s[0])
    return by_split(segs, "train", odom_only=True) + by_split(segs, "val", odom_only=True)


@pytest.fixture(scope="module")
def clean(toy_vehicle_module):
    return _data(toy_vehicle_module)


@pytest.fixture(scope="module")
def noisy(toy_vehicle_module):
    return _data(toy_vehicle_module, ground_noise_m=0.06)


def _start():
    return PLANT.replace(steer_gain_left=1.0, steer_gain_right=1.0, steer_offset_rad=0.0)


def test_rollout_fit_recovers_the_yaw_parameters(clean, toy_vehicle_module):
    es = eval_set(by_split(clean, "train"))
    got, flags = refit_v04.fit_yaw_rollout(es, toy_vehicle_module, _start())
    for name in YAW_NAMES:
        assert getattr(got, name) == pytest.approx(getattr(PLANT, name), abs=0.05), name
    assert got.understeer_s2_per_m2 == pytest.approx(PLANT.understeer_s2_per_m2, abs=0.25)
    assert not any("bound" in v for v in flags.values())
    # nothing but the yaw parameters moved
    assert refit_v04.fit.__name__ and all(
        getattr(got, n) == getattr(_start(), n) for n in fit.SPEED_PARAMS
    )


def test_ground_speed_noise_biases_the_one_step_fit_and_not_the_rollout_fit(
    noisy, toy_vehicle_module
):
    """The v0.3 failure as a test: the one-step objective reads the ICP ground speed, so noise in
    it biases the steering gains; the rollout objective reads the wheel speed."""
    train = by_split(noisy, "train")
    old, _ = fit.fit_yaw(train, toy_vehicle_module, _start(), None, tied=False)
    new, _ = refit_v04.fit_yaw_rollout(eval_set(train), toy_vehicle_module, _start())

    def err(p):
        return abs(p.understeer_s2_per_m2 - PLANT.understeer_s2_per_m2)

    # ICP noise on the ground speed (6 cm per 0.1 s interval, a large but real order for a bad
    # match) biases the one-step understeer by more than the whole value of the parameter
    assert err(old) > 0.4, err(old)
    assert err(new) < 0.1, err(new)
    assert abs(new.steer_gain_left - PLANT.steer_gain_left) < 0.05
    assert abs(new.steer_gain_right - PLANT.steer_gain_right) < 0.05


def test_the_objective_is_zero_at_the_plant_up_to_noise_and_larger_elsewhere(
    clean, toy_vehicle_module
):
    es = eval_set(by_split(clean, "train"))
    obj = refit_v04.make_objective(es, _start(), toy_vehicle_module, PLANT.command_delay_s)
    assert obj.cost(PLANT) < 0.2 * obj.cost(_start())
    assert obj.weights.mean() == pytest.approx(1.0)
    assert obj.residuals(PLANT).shape == (len(es) * 6,)


def test_strata_weigh_equally(clean, toy_vehicle_module):
    es = eval_set(by_split(clean, "train"))
    obj = refit_v04.make_objective(es, _start(), toy_vehicle_module, PLANT.command_delay_s)
    from racer_pnp_dynamics.strata import assign_strata

    strata = assign_strata(es.direction, es.start_speed)
    totals = [obj.weights[strata.label == n].sum() for n in strata.names]
    assert np.allclose(totals, totals[0])


def test_no_windows_is_refused(toy_vehicle):
    from racer_pnp_dynamics.evaluate import _empty_set

    with pytest.raises(ValueError, match="no windows"):
        refit_v04.make_objective(_empty_set(40, 10), TRUTH, toy_vehicle, 0.1)


def test_fit_n4_reports_every_parameter_and_changes_nothing_but_the_yaw(clean, toy_vehicle_module):
    # the speed fit is given (it is the existing fit, tested elsewhere), so this stays fast
    result = refit_v04.fit_n4(
        clean, toy_vehicle_module, _start(), speed_nominal=PLANT, speed_flags={}
    )
    rows = {r["parameter"]: r for r in result.parameter_table()}
    assert set(rows) == set(PLANT.to_dict())
    assert rows["steer_gain_left"]["free_in_yaw_fit"] and not rows["speed_gain"]["free_in_yaw_fit"]
    for name in ("speed_gain", "speed_tau_s", "command_delay_s", "speed_deadband_mps"):
        assert rows[name]["n4"] == getattr(PLANT, name)  # carried from the speed fit
    assert set(result.profiles) == set(refit_v04.YAW_FREE)
    assert all(len(v) == refit_v04.PROFILE_POINTS for v in result.profiles.values())
    assert result.cost["n4"] < result.cost["dyn-v0.1"]
    assert result.on_bound == [] and result.counts["windows_train"] > 100
