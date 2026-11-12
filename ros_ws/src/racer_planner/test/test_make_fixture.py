"""L1: the fixture generator (the second implementation of the model) behaves like the contract."""

from __future__ import annotations

import hashlib
import importlib.util
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location(
    "make_fixture", HERE / "fixtures" / "make_fixture.py"
)
mf = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mf)

VEHICLE = mf.vehicle_numbers(
    mf.read_vehicle_params(HERE.parents[3] / "config" / "vehicle_params.yaml")
)


def test_left_command_gives_positive_yaw_and_right_negative():
    state_l = state_r = (0.0, 0.0, 0.0, 0.5, 0.0)
    for _ in range(20):
        state_l = mf.step(state_l, (0.5, 0.2), mf.NOMINAL, VEHICLE, 0.05)
        state_r = mf.step(state_r, (0.5, -0.2), mf.NOMINAL, VEHICLE, 0.05)
    assert state_l[2] > 0.0 > state_r[2]
    assert math.isclose(state_l[2], -state_r[2], rel_tol=1e-9)


def test_clamps():
    lim = VEHICLE["steering_max_rad"]
    s = mf.step((0, 0, 0, 0, lim - 1e-4), (0.0, 10.0), mf.NOMINAL, VEHICLE, 0.05)
    assert s[4] <= lim + 1e-12
    s = mf.step((0, 0, 0, 0.0, 0), (5.0, 0.0), mf.NOMINAL, VEHICLE, 0.05)
    assert s[3] <= mf.NOMINAL["accel_max_mps2"] * 0.05 + 1e-12
    s = mf.step((0, 0, 0, 0.05, 0), (-5.0, 0.0), mf.NOMINAL, VEHICLE, 0.05)
    assert s[3] == 0.0


def test_bundle_is_deterministic_and_checksums_match(tmp_path):
    a, b = tmp_path / "a", tmp_path / "b"
    for out in (a, b):
        mf.make_bundle(
            out, VEHICLE, residual_on=True, nominal=mf.NOMINAL_ASYMMETRIC, v_max=0.9, seed=3
        )
    for name in ("manifest.yaml", "golden_rollouts.csv", "residual_mlp.f32"):
        assert (a / name).read_bytes() == (b / name).read_bytes()
    text = (a / "manifest.yaml").read_text(encoding="utf-8")
    for name in ("golden_rollouts.csv", "residual_mlp.f32"):
        assert hashlib.sha256((a / name).read_bytes()).hexdigest() in text


def test_golden_has_nine_rollouts_of_forty_steps(tmp_path):
    mf.make_bundle(tmp_path, VEHICLE, residual_on=False, nominal=mf.NOMINAL, v_max=0.9)
    rows = (tmp_path / "golden_rollouts.csv").read_text(encoding="utf-8").splitlines()
    assert rows[0] == "rollout,step,speed_cmd_mps,steering_cmd_rad,x_m,y_m,yaw_rad,v_mps,delta_rad"
    rollouts = {int(r.split(",")[0]) for r in rows[1:]}
    assert len(rollouts) == 9
    assert len(rows) - 1 == 9 * 41
    # the weights are all zero when the residual is off
    assert set((tmp_path / "residual_mlp.f32").read_bytes()) == {0}


def test_dead_zone_target_is_zero_below_and_gain_times_command_at_or_above():
    nom = dict(mf.NOMINAL, speed_deadband_mps=0.2, speed_tau_s=0.05)
    below = mf.step((0, 0, 0, 0.0, 0), (0.19, 0.0), nom, VEHICLE, 0.05)
    at = mf.step((0, 0, 0, 0.0, 0), (0.2, 0.0), nom, VEHICLE, 0.05)
    assert below[3] == 0.0
    assert math.isclose(at[3], (1.0 - math.exp(-1.0)) * 0.2, rel_tol=1e-12)
    rolling = mf.step((0, 0, 0, 0.4, 0), (0.1, 0.0), nom, VEHICLE, 0.05)
    assert math.isclose(rolling[3], 0.4 * math.exp(-1.0), rel_tol=1e-12)
