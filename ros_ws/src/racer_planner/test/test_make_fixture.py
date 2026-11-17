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
    # accel/decel limits far above the step so only the lag acts; expected values are computed
    # with the same expression the step uses (a_v * (target - v)), not a rearranged one
    nom = dict(
        mf.NOMINAL,
        speed_deadband_mps=0.2,
        speed_tau_s=0.05,
        accel_max_mps2=100.0,
        decel_max_mps2=100.0,
    )
    a_v = 1.0 - math.exp(-0.05 / nom["speed_tau_s"])
    below = mf.step((0, 0, 0, 0.0, 0), (0.19, 0.0), nom, VEHICLE, 0.05)
    at = mf.step((0, 0, 0, 0.0, 0), (0.2, 0.0), nom, VEHICLE, 0.05)
    assert below[3] == 0.0
    assert at[3] == max(0.0, 0.0 + a_v * (nom["speed_gain"] * 0.2 - 0.0))
    rolling = mf.step((0, 0, 0, 0.4, 0), (0.1, 0.0), nom, VEHICLE, 0.05)
    assert rolling[3] == max(0.0, 0.4 + a_v * (0.0 - 0.4))


# ---- step contract 2 (brief 43) ---------------------------------------------------------------

import numpy as np
import yaml

FIXTURES = HERE.parents[3] / "config" / "pnp" / "fixtures"


def _ground_from_manifest(path: Path):
    """The table, nominal and learned block of a contract 2 manifest, as make_fixture takes them."""
    m = yaml.safe_load((path / "manifest.yaml").read_text(encoding="utf-8"))
    g = m["io"]["ground_motion"]
    learned = None
    if g["learned"]["enabled"]:
        weights = np.frombuffer((path / "ground_mlp.f32").read_bytes(), dtype="<f4")
        learned = (
            weights.astype(np.float64),
            np.array(g["learned"]["input_mean"]),
            np.array(g["learned"]["input_std"]),
        )
    return m["io"]["nominal"], (g["table"]["values"], learned), m["io"]["dt_s"]


def _golden_rollouts(path: Path):
    rows = [r.split(",") for r in (path / "golden_rollouts.csv").read_text().splitlines()[1:]]
    out: dict[int, list[list[float]]] = {}
    for r in rows:
        out.setdefault(int(r[0]), []).append([float(x) for x in r[2:]])
    return out


def test_make_fixture_step_v2_reproduces_the_real_exported_goldens():
    """The independent contract 2 step here equals the Python reference that wrote the goldens of
    the exported candidates (PR 193), so the committed random-state file is a valid second
    implementation, not a copy of the C++."""
    for name in ("dyn-v0.4t-contract2-test-only", "dyn-v0.4tl-contract2-test-only"):
        path = FIXTURES / name
        nominal, ground, dt = _ground_from_manifest(path)
        nominal = {k: float(v) for k, v in nominal.items()}
        worst = 0.0
        for rows in _golden_rollouts(path).values():
            state = tuple(rows[0][2:7])
            for i in range(len(rows) - 1):
                u = (rows[i][0], rows[i][1])
                state = mf.step_v2(state, u, nominal, VEHICLE, dt, ground)
                worst = max(worst, max(abs(a - b) for a, b in zip(state, rows[i + 1][2:7])))
        assert worst < 1e-4, (name, worst)


def test_step_v2_zero_wheel_speed_is_zero_motion_and_identity_table_is_contract_1():
    table = [[1.0] * 5 for _ in range(7)]
    s = mf.step_v2((0.1, 0.2, 0.3, 0.0, 0.2), (0.5, 0.2), mf.NOMINAL, VEHICLE, 0.05, (table, None))
    assert s[:3] == (0.1, 0.2, 0.3)
    for st in [(0, 0, 0.4, 0.6, 0.15), (1, 1, -2.0, 1.1, -0.3)]:
        a = mf.step_v2(st, (0.7, 0.1), mf.NOMINAL_ASYMMETRIC, VEHICLE, 0.05, (table, None))
        b = mf.step(st, (0.7, 0.1), mf.NOMINAL_ASYMMETRIC, VEHICLE, 0.05)
        assert all(math.isclose(x, y, abs_tol=1e-15) for x, y in zip(a, b))


def test_contract_2_bundle_is_deterministic_and_committed_fixture_is_current(tmp_path):
    out = tmp_path / "v2"
    mf.make_bundle_v2(
        out, VEHICLE, learned_on=True, nominal=mf.NOMINAL_ASYMMETRIC, v_max=1.3, seed=1
    )
    committed = HERE / "fixtures" / "dyn_fixture_v2"
    for name in ("manifest.yaml", "golden_rollouts.csv", "ground_mlp.f32"):
        assert (out / name).read_bytes() == (committed / name).read_bytes(), name
    manifest = yaml.safe_load((out / "manifest.yaml").read_text())
    assert manifest["bundle_schema_version"] == "2.0.0"
    assert "residual" not in manifest["io"]
    rows = (committed / "step_v2_random.csv").read_text().splitlines()
    assert rows[0] == mf.RANDOM_HEADER and len(rows) == 10001
