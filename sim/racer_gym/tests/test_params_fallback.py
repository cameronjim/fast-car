"""L1 tests for racer_gym.params: the null-field fallback policy documented in that module's
docstring (claude-docs/00-project-overview.md's honesty rules; claude-docs/06-vehicle-
params.md invariant 2 -- no hand-written physical constants).

These tests regenerate the real config/vehicle_params.yaml binding (as of task 0.7, every
Pacejka/time-constant/delay/track-width field is null pending Phase 1-3 work) plus a
synthetic fully-fitted fixture, to exercise both branches of every fallback.
"""

from __future__ import annotations

import copy
import math
from pathlib import Path

import pytest
import yaml
from racer_gym.params import (
    DEFAULT_PARAMS_PATH,
    DEFAULT_SCHEMA_PATH,
    build_dyn_params,
    load_vehicle_params,
)

DT_S = 0.01


@pytest.fixture(scope="module")
def repo_vehicle_params():
    return load_vehicle_params()


def test_repo_params_file_is_currently_all_placeholders(repo_vehicle_params):
    """Sanity check on the fixture assumption above: fails loudly (telling the implementer to
    also add a "some fields now fitted" test) the moment task 0.7's provisional file is
    superseded by a real sysid fit."""
    result = build_dyn_params(repo_vehicle_params, DT_S)
    assert result.used_any_placeholder
    assert result.fallback_flags == {
        "pacejka_front": True,
        "pacejka_rear": True,
        "steer_time_constant": True,
        "throttle_time_constant": True,
        "command_to_torque_delay": True,
        "track_width": True,
    }


def test_placeholder_pacejka_peak_matches_mu_times_static_axle_load(repo_vehicle_params):
    result = build_dyn_params(repo_vehicle_params, DT_S)
    mu = repo_vehicle_params.tires.surface_friction_coefficient
    from racer_gym.dynamics.load_transfer import static_axle_loads

    static = static_axle_loads(
        repo_vehicle_params.chassis.mass_kg,
        repo_vehicle_params.chassis.cg_to_front_axle_m,
        repo_vehicle_params.chassis.cg_to_rear_axle_m,
    )
    assert math.isclose(result.dyn_params.pacejka_front.d_peak_n, mu * static.front_n, rel_tol=1e-9)
    assert math.isclose(result.dyn_params.pacejka_rear.d_peak_n, mu * static.rear_n, rel_tol=1e-9)


def test_placeholder_pacejka_small_slip_slope_matches_linear_cornering_stiffness(
    repo_vehicle_params,
):
    """B*C*D (Magic Formula's E=0 small-slip cornering stiffness identity) must reproduce
    tires.linear_cornering_stiffness_front/rear_n_per_rad exactly -- the whole point of the
    placeholder is to agree with stock f1tenth_gym's linear tire model at small slip."""
    result = build_dyn_params(repo_vehicle_params, DT_S)
    front = result.dyn_params.pacejka_front
    rear = result.dyn_params.pacejka_rear
    assert math.isclose(
        front.b_stiffness * front.c_shape * front.d_peak_n,
        repo_vehicle_params.tires.linear_cornering_stiffness_front_n_per_rad,
        rel_tol=1e-9,
    )
    assert math.isclose(
        rear.b_stiffness * rear.c_shape * rear.d_peak_n,
        repo_vehicle_params.tires.linear_cornering_stiffness_rear_n_per_rad,
        rel_tol=1e-9,
    )


def test_placeholder_actuator_time_constants_are_zero_ie_instantaneous(repo_vehicle_params):
    result = build_dyn_params(repo_vehicle_params, DT_S)
    assert result.dyn_params.steer_tau_s == 0.0
    assert result.dyn_params.throttle_tau_s == 0.0


def test_placeholder_delay_matches_f1tenth_gym_steer_buffer_size(repo_vehicle_params):
    from f1tenth_gym.envs.base_classes import RaceCar

    result = build_dyn_params(repo_vehicle_params, DT_S)
    # RaceCar.steer_buffer_size is set in __init__, not a class attribute -- construct
    # nothing, just confirm the fallback used the value racer_gym.params extracted from it
    # (see racer_gym/params.py::_f1tenth_gym_steer_buffer_size for why it's read this way).
    import inspect
    import re

    source = inspect.getsource(RaceCar.__init__)
    expected = int(re.search(r"self\.steer_buffer_size\s*=\s*(\d+)", source).group(1))
    assert result.dyn_params.delay_steps == expected


def test_placeholder_track_width_gives_none(repo_vehicle_params):
    result = build_dyn_params(repo_vehicle_params, DT_S)
    assert result.dyn_params.track_width_m is None


# --------------------------------------------------------------------------------------
# Fully-fitted fixture: every null field replaced -- fallback flags must all be False and
# the fitted values must be used verbatim (not silently re-derived).
# --------------------------------------------------------------------------------------


@pytest.fixture()
def fitted_vehicle_params(repo_vehicle_params, tmp_path: Path):
    raw = yaml.safe_load(DEFAULT_PARAMS_PATH.read_text())
    fitted = copy.deepcopy(raw)
    fitted["tires"]["pacejka_front"] = {
        "b_stiffness": 9.0,
        "c_shape": 1.4,
        "d_peak_n": 21.0,
        "e_curvature": 0.1,
    }
    fitted["tires"]["pacejka_rear"] = {
        "b_stiffness": 11.0,
        "c_shape": 1.4,
        "d_peak_n": 19.0,
        "e_curvature": 0.1,
    }
    fitted["chassis"]["track_width_m"] = 0.23
    fitted["steering"]["time_constant_s"] = 0.05
    fitted["actuation"]["throttle_time_constant_s"] = 0.08
    fitted["actuation"]["command_to_torque_delay_s"] = 0.03
    fitted_path = tmp_path / "vehicle_params_fitted.yaml"
    fitted_path.write_text(yaml.safe_dump(fitted))
    return load_vehicle_params(params_path=fitted_path, schema_path=DEFAULT_SCHEMA_PATH)


def test_fitted_params_use_no_fallbacks(fitted_vehicle_params):
    result = build_dyn_params(fitted_vehicle_params, DT_S)
    assert not result.used_any_placeholder
    assert all(v is False for v in result.fallback_flags.values())


def test_fitted_pacejka_used_verbatim(fitted_vehicle_params):
    result = build_dyn_params(fitted_vehicle_params, DT_S)
    front = result.dyn_params.pacejka_front
    assert front.b_stiffness == 9.0
    assert front.c_shape == 1.4
    assert front.d_peak_n == 21.0
    assert front.e_curvature == 0.1


def test_fitted_time_constants_and_delay_used_verbatim(fitted_vehicle_params):
    result = build_dyn_params(fitted_vehicle_params, DT_S)
    assert result.dyn_params.steer_tau_s == 0.05
    assert result.dyn_params.throttle_tau_s == 0.08
    assert result.dyn_params.delay_steps == round(0.03 / DT_S)


def test_fitted_track_width_used_verbatim(fitted_vehicle_params):
    result = build_dyn_params(fitted_vehicle_params, DT_S)
    assert result.dyn_params.track_width_m == 0.23


def test_invalid_dt_raises(repo_vehicle_params):
    with pytest.raises(ValueError):
        build_dyn_params(repo_vehicle_params, 0.0)
    with pytest.raises(ValueError):
        build_dyn_params(repo_vehicle_params, -0.01)
