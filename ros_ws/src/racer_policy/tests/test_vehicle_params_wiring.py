"""Integration test: `verify_against_environment`'s vehicle_params check against the REAL
generated binding of the committed config/vehicle_params.yaml (regenerated at test time via
tools/gen_params.py, never committed -- see conftest.py's `real_vehicle_params` fixture).
This is the proof that `racer_policy.environment.VehicleParamsLike` is actually satisfied by
what gen_params.py emits for `.meta`, not just by the hand-written `_VehicleParams` stub in
conftest.py's `live_environment_matching`. Mirrors
training/envelope/tests/test_from_vehicle_params_wiring.py.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest
from conftest import live_environment_matching, write_contract_dir
from racer_policy.contract import load_contract
from racer_policy.environment import LiveEnvironment
from racer_policy.errors import VehicleParamsVersionMismatchError
from racer_policy.verify import verify_against_environment


def test_accepts_when_contract_matches_the_real_committed_params(
    tmp_path: Path, valid_manifest: dict[str, Any], real_vehicle_params: Any
) -> None:
    # config/vehicle_params.yaml currently has meta.schema_version = "0.11.1" (0.1.0 ->
    # 0.2.0 for milestone 4, roadmap task 1.3; 0.2.0 -> 0.2.1 when that file's safety_mux
    # PWM/watchdog fields were given PROVISIONAL values; 0.2.1 -> 0.2.2 when
    # actuation.throttle_full_scale_mps was added and the TTC thresholds were filled in,
    # both PROVISIONAL; 0.2.2 -> 0.3.0 on 2026-09-21 when the steering PWM endpoints and
    # sign were measured; 0.3.0 -> 0.4.0 the same day when the Jetson PWM frame period became
    # two required actuation fields, GitHub issue #66; 0.4.0 -> 0.4.1 on 2026-09-27 when the
    # drivetrain motor constants and tire radius were filled in from the vendor Hobbywing G3
    # spec sheet, no field added/removed; 0.4.1 -> 0.5.0 on 2026-09-29 when the measured
    # achieved PWM frame periods, GitHub issue #77, and throttle_deadband_us were added as
    # required actuation fields, see that file's header; 0.5.0 -> 0.5.1 the same day when
    # throttle_deadband_us went 50 -> 20, value-only; 0.5.1 -> 0.6.0 on 2026-10-05 when the
    # required sensors.lidar_spec section was added for the RPLIDAR C1, roadmap 2.3; 0.6.0 ->
    # 0.6.1 on 2026-10-06 when the lidar mount was measured; 0.6.1 -> 0.7.0 the same day when
    # limits.min_forward_clearance_m and limits.ttc_forward_sector_half_angle_rad were added;
    # 0.7.2 -> 0.8.0 the same night when limits.obstacle_steering_hold_after_s was added;
    # 0.8.0 -> 0.9.0 after the first floor test when limits.obstacle_corridor_margin_m was
    # added; 0.9.0 -> 0.9.1 through 0.9.4 later that night, all value-only; 0.9.4 -> 0.10.0
    # when the required chassis.rear_overhang_m was added for the rear corridor; 0.10.0 ->
    # 0.11.0 when the required chassis.front_overhang_m was added for the forward arc
    # corridor's outer front corner sweep) and
    # meta.sysid_session_id = "none-preliminary" -- the same values `_TEMPLATE` in conftest.py records under
    # `vehicle_params`, by construction.
    assert (
        valid_manifest["vehicle_params"]["schema_version"]
        == real_vehicle_params.meta.schema_version
    )
    assert (
        valid_manifest["vehicle_params"]["sysid_session_id"]
        == real_vehicle_params.meta.sysid_session_id
    )

    live = live_environment_matching(valid_manifest)
    live = LiveEnvironment(vehicle_params=real_vehicle_params, observation=live.observation)
    directory = write_contract_dir(tmp_path, valid_manifest)
    contract = load_contract(directory)

    assert verify_against_environment(contract, live) is None


def test_refuses_when_contract_predates_the_real_committed_params(
    tmp_path: Path, valid_manifest: dict[str, Any], real_vehicle_params: Any
) -> None:
    valid_manifest["vehicle_params"]["sysid_session_id"] = "some-earlier-session"
    live = live_environment_matching(valid_manifest)
    live = LiveEnvironment(vehicle_params=real_vehicle_params, observation=live.observation)
    directory = write_contract_dir(tmp_path, valid_manifest)
    contract = load_contract(directory)

    with pytest.raises(VehicleParamsVersionMismatchError, match="sysid_session_id"):
        verify_against_environment(contract, live)
