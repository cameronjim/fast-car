"""Proves the comparison harness actually catches a dynamics regression (roadmap S.6 item
3: "the comparison harness catches an injected dynamics change ... this proves the canary
actually bites").

Perturbs ``actuation.max_acceleration_mps2`` in an in-memory copy of the loaded
vehicle_params object (never touches config/vehicle_params.yaml on disk -- CLAUDE.md
invariant 2 forbids hand-writing a physical constant in code, but mutating an already-loaded
object inside a test, to prove a detector works, is not that) and asserts the battery FAILS
against the committed references. ``max_acceleration_mps2`` directly gates the
acceleration-limiting branch (``accl_constraints``, sim/racer_gym/racer_gym/dynamics/
model.py) both ``throttle_step`` and ``coastdown`` command through, so doubling it is a
strong, obviously-real dynamics change with a clear expected effect (faster
accel/braking) -- not a contrived edge case.
"""

from __future__ import annotations

import dataclasses

import racer_gym
from racer_sim_regression.battery import compare_battery_to_references


def _perturbed_vehicle_params():
    vehicle_params = racer_gym.load_vehicle_params()
    doubled = vehicle_params.actuation.max_acceleration_mps2 * 2.0
    return dataclasses.replace(
        vehicle_params,
        actuation=dataclasses.replace(vehicle_params.actuation, max_acceleration_mps2=doubled),
    )


def _failing_names(results) -> set[str]:
    return {
        name for name, per_kind in results.items() for result in per_kind.values() if not result.ok
    }


class TestCanaryBites:
    def test_unperturbed_battery_matches_committed_references(self):
        # Sanity check first: the canary below is only meaningful if the CLEAN battery
        # passes -- otherwise "the perturbed run also fails" would prove nothing.
        results = compare_battery_to_references()
        failing = _failing_names(results)
        assert not failing, f"battery should match committed references cleanly; failing: {failing}"

    def test_perturbed_acceleration_limit_fails_against_committed_references(self):
        results = compare_battery_to_references(vehicle_params=_perturbed_vehicle_params())
        failing = _failing_names(results)
        assert failing, (
            "expected the injected actuation.max_acceleration_mps2 perturbation to fail at "
            "least one maneuver's golden comparison -- if this passes, the regression "
            "battery cannot actually catch a real dynamics change, which is S.6's entire point."
        )
        # throttle_step and coastdown both command straight-line accel/braking steps that
        # accl_constraints gates directly on max_acceleration_mps2 -- at least one of them
        # must be among the failures for this to be the expected mechanism, not a
        # coincidental failure elsewhere (e.g. a flaky unrelated field).
        assert failing & {"throttle_step", "coastdown"}, (
            f"expected throttle_step and/or coastdown to fail (they command straight-line "
            f"accel/braking steps gated by max_acceleration_mps2); failures were: {failing}"
        )
