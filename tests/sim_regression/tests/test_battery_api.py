"""Unit coverage for racer_sim_regression.battery's small API surface that
test_battery_determinism.py / test_battery_regression.py / test_canary_bites.py don't
already exercise incidentally: the unknown-maneuver guard and
``assert_battery_matches_references``'s pass/fail behavior."""

from __future__ import annotations

import dataclasses

import pytest
import racer_gym
from racer_sim_regression.battery import assert_battery_matches_references, run_maneuver


class TestRunManeuverGuard:
    def test_unknown_maneuver_raises_key_error(self):
        with pytest.raises(KeyError, match="unknown maneuver"):
            run_maneuver("not_a_real_maneuver")


class TestAssertBatteryMatchesReferences:
    def test_passes_silently_against_committed_references(self):
        assert assert_battery_matches_references() is None

    def test_raises_with_readable_report_on_mismatch(self):
        vehicle_params = racer_gym.load_vehicle_params()
        perturbed = dataclasses.replace(
            vehicle_params,
            actuation=dataclasses.replace(
                vehicle_params.actuation,
                max_acceleration_mps2=vehicle_params.actuation.max_acceleration_mps2 * 2.0,
            ),
        )
        with pytest.raises(AssertionError, match="sim dynamics regression battery FAILED"):
            assert_battery_matches_references(vehicle_params=perturbed)
