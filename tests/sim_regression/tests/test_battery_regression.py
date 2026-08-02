"""The actual S.6 regression gate: the committed battery vs. references/ (roadmap S.6,
claude-docs/12-testing.md L5 "Model-upgrade regression": "a fixed battery of maneuvers ...
produces outputs within tolerance of committed references, or the change is intentional and
the references are updated with a stated reason").

Runs on every push touching sim/racer_gym/** via the sim-regression-battery CI job (see
.github/workflows/ci.yml). Split trajectory vs. summary, and parametrized per maneuver, so a
failure names exactly which maneuver/kind regressed rather than a single opaque assertion.
"""

from __future__ import annotations

import pytest
from racer_sim_regression.battery import MANEUVERS, compare_battery_to_references


@pytest.fixture(scope="module")
def battery_results():
    return compare_battery_to_references()


class TestBatteryMatchesCommittedReferences:
    @pytest.mark.parametrize("name", sorted(MANEUVERS))
    def test_trajectory_matches_reference(self, battery_results, name):
        result = battery_results[name]["trajectory"]
        assert result.ok, result.format_report()

    @pytest.mark.parametrize("name", sorted(MANEUVERS))
    def test_summary_matches_reference(self, battery_results, name):
        result = battery_results[name]["summary"]
        assert result.ok, result.format_report()
