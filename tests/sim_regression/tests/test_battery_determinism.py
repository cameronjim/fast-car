"""L1: same seed, same battery -> identical output (claude-docs/12-testing.md; roadmap S.6
item 3: "the battery itself is deterministic (same seed, identical outputs)").

This is what makes the golden comparison meaningful at all: if the battery weren't
deterministic, no fixed per-field tolerance could ever separate a real dynamics regression
from ordinary run-to-run noise (see racer_sim_regression/tolerances.py's calibration note).
"""

from __future__ import annotations

from racer_sim_regression.battery import MANEUVERS, run_battery


class TestBatteryDeterminism:
    def test_same_seed_gives_identical_trajectory_and_summary_records(self):
        first = run_battery()
        second = run_battery()

        assert set(first) == set(MANEUVERS)
        assert set(first) == set(second)
        for name in first:
            traj_a, summary_a, _meta_a = first[name]
            traj_b, summary_b, _meta_b = second[name]
            assert traj_a == traj_b, f"{name} trajectory not deterministic across two runs"
            assert summary_a == summary_b, f"{name} summary not deterministic across two runs"

    def test_battery_output_is_non_trivial(self):
        # Guard against a vacuous determinism test (e.g. every maneuver silently returning
        # an empty list): every maneuver must produce at least one trajectory sample and one
        # summary record.
        results = run_battery()
        for name, (trajectory, summary, _meta) in results.items():
            assert len(trajectory) > 1, f"{name} trajectory has too few samples: {trajectory!r}"
            assert len(summary) >= 1, f"{name} summary is empty"
