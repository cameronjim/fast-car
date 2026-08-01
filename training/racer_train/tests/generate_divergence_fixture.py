"""Generates the committed cross-language base-controller divergence fixture (roadmap S.3).

Writes two committed files:

  - fixtures/divergence_states.csv: synthetic vehicle poses (x_m, y_m, yaw_rad), built
    deterministically (no RNG) from the committed config/tracks/gym_oval/raceline.csv.
  - fixtures/divergence_expected.json: THIS Python pure-pursuit port's
    (racer_train.raceline.PurePursuitController) commands for each pose in the states file.

ros_ws/src/racer_control/test/divergence/compare_divergence.py runs the SAME states through
the C++ pure_pursuit_cli binary (built from the real racer_control_core the on-vehicle
tracker_node uses) and asserts its output matches divergence_expected.json within a stated
tolerance. This is the S.3 analogue of claude-docs/12-testing.md's L5 "envelope-in-env test"
divergence pattern, applied to the base controller instead of the envelope: one
implementation's committed output is the oracle, the other is checked against it, rather
than trusting the two to independently agree.

Run this script to REGENERATE the fixture (e.g. after an intentional change to the
pure-pursuit algorithm in EITHER language) from training/racer_train/:

    uv run python tests/generate_divergence_fixture.py

Deterministic by construction: every state is a fixed arithmetic function of its index, so
re-running this script with no code changes reproduces byte-identical output.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from racer_gym.params import load_vehicle_params
from racer_train.raceline import PurePursuitConfig, PurePursuitController, Raceline

REPO_ROOT = Path(__file__).resolve().parents[3]
RACELINE_PATH = REPO_ROOT / "config" / "tracks" / "gym_oval" / "raceline.csv"
FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"
STATES_PATH = FIXTURES_DIR / "divergence_states.csv"
EXPECTED_PATH = FIXTURES_DIR / "divergence_expected.json"

# Matches tracker_node.cpp's declared ROS param defaults AND pure_pursuit_cli's own CLI flag
# defaults (ros_ws/src/racer_control/src/pure_pursuit_cli.cpp). These are tuning gains, not
# physical constants (CLAUDE.md invariant 2 does not apply to them) -- but if either of
# those two defaults ever changes without updating this constant, the divergence test will
# (correctly) start failing rather than silently comparing configs that no longer match.
LOOKAHEAD_MIN_M = 0.4
LOOKAHEAD_MAX_M = 1.5
LOOKAHEAD_CURVATURE_REF_1PM = 0.4


def build_states(raceline: Raceline) -> list[tuple[float, float, float]]:
    """Deterministic synthetic poses: walk the raceline at a fixed stride, offsetting each
    sample laterally (both sides, via sin) and perturbing its heading (via cos) so the
    fixture exercises off-raceline lateral recovery in both directions and non-aligned
    headings -- not just "sitting exactly on the line facing along it", which would miss the
    frame-rotation logic entirely (see ros_ws/src/racer_control/test/test_pure_pursuit.cpp's
    own hand-computed sign-convention cases for the same concern)."""
    states = []
    n = len(raceline)
    stride = max(n // 40, 1)
    for i in range(0, n, stride):
        point = raceline.at(i)
        lateral_offset_m = 0.3 * math.sin(0.7 * i)
        heading_perturbation_rad = 0.4 * math.cos(0.5 * i)
        x_m = point.x_m - lateral_offset_m * math.sin(point.heading_rad)
        y_m = point.y_m + lateral_offset_m * math.cos(point.heading_rad)
        yaw_rad = point.heading_rad + heading_perturbation_rad
        states.append((x_m, y_m, yaw_rad))
    return states


def main() -> None:
    raceline = Raceline.load_from_csv(RACELINE_PATH)
    vehicle_params = load_vehicle_params()
    controller = PurePursuitController(
        PurePursuitConfig(
            wheelbase_m=vehicle_params.chassis.wheelbase_m,
            lookahead_min_m=LOOKAHEAD_MIN_M,
            lookahead_max_m=LOOKAHEAD_MAX_M,
            lookahead_curvature_ref_1pm=LOOKAHEAD_CURVATURE_REF_1PM,
            max_steering_angle_rad=vehicle_params.steering.max_angle_rad,
        )
    )

    states = build_states(raceline)

    FIXTURES_DIR.mkdir(parents=True, exist_ok=True)
    with STATES_PATH.open("w", encoding="utf-8") as f:
        for x_m, y_m, yaw_rad in states:
            f.write(f"{x_m:.10f},{y_m:.10f},{yaw_rad:.10f}\n")

    # Recompute expected commands from the ROUND-TRIPPED text values (not the original
    # in-memory floats), so this fixture's expected.json matches exactly what any consumer
    # parsing divergence_states.csv as text will see -- including
    # test_base_controller_divergence.py and the C++ CLI, both of which only ever see text.
    reparsed_states: list[tuple[float, float, float]] = []
    for line in STATES_PATH.read_text(encoding="utf-8").splitlines():
        x_s, y_s, yaw_s = line.split(",")
        reparsed_states.append((float(x_s), float(y_s), float(yaw_s)))

    expected = []
    for x_m, y_m, yaw_rad in reparsed_states:
        cmd = controller.compute_command(raceline, x_m, y_m, yaw_rad)
        expected.append({"steering_angle_rad": cmd.steering_angle_rad, "speed_mps": cmd.speed_mps})
    EXPECTED_PATH.write_text(json.dumps(expected, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {len(reparsed_states)} states to {STATES_PATH}")
    print(f"Wrote {len(expected)} expected commands to {EXPECTED_PATH}")


if __name__ == "__main__":
    main()
