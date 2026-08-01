"""Python-side half of the S.3 cross-language base-controller divergence test.

The C++ side (ros_ws/src/racer_control/test/divergence/compare_divergence.py, wired into
ros_ws/src/racer_control/CMakeLists.txt) runs the SAME committed fixture through the real
C++ `pure_pursuit_cli` binary and compares against `fixtures/divergence_expected.json`. This
test is the other half: it proves `racer_train.raceline.PurePursuitController` (the thing
`generate_divergence_fixture.py` used to CREATE `divergence_expected.json`) still reproduces
that exact committed file today -- i.e. it is a regression test on the Python port itself,
independent of whether the C++ side agrees. If this test and the C++ colcon test are both
green, the two implementations agree with each other (transitively, through this shared
oracle file).
"""

from __future__ import annotations

import json
from pathlib import Path

from generate_divergence_fixture import (
    EXPECTED_PATH,
    LOOKAHEAD_CURVATURE_REF_1PM,
    LOOKAHEAD_MAX_M,
    LOOKAHEAD_MIN_M,
    RACELINE_PATH,
    STATES_PATH,
)
from racer_train.raceline import PurePursuitConfig, PurePursuitController, Raceline


def _load_states(path: Path) -> list[tuple[float, float, float]]:
    states = []
    for line in path.read_text(encoding="utf-8").splitlines():
        x_s, y_s, yaw_s = line.split(",")
        states.append((float(x_s), float(y_s), float(yaw_s)))
    return states


def test_python_controller_reproduces_committed_expected_commands(real_vehicle_params):
    states = _load_states(STATES_PATH)
    expected = json.loads(EXPECTED_PATH.read_text(encoding="utf-8"))
    assert len(states) == len(expected), (
        "fixtures/divergence_states.csv and divergence_expected.json have drifted apart in "
        "length -- regenerate both together with generate_divergence_fixture.py"
    )

    raceline = Raceline.load_from_csv(RACELINE_PATH)
    controller = PurePursuitController(
        PurePursuitConfig(
            wheelbase_m=real_vehicle_params.chassis.wheelbase_m,
            lookahead_min_m=LOOKAHEAD_MIN_M,
            lookahead_max_m=LOOKAHEAD_MAX_M,
            lookahead_curvature_ref_1pm=LOOKAHEAD_CURVATURE_REF_1PM,
            max_steering_angle_rad=real_vehicle_params.steering.max_angle_rad,
        )
    )

    for i, ((x_m, y_m, yaw_rad), expected_cmd) in enumerate(zip(states, expected)):
        cmd = controller.compute_command(raceline, x_m, y_m, yaw_rad)
        assert cmd.steering_angle_rad == expected_cmd["steering_angle_rad"], f"state {i}"
        assert cmd.speed_mps == expected_cmd["speed_mps"], f"state {i}"
