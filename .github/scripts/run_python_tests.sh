#!/usr/bin/env bash
# L1 (unit) + L2 (property, hypothesis) test job for every Python package in
# the repo. See claude-docs/12-testing.md "CI wiring" for the gate policy:
#
#   - training/envelope/                : 100% branch coverage (decision logic)
#   - ros_ws/src/racer_policy/          : >=90% coverage (contract loader)
#   - sysid/fitting/                    : >=90% coverage
#   - evaluation/analysis/              : >=90% coverage
#   - everything else (training/racer_train, tools/, sim/racer_gym, other
#     ros_ws Python packages)           : coverage reported, not gated
#
# Each package is delegated to pytest_gate.sh, which is scaffold-aware: a
# package with no .py source yet prints a NOTICE and passes. Once a package
# gains real source + tests, its gate binds automatically on the next push.
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GATE_SCRIPT="${SCRIPT_DIR}/pytest_gate.sh"
FAIL=0

run() {
  local dir="$1" gate="$2"
  if ! "$GATE_SCRIPT" "$dir" "$gate"; then
    FAIL=1
  fi
}

# Critical / correctness-critical packages with explicit coverage gates.
run "training/envelope" branch100
run "ros_ws/src/racer_policy" line90
run "sysid/fitting" line90
run "evaluation/analysis" line90

# Normal packages: reported, not gated.
run "training/racer_train" report
run "tools" report
run "sim/racer_gym" report

# L4/L5/L6 test harness scaffolds (roadmap task 0.9, claude-docs/12-testing.md).
# Each is a standalone, ROS/gym-free dependency manifest (same pattern as
# sim/bridge/racer_gym_bridge below): report-gated, not coverage-enforced,
# since these are harness/tooling packages rather than the correctness-
# critical decision logic 12-testing.md names for a hard gate.
run "tests/replay_harness" report
run "tests/sim_in_loop" report
run "tests/bench" report

# tests/sim_regression (roadmap task S.6) is deliberately NOT run here: it installs and
# steps the pinned f1tenth_gym through several maneuvers (like sim/racer_gym above), which
# is not free, and 12-testing.md only requires the S.6 battery on a racer_gym change -- it
# runs in its own path-filtered CI job instead (sim-regression-battery in ci.yml), gated on
# sim/racer_gym/**, tests/sim_regression/**, and tests/replay_harness/** actually changing.

# sim/bridge/racer_gym_bridge (roadmap task 0.5): the pure, ROS-free logic
# in conversions.py runs here with no ROS/gym install (its pyproject.toml
# is a standalone test-only manifest); bridge_node.py and the L3
# launch_testing node tests need rclpy + f1tenth_gym and skip cleanly here
# via pytest.importorskip -- they run for real in the sim-bridge-test CI
# job instead (see .github/workflows/ci.yml).
run "sim/bridge/racer_gym_bridge" report

# Any other ros_ws Python package (racer_policy is handled above with its
# explicit gate; C++-only packages like racer_safety are covered by the L3 + C++
# (ros-dev) job instead, and pytest_gate.sh no-ops cleanly on a directory with no .py
# source). racer_control (roadmap task S.2) IS excluded here despite having .py files
# (launch/tracker.launch.py, test/test_tracker_node_launch.py): both need rclpy/launch_ros/
# launch_testing to do anything (the launch file only makes sense under `ros2 launch`; the
# L3 test needs the real tracker_node executable colcon builds), so a bare `uv run pytest`
# here has nothing meaningful to run -- unlike sim/bridge/racer_gym_bridge, it has no
# ROS-free pure-Python logic of its own to test this way (its ROS-free logic is the C++
# core, gtest-covered instead). It is exercised for real by the l3-and-cpp CI job (colcon
# build + colcon test, including the L3 launch test) via ros_build_test.sh.
if [ -d ros_ws/src ]; then
  for pkg_dir in ros_ws/src/*/; do
    pkg_name="$(basename "$pkg_dir")"
    if [ "$pkg_name" = "racer_policy" ] || [ "$pkg_name" = "racer_control" ]; then
      continue
    fi
    run "$pkg_dir" report
  done
else
  echo "NOTICE: ros_ws/src does not exist yet. Nothing to test yet. Passing."
fi

if [ "$FAIL" -ne 0 ]; then
  echo "One or more Python package gates failed. See groups above." >&2
  exit 1
fi

echo "All Python package checks passed (gated packages green, scaffold packages noticed)."
