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
# Plug-and-Play (claude-docs/13-plug-and-play.md): the bundle schema, and the per-part training
# packages. A directory that does not exist yet passes with a notice.
run "config/pnp" line90
run "training/racer_pnp_perception" report
run "training/racer_pnp_dynamics" line90
# Lap memory (D): the start-scan gate, the station set, the engine and the cap schedule are the
# decision logic that raises the car's speed lap over lap. 90 percent line coverage; the ROS node
# and the mcap reader are omitted in pyproject.toml and covered by the L3 and real-bag checks. The
# generic ros_ws loop below runs the package again as report only.
run "ros_ws/src/racer_laps" line90

# Normal packages: reported, not gated.
run "training/racer_train" report
run "tools" report
# tools/pnp_session (brief 34, the car session timing report): 90 percent line coverage of the
# package itself. The `session` extra (racer_eval, scipy) is installed by --all-extras. The tests
# that need the built limiter_whatif_cli skip here and run in the ROS job (ctest).
echo "::group::pytest: tools/pnp_session (gate=line90 on the package)"
if ! (cd tools && uv sync --all-extras --dev &&
  uv run pytest tests/test_pnp_session.py tests/test_pnp_session_v04.py tests/test_pnp_session_lap_c0.py tests/test_pnp_cap_schedule.py tests/test_pnp_excitation_plan.py --cov=pnp_session --cov-report=term-missing \
    --cov-fail-under=90); then
  FAIL=1
fi
echo "::endgroup::"
# tools/pnp_data and tools/lapref (brief 36, the data loop and the offline lap reference): 90 percent
# line coverage of the two packages. They need the `session` extra (racer_eval) and mcap-ros2-support,
# both installed by --all-extras; the bag writer in the tests is mcap_ros2's.
echo "::group::pytest: tools/pnp_data and tools/lapref (gate=line90 on the two packages)"
if ! (cd tools && uv sync --all-extras --dev &&
  uv run pytest tests/test_pnp_data.py tests/test_pnp_data_v05.py tests/test_lapref.py tests/test_lapref_compare.py --cov=pnp_data --cov=lapref \
    --cov-report=term-missing --cov-fail-under=90); then
  FAIL=1
fi
echo "::endgroup::"
# The racer app's backend agent (tools/racer_app/agent): the docker command whitelist, the
# pre-drive checks, the disk guard and the HTTP API, all tested against a fake docker/systemd/ROS
# layer. Gated at 90 percent because it decides what the app is allowed to make the car's
# computer do. The web UI half (vitest) runs in the racer-app job of ci.yml.
run "tools/racer_app/agent" line90
run "sim/racer_gym" report

# L4/L5/L6 test harness scaffolds (roadmap task 0.9, claude-docs/12-testing.md).
# Each is a standalone, ROS/gym-free dependency manifest (same pattern as
# sim/bridge/racer_gym_bridge below): report-gated, not coverage-enforced,
# since these are harness/tooling packages rather than the correctness-
# critical decision logic 12-testing.md names for a hard gate.
run "tests/replay_harness" report
run "tests/sim_in_loop" report
run "tests/bench" report
run "tests/l5_reactive_bench" report
# tests/l5_pnp_laps (brief 30): the lap battery decision functions and the B4 tool, with their
# L2 properties (hypothesis). The ROS canary in the same directory skips here.
run "tests/l5_pnp_laps" report
# tests/l5_pnp_oracle (brief 40): the oracle track's geometry and the guard that the car's launch
# cannot start it. The ROS parity test and the canary in the same directory skip here.
run "tests/l5_pnp_oracle" report
# tests/launch_support: the orderly end of a launch_testing stack the L5 and e2e canaries use
# (one DDS participant leaves at a time). ROS-free, needs /proc; skips itself elsewhere.
run "tests/launch_support" report

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

# Any other ros_ws Python package (racer_policy is handled above with its explicit gate;
# racer_tools has its own standalone pyproject.toml -- see that package's comment on why --
# so it runs through this generic loop). racer_control and racer_safety (milestone 1) are
# BOTH excluded here despite having .py files (launch/*.launch.py, test/test_*_launch.py):
# both need rclpy/launch_ros/launch_testing to do anything (the launch file only makes sense
# under `ros2 launch`; the L3 test needs the real tracker_node/safety_node executable colcon
# builds), so a bare `uv run pytest` here has nothing meaningful to run -- unlike
# sim/bridge/racer_gym_bridge or racer_tools, they have no ROS-free pure-Python logic of
# their own to test this way (their ROS-free logic is the C++ core, gtest-covered instead).
# racer_session_tools (brief 34) is excluded too: a C++ tool with a gtest and one stdlib-only ctest
# script (test/check_limiter_reproduction.py), no pyproject.toml; its Python half is
# tools/pnp_session, gated above.
# racer_bringup is excluded for the same reason -- launch files plus an L3 launch test, no
# pyproject.toml, nothing a bare `uv run pytest` could exercise -- and so is racer_drivers,
# whose only .py file is test/test_pwm_output_node_launch.py: it launches the C++
# pwm_output_node executable colcon builds and needs rclpy, exactly like racer_safety's, and
# its ROS-free logic is C++ covered by gtest. racer_state (2026-11-03) is excluded for the
# same reason as racer_control: a C++ node with a gtest-covered core, an L3 launch test, and a
# config check registered with ament_cmake_pytest. All five are exercised for real by the
# l3-and-cpp CI job (colcon build + colcon test, including their L3 launch tests) via
# ros_build_test.sh.
if [ -d ros_ws/src ]; then
  for pkg_dir in ros_ws/src/*/; do
    pkg_name="$(basename "$pkg_dir")"
    if [ "$pkg_name" = "racer_policy" ] || [ "$pkg_name" = "racer_control" ] ||
       [ "$pkg_name" = "racer_safety" ] || [ "$pkg_name" = "racer_bringup" ] ||
       [ "$pkg_name" = "racer_drivers" ] || [ "$pkg_name" = "racer_state" ] ||
       [ "$pkg_name" = "racer_session_tools" ]; then
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
