#!/usr/bin/env bash
# racer_safety gate-logic branch coverage gate (claude-docs/12-testing.md: "envelope/ and
# racer_safety gate at 100% branch on decision logic"; CLAUDE.md: "Every task's PR includes
# the tests required by claude-docs/12-testing.md ... Never weaken a tolerance, golden file,
# or coverage gate to get green").
#
# Builds ONLY racer_safety (+ its racer_msgs dependency) with --coverage instrumentation
# (RACER_SAFETY_COVERAGE=ON, see ros_ws/src/racer_safety/CMakeLists.txt) in its own build
# tree (separate from the plain, non-instrumented build the l3-and-cpp job produces), runs
# its gtest suite (test_racer_safety_core), then reports branch coverage with gcovr SCOPED to
# gate_logic.cpp/gate_logic.hpp only -- not safety_node.cpp (ROS plumbing, not decision
# logic, and not what this gate is about), not gate_logic_formatting.cpp (message-formatting
# helpers factored out specifically because std::to_string/string-concatenation calls are
# implemented as inline/template code that an unoptimized build attributes to the calling
# line -- see that file's own header comment), and not any system/ROS header gcov may have
# instrumented incidentally (claude-docs/12-testing.md anticipates exactly this: "If lcov
# branch counting produces noise from system headers, scope the report to the gate source
# files"). --exclude-throw-branches drops the compiler-generated exception-unwind edge every
# call to a (possibly-throwing) function or std::vector::push_back gets at -O0 -- noise no
# test can meaningfully "cover" without forcing bad_alloc, standard practice for a branch-
# coverage gate on ordinary C++. --fail-under-branch 100 makes gcovr's own exit code this
# script's gate -- a regression here fails the job, honestly, rather than being silently
# under-reported.
set -euo pipefail

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

cd ros_ws

if [ ! -f src/racer_safety/package.xml ]; then
  echo "NOTICE: ros_ws/src/racer_safety does not exist yet. Nothing to cover. Passing."
  exit 0
fi

echo "Refreshing apt package index (Dockerfile clears it after its own installs)..."
apt-get update

echo "Resolving ROS package dependencies with rosdep..."
rosdep update >/dev/null
rosdep install --from-paths src --ignore-src -r -y

echo "Building racer_safety (+ racer_msgs) with coverage instrumentation..."
colcon build --symlink-install --packages-up-to racer_safety \
  --cmake-args -DRACER_SAFETY_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug

echo "Running racer_safety's gtest suite..."
colcon test --packages-select racer_safety --ctest-args -R test_racer_safety_core \
  --event-handlers console_direct+
colcon test-result --verbose

GATE_LOGIC_CPP="$(pwd)/src/racer_safety/src/gate_logic.cpp"
GATE_LOGIC_HPP="$(pwd)/src/racer_safety/include/racer_safety/gate_logic.hpp"

echo "Computing branch coverage for gate_logic.cpp/.hpp (gcovr, scoped to those files)..."
# Search root is narrowed to racer_safety_core's own object directory (not the whole
# build/racer_safety tree): safety_node.cpp and the gtest binary itself are also
# --coverage-instrumented (racer_safety_core's PUBLIC compile options propagate to anything
# linking it) but are irrelevant to this gate and their .gcda references ROS message headers
# gcovr cannot resolve from outside a colcon-sourced environment, which is pure log noise
# once --filter has already excluded them from the report.
gcovr \
  --root . \
  --filter "${GATE_LOGIC_CPP}" \
  --filter "${GATE_LOGIC_HPP}" \
  --exclude-unreachable-branches \
  --exclude-throw-branches \
  --print-summary \
  --fail-under-branch 100 \
  build/racer_safety/CMakeFiles/racer_safety_core.dir
