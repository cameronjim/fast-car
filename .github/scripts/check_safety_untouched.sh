#!/usr/bin/env bash
# Guard for the safety package (brief 45 and brief 49).
#
# The owner approved changes to ros_ws/src/racer_safety twice:
#   * decision 1 of owner-decisions-2026-11-14.md (brief 45): receipt-time instrumentation in
#     safety_node.cpp, with tests;
#   * decision 7 of owner-decisions-2026-11-16.md, taken on 2026-11-17 (brief 49): the rate
#     limiter's rate credit, so a late cycle followed by a catch-up cycle no longer clips an
#     ordinary command, with its own tests.
#
# This check makes "nothing else changed" mechanical. On a pull request it fails when the diff
# against the base lists anything under ros_ws/src/racer_safety other than
#
#   include/racer_safety/gate_logic.hpp   the rate rule and its two threaded fields (brief 49)
#   src/gate_logic.cpp                    the rate rule (brief 49)
#   src/safety_node.cpp                   the node (threading only; the A/B test proves the rest)
#   src/safety_replay_cli.cpp             the replay tool threads the credits as the node does
#   CMakeLists.txt                        CMake lines for the tests
#   test/**                               tests
#
# so forward_sector.* and gate_logic_formatting.* stay read only, and package.xml does too (brief
# 49 does not need it). A future change to the gate needs the owner's approval AND an edit to the
# allowed list below in the same PR, where a reviewer sees it.
#
# usage: check_safety_untouched.sh [base-ref]   (default origin/main; needs history, fetch-depth 0)
set -euo pipefail

BASE="${1:-origin/main}"
PACKAGE="ros_ws/src/racer_safety"
ALLOWED='^ros_ws/src/racer_safety/(include/racer_safety/gate_logic\.hpp|src/gate_logic\.cpp|src/safety_node\.cpp|src/safety_replay_cli\.cpp|CMakeLists\.txt|test/.*)$'

cd "$(git rev-parse --show-toplevel)"

CHANGED="$(git diff --name-only "${BASE}...HEAD" -- "${PACKAGE}")"
if [ -z "${CHANGED}" ]; then
  echo "OK: nothing under ${PACKAGE} differs from ${BASE}."
  exit 0
fi

echo "Changed under ${PACKAGE} relative to ${BASE}:"
echo "${CHANGED}" | sed 's/^/  /'

UNEXPECTED="$(echo "${CHANGED}" | grep -Ev "${ALLOWED}" || true)"
if [ -n "${UNEXPECTED}" ]; then
  echo "FAIL: these files are outside what owner decision 1 of 2026-11-14 and owner decision 7 of 2026-11-17 (brief 49) approve:" >&2
  echo "${UNEXPECTED}" | sed 's/^/  /' >&2
  echo "Allowed: gate_logic.hpp, gate_logic.cpp, safety_node.cpp, safety_replay_cli.cpp, CMakeLists.txt, test/**." >&2
  echo "forward_sector.* and gate_logic_formatting.* (and package.xml) are read only." >&2
  exit 1
fi

echo "OK: only the approved files changed; forward_sector.* and gate_logic_formatting.* are byte-identical to ${BASE}."
