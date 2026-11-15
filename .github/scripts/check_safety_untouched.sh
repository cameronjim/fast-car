#!/usr/bin/env bash
# Guard for the safety package (brief 45; owner decision 1 of owner-decisions-2026-11-14.md).
#
# The owner approved ONE change to ros_ws/src/racer_safety: receipt-time instrumentation in
# safety_node.cpp, with tests. This check makes "nothing else changed" mechanical. On a pull
# request it fails when the diff against the base lists anything under ros_ws/src/racer_safety
# other than
#
#   src/safety_node.cpp   the node (plumbing only; the A/B test proves the gated output identical)
#   package.xml           manifest
#   CMakeLists.txt        CMake lines for the message dependency and the tests
#   test/**               tests
#
# so gate_logic.*, forward_sector.*, gate_logic_formatting.* and safety_replay_cli.cpp can only
# be byte-identical to the base. A future change to the gate needs the owner's approval AND an
# edit to the allowed list below in the same PR, where a reviewer sees it.
#
# usage: check_safety_untouched.sh [base-ref]   (default origin/main; needs history, fetch-depth 0)
set -euo pipefail

BASE="${1:-origin/main}"
PACKAGE="ros_ws/src/racer_safety"
ALLOWED='^ros_ws/src/racer_safety/(src/safety_node\.cpp|package\.xml|CMakeLists\.txt|test/.*)$'

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
  echo "FAIL: these files are outside what owner decision 1 of 2026-11-14 approves:" >&2
  echo "${UNEXPECTED}" | sed 's/^/  /' >&2
  echo "The gate library (gate_logic.*, forward_sector.*, safety_replay_cli.cpp) is read only." >&2
  exit 1
fi

echo "OK: only the approved files changed; the gate library is byte-identical to ${BASE}."
