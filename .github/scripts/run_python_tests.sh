#!/usr/bin/env bash
# L1 (unit) + L2 (property, hypothesis) test job for every Python package in
# the repo. See claude-docs/12-testing.md "CI wiring" for the gate policy:
#
#   - training/envelope/                : 100% branch coverage (decision logic)
#   - ros_ws/src/racer_policy/          : >=90% coverage (contract loader)
#   - sysid/fitting/                    : >=90% coverage
#   - evaluation/analysis/              : >=90% coverage
#   - everything else (training/racer_train, tools/, other ros_ws Python
#     packages)                         : coverage reported, not gated
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

# Any other ros_ws Python package (racer_policy is handled above with its
# explicit gate; C++-only packages like racer_safety/racer_control are
# covered by the L3 + C++ (ros-dev) job instead, and pytest_gate.sh no-ops
# cleanly on a directory with no .py source).
if [ -d ros_ws/src ]; then
  for pkg_dir in ros_ws/src/*/; do
    pkg_name="$(basename "$pkg_dir")"
    if [ "$pkg_name" = "racer_policy" ]; then
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
