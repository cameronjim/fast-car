#!/usr/bin/env bash
# Runs pytest (with the appropriate coverage gate) for a single Python
# package directory. This is the scaffold-aware building block used by
# run_python_tests.sh for every package in claude-docs/12-testing.md's L1/L2
# coverage table.
#
# Usage: pytest_gate.sh <package_dir> <gate>
#   gate: "branch100" | "line90" | "report"
#     branch100 - envelope/: 100% branch coverage on decision logic, enforced
#     line90    - racer_policy, sysid/fitting, evaluation/analysis: >=90%
#     report    - everything else: coverage reported, not gated
#
# Scaffold-aware behavior (this is the mechanism, not a switch to flip by
# hand): if the package directory does not exist yet, or exists but has no
# .py source files, this prints a loud NOTICE and exits 0 - nothing to test
# yet. The moment a package gains real source + a pyproject.toml, this same
# script starts running pytest --cov with --cov-fail-under for gated
# packages automatically, on the very next push. There is no separate label
# or flag anywhere that skips a package once it has code; the only way past
# this gate at that point is a passing coverage run.
set -euo pipefail

PKG_DIR="$1"
GATE="$2"

echo "::group::pytest — ${PKG_DIR} (gate=${GATE})"

if [ ! -d "$PKG_DIR" ]; then
  echo "NOTICE: ${PKG_DIR} does not exist yet. Nothing to test. Passing."
  echo "::endgroup::"
  exit 0
fi

PY_FILES="$(find "$PKG_DIR" -name '*.py' \
  -not -path '*/.venv/*' \
  -not -path '*/__pycache__/*' \
  -not -name '*_generated.py' \
  2>/dev/null || true)"

if [ -z "$PY_FILES" ]; then
  echo "NOTICE: ${PKG_DIR} is still empty scaffold (no .py source files). Nothing to test yet. Passing."
  echo "::endgroup::"
  exit 0
fi

if [ ! -f "$PKG_DIR/pyproject.toml" ]; then
  echo "ERROR: ${PKG_DIR} has Python source but no pyproject.toml, so uv cannot" >&2
  echo "install it. Add a per-package pyproject.toml (claude-docs/10-conventions.md," >&2
  echo "claude-docs/02-repo-layout.md) before this gate can run." >&2
  echo "::endgroup::"
  exit 1
fi

COUNT="$(echo "$PY_FILES" | wc -l | tr -d ' ')"
echo "Found ${COUNT} .py file(s) and a pyproject.toml in ${PKG_DIR}; running pytest."

(
  cd "$PKG_DIR"
  uv sync --all-extras --dev

  case "$GATE" in
    branch100)
      uv run pytest --cov=. --cov-branch --cov-report=term-missing --cov-fail-under=100
      ;;
    line90)
      uv run pytest --cov=. --cov-report=term-missing --cov-fail-under=90
      ;;
    report)
      uv run pytest --cov=. --cov-report=term-missing
      ;;
    *)
      echo "Unknown gate type: ${GATE}" >&2
      exit 1
      ;;
  esac
)

echo "::endgroup::"
