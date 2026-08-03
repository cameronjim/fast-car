#!/usr/bin/env bash
# Ruff lint + format check across every Python file in the repo, at line
# length 100 per claude-docs/10-conventions.md ("Python": ruff, line length
# 100). Config lives in the root ruff.toml (there is no per-package pyproject
# yet, and lint config is repo-wide by nature, unlike per-image dependency
# lockfiles).
#
# Scaffold-aware: with no Python files committed yet, this prints a clear
# notice and passes instead of silently doing nothing.
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

PY_FILES="$(git ls-files -- '*.py' || true)"

if [ -z "$PY_FILES" ]; then
  echo "NOTICE: no Python files in the repo yet. Nothing to lint. Passing."
  exit 0
fi

COUNT="$(echo "$PY_FILES" | wc -l | tr -d ' ')"
echo "Found ${COUNT} Python file(s); running ruff check + ruff format --check."

ruff check .
ruff format --check .
