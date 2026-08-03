#!/usr/bin/env bash
# clang-format --dry-run -Werror across every C/C++ source file in the repo,
# checked against the .clang-format config committed at repo root
# (claude-docs/10-conventions.md: "C++: clang-format (config committed at
# repo root)").
#
# Scaffold-aware: with no C++ files committed yet, this prints a clear notice
# and passes instead of silently doing nothing.
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

CPP_FILES="$(git ls-files -- '*.cpp' '*.hpp' '*.cc' '*.h' '*.cxx' '*.hh' || true)"

if [ -z "$CPP_FILES" ]; then
  echo "NOTICE: no C/C++ source files in the repo yet. Nothing to lint. Passing."
  exit 0
fi

COUNT="$(echo "$CPP_FILES" | wc -l | tr -d ' ')"
echo "Found ${COUNT} C/C++ file(s); running clang-format --dry-run -Werror."

echo "$CPP_FILES" | xargs clang-format --dry-run -Werror
