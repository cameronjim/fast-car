#!/usr/bin/env bash
# Runs tools/tests/test_pnp_replay.py against the planner_replay_cli that colcon just built, so
# the end to end planner replay tests run in CI instead of being skipped (tools/pnp_replay.py,
# docs/notes/pnp-replay.md). Run after ros_build_test.sh, in the same ros-dev container.
# PNP_REPLAY_REQUIRE_CLI makes a missing binary a failure instead of a skip.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CLI="${ROOT}/ros_ws/install/racer_planner/lib/racer_planner/planner_replay_cli"
if [ ! -x "$CLI" ]; then
  echo "ERROR: ${CLI} was not built; run ros_build_test.sh first." >&2
  exit 1
fi

export PLANNER_REPLAY_CLI="$CLI"
export PNP_REPLAY_REQUIRE_CLI=1
# ROS's setup.bash puts its python3.10 site-packages (and pytest plugins) on PYTHONPATH, which
# reaches the tools/ venv's interpreter and breaks it; the CMake gen_params step unsets it too.
unset PYTHONPATH
cd "${ROOT}/tools"
uv run pytest tests/test_pnp_replay.py -v -rs
