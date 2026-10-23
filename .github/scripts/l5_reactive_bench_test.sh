#!/usr/bin/env bash
# L5 reactive benchmark, reduced battery (docs/notes/reactive-bench-2026-10-23.md): builds
# sim/bridge/racer_gym_bridge, ros_ws (racer_control, racer_safety, racer_msgs, ...), and
# tests/l5_reactive_bench together in ONE colcon workspace inside the repo's own `ros-dev` image,
# runs the runner's smoke test, then the CI battery of tools/reactive_bench.py: the
# floor-2026-10-06 profile on lane_square, lane_open_side and chicane (synthetic tracks of
# racer_gym_bridge's track_loader), each stopped at its first lap. FAILS if any run touches a wall
# or does not finish a lap. Bounded: at most 120 + 80 + 80 s of simulation plus start-up, about
# 5 to 6 minutes when every run fails to lap, 2 to 3 when they pass. A copy of
# .github/scripts/l5_reactive_lap_test.sh with the test package swapped; see that script and
# .github/scripts/l5_tracker_lap_test.sh for the reasoning behind every step (kept identical on
# purpose).
#
# Same f1tenth_gym-importable-to-system-python3 approach as sim_bridge_build_test.sh (see
# that script's own comment for why: PYTHONPATH, not prepending the whole uv venv).
# rosdep resolves ackermann_msgs (racer_gym_bridge, racer_control) and the launch_testing /
# gtest ROS packages' test dependencies the same way sim_bridge_build_test.sh already does.
#
# Build/install/log land in a scratch directory OUTSIDE the checked-out repo tree
# (RACER_L5_WS_DIR below) rather than under sim/bridge or ros_ws (which the existing
# per-package jobs' colcon invocations own). This job intentionally does not touch those
# other jobs' build layout.
#
# tools/gen_params.py's C++ binding generation (wired into racer_control's CMakeLists.txt
# via `uv run --project tools ...`) needs `uv` on PATH and network access on first run to
# sync tools/'s venv from its lockfile; both are available in this CI job.
set -euo pipefail

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

# GitHub Actions' docker-run step already sets cwd = the repo root (`-w /workspace`), so no
# need to shell out to git for that, and inside this container, git refuses to even
# answer `rev-parse` here with a "dubious ownership" error, because the bind-mounted
# workspace is owned by a different uid than the container's user (same reasoning as
# sim_bridge_build_test.sh, which avoids git for the same reason).
REPO_ROOT="$PWD"
RACER_L5_WS_DIR="/tmp/racer_l5_reactive_bench_ws"

PACKAGE_XMLS="$(find "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws/src/racer_control" \
  "$REPO_ROOT/tests/l5_reactive_bench" -maxdepth 2 -name package.xml 2>/dev/null || true)"

if [ -z "$PACKAGE_XMLS" ]; then
  echo "NOTICE: one or more of sim/bridge, ros_ws/src/racer_control, tests/l5_reactive_bench"
  echo "NOTICE: has no package.xml yet. Nothing to build or test. Passing."
  exit 0
fi

VENV_SITE_PACKAGES="$(find /ros-dev/.venv/lib -maxdepth 1 -type d -name 'python3.*' | head -n1)/site-packages"
if [ ! -d "$VENV_SITE_PACKAGES" ]; then
  echo "ERROR: expected ros-dev's uv venv site-packages at ${VENV_SITE_PACKAGES}, not found." >&2
  echo "ERROR: was this run inside the built ros-dev image?" >&2
  exit 1
fi
export PYTHONPATH="${VENV_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"

echo "Sanity check: f1tenth_gym importable from system python3 via PYTHONPATH..."
python3 -c "import f1tenth_gym; print('f1tenth_gym OK:', f1tenth_gym.__file__)"

export PYTHONFAULTHANDLER=1

echo "Refreshing apt package index..."
apt-get update

echo "Resolving ROS package dependencies with rosdep..."
rosdep update >/dev/null
# racer_safety and racer_msgs too: the benchmark runs safety_node in the loop. They are built from
# the same ros_ws base path below.
rosdep install --from-paths "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws/src/racer_control" \
  "$REPO_ROOT/ros_ws/src/racer_safety" "$REPO_ROOT/ros_ws/src/racer_msgs" \
  "$REPO_ROOT/tests/l5_reactive_bench" --ignore-src -r -y

echo "Building the combined L5 workspace (sim/bridge + racer_control + l5_reactive_bench)..."
colcon build --symlink-install \
  --base-paths "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws" "$REPO_ROOT/tests/l5_reactive_bench" \
  --build-base "$RACER_L5_WS_DIR/build" --install-base "$RACER_L5_WS_DIR/install"

set +u
# shellcheck disable=SC1091
source "$RACER_L5_WS_DIR/install/setup.bash"
set -u

echo "Running the runner smoke test (colcon test --shm-size handled by the caller's docker run)..."
colcon test --build-base "$RACER_L5_WS_DIR/build" --install-base "$RACER_L5_WS_DIR/install" \
  --packages-select l5_reactive_bench --event-handlers console_direct+
colcon test-result --test-result-base "$RACER_L5_WS_DIR/build" --verbose

echo "Running the reduced reactive battery (floor-2026-10-06 profile; fails on a wall contact or a missing lap)..."
python3 "$REPO_ROOT/tools/reactive_bench.py" --battery ci --check --no-note --profile floor-2026-10-06
