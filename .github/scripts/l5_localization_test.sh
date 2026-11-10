#!/usr/bin/env bash
# L5 localisation canaries (docs/notes/localization-2026-11-03.md, claude-docs/12-testing.md):
# builds sim/bridge/racer_gym_bridge, ros_ws (racer_state, racer_control, racer_bringup, ...),
# and tests/l5_localization together in ONE colcon workspace inside the repo's own `ros-dev`
# image and runs every test in tests/l5_localization: slam_toolbox maps a lane track and the
# saved map is compared with the sim's; tracker_node laps on the AMCL pose (ground-truth TF off)
# and the pose error is checked against ground truth. A copy of .github/scripts/l5_park_test.sh
# with the test package swapped; see .github/scripts/l5_tracker_lap_test.sh for the reasoning
# behind every step (kept identical on purpose). robot_localization, slam_toolbox and the nav2
# packages come from the image (docker/ros-dev/Dockerfile); rosdep finds them installed.
#
# Same f1tenth_gym-importable-to-system-python3 approach as sim_bridge_build_test.sh (see
# that script's own comment for why: PYTHONPATH, not prepending the whole uv venv).
#
# Build/install/log land in a scratch directory OUTSIDE the checked-out repo tree
# (RACER_L5_WS_DIR below) rather than under sim/bridge or ros_ws (which the existing
# per-package jobs' colcon invocations own). This job intentionally does not touch those
# other jobs' build layout.
#
# tools/gen_params.py's C++ binding generation (racer_control's and racer_state's
# CMakeLists.txt, `uv run --project tools ...`) needs `uv` on PATH and network access on first
# run to sync tools/'s venv from its lockfile; both are available in this CI job.
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
RACER_L5_WS_DIR="/tmp/racer_l5_localization_ws"

PACKAGE_XMLS="$(find "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws/src/racer_state" \
  "$REPO_ROOT/tests/l5_localization" -maxdepth 2 -name package.xml 2>/dev/null || true)"

if [ -z "$PACKAGE_XMLS" ]; then
  echo "NOTICE: one or more of sim/bridge, ros_ws/src/racer_state, tests/l5_localization"
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
# All of ros_ws/src, not a list: racer_bringup (whose mapping and localization launch files the
# canaries run) exec_depends most of the other racer_* packages, and rosdep only skips the
# workspace packages it was pointed at. Same set ros_build_test.sh resolves.
rosdep install --from-paths "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws/src" \
  "$REPO_ROOT/tests/l5_localization" --ignore-src -r -y

echo "Building the combined L5 workspace (sim/bridge + ros_ws + l5_localization)..."
colcon build --symlink-install \
  --base-paths "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws" "$REPO_ROOT/tests/l5_localization" \
  --build-base "$RACER_L5_WS_DIR/build" --install-base "$RACER_L5_WS_DIR/install"

set +u
# shellcheck disable=SC1091
source "$RACER_L5_WS_DIR/install/setup.bash"
set -u

echo "Running the L5 localisation canaries (colcon test --shm-size handled by the caller's docker run)..."
colcon test --build-base "$RACER_L5_WS_DIR/build" --install-base "$RACER_L5_WS_DIR/install" \
  --packages-select l5_localization --event-handlers console_direct+
colcon test-result --test-result-base "$RACER_L5_WS_DIR/build" --verbose
