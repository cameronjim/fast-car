#!/usr/bin/env bash
# L5 spiderbot test (docs/notes/spiderbot-2026-11-11.md, claude-docs/12-testing.md): builds
# sim/bridge/racer_gym_bridge, ros_ws (racer_apps, racer_safety, racer_control, ...) and
# tests/l5_spiderbot together in ONE colcon workspace inside the repo's own `ros-dev` image and
# runs tests/l5_spiderbot: the spiderbot patrols gym_oval through the real safety_node, stops for a
# synthetic person with raised arms, faces them, raises an alert, holds, resumes, and does not
# re-trigger inside its cooldown. A copy of .github/scripts/l5_localization_test.sh with the test
# package swapped (see .github/scripts/l5_tracker_lap_test.sh for the reasoning behind every step,
# kept identical on purpose). No map or AMCL here: the tracker reads the bridge's ground truth,
# the same sim-only adapter sim_autopilot.launch.py uses.
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
RACER_L5_WS_DIR="/tmp/racer_l5_spiderbot_ws"

PACKAGE_XMLS="$(find "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws/src/racer_apps" \
  "$REPO_ROOT/tests/l5_spiderbot" -maxdepth 2 -name package.xml 2>/dev/null || true)"

if [ -z "$PACKAGE_XMLS" ]; then
  echo "NOTICE: one or more of sim/bridge, ros_ws/src/racer_apps, tests/l5_spiderbot"
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
  "$REPO_ROOT/tests/l5_spiderbot" --ignore-src -r -y

echo "Building the combined L5 workspace (sim/bridge + ros_ws + l5_spiderbot)..."
colcon build --symlink-install \
  --base-paths "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws" "$REPO_ROOT/tests/l5_spiderbot" \
  --build-base "$RACER_L5_WS_DIR/build" --install-base "$RACER_L5_WS_DIR/install"

set +u
# shellcheck disable=SC1091
source "$RACER_L5_WS_DIR/install/setup.bash"
set -u

echo "Running the L5 spiderbot test (colcon test --shm-size handled by the caller's docker run)..."
colcon test --build-base "$RACER_L5_WS_DIR/build" --install-base "$RACER_L5_WS_DIR/install" \
  --packages-select l5_spiderbot --event-handlers console_direct+
colcon test-result --test-result-base "$RACER_L5_WS_DIR/build" --verbose
