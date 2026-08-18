#!/usr/bin/env bash
# Milestone 1 end-to-end test (claude-docs/12-testing.md L5-flavored): builds
# sim/bridge/racer_gym_bridge and ros_ws (racer_msgs + racer_safety) together in ONE colcon
# workspace (they need to see each other's installed executables for the launch_testing test
# to launch both bridge_node and safety_node with no remaps -- the REAL command path), inside
# the repo's own `ros-dev` image (docker/ros-dev/, roadmap task 0.4), and runs the test.
#
# Same shape as .github/scripts/l5_tracker_lap_test.sh (see that script's own comment for the
# full rationale behind each piece: f1tenth_gym-importable-to-system-python3 via PYTHONPATH,
# rosdep resolution, a scratch build dir outside the checked-out tree).
set -euo pipefail

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

REPO_ROOT="$PWD"
RACER_E2E_WS_DIR="/tmp/racer_e2e_sim_safety_ws"

PACKAGE_XMLS="$(find "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws/src/racer_msgs" \
  "$REPO_ROOT/ros_ws/src/racer_safety" "$REPO_ROOT/tests/e2e_sim_safety" \
  -maxdepth 2 -name package.xml 2>/dev/null || true)"

if [ -z "$PACKAGE_XMLS" ]; then
  echo "NOTICE: one or more of sim/bridge, ros_ws/src/racer_msgs, ros_ws/src/racer_safety,"
  echo "NOTICE: tests/e2e_sim_safety has no package.xml yet. Nothing to build or test. Passing."
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
rosdep install --from-paths "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws" \
  "$REPO_ROOT/tests/e2e_sim_safety" --ignore-src -r -y

echo "Building the combined e2e workspace (sim/bridge + ros_ws + e2e_sim_safety)..."
colcon build --symlink-install \
  --base-paths "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws" "$REPO_ROOT/tests/e2e_sim_safety" \
  --build-base "$RACER_E2E_WS_DIR/build" --install-base "$RACER_E2E_WS_DIR/install"

set +u
# shellcheck disable=SC1091
source "$RACER_E2E_WS_DIR/install/setup.bash"
set -u

echo "Running the milestone 1 sim+safety e2e test (colcon test --shm-size handled by the caller's docker run)..."
colcon test --build-base "$RACER_E2E_WS_DIR/build" --install-base "$RACER_E2E_WS_DIR/install" \
  --packages-select e2e_sim_safety --event-handlers console_direct+
colcon test-result --test-result-base "$RACER_E2E_WS_DIR/build" --verbose
