#!/usr/bin/env bash
# L5 lap learning canary (docs/notes/preregistration-pnp-v0.2.md section 5.1, brief 28): builds
# sim/bridge/racer_gym_bridge, ros_ws (racer_laps, racer_planner, racer_track_perception,
# racer_safety, racer_pnp_msgs, ...) and tests/l5_pnp_laps together in ONE colcon workspace inside
# the repo's own `ros-dev` image and runs tests/l5_pnp_laps: the Plug-and-Play driver with the
# lap memory (start-scan lap source, D v0) laps the tuning stadium through the real command path
# on a car-like plant, the system's lap count and lap times agree with the simulator's ground
# truth, no wall contact, the cap rises, one publisher each on /drive_raw and /drive; and the
# gate does not close at half a lap. Also runs the decision functions D-S0 to D-S2 without ROS.
# A copy of .github/scripts/l5_pnp_test.sh with the test package swapped; see that script and
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
RACER_L5_WS_DIR="/tmp/racer_l5_pnp_laps_ws"

PACKAGE_XMLS="$(find "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws/src/racer_control" \
  "$REPO_ROOT/tests/l5_pnp_laps" -maxdepth 2 -name package.xml 2>/dev/null || true)"

if [ -z "$PACKAGE_XMLS" ]; then
  echo "NOTICE: one or more of sim/bridge, ros_ws/src/racer_control, tests/l5_pnp_laps"
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
# All of ros_ws/src: racer_tools (vehicle_params loader), racer_safety, racer_msgs and
# racer_bringup (the launch file the runner reads the car-like bridge settings from) are needed
# too, and rosdep only skips the workspace packages it was pointed at.
rosdep install --from-paths "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws/src" \
  "$REPO_ROOT/tests/l5_pnp_laps" --ignore-src -r -y

echo "Building the combined L5 workspace (sim/bridge + ros_ws + l5_pnp)..."
colcon build --symlink-install \
  --base-paths "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws" "$REPO_ROOT/tests/l5_pnp_laps" \
  --build-base "$RACER_L5_WS_DIR/build" --install-base "$RACER_L5_WS_DIR/install"

set +u
# shellcheck disable=SC1091
source "$RACER_L5_WS_DIR/install/setup.bash"
set -u

echo "Running the lap learning canary (colcon test --shm-size handled by the caller's docker run)..."
colcon test --build-base "$RACER_L5_WS_DIR/build" --install-base "$RACER_L5_WS_DIR/install" \
  --packages-select l5_pnp_laps --event-handlers console_direct+
colcon test-result --test-result-base "$RACER_L5_WS_DIR/build" --verbose
