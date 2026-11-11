#!/usr/bin/env bash
# L5 policy canary (docs/notes/residual-v1-2026-11-11.md): builds sim/bridge/racer_gym_bridge,
# ros_ws (racer_control, racer_safety, racer_msgs, racer_tools, racer_policy, ...) and
# tests/l5_policy together in ONE colcon workspace inside the repo's own `ros-dev` image, then
# runs (1) racer_policy's torch-dependent tests (policy_node on a live graph), and (2)
# tests/l5_policy: tracker_node in handoff mode + policy_node + safety_node lap a stadium in both
# directions against the tracker alone. A copy of .github/scripts/l5_track_family_test.sh with
# the packages swapped and one addition: ros-dev has no torch (by design, docker/ros-dev), and
# the policy tests load a TorchScript fixture network, so this script installs the CPU torch
# wheel into a scratch directory on PYTHONPATH first. setuptools is deleted from that directory
# afterwards: colcon-core needs the image's own (older) setuptools, and a newer one that torch's
# dependencies drag in breaks every ament_python build with "invalid syntax".
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
RACER_L5_WS_DIR="/tmp/racer_l5_policy_ws"
TORCH_DIR="/tmp/racer_l5_policy_torch"

PACKAGE_XMLS="$(find "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws/src/racer_control" \
  "$REPO_ROOT/tests/l5_policy" -maxdepth 2 -name package.xml 2>/dev/null || true)"

if [ -z "$PACKAGE_XMLS" ]; then
  echo "NOTICE: one or more of sim/bridge, ros_ws/src/racer_control, tests/l5_policy"
  echo "NOTICE: has no package.xml yet. Nothing to build or test. Passing."
  exit 0
fi

VENV_SITE_PACKAGES="$(find /ros-dev/.venv/lib -maxdepth 1 -type d -name 'python3.*' | head -n1)/site-packages"
if [ ! -d "$VENV_SITE_PACKAGES" ]; then
  echo "ERROR: expected ros-dev's uv venv site-packages at ${VENV_SITE_PACKAGES}, not found." >&2
  echo "ERROR: was this run inside the built ros-dev image?" >&2
  exit 1
fi
export PYTHONPATH="${VENV_SITE_PACKAGES}:${TORCH_DIR}${PYTHONPATH:+:${PYTHONPATH}}"

echo "Sanity check: f1tenth_gym importable from system python3 via PYTHONPATH..."
python3 -c "import f1tenth_gym; print('f1tenth_gym OK:', f1tenth_gym.__file__)"

export PYTHONFAULTHANDLER=1

echo "Refreshing apt package index..."
apt-get update

echo "Installing the CPU torch wheel into ${TORCH_DIR} (the policy tests load a TorchScript fixture)..."
apt-get install -y python3-pip
pip3 install --target "$TORCH_DIR" torch --index-url https://download.pytorch.org/whl/cpu \
  --extra-index-url https://pypi.org/simple
rm -rf "$TORCH_DIR"/setuptools "$TORCH_DIR"/setuptools-*.dist-info "$TORCH_DIR"/_distutils_hack \
  "$TORCH_DIR"/pkg_resources "$TORCH_DIR"/distutils-precedence.pth
python3 -c "import torch; print('torch OK:', torch.__version__)"

echo "Resolving ROS package dependencies with rosdep..."
rosdep update >/dev/null
# All of ros_ws/src: racer_tools (vehicle_params loader), racer_safety, racer_msgs and
# racer_bringup (the launch file the runner reads the car-like bridge settings from) are needed
# too, and rosdep only skips the workspace packages it was pointed at.
rosdep install --from-paths "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws/src" \
  "$REPO_ROOT/tests/l5_policy" --ignore-src -r -y

echo "Building the combined L5 workspace (sim/bridge + ros_ws + l5_policy)..."
colcon build --symlink-install \
  --base-paths "$REPO_ROOT/sim/bridge" "$REPO_ROOT/ros_ws" "$REPO_ROOT/tests/l5_policy" \
  --build-base "$RACER_L5_WS_DIR/build" --install-base "$RACER_L5_WS_DIR/install"

set +u
# shellcheck disable=SC1091
source "$RACER_L5_WS_DIR/install/setup.bash"
set -u

echo "Running racer_policy's torch-dependent tests (policy_node on a live graph, model load)..."
# Only the files that need torch: the rest of racer_policy's suite runs in the l3-and-cpp job via
# colcon test, where torch is absent (one of its tests asserts exactly that).
python3 -m pytest -p no:cacheprovider -q \
  "$REPO_ROOT/ros_ws/src/racer_policy/tests/test_policy_node_ros.py" \
  "$REPO_ROOT/ros_ws/src/racer_policy/tests/test_startup.py"

echo "Running the policy canary..."
colcon test --build-base "$RACER_L5_WS_DIR/build" --install-base "$RACER_L5_WS_DIR/install" \
  --packages-select l5_policy --event-handlers console_direct+
colcon test-result --test-result-base "$RACER_L5_WS_DIR/build" --verbose
