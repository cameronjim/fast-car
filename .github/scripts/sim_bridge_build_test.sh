#!/usr/bin/env bash
# Builds and tests sim/bridge/racer_gym_bridge with colcon (roadmap task
# 0.5: the gym<->ROS bridge), inside the repo's own `ros-dev` image
# (docker/ros-dev/, roadmap task 0.4).
#
# `sim/bridge` is the workspace root here (there is only ever one package
# in it, so unlike ros_ws there is no extra `src/` layer) -- `--base-paths .`
# tells colcon to crawl this directory for packages instead of the default
# `src`, and build/install/log land in sim/bridge/ itself, mirroring how
# ros_ws/build sits next to ros_ws/src (see ros_build_test.sh, .gitignore).
#
# Unlike ros_ws, this package genuinely needs a dependency this image
# didn't carry before: f1tenth_gym, pinned in docker/ros-dev/pyproject.toml
# (same commit SHA as docker/sim-cpu) and installed into /ros-dev/.venv.
# This script makes it importable to the SAME system python3.10 that
# rclpy/colcon use -- without prepending the whole venv to PATH, which
# would shadow python3 and break colcon (see the Dockerfile comment) -- by
# adding just that venv's site-packages to PYTHONPATH for this invocation
# only.
#
# `rosdep install` resolves racer_gym_bridge's package.xml ROS
# dependencies (e.g. ackermann_msgs) that ros-dev's apt bootstrap does not
# install by default. The Dockerfile's own `apt-get update && ... && rm
# -rf /var/lib/apt/lists/*` (standard image-size hygiene) leaves this
# container's apt package index empty, so `apt-get update` is run again
# here before rosdep can install anything new.
#
# Scaffold-aware like ros_build_test.sh: with no package.xml under
# sim/bridge yet, prints a clear notice and passes instead of failing.
set -euo pipefail

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

WORKSPACE_DIR="sim/bridge"

PACKAGE_XMLS="$(find "$WORKSPACE_DIR" -maxdepth 2 -name package.xml 2>/dev/null || true)"

if [ -z "$PACKAGE_XMLS" ]; then
  echo "NOTICE: ${WORKSPACE_DIR} has no buildable packages yet (no package.xml found)."
  echo "NOTICE: nothing to build or test. Passing."
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

# PYTHONFAULTHANDLER: if a launch_testing subprocess ever aborts again (a
# missing native shared library surfaces as a bare SIGABRT with no Python
# traceback inside a colcon-test-launched subprocess, rather than a clean
# ImportError -- see the ros-dev Dockerfile comment on the X11/GL client
# libs for exactly this failure mode), this makes CPython dump each
# thread's Python-level stack on the fatal signal instead of nothing.
export PYTHONFAULTHANDLER=1

echo "Refreshing apt package index (Dockerfile clears it after its own installs)..."
apt-get update

echo "Resolving ROS package dependencies for ${WORKSPACE_DIR} with rosdep..."
rosdep update >/dev/null
rosdep install --from-paths "$WORKSPACE_DIR" --ignore-src -r -y

cd "$WORKSPACE_DIR"

COUNT="$(echo "$PACKAGE_XMLS" | wc -l | tr -d ' ')"
echo "Found ${COUNT} package.xml file(s); running colcon build + colcon test."

colcon build --symlink-install --base-paths .
colcon test --base-paths .
colcon test-result --verbose
