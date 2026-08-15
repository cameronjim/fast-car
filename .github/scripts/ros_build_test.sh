#!/usr/bin/env bash
# Builds and tests ros_ws with colcon (L3 node tests + C++ build/unit tests).
#
# CONTAINER SWAP (task 0.4 follow-up, done here): this used to run inside the stock
# `ros:humble` image from Docker Hub, deliberately deferred until ros_ws/src had real
# packages (see git history / the old comment here). Roadmap task S.2 lands
# ros_ws/src/racer_control, the first real package -- rclcpp/nav_msgs/ackermann_msgs and
# the launch_testing/gtest ROS test packages it needs are not all present in a bare
# `ros:humble` container, and resolving them there means duplicating rosdep bootstrap
# logic this repo's own `ros-dev` image (docker/ros-dev/, roadmap task 0.4) already
# carries. This script now runs inside that built ros-dev image (see the l3-and-cpp job in
# .github/workflows/ci.yml, which now builds+runs it the same way the sim-bridge-test job
# already does) instead of the `container:` + apt-get bootstrap shape used before.
#
# Same f1tenth_gym-importable-to-system-python3 approach as sim_bridge_build_test.sh is NOT
# needed here (ros_ws/src packages other than potential future gym-touching ones don't
# import f1tenth_gym), but rosdep resolution IS needed now that racer_control declares
# real `<depend>` entries (ackermann_msgs is not part of ros-base).
#
# Scaffold-aware: with no package.xml under ros_ws/src yet, prints a clear
# notice and passes instead of silently doing nothing.
set -euo pipefail

# GitHub Actions steps already run with cwd = the repo root, so no need to
# shell out to git for that (and inside a container job, git refuses to
# even answer `rev-parse` here with a "dubious ownership" error because
# the checkout is owned by a different uid than the container user).

# ROS's setup.bash references variables (e.g. AMENT_TRACE_SETUP_FILES) that
# are legitimately unset before sourcing it, which trips `set -u`. Relax
# just for the source line, per the standard ROS + `set -u` workaround.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

cd ros_ws

PACKAGE_XMLS="$(find src -maxdepth 2 -name package.xml 2>/dev/null || true)"

if [ -z "$PACKAGE_XMLS" ]; then
  echo "NOTICE: ros_ws/src has no buildable packages yet (no package.xml found)."
  echo "NOTICE: nothing to build or test. Passing."
  exit 0
fi

COUNT="$(echo "$PACKAGE_XMLS" | wc -l | tr -d ' ')"
echo "Found ${COUNT} package.xml file(s); running colcon build + colcon test."

echo "Refreshing apt package index (Dockerfile clears it after its own installs)..."
apt-get update

echo "Resolving ROS package dependencies for ros_ws/src with rosdep..."
rosdep update >/dev/null
rosdep install --from-paths src --ignore-src -r -y

# tools/gen_params.py's C++ binding generation (wired into racer_control's
# CMakeLists.txt via `uv run --project tools ...`) needs network access on first run to
# sync tools/'s venv from its lockfile; available in this CI job.
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
