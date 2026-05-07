#!/usr/bin/env bash
# Builds and tests ros_ws with colcon (L3 node tests + C++ build/unit tests).
#
# INTERIM CONTAINER: this runs inside the stock `ros:humble` image from
# Docker Hub because the repo's own `ros-dev` image (roadmap task 0.4,
# claude-docs/03-environments.md) does not exist yet.
# TODO(task 0.4): once docker/ros-dev/ has a built, pinned image, point the
# workflow's `container:` at that image instead, and drop the apt-get
# bootstrap step in ci.yml (colcon etc. will already be baked in).
#
# Scaffold-aware: with no package.xml under ros_ws/src yet, prints a clear
# notice and passes instead of silently doing nothing.
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd ros_ws

PACKAGE_XMLS="$(find src -maxdepth 2 -name package.xml 2>/dev/null || true)"

if [ -z "$PACKAGE_XMLS" ]; then
  echo "NOTICE: ros_ws/src has no buildable packages yet (no package.xml found)."
  echo "NOTICE: nothing to build or test. Passing."
  exit 0
fi

COUNT="$(echo "$PACKAGE_XMLS" | wc -l | tr -d ' ')"
echo "Found ${COUNT} package.xml file(s); running colcon build + colcon test."

colcon build --symlink-install
colcon test
colcon test-result --verbose
