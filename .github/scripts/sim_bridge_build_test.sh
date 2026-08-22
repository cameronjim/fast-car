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

# DIAGNOSTIC (task 0.5 CI debugging): colcon test's own crash report for
# racer_gym_bridge's launch_testing run is a bare "terminate called without
# an active exception" / SIGABRT with no Python traceback -- i.e. the abort
# happens below Python's exception machinery. PYTHONFAULTHANDLER makes CPython
# install a low-level handler that dumps the Python-level stack of every
# thread on a fatal signal, which colcon's own capture does not enable.
# Also run the same rclpy+gym construction colcon would run, standalone and
# with -X faulthandler, to localize the abort before colcon's own (noisier)
# invocation.
export PYTHONFAULTHANDLER=1
echo "Diagnostic: constructing rclpy + f1tenth_gym together outside colcon..."
set +e
python3 -X faulthandler -c "
import rclpy
print('rclpy imported', flush=True)
import f1tenth_gym  # noqa: F401
print('f1tenth_gym imported', flush=True)
import numpy as np
import gymnasium as gym
from f1tenth_gym.envs.track import Track
print('building track...', flush=True)
xs = np.linspace(0, 50, 100)
ys = np.sin(xs / 3.0) * 3.0
velxs = np.full_like(xs, 3.0)
track = Track.from_refline(x=xs, y=ys, velx=velxs)
print('track built, making env...', flush=True)
env = gym.make(
    'f1tenth_gym:f1tenth-v0',
    config={
        'seed': 42,
        'map': track,
        'num_agents': 1,
        'ego_idx': 0,
        'observation_config': {'type': 'features', 'features': ['scan', 'pose_x', 'pose_y', 'pose_theta', 'linear_vel_x', 'linear_vel_y', 'ang_vel_z']},
    },
    render_mode=None,
)
print('env made, calling rclpy.init()...', flush=True)
rclpy.init()
print('rclpy.init() OK, resetting env...', flush=True)
obs, info = env.reset(seed=42)
print('env.reset() OK, creating a node...', flush=True)
node = rclpy.create_node('diag_node')
print('node created, stepping env...', flush=True)
action = np.array([[0.0, 2.0]])
obs, r, term, trunc, info = env.step(action)
print('env.step() OK, spinning once...', flush=True)
rclpy.spin_once(node, timeout_sec=0.5)
print('spin_once OK, shutting down...', flush=True)
node.destroy_node()
env.close()
rclpy.shutdown()
print('DIAGNOSTIC DONE', flush=True)
"
diag_status=$?
set -e
echo "Diagnostic finished with exit code ${diag_status}"

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
