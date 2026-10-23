#!/usr/bin/env bash
# Runs tools/reactive_bench.py inside the ros-dev image, on an isolated docker network, with the
# L5 workspace (sim/bridge + ros_ws + tests/l5_reactive_bench) built into a docker volume once and
# reused. Never touches a car: the simulator only.
#
#   tools/reactive_bench_docker.sh --profile floor-2026-10-06,none --jobs 2
#   tools/reactive_bench_docker.sh --battery ci --check --no-note
#
# Every argument goes to tools/reactive_bench.py (see its --help). Environment:
#   IMAGE     ros-dev image to use (default ros-dev:local; build it with docker/ros-dev)
#   REBUILD   set to 1 to rebuild the workspace (needed after C++ changes; Python edits are
#             symlinked and picked up without it)
#   DOMAIN_BASE  first ROS_DOMAIN_ID (default 60); the container is the only thing on its network.
#
# The network is `--internal` (no route out), so nothing the benchmark starts can discover or be
# discovered by other ROS graphs on the machine, and the distinct ROS_DOMAIN_IDs keep runs inside
# the container apart from each other. The first build needs the default network for uv/rosdep,
# so it runs there.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-ros-dev:local}"
VOLUME="racer-reactive-bench-ws"
NETWORK="racer-reactive-bench-net"
DOMAIN_BASE="${DOMAIN_BASE:-60}"

COMMON=(--rm --shm-size=1gb -v "$REPO_ROOT:/workspace" -v "$VOLUME:/ws" -w /workspace)
ENV_SETUP='set +u; source /opt/ros/humble/setup.bash;
SITE="$(find /ros-dev/.venv/lib -maxdepth 1 -type d -name "python3.*" | head -n1)/site-packages";
export PYTHONPATH="${SITE}${PYTHONPATH:+:${PYTHONPATH}}"; export PYTHONFAULTHANDLER=1'

docker volume create "$VOLUME" >/dev/null
docker network inspect "$NETWORK" >/dev/null 2>&1 || docker network create --internal "$NETWORK" >/dev/null

if [ "${REBUILD:-0}" = 1 ] || ! docker run "${COMMON[@]}" --entrypoint bash "$IMAGE" \
    -c "test -f /ws/install/setup.bash"; then
  echo "Building the L5 workspace into volume $VOLUME ..." >&2
  docker run "${COMMON[@]}" --entrypoint bash "$IMAGE" -c "$ENV_SETUP
    colcon build --symlink-install \
      --base-paths sim/bridge ros_ws tests/l5_reactive_bench \
      --build-base /ws/build --install-base /ws/install" >&2
fi

exec docker run "${COMMON[@]}" --network "$NETWORK" -e "ROS_DOMAIN_ID=$DOMAIN_BASE" \
  --entrypoint bash "$IMAGE" -c "$ENV_SETUP
  source /ws/install/setup.bash
  exec python3 tools/reactive_bench.py --domain-base $DOMAIN_BASE \"\$@\"" reactive_bench "$@"
