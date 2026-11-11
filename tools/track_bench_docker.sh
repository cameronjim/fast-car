#!/usr/bin/env bash
# Runs tools/track_bench.py inside the ros-dev image, with no network (nothing a run starts can
# see another ROS graph on the machine; runs inside the container are told apart by ROS_DOMAIN_ID),
# the repo at /workspace and the data tree at /data. The workspace (sim/bridge + ros_ws +
# tests/l5_reactive_bench) is built into a docker volume on first use and reused.
#
#   tools/track_bench_docker.sh run --suite family --out /data/runs/family --jobs 6
#   tools/track_bench_docker.sh report /data/runs/family
#
# Environment: IMAGE (default ros-dev:local), VOLUME (default racer-track-ws), DATA (host data
# tree, default ~/code/car-data), REBUILD=1 to rebuild the workspace after C++ changes (Python
# edits are symlinked and need nothing).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-ros-dev:local}"
VOLUME="${VOLUME:-racer-track-ws}"
DATA="${DATA:-$HOME/code/car-data}"

COMMON=(--rm --shm-size=2gb -v "$REPO_ROOT:/workspace" -v "$VOLUME:/ws" -v "$DATA:/data"
  -w /workspace --entrypoint bash)
ENV_SETUP='set +u; source /opt/ros/humble/setup.bash;
SITE="$(find /ros-dev/.venv/lib -maxdepth 1 -type d -name "python3.*" | head -n1)/site-packages";
export PYTHONPATH="${SITE}${PYTHONPATH:+:${PYTHONPATH}}"; export PYTHONFAULTHANDLER=1'

docker volume create "$VOLUME" >/dev/null
if [ "${REBUILD:-0}" = 1 ] || ! docker run "${COMMON[@]}" "$IMAGE" -c "test -f /ws/install/setup.bash"; then
  echo "Building the workspace into volume $VOLUME ..." >&2
  docker run "${COMMON[@]}" "$IMAGE" -c "$ENV_SETUP
    colcon build --symlink-install \
      --base-paths sim/bridge ros_ws tests/l5_reactive_bench \
      --build-base /ws/build --install-base /ws/install" >&2
fi

exec docker run "${COMMON[@]}" --network none "$IMAGE" -c "$ENV_SETUP
source /ws/install/setup.bash
exec python3 tools/track_bench.py \"\$@\"" track_bench "$@"
