#!/usr/bin/env bash
# Runs tools/race_tune.py inside the ros-dev image, with no network (nothing a run starts can see
# another ROS graph on the machine), the repo at /workspace and the data tree at /data. The
# workspace (sim/bridge + ros_ws + tests/l5_reactive_bench) is built into a docker volume on first
# use and reused, exactly like tools/track_bench_docker.sh.
#
#   tools/race_tune_docker.sh search --out /data/runs/race/search --samples 24 --refine 2 --jobs 5
#   tools/race_tune_docker.sh evaluate --out /data/runs/race/heldout --jobs 5 \
#       --variant 'race={"profile": "race-2026-11-12", "params": {}}'
#   tools/race_tune_docker.sh replay --bags /data/bags --profile race-2026-11-12
#   tools/race_tune_docker.sh latency --bag /data/bags/<bag> --seconds 40
#
# Environment: IMAGE (default ros-dev:local; the image must carry vesc_msgs, see
# docker/ros-dev/Dockerfile), VOLUME (default racer-race-ws), DATA (host data tree, default
# ~/code/car-data), REBUILD=1 to rebuild the workspace after C++ changes.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-ros-dev:local}"
VOLUME="${VOLUME:-racer-race-ws}"
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

# Reading bags (replay, latency) needs mcap, which the image does not carry: installed once into the
# volume (the only step that needs the network), then found through PYTHONPATH.
if [ "${1:-}" = replay ] || [ "${1:-}" = latency ]; then
  if ! docker run "${COMMON[@]}" "$IMAGE" -c "test -d /ws/pydeps/mcap"; then
    echo "Installing mcap into volume $VOLUME ..." >&2
    docker run "${COMMON[@]}" "$IMAGE" -c "uv pip install --quiet --python /usr/bin/python3 \
      --target /ws/pydeps 'mcap>=1.6.0' 'mcap-ros2-support>=0.6.0'" >&2
  fi
fi

exec docker run "${COMMON[@]}" --network none "$IMAGE" -c "$ENV_SETUP
export PYTHONPATH=/ws/pydeps:\$PYTHONPATH
source /ws/install/setup.bash
export GAP_FOLLOW_REPLAY_CLI=/ws/install/racer_control/lib/racer_control/gap_follow_replay_cli
exec python3 tools/race_tune.py \"\$@\"" race_tune "$@"
