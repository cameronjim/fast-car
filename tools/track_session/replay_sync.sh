#!/usr/bin/env bash
# Deterministic offline mapping of one recorded run with a chosen slam_toolbox parameter file.
#
#   tools/track_session/replay_sync.sh <npz> <slam params yaml> <out prefix> [replay_sync.py options]
#
# Runs inside the ros-dev image (IMAGE, default ros-dev:track), no network, the repo at /workspace.
# Pass absolute paths under DATA (default ~/code/car-data), which is mounted at the same path.
# ROS_DOMAIN_ID (default random) keeps concurrent replays apart; CPUS limits the container. sync_slam_toolbox_node queues every scan (the live async node
# drops scans when busy), so the map depends on the inputs and not on how fast the machine is.
# tf_buffer_duration is raised so a long queue cannot outrun the transform buffer; nothing else
# in the parameter file is touched.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE="${IMAGE:-ros-dev:track}"
DATA="${DATA:-$HOME/code/car-data}"
NPZ="$1"; PARAMS="$2"; OUT="$3"; shift 3
DOMAIN="${ROS_DOMAIN_ID:-$((RANDOM % 100 + 1))}"
mkdir -p "$(dirname "$OUT")"
exec docker run --rm --network none --shm-size=1gb --cpus "${CPUS:-2}" \
  -v "$REPO_ROOT:/workspace" -v "$DATA:$DATA" -w /workspace --entrypoint bash "$IMAGE" -c '
set +u; source /opt/ros/humble/setup.bash; set -u
SITE="$(find /ros-dev/.venv/lib -maxdepth 1 -type d -name "python3.*" | head -n1)/site-packages"
export PYTHONPATH="${SITE}${PYTHONPATH:+:${PYTHONPATH}}" ROS_DOMAIN_ID='"$DOMAIN"'
ros2 run slam_toolbox sync_slam_toolbox_node --ros-args --params-file "'"$PARAMS"'" \
  -p use_sim_time:=true -p tf_buffer_duration:=600.0 > "'"$OUT"'.slam.log" 2>&1 &
SP=$!
sleep 4
python3 tools/track_session/replay_sync.py "'"$NPZ"'" "'"$OUT"'" '"$*"'
RC=$?
kill $SP 2>/dev/null || true
exit $RC'
