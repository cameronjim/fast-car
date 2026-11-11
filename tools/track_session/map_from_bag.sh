#!/usr/bin/env bash
# Maps a recorded teleop run offline with the car's own mapping stack (robot_localization EKF +
# slam_toolbox, racer_state's config), inside the ros-dev image. One command:
#
#   tools/track_session/map_from_bag.sh <bag dir | .mcap | .npz> <out prefix> [feeder options]
#
#   tools/track_session/map_from_bag.sh /data/bags/2026-11-11T03-54-11_car_teleop \
#       /data/maps/2026-11-10-oval/oval --speed-divisor 1.401 \
#       --steer-scale-left 1.203 --steer-scale-right 1.072   (the 2026-11-10 bags only)
#
# The bag is extracted to an npz first (extract_bag.py, `uv run --project tools`), then replayed
# by bag_feeder.py with a /clock; the map is saved as <out prefix>.yaml/.pgm (nav2 map_server
# format, what localization.launch.py wants) and <out prefix>.posegraph/.data. Feeder options:
# --speed-divisor X (divide /odom/wheel speeds by X: 1.0, the default, for a bag from the calibrated
# car; 1.401 for the 2026-11-10 bags, recorded before the tyre radius was calibrated),
# --steer-scale-left/right (true wheel angle per command radian: 1.0 default; 1.203 and 1.072 for
# the 2026-11-10 bags), --twist-source feeder|node (feeder = the twist from those scales, node = the
# real wheel_twist_node, which cannot represent an uncalibrated servo), --rate R (playback speed,
# 1.0 default; keep it at 1 for twist-source node, whose
# staleness check runs on the wall clock).
#
# Environment: IMAGE (default ros-dev:local), VOLUME (workspace volume, default
# racer-track-ws, built on first use), DATA (host dir mounted at /data, default the parent of
# the first argument... set it when the bag and the output live in different trees).
#
# The slam_toolbox exit status is printed at the end (see docs/notes/track-session-2026-11-11.md
# for why that is worth looking at).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE="${IMAGE:-ros-dev:local}"
VOLUME="${VOLUME:-racer-track-ws}"
DATA="${DATA:-$HOME/code/car-data}"
INPUT="$1"
OUT_PREFIX="$2"
shift 2

if [[ "$INPUT" != *.npz ]]; then
  NPZ="$(mktemp -d)/run.npz"
  uv run --project "$REPO_ROOT/tools" python "$REPO_ROOT/tools/track_session/extract_bag.py" "$INPUT" "$NPZ"
  NPZ_DIR="$(dirname "$NPZ")"
  NPZ_IN="/npz/run.npz"
else
  NPZ_DIR="$(cd "$(dirname "$INPUT")" && pwd)"
  NPZ_IN="/npz/$(basename "$INPUT")"
fi

docker volume create "$VOLUME" >/dev/null
COMMON=(--rm --shm-size=1gb -v "$REPO_ROOT:/workspace" -v "$VOLUME:/ws" -v "$DATA:/data"
  -v "$NPZ_DIR:/npz" -w /workspace --entrypoint bash)
ENV_SETUP='set +u; source /opt/ros/humble/setup.bash;
SITE="$(find /ros-dev/.venv/lib -maxdepth 1 -type d -name "python3.*" | head -n1)/site-packages";
export PYTHONPATH="${SITE}${PYTHONPATH:+:${PYTHONPATH}}"; export PYTHONFAULTHANDLER=1'

if ! docker run "${COMMON[@]}" "$IMAGE" -c "test -f /ws/install/setup.bash"; then
  echo "Building the workspace into volume $VOLUME ..." >&2
  docker run "${COMMON[@]}" "$IMAGE" -c "$ENV_SETUP
    colcon build --symlink-install --base-paths sim/bridge ros_ws \
      --build-base /ws/build --install-base /ws/install" >&2
fi

exec docker run "${COMMON[@]}" --network none "$IMAGE" -c "$ENV_SETUP
source /ws/install/setup.bash
set +e
TWIST=feeder
for a in \"\$@\"; do [ \"\$a\" = node ] && TWIST=node; done
mkdir -p \"\$(dirname '$OUT_PREFIX')\"
env --default-signal=INT ros2 launch tools/track_session/offline_map.launch.py twist_source:=\$TWIST > /tmp/launch.log 2>&1 &
LAUNCH=\$!
sleep 6
python3 tools/track_session/bag_feeder.py --npz '$NPZ_IN' --out-prefix '$OUT_PREFIX' \"\$@\"
FEED=\$?
# A background job of a non-interactive shell starts with SIGINT ignored, which ros2 launch
# inherits: env --default-signal above puts it back, or this INT would do nothing.
kill -INT \$LAUNCH
wait \$LAUNCH
echo \"feeder exit \$FEED, launch exit \$?\"
grep -E 'process has died|exit code|Aborted|terminate' /tmp/launch.log | head
tail -n 5 /tmp/launch.log
exit \$FEED" map_from_bag "$@"
