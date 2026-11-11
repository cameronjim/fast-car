#!/usr/bin/env bash
# Run the real-YOLO tests (and the CPU benchmark) inside the GPU car image, on the CPU.
#
#   tools/perception_cpu_tests.sh              # tests only
#   tools/perception_cpu_tests.sh bench        # plus the latency benchmark on the test photos
#
# Works on a Mac (colima, aarch64) and on the Jetson: the image is linux/arm64 either way and
# CUDA is never touched (device cpu). Needs the image built first, see docker/car/README.md
# "Perception"; the pinned weights are baked into it (PERCEPTION_WEIGHTS=bake), so no network is
# needed for the models. pytest is installed into a throwaway directory at run time (pinned) because
# the car image deliberately carries no test tooling. Nothing here needs ROS to be built: the tests
# it runs are the ones that do not use rclpy.
#
# Why this is not in CI: a CPU torch plus the weights is hundreds of MB per run, and the l4t base
# image cannot be pulled on a hosted runner (docker/car/README.md "CI"). The L1 tests that use the
# recorded keypoints, the weights checksums and the fake backends all run in CI.
set -euo pipefail

IMAGE="${IMAGE:-car:perception}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THREADS="${OMP_NUM_THREADS:-2}"

docker run --rm -e OMP_NUM_THREADS="$THREADS" -v "$ROOT":/workspace -w /workspace/ros_ws/src/racer_perception \
  "$IMAGE" bash -lc '
    set -euo pipefail
    pip3 install --quiet --no-cache-dir --target /tmp/pt pytest==8.3.4 2>&1 | tail -1
    PYTHONPATH=/tmp/pt:. python3 -m pytest -q -p no:cacheprovider test/test_yolo_images.py test/test_weights.py
  '

if [ "${1:-}" = "bench" ]; then
  for task in detect pose; do
    echo "--- $task, 640 px photographs, OMP_NUM_THREADS=$THREADS"
    docker run --rm -e OMP_NUM_THREADS="$THREADS" -v "$ROOT":/workspace -w /workspace/ros_ws/src/racer_perception \
      "$IMAGE" bash -lc "PYTHONPATH=. python3 -m racer_perception.bench --task $task --device cpu --runs 30 test/fixtures/images/*.jpg"
  done
fi
