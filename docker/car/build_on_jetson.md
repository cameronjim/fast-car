# Building and running `car` on the Jetson Orin Nano

DRAFT procedure (roadmap task 1.4) -- written before the Jetson exists, from JetPack/ROS/uv
documentation, never executed. Correct it against reality the first time it's actually run,
and note what changed in `claude-docs/01-roadmap.md`'s task 1.4 entry.

## Prerequisites

- Jetson Orin Nano 8GB flashed with JetPack 6.1 (L4T r36.4.0) via NVIDIA SDK Manager or the
  SD-card image, per NVIDIA's own Jetson Orin Nano setup guide. This is the FIRST hardware
  step (`claude-docs/01-roadmap.md` task 1.4 depends on 1.1-1.2 for the vehicle itself, but
  flashing the Jetson has no dependency on the rest of the chassis).
- Docker installed on the Jetson with the NVIDIA Container Runtime configured as default
  (JetPack images typically ship this already; confirm with
  `docker info | grep -i runtime` showing `nvidia`).
- SSH access from the Mac to the Jetson (`claude-docs/03-environments.md`: "The Mac never
  talks to hardware directly ... it SSHes to the car" -- this build happens over that SSH
  session, not on the Mac).

## 1. Look up the JetPack-6.1-matched torch wheel

The Dockerfile deliberately does not hardcode this (see its own comment: the exact wheel
filename/URL changes with every JetPack/torch release and could not be verified without the
actual hardware). On the Jetson itself, or from NVIDIA's docs:

1. Read NVIDIA's current install page:
   https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html
2. Confirm the JetPack version matches this image's base
   (`cat /etc/nv_tegra_release` on the Jetson should show R36 ... 4.x for JetPack 6.1).
3. Find the matching wheel under
   https://developer.download.nvidia.com/compute/redist/jp/ (a `v6x/pytorch/` subdirectory)
   or the URL NVIDIA's install page gives directly for that JetPack version.
4. Record the exact URL used -- it goes in the `docker build` command below AND should be
   written into this file (replacing this paragraph) once confirmed, so the next build does
   not have to re-derive it.

## 2. Build the image

Run on the Jetson (or over SSH from the Mac, per the environments doc above):

```
git clone <this repo> car && cd car   # or pull, if already cloned
docker build \
  --build-arg TORCH_WHEEL_URL=<the URL from step 1> \
  -t car:local \
  docker/car
```

Expect this to fail the first few times -- it has never been run. Likely early failures:
apt package name drift in the ROS 2 apt repo, the torch wheel needing a different `pip3`
invocation (e.g. `--index-url` instead of a direct wheel URL, or an additional
`--extra-index-url` for a dependency), or missing system libraries torch's wheel dynamically
links against (check `ldd` on the installed `torch/_C*.so` if `import torch` fails).

## 3. Sanity-check the built image

```
docker run --rm --runtime nvidia car:local python3 -c "
import torch
print('torch:', torch.__version__)
print('cuda available:', torch.cuda.is_available())
import yaml, jsonschema
print('pyyaml/jsonschema importable: OK')
"
docker run --rm car:local bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | head"
```

## 4. Building the workspace (ros_ws/src) on-device

This image is the toolchain, not a pre-baked `ros_ws` (same shape as `docker/ros-dev/` for
desktop builds) -- there is no arm64 cross-compilation setup in this repo. Build the
workspace inside a container from this image, with the repo bind-mounted:

```
docker run --rm -it --runtime nvidia \
  -v $(pwd):/workspace -w /workspace \
  car:local bash -c "
    source /opt/ros/humble/setup.bash
    rosdep install --from-paths ros_ws/src --ignore-src -r -y
    cd ros_ws && colcon build --symlink-install
  "
```

## 5. Running a launch file

```
docker run --rm -it --runtime nvidia \
  --network host \
  --device /dev/ttyUSB0 \
  -v $(pwd):/workspace -w /workspace/ros_ws \
  car:local bash -c "
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch racer_bringup <the on-vehicle launch file, once one exists>
  "
```

`--device` entries above are placeholders -- the real device paths (VESC, LiDAR, ingest
board) get filled in as `claude-docs/11-hardware.md`'s wiring is actually done; see
`docs/notes/hardware-arrival-checklist.md`.

## Once this all actually works

Tick roadmap task 1.4 to `[x]` in `claude-docs/01-roadmap.md` with a dated note, and correct
every "DRAFT"/"never executed" claim in this file and `README.md` -- an honest doc that says
"this hasn't been tried" stops being honest the moment it has been.
