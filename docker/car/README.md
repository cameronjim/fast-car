# car

On-vehicle image for the Jetson Orin Nano (roadmap task 1.4, `claude-docs/03-environments.md`).
**BUILT AND RUN on the real Jetson on 2026-09-21** (JetPack 6.2.1 / L4T R36.4.4): the image
builds, and `racer_bringup/car_teleop.launch.py` runs out of it with `safety_node`,
`pwm_output_node` and `foxglove_bridge` all up and both PWM channels reaching the safety-mux
Pico. Two Dockerfile bugs were found and fixed in the process (`DEBIAN_FRONTEND`, and the
missing `ros-humble-foxglove-bridge`) -- see `docs/notes/build-log.md`'s 2026-09-21 evening
entry. `build_on_jetson.md` has the build procedure; `docs/notes/first-boot-runbook.md`'s
"Launch and drive" section has the run procedure, which is the one that has actually been
executed.

Contains JetPack 6.1 (L4T r36.4.0, pinned by tag + digest), ROS 2 Humble installed from the
ROS apt repo, **optionally** the JetPack-matched NVIDIA torch wheel (build-arg, never a
guessed URL -- see `Dockerfile`), and `racer_policy`'s runtime Python deps via a `uv`-locked,
linux/aarch64 lockfile (`pyproject.toml` + `uv.lock`). Never contains dev tooling (test frameworks,
linters) -- that is `docker/ros-dev/`'s job, not this image's. It DOES contain
`ros-humble-foxglove-bridge`, which was in that exclusion list until 2026-09-21: see the
Dockerfile's comment on it, and `docs/notes/build-log.md`. On this image the bridge is the
owner's driving interface, not developer visualization. It also contains
`ros-humble-rosbag2-storage-mcap` (added 2026-09-21): `car_teleop.launch.py` records a bag on
every run because `CLAUDE.md` invariant 5 makes an unlogged drive a bug, and mcap is runtime
logging infrastructure rather than dev tooling. Without it the launch silently falls back to
sqlite3, which `.github/scripts/check_car_image_launch_packages.py` now refuses to allow.

## Torch is optional (changed 2026-09-13)

`INSTALL_TORCH` selects it, and defaults to `skip`:

| Build arg | Effect |
|---|---|
| `INSTALL_TORCH=skip` (default) | no torch. A loud NOTICE block in the build log, `/etc/racer/torch-status` = `absent`, an `/etc/racer/torch-absent` marker file, and `RACER_TORCH=absent` exported (with the notice repeated) into every login shell. |
| `INSTALL_TORCH=required` | installs `TORCH_WHEEL_URL`, and **fails the build** if that arg is empty -- the original refuse-rather-than-guess behaviour, unchanged. |
| anything else | hard build error. |

Why: torch exists in this image for exactly one consumer, `racer_policy` inference, which is
roadmap phase 5.x. The phase in front of us is the classical control stack (`safety_node`,
`pwm_output_node`, teleop), which imports no torch, and refusing the whole build until
someone hand-resolves a JetPack-matched wheel blocked a first boot on a dependency first boot
does not use. Flipping the default back to `required` when phase 5 starts is a one-word
change in the Dockerfile.

## DDS pinning (added 2026-09-14)

`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (Humble's default, pinned explicitly so it cannot drift)
and `ROS_DOMAIN_ID=42` (off the crowded default domain 0, and clear of the 77-82 range the
`ros_ws` launch tests already use for CI isolation) are baked into this image via `ENV`. This
matters specifically because trackside operation puts the car and the Mac's `ros-dev`
container on the same WiFi/subnet at the same time -- see `docker/car/Dockerfile`'s comment
and `docs/notes/first-boot-runbook.md` for the full reasoning. `docker/ros-dev/Dockerfile`
deliberately does NOT set `ROS_DOMAIN_ID`, so the launch tests' own per-file
`os.environ.setdefault("ROS_DOMAIN_ID", ...)` calls keep working.

## LiDAR driver (added 2026-10-05, roadmap 2.3)

The image builds Slamtec's `sllidar_ros2` (the RPLIDAR C1's ROS 2 driver) from source at a
pinned commit, `34300099fadfc772965962dec837bf436706188f` (the repo has no `humble` branch;
`main` supports Humble and lists the C1). It is installed into its own prefix,
`/opt/racer_thirdparty`, which the image puts on `AMENT_PREFIX_PATH`, so after the usual
`source /opt/ros/humble/setup.bash && source install/setup.bash` the package is found with no
extra step. The apt `ros-humble-rplidar-ros` package is deliberately not used: nobody has
verified it supports the C1. Why it lives in the image rather than `ros_ws/src` is in the
`Dockerfile` comment. **The image has to be rebuilt to get this layer**, and `ros_ws` rebuilt
for `lidar.launch.py` and `lidar_check`.

**Running with the LiDAR.** The C1 talks over a CP210x USB-UART adapter, which shows up on the
Jetson as `/dev/ttyUSB0` (group `dialout`). The unprivileged car container needs the device
node and the group, nothing else:

```sh
  --device /dev/ttyUSB0 \
  --group-add "$(getent group dialout | cut -d: -f3)" \
```

Or, with `tools/udev/99-racer-lidar.rules` installed on the host, use the stable symlink, so
the LiDAR keeps its name once another USB serial device is plugged in:

```sh
  --device /dev/lidar \
  --group-add "$(getent group dialout | cut -d: -f3)" \
  ...  ros2 launch racer_bringup lidar.launch.py serial_port:=/dev/lidar
```

`docs/notes/first-boot-runbook.md`'s "LiDAR first power-up" has the full command lines, the
`lidar_check` run and the expected numbers. With the whole teleop stack, add the same two
flags to the "Start the stack" command and pass `lidar:=true` (read that launch argument's
description first: `/scan` arms `safety_node`'s TTC gate).

## Cameras (added 2026-10-07; OPTIONAL, outside the thesis)

Cameras are for detection experiments and training-data collection. Nothing in the command
path reads them, and the car drives exactly the same without them. Hardware: one Waveshare
IMX219-160 on the Orin Nano's CAM0 socket (a second IMX219 later, sensor_id 1) and one ELP
AR0234 global-shutter UVC camera on USB 3.

**What the image adds** (one apt layer, all from the ROS and Ubuntu repos, nothing built from
source): `ros-humble-usb-cam` 0.8.1 (USB camera), `ros-humble-gscam` 2.0.2 (GStreamer to ROS,
used for the CSI camera with an `nvarguscamerasrc` pipeline), `ros-humble-image-transport-plugins`
(the compressed JPEG transport), the GStreamer tools and base/good plugins, and `v4l-utils`.
The build refuses unless `usb_cam`, `gscam`, `compressed_image_transport` and `videoconvert`
are all found. **The image has to be rebuilt to get this layer**, and `ros_ws` rebuilt for the
camera launch files and `camera_check`.

**Why gscam for the CSI camera.** The IMX219 on Jetson is a raw Bayer sensor behind the ISP,
reachable only through NVIDIA's libargus (the `nvarguscamerasrc` GStreamer element talking to
the host's `nvargus-daemon`), not through plain V4L2. Of the ROS 2 options for Humble on
JetPack 6, gscam is the lowest risk: it is in the apt repo for arm64, takes any GStreamer
pipeline, and is small. gscam2 adds nothing needed here and would be a source build;
NVIDIA's `isaac_ros_argus_camera` needs the Isaac ROS container and NITROS, far too heavy for
an optional camera. `camera_csi.launch.py`'s docstring has the details.

**Why a separate container, car-camera.** The CSI camera needs the NVIDIA container runtime
and the host's Argus socket. car-stack is deliberately started without either (unprivileged,
no `--runtime nvidia`, see `docs/notes/first-boot-runbook.md` "Start the stack"), so the
cameras run in their own container from the same `car:local` image, on the host network:

```sh
cd ~/car
USBCAM=/dev/video1          # see the runbook for how to find N
docker run --rm -it --name car-camera --network host \
  --runtime nvidia \
  --user "$(id -u):$(id -g)" \
  --group-add "$(getent group video | cut -d: -f3)" \
  -e HOME=/tmp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/ros_ws/src/racer_bringup/config/fastdds_udp_only.xml \
  -v /tmp/argus_socket:/tmp/argus_socket \
  --device "$USBCAM" \
  -v "$PWD":/workspace -w /workspace/ros_ws \
  car:local bash -lc "
    source /opt/ros/humble/setup.bash && source install/setup.bash
    exec ros2 launch racer_bringup cameras.launch.py usb_device:=$USBCAM"
```

What each camera flag is for:

| Flag | Why | Status |
|---|---|---|
| `--runtime nvidia` | The NVIDIA container runtime mounts the host's L4T libraries (libargus client, the `nvarguscamerasrc` / `nvvidconv` GStreamer plugins) and the Tegra device nodes (`/dev/nvmap`, `/dev/nvhost-*` or their R36 equivalents) into the container. The image does not carry them. | UNVERIFIED on this host: check `gst-inspect-1.0 nvarguscamerasrc` inside the container. |
| `-v /tmp/argus_socket:/tmp/argus_socket` | `nvarguscamerasrc` is only a client: the camera is driven by `nvargus-daemon` on the HOST, reached through this Unix socket. | UNVERIFIED |
| `--group-add video` | The Tegra device nodes and `/dev/videoN` are group `video` on L4T. The container runs as your UID, not root. | UNVERIFIED for the R36 Tegra nodes; certain for `/dev/videoN` on stock Ubuntu. |
| `--device /dev/videoN` | The USB camera. Must be the plain `/dev/videoN`, same path inside as outside: usb_cam 0.8.1 only accepts a device it finds under `/sys/class/video4linux`, so a `/dev/v4l/by-id/` path is rejected. The CSI sensor's own `/dev/video0` is NOT needed in the container (Argus opens it on the host). | from the usb_cam 0.8.1 source |
| `-e FASTRTPS_DEFAULT_PROFILES_FILE=...fastdds_udp_only.xml` | car-stack and car-camera share the host network but not `/dev/shm`. Fast DDS would otherwise try to hand data between them through shared memory the other side cannot open: topics visible, no messages. This profile makes car-camera's participants UDP-only, so car-stack's run command does not change. The alternative is `--ipc host` on both containers. | Reproduced on the Mac 2026-10-06 with two ros-dev containers sharing one network namespace and separate `/dev/shm`: without the profile `camera_check` saw the topics and 0 frames; with it in the publishing container only, 60.00 Hz PASS. Not yet on the Jetson. |

If `gst-inspect-1.0 nvarguscamerasrc` fails with an nvmap / nvhost error rather than "No such
element", the runtime did not pass a device node: list them on the host with `ls -l
/dev/nvhost* /dev/nvmap /dev/host1x* 2>/dev/null` and add each with `--device`. If it says "No
such element", the runtime did not mount the plugin: check
`grep -ri argus /etc/nvidia-container-runtime/host-files-for-container.d/` on the host. Neither
is `--privileged`, and car-camera should never need it.

**Same container instead.** `car_teleop.launch.py cameras:=true camera_usb_device:=/dev/videoN`
includes the cameras in car-stack itself. That only works if car-stack is started with the
camera flags above, which widens the one container that drives the car; prefer car-camera.

**Recording.** car-stack's recorder takes `/camera/.*/compressed` and leaves the raw images
out, wherever the cameras run. Expected bag growth at 720p60 is in the runbook ("Cameras first
power-up"): roughly 5 to 10 MB/s per camera, an estimate until `camera_check` measures it.

## Base image tag vs. the device (decided 2026-09-13)

The bench Jetson runs L4T R36.4.4 (JetPack 6.2.1); this image pins r36.4.0 (JetPack 6.1).
That gap is deliberate and not fixable by bumping the tag: `nvcr.io/nvidia/l4t-jetpack`
publishes nothing newer than `r36.4.0` (full tag list queried against the registry API on
2026-09-13; `l4t-base` stops at `r36.2.0`). An older L4T container on a newer L4T host of the
same major is the supported direction -- the driver comes from the host -- and both are
JetPack 6 / jammy, which is what the ROS 2 Humble apt install depends on. The digest was
re-resolved on 2026-09-13 and is unchanged. Whether the 6.1 CUDA stack behaves on a 6.2.1
host is an on-device check (`build_on_jetson.md` step 4) that has not been done; the control
stack needs no CUDA, so it gates phase 5, not first boot.

## What is and isn't verified

| Claim | Verified how |
|---|---|
| `nvcr.io/nvidia/l4t-jetpack:r36.4.0`'s pinned digest is a real, currently-published arm64 manifest | Yes -- resolved from this Mac against nvcr.io's registry API (`docker manifest inspect -v`), 2026-08-23. See `Dockerfile`'s comment. |
| `docker/car/pyproject.toml` + `uv.lock` resolve to real linux/aarch64 wheels for pyyaml/jsonschema | Yes -- `uv lock` run locally with `[tool.uv] environments` pinned to `linux`/`aarch64`; `uv.lock` contains only `manylinux*_aarch64` wheel URLs, no macOS/x86_64 entries. |
| The Dockerfile actually builds | **Yes** -- on the bench Jetson, 2026-09-21, `docker build -t car:local docker/car` with the default `INSTALL_TORCH=skip`. 10.2 GB on disk. It did NOT build first time: it hung forever on tzdata's interactive debconf prompt until `ARG DEBIAN_FRONTEND=noninteractive` was added. Still not a CI target (no arm64/L4T runner). |
| ROS 2 Humble installs cleanly via apt on L4T r36.4.0's Ubuntu 22.04 userspace | **Yes** -- 2026-09-21, same build. `ros2 pkg list` works out of the image and `ROS_DOMAIN_ID=42` / `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` are present in a login shell as intended. |
| The image can actually run `car_teleop.launch.py` | **Yes** -- 2026-09-21, unprivileged and non-root, both PWM channels driven and confirmed at the mux. See `docs/notes/first-boot-runbook.md`'s "Launch and drive". |
| A `colcon build` of `ros_ws` succeeds inside this image | **Yes** -- 2026-09-21, 6 packages in 68 s, after two fixes: `apt-get update` before `rosdep install` (the image deletes the apt lists), and `cmake -E env --unset=PYTHONPATH` around the vehicle_params codegen (this image's `ENV PYTHONPATH` was shadowing the `tools/` uv venv). |
| The `sllidar_ros2` layer builds and is found by `ros2` | **Partly** -- the layer's exact commands were replayed in the `ros-dev` image (Ubuntu 22.04, Humble, arm64) on 2026-10-05: it builds, `ros2 pkg prefix sllidar_ros2` resolves after sourcing `/opt/ros/humble` and an overlay workspace, and `ldd` finds every library. The car image itself has not been rebuilt with it, and the driver has never talked to a real C1. |
| The camera apt layer installs and the camera launch files work | **Partly** -- 2026-10-06, in the `ros-dev` image (Ubuntu 22.04, Humble, arm64) on the Mac: the same apt packages install (`usb_cam` 0.8.1, `gscam` 2.0.2, `image_transport` 3.1.13), `camera_csi.launch.py` with `videotestsrc` substituted for `nvarguscamerasrc` publishes `/camera/csi0/image_raw`, `/image_raw/compressed` and `/camera_info` only (no theora / compressedDepth), best_effort, `camera_check` PASSES at 60.00 Hz 1280x720, and a bag with `car_teleop.launch.py`'s regex holds the compressed topic and not the raw one. `camera_usb.launch.py` starts `usb_cam` with its parameters accepted and stops at the missing V4L2 device, as it must with no camera. The car image itself has not been rebuilt with the layer, Argus has never run in a container here, and neither camera has been plugged in. |
| The Jetson torch wheel installs and imports `torch` correctly | **No** -- and with the new default (`INSTALL_TORCH=skip`) it is not even attempted. Unchanged from before: no real `TORCH_WHEEL_URL` has ever been supplied. |
| The `INSTALL_TORCH` skip/required/invalid branches behave as documented | Yes, for the shell logic only -- the RUN step's script was extracted and executed in a plain `ubuntu:22.04` container on 2026-09-13: `skip` prints the notice, writes the marker and exports `RACER_TORCH=absent` in a login shell; `required` with no URL exits 1; an invalid value exits 1. That is the branch logic, NOT a build of this image. |
| `nvcr.io/nvidia/l4t-jetpack` has no tag matching the device's JetPack 6.2.1 | Yes -- registry tag list queried 2026-09-13, newest is `r36.4.0`. See "Base image tag vs. the device" above. |
| `PYTHONPATH` wiring actually makes `racer_policy` importable at `ros2 run` time | **No** -- pattern copied from a similar fix that WAS verified in milestone 3 (`docs/notes/milestone-3-sim-autopilot.md`, for `sim/bridge`'s gymnasium import), but never exercised against this image. What the 2026-09-21 build DID show is the other edge of it: that same `ENV PYTHONPATH` broke the `tools/` uv venv during `colcon build` until the CMake codegen steps started unsetting it. |

## CI

`.github/workflows/ci.yml` does **not** build this image (no L4T on GitHub-hosted runners --
see that file's own comment on the `docker-car-lint` job). The only CI coverage is a
Dockerfile lint (`hadolint`, static binary, no Docker build) -- see that job's comment for
exactly what it does and doesn't catch.

## Build (once real hardware exists -- see build_on_jetson.md for the full procedure)

```
# control-stack build (no torch, the default)
docker build -t car:local docker/car

# policy-phase build (roadmap 5.x)
docker build \
  --build-arg INSTALL_TORCH=required \
  --build-arg TORCH_WHEEL_URL=<the real JetPack-matched wheel URL, see build_on_jetson.md> \
  -t car:policy docker/car
```

Built and run on the Jetson 2026-09-21; roadmap task 1.4 ticked to `[x]` in
`claude-docs/01-roadmap.md` with a dated note that day.
