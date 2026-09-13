# Building and running `car` on the Jetson Orin Nano

Revised 2026-09-13 for the Jetson that now exists on the bench. Still **NEVER EXECUTED**: the
board was powered off when this was written, so every command below is a written-ahead
procedure. Correct it against reality the first time it is run, and note what changed in
`docs/notes/build-log.md` and `claude-docs/01-roadmap.md`'s task 1.4 entry.

The runbook that puts these steps in order with the safety checks around them is
`docs/notes/first-boot-runbook.md`. This file is the reference for the Docker part alone.

## What the device actually is (observed 2026-09-12, `docs/notes/build-log.md`)

| Fact | Value | Consequence here |
|---|---|---|
| Hostname / address | `racer-car`, 10.0.0.226 | `ssh racer@racer-car` |
| User | `racer`, **in the `docker` group**, **passwordless sudo** | no `sudo` needed in front of `docker`; `sudo` works unprompted for the pinmux steps |
| JetPack / L4T | JetPack 6.2.1, **L4T R36.4.4** | one minor ahead of this image's r36.4.0 base -- see the Dockerfile's "BASE TAG vs. THE DEVICE" comment for why that is deliberate and what to check |
| Docker | installed, **NVIDIA runtime present** | `--runtime nvidia` works; confirm with `docker info \| grep -i runtime` |
| This repo | **not cloned on the device yet** | step 1 |
| Already running | `racer-heartbeat.service` (`tools/jetson_heartbeat/`, physical pin 7) | leave it alone; it is the mux's watchdog input |

The old draft of this file assumed JetPack 6.1, that Docker might need installing, and that
`sudo` would prompt. All three were wrong; corrected above.

## 1. Clone the repo on the device

```sh
ssh racer@racer-car
git clone https://github.com/cameronjim/fast-car.git car
cd car
```

## 2. Build the image WITHOUT torch (the control-stack build)

Torch is optional as of 2026-09-13 (`Dockerfile`, `INSTALL_TORCH`). The control stack --
`safety_node`, `pwm_output_node`, teleop -- imports no torch, and the policy phase that does
is roadmap 5.x. Default is `skip`, so:

```sh
docker build -t car:local docker/car
```

Expect a loud `NOTICE: BUILDING WITHOUT TORCH` block in the build log. The image records this
in `/etc/racer/torch-status` and exports `RACER_TORCH=absent` into every login shell, so
nothing downstream can quietly assume torch is there.

Expect the build to fail the first few times regardless -- it has never been run. Likely
early failures: apt package name drift in the ROS 2 apt repo, and the r36.4.0 base's CUDA
stack behaving oddly on an R36.4.4 host (see step 4).

### When phase 5 needs torch

```sh
docker build \
  --build-arg INSTALL_TORCH=required \
  --build-arg TORCH_WHEEL_URL=<the JetPack-matched wheel URL> \
  -t car:policy docker/car
```

`INSTALL_TORCH=required` with no `TORCH_WHEEL_URL` is a hard build failure, on purpose: this
build cannot verify a guessed wheel URL, so it refuses instead of guessing (`CLAUDE.md`
invariant 3). Find the URL from NVIDIA's install page
(https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)
and the `jp/` redistributable tree
(https://developer.download.nvidia.com/compute/redist/jp/), matching the DEVICE's JetPack
(6.2.1), and write the exact URL into this file once it is confirmed.

## 3. Sanity-check the built image

```sh
docker run --rm car:local bash -lc 'echo "RACER_TORCH=$RACER_TORCH"; python3 -c "import yaml, jsonschema; print(\"pyyaml/jsonschema OK\")"'
docker run --rm car:local bash -lc 'source /opt/ros/humble/setup.bash && ros2 pkg list | head'
```

`RACER_TORCH=absent` and the two NOTICE lines are the expected output of the first command
for a `skip` build.

## 4. Check the JetPack-version gap on the device

The image's base is JetPack 6.1 (r36.4.0), the host is JetPack 6.2.1 (R36.4.4), because
NVIDIA publishes no `l4t-jetpack` image newer than r36.4.0 (registry tag list checked
2026-09-13). An older container on a newer host of the same major is the supported direction,
but it is unverified here:

```sh
cat /etc/nv_tegra_release                       # host: expect R36 ... 4.4
docker run --rm --runtime nvidia car:local bash -lc 'ls /usr/local/ | grep -i cuda'
```

The control stack needs no CUDA at all, so a wrinkle here does not block first boot -- it
blocks phase 5. If CUDA misbehaves out of this container on this host, the fallback is
installing ROS on top of the device's own JetPack rather than chasing an unpublished tag.

## 5. Build the workspace on-device

This image is the toolchain, not a pre-baked `ros_ws` (same shape as `docker/ros-dev/`).
There is no arm64 cross-compilation setup in this repo, so the workspace is built on the
Jetson, inside a container from this image, with the repo bind-mounted:

```sh
docker run --rm -it --runtime nvidia \
  -v "$PWD":/workspace -w /workspace \
  car:local bash -lc '
    source /opt/ros/humble/setup.bash
    rosdep update
    rosdep install --from-paths ros_ws/src --ignore-src -r -y
    cd ros_ws && colcon build --symlink-install
  '
```

`racer_safety` and `racer_drivers` both regenerate the `vehicle_params` C++ binding during
the build via `uv run --project tools`, which needs network access on its first run to sync
`tools/`'s venv from its lockfile.

## 6. Run the car launch file

The PWM pins must already be enabled and their `pwmchip` numbers known -- see
`ros_ws/src/racer_drivers/README.md`, "Enabling PWM pins on the Jetson".

```sh
docker run --rm -it --runtime nvidia \
  --network host \
  --privileged \
  -v /sys:/sys \
  -v "$PWD":/workspace -w /workspace/ros_ws \
  car:local bash -lc '
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch racer_bringup car_teleop.launch.py \
      viz:=false \
      steering_pwmchip:=N steering_pwm_channel:=M \
      throttle_pwmchip:=P throttle_pwm_channel:=Q
  '
```

**Why `--privileged` / `-v /sys:/sys`, and why that is flagged.** Docker mounts `/sys`
read-only by default, and `pwm_output_node` writes to `/sys/class/pwm/...`. Bind-mounting
just `/sys/class/pwm` does not help: those entries are symlinks into `/sys/devices`, which
would still be the container's own read-only copy. `--privileged` with a writable `/sys` is
the blunt instrument that certainly works; whether a narrower option (a udev rule granting a
group write access to the exported channel's attributes, plus `--group-add`) is enough is
UNVERIFIED and worth trying once the blunt version is proven, since `--privileged` on the
node that drives the actuators is not a resting place.

`--network host` is for DDS discovery and for Foxglove if `viz:=true` (port 8765).

`--device` entries for the VESC USB link, LiDAR and ingest board are deliberately absent:
none of them is on the command path for first boot (the VESC is commanded by PWM through the
mux, `docs/notes/build-log.md` 2026-09-12), and they get added as
`claude-docs/11-hardware.md`'s wiring is actually done.

## Once this all actually works

Tick roadmap task 1.4 to `[x]` in `claude-docs/01-roadmap.md` with a dated note, and correct
every "DRAFT"/"never executed" claim in this file and in `README.md` -- an honest doc that
says "this hasn't been tried" stops being honest the moment it has been.
