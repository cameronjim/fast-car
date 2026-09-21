# Building and running `car` on the Jetson Orin Nano

Revised 2026-09-13, then **EXECUTED FOR REAL on 2026-09-21** and corrected against what
actually happened (`docs/notes/build-log.md`, 2026-09-21 evening). The build works; it did not
work first time. Two Dockerfile fixes and one correction to step 5's command came out of that
session and are folded in below.

The run procedure that has actually been used -- device access, the exact `docker run`, the
pre-drive checklist -- is `docs/notes/first-boot-runbook.md`'s "Launch and drive" section, not
step 6 here. Step 6 below is kept only as the Docker-flag reference and now points there.

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
cd ~/car
nohup docker build -t car:local docker/car > ~/car-build.log 2>&1 &
```

Run it under `nohup` (or `tmux`): a dropped SSH session otherwise kills the build. Poll
`~/car-build.log`.

Expect a loud `NOTICE: BUILDING WITHOUT TORCH` block in the build log. The image records this
in `/etc/racer/torch-status` and exports `RACER_TORCH=absent` into every login shell, so
nothing downstream can quietly assume torch is there.

**Timing, measured 2026-09-21.** Pulling and extracting the 3.4 GB `l4t-jetpack:r36.4.0` base
is the long pole at roughly 25 minutes on this device's eMMC. With the base cached, the rest
is about 5 minutes (the ROS apt layer is 165 s of it). Result: **10.2 GB on disk**. A cold
first build is therefore 30 minutes, not the "expect it to fail for an hour" the old draft
implied -- but do check the log rather than assuming, because a build that has stopped making
progress may be waiting on something (see below).

**The two failures this actually hit, both now fixed in the Dockerfile:** it hung indefinitely
on `tzdata`'s interactive `Geographic area:` debconf prompt (fixed with `ARG
DEBIAN_FRONTEND=noninteractive`), and the image was missing `ros-humble-foxglove-bridge`, which
`car_teleop.launch.py` starts by default. If a future build appears stuck, look for a debconf
prompt in the log before assuming it is slow.

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
docker run --rm \
  -v "$PWD":/workspace -w /workspace \
  car:local bash -lc '
    source /opt/ros/humble/setup.bash
    apt-get update
    rosdep install --from-paths ros_ws/src --ignore-src -r -y
    cd ros_ws && colcon build --symlink-install
    chown -R 1000:1000 build install log /workspace/tools/.venv
  '
```

VERIFIED 2026-09-21: 6 packages in 68 s. Two corrections to the old draft, both found by
running it:

- **`apt-get update` is required**, and was missing. This image ends its apt layers with
  `rm -rf /var/lib/apt/lists/*`, so `rosdep install`'s own `apt-get install` cannot resolve
  anything: it failed with `E: Unable to locate package python3-jsonschema`. (`rosdep update`
  alone does not help -- that refreshes rosdep's rules, not apt's package lists.)
- **`chown` back to your UID at the end.** This container has to run as root, because
  `rosdep install` installs system packages; without the `chown` it leaves root-owned
  `build/`, `install/`, `log/` and `tools/.venv` directories in the repo that you then cannot
  clean without `sudo`.

`--runtime nvidia` is not needed for this and has been dropped: nothing in the workspace uses
CUDA.

`racer_safety`, `racer_drivers` and `racer_control` all regenerate the `vehicle_params` C++
binding during the build via `uv run --project tools`, which needs network access on its first
run to sync `tools/`'s venv from its lockfile. That step now runs under
`cmake -E env --unset=PYTHONPATH`: this image's own `ENV PYTHONPATH` was otherwise inherited by
the `tools/` venv's CPython 3.14 and shadowed its site-packages with python3.10 ones, failing
with `ModuleNotFoundError: No module named 'rpds.rpds'`.

## 6. Run the car launch file

**The procedure that has actually been executed lives in
`docs/notes/first-boot-runbook.md`'s "Launch and drive" section** (device access, the exact
`docker run`, the pre-drive checklist, how to stop it, and what the mux should read at each
step). Use that. What follows is only the Docker-flag rationale, corrected 2026-09-21.

The PWM pins must already be enabled and their `pwmchip` numbers known -- see
`ros_ws/src/racer_drivers/README.md`, "Enabling PWM pins on the Jetson". On this device the
defaults are already right (pin 15 = `pwmchip0` = steering, pin 33 = `pwmchip2` = throttle),
so the four chip/channel arguments the old draft passed explicitly are no longer needed.

```sh
docker run --rm -it --name car-stack --network host \
  --user "$(id -u):$(id -g)" \
  --group-add "$(getent group gpio | cut -d: -f3)" \
  -e HOME=/tmp \
  -v /sys/devices/platform/bus@0/3280000.pwm:/sys/devices/platform/bus@0/3280000.pwm \
  -v /sys/devices/platform/bus@0/32c0000.pwm:/sys/devices/platform/bus@0/32c0000.pwm \
  -v "$PWD":/workspace -w /workspace/ros_ws \
  car:local bash -lc '
    source /opt/ros/humble/setup.bash && source install/setup.bash
    exec ros2 launch racer_bringup car_teleop.launch.py browser_teleop:=true'
```

**`--privileged` is NOT needed. RESOLVED 2026-09-21, this was the open question.** Docker does
mount `/sys` read-only, and bind-mounting `/sys/class/pwm` genuinely does not help (those
entries are symlinks into `/sys/devices`). But the narrow option the old draft called
UNVERIFIED turned out to need no new host configuration at all: the Jetson already ships
`/lib/udev/rules.d/60-jetson-gpio-common.rules`, which chgrps `pwmchipN/{export,unexport}` and
each exported channel's `period`/`duty_cycle`/`enable` to the stock **`gpio`** group, and
`racer` is already in it. So `--group-add` that GID, plus read-write bind-mounts of the two
real device-tree paths the symlinks resolve to, is sufficient -- verified by driving both
channels for real from a container that is both unprivileged and non-root. No custom udev
rule, no `racer-pwm` group (one was created during that session, found redundant, and
removed).

`--runtime nvidia` has been dropped: nothing in the control stack uses CUDA.

`exec` before `ros2 launch` matters: without it `bash` is PID 1 and swallows the signal, so
the nodes never get a clean shutdown. Even with it, prefer Ctrl-C over `docker stop` -- see the
runbook's "Stopping" table for the measured difference.

`--network host` is for DDS discovery and for the Foxglove bridge (port 8765, `viz` defaults
to true).

`--device` entries for the VESC USB link, LiDAR and ingest board are deliberately absent:
none of them is on the command path for first boot (the VESC is commanded by PWM through the
mux, `docs/notes/build-log.md` 2026-09-12), and they get added as
`claude-docs/11-hardware.md`'s wiring is actually done. The safety-mux Pico's `/dev/ttyACM0`
is deliberately absent too: nothing in the ROS graph reads it, the diagnostic stream is read
from the HOST, and only one reader at a time can have it.

## Done, 2026-09-21

Roadmap task 1.4 ticked to `[x]` in `claude-docs/01-roadmap.md` with a dated note, and the
"DRAFT"/"never executed" claims in this file and in `README.md` corrected -- an honest doc that
says "this hasn't been tried" stops being honest the moment it has been. What remains untried
is listed at the end of `docs/notes/build-log.md`'s 2026-09-21 evening entry: no servo has
moved under ROS command, no ESC has been armed by this stack, and roadmap 1.3's kill test
against a genuinely frozen Jetson is still open.
