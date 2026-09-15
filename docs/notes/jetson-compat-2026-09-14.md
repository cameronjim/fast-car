# Jetson compatibility checklist (2026-09-14, deployment/CI/repo-hygiene review)

Written from the repo alone, on a Mac, with the bench Jetson (`racer-car`, 10.0.0.226,
JetPack 6.2.1 / L4T R36.4.4) powered off. Nothing below was run on the device. Where a
finding needs an on-device check, that check is named -- run it and correct this file the
same day, per `claude-docs/10-conventions.md` ("a measurement that lives only in a terminal
scrollback did not happen").

## Python version

| Where | Version |
|---|---|
| `docker/car`'s `uv sync` (`docker/car/pyproject.toml`: `requires-python = ">=3.10"`) | 3.10, the system interpreter `ros:humble`/L4T jammy ships |
| Jetson host (JetPack 6.2.1 = Ubuntu 22.04 jammy userspace) | 3.10 (jammy's default `python3`) |
| `docker/ros-dev` (`ros:humble-ros-base`, also jammy) | 3.10 |

**Match.** All three are jammy's system Python 3.10; `docker/car` deliberately uses
`--no-python-downloads` (see that Dockerfile's comment) so `uv` never fetches a different
interpreter that could drift from this. On-device check: `python3 --version` on `racer-car`
should print `3.10.x`; if JetPack ever moves to a 24.04-based release this whole row and the
`PYTHONPATH` site-packages path (`.../python3.10/site-packages`) both need revisiting.

## ROS Humble apt repo architecture (arm64)

`docker/car/Dockerfile` adds `packages.ros.org/ros2/ubuntu jammy main` and installs with
`arch=$(dpkg --print-architecture)` substituted at build time -- on the real Jetson build
that resolves to `arm64`, and ROS's Debian-package Humble repo does publish `arm64` binaries
for jammy (this is the same standard install path used on real Jetsons throughout the ROS
community; not specific to this repo). No repo-side gap here. On-device check: after step 3
of `docker/car/build_on_jetson.md`, `dpkg --print-architecture` inside the built image should
print `arm64`, and `apt-cache policy ros-humble-ros-base` should show a candidate from the
`ubuntu jammy/main arm64` source, not `all`-only or missing.

## Every apt package in `docker/car` for arm64 jammy

| Package | arm64 jammy? |
|---|---|
| `locales`, `curl`, `gnupg2`, `lsb-release` | Yes -- Ubuntu jammy main, all architectures |
| `ros-humble-ros-base` | Yes -- ROS Humble apt repo publishes arm64 (see above) |
| `ros-humble-ackermann-msgs` | Yes -- pure-message package, built for every ROS-supported arch including arm64 |
| `build-essential`, `cmake`, `git` | Yes -- Ubuntu jammy main |
| `python3-colcon-common-extensions`, `python3-rosdep`, `python3-vcstool` | Yes -- these are architecture-independent (`Architecture: all`) Python packages |

No package in `docker/car`'s apt list is amd64-only or otherwise arch-restricted. This was
checked by reasoning about package architecture classes (compiled ROS message packages exist
per-arch including arm64; the rest are either jammy-main or `Architecture: all`), not by
running `apt-get install` on arm64 hardware or emulation -- see `docker/car/README.md`'s "What
is and isn't verified" table, which already says the Dockerfile has never actually been
built. On-device check: `docker build -t car:local docker/car` (build_on_jetson.md step 2)
either succeeds through the apt layers or names the missing package explicitly; that is the
real answer, this table is the reasoned prediction.

## `libgpiod` version: heartbeat build vs. JetPack

`tools/jetson_heartbeat` builds against `libgpiod`'s **v1.x API**
(`gpiod_chip_open_by_name`, `gpiod_line_request_output`, `gpiod_line_set_value` --
`src/main.c`) via `pkg-config --cflags --libs libgpiod`. `docs/notes/build-log.md`'s
2026-09-12 entry records it was built and verified on the bench Jetson against
`libgpiod-dev` **1.6.3**, which is jammy's packaged version. JetPack 6.2.1's userspace is that
same Ubuntu 22.04 jammy, so the host and the already-installed binary agree by construction --
this is not a prediction, it is the recorded build-log result of the actual bench install.
The only residual risk is future drift: if a `apt upgrade` on the Jetson ever pulls jammy's
`libgpiod-dev` past a version where `gpiod_chip_open_by_name`'s v1 API is removed (Ubuntu has
no plan to do this inside jammy's lifecycle; that would be a `noble`/24.04-class change), the
heartbeat would fail to build, loudly, at compile time -- not silently at runtime. No action
needed now; re-check `libgpiod-dev --version` if the Jetson is ever moved off jammy.

## `ros-dev` vs. `car`: do they agree on ROS package versions?

Both images install from `packages.ros.org/ros2/ubuntu jammy main` with **no per-package apt
version pin** in either Dockerfile (both Dockerfiles document this as a deliberate, accepted
gap -- see `docker/ros-dev/Dockerfile`'s "Honesty about what this does and doesn't pin"
comment, which `docker/car/Dockerfile` references). That means: two builds run on the same
day resolve to the same ROS Humble package versions (both are jammy, same apt repo, same
`main` component); two builds run months apart do not necessarily agree, because Humble's apt
repo keeps shipping patch updates for the lifetime of the distro. This is a real, already-
documented-elsewhere gap, not a new one -- it is the same tradeoff both images already made
for their own apt package sets, applied consistently across both. The mitigation already in
place is CI building `ros-dev` fresh on every relevant push (so `ros-dev:ci` is always
current) rather than caching a stale image indefinitely; `car` has no CI build at all (see
`docker/car/README.md`'s "CI" section), so its apt-resolved versions are only as fresh as the
day someone last ran `docker build` on the Jetson. No fix proposed here beyond naming it: full
apt pinning was already rejected repo-wide as impractical to maintain by hand (see both
Dockerfiles), and re-litigating that is out of this review's scope.

## L4T base tag: r36.4.0 image vs. R36.4.4 host

Already decided and documented at length in `docker/car/Dockerfile`'s "BASE TAG vs. THE
DEVICE" comment and `docker/car/README.md`'s "Base image tag vs. the device" section
(2026-09-13): `nvcr.io/nvidia/l4t-jetpack` publishes nothing newer than `r36.4.0`
(JetPack 6.1), the bench Jetson reports `R36.4.4` (JetPack 6.2.1), and an older L4T container
on a newer L4T host of the same major is the supported direction because the driver comes
from the host, not the image. This review re-confirms that reasoning rather than re-deciding
it. **The one on-device check that would confirm it actually behaves**, already named in
`docker/car/build_on_jetson.md` step 4 and repeated here so it is not missed:

```sh
cat /etc/nv_tegra_release                       # host: expect R36 ... 4.4
docker run --rm --runtime nvidia car:local bash -lc 'ls /usr/local/ | grep -i cuda'
```

If CUDA tooling from the r36.4.0 container misbehaves against the R36.4.4 host driver, the
documented fallback is installing ROS on top of the device's own JetPack rather than chasing
an unpublished tag -- and since the control stack (`safety_node`, `pwm_output_node`, teleop)
imports no CUDA at all, a wrinkle here gates roadmap phase 5 (torch/`racer_policy`), not first
boot.

## Summary

| Item | Status |
|---|---|
| Python 3.10 car image vs. Jetson host | Match (both jammy system Python) |
| ROS Humble apt arch | arm64 published; standard install path |
| Every `docker/car` apt package on arm64 jammy | All either jammy-main or `Architecture: all`; no restricted package found |
| `libgpiod` heartbeat build vs. JetPack | Match, already verified on bench Jetson (1.6.3) per build-log |
| `ros-dev` vs. `car` ROS package versions | Agree if built same-day; documented, accepted apt-pinning gap, not new |
| L4T base tag r36.4.0 vs. host R36.4.4 | Deliberate, documented, one-minor gap; on-device CUDA check named above, not yet run |

Nothing here found a NEW blocking incompatibility. The one open item that actually needs a
human at the device is the L4T/CUDA on-device check above; everything else is either already
matched by construction (same jammy base) or an already-documented, already-accepted gap.
