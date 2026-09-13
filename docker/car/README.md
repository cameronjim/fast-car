# car

On-vehicle image for the Jetson Orin Nano (roadmap task 1.4, `claude-docs/03-environments.md`).
**DRAFT: authored ahead of the hardware, never built or run anywhere.** See
`build_on_jetson.md` for the exact owner commands once the Jetson exists, and this file's
"What is and isn't verified" below before assuming any of it works.

Contains JetPack 6.1 (L4T r36.4.0, pinned by tag + digest), ROS 2 Humble installed from the
ROS apt repo, **optionally** the JetPack-matched NVIDIA torch wheel (build-arg, never a
guessed URL -- see `Dockerfile`), and `racer_policy`'s runtime Python deps via a `uv`-locked,
linux/aarch64 lockfile (`pyproject.toml` + `uv.lock`). Never contains dev tooling (test frameworks,
linters, `ros-humble-foxglove-bridge`) -- that is `docker/ros-dev/`'s job, not this image's.

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
| The Dockerfile actually builds (any step, on any architecture) | **No.** No arm64 Jetson and no L4T-compatible emulation exists in this repo's dev containers or in CI. `docker build` has never been run against this file. |
| ROS 2 Humble installs cleanly via apt on L4T r36.4.0's Ubuntu 22.04 userspace | **No**, not directly -- this is the standard, documented ROS 2 Debian-package install procedure for jammy, which L4T r36.x's userspace is, but it has not been executed against this specific base image. |
| The Jetson torch wheel installs and imports `torch` correctly | **No** -- and with the new default (`INSTALL_TORCH=skip`) it is not even attempted. Unchanged from before: no real `TORCH_WHEEL_URL` has ever been supplied. |
| The `INSTALL_TORCH` skip/required/invalid branches behave as documented | Yes, for the shell logic only -- the RUN step's script was extracted and executed in a plain `ubuntu:22.04` container on 2026-09-13: `skip` prints the notice, writes the marker and exports `RACER_TORCH=absent` in a login shell; `required` with no URL exits 1; an invalid value exits 1. That is the branch logic, NOT a build of this image. |
| `nvcr.io/nvidia/l4t-jetpack` has no tag matching the device's JetPack 6.2.1 | Yes -- registry tag list queried 2026-09-13, newest is `r36.4.0`. See "Base image tag vs. the device" above. |
| `PYTHONPATH` wiring actually makes `racer_policy` importable at `ros2 run` time | **No** -- pattern copied from a similar fix that WAS verified in milestone 3 (`docs/notes/milestone-3-sim-autopilot.md`, for `sim/bridge`'s gymnasium import), but never exercised against this image. |

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

Once this actually builds and runs on a Jetson, tick roadmap task 1.4 to `[x]` in
`claude-docs/01-roadmap.md` with a dated completion note (mirrors how `docker/train-cuda/`'s
`[~]` note is written) -- until then it stays `[~]`.
