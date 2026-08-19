# car

On-vehicle image for the Jetson Orin Nano (roadmap task 1.4, `claude-docs/03-environments.md`).
**DRAFT: authored ahead of the hardware, never built or run anywhere.** See
`build_on_jetson.md` for the exact owner commands once the Jetson exists, and this file's
"What is and isn't verified" below before assuming any of it works.

Contains JetPack 6.1 (L4T r36.4.0, pinned by tag + digest), ROS 2 Humble installed from the
ROS apt repo, the JetPack-matched NVIDIA torch wheel (build-arg, not a guessed URL -- see
`Dockerfile`), and `racer_policy`'s runtime Python deps via a `uv`-locked, linux/aarch64
lockfile (`pyproject.toml` + `uv.lock`). Never contains dev tooling (test frameworks,
linters, `ros-humble-foxglove-bridge`) -- that is `docker/ros-dev/`'s job, not this image's.

## What is and isn't verified

| Claim | Verified how |
|---|---|
| `nvcr.io/nvidia/l4t-jetpack:r36.4.0`'s pinned digest is a real, currently-published arm64 manifest | Yes -- resolved from this Mac against nvcr.io's registry API (`docker manifest inspect -v`), 2026-08-23. See `Dockerfile`'s comment. |
| `docker/car/pyproject.toml` + `uv.lock` resolve to real linux/aarch64 wheels for pyyaml/jsonschema | Yes -- `uv lock` run locally with `[tool.uv] environments` pinned to `linux`/`aarch64`; `uv.lock` contains only `manylinux*_aarch64` wheel URLs, no macOS/x86_64 entries. |
| The Dockerfile actually builds (any step, on any architecture) | **No.** No arm64 Jetson and no L4T-compatible emulation exists in this repo's dev containers or in CI. `docker build` has never been run against this file. |
| ROS 2 Humble installs cleanly via apt on L4T r36.4.0's Ubuntu 22.04 userspace | **No**, not directly -- this is the standard, documented ROS 2 Debian-package install procedure for jammy, which L4T r36.x's userspace is, but it has not been executed against this specific base image. |
| The Jetson torch wheel installs and imports `torch` correctly | **No** -- and it cannot even be attempted without the owner supplying a real `TORCH_WHEEL_URL` (see `Dockerfile`); this build-arg has never been given a real value. |
| `PYTHONPATH` wiring actually makes `racer_policy` importable at `ros2 run` time | **No** -- pattern copied from a similar fix that WAS verified in milestone 3 (`docs/notes/milestone-3-sim-autopilot.md`, for `sim/bridge`'s gymnasium import), but never exercised against this image. |

## CI

`.github/workflows/ci.yml` does **not** build this image (no L4T on GitHub-hosted runners --
see that file's own comment on the `docker-car-lint` job). The only CI coverage is a
Dockerfile lint (`hadolint`, static binary, no Docker build) -- see that job's comment for
exactly what it does and doesn't catch.

## Build (once real hardware exists -- see build_on_jetson.md for the full procedure)

```
docker build \
  --build-arg TORCH_WHEEL_URL=<the real JetPack-6.1-matched wheel URL, see build_on_jetson.md> \
  -t car:local docker/car
```

Once this actually builds and runs on a Jetson, tick roadmap task 1.4 to `[x]` in
`claude-docs/01-roadmap.md` with a dated completion note (mirrors how `docker/train-cuda/`'s
`[~]` note is written) -- until then it stays `[~]`.
