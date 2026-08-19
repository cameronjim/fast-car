# car image (claude-docs/03-environments.md, roadmap task 1.4) -- DRAFT, roadmap 1.4 is
# marked [~] not [x]. See docker/car/README.md and build_on_jetson.md: this image builds ON
# the Jetson Orin Nano (or via Docker's linux/arm64 emulation on a desktop, which is not the
# same as a real Jetson build and proves nothing about JetPack/CUDA/torch), so nothing here
# has been built or run anywhere. Never assume interchangeable with sim-cpu/train-cuda/
# ros-dev (claude-docs/03-environments.md: "One image cannot honestly serve CUDA, JetPack,
# ROS, and Apple Silicon at once").
#
# --- Base image: JetPack 6.1 (L4T r36.4.0), pinned by tag + digest ----------------------
# nvcr.io/nvidia/l4t-jetpack (not the smaller l4t-base) deliberately: it ships CUDA, cuDNN,
# TensorRT, and VPI already installed and matched to this exact L4T release, which is what
# the Jetson torch wheel below is built against -- l4t-base ships only the L4T OS + CUDA
# runtime and would need cuDNN/TensorRT installed by hand, version-matched by hand, a much
# larger and more fragile undertaking for a first draft. The tradeoff is a bigger image;
# claude-docs/03-environments.md's "no dev tooling bloat" is about developer tooling
# (compilers, test frameworks, IDEs), not the on-device inference stack this image exists to
# run.
#
# r36.4.0 = JetPack 6.1 (JetPack 6 -> ROS Humble is the pinned pairing, claude-docs/03-
# environments.md). Digest resolved and recorded here 2026-08-23 via `docker manifest
# inspect -v nvcr.io/nvidia/l4t-jetpack:r36.4.0` (arm64/linux platform manifest) --
# reproducible from this file alone even if the `r36.4.0` tag is later moved, same pinning
# discipline as every other Dockerfile in docker/*/. This was resolved from this Mac's
# network access to nvcr.io's public registry API; it was NOT pulled, built, or run here (no
# arm64 hardware/emulation available) -- see README.md's "What is and isn't verified".
FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0@sha256:34ccf0f3b63c6da9eee45f2e79de9bf7fdf3beda9abfd72bbf285ae9d40bb673

# --- ROS 2 Humble (claude-docs/03-environments.md: "JetPack 6 -> Humble") ---------------
# L4T r36.4.0 is Ubuntu 22.04 (jammy) userspace, the same distro ros-dev's `ros:humble-
# ros-base` is built on -- so the standard ROS 2 apt-repo install (not a source build) works
# here too. This installs ros-base + the packages racer_* nodes need at RUNTIME plus the
# minimum to BUILD them on-device (see "Building the workspace" in build_on_jetson.md): this
# image is the toolchain, not a pre-baked ros_ws, exactly like docker/ros-dev/ -- there is no
# arm64 cross-compilation setup in this repo, so compiling racer_safety/racer_control/etc.
# for the Jetson happens ON the Jetson, inside a container from this image, the same way
# ros-dev is used for amd64/dev-machine builds.
#
# Deliberately NOT installed (claude-docs/03-environments.md "car: ... Never contains: dev
# tooling bloat", contrast docker/ros-dev/Dockerfile): ros-humble-launch-testing[-ament-cmake],
# ros-humble-ament-cmake-gtest/-pytest, clang-format, clang-tidy, gcovr, ros-humble-foxglove-
# bridge. Those are test/lint/dev-visualization tooling with no on-vehicle runtime role.
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales \
        curl \
        gnupg2 \
        lsb-release \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-humble-ackermann-msgs \
        build-essential \
        cmake \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# --- Jetson torch (JetPack-specific NVIDIA wheel, NOT on PyPI) --------------------------
# NVIDIA publishes JetPack-version-matched torch wheels at
# https://developer.download.nvidia.com/compute/redist/jp/ (one subdirectory per JetPack
# minor version, e.g. v61/ for JetPack 6.1) and documents the current install command at
# https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html
# -- THAT page, not this comment, is the source of truth for the exact wheel filename/URL,
# because it changes with every JetPack/torch release and cannot be verified without the
# actual Jetson (no arm64 hardware or L4T emulation is available in this repo's dev
# containers or in CI). Rather than hardcode a URL this milestone could not verify resolves
# to a real, JetPack-6.1-compatible wheel, the build REFUSES until the owner supplies one
# explicitly -- the same "refuse on mismatch, don't guess" discipline CLAUDE.md invariant 3
# applies to the deploy node applies here to the build itself. See build_on_jetson.md for the
# exact lookup + build command.
ARG TORCH_WHEEL_URL=REPLACE_ME_SEE_BUILD_ON_JETSON_MD
RUN set -eu; \
    if [ "$TORCH_WHEEL_URL" = "REPLACE_ME_SEE_BUILD_ON_JETSON_MD" ]; then \
        echo "ERROR: --build-arg TORCH_WHEEL_URL=<url> was not supplied." >&2; \
        echo "Look up the JetPack-6.1-matched torch wheel URL per docker/car/build_on_jetson.md" >&2; \
        echo "and pass it explicitly. Refusing to guess a wheel URL for a mismatch this build" >&2; \
        echo "cannot verify (CLAUDE.md invariant 3: 'refuses on mismatch', never a warning)." >&2; \
        exit 1; \
    fi; \
    pip3 install --no-cache-dir "$TORCH_WHEEL_URL"

# --- Racer runtime Python deps (uv lockfile, linux/aarch64) -----------------------------
# Pin the uv binary itself, matching every other image (docker/sim-cpu, docker/train-cuda,
# docker/ros-dev).
COPY --from=ghcr.io/astral-sh/uv:0.12.5 /uv /uvx /usr/local/bin/

WORKDIR /car-runtime
COPY pyproject.toml uv.lock ./
# --locked: fail the build instead of silently re-resolving if uv.lock drifted from
# pyproject.toml. --no-install-project: pyproject.toml here is a dependency manifest only
# (tool.uv.package = false). --no-python-downloads: use the system python3.10 that ROS
# Humble/jammy already provides (matches docker/ros-dev's reasoning) rather than fetching a
# separate interpreter uv would otherwise manage.
RUN uv sync --locked --no-install-project --no-python-downloads

# PYTHONPATH, not prepending the venv to PATH: `ros2 run`/`ros2 launch` invoke the SYSTEM
# python3 (the one colcon-built racer_policy is installed under), and prepending
# /car-runtime/.venv/bin to PATH would shadow that python3 with one that has colcon's own
# dependencies (catkin_pkg etc.) missing -- exactly the failure docker/ros-dev/Dockerfile's
# own comment documents and works around the same way, and the same PYTHONPATH-not-PATH fix
# already used to get bridge_node importing gymnasium correctly
# (docs/notes/milestone-3-sim-autopilot.md, "Standards pre-check"). Setting PYTHONPATH
# instead makes pyyaml/jsonschema importable to the SAME system python3 every ROS-launched
# racer_policy process actually runs under, without touching which `python3` resolves.
ENV PYTHONPATH="/car-runtime/.venv/lib/python3.10/site-packages:${PYTHONPATH}"

# No CMD: this image is the toolchain a running container builds/launches ros_ws from (see
# build_on_jetson.md), the same shape as docker/ros-dev -- not a fixed entrypoint baking in
# one launch file, since which launch file to run is a per-session choice (teleop vs.
# autopilot vs. eventually the residual policy), not a build-time one.
