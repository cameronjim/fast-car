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
#
# BASE TAG vs. THE DEVICE, decided 2026-09-13. The bench Jetson reports L4T R36.4.4 =
# JetPack 6.2.1 (docs/notes/build-log.md, 2026-09-12), while this line pins r36.4.0 =
# JetPack 6.1. The obvious fix would be to bump to a matching r36.4.x tag. There is no such
# tag: nvcr.io/nvidia/l4t-jetpack's full tag list (queried against the registry API on
# 2026-09-13) is r35.1.0, r35.2.1, r35.3.1, r35.4.1, r36.2.0, r36.3.0, r36.4.0 and nothing
# newer; l4t-base stops even earlier, at r36.2.0. NVIDIA has not published an l4t-jetpack
# image for JetPack 6.2/6.2.1.
#
# So the decision is to KEEP r36.4.0 at the same digest (re-resolved 2026-09-13 and
# unchanged: sha256:34ccf0f3...bb673), because:
#   * an older L4T container on a newer L4T host of the SAME major is the supported
#     direction for JetPack containers -- the driver comes from the host, not the image --
#     whereas the reverse (newer container, older host) is not;
#   * both are JetPack 6 / Ubuntu 22.04 jammy, which is what the ROS 2 Humble apt install
#     below depends on;
#   * pinning to a tag that does not exist, or floating the tag, would be worse than a
#     documented one-minor-version gap.
# This is a reasoned choice from registry data, NOT a tested one: whether CUDA/cuDNN/TensorRT
# out of a 6.1 container behave correctly on a 6.2.1 host is an ON-DEVICE check (see
# build_on_jetson.md step 3). If it misbehaves, the fallback is to install ROS on top of the
# device's own JetPack rather than to chase a tag that is not published. Re-check the tag
# list when phase 5 needs torch anyway.
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

# --- Jetson torch: OPTIONAL for the control-stack phase ---------------------------------
# NVIDIA publishes JetPack-version-matched torch wheels at
# https://developer.download.nvidia.com/compute/redist/jp/ (one subdirectory per JetPack
# minor version) and documents the current install command at
# https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html
# -- THAT page, not this comment, is the source of truth for the exact wheel filename/URL,
# because it changes with every JetPack/torch release and cannot be verified without the
# actual Jetson.
#
# WHY THIS IS NOW OPTIONAL (changed 2026-09-13, roadmap task 1.4 / milestone "car first
# boot"). Torch exists in this image for ONE consumer: racer_policy's on-vehicle inference,
# which is roadmap phase 5.x. The phase actually in front of us is the classical control
# stack -- safety_node, pwm_output_node, teleop -- which imports no torch at all. Refusing
# the whole build until someone hand-resolves a JetPack-matched torch wheel therefore blocked
# a first boot on a dependency that first boot does not use.
#
#   INSTALL_TORCH=skip      (DEFAULT) build without torch. Loud notice in the build log, an
#                           /etc/racer/torch-absent marker, and RACER_TORCH=absent exported
#                           into every login shell, so nothing can silently assume torch.
#   INSTALL_TORCH=required  install it, and REFUSE the build unless TORCH_WHEEL_URL is set to
#                           a real URL -- the original "never guess a wheel URL" behaviour,
#                           unchanged (CLAUDE.md invariant 3: refuse on mismatch, never a
#                           warning). This is what the policy phase builds with.
#
# Any other value is a hard error rather than a silent fallthrough. Switching the default
# back to `required` is a one-word change here when phase 5 starts.
ARG INSTALL_TORCH=skip
ARG TORCH_WHEEL_URL=""
# The single-quoted lines at the end of the RUN below are the TEXT of the /etc/profile.d
# script being generated; their `$RACER_TORCH` must reach that file UNEXPANDED, so
# shellcheck's SC2016 ("expressions don't expand in single quotes") is exactly the behaviour
# wanted here and is ignored for this one instruction only.
# hadolint ignore=SC2016
RUN set -eu; \
    mkdir -p /etc/racer; \
    case "$INSTALL_TORCH" in \
      required) \
        if [ -z "$TORCH_WHEEL_URL" ]; then \
            echo "ERROR: INSTALL_TORCH=required but --build-arg TORCH_WHEEL_URL=<url> was not supplied." >&2; \
            echo "Look up the JetPack-matched torch wheel URL per docker/car/build_on_jetson.md" >&2; \
            echo "and pass it explicitly. Refusing to guess a wheel URL for a mismatch this build" >&2; \
            echo "cannot verify (CLAUDE.md invariant 3: 'refuses on mismatch', never a warning)." >&2; \
            exit 1; \
        fi; \
        pip3 install --no-cache-dir "$TORCH_WHEEL_URL"; \
        echo present > /etc/racer/torch-status; \
        ;; \
      skip) \
        echo "################################################################" >&2; \
        echo "# NOTICE: BUILDING WITHOUT TORCH (INSTALL_TORCH=skip).          #" >&2; \
        echo "# This image CANNOT run racer_policy inference. It is the       #" >&2; \
        echo "# control-stack image: safety_node, pwm_output_node, teleop.    #" >&2; \
        echo "# Rebuild with --build-arg INSTALL_TORCH=required and a real    #" >&2; \
        echo "# TORCH_WHEEL_URL before roadmap phase 5.x.                     #" >&2; \
        echo "################################################################" >&2; \
        echo absent > /etc/racer/torch-status; \
        touch /etc/racer/torch-absent; \
        ;; \
      *) \
        echo "ERROR: INSTALL_TORCH must be 'required' or 'skip', got '$INSTALL_TORCH'." >&2; \
        exit 1; \
        ;; \
    esac; \
    { \
      echo '# Generated by docker/car/Dockerfile. Says whether this image has torch.'; \
      echo 'RACER_TORCH="$(cat /etc/racer/torch-status 2>/dev/null || echo unknown)"'; \
      echo 'export RACER_TORCH'; \
      echo 'if [ "$RACER_TORCH" = absent ]; then'; \
      echo '  echo "NOTICE: RACER_TORCH=absent -- this car image was built with INSTALL_TORCH=skip." >&2'; \
      echo '  echo "NOTICE: torch is NOT installed; racer_policy inference will not run here." >&2'; \
      echo 'fi'; \
    } > /etc/profile.d/racer-torch.sh; \
    chmod 0644 /etc/profile.d/racer-torch.sh

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

# --- DDS pinning (added 2026-09-14, deployment review) ----------------------------------
# Nothing in this repo pinned RMW_IMPLEMENTATION or ROS_DOMAIN_ID anywhere before this. On
# trackside WiFi, with the Mac ALSO running a ros-dev container for the same repo, Fast DDS's
# default multicast discovery on domain 0 is a well-known source of "nodes cannot see each
# other" or, worse, "the car sees the Mac's nodes and vice versa" -- two unrelated ROS graphs
# on the same domain, same subnet, both using the default RMW.
#
# RMW_IMPLEMENTATION=rmw_fastrtps_cpp: this is already Humble's default RMW, so this line
# changes no behavior today -- it exists so the default can never silently drift out from
# under this image if some future apt layer pulls in rmw_cyclonedds_cpp (see
# f1tenth-autonomous-racing/README.md's own note about needing to force cyclonedds on Foxy;
# pinning explicitly here means that class of surprise cannot recur without editing this
# line).
#
# ROS_DOMAIN_ID=42: ROS domain 0 is the multicast default every out-of-the-box ROS
# installation uses, which is exactly what a nearby Mac/desktop running ros-dev would also be
# on -- 42 takes the car off the crowded default domain. Chosen to also avoid colliding with
# the per-launch-test domain IDs (77-82) that ros_ws/src/*/test/*_launch.py already set via
# os.environ.setdefault() for CI test isolation: those are setdefault, so if this image ever
# ran that test suite, an ENV-set ROS_DOMAIN_ID would win and defeat that isolation -- which
# is exactly why this pin lives ONLY in the car image (the on-vehicle runtime) and
# deliberately NOT in docker/ros-dev (the image those tests actually run in). Document this
# value everywhere DDS is discussed: docs/notes/first-boot-runbook.md and
# docs/notes/jetson-compat-2026-09-14.md.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=42

# No CMD: this image is the toolchain a running container builds/launches ros_ws from (see
# build_on_jetson.md), the same shape as docker/ros-dev -- not a fixed entrypoint baking in
# one launch file, since which launch file to run is a per-session choice (teleop vs.
# autopilot vs. eventually the residual policy), not a build-time one.
