# ros-dev image (claude-docs/03-environments.md, roadmap task 0.4)
#
# Runs on Mac (arm64, via Docker Desktop's Linux VM) and amd64 desktops.
# Contains: ROS 2 Humble, colcon, rosdep, clang-format/clang-tidy (for the
# racer_safety/racer_control C++ lint gate, claude-docs/10-conventions.md),
# and a pinned uv-managed Python venv for standalone test tooling. It is
# the TOOLCHAIN the racer ROS packages build and test against, not a
# baked-in build of them: ros_ws/src has no packages yet (they land with
# later roadmap tasks), so this image's job is to prove a throwaway ROS 2
# package can be built and tested with `colcon build && colcon test`
# (see smoke_test.sh) and to be the container CI/dev builds ros_ws in.
#
# NEVER add CUDA to this image (claude-docs/03-environments.md: ros-dev
# never contains CUDA). Training happens in train-cuda; this image never
# imports torch.
#
# --- Base image pin -------------------------------------------------------
# FROM ros:humble-ros-base, pinned by digest rather than tag. This is a
# manifest LIST digest (covers both linux/amd64 and linux/arm64v8), so
# `docker buildx build` on either architecture resolves it to the correct
# per-platform image while still being the exact same upstream artifact
# every time -- the multi-arch equivalent of the sim-cpu/train-cuda
# base-image pin.
#
# Honesty about what this does and doesn't pin: unlike sim-cpu/train-cuda,
# apt packages in this image (colcon, rosdep, clang-format/clang-tidy, the
# ROS launch_testing packages) are NOT individually version-pinned --
# apt-get always installs whatever is currently newest in the Ubuntu
# 22.04 (jammy) and ROS Humble apt repos at build time, and full apt
# pinning (exact package=version for every package incl. transitive
# dependencies, snapshotted indefinitely) is impractical to maintain by
# hand here. The base image digest is the practical reproducibility
# anchor: it fixes the OS, the ROS Humble core packages, and glibc: apt
# drift on top of that is a real but bounded gap, consistent with how most
# ROS Docker workflows actually work. The Python side of this image (uv +
# pyproject.toml + uv.lock, same as sim-cpu/train-cuda) IS fully pinned
# and hashed; that combination -- digest-pinned base + hash-locked Python
# -- is this image's reproducibility story.
FROM ros:humble-ros-base@sha256:75dd3aba34a3838dadbb31a9f7bef769bdfa8713e6cec686fc868db2981b0987

# build-essential/cmake: compiling ament_cmake (C++) packages.
# git: colcon workspaces routinely vendor deps via vcs/git, and it's needed
#   for `uv sync` if any future ros-dev Python dependency is git-sourced.
# python3-colcon-common-extensions: `colcon build`/`colcon test` plus the
#   common extensions (test-result, mixin, etc).
# python3-rosdep: resolves ROS package system dependencies once ros_ws has
#   real packages with package.xml files (none yet -- see comment above).
# python3-vcstool: standard ROS workspace tool for multi-repo checkouts.
# clang-format/clang-tidy: the C++ lint/static-analysis gate
#   claude-docs/10-conventions.md requires in CI for racer_safety and
#   racer_control.
# ros-humble-launch-testing / ros-humble-launch-testing-ament-cmake: L3
#   node tests (claude-docs/12-testing.md) are launch_testing-based; these
#   are ROS packages that depend on rclpy/launch, so they come from the
#   ROS apt repo, not pip.
# ros-humble-ament-cmake-gtest / ros-humble-ament-cmake-pytest: wires gtest
#   (C++) and pytest (Python) into `colcon test` for ament_cmake packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        clang-format \
        clang-tidy \
        ros-humble-launch-testing \
        ros-humble-launch-testing-ament-cmake \
        ros-humble-ament-cmake-gtest \
        ros-humble-ament-cmake-pytest \
    && rm -rf /var/lib/apt/lists/*

# ros:humble-ros-base already ships with rosdep initialized (it runs
# `rosdep init` as part of its own image build, leaving
# /etc/ros/rosdep/sources.list.d/20-default.list in place) -- running
# `rosdep init` again here fails with "default sources list file already
# exists". Only `rosdep update` is ours to run: it fetches the rosdistro
# index into this build layer so a later `rosdep install` (once
# ros_ws/src has package.xml files with real dependencies) doesn't need
# network access at dev/CI time. This is the one network-dependent step
# in this build; it fetches from raw.githubusercontent.com and is
# expected to work in GitHub Actions.
RUN rosdep update

# Pin the uv binary itself to the same version used to generate uv.lock
# locally (matches sim-cpu/train-cuda), so `uv sync --locked` resolves the
# lockfile the same way here as it did during development.
COPY --from=ghcr.io/astral-sh/uv:0.12.5 /uv /uvx /usr/local/bin/

WORKDIR /ros-dev

COPY pyproject.toml uv.lock ./

# --locked: fail the build instead of silently re-resolving if uv.lock is
# out of date relative to pyproject.toml. --no-install-project:
# pyproject.toml here is a dependency manifest only (tool.uv.package =
# false), there is no ros-dev package to install. This installs into
# /ros-dev/.venv using the system python3.10 that ships with the
# jammy-based ros:humble-ros-base image (see pyproject.toml
# requires-python), not a separately-downloaded interpreter.
RUN uv sync --locked --no-install-project --no-python-downloads

# Deliberately NOT prepending the whole venv to PATH: /ros-dev/.venv/bin
# also contains a `python3`/`python`, and putting that ahead of the
# system interpreter breaks colcon/ament_cmake, which shell out to
# `python3` internally (e.g. ament_cmake_core's package_xml_2_cmake.py)
# expecting the SYSTEM python3.10 with catkin_pkg installed via apt --
# our uv venv is a separate site-packages tree without catkin_pkg, so a
# shadowed `python3` breaks every `colcon build`/`colcon test` call with
# "ModuleNotFoundError: No module named 'catkin_pkg'". Instead, symlink
# just the test-runner entry points this image promises (pytest,
# coverage) into /usr/local/bin, which already precedes /usr/bin on
# PATH -- this pins which `pytest` a bare invocation resolves to
# (including the one `colcon test` makes for ament_python/
# ament_cmake_pytest packages) without touching `python3` itself.
RUN for tool in pytest coverage; do \
        if [ -e "/ros-dev/.venv/bin/$tool" ]; then \
            ln -sf "/ros-dev/.venv/bin/$tool" "/usr/local/bin/$tool"; \
        fi; \
    done

COPY smoke_test.sh ./
RUN chmod +x smoke_test.sh

CMD ["./smoke_test.sh"]
