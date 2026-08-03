# train-cuda image (claude-docs/03-environments.md, roadmap task 0.3)
#
# Runs on Desktop A only (amd64, RTX 3060). Contains: CUDA-enabled torch
# (cu126 wheel index), racer_gym (currently upstream f1tenth_gym pinned to
# the same commit SHA as sim-cpu -- the racer_gym fork, task S.1, does not
# exist yet), and racer_train once it exists (training/racer_train/, task
# S.3 -- see the "does not exist yet" comment in pyproject.toml). NEVER add
# ROS to this image (claude-docs/03-environments.md: train-cuda never
# contains ROS).
#
# Base image choice: python:3.11-slim, NOT an nvidia/cuda base image.
# torch's cu126 wheels vendor their own CUDA 12.6 runtime shared libraries
# (nvidia-cublas-cu12, nvidia-cudnn-cu12, nvidia-cusolver-cu12, etc. -- see
# uv.lock) as ordinary pip dependencies; nothing in this image compiles
# CUDA code or links against a system-wide CUDA Toolkit install. Stacking
# an nvidia/cuda:12.6-runtime-ubuntu22.04 base underneath would duplicate
# most of that same CUDA runtime a second time for no functional gain --
# only the *driver* on the host matters, and that is supplied by Desktop A
# itself plus the NVIDIA Container Toolkit (`--gpus all`), not by anything
# baked into this image. python:3.11-slim keeps this image on the same
# minimal base as sim-cpu and avoids doubling an already ~3.8 GB dependency
# set (torch + cuDNN + cuBLAS + cuSPARSE + NCCL + triton, per uv.lock).
#
# Rebuilds are reproducible from this Dockerfile + uv.lock alone: `uv sync
# --locked` installs exactly the pinned, hashed dependency set and fails
# the build if the lockfile and pyproject.toml have drifted apart, rather
# than silently re-resolving. No `docker exec` state is ever required.

FROM python:3.11-slim

# git is needed only to fetch the pinned f1tenth_gym commit during `uv
# sync`. The X11/GL client libraries are the same runtime shared-library
# set sim-cpu needs for f1tenth_gym's opencv-python/pygame dependencies
# (manylinux wheels dynamically link against them even when no window is
# ever opened) -- see docker/sim-cpu/Dockerfile for the detailed rationale.
RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        libgl1 \
        libglib2.0-0 \
        libsm6 \
        libxext6 \
        libxrender1 \
        libxcb1 \
        libx11-6 \
        libgomp1 \
    && rm -rf /var/lib/apt/lists/*

# Pin the uv binary itself to the same version used to generate uv.lock
# locally (matches docker/sim-cpu), so `uv sync --locked` resolves the
# lockfile the same way here as it did during development.
COPY --from=ghcr.io/astral-sh/uv:0.12.5 /uv /uvx /usr/local/bin/

WORKDIR /train-cuda

COPY pyproject.toml uv.lock ./

# --locked: fail the build instead of silently re-resolving if uv.lock is
# out of date relative to pyproject.toml. --no-install-project: this
# pyproject.toml is a dependency manifest only (tool.uv.package = false),
# there is no train-cuda package to install.
RUN uv sync --locked --no-install-project

ENV PATH="/train-cuda/.venv/bin:${PATH}"

COPY gpu_check.py ./

# Default CMD is the real GPU check -- meant to be run on Desktop A with
# `docker run --gpus all` (see README.md). It requires a real NVIDIA GPU
# and driver, so it is never invoked as part of the CI build (CI runners
# have no GPU); CI instead runs its own CPU-safe sanity import, separately,
# against this same built image (see .github/workflows/ci.yml,
# train-cuda-image job).
CMD ["python", "gpu_check.py"]
