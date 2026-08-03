# sim-cpu image (claude-docs/03-environments.md, roadmap task 0.2)
#
# Contains: Python, CPU-only torch, and racer_gym. racer_gym currently means
# upstream f1tenth_gym pinned to a commit SHA (see pyproject.toml /
# uv.lock) — the racer_gym fork itself is sim-track task S.1 and does not
# exist yet.
#
# NEVER add ROS or CUDA to this image. torch comes from the pytorch CPU
# wheel index (see [tool.uv.sources] in pyproject.toml), never the default
# CUDA-enabled build.
#
# Rebuilds are reproducible from this Dockerfile + uv.lock alone: `uv sync
# --locked` installs exactly the pinned, hashed dependency set and fails
# the build if the lockfile and pyproject.toml have drifted apart, rather
# than silently re-resolving. No `docker exec` state is ever required.

FROM python:3.11-slim

# git is needed only to fetch the pinned f1tenth_gym commit during `uv
# sync`; it is not part of the runtime the smoke test exercises.
#
# The rest of these are runtime shared libraries, not a display/GUI stack:
# f1tenth_gym's opencv-python and pygame dependencies are manylinux wheels
# that dynamically link against a handful of X11/GL client libraries even
# when no window is ever opened (manylinux policy assumes these are always
# present on the host; python:3.11-slim has none of them). Without these,
# `import cv2` / `import pygame` fail at import time with a missing
# .so, well before render_mode or headlessness ever come into it.
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
# locally, so `uv sync --locked` resolves the lockfile the same way here
# as it did during development.
COPY --from=ghcr.io/astral-sh/uv:0.12.5 /uv /uvx /usr/local/bin/

WORKDIR /sim-cpu

COPY pyproject.toml uv.lock ./

# --locked: fail the build instead of silently re-resolving if uv.lock is
# out of date relative to pyproject.toml. --no-install-project: this
# pyproject.toml is a dependency manifest only (tool.uv.package = false),
# there is no sim-cpu package to install.
RUN uv sync --locked --no-install-project

ENV PATH="/sim-cpu/.venv/bin:${PATH}"

COPY smoke_test.py ./

CMD ["python", "smoke_test.py"]
