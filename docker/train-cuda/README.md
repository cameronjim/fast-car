# train-cuda

GPU training image for Desktop A (amd64, RTX 3060). See
`claude-docs/03-environments.md` and roadmap task 0.3 in
`claude-docs/01-roadmap.md`.

Contains CUDA-enabled torch (cu126 wheel index), racer_gym (currently
upstream f1tenth_gym pinned to the same commit SHA as `docker/sim-cpu`),
and racer_train once it exists (roadmap task S.3). Never contains ROS.

This image is built and sanity-checked (CPU-safe import only, no GPU) in
CI on every push touching `docker/train-cuda/**` -- see
`.github/workflows/ci.yml`, job `train-cuda-image`. CI has no GPU, so it
cannot prove "torch sees the GPU"; that verification only happens on
Desktop A, by running this image with `--gpus all`.

## Prerequisites on Desktop A

- NVIDIA driver installed and supporting CUDA 12.6 (check with
  `nvidia-smi`).
- Docker with the NVIDIA Container Toolkit installed, so `--gpus all`
  works (`docker run --rm --gpus all nvidia/cuda:12.6.0-base-ubuntu22.04
  nvidia-smi` should succeed as a sanity check before building this
  image).

## Build

```
docker build -t train-cuda:local docker/train-cuda
```

## Run the GPU check (the roadmap 0.3 done-criterion)

```
docker run --rm --gpus all train-cuda:local
```

This runs `gpu_check.py` (the image's default `CMD`): it asserts
`torch.cuda.is_available()`, prints the device name and compute
capability, runs a small matmul on the GPU, and cross-checks the result
against the same matmul on CPU. It exits nonzero on any failure.

Once this passes on Desktop A, tick roadmap task 0.3 to `[x]` in
`claude-docs/01-roadmap.md` with a dated completion note (see the `[~]`
note already there for what CI already proved).
