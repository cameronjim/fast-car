"""GPU verification for the train-cuda image (roadmap task 0.3).

This is the "torch sees the GPU" done-criterion check. It can only be run
for real on Desktop A (amd64, RTX 3060) with the NVIDIA driver + Container
Toolkit installed -- there is no GPU in CI and none on the Mac this image
was authored on, so this script is NOT run automatically anywhere. It ships
in the image, ready for the owner to run on Desktop A (see README.md for
the exact command).

Checks, in order:
  1. torch.cuda.is_available() is True (fails loud otherwise -- no silent
     CPU fallback).
  2. Device name and compute capability are printed (Ampere / 8.6 expected
     for the RTX 3060).
  3. A small matmul run on the GPU matches the same matmul run on the CPU,
     within a stated tolerance -- proves the GPU path isn't just "visible"
     but actually computing correctly.

Exits nonzero on any failure.
"""

from __future__ import annotations

import sys


def main() -> int:
    try:
        import torch
    except Exception as exc:  # noqa: BLE001 - report and fail
        print(f"GPU CHECK FAILED: could not import torch: {exc}", file=sys.stderr)
        return 1

    try:
        if not torch.cuda.is_available():
            raise AssertionError(
                "torch.cuda.is_available() is False -- train-cuda must see a GPU. "
                "Check: NVIDIA driver installed on Desktop A, NVIDIA Container "
                "Toolkit installed, and the container was run with --gpus all."
            )

        device_count = torch.cuda.device_count()
        device_index = 0
        device_name = torch.cuda.get_device_name(device_index)
        capability = torch.cuda.get_device_capability(device_index)

        print(f"torch {torch.__version__} (CUDA build {torch.version.cuda})")
        print(f"CUDA device count: {device_count}")
        print(f"CUDA device 0: {device_name}, compute capability {capability[0]}.{capability[1]}")

        # Small matmul on GPU, cross-checked against the identical
        # computation on CPU. Fixed seed for a reproducible check.
        torch.manual_seed(0)
        a_cpu = torch.randn(256, 256, dtype=torch.float32)
        b_cpu = torch.randn(256, 256, dtype=torch.float32)

        a_gpu = a_cpu.to(f"cuda:{device_index}")
        b_gpu = b_cpu.to(f"cuda:{device_index}")

        result_gpu = (a_gpu @ b_gpu).cpu()
        result_cpu = a_cpu @ b_cpu

        max_abs_diff = (result_gpu - result_cpu).abs().max().item()
        tolerance = 1e-2  # generous: GPU/CPU float32 matmul accumulation order differs
        if max_abs_diff > tolerance:
            raise AssertionError(
                f"GPU matmul result diverges from CPU result: max abs diff "
                f"{max_abs_diff} exceeds tolerance {tolerance}"
            )

        print(f"GPU/CPU matmul cross-check OK: max abs diff {max_abs_diff:.6g}")
    except Exception as exc:  # noqa: BLE001 - top-level check, report and fail
        print(f"GPU CHECK FAILED: {exc}", file=sys.stderr)
        return 1

    print("train-cuda gpu_check PASSED: torch sees the GPU")
    return 0


if __name__ == "__main__":
    sys.exit(main())
