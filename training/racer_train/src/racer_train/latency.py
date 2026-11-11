"""Inference latency of an exported policy directory, measured on THIS machine's CPU.

08-learning.md: "Inference target: 50 Hz on the Jetson ... Publish the inference jitter
histogram; port policy_node to C++ only if tail latency is bad." The 50 Hz budget is 20 ms per
step. This loads the policy the way the deploy node will (`load_contract` + `load_model`,
TorchScript, CPU, one thread) and times single-observation forward passes. It measures the
forward pass only: no ROS, no observation assembly, no envelope (those are microseconds of
plain arithmetic next to the model call).

A laptop number is not a Jetson number. The Jetson estimate in the v0 note is a stated scaling
assumption, not a measurement; the real number comes from running this same command on the
Orin Nano.

    uv run python -m racer_train.latency --policy-dir <dir> --out <json>
"""

from __future__ import annotations

import argparse
import itertools
import json
import platform
import time
from pathlib import Path

import numpy as np


def measure(policy_dir: Path, n_calls: int = 20000, warmup: int = 500, threads: int = 1) -> dict:
    import torch
    from racer_policy import load_contract, load_model

    torch.set_num_threads(threads)
    contract = load_contract(Path(policy_dir))
    model = load_model(contract)
    obs_dim = len(contract.observation_schema.fields) + contract.observation_schema.lidar.beam_count
    rng = np.random.default_rng(0)
    obs = torch.from_numpy(rng.uniform(0.0, 3.0, size=(1, obs_dim)).astype(np.float32))
    n_params = sum(p.numel() for p in model.parameters())
    with torch.no_grad():
        for _ in range(warmup):
            model(obs)
        samples = np.empty(n_calls)
        for i in range(n_calls):
            t0 = time.perf_counter()
            model(obs)
            samples[i] = time.perf_counter() - t0
    us = samples * 1e6
    hist_edges = [0, 25, 50, 75, 100, 150, 200, 300, 500, 1000, 5000, float("inf")]
    counts, _ = np.histogram(us, bins=hist_edges)
    return {
        "machine": platform.platform(),
        "processor": platform.processor(),
        "torch_threads": threads,
        "n_calls": n_calls,
        "obs_dim": obs_dim,
        "parameters": n_params,
        "mean_us": float(us.mean()),
        "p50_us": float(np.percentile(us, 50)),
        "p99_us": float(np.percentile(us, 99)),
        "p999_us": float(np.percentile(us, 99.9)),
        "max_us": float(us.max()),
        "budget_us_at_50hz": 20000.0,
        "histogram_us": {
            f"{lo}-{hi}": int(c)
            for (lo, hi), c in zip(itertools.pairwise(hist_edges), counts, strict=True)
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--calls", type=int, default=20000)
    parser.add_argument("--threads", type=int, default=1)
    args = parser.parse_args(argv)
    result = measure(args.policy_dir, args.calls, threads=args.threads)
    print(json.dumps(result, indent=2))
    if args.out is not None:
        args.out.write_text(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
