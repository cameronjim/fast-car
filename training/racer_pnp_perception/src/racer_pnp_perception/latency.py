"""Latency of the numpy forward pass and of scan to track on real scans (a proxy for the Orin).

    OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 \\
    uv run --group bags python -m racer_pnp_perception.latency --bundle <bundle dir> \\
        --scans <scans cache npz> [--n 2000] [--out latency.json]

`--scans` is a scan cache written by `selflabel` (`<labels>/cache/<bag>.scans.npz`). Evenly spaced
scans are taken (`--n`, default 2000), and for each the time of (a) `CircConv1dV0.forward` on the
scan's tensor and (b) `TrackPerceiver.process` for the net and for the geometric tracer (the
code the node runs, preprocess included) is measured with `time.perf_counter` after a warm-up.
Single thread only if the environment says so (the numpy runtime uses BLAS matmuls). The number
that decides anything is the Orin's, taken at the car; this is the dev Mac.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from racer_track_perception.core import BACKEND_GEOMETRIC, BACKEND_NET, TrackPerceiver
from racer_track_perception.preprocess import scan_to_tensor
from racer_track_perception.vehicle import load_live_vehicle

from racer_pnp_perception import selflabel as sl
from racer_pnp_perception.eval_bags import build_perceivers

log = logging.getLogger(__name__)

WARMUP = 20


def percentiles(ms: NDArray[np.float64]) -> dict[str, float]:
    return {
        "p50": float(np.percentile(ms, 50)),
        "p99": float(np.percentile(ms, 99)),
        "max": float(ms.max()),
        "mean": float(ms.mean()),
    }


def measure(scans: sl.ScanSet, perceivers: dict[str, TrackPerceiver], n: int) -> dict[str, Any]:
    """Latency in milliseconds over `n` evenly spaced scans."""
    pick = np.linspace(0, len(scans) - 1, min(n, len(scans))).round().astype(int)
    live = perceivers[BACKEND_GEOMETRIC].vehicle
    net = perceivers[BACKEND_NET].net
    assert net is not None
    geometry = (scans.angle_min, scans.angle_increment, scans.range_min, scans.range_max)
    tensors = [scan_to_tensor(scans.ranges[i], *geometry, live.mount, live.body) for i in pick]
    for k in range(min(WARMUP, len(pick))):
        net.forward(tensors[k])
        for p in perceivers.values():
            p.process(scans.ranges[pick[k]], *geometry)
    forward = np.zeros(len(pick))
    process = {name: np.zeros(len(pick)) for name in perceivers}
    for k, i in enumerate(pick):
        t0 = time.perf_counter()
        net.forward(tensors[k])
        forward[k] = (time.perf_counter() - t0) * 1000.0
        for name, p in perceivers.items():
            t0 = time.perf_counter()
            p.process(scans.ranges[i], *geometry)
            process[name][k] = (time.perf_counter() - t0) * 1000.0
    return {
        "scans": len(pick),
        "net_forward": percentiles(forward),
        "scan_to_track": {name: percentiles(v) for name, v in process.items()},
        **percentiles(forward),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--scans", type=Path, required=True)
    parser.add_argument("--n", type=int, default=2000)
    parser.add_argument("--out", type=Path, default=Path("latency.json"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    live = load_live_vehicle()
    perceivers = build_perceivers(live, args.bundle)
    scans, _t, _v = sl.load_scanset(
        args.scans.parent.parent / args.scans.name.removesuffix(".scans.npz"), args.scans.parent
    )
    result = measure(scans, perceivers, args.n)
    args.out.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
