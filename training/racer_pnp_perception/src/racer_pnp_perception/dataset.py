"""Seeded dataset generation: scans (as the net's input tensor) and labels, from simulation.

    uv run python -m racer_pnp_perception.dataset --n-train 300000 --n-val 20000 --seed 0

writes `<out>/trackperc-v0-train-s<seed>-n<n>.npz` and `...-val-...` under
/Users/cameronjim/code/car-data/pnp/datasets/. Every sample goes through the same
`scan_to_tensor` the node uses; the tensor is stored quantised (channel 0 as uint16 over [0, 1],
channel 1 as uint8) to keep 300k samples near 330 MB. Chunks are generated in worker processes,
each with its own child seed, so a (seed, n) pair always gives the same dataset.
"""

from __future__ import annotations

import argparse
import functools
import logging
import multiprocessing as mp
import sys
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from racer_track_perception.preprocess import N_BINS, scan_to_tensor
from racer_track_perception.vehicle import LiveVehicle, load_live_vehicle

from racer_pnp_perception.raycast import ScanGeometry
from racer_pnp_perception.sample import LaneGeometry, make_sample
from racer_pnp_perception.tracks import World, held_out_worlds, sample_world

log = logging.getLogger(__name__)

DEFAULT_DIR = Path("/Users/cameronjim/code/car-data/pnp/datasets")
SAMPLES_PER_WORLD = 16
CHUNK = 2000
QUANT = 65535.0


@functools.lru_cache(maxsize=1)
def _vehicle() -> LiveVehicle:
    return load_live_vehicle()


def quantise(tensor: NDArray[np.float32]) -> tuple[NDArray[np.uint16], NDArray[np.uint8]]:
    """[n, 2, 360] float tensors as (uint16 channel 0, uint8 channel 1)."""
    return (
        np.round(tensor[:, 0, :] * QUANT).astype(np.uint16),
        tensor[:, 1, :].astype(np.uint8),
    )


def dequantise(q0: NDArray[np.uint16], q1: NDArray[np.uint8]) -> NDArray[np.float32]:
    out = np.empty((len(q0), 2, N_BINS), dtype=np.float32)
    out[:, 0, :] = q0.astype(np.float32) / QUANT
    out[:, 1, :] = q1.astype(np.float32)
    return out


def generate_chunk(
    seed: int,
    n: int,
    held_out: str | None = None,
    clean: bool = False,
    keep_ranges: bool = False,
) -> dict[str, NDArray[Any]]:
    """`n` samples from random training worlds (or, with `held_out`, from that held-out track)."""
    live = _vehicle()
    geometry = ScanGeometry()
    rng = np.random.default_rng(seed)
    held = held_out_worlds() if held_out else {}
    tensors = np.empty((n, 2, N_BINS), dtype=np.float32)
    labels = np.empty((n, 16, 5), dtype=np.float32)
    ranges = np.empty((n, geometry.n_beams), dtype=np.float32) if keep_ranges else None
    world: World | None = None
    lane: LaneGeometry | None = None
    for i in range(n):
        if world is None or i % SAMPLES_PER_WORLD == 0:
            world = held[held_out] if held_out else sample_world(rng)
            lane = LaneGeometry(world.centreline)
        assert lane is not None
        s = make_sample(world, lane, rng, live.mount, geometry, clean=clean)
        tensors[i] = scan_to_tensor(
            s.ranges,
            geometry.angle_min_rad,
            geometry.angle_increment_rad,
            geometry.range_min_m,
            geometry.range_max_m,
            live.mount,
            live.body,
        )
        labels[i] = s.label
        if ranges is not None:
            ranges[i] = s.ranges
    out: dict[str, NDArray[Any]] = {"tensor": tensors, "label": labels}
    if ranges is not None:
        out["ranges"] = ranges
    return out


def _chunk_job(args: tuple[int, int, str | None, bool, bool]) -> dict[str, NDArray[Any]]:
    seed, n, held_out, clean, keep_ranges = args
    return generate_chunk(seed, n, held_out, clean, keep_ranges)


def generate(
    n: int,
    seed: int,
    workers: int | None = None,
    held_out: str | None = None,
    clean: bool = False,
    keep_ranges: bool = False,
) -> dict[str, NDArray[Any]]:
    """`n` samples in parallel chunks; deterministic in (n, seed, ...)."""
    sizes = [CHUNK] * (n // CHUNK) + ([n % CHUNK] if n % CHUNK else [])
    seeds = np.random.SeedSequence(seed).generate_state(len(sizes))
    jobs = [(int(s), k, held_out, clean, keep_ranges) for s, k in zip(seeds, sizes, strict=True)]
    workers = workers or max((mp.cpu_count() or 2) - 1, 1)
    if workers == 1 or len(jobs) == 1:
        parts = [_chunk_job(j) for j in jobs]
    else:
        with mp.get_context("spawn").Pool(workers) as pool:
            parts = pool.map(_chunk_job, jobs)
    return {k: np.concatenate([p[k] for p in parts]) for k in parts[0]}


def save(path: Path, data: dict[str, NDArray[Any]]) -> None:
    q0, q1 = quantise(data["tensor"])
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path, q0=q0, q1=q1, label=data["label"])


def load(path: Path) -> tuple[NDArray[np.float32], NDArray[np.float32]]:
    with np.load(path) as d:
        return dequantise(d["q0"], d["q1"]), d["label"]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--n-train", type=int, default=300_000)
    parser.add_argument("--n-val", type=int, default=20_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=Path, default=DEFAULT_DIR)
    parser.add_argument("--workers", type=int, default=None)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    for name, n, seed in (
        ("train", args.n_train, args.seed),
        ("val", args.n_val, args.seed + 1_000_003),
    ):
        path = args.out / f"trackperc-v0-{name}-s{args.seed}-n{n}.npz"
        log.info("generating %s: %d samples, seed %d -> %s", name, n, seed, path)
        save(path, generate(n, seed, args.workers))
    return 0


if __name__ == "__main__":
    sys.exit(main())
