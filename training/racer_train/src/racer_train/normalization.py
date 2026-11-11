"""Frozen observation normalisation, shared by the training env wrapper and the exported policy.

Why frozen and why it lives INSIDE `policy.pt`: the S.3 pipeline recorded running
mean/std into the contract but trained the network on RAW observations, so a deploy node that
applied the contract's statistics (as 08-learning.md implies) would have fed the network
inputs it never saw. v0 removes the ambiguity:

  * the statistics are computed ONCE, from tracker-only rollouts, before training starts, and
    never updated;
  * the network is trained on `normalize(raw)` and the exported TorchScript module takes the
    RAW observation (exactly the units the contract's observation schema declares) and applies
    `normalize` itself;
  * therefore the contract's own `normalization` block is written as the IDENTITY (mean 0,
    std 1) so no consumer can double-normalise, and the real statistics are recorded in
    `policy_meta.yaml` next to the policy for provenance.

`normalize` clips the LiDAR beams to `lidar_clip_m` first (the real sensor's 12 m range
differs from the sim's; beyond a few metres a 1 m lane policy gains nothing) and clips the
result to +-`CLIP_SIGMA` so a single wild reading cannot dominate the first layer.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

CLIP_SIGMA = 10.0


@dataclass(frozen=True)
class ObsNormalization:
    mean: tuple[float, ...]
    std: tuple[float, ...]
    low_dim_count: int
    lidar_clip_m: float

    def __post_init__(self) -> None:
        if len(self.mean) != len(self.std):
            raise ValueError("mean and std must be the same length")
        if not 0 < self.low_dim_count <= len(self.mean):
            raise ValueError("low_dim_count must be within the observation length")
        if any(s <= 0.0 for s in self.std):
            raise ValueError("every std must be > 0")

    def normalize(self, obs: np.ndarray) -> np.ndarray:
        x = np.asarray(obs, dtype=np.float32).copy()
        x[..., self.low_dim_count :] = np.minimum(x[..., self.low_dim_count :], self.lidar_clip_m)
        x = (x - np.asarray(self.mean, dtype=np.float32)) / np.asarray(self.std, dtype=np.float32)
        return np.clip(x, -CLIP_SIGMA, CLIP_SIGMA).astype(np.float32)


def fit_normalization(
    observations: np.ndarray,
    low_dim_count: int,
    lidar_clip_m: float,
    min_std: float = 1e-3,
) -> ObsNormalization:
    """Mean/std of `observations` (N x D) after the LiDAR clip. A (near-)constant column gets
    `min_std` so it is passed through rather than blown up."""
    x = np.asarray(observations, dtype=np.float64).copy()
    x[:, low_dim_count:] = np.minimum(x[:, low_dim_count:], lidar_clip_m)
    mean = x.mean(axis=0)
    std = np.maximum(x.std(axis=0), min_std)
    return ObsNormalization(
        mean=tuple(float(v) for v in mean),
        std=tuple(float(v) for v in std),
        low_dim_count=low_dim_count,
        lidar_clip_m=lidar_clip_m,
    )
