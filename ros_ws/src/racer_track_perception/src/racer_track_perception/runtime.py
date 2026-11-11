"""numpy forward pass of `circ_conv1d_v0` from a weights.npz (no torch on the car).

Architecture: n_conv circular-padded Conv1d layers (im2col), each followed by ReLU, flatten
(channel major, as torch does), then n_linear linear layers with ReLU between them and none after
the last, reshaped to [16, 5]. Input float32 [1, 2, 360], output float32 [16, 5]: per station x_m,
y_m, half_width_left_m, half_width_right_m, log_sigma (natural log of sigma in metres), base_link.

weights.npz keys (all float32 except the integer scalars):

    n_conv, n_linear                 integer scalars
    conv{i}_weight  [C_out, C_in, K]   i in 0 .. n_conv - 1
    conv{i}_bias    [C_out]
    conv{i}_stride  integer scalar
    conv{i}_pad     integer scalar     circular padding on each side
    linear{i}_weight [out, in]         i in 0 .. n_linear - 1
    linear{i}_bias   [out]

The export writes exactly these keys (training/racer_pnp_perception/export.py) and the
torch-versus-numpy test checks the two forward passes agree to 1e-5.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

INPUT_SHAPE = (1, 2, 360)
OUTPUT_SHAPE = (16, 5)


class ModelFormatError(ValueError):
    """weights.npz does not describe a circ_conv1d_v0 with the contract's input and output."""


class _Conv:
    def __init__(self, w: NDArray[np.float32], b: NDArray[np.float32], stride: int, pad: int):
        self.w2 = np.ascontiguousarray(w.reshape(w.shape[0], -1))
        self.b = b
        self.c_in, self.k = int(w.shape[1]), int(w.shape[2])
        self.stride, self.pad = stride, pad
        self._idx: dict[int, NDArray[np.int64]] = {}

    def out_length(self, length: int) -> int:
        return (length + 2 * self.pad - self.k) // self.stride + 1

    def __call__(self, x: NDArray[np.float32]) -> NDArray[np.float32]:
        length = x.shape[1]
        if self.pad > 0:
            x = np.concatenate([x[:, -self.pad :], x, x[:, : self.pad]], axis=1)
        idx = self._idx.get(length)
        if idx is None:
            n_out = self.out_length(length)
            idx = np.arange(n_out)[:, None] * self.stride + np.arange(self.k)[None, :]
            self._idx[length] = idx
        cols = x[:, idx].transpose(1, 0, 2).reshape(idx.shape[0], self.c_in * self.k)
        return np.maximum(cols @ self.w2.T + self.b, 0.0).T.astype(np.float32, copy=False)


class CircConv1dV0:
    """The loaded network. `forward` takes the [1, 2, 360] tensor and returns [16, 5]."""

    def __init__(self, arrays: Mapping[str, Any]) -> None:
        try:
            n_conv = int(arrays["n_conv"])
            n_linear = int(arrays["n_linear"])
            self._convs = [
                _Conv(
                    np.asarray(arrays[f"conv{i}_weight"], dtype=np.float32),
                    np.asarray(arrays[f"conv{i}_bias"], dtype=np.float32),
                    int(arrays[f"conv{i}_stride"]),
                    int(arrays[f"conv{i}_pad"]),
                )
                for i in range(n_conv)
            ]
            self._linears = [
                (
                    np.asarray(arrays[f"linear{i}_weight"], dtype=np.float32),
                    np.asarray(arrays[f"linear{i}_bias"], dtype=np.float32),
                )
                for i in range(n_linear)
            ]
        except KeyError as exc:
            raise ModelFormatError(f"weights.npz is missing key {exc}") from exc
        self._check_shapes()

    def _check_shapes(self) -> None:
        if not self._convs or not self._linears:
            raise ModelFormatError("weights.npz needs at least one conv and one linear layer")
        channels, length = INPUT_SHAPE[1], INPUT_SHAPE[2]
        for i, conv in enumerate(self._convs):
            if conv.c_in != channels or conv.stride < 1 or conv.pad < 0:
                raise ModelFormatError(f"conv{i} does not follow from {channels} channels")
            if conv.b.shape != (conv.w2.shape[0],) or conv.out_length(length) < 1:
                raise ModelFormatError(f"conv{i} bias or length is inconsistent")
            channels, length = conv.w2.shape[0], conv.out_length(length)
        features = channels * length
        for i, (w, b) in enumerate(self._linears):
            if w.ndim != 2 or w.shape[1] != features or b.shape != (w.shape[0],):
                raise ModelFormatError(f"linear{i} expects {features} inputs, weight is {w.shape}")
            features = w.shape[0]
        if features != OUTPUT_SHAPE[0] * OUTPUT_SHAPE[1]:
            raise ModelFormatError(f"the output has {features} values, the contract needs 80")

    @classmethod
    def from_npz(cls, path: str | Path) -> CircConv1dV0:
        try:
            with np.load(path, allow_pickle=False) as data:
                return cls({k: data[k] for k in data.files})
        except (OSError, ValueError) as exc:
            if isinstance(exc, ModelFormatError):
                raise
            raise ModelFormatError(f"cannot read weights {path}: {exc}") from exc

    def forward(self, tensor: NDArray[np.float32]) -> NDArray[np.float32]:
        x = np.asarray(tensor, dtype=np.float32)
        if x.shape != INPUT_SHAPE:
            raise ValueError(f"input must be {INPUT_SHAPE}, got {x.shape}")
        x = x[0]
        for conv in self._convs:
            x = conv(x)
        h = x.reshape(-1)
        last = len(self._linears) - 1
        for i, (w, b) in enumerate(self._linears):
            h = w @ h + b
            if i < last:
                h = np.maximum(h, 0.0)
        return h.reshape(OUTPUT_SHAPE).astype(np.float32, copy=False)
