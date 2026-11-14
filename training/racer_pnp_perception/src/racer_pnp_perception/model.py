"""The torch model `circ_conv1d_v0` and its conversion to the numpy runtime's weights.

torch is imported inside the functions that need it (never at module import), so importing this
package in the per-push CI job does not need it. The architecture: four circular Conv1d layers
(kernel 5, stride 2, circular padding 2, ReLU), flatten, linear to `hidden`, ReLU, linear to 80,
reshaped [16, 5]. The runtime in ros_ws/src/racer_track_perception/runtime.py reads the npz this
module writes (`to_npz_arrays`).
"""

from __future__ import annotations

import itertools
from typing import Any

import numpy as np
from numpy.typing import NDArray

CHANNELS = (2, 16, 32, 32, 32)
HIDDEN = 128
KERNEL = 5
STRIDE = 2
PAD = 2
N_BINS = 360
STATIONS = 16
OUT_PER_STATION = 5


def conv_out_length(length: int) -> int:
    return (length + 2 * PAD - KERNEL) // STRIDE + 1


def flat_features(channels: tuple[int, ...] = CHANNELS) -> int:
    length = N_BINS
    for _ in channels[1:]:
        length = conv_out_length(length)
    return channels[-1] * length


def build_model(channels: tuple[int, ...] = CHANNELS, hidden: int = HIDDEN) -> Any:
    """A fresh torch module. Input [B, 2, 360], output [B, 16, 5]."""
    import torch
    from torch import nn

    class CircConv1dV0(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.convs = nn.ModuleList(
                nn.Conv1d(
                    c_in,
                    c_out,
                    KERNEL,
                    stride=STRIDE,
                    padding=PAD,
                    padding_mode="circular",
                )
                for c_in, c_out in itertools.pairwise(channels)
            )
            self.fc0 = nn.Linear(flat_features(channels), hidden)
            self.fc1 = nn.Linear(hidden, STATIONS * OUT_PER_STATION)
            # Start with a straight 0.2 m spaced centreline and half widths of 0.5 m.
            with torch.no_grad():
                self.fc1.weight.mul_(0.1)
                bias = torch.zeros(STATIONS, OUT_PER_STATION)
                bias[:, 0] = 0.2 * torch.arange(STATIONS)
                bias[:, 2:4] = 0.5
                bias[:, 4] = float(np.log(0.1))
                self.fc1.bias.copy_(bias.reshape(-1))

        def forward(self, x: Any) -> Any:
            for conv in self.convs:
                x = torch.relu(conv(x))
            x = torch.relu(self.fc0(x.flatten(1)))
            return self.fc1(x).reshape(-1, STATIONS, OUT_PER_STATION)

    return CircConv1dV0()


def count_parameters(model: Any) -> int:
    return int(sum(p.numel() for p in model.parameters()))


def to_npz_arrays(model: Any) -> dict[str, NDArray[Any]]:
    """The weights in the key layout runtime.CircConv1dV0 reads."""
    out: dict[str, NDArray[Any]] = {
        "n_conv": np.array(len(model.convs)),
        "n_linear": np.array(2),
    }
    for i, conv in enumerate(model.convs):
        out[f"conv{i}_weight"] = conv.weight.detach().cpu().numpy().astype(np.float32)
        out[f"conv{i}_bias"] = conv.bias.detach().cpu().numpy().astype(np.float32)
        out[f"conv{i}_stride"] = np.array(conv.stride[0])
        out[f"conv{i}_pad"] = np.array(conv.padding[0])
    for i, fc in enumerate((model.fc0, model.fc1)):
        out[f"linear{i}_weight"] = fc.weight.detach().cpu().numpy().astype(np.float32)
        out[f"linear{i}_bias"] = fc.bias.detach().cpu().numpy().astype(np.float32)
    return out


def loss_fn(
    pred: Any,
    target: Any,
    width_huber_delta: float = 0.1,
    width_weight: float = 5.0,
    station_weight: Any = None,
) -> Any:
    """Gaussian negative log likelihood on (x, y) with the predicted sigma of each station, plus a
    Huber loss on the two half widths. pred and target are [B, 16, 5]; the target's log_sigma
    column is unused. `station_weight` [B, 16] (0 or 1) leaves out stations that have no label,
    as the stations past a real self-label's valid_count; the loss is then the weighted mean."""
    import torch
    from torch.nn import functional

    log_sigma = pred[..., 4].clamp(-7.0, 3.0)
    err2 = (pred[..., 0] - target[..., 0]) ** 2 + (pred[..., 1] - target[..., 1]) ** 2
    nll = 2.0 * log_sigma + err2 / (2.0 * torch.exp(2.0 * log_sigma))
    huber = functional.huber_loss(
        pred[..., 2:4], target[..., 2:4], delta=width_huber_delta, reduction="none"
    ).sum(-1)
    per_station = nll + width_weight * huber
    if station_weight is None:
        return per_station.mean()
    return (per_station * station_weight).sum() / station_weight.sum().clamp(min=1.0)
