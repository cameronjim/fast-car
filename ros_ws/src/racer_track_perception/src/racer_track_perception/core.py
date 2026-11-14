"""The node's decision logic without ROS: one scan in, one `LocalTrackData` out.

`TrackPerceiver.process` is what `track_perception_node` calls for every scan, and what the
real-bag gate and the simulation evaluation call, so all of them run exactly the same code:
preprocess, run the backend (the numpy net or the geometric tracer), compute the self-checks.
It never raises: a scan that cannot be used gives "no track" (`valid_count` 0).
"""

from __future__ import annotations

import logging
import math
from typing import Any

import numpy as np
from numpy.typing import NDArray

from racer_track_perception import geometric, preprocess, refine, selfcheck
from racer_track_perception import track as tr
from racer_track_perception.runtime import CircConv1dV0
from racer_track_perception.vehicle import LiveVehicle

log = logging.getLogger(__name__)

BACKEND_NET = "net"
BACKEND_GEOMETRIC = "geometric"
GEOMETRIC_MODEL_ID = "geometric-v1"
# Clamp on the net's log sigma before exponentiating (sigma 1e-3 m to 20 m).
LOG_SIGMA_RANGE = (-7.0, 3.0)


def net_output_to_track(output: NDArray[np.floating], sigma_max_m: float) -> tr.LocalTrackData:
    """The net's [16, 5] output as a LocalTrackData (valid_count from sigma)."""
    out = np.asarray(output, dtype=np.float64)
    if out.shape != (tr.K_STATIONS, 5):
        return tr.no_track()
    with np.errstate(invalid="ignore"):
        log_sigma = np.clip(out[:, 4], *LOG_SIGMA_RANGE)
    sigma = np.exp(log_sigma)  # NaN stays NaN and ends the valid run
    return tr.assemble(out[:, 0], out[:, 1], out[:, 2], out[:, 3], sigma, sigma_max_m)


class TrackPerceiver:
    def __init__(
        self,
        backend: str,
        vehicle: LiveVehicle,
        sigma_max_m: float = 0.25,
        default_half_width_m: float = 0.5,
        hw_min_m: float = geometric.DEFAULT_HW_MIN_M,
        hw_max_m: float = geometric.DEFAULT_HW_MAX_M,
        net: CircConv1dV0 | None = None,
        model_id: str | None = None,
        refine_net: bool = False,
    ) -> None:
        if backend not in (BACKEND_NET, BACKEND_GEOMETRIC):
            raise ValueError(f"backend must be 'net' or 'geometric', got {backend!r}")
        if backend == BACKEND_NET and net is None:
            raise ValueError("the net backend needs a loaded network")
        self.backend = backend
        self.vehicle = vehicle
        self.sigma_max_m = sigma_max_m
        self.net = net
        # `Bundle.refine`: the bundle's io.arch.type says the net output is refined against the
        # scan. Only the net backend has an output to refine; geometric-v1 is the scan itself.
        self.refine_net = bool(refine_net) and backend == BACKEND_NET
        self.model_id = (model_id or "unknown") if backend == BACKEND_NET else GEOMETRIC_MODEL_ID
        self._refine_cfg = refine.RefineConfig(sigma_max_m=sigma_max_m)
        self._geo_cfg = geometric.GeometricConfig(
            default_half_width_m=default_half_width_m, hw_min_m=hw_min_m, hw_max_m=hw_max_m
        )

    def process(
        self,
        ranges: Any,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
    ) -> tr.LocalTrackData:
        try:
            tensor, points = preprocess.scan_to_tensor_and_points(
                ranges,
                angle_min,
                angle_increment,
                range_min,
                range_max,
                self.vehicle.mount,
                self.vehicle.body,
            )
            if self.backend == BACKEND_NET:
                assert self.net is not None
                track = net_output_to_track(self.net.forward(tensor), self.sigma_max_m)
                if self.refine_net:
                    track = refine.refine_track(track, points, self._refine_cfg)
            else:
                track = geometric.trace(
                    points, self.vehicle.body.half_width_m, self.sigma_max_m, self._geo_cfg
                )
            selfcheck.apply_self_check(track, points)
        except Exception:
            log.exception("track perception failed on a scan; publishing no track")
            return tr.no_track()
        if not track.arrays_ok():
            return tr.no_track()
        if math.isinf(track.boundary_residual_m):
            track.boundary_residual_m = math.nan
        if math.isinf(track.intrusion_fraction):
            track.intrusion_fraction = math.nan
        return track
