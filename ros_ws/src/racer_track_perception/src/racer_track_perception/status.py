"""Running statistics of the node, and the JSON the 1 Hz /pnp/perception/status carries.

ROS-free so the arithmetic is tested without rclpy. NaN is written as null (strict JSON).
"""

from __future__ import annotations

import json
import math
from collections import deque

import numpy as np

from racer_track_perception.track import LocalTrackData

LATENCY_WINDOW = 1000
SILENCE_S = 1.0


def _clean(value: float) -> float | None:
    return None if not math.isfinite(value) else float(value)


class PerceptionStats:
    def __init__(self, backend: str, model_id: str, payload_sha256: str | None) -> None:
        self.backend = backend
        self.model_id = model_id
        self.payload_sha256 = payload_sha256
        self.scans_seen = 0
        self.scans_published = 0
        self._latency_ms: deque[float] = deque(maxlen=LATENCY_WINDOW)
        self._latency_max_ms = 0.0
        self._valid_sum = 0.0
        self._residual_sum = 0.0
        self._residual_n = 0
        self._intrusion_sum = 0.0
        self._intrusion_n = 0
        self.last_scan_s: float | None = None

    def scan_seen(self, now_s: float) -> None:
        self.scans_seen += 1
        self.last_scan_s = now_s

    def scan_published(self, track: LocalTrackData, latency_ms: float) -> None:
        self.scans_published += 1
        self._latency_ms.append(latency_ms)
        self._latency_max_ms = max(self._latency_max_ms, latency_ms)
        self._valid_sum += track.valid_count
        if math.isfinite(track.boundary_residual_m):
            self._residual_sum += track.boundary_residual_m
            self._residual_n += 1
        if math.isfinite(track.intrusion_fraction):
            self._intrusion_sum += track.intrusion_fraction
            self._intrusion_n += 1

    def scan_age_s(self, now_s: float) -> float | None:
        return None if self.last_scan_s is None else now_s - self.last_scan_s

    def as_dict(self, now_s: float) -> dict[str, object]:
        n = self.scans_published
        lat = np.array(self._latency_ms, dtype=np.float64)
        age = self.scan_age_s(now_s)
        silent = age is None or age > SILENCE_S
        return {
            "backend": self.backend,
            "model_id": self.model_id,
            "payload_sha256": self.payload_sha256,
            "state": "no_scan" if silent else "running",
            "scan_age_s": None if age is None else _clean(age),
            "scans_seen": self.scans_seen,
            "scans_published": n,
            "latency_ms": {
                "p50": _clean(float(np.percentile(lat, 50))) if lat.size else None,
                "p99": _clean(float(np.percentile(lat, 99))) if lat.size else None,
                "max": _clean(self._latency_max_ms) if lat.size else None,
            },
            "valid_count_mean": _clean(self._valid_sum / n) if n else None,
            "boundary_residual_mean_m": _clean(self._residual_sum / self._residual_n)
            if self._residual_n
            else None,
            "intrusion_fraction_mean": _clean(self._intrusion_sum / self._intrusion_n)
            if self._intrusion_n
            else None,
        }

    def as_json(self, now_s: float) -> str:
        return json.dumps(self.as_dict(now_s), allow_nan=False)
