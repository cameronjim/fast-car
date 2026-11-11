from __future__ import annotations

import json
import math

import numpy as np
from racer_track_perception import status as st
from racer_track_perception import track as tr


def track(valid: int, residual: float, intrusion: float) -> tr.LocalTrackData:
    t = tr.no_track()
    t.valid_count = valid
    t.boundary_residual_m = residual
    t.intrusion_fraction = intrusion
    return t


def test_before_any_scan_the_status_says_no_scan() -> None:
    s = st.PerceptionStats("geometric", "geometric-v0", None)
    d = s.as_dict(10.0)
    assert d["state"] == "no_scan" and d["scans_seen"] == 0 and d["scans_published"] == 0
    assert d["scan_age_s"] is None and d["payload_sha256"] is None
    assert d["latency_ms"] == {"p50": None, "p99": None, "max": None}
    assert d["valid_count_mean"] is None
    json.loads(s.as_json(10.0))  # strict JSON, no NaN


def test_numbers_accumulate() -> None:
    s = st.PerceptionStats("net", "m", "abc")
    for i in range(10):
        s.scan_seen(float(i))
        s.scan_published(track(10 + i % 2, 0.05 if i % 2 else math.nan, 0.1), 1.0 + i)
    d = s.as_dict(9.5)
    assert d["state"] == "running" and abs(d["scan_age_s"] - 0.5) < 1e-12
    assert d["scans_seen"] == 10 and d["scans_published"] == 10
    assert abs(d["valid_count_mean"] - 10.5) < 1e-12
    assert abs(d["boundary_residual_mean_m"] - 0.05) < 1e-12  # NaN residuals are not averaged
    assert abs(d["intrusion_fraction_mean"] - 0.1) < 1e-12
    lat = d["latency_ms"]
    assert abs(lat["p50"] - np.percentile(np.arange(1.0, 11.0), 50)) < 1e-9
    assert lat["max"] == 10.0 and lat["p99"] <= lat["max"]
    assert d["payload_sha256"] == "abc"


def test_silence_after_the_last_scan() -> None:
    s = st.PerceptionStats("geometric", "geometric-v0", None)
    s.scan_seen(0.0)
    s.scan_published(track(5, 0.1, 0.0), 1.0)
    assert s.as_dict(0.5)["state"] == "running"
    assert s.as_dict(0.0 + st.SILENCE_S + 0.1)["state"] == "no_scan"


def test_latency_window_is_bounded_but_max_is_all_time() -> None:
    s = st.PerceptionStats("geometric", "geometric-v0", None)
    s.scan_seen(0.0)
    s.scan_published(track(1, math.nan, math.nan), 500.0)
    for _ in range(st.LATENCY_WINDOW + 5):
        s.scan_published(track(1, math.nan, math.nan), 1.0)
    d = s.as_dict(0.0)
    assert d["latency_ms"]["p99"] == 1.0 and d["latency_ms"]["max"] == 500.0
    assert d["boundary_residual_mean_m"] is None and d["intrusion_fraction_mean"] is None
