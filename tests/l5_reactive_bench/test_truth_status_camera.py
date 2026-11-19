"""L1 (outside review of brief 51c): the truth oracle speaks the camera counter's freshness keys, so
that the simulator exercises the same rule the car does: a frame sequence that advances with the
ground-truth samples, a fresh camera and wheel odometry, one reset."""

from __future__ import annotations

import json

from truth_lap_source import TruthLapSource


def test_the_oracle_status_has_the_camera_keys_and_the_sequence_follows_the_samples() -> None:
    src = TruthLapSource()
    first = src.status()
    assert first["frame_seq"] == 0 and first["camera_fresh"] is True
    assert first["odom_fresh"] is True and first["reset_id"] == 1
    for k in range(5):
        src.on_truth_odom(0.02 * k, 0.0, 0.0, 0.0)
    status = src.status()
    assert status["frame_seq"] == 5 and status["frames_since_reset"] == 5
    assert status["camera"] == "fresh" and status["frame_age_s"] == 0.0
    json.dumps(status, allow_nan=False)


def test_the_old_keys_are_unchanged() -> None:
    status = TruthLapSource().status()
    for key in ("state", "similarity", "laps", "armed", "speed_mps", "distance_m"):
        assert key in status
    assert status["state"] == "running" and status["similarity"] == 1.0
