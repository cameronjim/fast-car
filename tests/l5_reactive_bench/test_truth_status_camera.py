"""L1 (outside review of brief 51c): the oracle's status publisher speaks the camera counter's
freshness keys, so that the simulator exercises the same rule the car does: a frame sequence that
advances with the ground-truth samples in every phase (the main loop is not running while the start
gate waits), a fresh camera and wheel odometry, one reset."""

from __future__ import annotations

import json

import track_family_runner as runner


def status() -> dict:
    return {"state": "running", "similarity": 1.0, "laps": 0, "armed": False}


def test_the_status_has_the_camera_keys_and_the_sequence_follows_the_samples() -> None:
    published: list[dict] = []
    pub = runner.OracleStatusPublisher(status, published.append)
    for k in range(250):  # 5 s of 50 Hz samples, no planner, no main loop
        pub.on_time(0.02 * k, now_s=0.02 * k, lap_first_status_s=None)
    assert len(published) == 5
    seqs = [p["frame_seq"] for p in published]
    assert seqs == sorted(set(seqs)) and seqs[0] == 1 and seqs[-1] > seqs[0]
    last = published[-1]
    assert last["camera_fresh"] is True and last["odom_fresh"] is True and last["reset_id"] == 1
    assert last["camera"] == "fresh" and last["frame_age_s"] == 0.0
    json.dumps(last, allow_nan=False)


def test_the_old_keys_are_unchanged_and_the_hold_still_applies() -> None:
    published: list[dict] = []
    pub = runner.OracleStatusPublisher(status, published.append, ready_after_s=6.0)
    pub.on_time(0.0, now_s=0.0, lap_first_status_s=None)
    body = published[0]
    assert body["state"] == "waiting" and body["similarity"] == 0.0 and body["laps"] == 0
    assert body["frame_seq"] == 1  # the camera keys are there while the source says not ready
