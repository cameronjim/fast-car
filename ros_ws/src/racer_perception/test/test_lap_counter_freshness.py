"""L1: what the lap counter says about its own inputs (outside review of brief 51c, finding 1).

The counter republishes `/lap/status` on a timer even when no image arrives, so a status that says
`running` with similarity 1.0 can be a camera that stopped. `LapCounter.freshness(now)` adds the age
of the last frame, an image sequence number that only advances with processed frames, the age of the
wheel odometry and the reset count. It is separate from `status()`, whose keys are pinned."""

from __future__ import annotations

import json
import math

import numpy as np
import pytest
from racer_perception.lap_counter import LapConfig, LapCounter

EPS = 1e-6


def frame(seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.uniform(0, 255, size=(48, 64)).astype(np.float32)


def counter(**kw) -> LapCounter:
    return LapCounter(LapConfig(start_speed_mps=0.0, **kw))


def test_the_default_stale_limit_is_one_second_and_must_be_positive_and_finite() -> None:
    assert LapConfig().frame_stale_s == 1.0
    for bad in (0.0, -1.0, math.nan, math.inf):
        with pytest.raises(ValueError):
            LapConfig(frame_stale_s=bad).validate()


def test_before_any_frame_there_is_no_camera() -> None:
    f = counter().freshness(10.0)
    assert f["camera"] == "no_frames" and f["camera_fresh"] is False
    assert f["frame_seq"] == 0 and f["frame_age_s"] is None
    assert f["frames_since_reset"] == 0 and f["reset_id"] == 0
    assert f["odom_fresh"] is False and f["odom_age_s"] is None


def test_each_processed_frame_advances_the_sequence_and_resets_the_age() -> None:
    c = counter()
    c.update_frame(1.0, frame(1))
    c.update_frame(1.1, frame(2))
    f = c.freshness(1.1)
    assert f["frame_seq"] == 2 and f["frame_age_s"] == pytest.approx(0.0, abs=1e-9)
    assert f["camera"] == "fresh" and f["camera_fresh"] is True
    f = c.freshness(1.5)
    assert f["frame_seq"] == 2 and f["frame_age_s"] == pytest.approx(0.4)


@pytest.mark.parametrize(
    ("age", "fresh"), [(0.999, True), (1.0, True), (1.0 + EPS, False), (30.0, False)]
)
def test_a_frame_older_than_the_limit_is_a_stale_camera(age: float, fresh: bool) -> None:
    c = counter()
    c.update_frame(5.0, frame(1))
    f = c.freshness(5.0 + age)
    assert f["camera_fresh"] is fresh
    assert f["camera"] == ("fresh" if fresh else "stale")


def test_the_status_alone_does_not_advance_the_sequence_the_process_stays_alive() -> None:
    # one frame, then nothing: only time passes (the node republishes its status on a timer)
    c = counter()
    c.update_frame(1.0, frame(1))
    seqs = {c.freshness(1.0 + k)["frame_seq"] for k in range(10)}
    assert seqs == {1}
    assert c.freshness(20.0)["camera"] == "stale"
    assert c.status()["state"] == "running"  # the old keys still say running


def test_a_non_finite_frame_time_is_not_a_frame() -> None:
    c = counter()
    c.update_frame(math.nan, frame(1))
    assert c.freshness(1.0)["frame_seq"] == 0


def test_a_reset_counts_and_clears_frames_since_reset_but_not_the_sequence() -> None:
    c = counter()
    c.update_frame(1.0, frame(1))
    c.update_frame(1.1, frame(2))
    assert c.freshness(1.1)["reset_id"] == 0
    c.reset(capture_next_frame=True)
    f = c.freshness(1.2)
    assert f["reset_id"] == 1 and f["frames_since_reset"] == 0 and f["frame_seq"] == 2
    c.update_frame(1.3, frame(3))
    f = c.freshness(1.3)
    assert f["frames_since_reset"] == 1 and f["frame_seq"] == 3
    assert c.status()["state"] == "running"  # the first frame after the reset is the reference
    c.reset(capture_next_frame=True)
    assert c.freshness(1.4)["reset_id"] == 2


@pytest.mark.parametrize(("age", "fresh"), [(0.5, True), (1.0, True), (1.0 + EPS, False)])
def test_the_odometry_has_an_age_too(age: float, fresh: bool) -> None:
    c = counter()
    c.update_odom(2.0, 0.5, 0.0)
    f = c.freshness(2.0 + age)
    assert f["odom_fresh"] is fresh and f["odom_age_s"] == pytest.approx(age)


def test_freshness_is_json_safe_and_has_exactly_these_keys() -> None:
    c = counter()
    c.update_frame(1.0, frame(1))
    c.update_odom(1.0, 0.3, 0.0)
    f = json.loads(json.dumps(c.freshness(1.2), allow_nan=False))
    assert set(f) == {
        "camera",
        "camera_fresh",
        "frame_seq",
        "frame_age_s",
        "frames_since_reset",
        "reset_id",
        "odom_age_s",
        "odom_fresh",
    }
    json.dumps(counter().freshness(0.0), allow_nan=False)
