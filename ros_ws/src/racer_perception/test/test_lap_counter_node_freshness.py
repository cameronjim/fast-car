"""L3: lap_counter_node says when its camera is stale (outside review of brief 51c, finding 1) and
which reset a status belongs to (finding 2). The harness is test_lap_counter_node.py's."""

from __future__ import annotations

import pytest

pytest.importorskip("cv2")
pytest.importorskip("rclpy")
pytest.importorskip("nav_msgs")
pytest.importorskip("std_srvs")

from std_srvs.srv import Trigger
from test_lap_counter_node import _NO_GATES, _texture, make_harness  # noqa: F401

FAST = {**_NO_GATES, "frame_stale_s": 0.5}


def test_the_status_carries_the_frame_sequence_and_a_fresh_camera(make_harness):  # noqa: F811
    h = make_harness(141, **FAST)
    assert h.connected()
    h.odom(0.5, 0.0)
    h.spin_for(0.1)
    for i in range(4):
        h.frame(_texture(i), settle_s=0.05)
    assert h.spin_until(lambda: h.statuses and h.statuses[-1].get("frame_seq", 0) >= 4)
    status = h.statuses[-1]
    assert status["camera_fresh"] is True and status["camera"] == "fresh"
    assert status["frame_seq"] >= 4 and status["frame_age_s"] < 0.5
    assert status["odom_fresh"] is True


def test_one_frame_then_no_images_with_the_process_alive_goes_stale(make_harness):  # noqa: F811
    """The finding: the status timer keeps publishing `running` and a similarity while nothing
    arrives. Now the status says the camera is stale and the sequence does not move."""
    h = make_harness(142, **FAST)
    assert h.connected()
    h.odom(0.5, 0.0)
    h.spin_for(0.1)
    h.frame(_texture(1), settle_s=0.1)  # the reference: running
    assert h.spin_until(lambda: h.statuses and h.statuses[-1]["state"] == "running")
    seq = h.statuses[-1]["frame_seq"]
    assert h.spin_until(lambda: h.statuses[-1].get("camera_fresh") is False, timeout_s=5.0)
    n = len(h.statuses)
    assert h.spin_until(lambda: len(h.statuses) >= n + 3)  # it is alive and keeps publishing
    last = h.statuses[-1]
    assert last["state"] == "running"  # the old keys still say so: that is why they are not enough
    assert last["camera"] == "stale" and last["frame_seq"] == seq
    assert last["frame_age_s"] > 0.5


def test_a_camera_that_comes_back_is_fresh_again_and_the_sequence_moves(make_harness):  # noqa: F811
    h = make_harness(143, **FAST)
    assert h.connected()
    h.odom(0.5, 0.0)
    h.spin_for(0.1)
    h.frame(_texture(1), settle_s=0.1)
    assert h.spin_until(lambda: h.statuses[-1].get("camera_fresh") is False, timeout_s=5.0)
    seq = h.statuses[-1]["frame_seq"]
    h.frame(_texture(2), settle_s=0.05)
    assert h.spin_until(lambda: h.statuses[-1].get("camera_fresh") is True, timeout_s=5.0)
    assert h.statuses[-1]["frame_seq"] > seq


def test_the_reset_service_says_its_id_and_the_status_carries_it(make_harness):  # noqa: F811
    h = make_harness(144, **FAST)
    assert h.connected()
    h.odom(0.0, 0.0)
    h.spin_for(0.1)
    client = h.helper.create_client(Trigger, "/lap_counter/reset")
    assert client.wait_for_service(timeout_sec=5.0)
    for expected in (1, 2):
        future = client.call_async(Trigger.Request())
        assert h.spin_until(future.done)
        assert future.result().success and f"reset_id={expected}" in future.result().message
        assert h.spin_until(lambda e=expected: h.statuses[-1].get("reset_id") == e)
    h.frame(_texture(5), settle_s=0.1)
    assert h.spin_until(lambda: h.statuses[-1].get("frames_since_reset") == 1)
