"""L1 tests for racer_perception.throttle and yolo_backend's pure helpers.

None of these need ROS, ultralytics, torch or opencv. The missing-package path is exercised by
asking for a backend in an environment where ultralytics cannot be imported (CI's).
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest
from racer_perception.throttle import FrameThrottle
from racer_perception.yolo_backend import (
    EXIT_DETECTOR_UNAVAILABLE,
    MISSING_MESSAGE,
    DetectorUnavailable,
    load_yolo_backend,
    resolve_model_path,
    validate_device,
)

# ----------------------------------------------------------------- throttle


def test_throttle_8_hz_over_64_hz_input_drops_seven_of_eight():
    # Powers of two so the timestamps are exact in floating point.
    t = FrameThrottle(8.0)
    accepted = [t.accept(i / 64.0) for i in range(65)]
    assert sum(accepted) == 9  # t = 0, 1/8, ... 1.0
    assert t.accepted == 9 and t.dropped == 56


def test_throttle_first_frame_is_always_accepted():
    assert FrameThrottle(1.0).accept(123456.0)


def test_throttle_slow_input_is_never_dropped():
    t = FrameThrottle(10.0)
    assert all(t.accept(i * 0.5) for i in range(10))
    assert t.dropped == 0


def test_throttle_non_positive_rate_means_no_limit():
    for rate in (0.0, -1.0):
        t = FrameThrottle(rate)
        assert all(t.accept(0.0) for _ in range(5))


def test_throttle_survives_a_clock_step_back():
    t = FrameThrottle(1.0)
    assert t.accept(1000.0)
    assert not t.accept(1000.5)
    assert t.accept(5.0)  # clock went backwards: restart instead of stalling for 995 s


@pytest.mark.parametrize("rate", [math.nan, math.inf])
def test_throttle_rejects_non_finite_rate(rate):
    with pytest.raises(ValueError):
        FrameThrottle(rate)


# ------------------------------------------------------------ backend helpers


@pytest.mark.parametrize("device", ["cpu", "cuda", "cuda:0", "cuda:12"])
def test_valid_devices(device):
    assert validate_device(device) == device


@pytest.mark.parametrize("device", ["", "gpu", "CUDA", "mps", "cuda:", "cuda:x", "cpu:0"])
def test_invalid_devices_are_refused(device):
    with pytest.raises(ValueError):
        validate_device(device)


def test_bare_model_name_lives_in_the_cache_dir():
    assert resolve_model_path("yolov8n.pt", "/var/cache/x") == Path("/var/cache/x/yolov8n.pt")


def test_model_with_a_directory_part_is_used_as_given():
    assert resolve_model_path("/models/a.pt", "/var/cache/x") == Path("/models/a.pt")
    assert resolve_model_path("weights/a.pt", "/var/cache/x") == Path("weights/a.pt")


def test_tilde_is_expanded():
    assert "~" not in str(resolve_model_path("yolov8n.pt", "~/.cache/racer_perception"))


def test_empty_model_name_is_refused():
    with pytest.raises(ValueError):
        resolve_model_path("", "/tmp")


def test_bad_device_surfaces_as_detector_unavailable():
    with pytest.raises(DetectorUnavailable, match="device"):
        load_yolo_backend("yolov8n.pt", "tpu", "/tmp/none", 0.25)


@pytest.mark.skipif(
    importlib.util.find_spec("ultralytics") is not None,
    reason="ultralytics is installed here; the missing-package path cannot be exercised",
)
def test_missing_ultralytics_is_a_clear_error():
    with pytest.raises(DetectorUnavailable) as err:
        load_yolo_backend("yolov8n.pt", "cpu", "/tmp/none", 0.25)
    assert "INSTALL_PERCEPTION=required" in str(err.value)
    assert "INSTALL_TORCH=required" in str(err.value)


def test_exit_code_is_non_zero_and_distinct():
    assert EXIT_DETECTOR_UNAVAILABLE not in (0, 1, 2)
    assert "perception.launch.py" in MISSING_MESSAGE
