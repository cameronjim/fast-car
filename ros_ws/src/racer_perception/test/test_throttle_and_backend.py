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


def _source(fps: float, seconds: float, jitter_s: float, seed: int) -> list[float]:
    """Arrival times of a camera at `fps` with Gaussian-free bounded jitter (never reordered)."""
    import random

    rng = random.Random(seed)
    n = int(fps * seconds)
    times = [i / fps + rng.uniform(-jitter_s, jitter_s) for i in range(n)]
    return sorted(times)


@pytest.mark.parametrize("fps", [30.0, 42.5, 60.0])
def test_throttle_delivers_the_configured_rate_for_a_fast_enough_source(fps):
    # S1: the delivered rate equals rate_hz (30), not a divisor of the source rate.
    seconds = 60.0
    t = FrameThrottle(30.0)
    arrivals = _source(fps, seconds, jitter_s=0.004, seed=7)
    accepted = sum(t.accept(a) for a in arrivals)
    assert accepted / seconds == pytest.approx(30.0, abs=0.2)


def test_throttle_passes_a_30_fps_source_at_30_hz_with_jitter_nearly_whole():
    t = FrameThrottle(30.0)
    arrivals = _source(30.0, 60.0, jitter_s=0.004, seed=3)
    assert sum(t.accept(a) for a in arrivals) >= 0.99 * len(arrivals)


@pytest.mark.parametrize("fps", [30.0, 42.5, 60.0, 90.0])
@pytest.mark.parametrize("rate", [15.0, 30.0])
@pytest.mark.parametrize("seed", range(5))
def test_throttle_property_bounded_above_and_below(fps, rate, seed):
    seconds = 20.0
    t = FrameThrottle(rate)
    arrivals = _source(fps, seconds, jitter_s=0.3 / fps, seed=seed)
    accepted = [a for a in arrivals if t.accept(a)]
    span = arrivals[-1] - arrivals[0]
    if fps >= rate:
        # never more than rate_hz (plus the one frame at the start), never less than one frame short
        assert len(accepted) <= rate * span + 1.0
        assert len(accepted) >= rate * span - 1.0 - 1e-9
    else:
        assert len(accepted) == len(arrivals)  # a slower source is never dropped


def test_throttle_phase_is_kept_not_reset_to_the_accepted_frame():
    # Frames every 0.01 s at 10 Hz: accepted at 0.00, 0.10, 0.20 ... exactly (phase kept).
    t = FrameThrottle(10.0)
    accepted = [i for i in range(101) if t.accept(i * 0.01)]
    assert accepted == list(range(0, 101, 10))


def test_throttle_a_late_accept_does_not_push_later_ones_late():
    t = FrameThrottle(10.0)
    assert t.accept(0.0)
    assert t.accept(0.19)  # one late frame (due 0.10)
    assert t.accept(0.20)  # next one is due on the 0.2 grid, not at 0.29


def test_throttle_does_not_burst_after_a_long_gap():
    t = FrameThrottle(10.0)
    assert t.accept(0.0)
    assert t.accept(50.0)  # camera stalled for 50 s
    burst = [t.accept(50.0 + i * 0.001) for i in range(1, 50)]
    assert not any(burst)  # no catch-up flood: one frame per period again


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
