"""L1 tests for racer_perception.lap_frames (JPEG decode with OpenCV).

Skipped where OpenCV is not installed (the bare `uv run pytest` run); runs in the ros-dev image.
"""

from __future__ import annotations

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from racer_perception.lap_counter import LapConfig
from racer_perception.lap_frames import FrameDecodeError, decode_gray


def _jpeg(img: np.ndarray, quality: int = 90) -> bytes:
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    assert ok
    return buf.tobytes()


def _texture(seed: int, width: int = 320, height: int = 240) -> np.ndarray:
    rng = np.random.default_rng(seed)
    small = rng.uniform(0, 255, size=(height // 16, width // 16)).astype(np.float32)
    big = cv2.resize(small, (width, height), interpolation=cv2.INTER_CUBIC)
    return np.clip(big, 0, 255).astype(np.uint8)


def test_decode_gray_reductions_and_types():
    data = _jpeg(_texture(1, 320, 240))
    assert decode_gray(data, 1).shape == (240, 320)
    assert decode_gray(data, 2).shape == (120, 160)
    assert decode_gray(data, 4).shape == (60, 80)
    assert decode_gray(data, 8).shape == (30, 40)
    assert decode_gray(data, 4).dtype == np.uint8
    assert decode_gray(bytearray(data), 4).shape == (60, 80)
    assert decode_gray(np.frombuffer(data, dtype=np.uint8), 4).shape == (60, 80)


def test_decode_gray_accepts_colour_jpeg_and_returns_one_channel():
    colour = cv2.cvtColor(_texture(2), cv2.COLOR_GRAY2BGR)
    assert decode_gray(_jpeg(colour), 2).ndim == 2


def test_decode_gray_refuses_junk_and_bad_reduction():
    with pytest.raises(FrameDecodeError):
        decode_gray(b"not-really-a-jpeg", 4)
    with pytest.raises(FrameDecodeError):
        decode_gray(b"", 4)
    with pytest.raises(ValueError, match="reduction"):
        decode_gray(_jpeg(_texture(1)), 3)


def test_the_same_texture_through_jpeg_is_still_the_same_place():
    from racer_perception.lap_counter import make_signature

    cfg = LapConfig()
    a = make_signature(decode_gray(_jpeg(_texture(3), 95), 2), cfg)
    b = make_signature(decode_gray(_jpeg(_texture(3), 40), 2), cfg)
    c = make_signature(decode_gray(_jpeg(_texture(4), 95), 2), cfg)
    assert float(a @ b) > 0.95
    assert abs(float(a @ c)) < 0.5
