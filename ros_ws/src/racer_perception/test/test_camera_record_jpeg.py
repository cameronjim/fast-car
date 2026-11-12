"""L1 tests for racer_perception.camera_record's JPEG half (jpeg_dimensions on real JPEGs, shrink_jpeg).

Skipped where OpenCV is not installed (the bare `uv run pytest` run); runs in the ros-dev image.
"""

from __future__ import annotations

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from racer_perception.camera_record import jpeg_dimensions, shrink_jpeg
from racer_perception.lap_frames import FrameDecodeError


def _texture(width: int, height: int, seed: int = 1) -> np.ndarray:
    rng = np.random.default_rng(seed)
    small = rng.uniform(0, 255, size=(max(height // 16, 2), max(width // 16, 2), 3))
    big = cv2.resize(small.astype(np.float32), (width, height), interpolation=cv2.INTER_CUBIC)
    return np.clip(big, 0, 255).astype(np.uint8)


def _jpeg(img: np.ndarray, quality: int = 90) -> bytes:
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    assert ok
    return buf.tobytes()


@pytest.mark.parametrize("size", [(1280, 720), (320, 240), (641, 361), (1920, 1200)])
def test_jpeg_dimensions_reads_the_frame_header(size):
    assert jpeg_dimensions(_jpeg(_texture(*size))) == size


def test_jpeg_dimensions_reads_a_progressive_jpeg():
    ok, buf = cv2.imencode(
        ".jpg", _texture(320, 240), [cv2.IMWRITE_JPEG_PROGRESSIVE, 1, cv2.IMWRITE_JPEG_QUALITY, 80]
    )
    assert ok
    assert jpeg_dimensions(buf.tobytes()) == (320, 240)


def test_jpeg_dimensions_accepts_bytearray_and_memoryview():
    data = _jpeg(_texture(320, 240))
    assert jpeg_dimensions(bytearray(data)) == (320, 240)
    assert jpeg_dimensions(memoryview(data)) == (320, 240)


# ------------------------------------------------------------------------------ shrink_jpeg


def _decoded_size(data: bytes) -> tuple[int, int]:
    img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    return img.shape[1], img.shape[0]


def test_shrink_makes_a_640x360_jpeg_from_720p_and_it_is_much_smaller():
    src = _jpeg(_texture(1280, 720), 80)
    out, size = shrink_jpeg(src, 640, 360, 60)
    assert size == (640, 360)
    assert _decoded_size(out) == (640, 360)
    assert len(out) < len(src) / 2


def test_shrink_fits_a_different_aspect_inside_the_box():
    out, size = shrink_jpeg(_jpeg(_texture(1920, 1200)), 640, 360, 60)
    assert size == (576, 360)
    assert _decoded_size(out) == (576, 360)


def test_shrink_never_upscales():
    out, size = shrink_jpeg(_jpeg(_texture(320, 240)), 640, 360, 60)
    assert size == (320, 240)
    assert _decoded_size(out) == (320, 240)


def test_shrink_quality_sets_the_size():
    src = _jpeg(_texture(1280, 720), 90)
    low, _ = shrink_jpeg(src, 640, 360, 20)
    high, _ = shrink_jpeg(src, 640, 360, 95)
    assert len(low) < len(high)


def test_shrink_keeps_the_picture():
    img = np.zeros((720, 1280, 3), dtype=np.uint8)
    img[:, :640] = (255, 0, 0)  # blue left half (BGR), black right half
    out, _ = shrink_jpeg(_jpeg(img, 95), 640, 360, 90)
    decoded = cv2.imdecode(np.frombuffer(out, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert decoded[180, 100, 0] > 200 and decoded[180, 100, 2] < 50
    assert decoded[180, 540].max() < 50


def test_shrink_by_the_reduced_decode_and_by_resize_agree_on_size_for_odd_sizes():
    # 641 x 361 -> fit 320 x 180: a reduction of 2 gives 321 x 181, which is resized down.
    out, size = shrink_jpeg(_jpeg(_texture(641, 361)), 320, 180, 60)
    assert size == (320, 180)
    assert _decoded_size(out) == (320, 180)


@pytest.mark.parametrize("junk", [b"", b"junk", b"\xff\xd8\xff\xd9"])
def test_shrink_refuses_junk(junk):
    with pytest.raises(FrameDecodeError):
        shrink_jpeg(junk, 640, 360, 60)


def test_shrink_refuses_a_header_with_no_decodable_data():
    good = _jpeg(_texture(640, 360))
    with pytest.raises(FrameDecodeError):
        shrink_jpeg(good[:60], 640, 360, 60)


@pytest.mark.parametrize("quality", [0, 101, -5])
def test_shrink_refuses_a_bad_quality(quality):
    with pytest.raises(ValueError, match="quality"):
        shrink_jpeg(_jpeg(_texture(320, 240)), 640, 360, quality)
