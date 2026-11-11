"""JPEG bytes -> small gray image for the lap counter. The only OpenCV use in the lap counter.

cv2.IMREAD_REDUCED_GRAYSCALE_N makes libjpeg decode at 1/N scale straight from the DCT
coefficients, which costs a fraction of a full decode: a 1280 x 720 frame at N = 4 comes out as
320 x 180 gray, far more than the 64 x 48 thumbnail needs. Shared by lap_counter_node and the
tools/lap_replay.py tuning tool so both see exactly the same pixels.
"""

from __future__ import annotations

import numpy as np

#: cv2 flag name for each supported reduction factor.
_FLAGS = {
    1: "IMREAD_GRAYSCALE",
    2: "IMREAD_REDUCED_GRAYSCALE_2",
    4: "IMREAD_REDUCED_GRAYSCALE_4",
    8: "IMREAD_REDUCED_GRAYSCALE_8",
}
REDUCTIONS = tuple(_FLAGS)


class FrameDecodeError(RuntimeError):
    pass


def cv2_available() -> bool:
    try:
        import cv2  # noqa: F401
    except ImportError:
        return False
    return True


def decode_gray(
    data: bytes | bytearray | memoryview | np.ndarray, reduction: int = 4
) -> np.ndarray:
    """Decode a JPEG/PNG to a 2-D uint8 gray image, reduced by `reduction` (1, 2, 4 or 8)."""
    if reduction not in _FLAGS:
        raise ValueError(f"decode reduction must be one of {REDUCTIONS}, got {reduction}")
    import cv2  # lazy: pure-Python tests and tools without OpenCV import this module

    buf = data if isinstance(data, np.ndarray) else np.frombuffer(data, dtype=np.uint8)
    try:
        img = cv2.imdecode(buf, getattr(cv2, _FLAGS[reduction]))
    except cv2.error as exc:  # an empty buffer raises in OpenCV 5, returns None in 4
        raise FrameDecodeError(f"not a decodable image ({exc.__class__.__name__})") from exc
    if img is None or img.ndim != 2:
        raise FrameDecodeError("not a decodable image")
    return img
