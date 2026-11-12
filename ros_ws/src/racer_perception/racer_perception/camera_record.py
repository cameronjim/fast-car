"""Pure helpers for camera_record_node: when to record, how big to record.

camera_record_node makes the stream that goes into the run's bag (the recorder regex takes
/camera/<name>/record/compressed and no longer takes the full-rate image_raw/compressed). Three
decisions live here so they are tested without ROS:

  * ActivityGate: record only while something has published /drive_raw recently, i.e. while a
    driving launch is up. An idle stack with the cameras on records no camera data at all.
  * plan_output_size / choose_reduction: fit the frame inside a maximum size without ever
    upscaling, and let libjpeg decode at 1/2, 1/4 or 1/8 scale straight from the DCT
    coefficients when that is still at least the output size (a fraction of a full decode).
  * shrink_jpeg: the JPEG bytes in, smaller JPEG bytes out (OpenCV, imported lazily).

Nothing here touches the command path.
"""

from __future__ import annotations

import math
import struct

from racer_perception.lap_frames import FrameDecodeError

#: JPEG start-of-frame markers (baseline, extended, progressive, ...). Not DHT/JPG/DAC (C4, C8, CC).
_SOF_MARKERS = frozenset(range(0xC0, 0xD0)) - {0xC4, 0xC8, 0xCC}
#: Decode scale factors OpenCV's IMREAD_REDUCED_COLOR_N supports.
REDUCTIONS = (1, 2, 4, 8)


class ActivityGate:
    """Open while the last activity was less than `hold_s` ago. `hold_s` <= 0 means never open.

    Uses the times the caller passes (the node's clock). A clock that steps backwards restarts
    the hold from the new time rather than holding the gate open for however far it stepped back.
    """

    def __init__(self, hold_s: float) -> None:
        if not math.isfinite(hold_s):
            raise ValueError(f"hold_s must be finite, got {hold_s}")
        self.hold_s = hold_s
        self._last_s = -math.inf

    def note(self, now_s: float) -> None:
        """Something that counts as a drive was seen at now_s."""
        self._last_s = now_s

    def is_open(self, now_s: float) -> bool:
        self._last_s = min(self._last_s, now_s)
        return now_s - self._last_s < self.hold_s


def jpeg_dimensions(data: bytes | bytearray | memoryview) -> tuple[int, int] | None:
    """(width, height) from the first start-of-frame marker, or None if it is not a JPEG."""
    buf = bytes(data)
    if len(buf) < 4 or buf[0] != 0xFF or buf[1] != 0xD8:
        return None
    i = 2
    while i + 4 <= len(buf):
        if buf[i] != 0xFF:
            return None
        marker = buf[i + 1]
        if marker == 0xFF:  # fill byte
            i += 1
            continue
        if marker in _SOF_MARKERS:
            if i + 9 > len(buf):
                return None
            height, width = struct.unpack(">HH", buf[i + 5 : i + 9])
            return (width, height) if width > 0 and height > 0 else None
        if marker == 0x01 or 0xD0 <= marker <= 0xD8:  # standalone markers, no length field
            i += 2
            continue
        if marker in (0xD9, 0xDA):  # end of image / start of scan before any SOF
            return None
        (length,) = struct.unpack(">H", buf[i + 2 : i + 4])
        i += 2 + length
    return None


def plan_output_size(src_w: int, src_h: int, max_w: int, max_h: int) -> tuple[int, int]:
    """Largest size that fits in max_w x max_h with the source aspect ratio, never upscaled."""
    if min(src_w, src_h, max_w, max_h) <= 0:
        raise ValueError(f"sizes must be positive: source {src_w}x{src_h}, max {max_w}x{max_h}")
    scale = min(1.0, max_w / src_w, max_h / src_h)
    return max(1, round(src_w * scale)), max(1, round(src_h * scale))


def choose_reduction(src_w: int, src_h: int, out_w: int, out_h: int) -> int:
    """Largest of 1, 2, 4, 8 whose reduced decode (ceil(src / n)) is still >= the output size."""
    best = 1
    for n in REDUCTIONS:
        if -(-src_w // n) >= out_w and -(-src_h // n) >= out_h:
            best = n
    return best


def shrink_jpeg(
    data: bytes | bytearray | memoryview, max_w: int, max_h: int, quality: int
) -> tuple[bytes, tuple[int, int]]:
    """JPEG in, smaller JPEG out: (bytes, (width, height)). Raises FrameDecodeError on junk."""
    if not 1 <= quality <= 100:
        raise ValueError(f"jpeg quality must be 1..100, got {quality}")
    import cv2  # lazy: the pure tests and tools without OpenCV import this module
    import numpy as np

    dims = jpeg_dimensions(data)
    if dims is None:
        raise FrameDecodeError("not a JPEG")
    out_w, out_h = plan_output_size(dims[0], dims[1], max_w, max_h)
    reduction = choose_reduction(dims[0], dims[1], out_w, out_h)
    flag = {
        1: cv2.IMREAD_COLOR,
        2: cv2.IMREAD_REDUCED_COLOR_2,
        4: cv2.IMREAD_REDUCED_COLOR_4,
        8: cv2.IMREAD_REDUCED_COLOR_8,
    }[reduction]
    try:
        img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), flag)
    except cv2.error as exc:
        raise FrameDecodeError(f"not a decodable image ({exc.__class__.__name__})") from exc
    if img is None:
        raise FrameDecodeError("not a decodable image")
    if img.shape[1] != out_w or img.shape[0] != out_h:
        img = cv2.resize(img, (out_w, out_h), interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise FrameDecodeError("JPEG encode failed")
    return encoded.tobytes(), (out_w, out_h)
