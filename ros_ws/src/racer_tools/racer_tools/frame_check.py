"""Pure statistics and pass/fail logic for the camera frame-rate check (camera_check).

No ROS, no hardware: everything here works on plain FrameSample records, so it is unit-tested
without rclpy (claude-docs/12-testing.md L1). racer_tools/camera_check_node.py is the thin ROS
plumbing that turns sensor_msgs/CompressedImage messages into FrameSample records.

Cameras are an optional subsystem, outside the project thesis (docs/notes/first-boot-runbook.md
"Cameras first power-up"). Nothing here is a physical constant: the rate threshold and the
expected resolution are check settings chosen by whoever runs the check, and camera mount poses
are not needed for a rate check, so none are read or assumed.

Resolution comes from the compressed bytes themselves: the JPEG SOF segment or the PNG IHDR
chunk, found by magic number rather than by trusting the message's free-text `format` field
(compressed_image_transport writes strings like "rgb8; jpeg compressed bgr8" there).

Units: seconds, hertz, bytes.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from itertools import pairwise

_JPEG_SOI = b"\xff\xd8"
_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"

#: JPEG start-of-frame markers that carry the frame size: SOF0..SOF15, except C4 (DHT),
#: C8 (JPG extension) and CC (DAC), which share the range but are not frame headers.
_JPEG_SOF_MARKERS = frozenset(range(0xC0, 0xD0)) - {0xC4, 0xC8, 0xCC}

#: Markers with no length field: TEM, RST0..RST7, SOI, EOI.
_JPEG_STANDALONE_MARKERS = frozenset({0x01, *range(0xD0, 0xD8), 0xD8, 0xD9})

_JPEG_EOI = 0xD9
_JPEG_SOS = 0xDA


def jpeg_dimensions(data: bytes) -> tuple[int, int] | None:
    """(width, height) from a JPEG's start-of-frame segment, or None if there is none.

    Walks the marker segments from SOI up to the first SOF; stops (None) at SOS or EOI, on a
    truncated segment, or on bytes that are not a marker where one must be.
    """
    if len(data) < 4 or data[:2] != _JPEG_SOI:
        return None
    i = 2
    n = len(data)
    while i < n:
        if data[i] != 0xFF:
            return None
        # Any number of 0xFF fill bytes may precede a marker code.
        while i < n and data[i] == 0xFF:
            i += 1
        if i >= n:
            return None
        marker = data[i]
        i += 1
        if marker in _JPEG_STANDALONE_MARKERS:
            if marker == _JPEG_EOI:
                return None
            continue
        if i + 2 > n:
            return None
        length = (data[i] << 8) | data[i + 1]
        if length < 2 or i + length > n:
            return None
        if marker in _JPEG_SOF_MARKERS:
            # Segment: length(2) precision(1) height(2) width(2) components(1) ...
            if length < 7:
                return None
            height = (data[i + 3] << 8) | data[i + 4]
            width = (data[i + 5] << 8) | data[i + 6]
            if width == 0 or height == 0:
                return None
            return width, height
        if marker == _JPEG_SOS:
            return None
        i += length
    return None


def png_dimensions(data: bytes) -> tuple[int, int] | None:
    """(width, height) from a PNG's IHDR chunk, or None if it is not a well-formed PNG header."""
    if len(data) < 24 or data[:8] != _PNG_SIGNATURE or data[12:16] != b"IHDR":
        return None
    width = int.from_bytes(data[16:20], "big")
    height = int.from_bytes(data[20:24], "big")
    if width == 0 or height == 0:
        return None
    return width, height


def image_dimensions(data: bytes) -> tuple[int, int] | None:
    """(width, height) of a JPEG or PNG payload, picked by magic number. None if neither."""
    if data[:2] == _JPEG_SOI:
        return jpeg_dimensions(data)
    if data[:8] == _PNG_SIGNATURE:
        return png_dimensions(data)
    return None


@dataclass(frozen=True)
class FrameSample:
    """The parts of one sensor_msgs/CompressedImage the check needs.

    stamp_s is the message header stamp; receipt_s is when this process received it (any
    monotonic clock). width and height are None when the payload could not be parsed.
    """

    stamp_s: float
    receipt_s: float
    size_bytes: int
    width: int | None
    height: int | None


@dataclass(frozen=True)
class FrameStats:
    """Summary of a run of frames."""

    frame_count: int
    duration_s: float
    rate_hz: float
    arrival_rate_hz: float
    max_gap_s: float
    non_increasing_stamps: int
    undecodable_frames: int
    resolutions: tuple[tuple[int, int], ...]
    mean_size_bytes: float
    bytes_per_s: float


@dataclass(frozen=True)
class Expectations:
    """What a healthy compressed image stream must look like.

    expected_width / expected_height None skips that check. min_rate_hz is a check threshold
    the operator picks against the fps the camera was launched with, not a camera property.
    """

    min_rate_hz: float
    expected_width: int | None = None
    expected_height: int | None = None


def _rate(times: Sequence[float]) -> float:
    """(N - 1) intervals over the time they span; 0 when the times do not advance."""
    span = times[-1] - times[0]
    return (len(times) - 1) / span if span > 0.0 else 0.0


def compute_stats(samples: Sequence[FrameSample]) -> FrameStats:
    """Summarise a run of frames, in arrival order. Needs at least two frames for a rate.

    rate_hz is from header stamps, so it counts only the frames that actually arrived here:
    frames dropped by the driver or by DDS lower it. arrival_rate_hz is the same count over
    receipt times, a cross-check that does not trust the driver's stamps. bytes_per_s is the
    compressed payload rate, which is what a rosbag of this topic grows by (plus a small
    per-message overhead).
    """
    if len(samples) < 2:
        raise ValueError(
            f"need at least 2 frames to measure a rate, got {len(samples)}; is the camera "
            "publishing?"
        )
    stamps = [s.stamp_s for s in samples]
    gaps = [b - a for a, b in pairwise(stamps)]
    duration_s = stamps[-1] - stamps[0]
    rate_hz = _rate(stamps)
    arrival_rate_hz = _rate([s.receipt_s for s in samples])

    resolutions = sorted(
        {(s.width, s.height) for s in samples if s.width is not None and s.height is not None}
    )
    undecodable = sum(1 for s in samples if s.width is None or s.height is None)
    mean_size = sum(s.size_bytes for s in samples) / len(samples)

    return FrameStats(
        frame_count=len(samples),
        duration_s=duration_s,
        rate_hz=rate_hz,
        arrival_rate_hz=arrival_rate_hz,
        max_gap_s=max(gaps),
        non_increasing_stamps=sum(1 for g in gaps if g <= 0.0),
        undecodable_frames=undecodable,
        resolutions=tuple(resolutions),
        mean_size_bytes=mean_size,
        bytes_per_s=mean_size * rate_hz,
    )


def evaluate(stats: FrameStats, expected: Expectations) -> list[str]:
    """Every way `stats` fails `expected`, as human-readable strings. Empty means pass."""
    failures: list[str] = []
    if stats.rate_hz < expected.min_rate_hz:
        failures.append(
            f"frame rate {stats.rate_hz:.2f} Hz is below the minimum {expected.min_rate_hz:.2f} Hz"
        )
    if stats.non_increasing_stamps:
        failures.append(
            f"{stats.non_increasing_stamps} frame(s) had a header stamp not after the previous one"
        )
    if stats.undecodable_frames:
        failures.append(
            f"{stats.undecodable_frames} frame(s) were not a JPEG or PNG with a readable size"
        )
    if len(stats.resolutions) > 1:
        sizes = ", ".join(f"{w}x{h}" for w, h in stats.resolutions)
        failures.append(f"resolution changed during the window: {sizes}")
    for w, h in stats.resolutions:
        if expected.expected_width is not None and w != expected.expected_width:
            failures.append(f"width {w} is not the expected {expected.expected_width}")
        if expected.expected_height is not None and h != expected.expected_height:
            failures.append(f"height {h} is not the expected {expected.expected_height}")
    return failures


def _fmt_resolutions(stats: FrameStats) -> str:
    if not stats.resolutions:
        return "unknown"
    return ", ".join(f"{w}x{h}" for w, h in stats.resolutions)


def _fmt_expected_resolution(expected: Expectations) -> str:
    w = "any" if expected.expected_width is None else str(expected.expected_width)
    h = "any" if expected.expected_height is None else str(expected.expected_height)
    return f"{w}x{h}"


def format_report(stats: FrameStats, expected: Expectations, failures: Sequence[str]) -> str:
    """Multi-line plain-text report for the terminal and for pasting into docs/notes."""
    mb_per_s = stats.bytes_per_s / 1e6
    lines = [
        f"frames received       {stats.frame_count} over {stats.duration_s:.3f} s",
        f"frame rate (stamps)   {stats.rate_hz:.2f} Hz (minimum {expected.min_rate_hz:.2f} Hz)",
        f"frame rate (arrival)  {stats.arrival_rate_hz:.2f} Hz",
        f"largest stamp gap     {stats.max_gap_s:.4f} s",
        (
            f"resolution            {_fmt_resolutions(stats)} "
            f"(expected {_fmt_expected_resolution(expected)})"
        ),
        f"undecodable frames    {stats.undecodable_frames}",
        f"mean frame size       {stats.mean_size_bytes / 1e3:.1f} kB",
        (
            f"payload rate          {mb_per_s:.2f} MB/s = {mb_per_s * 3.6:.1f} GB/h "
            "(bag growth for this topic)"
        ),
    ]
    if failures:
        lines.append("RESULT: FAIL")
        lines.extend(f"  - {failure}" for failure in failures)
    else:
        lines.append("RESULT: PASS")
    return "\n".join(lines)
