"""L1 tests for racer_tools/frame_check.py (optional camera bring-up): pure logic, no ROS.

JPEG and PNG headers are built by hand here, byte by byte, so the size parsing is pinned
against the file formats and not against whatever an encoder library happens to emit. Rates
and sizes are arbitrary hand-computed numbers, not any particular camera's.
"""

from __future__ import annotations

import struct

import pytest
from racer_tools.frame_check import (
    Expectations,
    FrameSample,
    compute_stats,
    evaluate,
    format_report,
    image_dimensions,
    jpeg_dimensions,
    png_dimensions,
)


def _segment(marker: int, payload: bytes) -> bytes:
    return bytes([0xFF, marker]) + struct.pack(">H", len(payload) + 2) + payload


def _sof(width: int, height: int, marker: int = 0xC0) -> bytes:
    # precision 8, height, width, 3 components (id, sampling, table) each
    payload = struct.pack(">BHHB", 8, height, width, 3) + bytes(9)
    return _segment(marker, payload)


def _jpeg(width: int, height: int, marker: int = 0xC0, before: bytes = b"") -> bytes:
    app0 = _segment(0xE0, b"JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00")
    sos = _segment(0xDA, bytes(10))
    return b"\xff\xd8" + app0 + before + _sof(width, height, marker) + sos + b"\x12\x34\xff\xd9"


def _png(width: int, height: int) -> bytes:
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + struct.pack(">I", 13) + b"IHDR" + ihdr + bytes(4)


# --- jpeg_dimensions --------------------------------------------------------------------


def test_jpeg_baseline_sof0():
    assert jpeg_dimensions(_jpeg(1280, 720)) == (1280, 720)


def test_jpeg_progressive_sof2():
    assert jpeg_dimensions(_jpeg(1920, 1200, marker=0xC2)) == (1920, 1200)


@pytest.mark.parametrize("not_a_frame_marker", [0xC4, 0xC8, 0xCC])
def test_jpeg_skips_markers_inside_the_sof_range_that_are_not_frame_headers(not_a_frame_marker):
    decoy = _segment(not_a_frame_marker, struct.pack(">BHHB", 8, 1, 1, 1))
    assert jpeg_dimensions(_jpeg(640, 480, before=decoy)) == (640, 480)


def test_jpeg_skips_dqt_and_dht_before_sof():
    tables = _segment(0xDB, bytes(65)) + _segment(0xC4, bytes(29))
    assert jpeg_dimensions(_jpeg(320, 240, before=tables)) == (320, 240)


def test_jpeg_fill_bytes_before_a_marker():
    data = _jpeg(800, 600)
    # Insert two 0xFF fill bytes in front of the APP0 marker.
    data = data[:2] + b"\xff\xff" + data[2:]
    assert jpeg_dimensions(data) == (800, 600)


def test_jpeg_standalone_restart_marker_has_no_length():
    data = b"\xff\xd8" + b"\xff\xd0" + _sof(64, 48) + b"\xff\xd9"
    assert jpeg_dimensions(data) == (64, 48)


@pytest.mark.parametrize(
    "data",
    [
        b"",
        b"\xff\xd8",
        b"\x00\x00\x00\x00",
        b"\xff\xd8\xff\xd9",  # SOI then EOI, no frame
        b"\xff\xd8\xff\xda\x00\x04\x00\x00",  # SOS before any SOF
        b"\xff\xd8\x00\xe0\x00\x10",  # not a marker where one must be
        b"\xff\xd8\xff\xe0\x00",  # length cut off
        b"\xff\xd8\xff\xe0\x00\x10\x00",  # segment runs past the end
        b"\xff\xd8\xff\xe0\x00\x01",  # length below its own two bytes
        b"\xff\xd8\xff\xc0\x00\x05\x08\x00\x10",  # SOF too short to hold a size
        b"\xff\xd8\xff\xff",  # only fill bytes after SOI
    ],
)
def test_jpeg_malformed_is_none(data):
    assert jpeg_dimensions(data) is None


def test_jpeg_zero_size_is_none():
    assert jpeg_dimensions(_jpeg(0, 720)) is None
    assert jpeg_dimensions(_jpeg(1280, 0)) is None


def test_jpeg_truncated_before_sof_is_none():
    data = _jpeg(1280, 720)
    sof_at = data.index(b"\xff\xc0")
    assert jpeg_dimensions(data[:sof_at]) is None


# --- png_dimensions / image_dimensions ---------------------------------------------------


def test_png_ihdr():
    assert png_dimensions(_png(1280, 720)) == (1280, 720)


@pytest.mark.parametrize(
    "data",
    [
        b"",
        _png(1280, 720)[:23],
        b"\x89PNG\r\n\x1a\n" + struct.pack(">I", 13) + b"IDAT" + bytes(17),
        _png(0, 720),
        _png(1280, 0),
    ],
)
def test_png_malformed_is_none(data):
    assert png_dimensions(data) is None


def test_image_dimensions_picks_by_magic_number():
    assert image_dimensions(_jpeg(1280, 720)) == (1280, 720)
    assert image_dimensions(_png(640, 480)) == (640, 480)
    assert image_dimensions(b"GIF89a" + bytes(20)) is None
    assert image_dimensions(b"") is None


# --- compute_stats ----------------------------------------------------------------------


def _frames(n, period_s, size=1000, width=1280, height=720, t0=100.0, receipt_period_s=None):
    receipt_period_s = period_s if receipt_period_s is None else receipt_period_s
    return [
        FrameSample(
            stamp_s=t0 + i * period_s,
            receipt_s=5.0 + i * receipt_period_s,
            size_bytes=size,
            width=width,
            height=height,
        )
        for i in range(n)
    ]


def test_stats_rate_from_stamps_and_from_arrival():
    stats = compute_stats(_frames(11, 0.02, receipt_period_s=0.025))
    assert stats.frame_count == 11
    assert stats.duration_s == pytest.approx(0.2)
    assert stats.rate_hz == pytest.approx(50.0)
    assert stats.arrival_rate_hz == pytest.approx(40.0)
    assert stats.max_gap_s == pytest.approx(0.02)
    assert stats.non_increasing_stamps == 0
    assert stats.undecodable_frames == 0
    assert stats.resolutions == ((1280, 720),)


def test_stats_payload_rate_is_mean_size_times_rate():
    frames = _frames(5, 0.1)
    frames = [
        FrameSample(f.stamp_s, f.receipt_s, size, f.width, f.height)
        for f, size in zip(frames, [100, 200, 300, 400, 500], strict=True)
    ]
    stats = compute_stats(frames)
    assert stats.mean_size_bytes == pytest.approx(300.0)
    assert stats.rate_hz == pytest.approx(10.0)
    assert stats.bytes_per_s == pytest.approx(3000.0)


def test_stats_dropped_frame_shows_in_rate_and_gap():
    frames = _frames(11, 0.02)
    del frames[5]
    stats = compute_stats(frames)
    assert stats.rate_hz == pytest.approx(9 / 0.2)
    assert stats.max_gap_s == pytest.approx(0.04)


def test_stats_counts_non_increasing_stamps():
    frames = _frames(4, 0.1)
    frames[2] = FrameSample(frames[1].stamp_s, 0.0, 10, 1280, 720)  # repeat
    frames[3] = FrameSample(frames[0].stamp_s, 0.0, 10, 1280, 720)  # backwards
    assert compute_stats(frames).non_increasing_stamps == 2


def test_stats_frozen_stamps_rate_is_zero():
    frames = _frames(3, 0.0)
    stats = compute_stats(frames)
    assert stats.rate_hz == 0.0
    assert stats.bytes_per_s == 0.0


def test_stats_undecodable_and_mixed_resolutions():
    frames = _frames(4, 0.1)
    frames[1] = FrameSample(frames[1].stamp_s, 0.0, 10, None, None)
    frames[2] = FrameSample(frames[2].stamp_s, 0.0, 10, 640, 480)
    stats = compute_stats(frames)
    assert stats.undecodable_frames == 1
    assert stats.resolutions == ((640, 480), (1280, 720))


def test_stats_width_without_height_counts_as_undecodable():
    frames = _frames(2, 0.1)
    frames[0] = FrameSample(frames[0].stamp_s, 0.0, 10, 1280, None)
    stats = compute_stats(frames)
    assert stats.undecodable_frames == 1
    assert stats.resolutions == ((1280, 720),)


@pytest.mark.parametrize("n", [0, 1])
def test_stats_needs_two_frames(n):
    with pytest.raises(ValueError, match="at least 2 frames"):
        compute_stats(_frames(n, 0.1))


# --- evaluate ---------------------------------------------------------------------------


def test_evaluate_healthy_stream_passes():
    stats = compute_stats(_frames(61, 1 / 60))
    assert evaluate(stats, Expectations(54.0, 1280, 720)) == []


def test_evaluate_rate_exactly_at_threshold_passes():
    stats = compute_stats(_frames(11, 0.02))
    assert evaluate(stats, Expectations(min_rate_hz=stats.rate_hz)) == []


def test_evaluate_rate_below_threshold_fails():
    stats = compute_stats(_frames(31, 1 / 30))
    failures = evaluate(stats, Expectations(54.0))
    assert len(failures) == 1
    assert "below the minimum 54.00 Hz" in failures[0]


def test_evaluate_non_increasing_stamps_fail():
    frames = _frames(4, 0.01)
    frames[3] = FrameSample(frames[2].stamp_s, 0.0, 10, 1280, 720)
    failures = evaluate(compute_stats(frames), Expectations(1.0))
    assert any("header stamp not after" in f for f in failures)


def test_evaluate_undecodable_fails():
    frames = _frames(3, 0.01)
    frames[0] = FrameSample(frames[0].stamp_s, 0.0, 10, None, None)
    failures = evaluate(compute_stats(frames), Expectations(1.0))
    assert failures == ["1 frame(s) were not a JPEG or PNG with a readable size"]


def test_evaluate_resolution_change_fails_even_without_expectation():
    frames = _frames(3, 0.01)
    frames[2] = FrameSample(frames[2].stamp_s, 0.0, 10, 640, 480)
    failures = evaluate(compute_stats(frames), Expectations(1.0))
    assert failures == ["resolution changed during the window: 640x480, 1280x720"]


def test_evaluate_wrong_width_and_height_fail_separately():
    stats = compute_stats(_frames(3, 0.01, width=1920, height=1080))
    failures = evaluate(stats, Expectations(1.0, expected_width=1280, expected_height=720))
    assert failures == [
        "width 1920 is not the expected 1280",
        "height 1080 is not the expected 720",
    ]


def test_evaluate_only_width_checked():
    stats = compute_stats(_frames(3, 0.01, width=1280, height=1080))
    assert evaluate(stats, Expectations(1.0, expected_width=1280)) == []


# --- format_report ----------------------------------------------------------------------


def test_report_pass():
    stats = compute_stats(_frames(61, 1 / 60, size=100_000))
    report = format_report(stats, Expectations(54.0, 1280, 720), [])
    assert "frame rate (stamps)   60.00 Hz (minimum 54.00 Hz)" in report
    assert "resolution            1280x720 (expected 1280x720)" in report
    assert "mean frame size       100.0 kB" in report
    # 100 kB x 60 Hz = 6 MB/s = 21.6 GB/h
    assert "payload rate          6.00 MB/s = 21.6 GB/h" in report
    assert report.endswith("RESULT: PASS")


def test_report_fail_lists_failures_and_unknown_resolution():
    frames = [FrameSample(float(i), float(i), 10, None, None) for i in range(3)]
    stats = compute_stats(frames)
    expected = Expectations(54.0)
    failures = evaluate(stats, expected)
    report = format_report(stats, expected, failures)
    assert "resolution            unknown (expected anyxany)" in report
    assert "RESULT: FAIL" in report
    for failure in failures:
        assert f"  - {failure}" in report
