"""L1 tests for racer_perception.camera_record: the drive gate, the size plan, the JPEG shrink.

Standard library only. The JPEG half (needs OpenCV) is test_camera_record_jpeg.py.
"""

from __future__ import annotations

import math

import pytest
from racer_perception.camera_record import (
    ActivityGate,
    choose_reduction,
    jpeg_dimensions,
    plan_output_size,
)

# ------------------------------------------------------------------------------ ActivityGate


def test_gate_is_closed_until_activity_is_seen():
    gate = ActivityGate(5.0)
    assert gate.is_open(0.0) is False
    assert gate.is_open(1e9) is False


def test_gate_opens_on_activity_and_closes_after_the_hold():
    gate = ActivityGate(5.0)
    gate.note(100.0)
    assert gate.is_open(100.0)
    assert gate.is_open(104.9)
    assert not gate.is_open(105.0)  # exactly hold_s later is closed
    assert not gate.is_open(500.0)


def test_gate_activity_extends_the_hold():
    gate = ActivityGate(2.0)
    for t in (0.0, 1.5, 3.0, 4.5):
        gate.note(t)
        assert gate.is_open(t + 1.9)
    assert not gate.is_open(4.5 + 2.0)


def test_gate_reopens_on_new_activity_after_closing():
    gate = ActivityGate(1.0)
    gate.note(0.0)
    assert not gate.is_open(10.0)
    gate.note(10.0)
    assert gate.is_open(10.5)


def test_gate_with_zero_hold_never_opens():
    gate = ActivityGate(0.0)
    gate.note(1.0)
    assert not gate.is_open(1.0)


def test_gate_clock_stepping_back_does_not_hold_it_open_for_the_step():
    gate = ActivityGate(5.0)
    gate.note(1000.0)
    assert gate.is_open(1001.0)
    # The clock jumps back an hour: the hold restarts from the new time, it is not 3600 s long.
    assert gate.is_open(-2600.0)
    assert gate.is_open(-2596.0)
    assert not gate.is_open(-2595.0)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_gate_refuses_a_non_finite_hold(bad):
    with pytest.raises(ValueError, match="finite"):
        ActivityGate(bad)


# ------------------------------------------------------------------------------ size planning


@pytest.mark.parametrize(
    ("src", "limit", "expected"),
    [
        ((1280, 720), (640, 360), (640, 360)),
        ((1920, 1200), (640, 360), (576, 360)),  # height limits, aspect kept
        ((1280, 960), (640, 360), (480, 360)),
        ((320, 240), (640, 360), (320, 240)),  # never upscaled
        ((640, 360), (640, 360), (640, 360)),
        ((1000, 10), (100, 100), (100, 1)),  # never below 1 px
    ],
)
def test_plan_output_size(src, limit, expected):
    assert plan_output_size(*src, *limit) == expected


@pytest.mark.parametrize(
    "args", [(0, 720, 640, 360), (1280, 0, 640, 360), (1, 1, 0, 1), (1, 1, 1, -1)]
)
def test_plan_output_size_refuses_non_positive_sizes(args):
    with pytest.raises(ValueError, match="positive"):
        plan_output_size(*args)


@pytest.mark.parametrize(
    ("src", "out", "expected"),
    [
        ((1280, 720), (640, 360), 2),
        ((1280, 720), (320, 180), 4),
        ((1280, 720), (160, 90), 8),
        ((1280, 720), (1280, 720), 1),
        ((1280, 720), (641, 360), 1),  # a reduced decode would be smaller than the output
        ((1281, 721), (641, 361), 2),  # ceil(1281 / 2) = 641
        ((320, 240), (320, 240), 1),
    ],
)
def test_choose_reduction_never_decodes_smaller_than_the_output(src, out, expected):
    assert choose_reduction(*src, *out) == expected


# ------------------------------------------------------------------------------ jpeg_dimensions


@pytest.mark.parametrize(
    "junk",
    [
        b"",
        b"\xff",
        b"not a jpeg at all",
        b"\xff\xd8",  # SOI and nothing else
        b"\xff\xd8\xff\xda\x00\x02",  # start of scan before any frame header
        b"\xff\xd8\xff\xc0\x00\x11\x08\x00",  # truncated frame header
        b"\xff\xd8\x00\x00\x00\x00",  # no marker where one belongs
    ],
)
def test_jpeg_dimensions_is_none_for_junk(junk):
    assert jpeg_dimensions(junk) is None


def test_jpeg_dimensions_is_none_for_a_zero_sized_frame():
    # SOI, SOF0 with width 0, height 0.
    data = b"\xff\xd8\xff\xc0\x00\x0b\x08\x00\x00\x00\x00\x01\x01\x11\x00"
    assert jpeg_dimensions(data) is None


def test_jpeg_dimensions_skips_other_segments_to_find_the_frame_header():
    # SOI, APP0 (length 4 = 2 payload bytes), then SOF0 for 7 x 5.
    data = b"\xff\xd8\xff\xe0\x00\x04\xaa\xbb\xff\xc0\x00\x0b\x08\x00\x05\x00\x07\x01\x01\x11\x00"
    assert jpeg_dimensions(data) == (7, 5)
