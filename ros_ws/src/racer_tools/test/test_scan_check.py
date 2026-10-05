"""L1 tests for racer_tools/scan_check.py (roadmap 2.3): pure logic, no ROS.

Hand-computed cases. The spec object here is a stand-in with arbitrary numbers, deliberately
NOT the RPLIDAR C1's, so these tests pin the arithmetic and not a datasheet; the C1's own
720-beam figure is checked against the driver formula in one named case below.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest
from racer_tools.scan_check import (
    Expectations,
    ScanSample,
    angle_compensated_beam_count,
    compute_stats,
    evaluate,
    expectations_from_spec,
    format_report,
    is_valid_range,
    scan_fov_rad,
)

INF = float("inf")
NAN = float("nan")


def _scan(stamp_s, ranges, angle_min=-math.pi, angle_max=math.pi, rmin=0.05, rmax=12.0):
    n = len(ranges)
    inc = (angle_max - angle_min) / (n - 1) if n > 1 else 0.0
    return ScanSample(
        stamp_s=stamp_s,
        angle_min_rad=angle_min,
        angle_max_rad=angle_max,
        angle_increment_rad=inc,
        range_min_m=rmin,
        range_max_m=rmax,
        ranges=tuple(ranges),
    )


def _expect(**overrides):
    base = {
        "min_rate_hz": 8.0,
        "expected_beam_count": 4,
        "beam_count_tolerance": 0,
        "fov_rad": 2.0 * math.pi,
        "fov_tolerance_rad": 0.01,
        "max_invalid_fraction": 0.5,
    }
    base.update(overrides)
    return Expectations(**base)


# --- is_valid_range ---------------------------------------------------------------------


@pytest.mark.parametrize(
    ("value", "valid"),
    [
        (1.0, True),
        (0.05, True),  # exactly at range_min
        (12.0, True),  # exactly at range_max
        (0.0499, False),
        (12.0001, False),
        (0.0, False),
        (-1.0, False),
        (INF, False),
        (-INF, False),
        (NAN, False),
    ],
)
def test_is_valid_range(value, valid):
    assert is_valid_range(value, 0.05, 12.0) is valid


def test_scan_fov_is_absolute_span():
    assert scan_fov_rad(_scan(0.0, [1, 1], angle_min=1.0, angle_max=-0.5)) == pytest.approx(1.5)


# --- compute_stats ----------------------------------------------------------------------


def test_compute_stats_hand_computed():
    samples = [
        _scan(100.0, [1.0, 2.0, INF, 3.0]),
        _scan(100.1, [0.5, 2.0, 2.0, 2.0]),
        _scan(100.3, [1.0, 13.0, NAN, 4.0]),
    ]
    stats = compute_stats(samples)
    assert stats.scan_count == 3
    assert stats.duration_s == pytest.approx(0.3)
    assert stats.rate_hz == pytest.approx(2 / 0.3)
    assert stats.max_gap_s == pytest.approx(0.2)
    assert stats.non_increasing_stamps == 0
    assert (stats.beam_count_min, stats.beam_count_max) == (4, 4)
    assert stats.fov_min_rad == pytest.approx(2 * math.pi)
    assert stats.observed_range_min_m == 0.5
    assert stats.observed_range_max_m == 4.0
    # INF, 13.0 (above range_max), NAN: 3 of 12.
    assert stats.invalid_fraction == pytest.approx(3 / 12)
    assert stats.msg_range_min_m == 0.05
    assert stats.msg_range_max_m == 12.0


def test_compute_stats_ten_hz_stream():
    samples = [_scan(i * 0.1, [1.0] * 4) for i in range(11)]
    assert compute_stats(samples).rate_hz == pytest.approx(10.0)


@pytest.mark.parametrize("count", [0, 1])
def test_compute_stats_refuses_fewer_than_two_scans(count):
    with pytest.raises(ValueError, match="at least 2 scans"):
        compute_stats([_scan(0.0, [1.0])] * count)


def test_compute_stats_counts_non_increasing_stamps_and_zero_duration_rate():
    samples = [_scan(5.0, [1.0, 1.0]), _scan(5.0, [1.0, 1.0])]
    stats = compute_stats(samples)
    assert stats.non_increasing_stamps == 1
    assert stats.rate_hz == 0.0


def test_compute_stats_backwards_stamp():
    samples = [_scan(1.0, [1.0, 1.0]), _scan(1.1, [1.0, 1.0]), _scan(1.05, [1.0, 1.0])]
    assert compute_stats(samples).non_increasing_stamps == 1


def test_compute_stats_all_invalid():
    stats = compute_stats([_scan(0.0, [INF, INF]), _scan(0.1, [0.0, NAN])])
    assert stats.observed_range_min_m is None
    assert stats.observed_range_max_m is None
    assert stats.invalid_fraction == 1.0


def test_compute_stats_empty_ranges_counts_as_fully_invalid():
    stats = compute_stats([_scan(0.0, []), _scan(0.1, [])])
    assert stats.invalid_fraction == 1.0
    assert stats.beam_count_max == 0


def test_compute_stats_varying_beam_counts():
    stats = compute_stats([_scan(0.0, [1.0] * 3), _scan(0.1, [1.0] * 5)])
    assert (stats.beam_count_min, stats.beam_count_max) == (3, 5)


# --- angle_compensated_beam_count -------------------------------------------------------


def test_beam_count_rplidar_c1_numbers_match_the_driver_arithmetic():
    # sllidar_node.cpp: 1e6 / 200 us / 10 Hz = 500 points; 500 / 360 + 1 = 2.39 -> 2; 720.
    assert angle_compensated_beam_count(5000.0, 10.0) == 720


@pytest.mark.parametrize(
    ("sample_rate_hz", "scan_hz", "beams"),
    [
        (3590.0, 10.0, 360),  # 359 points -> multiple 1
        (3600.0, 10.0, 720),  # exactly 360 points -> multiple 2
        (7199.0, 10.0, 720),  # 719 points -> multiple 2
        (7200.0, 10.0, 1080),  # 720 points -> multiple 3
        (100.0, 10.0, 360),  # 10 points, multiple floors to 1
    ],
)
def test_beam_count_boundaries(sample_rate_hz, scan_hz, beams):
    assert angle_compensated_beam_count(sample_rate_hz, scan_hz) == beams


@pytest.mark.parametrize(("sample_rate_hz", "scan_hz"), [(0.0, 10.0), (5000.0, 0.0), (-1, 10)])
def test_beam_count_rejects_non_positive(sample_rate_hz, scan_hz):
    with pytest.raises(ValueError):
        angle_compensated_beam_count(sample_rate_hz, scan_hz)


# --- expectations_from_spec -------------------------------------------------------------

_SPEC = SimpleNamespace(
    nominal_scan_rate_hz=12.0,
    min_scan_rate_hz=9.0,
    sample_rate_hz=6000.0,
    angular_resolution_rad=0.02,
    field_of_view_rad=4.0,
)


def test_expectations_angle_compensated():
    exp = expectations_from_spec(_SPEC, angle_compensate=True, max_invalid_fraction=0.3)
    assert exp.min_rate_hz == 9.0
    assert exp.expected_beam_count == 720  # 6000 / 12 = 500 points
    assert exp.beam_count_tolerance == 0
    assert exp.fov_rad == 4.0
    assert exp.fov_tolerance_rad == 0.02
    assert exp.max_invalid_fraction == 0.3


def test_expectations_raw_scans_are_loose():
    exp = expectations_from_spec(_SPEC, angle_compensate=False, max_invalid_fraction=0.5)
    assert exp.expected_beam_count == 500
    assert exp.beam_count_tolerance == 125
    assert exp.fov_tolerance_rad == pytest.approx(0.4)


def test_expectations_min_rate_override():
    exp = expectations_from_spec(
        _SPEC, angle_compensate=True, max_invalid_fraction=0.5, min_rate_hz=9.5
    )
    assert exp.min_rate_hz == 9.5


# --- evaluate ---------------------------------------------------------------------------


def _good_stats():
    return compute_stats([_scan(i * 0.1, [1.0, 2.0, 3.0, 4.0]) for i in range(11)])


def test_evaluate_passes_a_healthy_stream():
    assert evaluate(_good_stats(), _expect()) == []


def test_evaluate_rate_exactly_at_minimum_passes_and_just_below_fails():
    stats = _good_stats()  # 10 Hz
    assert evaluate(stats, _expect(min_rate_hz=10.0 - 1e-9)) == []
    failures = evaluate(stats, _expect(min_rate_hz=10.0 + 1e-6))
    assert len(failures) == 1 and "scan rate" in failures[0]


def test_evaluate_beam_count_mismatch_and_tolerance():
    stats = _good_stats()  # 4 beams
    assert any("beam count" in f for f in evaluate(stats, _expect(expected_beam_count=5)))
    assert evaluate(stats, _expect(expected_beam_count=5, beam_count_tolerance=1)) == []
    assert any("beam count" in f for f in evaluate(stats, _expect(expected_beam_count=3)))


def test_evaluate_beam_count_skipped_when_none():
    assert evaluate(_good_stats(), _expect(expected_beam_count=None)) == []


def test_evaluate_fov_mismatch_both_sides():
    stats = _good_stats()  # 2 pi
    assert any("field of view" in f for f in evaluate(stats, _expect(fov_rad=math.pi)))
    assert any("field of view" in f for f in evaluate(stats, _expect(fov_rad=3 * math.pi)))
    assert evaluate(stats, _expect(fov_rad=2 * math.pi + 0.009)) == []


def test_evaluate_non_increasing_stamps_fail():
    stats = compute_stats(
        [_scan(0.0, [1.0] * 4), _scan(0.1, [1.0] * 4), _scan(0.1, [1.0] * 4), _scan(0.3, [1.0] * 4)]
    )
    failures = evaluate(stats, _expect(min_rate_hz=1.0))
    assert len(failures) == 1 and "header stamp" in failures[0]


def test_evaluate_invalid_fraction_threshold():
    stats = compute_stats([_scan(i * 0.1, [1.0, INF, INF, 1.0]) for i in range(11)])
    assert stats.invalid_fraction == pytest.approx(0.5)
    assert evaluate(stats, _expect(max_invalid_fraction=0.5)) == []
    assert any("invalid range" in f for f in evaluate(stats, _expect(max_invalid_fraction=0.4)))


def test_evaluate_blind_head_fails_even_with_permissive_threshold():
    stats = compute_stats([_scan(i * 0.1, [INF] * 4) for i in range(11)])
    failures = evaluate(stats, _expect(max_invalid_fraction=1.0))
    assert failures == ["no valid range in any scan: the head is blind or not spinning"]


def test_evaluate_reports_every_failure_at_once():
    stats = compute_stats([_scan(0.0, [INF] * 3), _scan(1.0, [INF] * 3)])  # 1 Hz, 3 beams
    failures = evaluate(stats, _expect(fov_rad=1.0))
    assert len(failures) == 4


# --- format_report ----------------------------------------------------------------------


def test_format_report_pass_and_fail_lines():
    stats = _good_stats()
    exp = _expect()
    text = format_report(stats, exp, [])
    assert text.endswith("RESULT: PASS")
    assert "10.000 Hz" in text
    text = format_report(stats, exp, ["a problem"])
    assert "RESULT: FAIL" in text and "  - a problem" in text


def test_format_report_handles_blind_and_unchecked_beams():
    stats = compute_stats([_scan(0.0, [INF] * 4), _scan(0.1, [INF] * 4)])
    exp = _expect(expected_beam_count=None, notes=("hello",))
    text = format_report(stats, exp, [])
    assert "none .. none" in text
    assert "not checked" in text
    assert "note: hello" in text
