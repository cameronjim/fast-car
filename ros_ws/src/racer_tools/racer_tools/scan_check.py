"""Pure statistics and pass/fail logic for the /scan rate check (roadmap 2.3).

No ROS, no hardware: everything here works on plain ScanSample records, so it is unit-tested
without rclpy (claude-docs/12-testing.md L1). racer_tools/lidar_check_node.py is the thin
ROS plumbing that collects ScanSample records from a live /scan and calls into this module.

Everything is SI: seconds, metres, radians, hertz. The expectations a scan stream is held to
come from config/vehicle_params.yaml's sensors.lidar_spec (datasheet values) through
`expectations_from_spec`; nothing physical is written down in this file.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from itertools import pairwise
from typing import Any


@dataclass(frozen=True)
class ScanSample:
    """The parts of one sensor_msgs/LaserScan the check needs."""

    stamp_s: float
    angle_min_rad: float
    angle_max_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    ranges: Sequence[float]


@dataclass(frozen=True)
class ScanStats:
    """Summary of a run of scans."""

    scan_count: int
    duration_s: float
    rate_hz: float
    max_gap_s: float
    non_increasing_stamps: int
    beam_count_min: int
    beam_count_max: int
    fov_min_rad: float
    fov_max_rad: float
    msg_range_min_m: float
    msg_range_max_m: float
    observed_range_min_m: float | None
    observed_range_max_m: float | None
    invalid_fraction: float


@dataclass(frozen=True)
class Expectations:
    """What a healthy scan stream must look like. `expected_beam_count` None skips that check."""

    min_rate_hz: float
    expected_beam_count: int | None
    beam_count_tolerance: int
    fov_rad: float
    fov_tolerance_rad: float
    max_invalid_fraction: float
    notes: tuple[str, ...] = field(default_factory=tuple)


def is_valid_range(value: float, range_min_m: float, range_max_m: float) -> bool:
    """A return counts as valid if it is finite and inside the message's own declared band.

    Same rule racer_safety/safety_node applies before its TTC gate (compute_min_scan_range_m),
    so "invalid" here means exactly what the safety node ignores. sllidar_ros2 reports "no
    return" as +inf, which this excludes.
    """
    if not math.isfinite(value):
        return False
    return range_min_m <= value <= range_max_m


def scan_fov_rad(sample: ScanSample) -> float:
    """Angular span the scan declares, |angle_max - angle_min|."""
    return abs(sample.angle_max_rad - sample.angle_min_rad)


def compute_stats(samples: Sequence[ScanSample]) -> ScanStats:
    """Summarise a run of scans, in arrival order. Needs at least two scans for a rate."""
    if len(samples) < 2:
        raise ValueError(
            f"need at least 2 scans to measure a rate, got {len(samples)}; is the driver "
            "publishing /scan?"
        )

    stamps = [s.stamp_s for s in samples]
    gaps = [b - a for a, b in pairwise(stamps)]
    non_increasing = sum(1 for g in gaps if g <= 0.0)
    duration_s = stamps[-1] - stamps[0]
    # Rate over the whole window, (N - 1) intervals over the time they span. A window whose
    # stamps do not advance has no meaningful rate; report 0 so the rate check fails loudly.
    rate_hz = (len(samples) - 1) / duration_s if duration_s > 0.0 else 0.0

    beam_counts = [len(s.ranges) for s in samples]
    fovs = [scan_fov_rad(s) for s in samples]

    total = 0
    invalid = 0
    observed_min: float | None = None
    observed_max: float | None = None
    for s in samples:
        for r in s.ranges:
            total += 1
            if not is_valid_range(r, s.range_min_m, s.range_max_m):
                invalid += 1
                continue
            observed_min = r if observed_min is None else min(observed_min, r)
            observed_max = r if observed_max is None else max(observed_max, r)

    return ScanStats(
        scan_count=len(samples),
        duration_s=duration_s,
        rate_hz=rate_hz,
        max_gap_s=max(gaps),
        non_increasing_stamps=non_increasing,
        beam_count_min=min(beam_counts),
        beam_count_max=max(beam_counts),
        fov_min_rad=min(fovs),
        fov_max_rad=max(fovs),
        msg_range_min_m=min(s.range_min_m for s in samples),
        msg_range_max_m=max(s.range_max_m for s in samples),
        observed_range_min_m=observed_min,
        observed_range_max_m=observed_max,
        invalid_fraction=(invalid / total) if total else 1.0,
    )


def angle_compensated_beam_count(sample_rate_hz: float, scan_frequency_hz: float) -> int:
    """Beam count sllidar_ros2 publishes with angle_compensate:=true.

    Mirrors the driver at the commit docker/car/Dockerfile pins (src/sllidar_node.cpp):
    points_per_circle = int(1e6 / us_per_sample / scan_frequency), multiple =
    size_t(points_per_circle / 360.0 + 1), beams = 360 * multiple. us_per_sample is
    1e6 / sample_rate_hz, so points_per_circle is sample_rate_hz / scan_frequency_hz. For the
    C1 (5000 Hz, 10 Hz) that is 500 points, multiple 2, 720 beams.
    """
    if sample_rate_hz <= 0.0 or scan_frequency_hz <= 0.0:
        raise ValueError("sample_rate_hz and scan_frequency_hz must be positive")
    points_per_circle = int(sample_rate_hz / scan_frequency_hz)
    multiple = max(1, int(points_per_circle / 360.0 + 1))
    return 360 * multiple


def expectations_from_spec(
    spec: Any,
    *,
    angle_compensate: bool,
    max_invalid_fraction: float,
    min_rate_hz: float | None = None,
) -> Expectations:
    """Build Expectations from vehicle_params' sensors.lidar_spec.

    With angle_compensate (the lidar.launch.py default) the driver bins every revolution into
    a fixed array covering exactly field_of_view_rad, so the beam count must match the driver's
    own arithmetic exactly and the declared span must be within one angular step of the
    datasheet field of view. Without it, each scan carries however many samples arrived in
    that revolution, and its span runs from the first to the last valid sample, so beam count
    is only checked to within a quarter of sample rate / nominal scan rate and the span to
    within a tenth of the field of view.
    """
    notes: list[str] = []
    if angle_compensate:
        expected = angle_compensated_beam_count(spec.sample_rate_hz, spec.nominal_scan_rate_hz)
        tolerance = 0
        fov_tolerance = spec.angular_resolution_rad
        notes.append("angle_compensate on: exact beam count from the driver's binning")
    else:
        expected = round(spec.sample_rate_hz / spec.nominal_scan_rate_hz)
        tolerance = math.ceil(expected / 4)
        fov_tolerance = spec.field_of_view_rad / 10.0
        notes.append("angle_compensate off: beam count and span checked loosely")
    rate = spec.min_scan_rate_hz if min_rate_hz is None else min_rate_hz
    return Expectations(
        min_rate_hz=rate,
        expected_beam_count=expected,
        beam_count_tolerance=tolerance,
        fov_rad=spec.field_of_view_rad,
        fov_tolerance_rad=fov_tolerance,
        max_invalid_fraction=max_invalid_fraction,
        notes=tuple(notes),
    )


def evaluate(stats: ScanStats, expected: Expectations) -> list[str]:
    """Every way `stats` fails `expected`, as human-readable strings. Empty means pass."""
    failures: list[str] = []
    if stats.rate_hz < expected.min_rate_hz:
        failures.append(
            f"scan rate {stats.rate_hz:.3f} Hz is below the minimum {expected.min_rate_hz:.3f} Hz"
        )
    if stats.non_increasing_stamps:
        failures.append(
            f"{stats.non_increasing_stamps} scan(s) had a header stamp not after the previous one"
        )
    if expected.expected_beam_count is not None:
        low = expected.expected_beam_count - expected.beam_count_tolerance
        high = expected.expected_beam_count + expected.beam_count_tolerance
        if stats.beam_count_min < low or stats.beam_count_max > high:
            failures.append(
                f"beam count {stats.beam_count_min}..{stats.beam_count_max} is outside the "
                f"expected {expected.expected_beam_count} +- {expected.beam_count_tolerance}"
            )
    fov_low = expected.fov_rad - expected.fov_tolerance_rad
    fov_high = expected.fov_rad + expected.fov_tolerance_rad
    if stats.fov_min_rad < fov_low or stats.fov_max_rad > fov_high:
        failures.append(
            f"field of view {stats.fov_min_rad:.4f}..{stats.fov_max_rad:.4f} rad is outside the "
            f"expected {expected.fov_rad:.4f} +- {expected.fov_tolerance_rad:.4f} rad"
        )
    if stats.observed_range_min_m is None:
        failures.append("no valid range in any scan: the head is blind or not spinning")
    elif stats.invalid_fraction > expected.max_invalid_fraction:
        failures.append(
            f"invalid range fraction {stats.invalid_fraction:.3f} exceeds "
            f"{expected.max_invalid_fraction:.3f}"
        )
    return failures


def _fmt_optional(value: float | None, unit: str) -> str:
    return "none" if value is None else f"{value:.3f} {unit}"


def format_report(stats: ScanStats, expected: Expectations, failures: Sequence[str]) -> str:
    """Multi-line plain-text report for the terminal and for pasting into docs/notes."""
    beams = (
        "not checked"
        if expected.expected_beam_count is None
        else f"{expected.expected_beam_count} +- {expected.beam_count_tolerance}"
    )
    lines = [
        f"scans received        {stats.scan_count} over {stats.duration_s:.3f} s",
        f"scan rate             {stats.rate_hz:.3f} Hz (minimum {expected.min_rate_hz:.3f} Hz)",
        f"largest stamp gap     {stats.max_gap_s:.4f} s",
        f"beam count            {stats.beam_count_min}..{stats.beam_count_max} (expected {beams})",
        (
            f"field of view         {stats.fov_min_rad:.4f}..{stats.fov_max_rad:.4f} rad "
            f"(expected {expected.fov_rad:.4f} +- {expected.fov_tolerance_rad:.4f} rad)"
        ),
        f"declared range band   {stats.msg_range_min_m:.3f} .. {stats.msg_range_max_m:.3f} m",
        (
            f"observed valid ranges {_fmt_optional(stats.observed_range_min_m, 'm')} .. "
            f"{_fmt_optional(stats.observed_range_max_m, 'm')}"
        ),
        (
            f"invalid fraction      {stats.invalid_fraction:.3f} "
            f"(maximum {expected.max_invalid_fraction:.3f})"
        ),
    ]
    lines.extend(f"note: {note}" for note in expected.notes)
    if failures:
        lines.append("RESULT: FAIL")
        lines.extend(f"  - {failure}" for failure in failures)
    else:
        lines.append("RESULT: PASS")
    return "\n".join(lines)
