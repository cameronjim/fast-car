"""Per-scan table, summary and CSV for a follower replay. Pure functions, no I/O but the CSV."""

from __future__ import annotations

import csv
import math
import statistics
from bisect import bisect_left
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from follower_replay.bag import (
    PHASE_ENGAGE,
    PHASE_RELEASE,
    SEVERITY_BRAKE,
    BagData,
    DriveRecord,
    EventRecord,
    ScanRecord,
)

# |steering| below this counts as "straight" in the left / right / straight tally (rad, about
# 3 degrees; the servo's own dead band is wider).
DEFAULT_STRAIGHT_THRESHOLD_RAD = 0.05

CSV_COLUMNS = (
    "stamp_s",
    "t_s",
    "valid",
    "gap_found",
    "target_bearing_rad",
    "target_range_m",
    "wanted_steering_rad",
    "steering_rad",
    "filtered_steering_rad",
    "clamped",
    "corner_blocked",
    "centering_rad",
    "recorded_drive_raw_rad",
    "recorded_drive_rad",
)

NAN = float("nan")


def recorded_before_next(
    records: Sequence[DriveRecord], start_s: float, end_s: float | None
) -> float:
    """Steering of the last record with start_s <= stamp < end_s (NaN if none).

    The CLI reports its low-pass output at the END of each scan's interval, so the recorded
    value it is compared with is the last command the car published before the next scan.
    """
    stamps = [r.stamp_s for r in records]
    hi = len(records) if end_s is None else bisect_left(stamps, end_s)
    lo = bisect_left(stamps, start_s)
    if hi <= lo:
        return NAN
    return records[hi - 1].steering_rad


def build_table(
    scans: Sequence[ScanRecord],
    cli_rows: Sequence[dict[str, Any]],
    drive_raw: Sequence[DriveRecord],
    drive: Sequence[DriveRecord],
) -> list[dict[str, Any]]:
    """One row per scan: the CLI's result plus the recorded commands over the same interval."""
    if len(scans) != len(cli_rows):
        raise ValueError(f"{len(scans)} scans but {len(cli_rows)} CLI rows")
    raw_sorted = sorted(drive_raw, key=lambda r: r.stamp_s)
    gated_sorted = sorted(drive, key=lambda r: r.stamp_s)
    t0 = scans[0].stamp_s if scans else 0.0
    table: list[dict[str, Any]] = []
    for i, (scan, row) in enumerate(zip(scans, cli_rows, strict=True)):
        end = scans[i + 1].stamp_s if i + 1 < len(scans) else None
        table.append(
            {
                "stamp_s": scan.stamp_s,
                "t_s": scan.stamp_s - t0,
                "valid": bool(row["valid"]),
                "gap_found": bool(row["gap_found"]),
                "target_bearing_rad": row["target_bearing_rad"],
                "target_range_m": row["target_range_m"],
                "wanted_steering_rad": row["wanted_steering_rad"],
                "steering_rad": row["steering_rad"],
                "filtered_steering_rad": row["filtered_steering_rad"],
                "clamped": bool(row["clamped"]),
                "corner_blocked": bool(row["corner_blocked"]),
                "centering_rad": row["centering_rad"],
                "recorded_drive_raw_rad": recorded_before_next(raw_sorted, scan.stamp_s, end),
                "recorded_drive_rad": recorded_before_next(gated_sorted, scan.stamp_s, end),
            }
        )
    return table


def write_csv(table: Sequence[dict[str, Any]], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as out:
        writer = csv.DictWriter(out, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        for row in table:
            writer.writerow({k: "" if _missing(row[k]) else row[k] for k in CSV_COLUMNS})


def _missing(value: Any) -> bool:
    return value is None or (isinstance(value, float) and math.isnan(value))


def steer_counts(values: Sequence[float | None], threshold_rad: float) -> dict[str, int]:
    """Left (positive), right (negative) and straight tallies; missing values are skipped."""
    counts = {"left": 0, "right": 0, "straight": 0}
    for v in values:
        if _missing(v):
            continue
        assert v is not None
        if v > threshold_rad:
            counts["left"] += 1
        elif v < -threshold_rad:
            counts["right"] += 1
        else:
            counts["straight"] += 1
    return counts


def event_summary(events: Sequence[EventRecord]) -> dict[str, Any]:
    """Interventions from /safety/events, counted the way claude-docs/09 counts them.

    One intervention is one PHASE_ENGAGE record; a PHASE_RELEASE record closes it and carries
    its duration (a latch's duration is how long the car was held).
    """
    engages: dict[str, int] = {}
    brakes: dict[str, int] = {}
    releases: dict[str, list[float]] = {}
    for event in events:
        if event.phase == PHASE_ENGAGE:
            engages[event.source] = engages.get(event.source, 0) + 1
            if event.severity == SEVERITY_BRAKE:
                brakes[event.source] = brakes.get(event.source, 0) + 1
        elif event.phase == PHASE_RELEASE:
            releases.setdefault(event.source, []).append(event.duration_s)
    latches = {
        source: {
            "count": len(durations),
            "total_s": sum(durations),
            "median_s": statistics.median(durations),
            "max_s": max(durations),
        }
        for source, durations in sorted(releases.items())
    }
    still_engaged = {
        source: count - len(releases.get(source, []))
        for source, count in engages.items()
        if count > len(releases.get(source, []))
    }
    return {
        "engages": dict(sorted(engages.items())),
        "brakes": dict(sorted(brakes.items())),
        "brakes_total": sum(brakes.values()),
        "latch_durations": latches,
        "still_engaged_at_end": still_engaged,
    }


def summarise(
    bag: BagData,
    table: Sequence[dict[str, Any]],
    profile: str,
    params: dict[str, Any],
    straight_threshold_rad: float = DEFAULT_STRAIGHT_THRESHOLD_RAD,
) -> dict[str, Any]:
    """The numbers replay_follower.py prints. Rates are over scans the core accepted."""
    valid = [r for r in table if r["valid"]]
    n_valid = len(valid)

    def rate(count: int) -> float:
        return count / n_valid if n_valid else 0.0

    paired = [
        (r["filtered_steering_rad"], r["recorded_drive_raw_rad"])
        for r in table
        if not _missing(r["filtered_steering_rad"]) and not _missing(r["recorded_drive_raw_rad"])
    ]
    errors = [abs(a - b) for a, b in paired]
    both_turning = [
        (a, b)
        for a, b in paired
        if abs(a) > straight_threshold_rad and abs(b) > straight_threshold_rad
    ]
    duration = table[-1]["t_s"] if table else 0.0
    return {
        "bag": {
            "files": bag.files,
            "truncated": bag.truncated,
            "truncation_note": bag.truncation_note,
            "decode_errors": bag.decode_errors,
            "topic_counts": dict(sorted(bag.topic_counts.items())),
            "duration_s": duration,
        },
        "profile": profile,
        "params": params,
        "scans": len(table),
        "valid_scans": n_valid,
        "gap_not_found": sum(1 for r in valid if not r["gap_found"]),
        "straight_threshold_rad": straight_threshold_rad,
        "steering_replay": steer_counts([r["steering_rad"] for r in valid], straight_threshold_rad),
        "steering_replay_filtered": steer_counts(
            [r["filtered_steering_rad"] for r in table], straight_threshold_rad
        ),
        "steering_recorded_drive_raw": steer_counts(
            [r["recorded_drive_raw_rad"] for r in table], straight_threshold_rad
        ),
        "clamp_rate": rate(sum(1 for r in valid if r["clamped"])),
        "clamped_scans": sum(1 for r in valid if r["clamped"]),
        "corner_override_rate": rate(sum(1 for r in valid if r["corner_blocked"])),
        "corner_override_scans": sum(1 for r in valid if r["corner_blocked"]),
        "agreement_with_recorded": {
            "scans_compared": len(paired),
            "mean_abs_error_rad": statistics.fmean(errors) if errors else None,
            "sign_disagreements_when_both_turning": sum(
                1 for a, b in both_turning if (a > 0) != (b > 0)
            ),
            "both_turning": len(both_turning),
        },
        "events": event_summary(bag.events),
    }


def _counts_text(counts: dict[str, int]) -> str:
    return f"left {counts['left']}, right {counts['right']}, straight {counts['straight']}"


def format_summary(summary: dict[str, Any]) -> str:
    """The human-readable summary printed at the end of a run."""
    bag = summary["bag"]
    lines = [
        (
            f"bag: {', '.join(Path(f).name for f in bag['files'])}"
            f" ({bag['duration_s']:.1f} s of scans)"
        ),
    ]
    if bag["truncated"]:
        lines.append(f"  NOTE truncated/unindexed: {bag['truncation_note']}")
    if bag["decode_errors"]:
        lines.append(f"  NOTE {bag['decode_errors']} message(s) could not be decoded")
    n = summary["valid_scans"]
    lines += [
        f"profile: {summary['profile']}",
        (
            f"scans: {summary['scans']} ({n} accepted by the core, "
            f"{summary['gap_not_found']} with no gap found)"
        ),
        f"steering, replay (before low-pass), |s| > {summary['straight_threshold_rad']} rad: "
        + _counts_text(summary["steering_replay"]),
        "steering, replay (low-passed):  " + _counts_text(summary["steering_replay_filtered"]),
        "steering, recorded /drive_raw:  " + _counts_text(summary["steering_recorded_drive_raw"]),
        (
            f"swept-path clamp: {summary['clamped_scans']} of {n} scans "
            f"({100 * summary['clamp_rate']:.1f} %)"
        ),
        (
            f"corner override:  {summary['corner_override_scans']} of {n} scans "
            f"({100 * summary['corner_override_rate']:.1f} %)"
        ),
    ]
    agree = summary["agreement_with_recorded"]
    if agree["mean_abs_error_rad"] is not None:
        lines.append(
            f"vs recorded /drive_raw over {agree['scans_compared']} scans: mean |difference| "
            f"{agree['mean_abs_error_rad']:.3f} rad, opposite sign in "
            f"{agree['sign_disagreements_when_both_turning']} of {agree['both_turning']} scans "
            "where both turn"
        )
    ev = summary["events"]
    if ev["engages"] or ev["latch_durations"]:
        brake_text = ", ".join(f"{s} {c}" for s, c in ev["brakes"].items()) or "none"
        lines.append(f"/safety/events: {ev['brakes_total']} brake engagement(s) ({brake_text})")
        engage_text = ", ".join(f"{s} {c}" for s, c in ev["engages"].items())
        lines.append(f"  engagements by gate: {engage_text}")
        for source, d in ev["latch_durations"].items():
            lines.append(
                f"  {source} releases: {d['count']}, total {d['total_s']:.1f} s, "
                f"median {d['median_s']:.1f} s, longest {d['max_s']:.1f} s"
            )
        for source, count in ev["still_engaged_at_end"].items():
            lines.append(f"  {source}: {count} engagement(s) still held when the bag ended")
    else:
        lines.append("/safety/events: none recorded")
    return "\n".join(lines)
