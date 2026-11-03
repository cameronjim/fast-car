"""Recorded brakes against two replays of the gate, and what the car actually did.

For every recorded forward obstacle brake (a `ttc` BRAKE engage whose detail is a TTC or a
clearance trip, not a latch held across a short-circuit):

  * MEASURED from the bag's LiDAR (motion.py): the end clearance (the smallest gap between any
    return and the car's footprint, no margin, over the scans 0.25 to 1.0 s after the brake, when
    the car has stopped and before any reverse escape), whether that is contact (<= 2 cm, the
    foam), the stopping travel (the rear axle's displacement from the scan the gate judged to the
    farthest point within 0.8 s, by scan matching) and the speed just before (scan matching over
    the two scans before).
  * BEFORE: the replay with the rule the bag was driven with must trip within 0.15 s of the
    recorded brake. This checks the replay, not the rule.
  * AFTER: the first trip of the rule under test on the same approach: from 0.8 s before the
    recorded brake (or the release of the previous recorded engagement, if later) up to 0.3 s
    after it; or, if there is none, an earlier trip on the approach that had not released by
    then (the car would already have been stopped). Its lead over the recorded brake, and the
    car's displacement between the scan it tripped on and the scan the recorded brake tripped on
    (the gain: how much earlier on the approach it starts stopping).
  * PREDICTED end clearance under the rule under test. On a steady approach the stopping travel
    from the same speed is the same, so braking `gain` metres earlier ends `gain` metres farther
    from the wall: predicted = end + gain. Where the recorded brake ended in contact, the wall cut
    the travel short, so the end is first corrected by the travel the car would have needed: the
    90th percentile of travel / request speed over the brakes that stopped clear, times this
    brake's request speed, less the travel measured. Where the rule under test trips LATER than
    the recorded brake, or not before the bag shows the car stopping, the bag cannot show what
    follows (the car was already braking); the extra distance is taken from the rule's own
    in-path distance at the recorded brake down to the distance it trips at (its trip, or
    max(ttc_brake_s x request, floor)): predicted = end - that. A brake where the car was
    already standing (measured approach speed below 0.1 m/s: a latch re-tripping at rest) is no
    approach and gets no prediction.

A trip of the rule under test that is on no recorded approach is listed separately, with whether
the car drove on freely after it (recorded /drive above zero for the next second and no recorded
brake): that is a new brake where the bag shows none was needed.
"""

from __future__ import annotations

import bisect
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from follower_replay.bag import (
    PHASE_ENGAGE,
    PHASE_RELEASE,
    SEVERITY_BRAKE,
    BagData,
    DriveRecord,
    ScanRecord,
)

from safety_replay.motion import Body, Mount, box_clearance, car_motion, scan_points

MATCH_WINDOW_S = 0.15  # BEFORE replay vs recorded brake
AFTER_SLACK_S = 0.3  # an AFTER trip up to this long after the recorded brake still belongs to it
APPROACH_WINDOW_S = 0.8  # ... and from this long before it (earlier ones are trips of their own)
END_WINDOW_S = (0.25, 1.0)  # scans used for the end clearance, after the brake
TRAVEL_WINDOW_S = 0.8  # the farthest point the car reaches within this, after the brake
CONTACT_M = 0.02
AT_REST_MPS = 0.1  # measured approach speed below this: the car was standing when it braked
LATER_TOLERANCE_S = 0.02
FREE_DRIVE_WINDOW_S = 1.0


@dataclass
class BrakeRow:
    t_s: float  # recorded brake, arrival time base
    cause: str  # ttc or floor (recorded detail)
    speed_mps: float  # request at the brake
    steering_rad: float
    end_clearance_m: float = math.nan
    travel_m: float = math.nan
    approach_speed_mps: float = math.nan  # measured by scan matching
    before: dict[str, Any] | None = None
    after: dict[str, Any] | None = None
    lead_s: float = math.nan  # recorded brake time - AFTER trip time (> 0: earlier)
    gain_m: float = 0.0
    after_distance_at_brake_m: float = math.nan  # the rule under test's in-path distance then
    predicted_m: float = math.nan
    basis: str = ""

    @property
    def contact(self) -> bool:
        return self.end_clearance_m <= CONTACT_M


@dataclass
class Report:
    rows: list[BrakeRow] = field(default_factory=list)
    extra_trips: list[dict[str, Any]] = field(default_factory=list)
    travel_ratio_p90_s: float = math.nan
    before_watchdog_engages: int = 0
    after_watchdog_engages: int = 0
    recorded_watchdog_engages: int = 0


def arrival(record: Any) -> float:
    log_time = float(getattr(record, "log_time_s", 0.0))
    return log_time if log_time > 0.0 else float(record.stamp_s)


def recorded_brakes(bag: BagData) -> tuple[list[tuple[float, str]], list[tuple[float, float]]]:
    """(brakes, engagements): every recorded forward TTC / floor brake (time, cause) and every
    recorded forward `ttc` BRAKE engagement as (engage, release) (release +inf if none)."""
    brakes: list[tuple[float, str]] = []
    engagements: list[tuple[float, float]] = []
    open_at: float | None = None
    for e in sorted(bag.events, key=arrival):
        if e.source != "ttc" or e.severity != SEVERITY_BRAKE:
            continue
        t = arrival(e)
        if e.phase == PHASE_ENGAGE:
            open_at = t
            if e.detail.startswith("time-to-collision"):
                brakes.append((t, "ttc"))
            elif e.detail.startswith("forward clearance"):
                brakes.append((t, "floor"))
        elif e.phase == PHASE_RELEASE and open_at is not None:
            engagements.append((open_at, t))
            open_at = None
    if open_at is not None:
        engagements.append((open_at, math.inf))
    return brakes, engagements


def _last_before(times: Sequence[float], t: float) -> int:
    return bisect.bisect_right(times, t) - 1


def _command_at(drive: Sequence[DriveRecord], times: Sequence[float], t: float) -> DriveRecord:
    i = max(0, _last_before(times, t))
    return drive[i]


def forward_trips(results: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    return [r for r in results if r.get("type") == "trip" and r.get("gate") == "ttc"]


def watchdog_engages(results: Sequence[dict[str, Any]]) -> int:
    return sum(
        1
        for r in results
        if r.get("type") == "event" and r.get("source") == "watchdog" and r.get("phase") == "engage"
    )


def measure_brake(
    row: BrakeRow,
    scans: Sequence[ScanRecord],
    scan_times: Sequence[float],
    mount: Mount,
    body: Body,
) -> None:
    """Fills end clearance, travel and approach speed from the scans (motion.py)."""
    i0 = _last_before(scan_times, row.t_s)
    if i0 < 0:
        return
    lo, hi = END_WINDOW_S
    ends = [
        box_clearance(scan_points(scans[i], mount), body)
        for i in range(i0 + 1, len(scans))
        if row.t_s + lo < scan_times[i] < row.t_s + hi
    ]
    if ends:
        row.end_clearance_m = min(ends)
    farthest = 0.0
    for i in range(i0 + 1, len(scans)):
        if scan_times[i] > row.t_s + TRAVEL_WINDOW_S:
            break
        try:
            x, y, _, _ = car_motion(scans[i0], scans[i], mount)
        except ValueError:
            continue
        farthest = max(farthest, math.hypot(x, y))
    row.travel_m = farthest
    if i0 >= 2:
        try:
            x, y, _, _ = car_motion(scans[i0 - 2], scans[i0], mount)
            row.approach_speed_mps = math.hypot(x, y) / max(
                scan_times[i0] - scan_times[i0 - 2], 1e-6
            )
        except ValueError:
            pass


def _no_threshold(speed_mps: float) -> float:
    return math.nan


def trip_distance_for(rule: dict[str, Any]) -> Callable[[float], float]:
    """The in-path distance a rule trips at for a request speed: max(ttc_brake_s x speed,
    min_forward_clearance_m), each only when set (> 0). From the CLI's --describe output."""
    ttc = float(rule.get("ttc_brake_s", -1.0))
    floor = float(rule.get("min_forward_clearance_m", -1.0))

    def distance(speed_mps: float) -> float:
        candidates = []
        if ttc > 0.0:
            candidates.append(ttc * max(speed_mps, 0.0))
        if floor > 0.0:
            candidates.append(floor)
        return max(candidates) if candidates else math.nan

    return distance


def analyse(
    bag: BagData,
    before: Sequence[dict[str, Any]],
    after: Sequence[dict[str, Any]],
    mount: Mount,
    body: Body,
    measurements: dict[float, tuple[float, float, float]] | None = None,
    trip_distance_m: Callable[[float], float] | None = None,
) -> Report:
    """`measurements`, when given, caches the bag-only measurements (end clearance, travel,
    approach speed) per brake time across calls: they do not depend on the rules."""
    report = Report()
    if trip_distance_m is None:
        trip_distance_m = _no_threshold
    after_cycles = [r for r in after if r.get("type") == "cycle"]
    cycle_times = [r["t"] for r in after_cycles]
    after_releases = [
        r["t"]
        for r in after
        if r.get("type") == "event"
        and r.get("source") == "ttc"
        and r.get("severity") == "brake"
        and r.get("phase") == "release"
    ]
    assigned: set[int] = set()
    scans = sorted(bag.scans, key=arrival)
    scan_times = [arrival(s) for s in scans]
    drive_raw = sorted(bag.drive_raw, key=arrival)
    raw_times = [arrival(d) for d in drive_raw]
    drive = sorted(bag.drive, key=arrival)
    drive_times = [arrival(d) for d in drive]
    brakes, engagements = recorded_brakes(bag)
    before_trips = forward_trips(before)
    after_trips = forward_trips(after)
    report.before_watchdog_engages = watchdog_engages(before)
    report.after_watchdog_engages = watchdog_engages(after)
    report.recorded_watchdog_engages = sum(
        1 for e in bag.events if e.source == "watchdog" and e.phase == PHASE_ENGAGE
    )

    for t, cause in brakes:
        command = _command_at(drive_raw, raw_times, t) if drive_raw else DriveRecord(t, 0.0, 0.0, t)
        row = BrakeRow(t, cause, command.speed_mps, command.steering_rad)
        if measurements is not None and t in measurements:
            row.end_clearance_m, row.travel_m, row.approach_speed_mps = measurements[t]
        else:
            measure_brake(row, scans, scan_times, mount, body)
            if measurements is not None:
                measurements[t] = (row.end_clearance_m, row.travel_m, row.approach_speed_mps)
        matches = [r for r in before_trips if abs(r["t"] - t) <= MATCH_WINDOW_S]
        row.before = matches[0] if matches else None
        approach_start = max([rel for eng, rel in engagements if rel <= t], default=-math.inf)
        window_start = max(approach_start, t - APPROACH_WINDOW_S)
        in_window = [r for r in after_trips if window_start < r["t"] <= t + AFTER_SLACK_S]
        trip = in_window[0] if in_window else None
        if trip is None:
            # Already stopped by an earlier trip of the rule under test that had not released?
            earlier = [r for r in after_trips if approach_start < r["t"] <= window_start]
            if earlier and not any(earlier[-1]["t"] < rel <= t for rel in after_releases):
                trip = earlier[-1]
        if trip is not None:
            assigned.add(id(trip))
            row.after = trip
            row.lead_s = t - trip["t"]
            i_after = _last_before(scan_times, trip["t"])
            i_rec = _last_before(scan_times, t)
            if 0 <= i_after < i_rec:
                try:
                    x, y, _, _ = car_motion(scans[i_after], scans[i_rec], mount)
                    row.gain_m = math.hypot(x, y)
                except ValueError:
                    row.gain_m = 0.0
        cycle = _cycle_at(after_cycles, cycle_times, t)
        row.after_distance_at_brake_m = _distance(cycle["distance_m"]) if cycle else math.nan
        report.rows.append(row)

    clear = [
        r.travel_m / r.speed_mps
        for r in report.rows
        if not r.contact and r.speed_mps > 0.1 and math.isfinite(r.travel_m)
    ]
    report.travel_ratio_p90_s = float(np.percentile(clear, 90)) if clear else math.nan
    for row in report.rows:
        predict(row, report.travel_ratio_p90_s, trip_distance_m)

    # Trips of the rule under test that belong to no recorded brake.
    brake_times = [t for t, _ in brakes]
    for trip in after_trips:
        t = trip["t"]
        if id(trip) in assigned:
            continue
        if any(eng <= t <= rel for eng, rel in engagements):
            continue
        if any(t - AFTER_SLACK_S <= b < t for b in brake_times):
            continue
        i = _last_before(drive_times, t)
        following = [
            drive[k].speed_mps
            for k in range(max(i, 0), len(drive))
            if t < drive_times[k] <= t + FREE_DRIVE_WINDOW_S
        ]
        free = (
            bool(following)
            and min(following) > 0.0
            and not any(t < b <= t + FREE_DRIVE_WINDOW_S for b in brake_times)
        )
        report.extra_trips.append({**trip, "drove_freely": free})
    return report


def _distance(value: float | None) -> float:
    """A CLI distance: JSON null is +infinity (nothing in the path)."""
    return math.inf if value is None else float(value)


def _cycle_at(cycles: Sequence[dict[str, Any]], times: Sequence[float], t: float):
    i = _last_before(times, t)
    return cycles[i] if i >= 0 else None


def predict(
    row: BrakeRow, travel_ratio_p90_s: float, trip_distance_m: Callable[[float], float]
) -> None:
    """The predicted end clearance under the rule under test (module docstring)."""
    if not math.isfinite(row.end_clearance_m):
        row.basis = "no end clearance"
        return
    if math.isfinite(row.approach_speed_mps) and row.approach_speed_mps < AT_REST_MPS:
        # The car was not moving when the gate braked (a latch re-tripping at a standstill): no
        # approach, nothing to predict.
        row.basis = "at rest"
        return
    end = row.end_clearance_m
    if row.contact and math.isfinite(travel_ratio_p90_s) and math.isfinite(row.travel_m):
        end -= max(0.0, travel_ratio_p90_s * row.speed_mps - row.travel_m)
    if row.after is not None and row.lead_s >= -LATER_TOLERANCE_S:
        row.predicted_m = end + row.gain_m
        row.basis = "earlier" if row.gain_m > 0.0 else "same"
        return
    # Later than the recorded brake, or no trip before the bag shows the car stopping: the extra
    # distance the car covers before the rule under test trips, from its own in-path distance at
    # the recorded brake down to where it trips (its trip, or the distance it would trip at).
    d_now = row.after_distance_at_brake_m
    if row.after is not None:
        d_trip = _distance(row.after["distance_m"])
        row.basis = "later"
    else:
        d_trip = trip_distance_m(row.speed_mps)
        row.basis = "no trip"
    if math.isfinite(d_now) and math.isfinite(d_trip):
        row.predicted_m = end - max(0.0, d_now - d_trip)
    elif math.isinf(d_now):
        # Nothing in the path of the rule under test at the recorded brake: it never brakes here.
        row.basis = "never"


def _fmt(value: float | None, digits: int = 3) -> str:
    if value is None or not math.isfinite(value):
        return "-"
    return f"{value:.{digits}f}"


def table(rows: Sequence[BrakeRow], t0: float) -> str:
    """Markdown table, one line per recorded brake (times from t0)."""
    head = (
        "| t (s) | cause | request (m/s, rad) | measured speed (m/s) | travel (m) | end (m) | "
        "before trip | after trip | after lead (s) | after distance (m) | predicted end (m) | "
        "basis |\n|---|---|---|---|---|---|---|---|---|---|---|---|"
    )
    lines = [head]
    for r in rows:
        before = "yes" if r.before else "NO"
        after = r.after["cause"] if r.after else "none"
        lines.append(
            f"| {r.t_s - t0:.2f} | {r.cause} | {r.speed_mps:.2f}, {r.steering_rad:+.3f} | "
            f"{_fmt(r.approach_speed_mps, 2)} | {_fmt(r.travel_m)} | {_fmt(r.end_clearance_m)}"
            f"{' contact' if r.contact else ''} | {before} | {after} | {_fmt(r.lead_s, 2)} | "
            f"{_fmt(r.after['distance_m'] if r.after else None)} | {_fmt(r.predicted_m)} | "
            f"{r.basis} |"
        )
    return "\n".join(lines)


def summary(report: Report, min_clearance_m: float = 0.04) -> dict[str, Any]:
    rows = report.rows
    ends = [r.end_clearance_m for r in rows if math.isfinite(r.end_clearance_m)]
    predicted = [r.predicted_m for r in rows if math.isfinite(r.predicted_m)]
    replayable = [r for r in rows if r.basis in ("earlier", "same")]
    leads = [r.lead_s for r in rows if math.isfinite(r.lead_s)]
    return {
        "recorded_brakes": len(rows),
        "before_matched": sum(1 for r in rows if r.before is not None),
        "contacts_recorded": sum(1 for r in rows if r.contact),
        "end_median_m": float(np.median(ends)) if ends else math.nan,
        "after_tripped": sum(1 for r in rows if r.after is not None),
        "after_later": sum(1 for r in rows if r.basis == "later"),
        "after_none": sum(1 for r in rows if r.basis in ("no trip", "never")),
        "at_rest": sum(1 for r in rows if r.basis == "at rest"),
        "after_never": sum(1 for r in rows if r.basis == "never"),
        "replayable": len(replayable),
        "lead_median_s": float(np.median(leads)) if leads else math.nan,
        "predicted_median_m": float(np.median(predicted)) if predicted else math.nan,
        "predicted_min_m": float(min(predicted)) if predicted else math.nan,
        "predicted_below": sum(1 for p in predicted if p < min_clearance_m),
        "extra_trips": len(report.extra_trips),
        "extra_trips_free": sum(1 for e in report.extra_trips if e["drove_freely"]),
        "travel_ratio_p90_s": report.travel_ratio_p90_s,
        "watchdog": (
            report.recorded_watchdog_engages,
            report.before_watchdog_engages,
            report.after_watchdog_engages,
        ),
    }


def summary_text(s: dict[str, Any], min_clearance_m: float = 0.04) -> str:
    rec, wb, wa = s["watchdog"]
    lines = [
        (
            f"recorded forward obstacle brakes: {s['recorded_brakes']}, ended in contact "
            f"(<= {CONTACT_M:.2f} m): {s['contacts_recorded']}, median end clearance "
            f"{_fmt(s['end_median_m'])} m"
        ),
        (
            f"before-rule replay reproduces {s['before_matched']} of them within "
            f"{MATCH_WINDOW_S:.2f} s"
        ),
        (
            f"after rule: trips on {s['after_tripped']} approaches (median lead "
            f"{_fmt(s['lead_median_s'], 2)} s), later than recorded on {s['after_later']}, "
            f"no trip on {s['after_none']}, car at rest when braked {s['at_rest']}"
        ),
        (
            f"predicted end clearance: median {_fmt(s['predicted_median_m'])} m, min "
            f"{_fmt(s['predicted_min_m'])} m, below {min_clearance_m:.2f} m: "
            f"{s['predicted_below']}"
        ),
        (
            f"after-rule trips on no recorded approach: {s['extra_trips']} "
            f"({s['extra_trips_free']} where the car drove on freely)"
        ),
        (
            "stopping travel / request speed, p90 over brakes that stopped clear: "
            f"{_fmt(s['travel_ratio_p90_s'])} s"
        ),
        f"watchdog engages: recorded {rec}, before replay {wb}, after replay {wa}",
    ]
    return "\n".join(lines)
