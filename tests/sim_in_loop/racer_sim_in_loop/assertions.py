"""L5 regression assertions over a recorded :class:`~racer_sim_in_loop.runner.TrajectoryRecord`.

Covers the four properties ``claude-docs/12-testing.md`` L5 names for the
tracker lap test (S.2) and the dynamics regression battery (S.6): lap
completion, wall contact, lap-time band, and trajectory-vs-reference
tolerance.

None of these hardcode a real env's info-dict schema (f1tenth_gym's exact
keys are an S.2/S.6 integration detail, not this scaffold's). Instead
``assert_lap_completed``/``assert_no_wall_contact`` take a small extraction
callable so a caller wires in its own env's schema explicitly, the same way
``racer_replay.tolerance`` forces an explicit tolerance rather than
guessing one.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence

from racer_sim_in_loop.runner import TrajectoryRecord, TrajectoryStep

ProgressFn = Callable[[TrajectoryRecord], float]
CollisionPredicate = Callable[[TrajectoryStep], bool]
Point2D = tuple[float, float]


def assert_lap_completed(
    record: TrajectoryRecord,
    *,
    progress_fn: ProgressFn,
    target_progress: float,
) -> None:
    """Assert the recorded episode reached at least ``target_progress``.

    ``progress_fn`` extracts a scalar progress measure (e.g. fraction of
    track completed, or arc length traveled) from the trajectory; what
    counts as "progress" is env-specific and left to the caller.
    """
    progress = progress_fn(record)
    if progress < target_progress:
        raise AssertionError(
            f"lap not completed: progress={progress!r} < target={target_progress!r} "
            f"({record.step_count} step(s) recorded, terminated={record.terminated}, "
            f"truncated={record.truncated})"
        )


def assert_no_wall_contact(
    record: TrajectoryRecord,
    *,
    collision_predicate: CollisionPredicate,
) -> None:
    """Assert no step in the trajectory reports a collision.

    ``collision_predicate`` inspects one :class:`TrajectoryStep` (typically
    its ``info`` dict) and returns whether that step is a collision.
    """
    for step in record.steps:
        if collision_predicate(step):
            raise AssertionError(f"wall contact detected at step {step.index}: info={step.info!r}")


def assert_lap_time_in_band(lap_time_s: float, *, low_s: float, high_s: float) -> None:
    """Assert a measured lap time falls within a committed [low, high] band."""
    if low_s > high_s:
        raise ValueError(f"invalid band: low_s={low_s} > high_s={high_s}")
    if not (low_s <= lap_time_s <= high_s):
        raise AssertionError(
            f"lap time {lap_time_s!r}s outside committed band [{low_s}, {high_s}]s"
        )


def lap_time_seconds(record: TrajectoryRecord, *, dt_s: float) -> float:
    """Elapsed sim time for a fixed-timestep episode: ``step_count * dt_s``."""
    return record.step_count * dt_s


def assert_trajectory_matches_reference(
    actual_xy: Sequence[Point2D],
    reference_xy: Sequence[Point2D],
    *,
    atol_m: float,
) -> None:
    """Assert a pointwise (index-aligned) trajectory matches a reference within ``atol_m``.

    Index-aligned, not path-aligned: this assumes ``actual_xy`` and
    ``reference_xy`` were sampled at the same seed/timestep/step count, as
    S.6's "fixed battery of maneuvers ... within tolerance of committed
    references" implies. Path-based (e.g. nearest-point) alignment is out
    of scope for this scaffold.
    """
    if len(actual_xy) != len(reference_xy):
        raise AssertionError(
            f"trajectory length mismatch: actual has {len(actual_xy)} point(s), "
            f"reference has {len(reference_xy)} point(s)"
        )
    for i, ((ax, ay), (rx, ry)) in enumerate(zip(actual_xy, reference_xy)):
        distance = math.hypot(ax - rx, ay - ry)
        if distance > atol_m:
            raise AssertionError(
                f"trajectory diverges from reference at index {i}: "
                f"actual=({ax}, {ay}) reference=({rx}, {ry}) "
                f"distance={distance:.6g}m > atol={atol_m}m"
            )
