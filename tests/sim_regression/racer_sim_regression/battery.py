"""The S.6 sim dynamics regression battery: top-level API (roadmap task S.6,
claude-docs/12-testing.md L5 "Model-upgrade regression").

Reused, not reinvented: this module is a thin registry + runner over racer_gym (the
dynamics under test) and racer_replay (the golden/tolerance engine,
tests/replay_harness/racer_replay/golden.py + tolerance.py). See maneuvers.py for what each
maneuver does, and tolerances.py for the tolerance calibration story.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any

import racer_gym

from . import maneuvers
from ._replay_import import ensure_importable

ensure_importable()

from racer_replay.golden import GoldenComparisonResult, compare_to_golden

from .provenance import build_provenance_meta
from .tolerances import SUMMARY_TOLERANCES, TRAJECTORY_TOLERANCES

REFERENCES_DIR = Path(__file__).resolve().parent / "references"

# Fixed and committed -- the point is that this is the SAME seed every run, not the
# particular value (same spirit as sim/racer_gym/tests/test_determinism.py's seed=123).
BATTERY_SEED = 1234

Record = dict[str, Any]
ManeuverFn = Callable[..., tuple[list[Record], list[Record], Any]]

MANEUVERS: dict[str, ManeuverFn] = {
    "throttle_step": maneuvers.throttle_step,
    "steering_step": maneuvers.steering_step,
    "constant_radius_circle": maneuvers.constant_radius_circle,
    "coastdown": maneuvers.coastdown,
}


def _trajectory_path(name: str, references_dir: Path) -> Path:
    return references_dir / f"{name}_trajectory.json"


def _summary_path(name: str, references_dir: Path) -> Path:
    return references_dir / f"{name}_summary.json"


def run_maneuver(
    name: str, *, seed: int = BATTERY_SEED, vehicle_params: Any = None
) -> tuple[list[Record], list[Record], dict[str, Any]]:
    """Run one named maneuver, returning ``(trajectory_records, summary_records, meta)``.

    ``vehicle_params`` defaults to the real ``config/vehicle_params.yaml`` (loaded once
    here, then passed through explicitly to both the maneuver and the provenance builder,
    so both see the exact same object -- and so a caller can pass an in-memory-perturbed
    copy, as the injected-change canary test does, and have it actually take effect).
    """
    if name not in MANEUVERS:
        raise KeyError(f"unknown maneuver {name!r}; known maneuvers: {sorted(MANEUVERS)}")
    if vehicle_params is None:
        vehicle_params = racer_gym.load_vehicle_params()

    trajectory, summary, dyn_params_result = MANEUVERS[name](
        seed=seed, vehicle_params=vehicle_params
    )
    meta = build_provenance_meta(
        seed=seed, dyn_params_result=dyn_params_result, vehicle_params=vehicle_params
    )
    return trajectory, summary, meta


def run_battery(
    *, seed: int = BATTERY_SEED, vehicle_params: Any = None
) -> dict[str, tuple[list[Record], list[Record], dict[str, Any]]]:
    """Run every maneuver in the battery. See :func:`run_maneuver` for ``vehicle_params``."""
    if vehicle_params is None:
        vehicle_params = racer_gym.load_vehicle_params()
    return {
        name: run_maneuver(name, seed=seed, vehicle_params=vehicle_params) for name in MANEUVERS
    }


def compare_battery_to_references(
    *,
    seed: int = BATTERY_SEED,
    vehicle_params: Any = None,
    references_dir: Path = REFERENCES_DIR,
) -> dict[str, dict[str, GoldenComparisonResult]]:
    """Run the full battery and compare each maneuver's trajectory + summary against its
    committed reference in ``references_dir``."""
    results: dict[str, dict[str, GoldenComparisonResult]] = {}
    for name, (trajectory, summary, _meta) in run_battery(
        seed=seed, vehicle_params=vehicle_params
    ).items():
        results[name] = {
            "trajectory": compare_to_golden(
                trajectory, _trajectory_path(name, references_dir), TRAJECTORY_TOLERANCES
            ),
            "summary": compare_to_golden(
                summary, _summary_path(name, references_dir), SUMMARY_TOLERANCES
            ),
        }
    return results


def assert_battery_matches_references(**kwargs: Any) -> None:
    """Raise a single, readable ``AssertionError`` naming every failing maneuver/kind if any
    comparison from :func:`compare_battery_to_references` is not ``ok``."""
    results = compare_battery_to_references(**kwargs)
    failures = []
    for name, per_kind in results.items():
        for kind, result in per_kind.items():
            if not result.ok:
                failures.append(f"[{name}/{kind}]\n{result.format_report()}")
    if failures:
        raise AssertionError(
            "sim dynamics regression battery FAILED against committed references "
            f"({REFERENCES_DIR}):\n\n" + "\n\n".join(failures)
        )
