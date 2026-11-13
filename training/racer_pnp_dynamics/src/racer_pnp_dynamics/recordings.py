"""Loading the real bags with their cached ICP motion (thin glue, run on real data only)."""

from __future__ import annotations

import logging
from pathlib import Path

from .bags import Run, calibration, load_run
from .dataset import Segment, drop_nested, make_segments
from .icp_motion import MotionSeries, compute_motion, driving_intervals, load_motion, save_motion
from .vehicle import Vehicle

LOG = logging.getLogger(__name__)

# Bags recorded in the units of the current vehicle_params (0.14.0): nothing is converted. Every
# other bag must be dated 2026-11-11 or earlier and was recorded under 0.13.0 (converted). A bag
# in neither group is refused rather than guessed.
CURRENT_UNITS_BAGS = frozenset({"2026-11-12T02-57-50_car_teleop", "2026-11-12T03-04-56_car_teleop"})
LAST_OLD_UNITS_DATE = "2026-11-11"


def recorded_for(bag_name: str) -> calibration.Recorded | None:
    """The calibration a bag was recorded under, None for a bag in current units."""
    if bag_name in CURRENT_UNITS_BAGS:
        return None
    if bag_name[:10] <= LAST_OLD_UNITS_DATE:
        return calibration.RECORDED_0_13_0
    raise ValueError(
        f"{bag_name}: recorded after {LAST_OLD_UNITS_DATE} and not listed in CURRENT_UNITS_BAGS; "
        "say which units it is in before it is used"
    )


def bag_dirs(bags_dir: Path) -> list[Path]:  # pragma: no cover - real data
    return sorted(p for p in Path(bags_dir).iterdir() if p.is_dir())


def load_recordings(  # pragma: no cover - real data
    bags_dir: Path, vehicle: Vehicle, cache_dir: Path, workers: int = 1
) -> tuple[list[Run], dict[str, MotionSeries], list[str]]:
    """Runs (nested copies dropped), their motion series and the names of the dropped bags."""
    runs = [load_run(p, recorded_for(p.name)) for p in bag_dirs(bags_dir)]
    kept = drop_nested(runs)
    kept_names = {r.name for r in kept}
    dropped = sorted(r.name for r in runs if r.name not in kept_names)
    motions: dict[str, MotionSeries] = {}
    for run in kept:
        cache = Path(cache_dir) / f"{run.name}.motion.npz"
        if cache.is_file():
            motions[run.name] = load_motion(cache)
        else:
            LOG.info("ICP over %s (%d scans)", run.name, len(run.scans))
            motions[run.name] = compute_motion(
                run.scans, vehicle, driving_intervals(run), workers=workers
            )
            save_motion(cache, motions[run.name])
    return kept, motions, dropped


def all_segments(runs: list[Run], motions: dict[str, MotionSeries]) -> list[Segment]:
    out: list[Segment] = []
    for run in runs:
        out.extend(make_segments(run, motions.get(run.name)))
    return out
