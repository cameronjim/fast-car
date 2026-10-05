# reference line geometry for classical planners

# maps with a raceline csv ship a real speed profile in vxs; generated racelines carry one
# built from curvature by f1rl.track.generate_raceline, which planners read the same way
from .clearance import ClearanceMap
from .raceline_index import RacelineIndex
from .raceline_io import (
    generated_raceline_path,
    raceline_index_from_csv,
    read_raceline_csv,
    write_raceline_csv,
)

__all__ = [
    "ClearanceMap",
    "RacelineIndex",
    "generated_raceline_path",
    "raceline_index_from_csv",
    "read_raceline_csv",
    "write_raceline_csv",
]
