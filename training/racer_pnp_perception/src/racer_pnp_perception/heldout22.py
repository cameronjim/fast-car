"""The runs R0 to R4 of the 2026-11-22 bag as a `Split` (held-out set 22 of the pre-registration
pnp-v0.6-perception-fused, "whole": every moving scan of the five runs).

The run list is the five launch logs of the session (`pnp-*.log` next to the bag), in the order they
were started. A run spans the first to the last ROS time stamp in its log, on the split's time axis
(seconds after the first /scan message of the bag by recorder log time). Directions: the three
`observe` runs are the three counter attempts of D10-C0 (ccw), `cw-check` and `dv0-cw` are clockwise
(`memory/car-session-2026-11-22.md`). A label's submap must lie inside one run, as everywhere.

Nothing here reads a scan: `first_scan_log_time_s` is one message's recorder time.
"""

from __future__ import annotations

import re
from pathlib import Path

import yaml

from racer_pnp_perception.gp1d import HELDOUT_22_BAG
from racer_pnp_perception.split import Split, parse_split

SESSION_DIR = Path("/Users/cameronjim/code/car-data/session-2026-11-22")
RUNS = (
    ("R0", "pnp-observe-205702.log", "ccw"),
    ("R1", "pnp-observe-210013.log", "ccw"),
    ("R2", "pnp-observe-210414.log", "ccw"),
    ("R3", "pnp-cw-check-210748.log", "cw"),
    ("R4", "pnp-dv0-cw-211016.log", "cw"),
)
_STAMP = re.compile(r"\[(\d{10}\.\d+)\]")


def log_span_s(log: Path) -> tuple[float, float]:
    """First and last ROS time stamp (s) in a launch log."""
    stamps = [float(m.group(1)) for m in _STAMP.finditer(log.read_text(errors="replace"))]
    if not stamps:
        raise ValueError(f"{log}: no ROS time stamp")
    return min(stamps), max(stamps)


def run_windows(session_dir: Path, first_scan_log_s: float) -> list[dict[str, object]]:
    """The five runs as split segments on the bag's time axis."""
    out: list[dict[str, object]] = []
    for name, log_name, direction in RUNS:
        lo, hi = log_span_s(session_dir / log_name)
        out.append(
            {
                "name": name,
                "from": round(lo - first_scan_log_s, 3),
                "to": round(hi - first_scan_log_s, 3),
                "direction": direction,
                "perception": "heldout",
            }
        )
    return out


def split_22(session_dir: Path, first_scan_log_s: float) -> Split:
    doc = {
        "split_id": "pnp-v0.6-heldout-22",
        "bags": {
            HELDOUT_22_BAG: {
                "first_scan_log_time_s": first_scan_log_s,
                "segments": run_windows(session_dir, first_scan_log_s),
            }
        },
    }
    return parse_split(yaml.safe_dump(doc), "perception", "heldout-22")
