"""Per-scan scores of a predicted track against a self-label, shared by the training validation
and the GP1 evaluation so both use one definition.

E_c of the pre-registration: for one scan, the mean distance between the predicted and the
labelled centreline points over the stations valid in both (`valid_count` of each); E_far is the
same over stations 8 to 15 only. A scan with no station valid in both has no error (NaN).
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from racer_track_perception import track as tr

FAR_FIRST_STATION = 8


def scan_centreline_errors(
    track: tr.LocalTrackData,
    label: NDArray[np.floating],
    label_valid: int,
) -> tuple[float, float, int]:
    """(E_c, E_far, stations in common) of one scan. NaN where there is no common station (in
    the far range for E_far)."""
    n = min(int(track.valid_count), int(label_valid), tr.K_STATIONS)
    if n <= 0:
        return float("nan"), float("nan"), 0
    d = np.hypot(
        track.x_m[:n].astype(np.float64) - label[:n, 0],
        track.y_m[:n].astype(np.float64) - label[:n, 1],
    )
    far = d[FAR_FIRST_STATION:]
    return float(d.mean()), float(far.mean()) if len(far) else float("nan"), n
