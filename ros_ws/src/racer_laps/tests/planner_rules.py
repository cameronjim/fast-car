"""The planner's validation rules for a LapGuidance message (brief 26), as a predicate on the
lap memory's `Guidance`. Shared by the property test and the node test."""

from __future__ import annotations

import numpy as np
from racer_laps.engine import Guidance


def planner_accepts(g: Guidance) -> bool:
    n = g.s_m.shape[0]
    if n == 0:
        return g.v_ref_mps.shape[0] == 0 and g.cost_to_go_s.shape[0] == 0
    if not (2 <= n <= 128) or g.v_ref_mps.shape[0] != n or g.cost_to_go_s.shape[0] != n:
        return False
    s = g.s_m.astype(np.float32)
    v = g.v_ref_mps.astype(np.float32)
    c = g.cost_to_go_s.astype(np.float32)
    if not (np.all(np.isfinite(s)) and np.all(np.isfinite(v)) and np.all(np.isfinite(c))):
        return False
    if not np.all(np.abs(np.diff(s.astype(np.float64)) - 0.1) <= 1e-3):
        return False
    if not np.all(v >= 0.0):
        return False
    return bool(np.all(np.diff(c.astype(np.float64)) <= 1e-3))
