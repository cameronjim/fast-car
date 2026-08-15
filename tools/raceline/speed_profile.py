"""Curvature-and-friction-limited target speed profile.

Two limits are combined, both sourced from generated `vehicle_params` bindings
(``params_loader.py``) -- never hand-typed (CLAUDE.md invariant 2):

  1. A curvature/friction cap at each point: ``v <= sqrt(a_lat_max / |kappa|)``, capped at
     ``v_max``. ``vehicle_params`` has no separate lateral-acceleration limit field (that is
     a Phase 5 envelope-tuning value, currently null -- claude-docs/06-vehicle-params.md);
     this uses ``actuation.max_acceleration_mps2`` (the gym-derived ``a_max``) as an
     isotropic friction-circle surrogate for the lateral limit too. This is a documented
     simplification, not a silent one: a real lateral limit (from a Pacejka fit or a
     measured friction circle) should replace it once Phase 3 sysid exists.
  2. A longitudinal accel/decel limit between adjacent points, enforced by a forward
     (acceleration-limited) and backward (deceleration-limited) pass around the closed
     loop, repeated a few times to converge at the seam -- the standard racing-line speed
     profile algorithm.
"""

from __future__ import annotations

import math

import numpy as np


def curvature_speed_cap(kappa: np.ndarray, a_lat_max_mps2: float, v_max_mps: float) -> np.ndarray:
    """Per-point speed cap from lateral friction: ``sqrt(a_lat_max / |kappa|)``, capped at
    ``v_max_mps``. Straight segments (``kappa == 0``) are capped only by ``v_max_mps``.
    """
    if a_lat_max_mps2 <= 0.0:
        raise ValueError(f"a_lat_max_mps2 must be positive, got {a_lat_max_mps2}")
    if v_max_mps <= 0.0:
        raise ValueError(f"v_max_mps must be positive, got {v_max_mps}")
    kappa_abs = np.abs(kappa)
    with np.errstate(divide="ignore"):
        v_curve = np.where(
            kappa_abs > 1e-9, np.sqrt(a_lat_max_mps2 / np.maximum(kappa_abs, 1e-12)), v_max_mps
        )
    return np.minimum(v_curve, v_max_mps)


def accel_limited_profile_closed(
    v_cap: np.ndarray, seg_lengths_m: np.ndarray, a_max_mps2: float, num_passes: int = 3
) -> np.ndarray:
    """Forward/backward accel-limited smoothing of a per-point speed cap around a closed loop.

    ``seg_lengths_m[i]`` is the distance from point ``i`` to point ``(i+1) % n``
    (``geometry.arc_length_closed``'s ``seg``). The result never exceeds ``v_cap``
    elementwise, and the implied longitudinal acceleration between every adjacent pair
    (both directions around the loop) never exceeds ``a_max_mps2`` (up to the pass count's
    convergence -- verified empirically in tests, not just asserted by construction, since
    a closed loop's forward pass depends on the backward pass's own last output at the seam).
    """
    if a_max_mps2 <= 0.0:
        raise ValueError(f"a_max_mps2 must be positive, got {a_max_mps2}")
    n = len(v_cap)
    v = v_cap.astype(float).copy()
    for _ in range(num_passes):
        # Forward pass: v[i] limited by how fast the car could accelerate from v[i-1].
        for i in range(n):
            j = (i - 1) % n
            v_allowed = math.sqrt(max(v[j] * v[j] + 2.0 * a_max_mps2 * seg_lengths_m[j], 0.0))
            v[i] = min(v[i], v_allowed)
        # Backward pass: v[i] limited by how fast the car could decelerate to v[i+1].
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            v_allowed = math.sqrt(max(v[j] * v[j] + 2.0 * a_max_mps2 * seg_lengths_m[i], 0.0))
            v[i] = min(v[i], v_allowed)
    return v
