"""Pure-math geometry helpers for closed-loop centerlines.

Every function here treats its ``x``/``y`` arrays as a CLOSED polyline (point ``n-1``
connects back to point ``0``) sampled roughly at uniform arc-length spacing -- that is what
``resample_closed_uniform`` produces, and what every other function in this module assumes
as its precondition. There is no ROS, no gym, no file I/O in this module: it is exercised
directly by hand-computed unit tests (claude-docs/12-testing.md L1) against synthetic
circles, where the true curvature/heading are known analytically.

Sign convention (claude-docs/06-vehicle-params.md, REP-103): yaw counter-clockwise
positive, steering LEFT positive. The finite-difference curvature formula used here
(``x'y'' - y'x''`` over speed-cubed, derivatives taken with respect to arc length) gives
POSITIVE curvature for a counter-clockwise (left-turning) closed curve and NEGATIVE
curvature for a clockwise (right-turning) one -- this is verified against an analytic
circle in ``tests/test_geometry.py`` for both orientations, matching the same sign
convention used for `racer_control`'s pure pursuit steering (left positive).
"""

from __future__ import annotations

import numpy as np


def arc_length_closed(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Cumulative arc length ``s`` (``s[0] == 0``) and per-segment length ``seg``.

    ``seg[i]`` is the distance from point ``i`` to point ``(i + 1) % n`` -- i.e. it
    includes the closing segment from the last point back to the first.
    """
    x_next = np.roll(x, -1)
    y_next = np.roll(y, -1)
    seg = np.hypot(x_next - x, y_next - y)
    s = np.concatenate(([0.0], np.cumsum(seg)[:-1]))
    return s, seg


def resample_closed_uniform(
    x: np.ndarray, y: np.ndarray, ds: float
) -> tuple[np.ndarray, np.ndarray]:
    """Resample a closed polyline to (approximately) uniform arc-length spacing ``ds``.

    Linear interpolation along cumulative arc length. The number of output points is
    ``round(total_length / ds)``, so actual spacing is ``total_length / n`` (close to
    ``ds`` but not exactly it, since the loop must close evenly).
    """
    if ds <= 0.0:
        raise ValueError(f"ds must be positive, got {ds}")
    s, seg = arc_length_closed(x, y)
    total_length = float(s[-1] + seg[-1])
    if total_length <= 0.0:
        raise ValueError("degenerate centerline: zero total arc length")
    n_out = max(round(total_length / ds), 3)

    # Append the closing point (index 0 again) so interpolation covers the full loop,
    # with its "s" being the total length rather than 0.
    s_closed = np.concatenate((s, [total_length]))
    x_closed = np.concatenate((x, [x[0]]))
    y_closed = np.concatenate((y, [y[0]]))

    s_query = np.linspace(0.0, total_length, n_out, endpoint=False)
    x_out = np.interp(s_query, s_closed, x_closed)
    y_out = np.interp(s_query, s_closed, y_closed)
    return x_out, y_out


def smooth_closed(x: np.ndarray, y: np.ndarray, window: int) -> tuple[np.ndarray, np.ndarray]:
    """Circular moving-average smoothing of a closed, uniformly-sampled polyline.

    ``window`` must be a positive odd integer (a centered average); ``window == 1`` is a
    no-op. This is the "curvature-smoothed path" step: a cheap stand-in for a full
    minimum-curvature optimization, documented as such in this package's docstring.
    """
    if window <= 1:
        return x.copy(), y.copy()
    if window % 2 == 0:
        raise ValueError(f"window must be odd, got {window}")
    n = len(x)
    if window >= n:
        raise ValueError(f"window ({window}) must be smaller than the point count ({n})")
    kernel = np.ones(window) / window
    # Circular convolution via padding with wrapped-around samples.
    half = window // 2
    x_padded = np.concatenate((x[-half:], x, x[:half]))
    y_padded = np.concatenate((y[-half:], y, y[:half]))
    x_smooth = np.convolve(x_padded, kernel, mode="valid")
    y_smooth = np.convolve(y_padded, kernel, mode="valid")
    return x_smooth, y_smooth


def heading_from_points_closed(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Tangent heading (rad) at each point via a central difference, wrapping at the seam."""
    x_next, x_prev = np.roll(x, -1), np.roll(x, 1)
    y_next, y_prev = np.roll(y, -1), np.roll(y, 1)
    return np.arctan2(y_next - y_prev, x_next - x_prev)


def curvature_from_points_closed(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Signed curvature (1/m) at each point via central-difference derivatives w.r.t. index.

    Uses the standard planar-curve formula ``kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^1.5``
    with ``'`` denoting a central difference over the (assumed near-uniform) sample spacing.
    Dividing by the sample spacing cancels out of the ratio (both the numerator's degree-3
    term and the denominator's degree-3 term scale the same way), so this is correct for any
    (near-)uniform spacing without needing to pass ``ds`` in -- verified against an analytic
    circle of known radius in ``tests/test_geometry.py``.
    """
    x_next, x_prev = np.roll(x, -1), np.roll(x, 1)
    y_next, y_prev = np.roll(y, -1), np.roll(y, 1)
    dx = (x_next - x_prev) / 2.0
    dy = (y_next - y_prev) / 2.0
    ddx = x_next - 2.0 * x + x_prev
    ddy = y_next - 2.0 * y + y_prev
    denom = np.power(dx * dx + dy * dy, 1.5)
    denom = np.where(denom < 1e-12, 1e-12, denom)
    return (dx * ddy - dy * ddx) / denom


def max_curvature_discontinuity(kappa: np.ndarray) -> float:
    """Largest absolute jump between curvature at adjacent (wraparound) samples.

    Used by the "curvature continuity" L1 invariant: a curvature-smoothed path should not
    have large point-to-point jumps.
    """
    return float(np.max(np.abs(kappa - np.roll(kappa, -1))))
