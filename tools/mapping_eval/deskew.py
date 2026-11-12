"""Scan motion distortion correction (shared by the offline mapper and the scan-to-map tools)."""

from __future__ import annotations

import math

import numpy as np


def deskew_scan(
    ranges: np.ndarray,
    angle_min: float,
    angle_inc: float,
    v: float,
    wz: float,
    period_s: float,
    stamp_at: str,
    index_forward: bool,
    mount: tuple[float, float, float],
) -> np.ndarray:
    """Ranges as if the whole sweep were taken at the scan's stamp. The C1 takes ``period_s``
    (0.1 s) to turn once, so a car moving at 0.5 m/s and turning at 1 rad/s smears a wall seen
    at 1.5 m by 0.1 rad x 1.5 m = 15 cm: more than a tenth of a 1 m lane. Each beam is moved by the
    car's motion (constant ``v``, ``wz`` over the sweep) between its own time and the stamp, then
    the beams are put back on the original angle grid.

    ``stamp_at``: "start" (the stamp is the sweep's first beam) or "end". ``index_forward``: beam
    time increases with the beam index (else decreases)."""
    n = len(ranges)
    frac = (np.arange(n) / n) if index_forward else (1.0 - np.arange(n) / n)
    t_beam = frac * period_s - (period_s if stamp_at == "end" else 0.0)  # relative to the stamp
    theta = angle_min + angle_inc * np.arange(n)
    valid = np.isfinite(ranges) & (ranges > 0.0)
    r = np.where(valid, ranges, 0.0)
    mx, my, myaw = mount
    # Beam endpoint in the base frame at its own time.
    px = mx + r * np.cos(theta + myaw)
    py = my + r * np.sin(theta + myaw)
    # Car pose at t_beam relative to the pose at the stamp (rear-axle frame, constant twist).
    dpsi = wz * t_beam
    if abs(wz) < 1e-6:
        dx, dy = v * t_beam, np.zeros(n)
    else:
        dx = v / wz * np.sin(dpsi)
        dy = v / wz * (1.0 - np.cos(dpsi))
    qx = np.cos(dpsi) * px - np.sin(dpsi) * py + dx
    qy = np.sin(dpsi) * px + np.cos(dpsi) * py + dy
    # Back to the laser frame (the mount is a pure translation plus yaw) and to polar.
    lx, ly = qx - mx, qy - my
    c, sn = math.cos(-myaw), math.sin(-myaw)
    ux, uy = c * lx - sn * ly, sn * lx + c * ly
    new_r = np.hypot(ux, uy)
    new_theta = np.arctan2(uy, ux)
    # Re-bin: nearest beam of the original grid, nearest return wins.
    out = np.full(n, np.inf)
    idx = np.rint((np.mod(new_theta - angle_min + math.pi, 2.0 * math.pi) - math.pi) / angle_inc)
    idx = np.mod(idx, n).astype(int)
    for i in np.argsort(-new_r):  # far first, near overwrites
        if valid[i]:
            out[idx[i]] = new_r[i]
    # Fill single-beam holes the re-binning leaves (a sweep stretched by the motion).
    holes = ~np.isfinite(out)
    left, right = np.roll(out, 1), np.roll(out, -1)
    fill = holes & np.isfinite(left) & np.isfinite(right) & (np.abs(left - right) < 0.1)
    out[fill] = 0.5 * (left[fill] + right[fill])
    # Beams that had no return stay no return.
    return out
