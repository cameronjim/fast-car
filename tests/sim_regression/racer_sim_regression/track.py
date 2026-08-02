"""The synthetic reference line shared by every maneuver in the battery.

A single generous, straight, wide track -- long enough that none of the battery's
open-loop maneuvers (which do not steer to follow it; they are fixed throttle/steer
commands, not a tracker-driven lap, see scenarios.py) can reach a wall within its step
budget. Built with ``Track.from_refline`` the same way
sim/racer_gym/tests/test_determinism.py and tests/sim_in_loop's gym-backed test build their
synthetic tracks: no network access, fully reproducible from the pinned f1tenth_gym commit
alone.
"""

from __future__ import annotations

import numpy as np
from f1tenth_gym.envs.track import Track

# Longest maneuver here is ~10 sim-seconds at up to 5 m/s -> well under 100m of travel even
# allowing for lateral drift during the steering/circle maneuvers; 400m is ample headroom.
TRACK_LENGTH_M = 400.0
TRACK_POINTS = 800
# Only feeds the refline's velx column (f1tenth_gym's own raceline-speed hint for
# rendering/lap-progress bookkeeping) -- irrelevant to this battery's open-loop commands.
TRACK_REFERENCE_SPEED_MPS = 6.0


def build_battery_track() -> Track:
    xs = np.linspace(0.0, TRACK_LENGTH_M, TRACK_POINTS)
    ys = np.zeros_like(xs)
    velxs = np.full_like(xs, TRACK_REFERENCE_SPEED_MPS)
    return Track.from_refline(x=xs, y=ys, velx=velxs)
