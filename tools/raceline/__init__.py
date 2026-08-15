"""Offline raceline optimizer (roadmap task S.2, claude-docs/02-repo-layout.md).

This is a `tools/` package, not a ROS/Python library shipped on the vehicle: it is run
once (by a person or CI, never at 50 Hz) to turn a track description into a committed
raceline file under `config/tracks/<venue>_<layout>/raceline.csv`
(claude-docs/02-repo-layout.md). `ros_ws/src/racer_control` loads that committed file at
runtime; it does not run any of this package's code.

Pipeline (see each module's docstring for detail):

  1. A centerline is obtained either from a track mask (`occupancy.py`, a simplified
     drivable/non-drivable grid -- see that module's docstring for how this differs from a
     full ROS map_server/PGM map) or from a synthetic analytic generator (`synthetic_tracks.py`,
     used for the reference track and for hand-computed geometry tests).
  2. The centerline is resampled to uniform arc-length spacing and lightly smoothed
     (`geometry.py`) -- this is the "curvature-smoothed path" step CLAUDE.md's task brief
     calls for, in place of a full minimum-curvature QP optimization (a stated, documented
     simplification for this first version).
  3. Heading and curvature are computed from the smoothed centerline (`geometry.py`).
  4. A curvature-and-friction-limited target speed profile is computed from vehicle
     parameters (`speed_profile.py`), using the generated `vehicle_params` bindings
     (`params_loader.py`) -- never a hand-typed physical constant, per CLAUDE.md invariant 2.
  5. The result (`raceline.py`'s `Raceline` dataclass) is written to a provenanced CSV
     (`io.py`): s, x, y, heading, curvature, target_speed, plus a header recording the tool
     version and the vehicle_params schema_version/sysid_session_id that produced it.

`cli.py` wires this into a command run by a person (`python -m raceline.cli ...`); it is
not imported by anything on the control path.
"""
