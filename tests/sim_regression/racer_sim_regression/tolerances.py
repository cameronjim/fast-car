"""Per-field tolerances for the S.6 golden comparisons (claude-docs/12-testing.md L4/L5:
"never exact float equality" -- racer_replay.tolerance.FieldTolerance forces every field
compared here to state one explicitly, see tests/replay_harness/racer_replay/tolerance.py's
``MissingToleranceError``).

Calibration story (read before touching any value below):

racer_gym's own L1 determinism test (sim/racer_gym/tests/test_determinism.py) already
proves same-seed-same-commands gives a BIT-IDENTICAL trajectory on one machine; this
package's own tests/test_battery_determinism.py proves the same for the full battery. So
these tolerances are NOT absorbing run-to-run noise -- there isn't any on a single
platform. They exist to absorb legitimate cross-platform floating-point differences between
the machine that generated ``references/`` (this task's local uv venv, arm64 macOS) and
whatever machine later CHECKS a golden (the ubuntu-latest x86_64 GitHub Actions runner the
sim-regression-battery CI job runs on, see .github/workflows/ci.yml): different libm
sin/cos/atan2 implementations can differ by a few ULPs, and RK4-integrating that difference
over a few hundred steps of a nonlinear (though well-damped and non-chaotic in the regime
these maneuvers stay in -- claude-docs/00-project-overview.md's regime table excludes
claims at the friction limit) ODE can amplify it somewhat.

The values below are a first, deliberately modest cut -- the exact same "commit, then widen
from measured CI variance if it's real, and say why" approach roadmap task S.2 used for its
tracker-lap-time band (see that task's note in claude-docs/01-roadmap.md), not a value
picked to make a known-bad trajectory pass. If the sim-regression-battery CI job's first
real run shows a genuine, understood cross-platform gap wider than what is stated here, the
fix is to widen the specific field's tolerance and say so in the PR -- never to delete or
skip the comparison (claude-docs/12-testing.md: "Never weaken a tolerance... to make a
build pass" governs the CODE gate, but a documented, measured widening for a legitimate
cross-platform reason is the same move S.2 already made and left as precedent).

``t_s`` is the one exception: it is ``(step_index + 1) * dt_s``, plain double
multiplication of an exact literal, which IEEE 754 guarantees is bit-identical on every
conforming platform -- so it is held to exact equality (``atol=0, rtol=0``, an explicit,
deliberate "must match exactly" per ``FieldTolerance``'s own docstring, not a silent
default). The ``*_mps``/``*_rad``/``*_radps``/``*_mps2`` command-parameter fields
(``target_speed_mps``, ``steer_target_rad``, ...) are likewise exact: they are the literal
constants each maneuver commands, not something the dynamics under test could perturb.
"""

from __future__ import annotations

from ._replay_import import ensure_importable

ensure_importable()

from racer_replay.tolerance import FieldTolerance

_EXACT_COMMAND = FieldTolerance(
    atol=0.0,
    rtol=0.0,
    note="a literal command parameter, not dynamics output -- exact by construction",
)
_EXACT_TIME = FieldTolerance(
    atol=0.0, rtol=0.0, note="(step_index+1)*dt_s; exact under IEEE 754 on any conforming platform"
)

# "Settling time" / "time to X" fields are step-quantized crossings (metrics.py): a few
# ULPs of state difference can shift which discrete sample first satisfies the
# crossing condition by a step or two. One dt=0.01s step's worth of jitter, doubled for
# margin, is generous without hiding a real multi-step (i.e. multi-tens-of-ms) regression.
_CROSSING_TIME_TOLERANCE = FieldTolerance(
    atol=0.02,
    note="settling/crossing time is step-quantized (dt=0.01s); ~2 steps of jitter allowed "
    "for cross-platform ULP-level state differences shifting the crossing sample, see module docstring",
)

_STATE_HEADROOM_NOTE = "cross-platform (arm64 macOS generation vs. ubuntu x86_64 CI) libm/RK4 headroom, see module docstring"

TRAJECTORY_TOLERANCES: dict[str, FieldTolerance] = {
    "t_s": _EXACT_TIME,
    "x_m": FieldTolerance(atol=1e-3, note=f"position, 1 mm: {_STATE_HEADROOM_NOTE}"),
    "y_m": FieldTolerance(atol=1e-3, note=f"position, 1 mm: {_STATE_HEADROOM_NOTE}"),
    "yaw_rad": FieldTolerance(atol=1e-5, note=_STATE_HEADROOM_NOTE),
    "speed_mps": FieldTolerance(atol=1e-5, rtol=1e-6, note=_STATE_HEADROOM_NOTE),
    "yaw_rate_radps": FieldTolerance(atol=1e-5, note=_STATE_HEADROOM_NOTE),
    "slip_angle_rad": FieldTolerance(atol=1e-5, note=_STATE_HEADROOM_NOTE),
    "steer_angle_rad": FieldTolerance(atol=1e-5, note=_STATE_HEADROOM_NOTE),
}

SUMMARY_TOLERANCES: dict[str, FieldTolerance] = {
    # command parameters (exact)
    "target_speed_mps": _EXACT_COMMAND,
    "steer_target_rad": _EXACT_COMMAND,
    "cruise_speed_mps": _EXACT_COMMAND,
    "speed_target_mps": _EXACT_COMMAND,
    "brake_target_speed_mps": _EXACT_COMMAND,
    # derived scalars (dynamics output -- cross-platform headroom)
    "steady_state_speed_mps": FieldTolerance(atol=1e-5, note=_STATE_HEADROOM_NOTE),
    "steady_state_yaw_rate_radps": FieldTolerance(atol=1e-5, note=_STATE_HEADROOM_NOTE),
    "steady_state_slip_angle_rad": FieldTolerance(atol=1e-5, note=_STATE_HEADROOM_NOTE),
    "steady_state_lateral_accel_mps2": FieldTolerance(
        atol=1e-4, note=f"v*yaw_rate, {_STATE_HEADROOM_NOTE}"
    ),
    "yaw_rate_overshoot_frac": FieldTolerance(atol=1e-3, note=_STATE_HEADROOM_NOTE),
    "mean_decel_mps2": FieldTolerance(atol=1e-3, note=_STATE_HEADROOM_NOTE),
    "settling_time_s": _CROSSING_TIME_TOLERANCE,
    "yaw_rate_settling_time_s": _CROSSING_TIME_TOLERANCE,
    "time_to_near_zero_s": _CROSSING_TIME_TOLERANCE,
}
