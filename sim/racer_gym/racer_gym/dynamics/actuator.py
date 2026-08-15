"""First-order actuator dynamics for steering and throttle.

Roadmap S.1 requirement 3 (claude-docs/07-sim-and-sysid.md): "Explicit first-order actuator
dynamics for steering and throttle (measured time constants from vehicle_params)."

Models the classic first-order lag: tau * d(y)/dt = target - y. The EXACT discrete-time
solution for a fixed step dt is used (not a first-order Euler approximation of the ODE), so
the step response is correct at any dt/tau ratio, not just dt << tau:

    y[k+1] = y[k] + (1 - exp(-dt/tau)) * (target - y[k])

At t = tau (i.e. after `tau/dt` steps of a fixed step dt, or in continuous time), a unit step
response of this system reaches 1 - exp(-1) ~= 63.2% of the way to the target -- the standard
first-order step-response fact used in tests/test_actuator.py::test_step_response_63_percent.

`tau_s <= 0` means "instantaneous" (no lag at all): this is both a legitimate degenerate case
of the ODE (tau -> 0 => output snaps to target) and the documented fallback used by
racer_gym/params.py when `vehicle_params.yaml`'s `steering.time_constant_s` /
`actuation.throttle_time_constant_s` is null (Phase 2/3 not measured yet) -- see that
module's FALLBACK_FLAGS. An unfitted actuator behaves exactly like stock f1tenth_gym's
rate-limited-but-otherwise-instantaneous actuation, which is the honest "we have not modeled
this yet" baseline.
"""

from __future__ import annotations

import math


class FirstOrderActuator:
    """Stateful first-order lag from a raw target to an "actual" output value.

    `output` is always a convex combination of the previous `output` and `target` (see
    `step`), which is exactly the L2 "monotone first-order" property required by
    claude-docs/12-testing.md: for any target, the new output never overshoots and always
    lies between the previous output and the target.
    """

    def __init__(self, tau_s: float, dt_s: float, initial_output: float = 0.0) -> None:
        if tau_s < 0.0:
            raise ValueError("tau_s must be >= 0")
        if dt_s <= 0.0:
            raise ValueError("dt_s must be > 0")
        self.tau_s = tau_s
        self.dt_s = dt_s
        self.output = initial_output
        # alpha == 1.0 (instantaneous tracking) when tau_s <= 0.
        self._alpha = 1.0 if tau_s <= 0.0 else 1.0 - math.exp(-dt_s / tau_s)

    def reset(self, output: float = 0.0) -> None:
        self.output = output

    def step(self, target: float) -> float:
        self.output = self.output + self._alpha * (target - self.output)
        return self.output
