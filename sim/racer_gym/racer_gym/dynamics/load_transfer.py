"""Longitudinal and lateral load transfer for the racer_gym single-track model.

Roadmap S.1 requirement 1 (claude-docs/07-sim-and-sysid.md). All inputs are SI (kg, m, m/s^2,
N) per CLAUDE.md invariant 4; none of the numbers here are hand-written physical constants --
callers pass mass_kg / cg_height_m / cg_to_front_axle_m / cg_to_rear_axle_m / track_width_m
straight from the generated `vehicle_params` binding (see racer_gym/params.py).

Two separate, independently testable effects:

  * `longitudinal_load_transfer`: redistributes normal load between the front and rear axle
    under acceleration/braking. This is a pure redistribution -- front + rear always sums to
    the static total `mass_kg * g` (see the L2 property test in
    tests/test_load_transfer.py::test_longitudinal_conserves_total_load).

  * `lateral_grip_derate`: lateral (side-to-side) weight transfer during cornering. A
    single-track (bicycle) model has no left/right wheels, so this cannot be expressed as a
    load redistribution the way the longitudinal case can. What it changes instead is the
    axle's *total* cornering capacity: because tire lateral force is a concave (sub-linear)
    function of normal load (a standard tire-engineering fact, sometimes called "tire load
    sensitivity" -- see e.g. Milliken & Milliken, "Race Car Vehicle Dynamics", ch. 4), moving
    load from the inside tire to the outside tire during a corner *reduces* the axle's total
    available lateral force even though the axle's total normal load is unchanged. This
    function returns a dimensionless derate factor in (0, 1] that tire.py multiplies into the
    effective normal load fed to the Pacejka curve.

    Reproducing this exactly requires the per-side load split, i.e. `track_width_m`, which is
    NOT modeled by the stock f1tenth_gym single-track model and is null in
    config/vehicle_params.yaml pending a Phase 1 chassis measurement (see
    claude-docs/06-vehicle-params.md). When it is None, this function returns a derate factor
    of 1.0 for both axles -- i.e. no lateral load transfer effect -- which is exactly stock
    f1tenth_gym's (non-)treatment of this effect. Callers must flag this fallback (see
    racer_gym/params.py FALLBACK_FLAGS).

    `_TIRE_LOAD_SENSITIVITY_EXPONENT` below is a fixed, generic Magic-Formula/tire-physics
    modeling constant (how sub-linearly lateral force grows with normal load), not a
    per-vehicle fitted quantity -- it does not belong in vehicle_params.yaml any more than the
    choice of RK4 vs. Euler integration does. It is not superseded by a Phase 3 sysid Pacejka
    fit: that fit produces `d_peak_n` at one reference load, and this exponent is what lets
    the model extrapolate D away from that reference load regardless of whether B/C/D/E are
    fitted or placeholder (see racer_gym/dynamics/tire.py).
"""

from __future__ import annotations

import dataclasses

GRAVITY_MPS2 = 9.81  # standard gravity; a physical/mathematical constant, not a vehicle param

# See module docstring: a tire-physics modeling constant, not a fitted vehicle parameter.
# Typical literature range for radial tires is roughly 0.8-1.0; 0.9 is a conservative,
# middle-of-the-road default. Only affects the *derate*, and only when track_width_m and a
# nonzero lateral acceleration are both present -- it has zero effect at ay=0 or when
# track_width_m is None (derate == 1.0 exactly in both cases, see below).
TIRE_LOAD_SENSITIVITY_EXPONENT = 0.9


@dataclasses.dataclass(frozen=True)
class StaticAxleLoads:
    front_n: float
    rear_n: float


def static_axle_loads(
    mass_kg: float, cg_to_front_axle_m: float, cg_to_rear_axle_m: float
) -> StaticAxleLoads:
    """Static (zero-acceleration) normal load on each axle.

    Standard result for a two-axle rigid body in static equilibrium: the front axle carries
    the fraction of weight proportional to the REAR cg distance (and vice versa), because a
    CG closer to an axle puts more of the vehicle's weight on it.
    """
    wheelbase_m = cg_to_front_axle_m + cg_to_rear_axle_m
    total_n = mass_kg * GRAVITY_MPS2
    front_n = total_n * cg_to_rear_axle_m / wheelbase_m
    rear_n = total_n * cg_to_front_axle_m / wheelbase_m
    return StaticAxleLoads(front_n=front_n, rear_n=rear_n)


@dataclasses.dataclass(frozen=True)
class AxleLoads:
    front_n: float
    rear_n: float


def longitudinal_load_transfer(
    mass_kg: float,
    cg_height_m: float,
    cg_to_front_axle_m: float,
    cg_to_rear_axle_m: float,
    long_accel_mps2: float,
) -> AxleLoads:
    """Redistribute normal load between axles under longitudinal acceleration.

    Sign convention (claude-docs/06-vehicle-params.md: x forward positive): a positive
    (forward, accelerating) `long_accel_mps2` shifts load AFT (front decreases, rear
    increases) -- this is the familiar "weight transfers to the rear under acceleration, to
    the front under braking" effect. Always conserves total normal load: front + rear ==
    mass_kg * GRAVITY_MPS2 regardless of long_accel_mps2 (see
    tests/test_load_transfer.py::test_longitudinal_conserves_total_load and the L2 property
    test using arbitrary accelerations).

    Loads are clipped at zero and the total is always re-conserved onto the other axle: a
    large enough deceleration/acceleration can in principle demand negative load at an axle
    (a wheelie/stoppie), which means the load-transfer model is no longer valid -- this is
    exactly the saturated/sliding regime that claude-docs/00-project-overview.md says is not
    modeled and is bounded out by the policy envelope, not extrapolated here. "All the static
    weight moves onto the axle still touching the ground" is the correct saturation limit
    (never exceeding or losing total normal load), unlike clipping each axle independently.
    """
    wheelbase_m = cg_to_front_axle_m + cg_to_rear_axle_m
    static = static_axle_loads(mass_kg, cg_to_front_axle_m, cg_to_rear_axle_m)
    total_n = static.front_n + static.rear_n
    delta_n = mass_kg * long_accel_mps2 * cg_height_m / wheelbase_m
    front_n = static.front_n - delta_n
    rear_n = static.rear_n + delta_n
    if front_n < 0.0:
        return AxleLoads(front_n=0.0, rear_n=total_n)
    if rear_n < 0.0:
        return AxleLoads(front_n=total_n, rear_n=0.0)
    return AxleLoads(front_n=front_n, rear_n=rear_n)


@dataclasses.dataclass(frozen=True)
class GripDerate:
    front: float
    rear: float


def lateral_grip_derate(
    mass_kg: float,
    cg_height_m: float,
    static_front_n: float,
    static_rear_n: float,
    lat_accel_mps2: float,
    track_width_m: float | None,
) -> GripDerate:
    """Per-axle lateral-grip derate factor from side-to-side (lateral) load transfer.

    Returns a factor in (0, 1] per axle; 1.0 means "no reduction" (either track_width_m is
    unknown -- the documented fallback, see module docstring -- or lat_accel_mps2 is zero).
    Distributes the total lateral transfer between axles in proportion to each axle's share
    of static load, since per-axle roll-stiffness distribution is not in vehicle_params and
    is not needed for a single-track model's other computations.
    """
    if track_width_m is None or track_width_m <= 0.0:
        return GripDerate(front=1.0, rear=1.0)

    static_total_n = static_front_n + static_rear_n
    if static_total_n <= 0.0:
        return GripDerate(front=1.0, rear=1.0)

    transfer_total_n = mass_kg * abs(lat_accel_mps2) * cg_height_m / track_width_m

    def _axle_derate(static_axle_n: float) -> float:
        if static_axle_n <= 0.0:
            return 1.0
        transfer_axle_n = transfer_total_n * (static_axle_n / static_total_n)
        outer_n = max(static_axle_n / 2.0 + transfer_axle_n, 0.0)
        inner_n = max(static_axle_n / 2.0 - transfer_axle_n, 0.0)
        p = TIRE_LOAD_SENSITIVITY_EXPONENT
        combined_capacity = outer_n**p + inner_n**p
        even_split_capacity = 2.0 * (static_axle_n / 2.0) ** p
        if even_split_capacity <= 0.0:
            return 1.0
        # <= 1 always: concavity of x**p (p<1) means splitting load unevenly can only reduce
        # (never increase) combined capacity relative to an even 50/50 split.
        return min(combined_capacity / even_split_capacity, 1.0)

    return GripDerate(front=_axle_derate(static_front_n), rear=_axle_derate(static_rear_n))
