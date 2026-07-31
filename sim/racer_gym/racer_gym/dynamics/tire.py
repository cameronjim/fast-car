"""Pacejka "Magic Formula" lateral tire model, separate front/rear parameters.

Roadmap S.1 requirement 2 (claude-docs/07-sim-and-sysid.md). Implements the standard
simplified Magic Formula:

    F_y(alpha) = -D * sin(C * atan(B*alpha - E*(B*alpha - atan(B*alpha))))

`B, C, D, E` are `config/vehicle_params.yaml`'s `tires.pacejka_front` /
`tires.pacejka_rear` (`b_stiffness, c_shape, d_peak_n, e_curvature`) when a Phase 3 sysid fit
exists; see racer_gym/params.py for the fallback used while they are null.

Sign convention (claude-docs/06-vehicle-params.md: slip angle positive when the velocity
vector points left of heading, applied per-axle in racer_gym/dynamics/model.py): `alpha`
here is defined so that alpha > 0 means the wheel's velocity points to the LEFT of the
wheel's own heading. A tire resists that slip, so the returned force is the restoring
(rightward, negative) lateral force -- hence the leading minus sign. This is why `D`
(`d_peak_n`, always a positive "peak force" magnitude in the schema) produces a
NEGATIVE peak F_y for positive alpha: see tests/test_tire_pacejka.py::test_sign_symmetry
and ::test_peak_is_negative_for_positive_alpha.

`d_peak_n` is fit (or, while null, placeholder-derived -- see params.py) at a reference
("nominal") normal load, generally the static axle load from a constant-radius sweep. Away
from that reference load -- e.g. under the load transfer computed by load_transfer.py -- D is
scaled linearly by Fz / Fz_nominal, per the standard Magic-Formula load-scaling convention.
Any further sub-linear ("load-sensitivity") correction from *lateral* weight transfer is
already folded into the effective Fz by load_transfer.lateral_grip_derate before it reaches
this function, so the scaling here is deliberately linear (see load_transfer.py's docstring
for why applying the same nonlinearity twice would double-count it).
"""

from __future__ import annotations

import dataclasses
import math


@dataclasses.dataclass(frozen=True)
class PacejkaParams:
    b_stiffness: float
    c_shape: float
    d_peak_n: float
    e_curvature: float


def lateral_force(
    alpha_rad: float,
    fz_n: float,
    fz_nominal_n: float,
    params: PacejkaParams,
) -> float:
    """Lateral tire force at slip angle `alpha_rad` and normal load `fz_n`.

    `fz_nominal_n` is the normal load `params.d_peak_n` was referenced to (see module
    docstring). `fz_n` may be zero or negative only in degenerate/edge-case inputs (e.g. an
    axle fully unloaded by longitudinal transfer); force is zero in that case, not a sign
    flip or NaN.
    """
    if fz_nominal_n <= 0.0:
        raise ValueError("fz_nominal_n must be positive")
    if fz_n <= 0.0:
        return 0.0

    b, c, d0, e = (
        params.b_stiffness,
        params.c_shape,
        params.d_peak_n,
        params.e_curvature,
    )
    d = d0 * (fz_n / fz_nominal_n)
    b_alpha = b * alpha_rad
    arg = c * math.atan(b_alpha - e * (b_alpha - math.atan(b_alpha)))
    return -d * math.sin(arg)


def peak_slip_angle_rad(params: PacejkaParams) -> float:
    """Slip angle magnitude at which |F_y| is maximized, for E == 0 (no curvature term).

    Closed form used only by tests (tests/test_tire_pacejka.py::test_peak_location): with
    E=0, F_y = -D*sin(C*atan(B*alpha)), whose derivative w.r.t. alpha vanishes where
    C*atan(B*alpha) == pi/2, i.e. alpha == tan(pi / (2*C)) / B.
    """
    if params.e_curvature != 0.0:
        raise ValueError("closed-form peak location requires e_curvature == 0")
    return math.tan(math.pi / (2.0 * params.c_shape)) / params.b_stiffness
