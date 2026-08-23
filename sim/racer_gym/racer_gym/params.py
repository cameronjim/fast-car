"""Loads config/vehicle_params.yaml through the GENERATED binding and builds the dynamics
parameters racer_gym's upgraded single-track model needs, with an explicit, logged fallback
for every field that is still null pending Phase 1-3 hardware/sysid work.

CLAUDE.md invariant 2: "All vehicle parameters live in config/vehicle_params.yaml and are
consumed via generated bindings... Never hand-write a mass, wheelbase, gear ratio, unit
conversion, or sign convention in code." Accordingly this module never hand-writes a
physical constant: it (a) regenerates tools/gen_params.py's Python binding at call time
(never committed -- claude-docs/10-conventions.md) and reads every physical constant from
the resulting `VEHICLE_PARAMS` instance, and (b) for fields that are null, reads the
fallback value out of the f1tenth_gym dependency itself (its `F110Env.default_config()`
params dict, or a class attribute like `RaceCar.steer_buffer_size`) rather than copying a
number.

Fallback policy per null field (see claude-docs/07-sim-and-sysid.md, claude-docs/00-project-
overview.md's regime table -- the saturated regime is never claimed regardless of fallback
state):

  * `tires.pacejka_front` / `tires.pacejka_rear` (null until a Phase 3 sysid fit): a Pacejka
    curve is SYNTHESIZED from fields that ARE already populated in vehicle_params.yaml
    (`tires.surface_friction_coefficient`, `tires.linear_cornering_stiffness_front/rear_n_per_rad`
    -- both currently copied from f1tenth_gym's own `mu`/`C_Sf`/`C_Sr` defaults, see that
    file's header comment) so that the placeholder curve's PEAK matches mu*Fz_nominal and its
    SMALL-SLIP-ANGLE SLOPE matches the existing linear cornering stiffness -- i.e. it agrees
    with stock f1tenth_gym's linear tire model at small slip angles and saturates at the same
    friction limit, which is the best-justified placeholder available before a real fit
    exists. Shape/curvature factors (C=1.3, E=0) are generic Magic-Formula textbook defaults,
    not per-vehicle numbers.

  * `steering.time_constant_s` / `actuation.throttle_time_constant_s` (null until Phase 2/3
    measurement): fall back to tau=0, i.e. NO first-order lag -- identical to stock
    f1tenth_gym's instantaneous (rate-limited-only) actuation. This is the honest "not
    modeled yet" baseline rather than an invented time constant.

  * `actuation.command_to_torque_delay_s` (null until Phase 2/3 measurement): falls back to
    `f1tenth_gym.envs.base_classes.RaceCar.steer_buffer_size` steps (the delay stock
    f1tenth_gym already hard-codes for its own steering-delay buffer), converted through the
    caller's `dt_s`, rather than assuming zero delay.

  * `chassis.track_width_m` (null until Phase 1 chassis measurement; not modeled by stock
    f1tenth_gym at all): lateral load transfer's grip-derate factor is 1.0 (no effect) --
    see racer_gym/dynamics/load_transfer.py.

Every fallback taken is recorded in the returned `DynParamsResult.fallback_flags` dict and
logged at WARNING level (claude-docs/10-conventions.md: no `print()` in library code) so a
run using unfitted placeholders is never silently indistinguishable from a run using real
sysid data.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import logging
import os
import re
import sys
import types
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from .dynamics.tire import PacejkaParams

logger = logging.getLogger(__name__)


class RepoLayoutNotFoundError(RuntimeError):
    """Raised when no anchor (env override, __file__, cwd) leads to a directory containing
    both config/vehicle_params.yaml and tools/gen_params.py.

    claude-docs/06-vehicle-params.md rule 2: "A consumer that cannot validate must refuse to
    start." This is that refusal for the "where even IS the params file" step that has to
    happen before validation can run at all.
    """


def _has_repo_layout(candidate: Path) -> bool:
    return (candidate / "config" / "vehicle_params.yaml").is_file() and (
        candidate / "tools" / "gen_params.py"
    ).is_file()


def _walk_up_for_repo_root(start: Path) -> Path | None:
    for candidate in (start, *start.parents):
        if _has_repo_layout(candidate):
            return candidate
    return None


def discover_repo_root(
    file_hint: Path | None = None,
    cwd_hint: Path | None = None,
    env: Mapping[str, str] | None = None,
) -> Path:
    """Locate the fast-car repo root (a directory containing both
    config/vehicle_params.yaml and tools/gen_params.py), robust to how racer_gym itself got
    installed.

    Bug this replaces (found during roadmap task S.3): the old implementation was
    `Path(__file__).resolve().parents[3]` -- a fixed walk-up from this module's own file.
    That only lands on the repo root when `__file__` still points at racer_gym's real
    source tree (an editable install, or running straight out of the checkout). A
    NON-editable install -- e.g. `uv sync`ing `racer-gym` as a plain (non-`editable`) path
    dependency of a sibling package, which copies racer_gym's source into THAT package's own
    venv site-packages -- puts `__file__` somewhere with no repo tree above it at all, and
    `training/racer_train` worked around this by forcing `editable = true` on its
    `racer-gym` path dependency (see that package's pyproject.toml, now reverted alongside
    this fix).

    This tries, in order, and returns the first that finds BOTH files:

      1. The `RACER_REPO_ROOT` environment variable, if set -- an explicit override for a
         install with no other anchor back to a checkout (e.g. racer_gym installed globally
         and invoked from outside any fast-car checkout).
      2. Walking up from this module's own `__file__` -- the editable-install / run-from-
         source-tree case, identical to the old behavior.
      3. Walking up from the current working directory -- the practical fix for the non-
         editable case this project actually hits: claude-docs/12-testing.md's CI scripts
         (.github/scripts/pytest_gate.sh) always `cd` into a package directory *inside the
         repo checkout* before running `uv sync` + `uv run pytest` for that package, so even
         though a non-editable `racer-gym` dependency's `__file__` lands in some other
         package's site-packages, the process's cwd is still somewhere under the same
         checkout.

    Raises `RepoLayoutNotFoundError` (naming every path tried) if none of the three finds a
    directory with both files -- e.g. racer_gym installed non-editably and invoked from
    completely outside any fast-car checkout, with no override set. This is a hard failure
    by design, not a guess.

    `file_hint`/`cwd_hint`/`env` default to the real `Path(__file__)` / `Path.cwd()` /
    `os.environ`; the parameters exist so this function can be unit-tested against synthetic
    directory layouts without needing an actual editable/non-editable install of anything
    (see tests/test_params_repo_discovery.py).
    """
    file_hint = Path(__file__).resolve() if file_hint is None else Path(file_hint).resolve()
    cwd_hint = Path.cwd() if cwd_hint is None else Path(cwd_hint).resolve()
    env = os.environ if env is None else env

    tried: list[str] = []

    override = env.get("RACER_REPO_ROOT")
    if override:
        candidate = Path(override).resolve()
        tried.append(f"$RACER_REPO_ROOT={candidate}")
        if _has_repo_layout(candidate):
            return candidate

    tried.append(f"walking up from __file__ ({file_hint.parent})")
    found = _walk_up_for_repo_root(file_hint.parent)
    if found is not None:
        return found

    tried.append(f"walking up from cwd ({cwd_hint})")
    found = _walk_up_for_repo_root(cwd_hint)
    if found is not None:
        return found

    raise RepoLayoutNotFoundError(
        "racer_gym.params: could not locate the fast-car repo root (a directory containing "
        "both config/vehicle_params.yaml and tools/gen_params.py). racer_gym appears to be "
        "installed non-editably and invoked from outside any fast-car checkout, with no "
        "anchor pointing back at one. Fix by either running from inside a fast-car checkout, "
        "or setting RACER_REPO_ROOT=/path/to/fast-car. Tried: " + "; ".join(tried)
    )


REPO_ROOT = discover_repo_root()
DEFAULT_PARAMS_PATH = REPO_ROOT / "config" / "vehicle_params.yaml"
DEFAULT_SCHEMA_PATH = REPO_ROOT / "config" / "vehicle_params.schema.json"
GEN_PARAMS_PATH = REPO_ROOT / "tools" / "gen_params.py"


def _load_gen_params_module() -> types.ModuleType:
    """Import tools/gen_params.py by path (tools/ is a script dir, not an installed package
    -- see tools/pyproject.toml -- so this does not rely on tools/ being on sys.path)."""
    spec = importlib.util.spec_from_file_location("racer_gym._gen_params", GEN_PARAMS_PATH)
    if spec is None or spec.loader is None:  # pragma: no cover - defensive
        raise ImportError(f"could not load gen_params.py from {GEN_PARAMS_PATH}")
    module = importlib.util.module_from_spec(spec)
    # dataclasses.dataclass() (used by gen_params.FieldShape/RecordType) looks the defining
    # module up in sys.modules by name; it must be registered before exec_module runs.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def generate_vehicle_params_module(
    params_path: Path = DEFAULT_PARAMS_PATH, schema_path: Path = DEFAULT_SCHEMA_PATH
) -> types.ModuleType:
    """Regenerate the Python vehicle_params binding at call time (never committed --
    claude-docs/10-conventions.md) and return it as an executed module exposing
    `VEHICLE_PARAMS` (see tools/gen_params.py:generate_python)."""
    gen_params = _load_gen_params_module()
    params = gen_params.load_yaml_file(params_path)
    schema = gen_params.load_schema_file(schema_path)
    gen_params.validate_params(params, schema)  # refuses (raises) on any mismatch, rule 06-2
    source = gen_params.generate_python(params, schema)

    module_name = "racer_gym._vehicle_params_generated"
    module = types.ModuleType(module_name)
    module.__file__ = "<generated by tools/gen_params.py, not committed>"
    # Must be registered before exec: the generated module defines its own
    # @dataclass(frozen=True) classes, and dataclasses.dataclass() looks the defining module
    # up in sys.modules by name while processing them.
    sys.modules[module_name] = module
    # exec is the intended mechanism here, not a shortcut: `source` is trusted output freshly
    # produced by tools/gen_params.py:generate_python from the schema-validated params file
    # above (never external/untrusted input), and executing it in-memory is exactly how the
    # generated-binding-never-committed rule (claude-docs/10-conventions.md) is satisfied
    # without writing a temp file to disk.
    exec(compile(source, module.__file__, "exec"), module.__dict__)  # noqa: S102
    return module


def load_vehicle_params(
    params_path: Path = DEFAULT_PARAMS_PATH, schema_path: Path = DEFAULT_SCHEMA_PATH
) -> Any:
    """Return the generated `VehicleParams` dataclass instance (`VEHICLE_PARAMS`)."""
    module = generate_vehicle_params_module(params_path, schema_path)
    return module.VEHICLE_PARAMS


def _f1tenth_gym_defaults() -> dict:
    """Programmatic reference into the pinned f1tenth_gym dependency's own defaults
    (never hand-copied -- see module docstring)."""
    from f1tenth_gym.envs.f110_env import F110Env

    return F110Env.default_config()["params"]


def _f1tenth_gym_steer_buffer_size() -> int:
    """Programmatic reference to f1tenth_gym's own built-in steering-delay buffer length.

    `RaceCar.steer_buffer_size` is set as a plain instance attribute inside `__init__`
    (`self.steer_buffer_size = 2`), not a class attribute or a constructor default, and
    constructing a real `RaceCar` just to read it would require a full scan-simulator /
    params / action_type setup unrelated to this value. Extracting it straight from the
    dependency's own source at runtime is a closer reading of "reference programmatically,
    do not copy numbers" than hand-copying the literal `2` would be: if the pinned
    f1tenth_gym commit ever changes this value, this fallback moves with it automatically.
    """
    import inspect

    from f1tenth_gym.envs.base_classes import RaceCar

    source = inspect.getsource(RaceCar.__init__)
    match = re.search(r"self\.steer_buffer_size\s*=\s*(\d+)", source)
    if not match:
        raise RuntimeError(
            "could not find 'self.steer_buffer_size = <N>' in f1tenth_gym.envs.base_classes."
            "RaceCar.__init__; the pinned f1tenth_gym commit may have changed this attribute "
            "-- update racer_gym/params.py's fallback for actuation.command_to_torque_delay_s"
        )
    return int(match.group(1))


@dataclasses.dataclass(frozen=True)
class DynParams:
    """Everything racer_gym's dynamics (racer_gym/dynamics/model.py) needs, already resolved
    (no nulls, no fallback logic left to do at simulation time)."""

    mass_kg: float
    cg_height_m: float
    cg_to_front_axle_m: float
    cg_to_rear_axle_m: float
    yaw_inertia_kg_m2: float
    track_width_m: float | None

    pacejka_front: PacejkaParams
    pacejka_rear: PacejkaParams

    steer_tau_s: float
    throttle_tau_s: float
    delay_steps: int

    # Passed straight through to f1tenth_gym's own steering_constraint/accl_constraints
    # (reused, not reimplemented -- see racer_gym/dynamics/model.py).
    s_min: float
    s_max: float
    sv_min: float
    sv_max: float
    v_switch: float
    a_max: float
    v_min: float
    v_max: float


@dataclasses.dataclass(frozen=True)
class DynParamsResult:
    dyn_params: DynParams
    fallback_flags: dict[str, bool]

    @property
    def used_any_placeholder(self) -> bool:
        return any(self.fallback_flags.values())


def _synthesize_pacejka(
    mu: float, linear_cornering_stiffness_n_per_rad: float, fz_nominal_n: float
) -> PacejkaParams:
    """Build a placeholder Pacejka curve calibrated to the existing (gym-sourced) mu and
    linear cornering stiffness -- see module docstring's fallback policy."""
    c_shape = 1.3  # generic Magic-Formula shape-factor default (textbook value), not fitted
    e_curvature = 0.0  # no curvature correction until a real fit supplies one
    d_peak_n = mu * fz_nominal_n
    # Small-slip-angle cornering stiffness of the Magic Formula is B*C*D (standard identity
    # for E=0); solve for B so the placeholder curve's initial slope matches the existing
    # linear cornering stiffness exactly.
    b_stiffness = linear_cornering_stiffness_n_per_rad / (c_shape * d_peak_n)
    return PacejkaParams(
        b_stiffness=b_stiffness, c_shape=c_shape, d_peak_n=d_peak_n, e_curvature=e_curvature
    )


def build_dyn_params(vehicle_params: Any, dt_s: float) -> DynParamsResult:
    """Resolve a generated `VEHICLE_PARAMS` instance (see `load_vehicle_params`) plus the
    simulation step `dt_s` into a fully-populated `DynParams`, applying and flagging every
    documented null fallback."""
    if dt_s <= 0.0:
        raise ValueError("dt_s must be > 0")

    chassis = vehicle_params.chassis
    tires = vehicle_params.tires
    steering = vehicle_params.steering
    actuation = vehicle_params.actuation

    fallback_flags: dict[str, bool] = {
        "pacejka_front": tires.pacejka_front is None,
        "pacejka_rear": tires.pacejka_rear is None,
        "steer_time_constant": steering.time_constant_s is None,
        "throttle_time_constant": actuation.throttle_time_constant_s is None,
        "command_to_torque_delay": actuation.command_to_torque_delay_s is None,
        "track_width": chassis.track_width_m is None,
    }
    for name, used in fallback_flags.items():
        if used:
            logger.warning(
                "racer_gym: vehicle_params field for %r is null; using the documented "
                "f1tenth_gym-derived fallback, not a fitted value (see racer_gym/params.py).",
                name,
            )

    from .dynamics.load_transfer import static_axle_loads

    static = static_axle_loads(
        chassis.mass_kg, chassis.cg_to_front_axle_m, chassis.cg_to_rear_axle_m
    )

    if tires.pacejka_front is not None:
        pacejka_front = PacejkaParams(**dataclasses.asdict(tires.pacejka_front))
    else:
        gym_defaults = _f1tenth_gym_defaults()
        pacejka_front = _synthesize_pacejka(
            tires.surface_friction_coefficient, gym_defaults["C_Sf"], static.front_n
        )

    if tires.pacejka_rear is not None:
        pacejka_rear = PacejkaParams(**dataclasses.asdict(tires.pacejka_rear))
    else:
        gym_defaults = _f1tenth_gym_defaults()
        pacejka_rear = _synthesize_pacejka(
            tires.surface_friction_coefficient, gym_defaults["C_Sr"], static.rear_n
        )

    steer_tau_s = 0.0 if steering.time_constant_s is None else steering.time_constant_s
    throttle_tau_s = (
        0.0 if actuation.throttle_time_constant_s is None else actuation.throttle_time_constant_s
    )

    if actuation.command_to_torque_delay_s is None:
        delay_steps = _f1tenth_gym_steer_buffer_size()
    else:
        delay_steps = round(actuation.command_to_torque_delay_s / dt_s)

    dyn_params = DynParams(
        mass_kg=chassis.mass_kg,
        cg_height_m=chassis.cg_height_m,
        cg_to_front_axle_m=chassis.cg_to_front_axle_m,
        cg_to_rear_axle_m=chassis.cg_to_rear_axle_m,
        yaw_inertia_kg_m2=chassis.yaw_inertia_kg_m2,
        track_width_m=chassis.track_width_m,
        pacejka_front=pacejka_front,
        pacejka_rear=pacejka_rear,
        steer_tau_s=steer_tau_s,
        throttle_tau_s=throttle_tau_s,
        delay_steps=delay_steps,
        s_min=steering.min_angle_rad,
        s_max=steering.max_angle_rad,
        sv_min=steering.min_rate_rad_per_s,
        sv_max=steering.max_rate_rad_per_s,
        v_switch=tires.kinematic_dynamic_switch_velocity_mps,
        a_max=actuation.max_acceleration_mps2,
        v_min=vehicle_params.limits.min_velocity_mps,
        v_max=vehicle_params.limits.global_speed_cap_mps,
    )
    return DynParamsResult(dyn_params=dyn_params, fallback_flags=fallback_flags)
