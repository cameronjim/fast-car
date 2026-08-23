# racer_gym

Roadmap task S.1 (`claude-docs/01-roadmap.md`, `claude-docs/07-sim-and-sysid.md`): model
upgrades over stock `f1tenth_gym` -- longitudinal + lateral load transfer, front/rear
Pacejka tire curves, first-order steering/throttle actuator dynamics, and a
command-to-torque transport delay.

## Extension, not a vendor/fork

This package depends on the pinned `f1tenth_gym` commit (same SHA as `docker/sim-cpu`, see
`pyproject.toml`) and does not copy or modify any of its source files. `F110Env` only
accepts `model` / `integrator` / `action_type` as fixed strings resolved through closed
Enums (`DynamicModel.from_string`, `IntegratorType.from_string`), so there is no *config*-
level way to inject a custom dynamics object -- but `RaceCar` and `Simulator` re-read
`self.model` / `self.integrator` / `self.action_type` fresh on every `update_pose()` call
rather than caching them, and `RaceCar.reset()` never touches those three attributes.
`racer_gym/env.py::build_env` builds a completely normal `gym.make(...)` env and then
replaces each agent's `model`/`integrator`/`action_type` with racer_gym's own
(`racer_gym/dynamics/model.py`), which is enough to run the upgraded single-track dynamics
end to end without touching upstream code. Everything else -- observation types, collision
detection, LiDAR scan simulation, lap counting, rendering -- is unmodified upstream
f1tenth_gym.

Null vehicle_params fields (Pacejka fits, actuator time constants, transport delay, track
width -- none of Phase 1-3 sysid has happened yet) fall back to values read programmatically
from the f1tenth_gym dependency itself (never hand-copied numeric literals) and are flagged
in `env.racer_fallback_flags`; see `racer_gym/params.py`'s module docstring for the exact
policy per field.

## Layout

- `racer_gym/dynamics/load_transfer.py` -- longitudinal + lateral load transfer
- `racer_gym/dynamics/tire.py` -- Pacejka Magic Formula, front/rear
- `racer_gym/dynamics/actuator.py` -- first-order actuator lag
- `racer_gym/dynamics/delay.py` -- fixed transport delay (FIFO)
- `racer_gym/dynamics/model.py` -- combines the above into the duck-typed
  model/integrator/action_type objects f1tenth_gym's `RaceCar` accepts
- `racer_gym/params.py` -- loads `config/vehicle_params.yaml` via the generated binding,
  resolves fallbacks
- `racer_gym/env.py` -- `build_env()`: constructs and patches the gym env

## Running the tests

```
uv sync --all-extras --dev
uv run pytest --cov=. --cov-report=term-missing
```

Covers L1 (hand-computed cases: load-transfer signs/magnitudes, Pacejka peak/symmetry/
front-rear separation, actuator step response, delay timing, sign conventions per
`claude-docs/06-vehicle-params.md`), L2 (hypothesis property tests per
`claude-docs/12-testing.md`), and a determinism test (same seed + commands -> identical
trajectory). The committed-reference sim-dynamics regression battery (comparing against the
real sysid maneuvers) is roadmap task S.6, not this one.
