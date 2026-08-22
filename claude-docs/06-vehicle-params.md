# 06 — Vehicle Parameters: Schema, Units, Sign Conventions

`config/vehicle_params.yaml` is consumed by Python, C++, firmware, and sim. Four consumers
WILL silently disagree about units and signs unless prevented structurally. So:

## Rules

1. Every physical constant lives here and ONLY here. Grep-able rule: a numeric literal with
   physical meaning in code is a review-blocking defect.
2. The file validates against `config/vehicle_params.schema.json` at load, in every language.
   A consumer that cannot validate must refuse to start.
3. Bindings are GENERATED (Python module, C++ header, C struct for firmware) by
   `tools/gen_params.py` in CI. Never hand-write a binding.
4. The schema carries units and sign conventions per field, machine-readable.
5. Any change bumps `schema_version`; consumers refuse on major mismatch.

## Conventions (fixed, project-wide)

- SI units: m, s, kg, rad, N, A, V. No degrees, no RPM, no km/h anywhere past a driver
  boundary.
- Frames per REP-103: x forward, y left, z up. Yaw counter-clockwise positive.
- Steering angle: road-wheel angle in radians, LEFT positive. (Servo PWM ↔ wheel angle
  mapping lives here as a calibrated table, measured in Phase 2, not assumed linear.)
- Slip angle: positive when velocity vector points left of heading.
- Motor current: positive = drive torque forward.
- ERPM ↔ wheel speed conversions (pole pairs, gear ratio, wheel radius) live here; wheel
  radius is a MEASURED, drift-tracked value (tires wear — see sysid drift record).

## Contents (sections)

- `chassis`: mass, wheelbase, track width, CG height/position, yaw inertia
- `tires`: Pacejka params front/rear (from sysid), nominal radius, compound id, surface id
- `drivetrain`: gear ratio, pole pairs, motor Kv, current limits (mirror of VESC layer-2 config)
- `steering`: rack limits, servo rate limit, first-order time constant (measured), pwm↔angle table
- `actuation`: command-to-torque transport delay (measured), throttle first-order time constant
- `sensors`: mounting extrinsics (IMU at CG, LiDAR pose), per-sensor timestamp offsets from
  the sync design doc
- `limits`: global speed cap, TTC thresholds, envelope fractions — the values layers 3–4 enforce
- `meta`: schema_version, sysid session id that produced the fitted values, fit date

## Sysid coupling

Fitted parameters are written back by `sysid/fitting/` with the producing session id in
`meta`. Sim, training configs, and the deployment contract all record which
`vehicle_params` version they used — a policy trained against one parameter set and deployed
against another is a refuse-on-mismatch case (see `08-learning.md`).
