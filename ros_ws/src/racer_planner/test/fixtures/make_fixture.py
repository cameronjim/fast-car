#!/usr/bin/env python3
"""Writes a dynamics bundle for racer_planner's tests, with its OWN implementation of the model.

This is a second implementation on purpose. It is written from the model step in the interface
contract (claude-docs/13 section 4.4), not from the C++ code, and shares nothing with it: the
C++ planner has to reproduce the golden rollouts this script writes, within 1e-4, or it refuses
the bundle. Standalone, numpy only (vehicle_params.yaml is read by a tiny indentation parser so
no YAML library is needed).

    make_fixture.py --out DIR [--vehicle-params PATH] [--residual on|off] [--v-max 0.9]
                    [--sidecar] [--seed 1]

--sidecar also writes vehicle_geometry.yaml (the vehicle numbers the golden rollouts were made
with), so the committed fixture can be tested without depending on the live vehicle_params.
"""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[5]
STATE_ORDER = ["x_m", "y_m", "yaw_rad", "v_mps", "delta_rad"]
LAYERS = [(4, 32), (32, 32), (32, 2)]


def layers_of(hidden: int = 32) -> list[tuple[int, int]]:
    """The 4-H-H-2 residual (dyn-v0.8 on may be narrower than 32)."""
    return [(4, hidden), (hidden, hidden), (hidden, 2)]


def hidden_of(n_weights: int) -> int:
    """H of a 4-H-H-2 net from its weight count H^2 + 8H + 2."""
    h = round(math.sqrt(n_weights + 14) - 4)
    if h * h + 8 * h + 2 != n_weights:
        raise ValueError(f"{n_weights} weights are not a 4-H-H-2 net")
    return h


NOMINAL = {
    "speed_gain": 1.0,
    "speed_deadband_mps": 0.0,
    "speed_tau_s": 0.25,
    "accel_max_mps2": 2.0,
    "decel_max_mps2": 1.5,
    "steer_gain_left": 1.0,
    "steer_gain_right": 1.0,
    "steer_offset_rad": 0.0,
    "steer_tau_s": 0.10,
    "command_delay_s": 0.08,
    "understeer_s2_per_m2": 0.0,
}
# The committed fixture is deliberately not symmetric, so a swapped sign or gain shows up.
NOMINAL_ASYMMETRIC = dict(
    NOMINAL,
    speed_gain=0.95,
    steer_gain_left=1.1,
    steer_gain_right=0.9,
    steer_offset_rad=0.01,
    understeer_s2_per_m2=0.05,
)


def read_vehicle_params(path: Path) -> dict[str, str]:
    """Flat 'section.key' -> raw value for the keys of vehicle_params.yaml. Indentation parser."""
    out: dict[str, str] = {}
    stack: list[tuple[int, str]] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip() or line.lstrip().startswith("-"):
            continue
        indent = len(line) - len(line.lstrip())
        key, _, value = line.strip().partition(":")
        while stack and stack[-1][0] >= indent:
            stack.pop()
        value = value.strip()
        if value == "":
            stack.append((indent, key))
        else:
            out[".".join([k for _, k in stack] + [key])] = value.strip('"')
    return out


def vehicle_numbers(vp: dict[str, str]) -> dict[str, object]:
    f = lambda k: float(vp[k])
    return {
        "wheelbase_m": f("chassis.wheelbase_m"),
        "width_m": f("chassis.width_m"),
        "rear_overhang_m": f("chassis.rear_overhang_m"),
        "front_overhang_m": f("chassis.front_overhang_m"),
        "steering_min_rad": f("steering.min_angle_rad"),
        "steering_max_rad": f("steering.max_angle_rad"),
        "steering_max_rate_rad_per_s": f("steering.max_rate_rad_per_s"),
        "max_acceleration_mps2": f("actuation.max_acceleration_mps2"),
        "stop_deceleration_mps2": f("actuation.stop_deceleration_mps2"),
        "lidar_mount_x_m": f("sensors.lidar.mount_x_m"),
        "lidar_mount_y_m": f("sensors.lidar.mount_y_m"),
        "lidar_mount_yaw_rad": f("sensors.lidar.mount_yaw_rad"),
        "schema_version": vp["meta.schema_version"],
        "sysid_session_id": vp["meta.sysid_session_id"],
    }


# ---- the model, written from the contract -------------------------------------------------


def mlp(weights: np.ndarray, x: np.ndarray, layers=None) -> np.ndarray:
    layers = LAYERS if layers is None else layers
    o = 0
    h = x
    for i, (n_in, n_out) in enumerate(layers):
        w = weights[o : o + n_out * n_in].reshape(n_out, n_in)
        o += n_out * n_in
        b = weights[o : o + n_out]
        o += n_out
        h = w @ h + b
        if i < len(layers) - 1:
            h = np.tanh(h)
    return h


def step(state, control, nom, veh, dt, residual=None, ground_scale=1.0):
    x, y, yaw, v, delta = state
    speed_cmd, steering_cmd = control
    g = nom["steer_gain_left"] if steering_cmd >= 0 else nom["steer_gain_right"]
    delta_target = min(
        max(g * steering_cmd + nom["steer_offset_rad"], veh["steering_min_rad"]),
        veh["steering_max_rad"],
    )
    a_d = 1.0 - math.exp(-dt / nom["steer_tau_s"])
    rate = veh["steering_max_rate_rad_per_s"] * dt
    delta_next = delta + min(max(a_d * (delta_target - delta), -rate), rate)
    a_v = 1.0 - math.exp(-dt / nom["speed_tau_s"])
    target = nom["speed_gain"] * speed_cmd if abs(speed_cmd) >= nom["speed_deadband_mps"] else 0.0
    dv = min(
        max(a_v * (target - v), -nom["decel_max_mps2"] * dt),
        nom["accel_max_mps2"] * dt,
    )
    yaw_rate = (
        v * math.tan(delta) / veh["wheelbase_m"] / (1.0 + nom["understeer_s2_per_m2"] * v * v)
    )
    if residual is not None:
        weights, mean, std, scale = residual
        x_in = (np.array([v, delta, speed_cmd, steering_cmd]) - mean) / std
        r = scale * mlp(weights, x_in, layers_of(hidden_of(len(weights))))
        dv += float(r[0]) * dt
        yaw_rate += float(r[1])
    vg = v * ground_scale if residual is not None else v  # bundle 1.2 ground scale
    return (
        x + vg * math.cos(yaw) * dt,
        y + vg * math.sin(yaw) * dt,
        yaw + yaw_rate * dt,
        max(0.0, v + dv),
        delta_next,
    )


def rollout_controls(veh: dict, n: int = 40):
    """Nine control programs: left, right, accelerating, braking, saturated both ways, S bend."""
    lim = float(veh["steering_max_rad"])
    t = np.arange(n)
    progs = [
        ([0.5] * n, [0.269] * n, (0.0, 0.0, 0.0, 0.5, 0.0)),  # left, cruising
        ([0.5] * n, [-0.269] * n, (0.0, 0.0, 0.0, 0.5, 0.0)),  # right, cruising
        ([0.9] * n, [0.0] * n, (0.0, 0.0, 0.0, 0.0, 0.0)),  # accelerating from rest
        ([0.0] * n, [0.0] * n, (0.0, 0.0, 0.0, 0.8, 0.0)),  # braking
        ([0.6] * n, [3.0 * lim] * n, (0.0, 0.0, 0.0, 0.6, 0.0)),  # saturated left
        ([0.6] * n, [-3.0 * lim] * n, (0.0, 0.0, 0.0, 0.6, 0.0)),  # saturated right
        (
            list(0.6 + 0.3 * np.sin(t / 6.0)),
            list(0.8 * lim * np.sin(t / 5.0)),
            (0.0, 0.0, 0.1, 0.4, 0.0),
        ),  # S bend with speed changes
        ([2.0] * n, [0.0] * n, (0.0, 0.0, 0.0, 0.0, 0.0)),  # speed command far above the cap
        (
            [0.7] * 15 + [0.0] * 10 + [0.7] * 15,
            [0.3] * 10 + [-0.3] * 15 + [0.1] * 15,
            (1.0, -0.5, -0.4, 0.7, 0.05),
        ),  # start, brake and go again from an offset pose
    ]
    return progs


def make_bundle(
    out: Path,
    vehicle: dict,
    *,
    residual_on: bool,
    nominal: dict,
    v_max: float,
    seed: int = 1,
    dt: float = 0.05,
    model_id: str = "dyn-fixture-v0-2026-11-12",
    hidden: int = 32,
    ground_scale: float = 1.0,
) -> None:
    out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    n_weights = sum(o * i + o for i, o in layers_of(hidden))
    if residual_on:
        weights32 = (rng.standard_normal(n_weights) * 0.15).astype("<f4")
        mean = np.array([0.3, 0.0, 0.3, 0.0])
        std = np.array([0.3, 0.2, 0.3, 0.2])
        scale = np.array([0.2, 0.03])
    else:
        weights32 = np.zeros(n_weights, dtype="<f4")
        mean = np.zeros(4)
        std = np.ones(4)
        scale = np.ones(2)
    (out / "residual_mlp.f32").write_bytes(weights32.tobytes())
    residual = (weights32.astype(np.float64), mean, std, scale) if residual_on else None

    rows = ["rollout,step,speed_cmd_mps,steering_cmd_rad,x_m,y_m,yaw_rad,v_mps,delta_rad"]
    for r, (speeds, steers, start) in enumerate(rollout_controls(vehicle)):
        state = tuple(float(f"{c:.9g}") for c in start)
        n = len(speeds)
        for k in range(n + 1):
            if k < n:
                u = (float(f"{speeds[k]:.9g}"), float(f"{steers[k]:.9g}"))
            else:
                u = (0.0, 0.0)  # the last row's controls are not applied
            rows.append(f"{r},{k}," + ",".join(f"{c:.9g}" for c in (*u, *state)))
            if k < n:
                state = step(state, u, nominal, vehicle, dt, residual, ground_scale)
    (out / "golden_rollouts.csv").write_text("\n".join(rows) + "\n", encoding="utf-8")

    def sha(name: str) -> str:
        return hashlib.sha256((out / name).read_bytes()).hexdigest()

    nom_lines = "\n".join(f"    {k}: {v}" for k, v in nominal.items())
    version = "1.1.0" if ground_scale == 1.0 else "1.2.0"
    gs_line = "" if ground_scale == 1.0 else f"\n    ground_scale: {ground_scale!r}"
    manifest = f"""bundle_schema_version: "{version}"
kind: dynamics
model_id: "{model_id}"
created_utc: "2026-11-12T00:00:00Z"
git_sha: "0000000000000000000000000000000000000000"
training_config_sha256: "{"0" * 64}"
vehicle_params:
  schema_version: "{vehicle["schema_version"]}"
  sysid_session_id: "{vehicle["sysid_session_id"]}"
payload:
  - {{file: "residual_mlp.f32", sha256: "{sha("residual_mlp.f32")}"}}
  - {{file: "golden_rollouts.csv", sha256: "{sha("golden_rollouts.csv")}"}}
io:
  state: [x_m, y_m, yaw_rad, v_mps, delta_rad]
  control: [speed_cmd_mps, steering_cmd_rad]
  dt_s: {dt}
  nominal:
{nom_lines}
  residual:
    enabled: {"true" if residual_on else "false"}
    inputs: [v_mps, delta_rad, speed_cmd_mps, steering_cmd_rad]
    outputs: [dv_dt_mps2, dyaw_dt_rad_per_s]
    layers: [[4, {hidden}], [{hidden}, {hidden}], [{hidden}, 2]]
    activation: tanh
    input_mean: [{", ".join(repr(float(m)) for m in mean)}]
    input_std: [{", ".join(repr(float(s)) for s in std)}]
    output_scale: [{", ".join(repr(float(s)) for s in scale)}]{gs_line}
    file: residual_mlp.f32
  validity: {{v_min_mps: 0.0, v_max_mps: {v_max}}}
  golden: golden_rollouts.csv
data: ["racer_planner test fixture, synthetic, seed {seed}, not fitted"]
metrics: {{}}
"""
    (out / "manifest.yaml").write_text(manifest, encoding="utf-8")


# ---- step contract 2 (addendum 1 section 2.2), written from the contract, not from the C++ ------

GROUND = {
    "alpha_min": 0.60,
    "alpha_max": 1.05,
    "d_alpha_max": 0.10,
    "rho_max": 0.25,
    "steer_nodes": [-1.0, -0.6667, -0.3333, 0.0, 0.3333, 0.6667, 1.0],
    "speed_nodes": [0.25, 0.6, 0.85, 1.25, 1.5],
}
INTERPOLATION = (
    "bilinear over steering as a share of each side's limit and wheel speed, "
    "held constant outside the grid"
)


def _axis(nodes, x):
    nodes = np.asarray(nodes, dtype=float)
    xc = min(max(x, nodes[0]), nodes[-1])
    hi = int(np.searchsorted(nodes, xc, side="right"))
    hi = min(max(hi, 1), len(nodes) - 1)
    lo = hi - 1
    return lo, hi, (xc - nodes[lo]) / (nodes[hi] - nodes[lo])


def table_alpha(values, delta, v, veh):
    share = delta / veh["steering_max_rad"] if delta >= 0 else delta / -veh["steering_min_rad"]
    i0, i1, wi = _axis(GROUND["steer_nodes"], share)
    j0, j1, wj = _axis(GROUND["speed_nodes"], v)
    t = values
    return (
        (1 - wi) * (1 - wj) * t[i0][j0]
        + (1 - wi) * wj * t[i0][j1]
        + wi * (1 - wj) * t[i1][j0]
        + wi * wj * t[i1][j1]
    )


def step_v2(state, control, nom, veh, dt, ground):
    """ground = (values 7x5, learned) with learned None or (weights, mean, std)."""
    x, y, yaw, v, delta = state
    wheel = step(state, control, nom, veh, dt, None)  # the wheel channel of contract 1
    values, learned = ground
    lo, hi = GROUND["alpha_min"], GROUND["alpha_max"]
    alpha = min(max(table_alpha(values, delta, v, veh), lo), hi)
    rho = 0.0
    if learned is not None:
        weights, mean, std = learned
        m = mlp(weights, (np.array([v, delta, control[0], control[1]]) - mean) / std)
        alpha = min(max(alpha + GROUND["d_alpha_max"] * math.tanh(m[0]), lo), hi)
        rho = GROUND["rho_max"] * math.tanh(m[1])
    v_g = alpha * v
    yaw_rate = (
        (1.0 + rho)
        * v
        * math.tan(delta)
        / veh["wheelbase_m"]
        / (1.0 + nom["understeer_s2_per_m2"] * v * v)
    )
    return (
        x + v_g * math.cos(yaw) * dt,
        y + v_g * math.sin(yaw) * dt,
        yaw + yaw_rate * dt,
        wheel[3],
        wheel[4],
    )


def synthetic_table():
    """Distinct values everywhere, one node on each bound (a value on a bound is inside)."""
    rows = []
    for i in range(7):
        rows.append(
            [
                round(0.92 - 0.12 * abs(i - 3) / 3 + 0.04 * math.sin(2.0 * i + j), 6)
                for j in range(5)
            ]
        )
    rows[0][0] = GROUND["alpha_min"]
    rows[6][4] = GROUND["alpha_max"]
    return rows


def make_bundle_v2(
    out: Path,
    vehicle: dict,
    *,
    learned_on: bool,
    nominal: dict,
    v_max: float,
    seed: int = 1,
    dt: float = 0.05,
    model_id: str = "dyn-fixture-v2-2026-11-15",
) -> tuple:
    out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    n_weights = sum(o * i + o for i, o in LAYERS)
    if learned_on:
        weights32 = (rng.standard_normal(n_weights) * 0.4).astype("<f4")
        mean = np.array([0.5, 0.1, 0.5, 0.1])
        std = np.array([0.3, 0.3, 0.3, 0.3])
    else:
        weights32 = np.zeros(n_weights, dtype="<f4")
        mean = np.zeros(4)
        std = np.ones(4)
    (out / "ground_mlp.f32").write_bytes(weights32.tobytes())
    values = synthetic_table()
    ground = (values, (weights32.astype(np.float64), mean, std) if learned_on else None)

    rows = ["rollout,step,speed_cmd_mps,steering_cmd_rad,x_m,y_m,yaw_rad,v_mps,delta_rad"]
    for r, (speeds, steers, start) in enumerate(rollout_controls(vehicle)):
        state = tuple(float(f"{c:.9g}") for c in start)
        n = len(speeds)
        for k in range(n + 1):
            u = (float(f"{speeds[k]:.9g}"), float(f"{steers[k]:.9g}")) if k < n else (0.0, 0.0)
            rows.append(f"{r},{k}," + ",".join(f"{c:.9g}" for c in (*u, *state)))
            if k < n:
                state = step_v2(state, u, nominal, vehicle, dt, ground)
    (out / "golden_rollouts.csv").write_text("\n".join(rows) + "\n", encoding="utf-8")

    def sha(name: str) -> str:
        return hashlib.sha256((out / name).read_bytes()).hexdigest()

    nom_lines = "\n".join(f"    {k}: {v}" for k, v in nominal.items())
    table_lines = "\n".join(f"        - [{', '.join(repr(v) for v in r)}]" for r in values)
    manifest = f"""bundle_schema_version: "2.0.0"
kind: dynamics
model_id: "{model_id}"
created_utc: "2026-11-15T00:00:00Z"
git_sha: "0000000000000000000000000000000000000000"
training_config_sha256: "{"0" * 64}"
vehicle_params:
  schema_version: "{vehicle["schema_version"]}"
  sysid_session_id: "{vehicle["sysid_session_id"]}"
payload:
  - {{file: "ground_mlp.f32", sha256: "{sha("ground_mlp.f32")}"}}
  - {{file: "golden_rollouts.csv", sha256: "{sha("golden_rollouts.csv")}"}}
io:
  state: [x_m, y_m, yaw_rad, v_mps, delta_rad]
  control: [speed_cmd_mps, steering_cmd_rad]
  dt_s: {dt}
  nominal:
{nom_lines}
  ground_motion:
    alpha_bounds: [0.6, 1.05]
    table:
      steering_nodes_share_of_side_limit: {GROUND["steer_nodes"]}
      wheel_speed_nodes_mps: {GROUND["speed_nodes"]}
      interpolation: "{INTERPOLATION}"
      values:
{table_lines}
    learned:
      enabled: {"true" if learned_on else "false"}
      inputs: [v_mps, delta_rad, speed_cmd_mps, steering_cmd_rad]
      outputs: [d_alpha, rho]
      layers: [[4, 32], [32, 32], [32, 2]]
      activation: tanh
      input_mean: [{", ".join(repr(float(m)) for m in mean)}]
      input_std: [{", ".join(repr(float(s)) for s in std)}]
      d_alpha_max: 0.1
      rho_max: 0.25
      file: ground_mlp.f32
  validity: {{v_min_mps: 0.0, v_max_mps: {v_max}}}
  golden: golden_rollouts.csv
data: ["racer_planner contract 2 test fixture, synthetic, seed {seed}, not fitted"]
metrics: {{}}
"""
    (out / "manifest.yaml").write_text(manifest, encoding="utf-8")
    return nominal, ground


RANDOM_HEADER = "x,y,yaw,v,delta,speed_cmd,steering_cmd,x1,y1,yaw1,v1,delta1"


def write_random_states(path: Path, vehicle: dict, nominal: dict, ground, n=10000, seed=5, dt=0.05):
    """n random states and controls and the contract 2 step of each, inputs to 9 digits (the
    step is taken from the rounded inputs), outputs to 13."""
    rng = np.random.default_rng(seed)
    lim = float(vehicle["steering_max_rad"])
    rows = [RANDOM_HEADER]
    for _ in range(n):
        u = rng.uniform(
            [-2, -2, -4, -0.3, -1.2 * lim, -0.3, -2.5 * lim],
            [2, 2, 4, 1.45, 1.2 * lim, 1.5, 2.5 * lim],
        )
        u = [float(f"{c:.9g}") for c in u]
        out = step_v2(tuple(u[:5]), (u[5], u[6]), nominal, vehicle, dt, ground)
        rows.append(",".join(f"{c:.9g}" for c in u) + "," + ",".join(f"{c:.13g}" for c in out))
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")


def write_sidecar(out: Path, vehicle: dict) -> None:
    lines = [f"{k}: {v}" for k, v in vehicle.items()]
    (out / "vehicle_geometry.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--vehicle-params", type=Path, default=REPO_ROOT / "config/vehicle_params.yaml")
    ap.add_argument("--residual", choices=["on", "off"], default="on")
    ap.add_argument("--nominal", choices=["asymmetric", "plain"], default="asymmetric")
    ap.add_argument("--v-max", type=float, default=0.9)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--sidecar", action="store_true")
    ap.add_argument("--contract", type=int, choices=[1, 2], default=1)
    ap.add_argument("--hidden", type=int, default=32, help="contract 1: residual width H")
    ap.add_argument("--ground-scale", type=float, default=1.0, help="contract 1: bundle 1.2 scale")
    ap.add_argument(
        "--random-states",
        type=Path,
        default=None,
        help="contract 2 only: also write 10^4 random step cases here",
    )
    args = ap.parse_args()
    vehicle = vehicle_numbers(read_vehicle_params(args.vehicle_params))
    nominal = NOMINAL_ASYMMETRIC if args.nominal == "asymmetric" else NOMINAL
    if args.contract == 2:
        nom, ground = make_bundle_v2(
            args.out,
            vehicle,
            learned_on=args.residual == "on",
            nominal=nominal,
            v_max=args.v_max,
            seed=args.seed,
        )
        if args.random_states:
            write_random_states(args.random_states, vehicle, nom, ground)
        if args.sidecar:
            write_sidecar(args.out, vehicle)
        return
    make_bundle(
        args.out,
        vehicle,
        residual_on=args.residual == "on",
        nominal=nominal,
        v_max=args.v_max,
        seed=args.seed,
        hidden=args.hidden,
        ground_scale=args.ground_scale,
        model_id="dyn-fixture-v0-2026-11-12"
        if args.hidden == 32 and args.ground_scale == 1.0
        else f"dyn-fixture-h{args.hidden}{'-gs' if args.ground_scale != 1.0 else ''}-2026-11-25",
    )
    if args.sidecar:
        write_sidecar(args.out, vehicle)


if __name__ == "__main__":
    main()
