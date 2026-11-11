"""Domain randomisation for the v0 residual (claude-docs/08-learning.md: "Domain
randomization: around fitted parameters, ranges from fit residuals + drift record").

THERE IS NO FIT YET. Phase 3 sysid has not run, so every range below is centred on the real
car's 2026-11-10 hand measurements where one exists and is otherwise a deliberately WIDE
guess, with the reason stated next to the number in `training/configs/v0/`. When sysid lands,
ranges are re-derived from fit residuals + the drift record and this v0 is superseded
(roadmap 5.1). Nothing here is evidence about the real car.

What is randomised, per episode (`EpisodeDraw`):

  * tyre grip: friction coefficient and front/rear cornering stiffness,
  * mass (and yaw inertia with it),
  * steering: left and right limit separately, gain, and a trim bias,
  * speed: command-to-speed scale error, top speed, acceleration limit,
  * actuator lag: steering and throttle first-order time constants,
  * transport delay: command-to-torque delay,
  * sensing: speed/yaw-rate noise and bias, pose-estimate error (the localiser), LiDAR
    noise and dropout, LiDAR refresh period (the real RPLidar C1 runs at ~10 Hz, the policy at
    50 Hz) and an extra observation latency.

Everything is sampled from a `numpy.random.Generator` the caller owns, so a (seed, config)
pair always yields the same draw: the paired evaluation hands the SAME draw to the tracker-only
arm and the tracker+residual arm.

Torch-free by construction (like the rest of the env code).
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass

import numpy as np

Range = tuple[float, float]


@dataclass(frozen=True)
class RandomizationConfig:
    enabled: bool = False

    # -- tyre / chassis (sim default is a gym placeholder; see the v0 note on its units) --
    friction_mu: Range = (0.55, 1.1)
    # Linear cornering stiffness per axle at the static load, N/rad. The committed
    # placeholder (4.7 N/rad) reproduces ~1.8 m radius at full lock where the real car
    # turns in 0.60 m, so v0 replaces it with a range that reproduces the measured radius.
    cornering_stiffness_front_n_per_rad: Range = (80.0, 400.0)
    cornering_stiffness_rear_ratio: Range = (0.8, 1.3)
    mass_scale: Range = (0.85, 1.20)

    # -- steering (real: left 0.504 rad, right 0.449 rad, 2026-11-10) --
    steering_limit_left_rad: Range = (0.46, 0.52)
    steering_limit_right_rad: Range = (0.41, 0.47)
    steering_gain: Range = (0.92, 1.08)
    steering_bias_rad: Range = (-0.03, 0.03)

    # -- speed (real top speed ~2.08 m/s at the VESC ERPM cap) --
    speed_scale: Range = (0.90, 1.10)
    top_speed_mps: Range = (1.95, 2.20)
    max_accel_mps2: Range = (1.5, 5.0)

    # -- actuators --
    steering_tau_s: Range = (0.02, 0.10)
    throttle_tau_s: Range = (0.05, 0.30)
    command_delay_s: Range = (0.03, 0.13)  # real stop latency ~0.13 s incl. ESC (upper end)

    # -- sensing --
    velocity_noise_std_mps: Range = (0.02, 0.08)
    velocity_bias_mps: Range = (-0.05, 0.05)
    yaw_rate_noise_std_rad_s: Range = (0.01, 0.05)
    yaw_rate_bias_rad_s: Range = (-0.03, 0.03)
    pose_noise_std_m: Range = (0.005, 0.03)  # localiser position error (AR(1), ~1 s)
    pose_yaw_noise_std_rad: Range = (0.005, 0.04)
    lidar_noise_std_m: Range = (0.003, 0.02)
    lidar_dropout_prob: Range = (0.0, 0.03)
    lidar_period_steps: tuple[int, int] = (4, 6)  # 8-12 Hz at the 50 Hz control rate
    obs_latency_s: Range = (0.0, 0.06)

    def nominal(self) -> RandomizationConfig:
        """Every range collapsed to its centre, all noise, bias and latency removed, but still
        `enabled` so the nominal real-car dynamics (not the committed gym placeholders) are
        applied. The evaluation's "nominal" condition."""
        updates = {}
        quiet = {
            "velocity_noise_std_mps",
            "velocity_bias_mps",
            "yaw_rate_noise_std_rad_s",
            "yaw_rate_bias_rad_s",
            "pose_noise_std_m",
            "pose_yaw_noise_std_rad",
            "lidar_noise_std_m",
            "lidar_dropout_prob",
            "obs_latency_s",
            "steering_bias_rad",
        }
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            if f.name in quiet:
                updates[f.name] = (0.0, 0.0)
            elif (
                isinstance(value, tuple)
                and len(value) == 2
                and all(isinstance(v, float) for v in value)
            ):
                mid = 0.5 * (value[0] + value[1])
                updates[f.name] = (mid, mid)
        updates["lidar_period_steps"] = (5, 5)
        return dataclasses.replace(self, enabled=True, **updates)

    def widened(self, factor: float) -> RandomizationConfig:
        """A copy with every continuous range widened about its midpoint by `factor` (used by
        the evaluation's out-of-range robustness check; `enabled` and integer ranges kept)."""
        updates = {}
        for f in dataclasses.fields(self):
            value = getattr(self, f.name)
            if (
                isinstance(value, tuple)
                and len(value) == 2
                and all(isinstance(v, float) for v in value)
            ):
                mid = 0.5 * (value[0] + value[1])
                half = 0.5 * (value[1] - value[0]) * factor
                lo = mid - half
                # Keep strictly-positive quantities positive.
                if value[0] >= 0.0:
                    lo = max(lo, 0.0)
                updates[f.name] = (lo, mid + half)
        return dataclasses.replace(self, **updates)


@dataclass(frozen=True)
class EpisodeDraw:
    """One concrete realisation of every randomised quantity. The nominal draw (centres of
    the ranges, no noise) is `EpisodeDraw.nominal()`."""

    friction_mu: float = 1.0
    cornering_stiffness_front_n_per_rad: float = 200.0
    cornering_stiffness_rear_n_per_rad: float = 200.0
    mass_scale: float = 1.0
    steering_limit_left_rad: float = 0.504
    steering_limit_right_rad: float = 0.449
    steering_gain: float = 1.0
    steering_bias_rad: float = 0.0
    speed_scale: float = 1.0
    top_speed_mps: float = 2.08
    max_accel_mps2: float = 3.0
    steering_tau_s: float = 0.05
    throttle_tau_s: float = 0.10
    command_delay_s: float = 0.08
    velocity_noise_std_mps: float = 0.0
    velocity_bias_mps: float = 0.0
    yaw_rate_noise_std_rad_s: float = 0.0
    yaw_rate_bias_rad_s: float = 0.0
    pose_noise_std_m: float = 0.0
    pose_yaw_noise_std_rad: float = 0.0
    lidar_noise_std_m: float = 0.0
    lidar_dropout_prob: float = 0.0
    lidar_period_steps: int = 5
    obs_latency_steps: int = 0

    def as_dict(self) -> dict[str, float]:
        return dataclasses.asdict(self)


def _uniform(rng: np.random.Generator, r: Range) -> float:
    return float(rng.uniform(r[0], r[1]))


def sample_draw(rng: np.random.Generator, cfg: RandomizationConfig, dt_s: float) -> EpisodeDraw:
    """Draw one `EpisodeDraw`. Uses a FIXED number of `rng` calls regardless of `cfg` values,
    so adding or removing a range never shifts the stream for later fields."""
    d = {}
    d["friction_mu"] = _uniform(rng, cfg.friction_mu)
    d["cornering_stiffness_front_n_per_rad"] = _uniform(
        rng, cfg.cornering_stiffness_front_n_per_rad
    )
    d["cornering_stiffness_rear_n_per_rad"] = d["cornering_stiffness_front_n_per_rad"] * _uniform(
        rng, cfg.cornering_stiffness_rear_ratio
    )
    d["mass_scale"] = _uniform(rng, cfg.mass_scale)
    d["steering_limit_left_rad"] = _uniform(rng, cfg.steering_limit_left_rad)
    d["steering_limit_right_rad"] = _uniform(rng, cfg.steering_limit_right_rad)
    d["steering_gain"] = _uniform(rng, cfg.steering_gain)
    d["steering_bias_rad"] = _uniform(rng, cfg.steering_bias_rad)
    d["speed_scale"] = _uniform(rng, cfg.speed_scale)
    d["top_speed_mps"] = _uniform(rng, cfg.top_speed_mps)
    d["max_accel_mps2"] = _uniform(rng, cfg.max_accel_mps2)
    d["steering_tau_s"] = _uniform(rng, cfg.steering_tau_s)
    d["throttle_tau_s"] = _uniform(rng, cfg.throttle_tau_s)
    d["command_delay_s"] = _uniform(rng, cfg.command_delay_s)
    d["velocity_noise_std_mps"] = _uniform(rng, cfg.velocity_noise_std_mps)
    d["velocity_bias_mps"] = _uniform(rng, cfg.velocity_bias_mps)
    d["yaw_rate_noise_std_rad_s"] = _uniform(rng, cfg.yaw_rate_noise_std_rad_s)
    d["yaw_rate_bias_rad_s"] = _uniform(rng, cfg.yaw_rate_bias_rad_s)
    d["pose_noise_std_m"] = _uniform(rng, cfg.pose_noise_std_m)
    d["pose_yaw_noise_std_rad"] = _uniform(rng, cfg.pose_yaw_noise_std_rad)
    d["lidar_noise_std_m"] = _uniform(rng, cfg.lidar_noise_std_m)
    d["lidar_dropout_prob"] = _uniform(rng, cfg.lidar_dropout_prob)
    d["lidar_period_steps"] = int(
        rng.integers(cfg.lidar_period_steps[0], cfg.lidar_period_steps[1] + 1)
    )
    latency_s = _uniform(rng, cfg.obs_latency_s)
    d["obs_latency_steps"] = round(latency_s / dt_s)
    d["command_delay_s"] = max(d["command_delay_s"], 0.0)
    return EpisodeDraw(**d)


def apply_draw_to_dyn_params(dyn_params, draw: EpisodeDraw, dt_s: float):
    """A copy of racer_gym `DynParams` with `draw`'s physical quantities applied.

    Pacejka curves are rebuilt from (mu, cornering stiffness) at the new static axle loads:
    peak force D = mu * Fz_static, small-slip slope B*C*D = C_alpha. The shape factor C and
    curvature E are kept from the base params (generic Magic-Formula defaults upstream).
    """
    from racer_gym.dynamics.load_transfer import static_axle_loads
    from racer_gym.dynamics.tire import PacejkaParams

    mass = dyn_params.mass_kg * draw.mass_scale
    inertia = dyn_params.yaw_inertia_kg_m2 * draw.mass_scale
    static = static_axle_loads(mass, dyn_params.cg_to_front_axle_m, dyn_params.cg_to_rear_axle_m)

    def pacejka(base: PacejkaParams, fz_n: float, c_alpha: float) -> PacejkaParams:
        d_peak = draw.friction_mu * fz_n
        return PacejkaParams(
            b_stiffness=c_alpha / (base.c_shape * d_peak),
            c_shape=base.c_shape,
            d_peak_n=d_peak,
            e_curvature=base.e_curvature,
        )

    return dataclasses.replace(
        dyn_params,
        mass_kg=mass,
        yaw_inertia_kg_m2=inertia,
        pacejka_front=pacejka(
            dyn_params.pacejka_front, static.front_n, draw.cornering_stiffness_front_n_per_rad
        ),
        pacejka_rear=pacejka(
            dyn_params.pacejka_rear, static.rear_n, draw.cornering_stiffness_rear_n_per_rad
        ),
        steer_tau_s=draw.steering_tau_s,
        throttle_tau_s=draw.throttle_tau_s,
        delay_steps=max(round(draw.command_delay_s / dt_s), 0),
        s_min=-draw.steering_limit_right_rad,
        s_max=draw.steering_limit_left_rad,
        v_max=draw.top_speed_mps,
        a_max=draw.max_accel_mps2,
    )


class SensorNoise:
    """Per-episode sensing error process. One instance per episode; its internal RNG is
    seeded from the episode seed and consumes a FIXED number of draws per `step()` call, so
    two arms of a paired evaluation (different actions, same seed) see identical noise."""

    def __init__(self, draw: EpisodeDraw, seed: int, dt_s: float, n_beams: int) -> None:
        self._draw = draw
        self._rng = np.random.default_rng([seed, 7919])
        self._dt = dt_s
        self._n_beams = n_beams
        # AR(1) with ~1 s correlation time: x[k+1] = a x[k] + sqrt(1-a^2) sigma w.
        self._a = math.exp(-dt_s / 1.0)
        self._pose_err = np.zeros(3)  # x, y, yaw error
        self._steps = 0

    def step(self) -> dict[str, object]:
        d = self._draw
        w = self._rng.standard_normal(3)
        sigma = np.array([d.pose_noise_std_m, d.pose_noise_std_m, d.pose_yaw_noise_std_rad])
        self._pose_err = self._a * self._pose_err + math.sqrt(1.0 - self._a**2) * sigma * w
        v_noise = (
            float(self._rng.standard_normal()) * d.velocity_noise_std_mps + d.velocity_bias_mps
        )
        yaw_noise = (
            float(self._rng.standard_normal()) * d.yaw_rate_noise_std_rad_s + d.yaw_rate_bias_rad_s
        )
        beam_noise = self._rng.standard_normal(self._n_beams) * d.lidar_noise_std_m
        beam_drop = self._rng.random(self._n_beams) < d.lidar_dropout_prob
        refresh = (self._steps % d.lidar_period_steps) == 0
        self._steps += 1
        return {
            "pose_error": self._pose_err.copy(),
            "velocity_noise_mps": v_noise,
            "yaw_rate_noise_rad_s": yaw_noise,
            "lidar_noise_m": beam_noise,
            "lidar_dropout": beam_drop,
            "lidar_refresh": refresh,
        }
