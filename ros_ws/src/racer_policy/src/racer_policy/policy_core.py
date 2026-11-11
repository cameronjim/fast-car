"""The decision logic of the policy node, free of ROS (claude-docs/10-conventions.md).

One `step` per base command: build the observation exactly as in training, run the network,
scale its action, and pass base + residual through the layer-4 `envelope.apply` (the same
library training runs, never forked). The output of `step` is the only thing the node
publishes on /drive_raw.

Fallback to the plain tracker (the residual forced to zero, the base command still going
through the envelope's rate limits and speed cap) happens when:

* the envelope's own OOD scorer fires (state far from the training distribution),
* there is no scan yet or the last one is older than `scan_timeout_s` (the LiDAR is the one
  slow input, about 10 Hz; pose and odometry freshness are the node's own watchdog),
* the network raises or returns a non-finite action.

Each entry to and exit from fallback is returned as a transition so the node can log it once
(engage/release, like the safety node's events); per-step flags are counted, not logged.
"""

from __future__ import annotations

import dataclasses
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from racer_policy.deploy import Deployment, build_envelope_config
from racer_policy.jitter import INFERENCE_EDGES_US, PERIOD_EDGES_MS, Histogram
from racer_policy.live_observation import (
    OOD_STATE_DIM,
    Array,
    LiveInputs,
    LiveObservationBuilder,
    Raceline,
)

ModelFn = Callable[[Array], Array]

REASON_NONE = ""
REASON_OOD = "ood"
REASON_NO_SCAN = "no_scan"
REASON_SCAN_STALE = "scan_stale"
REASON_MODEL_ERROR = "model_error"
REASON_NONFINITE = "nonfinite_action"


@dataclass(frozen=True)
class StepResult:
    steering_rad: float
    speed_mps: float
    active: bool  # False = the plain tracker's command went out
    reason: str
    residual_steering_rad: float  # the residual the envelope was handed (after scaling)
    residual_speed_mps: float
    clipped: bool
    rate_limited: bool
    ood: bool
    inference_us: float
    transition: str | None  # "engage" / "release" of a fallback, else None
    fallback_duration_s: float | None = None


@dataclass
class Counters:
    steps: int = 0
    fallback_steps: int = 0
    clipped_steps: int = 0
    rate_limited_steps: int = 0
    ood_steps: int = 0
    no_scan_steps: int = 0
    scan_stale_steps: int = 0
    model_error_steps: int = 0
    nonfinite_steps: int = 0
    fallback_engagements: int = 0
    # Over steps where the residual was actually applied (active, not fallback).
    active_abs_residual_steering_sum_rad: float = 0.0
    active_abs_residual_steering_max_rad: float = 0.0
    active_abs_residual_speed_sum_mps: float = 0.0
    active_abs_residual_speed_max_mps: float = 0.0


@dataclass
class PolicyCore:
    deployment: Deployment
    vehicle_params: Any
    raceline: Raceline
    model: ModelFn
    scan_timeout_s: float = 0.5
    counters: Counters = field(default_factory=Counters)

    def __post_init__(self) -> None:
        from envelope import Command, EnvelopeState

        self._Command = Command
        self._EnvelopeState = EnvelopeState
        self._config = build_envelope_config(self.deployment, self.vehicle_params)
        self._builder = LiveObservationBuilder(
            self.raceline, float(self.vehicle_params.chassis.wheelbase_m)
        )
        self.inference_us = Histogram(INFERENCE_EDGES_US)
        self.period_ms = Histogram(PERIOD_EDGES_MS)
        self.reset()

    # -- lifecycle -----------------------------------------------------------------------

    def reset(self) -> None:
        """Forget the previous command and the fallback state: call when the base controller
        (re)starts producing commands after a gap, like training's episode reset."""
        self._state = self._EnvelopeState(last_output=self._Command(0.0, 0.0))
        self._prev = (0.0, 0.0)
        self._started = False
        self._in_fallback = False
        self._fallback_since_s = 0.0
        self._last_step_s: float | None = None
        self.last_reason = REASON_NONE

    @property
    def envelope_config(self) -> Any:
        return self._config

    # -- the step ------------------------------------------------------------------------

    def step(
        self,
        *,
        now_s: float,
        velocity_mps: float,
        yaw_rate_rad_s: float,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        base_steering_rad: float,
        base_speed_mps: float,
        beams: Array | None,
        scan_age_s: float | None,
    ) -> StepResult:
        from envelope import apply as envelope_apply

        c = self.counters
        if not self._started:
            # First command after (re)start: seed the rate limiter at the base command, as the
            # training env seeds it at the start state, so the first output is not clamped
            # toward zero.
            self._state = self._EnvelopeState(
                last_output=self._Command(base_steering_rad, base_speed_mps)
            )
            self._prev = (base_steering_rad, base_speed_mps)
            self._started = True
        if self._last_step_s is not None:
            self.period_ms.add(1000.0 * (now_s - self._last_step_s))
        self._last_step_s = now_s

        t0 = time.perf_counter()
        reason = REASON_NONE
        action: Array = np.zeros(2, dtype=np.float64)
        obs: Array | None = None
        if beams is None or scan_age_s is None:
            reason = REASON_NO_SCAN
        elif scan_age_s > self.scan_timeout_s:
            reason = REASON_SCAN_STALE
        else:
            try:
                obs = self._builder.build(
                    LiveInputs(
                        velocity_mps=velocity_mps,
                        yaw_rate_rad_s=yaw_rate_rad_s,
                        x_m=x_m,
                        y_m=y_m,
                        yaw_rad=yaw_rad,
                        base_steering_rad=base_steering_rad,
                        base_speed_mps=base_speed_mps,
                        prev_steering_cmd_rad=self._prev[0],
                        prev_speed_cmd_mps=self._prev[1],
                        beams=beams,
                    )
                )
                out = np.asarray(self.model(obs), dtype=np.float64).reshape(-1)
                if out.shape != (2,) or not np.all(np.isfinite(out)):
                    reason = REASON_NONFINITE
                else:
                    action = np.clip(out, -1.0, 1.0)
            except Exception:  # noqa: BLE001 - any inference failure is a fallback, never a crash
                reason = REASON_MODEL_ERROR
        inference_us = 1e6 * (time.perf_counter() - t0)
        self.inference_us.add(inference_us)

        residual = self._Command(
            steering_rad=float(action[0]) * self.deployment.steering_scale_rad,
            speed_mps=float(action[1]) * self.deployment.speed_scale_mps,
        )
        if obs is not None:
            observed = tuple(float(v) for v in obs[:OOD_STATE_DIM])
        else:
            # No observation to score: score the reference state itself (distance zero) so a
            # made-up state is never read as an OOD verdict.
            observed = tuple(self.deployment.envelope.ood_reference_state)
        base = self._Command(steering_rad=base_steering_rad, speed_mps=base_speed_mps)

        result = envelope_apply(self._config, self._state, base, residual, observed_state=observed)
        self._state = result.next_state
        self._prev = (result.command.steering_rad, result.command.speed_mps)

        if result.ood_triggered and reason == REASON_NONE:
            reason = REASON_OOD
        fallback = reason != REASON_NONE

        transition: str | None = None
        duration: float | None = None
        if fallback and not self._in_fallback:
            self._in_fallback = True
            self._fallback_since_s = now_s
            c.fallback_engagements += 1
            transition = "engage"
        elif not fallback and self._in_fallback:
            self._in_fallback = False
            transition = "release"
            duration = now_s - self._fallback_since_s
        self.last_reason = reason

        c.steps += 1
        c.fallback_steps += int(fallback)
        c.clipped_steps += int(result.residual_clipped)
        c.rate_limited_steps += int(result.rate_limited)
        c.ood_steps += int(result.ood_triggered)
        c.no_scan_steps += int(reason == REASON_NO_SCAN)
        c.scan_stale_steps += int(reason == REASON_SCAN_STALE)
        c.model_error_steps += int(reason == REASON_MODEL_ERROR)
        c.nonfinite_steps += int(reason == REASON_NONFINITE)
        if not fallback:
            a_s, a_v = abs(residual.steering_rad), abs(residual.speed_mps)
            c.active_abs_residual_steering_sum_rad += a_s
            c.active_abs_residual_steering_max_rad = max(
                c.active_abs_residual_steering_max_rad, a_s
            )
            c.active_abs_residual_speed_sum_mps += a_v
            c.active_abs_residual_speed_max_mps = max(c.active_abs_residual_speed_max_mps, a_v)

        return StepResult(
            steering_rad=result.command.steering_rad,
            speed_mps=result.command.speed_mps,
            active=not fallback,
            reason=reason,
            residual_steering_rad=residual.steering_rad,
            residual_speed_mps=residual.speed_mps,
            clipped=result.residual_clipped,
            rate_limited=result.rate_limited,
            ood=result.ood_triggered,
            inference_us=inference_us,
            transition=transition,
            fallback_duration_s=duration,
        )

    def snapshot(self) -> dict[str, Any]:
        """Status for the 1 Hz /policy/status record: counters, state, both histograms."""
        return {
            "state": "fallback" if self._in_fallback else "active",
            "reason": self.last_reason,
            "counters": dataclasses.asdict(self.counters),
            "inference_us": self.inference_us.to_dict(),
            "loop_period_ms": self.period_ms.to_dict(),
        }
