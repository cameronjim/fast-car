"""The layer-4 policy envelope entry point: `apply`.

Enforced in the deployment node, not learned, not trusted from the policy
(claude-docs/05-safety.md layer 4: "risk reduction", never described as a guarantee -- only
layer 1, the hardware RC mux, is that). The same function runs inside the training
environment so training never rewards a behavior the deployment will clip
(claude-docs/02-repo-layout.md: `envelope/` is one library, never forked).

`apply` is a pure function of its arguments: same inputs -> same outputs, every time, with
no state beyond the explicit `EnvelopeState` the caller threads through calls. It does not
read a clock or a call counter, so "rate limit" here means "maximum change in the output
per `apply` call", not a wall-clock rate; a caller running the deploy loop at a fixed control
rate (08-learning.md: 50 Hz) converts a physical rad/s or (m/s)/s limit into a per-call delta
once, at config-construction time.

NAN / INF POLICY (claude-docs/12-testing.md L2 -- "the single most important test in the
repo": for ANY input command and ANY internal state, output is always within bounds and
within rate limits of the previous output). This is deliberate fail-closed handling, not an
omission:

  * A non-finite (`NaN` or +-`inf`) BASE command channel is replaced with `0.0` for that
    channel (0 rad steering, 0 m/s speed) before anything else happens. The base command is
    supposed to come from the tuned classical stack; if it is garbage, the safe fallback is
    the same as a layer-3 fail-closed brake, not "trust it because the residual looks fine".
  * A non-finite RESIDUAL (either channel) is treated as an OOD condition for that call: the
    residual is dropped entirely and the (sanitized) base command is used unmodified. This
    reuses the OOD fallback path rather than adding a second "ignore the residual" code path.
  * A non-finite OOD score (the scorer itself misbehaving) is treated as maximally
    out-of-distribution (`+inf`), which also triggers the OOD fallback -- `DistanceOODScorer`
    already promises this (see `envelope.ood`), but `apply` does not trust callers to have
    used it.
  * The previous output carried in `EnvelopeState.last_output` is sanitized (non-finite ->
    `0.0` per channel) AND clamped into the current absolute bounds before it is used as the
    rate-limit anchor. Ordinarily this is a no-op: `EnvelopeState` is only ever constructed
    from a previous `EnvelopeResult.next_state`, which is always already a legal command. The
    clamp exists so an arbitrary/corrupted carried state (or bounds that changed between
    calls) can never make "stay in bounds" and "stay within a rate-limit step of the previous
    output" mutually impossible to satisfy at the same time.
  * The final output is unconditionally clamped to `steering_range` and to
    `[speed_range_low, min(speed_range_high, speed_cap_mps)]` as the very last step -- this
    is the absolute-bounds half of the property test, and it holds regardless of which branch
    above fired.
"""

from __future__ import annotations

import math

from envelope.params import EnvelopeConfig
from envelope.types import Command, EnvelopeResult, EnvelopeState


def _sanitize(value: float) -> float:
    if math.isfinite(value):
        return value
    return 0.0


def _clip(value: float, low: float, high: float) -> float:
    if value < low:
        return low
    if value > high:
        return high
    return value


def _sanitize_command(command: Command) -> Command:
    return Command(
        steering_rad=_sanitize(command.steering_rad),
        speed_mps=_sanitize(command.speed_mps),
    )


def apply(
    config: EnvelopeConfig,
    state: EnvelopeState,
    base_command: Command,
    residual: Command,
    observed_state: tuple[float, ...],
) -> EnvelopeResult:
    """Clip `base_command + residual` into the layer-4 envelope. See module docstring."""

    steering_low, steering_high = config.steering_range
    speed_low, speed_range_high = config.speed_range
    speed_high = min(speed_range_high, config.speed_cap_mps)

    safe_base = _sanitize_command(base_command)

    prev_sanitized = _sanitize_command(state.last_output)
    prev_legal = Command(
        steering_rad=_clip(prev_sanitized.steering_rad, steering_low, steering_high),
        speed_mps=_clip(prev_sanitized.speed_mps, speed_low, speed_high),
    )

    if math.isfinite(residual.steering_rad) and math.isfinite(residual.speed_mps):
        residual_finite = True
    else:
        residual_finite = False

    raw_score = config.ood_scorer.score(observed_state)
    if math.isfinite(raw_score):
        score = raw_score
    else:
        score = math.inf

    if not residual_finite or score > config.ood_threshold:
        ood_triggered = True
    else:
        ood_triggered = False

    residual_clipped = False
    if ood_triggered:
        target = safe_base
    else:
        max_steering_delta = config.residual_fraction_steering * (steering_high - steering_low)
        max_speed_delta = config.residual_fraction_speed * (speed_range_high - speed_low)

        clipped_steering_residual = _clip(
            residual.steering_rad, -max_steering_delta, max_steering_delta
        )
        clipped_speed_residual = _clip(residual.speed_mps, -max_speed_delta, max_speed_delta)
        if clipped_steering_residual != residual.steering_rad:
            residual_clipped = True
        if clipped_speed_residual != residual.speed_mps:
            residual_clipped = True
        target = Command(
            steering_rad=safe_base.steering_rad + clipped_steering_residual,
            speed_mps=safe_base.speed_mps + clipped_speed_residual,
        )

    bounded = Command(
        steering_rad=_clip(target.steering_rad, steering_low, steering_high),
        speed_mps=_clip(target.speed_mps, speed_low, speed_high),
    )

    rate_limited_steering = _clip(
        bounded.steering_rad,
        prev_legal.steering_rad - config.max_delta_steering_rad,
        prev_legal.steering_rad + config.max_delta_steering_rad,
    )
    rate_limited_speed = _clip(
        bounded.speed_mps,
        prev_legal.speed_mps - config.max_delta_speed_mps,
        prev_legal.speed_mps + config.max_delta_speed_mps,
    )
    rate_limited = False
    if rate_limited_steering != bounded.steering_rad:
        rate_limited = True
    if rate_limited_speed != bounded.speed_mps:
        rate_limited = True

    final_command = Command(
        steering_rad=_clip(rate_limited_steering, steering_low, steering_high),
        speed_mps=_clip(rate_limited_speed, speed_low, speed_high),
    )

    return EnvelopeResult(
        command=final_command,
        next_state=EnvelopeState(last_output=final_command),
        ood_triggered=ood_triggered,
        residual_clipped=residual_clipped,
        rate_limited=rate_limited,
    )
