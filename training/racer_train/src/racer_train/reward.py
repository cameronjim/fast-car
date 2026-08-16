"""Reward = progress along the raceline, minus crash, minus envelope violation. NOTHING ELSE.

claude-docs/08-learning.md "Boring choices, committed": "Reward: progress along raceline -
crash - envelope violation. Nothing else. Every shaping term added later (including
action-rate penalties) is logged in docs/notes/reward-confessions.md and treated as evidence
of a modeling defect to chase. Order of attack for oscillation: model actuator dynamics
properly -> hard rate constraints in the env matching the physical actuator -> only then a
penalty."

This module is the ONE place reward is computed for the S.3 training env. Adding a fourth
term here without first logging it in docs/notes/reward-confessions.md is exactly the thing
that comment is warning against -- if you are here to add one, stop and write that entry
first (it is a modeling-defect confession, not a design decision to make quietly in a diff).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RewardWeights:
    """Scalar weights on the three (and only three) reward terms. All three are tuning
    knobs, not physical constants -- there is no vehicle_params field for "how much a crash
    should hurt"."""

    progress_weight: float = 1.0
    crash_penalty: float = 10.0
    envelope_violation_penalty: float = 0.1


@dataclass(frozen=True)
class RewardTerms:
    """The three reward terms for one step, kept separate (rather than collapsed into a
    single float) so callers can log the breakdown -- claude-docs/05-safety.md: "an unlogged
    intervention is a bug" applies to the envelope_violation term here too."""

    progress: float
    crash: float
    envelope_violation: float

    @property
    def total(self) -> float:
        return self.progress - self.crash - self.envelope_violation


def progress_reward(s_prev_m: float, s_new_m: float, track_length_m: float) -> float:
    """Signed arc-length progress since the last step, unwrapped across the closed loop's
    start/finish line.

    A step that completes a lap (s wraps from near `track_length_m` back to near 0) must
    register as a small POSITIVE step, not a huge negative regression; a step that drives
    backward across the line must register as a small NEGATIVE step, not a huge positive
    "progress". This is the standard circular-shortest-delta computation: if the naive delta
    is more than half the track length in either direction, it is shorter (and therefore the
    intended interpretation) to have wrapped the other way around the loop.
    """
    if track_length_m <= 0.0:
        raise ValueError(f"track_length_m must be > 0, got {track_length_m}")
    delta = s_new_m - s_prev_m
    half = track_length_m / 2.0
    if delta < -half:
        delta += track_length_m
    elif delta > half:
        delta -= track_length_m
    return delta


def compute_reward(
    *,
    s_prev_m: float,
    s_new_m: float,
    track_length_m: float,
    crashed: bool,
    envelope_intervened: bool,
    weights: RewardWeights,
) -> RewardTerms:
    """`envelope_intervened` is true whenever this step's `envelope.apply()` call clipped the
    residual, rate-limited the command, or fell back to the base controller on OOD -- see
    `racer_train.env.ResidualRacerEnv.step`, which derives it from the `EnvelopeResult` flags
    (claude-docs/12-testing.md's L5 "envelope-in-env test" divergence check exercises the
    same `apply()` call this flag comes from)."""
    progress = weights.progress_weight * progress_reward(s_prev_m, s_new_m, track_length_m)
    crash = weights.crash_penalty if crashed else 0.0
    envelope_violation = weights.envelope_violation_penalty if envelope_intervened else 0.0
    return RewardTerms(progress=progress, crash=crash, envelope_violation=envelope_violation)
