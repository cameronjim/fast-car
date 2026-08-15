"""Value types shared across the envelope library (training/envelope).

Every dataclass here is frozen: the envelope has no hidden state anywhere. Anything that
must persist between calls to `envelope.envelope.apply` is carried explicitly by the caller
as an `EnvelopeState`, so the training environment and the on-vehicle deploy node share
identical semantics (claude-docs/02-repo-layout.md, claude-docs/05-safety.md layer 4).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Command:
    """A two-channel Ackermann-style drive command.

    steering_rad: road-wheel angle, radians, LEFT positive
        (claude-docs/06-vehicle-params.md sign convention).
    speed_mps: forward speed, m/s, positive forward, negative reverse.
    """

    steering_rad: float
    speed_mps: float


@dataclass(frozen=True)
class EnvelopeState:
    """The only state `apply` depends on besides its explicit arguments.

    Holds the previous SAFE (already-enveloped) command, which rate limiting measures
    change against. Callers own this value: construct the initial one themselves -- there
    is no library-side default "zero" state -- and thread the `next_state` returned by one
    `apply` call into the next one.
    """

    last_output: Command


@dataclass(frozen=True)
class EnvelopeResult:
    """Everything one `apply` call produces.

    `command` is the safe command to send to the actuators (or to the sim env). `next_state`
    is what the following `apply` call must be given. The three flags are diagnostics only --
    they never feed back into behavior -- so deployment/training code can log interventions
    per claude-docs/05-safety.md ("an unlogged intervention is a bug") without re-deriving
    them from `command` after the fact.
    """

    command: Command
    next_state: EnvelopeState
    ood_triggered: bool
    residual_clipped: bool
    rate_limited: bool
