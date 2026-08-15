"""Layer-4 policy envelope: hard residual bounds, rate limits, OOD fallback, speed cap.

See claude-docs/05-safety.md (layer 4) and claude-docs/08-learning.md for the architecture
this implements, and `envelope.envelope` for the `apply` entry point's exact semantics
(especially its NaN/inf policy). This is one library, imported unmodified by both the
training environment and the on-vehicle `racer_policy` deploy node
(claude-docs/02-repo-layout.md) -- nothing here may depend on ROS, torch, or numpy.
"""

from __future__ import annotations

from envelope.envelope import apply
from envelope.ood import DistanceOODScorer, OODScorer
from envelope.params import EnvelopeConfig, LimitsParamsLike, SteeringParamsLike, VehicleParamsLike
from envelope.types import Command, EnvelopeResult, EnvelopeState

__all__ = [
    "Command",
    "DistanceOODScorer",
    "EnvelopeConfig",
    "EnvelopeResult",
    "EnvelopeState",
    "LimitsParamsLike",
    "OODScorer",
    "SteeringParamsLike",
    "VehicleParamsLike",
    "apply",
]
