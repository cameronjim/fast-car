"""OOD (out-of-distribution) scoring for the layer-4 fallback trigger.

`OODScorer` is the seam between this library and the real OOD detector (critic disagreement
or a fitted state-distribution distance), which is S.3-adjacent and not built yet. All this
module ships is that interface plus one simple, dependency-free reference implementation --
`DistanceOODScorer` -- good enough to unblock S.4/S.5 development and testing: a normalized
Euclidean distance from a fixed reference state.

Fail-closed convention: a scorer that cannot compute a meaningful distance (mismatched
dimensionality, non-finite input) returns `math.inf`, never raises and never returns a
finite-but-wrong number, so the envelope's threshold trigger fails toward the safe fallback
rather than silently treating bad state as "not OOD" (claude-docs/05-safety.md's fail-closed
rule, stated there for layer 3 and adopted here for layer 4).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Protocol, runtime_checkable


@runtime_checkable
class OODScorer(Protocol):
    """Anything that can score how far a state is from the training distribution.

    Higher is more out-of-distribution; the scale is whatever the implementation defines
    (the envelope only compares it to a configured threshold, never interprets it). An
    implementation must never raise: on any input it cannot meaningfully score, it returns
    `math.inf` (see module docstring).
    """

    def score(self, state: Sequence[float]) -> float: ...


@dataclass(frozen=True)
class DistanceOODScorer:
    """Reference OOD scorer: normalized Euclidean distance from a fixed reference state.

    This stands in for the real S.3-adjacent scorer -- cleanly separated behind `OODScorer`
    so swapping in critic disagreement or a fitted distribution distance later touches no
    envelope decision logic. `reference` is typically the training distribution's mean
    state; `scale` is a per-dimension normalizer (e.g. per-dimension standard deviation) so
    no single dimension dominates purely because of its units. `scale=None` means "no
    normalization" (equivalent to a scale of 1.0 in every dimension).

    A `scale` entry of exactly `0.0` means "this dimension has no spread, don't divide by
    it" -- the raw difference is used unnormalized for that dimension -- rather than
    producing `inf`/`nan` from a division by zero.
    """

    reference: tuple[float, ...]
    scale: tuple[float, ...] | None = None

    def score(self, state: Sequence[float]) -> float:
        if self.scale is not None:
            scale = self.scale
        else:
            scale = tuple(1.0 for _ in self.reference)

        if len(state) != len(self.reference) or len(scale) != len(self.reference):
            return math.inf

        total = 0.0
        for value, ref, dim_scale in zip(state, self.reference, scale):
            if not math.isfinite(value) or not math.isfinite(ref) or not math.isfinite(dim_scale):
                return math.inf
            diff = value - ref
            if dim_scale != 0.0:
                normalized = diff / dim_scale
            else:
                normalized = diff
            total += normalized * normalized

        result = math.sqrt(total)
        if math.isfinite(result):
            return result
        return math.inf
