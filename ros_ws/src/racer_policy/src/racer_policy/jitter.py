"""A fixed-bin histogram for the inference-latency and loop-period jitter that
claude-docs/08-learning.md asks to be published ("port policy_node to C++ only if tail
latency is bad"). Plain Python, no numpy, O(log bins) per sample."""

from __future__ import annotations

import bisect
import math
from typing import Any


class Histogram:
    """Counts samples into `edges`-delimited bins plus an overflow bin, and tracks the max.

    `edges` are ascending upper bounds; bin i holds edges[i-1] <= x < edges[i], the last bin
    holds x >= edges[-1]. Quantiles are the upper edge of the bin the quantile falls in
    (an upper bound, never an underestimate), the exact max is kept separately."""

    def __init__(self, edges: tuple[float, ...]) -> None:
        if not edges or list(edges) != sorted(set(edges)):
            raise ValueError("edges must be non-empty, strictly ascending")
        self.edges = tuple(edges)
        self.counts = [0] * (len(edges) + 1)
        self.n = 0
        self.max = 0.0
        self.total = 0.0

    def add(self, value: float) -> None:
        if not math.isfinite(value):
            raise ValueError(f"cannot histogram a non-finite sample: {value!r}")
        self.counts[bisect.bisect_right(self.edges, value)] += 1
        self.n += 1
        self.total += value
        self.max = max(self.max, value)

    def quantile(self, q: float) -> float:
        if self.n == 0:
            return 0.0
        target = q * self.n
        running = 0
        for i, c in enumerate(self.counts):
            running += c
            if running >= target and c:
                return self.edges[i] if i < len(self.edges) else self.max
        return self.max

    def to_dict(self) -> dict[str, Any]:
        return {
            "edges": list(self.edges),
            "counts": list(self.counts),
            "n": self.n,
            "mean": self.total / self.n if self.n else 0.0,
            "p50": self.quantile(0.5),
            "p99": self.quantile(0.99),
            "max": self.max,
        }


# Inference time, microseconds: 50 us up to 20 ms (one 50 Hz period).
INFERENCE_EDGES_US = (50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10000.0, 20000.0)
# Loop period, milliseconds: nominal 20 ms; resolution around it, tail to 100 ms.
PERIOD_EDGES_MS = (10.0, 15.0, 18.0, 19.0, 21.0, 22.0, 25.0, 30.0, 40.0, 60.0, 100.0)
