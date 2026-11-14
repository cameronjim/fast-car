"""Seeded interleaved run order (pre-registration sections 5.2 and 5.3): every condition gets the
same number of runs, the order is random for a given seed, and no condition runs more than twice
in a row."""

from __future__ import annotations

import random
from collections.abc import Mapping, Sequence

#: "no condition more than twice in a row" (sections 5.2 and 5.3).
MAX_IN_A_ROW = 2


def _feasible(counts: dict[str, int], last: str | None, run_len: int) -> bool:
    """Necessary condition for ordering the remaining runs after a sequence that ends in a run of
    `last` of length `run_len`: a condition with m runs left and o runs of others left fits in
    at most MAX_IN_A_ROW * (o + 1) places, less the run_len already used if it is `last`."""
    left = sum(counts.values())
    for cond, m in counts.items():
        room = MAX_IN_A_ROW * (left - m + 1) - (run_len if cond == last else 0)
        if m > room:
            return False
    return True


def run_order(
    conditions: Sequence[str],
    runs_per_condition: int | Mapping[str, int],
    seed: int,
) -> list[str]:
    """A seeded order of condition names with each condition exactly `runs_per_condition` times
    (an int for all, or a mapping per condition) and no condition more than twice in a row.
    Raises ValueError when no such order exists."""
    names = list(conditions)
    if not names or len(set(names)) != len(names):
        raise ValueError("conditions must be a non-empty list of distinct names")
    if isinstance(runs_per_condition, int):
        counts = dict.fromkeys(names, runs_per_condition)
    else:
        if set(runs_per_condition) != set(names):
            raise ValueError("runs_per_condition must name exactly the conditions")
        counts = {n: int(runs_per_condition[n]) for n in names}
    if any(c < 0 for c in counts.values()) or sum(counts.values()) == 0:
        raise ValueError("every condition needs a non-negative count and the total must be > 0")
    if not _feasible(dict(counts), None, 0):
        raise ValueError(f"no order has at most {MAX_IN_A_ROW} in a row for counts {counts}")
    rng = random.Random(seed)
    order: list[str] = []

    def place(last: str | None, run_len: int) -> bool:
        if all(c == 0 for c in counts.values()):
            return True
        options = [n for n in names if counts[n] > 0 and not (n == last and run_len >= 2)]
        rng.shuffle(options)
        for n in options:
            counts[n] -= 1
            new_len = run_len + 1 if n == last else 1
            if _feasible(counts, n, new_len):
                order.append(n)
                if place(n, new_len):
                    return True
                order.pop()
            counts[n] += 1
        return False

    if not place(None, 0):
        raise ValueError(f"no order has at most {MAX_IN_A_ROW} in a row for counts {counts}")
    return order
