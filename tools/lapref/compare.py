"""Tables and verdicts of M-LR and M-L1 (pre-registration v0.4 sections 4.2 and 4.3).

Pure functions on lap time lists: the bag work is in tools/lapref_measure.py. The thresholds are
read from `lap_timing` of evaluation/prereg/pnp-v0.4.yaml, never typed here.

Pairing is by order inside the run: lap k of one source against lap k of the other, over the laps
both have. The start lines of the two sources differ, so only lap times and counts are compared,
never crossing stamps.

M-L1 reading. Pre-registration 4.3: "in every test run the two lap counts are equal and at least
95 percent of the laps differ by at most 0.3 s". Two readings are possible for the second half:
the share inside every run (A), or over all laps of the five runs together (B). Both are computed
and both are reported. The headline verdict is A, the stricter, and a difference between the two
is printed. Nothing is chosen in favour of a pass.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class LapTiming:
    tolerance_s: float
    min_share: float


def load_lap_timing(path: Path) -> LapTiming:
    block = yaml.safe_load(Path(path).read_text())["lap_timing"]
    return LapTiming(float(block["tolerance_s"]), float(block["min_share_within_tolerance"]))


@dataclass
class RunComparison:
    name: str
    a_label: str
    b_label: str
    a_times: list[float]
    b_times: list[float]
    tolerance_s: float
    #: b - a for lap k over the laps both have.
    diffs: list[float] = field(default_factory=list)

    def __post_init__(self) -> None:
        n = min(len(self.a_times), len(self.b_times))
        self.diffs = [self.b_times[k] - self.a_times[k] for k in range(n)]

    @property
    def counts_equal(self) -> bool:
        return len(self.a_times) == len(self.b_times)

    @property
    def n_compared(self) -> int:
        return len(self.diffs)

    @property
    def n_within(self) -> int:
        return int(np.sum(np.abs(self.diffs) <= self.tolerance_s + 1e-9)) if self.diffs else 0

    @property
    def n_laps_max(self) -> int:
        """Laps of either source: laps one source lacks count as a miss."""
        return max(len(self.a_times), len(self.b_times))

    @property
    def share_within(self) -> float:
        """Laps within tolerance over the laps of the longer list (a lap one source lacks is a
        lap that does not agree)."""
        return self.n_within / self.n_laps_max if self.n_laps_max else 0.0

    def differing(self) -> list[tuple[int, float | None, float | None, float | None]]:
        """(lap, a, b, b - a) of every lap that does not agree: out of tolerance or missing."""
        out = []
        for k in range(self.n_laps_max):
            a = self.a_times[k] if k < len(self.a_times) else None
            b = self.b_times[k] if k < len(self.b_times) else None
            if a is None or b is None:
                out.append((k + 1, a, b, None))
            elif abs(b - a) > self.tolerance_s + 1e-9:
                out.append((k + 1, a, b, b - a))
        return out


def compare_run(
    name: str,
    a_label: str,
    a_times: Sequence[float],
    b_label: str,
    b_times: Sequence[float],
    tolerance_s: float,
) -> RunComparison:
    return RunComparison(name, a_label, b_label, list(a_times), list(b_times), tolerance_s)


@dataclass(frozen=True)
class MLOneVerdict:
    #: Reading A: every run has equal counts and a share within tolerance at or above the minimum.
    per_run: bool
    #: Reading B: every run has equal counts and the share over all laps together is at or above.
    pooled: bool
    pooled_share: float
    counts_equal_everywhere: bool

    @property
    def headline(self) -> str:
        return "PASS" if self.per_run else "FAIL"

    @property
    def readings_differ(self) -> bool:
        return self.per_run != self.pooled


def m_l1_verdict(runs: Sequence[RunComparison], timing: LapTiming) -> MLOneVerdict:
    counts = all(r.counts_equal for r in runs) and len(runs) > 0
    per_run = counts and all(r.share_within >= timing.min_share - 1e-12 for r in runs)
    total = sum(r.n_laps_max for r in runs)
    within = sum(r.n_within for r in runs)
    share = within / total if total else 0.0
    pooled = counts and share >= timing.min_share - 1e-12
    return MLOneVerdict(per_run, pooled, share, counts)


def _times(values: Sequence[float]) -> str:
    """Signed differences, one decimal pair each."""
    return " ".join(f"{v:+.2f}" for v in values) or "-"


def format_m_l1(runs: Sequence[RunComparison], timing: LapTiming) -> str:
    verdict = m_l1_verdict(runs, timing)
    a, b = (runs[0].a_label, runs[0].b_label) if runs else ("a", "b")
    lines = [
        (
            f"| run | {a} laps | {b} laps | counts | within {timing.tolerance_s:g} s | "
            f"{b} minus {a} (s, per lap) |"
        ),
        "|---|---|---|---|---|---|",
    ]
    for r in runs:
        lines.append(
            f"| {r.name} | {len(r.a_times)} | {len(r.b_times)} | "
            f"{'equal' if r.counts_equal else 'DIFFER'} | {r.n_within} of {r.n_laps_max} "
            f"({100.0 * r.share_within:.0f} percent) | {_times(r.diffs)} |"
        )
    lines.append("")
    lines.append(
        f"M-L1, reading A (every run: equal counts and at least "
        f"{100.0 * timing.min_share:.0f} percent of its laps within {timing.tolerance_s:g} s): "
        f"{verdict.headline}"
    )
    lines.append(
        f"M-L1, reading B (equal counts everywhere and {100.0 * verdict.pooled_share:.1f} percent "
        f"of all laps together within {timing.tolerance_s:g} s): "
        f"{'PASS' if verdict.pooled else 'FAIL'}"
    )
    if verdict.readings_differ:
        lines.append("The two readings give different verdicts; the headline is reading A.")
    differing = [(r.name, d) for r in runs for d in r.differing()]
    if differing:
        lines.append("")
        lines.append("Every differing lap:")
        for name, (lap, ta, tb, diff) in differing:
            lines.append(
                f"- {name} lap {lap}: {a} "
                f"{'-' if ta is None else f'{ta:.2f} s'}, {b} "
                f"{'-' if tb is None else f'{tb:.2f} s'}"
                + ("" if diff is None else f", difference {diff:+.2f} s")
            )
    return "\n".join(lines)


def format_m_lr(runs: Sequence[RunComparison]) -> str:
    """The M-LR table against the pose detector: counts and per-lap differences (no verdict:
    development numbers)."""
    lines = [
        (
            "| segment | pose laps | lapref laps | counts | per lap, lapref minus pose (s) | "
            "pose mean (s) | lapref mean (s) |"
        ),
        "|---|---|---|---|---|---|---|",
    ]
    for r in runs:
        pose_mean = f"{np.mean(r.a_times):.2f}" if r.a_times else "-"
        lap_mean = f"{np.mean(r.b_times):.2f}" if r.b_times else "-"
        lines.append(
            f"| {r.name} | {len(r.a_times)} | {len(r.b_times)} | "
            f"{'equal' if r.counts_equal else 'DIFFER'} | {_times(r.diffs)} | {pose_mean} | "
            f"{lap_mean} |"
        )
    return "\n".join(lines)
