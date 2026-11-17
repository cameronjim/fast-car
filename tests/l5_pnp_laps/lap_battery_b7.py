"""Battery B7 of the v0.5 lap pre-registration (docs/notes/preregistration-pnp-v0.5-laps.md section 2
and 3): run loading and the decisions D7-S0 and D7-S1 as pure functions.

Every threshold is read from `evaluation/prereg/pnp-v0.5-laps.yaml` (block `lap_battery_b7`), whose
SHA-256 is a constant of this file and is checked on every load. D7-S1 reads D3-S1's contacts
threshold (0) from `pnp-v0.3.yaml` through `lap_battery_b4.load_constants`, which checks that file's
hash too: the pre-registration says D7-S1 is "the four items of D3-S1 on conditions c and b,
unchanged". A threshold that cannot be applied as written is reported, never edited.

The decisions call the D3 machinery of `lap_battery_b4` unchanged: D7-S0 is `d3_s0` restricted to
conditions a (S0a) and c (S0b), built from the same helpers because `d3_s0` itself also asks for
conditions d and e, which B7 does not have; D7-S1 is `d3_s1`. Lap 1 is inside every item with no
allowance, and truth is the registered truth of the runner (`gt_lap_times_registered_s`): a run
whose truth is invalid (`truth_valid` false) is a run that failed to execute.
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import lap_battery_b4 as b4
import yaml
from lap_battery import Decision, RunSummary

REPO_ROOT = Path(__file__).resolve().parents[2]
PREREG_YAML = REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.5-laps.yaml"
PREREG_NOTE = REPO_ROOT / "docs" / "notes" / "preregistration-pnp-v0.5-laps.md"
#: SHA-256 of `pnp-v0.5-laps.yaml` as committed with the pre-registration (commit 1 of brief 48).
#: Checked on every load and written into every run and the report.
REGISTERED_YAML_SHA256 = "35949c7d86dbdd3c5a3dde11eb0551465ea893b5f26e78dc7de0f1a0890428de"
B7_CONDITIONS = ("a", "b", "c")
B7_SEEDS = (51, 52, 53)
B7_DIRECTIONS = ("ccw", "cw")

PreregError = b4.PreregError
MissingPairError = b4.MissingPairError
RunFileError = b4.RunFileError


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


@dataclass(frozen=True)
class Constants:
    """The `lap_battery_b7` block of the yaml, the SHA-256 it was read under and the one number
    D7-S1 takes from D3 (`max_contacts`). Has the interface of `lap_battery_b4.Constants` that
    the D3 functions use."""

    yaml_sha256: str
    raw: dict[str, Any]
    max_contacts: int

    @property
    def battery_cases(self) -> dict[str, dict]:
        return dict(self.raw["cases"])

    @property
    def battery_seeds(self) -> tuple[int, ...]:
        return tuple(int(s) for s in self.raw["seeds"])

    @property
    def directions(self) -> tuple[str, ...]:
        return tuple(self.raw["directions"])

    @property
    def laps(self) -> int:
        return int(self.raw["laps"])

    @property
    def lap1_cap_mps(self) -> float:
        return float(self.raw["lap1_cap_mps"])

    @property
    def conditions(self) -> dict[str, dict]:
        return dict(self.raw["conditions"])

    @property
    def drop_rule(self) -> dict[str, Any]:
        return dict(self.raw["drop_rule"])

    @property
    def validity(self) -> dict[str, Any]:
        return dict(self.raw["validity"])

    @property
    def th(self) -> dict[str, float]:
        """The thresholds under the names the D3 functions read."""
        t = self.raw["thresholds"]
        return {
            "lap_time_tolerance_s": float(t["s0_tolerance_s"]),
            "s0b_min_share_within_tolerance": float(t["s0b_min_share_within_tolerance"]),
            "s0b_max_single_error_s": float(t["s0b_max_single_error_s"]),
            "max_contacts": float(self.max_contacts),
            "required_complete_of_24": float(t["s1_min_complete"]["of_24"]),
            "required_complete_of_18": float(t["s1_min_complete"]["of_18"]),
            "s1_reach_max_ratio_to_b": float(t["s1_reach_ratio"]),
            "s1_max_median_r": float(t["s1_median_r_max"]),
        }

    def required_complete(self, n_runs: int) -> int:
        """22 of 24, or 16 of 18 with a case dropped. Nothing else is registered."""
        if n_runs == 24:
            return int(self.th["required_complete_of_24"])
        if n_runs == 18:
            return int(self.th["required_complete_of_18"])
        raise PreregError(f"{n_runs} runs in a condition: the pre-registration has 24 or 18")


def _check_consistent(block: dict[str, Any]) -> None:
    """The yaml says some numbers twice; the D3 functions use one. A disagreement is a
    pre-registration that cannot be applied as written."""
    t = block["thresholds"]
    if t["s1_reach_min_runs"] != t["s1_min_complete"]:
        raise PreregError("s1_reach_min_runs differs from s1_min_complete: D3-S1 uses one number")
    if int(t["s1_complete_laps"]) != b4.LAPS_TO_COMPLETE:
        raise PreregError(f"s1_complete_laps is not {b4.LAPS_TO_COMPLETE}")
    if tuple(int(k) for k in t["late_laps"]) != b4.LATE_LAPS:
        raise PreregError(f"late_laps is not {list(b4.LATE_LAPS)}")
    if tuple(int(s) for s in block["seeds"]) != B7_SEEDS:
        raise PreregError(f"seeds are not {list(B7_SEEDS)}")
    if tuple(block["directions"]) != B7_DIRECTIONS:
        raise PreregError(f"directions are not {list(B7_DIRECTIONS)}")


def load_constants(
    yaml_path: Path = PREREG_YAML, expected_sha: str = REGISTERED_YAML_SHA256
) -> Constants:
    """The yaml's `lap_battery_b7` block, refused when its SHA-256 is not the registered one."""
    actual = sha256_file(yaml_path)
    if actual != expected_sha:
        raise PreregError(
            f"{yaml_path} has SHA-256 {actual}, the registered value is {expected_sha}: refusing"
        )
    block = yaml.safe_load(yaml_path.read_text())["lap_battery_b7"]
    _check_consistent(block)
    d3 = b4.load_constants()  # checks the v0.3 file's hash against its pre-registration
    return Constants(actual, block, int(d3.th["max_contacts"]))


# -- runs -------------------------------------------------------------------------------------------


def summarise_run(result: dict, condition: str, case: str, direction: str, seed: int) -> RunSummary:
    """A RunSummary on the registered truth. A result without it (a run of an earlier runner) or
    without `truth_valid` is refused: B7 is scored by the runner of brief 48 only."""
    name = f"{condition}_{case}_{direction}_{seed}"
    if "gt_lap_times_registered_s" not in result or "truth_valid" not in result:
        raise RunFileError(
            f"{name}: no gt_lap_times_registered_s and truth_valid "
            "(the runner of brief 48 is required)"
        )
    return b4.summarise_b4_run(result, condition, case, direction, seed)


def load_runs(results_dir: Path, mode: str = "battery") -> list[RunSummary]:
    """Every `<condition>_<case>_<direction>_<seed>.json` of a B7 directory (condition a, b, c).
    Each must carry the `battery_b7` block with the mode asked for."""
    out = []
    for path in sorted(results_dir.glob("*.json")):
        parts = path.stem.split("_")
        if len(parts) != 4 or parts[0] not in B7_CONDITIONS:
            continue
        condition, case, direction, seed = parts
        data = json.loads(path.read_text())
        meta = data.get("battery_b7")
        if not isinstance(meta, dict) or meta.get("mode") != mode:
            raise RunFileError(f"{path.name}: not a B7 {mode} run file")
        out.append(summarise_run(data, condition, case, direction, int(seed)))
    return out


# -- D7-S0 and D7-S1 ----------------------------------------------------------------------------------


def d7_s0(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """S0a on condition a and S0b on condition c: the items and thresholds of `d3_s0` for those
    two conditions (pre-registration section 3). Lap 1 is inside both, with no allowance."""
    const = const or load_constants()
    b4._check_unique(runs)
    tol = float(const.th["lap_time_tolerance_s"])
    min_share = float(const.th["s0b_min_share_within_tolerance"])
    max_single = float(const.th["s0b_max_single_error_s"])
    items: dict[str, tuple[bool | None, str]] = {}

    a = b4._need(runs, "a", "S0a")
    st = b4._lap_count_and_time_stats(a, tol)
    items["S0a: system lap count equals ground truth in every condition (a) run"] = (
        not st["count_bad"],
        f"{len(a) - len(st['count_bad'])} of {len(a)} runs agree {st['count_bad'][:6]}",
    )
    items[f"S0a: every system lap time within {tol} s of ground truth"] = (
        not st["outside"],
        f"{len(st['outside'])} of {st['total']} laps outside {[t for t, _ in st['outside']][:8]}",
    )
    items["S0a: no condition (a) run failed to execute"] = (
        not st["errors"],
        f"{len(st['errors'])} errors {st['errors'][:4]}",
    )
    c = b4._need(runs, "c", "S0b(c)")
    st = b4._lap_count_and_time_stats(c, tol)
    share = st["within"] / st["total"] if st["total"] else 0.0
    items["S0b(c): lap count equal in every run"] = (
        not st["count_bad"],
        f"{len(c) - len(st['count_bad'])} of {len(c)} runs agree {st['count_bad'][:6]}",
    )
    items[f"S0b(c): at least {min_share:.0%} of laps within {tol} s"] = (
        share >= min_share,
        (
            f"{st['within']} of {st['total']} laps within ({share:.4f}); outside: "
            f"{[t for t, _ in st['outside']][:8]}"
        ),
    )
    items[f"S0b(c): no lap off by more than {max_single} s"] = (
        st["max_error"] <= max_single,
        f"largest error {st['max_error']:.2f} s",
    )
    items["S0b(c): no run failed to execute"] = (
        not st["errors"],
        f"{len(st['errors'])} errors {st['errors'][:4]}",
    )
    return b4._decision("D7-S0", items)


def d7_s1(runs: list[RunSummary], const: Constants | None = None) -> Decision:
    """D3-S1's four items on condition c against its paired control b, unchanged."""
    const = const or load_constants()
    return dataclasses.replace(b4.d3_s1(runs, const), name="D7-S1")  # type: ignore[arg-type]


# -- reported without a threshold ---------------------------------------------------------------------


def lap1_rows(results: dict[str, dict]) -> list[dict[str, Any]]:
    """One row per run: the system's lap 1, the registered truth's, the difference, the launch
    dwell the lap node reported and whether the truth is valid. `results` maps run name to its
    JSON."""
    rows = []
    for name, data in sorted(results.items()):
        events = data.get("system_lap_events") or []
        truth = data.get("gt_lap_times_registered_s") or []
        system = float(events[0]["lap_time_s"]) if events else math.nan
        gt = float(truth[0]) if truth else math.nan
        status = data.get("lap_status") or {}
        dwell = status.get("launch_dwell_s")
        rows.append(
            {
                "run": name,
                "system_lap1_s": system,
                "truth_lap1_s": gt,
                "difference_s": system - gt,
                "launch_dwell_s": math.nan if dwell is None else float(dwell),
                "truth_valid": data.get("truth_valid"),
            }
        )
    return rows


def lap1_table(rows: list[dict[str, Any]]) -> str:
    lines = [
        "| run | system lap 1 s | truth lap 1 s | difference s | launch dwell s | truth valid |",
        "|---|---|---|---|---|---|",
    ]
    for r in rows:
        lines.append(
            f"| {r['run']} | {r['system_lap1_s']:.2f} | {r['truth_lap1_s']:.2f} "
            f"| {r['difference_s']:+.2f} | {r['launch_dwell_s']:.2f} | {r['truth_valid']} |"
        )
    finite = [abs(r["difference_s"]) for r in rows if math.isfinite(r["difference_s"])]
    if finite:
        lines.append("")
        lines.append(
            f"lap 1, system minus truth: median |difference| {statistics.median(finite):.3f} s, "
            f"largest {max(finite):.3f} s over {len(finite)} runs"
        )
    return "\n".join(lines)


def missed_flags_per_condition(flags: dict[str, int]) -> list[str]:
    """`flags` maps run name to its number of laps flagged missed gate."""
    out = []
    for cond in B7_CONDITIONS:
        n_runs = sum(1 for k in flags if k.startswith(f"{cond}_"))
        total = sum(v for k, v in flags.items() if k.startswith(f"{cond}_"))
        runs_with = sum(1 for k, v in flags.items() if k.startswith(f"{cond}_") and v)
        out.append(
            f"condition {cond}: {total} laps flagged missed gate in {runs_with} of {n_runs} runs"
        )
    return out
