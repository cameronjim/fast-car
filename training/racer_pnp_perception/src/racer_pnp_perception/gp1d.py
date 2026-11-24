"""GP1d and GP1d-K: `fused-v2` (the tracer's corridor, the net's centre, half widths by ray cast)
against the geometric tracer, decided as written in docs/notes/preregistration-pnp-v0.6-perception-
fused.md (constants `evaluation/prereg/pnp-v0.6-perception-fused.yaml`, brief 55, owner decisions 27
and 28).

This module holds the decision rules and the data-role guards; the commands are

    python -m racer_pnp_perception.tune_v2 ...     # development tuning, before the freeze
    python -m racer_pnp_perception.gp1d_score ...  # the one pass on H14/A14 and H22/A22

Two held-out sets, each judged by itself: 14 (the perception `heldout` segments of 2026-11-14) and
22 (runs R0 to R4 of the 2026-11-22 bag, whole). Items 1 to 5 and GP1d-K are per set; items 6 to 8
are global. Items are the ones of GP1c with the thresholds of `gp1` and `gp1c`, unchanged; the yaml
is read and compared with those constants (`load_prereg`).

Outcome, in this order (the brief lists the outcome names; the order is fixed here, before any
number):

1. a set with fewer than 300 labelled scans: "undecided";
2. item 2 or 3 failed on a set: "no better than geometric in the layout";
3. items 1 to 8 and K pass on both sets: "pass";
4. items 1 to 8 pass on both sets, K fails on a set: "better centre, corridor not kept";
5. everything passes on exactly one set and the other set fails only items other than 2 and 3:
   "not shown across sessions";
6. otherwise (2 and 3 pass on both sets, another item fails): "better where it trained, not shown
   elsewhere".

Geometric stays deployed in every case. Every report says: M-A2 measured under 3 percent of lap time
for a perfect track in sim, so a pass claims a more accurate centre, not a faster lap.
"""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Any

import yaml
from racer_track_perception.core import BACKEND_GEOMETRIC, BACKEND_NET
from racer_track_perception.vehicle import find_repo_root

from racer_pnp_perception import gp1, gp1c, slope_select

PREREG_SHA256 = "22d65de750ef56bcc90e33cc0a7a5a85eb69b5291d606ebd30fd452ed1e3e0b5"
PREREG_RELATIVE_PATH = Path("evaluation") / "prereg" / "pnp-v0.6-perception-fused.yaml"
DEV_BENEFIT_MAX_RATIO = 0.95

SET_14 = "14"
SET_22 = "22"

OUTCOME_UNDECIDED = "undecided: fewer than 300 labelled scans on a held-out set, geometric stays"
OUTCOME_NO_BETTER = "no better than geometric in the layout (item 2 or 3 failed), geometric stays"
OUTCOME_PASS = (
    "pass: candidate backend in the layout family it trained on, on two fresh sessions "
    "(geometric stays deployed until the on-car rule)"
)
OUTCOME_K_NOT_KEPT = "better centre, corridor not kept (GP1d passed, GP1d-K failed)"
OUTCOME_ONE_SET = "not shown across sessions (one set passed, the other did not), geometric stays"
OUTCOME_NOT_ELSEWHERE = (
    "better where it trained, not shown elsewhere (items 2 and 3 passed, another item failed), "
    "geometric stays deployed"
)
NOT_ATTEMPTED = "GP1d not attempted"
ALWAYS_SAID = (
    "M-A2 measured under 3 percent of lap time for a perfect track in sim. A pass claims a more "
    "accurate centre, not a faster lap. GP1b stays FAIL and GP1c stays not attempted."
)

# ---------------------------------------------------------------------------------------------
# The constants file
# ---------------------------------------------------------------------------------------------


class PreregMismatch(ValueError):
    """The constants file is not the registered one, or differs from the coded thresholds."""


def load_prereg(path: Path | None = None, expected_sha256: str = PREREG_SHA256) -> dict[str, Any]:
    """The parsed constants file. Refuses a file whose SHA-256 is not the registered one, or whose
    thresholds differ from the constants of `gp1`, `gp1c` and `slope_select` that decide."""
    p = Path(path) if path is not None else find_repo_root() / PREREG_RELATIVE_PATH
    data = p.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if digest != expected_sha256:
        raise PreregMismatch(f"{p}: SHA-256 {digest} is not the registered {expected_sha256}")
    doc = yaml.safe_load(data.decode("utf-8"))
    check_thresholds(doc)
    return doc


def check_thresholds(doc: dict[str, Any]) -> None:
    """Compare the yaml with the coded constants; raise PreregMismatch on any difference."""
    g, k, t = doc["gate"], doc["gate_k"], doc["tuning"]
    wrong: list[str] = []

    def same(name: str, got: object, want: object) -> None:
        if got != want:
            wrong.append(f"{name}: yaml {got!r}, code {want!r}")

    same("item_1", g["item_1_min_labelled_scans"], gp1.MIN_H_RUN_LABELS)
    same("item_2 ratio", g["item_2"]["max_ec_ratio"], gp1.ITEM2_MAX_RATIO)
    same("item_2 ci", g["item_2"]["bootstrap_upper_below"], gp1.ITEM2_CI_UPPER_BELOW)
    same("item_3 margin", gp1.ITEM3_V_MARGIN, 0.0)
    same("item_4 min", g["item_4"]["min_labelled_scans"], gp1.CELL_MIN_LABELS)
    same("item_4 ratio", g["item_4"]["max_ec_ratio"], gp1.CELL_MAX_RATIO)
    i5 = g["item_5"]
    same(
        "item_5 intrusion",
        i5["max_mean_intrusion"],
        f"max({gp1.ITEM5_INTRUSION_FLOOR}, geometric + {gp1.ITEM5_INTRUSION_MARGIN})",
    )
    same("item_5 residual", i5["max_boundary_residual_median_m"], gp1.ITEM5_MAX_RESIDUAL_MEDIAN_M)
    same("item_5 deficit", i5["max_clearance_deficit_p90_m"], gp1.ITEM5_MAX_DEFICIT_P90_M)
    same("item_5 c95", i5["min_c95"], f"geometric minus {gp1.ITEM5_C95_MARGIN}")
    lat = g["item_7_latency"]
    same("item_7 scans", lat["scans"], 2000)
    same("item_7 threads", lat["threads"], 1)
    same("item_7 p99", lat["max_p99_ms"], gp1c.LATENCY_P99_MAX_MS)
    can = g["item_8_canary"]
    same("item_8 contacts", can["contacts_allowed"], gp1c.CANARY_MAX_WALL_CONTACTS)
    same("item_8 ratio", can["max_median_lap_ratio_to_geometric"], gp1c.CANARY_MAX_LAP_RATIO)
    same("gate_k mean loss", k["max_mean_loss"], gp1c.K_MAX_MEAN)
    same("gate_k mean excess", k["max_mean_excess"], gp1c.K_MAX_MEAN)
    same("gate_k p99 loss", k["max_p99_loss"], gp1c.K_MAX_P99)
    same("gate_k p99 excess", k["max_p99_excess"], gp1c.K_MAX_P99)
    m = t["development_margin"]
    same("margin mean loss", m["max_mean_loss"], slope_select.MARGIN_MAX_MEAN)
    same("margin mean excess", m["max_mean_excess"], slope_select.MARGIN_MAX_MEAN)
    same("margin p99 loss", m["max_p99_loss"], slope_select.MARGIN_MAX_P99)
    same("margin p99 excess", m["max_p99_excess"], slope_select.MARGIN_MAX_P99)
    same("development benefit", t["development_benefit"]["max_ratio_on_every_labelled_development_set"], DEV_BENEFIT_MAX_RATIO)
    if wrong:
        raise PreregMismatch("the constants file differs from the coded thresholds: " + "; ".join(wrong))


# ---------------------------------------------------------------------------------------------
# Data roles
# ---------------------------------------------------------------------------------------------

DEV_2026_11_16 = (
    "2026-11-16T02-18-54_car_teleop",
    "2026-11-16T02-29-27_car_teleop",
    "2026-11-16T22-38-47_car_teleop",
    "2026-11-16T22-45-39_car_teleop",
    "2026-11-16T23-04-09_car_teleop",
)
HELDOUT_22_BAG = "2026-11-23T03-50-32_car_teleop"  # recorded the evening of 2026-11-22
FRESH_14_PREFIX = "2026-11-14"
LATEST_OPENABLE_BAG = HELDOUT_22_BAG  # every bag named later than this is opened by nobody


class DataRoleRefused(ValueError):
    """A loader was asked for data the registered roles do not allow."""


def assert_bag_allowed(name: str, purpose: str) -> None:
    """The role guard. `purpose` is `development` (free to tune on) or `heldout` (the one pass).

    Always refused: a bag named after the 2026-11-22 bag, and a `reserve` request (the segment
    loaders refuse that themselves, `split_v04`). Development refuses every 2026-11-14 bag and
    the 2026-11-22 bag; the held-out pass reads only those two sessions."""
    if name > LATEST_OPENABLE_BAG and not name.startswith(LATEST_OPENABLE_BAG):
        raise DataRoleRefused(f"{name} was recorded after 2026-11-22: opened by nobody")
    if purpose == "development":
        if name.startswith(FRESH_14_PREFIX) or name.startswith(HELDOUT_22_BAG):
            raise DataRoleRefused(f"{name} is a held-out set, not development data")
    elif purpose == "heldout":
        if not (name.startswith(FRESH_14_PREFIX) or name == HELDOUT_22_BAG):
            raise DataRoleRefused(f"{name} is not one of the two held-out sessions")
    else:
        raise DataRoleRefused(f"unknown purpose {purpose!r}")


def assert_role_allowed(role: str) -> None:
    if role == "reserve":
        raise DataRoleRefused("perception `reserve` segments are opened by nobody")


# ---------------------------------------------------------------------------------------------
# Decisions
# ---------------------------------------------------------------------------------------------


def decide_gp1d(
    label: str,
    h: dict[str, gp1.Labelled],
    a: dict[str, gp1.LabelFree],
    ratio_ci_hi: float,
    cells: list[gp1.Cell],
) -> tuple[list[gp1.Item], list[int]]:
    """Items 1 to 5 on one held-out set. `h` and `a` map `net` (the candidate, fused-v2) and
    `geometric` to their numbers on H and A of the set; `cells` are the cells of H. Returns the
    items and the failed numbers. An item that could not be computed does not pass."""
    net, geo = BACKEND_NET, BACKEND_GEOMETRIC
    items: list[gp1.Item] = []
    items.append(
        gp1.Item(
            1,
            f"H{label} has at least 300 labelled scans",
            {"labelled_scans": h[net].scans},
            {"min": gp1.MIN_H_RUN_LABELS},
            h[net].scans >= gp1.MIN_H_RUN_LABELS,
        )
    )
    ratio = h[net].e_c_m / h[geo].e_c_m if h[geo].e_c_m > 0 else math.nan
    items.append(
        gp1.Item(
            2,
            f"centreline accuracy on H{label}",
            {
                "e_c_fused_m": h[net].e_c_m,
                "e_c_geometric_m": h[geo].e_c_m,
                "ratio": ratio,
                "ratio_ci_upper": ratio_ci_hi,
            },
            {"ratio_max": gp1.ITEM2_MAX_RATIO, "ci_upper_below": gp1.ITEM2_CI_UPPER_BELOW},
            gp1._le(ratio, gp1.ITEM2_MAX_RATIO) and bool(ratio_ci_hi < gp1.ITEM2_CI_UPPER_BELOW),
        )
    )
    items.append(
        gp1.Item(
            3,
            f"range on H{label}",
            {"v_fused": h[net].v, "v_geometric": h[geo].v},
            {"v_fused_min": h[geo].v},
            gp1._ge(h[net].v, h[geo].v),
        )
    )
    judged = [c for c in cells if c.judged]
    bad = [
        f"{c.set}|{c.direction}|{c.speed_bin}"
        for c in judged
        if not gp1._le(c.ratio, gp1.CELL_MAX_RATIO)
    ]
    items.append(
        gp1.Item(
            4,
            f"per cell on H{label}",
            {
                "cells_judged": len(judged),
                "cells_not_judged": len(cells) - len(judged),
                "worst_ratio": max((c.ratio for c in judged), default=math.nan),
                "failing_cells": bad,
            },
            {"ratio_max": gp1.CELL_MAX_RATIO, "min_labelled_scans": gp1.CELL_MIN_LABELS},
            not bad,
            "" if judged else "no cell has 40 labelled scans: nothing judged",
        )
    )
    ok, value, thr = gp1.label_free_ok(a[net], a[geo])
    items.append(
        gp1.Item(
            5, f"label-free rule on A{label}", {**value, "moving_scans": a[net].scans}, thr, ok
        )
    )
    return items, [i.number for i in items if not i.passed]


def decide_global(
    dev_passed: bool | None,
    dev_note: str,
    latency_p99_ms: float | None,
    canary: list[gp1c.CanaryCase] | None,
) -> tuple[list[gp1.Item], list[int]]:
    """Items 6 to 8. `dev_passed` is the v0.3 section 3.3 rule on the development sets (None: not
    measured). An item that was not measured does not pass."""
    items: list[gp1.Item] = []
    if dev_passed is None:
        items.append(
            gp1.Item(6, "v0.3 3.3 items 1 to 5, development sets", {}, {}, False, gp1c.NOT_MEASURED)
        )
    else:
        items.append(
            gp1.Item(
                6,
                "v0.3 3.3 items 1 to 5, development sets",
                {"all_items_pass": dev_passed},
                {"all_items_pass": True},
                dev_passed,
                ("development data, reported and not evidence; " + dev_note).strip("; "),
            )
        )
    if latency_p99_ms is None:
        items.append(gp1.Item(7, "latency p99, 2000 real scans", {}, {}, False, gp1c.NOT_MEASURED))
    else:
        items.append(
            gp1.Item(
                7,
                "latency p99, 2000 real scans",
                {"p99_ms": latency_p99_ms},
                {"p99_max_ms": gp1c.LATENCY_P99_MAX_MS},
                bool(latency_p99_ms < gp1c.LATENCY_P99_MAX_MS),
            )
        )
    if not canary:
        items.append(
            gp1.Item(8, "L5 PnP canary with the fused backend", {}, {}, False, gp1c.NOT_MEASURED)
        )
    else:
        good = all(
            c.completed
            and c.wall_contacts <= gp1c.CANARY_MAX_WALL_CONTACTS
            and gp1._le(c.ratio, gp1c.CANARY_MAX_LAP_RATIO)
            for c in canary
        )
        items.append(
            gp1.Item(
                8,
                "L5 PnP canary with the fused backend",
                {
                    "cases": {
                        c.name: {
                            "completed": c.completed,
                            "wall_contacts": c.wall_contacts,
                            "lap_ratio": c.ratio,
                        }
                        for c in canary
                    }
                },
                {
                    "wall_contacts_max": gp1c.CANARY_MAX_WALL_CONTACTS,
                    "lap_ratio_max": gp1c.CANARY_MAX_LAP_RATIO,
                },
                good,
            )
        )
    return items, [i.number for i in items if not i.passed]


def decide_gp1d_k(
    label: str,
    losses: list[float],
    excesses: list[float],
    worst: dict[str, Any] | None = None,
) -> tuple[list[gp1.Item], bool]:
    """GP1d-K on A of one set: the four items of GP1c-K (same thresholds), named for the set."""
    items, ok = gp1c.decide_gp1c_k(losses, excesses, worst)
    for i in items:
        i.name = i.name.replace("A14", f"A{label}")
    return items, ok


def combine_outcomes(
    set_failed: dict[str, list[int]],
    global_failed: list[int],
    k_passed: dict[str, bool],
) -> str:
    """The named outcome (module docstring, in the order given there). `set_failed[label]` are the
    failed numbers among items 1 to 5 of that set; the global items 6 to 8 count against both."""
    labels = sorted(set_failed)
    if any(1 in set_failed[s] for s in labels):
        return OUTCOME_UNDECIDED
    if any(2 in set_failed[s] or 3 in set_failed[s] for s in labels):
        return OUTCOME_NO_BETTER
    full = {s: not set_failed[s] and not global_failed and k_passed[s] for s in labels}
    if all(full.values()):
        return OUTCOME_PASS
    gp1d_ok = {s: not set_failed[s] and not global_failed for s in labels}
    if all(gp1d_ok.values()):
        return OUTCOME_K_NOT_KEPT
    if sum(full.values()) == 1:
        return OUTCOME_ONE_SET
    return OUTCOME_NOT_ELSEWHERE


def meets_benefit(ratios: dict[str, float]) -> bool:
    """The development benefit: E_c(fused) / E_c(geometric) at most 0.95 on every labelled
    development set (a NaN ratio fails; an empty mapping fails)."""
    return bool(ratios) and all(gp1._le(r, DEV_BENEFIT_MAX_RATIO) for r in ratios.values())
