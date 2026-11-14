"""The JSON of a report and its markdown tables.

The JSON carries SCHEMA_VERSION and no wall-clock time, so the same bag always gives the same
file. It is what the session note cites.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from racer_eval.cli import HAND_COUNT

from pnp_session.constants import BASELINE_BAG, GAP_LIMITS_S, SCHEMA_VERSION
from pnp_session.gaps import limit_key

LIMIT_KEYS = [limit_key(x) for x in GAP_LIMITS_S]


def baseline_check(report: dict[str, Any]) -> dict[str, Any]:
    """The pre-registered hand count of v0.2 section 4.2 against this report's pooled numbers."""
    safety = report["pooled"]["safety"]
    counts = safety["engage_counts"]
    measured = {
        "obstacle": counts.get("ttc", 0) + counts.get("ttc_reverse", 0),
        "watchdog": counts.get("watchdog", 0),
        "rate_limit": counts.get("rate_limit", 0),
        "starts_with_event": safety["starts_with_event"],
        "starts": safety["starts"],
    }
    expected = {k: HAND_COUNT[k] for k in measured}
    return {
        "section": "v0.2 4.2 baseline (2026-11-13 R0 to R8)",
        "expected": expected,
        "measured": measured,
        "matches": measured == expected,
        "note": (
            "the pre-registration says one more rate_limit lands within 0.05 s of a run's last "
            "status and is outside the window"
        ),
    }


def assemble(
    bag_name: str,
    timing: dict[str, Any],
    limiter: dict[str, Any],
    check_baseline: bool,
) -> dict[str, Any]:
    out: dict[str, Any] = {
        "schema": SCHEMA_VERSION,
        "report": "timing",
        "bag": bag_name,
        "preregistration": "docs/notes/preregistration-pnp-v0.3.md section 5 (M-T)",
        "gap_limits_s": list(GAP_LIMITS_S),
        "percentile_method": "linear",
        **timing,
        "rate_limiter_whatif": limiter,
    }
    if check_baseline:
        out["baseline_check"] = baseline_check(out)
    return out


def _ms(x: float | None) -> str:
    return "-" if x is None else f"{x * 1000.0:.1f}"


def _num(x: object) -> object:
    return f"{x:.3f}" if isinstance(x, float) else x


def _row(cells: Sequence[object]) -> str:
    return "| " + " | ".join(str(c) for c in cells) + " |"


def _table(header: Sequence[str], rows: Sequence[Sequence[object]]) -> list[str]:
    return [_row(header), _row(["---"] * len(header)), *[_row(r) for r in rows], ""]


def _entries(report: dict[str, Any]) -> list[dict[str, Any]]:
    runs = [r for r in report["runs"] if r.get("commanded")]
    return [*runs, report["pooled"]]


def _gap_table(report: dict[str, Any], clock: str) -> list[str]:
    header = ["run", "window s", "msgs", "p50 ms", "p99 ms", "max ms", *[f">{k} ms" for k in LIMIT_KEYS]]
    rows = []
    for e in _entries(report):
        g = e["drive_raw"][clock]
        rows.append(
            [
                e["run_id"],
                f"{e['duration_s']:.1f}",
                g["n_messages"],
                _ms(g["p50_s"]),
                _ms(g["p99_s"]),
                _ms(g["max_s"]),
                *[g["above"][k] for k in LIMIT_KEYS],
            ]
        )
    return _table(header, rows)


def _planner_table(report: dict[str, Any]) -> list[str]:
    header = [
        "run", "ticks", "solve p50 ms", "solve p99 ms", "solve max ms", "tick gap p50 ms",
        "tick gap p99 ms", "tick gap max ms", "bunched", "FAULT ticks", "STARTING ticks",
        "STARTING ms",
    ]  # fmt: skip
    rows = []
    for e in _entries(report):
        p = e["planner"]
        s, g = p["solve_time_s"], p["tick_gap_stamp"]
        if "starting" in e:
            st_ticks = e["starting"]["ticks"]
            st_s = _ms(e["starting"]["duration_s"])
        else:
            st_ticks, st_s = "", ""
        rows.append(
            [
                e["run_id"], p["ticks"], _ms(s["p50"]), _ms(s["p99"]), _ms(s["max"]),
                _ms(g["p50_s"]), _ms(g["p99_s"]), _ms(g["max_s"]), p["bunched_ticks"],
                p["fault_ticks"], st_ticks, st_s,
            ]
        )  # fmt: skip
    return _table(header, rows)


def _tick_gap_counts(report: dict[str, Any]) -> list[str]:
    header = ["run", "clock", *[f">{k} ms" for k in LIMIT_KEYS]]
    rows = []
    for e in _entries(report):
        for clock, key in (("stamp", "tick_gap_stamp"), ("recv", "tick_gap_recv")):
            rows.append([e["run_id"], clock, *[e["planner"][key]["above"][k] for k in LIMIT_KEYS]])
    return _table(header, rows)


def _safety_table(report: dict[str, Any]) -> list[str]:
    header = [
        "run", "watchdog", "rate_limit", "ttc+reverse", "other", "watchdog /min", "rate_limit /min",
        "first 2 s: watchdog", "first 2 s: rate_limit", "start event",
    ]  # fmt: skip
    rows = []
    for e in _entries(report):
        s = e["safety"]
        c, m, f = s["engage_counts"], s["engage_per_minute"], s["first_2s_counts"]
        known = c.get("watchdog", 0) + c.get("rate_limit", 0) + c.get("ttc", 0) + c.get("ttc_reverse", 0)
        pm = lambda k: "-" if m.get(k) is None else f"{m[k]:.2f}"  # noqa: E731
        if e["run_id"] == "pooled":
            start = f"{s['starts_with_event']}/{s['starts']}"
        else:
            start = "yes" if s["start_event"] else "no"
        rows.append(
            [
                e["run_id"], c.get("watchdog", 0), c.get("rate_limit", 0),
                c.get("ttc", 0) + c.get("ttc_reverse", 0), sum(c.values()) - known,
                pm("watchdog"), pm("rate_limit"), f.get("watchdog", 0), f.get("rate_limit", 0), start,
            ]
        )  # fmt: skip
    return _table(header, rows)


def _detail_lines(report: dict[str, Any]) -> list[str]:
    d = report["pooled"]["safety"]["details"]
    rl, wd = d["rate_limit"], d["watchdog"]
    lines = [
        "rate_limit, the limiter's own dt parsed from the event detail (pooled): "
        f"n {rl['dt_s']['n']}, min {_ms(rl['dt_s']['min'])} ms, p50 {_ms(rl['dt_s']['p50'])} ms, "
        f"max {_ms(rl['dt_s']['max'])} ms, under half a tick (10 ms) {rl['dt_below_half_tick']}, "
        f"unparsed {rl['n_unparsed']}",
        "watchdog, the age parsed from the event detail (pooled): "
        f"n {wd['age_s']['n']}, min {_ms(wd['age_s']['min'])} ms, p50 {_ms(wd['age_s']['p50'])} ms, "
        f"max {_ms(wd['age_s']['max'])} ms, timeouts {wd['timeouts_s']}, unparsed {wd['n_unparsed']}",
    ]
    for s in rl["unparsed_shown"] + wd["unparsed_shown"]:
        lines.append(f"unparsed detail shown as recorded: {s!r}")
    return [*lines, ""]


def _watchdog_whatif(report: dict[str, Any]) -> list[str]:
    w = report["watchdog_whatif"]
    out = _table(
        ["limit ms", "receive gaps above it (pooled)"],
        [[k, w["gaps_above_limit"][k]] for k in LIMIT_KEYS],
    )
    out += [
        w["note"],
        "",
        f"Agreement of the {int(w['current_limit_s'] * 1000)} ms row with the recorded `watchdog` "
        f"engagements (tolerance {w['match_tolerance_s'] * 1000:.0f} ms): "
        f"{w['gaps_above_current_limit']} gaps above the limit, "
        f"{w['recorded_watchdog_engagements']} recorded engagements, {w['matched']} matched, "
        f"{w['recorded_without_gap']} recorded without a gap, "
        f"{w['gaps_without_recorded_event']} gaps without a recorded engagement.",
        "",
    ]
    return out


def _limiter(report: dict[str, Any]) -> list[str]:
    w = report["rate_limiter_whatif"]
    if w["status"] != "run":
        return [f"NOT RUN: {w['reason']}", ""]
    rows = []
    labels = {
        "recorded": "recorded dt, each cycle from the recorded previous output",
        "nominal": "nominal dt, each cycle from the recorded previous output",
        "recorded_free_running": "recorded dt, free-running chain",
        "nominal_free_running": "nominal dt, free-running chain",
    }
    for mode, label in labels.items():
        p = w["modes"][mode]["pooled"]
        rows.append([label, p["clip_cycles"], p["engage_rate_limit"], p["engage_watchdog"]])
    out = _table(["replay", "clipped cycles", "rate_limit engagements", "watchdog engagements"], rows)
    rec = report["pooled"]["safety"]["engage_counts"].get("rate_limit", 0)
    out.append(f"Recorded `rate_limit` engagements in the windows: {rec}. In the table the counts are "
               "of the replay inside the counting windows.")
    out.append("")
    out += ["Reproduction with the recorded dt (rows without a suffix: each cycle judged from the recorded previous output; `free_running`: the replay's own chain):", ""]
    repro = w["reproduction_recorded_dt"]
    rrows = []
    for key in ("rate_limit", "rate_limit_free_running", "watchdog", "watchdog_free_running"):
        r = repro[key]
        diff = r["dt_detail_max_abs_diff_s"]
        rrows.append(
            [key, r["recorded"], r["replayed"], r["matched"], r["missed"], r["extra"],
             "-" if diff is None else f"{diff * 1000:.3f} ms ({r['dt_detail_compared']} pairs)"]
        )  # fmt: skip
    out += _table(["events", "recorded", "replayed", "matched", "missed", "extra", "dt detail max diff"], rrows)
    out += [f"Not modelled: {w['not_modelled']}.", ""]
    return out


def _acceptance(report: dict[str, Any]) -> list[str]:
    a = report["acceptance_4_2"]
    if a["status"] != "computed":
        return [f"NOT COMPUTED: {a['reason']}", ""]
    rows = [
        [i["item_id"], "PASS" if i["passed"] else "FAIL", i["description"], _num(i["measured"]),
         i["comparator"], i["threshold"], i["note"]]
        for i in a["items"]
    ]  # fmt: skip
    head = f"Outcome: {a['outcome'] or ('accepted' if a['passed'] else 'miss')} (wall contacts as given: {a['wall_contacts_given']}, not observable from a bag)"
    return [head, "", *_table(["item", "", "description", "measured", "", "threshold", "note"], rows)]


def markdown(report: dict[str, Any]) -> str:
    lines = [
        f"# Timing report: {report['bag']}",
        "",
        f"Schema {report['schema']}. {report['preregistration']}. Counting window of each run: "
        "first non-zero `/drive_raw` command to the last `/pnp/planner/status` (v0.2 4.2). "
        "Percentiles are linear interpolation.",
        "",
        "## `/drive_raw` gaps, recorder receive time",
        "",
        *_gap_table(report, "recv"),
        "## `/drive_raw` gaps, header stamp",
        "",
        *_gap_table(report, "stamp"),
        "## Planner (`/pnp/planner/status`)",
        "",
        *_planner_table(report),
        "Plan age at publish: not carried by PlannerStatus. Tick gaps are header-stamp gaps; "
        "bunched is a gap under 10 ms (half a tick).",
        "",
        "Status gaps above each limit:",
        "",
        *_tick_gap_counts(report),
        "## `/safety/events` ENGAGE by source, inside the window",
        "",
        *_safety_table(report),
        *_detail_lines(report),
        "## Watchdog what-if",
        "",
        *_watchdog_whatif(report),
        "## Rate limiter what-if",
        "",
        *_limiter(report),
        "## v0.2 4.2 acceptance for C",
        "",
        *_acceptance(report),
    ]
    if "baseline_check" in report:
        b = report["baseline_check"]
        lines += [
            "## Baseline check",
            "",
            f"Expected (pre-registered): {b['expected']}",
            f"Measured: {b['measured']}",
            f"Result: {'MATCH' if b['matches'] else 'DIFFERS: both numbers are reported, the counting is not adjusted'}",
            "",
        ]
    return "\n".join(lines) + "\n"
