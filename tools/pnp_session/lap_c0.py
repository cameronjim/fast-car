"""D3-C0, the D7-C0 rule and D8-C0 for the same runs (brief 50, lap gate v3).

    cd tools && uv run python pnp_session_report.py lap-c0 --bag BAG --bag BAG \\
        [--video laps.csv] [--rule d7|d8] [--replay] [--out report.json]

Each bag is one run (observe mode, at least 10 laps, one per direction). Three blocks are printed
for the same runs:

* D3-C0, by `laps.d3c0` unchanged (pre-registration PnP v0.3 section 1.5);
* the D7-C0 rule, items (i) to (iv) of the v0.5 laps pre-registration section 4;
* D8-C0, the same four items and item (v) of the lap gate v3 pre-registration section 5.

Items, over the first `laps_judged` (10) laps of each run: (i) the lap count equals the reference's
in every run; (ii) of the 18 lap times of laps 2 to 10 at least 17 are within 0.3 s of the
reference; (iii) lap 1 is judged by its crossing instant: system crossing 1 minus reference
crossing 1 is within 0.3 s of the median of that difference over crossings 2 to 10 of the same run;
(iv) no lap is flagged missed gate (D7: no lap of the run, D8: no judged lap); (v) (D8 only) the lap
node's `on_scan_ms_p99` is at most 80 ms in every run, read from its status.

The reference is the camera lap counter (`/lap/event`: lap time, crossing time and lap distance), or
with `--video` the CSV of `laps.read_video_times` plus a `t_cross_s` column (and, for the validity
rule, a `distance_m` column). A reference whose count over the run is not the wheel distance count
(driven wheel distance over the median reference lap distance, rounded down) is not a reference: the
verdict is REPEAT, never PASS or FAIL. One run, or a run of under 10 laps, gives INSUFFICIENT.

Verdicts per item and per block: PASS, FAIL, REPEAT, INSUFFICIENT (and NO REFERENCE when no run has
any reference lap). `--replay` recomputes the lap events by replaying the bag through the
checked-out gate (`racer_laps.replay`) and labels the report REGRESSION, NOT A GATE: without it the
recorded events are used, and that is the gate. The thresholds come from the two pre-registration
yaml files (`d7_c0` of `pnp-v0.5-laps.yaml`, `d8_c0` of `pnp-v0.5-laps-ring.yaml`), each checked
against its SHA-256. Nothing here drives a car or writes a parameter.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import statistics
import sys
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from pnp_session.constants import (
    D7C0_LAPS_YAML,
    D7C0_LAPS_YAML_SHA256,
    D8C0_RING_YAML,
    D8C0_RING_YAML_SHA256,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
PASS, FAIL, REPEAT, INSUFFICIENT, NO_REFERENCE = (
    "PASS",
    "FAIL",
    "REPEAT",
    "INSUFFICIENT",
    "NO REFERENCE",
)
#: Tolerance of "within" against the float noise of a difference of two times.
EPS = 1e-9


class PreregError(ValueError):
    """A constants file is not the registered one."""


@dataclass(frozen=True)
class LapRun:
    """One run: the laps of the lap memory and of the reference, in order."""

    run_id: str
    sys_times: tuple[float, ...] = ()
    sys_cross: tuple[float, ...] = ()  # crossing instants (LapEvent.header.stamp)
    ref_times: tuple[float, ...] = ()
    ref_cross: tuple[float, ...] = ()
    ref_distance_m: tuple[float, ...] = ()  # per reference lap (empty: unknown)
    wheel_distance_m: float = math.nan
    #: Lap indices flagged missed gate; None when the source has no such information.
    missed_laps: tuple[int, ...] | None = None
    line_sources: tuple[str, ...] = ()
    #: Largest `on_scan_ms_p99` the lap node's status reported in the run (NaN: no such key).
    on_scan_p99_ms: float = math.nan
    lap_mode: str = ""


@dataclass
class Thresholds:
    """The `d7_c0` and `d8_c0` blocks, with the hashes of the files they were read from."""

    d7: dict[str, Any]
    d8: dict[str, Any]
    d7_sha256: str
    d8_sha256: str


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_thresholds(
    d7_path: Path | None = None,
    d8_path: Path | None = None,
    d7_expected: str = D7C0_LAPS_YAML_SHA256,
    d8_expected: str = D8C0_RING_YAML_SHA256,
) -> Thresholds:
    """The two blocks, each file refused unless its SHA-256 is the registered one."""
    d7_path = d7_path or REPO_ROOT / D7C0_LAPS_YAML
    d8_path = d8_path or REPO_ROOT / D8C0_RING_YAML
    out = []
    for path, expected, key in ((d7_path, d7_expected, "d7_c0"), (d8_path, d8_expected, "d8_c0")):
        actual = _sha256(path)
        if actual != expected:
            raise PreregError(f"{path} has SHA-256 {actual}, the registered value is {expected}")
        out.append((yaml.safe_load(path.read_text())[key], actual))
    return Thresholds(out[0][0], out[1][0], out[0][1], out[1][1])


def read_video_laps(path: Path) -> dict[str, dict[str, tuple[float, ...]]]:
    """Hand-timed laps: the CSV of `laps.read_video_times` (`run,lap,lap_time_s`) plus `t_cross_s`
    and, optionally, `distance_m`. Per run: lap times, crossing instants and distances in `lap`
    order (the last two empty when the column is missing)."""
    rows: dict[str, list[tuple[int, float, float | None, float | None]]] = {}
    with Path(path).open(newline="") as f:
        reader = csv.DictReader(f)
        needed = {"run", "lap", "lap_time_s", "t_cross_s"}
        if reader.fieldnames is None or not needed <= set(reader.fieldnames):
            raise ValueError(f"{path}: the CSV needs the columns run, lap, lap_time_s, t_cross_s")
        has_distance = "distance_m" in reader.fieldnames
        for row in reader:
            dist = row.get("distance_m", "").strip() if has_distance else ""
            rows.setdefault(row["run"].strip(), []).append(
                (
                    int(row["lap"]),
                    float(row["lap_time_s"]),
                    float(row["t_cross_s"]),
                    float(dist) if dist else None,
                )
            )
    out: dict[str, dict[str, tuple[float, ...]]] = {}
    for run, laps in rows.items():
        ordered = sorted(laps, key=lambda r: r[0])
        distances = tuple(r[3] for r in ordered if r[3] is not None)
        out[run] = {
            "times": tuple(r[1] for r in ordered),
            "cross": tuple(r[2] for r in ordered if r[2] is not None),
            "distance": distances if len(distances) == len(ordered) else (),
        }
    return out


# -- the rules ----------------------------------------------------------------------------------------


def reference_valid(run: LapRun) -> tuple[bool | None, str]:
    """The registered validity of the reference: its lap count over the run equals the wheel
    distance count (driven wheel distance over the median reference lap distance, rounded down).
    None: no reference lap at all."""
    if not run.ref_times:
        return None, "no reference lap"
    if not run.ref_distance_m:
        return False, "the reference has no lap distances: the validity rule cannot be applied"
    if not math.isfinite(run.wheel_distance_m):
        return False, "no wheel odometry in the run"
    median = statistics.median(run.ref_distance_m)
    if median <= 0.0:
        return False, f"median reference lap distance {median}"
    expected = math.floor(run.wheel_distance_m / median + EPS)
    ok = len(run.ref_times) == expected
    return (
        ok,
        (
            f"{len(run.ref_times)} reference laps, wheel distance {run.wheel_distance_m:.1f} m "
            f"over the median reference lap {median:.2f} m gives {expected}"
        ),
    )


def lap_rows(run: LapRun, judged: int | None = None) -> list[dict[str, Any]]:
    """Per lap: the system time, the reference time, the difference, the crossing offset (system
    crossing minus reference crossing), the line source and the missed flag."""
    n = max(len(run.sys_times), len(run.ref_times))
    if judged is not None:
        n = min(n, judged)
    missed = set(run.missed_laps or ())
    rows = []
    for k in range(n):
        s = run.sys_times[k] if k < len(run.sys_times) else None
        r = run.ref_times[k] if k < len(run.ref_times) else None
        sc = run.sys_cross[k] if k < len(run.sys_cross) else None
        rc = run.ref_cross[k] if k < len(run.ref_cross) else None
        rows.append(
            {
                "lap": k + 1,
                "system_s": s,
                "reference_s": r,
                "diff_s": None if s is None or r is None else s - r,
                "crossing_offset_s": None if sc is None or rc is None else sc - rc,
                "line_source": run.line_sources[k] if k < len(run.line_sources) else None,
                "missed": None if run.missed_laps is None else (k + 1) in missed,
            }
        )
    return rows


def _item(name: str, verdict: str, numbers: dict[str, Any], note: str = "") -> dict[str, Any]:
    return {"item": name, "verdict": verdict, "numbers": numbers, "note": note}


def evaluate(runs: Sequence[LapRun], rule: str, th: Thresholds) -> dict[str, Any]:
    """The D7-C0 rule (`rule="d7"`, items i to iv) or D8-C0 (`"d8"`, items i to v) over the runs."""
    if rule not in ("d7", "d8"):
        raise ValueError("rule is d7 or d8")
    block = th.d7 if rule == "d7" else th.d8
    judged = int(block["laps_judged"])
    tol = float(block["tolerance_s"])
    n_runs = int(block["runs"])
    min_within = int(block["laps_2_to_10_min_within"])
    of_laps = n_runs * (judged - 1)
    missed_allowed = int(block["missed_gate_flags_allowed"])
    p99_max = float(block.get("on_scan_p99_max_ms", math.nan))
    names = ["i", "ii", "iii", "iv"] + (["v"] if rule == "d8" else [])
    label = {
        "i": "(i) lap count equals the reference's in every run",
        "ii": f"(ii) at least {min_within} of {of_laps} lap times of laps 2 to {judged} within {tol} s",
        "iii": f"(iii) lap 1 crossing offset within {tol} s of the median offset of crossings 2 to {judged}",
        "iv": "(iv) no lap flagged missed gate" + (" (judged laps)" if rule == "d8" else ""),
        "v": f"(v) on_scan p99 at most {p99_max:g} ms in every run",
    }

    def all_items(verdict: str, note: str) -> list[dict[str, Any]]:
        return [_item(label[n], verdict, {}, note) for n in names]

    validity = []
    for run in runs:
        ok, why = reference_valid(run)
        validity.append({"run_id": run.run_id, "valid": ok, "detail": why})
    out: dict[str, Any] = {
        "rule": rule,
        "tolerance_s": tol,
        "laps_judged": judged,
        "runs": [r.run_id for r in runs],
        "validity": validity,
        "lap_rows": {r.run_id: lap_rows(r) for r in runs},
    }
    if all(v["valid"] is None for v in validity):
        out.update(
            verdict=NO_REFERENCE, items=all_items(NO_REFERENCE, "no reference lap in any run")
        )
        return out
    if len(runs) < n_runs or any(
        len(r.sys_times) < judged or len(r.ref_times) < judged for r in runs
    ):
        out.update(
            verdict=INSUFFICIENT,
            items=all_items(
                INSUFFICIENT,
                f"needs {n_runs} runs of at least {judged} laps in the system and the reference",
            ),
        )
        return out
    if any(v["valid"] is not True for v in validity):
        bad = [v["run_id"] for v in validity if v["valid"] is not True]
        out.update(
            verdict=REPEAT,
            items=all_items(REPEAT, f"the reference of {bad} fails its validity rule: repeat"),
        )
        return out

    items: list[dict[str, Any]] = []
    # (i)
    counts = {r.run_id: (len(r.sys_times), len(r.ref_times)) for r in runs}
    items.append(
        _item(
            label["i"],
            PASS if all(a == b for a, b in counts.values()) else FAIL,
            {"system_reference_counts": counts},
        )
    )
    # (ii)
    diffs = [abs(r.sys_times[k] - r.ref_times[k]) for r in runs for k in range(1, judged)]
    within = sum(1 for d in diffs if d <= tol + EPS)
    items.append(
        _item(
            label["ii"],
            PASS if within >= min_within else FAIL,
            {"within": within, "of": len(diffs), "needed": min_within},
        )
    )
    # (iii)
    offsets: dict[str, dict[str, float]] = {}
    ok_iii: bool | None = True
    for r in runs:
        if len(r.sys_cross) < judged or len(r.ref_cross) < judged:
            ok_iii = None
            break
        d = [r.sys_cross[k] - r.ref_cross[k] for k in range(judged)]
        med = statistics.median(d[1:])
        offsets[r.run_id] = {"offset_1": d[0], "median_2_to_n": med, "difference": d[0] - med}
        if abs(d[0] - med) > tol + EPS:
            ok_iii = False
    items.append(
        _item(
            label["iii"],
            INSUFFICIENT if ok_iii is None else (PASS if ok_iii else FAIL),
            offsets,
            "no crossing instants in the events" if ok_iii is None else "",
        )
    )
    # (iv)
    if any(r.missed_laps is None for r in runs):
        items.append(_item(label["iv"], INSUFFICIENT, {}, "the lap status has no missed_gate_laps"))
    else:
        flagged: dict[str, list[int]] = {}
        for r in runs:
            laps = sorted(r.missed_laps or ())
            if rule == "d8":
                laps = [k for k in laps if k <= judged]
            flagged[r.run_id] = laps
        total = sum(len(v) for v in flagged.values())
        items.append(
            _item(label["iv"], PASS if total <= missed_allowed else FAIL, {"flagged": flagged})
        )
    # (v)
    if rule == "d8":
        p99 = {r.run_id: r.on_scan_p99_ms for r in runs}
        if any(math.isnan(v) for v in p99.values()):
            items.append(
                _item(label["v"], INSUFFICIENT, p99, "the lap status has no on_scan_ms_p99")
            )
        else:
            items.append(
                _item(label["v"], PASS if all(v <= p99_max for v in p99.values()) else FAIL, p99)
            )
    verdicts = [i["verdict"] for i in items]
    if FAIL in verdicts:
        verdict = FAIL
    elif INSUFFICIENT in verdicts:
        verdict = INSUFFICIENT
    else:
        verdict = PASS
    out.update(verdict=verdict, items=items)
    return out


def sources_summary(runs: Sequence[LapRun]) -> dict[str, dict[str, int]]:
    """Reported: the line source of every lap, counted per run."""
    out: dict[str, dict[str, int]] = {}
    for r in runs:
        counts: dict[str, int] = {}
        for s in r.line_sources:
            counts[s] = counts.get(s, 0) + 1
        out[r.run_id] = counts
    return out


# -- reading a bag ----------------------------------------------------------------------------------------

TOPIC_CAMERA = "/lap/event"
TOPIC_LAP = "/pnp/lap/event"
TOPIC_STATUS = "/pnp/lap/status"
TOPIC_ODOM = "/odom/wheel"
MAX_ODOM_DT_S = 1.0  # the gate's own cap on a dropout (gate.MAX_ODOM_DT_S)


@dataclass
class BagLaps:
    """What the bag holds, before it becomes a LapRun."""

    name: str
    sys_times: list[float] = field(default_factory=list)
    sys_cross: list[float] = field(default_factory=list)
    ref_times: list[float] = field(default_factory=list)
    ref_cross: list[float] = field(default_factory=list)
    ref_distance: list[float] = field(default_factory=list)
    wheel_distance_m: float = 0.0
    statuses: list[dict[str, Any]] = field(default_factory=list)


def _stamp(s: Any) -> float:
    return float(s.sec) + float(s.nanosec) * 1e-9


def read_bag(path: Path) -> BagLaps:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    from pnp_session.bagread import bag_files

    out = BagLaps(Path(path).name)
    last_t: float | None = None
    last_v = 0.0
    topics = [TOPIC_CAMERA, TOPIC_LAP, TOPIC_STATUS, TOPIC_ODOM]
    for file in bag_files(Path(path)):
        with file.open("rb") as stream:
            reader = make_reader(stream, decoder_factories=[DecoderFactory()])
            for _s, channel, _m, msg in reader.iter_decoded_messages(topics=topics):
                topic = channel.topic
                if topic == TOPIC_CAMERA:
                    d = json.loads(msg.data)
                    out.ref_times.append(float(d["lap_time_s"]))
                    out.ref_cross.append(float(d.get("crossing_time_s", math.nan)))
                    out.ref_distance.append(float(d.get("distance_m", math.nan)))
                elif topic == TOPIC_LAP:
                    out.sys_times.append(float(msg.lap_time_s))
                    out.sys_cross.append(_stamp(msg.header.stamp))
                elif topic == TOPIC_STATUS:
                    try:
                        out.statuses.append(json.loads(msg.data))
                    except json.JSONDecodeError:
                        continue
                elif topic == TOPIC_ODOM:
                    t = _stamp(msg.header.stamp)
                    v = max(0.0, float(msg.twist.twist.linear.x))
                    if last_t is not None and t > last_t:
                        out.wheel_distance_m += 0.5 * (last_v + v) * min(t - last_t, MAX_ODOM_DT_S)
                    last_t, last_v = t, v
    return out


def run_from_bag(
    bag: BagLaps,
    video: dict[str, dict[str, tuple[float, ...]]] | None = None,
    replayed: Any | None = None,
) -> LapRun:
    """A LapRun from the bag's recorded events (the gate) or from a replay of the checked-out gate
    (`replayed`: a `racer_laps.replay.ScanReplayResult`, a regression)."""
    ref_times = tuple(bag.ref_times)
    ref_cross = tuple(bag.ref_cross)
    ref_dist: tuple[float, ...] = tuple(bag.ref_distance)
    if video is not None:
        v = video.get(bag.name, {})
        ref_times = v.get("times", ())
        ref_cross = v.get("cross", ())
        ref_dist = v.get("distance", ())
    if any(not math.isfinite(x) for x in ref_dist):
        ref_dist = ()
    last = next((s for s in reversed(bag.statuses) if "missed_gate_laps" in s), None)
    p99s = [float(s["on_scan_ms_p99"]) for s in bag.statuses if s.get("on_scan_ms_p99") is not None]
    mode = next((str(s["lap_mode"]) for s in reversed(bag.statuses) if "lap_mode" in s), "")
    if replayed is not None:
        laps = replayed.laps
        return LapRun(
            run_id=bag.name,
            sys_times=tuple(float(x.lap_time_s) for x in laps),
            sys_cross=tuple(float(x.t_cross_s) for x in laps),
            ref_times=ref_times,
            ref_cross=ref_cross,
            ref_distance_m=ref_dist,
            wheel_distance_m=bag.wheel_distance_m,
            missed_laps=tuple(int(x.lap_index) for x in laps if x.missed_gate),
            line_sources=tuple(str(x.line_source) for x in laps),
            on_scan_p99_ms=_p99(replayed.scan_times_s),
            lap_mode="replay",
        )
    return LapRun(
        run_id=bag.name,
        sys_times=tuple(bag.sys_times),
        sys_cross=tuple(bag.sys_cross),
        ref_times=ref_times,
        ref_cross=ref_cross,
        ref_distance_m=ref_dist,
        wheel_distance_m=bag.wheel_distance_m,
        missed_laps=None if last is None else tuple(int(k) for k in last["missed_gate_laps"]),
        line_sources=tuple(str(s) for s in (last or {}).get("lap_line_sources", ())),
        on_scan_p99_ms=max(p99s) if p99s else math.nan,
        lap_mode=mode,
    )


def _p99(seconds: Sequence[float]) -> float:
    if not seconds:
        return math.nan
    ordered = sorted(1000.0 * s for s in seconds)
    return ordered[min(len(ordered) - 1, int(0.99 * len(ordered)))]


def replay_bag(path: Path) -> Any:
    """The bag through the checked-out gate (racer_laps.replay), a regression record."""
    sys.path.insert(0, str(REPO_ROOT / "ros_ws" / "src" / "racer_laps" / "src"))
    from racer_laps import replay
    from racer_laps.vehicle import load_live_vehicle

    vehicle = load_live_vehicle()
    messages = list(replay.read_messages(path))
    return replay.run_replay_start_scan(
        messages, vehicle, replay.default_start_scan_config(vehicle)
    )


# -- the report ----------------------------------------------------------------------------------------------


def markdown(report: dict[str, Any]) -> str:
    lines = []
    if report["replay"]:
        lines += [
            "REGRESSION, NOT A GATE: the lap events were recomputed by replaying the bags",
            "",
        ]
    d3 = report["d3_c0"]
    lines += [
        f"# D3-C0 (v0.3 1.5): {d3['verdict']}",
        "",
        f"rule: {d3['rule']}; laps compared {d3['laps_compared']}, within {d3['laps_within']}",
        *[f"- {x}" for x in d3["reasons"]],
        "",
    ]
    for key, title in (
        ("d7_c0", "D7-C0 rule (v0.5 laps section 4)"),
        ("d8_c0", "D8-C0 (v0.5 laps ring section 5)"),
    ):
        block = report[key]
        lines += [f"# {title}: {block['verdict']}", ""]
        for v in block["validity"]:
            lines.append(f"- reference of {v['run_id']}: {v['detail']}")
        lines.append("")
        for it in block["items"]:
            lines.append(
                f"- {it['item']}: {it['verdict']} {json.dumps(it['numbers'], default=str)}"
            )
            if it["note"]:
                lines.append(f"  {it['note']}")
        lines.append("")
        for run_id, rows in block["lap_rows"].items():
            lines += [
                f"{run_id}",
                "",
                "| lap | system s | reference s | diff s | crossing offset s | line source | missed |",
                "|---|---|---|---|---|---|---|",
            ]
            for r in rows:
                lines.append(
                    f"| {r['lap']} | {_n(r['system_s'])} | {_n(r['reference_s'])} | "
                    f"{_n(r['diff_s'], signed=True)} | {_n(r['crossing_offset_s'], signed=True)} | "
                    f"{r['line_source']} | {r['missed']} |"
                )
            lines.append("")
    lines += [f"line sources per run: {report['line_sources']}", ""]
    return "\n".join(lines)


def _n(x: float | None, signed: bool = False) -> str:
    if x is None:
        return "-"
    return f"{x:+.2f}" if signed else f"{x:.2f}"


def build_report(
    runs: Sequence[LapRun], reference_name: str, th: Thresholds, replay: bool, rule: str | None
) -> dict[str, Any]:
    """The three blocks for the same runs. `rule` d7 or d8 only picks what the exit code follows."""
    from pnp_session import laps as d3

    d3_inputs = [d3.RunLapInput(r.run_id, r.sys_times, r.ref_times) for r in runs]
    report: dict[str, Any] = {
        "schema": "pnp-session-lap-c0/1",
        "replay": replay,
        "label": "REGRESSION, NOT A GATE" if replay else "recorded events: the gate",
        "reference": reference_name,
        "d7_yaml_sha256": th.d7_sha256,
        "d8_yaml_sha256": th.d8_sha256,
        "d3_c0": d3.d3c0(d3_inputs, reference_name),
        "d7_c0": evaluate(runs, "d7", th),
        "d8_c0": evaluate(runs, "d8", th),
        "line_sources": sources_summary(runs),
        "rule_for_exit_code": rule,
    }
    return report
