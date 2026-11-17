"""D9-C0 and D9-B0 for the camera lap source (brief 51), read from a car bag.

    cd tools && uv run python -m pnp_session lap-src --bag BAG [--bag BAG] \\
        [--laps-counted N ...] [--offline] [--assume-ready] [--drive] [--out report.json]

Default, D9-C0 per bag from the RECORDED topics (`/lap/event`, `/lap/status` and the lap node whose
status says `lap_source` `camera`, on `/pnp/lap/*` as the primary or on `/pnp/lap_shadow/*`): (i)
the camera's count equals the owner's (`--laps-counted`, once per bag in order; without it the
wheel rule of the v0.5 laps pre-registration stands in) and each of laps 1 to 10 has a distance
within 10 percent of the median of laps 2 to 10; (ii) every peak similarity of laps 1 to 10 is at
least 0.88; (iii) the lap node started ready, confirmed laps 1 to 10, rejected no event and its lap
lengths of laps 2 to 10 are within 0.10 m of the events' distance; (iv) no event arrived more than
0.5 s after its crossing time; (v) no gap above 3.0 s in the camera status. Verdicts per item and
per bag: PASS, FAIL, REPEAT (the count cannot be checked: take it and repeat), INSUFFICIENT (fewer
than 10 camera laps). A bag with no lap node on the camera source is refused.

`--offline`: D9-B0, the camera's events replayed through the lap source rule
(`racer_laps.external_replay`) per run of each bag (`pnp_data.runs` splits a bag into runs): (i)
time, (ii) length, (iii) the verdict against `lapref`. Labelled REGRESSION, NOT A GATE unless every
bag is an evaluation bag and `evaluation/prereg/freeze/b9.json` exists. `--assume-ready` is the
replay's development switch: the real-ready replay is printed first and the assumed-ready replay
(the one item iii judges) is the one the lap table shows.

`--drive`: the table of a D run (per lap the verdict, the reasons, the cap, the lap time, the clean
flag) from the lap node's recorded events and status.

Thresholds come from the yaml (`d9_c0`, `d9_b0`), refused unless its SHA-256 is the registered one.
An evaluation bag of D9-B0 is refused (not even opened) unless the freeze file exists and carries
that hash. Nothing here drives a car or writes a parameter.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import math
import re
import statistics
import sys
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pnp_session.constants import D9_FREEZE_FILE, D9_YAML, D9_YAML_SHA256

REPO_ROOT = Path(__file__).resolve().parents[2]
PASS, FAIL, REPEAT, INSUFFICIENT = "PASS", "FAIL", "REPEAT", "INSUFFICIENT"
#: Tolerance of "within" against the float noise of a difference of two times.
EPS = 1e-9

TOPIC_CAMERA = "/lap/event"
TOPIC_CAMERA_STATUS = "/lap/status"
#: The lap node's output prefixes, primary first.
NODE_PREFIXES = ("/pnp/lap", "/pnp/lap_shadow")
TOPIC_ODOM = "/odom/wheel"
TOPIC_DRIVE = "/drive"
TOPIC_DRIVE_RAW = "/drive_raw"
TOPIC_PLANNER = "/pnp/planner/status"
TOPIC_SAFETY = "/safety/events"
TOPICS = (
    TOPIC_CAMERA,
    TOPIC_CAMERA_STATUS,
    *(f"{p}/{k}" for p in NODE_PREFIXES for k in ("event", "status")),
    TOPIC_ODOM,
    TOPIC_DRIVE,
    TOPIC_DRIVE_RAW,
    TOPIC_PLANNER,
    TOPIC_SAFETY,
)
MAX_ODOM_DT_S = 1.0  # the gate's own cap on a dropout (gate.MAX_ODOM_DT_S)
#: Planner status is looked for this long before a run's first command (the planner reports
#: DRIVING before the car moves).
RUN_PRE_S = 3.0
#: Matching a lap node's event to the camera's by crossing time.
MATCH_TOL_S = 1e-3


class PreregError(ValueError):
    """The constants file is not the registered one."""


class FreezeRequired(PermissionError):
    """An evaluation bag was asked for and the freeze file is missing or wrong."""


class NoCameraNode(ValueError):
    """The bag has no lap node on the camera source."""


# -- the registered constants and the freeze guard ---------------------------------------------------


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_prereg(path: Path | None = None, expected: str = D9_YAML_SHA256) -> dict[str, Any]:
    """The yaml of the pre-registration, refused unless its SHA-256 is the registered one."""
    path = path or REPO_ROOT / D9_YAML
    actual = _sha256(path)
    if actual != expected:
        raise PreregError(f"{path} has SHA-256 {actual}, the registered value is {expected}")
    out = yaml.safe_load(path.read_text())
    assert isinstance(out, dict)
    return out


def freeze_state(path: Path | None = None, expected: str = D9_YAML_SHA256) -> tuple[bool, str]:
    """(frozen, why not). The freeze file holds the freeze commit SHA (`commit`, 40 hex digits)
    and the yaml hash (`yaml_sha256`, the registered one)."""
    path = path or REPO_ROOT / D9_FREEZE_FILE
    if not path.is_file():
        return False, f"no freeze file {path}"
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError) as error:
        return False, f"unreadable freeze file {path}: {error}"
    if not isinstance(data, dict):
        return False, "the freeze file is not a JSON object"
    commit = data.get("commit")
    if not isinstance(commit, str) or not re.fullmatch(r"[0-9a-f]{40}", commit):
        return False, "the freeze file has no 40 digit `commit`"
    if data.get("yaml_sha256") != expected:
        return False, "the freeze file's `yaml_sha256` is not the registered one"
    return True, ""


def evaluation_bags(prereg: dict[str, Any]) -> list[str]:
    return [str(b) for b in prereg["d9_b0"]["evaluation_bags"]]


def guard_bags(
    bags: Sequence[Path], prereg: dict[str, Any], freeze_path: Path | None = None
) -> bool:
    """Refuse (before anything is opened) an evaluation bag unless the freeze file exists. Returns
    whether every bag is an evaluation bag and the freeze file is there (the D9-B0 gate)."""
    names = evaluation_bags(prereg)
    frozen, why = freeze_state(freeze_path)
    evals = [Path(b).name in names for b in bags]
    if any(evals) and not frozen:
        raise FreezeRequired(
            f"{[Path(b).name for b, e in zip(bags, evals, strict=True) if e]} is an evaluation bag "
            f"of D9-B0 and is not opened before the freeze: {why}"
        )
    return bool(bags) and all(evals) and frozen


# -- reading a bag -----------------------------------------------------------------------------------------


@dataclass
class NodeTrace:
    """What one lap node recorded: its status JSON (with arrival) and its LapEvents."""

    prefix: str
    statuses: list[tuple[float, dict[str, Any]]] = field(default_factory=list)
    events: list[dict[str, float]] = field(default_factory=list)

    @property
    def last_status(self) -> dict[str, Any]:
        return self.statuses[-1][1] if self.statuses else {}


@dataclass
class BagData:
    name: str
    t0_log_s: float = 0.0
    streams: Any = None  # racer_laps.external_replay.ReplayStreams on the bag axis
    cam_events: list[dict[str, float]] = field(default_factory=list)
    cam_status: list[tuple[float, dict[str, Any]]] = field(default_factory=list)
    nodes: dict[str, NodeTrace] = field(default_factory=dict)
    draw_t: Any = None
    draw_speed: Any = None
    draw_steer: Any = None
    wheel_distance_m: float = 0.0
    reports: list[Any] = field(default_factory=list)


def racer_laps_path() -> None:
    """Make `racer_laps` importable from the checkout (numpy only)."""
    src = str(REPO_ROOT / "ros_ws" / "src" / "racer_laps" / "src")
    if src not in sys.path:
        sys.path.insert(0, src)


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _json(data: str) -> dict[str, Any] | None:
    try:
        d = json.loads(data)
    except ValueError:
        return None
    return d if isinstance(d, dict) else None


def bag_data_from_messages(
    name: str, t0_ns: int, messages: Sequence[tuple[int, str, Any]]
) -> BagData:
    """The arrays and lists of a bag from decoded messages (log time ns, topic, message), on the
    bag axis (seconds after the log time of the first /scan)."""
    racer_laps_path()
    from racer_laps.external_replay import ReplayStreams

    def ax(ns: int) -> float:
        return (ns - t0_ns) / 1e9

    out = BagData(name=name, t0_log_s=t0_ns / 1e9)
    odom: list[tuple[float, float, float]] = []
    drive: list[tuple[float, float]] = []
    draw: list[tuple[float, float, float]] = []
    planner: list[tuple[float, int, float]] = []
    safety: list[tuple[float, int, int, str]] = []
    events: list[tuple[float, int, float, float, float, float]] = []
    status: list[tuple[float, str, int, float]] = []
    for t_ns, topic, msg in messages:
        t = ax(t_ns)
        if topic == TOPIC_ODOM:
            stamp = ax(_stamp_ns(msg.header.stamp)) if _stamp_ns(msg.header.stamp) > 0 else t
            odom.append((t, stamp, float(msg.twist.twist.linear.x)))
        elif topic == TOPIC_DRIVE:
            drive.append((t, float(msg.drive.steering_angle)))
        elif topic == TOPIC_DRIVE_RAW:
            draw.append((t, float(msg.drive.speed), float(msg.drive.steering_angle)))
        elif topic == TOPIC_PLANNER:
            planner.append((t, int(msg.state), float(msg.plan_min_clearance_m)))
        elif topic == TOPIC_SAFETY:
            safety.append((t, int(msg.phase), int(msg.severity), str(msg.source)))
        elif topic == TOPIC_CAMERA:
            d = _json(msg.data)
            if d is None or "lap" not in d or "crossing_time_s" not in d:
                continue
            ev = {
                "lap": float(d["lap"]),
                "lap_time_s": float(d.get("lap_time_s", math.nan)),
                "peak_similarity": float(d.get("peak_similarity", math.nan)),
                "crossing_s": float(d["crossing_time_s"]) - out.t0_log_s,
                "distance_m": float(d.get("distance_m", math.nan)),
                "arrival_s": t,
            }
            out.cam_events.append(ev)
            events.append(
                (
                    t,
                    int(d["lap"]),
                    ev["crossing_s"],
                    ev["lap_time_s"],
                    ev["distance_m"],
                    ev["peak_similarity"],
                )
            )
        elif topic == TOPIC_CAMERA_STATUS:
            d = _json(msg.data)
            if d is None:
                continue
            out.cam_status.append((t, d))
            sim = d.get("similarity")
            status.append(
                (
                    t,
                    str(d.get("state", "")),
                    int(d.get("laps", -1)),
                    float(sim) if isinstance(sim, (int, float)) else math.nan,
                )
            )
        else:
            for prefix in NODE_PREFIXES:
                if topic == f"{prefix}/status":
                    d = _json(msg.data)
                    if d is not None:
                        out.nodes.setdefault(prefix, NodeTrace(prefix)).statuses.append((t, d))
                elif topic == f"{prefix}/event":
                    out.nodes.setdefault(prefix, NodeTrace(prefix)).events.append(
                        {
                            "lap_index": float(msg.lap_index),
                            "stamp_s": ax(_stamp_ns(msg.header.stamp)),
                            "lap_time_s": float(msg.lap_time_s),
                            "length_m": float(msg.lap_length_m),
                            "cap_mps": float(msg.v_cap_mps),
                            "clean": float(bool(msg.clean)),
                            "arrival_s": t,
                        }
                    )
    a = np.array

    def col(rows: list[Any], i: int, dtype: Any = float) -> Any:
        return a([r[i] for r in rows], dtype=dtype)

    s = ReplayStreams(
        odom_arrival=col(odom, 0),
        odom_stamp=col(odom, 1),
        odom_v=col(odom, 2),
        drive_arrival=col(drive, 0),
        drive_steer=col(drive, 1),
        planner_arrival=col(planner, 0),
        planner_state=col(planner, 1, np.int64),
        planner_clearance=col(planner, 2),
        safety_arrival=col(safety, 0),
        safety_phase=col(safety, 1, np.int64),
        safety_severity=col(safety, 2, np.int64),
        safety_source=col(safety, 3, str),
        event_arrival=col(events, 0),
        event_lap=col(events, 1, np.int64),
        event_cross=col(events, 2),
        event_lap_time=col(events, 3),
        event_distance=col(events, 4),
        event_peak=col(events, 5),
        status_arrival=col(status, 0),
        status_state=col(status, 1, str),
        status_laps=col(status, 2, np.int64),
        status_sim=col(status, 3),
    )
    out.streams = s
    out.draw_t, out.draw_speed, out.draw_steer = col(draw, 0), col(draw, 1), col(draw, 2)
    out.wheel_distance_m = _wheel_distance(s.odom_stamp, s.odom_v)
    return out


def _wheel_distance(t: Any, v: Any) -> float:
    """Trapezoid integral of max(0, speed); a gap above 1 s adds nothing (the gate's rule)."""
    if len(t) < 2:
        return 0.0
    speed = np.maximum(np.asarray(v, dtype=float), 0.0)
    dt = np.diff(np.asarray(t, dtype=float))
    step = np.where((dt > 0.0) & (dt <= MAX_ODOM_DT_S), 0.5 * (speed[1:] + speed[:-1]) * dt, 0.0)
    return float(np.sum(step))


def read_bag(path: Path) -> BagData:
    """A bag, read with the lenient stream reader (a bag that was not finalised is read as far as
    it goes; `reports` says how far)."""
    from pnp_session import streamread

    t0_ns, messages, reports = streamread.read_topics(Path(path), TOPICS)
    data = bag_data_from_messages(Path(path).name, t0_ns, messages)
    data.reports = reports
    return data


# -- D9-C0 ------------------------------------------------------------------------------------------------------


def _item(name: str, verdict: str, numbers: dict[str, Any], note: str = "") -> dict[str, Any]:
    return {"item": name, "verdict": verdict, "numbers": numbers, "note": note}


def camera_node(bag: BagData) -> NodeTrace:
    """The lap node whose status says `lap_source` camera: the primary's topics first, then the
    shadow's. Raises NoCameraNode with a clear message when there is none."""
    seen: list[str] = []
    for prefix in NODE_PREFIXES:
        node = bag.nodes.get(prefix)
        if node is None or not node.statuses:
            continue
        source = str(node.last_status.get("lap_source", "?"))
        seen.append(f"{prefix}: {source}")
        if source == "camera":
            return node
    raise NoCameraNode(
        f"{bag.name}: no lap node on the camera source in this bag (lap nodes seen: "
        f"{seen or 'none'}). D9-C0 needs `lap_source:=camera` as the primary or "
        "`lap_shadow:=camera`. D8-C0 is read with `lap-c0`."
    )


def _wheel_count(cam_distances: Sequence[float], wheel_m: float) -> tuple[int | None, str]:
    """The wheel rule of the v0.5 laps pre-registration section 4: driven wheel distance over the
    median camera lap distance, rounded down. None when it cannot be applied."""
    good = [d for d in cam_distances if math.isfinite(d)]
    if not good:
        return None, "the camera events carry no lap distances"
    if not math.isfinite(wheel_m) or wheel_m <= 0.0:
        return None, "no wheel odometry in the bag"
    median = statistics.median(good)
    if median <= 0.0:
        return None, f"median camera lap distance {median}"
    return math.floor(wheel_m / median + EPS), (
        f"wheel distance {wheel_m:.1f} m over the median camera lap {median:.2f} m"
    )


def evaluate_c0(bag: BagData, laps_counted: int | None, th: dict[str, Any]) -> dict[str, Any]:
    """D9-C0 of one bag from its recorded topics."""
    c0 = th["d9_c0"]
    judged = int(c0["laps_judged"])
    share = float(c0["lap_distance_within_share_of_median_laps_2_to_10"])
    peak_min = float(c0["peak_similarity_min"])
    lat_max = float(c0["event_latency_max_s"])
    gap_max = float(c0["status_gap_max_s"])
    node_cfg = c0["lap_node"]
    len_tol = float(node_cfg["length_tolerance_m"])
    node = camera_node(bag)
    ev = bag.cam_events
    label = {
        "i": f"(i) camera count equals the count called at the stop; laps 1 to {judged} within "
        f"{share:.0%} of the median distance of laps 2 to {judged}",
        "ii": f"(ii) every peak similarity of laps 1 to {judged} at least {peak_min}",
        "iii": f"(iii) lap node started ready, confirmed laps 1 to {judged}, rejected no event, "
        f"lengths of laps 2 to {judged} within {len_tol} m of the events' distance",
        "iv": f"(iv) no event arrived more than {lat_max} s after its crossing time",
        "v": f"(v) no gap above {gap_max} s in the camera status",
    }
    out: dict[str, Any] = {
        "bag": bag.name,
        "lap_node": node.prefix,
        "camera_laps": len(ev),
        "laps_counted": laps_counted,
    }
    if len(ev) < judged:
        out["verdict"] = INSUFFICIENT
        out["items"] = [
            _item(label[k], INSUFFICIENT, {"camera_laps": len(ev)}, f"needs {judged} camera laps")
            for k in label
        ]
        return out
    first = ev[:judged]
    items: list[dict[str, Any]] = []
    # (i)
    dist = [e["distance_m"] for e in ev]
    first_dist = [e["distance_m"] for e in first]
    median = (
        statistics.median(first_dist[1:])
        if all(math.isfinite(d) for d in first_dist[1:])
        else math.nan
    )
    deviations = [abs(d - median) / median if median > 0 else math.inf for d in first_dist]
    count_note = ""
    if laps_counted is not None:
        expected, how = laps_counted, "the owner's count"
        count_ok: bool | None = len(ev) == laps_counted
    else:
        wheel, why = _wheel_count(dist, bag.wheel_distance_m)
        expected, how = wheel, f"the wheel rule ({why})"
        count_ok = None if wheel is None else len(ev) == wheel
        count_note = "no count was taken: the wheel rule stands in for it"
    numbers_i = {
        "camera_count": len(ev),
        "expected_count": expected,
        "expected_from": how,
        "median_distance_laps_2_to_n_m": median,
        "max_deviation_share": max(deviations) if deviations else math.nan,
        "deviations": deviations,
    }
    if count_ok is None:
        items.append(_item(label["i"], REPEAT, numbers_i, f"{count_note}; it cannot be applied"))
    else:
        within = all(d <= share + EPS for d in deviations)
        items.append(
            _item(label["i"], PASS if count_ok and within else FAIL, numbers_i, count_note)
        )
    # (ii)
    peaks = [e["peak_similarity"] for e in first]
    items.append(
        _item(
            label["ii"],
            PASS if all(p >= peak_min - EPS for p in peaks) else FAIL,
            {"peaks": peaks, "lowest": min(peaks)},
        )
    )
    # (iii)
    st = node.last_status
    ready = st.get("source_ready")
    confirmed = st.get("confirmed_laps")
    doubtful = [d for d in st.get("doubtful_laps", []) if int(d.get("lap", 0)) <= judged]
    rejected = st.get("rejected_events", {})
    rejected_total = int(sum(rejected.values())) if isinstance(rejected, dict) else -1
    length_diffs: list[float] = []
    unmatched: list[int] = []
    for k, e in enumerate(first[1:], start=2):
        match = [n for n in node.events if abs(n["stamp_s"] - e["crossing_s"]) <= MATCH_TOL_S]
        if not match:
            unmatched.append(k)
            continue
        length_diffs.append(abs(match[0]["length_m"] - e["distance_m"]))
    numbers_iii = {
        "source_ready": ready,
        "source_not_ready_reason": st.get("source_not_ready_reason"),
        "confirmed_laps": confirmed,
        "doubtful_laps_1_to_n": doubtful,
        "rejected_events": rejected,
        "length_diff_m": length_diffs,
        "laps_with_no_node_event": unmatched,
    }
    if ready is None or confirmed is None:
        items.append(
            _item(label["iii"], INSUFFICIENT, numbers_iii, "the lap status has no source keys")
        )
    else:
        ok = (
            bool(ready)
            and int(confirmed) >= judged
            and not doubtful
            and rejected_total == 0
            and not unmatched
            and all(d <= len_tol + EPS for d in length_diffs)
        )
        items.append(_item(label["iii"], PASS if ok else FAIL, numbers_iii))
    # (iv)
    lat = [e["arrival_s"] - e["crossing_s"] for e in first]
    items.append(
        _item(
            label["iv"],
            PASS if all(x <= lat_max + EPS for x in lat) else FAIL,
            {"latency_s": lat, "largest": max(lat)},
        )
    )
    # (v)
    t_status = [t for t, _ in bag.cam_status]
    lo, hi = (t_status[0] if t_status else math.nan), first[-1]["arrival_s"]
    inside = [t for t in t_status if lo <= t <= hi]
    gaps = [b - a for a, b in itertools.pairwise(inside)]
    if not inside:
        items.append(_item(label["v"], INSUFFICIENT, {}, "no /lap/status in the bag"))
    else:
        # a gap at the end counts too: the last status before the 10th event to that event
        gaps.append(max(0.0, hi - inside[-1]))
        items.append(
            _item(
                label["v"],
                PASS if max(gaps) <= gap_max + EPS else FAIL,
                {"largest_gap_s": max(gaps), "statuses": len(inside)},
            )
        )
    verdicts = [i["verdict"] for i in items]
    out["items"] = items
    out["verdict"] = (
        FAIL
        if FAIL in verdicts
        else REPEAT
        if REPEAT in verdicts
        else INSUFFICIENT
        if INSUFFICIENT in verdicts
        else PASS
    )
    return out


# -- D9-B0 --------------------------------------------------------------------------------------------------------


def odometry_model(vehicle: Any | None = None) -> Any:
    """The odometry model of the lap source (wheelbase and yaw gain) from the live vehicle_params,
    through the generated binding (needs the `replay` extra of racer_laps)."""
    racer_laps_path()
    from racer_laps.gate import GateConfig
    from racer_laps.vehicle import load_live_vehicle

    v = vehicle or load_live_vehicle()
    return GateConfig(wheelbase_m=v.wheelbase_m, yaw_gain_table=v.yaw_gain_table)


@dataclass
class RunReplay:
    run_id: str
    bag: str
    start_s: float
    end_s: float
    events: list[dict[str, float]]
    real: Any  # ReplayResult, the real ready rule
    assumed: Any  # ReplayResult, ready assumed
    lapref_laps: int = 0
    lapref_length_m: float | None = None
    lapref_reason: str = ""


def run_windows(bag: BagData) -> list[Any]:
    """The runs of a bag (pnp_data.runs): rows with start_s, end_s on the bag axis."""
    from pnp_data.runs import run_rows

    s = bag.streams
    return run_rows(
        bag.draw_t, bag.draw_speed, bag.draw_steer, s.odom_arrival, s.odom_v, s.planner_arrival
    )


def replay_runs(bag: BagData, odom: Any, th: dict[str, Any]) -> list[RunReplay]:
    """Every run of the bag replayed twice: with the ready rule as recorded and with ready assumed."""
    racer_laps_path()
    from lapref.method import lapref
    from racer_laps.external_replay import external_config, replay_external

    rows = run_windows(bag)
    s = bag.streams
    out: list[RunReplay] = []
    previous_end = -math.inf
    for row in rows:
        t0 = max(previous_end + 1e-6, row.start_s - RUN_PRE_S)
        t1 = row.end_s + 1.0
        previous_end = row.end_s
        w = s.window(t0, t1)
        n_events = len(w.event_arrival)
        cfg = external_config(odom, "camera", "v0", 0.5, 1.0)
        real = replay_external(w, cfg, assume_ready=False, renumber_events=True)
        assumed = replay_external(w, cfg, assume_ready=True, renumber_events=True)
        events = [
            {
                "lap": float(w.event_lap[k]),
                "crossing_s": float(w.event_cross[k]),
                "lap_time_s": float(w.event_lap_time[k]),
                "distance_m": float(w.event_distance[k]),
                "peak_similarity": float(w.event_peak[k]),
                "arrival_s": float(w.event_arrival[k]),
            }
            for k in range(n_events)
        ]
        o = (s.odom_arrival >= row.start_s) & (s.odom_arrival <= row.end_s)
        d = (s.drive_arrival >= row.start_s) & (s.drive_arrival <= row.end_s)
        ref = lapref(s.odom_arrival[o], s.odom_v[o], s.drive_arrival[d], s.drive_steer[d])
        out.append(
            RunReplay(
                row.run_id,
                bag.name,
                row.start_s,
                row.end_s,
                events,
                real,
                assumed,
                ref.n_laps,
                ref.lap_length_m if ref.n_laps else None,
                ref.reason,
            )
        )
    return out


def _verdict_for(run: Any, crossing_s: float) -> Any | None:
    for v in run.verdicts:
        if abs(v.t_cross_s - crossing_s) <= MATCH_TOL_S:
            return v
    return None


def evaluate_b0(runs: Sequence[RunReplay], th: dict[str, Any]) -> dict[str, Any]:
    """D9-B0 over the runs of one or more bags: every camera event after the first of its run is a
    judged lap."""
    b0 = th["d9_b0"]
    t_tol = float(b0["crossing_stamp_tolerance_s"])
    lt_tol = float(b0["lap_time_tolerance_s"])
    len_tol = float(b0["length_tolerance_m"])
    lo, hi = (float(x) for x in b0["lapref_band"])
    outside_allowed = int(b0["confirmed_outside_band_allowed"])
    inside_share = float(b0["confirmed_inside_band_min_share"])
    min_laps = int(b0["insufficient_if"]["judged_laps_below"])
    label = {
        "i": f"(i) replayed crossing stamp within {t_tol * 1000:g} ms and lap time within "
        f"{lt_tol} s of the event, every judged lap",
        "ii": f"(ii) replayed wheel length within {len_tol} m of the event's distance",
        "iii": f"(iii) against lapref: confirmed laps outside {lo} to {hi} of the lapref length: "
        f"at most {outside_allowed}; at least {inside_share:.0%} of those inside it confirmed",
    }
    judged = 0
    t_bad: list[dict[str, Any]] = []
    l_bad: list[dict[str, Any]] = []
    rows: list[dict[str, Any]] = []
    outside_confirmed: list[dict[str, Any]] = []
    inside_total = inside_confirmed = 0
    lapref_runs = 0
    for run in runs:
        has_ref = run.lapref_laps > 0 and run.lapref_length_m is not None
        lapref_runs += int(has_ref)
        for k, e in enumerate(run.events):
            is_judged = k > 0  # the first event of a run is reported, not judged
            row: dict[str, Any] = {
                "bag": run.bag,
                "run": run.run_id,
                "event": k + 1,
                "judged": is_judged,
                "camera_lap_time_s": e["lap_time_s"],
                "camera_distance_m": e["distance_m"],
            }
            judged += int(is_judged)
            v = _verdict_for(run.assumed, e["crossing_s"])
            if v is None:
                if is_judged:
                    t_bad.append({**row, "why": "the event was rejected, no lap was closed"})
                    l_bad.append({**row, "why": "the event was rejected, no lap was closed"})
                rows.append({**row, "replayed": None})
                continue
            real = _verdict_for(run.real, e["crossing_s"])
            dt = abs(v.t_cross_s - e["crossing_s"])
            dlt = abs(v.lap_time_s - e["lap_time_s"])
            dl = abs(v.length_m - e["distance_m"])
            row.update(
                replayed_lap_time_s=v.lap_time_s,
                replayed_length_m=v.length_m,
                crossing_diff_s=dt,
                lap_time_diff_s=dlt,
                length_diff_m=dl,
                confirmed_assumed_ready=v.confirmed,
                reasons_assumed_ready=list(v.reasons),
                confirmed_real_ready=None if real is None else real.confirmed,
                reasons_real_ready=None if real is None else list(real.reasons),
                next_cap_mps=v.next_cap_mps,
            )
            rows.append(row)
            if not is_judged:
                continue
            if not (dt <= t_tol + EPS and dlt <= lt_tol + EPS):
                t_bad.append(row)
            if not dl <= len_tol + EPS:
                l_bad.append(row)
            if has_ref:
                ratio = v.length_m / float(run.lapref_length_m)  # type: ignore[arg-type]
                if lo - EPS <= ratio <= hi + EPS:
                    inside_total += 1
                    inside_confirmed += int(v.confirmed)
                elif v.confirmed:
                    outside_confirmed.append({**row, "ratio_to_lapref": ratio})
    numbers_iii = {
        "runs_with_a_lapref_period": lapref_runs,
        "confirmed_outside_band": len(outside_confirmed),
        "inside_band_laps": inside_total,
        "inside_band_confirmed": inside_confirmed,
        "inside_band_share": inside_confirmed / inside_total if inside_total else math.nan,
    }
    insufficient_why = ""
    if judged < min_laps:
        insufficient_why = f"{judged} judged laps, fewer than {min_laps}"
    elif lapref_runs == 0:
        insufficient_why = "no run with a lapref period"
    items = [
        _item(
            label["i"],
            PASS if not t_bad else FAIL,
            {"judged_laps": judged, "laps_over_tolerance": len(t_bad)},
        ),
        _item(
            label["ii"],
            PASS if not l_bad else FAIL,
            {"judged_laps": judged, "laps_over_tolerance": len(l_bad)},
        ),
    ]
    if lapref_runs == 0:
        items.append(_item(label["iii"], INSUFFICIENT, numbers_iii, "no run with a lapref period"))
    else:
        ok = len(outside_confirmed) <= outside_allowed and (
            inside_total == 0 or inside_confirmed / inside_total >= inside_share - EPS
        )
        items.append(_item(label["iii"], PASS if ok else FAIL, numbers_iii))
    verdicts = [i["verdict"] for i in items]
    if insufficient_why:
        verdict = INSUFFICIENT
    elif FAIL in verdicts:
        verdict = FAIL
    else:
        verdict = PASS
    return {
        "verdict": verdict,
        "insufficient_because": insufficient_why,
        "judged_laps": judged,
        "items": items,
        "laps": rows,
        "time_exceptions": t_bad,
        "length_exceptions": l_bad,
        "confirmed_outside_band": outside_confirmed,
        "runs": [
            {
                "bag": r.bag,
                "run": r.run_id,
                "start_s": r.start_s,
                "end_s": r.end_s,
                "camera_events": len(r.events),
                "source_ready_real": r.real.source_ready,
                "source_not_ready_reason": r.real.source_not_ready_reason,
                "real_ready": _replay_summary(r.real),
                "assumed_ready": _replay_summary(r.assumed),
                "lapref_laps": r.lapref_laps,
                "lapref_length_m": r.lapref_length_m,
                "lapref_reason": r.lapref_reason,
            }
            for r in runs
        ],
    }


def _replay_summary(result: Any) -> dict[str, Any]:
    return {
        "laps": len(result.verdicts),
        "confirmed": result.confirmed,
        "doubtful": result.doubtful,
        "rejected": result.rejected,
        "doubtful_laps": [
            {"lap": v.lap, "event_lap": v.event_lap, "reasons": list(v.reasons)}
            for v in result.verdicts
            if not v.confirmed
        ],
        "next_cap_mps": [v.next_cap_mps for v in result.verdicts],
        "caps_driven_mps": [v.cap_mps for v in result.verdicts],
    }


# -- the drive table ---------------------------------------------------------------------------------------------


def drive_table(bag: BagData) -> dict[str, Any]:
    """Per lap of a D run: verdict, reasons, cap driven, lap time, clean flag, from the lap node's
    recorded events and its last status. The primary node (`/pnp/lap`) when it has events, else
    the shadow."""
    node = next(
        (bag.nodes[p] for p in NODE_PREFIXES if p in bag.nodes and bag.nodes[p].events), None
    )
    if node is None:
        raise NoCameraNode(f"{bag.name}: no lap node events in this bag")
    st = node.last_status
    doubtful = {int(d["lap"]): list(d.get("reasons", [])) for d in st.get("doubtful_laps", [])}
    missed = {int(k) for k in st.get("missed_gate_laps", [])}
    rows = []
    for e in node.events:
        lap = int(e["lap_index"])
        bad = lap in doubtful or lap in missed
        rows.append(
            {
                "lap": lap,
                "verdict": "doubtful" if bad else "confirmed",
                "reasons": doubtful.get(lap, []),
                "cap_mps": e["cap_mps"],
                "lap_time_s": e["lap_time_s"],
                "length_m": e["length_m"],
                "clean": bool(e["clean"]),
            }
        )
    return {
        "bag": bag.name,
        "lap_node": node.prefix,
        "lap_source": st.get("lap_source"),
        "lap_mode": st.get("lap_mode"),
        "source_ready": st.get("source_ready"),
        "reference_suspect": st.get("reference_suspect"),
        "reference_suspect_lap": st.get("reference_suspect_lap"),
        "rows": rows,
    }


# -- the report -------------------------------------------------------------------------------------------------


def _n(x: Any, fmt: str = ".2f") -> str:
    return "-" if x is None or (isinstance(x, float) and math.isnan(x)) else format(x, fmt)


def markdown_c0(blocks: Sequence[dict[str, Any]]) -> str:
    lines: list[str] = []
    for b in blocks:
        lines += [
            f"# D9-C0 {b['bag']}: {b['verdict']}",
            "",
            (
                f"lap node {b['lap_node']}, camera laps {b['camera_laps']}, "
                f"owner's count {b['laps_counted']}"
            ),
            "",
        ]
        for it in b["items"]:
            lines.append(f"- {it['item']}: {it['verdict']} {_short(it['numbers'])}")
            if it["note"]:
                lines.append(f"  {it['note']}")
        lines.append("")
    return "\n".join(lines)


def _short(numbers: dict[str, Any]) -> str:
    def cut(v: Any) -> Any:
        if isinstance(v, float):
            return round(v, 3)
        if isinstance(v, list):
            return [cut(x) for x in v]
        if isinstance(v, dict):
            return {k: cut(x) for k, x in v.items()}
        return v

    return json.dumps(cut(numbers), default=str)


def markdown_b0(block: dict[str, Any], label: str, assume_ready: bool) -> str:
    lines = [
        label,
        "",
        f"# D9-B0: {block['verdict']}"
        + (f" ({block['insufficient_because']})" if block["insufficient_because"] else ""),
        "",
    ]
    for it in block["items"]:
        lines.append(f"- {it['item']}: {it['verdict']} {_short(it['numbers'])}")
        if it["note"]:
            lines.append(f"  {it['note']}")
    lines.append("")
    for r in block["runs"]:
        r["assumed_ready"] if assume_ready else r["real_ready"]
        lines += [
            (
                f"{r['bag']} {r['run']} ({r['start_s']:.1f} to {r['end_s']:.1f} s), "
                f"{r['camera_events']} camera events; lapref laps {r['lapref_laps']}, "
                f"length {_n(r['lapref_length_m'])}"
            ),
            f"  real ready rule: source_ready {r['source_ready_real']}"
            + (f" ({r['source_not_ready_reason']})" if not r["source_ready_real"] else "")
            + f"; {r['real_ready']['laps']} laps, {r['real_ready']['confirmed']} confirmed, "
            f"{r['real_ready']['doubtful']} doubtful, rejected {r['real_ready']['rejected']}",
            (
                f"  ready assumed: {r['assumed_ready']['laps']} laps, "
                f"{r['assumed_ready']['confirmed']} confirmed, {r['assumed_ready']['doubtful']} doubtful"
            ),
            "",
            (
                "| event | camera s | camera m | replay s | replay m | d stamp s | d length m | "
                "confirmed | reasons | next cap |"
            ),
            "|---|---|---|---|---|---|---|---|---|---|",
        ]
        for row in block["laps"]:
            if row["bag"] != r["bag"] or row["run"] != r["run"]:
                continue
            ok = "confirmed_assumed_ready" if assume_ready else "confirmed_real_ready"
            why = "reasons_assumed_ready" if assume_ready else "reasons_real_ready"
            lines.append(
                f"| {row['event']}{'' if row['judged'] else ' (lap 1, not judged)'} | "
                f"{_n(row['camera_lap_time_s'])} | "
                f"{_n(row['camera_distance_m'])} | {_n(row.get('replayed_lap_time_s'))} | "
                f"{_n(row.get('replayed_length_m'))} | {_n(row.get('crossing_diff_s'), '.4f')} | "
                f"{_n(row.get('length_diff_m'), '.3f')} | {row.get(ok)} | "
                f"{','.join(row.get(why) or [])} | {_n(row.get('next_cap_mps'))} |"
            )
        lines.append("")
    return "\n".join(lines)


def _suspect_text(table: dict[str, Any]) -> str:
    flag = table.get("reference_suspect")
    if flag is None:
        return "unknown"
    return f"True (lap {table.get('reference_suspect_lap')})" if flag else "False"


def markdown_drive(table: dict[str, Any]) -> str:
    lines = [
        (
            f"# drive table {table['bag']} ({table['lap_node']}, source {table['lap_source']}, "
            f"mode {table['lap_mode']}, source_ready {table['source_ready']}, "
            f"reference_suspect {_suspect_text(table)})"
        ),
        "",
        "| lap | verdict | reasons | cap m/s | lap time s | length m | clean |",
        "|---|---|---|---|---|---|---|",
    ]
    for r in table["rows"]:
        lines.append(
            f"| {r['lap']} | {r['verdict']} | {','.join(r['reasons'])} | {_n(r['cap_mps'])} | "
            f"{_n(r['lap_time_s'])} | {_n(r['length_m'])} | {r['clean']} |"
        )
    lines.append("")
    return "\n".join(lines)


def run(
    bags: Sequence[Path],
    laps_counted: Sequence[int] = (),
    offline: bool = False,
    assume_ready: bool = False,
    drive: bool = False,
    prereg_path: Path | None = None,
    freeze_path: Path | None = None,
    odom: Any | None = None,
) -> tuple[dict[str, Any], str, int]:
    """(report, text, exit code). Raises PreregError, FreezeRequired or NoCameraNode."""
    th = load_prereg(prereg_path)
    gate = guard_bags(bags, th, freeze_path)
    names = evaluation_bags(th)
    if any(Path(b).name in names for b in bags) and not offline:
        raise FreezeRequired("an evaluation bag of D9-B0 is read with --offline only")
    counts: list[int | None] = list(laps_counted) + [None] * max(0, len(bags) - len(laps_counted))
    report: dict[str, Any] = {
        "schema": "pnp-session-lap-src/1",
        "prereg_yaml_sha256": D9_YAML_SHA256,
        "bags": [Path(b).name for b in bags],
    }
    text: list[str] = []
    data = [read_bag(Path(b)) for b in bags]
    for d in data:
        for rep in d.reports:
            if not rep.complete:
                text.append(f"NOTE: {rep.say()}")
    if offline:
        label = (
            "D9-B0 on evaluation bags after the freeze: the gate, scored once"
            if gate
            else "REGRESSION, NOT A GATE: D9-B0 items on bags that are not the frozen "
            "evaluation bags"
        )
        model = odom if odom is not None else odometry_model()
        runs: list[RunReplay] = []
        for d in data:
            runs += replay_runs(d, model, th)
        block = evaluate_b0(runs, th)
        report.update(label=label, d9_b0=block, gate=gate, assume_ready=assume_ready)
        text.append(markdown_b0(block, label, assume_ready))
        code = 0 if block["verdict"] in (PASS, INSUFFICIENT) else 1
        return report, "\n".join(text), code
    if drive:
        tables = [drive_table(d) for d in data]
        report.update(label="the lap table of a D run, from the recorded lap node", drive=tables)
        text += [markdown_drive(t) for t in tables]
        return report, "\n".join(text), 0
    blocks = [evaluate_c0(d, c, th) for d, c in zip(data, counts, strict=True)]
    report.update(label="recorded topics: the gate", d9_c0=blocks)
    text.append(markdown_c0(blocks))
    return report, "\n".join(text), 0 if all(b["verdict"] == PASS for b in blocks) else 1
