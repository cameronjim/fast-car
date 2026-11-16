"""The planner-to-safety timing chain, read from a bag (brief 42, addendum 1 section 4: M-C2, M-C3
and the command latch of M-T2).

safety_node stamps every `/drive` with the time of its 50 Hz gate cycle and passes an unaltered
field of the latched `/drive_raw` through as the same float32. So the first `/drive` that carries
the values of a `/drive_raw` says that command was latched between the previous cycle stamp and
that one: the receipt to within one cycle, read from the bag, with safety_node unchanged. When the
bag holds `/safety/cycle` (brief 45: the latched command's header stamp, its age and count per
cycle) the latch is taken from it exactly and the report says so.

Per command (one `/drive_raw`):

    identified                 a first cycle where a field that CHANGED against the previous
                               command equals the command's float32 (both fields or one)
    ambiguous_equal            the command equals the previous one in both fields: no cycle can
                               say which of the two it echoes
    unmatched                  the gate changed every field that changed, or the command was
                               never latched inside the window (a later command took its place)

The legs reported (p50, p99, max, seconds), each on the node clock: scan stamp to receipt, scan
receipt to solve end, solve end to publish (these three need `/pnp/planner/timing`, absent from
older bags, and the report says so), and publish to the first safety cycle that echoes the command
(needs only `/drive_raw` and `/drive`).

Pure numpy on arrays; `read_chain_bag` is the mcap glue (excluded from coverage like bag_io).
Everything here is a measurement: no threshold, nothing is pass or fail.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

# Pre-registration files this tool's numbers are governed by (docs/notes/preregistration-pnp-v0.4*).
# The v0.4 constants file and addendum 1; both frozen. The command line checks them on every run.
V04_YAML = "evaluation/prereg/pnp-v0.4.yaml"
V04_SHA256 = "78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63"
ADDENDUM_YAML = "evaluation/prereg/pnp-v0.4-addendum-1.yaml"
ADDENDUM_SHA256 = "ada5a83675e5c843fb8e121a1190e4458f823b1ed2ff32a34a46c5fb4ff76824"

#: How long after a command's publish stamp a cycle may echo it (addendum 1: "to one cycle"; the
#: window only has to be longer than any real latch delay and shorter than the command stream's
#: memory). A latch later than this is reported as unmatched.
ECHO_WINDOW_S = 0.25

#: M-C3 measurement parameters (a measurement, no threshold): a commanded speed counts as held
#: when the command stays within the float32 of itself for STEADY_HOLD_S, and the steady wheel
#: speed it produced is the median of the last STEADY_TAIL_S of that hold.
STEADY_HOLD_S = 1.0
STEADY_TAIL_S = 0.5

STATUS_IDENTIFIED = "identified"
STATUS_EQUAL = "ambiguous_equal"
STATUS_UNMATCHED = "unmatched"

FINAL_NAMES = ("mean", "fallback", "all_collide")
LIMITER_NAMES = (
    "none",
    "cap",
    "clearance",
    "terminal",
    "gate",
    "lap_bound",
    "validity",
    "sample_choice",
)
COST_TERM_NAMES = (
    "progress",
    "collision",
    "clearance",
    "boundary",
    "lateral",
    "smoothness",
    "steer_rate",
    "terminal",
    "gate",
    "lap_trust",
    "lap_terminal",
)


def _f() -> NDArray[np.float64]:
    return np.zeros(0, dtype=np.float64)


def _i() -> NDArray[np.int64]:
    return np.zeros(0, dtype=np.int64)


@dataclass
class ChainInputs:
    """Everything the chain needs, on one time axis (seconds; header stamps and receive times may
    be on the bag axis or the epoch, the tool only subtracts values of the same kind)."""

    # /drive_raw: header stamp (the planner's publish time), receive time, speed, steering.
    cmd_stamp: NDArray[np.float64] = field(default_factory=_f)
    cmd_speed: NDArray[np.float64] = field(default_factory=_f)
    cmd_steer: NDArray[np.float64] = field(default_factory=_f)
    # /drive: the cycle stamp, speed, steering.
    cyc_stamp: NDArray[np.float64] = field(default_factory=_f)
    cyc_speed: NDArray[np.float64] = field(default_factory=_f)
    cyc_steer: NDArray[np.float64] = field(default_factory=_f)
    # /pnp/planner/timing (optional, empty on old bags).
    tim_stamp: NDArray[np.float64] = field(default_factory=_f)
    tim_scan_stamp: NDArray[np.float64] = field(default_factory=_f)
    tim_scan_receipt: NDArray[np.float64] = field(default_factory=_f)
    tim_solve_end: NDArray[np.float64] = field(default_factory=_f)
    tim_final: NDArray[np.int64] = field(default_factory=_i)
    tim_limiter: NDArray[np.int64] = field(default_factory=_i)
    # /safety/cycle (optional): cycle stamp, the latched command's header stamp (nan: none), age.
    sc_stamp: NDArray[np.float64] = field(default_factory=_f)
    sc_raw_stamp: NDArray[np.float64] = field(default_factory=_f)
    sc_age: NDArray[np.float64] = field(default_factory=_f)


def _f32(a: NDArray[np.float64]) -> NDArray[np.float32]:
    return np.asarray(a, dtype=np.float64).astype(np.float32)


@dataclass
class Echo:
    """Per command: how it was identified, and the first echoing cycle."""

    status: list[str]
    cycle: NDArray[np.int64]  # index into the cycle arrays, -1 for none
    latency_s: NDArray[np.float64]  # cycle stamp minus command stamp, nan for none
    source: str  # "echo" or "safety_cycle"

    def share_identified(self) -> float:
        n = len(self.status)
        return sum(1 for s in self.status if s == STATUS_IDENTIFIED) / n if n else float("nan")


def identify_echoes(inp: ChainInputs, window_s: float = ECHO_WINDOW_S) -> Echo:
    """The first safety cycle that echoes each command (module docstring). With `/safety/cycle`
    in the inputs the latched command of every cycle is exact and the echo is the first cycle that
    latched the command."""
    n = len(inp.cmd_stamp)
    cycle = np.full(n, -1, dtype=np.int64)
    latency = np.full(n, np.nan)
    status = [STATUS_UNMATCHED] * n
    if len(inp.sc_stamp) > 0:
        return _echo_from_safety_cycle(inp, window_s)
    cs = _f32(inp.cmd_speed)
    cd = _f32(inp.cmd_steer)
    ys = _f32(inp.cyc_speed)
    yd = _f32(inp.cyc_steer)
    stamps = np.asarray(inp.cyc_stamp, dtype=np.float64)
    for j in range(n):
        new_s = j == 0 or cs[j] != cs[j - 1]
        new_d = j == 0 or cd[j] != cd[j - 1]
        if not (new_s or new_d):
            status[j] = STATUS_EQUAL
            continue
        lo = int(np.searchsorted(stamps, inp.cmd_stamp[j], side="left"))
        hi = int(np.searchsorted(stamps, inp.cmd_stamp[j] + window_s, side="right"))
        for i in range(lo, hi):
            if (new_s and ys[i] == cs[j]) or (new_d and yd[i] == cd[j]):
                cycle[j] = i
                latency[j] = stamps[i] - inp.cmd_stamp[j]
                status[j] = STATUS_IDENTIFIED
                break
    return Echo(status, cycle, latency, "echo")


def _echo_from_safety_cycle(inp: ChainInputs, window_s: float) -> Echo:
    n = len(inp.cmd_stamp)
    cycle = np.full(n, -1, dtype=np.int64)
    latency = np.full(n, np.nan)
    status = [STATUS_UNMATCHED] * n
    stamps = np.asarray(inp.cyc_stamp, dtype=np.float64)
    raw = np.asarray(inp.sc_raw_stamp, dtype=np.float64)
    sc = np.asarray(inp.sc_stamp, dtype=np.float64)
    # A command's first latch: the first record whose latched header stamp equals it.
    order = np.argsort(inp.cmd_stamp, kind="stable")
    sorted_stamp = np.asarray(inp.cmd_stamp, dtype=np.float64)[order]
    seen: dict[int, int] = {}
    for k in range(len(sc)):
        if not np.isfinite(raw[k]):
            continue
        pos = int(np.searchsorted(sorted_stamp, raw[k], side="left"))
        if pos < n and sorted_stamp[pos] == raw[k]:
            seen.setdefault(int(order[pos]), k)
    for j, k in seen.items():
        lat = sc[k] - inp.cmd_stamp[j]
        if lat <= window_s:
            idx = int(np.searchsorted(stamps, sc[k], side="left"))
            cycle[j] = idx if idx < len(stamps) else -1
            latency[j] = lat
            status[j] = STATUS_IDENTIFIED
    return Echo(status, cycle, latency, "safety_cycle")


def _stats(values: NDArray[np.float64]) -> dict[str, Any]:
    v = values[np.isfinite(values)]
    if len(v) == 0:
        return {"n": 0, "p50": None, "p99": None, "max": None}
    return {
        "n": len(v),
        "p50": float(np.percentile(v, 50)),
        "p99": float(np.percentile(v, 99)),
        "max": float(np.max(v)),
    }


def chain_report(inp: ChainInputs, window_s: float = ECHO_WINDOW_S) -> dict[str, Any]:
    """M-C2: the legs, the share identified, the ambiguous commands, and (when the bag has the
    timing topic) the final sequence outcomes and speed limiters."""
    echo = identify_echoes(inp, window_s)
    n = len(echo.status)
    counts = {
        s: sum(1 for x in echo.status if x == s)
        for s in (STATUS_IDENTIFIED, STATUS_EQUAL, STATUS_UNMATCHED)
    }
    ambiguous = [
        {"command": j, "stamp": float(inp.cmd_stamp[j]), "why": echo.status[j]}
        for j in range(n)
        if echo.status[j] != STATUS_IDENTIFIED
    ]
    has_timing = len(inp.tim_stamp) > 0
    report: dict[str, Any] = {
        "latch_source": echo.source,
        "commands": n,
        "cycles": len(inp.cyc_stamp),
        "share_identified": echo.share_identified() if n else None,
        "status_counts": counts,
        "ambiguous": ambiguous,
        "legs_s": {"publish_to_echo": _stats(echo.latency_s)},
        "timing_topic": has_timing,
    }
    if echo.source == "safety_cycle":
        report["latch_note"] = (
            "receipt and latch taken from /safety/cycle (the latched command's header stamp and "
            "age), exact; the echo of /drive was not used"
        )
    else:
        report["latch_note"] = (
            "no /safety/cycle in this bag: the latch is bounded to one gate cycle by the first "
            "/drive that echoes the command (safety_node's own receive time is not recorded)"
        )
    if not has_timing:
        report["timing_note"] = (
            "no /pnp/planner/timing in this bag (recorded since brief 42): only the "
            "publish-to-echo leg and the echo identification are computed"
        )
        return report
    scan_leg = inp.tim_scan_receipt - inp.tim_scan_stamp
    have_scan = (inp.tim_scan_stamp > 0.0) & (inp.tim_scan_receipt > 0.0)
    solve_leg = inp.tim_solve_end - inp.tim_scan_receipt
    pub_leg = inp.tim_stamp - inp.tim_solve_end
    report["legs_s"]["scan_stamp_to_receipt"] = _stats(np.where(have_scan, scan_leg, np.nan))
    report["legs_s"]["scan_receipt_to_solve_end"] = _stats(np.where(have_scan, solve_leg, np.nan))
    report["legs_s"]["solve_end_to_publish"] = _stats(
        np.where(inp.tim_solve_end > 0, pub_leg, np.nan)
    )
    nt = len(inp.tim_stamp)
    report["final_sequence"] = {
        name: int(np.sum(inp.tim_final == k)) for k, name in enumerate(FINAL_NAMES)
    }
    report["speed_limiter"] = {
        name: int(np.sum(inp.tim_limiter == k)) for k, name in enumerate(LIMITER_NAMES)
    }
    report["timing_rows"] = nt
    return report


@dataclass
class Latched:
    """The command latched at every recorded safety cycle."""

    stamp: NDArray[np.float64]  # cycle stamps
    steer: NDArray[np.float64]
    speed: NDArray[np.float64]
    age: NDArray[np.float64]  # cycle stamp minus the latched command's stamp, inf before any
    command: NDArray[np.int64]  # index of the latched command, -1 before any
    ambiguous: list[int]  # cycle indices where the latch is not known from the echo
    source: str


def latched_commands(inp: ChainInputs, window_s: float = ECHO_WINDOW_S) -> Latched:
    """The latched command of every cycle, for the limiter replay M-T2.

    With `/safety/cycle`: exactly the command whose header stamp the record carries, its age from
    the record. With the echo: a command is known to be latched from the cycle that echoes it
    until the next identified command's echo cycle. A cycle is listed as ambiguous when a command
    that could not be identified (equal to the previous one, or never matched) was published after
    the latched one and no later than the cycle: it may have been latched there. For those cycles
    the command is the latest one published at or before the cycle stamp (the replay of the M-T
    code), and they are listed so the replay can be read with them in view."""
    echo = identify_echoes(inp, window_s)
    stamps = np.asarray(inp.cyc_stamp, dtype=np.float64)
    nc = len(stamps)
    command = np.full(nc, -1, dtype=np.int64)
    ambiguous: list[int] = []
    n = len(inp.cmd_stamp)
    if echo.source == "safety_cycle":
        raw = np.asarray(inp.sc_raw_stamp, dtype=np.float64)
        sc = np.asarray(inp.sc_stamp, dtype=np.float64)
        order = np.argsort(inp.cmd_stamp, kind="stable")
        sorted_stamp = np.asarray(inp.cmd_stamp, dtype=np.float64)[order]
        age = np.full(nc, np.inf)
        for i in range(nc):
            k = int(np.searchsorted(sc, stamps[i], side="left"))
            if k >= len(sc) or sc[k] != stamps[i] or not np.isfinite(raw[k]):
                if k >= len(sc) or sc[k] != stamps[i]:
                    ambiguous.append(i)  # no record for this cycle
                continue
            pos = int(np.searchsorted(sorted_stamp, raw[k], side="left"))
            if pos < n and sorted_stamp[pos] == raw[k]:
                command[i] = int(order[pos])
                age[i] = float(inp.sc_age[k]) if inp.sc_age[k] >= 0 else np.inf
            else:
                ambiguous.append(i)  # the latched command is not in the bag
    else:
        # Identified commands by echo cycle; the latched one at cycle i is the identified command
        # with the largest echo cycle at or before i.
        latest = np.full(nc, -1, dtype=np.int64)
        marks = sorted(
            (int(echo.cycle[j]), j) for j in range(n) if echo.status[j] == STATUS_IDENTIFIED
        )
        m = 0
        cur = -1
        for i in range(nc):
            while m < len(marks) and marks[m][0] <= i:
                cur = marks[m][1]
                m += 1
            latest[i] = cur
        unidentified = [j for j in range(n) if echo.status[j] != STATUS_IDENTIFIED]
        ustamps = np.asarray([inp.cmd_stamp[j] for j in unidentified], dtype=np.float64)
        for i in range(nc):
            cur = int(latest[i])
            lo_t = inp.cmd_stamp[cur] if cur >= 0 else -np.inf
            if len(ustamps):
                hit = np.any((ustamps > lo_t) & (ustamps <= stamps[i]))
            else:
                hit = False
            if cur < 0 or hit:
                ambiguous.append(i)
                # fall back to the latest command published at or before the cycle
                k = int(np.searchsorted(np.asarray(inp.cmd_stamp), stamps[i], side="right")) - 1
                command[i] = k if k >= 0 else -1
            else:
                command[i] = cur
        age = np.full(nc, np.inf)
    have = command >= 0
    steer = np.zeros(nc, dtype=np.float64)
    speed = np.zeros(nc, dtype=np.float64)
    steer[have] = np.asarray(inp.cmd_steer)[command[have]]
    speed[have] = np.asarray(inp.cmd_speed)[command[have]]
    if echo.source != "safety_cycle":
        age[have] = stamps[have] - np.asarray(inp.cmd_stamp)[command[have]]
    return Latched(stamps, steer, speed, age, command, ambiguous, echo.source)


def command_quantisation(
    cmd_stamp: NDArray[np.float64],
    cmd_speed: NDArray[np.float64],
    odom_t: NDArray[np.float64],
    odom_v: NDArray[np.float64],
    hold_s: float = STEADY_HOLD_S,
    tail_s: float = STEADY_TAIL_S,
) -> dict[str, Any]:
    """M-C3: the distinct commanded speeds (float32) and the steady wheel speed each produced.

    A hold is a run of commands with the same float32 speed lasting at least `hold_s` (first to
    last stamp); the steady wheel speed of a hold is the median of the wheel speed over its last
    `tail_s`. Reported per distinct commanded speed: commands, holds, and the median, min and max
    of the holds' steady wheel speeds. `odom_t` must be on the clock of `cmd_stamp`."""
    speeds = _f32(cmd_speed)
    distinct = np.unique(speeds)
    holds: dict[float, list[float]] = {float(s): [] for s in distinct}
    n = len(speeds)
    j = 0
    while j < n:
        k = j
        while k + 1 < n and speeds[k + 1] == speeds[j]:
            k += 1
        t0, t1 = float(cmd_stamp[j]), float(cmd_stamp[k])
        if t1 - t0 >= hold_s:
            sel = (odom_t >= t1 - tail_s) & (odom_t <= t1)
            if np.any(sel):
                holds[float(speeds[j])].append(float(np.median(odom_v[sel])))
        j = k + 1
    rows = []
    for s in distinct:
        h = holds[float(s)]
        rows.append(
            {
                "commanded_mps": float(s),
                "commands": int(np.sum(speeds == s)),
                "holds": len(h),
                "steady_wheel_mps_median": float(np.median(h)) if h else None,
                "steady_wheel_mps_min": float(np.min(h)) if h else None,
                "steady_wheel_mps_max": float(np.max(h)) if h else None,
            }
        )
    return {
        "distinct_commanded_speeds": len(distinct),
        "hold_s": hold_s,
        "tail_s": tail_s,
        "rows": rows,
    }


# ------------------------------------------------------------------------------- bag glue (mcap)

DRIVE_RAW = "/drive_raw"
DRIVE = "/drive"
TIMING = "/pnp/planner/timing"
SAFETY_CYCLE = "/safety/cycle"
ODOM = "/odom/wheel"


def _t(stamp: Any) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def read_chain_bag(  # pragma: no cover - mcap glue, run on the car's bags
    bag: Path,
) -> tuple[ChainInputs, NDArray[np.float64], NDArray[np.float64]]:
    """(inputs, wheel receive times, wheel speeds) of a bag (a .mcap file or a directory). The
    wheel arrays are on the clock of the header stamps only if the recorder clock equals the
    node clock (it does on the car: one host); they feed M-C3 only."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    files = [bag] if bag.is_file() else sorted(bag.glob("*.mcap"))
    if not files:
        raise FileNotFoundError(f"no .mcap file in {bag}")
    rows: dict[str, list[tuple[Any, ...]]] = {
        k: [] for k in (DRIVE_RAW, DRIVE, TIMING, SAFETY_CYCLE, ODOM)
    }
    for file in files:
        with file.open("rb") as stream:
            reader = make_reader(stream, decoder_factories=[DecoderFactory()])
            for _s, channel, message, msg in reader.iter_decoded_messages(topics=list(rows)):
                topic = channel.topic
                if topic in (DRIVE_RAW, DRIVE):
                    rows[topic].append(
                        (
                            _t(msg.header.stamp),
                            float(msg.drive.speed),
                            float(msg.drive.steering_angle),
                        )
                    )
                elif topic == TIMING:
                    rows[topic].append(
                        (
                            _t(msg.header.stamp),
                            float(msg.scan_stamp_s),
                            float(msg.scan_receipt_s),
                            float(msg.solve_end_s),
                            int(msg.final_sequence),
                            int(msg.speed_limiter),
                        )
                    )
                elif topic == SAFETY_CYCLE:
                    raw = _t(msg.drive_raw_stamp)
                    rows[topic].append(
                        (
                            _t(msg.stamp),
                            raw if msg.has_command and raw > 0 else float("nan"),
                            float(msg.drive_raw_age_s),
                        )
                    )
                elif topic == ODOM:
                    rows[topic].append((message.log_time / 1e9, float(msg.twist.twist.linear.x)))
    return inputs_from_rows(rows)


def inputs_from_rows(
    rows: dict[str, list[tuple[Any, ...]]],
) -> tuple[ChainInputs, NDArray[np.float64], NDArray[np.float64]]:
    def col(topic: str, i: int, dtype: Any = np.float64) -> Any:
        return np.array([r[i] for r in rows.get(topic, [])], dtype=dtype)

    inp = ChainInputs(
        cmd_stamp=col(DRIVE_RAW, 0),
        cmd_speed=col(DRIVE_RAW, 1),
        cmd_steer=col(DRIVE_RAW, 2),
        cyc_stamp=col(DRIVE, 0),
        cyc_speed=col(DRIVE, 1),
        cyc_steer=col(DRIVE, 2),
        tim_stamp=col(TIMING, 0),
        tim_scan_stamp=col(TIMING, 1),
        tim_scan_receipt=col(TIMING, 2),
        tim_solve_end=col(TIMING, 3),
        tim_final=col(TIMING, 4, np.int64),
        tim_limiter=col(TIMING, 5, np.int64),
        sc_stamp=col(SAFETY_CYCLE, 0),
        sc_raw_stamp=col(SAFETY_CYCLE, 1),
        sc_age=col(SAFETY_CYCLE, 2),
    )
    return inp, col(ODOM, 0), col(ODOM, 1)


# ------------------------------------------------------------------------------------------ CLI


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require_prereg(repo_root: Path) -> None:
    """Refuse to produce a number the frozen pre-registration files do not govern yet: they must
    be on this checkout with the registered SHA-256."""
    for rel, want in ((V04_YAML, V04_SHA256), (ADDENDUM_YAML, ADDENDUM_SHA256)):
        p = repo_root / rel
        if not p.is_file():
            raise SystemExit(
                f"{rel} is not on this checkout (PR 186 / PR 189 not merged): no number it "
                "governs is produced"
            )
        got = file_sha256(p)
        if got != want:
            raise SystemExit(f"{rel}: SHA-256 {got} is not the registered {want}")


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="planner to safety timing chain of a bag (M-C2, M-C3)")
    ap.add_argument("bag", type=Path)
    ap.add_argument("--json", type=Path, help="write the report here")
    ap.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[4])
    args = ap.parse_args(argv)
    require_prereg(args.repo_root)
    inp, odom_t, odom_v = read_chain_bag(args.bag)
    report = {
        "bag": args.bag.name,
        "prereg": {V04_YAML: V04_SHA256, ADDENDUM_YAML: ADDENDUM_SHA256},
        "m_c2": chain_report(inp),
        "m_c3": command_quantisation(inp.cmd_stamp, inp.cmd_speed, odom_t, odom_v),
    }
    text = json.dumps(report, indent=2)
    if args.json:
        args.json.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
