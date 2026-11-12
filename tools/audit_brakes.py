#!/usr/bin/env python3
"""Classify every TTC engagement of a recorded run and measure how the car really stopped.

    uv run --project tools python tools/audit_brakes.py \\
        /Users/cameronjim/code/car-data/bags/2026-11-12T03-04-56_car_teleop \\
        --window 1733 1925 --json audit.json

The method is stated, and was committed, before the first run: tools/safety_replay/brake_audit.py
(module docstring). Needs safety_replay_cli only for the vehicle geometry (`--describe`); nothing
here talks to a car. Prints markdown tables: the TTC engagements with the counterfactual verdict,
the measured slip and plant, and the measured stopping response.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import replay_safety
from follower_replay.bag import bag_origin_s, read_bag
from safety_replay import brake_audit as ba
from safety_replay.counterfactual import BodyBox
from safety_replay.motion import Mount, car_motion, static_self_returns
from safety_replay.stream import describe_cli, find_cli

_SCANS: list = []
_MOUNT: Mount | None = None
_IGNORE: frozenset | None = None


def _init(scans, mount, ignore):
    global _SCANS, _MOUNT, _IGNORE
    _SCANS, _MOUNT, _IGNORE = scans, mount, ignore


def _pair(i: int):
    a, c = _SCANS[i - 1], _SCANS[i]
    dt = c.stamp_s - a.stamp_s
    if not 0.05 < dt < 0.2:
        return None
    try:
        x, y, th, rms = car_motion(a, c, _MOUNT, ignore=_IGNORE)
    except ValueError:
        return None
    return (a.stamp_s, c.stamp_s, x, y, th, rms)


def scan_pairs(scans, mount, ignore, processes: int = 8) -> np.ndarray:
    """Scan matching of every consecutive pair (t_a, t_b, x, y, heading change, rms)."""
    with Pool(processes, initializer=_init, initargs=(scans, mount, ignore)) as pool:
        rows = [r for r in pool.map(_pair, range(1, len(scans)), chunksize=10) if r]
    return np.array(rows)


def _fmt(v: float | None, digits: int = 2) -> str:
    if v is None or (isinstance(v, float) and (math.isnan(v) or math.isinf(v))):
        return "-" if v is None or math.isnan(v) else ("inf" if v > 0 else "-inf")
    return f"{v:.{digits}f}"


def command_drops(drive, odom_t, odom_v, min_speed: float = 0.9):
    """Times the /drive output speed falls by at least 0.5 m/s within 0.1 s while the wheel speed
    is at least min_speed: a stop or a hard slow-down the car was asked to make."""
    t = np.array([d.log_time_s for d in drive])
    u = np.array([d.speed_mps for d in drive])
    out: list[float] = []
    last = -1.0
    for i in range(5, len(t)):
        if t[i] - last < 1.0:
            continue
        j = np.searchsorted(t, t[i] - 0.1)
        if u[j] - u[i] >= 0.5:
            v = float(np.interp(t[i], odom_t, odom_v))
            if v >= min_speed:
                out.append(float(t[i]))
                last = float(t[i])
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument(
        "--window",
        nargs=2,
        type=float,
        metavar=("START", "END"),
        required=True,
        help="seconds from the first message",
    )
    ap.add_argument("--cli", default=None)
    ap.add_argument("--json", default=None)
    ap.add_argument("--processes", type=int, default=8)
    args = ap.parse_args()

    cli = find_cli(args.cli)
    described = describe_cli(cli)
    mount, body = replay_safety.geometry(described)
    wheelbase = float(described["wheelbase_m"])
    origin = bag_origin_s(args.bag)
    # A little around the window, so the scan before the first engagement and the second after the
    # last are there.
    bag = read_bag(args.bag, window_s=(args.window[0] - 2.0, args.window[1] + 2.0))
    streams = ba.Streams.of(bag.scans, bag.drive_raw, bag.drive, bag.odom)
    t_start = origin + args.window[0]
    t_end = origin + args.window[1]

    ignore = static_self_returns(bag.scans, mount, body)
    pairs = scan_pairs(bag.scans, mount, ignore, args.processes)
    # Wheel speed and steering over each pair, for the slip table.
    ot = np.array([ba.arrival(o) for o in bag.odom])
    ov = np.array([o.speed_mps for o in bag.odom])
    dt_ = np.array([ba.arrival(d) for d in bag.drive])
    ds = np.array([d.steering_rad for d in bag.drive])
    wheel = np.array(
        [
            ov[(ot >= a) & (ot < b)].mean() if ((ot >= a) & (ot < b)).any() else np.nan
            for a, b in zip(pairs[:, 0], pairs[:, 1], strict=True)
        ]
    )
    steer = np.array(
        [
            ds[(dt_ >= a - 0.1) & (dt_ < b - 0.1)].mean()
            if ((dt_ >= a - 0.1) & (dt_ < b - 0.1)).any()
            else 0.0
            for a, b in zip(pairs[:, 0], pairs[:, 1], strict=True)
        ]
    )
    sel = pairs[:, 0] >= t_start
    slip = ba.build_slip_table(pairs[sel], wheel[sel], steer[sel], wheelbase)
    print("## Slip table (scan matching against wheel odometry and the steering command)\n")
    print(
        "| |steering| below (rad) | pairs | true speed / wheel speed | true curvature / kinematic |"
    )
    print("|---|---|---|---|")
    for e, n, r, k in zip(
        slip.edges, slip.counts, slip.speed_ratio, slip.curvature_ratio, strict=True
    ):
        print(f"| {e:.1f} | {n} | {r:.2f} | {k:.2f} |")

    # Plant fit on the whole drive in the window.
    m = (dt_ >= t_start) & (dt_ <= t_end)
    cmd = np.array([d.speed_mps for d in bag.drive])
    # the fit input is the gate OUTPUT (/drive) against the wheel speed
    delay, tau, aa, ad, gain = ba.fit_speed_plant(dt_[m], cmd[m], np.interp(dt_[m], ot, ov))
    print(
        f"\nPlant fit (/drive speed -> wheel speed): delay {delay:.2f} s, tau {tau:.2f} s, "
        f"max accel {aa:.1f}, max decel {ad:.1f} (grid edge means not identified), command gain {gain:.2f}"
    )
    model = ba.AuditModel(
        BodyBox(body.front_x_m, body.rear_x_m, body.half_width_m),
        mount,
        {
            "delay_s": delay,
            "tau_s": tau,
            "max_accel": aa,
            "max_decel": ad,
            "cmd_gain": gain,
            "steer_tau_s": 0.05,
        },
        slip,
        wheelbase,
        ignore,
    )

    engagements = [e for e in ba.ttc_engagements(bag.events) if t_start <= e.t_s <= t_end]
    rows = [ba.analyse_engagement(e, streams, model) for e in engagements]
    brakes = [r for r in rows if r.engagement.kind == "brake"]
    advis = [r for r in rows if r.engagement.kind == "advisory"]
    print(
        f"\n## TTC engagements in the window: {len(rows)} ({len(brakes)} brakes, "
        f"{len(advis)} advisories)\n"
    )
    print(
        "| # | t (s) | kind | corner | request m/s, rad | wheel m/s | true m/s | body clear. m | "
        "CF min clear. m | hit in s | class | sens. | real min clear. m | dur. s |"
    )
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for k, r in enumerate(rows, 1):
        print(
            f"| {k} | {r.engagement.t_s - t_start + args.window[0]:.2f} | {r.engagement.kind} | "
            f"{'corner' if r.corner else 'straight'} | {_fmt(r.request_speed_mps)}, "
            f"{_fmt(r.request_steering_rad)} | {_fmt(r.wheel_speed_mps)} | {_fmt(r.true_speed_mps)} | "
            f"{_fmt(r.body_clearance_m)} | {_fmt(r.cf_min_clearance_m, 3)} | {_fmt(r.cf_hit_s)} | "
            f"{r.classification or '-'} | {'yes' if r.sensitive else ''} | "
            f"{_fmt(r.real_min_clearance_m, 3)} | {_fmt(r.engagement.duration_s)} |"
        )
    for label, group in (("brakes", brakes),):
        print(f"\n### Brake classes ({len(group)} {label})\n")
        print("| | all | corner | straight |")
        print("|---|---|---|---|")
        for cls in ("necessary", "marginal", "false alarm"):
            allc = sum(1 for r in group if r.classification == cls)
            cor = sum(1 for r in group if r.classification == cls and r.corner)
            st = allc - cor
            print(f"| {cls} | {allc} | {cor} | {st} |")
        print(
            f"\nClass changes under a bracketing plant (sensitive): "
            f"{sum(1 for r in group if r.sensitive)} of {len(group)}"
        )
    # advisories: escalation within 1 s
    brake_times = [r.engagement.t_s for r in brakes]
    for r in advis:
        nxt = [b - r.engagement.t_s for b in brake_times if 0.0 <= b - r.engagement.t_s <= 1.0]
        r.brake_followed_s = nxt[0] if nxt else None
    print(
        f"\nAdvisories followed by a brake within 1 s: "
        f"{sum(1 for r in advis if r.brake_followed_s is not None)} of {len(advis)}; "
        f"advisories with a smallest real clearance below 0.04 m in the next second: "
        f"{sum(1 for r in advis if r.real_min_clearance_m < 0.04)}"
    )

    # Stopping.
    print("\n## Measured stopping (wheel speed after a trip)\n")
    print(
        "| t (s) | v0 m/s | true v0 | latency s | mean decel | peak decel | v min | to v min s | "
        "wheel travel m | scan travel m | model travel m |"
    )
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    stops = []
    for r in brakes:
        s = ba.stop_response(ot, ov, r.engagement.t_s)
        if not s:
            continue
        ts = r.scan_stamp_s
        tmin = r.engagement.t_s + s.get("t_min_s", 0.0)
        st_travel = ba.scan_travel_m(streams.scans, streams.scan_times, mount, ts, tmin, ignore)
        model_m = ba.model_stop_travel_m(
            s["v0"], described["stop_latency_s"], described["stop_deceleration_mps2"]
        )
        s["scan_travel_m"] = st_travel
        s["model_travel_m"] = model_m
        s["true_v0"] = r.true_speed_mps
        stops.append((r, s))
        print(
            f"| {r.engagement.t_s - t_start + args.window[0]:.2f} | {_fmt(s['v0'])} | "
            f"{_fmt(r.true_speed_mps)} | {_fmt(s.get('latency_s'))} | "
            f"{_fmt(s.get('mean_decel_mps2'))} | {_fmt(s.get('peak_decel_mps2'))} | "
            f"{_fmt(s.get('v_min'))} | {_fmt(s.get('t_min_s'))} | {_fmt(s.get('wheel_travel_m'), 3)} | "
            f"{_fmt(st_travel, 3)} | {_fmt(model_m, 3)} |"
        )
    drops = command_drops(bag.drive, ot, ov)
    print(f"\n## Hard slow-downs from at least 0.9 m/s wheel speed (command drops): {len(drops)}\n")
    print(
        "| t (s) | v0 m/s | latency s | mean decel | peak decel | v min | scan travel m | "
        "model travel m |"
    )
    print("|---|---|---|---|---|---|---|---|")
    drop_rows = []
    for td in drops:
        if not t_start <= td <= t_end:
            continue
        s = ba.stop_response(ot, ov, td)
        if not s or "latency_s" not in s:
            continue
        tmin = td + s["t_min_s"]
        ts_scan = streams.scans[ba.last_before(streams.scan_times, td)].stamp_s
        s["scan_travel_m"] = ba.scan_travel_m(
            streams.scans, streams.scan_times, mount, ts_scan, tmin, ignore
        )
        s["model_travel_m"] = ba.model_stop_travel_m(
            s["v0"], described["stop_latency_s"], described["stop_deceleration_mps2"]
        )
        drop_rows.append((td, s))
        print(
            f"| {td - t_start + args.window[0]:.2f} | {_fmt(s['v0'])} | {_fmt(s['latency_s'])} | "
            f"{_fmt(s.get('mean_decel_mps2'))} | {_fmt(s.get('peak_decel_mps2'))} | "
            f"{_fmt(s['v_min'])} | {_fmt(s['scan_travel_m'], 3)} | {_fmt(s['model_travel_m'], 3)} |"
        )
    if args.json:
        payload = {
            "slip": {
                "edges": slip.edges,
                "speed_ratio": slip.speed_ratio,
                "curvature_ratio": slip.curvature_ratio,
                "counts": slip.counts,
            },
            "plant": model.plant_base,
            "rows": [
                {
                    "t_s": r.engagement.t_s - t_start + args.window[0],
                    "kind": r.engagement.kind,
                    "detail": r.engagement.detail,
                    "corner": r.corner,
                    "request_steering_rad": r.request_steering_rad,
                    "request_speed_mps": r.request_speed_mps,
                    "wheel_speed_mps": r.wheel_speed_mps,
                    "true_speed_mps": r.true_speed_mps,
                    "body_clearance_m": r.body_clearance_m,
                    "cf_min_clearance_m": r.cf_min_clearance_m,
                    "cf_hold_clearance_m": r.cf_hold_clearance_m,
                    "cf_agile_clearance_m": r.cf_agile_clearance_m,
                    "cf_hit_s": r.cf_hit_s,
                    "classification": r.classification,
                    "sensitive": r.sensitive,
                    "real_min_clearance_m": r.real_min_clearance_m,
                    "duration_s": r.engagement.duration_s,
                }
                for r in rows
            ],
            "stops": [{"t_s": r.engagement.t_s - t_start + args.window[0], **s} for r, s in stops],
            "drops": [{"t_s": t - t_start + args.window[0], **s} for t, s in drop_rows],
        }
        Path(args.json).write_text(json.dumps(payload, indent=1, default=float))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
