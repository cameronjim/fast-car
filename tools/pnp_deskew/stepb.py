"""M-DSK step B: the result, scored ONCE on the `heldout` role (pre-registration section 4).

Items, all on the held-out pairs R8 against R3 (ccw) and R7b against R4 (cw), raw against the
hypothesis step A chose:

1. C(deskewed) / C(raw) pooled over the two pairs, fast corner scans: at most 0.90 and the upper
   end of the paired block bootstrap interval below 1.00.
2. The same ratio of medians over the fast run's scans that are not fast corner scans: at most
   1.02.
3. `geometric-v1` on A-run (every moving scan of the held-out segments of the split) with deskewed
   scans: mean intrusion at most raw + 0.002, clearance deficit p90 at most raw + 0.005 m. The
   definitions are racer_pnp_perception.eval_bags' (`clearance_deficit`) and
   `TrackPerceiver.process` (the intrusion fraction), as in GP1.
4. Replay of R0 to R8 through the planner (planner_replay_cli, the real planner) with deskewed
   scans: no tick in NO_SCAN or FAULT that the raw replay does not have at the same tick.
5. Node latency p99 under 3 ms on this machine, timing `deskew_scan` (core and rebin) over 2000
   real scans. (Under 5 ms on the Orin is measured at the car, brief 34.)
"""

from __future__ import annotations

import json
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from yaw_rate import matching, vehicle
from yaw_rate import run as yaw_run

from pnp_deskew import bagio, metric, replay

# Pre-registration section 4 and evaluation/prereg/pnp-v0.3.yaml block deskew.
MAX_RATIO_FAST_CORNER = 0.90
MAX_RATIO_OTHER = 1.02
BOOTSTRAP_UPPER_BELOW = 1.00
# Item 3 (section 4).
MAX_INTRUSION_EXCESS = 0.002
MAX_DEFICIT_P90_EXCESS_M = 0.005
# Item 5 (section 4): dev machine.
MAX_LATENCY_P99_MS = 3.0
LATENCY_SCANS = 2000
# GP1's moving-scan definition (pre-registration v0.2 section 2): wheel speed at least 0.2 m/s.
MOVING_MPS = 0.2
STATE_NO_SCAN = 3
STATE_FAULT = 6
REPO_ROOT = Path(__file__).resolve().parents[2]
DYNAMICS_BUNDLE = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-car-2026-11-12"


def _clean(obj: Any) -> Any:
    if isinstance(obj, dict):
        return {k: _clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_clean(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return _clean(obj.tolist())
    if isinstance(obj, (float, np.floating)):
        return None if not np.isfinite(float(obj)) else float(obj)
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.bool_):
        return bool(obj)
    return obj


# -- items 1 and 2 -------------------------------------------------------------------------------------


def items_1_2(
    ev: Any, cfg: dict[str, Any], split: dict[str, Any], chosen: str, workers: int
) -> dict[str, Any]:
    names = tuple(
        {p["fast"] for p in cfg["result_pairs"]} | {p["slow"] for p in cfg["result_pairs"]}
    )
    specs = {s.name: s for s in yaw_run.split_segments(split, names)}
    veh = vehicle.load_vehicle()
    bag = {specs[n].bag for n in names}
    if len(bag) != 1:
        raise SystemExit("result pairs span more than one bag")
    rec = ev.recording(bag.pop())
    ignore = matching.self_returns(rec, veh)
    out: dict[str, Any] = {"pairs": {}}
    arrays: dict[str, dict[str, list[np.ndarray]]] = {
        w: {"raw": [], "dsk": [], "block": []} for w in ("corner", "other")
    }
    for k, pair in enumerate(cfg["result_pairs"]):
        fast, slow = specs[pair["fast"]], specs[pair["slow"]]
        key = f"{fast.name}/{slow.name}"
        out["pairs"][key] = {}
        for which in ("corner", "other"):
            raw = ev.pair_scores(
                rec, fast, slow, "raw", which, cfg["fast_corner"], ignore, veh, workers
            )
            dsk = ev.pair_scores(
                rec, fast, slow, chosen, which, cfg["fast_corner"], ignore, veh, workers
            )
            assert np.array_equal(raw["scan_index"], dsk["scan_index"])
            block = k * 100000 + np.floor((raw["t_s"] - fast.t_from) / metric.BOOTSTRAP_BLOCK_S)
            arrays[which]["raw"].append(raw["rms_m"])
            arrays[which]["dsk"].append(dsk["rms_m"])
            arrays[which]["block"].append(block.astype(np.int64))
            out["pairs"][key][which] = {
                "scans": len(raw["rms_m"]),
                "scored_raw": int(np.isfinite(raw["rms_m"]).sum()),
                "scored_deskewed": int(np.isfinite(dsk["rms_m"]).sum()),
                "C_raw_m": metric.median_ignoring_nan(raw["rms_m"]),
                "C_deskewed_m": metric.median_ignoring_nan(dsk["rms_m"]),
                "deskewed_passed_through": dsk["passed_through"],
            }
    res: dict[str, Any] = {}
    for which in ("corner", "other"):
        raw_all = np.concatenate(arrays[which]["raw"])
        dsk_all = np.concatenate(arrays[which]["dsk"])
        blk = np.concatenate(arrays[which]["block"])
        both = np.isfinite(raw_all) & np.isfinite(dsk_all)
        boot = metric.block_bootstrap_median_ratio(dsk_all, raw_all, blk)
        res[which] = {
            "scans": len(raw_all),
            "paired_scored": int(both.sum()),
            "C_raw_m": metric.median_ignoring_nan(raw_all[both]),
            "C_deskewed_m": metric.median_ignoring_nan(dsk_all[both]),
            "ratio": boot.ratio,
            "ci95_lo": boot.lo,
            "ci95_hi": boot.hi,
            "blocks": boot.blocks,
        }
    c, o = res["corner"], res["other"]
    out["item1"] = {
        **c,
        "max_ratio": MAX_RATIO_FAST_CORNER,
        "passed": bool(
            c["ratio"] <= MAX_RATIO_FAST_CORNER and c["ci95_hi"] < BOOTSTRAP_UPPER_BELOW
        ),
    }
    out["item2"] = {
        **o,
        "max_ratio": MAX_RATIO_OTHER,
        "passed": bool(o["ratio"] <= MAX_RATIO_OTHER),
    }
    return out


# -- item 3 ----------------------------------------------------------------------------------------------


def item_3(ev: Any, split: dict[str, Any], chosen: str) -> dict[str, Any]:
    from pnp_replay import perception

    sys.path.insert(0, str(REPO_ROOT / "training" / "racer_pnp_perception" / "src"))
    from racer_pnp_perception.eval_bags import clearance_deficit
    from racer_track_perception.preprocess import scan_to_points

    perceiver = perception.build_perceiver("geometric")
    live = perceiver.vehicle
    acc: dict[str, dict[str, list[Any]]] = {
        c: {"intrusion": [], "deficit": []} for c in ("raw", chosen)
    }
    segments = 0
    for bag, entry in split["bags"].items():
        heldout = [s for s in entry.get("segments", []) if s.get("perception") == "heldout"]
        if not heldout:
            continue
        rec = ev.recording(bag)
        for seg in heldout:
            t = rec["scan_t"]
            idx = np.nonzero((t >= float(seg["from"])) & (t <= float(seg["to"])))[0].astype(
                np.int64
            )
            v, _ = replay.wheel_yaw_rate_at_scans(rec, idx)
            idx = idx[np.abs(v) > MOVING_MPS]
            if len(idx) == 0:
                continue
            segments += 1
            for cond in acc:
                d = ev.process(rec, idx, cond)
                for k, i in enumerate(idx):
                    args = (
                        d.ranges[k],
                        float(rec["scan_angle_min"][i]),
                        float(rec["scan_angle_increment"][i]),
                        float(rec["scan_range_min"][i]),
                        float(rec["scan_range_max"][i]),
                    )
                    track = perceiver.impl.process(*args)
                    pts = scan_to_points(*args, live.mount, live.body)
                    acc[cond]["intrusion"].append(track.intrusion_fraction)
                    acc[cond]["deficit"].append(clearance_deficit(track, pts))
    summary: dict[str, Any] = {"segments": segments}
    for cond, a in acc.items():
        intr = np.array(a["intrusion"], dtype=np.float64)
        cat = np.concatenate([np.zeros(0), *a["deficit"]])
        summary[cond] = {
            "scans": len(intr),
            "intrusion_mean": float(np.nanmean(intr)),
            "deficit_p90_m": float(np.percentile(cat, 90)) if len(cat) else float("nan"),
        }
    raw, dsk = summary["raw"], summary[chosen]
    summary["intrusion_excess"] = dsk["intrusion_mean"] - raw["intrusion_mean"]
    summary["deficit_p90_excess_m"] = dsk["deficit_p90_m"] - raw["deficit_p90_m"]
    summary["passed"] = bool(
        summary["intrusion_excess"] <= MAX_INTRUSION_EXCESS
        and summary["deficit_p90_excess_m"] <= MAX_DEFICIT_P90_EXCESS_M
    )
    return summary


# -- item 4 ----------------------------------------------------------------------------------------------


def item_4(
    ev: Any, split: dict[str, Any], chosen: str, out_dir: Path, cli_path: str | None, bundle: Path
) -> dict[str, Any]:
    from pnp_replay import bag as bagmod
    from pnp_replay import perception, planner_stream

    bag_name = "2026-11-13T22-20-18_car_teleop"
    names = tuple(f"R{i}" for i in range(9))
    specs = yaw_run.split_segments(split, names)
    caps = {
        s["name"]: float(s["cap_mps"])
        for s in split["bags"][bag_name]["segments"]
        if s["name"] in names
    }
    base = bagmod.load_recording(ev.BAGS_ROOT / bag_name, out_dir / f"{bag_name}.recording.npz")
    mine = ev.recording(bag_name)
    if not np.allclose(base["scan_t"], mine["scan_t"]):
        raise SystemExit("the replay recording and the deskew recording disagree on the scans")
    cli = planner_stream.find_cli(cli_path)
    perceiver = perception.build_perceiver("geometric")
    result: dict[str, Any] = {"segments": {}}
    new_states: list[int] = []
    with tempfile.TemporaryDirectory(prefix="deskew_replay_", dir=out_dir) as tmp:
        for spec in specs:
            states: dict[str, np.ndarray] = {}
            for cond in ("raw", chosen):
                rec = {k: np.array(v, copy=True) for k, v in base.items()}
                lo = spec.t_from - planner_stream.PREROLL_S
                idx = np.nonzero((mine["scan_t"] >= lo) & (mine["scan_t"] <= spec.t_to))[0].astype(
                    np.int64
                )
                d = ev.process(mine, idx, cond)
                scan_f = np.stack(
                    [bagmod.scan_ranges(rec, i) for i in range(len(rec["scan_t"]))]
                ).astype(np.float32)
                scan_f[idx] = d.ranges
                rec["scan_f"] = scan_f
                stamps = np.array(rec["scan_stamp"], copy=True)
                stamps[idx] += d.offset_s
                rec["scan_stamp"] = stamps
                tracks = perception.run_perceiver(perceiver, rec, idx)
                lines = planner_stream.stream_lines(
                    rec, spec.t_from, spec.t_to, tracks_mode="replay", tracks=tracks
                )
                stream = Path(tmp) / f"{spec.name}.{cond.replace('/', '_')}.txt"
                csv = stream.with_suffix(".csv")
                planner_stream.write_stream(lines, stream)
                rows = planner_stream.run_cli(cli, bundle, caps[spec.name], {}, stream, csv)
                states[cond] = np.asarray(rows["state"])
            raw_s, dsk_s = states["raw"], states[chosen]
            n = min(len(raw_s), len(dsk_s))
            bad = np.isin(dsk_s[:n], (STATE_NO_SCAN, STATE_FAULT)) & ~np.isin(
                raw_s[:n], (STATE_NO_SCAN, STATE_FAULT)
            )
            result["segments"][spec.name] = {
                "ticks_raw": len(raw_s),
                "ticks_deskewed": len(dsk_s),
                "no_scan_or_fault_raw": int(np.isin(raw_s, (STATE_NO_SCAN, STATE_FAULT)).sum()),
                "no_scan_or_fault_deskewed": int(
                    np.isin(dsk_s, (STATE_NO_SCAN, STATE_FAULT)).sum()
                ),
                "ticks_only_deskewed_has": int(bad.sum()),
                "tick_count_differs": bool(len(raw_s) != len(dsk_s)),
            }
            new_states.append(int(bad.sum()))
    result["passed"] = bool(
        sum(new_states) == 0
        and not any(s["tick_count_differs"] for s in result["segments"].values())
    )
    return result


# -- item 5 ----------------------------------------------------------------------------------------------


def item_5(ev: Any, split: dict[str, Any], chosen: str) -> dict[str, Any]:
    deskew, _relay, vmod = replay._modules()
    vp = vmod.load_generated_params(replay.REPO_ROOT)
    live = vmod.from_generated(vp)
    stamp_is, order = chosen.split("/")
    cfg = deskew.DeskewConfig(stamp_is=stamp_is, beam_order=order)
    rec = ev.recording("2026-11-13T22-20-18_car_teleop")
    moving = np.nonzero(
        np.abs(
            rec["odom_v"][
                np.clip(np.searchsorted(rec["odom_t"], rec["scan_t"], side="right") - 1, 0, None)
            ]
        )
        > MOVING_MPS
    )[0]
    pick = moving[np.linspace(0, len(moving) - 1, LATENCY_SCANS).astype(int)]
    times = []
    for i in pick:
        raw = bagio.scan_ranges(rec, int(i)).astype(np.float64)
        static = deskew.body_beams(
            raw,
            float(rec["scan_angle_min"][i]),
            float(rec["scan_angle_increment"][i]),
            live.mount,
            live.body,
        )
        t0 = time.perf_counter()
        deskew.deskew_scan(
            raw,
            float(rec["scan_angle_min"][i]),
            float(rec["scan_angle_increment"][i]),
            float(rec["scan_time"][i]),
            float(rec["scan_dt"][i]),
            deskew.Twist(1.0, 1.0),
            live.mount,
            cfg,
            range_min=float(rec["scan_range_min"][i]),
            range_max=float(rec["scan_range_max"][i]),
            static=static,
        )
        times.append((time.perf_counter() - t0) * 1000.0)
    ms = np.array(times)
    p99 = float(np.percentile(ms, 99))
    return {
        "scans": len(ms),
        "p50_ms": float(np.percentile(ms, 50)),
        "p99_ms": p99,
        "max_ms": float(ms.max()),
        "max_p99_ms": MAX_LATENCY_P99_MS,
        "passed": bool(p99 < MAX_LATENCY_P99_MS),
    }


def run_step(args: Any, ev: Any) -> int:
    cfg = ev.load_config()
    split = yaml.safe_load(ev.SPLIT.read_text())
    chosen = f"{args.stamp_is}/{args.beam_order}"
    args.out.mkdir(parents=True, exist_ok=True)
    bundle = args.dynamics_bundle or DYNAMICS_BUNDLE
    result: dict[str, Any] = {"chosen": chosen, "split_sha256": ev.SPLIT_SHA256}
    result.update(items_1_2(ev, cfg, split, chosen, args.workers))
    result["item3"] = item_3(ev, split, chosen)
    result["item4"] = item_4(ev, split, chosen, args.out, args.cli, bundle)
    result["item5"] = item_5(ev, split, chosen)
    items = {k: result[k]["passed"] for k in ("item1", "item2", "item3", "item4", "item5")}
    result["items_passed"] = items
    result["verdict"] = (
        "PASS: deskew may be switched on at the car"
        if all(items.values())
        else "no measurable gain, deskew stays off"
        if not items["item1"]
        else "deskew stays off: item(s) " + ", ".join(k for k, v in items.items() if not v)
    )
    (args.out / "step_b.json").write_text(json.dumps(_clean(result), indent=2))
    print(json.dumps(_clean(result), indent=2))
    return 0


run = run_step
