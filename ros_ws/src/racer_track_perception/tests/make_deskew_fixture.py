"""Cut the committed L4 fixture of the scan deskew and write its golden output.

    cd tools
    uv run python ../ros_ws/src/racer_track_perception/tests/make_deskew_fixture.py

30 scans from the TRAIN segments R2 (ccw, cap 1.0 m/s) and R7a (cw, cap 1.0 m/s) of the
2026-11-13T22-20-18 bag (never a held-out segment): the 15 scans of each with the highest wheel
yaw rate, so every one is in a corner. Each scan keeps its timing fields, and the fixture carries
the /odom/wheel (arrival, header stamp, speed) and /drive (arrival, steering) samples from 0.5 s
before the first scan to the last, all on absolute seconds. Ranges are uint16 quarter millimetres
(lossless, the LiDAR's own resolution; 0 is not a return) in a compressed npz.

The golden file is what `DeskewRelay` (stamp_is sweep_start, beam_order index_descending: a TEST
configuration, not a claim about the car) gives on the stored stream. Tolerance for the
comparison: 0.005 m per beam (test_deskew_real_scans.py).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

TESTS = Path(__file__).resolve().parent
FIXTURES = TESTS / "fixtures"
SCANS = FIXTURES / "deskew_real_scans_2026-11-13.npz"
GOLDEN = FIXTURES / "deskew_real_scans_2026-11-13_golden.npz"
CACHE = Path("/Users/cameronjim/code/car-data/pnp/cache/deskew_2026-11-13T22-20-18_car_teleop.npz")
SEGMENTS = {"R2": (626.1, 645.8), "R7a": (2967.2, 2980.6)}
PER_SEGMENT = 15
PRELOAD_S = 0.5
QUANTUM_M = 0.00025


def dequantise(q: np.ndarray) -> np.ndarray:
    out = q.astype(np.float32) / np.float32(1.0 / QUANTUM_M)
    out[q == 0] = np.inf
    return out


def run_relay(
    data: dict[str, np.ndarray], stamp_is: str = "sweep_start", beam_order: str = "index_descending"
):
    """The relay over the stored stream in arrival order. Returns (ranges [n, beams] with the
    input where a scan passed through, offset [n], reason [n])."""
    from racer_track_perception import deskew, deskew_relay
    from racer_track_perception.vehicle import from_generated, load_generated_params

    vp = load_generated_params()
    live = from_generated(vp)
    table = tuple((float(p.abs_angle_rad), float(p.gain)) for p in vp.steering.yaw_rate_gain_table)
    relay = deskew_relay.DeskewRelay(
        deskew.DeskewConfig(stamp_is=stamp_is, beam_order=beam_order), live.mount, live.body, table
    )
    events = [(float(t), 0, i) for i, t in enumerate(data["odom_t"])]
    events += [(float(t), 1, i) for i, t in enumerate(data["drive_t"])]
    events += [(float(t), 2, i) for i, t in enumerate(data["scan_t"])]
    events.sort(key=lambda e: (e[0], e[1]))
    n = len(data["scan_t"])
    out = np.zeros((n, data["scan_q"].shape[1]), np.float32)
    offset = np.zeros(n)
    reason = [""] * n
    for t, kind, i in events:
        if kind == 0:
            relay.on_odom(t, float(data["odom_stamp"][i]), float(data["odom_v"][i]))
        elif kind == 1:
            relay.on_drive(t, 0.0, float(data["drive_steer"][i]))
        else:
            raw = dequantise(data["scan_q"][i])
            res = relay.process(
                t,
                float(data["scan_stamp"][i]),
                raw,
                float(data["angle_min"]),
                float(data["angle_increment"]),
                float(data["range_min"]),
                float(data["range_max"]),
                float(data["scan_time"][i]),
                float(data["scan_dt"][i]),
            )
            if res.ranges is None:
                out[i], reason[i] = raw, res.reason
            else:
                out[i], offset[i] = res.ranges, res.stamp_offset_s
    return out, offset, reason


def main() -> int:
    tools = TESTS.parents[3] / "tools"
    sys.path.insert(0, str(tools))
    sys.path.insert(0, str(TESTS.parent / "src"))
    from pnp_deskew import replay

    z = np.load(CACHE)
    rec = {k: z[k] for k in z.files}
    t0 = float(rec["t0_log_s"])
    picked: list[int] = []
    for a, b in SEGMENTS.values():
        idx = np.nonzero((rec["scan_t"] >= a) & (rec["scan_t"] <= b))[0]
        _v, rate = replay.wheel_yaw_rate_at_scans(rec, idx)
        order = np.argsort(-np.abs(rate))[:PER_SEGMENT]
        picked += sorted(int(idx[k]) for k in order)
    picked.sort()
    om = np.zeros(len(rec["odom_t"]), dtype=bool)
    dm = np.zeros(len(rec["drive_t"]), dtype=bool)
    for i in picked:  # the samples of PRELOAD_S before each scan up to its arrival
        t = float(rec["scan_t"][i])
        om |= (rec["odom_t"] >= t - PRELOAD_S) & (rec["odom_t"] <= t)
        dm |= (rec["drive_t"] >= t - PRELOAD_S) & (rec["drive_t"] <= t)
    data = {
        "scan_t": rec["scan_t"][picked] + t0,
        "scan_stamp": rec["scan_stamp"][picked] + t0,
        "scan_time": rec["scan_time"][picked],
        "scan_dt": rec["scan_dt"][picked],
        "scan_q": rec["scan_q"][picked],
        "angle_min": rec["scan_angle_min"][picked[0]],
        "angle_increment": rec["scan_angle_increment"][picked[0]],
        "range_min": rec["scan_range_min"][picked[0]],
        "range_max": rec["scan_range_max"][picked[0]],
        "odom_t": rec["odom_t"][om] + t0,
        "odom_stamp": rec["odom_stamp"][om] + t0,
        "odom_v": rec["odom_v"][om],
        "drive_t": rec["drive_t"][dm] + t0,
        "drive_steer": rec["drive_steer"][dm],
    }
    FIXTURES.mkdir(exist_ok=True)
    np.savez_compressed(SCANS, **data)
    loaded = {k: v for k, v in np.load(SCANS).items()}
    ranges, offset, reason = run_relay(loaded)
    np.savez_compressed(GOLDEN, ranges=ranges, offset=offset, reason=np.array(reason))
    print(f"{len(picked)} scans, {SCANS.stat().st_size} B scans, {GOLDEN.stat().st_size} B golden")
    print("passed through:", sum(1 for r in reason if r), sorted(set(reason)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
