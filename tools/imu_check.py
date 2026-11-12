"""Is the VESC's IMU real, and how does its yaw rate compare with the LiDAR's?

    uv run python imu_check.py BAG [--gyro-units deg|rad] [--axis z]

Reads /vesc/sensors/imu/raw (sensor_msgs/Imu, published with `vesc_imu:=true`), /odom/wheel_twist
and /scan from a bag and prints a JSON report:

  health   rate, jitter, share of all-zero samples, per-axis std, standstill bias and noise.
           verdict "silent" (no messages: the driver was not polling), "zeros" (a board with no
           IMU answers the request with zeros), "junk" (flat or not a gyro), or "alive".
  fit      gyro z (and, for comparison, the wheel_twist yaw rate) against the yaw rate of
           point-to-point ICP between consecutive scans (safety_replay/motion.py): regression
           scale (1.0 is right), correlation, residual, best lag in scan periods.

The driver publishes gyro in deg/s and accel in g (f1tenth/vesc packet comments), not SI;
`--gyro-units` says which the bag holds, and the report converts. Sign and mount: REP-103, +z up,
so a left turn is positive; the fit's sign tells whether the board is mounted flipped.

The pure functions below are what tools/tests/test_imu_check.py covers; the bag reading and ICP
are thin wrappers (the 2026-11-12 bags have no IMU topics, so there is no real fixture yet).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from itertools import pairwise
from pathlib import Path

import numpy as np

DEG = math.pi / 180.0
STANDSTILL_MPS = 0.02


def imu_health(
    stamps_s: np.ndarray, gyro: np.ndarray, accel: np.ndarray, speed_mps: np.ndarray | None = None
) -> dict:
    """Rate, zeros, noise and standstill bias of an IMU stream.

    gyro, accel: (N, 3) in whatever units the stream has. speed_mps: wheel speed at each stamp,
    to find the standstill samples (bias and noise); without it all samples count as standstill
    only when the whole gyro series is flat.
    """
    n = len(stamps_s)
    if n == 0:
        return {"verdict": "silent", "n": 0}
    dt = np.diff(stamps_s)
    zero = np.all(gyro == 0.0, axis=1) & np.all(accel == 0.0, axis=1)
    out: dict = {
        "n": n,
        "rate_hz": float(1.0 / np.median(dt)) if n > 1 else 0.0,
        "dt_std_ms": float(dt.std() * 1e3) if n > 1 else 0.0,
        "dt_max_ms": float(dt.max() * 1e3) if n > 1 else 0.0,
        "zero_fraction": float(zero.mean()),
        "gyro_std": gyro.std(axis=0).tolist(),
        "accel_mean": accel.mean(axis=0).tolist(),
        "accel_norm_mean": float(np.linalg.norm(accel, axis=1).mean()),
    }
    still = np.zeros(n, bool) if speed_mps is None else np.abs(speed_mps) < STANDSTILL_MPS
    if still.sum() >= 20:
        out["standstill_n"] = int(still.sum())
        out["standstill_gyro_bias"] = gyro[still].mean(axis=0).tolist()
        out["standstill_gyro_noise"] = gyro[still].std(axis=0).tolist()
    if out["zero_fraction"] > 0.99:
        out["verdict"] = "zeros"
    elif float(np.abs(gyro).max()) == 0.0 or float(gyro.std(axis=0).max()) < 1e-9:
        out["verdict"] = "junk"
    else:
        out["verdict"] = "alive"
    return out


def resample(t_from: np.ndarray, x_from: np.ndarray, t_to: np.ndarray) -> np.ndarray:
    return np.interp(t_to, t_from, x_from)


def rate_fit(
    t: np.ndarray, rate: np.ndarray, ref_t: np.ndarray, ref_rate: np.ndarray, max_lag: int = 5
) -> dict:
    """Compare a yaw-rate series with a reference (ICP) series.

    Both are brought to the reference stamps by averaging `rate` over each reference interval, so
    a 50 Hz gyro and 10 Hz scan pairs line up. Returns the regression scale of the reference on
    the series through the origin (ref ~ scale * series), the correlation, the residual mean and
    std (rad/s), and the correlation at lags of -max_lag..max_lag reference periods (positive lag:
    the series is early, i.e. the reference is delayed relative to it).
    """
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1]) * np.diff(t))])
    # ref_t are the interval midpoints of the reference: each reference rate covers
    # [ref_t - half, ref_t + half], half = the median spacing / 2.
    half = float(np.median(np.diff(ref_t))) / 2.0
    a = resample(t, cum, ref_t - half)
    b = resample(t, cum, ref_t + half)
    series = (b - a) / (2.0 * half)
    ok = np.isfinite(series) & np.isfinite(ref_rate)
    s, r = series[ok], ref_rate[ok]
    if len(s) < 10 or not np.any(s):
        return {"n": len(s), "scale": float("nan"), "corr": float("nan")}
    out = {
        "n": len(s),
        "scale": float(s @ r / (s @ s)),
        "corr": float(np.corrcoef(s, r)[0, 1]),
        "residual_mean": float((r - s).mean()),
        "residual_std": float((r - s).std()),
    }
    lags = {}
    for lag in range(-max_lag, max_lag + 1):
        x1, y1 = (s[: len(s) - lag], r[lag:]) if lag >= 0 else (s[-lag:], r[: len(r) + lag])
        lags[lag] = float(np.corrcoef(x1, y1)[0, 1])
    out["lag_corr"] = lags
    out["best_lag"] = max(lags, key=lags.get)
    return out


def icp_yaw_rates(scans, mount, max_range_m: float = 5.0, max_rms_m: float = 0.03):
    """(midpoint stamps, yaw rate rad/s, forward speed m/s) from ICP between consecutive scans."""
    from safety_replay.motion import car_motion

    t, w, v = [], [], []
    for a, b in pairwise(scans):
        dt = b.stamp_s - a.stamp_s
        if not 0.05 < dt < 0.2:
            continue
        try:
            x, _, th, rms = car_motion(a, b, mount, max_range_m=max_range_m)
        except ValueError:
            continue
        if rms > max_rms_m:
            continue
        t.append(0.5 * (a.stamp_s + b.stamp_s))
        w.append(th / dt)
        v.append(x / dt)
    return np.array(t), np.array(w), np.array(v)


def _read(path: Path):
    from follower_replay.bag import read_bag
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    files = [path] if path.is_file() else sorted(path.glob("*.mcap"))
    imu, wheel = [], []
    for f in files:
        with f.open("rb") as stream:
            reader = make_reader(stream, decoder_factories=[DecoderFactory()])
            for _, ch, _, m in reader.iter_decoded_messages(
                topics=["/vesc/sensors/imu/raw", "/odom/wheel_twist"]
            ):
                stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                if ch.topic == "/odom/wheel_twist":
                    wheel.append((stamp, m.twist.twist.linear.x, m.twist.twist.angular.z))
                else:
                    g, a = m.angular_velocity, m.linear_acceleration
                    imu.append((stamp, g.x, g.y, g.z, a.x, a.y, a.z))
    return np.array(imu), np.array(wheel), read_bag(path)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("bag", type=Path)
    p.add_argument("--gyro-units", choices=["deg", "rad"], default="deg")
    p.add_argument("--mount", default="0.285,0.0,3.14159265", help="laser x,y,yaw in base_link")
    args = p.parse_args(argv)

    from safety_replay.motion import Mount

    imu, wheel, bag = _read(args.bag)
    report: dict = {"bag": str(args.bag)}
    scale = DEG if args.gyro_units == "deg" else 1.0
    speed = np.interp(imu[:, 0], wheel[:, 0], wheel[:, 1]) if len(imu) and len(wheel) else None
    report["health"] = imu_health(
        imu[:, 0] if len(imu) else np.zeros(0),
        imu[:, 1:4] * scale if len(imu) else np.zeros((0, 3)),
        imu[:, 4:7] if len(imu) else np.zeros((0, 3)),
        speed,
    )
    x, y, yaw = (float(v) for v in args.mount.split(","))
    ref_t, ref_w, _ = icp_yaw_rates(bag.scans, Mount(x, y, yaw))
    moving = np.abs(np.interp(ref_t, wheel[:, 0], wheel[:, 1])) > 0.3 if len(wheel) else None
    if moving is not None and moving.sum() >= 10:
        ref_t, ref_w = ref_t[moving], ref_w[moving]
        report["fit_wheel_twist_yaw"] = rate_fit(wheel[:, 0], wheel[:, 2], ref_t, ref_w)
        if report["health"]["verdict"] == "alive":
            report["fit_gyro_z"] = rate_fit(imu[:, 0], imu[:, 3] * scale, ref_t, ref_w)
    json.dump(report, sys.stdout, indent=2)
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
