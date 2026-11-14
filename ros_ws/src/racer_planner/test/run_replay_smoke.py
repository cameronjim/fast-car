#!/usr/bin/env python3
"""ctest: planner_replay_cli on a 3 s synthetic stream in a straight corridor.

The stream is what tools/pnp_replay/planner_stream.py writes (scans at 10 Hz, wheel speed at
50 Hz, a local track per scan, one K line every 0.02 s). The planner must reach DRIVING with a
positive speed command and never report FAULT. A second run checks the CLI refuses a bundle with
a bad checksum, as the node does, and rejects a parameter outside the node's range.
"""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import planner_launch_common as common

HALF_WIDTH_M = 0.5
SPEED_MPS = 0.3
BEAMS = 720


def corridor_ranges() -> list[float]:
    """Ranges of a 1 m wide corridor from the LiDAR head, laser bearing = vehicle bearing (the
    CLI runs with laser_yaw_from_vehicle_params=false, as the planner launch tests do)."""
    ly = common.VEHICLE["lidar_mount_y_m"]
    out = []
    for i in range(BEAMS):
        b = -math.pi + i * 2.0 * math.pi / BEAMS
        s = math.sin(b)
        if s > 1e-6:
            r = (HALF_WIDTH_M - ly) / s
        elif s < -1e-6:
            r = (-HALF_WIDTH_M - ly) / s
        else:
            r = math.inf
        out.append(r if r <= 12.0 else math.inf)
    return out


# K offsets inside a tick for the jittered stream: on time, late (12 ms after the tick's inputs, so
# the next one lands 8 ms after it: a bunched callback) and early.
JITTER_OFFSETS_S = (0.005, 0.005, 0.012, 0.0, 0.005, 0.005, 0.012, 0.0)


def stream_text(seconds: float = 3.0, jitter: bool = False) -> str:
    ranges = " ".join("inf" if math.isinf(r) else f"{r:.6f}" for r in corridor_ranges())
    inc = 2.0 * math.pi / BEAMS
    lines = []
    n_ticks = round(seconds / 0.02)
    for k in range(n_ticks):
        t = k * 0.02
        if k % 5 == 0:
            lines.append(f"S {t:.6f} {t:.6f} {-math.pi:.9g} {inc:.9g} 0.05 12 {BEAMS} {ranges}")
            xs = " ".join(f"{0.2 * j:.3f}" for j in range(16))
            ys = " ".join("0" for _ in range(16))
            hw = " ".join(f"{HALF_WIDTH_M}" for _ in range(16))
            sg = " ".join("0.02" for _ in range(16))
            lines.append(f"T {t + 0.01:.6f} {t:.6f} 16 0.02 0 {xs} {ys} {hw} {hw} {sg}")
        lines.append(f"O {t:.6f} {SPEED_MPS}")
        offset = JITTER_OFFSETS_S[k % len(JITTER_OFFSETS_S)] if jitter else 0.005
        lines.append(f"K {t + offset:.6f}")
    return "\n".join(lines) + "\n"


def run(cli: str, bundle: Path, stream: Path, *extra: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            cli,
            "--bundle",
            str(bundle),
            "--v-max",
            "0.5",
            "--set",
            "laser_yaw_from_vehicle_params=false",
            "--stream",
            str(stream),
            *extra,
        ],
        capture_output=True,
        text=True,
        check=False,
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cli", required=True)
    ap.add_argument("--repo-root", required=True)
    args = ap.parse_args()
    with tempfile.TemporaryDirectory(prefix="racer_planner_replay_") as tmp:
        root = Path(tmp)
        bundle = common.make_bundle(root, "replay")
        stream = root / "stream.txt"
        stream.write_text(stream_text(), encoding="utf-8")

        proc = run(args.cli, bundle, stream)
        if proc.returncode != 0:
            print(f"planner_replay_cli exited {proc.returncode}: {proc.stderr}")
            return 1
        rows = list(csv.DictReader(proc.stdout.splitlines()))
        if len(rows) != 150:
            print(f"expected 150 rows, got {len(rows)}")
            return 1
        if any(r["slew_violation"] != "0" for r in rows):
            print("a slew violation on the regular stream")
            return 1
        states = {r["state_name"] for r in rows}
        print(f"states {sorted(states)}")
        if "FAULT" in states:
            print("FAULT in a clean corridor")
            return 1
        driving = [r for r in rows if r["state_name"] == "DRIVING"]
        if len(driving) < 100:
            print(f"only {len(driving)} of 150 ticks are DRIVING")
            return 1
        if max(float(r["cmd_speed_mps"]) for r in driving) <= 0.0:
            print("no positive speed command while DRIVING")
            return 1

        # The same corridor with tick times that jitter and bunch: the elapsed-time slew check the
        # CLI writes next to every row (steering and speed step against f * rate * dt) has no
        # violation, and callbacks just after a late one are republished.
        jittered = root / "stream_jitter.txt"
        jittered.write_text(stream_text(jitter=True), encoding="utf-8")
        proc = run(args.cli, bundle, jittered)
        if proc.returncode != 0:
            print(f"planner_replay_cli (jitter) exited {proc.returncode}: {proc.stderr}")
            return 1
        jrows = list(csv.DictReader(proc.stdout.splitlines()))
        violations = [r for r in jrows if r["slew_violation"] != "0"]
        if violations:
            print(f"{len(violations)} slew violations on jittered ticks, first {violations[0]}")
            return 1
        bunched = sum(int(r["bunched"]) for r in jrows)
        if bunched < 10:
            print(f"only {bunched} bunched callbacks in the jittered stream")
            return 1
        if sum(1 for r in jrows if r["state_name"] == "DRIVING") < 100:
            print("the jittered stream never settled into DRIVING")
            return 1
        if any(r["state_name"] == "STARTING" and float(r["cmd_speed_mps"]) != 0.0 for r in jrows):
            print("a STARTING tick published a speed")
            return 1

        desc = subprocess.run(
            [args.cli, "--bundle", str(bundle), "--v-max", "0.5", "--describe"],
            capture_output=True,
            text=True,
            check=False,
        )
        if desc.returncode != 0 or '"v_max_mps":0.5' not in desc.stdout:
            print(f"--describe failed: {desc.stdout} {desc.stderr}")
            return 1

        bad = common.make_bad_checksum_bundle(root)
        proc = run(args.cli, bad, stream)
        if proc.returncode == 0 or "ChecksumMismatch" not in proc.stderr:
            print(f"a bad checksum was not refused: {proc.returncode} {proc.stderr!r}")
            return 1

        proc = run(args.cli, bundle, stream, "--set", "samples=3")
        if proc.returncode == 0 or "range" not in proc.stderr:
            print(f"an out of range parameter was accepted: {proc.returncode} {proc.stderr!r}")
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
