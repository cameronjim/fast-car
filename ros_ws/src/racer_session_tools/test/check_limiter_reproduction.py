#!/usr/bin/env python3
"""The tool binary on a stream cut from 2026-11-13T22-20-18 (run R2, tools/pnp_session cut):
with the recorded dt, each cycle judged from the recorded previous output, it must raise the
rate_limit engagement the bag recorded, at the same cycle and with the same dt in its detail, and
nothing else. With the nominal period the same stream must run and report its own summary.

Standard library only: this runs in the ROS test job, which has no mcap or numpy for it.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

DT = re.compile(r"dt=([0-9.]+)s")
TOL_S = 1e-3


def run(cli: str, stream: Path, mode: str, *extra: str) -> list[dict]:
    done = subprocess.run(
        [cli, "--stream", str(stream), "--dt-mode", mode, *extra],
        capture_output=True,
        text=True,
        check=False,
    )
    if done.returncode != 0:
        sys.exit(f"{cli} exited {done.returncode}: {done.stderr}")
    return [json.loads(line) for line in done.stdout.splitlines() if line.strip()]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cli", required=True)
    ap.add_argument("--fixture-dir", required=True)
    args = ap.parse_args()
    fixtures = Path(args.fixture_dir)
    stream = fixtures / "r2_cycles_teacher.txt"
    recorded = json.loads((fixtures / "r2_recorded_rate_limit.json").read_text())

    lines = run(args.cli, stream, "recorded", "--teacher")
    replay = [
        ln
        for ln in lines
        if ln["type"] == "event" and ln["phase"] == "engage" and ln["source"] == "rate_limit"
    ]
    summary = [ln for ln in lines if ln["type"] == "summary"]
    if len(summary) != 1 or summary[0]["cycles"] != 984:
        sys.exit(f"expected 984 cycles in the summary, got {summary}")
    if len(replay) != len(recorded):
        sys.exit(f"recorded {len(recorded)} rate_limit engagements, replay raised {len(replay)}")
    for rec, rep in zip(recorded, replay):
        if abs(rec["t"] - rep["t"]) > TOL_S:
            sys.exit(f"engagement at {rec['t']} recorded, replayed at {rep['t']}")
        a = float(DT.search(rec["detail"]).group(1))
        b = float(DT.search(rep["detail"]).group(1))
        if abs(a - b) > 1e-5:
            sys.exit(f"dt in detail: recorded {a}, replayed {b}")
    if any(
        ln["type"] == "event" and ln["phase"] == "engage" and ln["source"] != "rate_limit"
        for ln in lines
    ):
        sys.exit("an engagement of another source was raised")

    nominal = run(args.cli, stream, "nominal", "--teacher")
    if next(ln for ln in nominal if ln["type"] == "summary")["dt_mode"] != "nominal":
        sys.exit("nominal run did not report its mode")
    print("limiter reproduction: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
