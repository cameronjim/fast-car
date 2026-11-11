#!/usr/bin/env python3
"""ctest: mppi_bench runs against a bundle made from the live vehicle_params and reports timings.

The budget here is deliberately loose (CI machines vary); the real budget check is run by hand on
the dev machine and on the Jetson with the default --budget-ms 12.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import planner_launch_common as common


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bench", required=True)
    ap.add_argument("--repo-root", required=True)
    args = ap.parse_args()
    with tempfile.TemporaryDirectory(prefix="racer_planner_bench_") as tmp:
        bundle = common.make_bundle(Path(tmp), "bench")
        proc = subprocess.run(
            [args.bench, "--bundle", str(bundle), "--solves", "200", "--budget-ms", "250"],
            capture_output=True,
            text=True,
            check=False,
        )
    sys.stdout.write(proc.stdout)
    sys.stderr.write(proc.stderr)
    if proc.returncode != 0:
        print(f"mppi_bench exited {proc.returncode}")
        return 1
    if not re.search(r"p50 [0-9.]+\s+p90 [0-9.]+\s+p99 [0-9.]+\s+max [0-9.]+", proc.stdout):
        print("mppi_bench did not print p50 / p90 / p99 / max")
        return 1
    if "samples 768" not in proc.stdout:
        print("mppi_bench did not report the sample count")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
