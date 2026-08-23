#!/usr/bin/env python3
"""Cross-language base-controller divergence test (roadmap S.3): runs the committed
divergence fixture states through the built C++ `pure_pursuit_cli` binary and asserts the
result matches the committed `divergence_expected.json` (computed by
training/racer_train/tests/generate_divergence_fixture.py from
racer_train.raceline.PurePursuitController, the Python port of the SAME algorithm) within a
stated tolerance.

Wired into ros_ws/src/racer_control/CMakeLists.txt as a plain CTest `add_test()` (this
package has no ament_cmake_pytest dependency and needs none: this script is invoked as a
subprocess, not collected as a pytest test module), so `colcon test` runs it in the same job
that builds pure_pursuit_cli. Deliberately stdlib-only (json, csv, subprocess, argparse) --
no pytest/numpy dependency needed inside the ros-dev image's system Python for this one
script.

Plain script, not a pytest module: exits 0 on success, non-zero with a message on any
mismatch (CTest treats a nonzero exit as a test failure).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cli", required=True, help="Path to the built pure_pursuit_cli binary.")
    parser.add_argument("--raceline", required=True, help="Path to the committed raceline CSV.")
    parser.add_argument("--states", required=True, help="Path to divergence_states.csv.")
    parser.add_argument("--expected", required=True, help="Path to divergence_expected.json.")
    parser.add_argument(
        "--tolerance",
        type=float,
        default=1e-6,
        help="Max allowed |difference| per field (steering_angle_rad, speed_mps).",
    )
    args = parser.parse_args()

    with open(args.expected, "r", encoding="utf-8") as f:
        expected = json.load(f)

    result = subprocess.run(
        [args.cli, "--raceline", args.raceline, "--states", args.states],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        print(f"pure_pursuit_cli exited {result.returncode}", file=sys.stderr)
        print(f"stdout:\n{result.stdout}", file=sys.stderr)
        print(f"stderr:\n{result.stderr}", file=sys.stderr)
        return 1

    actual_lines = [line for line in result.stdout.splitlines() if line.strip()]
    if len(actual_lines) != len(expected):
        print(
            f"line count mismatch: pure_pursuit_cli printed {len(actual_lines)} commands, "
            f"expected {len(expected)} (one per state in {args.states})",
            file=sys.stderr,
        )
        return 1

    failures = []
    for i, (line, expected_cmd) in enumerate(zip(actual_lines, expected)):
        try:
            actual_cmd = json.loads(line)
        except json.JSONDecodeError as exc:
            failures.append(f"  [{i}] could not parse CLI output line as JSON: {line!r} ({exc})")
            continue
        for field in ("steering_angle_rad", "speed_mps"):
            diff = abs(actual_cmd[field] - expected_cmd[field])
            if diff > args.tolerance:
                failures.append(
                    f"  [{i}] {field}: cli={actual_cmd[field]!r} python={expected_cmd[field]!r} "
                    f"|diff|={diff!r} > tolerance={args.tolerance!r}"
                )

    if failures:
        print(
            f"pure_pursuit_cli diverged from racer_train.raceline.PurePursuitController on "
            f"{len(failures)} field(s) across {len(expected)} state(s):",
            file=sys.stderr,
        )
        print("\n".join(failures), file=sys.stderr)
        return 1

    print(
        f"pure_pursuit_cli matched the Python base controller on all {len(expected)} "
        f"state(s), within tolerance {args.tolerance}."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
