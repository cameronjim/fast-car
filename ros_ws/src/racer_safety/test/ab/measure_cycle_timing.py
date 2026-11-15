"""Duration of safety_node's gate cycle with and without the cycle record (brief 45).

Runs the timing drivers (real publishers, a real subscriber on /drive, /safety/events and
/safety/cycle, a manual clock, steady input, no gate engaged) for N cycles, alternating:

  main      the frozen copy of main's safety_node.cpp
  false     this branch, publish_cycle_record false
  true      this branch, publish_cycle_record true

and prints p50 and p99 of one on_timer call per run, plus the run to run spread of each (the
noise). It asserts nothing: the brief invents no threshold. Read the numbers in the report.
"""

from __future__ import annotations

import argparse
import re
import statistics
import subprocess
import sys

LINE = re.compile(r"TIMING cycles=(\d+) p50_ns=(\d+) p99_ns=(\d+) max_ns=(\d+)")


def run(exe: str, cycles: int, *flags: str) -> tuple[int, int]:
    result = subprocess.run(
        [exe, "--timing", str(cycles), *flags],
        capture_output=True,
        text=True,
        timeout=600,
        check=False,
    )
    match = LINE.search(result.stdout)
    if result.returncode != 0 or match is None:
        sys.exit(
            f"FAIL {exe} {flags}: exit {result.returncode}\n{result.stdout}\n{result.stderr[-1500:]}"
        )
    return int(match.group(2)), int(match.group(3))


def spread(values: list[int]) -> str:
    return f"median {statistics.median(values):.0f} ns, min {min(values)}, max {max(values)}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", required=True)
    parser.add_argument("--branch", required=True)
    parser.add_argument("--cycles", type=int, default=100000)
    parser.add_argument("--repeats", type=int, default=5)
    args = parser.parse_args()

    runs: dict[str, list[tuple[int, int]]] = {"main": [], "false": [], "true": []}
    for _ in range(args.repeats):
        runs["main"].append(run(args.reference, args.cycles))
        runs["false"].append(run(args.branch, args.cycles))
        runs["true"].append(run(args.branch, args.cycles, "--cycle-record"))

    print(f"gate cycle duration, {args.cycles} cycles per run, {args.repeats} runs per variant")
    for name, results in runs.items():
        p50 = [r[0] for r in results]
        p99 = [r[1] for r in results]
        print(f"  {name:5s} p50: {spread(p50)}")
        print(f"  {name:5s} p99: {spread(p99)}")


if __name__ == "__main__":
    main()
