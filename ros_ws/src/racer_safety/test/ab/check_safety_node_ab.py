"""A/B/C proof that safety_node's gated output is bit-identical (brief 45, owner decision 1).

Runs safety_node_ab_driver, which feeds one scripted input sequence of 2400 gate cycles under a
manual clock (see that file's header), through:

  A  the frozen copy of main's safety_node.cpp plus brief 49's credit threading
     (test/ab/reference/), run twice: the two runs must agree, which proves the harness itself is
     deterministic
  B  this branch's safety_node.cpp, publish_cycle_record false (the default)
  C  this branch's safety_node.cpp, publish_cycle_record true
  D  as C with the /safety/cycle publish made to throw on every cycle

and requires the /drive sequence (float bit patterns of speed and steering, count and order) and
the /safety/events sequence (source, phase, severity, duration bits, detail) to be identical in
all of them. It also checks that the script really exercises each gate, that every C record
matches the script, and that D loses no /drive and no event and counts every failure.

Nothing here is a golden: A is computed on every run from a source file whose blob hash is
pinned below, so it is that file and nothing else. Never edit that file or the hash.
"""

from __future__ import annotations

import argparse
import hashlib
import pathlib
import subprocess
import sys

# The reference copy is f5db7f1's safety_node.cpp plus brief 49's credit threading, and nothing
# else. f5db7f1 is the commit brief 45 was cut from; the blob hash of its safety_node.cpp was
# 8539839f73a85fe1f21f2328fb1cc4a2985b7698. Brief 49 added the rate credit to the gate library
# (gate_logic.hpp "RATE CREDIT") and the same threading edits to both node files (two credit
# members, two inputs and two outputs of the gate call, the reset to 0 on the fault path), so that
# A = B = C = D still holds with the credit threaded on both sides. A reference that did not thread
# the credit would run the old rule against the branch's new one and differ at the first stalled
# cycle, which is the change, not a regression. The copy must be this file, byte for byte. Never
# edit it again, and never edit the hash without the owner.
MAIN_SAFETY_NODE_BLOB_SHA1 = "63ae49fe0939e1afae7912db224e1fd96c658cb4"

# Every gate the script must engage at least once, as /safety/events sources.
REQUIRED_SOURCES = (
    "watchdog",
    "rate_limit",
    "bounds_clamp",
    "ttc",
    "ttc_reverse",
    "internal_fault",
)
MIN_CYCLES = 2000


def git_blob_sha1(path: pathlib.Path) -> str:
    data = path.read_bytes()
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def run(exe: str, out: pathlib.Path, *flags: str) -> list[str]:
    result = subprocess.run(
        [exe, "--out", str(out), *flags], capture_output=True, text=True, timeout=600, check=False
    )
    if result.returncode != 0:
        sys.exit(
            f"FAIL {exe} {' '.join(flags)} exited {result.returncode}\n"
            f"{result.stdout}\n{result.stderr[-2000:]}"
        )
    return out.read_text().splitlines()


def gated(lines: list[str]) -> list[str]:
    return [line for line in lines if line.startswith(("D ", "E "))]


# Brief 49's rate credit scenarios (see run_scenario in safety_node_ab_driver.cpp): scripted event
# lists with late and catch-up cycles. Each is played through the reference (A), the branch (B) and
# the branch with the cycle record on (C), which must agree on /drive, /safety/events and the
# node's two credits, and the branch's own expectations must all PASS.
SCENARIOS = {
    "credit_ramp": 3,
    "credit_fault": 3,
    "credit_nofault": 3,
}


def scenario_lines(lines: list[str]) -> list[str]:
    return [line for line in lines if line.startswith(("D ", "E ", "C "))]


def first_difference(a: list[str], b: list[str]) -> str:
    for index, (left, right) in enumerate(zip(a, b)):
        if left != right:
            return f"line {index}: {left!r} != {right!r}"
    return f"length {len(a)} != {len(b)}"


def require_identical(name: str, reference: list[str], candidate: list[str]) -> None:
    if reference != candidate:
        sys.exit(
            f"FAIL {name}: gated output differs from main: {first_difference(reference, candidate)}"
        )


def engaged_sources(lines: list[str]) -> set[str]:
    sources = set()
    for line in lines:
        if line.startswith("E "):
            source, phase = line.split(" ", 2)[2].split("|")[:2]
            if phase == "0":
                sources.add(source)
    return sources


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ref", required=True, help="driver built from the frozen main source")
    parser.add_argument("--branch", required=True, help="driver built from src/safety_node.cpp")
    parser.add_argument("--reference-source", required=True)
    parser.add_argument("--workdir", required=True)
    parser.add_argument(
        "--have-cycle", action="store_true", help="the branch source has publish_cycle_record"
    )
    args = parser.parse_args()

    work = pathlib.Path(args.workdir)
    work.mkdir(parents=True, exist_ok=True)

    blob = git_blob_sha1(pathlib.Path(args.reference_source))
    if blob != MAIN_SAFETY_NODE_BLOB_SHA1:
        sys.exit(f"FAIL the reference copy is not main's safety_node.cpp (blob {blob})")

    a1 = run(args.ref, work / "a1.txt")
    a2 = run(args.ref, work / "a2.txt")
    require_identical("harness determinism (A twice)", gated(a1), gated(a2))

    cycles = int(next(line for line in a1 if line.startswith("CYCLES ")).split()[1])
    if cycles < MIN_CYCLES:
        sys.exit(f"FAIL the script has {cycles} cycles, the brief needs at least {MIN_CYCLES}")
    drives = [line for line in a1 if line.startswith("D ")]
    if len(drives) != cycles:
        sys.exit(f"FAIL {len(drives)} /drive messages for {cycles} cycles")
    engaged = engaged_sources(a1)
    missing = [source for source in REQUIRED_SOURCES if source not in engaged]
    if missing:
        sys.exit(f"FAIL the script never engages {missing}; engaged: {sorted(engaged)}")

    b = run(args.branch, work / "b.txt")
    require_identical("B (branch, publish_cycle_record false)", gated(a1), gated(b))
    events = len([line for line in a1 if line.startswith("E ")])
    summary = [f"cycles={cycles}", f"events={events}", f"gates={sorted(engaged)}"]

    if args.have_cycle:
        c = run(args.branch, work / "c.txt", "--cycle-record")
        require_identical("C (branch, publish_cycle_record true)", gated(a1), gated(c))
        mismatches = [line for line in c if line.startswith("RECORD_MISMATCH ")]
        records = [line for line in c if line.startswith("R ")]
        if mismatches or len(records) != cycles:
            sys.exit(
                f"FAIL L3 record: {len(records)} records for {cycles} cycles, "
                f"mismatches: {mismatches[:5]}"
            )
        d = run(args.branch, work / "d.txt", "--cycle-record", "--throw-record")
        require_identical("D (record publish throws)", gated(a1), gated(d))
        failures = [line for line in d if line.startswith("FAILURES ")]
        if failures != [f"FAILURES {cycles}"] or any(line.startswith("R ") for line in d):
            sys.exit(f"FAIL failure isolation: {failures}")
        summary.append(
            "C identical, records match the script, D identical with every record failing"
        )

    for name, expected_checks in SCENARIOS.items():
        sa = run(args.ref, work / f"{name}_a.txt", "--scenario", name)
        sb = run(args.branch, work / f"{name}_b.txt", "--scenario", name)
        require_identical(f"scenario {name}, B against A", scenario_lines(sa), scenario_lines(sb))
        if args.have_cycle:
            sc = run(args.branch, work / f"{name}_c.txt", "--scenario", name, "--cycle-record")
            require_identical(
                f"scenario {name}, C against A", scenario_lines(sa), scenario_lines(sc)
            )
        checks = [line for line in sb if line.startswith("CHECK ")]
        failed = [line for line in checks if not line.endswith(": PASS")]
        if len(checks) != expected_checks or failed:
            detail = "\n".join(line for line in sb if not line.startswith(("D ", "C ")))
            sys.exit(f"FAIL scenario {name}: {len(checks)} checks, failed: {failed}\n{detail}")
        summary.append(f"{name} {len(checks)} checks pass")

    print("PASS bit-identical /drive and /safety/events:", ", ".join(summary))


if __name__ == "__main__":
    main()
