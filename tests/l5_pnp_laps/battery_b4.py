#!/usr/bin/env python3
"""Run battery B4 of the v0.3 pre-registration (section 1) and report D3-S0 to D3-S2.

    python3 tests/l5_pnp_laps/battery_b4.py n4-check --out DIR
    python3 tests/l5_pnp_laps/battery_b4.py run --out DIR --conditions abcde --workers 5
    python3 tests/l5_pnp_laps/battery_b4.py run --development --out DEVDIR --conditions ac
    python3 tests/l5_pnp_laps/battery_b4.py report --out DIR

Needs the ROS workspace sourced (see .github/scripts/l5_pnp_laps_test.sh) and f1tenth_gym.

Cases, seeds, laps, caps and conditions are read from `evaluation/prereg/pnp-v0.3.yaml`, block
`lap_battery`, whose SHA-256 is checked against the pre-registration and written into every run
JSON and the report. `battery.py` and its v0.2 constants are the record of v0.2 and are not used.

Conditions: a (D off, cap 0.5), b (D off, cap at the ceiling), c (D v0), d (D v1, floor 0.3 m/s,
zone merge), e (D v1, no floor, no merge). The ceiling is the dynamics bundle's validity.

Files: every attempt is written once to `<out>/attempts/<name>.attempt<k>.json` and never
overwritten. `<out>/<name>.json` is the accepted attempt (`<condition>_<case>_<dir>_<seed>`),
written once. A run whose own /drive_raw stream has a gap over 40 ms was starved by the host: it
is rerun alone, at most three attempts in all, criteria unchanged, and every attempt is reported
(the rule of PR 166). `--resume` is the default: finished names are skipped. `--development` runs
the development cases into its own directory and cannot be combined with battery cases.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import lap_battery_b4 as b4

REPO_ROOT = HERE.parents[1]
RUNNER = REPO_ROOT / "tests" / "l5_reactive_bench" / "track_family_runner.py"
DEFAULT_BUNDLE = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-car-2026-11-12"
DEFAULT_DATA_ROOT = Path("/Users/cameronjim/code")  # the parent of car-data/
GUARDED_PATHS = ("ros_ws/src/racer_laps", "ros_ws/src/racer_planner", "tests/l5_reactive_bench")
STARVED_GAP_S = 0.040  # brief 30: a /drive_raw gap over 40 ms means the host starved the run
MAX_ATTEMPTS = 3
MAX_WORKERS = 5  # host limit set by the launcher: the Mac thermal-slept under full load
N4_CHECK_LAPS = 3
N4_CHECK_SEED = 21
N4_CASE = "N4"
N4_CHECK_FILE = "n4_check.json"


class BatteryError(RuntimeError):
    """The run is refused."""


# -- cases and conditions --------------------------------------------------------------------------


def manifest(bundle: Path) -> dict:
    import yaml

    return yaml.safe_load((bundle / "manifest.yaml").read_text())["io"]


def ceiling_mps(bundle: Path) -> float:
    return float(manifest(bundle)["validity"]["v_max_mps"])


def map_files(map_dir: Path) -> tuple[Path, Path]:
    """(map yaml, centreline csv) of a saved map directory: the single `*_centreline.csv` and the
    `<prefix>.yaml` next to it. The 2026-11-10-oval directory holds oval.yaml and oval_v3.yaml:
    the one that matches the centreline's prefix is meant."""
    csvs = sorted(map_dir.glob("*_centreline.csv"))
    if len(csvs) != 1:
        raise BatteryError(f"{map_dir}: expected one *_centreline.csv, found {len(csvs)}")
    prefix = csvs[0].name[: -len("_centreline.csv")]
    yml = map_dir / f"{prefix}.yaml"
    if not yml.exists():
        raise BatteryError(f"{map_dir}: no {yml.name} next to {csvs[0].name}")
    return yml, csvs[0]


def case_track(case_spec: dict, data_root: Path) -> tuple[str, dict]:
    """(track, track params) of a yaml case."""
    spec = dict(case_spec)
    track = spec.pop("track")
    if track == "map":
        yml, csv = map_files(data_root / spec.pop("map_dir"))
        return "map", {"map_yaml": str(yml), "map_centreline_csv": str(csv)}
    return track, spec


def condition_flags(const: b4.Constants, condition: str, ceiling: float) -> list[str]:
    """Runner flags of a condition, from the yaml's `conditions` block."""
    try:
        spec = const.conditions[condition]
    except KeyError:
        raise BatteryError(f"condition {condition!r} is not in the yaml") from None
    mode = str(spec["lap_memory"])
    lap1 = const.lap1_cap_mps
    if mode == "off":
        cap = {"lap1": lap1, "ceiling": ceiling}[str(spec["cap"])]
        return ["--lap-memory", "off", "--v-max-mps", str(cap)]
    flags = ["--lap-memory", mode, "--v-max-mps", str(ceiling), "--v-lap1-mps", str(lap1)]
    if mode == "v1":
        flags += [
            "--lap-bound-floor-mps",
            str(float(spec["v_bound_floor_mps"])),
            "--lap-zone-merge" if spec["zone_merge"] else "--no-lap-zone-merge",
        ]
    return flags


def command_for(
    const: b4.Constants,
    condition: str,
    case: str,
    direction: str,
    seed: int,
    domain: int,
    work: Path,
    out: Path,
    bundle: Path,
    data_root: Path,
    laps: int | None = None,
    development: bool = False,
) -> list[str]:
    """The runner's argv for one run. A case or seed not in the yaml refuses."""
    cases = const.development_cases if development else const.battery_cases
    seeds = const.development_seeds if development else const.battery_seeds
    if case not in cases:
        raise BatteryError(
            f"case {case!r} is not a {'development' if development else 'battery'} case "
            f"of the yaml ({sorted(cases)})"
        )
    if seed not in seeds:
        raise BatteryError(f"seed {seed} is not in the yaml's seeds {list(seeds)}")
    if direction not in const.directions:
        raise BatteryError(f"direction {direction!r} is not in {list(const.directions)}")
    io = manifest(bundle)
    nominal = io["nominal"]
    track, params = case_track(cases[case], data_root)
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        track,
        "--track-params-json",
        json.dumps(params),
        "--driver",
        "pnp",
        "--backend",
        "geometric",
        "--dynamics-bundle",
        str(bundle),
        "--plant-bundle",
        str(bundle),
        "--plant-speed-deadband",
        str(nominal["speed_deadband_mps"]),
        "--plant-steer-tau",
        str(nominal["steer_tau_s"]),
        "--plant-command-delay",
        str(nominal["command_delay_s"]),
        *condition_flags(const, condition, float(io["validity"]["v_max_mps"])),
        "--car",
        "params",
        "--seed",
        str(seed),
        "--stop-after-laps",
        str(const.laps if laps is None else laps),
        "--duration",
        "900",
        "--stall-s",
        "30",
        "--domain-id",
        str(domain),
        "--work-dir",
        str(work),
        "--out",
        str(out),
    ]
    if direction == "cw":
        command.append("--reverse")
    return command


# -- the checkout ----------------------------------------------------------------------------------


def _git(*args: str) -> str:
    # safe.directory: the container's user does not own the bind-mounted checkout
    out = subprocess.run(
        ["git", "-c", "safe.directory=*", "-C", str(REPO_ROOT), *args],
        capture_output=True,
        text=True,
        check=True,
    )
    return out.stdout


GIT_SHA_ENV = "B4_GIT_SHA"


def git_sha() -> str:
    """HEAD of the checkout. Inside the container the worktree's git directory is not mounted, so
    the caller may state the SHA it verified on the host (B4_GIT_SHA, a 40 hex digit string); the
    working tree check is then the caller's too (see check_clean)."""
    stated = os.environ.get(GIT_SHA_ENV, "")
    if stated:
        if len(stated) != 40 or any(c not in "0123456789abcdef" for c in stated):
            raise BatteryError(f"{GIT_SHA_ENV} is not a 40 digit SHA")
        return stated
    return _git("rev-parse", "HEAD").strip()


def dirty_paths() -> list[str]:
    """Modified or untracked files under the code the battery tests (the lap node, the planner and
    the runner)."""
    lines = _git("status", "--porcelain", "--", *GUARDED_PATHS).splitlines()
    return [ln for ln in lines if ln.strip()]


def check_clean() -> None:
    if os.environ.get(GIT_SHA_ENV):
        return  # git is not available here: the caller checked the tree on the host
    dirty = dirty_paths()
    if dirty:
        raise BatteryError(f"the working tree is dirty under the tested code: {dirty[:8]}")


# -- jobs and attempts -----------------------------------------------------------------------------


def expected_cases(const: b4.Constants, out: Path, mode: str, requested: list[str]) -> list[str]:
    """Validate the requested cases against the mode and the N4 rule."""
    if mode == "development":
        bad = [c for c in requested if c not in const.development_cases]
        if bad:
            raise BatteryError(
                f"--development runs development cases only, not {bad} "
                f"(allowed: {sorted(const.development_cases)})"
            )
        return requested
    bad = [c for c in requested if c not in const.battery_cases]
    if bad:
        raise BatteryError(
            f"{bad} are not battery cases ({sorted(const.battery_cases)}); development cases "
            "run with --development in a separate directory"
        )
    if N4_CASE in requested:
        check = out / N4_CHECK_FILE
        if not check.exists():
            raise BatteryError("N4 is requested but n4-check has not been run in this directory")
        if json.loads(check.read_text())["dropped"]:
            raise BatteryError("N4 was dropped by the n4-check: do not request it")
    return requested


def battery_cases(const: b4.Constants, out: Path) -> list[str]:
    """The battery's cases: all of the yaml's, without N4 when the n4-check dropped it."""
    cases = list(const.battery_cases)
    check = out / N4_CHECK_FILE
    if not check.exists():
        raise BatteryError("run n4-check first (N4 rule of pre-registration 1.3)")
    if json.loads(check.read_text())["dropped"]:
        cases.remove(N4_CASE)
    return cases


def starved(result: dict) -> bool:
    gap = (result.get("drive_raw") or {}).get("max_gap_s")
    return gap is not None and float(gap) > STARVED_GAP_S


def needs_rerun(result: dict) -> bool:
    """No usable result (the runner died before writing one) or a starved run."""
    unusable = "error" in result and "gt_lap_times_by_crossing_s" not in result
    return unusable or starved(result)


def attempt_path(out: Path, name: str, k: int) -> Path:
    return out / "attempts" / f"{name}.attempt{k}.json"


def existing_attempts(out: Path, name: str) -> list[Path]:
    return sorted((out / "attempts").glob(f"{name}.attempt*.json"))


def write_once(path: Path, text: str) -> None:
    """Create `path` and fail when it exists: a result is never overwritten."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as fh:
        fh.write(text)


def meta_for(
    const: b4.Constants, sha: str, mode: str, job: tuple, attempt: int, command: list[str]
) -> dict:
    name, condition, case, direction, seed = job
    return {
        "id": "B4",
        "mode": mode,
        "name": name,
        "condition": condition,
        "case": case,
        "direction": direction,
        "seed": seed,
        "attempt": attempt,
        "prereg_yaml_sha256": const.yaml_sha256,
        "git_sha": sha,
        "command": command[1:],
    }


def run_attempt(
    const: b4.Constants,
    args: argparse.Namespace,
    sha: str,
    mode: str,
    job: tuple,
    attempt: int,
    domain: int,
    out: Path,
) -> dict:
    name, condition, case, direction, seed = job
    work = out / "work" / f"{name}.attempt{attempt}"
    work.mkdir(parents=True, exist_ok=True)
    result_path = work / "result.json"
    command = command_for(
        const,
        condition,
        case,
        direction,
        seed,
        domain,
        work,
        result_path,
        Path(args.bundle),
        Path(args.data_root),
        development=mode == "development",
    )
    started = time.monotonic()
    with (work / "runner.log").open("w") as log:
        proc = subprocess.run(
            command, stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ), check=False
        )
    if result_path.exists():
        result = json.loads(result_path.read_text())
    else:
        result = {"error": f"no result file, runner exit {proc.returncode}"}
    result["battery_b4"] = meta_for(const, sha, mode, job, attempt, command) | {
        "runner_exit": proc.returncode,
        "wall_s": round(time.monotonic() - started, 1),
        "starved": starved(result),
    }
    write_once(attempt_path(out, name, attempt), json.dumps(result, indent=1))
    return result


def pending_work(out: Path, jobs: list[tuple]) -> list[tuple[tuple, int]]:
    """(job, attempt number) still to run, and the accepted results written for finished jobs."""
    todo = []
    for job in jobs:
        name = job[0]
        final = out / f"{name}.json"
        if final.exists():
            continue
        done = existing_attempts(out, name)
        if not done:
            todo.append((job, 1))
            continue
        last = json.loads(done[-1].read_text())
        k = len(done)
        if needs_rerun(last) and k < MAX_ATTEMPTS:
            todo.append((job, k + 1))
        else:
            # accepted: every attempt is listed in the file
            attempts = [
                {
                    "attempt": i,
                    "file": p.name,
                    "starved": bool(json.loads(p.read_text())["battery_b4"]["starved"]),
                    "max_gap_s": (json.loads(p.read_text()).get("drive_raw") or {}).get(
                        "max_gap_s"
                    ),
                    "error": json.loads(p.read_text()).get("error", ""),
                }
                for i, p in enumerate(done, start=1)
            ]
            last["battery_b4"]["attempts"] = attempts
            last["battery_b4"]["accepted_attempt"] = k
            last["battery_b4"]["accepted_while_starved"] = bool(last["battery_b4"]["starved"])
            write_once(final, json.dumps(last, indent=1))
    return todo


def run_jobs(
    const: b4.Constants,
    args: argparse.Namespace,
    sha: str,
    mode: str,
    jobs: list[tuple],
    out: Path,
    workers: int,
) -> None:
    """Attempt 1 of every job with `workers` at a time, then every rerun alone."""
    lock = threading.Lock()
    while True:
        todo = pending_work(out, jobs)
        if not todo:
            return
        first = [t for t in todo if t[1] == 1]
        batch = first if first else todo[:1]  # reruns go one at a time, alone
        n_workers = min(workers, len(batch)) if first else 1
        print(f"{len(batch)} attempts, {n_workers} at a time -> {out}", flush=True)
        domains: queue.Queue[int] = queue.Queue()
        for k in range(n_workers):
            domains.put(args.domain_base + k)
        pending: queue.Queue[tuple] = queue.Queue()
        for item in batch:
            pending.put(item)

        def worker(
            pending: queue.Queue[tuple] = pending, domains: queue.Queue[int] = domains
        ) -> None:
            while True:
                try:
                    job, attempt = pending.get_nowait()
                except queue.Empty:
                    return
                domain = domains.get()
                try:
                    result = run_attempt(const, args, sha, mode, job, attempt, domain, out)
                    gap = (result.get("drive_raw") or {}).get("max_gap_s")
                    with lock:
                        print(
                            f"{job[0]} attempt {attempt}: "
                            f"{len(result.get('gt_lap_times_by_crossing_s', []))} laps, "
                            f"max /drive_raw gap {gap}, starved {starved(result)}, "
                            f"error {result.get('error', '')[:60]!r}",
                            flush=True,
                        )
                finally:
                    domains.put(domain)

        threads = [threading.Thread(target=worker) for _ in range(n_workers)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()


def make_jobs(
    conditions: list[str], cases: list[str], seeds: list[int], directions: tuple[str, ...]
) -> list[tuple]:
    return [
        (f"{cond}_{case}_{direction}_{seed}", cond, case, direction, seed)
        for cond in conditions
        for case in cases
        for direction in directions
        for seed in seeds
    ]


def check_mode_of_directory(out: Path, mode: str) -> None:
    """A directory holds battery runs or development runs, never both."""
    for p in list(out.glob("*.json")) + list((out / "attempts").glob("*.json")):
        if p.name == N4_CHECK_FILE:
            continue
        have = json.loads(p.read_text()).get("battery_b4", {}).get("mode")
        if have is not None and have != mode:
            raise BatteryError(f"{out} already holds {have} runs: use a separate directory")


def run_cmd(args: argparse.Namespace) -> None:
    const = b4.load_constants()
    mode = "development" if args.development else "battery"
    if args.workers < 1 or args.workers > MAX_WORKERS:
        raise BatteryError(f"--workers must be 1 to {MAX_WORKERS} (host limit)")
    conditions = list(args.conditions)
    unknown = [c for c in conditions if c not in const.conditions]
    if unknown:
        raise BatteryError(f"conditions {unknown} are not in the yaml")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    check_mode_of_directory(out, mode)
    if mode == "development":
        requested = args.cases.split(",") if args.cases else list(const.development_cases)
        cases = expected_cases(const, out, mode, requested)
        seeds = list(const.development_seeds)
    else:
        cases = (
            expected_cases(const, out, mode, args.cases.split(","))
            if args.cases
            else battery_cases(const, out)
        )
        seeds = list(const.battery_seeds)
    check_clean()
    sha = git_sha()
    jobs = make_jobs(conditions, cases, seeds, const.directions)
    for job in jobs:  # validates every command before anything starts
        command_for(
            const,
            job[1],
            job[2],
            job[3],
            job[4],
            args.domain_base,
            out,
            out,
            Path(args.bundle),
            Path(args.data_root),
            development=mode == "development",
        )
    print(f"B4 {mode}: git {sha}, yaml sha256 {const.yaml_sha256}, {len(jobs)} runs", flush=True)
    run_jobs(const, args, sha, mode, jobs, out, args.workers)


def n4_check_cmd(args: argparse.Namespace) -> None:
    """Condition a, seed 21, both directions, 3 laps on N4. Once. Writes `n4_check.json`."""
    const = b4.load_constants()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    check_path = out / N4_CHECK_FILE
    if check_path.exists():
        raise BatteryError(f"{check_path} exists: the N4 check may run once")
    check_clean()
    sha = git_sha()
    results: dict[str, dict] = {}
    notes: list[str] = []
    domains = [args.domain_base, args.domain_base + 1]

    def one(direction: str, domain: int) -> None:
        name = f"a_{N4_CASE}_{direction}_{N4_CHECK_SEED}"
        work = out / "n4_check" / name
        work.mkdir(parents=True, exist_ok=True)
        result_path = work / "result.json"
        try:
            command = command_for(
                const,
                "a",
                N4_CASE,
                direction,
                N4_CHECK_SEED,
                domain,
                work,
                result_path,
                Path(args.bundle),
                Path(args.data_root),
                laps=N4_CHECK_LAPS,
            )
        except (BatteryError, OSError, KeyError) as exc:  # the map cannot be loaded
            results[direction] = {"error": f"command: {exc}"}
            return
        with (work / "runner.log").open("w") as log:
            proc = subprocess.run(
                command, stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ), check=False
            )
        res = (
            json.loads(result_path.read_text())
            if result_path.exists()
            else {"error": f"no result file, runner exit {proc.returncode}"}
        )
        res["battery_b4"] = meta_for(
            const, sha, "n4_check", (name, "a", N4_CASE, direction, N4_CHECK_SEED), 1, command
        ) | {"starved": starved(res)}
        results[direction] = res

    threads = [
        threading.Thread(target=one, args=(d, domains[i])) for i, d in enumerate(const.directions)
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    laps = {
        d: len(r.get("gt_lap_times_by_crossing_s", [])) if "error" not in r else 0
        for d, r in results.items()
    }
    dropped = any(n < N4_CHECK_LAPS for n in laps.values())
    for d, r in results.items():
        if starved(r):
            notes.append(f"{d}: /drive_raw gap over {STARVED_GAP_S * 1000:.0f} ms (host starved)")
        if "error" in r:
            notes.append(f"{d}: {r['error'][:120]}")
    doc = {
        "dropped": dropped,
        "laps_by_direction": laps,
        "required_laps": N4_CHECK_LAPS,
        "git_sha": sha,
        "prereg_yaml_sha256": const.yaml_sha256,
        "notes": notes,
        "runs": results,
    }
    write_once(check_path, json.dumps(doc, indent=1))
    print(json.dumps({k: v for k, v in doc.items() if k != "runs"}, indent=1))


# -- the report ------------------------------------------------------------------------------------


def report_text(out: Path, mode: str = "battery") -> str:
    const = b4.load_constants()
    runs = b4.load_b4_runs(out, mode)
    metas = [json.loads(p.read_text())["battery_b4"] for p in sorted(out.glob("*_*_*_*.json"))]
    shas = sorted({m["git_sha"] for m in metas})
    yamls = sorted({m["prereg_yaml_sha256"] for m in metas})
    lines = [
        f"# Battery B4 report ({mode})",
        "",
        f"constants file sha256 now: {const.yaml_sha256} (registered {b4.registered_sha()})",
        f"constants file sha256 in the run files: {yamls}",
        f"git sha of the checkout in the run files: {shas}",
        f"runs: {len(runs)}",
    ]
    check = out / N4_CHECK_FILE
    cases = list(const.development_cases) if mode == "development" else list(const.battery_cases)
    if mode == "battery":
        if check.exists():
            doc = json.loads(check.read_text())
            lines.append(
                f"N4 check: dropped {doc['dropped']}, laps by direction {doc['laps_by_direction']}"
                f", notes {doc['notes']}"
            )
            if doc["dropped"]:
                cases.remove(N4_CASE)
        else:
            lines.append("N4 check: not run")
    lines += ["", "## Attempts", ""]
    multi = [m for m in metas if m.get("accepted_attempt", 1) > 1]
    for m in sorted(metas, key=lambda x: x["name"]):
        if m.get("accepted_attempt", 1) > 1 or m.get("accepted_while_starved"):
            lines.append(f"{m['name']}: {m['attempts']}")
    if not multi and not any(m.get("accepted_while_starved") for m in metas):
        lines.append("every run was accepted on its first attempt, none starved")
    lines += ["", "## Runs", "", b4.run_table(runs), ""]
    if mode == "development":
        lines.append("Development regression: labelled development, no gate decided.")
        return "\n".join(lines)
    conds = b4.CONDITIONS
    want = {
        (c, case, d, s)
        for c in conds
        for case in cases
        for d in const.directions
        for s in const.battery_seeds
    }
    have = {(r.condition, r.case, r.direction, r.seed) for r in runs}
    if want != have:
        lines.append(
            f"INCOMPLETE: {len(want - have)} runs missing, {len(have - want)} unexpected; "
            f"no decision is made"
        )
        return "\n".join(lines)
    rep = b4.build_report(runs, const)
    lines.append("## Decisions")
    for dec in rep.decisions.values():
        lines += ["", str(dec)]
    lines += ["", "## Reported without a threshold", ""] + [f"- {x}" for x in rep.extra]
    lines += ["", "## What goes on the car, by owner answer on the floor and merge", ""]
    lines += [f"- owner {k}: {v}" for k, v in rep.car.items()]
    return "\n".join(lines)


def report_cmd(args: argparse.Namespace) -> None:
    out = Path(args.out)
    mode = "development" if args.development else "battery"
    text = report_text(out, mode)
    print(text)
    (out / "report.md").write_text(text + "\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter
    )
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("run", "n4-check", "report"):
        p = sub.add_parser(name)
        p.add_argument("--out", required=True)
        if name != "n4-check":
            p.add_argument("--development", action="store_true")
        if name != "report":
            p.add_argument("--workers", type=int, default=4)
            p.add_argument("--domain-base", type=int, default=120)
            p.add_argument("--bundle", default=str(DEFAULT_BUNDLE))
            p.add_argument("--data-root", default=str(DEFAULT_DATA_ROOT))
        if name == "run":
            p.add_argument("--conditions", default="abcde")
            p.add_argument("--cases", default="", help="default: the whole battery (or dev set)")
            p.add_argument("--resume", action="store_true", help="accepted, always on")
    args = parser.parse_args(argv)
    try:
        {"run": run_cmd, "n4-check": n4_check_cmd, "report": report_cmd}[args.command](args)
    except (BatteryError, b4.PreregError) as exc:
        print(f"REFUSED: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
