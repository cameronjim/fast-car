"""Run battery B8 of the lap gate v3 pre-registration and report D8-S0 and D8-S1, and run the
development checks of brief 50 (A5) on development cases.

    python3 tests/l5_pnp_laps/battery_b8.py check-freeze
    python3 tests/l5_pnp_laps/battery_b8.py run --out DIR --workers 3
    python3 tests/l5_pnp_laps/battery_b8.py report --out DIR
    python3 tests/l5_pnp_laps/battery_b8.py dev-run --out DIR --cases P4,OD,ND --seeds 11,12,13
    python3 tests/l5_pnp_laps/battery_b8.py dev-report --out DIR

Needs the ROS workspace sourced (see .github/scripts/l5_pnp_laps_test.sh) and f1tenth_gym.

Cases R1 to R4, Q1, Q2, seeds 61 to 63, ccw and cw, 13 driven laps (12 judged) and the conditions a
(D off, cap 0.5), b (D off, ceiling) and c (D v0) are read from
`evaluation/prereg/pnp-v0.5-laps-ring.yaml`, block `lap_battery_b8`, whose SHA-256 is a constant of
`lap_battery_b8.py` and is written into every run JSON and the report.

NO R OR Q CASE AND NO SEED 61, 62 OR 63 RUNS UNLESS `evaluation/prereg/freeze/b8.json` EXISTS with
the freeze commit and the yaml's hash (`require_freeze`, called by `command_for`: there is no other
way to build a B8 command). Development runs (`dev-run`) are the development cases P4, OD and ND on
the development seeds 11 to 13 only, in a directory of their own, labelled development; every
other case or seed (B5's 31 and 32, B6's 41 and 42, B7's 51 to 53) is refused. `check-freeze`
verifies the file against git (the freeze commit is an ancestor of HEAD and the code under test and
the scoring harness are unchanged since) and is run on the host before the container runs the
battery with B4_GIT_SHA set to the SHA it verified, as B4 did.

Order: the condition a runs of seed 61 first (both directions of every case); the drop rule
(`lap_battery_b8.drop_rule`, `drops.json`, written once) then says which cases the rest of the
battery runs without, or that nothing is decided. Every attempt is written once to
`<out>/attempts/<name>.attempt<k>.json`. A run is starved (a /drive_raw gap over 40 ms, or a
real-time factor under 0.8), has an invalid ground truth (`truth_valid` false) or a truth recording
that ends before the count window: it is rerun alone, at most 3 attempts in all, every attempt
kept; still invalid counts as failed to execute. At most 3 workers (host limit of the launcher).
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
import battery_b4 as b4t
import battery_b5 as b5t
import lap_battery_b4 as b4
import lap_battery_b8 as b8

REPO_ROOT = HERE.parents[1]
RUNNER = b4t.RUNNER
FREEZE_PATH = REPO_ROOT / "evaluation" / "prereg" / "freeze" / "b8.json"
GUARDED_PATHS = b4t.GUARDED_PATHS  # the lap node, the planner and the runner
META = "battery_b8"
MAX_WORKERS = 3  # B4 at 5 workers had 44 of 90 first attempts starved
MAX_ATTEMPTS = b4t.MAX_ATTEMPTS
BatteryError = b4t.BatteryError
DROPS_FILE = "drops.json"
GIT_SHA_ENV = b4t.GIT_SHA_ENV
#: Seeds of every other registered attempt: never run again, by anyone, for any reason.
OTHER_REGISTERED_SEEDS = (31, 32, 41, 42, 51, 52, 53)
FIRST_SEED = 61  # the seed whose condition a runs decide the drop rule

# -- the freeze guard ------------------------------------------------------------------------------


def read_freeze(path: Path = FREEZE_PATH) -> dict:
    if not path.exists():
        raise BatteryError(f"{path} does not exist: B8 may not run before the freeze commit")
    try:
        doc = json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        raise BatteryError(f"{path} is not valid JSON: {exc}") from exc
    commit = doc.get("freeze_commit", "")
    if not (
        isinstance(commit, str)
        and len(commit) == 40
        and all(c in "0123456789abcdef" for c in commit)
    ):
        raise BatteryError(f"{path} has no 40 digit freeze_commit")
    if doc.get("yaml_sha256") != b8.REGISTERED_YAML_SHA256:
        raise BatteryError(
            f"{path} names yaml hash {doc.get('yaml_sha256')}, the registered one is "
            f"{b8.REGISTERED_YAML_SHA256}"
        )
    return doc


def _git(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["git", "-c", "safe.directory=*", "-C", str(REPO_ROOT), *args],
        capture_output=True,
        text=True,
        check=False,
    )


def verify_freeze_in_git(doc: dict) -> None:
    """The freeze commit is an ancestor of HEAD and the guarded code is as it was there."""
    commit = doc["freeze_commit"]
    if _git("merge-base", "--is-ancestor", commit, "HEAD").returncode != 0:
        raise BatteryError(f"freeze commit {commit} is not an ancestor of HEAD")
    diff = _git("diff", "--name-only", commit, "HEAD", "--", *GUARDED_PATHS)
    if diff.stdout.strip():
        raise BatteryError(
            f"the code under test changed since the freeze commit {commit}: "
            f"{diff.stdout.split()[:6]}"
        )


def require_freeze(path: Path = FREEZE_PATH, verify_git: bool | None = None) -> dict:
    """The freeze file, or a refusal. With `B4_GIT_SHA` set (the container has no git) the host's
    `check-freeze` is trusted for the git part."""
    doc = read_freeze(path)
    if verify_git is None:
        verify_git = not os.environ.get(GIT_SHA_ENV)
    if verify_git:
        verify_freeze_in_git(doc)
    return doc


def needs_freeze(const: b8.Constants, case: str, seed: int) -> bool:
    """A B8 case or a B8 seed: the freeze file is required. A development case on a development seed
    is not, and every combination that is neither is refused."""
    battery = case in const.battery_cases or seed in const.battery_seeds
    if battery:
        if case not in const.battery_cases or seed not in const.battery_seeds:
            raise BatteryError(
                f"case {case!r} with seed {seed}: a B8 case runs on seeds {list(B8_SEEDS)} only "
                "and a B8 seed runs on a B8 case only"
            )
        return True
    if seed in OTHER_REGISTERED_SEEDS:
        raise BatteryError(f"seed {seed} belongs to another registered attempt: never run again")
    if case in const.development_cases and seed in const.development_seeds:
        return False
    raise BatteryError(
        f"case {case!r} with seed {seed} is neither a B8 run nor a development run "
        f"(development: {sorted(const.development_cases)} on seeds {list(const.development_seeds)})"
    )


B8_SEEDS = b8.B8_SEEDS

# -- cases and commands ------------------------------------------------------------------------------


def case_track(spec: dict, data_root: Path) -> tuple[str, dict]:
    """(track, track params) of a yaml case: the keys the yaml adds (`class`) are not the track's."""
    track, params = b4t.case_track(spec, data_root)
    params = {k: v for k, v in params.items() if k != "class"}
    return track, params


def condition_flags(const: b8.Constants, condition: str, ceiling: float) -> list[str]:
    try:
        spec = const.conditions[condition]
    except KeyError:
        raise BatteryError(f"condition {condition!r} is not in the yaml") from None
    # the frozen yaml writes `lap_memory: off` unquoted, which YAML 1.1 reads as the boolean False
    raw = spec["lap_memory"]
    mode = "off" if raw is False else str(raw)
    if mode == "off":
        cap = spec.get("cap_mps", spec.get("cap"))
        value = ceiling if cap == "ceiling" else float(cap)
        return ["--lap-memory", "off", "--v-max-mps", str(value)]
    return [
        "--lap-memory",
        mode,
        "--v-max-mps",
        str(ceiling),
        "--v-lap1-mps",
        str(const.lap1_cap_mps),
    ]


def command_for(
    const: b8.Constants,
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
    freeze_path: Path = FREEZE_PATH,
) -> list[str]:
    """The runner's argv for one run. Refuses a B8 case or seed without the freeze file, a
    development run of anything but a development case on a development seed, and an unknown
    direction or condition."""
    if needs_freeze(const, case, seed):
        require_freeze(freeze_path)
    cases = {**const.development_cases, **const.battery_cases}
    if direction not in const.directions:
        raise BatteryError(f"direction {direction!r} is not in {list(const.directions)}")
    io = b4t.manifest(bundle)
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
        str(const.laps_driven if laps is None else laps),
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


# -- attempts --------------------------------------------------------------------------------------------


def needs_rerun(result: dict) -> bool:
    """No usable result, a starved run (gap over 40 ms or real-time factor under 0.8), an invalid
    ground truth, or a truth recording that ends before the count window."""
    if b5t.needs_rerun(result):
        return True
    return recording_too_short(result)


def recording_too_short(result: dict, laps_judged: int = 12, share: float = 0.5) -> bool:
    """The truth has the crossings of the count window but its recording ends before the window
    does (truth crossing `laps_judged` plus `share` of truth lap `laps_judged`)."""
    crossings = result.get("gt_lap_crossings_registered_s") or []
    laps = result.get("gt_lap_times_registered_s") or []
    if len(crossings) < laps_judged or len(laps) < laps_judged:
        return False  # fewer laps than the window: a short run, not a short recording
    last = result.get("truth_last_odom_s")
    end = float(crossings[laps_judged - 1]) + share * float(laps[laps_judged - 1])
    return last is None or float(last) < end


def meta_for(
    const: b8.Constants,
    freeze: dict | None,
    sha: str,
    job: tuple,
    attempt: int,
    command: list[str],
    mode: str,
) -> dict:
    name, condition, case, direction, seed = job
    return {
        "id": "B8",
        "mode": mode,
        "name": name,
        "condition": condition,
        "case": case,
        "direction": direction,
        "seed": seed,
        "attempt": attempt,
        "prereg_yaml_sha256": const.yaml_sha256,
        "freeze_commit": None if freeze is None else freeze["freeze_commit"],
        "git_sha": sha,
        "command": command[1:],
    }


def run_attempt(
    const: b8.Constants,
    args: argparse.Namespace,
    freeze: dict | None,
    sha: str,
    job: tuple,
    attempt: int,
    domain: int,
    out: Path,
    mode: str,
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
        laps=getattr(args, "laps", None) if mode == "development" else None,
        freeze_path=Path(args.freeze_path),
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
    result[META] = meta_for(const, freeze, sha, job, attempt, command, mode) | {
        "runner_exit": proc.returncode,
        "wall_s": round(time.monotonic() - started, 1),
        "starved": b5t.starved(result),
        "invalid_truth": b4t.invalid_truth(result),
        "recording_too_short": recording_too_short(result),
    }
    b4t.write_once(b4t.attempt_path(out, name, attempt), json.dumps(result, indent=1))
    return result


def pending_work(out: Path, jobs: list[tuple]) -> list[tuple[tuple, int]]:
    """(job, attempt number) still to run; the accepted result of a finished job is written once."""
    todo = []
    for job in jobs:
        name = job[0]
        final = out / f"{name}.json"
        if final.exists():
            continue
        done = b4t.existing_attempts(out, name)
        if not done:
            todo.append((job, 1))
            continue
        last = json.loads(done[-1].read_text())
        k = len(done)
        if needs_rerun(last) and k < MAX_ATTEMPTS:
            todo.append((job, k + 1))
            continue
        attempts = []
        for i, p in enumerate(done, start=1):
            doc = json.loads(p.read_text())
            attempts.append(
                {
                    "attempt": i,
                    "file": p.name,
                    "starved": bool(doc[META]["starved"]),
                    "invalid_truth": bool(doc[META].get("invalid_truth")),
                    "recording_too_short": bool(doc[META].get("recording_too_short")),
                    "truth_invalid_reason": doc.get("truth_invalid_reason", ""),
                    "max_gap_s": (doc.get("drive_raw") or {}).get("max_gap_s"),
                    "real_time_factor": doc.get("real_time_factor"),
                    "error": doc.get("error", ""),
                }
            )
        last[META]["attempts"] = attempts
        last[META]["accepted_attempt"] = k
        last[META]["accepted_while_starved"] = bool(last[META]["starved"])
        last[META]["accepted_while_invalid"] = bool(last[META]["invalid_truth"])
        last[META]["accepted_while_short"] = bool(last[META].get("recording_too_short"))
        b4t.write_once(final, json.dumps(last, indent=1))
    return todo


def run_jobs(
    const: b8.Constants,
    args: argparse.Namespace,
    freeze: dict | None,
    sha: str,
    jobs: list[tuple],
    out: Path,
    workers: int,
    mode: str,
) -> None:
    """Attempt 1 of every job with `workers` at a time, then every rerun alone."""
    lock = threading.Lock()
    while True:
        todo = pending_work(out, jobs)
        if not todo:
            return
        first = [t for t in todo if t[1] == 1]
        batch = first if first else todo[:1]
        n_workers = min(workers, len(batch)) if first else 1
        print(f"{len(batch)} attempts, {n_workers} at a time -> {out}", flush=True)
        domains: queue.Queue[int] = queue.Queue()
        for k in range(n_workers):
            domains.put(args.domain_base + k)
        failures: list[BaseException] = []
        pending: queue.Queue[tuple] = queue.Queue()
        for item in batch:
            pending.put(item)

        def worker(
            pending: queue.Queue[tuple] = pending,
            domains: queue.Queue[int] = domains,
            failures: list[BaseException] = failures,
        ) -> None:
            while True:
                try:
                    job, attempt = pending.get_nowait()
                except queue.Empty:
                    return
                domain = domains.get()
                try:
                    try:
                        result = run_attempt(
                            const, args, freeze, sha, job, attempt, domain, out, mode
                        )
                    except BaseException as exc:  # noqa: BLE001 - a dead worker (argparse's SystemExit too)
                        failures.append(exc)
                        return
                    with lock:
                        print(
                            f"{job[0]} attempt {attempt}: "
                            f"{len(result.get('gt_lap_times_registered_s', []))} laps, "
                            f"starved {b5t.starved(result)}, truth_valid "
                            f"{result.get('truth_valid')} {result.get('truth_invalid_reason', '')}, "
                            f"short {recording_too_short(result)}, "
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
        if failures:
            raise BatteryError(
                f"a worker failed, the battery stops: {failures[0]!r}"
            ) from failures[0]


def make_jobs(
    conditions: list[str], cases: list[str], seeds: list[int], directions: tuple[str, ...]
) -> list[tuple]:
    return b4t.make_jobs(conditions, cases, seeds, directions)


# -- the drop rule -----------------------------------------------------------------------------------------


def drop_decision(out: Path, const: b8.Constants) -> b8.DropResult:
    """The registered drop rule on the accepted runs of condition a, seed 61. A run that failed to
    execute, or whose truth is invalid, has no laps."""
    rule = const.drop_rule
    cond, seed = str(rule["condition"]), int(rule["seed"])
    laps: dict[str, int] = {}
    for case in const.battery_cases:
        for direction in const.directions:
            name = f"{cond}_{case}_{direction}_{seed}"
            path = out / f"{name}.json"
            if not path.exists():
                raise BatteryError(f"{name} has not run: the drop rule cannot be applied")
            doc = json.loads(path.read_text())
            n = len(doc.get("gt_lap_times_registered_s", [])) if not doc.get("error") else 0
            if doc.get("truth_valid") is False:
                n = 0
            laps[f"{case}_{direction}"] = n
    return b8.drop_rule(laps, const)


def drop_to_json(d: b8.DropResult) -> dict:
    return {"dropped": list(d.dropped), "decided": d.decided, "reason": d.reason, "laps": d.laps}


# -- the commands ----------------------------------------------------------------------------------------


def check_freeze_cmd(args: argparse.Namespace) -> None:
    const = b8.load_constants()
    doc = require_freeze(Path(args.freeze_path), verify_git=True)
    print(
        f"freeze OK: commit {doc['freeze_commit']}, yaml sha256 {const.yaml_sha256}, "
        f"HEAD {b4t.git_sha()}"
    )


def run_cmd(args: argparse.Namespace) -> None:
    const = b8.load_constants()
    if args.workers < 1 or args.workers > MAX_WORKERS:
        raise BatteryError(f"--workers must be 1 to {MAX_WORKERS} (host limit)")
    freeze = require_freeze(Path(args.freeze_path))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    b4t.check_clean()
    sha = b4t.git_sha()
    seeds = list(const.battery_seeds)
    all_cases = list(const.battery_cases)
    first = make_jobs(["a"], all_cases, [FIRST_SEED], const.directions)
    for job in first:  # validates every command before anything starts
        command_for(
            const, job[1], job[2], job[3], job[4], args.domain_base, out, out,
            Path(args.bundle), Path(args.data_root), freeze_path=Path(args.freeze_path),
        )  # fmt: skip
    print(
        f"B8: git {sha}, freeze {freeze['freeze_commit']}, yaml sha256 {const.yaml_sha256}",
        flush=True,
    )
    run_jobs(const, args, freeze, sha, first, out, args.workers, "battery")
    drops_path = out / DROPS_FILE
    if not drops_path.exists():
        b4t.write_once(drops_path, json.dumps(drop_to_json(drop_decision(out, const)), indent=1))
    drops = json.loads(drops_path.read_text())
    print(f"drop rule: {drops}", flush=True)
    if not drops["decided"]:
        raise BatteryError(
            f"nothing is decided: {drops['reason']}. Stop and tell the owner (pre-registration "
            "section 2)."
        )
    cases = [c for c in all_cases if c not in drops["dropped"]]
    rest = [
        j
        for j in make_jobs(list(b8.B8_CONDITIONS), cases, seeds, const.directions)
        if not (j[1] == "a" and j[4] == FIRST_SEED)
    ]
    run_jobs(const, args, freeze, sha, rest, out, args.workers, "battery")


def dev_run_cmd(args: argparse.Namespace) -> None:
    """Development runs: development cases on development seeds, no freeze file, labelled."""
    const = b8.load_constants()
    if args.workers < 1 or args.workers > MAX_WORKERS:
        raise BatteryError(f"--workers must be 1 to {MAX_WORKERS} (host limit)")
    cases = args.cases.split(",")
    seeds = [int(s) for s in args.seeds.split(",")]
    for case in cases:
        for seed in seeds:
            if needs_freeze(const, case, seed):
                raise BatteryError(f"{case} seed {seed} is a B8 run: not a development run")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    sha = b4t.git_sha() if os.environ.get(GIT_SHA_ENV) or _git("rev-parse", "HEAD").stdout else ""
    sha = sha or "unknown"
    jobs = make_jobs(list(args.conditions), cases, seeds, const.directions)
    for job in jobs:
        command_for(
            const, job[1], job[2], job[3], job[4], args.domain_base, out, out,
            Path(args.bundle), Path(args.data_root), freeze_path=Path(args.freeze_path),
        )  # fmt: skip
    print(f"B8 DEVELOPMENT runs (never evidence): git {sha} -> {out}", flush=True)
    run_jobs(const, args, None, sha, jobs, out, args.workers, "development")


# -- the report ------------------------------------------------------------------------------------------


def missed_flags(out: Path, name: str, accepted_attempt: int) -> int:
    """Laps the lap node flagged missed gate in the accepted attempt (its log's lap lines)."""
    log = out / "work" / f"{name}.attempt{accepted_attempt}" / "lap_memory_node.log"
    if not log.exists():
        return 0
    return sum(1 for line in log.read_text(errors="replace").splitlines() if "missed gate" in line)


def report_text(out: Path) -> str:
    const = b8.load_constants()
    drops_path = out / DROPS_FILE
    drops = json.loads(drops_path.read_text()) if drops_path.exists() else None
    dropped: list[str] = drops["dropped"] if drops else []
    cases = [c for c in const.battery_cases if c not in dropped]
    runs = [r for r in b8.load_runs(out, const) if r.case in cases]
    every = {p.stem: json.loads(p.read_text()) for p in sorted(out.glob("*_*_*_*.json"))}
    docs = {n: d for n, d in every.items() if n.split("_")[1] in cases}
    metas = [d[META] for d in every.values()]
    lines = [
        "# Battery B8 report",
        "",
        f"constants file sha256 now: {const.yaml_sha256} (registered {b8.REGISTERED_YAML_SHA256})",
        f"constants file sha256 in the run files: {sorted({m['prereg_yaml_sha256'] for m in metas})}",
        f"freeze commit in the run files: {sorted({str(m['freeze_commit']) for m in metas})}",
        f"git sha of the checkout in the run files: {sorted({m['git_sha'] for m in metas})}",
        f"runs: {len(runs)}",
    ]
    if drops:
        lines.append(f"drop rule: {drops}")
        if not drops["decided"]:
            lines.append(f"NOTHING IS DECIDED: {drops['reason']}. The owner is told.")
            return "\n".join(lines)
    else:
        lines.append("drop rule: not applied")
    lines += ["", "## Attempts", ""]
    unusual = [
        m
        for m in metas
        if m.get("accepted_attempt", 1) > 1
        or m.get("accepted_while_starved")
        or m.get("accepted_while_invalid")
        or m.get("accepted_while_short")
    ]
    for m in sorted(unusual, key=lambda x: x["name"]):
        lines.append(f"{m['name']}: {m['attempts']}")
    if not unusual:
        lines.append("every run was accepted on its first attempt, none starved or invalid")
    lines += ["", "## Runs", "", b4.run_table(runs), ""]
    want = {
        (c, case, d, s)
        for c in b8.B8_CONDITIONS
        for case in cases
        for d in const.directions
        for s in const.battery_seeds
    }
    have = {(r.condition, r.case, r.direction, r.seed) for r in runs}
    if want != have:
        lines.append(
            f"INCOMPLETE: {len(want - have)} runs missing, {len(have - want)} unexpected; "
            "no decision is made"
        )
        return "\n".join(lines)
    lines.append("## Decisions")
    decisions = {"D8-S0": b8.d8_s0(runs, const), "D8-S1": b8.d8_s1(runs, const)}
    for dec in decisions.values():
        lines += ["", str(dec)]
    both = all(d.passed for d in decisions.values())
    lines += [
        "",
        "## What this lets D do",
        "",
        (
            "D8-S0 and D8-S1 both PASS on B8 at the freeze commit. By section 6 of the "
            "pre-registration this decides nothing about driving by itself: it needs the owner's "
            "yes to question 1, the freeze commit on the car, lap-1 cap 0.5 m/s, 10 laps per "
            "direction, and D8-C0 passed earlier in the same session. D v1 does not drive."
            if both
            else "D8-S0 or D8-S1 FAILED: D does not drive, the cap stays fixed "
            "(pre-registration section 6). A failed gate is a result: no tuning, no re-scoring."
        ),
        "",
        "## Reported without a threshold",
        "",
        "Per case and condition:",
        "",
        b8.case_table(b8.case_rows(docs, const)),
        "",
        "Lap 1, system against the registered truth:",
        "",
        b8.lap1_table(b8.lap1_rows(docs)),
        "",
        "The offset of crossing 12:",
        "",
        b8.crossing12_table(b8.crossing12_rows(docs, const)),
        "",
        f"Condition b against the same tolerances: {b8.condition_b_laps(runs, const)}",
        "",
    ]
    flags = {
        n: missed_flags(out, n, int(d[META].get("accepted_attempt", 1))) for n, d in docs.items()
    }
    lines += [f"- {x}" for x in b8.missed_flags_per_condition(flags)]
    return "\n".join(lines)


def _g(x) -> str:  # type: ignore[no-untyped-def]
    return "-" if x is None else format(float(x), ".3f")


def _dev_rows(runs, docs, const) -> tuple[list[str], dict]:  # type: ignore[no-untyped-def]
    """Table rows of development runs and their totals."""
    lines = [
        (
            "| run | stopped | contacts | laps truth | laps system | count rule | judged laps "
            "outside | largest judged error s | sources | missed | verdicts | lap 1 d_beta | "
            "pair share lap 1 | gap unique min / median | gap ambiguous max |"
        ),
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    total = outside = count_bad = 0
    for r in runs:
        ok, detail = b8.count_rule(r, const)
        errs = b8.judged_errors(r, const)
        out_laps = [k for k, e, tol, _ in errs if e > tol]
        total += len(errs)
        outside += len(out_laps)
        count_bad += not ok
        doc = docs.get(f"{r.condition}_{r.case}_{r.direction}_{r.seed}", {})
        st = doc.get("lap_status") or {}
        sources = list(st.get("lap_line_sources") or [])
        srcs = {s: sources.count(s) for s in sorted(set(sources))}
        verdicts = sorted(set(st.get("lap_start_verdicts") or []))
        shares = st.get("lap_pair_shares") or []
        d_beta = st.get("lap1_d_beta_rad")
        largest = max((e for _, e, _, _ in errs), default=float("nan"))
        lines.append(
            f"| {r.condition}_{r.case}_{r.direction}_{r.seed} | {doc.get('stopped_because')} | "
            f"{doc.get('wall_contacts')} | {r.laps} | {len(r.system_lap_times_s)} | "
            f"{'ok' if ok else 'FAIL ' + detail} | {out_laps} | {largest:.2f} | {srcs} | "
            f"{st.get('missed_gate_laps')} | {verdicts} | "
            f"{'-' if d_beta is None else format(d_beta, '+.2f')} | "
            f"{'-' if not shares else format(shares[0], '.2f')} | "
            f"{_g(st.get('unique_gap_min'))} / {_g(st.get('unique_gap_median'))} | "
            f"{_g(st.get('ambiguous_gap_max'))} |"
        )
    return lines, {"total": total, "outside": outside, "count_bad": count_bad}


def rd_clean(probe, docs, const) -> bool:  # type: ignore[no-untyped-def]
    """C1 of brief 50c: 12 of 12 RD runs clean: the count rule holds, no judged lap outside the
    class tolerance, no lap flagged missed gate, one line source per run."""
    if len(probe) != 12:
        return False
    for r in probe:
        doc = docs.get(f"{r.condition}_{r.case}_{r.direction}_{r.seed}", {})
        st = doc.get("lap_status") or {}
        errs = b8.judged_errors(r, const)
        if (
            not b8.count_rule(r, const)[0]
            or any(e > tol for _, e, tol, _ in errs)
            or (st.get("missed_gate_laps") or [])
            or len(set(st.get("lap_line_sources") or [])) != 1
            or r.error
        ):
            return False
    return True


def dev_dropped_cases(runs, const) -> dict[str, str]:  # type: ignore[no-untyped-def]
    """Owner decision 14 (brief 50b): A5 applies B8's drop rule to the development cases. A case is
    dropped when its condition a run of the first development seed has fewer than `min_laps` truth
    laps in either direction; the reason says which run. Probe cases are not judged at all."""
    need = int(const.drop_rule["min_laps"])
    first_seed = min((r.seed for r in runs), default=0)
    dropped: dict[str, str] = {}
    for r in runs:
        if r.case in b8.PROBE_CASES or r.condition != "a" or r.seed != first_seed:
            continue
        if r.laps < need:
            dropped.setdefault(
                r.case, f"{r.condition}_{r.case}_{r.direction}_{r.seed}: {r.laps} laps"
            )
    return dropped


def dev_report_text(out: Path) -> str:
    """A5: the development runs against the count rule and the class tolerance (aliased class for
    P4, OD and ND), with B8's drop rule applied to the development cases (owner decision 14:
    stalled condition a runs drop their case, A5 judges the rest). The probe case RD is reported
    and decides nothing. Labelled development: never evidence."""
    const = b8.load_constants()
    runs = b8.load_runs(out, const, mode="development")
    docs = {p.stem: json.loads(p.read_text()) for p in sorted(out.glob("*_*_*_*.json"))}
    probe = [r for r in runs if r.case in b8.PROBE_CASES]
    judged_all = [r for r in runs if r.case not in b8.PROBE_CASES]
    dropped = dev_dropped_cases(judged_all, const)
    judged = [r for r in judged_all if r.case not in dropped]
    lines = [
        "# Development runs of briefs 50 and 50b (A5): DEVELOPMENT, never evidence",
        "",
        f"constants file sha256: {const.yaml_sha256}",
        f"drop rule (owner decision 14): dropped {dropped or 'none'}",
        "",
    ]
    rows, tot = _dev_rows(judged, docs, const)
    lines += rows
    share = tot["outside"] / tot["total"] if tot["total"] else float("nan")
    errored = [r for r in judged if r.error]
    verdict = tot["count_bad"] == 0 and share <= 0.02 and not errored and len(judged) > 0
    lines += [
        "",
        (
            f"A5 runs {len(judged)}, count rule broken in {tot['count_bad']}, {tot['outside']} of "
            f"{tot['total']} judged laps outside the class tolerance ({share:.4f}), runs with an "
            f"error {len(errored)}"
        ),
        f"A5: {'PASS' if verdict else 'FAIL'}",
    ]
    if dropped:
        drows, _ = _dev_rows([r for r in judged_all if r.case in dropped], docs, const)
        lines += ["", "Dropped by the rule (reported, not judged):", ""] + drows
    if probe:
        prows, ptot = _dev_rows(probe, docs, const)
        stalls = [r for r in probe if len(r.gt_lap_times_s) < const.laps_driven]
        lines += [
            "",
            "Stall probe RD (report only, decides nothing by rule):",
            "",
            *prows,
            "",
            (
                f"RD runs {len(probe)}, stalled or short {len(stalls)}, count rule broken in "
                f"{ptot['count_bad']}, {ptot['outside']} of {ptot['total']} judged laps outside "
                "the class tolerance"
            ),
            f"C1: {'PASS' if rd_clean(probe, docs, const) else 'FAIL'}",
        ]
    return "\n".join(lines)


def report_cmd(args: argparse.Namespace) -> None:
    out = Path(args.out)
    text = report_text(out)
    print(text)
    (out / "report.md").write_text(text + "\n")


def dev_report_cmd(args: argparse.Namespace) -> None:
    out = Path(args.out)
    text = dev_report_text(out)
    print(text)
    (out / "dev_report.md").write_text(text + "\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter
    )
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("check-freeze", "run", "report", "dev-run", "dev-report"):
        p = sub.add_parser(name)
        if name != "check-freeze":
            p.add_argument("--out", required=True)
        if name in ("check-freeze", "run", "dev-run"):
            p.add_argument("--freeze-path", default=str(FREEZE_PATH))
        if name in ("run", "dev-run"):
            p.add_argument("--workers", type=int, default=3)
            p.add_argument("--domain-base", type=int, default=120)
            p.add_argument("--bundle", default=str(b4t.DEFAULT_BUNDLE))
            p.add_argument("--data-root", default=str(b4t.DEFAULT_DATA_ROOT))
        if name == "dev-run":
            p.add_argument("--cases", default="P4,OD,ND")
            p.add_argument("--seeds", default="11,12,13")
            p.add_argument("--conditions", default="ac")
            p.add_argument(
                "--laps", type=int, help="development only: stop after this many laps (default 13)"
            )
    args = parser.parse_args(argv)
    cmds = {
        "check-freeze": check_freeze_cmd,
        "run": run_cmd,
        "report": report_cmd,
        "dev-run": dev_run_cmd,
        "dev-report": dev_report_cmd,
    }
    try:
        cmds[args.command](args)
    except (BatteryError, b8.PreregError) as exc:
        print(f"REFUSED: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
