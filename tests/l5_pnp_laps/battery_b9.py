"""Run battery B9 of the camera lap source pre-registration and report D9-S0, D9-S1 and D9-S2, and
run the development checks of brief 51 (E4, E5) on development cases.

    python3 tests/l5_pnp_laps/battery_b9.py check-freeze
    python3 tests/l5_pnp_laps/battery_b9.py run --out DIR --workers 3
    python3 tests/l5_pnp_laps/battery_b9.py report --out DIR
    python3 tests/l5_pnp_laps/battery_b9.py dev-run --out DIR --cases P1,P2,P4 --seeds 11 --conditions bc
    python3 tests/l5_pnp_laps/battery_b9.py dev-report --out DIR

Needs the ROS workspace sourced (see .github/scripts/l5_pnp_laps_test.sh) and f1tenth_gym.

Cases U1 to U3, seeds 71 and 72, ccw and cw, 13 driven laps (12 judged) and the conditions a (D
off, cap 0.5, seed 71 only), b (D off, ceiling), c (D v0, `lap_source truth`) and f (the same with
the fault plan) are read from `evaluation/prereg/pnp-v0.5-laps-camera.yaml`, block
`lap_battery_b9`, whose SHA-256 is a constant of `lap_battery_b9.py` and is written into every run
JSON and the report. The oracle is the simulator's ground-truth line crossing
(`tests/l5_reactive_bench/truth_lap_source.py`), so every run is on simulated time.

NO U CASE, NO SEED 71 OR 72 AND NEITHER EVALUATION BAG RUNS OR IS OPENED UNLESS
`evaluation/prereg/freeze/b9.json` EXISTS with the freeze commit and the yaml's hash
(`require_freeze`, called by `command_for` and `guard_bag`: there is no other way to build a B9
command). Development runs (`dev-run`) are the development cases on the development seeds 11 to 13
only, in a directory of their own, labelled development; every other case or seed (B5's 31 and 32,
B6's 41 and 42, B7's 51 to 53, B8's 61 to 63) is refused. `check-freeze` verifies the file against
git (the freeze commit is an ancestor of HEAD and the code under test and the scoring harness are
unchanged since) and is run on the host before the container runs the battery with B4_GIT_SHA set
to the SHA it verified.

Order: the condition a runs of seed 71 first (both directions of every case); the drop rule
(`lap_battery_b9.drop_rule`, `drops.json`, written once) then says which case the rest of the
battery runs without, or that nothing is decided. Every attempt is written once to
`<out>/attempts/<name>.attempt<k>.json`. A run is starved (a /drive_raw gap over 40 ms, or a
real-time factor under 0.8), has an invalid ground truth or a truth recording that ends before the
count window: it is rerun alone, at most 3 attempts in all, every attempt kept; still invalid counts
as failed to execute. At most 3 workers.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
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
import lap_battery_b9 as b9
import yaml

REPO_ROOT = HERE.parents[1]
RUNNER = b4t.RUNNER
FREEZE_PATH = REPO_ROOT / "evaluation" / "prereg" / "freeze" / "b9.json"
#: The code under test and the scoring harness: the lap node and the external rule, the planner,
#: the runner with the truth oracle, and the battery's own decision and run files.
GUARDED_PATHS = (
    *b4t.GUARDED_PATHS,
    "tests/l5_pnp_laps/lap_battery_b9.py",
    "tests/l5_pnp_laps/battery_b9.py",
)
META = "battery_b9"
MAX_WORKERS = 3
MAX_ATTEMPTS = b4t.MAX_ATTEMPTS
BatteryError = b4t.BatteryError
DROPS_FILE = "drops.json"
GIT_SHA_ENV = b4t.GIT_SHA_ENV
#: Seeds of every other registered attempt: never run again, by anyone, for any reason.
OTHER_REGISTERED_SEEDS = (31, 32, 41, 42, 51, 52, 53, 61, 62, 63)
FIRST_SEED = 71  # the seed whose condition a runs decide the drop rule
#: The two car bags of D9-B0: not opened before the freeze file exists.
EVALUATION_BAGS = ("2026-11-16T22-45-39_car_teleop", "2026-11-16T23-04-09_car_teleop")
DEV_RESULTS = Path("/Users/cameronjim/code/car-data/pnp/battery/v0.5-camera-dev")

# -- the freeze guard ------------------------------------------------------------------------------


def read_freeze(path: Path = FREEZE_PATH) -> dict:
    if not path.exists():
        raise BatteryError(f"{path} does not exist: B9 may not run before the freeze commit")
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
    if doc.get("yaml_sha256") != b9.REGISTERED_YAML_SHA256:
        raise BatteryError(
            f"{path} names yaml hash {doc.get('yaml_sha256')}, the registered one is "
            f"{b9.REGISTERED_YAML_SHA256}"
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


def guard_bag(name: str, freeze_path: Path = FREEZE_PATH, verify_git: bool | None = None) -> None:
    """An evaluation bag of D9-B0 is not opened before the freeze file exists."""
    if name in EVALUATION_BAGS:
        try:
            require_freeze(freeze_path, verify_git=verify_git)
        except BatteryError as exc:
            raise BatteryError(
                f"{name} is an evaluation bag, not opened before the freeze: {exc}"
            ) from exc


def dev_cases(const: b9.Constants) -> dict[str, dict]:
    """The development cases the battery may run on seeds 11 to 13, by name, from the registered
    files that define them (each read under its own hash check where the repo keeps one)."""
    cases: dict[str, dict] = {}
    d3 = b4.load_constants()
    cases.update(d3.development_cases)
    cases.update(d3.battery_cases)  # N1 to N3 of v0.3: development data for this battery
    v05 = b9.V05_LAPS_YAML
    if b9.sha256_file(v05) != b9.V05_LAPS_SHA256:
        raise BatteryError(f"{v05} is not the registered v0.5 laps file")
    cases.update(yaml.safe_load(v05.read_text())["lap_battery_b7"]["cases"])
    ring = yaml.safe_load(b9.V05_RING_YAML.read_text())["lap_battery_b8"]
    cases["ND"] = dict(ring["development"]["cases"]["ND"])
    v04 = yaml.safe_load((REPO_ROOT / "evaluation" / "prereg" / "pnp-v0.4.yaml").read_text())
    cases["OD"] = dict(v04["lap_battery_b5"]["development"]["cases"]["OD"])
    return {k: v for k, v in cases.items() if k in const.development_case_names}


def needs_freeze(const: b9.Constants, case: str, seed: int) -> bool:
    """A B9 case or a B9 seed: the freeze file is required. A development case on a development seed
    is not, and every combination that is neither is refused."""
    battery = case in const.battery_cases or seed in const.battery_seeds
    if battery:
        if case not in const.battery_cases or seed not in const.battery_seeds:
            raise BatteryError(
                f"case {case!r} with seed {seed}: a B9 case runs on seeds "
                f"{list(const.battery_seeds)} only and a B9 seed runs on a B9 case only"
            )
        return True
    if seed in OTHER_REGISTERED_SEEDS:
        raise BatteryError(f"seed {seed} belongs to another registered attempt: never run again")
    if case in dev_cases(const) and seed in const.development_seeds:
        return False
    raise BatteryError(
        f"case {case!r} with seed {seed} is neither a B9 run nor a development run "
        f"(development: {sorted(dev_cases(const))} on seeds {list(const.development_seeds)})"
    )


# -- cases and commands ------------------------------------------------------------------------------


def case_track(spec: dict, data_root: Path) -> tuple[str, dict]:
    """(track, track params) of a yaml case: the keys the yaml adds (`class`) are not the track's."""
    track, params = b4t.case_track(spec, data_root)
    return track, {k: v for k, v in params.items() if k != "class"}


def condition_flags(const: b9.Constants, condition: str, ceiling: float) -> list[str]:
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
    flags = [
        "--lap-memory",
        mode,
        "--v-max-mps",
        str(ceiling),
        "--v-lap1-mps",
        str(const.lap1_cap_mps),
        "--lap-source",
        str(spec["lap_source"]),
    ]
    if spec.get("faults") == "fault_plan":
        flags += ["--lap-truth-faults-json", json.dumps(const.fault_plan, sort_keys=True)]
    return flags


def command_for(
    const: b9.Constants,
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
    verify_git: bool | None = None,
) -> list[str]:
    """The runner's argv for one run. Refuses a B9 case or seed without the freeze file, a
    development run of anything but a development case on a development seed, and an unknown
    direction or condition."""
    if needs_freeze(const, case, seed):
        require_freeze(freeze_path, verify_git=verify_git)
    cases = {**dev_cases(const), **const.battery_cases}
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
        "--sim-time",  # the oracle is on simulated time: every run is
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


def recording_too_short(result: dict, laps_judged: int = 12) -> bool:
    """The truth has the crossings of the count window but its recording ends before truth
    crossing `laps_judged`."""
    crossings = result.get("gt_lap_crossings_registered_s") or []
    if len(crossings) < laps_judged:
        return False  # fewer laps than the window: a short run, not a short recording
    last = result.get("truth_last_odom_s")
    return last is None or float(last) < float(crossings[laps_judged - 1])


def needs_rerun(result: dict) -> bool:
    """No usable result, a starved run (gap over 40 ms or real-time factor under 0.8), an invalid
    ground truth, or a truth recording that ends before the count window."""
    return bool(b5t.needs_rerun(result) or recording_too_short(result))


def make_jobs(const: b9.Constants, dropped: tuple[str, ...] = ()) -> list[tuple]:
    """The 42 runs (a: seed 71 only; b, c, f: both seeds) of the cases not dropped, condition a
    first."""
    cases = [c for c in const.battery_cases if c not in dropped]
    jobs: list[tuple] = []
    for cond in b9.B9_CONDITIONS:
        seeds = [int(s) for s in const.conditions[cond].get("seeds", const.battery_seeds)]
        jobs += b4t.make_jobs([cond], cases, seeds, const.directions)
    return jobs


def meta_for(
    const: b9.Constants,
    freeze: dict | None,
    sha: str,
    job: tuple,
    attempt: int,
    command: list[str],
    mode: str,
) -> dict:
    name, condition, case, direction, seed = job
    return {
        "id": "B9",
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
    const: b9.Constants,
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
    const: b9.Constants,
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


# -- the drop rule -----------------------------------------------------------------------------------------


def drop_decision(out: Path, const: b9.Constants) -> b9.DropResult:
    """The registered drop rule on the accepted runs of condition a, seed 71. A run that failed to
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
    return b9.drop_rule(laps, const)


def drop_to_json(d: b9.DropResult) -> dict:
    return {"dropped": list(d.dropped), "decided": d.decided, "reason": d.reason, "laps": d.laps}


# -- the commands ----------------------------------------------------------------------------------------


def check_freeze_cmd(args: argparse.Namespace) -> None:
    const = b9.load_constants()
    doc = require_freeze(Path(args.freeze_path), verify_git=True)
    print(
        f"freeze OK: commit {doc['freeze_commit']}, yaml sha256 {const.yaml_sha256}, "
        f"HEAD {b4t.git_sha()}"
    )


def run_cmd(args: argparse.Namespace) -> None:
    const = b9.load_constants()
    if args.workers < 1 or args.workers > MAX_WORKERS:
        raise BatteryError(f"--workers must be 1 to {MAX_WORKERS} (host limit)")
    freeze = require_freeze(Path(args.freeze_path))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    b4t.check_clean()
    sha = b4t.git_sha()
    first = [j for j in make_jobs(const) if j[1] == "a"]
    for job in make_jobs(const):  # validates every command before anything starts
        command_for(
            const, job[1], job[2], job[3], job[4], args.domain_base, out, out,
            Path(args.bundle), Path(args.data_root), freeze_path=Path(args.freeze_path),
        )  # fmt: skip
    print(
        f"B9: git {sha}, freeze {freeze['freeze_commit']}, yaml sha256 {const.yaml_sha256}",
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
    rest = [j for j in make_jobs(const, tuple(drops["dropped"])) if j[1] != "a"]
    run_jobs(const, args, freeze, sha, rest, out, args.workers, "battery")


def dev_run_cmd(args: argparse.Namespace) -> None:
    """Development runs: development cases on development seeds, no freeze file, labelled."""
    const = b9.load_constants()
    if args.workers < 1 or args.workers > MAX_WORKERS:
        raise BatteryError(f"--workers must be 1 to {MAX_WORKERS} (host limit)")
    cases = args.cases.split(",")
    seeds = [int(s) for s in args.seeds.split(",")]
    for case in cases:
        for seed in seeds:
            if needs_freeze(const, case, seed):
                raise BatteryError(f"{case} seed {seed} is a B9 run: not a development run")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    sha = b4t.git_sha() if os.environ.get(GIT_SHA_ENV) or _git("rev-parse", "HEAD").stdout else ""
    sha = sha or "unknown"
    jobs = b4t.make_jobs(list(args.conditions), cases, seeds, const.directions)
    for job in jobs:
        command_for(
            const, job[1], job[2], job[3], job[4], args.domain_base, out, out,
            Path(args.bundle), Path(args.data_root), freeze_path=Path(args.freeze_path),
        )  # fmt: skip
    print(f"B9 DEVELOPMENT runs (never evidence): git {sha} -> {out}", flush=True)
    run_jobs(const, args, None, sha, jobs, out, args.workers, "development")


# -- the report ------------------------------------------------------------------------------------------


def verdict(d) -> str:
    return {True: "PASS", False: "FAIL", None: "INSUFFICIENT"}[d.passed]


def _decide(fn, runs, const, need: str):
    """A decision, or an INSUFFICIENT one when the runs it needs are not there."""
    try:
        return fn(runs, const)
    except b4.MissingPairError as exc:
        return b9.Decision(fn.__name__.upper().replace("_", "-"), None, {}, f"{need}: {exc}")


def decisions_text(runs, const) -> list[str]:
    out = []
    for name, fn, need in (
        ("D9-S0", b9.d9_s0, "needs condition c runs"),
        ("D9-S1", b9.d9_s1, "needs conditions b and c runs"),
        ("D9-S2", b9.d9_s2, "needs condition f runs"),
    ):
        d = _decide(fn, runs, const, need)
        out.append(f"### {name}: {verdict(d)}" + (f" ({d.note})" if d.note else ""))
        out += [
            "",
            *[f"- {'ok' if ok else 'FAIL'}: {k}: {v}" for k, (ok, v) in d.items.items()],
            "",
        ]
    return out


def reported_text(runs) -> list[str]:
    out = ["### Reported, no threshold", ""]
    rows = b9.late_lap_rows(runs)
    out += [
        "Late lap time (median of laps 8 to 10), condition f against c:",
        "",
        "| run | T_late(f) s | T_late(c) s | f / c |",
        "|---|---|---|---|",
        *[
            f"| {r['run']} | {r['t_late_f_s']:.2f} | {r['t_late_c_s']:.2f} | {r['ratio']:.3f} |"
            for r in rows
        ],
        "",
        "Event latency, reference length and every doubtful lap with its reasons:",
        "",
        "| run | latency max s | reference length m | confirmed | doubtful laps | rejected |",
        "|---|---|---|---|---|---|",
    ]
    for r in b9.run_detail_rows(runs):
        lat, ref = r["event_latency_s_max"], r["l_ref_m"]
        out.append(
            f"| {r['run']} | {'-' if lat is None else format(lat, '.3f')} | "
            f"{'-' if ref is None else format(ref, '.2f')} | {r['confirmed_laps']} | "
            f"{r['doubtful']} | {r['rejected']} |"
        )
    return out


def report_text(out: Path) -> str:
    const = b9.load_constants()
    drops_path = out / DROPS_FILE
    drops = json.loads(drops_path.read_text()) if drops_path.exists() else None
    runs = b9.load_runs(out, const)
    lines = [
        "# B9: the camera lap source, truth oracle (sim)",
        "",
        f"yaml sha256 {const.yaml_sha256}; {len(runs)} run files; drop rule: {drops}",
        "",
    ]
    lines += decisions_text(runs, const)
    lines += reported_text(runs)
    return "\n".join(lines)


@dataclasses.dataclass(frozen=True)
class DevConstants(b9.Constants):
    """Development: 6 runs per condition (P1, P2, P4 both ways on seed 11): 6 of 6 complete."""

    def required_complete(self, n_runs: int) -> int:
        return n_runs


def oracle_rows(runs) -> list[dict]:
    """E5: the oracle against the registered truth, per run: the count of published truth events,
    the worst crossing difference (s) over crossings 1 to 12."""
    rows = []
    for r in runs:
        truth = r.extras.get("truth_crossings_s", ())
        events = b9.truth_event_times(r)
        worst = 0.0
        count = 0
        for k in range(1, min(len(truth), 12) + 1):
            got = events.get(k, [])
            count += len(got)
            if got:
                worst = max(worst, abs(got[0] - truth[k - 1]))
            else:
                worst = math.inf
        rows.append(
            {
                "run": b9._label(r),
                "events_1_to_12": count,
                "truth_crossings": len(truth),
                "worst_s": worst,
            }
        )
    return rows


def dev_report_text(out: Path) -> str:
    const = b9.load_constants()
    dev = DevConstants(const.yaml_sha256, const.raw, const.doc, const.max_contacts)
    runs = b9.load_runs(out, const, mode="development")
    lines = [
        "# B9 development runs (never evidence)",
        "",
        f"yaml sha256 {const.yaml_sha256}; {len(runs)} run files",
        "",
        "## E4: D9-S0, D9-S1 (6 of 6) and D9-S2 on the development runs",
        "",
    ]
    lines += decisions_text(runs, dev)
    lines += ["## E5: the oracle against the registered truth", ""]
    rows = oracle_rows([r for r in runs if r.condition == "c"])
    lines += [
        "| run | oracle events for crossings 1 to 12 | truth crossings | worst difference s |",
        "|---|---|---|---|",
        *[
            f"| {r['run']} | {r['events_1_to_12']} | {r['truth_crossings']} | {r['worst_s']:.3f} |"
            for r in rows
        ],
        "",
        (
            f"every crossing within 0.10 s: {all(r['worst_s'] <= 0.10 for r in rows)} "
            "(reported; the gate asks 0.3 s)"
        ),
        "",
    ]
    lines += reported_text(runs)
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
            p.add_argument("--cases", default="P1,P2,P4")
            p.add_argument("--seeds", default="11")
            p.add_argument("--conditions", default="bc")
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
    except (BatteryError, b9.PreregError) as exc:
        print(f"REFUSED: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
