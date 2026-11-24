"""Run the development ablations M-D1b and battery B11 of the D v1 pre-registration (brief 53), and
report the candidate rule and the decisions D11-S0 to D11-S4.

    python3 tests/l5_pnp_laps/battery_b11.py dev-run --out DIR --conditions c,K1 --cases T,S1 --seeds 11
    python3 tests/l5_pnp_laps/battery_b11.py dev-report --out DIR
    python3 tests/l5_pnp_laps/battery_b11.py check-freeze
    python3 tests/l5_pnp_laps/battery_b11.py run --out DIR --workers 3
    python3 tests/l5_pnp_laps/battery_b11.py report --out DIR

Needs the ROS workspace sourced (see .github/scripts/l5_pnp_laps_test.sh) and f1tenth_gym.

The conditions, cases, seeds and thresholds are read from `evaluation/prereg/pnp-v0.6-laps-dv1.yaml`
and its addendum 1 under their registered hashes (`lap_battery_b11.load_constants`). Every run is on
the truth lap source through the start gate, as B10 ran it. D v1 runs with `--lap-memory v1` on that
source; the switches of brief 44 section 1 are passed as lap node and planner parameters (every
default reproduces main).

NO W CASE AND NO SEED 91 OR 92 RUNS UNLESS `evaluation/prereg/freeze/b11.json` EXISTS WITH THE
COMMIT AND THE YAML'S HASHES (`require_freeze`, called by `command_for`: there is no other way to
build a B11 command). Development runs are the development cases on seeds 11 to 13 only; every
other case or seed is refused (the seeds of every other registered attempt are spent).

Validity as B10: an attempt that is starved (a /drive_raw gap over 40 ms or a real-time factor
under 0.8), has an invalid truth or a truth recording that ends before lap 12 is rerun alone, at
most 3 attempts, every attempt kept; still invalid counts as failed to execute. At most 3 workers.
"""

from __future__ import annotations

import argparse
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
import battery_b10 as bt10
import lap_battery_b4 as b4
import lap_battery_b11 as b11

REPO_ROOT = HERE.parents[1]
RUNNER = b4t.RUNNER
FREEZE_PATH = REPO_ROOT / "evaluation" / "prereg" / "freeze" / "b11.json"
GUARDED_PATHS = (
    "ros_ws/src/racer_laps",
    "ros_ws/src/racer_planner",
    "tests/l5_reactive_bench",
    "tests/l5_pnp_laps/battery_b11.py",
    "tests/l5_pnp_laps/lap_battery_b11.py",
)
META = "battery_b11"
MAX_WORKERS = 3
MAX_ATTEMPTS = b4t.MAX_ATTEMPTS
DROPS_FILE = "drops.json"
GIT_SHA_ENV = b4t.GIT_SHA_ENV
#: Seeds of every earlier registered attempt: never run again.
OTHER_REGISTERED_SEEDS = (31, 32, 41, 42, 51, 52, 53, 61, 62, 63, 71, 72, 81, 82)
DEV_READY_AFTER_S = bt10.DEV_READY_AFTER_S
SPENT_RUNS = (
    Path("/Users/cameronjim/code/car-data/pnp/battery/v0.5-b9"),
    Path("/Users/cameronjim/code/car-data/pnp/battery/v0.5-b10"),
)

#: The switches of brief 44 section 1 per condition: (lap node parameters, planner parameters).
#: Every key not named here keeps its default (main's behaviour).
CANDIDATE_COMMON_LAP = {
    "zone_floor_mode": "cap_wins",
    "station_uncertainty": True,
    "reset_on_change": True,
}
CANDIDATE_COMMON_PLANNER = {"lap_station_mode": "track"}
SWITCHES: dict[str, tuple[dict, dict]] = {
    "m": ({}, {"lap_reward_mode": "keep_progress"}),
    "f": ({"value_profile_mode": "freeze_after_first_clean_lap"}, {}),
    "u": ({}, {}),
    "z": ({"zone_floor_mode": "cap_wins"}, {}),
    "K1": (
        dict(CANDIDATE_COMMON_LAP),
        {**CANDIDATE_COMMON_PLANNER, "lap_reward_mode": "keep_progress"},
    ),
    "K3": (
        {**CANDIDATE_COMMON_LAP, "value_profile_mode": "freeze_after_first_clean_lap"},
        {**CANDIDATE_COMMON_PLANNER, "lap_reward_mode": "replace"},
    ),
    "K2": (
        {**CANDIDATE_COMMON_LAP, "value_profile_mode": "update"},
        {**CANDIDATE_COMMON_PLANNER, "lap_reward_mode": "replace"},
    ),
}
#: The values tunable on development only (yaml `tunable_on_development_only`), chosen before any
#: development run and unchanged unless a dev-report says otherwise (then recorded in the freeze).
TUNED = {
    "station_uncertainty_rate": 0.02,  # half width per metre of wheel distance since the crossing
    "station_uncertainty_cap_m": 0.5,
    "reset_length_share": 0.08,  # the registered camera length band's 8 percent
    "curvature_bound_a_lat_mps2": 2.0,  # the planner's own a_lat_max_mps2
}
TUNED_LAP_KEYS = ("station_uncertainty_rate", "station_uncertainty_cap_m", "reset_length_share")


class BatteryError(RuntimeError):
    pass


# -- the freeze and the guards ---------------------------------------------------------------------------


def read_freeze(path: Path = FREEZE_PATH) -> dict:
    if not path.exists():
        raise BatteryError(f"{path} does not exist: B11 may not run before the freeze commit")
    try:
        doc = json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        raise BatteryError(f"{path} is not valid JSON: {exc}") from exc
    for key in ("commit_sha", "main_sha_contained"):
        v = doc.get(key, "")
        if not (isinstance(v, str) and len(v) == 40 and all(c in "0123456789abcdef" for c in v)):
            raise BatteryError(f"{path} has no 40 digit {key}")
    if doc.get("yaml_sha256") != b11.REGISTERED_YAML_SHA256:
        raise BatteryError(f"{path} names yaml hash {doc.get('yaml_sha256')}")
    if doc.get("addendum_sha256") != b11.ADDENDUM_SHA256:
        raise BatteryError(f"{path} names addendum hash {doc.get('addendum_sha256')}")
    if doc.get("candidate") not in b11.CANDIDATES:
        raise BatteryError(f"{path} names no candidate of {b11.CANDIDATES}")
    if not isinstance(doc.get("tuned_values"), dict):
        raise BatteryError(f"{path} has no tuned_values")
    return doc


def _git(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["git", "-c", "safe.directory=*", "-C", str(REPO_ROOT), *args],
        capture_output=True,
        text=True,
        check=False,
    )


def verify_freeze_in_git(doc: dict) -> None:
    """The freeze commit is an ancestor of HEAD and the code under test is as it was there."""
    commit = doc["commit_sha"]
    if _git("merge-base", "--is-ancestor", commit, "HEAD").returncode != 0:
        raise BatteryError(f"freeze commit {commit} is not an ancestor of HEAD")
    if _git("merge-base", "--is-ancestor", doc["main_sha_contained"], commit).returncode != 0:
        raise BatteryError("the main SHA of the freeze is not contained in the freeze commit")
    diff = _git("diff", "--name-only", commit, "HEAD", "--", *GUARDED_PATHS)
    if diff.stdout.strip():
        raise BatteryError(
            f"the code under test changed since the freeze commit {commit}: {diff.stdout.split()[:6]}"
        )


def require_freeze(path: Path = FREEZE_PATH, verify_git: bool | None = None) -> dict:
    doc = read_freeze(path)
    if verify_git is None:
        verify_git = not os.environ.get(GIT_SHA_ENV)
    if verify_git:
        verify_freeze_in_git(doc)
    return doc


def guard_run_dir(path: Path) -> None:
    """B9's and B10's run files are never opened or written."""
    for spent in SPENT_RUNS:
        try:
            p = Path(path).resolve()
            inside = p == spent.resolve() or spent.resolve() in p.parents
        except OSError:
            inside = False
        if inside:
            raise BatteryError(f"{path} holds the run files of an earlier battery")


def dev_cases(const: b11.Constants) -> dict[str, dict]:
    """The development cases of the yaml, from the registered files that define them."""
    cases = dict(bt10.dev_cases(const.b10))  # T, S1 to S4, ND, N1 to N3, P1 to P3 (B10's dev set)
    cases.update(const.b10.battery_cases)  # V1 to V3, read in B10, development here
    names = const.development_case_names
    missing = [n for n in names if n not in cases]
    if missing:
        raise BatteryError(f"development cases {missing} are defined in no registered file")
    return {k: v for k, v in cases.items() if k in names}


def needs_freeze(const: b11.Constants, case: str, seed: int) -> bool:
    """A B11 case or seed needs the freeze file; a development case on a development seed does
    not; everything else is refused."""
    spent = const.raw["spent_never_used"]["seeds"]
    battery = case in const.battery_cases or seed in const.battery_seeds
    if battery:
        if case not in const.battery_cases or seed not in const.battery_seeds:
            raise BatteryError(
                f"case {case!r} with seed {seed}: a W case runs on seeds {list(const.battery_seeds)} "
                "only and seeds 91, 92 on a W case only"
            )
        return True
    if seed in OTHER_REGISTERED_SEEDS or seed in spent:
        raise BatteryError(f"seed {seed} belongs to another registered attempt: never run again")
    if case in dev_cases(const) and seed in const.development_seeds:
        return False
    raise BatteryError(f"case {case!r} with seed {seed} is neither a B11 nor a development run")


# -- conditions and commands -----------------------------------------------------------------------------


def switches_for(condition: str, candidate: str | None, tuned: dict) -> tuple[dict, dict]:
    """(lap node parameters, planner parameters) of a condition. `k` and `kf` are the frozen
    candidate's; `t` is the planner's terminal curvature bound."""
    if condition in ("k", "kf"):
        if candidate not in b11.CANDIDATES:
            raise BatteryError(f"condition {condition} needs the frozen candidate")
        condition = candidate  # type: ignore[assignment]
    if condition == "t":
        return {}, {
            "terminal_curvature_bound": True,
            "curvature_bound_a_lat_mps2": float(tuned["curvature_bound_a_lat_mps2"]),
        }
    lap, planner = SWITCHES.get(condition, ({}, {}))
    lap = dict(lap)
    if lap.get("station_uncertainty") or lap.get("reset_on_change"):
        lap.update({k: float(tuned[k]) for k in TUNED_LAP_KEYS})
    return lap, dict(planner)


def condition_flags(
    const: b11.Constants,
    condition: str,
    ceiling: float,
    ready_after_s: float,
    candidate: str | None = None,
    tuned: dict | None = None,
) -> list[str]:
    tuned = dict(TUNED if tuned is None else tuned)
    lap1 = str(const.lap1_cap_mps)
    source = ["--lap-source", "truth", "--lap-truth-ready-after-s", repr(float(ready_after_s))]
    # a, b and t: lap memory off, the node observes only, flags exactly as B10's a and b (no
    # truth source flags: B10 ran its D10-S1 control so)
    if condition == "a":
        return ["--lap-memory", "off", "--v-max-mps", lap1]
    if condition in ("b", "t"):
        flags = ["--lap-memory", "off", "--v-max-mps", str(ceiling)]
    elif condition == "c":
        flags = ["--lap-memory", "v0", "--v-max-mps", str(ceiling), "--v-lap1-mps", lap1, *source]
    elif condition in ("m", "f", "u", "z", "k", "kf", *b11.CANDIDATES):
        flags = ["--lap-memory", "v1", "--v-max-mps", str(ceiling), "--v-lap1-mps", lap1, *source]
    else:
        raise BatteryError(f"unknown condition {condition!r}")
    lap, planner = switches_for(condition, candidate, tuned)
    if lap:
        flags += ["--lap-params-json", json.dumps(lap, sort_keys=True)]
    if planner:
        flags += ["--planner-params-json", json.dumps(planner, sort_keys=True)]
    if condition == "kf":
        flags += ["--lap-truth-faults-json", json.dumps(const.fault_plan, sort_keys=True)]
    return flags


def command_for(
    const: b11.Constants,
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
    candidate: str | None = None,
    tuned: dict | None = None,
) -> list[str]:
    """The runner's argv for one run. A B11 run takes the candidate and the tuned values from the
    freeze file (and refuses without it)."""
    if needs_freeze(const, case, seed):
        freeze = require_freeze(freeze_path, verify_git=verify_git)
        candidate, tuned = freeze["candidate"], freeze["tuned_values"]
        ready_after_s = const.oracle_ready_after_s[seed]
        if condition not in b11.B11_CONDITIONS:
            raise BatteryError(f"condition {condition!r} is not a B11 condition")
    else:
        ready_after_s = DEV_READY_AFTER_S.get(seed, 0.0)
        if condition not in b11.DEV_CONDITIONS and condition not in ("k", "kf"):
            raise BatteryError(f"condition {condition!r} is not a development condition")
    if direction not in const.directions:
        raise BatteryError(f"direction {direction!r} is not in {list(const.directions)}")
    cases = {**dev_cases(const), **const.battery_cases}
    io = b4t.manifest(bundle)
    nominal = io["nominal"]
    track, params = bt10.case_track(cases[case], data_root)
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
        *condition_flags(
            const, condition, float(io["validity"]["v_max_mps"]), ready_after_s, candidate, tuned
        ),
        "--car",
        "params",
        "--seed",
        str(seed),
        "--stop-after-laps",
        str(const.laps_driven if laps is None else laps),
        "--sim-time",
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


# -- attempts ---------------------------------------------------------------------------------------------


def needs_rerun(result: dict) -> bool:
    if result.get("stopped_because") == "start_refused":
        return False
    return bool(b5t.needs_rerun(result) or bt10.recording_too_short(result))


def make_jobs(conditions, cases, seeds, directions) -> list[tuple]:
    return b4t.make_jobs(list(conditions), list(cases), list(seeds), tuple(directions))


def b11_jobs(const: b11.Constants, dropped: tuple[str, ...] = ()) -> list[tuple]:
    """The 88 runs (a: seed 91 only, then b, c, k, t, kf on both seeds) of the cases not dropped."""
    cases = [c for c in const.battery_cases if c not in dropped]
    jobs: list[tuple] = []
    for cond in b11.B11_CONDITIONS:
        spec = const.conditions[cond]
        seeds = [int(s) for s in spec.get("seeds", const.battery_seeds)]
        jobs += make_jobs([cond], cases, seeds, const.directions)
    return jobs


def run_attempt(const, args, freeze, sha, job, attempt, domain, out, mode) -> dict:
    name, condition, case, direction, seed = job
    work = out / "work" / f"{name}.attempt{attempt}"
    work.mkdir(parents=True, exist_ok=True)
    result_path = work / "result.json"
    command = command_for(
        const, condition, case, direction, seed, domain, work, result_path,
        Path(args.bundle), Path(args.data_root),
        laps=getattr(args, "laps", None) if mode == "development" else None,
        freeze_path=Path(args.freeze_path),
        candidate=getattr(args, "candidate", None),
    )  # fmt: skip
    started = time.monotonic()
    with (work / "runner.log").open("w") as log:
        proc = subprocess.run(
            command, stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ), check=False
        )
    result = (
        json.loads(result_path.read_text())
        if result_path.exists()
        else {"error": f"no result file, runner exit {proc.returncode}"}
    )
    result[META] = {
        "id": "B11" if mode == "battery" else "M-D1b",
        "mode": mode,
        "name": name,
        "condition": condition,
        "case": case,
        "direction": direction,
        "seed": seed,
        "attempt": attempt,
        "prereg_yaml_sha256": const.yaml_sha256,
        "addendum_sha256": const.addendum_sha256,
        "freeze_commit": None if freeze is None else freeze["commit_sha"],
        "git_sha": sha,
        "command": command[1:],
        "runner_exit": proc.returncode,
        "wall_s": round(time.monotonic() - started, 1),
        "starved": b5t.starved(result),
        "invalid_truth": b4t.invalid_truth(result),
        "recording_too_short": bt10.recording_too_short(result),
    }
    b4t.write_once(b4t.attempt_path(out, name, attempt), json.dumps(result, indent=1))
    return result


def pending_work(out: Path, jobs: list[tuple]) -> list[tuple[tuple, int]]:
    """(job, attempt) still to run. A finished job's accepted attempt is written once, with the
    list of its attempts."""
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
                    "real_time_factor": doc.get("real_time_factor"),
                    "error": doc.get("error", ""),
                }
            )
        last[META]["attempts"] = attempts
        last[META]["accepted_attempt"] = k
        last[META]["accepted_while_invalid"] = bool(needs_rerun(last))
        if needs_rerun(last) and not last.get("error"):
            # still invalid after the last attempt: counts as failed to execute (yaml validity)
            last["error"] = f"invalid after {k} attempts (failed to execute)"
        b4t.write_once(final, json.dumps(last, indent=1))
    return todo


def run_jobs(const, args, freeze, sha, jobs, out: Path, workers: int, mode: str) -> None:
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

        def worker(pending=pending, domains=domains, failures=failures) -> None:
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
                    except BaseException as exc:  # noqa: BLE001
                        failures.append(exc)
                        return
                    with lock:
                        print(
                            f"{job[0]} attempt {attempt}: "
                            f"{len(result.get('gt_lap_times_registered_s', []))} laps, contacts "
                            f"{result.get('wall_contacts')}, rtf {result.get('real_time_factor')}, "
                            f"rerun {needs_rerun(result)}, error {result.get('error', '')[:60]!r}",
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


def drop_decision(out: Path, const: b11.Constants) -> b11.DropResult:
    rule = const.drop_rule
    cond, seed = str(rule["condition"]), int(rule["seed"])
    laps: dict[str, int] = {}
    for case in const.battery_cases:
        for direction in const.directions:
            path = out / f"{cond}_{case}_{direction}_{seed}.json"
            if not path.exists():
                raise BatteryError(f"{path.name} has not run: the drop rule cannot be applied")
            doc = json.loads(path.read_text())
            n = len(doc.get("gt_lap_times_registered_s", [])) if not doc.get("error") else 0
            if doc.get("truth_valid") is False:
                n = 0
            laps[f"{case}_{direction}"] = n
    return b11.drop_rule(laps, const)


# -- commands --------------------------------------------------------------------------------------------


def _sha() -> str:
    sha = os.environ.get(GIT_SHA_ENV) or _git("rev-parse", "HEAD").stdout.strip()
    return sha or "unknown"


def check_freeze_cmd(args: argparse.Namespace) -> None:
    const = b11.load_constants()
    doc = require_freeze(Path(args.freeze_path), verify_git=True)
    print(
        f"freeze OK: commit {doc['commit_sha']}, main {doc['main_sha_contained']}, candidate "
        f"{doc['candidate']}, yaml {const.yaml_sha256}, addendum {const.addendum_sha256}, HEAD {_sha()}"
    )


def dev_run_cmd(args: argparse.Namespace) -> None:
    guard_run_dir(Path(args.out))
    const = b11.load_constants()
    if not 1 <= args.workers <= MAX_WORKERS:
        raise BatteryError(f"--workers must be 1 to {MAX_WORKERS}")
    cases = args.cases.split(",")
    seeds = [int(s) for s in args.seeds.split(",")]
    conditions = args.conditions.split(",")
    for case in cases:
        for seed in seeds:
            if needs_freeze(const, case, seed):
                raise BatteryError(f"{case} seed {seed} is a B11 run: not a development run")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    directions = tuple(args.directions.split(",")) if args.directions else const.directions
    jobs = make_jobs(conditions, cases, seeds, directions)
    for job in jobs:
        command_for(
            const, job[1], job[2], job[3], job[4], args.domain_base, out, out,
            Path(args.bundle), Path(args.data_root), freeze_path=Path(args.freeze_path),
            candidate=args.candidate,
        )  # fmt: skip
    sha = _sha()
    print(f"M-D1b DEVELOPMENT runs (never evidence): git {sha} -> {out}", flush=True)
    run_jobs(const, args, None, sha, jobs, out, args.workers, "development")


def run_cmd(args: argparse.Namespace) -> None:
    guard_run_dir(Path(args.out))
    const = b11.load_constants()
    if not 1 <= args.workers <= MAX_WORKERS:
        raise BatteryError(f"--workers must be 1 to {MAX_WORKERS}")
    freeze = require_freeze(Path(args.freeze_path))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    b4t.check_clean()
    sha = b4t.git_sha()
    for job in b11_jobs(const):  # every command is validated before anything starts
        command_for(
            const, job[1], job[2], job[3], job[4], args.domain_base, out, out,
            Path(args.bundle), Path(args.data_root), freeze_path=Path(args.freeze_path),
        )  # fmt: skip
    print(
        f"B11: git {sha}, freeze {freeze['commit_sha']}, candidate {freeze['candidate']}",
        flush=True,
    )
    first = [j for j in b11_jobs(const) if j[1] == "a"]
    run_jobs(const, args, freeze, sha, first, out, args.workers, "battery")
    drops_path = out / DROPS_FILE
    if not drops_path.exists():
        d = drop_decision(out, const)
        b4t.write_once(
            drops_path,
            json.dumps(
                {
                    "dropped": list(d.dropped),
                    "decided": d.decided,
                    "reason": d.reason,
                    "laps": d.laps,
                },
                indent=1,
            ),
        )
    drops = json.loads(drops_path.read_text())
    print(f"drop rule: {drops}", flush=True)
    if not drops["decided"]:
        raise BatteryError(f"nothing is decided: {drops['reason']}. Stop and tell the owner.")
    rest = [j for j in b11_jobs(const, tuple(drops["dropped"])) if j[1] != "a"]
    run_jobs(const, args, freeze, sha, rest, out, args.workers, "battery")


# -- reports ---------------------------------------------------------------------------------------------


def _f(x, fmt: str = ".3f") -> str:
    if x is None:
        return "-"
    if isinstance(x, float):
        return "nan" if math.isnan(x) else format(x, fmt)
    return str(x)


def decision_lines(d) -> list[str]:
    verdict = {True: "PASS", False: "FAIL", None: "INSUFFICIENT"}[d.passed]
    out = [f"### {d.name}: {verdict}" + (f" ({d.note})" if d.note else ""), ""]
    out += [f"- {'ok' if ok else 'FAIL'}: {k}: {v}" for k, (ok, v) in d.items.items()]
    return [*out, ""]


def dev_table(runs) -> list[str]:
    """Per development condition: runs, contacts, runs not completing 12 laps, longest stop, late
    brakes per lap, median T_late against the paired b and c runs."""
    by = {cond: {r.key: r for r in runs if r.condition == cond} for cond in b11.DEV_CONDITIONS}
    out = [
        (
            "| condition | runs | contacts | not 12 laps | stalls > 5 s | late brakes/lap | "
            "median T_late / b | median T_late / c | median T_late / m | median sum 2-10 / c |"
        ),
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for cond in b11.DEV_CONDITIONS:
        rs = list(by[cond].values())
        if not rs:
            continue

        def med(ref: str, rs=rs, fn=b11.t_late) -> float:
            qs = [b11._ratio(fn(r), fn(by[ref][r.key])) for r in rs if r.key in by[ref]]
            qs = [q for q in qs if math.isfinite(q)]
            return (
                sorted(qs)[len(qs) // 2]
                if len(qs) % 2
                else (
                    (sorted(qs)[len(qs) // 2 - 1] + sorted(qs)[len(qs) // 2]) / 2
                    if qs
                    else math.nan
                )
            )

        late = [r.late_brakes_per_lap for r in rs if not math.isnan(r.late_brakes_per_lap)]
        stalls = sum(1 for r in rs if not ((r.extras.get("longest_stop_s") or 99) <= 5.0))
        out.append(
            f"| {cond} | {len(rs)} | {sum(r.contacts for r in rs)} | "
            f"{sum(1 for r in rs if r.error or r.laps < 12)} | {stalls} | "
            f"{_f(sum(late) / len(late) if late else math.nan)} | {_f(med('b'))} | {_f(med('c'))} | "
            f"{_f(med('m'))} | {_f(med('c', fn=b11.sum_laps))} |"
        )
    return out


def dev_report_text(out: Path) -> str:
    const = b11.load_constants()
    runs = b11.load_runs(out, const, b11.DEV_CONDITIONS)
    _pick, scores, sentence = b11.candidate_rule(runs, const)
    lines = [
        "# M-D1b development ablations (development only, never evidence that D works)",
        "",
        f"yaml {const.yaml_sha256}; {len(runs)} run files in {out}",
        "",
        *dev_table(runs),
        "",
        "## Candidate rule",
        "",
        "| candidate | eligible | score (median T_late(K) / T_late(c)) | pairs | why not |",
        "|---|---|---|---|---|",
        *[
            f"| {s.name} | {s.eligible} | {_f(s.score)} | {s.n_pairs} | {'; '.join(s.why_not)} |"
            for s in scores
        ],
        "",
        f"Result: {sentence}",
        "",
        "## Reading fixed by brief 44 (addendum 1 section 5.3)",
        "",
        b11.value_profile_reading(runs),
        "",
    ]
    return "\n".join(lines)


def report_text(out: Path) -> str:
    const = b11.load_constants()
    runs = b11.load_runs(out, const, b11.B11_CONDITIONS)
    drops_path = out / DROPS_FILE
    drops = json.loads(drops_path.read_text()) if drops_path.exists() else None
    dropped = set(drops["dropped"]) if drops else set()
    runs = [r for r in runs if r.case not in dropped]
    decisions = []
    for fn in (b11.d11_s0, b11.d11_s1, b11.d11_s2, b11.d11_s3):
        try:
            decisions.append(fn(runs, const))
        except (b4.MissingPairError, b11.PreregError) as exc:
            decisions.append(
                b11.Decision(fn.__name__.upper().replace("_", "-"), None, {}, str(exc))
            )
    s0, s1, s2, s3 = decisions
    within, within_text = b11.t_within_2pct(runs)
    s4 = (
        b11.d11_s4(runs, const, s0, s1, s2, s3)
        if all(d.passed is not None for d in decisions)
        else None
    )
    lines = [
        "# B11: D v1 against D v0 in sim (truth lap source)",
        "",
        f"yaml {const.yaml_sha256}; addendum {const.addendum_sha256}; {len(runs)} run files; drop rule {drops}",
        "",
        f"**Outcome (registered):** {b11.outcome(s0, s1, s2, s3, within) if all(d.passed is not None for d in decisions) else 'not decided'}",
        "",
        f"**Outcome D11-S4 (addendum 1):** {b11.outcome_s4(s4, s0, s1) if s4 else 'not decided'}",
        "",
        f"T_late(t) against T_late(k): {within_text}",
        "",
    ]
    for d in [*decisions, *([s4] if s4 else [])]:
        lines += decision_lines(d)
    lines += [
        "## Reported, no threshold",
        "",
        "| run | T_late k | c | b | t | k/c | k/b | t/k | sum 2-10 k | sum 2-10 c | reach k | reach c | resets |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in b11.reported_rows(runs):
        lines.append(
            f"| {r['run']} | {_f(r['T_late_k'], '.2f')} | {_f(r['T_late_c'], '.2f')} | "
            f"{_f(r['T_late_b'], '.2f')} | {_f(r['T_late_t'], '.2f')} | {_f(r['k_over_c'])} | "
            f"{_f(r['k_over_b'])} | {_f(r['t_over_k'])} | {_f(r['sum2_10_k'], '.1f')} | "
            f"{_f(r['sum2_10_c'], '.1f')} | {_f(r['reach_k'])} | {_f(r['reach_c'])} | {r['resets']} |"
        )
    kf_rows = b11.reported_rows(runs, "kf")
    lines += ["", "kf runs (faults): resets per run", ""]
    lines += [f"- {r['run']}: {r['resets']}" for r in kf_rows]
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp) -> None:
        sp.add_argument("--bundle", default=str(b4t.DEFAULT_BUNDLE))
        sp.add_argument("--data-root", default=str(b4t.DEFAULT_DATA_ROOT))
        sp.add_argument("--freeze-path", default=str(FREEZE_PATH))
        sp.add_argument("--domain-base", type=int, default=150)
        sp.add_argument("--workers", type=int, default=1)

    sp = sub.add_parser("check-freeze")
    sp.add_argument("--freeze-path", default=str(FREEZE_PATH))
    sp = sub.add_parser("dev-run")
    common(sp)
    sp.add_argument("--out", required=True)
    sp.add_argument("--conditions", required=True)
    sp.add_argument("--cases", required=True)
    sp.add_argument("--seeds", required=True)
    sp.add_argument("--directions", default="")
    sp.add_argument("--laps", type=int, default=None)
    sp.add_argument("--candidate", default=None, help="development k runs (the canary)")
    sp = sub.add_parser("dev-report")
    sp.add_argument("--out", required=True)
    sp = sub.add_parser("run")
    common(sp)
    sp.add_argument("--out", required=True)
    sp = sub.add_parser("report")
    sp.add_argument("--out", required=True)
    args = p.parse_args(argv)
    if args.cmd == "check-freeze":
        check_freeze_cmd(args)
    elif args.cmd == "dev-run":
        dev_run_cmd(args)
    elif args.cmd == "dev-report":
        print(dev_report_text(Path(args.out)))
    elif args.cmd == "run":
        run_cmd(args)
    elif args.cmd == "report":
        print(report_text(Path(args.out)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
