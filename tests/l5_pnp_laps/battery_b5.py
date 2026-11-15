#!/usr/bin/env python3
"""Run battery B5 (obstacle variants in sim) and the oracle measurements M-A2, M-A2x and M-A2w of
the v0.4 pre-registration, and report them (brief 40 and 40a).

    python3 tests/l5_pnp_laps/battery_b5.py track-check
    python3 tests/l5_pnp_laps/battery_b5.py freeze-check --b4-sha SHA
    python3 tests/l5_pnp_laps/battery_b5.py ma2 --out DIR [--stage ma2|ma2x|ma2w] ...
    python3 tests/l5_pnp_laps/battery_b5.py b5 --out DIR --development
    python3 tests/l5_pnp_laps/battery_b5.py b5 --out DIR --b4-dir B4DIR --conditions abc[v]
    python3 tests/l5_pnp_laps/battery_b5.py report --kind b5|ma2|ma2x|ma2w --out DIR

Needs the ROS workspace sourced (see .github/scripts/l5_pnp_oracle_test.sh) and f1tenth_gym.

Cases, seeds, laps, caps and conditions are read from `evaluation/prereg/pnp-v0.4.yaml` (blocks
`lap_battery_b5` and `oracle_track`), whose SHA-256 is checked against the pre-registration and
written into every run JSON and the report; the M-A2 cases are the development cases of
`pnp-v0.3.yaml`. Nothing is copied here.

B5 (the battery, seeds 31 and 32) is REFUSED unless, in this order: the B4 directory it is pointed
at is a complete B4 battery whose D3-S0 and D3-S1 both PASS (computed here, not taken on trust), and
the checkout changes nothing under racer_laps, racer_planner or the lap scoring since the commit B4
ran on (`freeze-check`). Development runs (case OD, seeds 11 to 13) and M-A2 need neither. Nothing
here ever runs an O1 to O4 case with a seed outside the yaml's list for its mode.

Files and the starved-run rule are those of battery_b4.py: every attempt is written once to
`<out>/attempts/`, a run whose /drive_raw stream has a gap over 40 ms or whose real-time factor is
under 0.8 was starved by the host and is rerun alone (at most three attempts, all reported).
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import queue
import re
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO_ROOT / "sim" / "bridge" / "racer_gym_bridge"))
import battery_b4 as b4t
import lap_battery_b4 as b4
import lap_battery_b5 as b5

RUNNER = REPO_ROOT / "tests" / "l5_reactive_bench" / "track_family_runner.py"
DEFAULT_BUNDLE = REPO_ROOT / "config" / "pnp" / "fixtures" / "dyn-car-2026-11-12"
DEFAULT_DATA_ROOT = Path("/Users/cameronjim/code")
MIN_REAL_TIME_FACTOR = 0.8  # brief 40: a run below this is discarded unread and repeated
MAX_WORKERS = b4t.MAX_WORKERS
MAX_ATTEMPTS = b4t.MAX_ATTEMPTS
BatteryError = b4t.BatteryError
META = "battery_b5"
MODES = ("b5", "b5-development", "ma2", "ma2x", "ma2w")
#: What must not change between the commit B4 ran on and the commit B5 runs on (brief 40).
FROZEN_TREES = ("ros_ws/src/racer_laps", "ros_ws/src/racer_planner")
#: The lap scoring of tests/l5_reactive_bench/track_family_runner.py and bench_metrics.py.
LAP_SCORING_FUNCTIONS = (
    "rescore_from_start_scan",
    "first_scan_at_or_after",
    "crossing_lap_times",
    "lap_results",
)
LAP_SCORING_FILES = ("tests/l5_reactive_bench/bench_metrics.py",)
MPPI_HEADER = (
    REPO_ROOT / "ros_ws" / "src" / "racer_planner" / "include" / "racer_planner" / "mppi.hpp"
)
ORACLE_MODEL_LABELS = {"fused-v1": "fused-v1", "net": "trackperc-v2-2026-11-14 refined"}


# -- the cases ---------------------------------------------------------------------------------------


def case_track(case_spec: dict, data_root: Path) -> tuple[str, dict]:
    """(track, track params) of a yaml case. battery_b4.case_track returns only the map files for a
    map case; the obstacles of the case (O4) must go with them."""
    track, params = b4t.case_track(case_spec, data_root)
    if track == "map" and "obstacles" in case_spec:
        params = {**params, "obstacles": list(case_spec["obstacles"])}
    return track, params


def check_obstacle_tracks(const: b5.Constants, data_root: Path) -> list[str]:
    """Build every B5 case (battery and development), both directions, offline: the start pose
    rule, a lane that is not blocked, and for a map that the obstacles can be drawn into it.
    Returns one line per case. A failure is reported, never worked around."""
    from racer_gym_bridge import track_loader, track_obstacles

    lines = []
    for label, cases in (
        ("development", const.development_cases),
        ("battery", const.battery_cases),
    ):
        for name, spec in cases.items():
            try:
                track, params = case_track(spec, data_root)
                if track == "map":
                    import tempfile

                    sys.path.insert(0, str(REPO_ROOT / "tools" / "track_session"))
                    from pipeline import raceline_from_map

                    with tempfile.TemporaryDirectory() as tmp:
                        csv = Path(tmp) / "centre.csv"
                        raceline_from_map(
                            Path(params["map_yaml"]),
                            Path(tmp) / "unused.csv",
                            centre_only=True,
                            centre_csv=csv,
                        )
                        params = {**params, "map_centreline_csv": str(csv)}
                        built = track_loader.build_synthetic_track(track, **params)
                else:
                    built = track_loader.build_synthetic_track(track, **params)
                built.reversed()
                lines.append(
                    f"{label} {name}: ok, {len(built.obstacles)} obstacle(s) drawn, loop "
                    f"{track_obstacles.arc_positions(built.centreline_x, built.centreline_y)[1]:.2f} m"
                )
            except (
                track_obstacles.ObstacleError,
                BatteryError,
                ValueError,
                OSError,
                KeyError,
                ImportError,
            ) as exc:
                lines.append(f"{label} {name}: FAILED: {type(exc).__name__}: {exc}")
    return lines


def default_boundary_weight() -> float:
    """The planner's default boundary weight, read from the source (M-A2w scales this value)."""
    m = re.findall(r"double\s+w_bound\s*=\s*([0-9.eE+-]+)\s*;", MPPI_HEADER.read_text())
    if len(m) != 1:
        raise BatteryError(f"expected one w_bound default in {MPPI_HEADER}, found {len(m)}")
    return float(m[0])


# -- the freeze --------------------------------------------------------------------------------------


def _git_host(*args: str) -> str:
    out = subprocess.run(
        ["git", "-c", "safe.directory=*", "-C", str(REPO_ROOT), *args],
        capture_output=True,
        text=True,
        check=False,
    )
    if out.returncode != 0:
        raise BatteryError(f"git {' '.join(args)} failed: {out.stderr.strip()[:200]}")
    return out.stdout


def function_sources(text: str, names: tuple[str, ...]) -> dict[str, str]:
    """Source text of the named top level functions of a module."""
    tree = ast.parse(text)
    out = {}
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in names:
            out[node.name] = ast.get_source_segment(text, node) or ""
    missing = [n for n in names if n not in out]
    if missing:
        raise BatteryError(f"functions {missing} not found")
    return out


def freeze_problems(
    b4_sha: str,
    head: str = "HEAD",
    show=None,
    diff=None,
) -> list[str]:
    """What changed since the commit B4 ran on in the code B5 may not change: any file of
    FROZEN_TREES, bench_metrics.py, and the source of the lap scoring functions of the runner.
    Empty means frozen. `show` and `diff` are injectable for tests."""
    show = show or (lambda rev, path: _git_host("show", f"{rev}:{path}"))
    diff = diff or (lambda a, b, paths: _git_host("diff", "--name-only", a, b, "--", *paths))
    problems = []
    changed = [
        ln
        for ln in diff(b4_sha, head, list(FROZEN_TREES) + list(LAP_SCORING_FILES)).splitlines()
        if ln
    ]
    problems += [f"changed since B4: {path}" for path in changed]
    runner = "tests/l5_reactive_bench/track_family_runner.py"
    before = function_sources(show(b4_sha, runner), LAP_SCORING_FUNCTIONS)
    after = function_sources(show(head, runner), LAP_SCORING_FUNCTIONS)
    problems += [
        f"lap scoring function changed since B4: {n}" for n in before if before[n] != after[n]
    ]
    return problems


def freeze_check_cmd(args: argparse.Namespace) -> None:
    problems = freeze_problems(args.b4_sha)
    head = _git_host("rev-parse", "HEAD").strip()
    if problems:
        print(f"NOT FROZEN (HEAD {head} against B4 commit {args.b4_sha}):")
        for p in problems:
            print(f"  {p}")
        raise BatteryError("the checkout changed code B5 may not change")
    print(
        f"frozen: HEAD {head} changes nothing under racer_laps, racer_planner or the lap scoring since {args.b4_sha}"
    )


def b4_gate(b4_dir: Path) -> tuple[bool, str, str]:
    """(both D3-S0 and D3-S1 passed, the B4 commit, a text). The B4 directory must be a complete
    battery (every run of every condition); the decisions are computed from the run files."""
    const3 = b4.load_constants()
    runs = b4.load_b4_runs(b4_dir, "battery")
    metas = [json.loads(p.read_text())["battery_b4"] for p in sorted(b4_dir.glob("*_*_*_*.json"))]
    shas = sorted({m["git_sha"] for m in metas})
    if len(shas) != 1:
        raise BatteryError(f"the B4 run files carry {len(shas)} git SHAs ({shas}), expected one")
    cases = b4t.battery_cases(const3, b4_dir)
    want = {
        (c, case, d, s)
        for c in b4.CONDITIONS
        for case in cases
        for d in const3.directions
        for s in const3.battery_seeds
    }
    have = {(r.condition, r.case, r.direction, r.seed) for r in runs}
    if want != have:
        raise BatteryError(
            f"{b4_dir} is not a complete B4 battery: {len(want - have)} runs missing"
        )
    s0 = b4.d3_s0(runs, const3)
    s1 = b4.d3_s1(runs, const3)
    text = f"{s0}\n{s1}"
    return bool(s0.passed) and bool(s1.passed), shas[0], text


# -- commands ----------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Job:
    name: str
    condition: str
    case: str
    direction: str
    seed: int

    @staticmethod
    def make(condition: str, case: str, direction: str, seed: int) -> Job:
        return Job(f"{condition}_{case}_{direction}_{seed}", condition, case, direction, seed)


def b5_condition_flags(
    const: b5.Constants, condition: str, ceiling: float, variant: str
) -> list[str]:
    """Runner flags of a B5 condition: a, b, c from the yaml, v the D v1 variant of v0.3 section
    1.4 the owner's answer allows (`variant` d or e: the floor and merge of that condition)."""
    if condition != "v":
        return b4t.condition_flags(const, condition, ceiling)
    if variant not in ("d", "e"):
        raise BatteryError("condition v needs --variant d or e (the owner's answer on v0.3 1.4)")
    spec = b4.load_constants().conditions[variant]
    shim = SimpleNamespace(conditions={"v": spec}, lap1_cap_mps=const.lap1_cap_mps)
    return b4t.condition_flags(shim, "v", ceiling)


def common_flags(bundle: Path) -> list[str]:
    nominal = b4t.manifest(bundle)["nominal"]
    return [
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
        "--car",
        "params",
    ]


def runner_command(
    track: str,
    params: dict,
    backend_flags: list[str],
    speed_flags: list[str],
    job: Job,
    laps: int,
    domain: int,
    work: Path,
    out: Path,
    bundle: Path,
    extra: list[str],
) -> list[str]:
    command = [
        sys.executable,
        str(RUNNER),
        "--track",
        track,
        "--track-params-json",
        json.dumps(params),
        "--driver",
        "pnp",
        *backend_flags,
        *common_flags(bundle),
        *speed_flags,
        "--seed",
        str(job.seed),
        "--stop-after-laps",
        str(laps),
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
        *extra,
    ]
    if job.direction == "cw":
        command.append("--reverse")
    return command


def b5_command(
    const: b5.Constants,
    job: Job,
    mode: str,
    domain: int,
    work: Path,
    out: Path,
    bundle: Path,
    data_root: Path,
    variant: str,
) -> list[str]:
    """The runner's argv for one B5 run (battery or development). A case or seed that is not the
    yaml's for the mode refuses."""
    development = mode == "b5-development"
    cases = const.development_cases if development else const.battery_cases
    seeds = const.development_seeds if development else const.battery_seeds
    if job.case not in cases:
        raise BatteryError(f"case {job.case!r} is not a {mode} case of the yaml ({sorted(cases)})")
    if job.seed not in seeds:
        raise BatteryError(f"seed {job.seed} is not in the yaml's {mode} seeds {list(seeds)}")
    if job.direction not in const.directions:
        raise BatteryError(f"direction {job.direction!r} is not in {list(const.directions)}")
    track, params = case_track(cases[job.case], data_root)
    ceiling = b4t.ceiling_mps(bundle)
    return runner_command(
        track,
        params,
        ["--backend", "geometric"],
        b5_condition_flags(const, job.condition, ceiling, variant),
        job,
        const.laps,
        domain,
        work,
        out,
        bundle,
        ["--count-innovation"],
    )


def ma2_command(
    const: b5.Constants,
    job: Job,
    mode: str,
    domain: int,
    work: Path,
    out: Path,
    bundle: Path,
    data_root: Path,
    perception_bundle: Path | None,
    weight_factor: float | None,
) -> list[str]:
    """The runner's argv for one run of M-A2 (g, o), M-A2x (f, and g, o on OD) or M-A2w (g, o with
    the boundary weight scaled). The cases are the development cases of pnp-v0.3.yaml (the M-A2
    cases) and, for M-A2x, OD of the v0.4 yaml."""
    const3 = b4.load_constants()
    ox = const.oracle
    if job.seed != int(ox["seed"]):
        raise BatteryError(f"seed {job.seed} is not M-A2's seed {ox['seed']}")
    if job.direction not in ox["directions"]:
        raise BatteryError(f"direction {job.direction!r} is not in {ox['directions']}")
    if job.case in ox["cases"]:
        spec = const3.development_cases[job.case]
    elif job.case == "OD" and mode == "ma2x":
        spec = const.development_cases["OD"]
    else:
        raise BatteryError(
            f"case {job.case!r} is not an M-A2 case ({ox['cases']}) for stage {mode}"
        )
    track, params = case_track(spec, data_root)
    ceiling = b4t.ceiling_mps(bundle)
    if str(ox["cap"]) != "ceiling":
        raise BatteryError("M-A2's cap is registered as the ceiling")
    backend = {"g": ["--backend", "geometric"], "o": ["--backend", "oracle"]}.get(job.condition)
    if job.condition == "f":
        if perception_bundle is None:
            raise BatteryError("condition f needs --perception-bundle")
        backend = ["--backend", "net", "--perception-bundle", str(perception_bundle)]
    if backend is None:
        raise BatteryError(f"condition {job.condition!r} is not g, o or f")
    extra: list[str] = []
    if weight_factor is not None:
        extra = [
            "--planner-params-json",
            json.dumps({"w_bound": default_boundary_weight() * weight_factor}),
        ]
    return runner_command(
        track,
        params,
        backend,
        ["--lap-memory", "off", "--v-max-mps", str(ceiling)],
        job,
        int(ox["laps"]),
        domain,
        work,
        out,
        bundle,
        extra,
    )


# -- attempts ---------------------------------------------------------------------------------------


def real_time_factor_low(result: dict) -> bool:
    rtf = result.get("real_time_factor")
    return rtf is not None and float(rtf) < MIN_REAL_TIME_FACTOR


def starved(result: dict) -> bool:
    """Starved by the host: the B4 rule (a /drive_raw gap over 40 ms) or a real-time factor under 0.8."""
    return b4t.starved(result) or real_time_factor_low(result)


def needs_rerun(result: dict) -> bool:
    unusable = "error" in result and "gt_lap_times_by_crossing_s" not in result
    return unusable or starved(result)


def attempt_summary(path: Path) -> dict:
    d = json.loads(path.read_text())
    return {
        "attempt": int(re.search(r"attempt(\d+)", path.name).group(1)),
        "file": path.name,
        "starved": bool(d[META]["starved"]),
        "max_gap_s": (d.get("drive_raw") or {}).get("max_gap_s"),
        "real_time_factor": d.get("real_time_factor"),
        "error": d.get("error", ""),
    }


def pending_work(out: Path, jobs: list[Job]) -> list[tuple[Job, int]]:
    """(job, attempt number) still to run; accepted results are written once for finished jobs."""
    todo = []
    for job in jobs:
        final = out / f"{job.name}.json"
        if final.exists():
            continue
        done = b4t.existing_attempts(out, job.name)
        if not done:
            todo.append((job, 1))
            continue
        last = json.loads(done[-1].read_text())
        k = len(done)
        if needs_rerun(last) and k < MAX_ATTEMPTS:
            todo.append((job, k + 1))
        else:
            last[META]["attempts"] = [attempt_summary(p) for p in done]
            last[META]["accepted_attempt"] = k
            last[META]["accepted_while_starved"] = bool(last[META]["starved"])
            b4t.write_once(final, json.dumps(last, indent=1))
    return todo


def run_jobs(
    jobs: list[Job], make_command, meta_base: dict, out: Path, workers: int, domain_base: int
) -> None:
    """Attempt 1 of every job `workers` at a time, then every rerun alone. `make_command(job,
    domain, work, result_path)` is the runner argv; every finished attempt is written at once."""
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
            domains.put(domain_base + k)
        pending: queue.Queue[tuple[Job, int]] = queue.Queue()
        for item in batch:
            pending.put(item)

        def worker(pending=pending, domains=domains) -> None:
            while True:
                try:
                    job, attempt = pending.get_nowait()
                except queue.Empty:
                    return
                domain = domains.get()
                try:
                    work = out / "work" / f"{job.name}.attempt{attempt}"
                    work.mkdir(parents=True, exist_ok=True)
                    result_path = work / "result.json"
                    command = make_command(job, domain, work, result_path)
                    started = time.monotonic()
                    with (work / "runner.log").open("w") as log:
                        proc = subprocess.run(
                            command,
                            stdout=log,
                            stderr=subprocess.STDOUT,
                            env=dict(os.environ),
                            check=False,
                        )
                    result = (
                        json.loads(result_path.read_text())
                        if result_path.exists()
                        else {"error": f"no result file, runner exit {proc.returncode}"}
                    )
                    result[META] = {
                        **meta_base,
                        "name": job.name,
                        "condition": job.condition,
                        "case": job.case,
                        "direction": job.direction,
                        "seed": job.seed,
                        "attempt": attempt,
                        "command": command[1:],
                        "runner_exit": proc.returncode,
                        "wall_s": round(time.monotonic() - started, 1),
                        "starved": starved(result),
                    }
                    b4t.write_once(
                        b4t.attempt_path(out, job.name, attempt), json.dumps(result, indent=1)
                    )
                    with lock:
                        print(
                            f"{job.name} attempt {attempt}: "
                            f"{len(result.get('gt_lap_times_by_crossing_s', []))} laps, "
                            f"rtf {result.get('real_time_factor')}, starved {starved(result)}, "
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


def check_mode_of_directory(out: Path, mode: str) -> None:
    """A directory holds the runs of one mode, never two."""
    for p in list(out.glob("*_*_*_*.json")) + list((out / "attempts").glob("*.json")):
        have = json.loads(p.read_text()).get(META, {}).get("mode")
        if have is not None and have != mode:
            raise BatteryError(f"{out} already holds {have} runs: use a separate directory")


def meta_base(mode: str, const: b5.Constants, sha: str, extra: dict | None = None) -> dict:
    return {
        "id": "B5" if mode.startswith("b5") else "M-A2",
        "mode": mode,
        "prereg_yaml_sha256": const.yaml_sha256,
        "git_sha": sha,
        **(extra or {}),
    }


def b5_run_cmd(args: argparse.Namespace) -> None:
    const = b5.load_constants()
    mode = "b5-development" if args.development else "b5"
    if args.workers < 1 or args.workers > MAX_WORKERS:
        raise BatteryError(f"--workers must be 1 to {MAX_WORKERS} (host limit)")
    conditions = list(args.conditions)
    bad = [c for c in conditions if c not in const.conditions]
    if bad:
        raise BatteryError(f"conditions {bad} are not in the yaml")
    if "v" in conditions and args.variant not in ("d", "e"):
        raise BatteryError("condition v needs --variant d or e; without the owner's answer omit v")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    check_mode_of_directory(out, mode)
    gate_text = ""
    if mode == "b5":
        if not args.b4_dir:
            raise BatteryError("B5 runs only after B4: pass --b4-dir (D3-S0 and D3-S1 must pass)")
        ok, b4_sha, gate_text = b4_gate(Path(args.b4_dir))
        print(gate_text, flush=True)
        if not ok:
            raise BatteryError(
                "D3-S0 or D3-S1 of B4 did not pass: B5 does not run, nothing started"
            )
        problems = freeze_problems(b4_sha)
        if problems:
            raise BatteryError(f"the code is not the B4 freeze: {problems[:6]}")
        cases = list(args.cases.split(",")) if args.cases else list(const.battery_cases)
        seeds = list(const.battery_seeds)
        if "O4" in cases and not (out / "o4_check.json").exists():
            raise BatteryError("O4 is requested but o4-check has not been run in this directory")
        if "O4" in cases and json.loads((out / "o4_check.json").read_text())["dropped"]:
            cases.remove("O4")
    else:
        cases = list(args.cases.split(",")) if args.cases else list(const.development_cases)
        seeds = list(const.development_seeds)
    b4t.check_clean()
    sha = b4t.git_sha()
    jobs = [
        Job.make(c, case, d, s)
        for c in conditions
        for case in cases
        for d in const.directions
        for s in seeds
    ]
    for job in jobs:  # validates every command before anything starts
        b5_command(
            const,
            job,
            mode,
            args.domain_base,
            out,
            out,
            Path(args.bundle),
            Path(args.data_root),
            args.variant,
        )
    print(f"B5 {mode}: git {sha}, yaml sha256 {const.yaml_sha256}, {len(jobs)} runs", flush=True)
    base = meta_base(mode, const, sha, {"variant": args.variant, "b4_gate": gate_text})
    run_jobs(
        jobs,
        lambda job, domain, work, result: b5_command(
            const,
            job,
            mode,
            domain,
            work,
            result,
            Path(args.bundle),
            Path(args.data_root),
            args.variant,
        ),
        base,
        out,
        args.workers,
        args.domain_base,
    )


def o4_check_cmd(args: argparse.Namespace) -> None:
    """Condition a, seed 31, both directions, 3 laps on O4, once; O4 is dropped when the map cannot
    be loaded, an obstacle cannot be drawn or either run completes fewer than 3 laps. Under the
    same guard as the battery (it uses seed 31)."""
    const = b5.load_constants()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    path = out / "o4_check.json"
    if path.exists():
        raise BatteryError(f"{path} exists: the O4 check may run once")
    ok, b4_sha, _gate_text = b4_gate(Path(args.b4_dir))
    if not ok:
        raise BatteryError("D3-S0 or D3-S1 of B4 did not pass: nothing with seed 31 runs")
    problems = freeze_problems(b4_sha)
    if problems:
        raise BatteryError(f"the code is not the B4 freeze: {problems[:6]}")
    b4t.check_clean()
    sha = b4t.git_sha()
    data_root = Path(args.data_root)
    notes: list[str] = []
    lines = [ln for ln in check_obstacle_tracks(const, data_root) if " O4:" in ln]
    laps: dict[str, int] = {}
    dropped = any("FAILED" in ln for ln in lines)
    notes += lines
    if not dropped:
        results: dict[str, dict] = {}

        def one(direction: str, domain: int) -> None:
            job = Job.make("a", "O4", direction, 31)
            work = out / "o4_check" / job.name
            work.mkdir(parents=True, exist_ok=True)
            rp = work / "result.json"
            o4_track, o4_params = case_track(const.battery_cases["O4"], data_root)
            command = runner_command(
                o4_track,
                o4_params,
                ["--backend", "geometric"],
                b5_condition_flags(const, "a", b4t.ceiling_mps(Path(args.bundle)), ""),
                job,
                3,
                domain,
                work,
                rp,
                Path(args.bundle),
                ["--count-innovation"],
            )
            with (work / "runner.log").open("w") as log:
                proc = subprocess.run(
                    command, stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ), check=False
                )
            results[direction] = (
                json.loads(rp.read_text())
                if rp.exists()
                else {"error": f"no result, exit {proc.returncode}"}
            )

        threads = [
            threading.Thread(target=one, args=(d, args.domain_base + i))
            for i, d in enumerate(const.directions)
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        laps = {
            d: len(r.get("gt_lap_times_by_crossing_s", [])) if "error" not in r else 0
            for d, r in results.items()
        }
        dropped = any(n < 3 for n in laps.values())
        notes += [f"{d}: {r['error'][:120]}" for d, r in results.items() if "error" in r]
    doc = {
        "dropped": dropped,
        "laps_by_direction": laps,
        "required_laps": 3,
        "git_sha": sha,
        "notes": notes,
    }
    b4t.write_once(path, json.dumps(doc, indent=1))
    print(json.dumps(doc, indent=1))


def ma2_run_cmd(args: argparse.Namespace) -> None:
    const = b5.load_constants()
    stage = args.stage
    if args.workers < 1 or args.workers > MAX_WORKERS:
        raise BatteryError(f"--workers must be 1 to {MAX_WORKERS} (host limit)")
    ext = b5.load_extensions_checked()
    ox = const.oracle
    directions = tuple(ox["directions"])
    seed = int(ox["seed"])
    perception = Path(args.perception_bundle) if args.perception_bundle else None
    factor = None
    if stage == "ma2":
        jobs = [
            Job.make(c, case, d, seed)
            for c in ("g", "o")
            for case in ox["cases"]
            for d in directions
        ]
    elif stage == "ma2x":
        jobs = [Job.make("f", case, d, seed) for case in ox["cases"] for d in directions]
        jobs += [Job.make(c, "OD", d, seed) for c in ("g", "o", "f") for d in directions]
        if perception is None:
            raise BatteryError("ma2x needs --perception-bundle (the frozen learned candidate)")
    elif stage == "ma2w":
        if args.weight_factor is None:
            raise BatteryError("ma2w needs --weight-factor")
        factors = [float(f) for f in ext["weight_check"]["factors"]]
        if args.weight_factor not in factors:
            raise BatteryError(f"--weight-factor must be one of {factors}")
        factor = args.weight_factor
        jobs = [
            Job.make(c, case, d, seed)
            for c in ("g", "o")
            for case in ox["cases"]
            for d in directions
        ]
    else:
        raise BatteryError(f"unknown stage {stage}")
    if args.only:
        wanted = args.only.split(",")
        unknown = [n for n in wanted if n not in {j.name for j in jobs}]
        if unknown:
            raise BatteryError(f"--only names {unknown} are not jobs of stage {stage}")
        jobs = [j for j in jobs if j.name in wanted]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    check_mode_of_directory(out, stage)
    b4t.check_clean()
    sha = b4t.git_sha()
    for job in jobs:
        ma2_command(
            const,
            job,
            stage,
            args.domain_base,
            out,
            out,
            Path(args.bundle),
            Path(args.data_root),
            perception,
            factor,
        )
    print(f"{stage}: git {sha}, yaml sha256 {const.yaml_sha256}, {len(jobs)} runs", flush=True)
    base = meta_base(
        stage,
        const,
        sha,
        {
            "weight_factor": factor,
            "w_bound": None if factor is None else default_boundary_weight() * factor,
            "perception_bundle": str(perception) if perception else None,
        },
    )
    run_jobs(
        jobs,
        lambda job, domain, work, result: ma2_command(
            const,
            job,
            stage,
            domain,
            work,
            result,
            Path(args.bundle),
            Path(args.data_root),
            perception,
            factor,
        ),
        base,
        out,
        args.workers,
        args.domain_base,
    )


# -- reports ----------------------------------------------------------------------------------------


def attempts_lines(out: Path) -> list[str]:
    metas = [json.loads(p.read_text())[META] for p in sorted(out.glob("*_*_*_*.json"))]
    lines = []
    for m in sorted(metas, key=lambda x: x["name"]):
        if m.get("accepted_attempt", 1) > 1 or m.get("accepted_while_starved"):
            lines.append(f"{m['name']}: {m['attempts']}")
    return lines or ["every run was accepted on its first attempt, none starved"]


def header_lines(title: str, out: Path, const: b5.Constants) -> list[str]:
    metas = [json.loads(p.read_text())[META] for p in sorted(out.glob("*_*_*_*.json"))]
    return [
        f"# {title}",
        "",
        f"constants file sha256 now: {const.yaml_sha256} (registered {b5.registered_sha()})",
        f"constants file sha256 in the run files: {sorted({m['prereg_yaml_sha256'] for m in metas})}",
        f"git sha of the checkout in the run files: {sorted({m['git_sha'] for m in metas})}",
        f"runs: {len(metas)}",
        "",
        "## Attempts",
        "",
        *attempts_lines(out),
        "",
    ]


def b5_report_text(out: Path, development: bool) -> str:
    const = b5.load_constants()
    mode = "b5-development" if development else "b5"
    runs = b5.load_runs(out, mode, b5.B5_CONDITIONS)
    lines = header_lines(f"Battery B5 report ({mode})", out, const)
    lines += ["## Runs", "", b5.run_table(runs), ""]
    if development:
        lines.append("Development: labelled development, no gate decided.")
        lines += ["", *b5.per_case_medians(runs), "", *b5.innovation_per_lap(runs, "OD"), ""]
        lines += b5.obstacle_clearance_lines(runs)
        return "\n".join(lines)
    cases = list(const.battery_cases)
    check = out / "o4_check.json"
    if check.exists():
        doc = json.loads(check.read_text())
        lines.append(
            f"O4 check: dropped {doc['dropped']}, laps {doc['laps_by_direction']}, notes {doc['notes']}"
        )
        if doc["dropped"]:
            cases.remove("O4")
    conditions = sorted({r.condition for r in runs})
    want = {
        (c, case, d, s)
        for c in conditions
        for case in cases
        for d in const.directions
        for s in const.battery_seeds
    }
    have = {(r.condition, r.case, r.direction, r.seed) for r in runs}
    if want != have or not {"a", "b", "c"} <= set(conditions):
        lines.append(
            f"INCOMPLETE: {len(want - have)} runs missing, {len(have - want)} unexpected; no decision is made"
        )
        return "\n".join(lines)
    rep = b5.build_report(runs, const)
    lines.append("## Decisions")
    for dec in rep.decisions.values():
        lines += ["", str(dec)]
    lines += ["", "## Reported without a threshold", ""] + [
        x if x == "" or x.startswith("|") else f"- {x}" for x in rep.extra
    ]
    return "\n".join(lines)


def load_dirs(dirs: list[Path], mode_for: dict[Path, str], conditions: tuple[str, ...]) -> list:
    runs = []
    for d in dirs:
        runs += b5.load_runs(d, mode_for[d], conditions)
    return runs


def merge_reruns(runs: list, rerun_dir: Path | None, mode: str, conditions: tuple[str, ...]):
    """Runs of `rerun_dir` replace the runs of the same condition, case, direction and seed (a
    rerun forced by the host, reported next to the first run). Returns (runs, report lines)."""
    if rerun_dir is None:
        return runs, []
    reruns = b5.load_runs(rerun_dir, mode, conditions)
    keys = {(r.condition, *r.key) for r in reruns}
    kept = [r for r in runs if (r.condition, *r.key) not in keys]
    lines = [
        f"rerun of {r.condition}/{r.case}/{r.direction}/{r.seed} from {rerun_dir.name} replaces "
        f"the first run (the first run stays in its own directory)"
        for r in reruns
    ]
    return kept + reruns, lines


def ma2_report_text(
    kind: str,
    out: Path,
    ma2_dir: Path | None,
    f_label: str,
    factor_dirs: list[Path],
    rerun_dir: Path | None = None,
) -> str:
    const = b5.load_constants()
    ext = b5.load_extensions_checked()
    if kind == "ma2":
        runs = b5.load_runs(out, "ma2", ("g", "o"))
        runs, notes = merge_reruns(runs, rerun_dir, "ma2", ("g", "o"))
        lines = header_lines("M-A2, does a perfect track change lap time", out, const)
        lines += [*notes, b5.ma2_report(runs, const), ""]
        lines += [
            "Stated limit: a fixed-planner oracle measures the value of a better input to THIS planner,",
            "not a bound on perception.",
        ]
        return "\n".join(lines)
    if kind == "ma2x":
        if ma2_dir is None:
            raise BatteryError("report ma2x needs --ma2-dir (the M-A2 runs, conditions g and o)")
        runs = b5.load_runs(out, "ma2x", ("g", "o", "f")) + b5.load_runs(ma2_dir, "ma2", ("g", "o"))
        lines = header_lines("M-A2x, the learned candidate as a third condition", out, const)
        lines += [b5.ma2x_report(runs, f_label, const)]
        return "\n".join(lines)
    if kind == "ma2w":
        lines = header_lines("M-A2w, boundary weight at half and double", out, const)
        limit = float(ext["weight_check"]["headroom_if_median_ratio_at_most"])
        sentences = []
        for d in factor_dirs:
            runs = b5.load_runs(d, "ma2w", ("g", "o"))
            metas = [json.loads(p.read_text())[META] for p in sorted(d.glob("*_*_*_*.json"))]
            factor = metas[0]["weight_factor"]
            pairs = b5.pairs_against(runs, "o", "g", const)
            _s, numbers = b5.ma2_reading(pairs, const)
            med = sorted(p.ratio for p in pairs if p.valid)
            import statistics

            median = statistics.median(med) if med else float("nan")
            lines += [
                f"## boundary weight x {factor} (w_bound {metas[0]['w_bound']})",
                "",
                b5.pair_table(pairs, "o", "g"),
                "",
                numbers,
                "",
            ]
            if median <= limit:
                sentences.append(f"x {factor}: median {median:.3f} <= {limit}")
        lines.append(
            "headroom only with other planner weights: "
            + (
                "YES (" + "; ".join(sentences) + ")"
                if sentences
                else f"no (median ratio above {limit} at both settings)"
            )
        )
        return "\n".join(lines)
    raise BatteryError(f"unknown report kind {kind}")


def report_cmd(args: argparse.Namespace) -> None:
    out = Path(args.out)
    if args.kind == "b5":
        text = b5_report_text(out, args.development)
    else:
        text = ma2_report_text(
            args.kind,
            out,
            Path(args.ma2_dir) if args.ma2_dir else None,
            args.f_label,
            [Path(d) for d in args.factor_dirs],
            Path(args.rerun_dir) if args.rerun_dir else None,
        )
    print(text)
    (out / "report.md").write_text(text + "\n")


def track_check_cmd(args: argparse.Namespace) -> None:
    const = b5.load_constants()
    lines = check_obstacle_tracks(const, Path(args.data_root))
    print("\n".join(lines))
    if any("FAILED" in ln for ln in lines):
        raise BatteryError("a registered case cannot be built as written: report it, move nothing")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter
    )
    sub = parser.add_subparsers(dest="command", required=True)

    def host(p: argparse.ArgumentParser) -> None:
        p.add_argument("--workers", type=int, default=1)
        p.add_argument("--domain-base", type=int, default=120)
        p.add_argument("--bundle", default=str(DEFAULT_BUNDLE))
        p.add_argument("--data-root", default=str(DEFAULT_DATA_ROOT))

    p = sub.add_parser("track-check")
    p.add_argument("--data-root", default=str(DEFAULT_DATA_ROOT))
    p = sub.add_parser("freeze-check")
    p.add_argument("--b4-sha", required=True)
    p = sub.add_parser("b5")
    p.add_argument("--out", required=True)
    p.add_argument("--development", action="store_true")
    p.add_argument("--conditions", default="abc")
    p.add_argument("--variant", default="", help="condition v: the D v1 variant d or e")
    p.add_argument("--cases", default="")
    p.add_argument("--b4-dir", default="")
    host(p)
    p = sub.add_parser("o4-check")
    p.add_argument("--out", required=True)
    p.add_argument("--b4-dir", required=True)
    host(p)
    p = sub.add_parser("ma2")
    p.add_argument("--out", required=True)
    p.add_argument("--stage", default="ma2", choices=("ma2", "ma2x", "ma2w"))
    p.add_argument("--perception-bundle", default="")
    p.add_argument("--weight-factor", type=float, default=None)
    p.add_argument("--only", default="", help="comma separated job names: a rerun of those runs")
    host(p)
    p = sub.add_parser("report")
    p.add_argument("--kind", required=True, choices=("b5", "ma2", "ma2x", "ma2w"))
    p.add_argument("--out", required=True)
    p.add_argument("--development", action="store_true")
    p.add_argument("--ma2-dir", default="")
    p.add_argument("--f-label", default="")
    p.add_argument("--factor-dirs", nargs="*", default=[])
    p.add_argument("--rerun-dir", default="", help="ma2: runs that replace first runs")
    args = parser.parse_args(argv)
    try:
        {
            "track-check": track_check_cmd,
            "freeze-check": freeze_check_cmd,
            "b5": b5_run_cmd,
            "o4-check": o4_check_cmd,
            "ma2": ma2_run_cmd,
            "report": report_cmd,
        }[args.command](args)
    except (BatteryError, b4.PreregError) as exc:
        print(f"REFUSED: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
