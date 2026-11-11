"""Run specs, resumable execution on the simulator, and loading results back as RunRecords.

One run = one subprocess of tests/l5_reactive_bench/track_family_runner.py (the real command path:
gap_follow_node -> /drive_raw -> safety_node -> /drive -> bridge_node, the car as measured in
vehicle_params). It needs the ROS workspace sourced, so tools/race_tune_docker.sh runs the harness
inside the ros-dev image. Each run gets its own ROS_DOMAIN_ID; a run whose JSON exists is skipped,
so a killed search resumes.
"""

from __future__ import annotations

import json
import queue
import shutil
import subprocess
import sys
import time
from collections.abc import Callable, Iterable
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from race_tuning.selection import RunRecord, run_record
from race_tuning.tracks import DIRECTIONS, TrackSpec

REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER = REPO_ROOT / "tests" / "l5_reactive_bench" / "track_family_runner.py"
DEFAULT_PROFILE = "lane-2026-10-25"
DURATION_S = 150.0


@dataclass(frozen=True)
class RunSpec:
    variant: str
    track: TrackSpec
    direction: str
    params: tuple[tuple[str, Any], ...]  # gap_follow_node parameters on top of the profile
    profile: str = DEFAULT_PROFILE
    laps: int = 2

    @property
    def run_id(self) -> str:
        return f"{self.track.name}-s{self.track.seed}__{self.direction}"

    def path(self, out: Path) -> Path:
        return out / self.variant / f"{self.run_id}.json"


def make_specs(
    variant: str,
    params: dict[str, Any],
    tracks: Iterable[TrackSpec],
    laps: int,
    profile: str = DEFAULT_PROFILE,
    directions: Iterable[str] = DIRECTIONS,
) -> list[RunSpec]:
    frozen = tuple(sorted(params.items()))
    return [
        RunSpec(variant, track, direction, frozen, profile, laps)
        for track in tracks
        for direction in directions
    ]


def command(spec: RunSpec, domain_id: int, out: Path, python: str = sys.executable) -> list[str]:
    cmd = [
        python,
        str(RUNNER),
        "--track",
        spec.track.track,
        "--track-params-json",
        json.dumps(spec.track.params_dict()),
        "--driver",
        "gap_follow",
        "--profile",
        spec.profile,
        "--params-json",
        json.dumps(dict(spec.params)),
        "--stop-after-laps",
        str(spec.laps),
        "--duration",
        str(DURATION_S),
        "--seed",
        str(spec.track.seed),
        "--domain-id",
        str(domain_id),
        "--work-dir",
        str(out / "work" / spec.variant / spec.run_id),
        "--out",
        str(spec.path(out)),
    ]
    if spec.direction == "cw":
        cmd.append("--reverse")
    return cmd


def run_one(spec: RunSpec, domain_id: int, out: Path) -> dict[str, Any]:
    path = spec.path(out)
    if path.exists():
        return json.loads(path.read_text())
    path.parent.mkdir(parents=True, exist_ok=True)
    start = time.time()
    proc = subprocess.run(
        command(spec, domain_id, out),
        capture_output=True,
        text=True,
        check=False,
        timeout=900,
    )
    if not path.exists():
        path.write_text(
            json.dumps({"error": f"runner exited {proc.returncode}: {proc.stderr[-800:]}"}) + "\n"
        )
    result = json.loads(path.read_text())
    result["wall_s"] = round(time.time() - start, 1)
    path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    if "error" not in result:
        shutil.rmtree(out / "work" / spec.variant / spec.run_id, ignore_errors=True)
    return result


def execute(
    specs: list[RunSpec],
    out: Path,
    jobs: int,
    domain_base: int = 200,
    progress: Callable[[RunSpec, dict[str, Any]], None] | None = None,
) -> None:
    """Run every spec not yet done, `jobs` at a time, one ROS domain per concurrent runner."""
    domains: queue.Queue[int] = queue.Queue()
    for i in range(jobs):
        domains.put(domain_base + i)

    def worker(spec: RunSpec) -> dict[str, Any]:
        domain = domains.get()
        try:
            return run_one(spec, domain, out)
        finally:
            domains.put(domain)

    with ThreadPoolExecutor(max_workers=jobs) as pool:
        futures = {pool.submit(worker, s): s for s in specs}
        for future in as_completed(futures):
            if progress:
                progress(futures[future], future.result())


def load_records(out: Path, specs: Iterable[RunSpec]) -> list[RunRecord]:
    """The RunRecords of the specs whose result file exists, in spec order."""
    records = []
    for spec in specs:
        path = spec.path(out)
        if path.exists():
            records.append(
                run_record(json.loads(path.read_text()), spec.track.name, spec.direction, spec.laps)
            )
    return records
