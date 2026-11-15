"""The v0.4 session report (brief 41, pre-registration PnP v0.4).

One command after step 1: the M-T timing report (v0.2 4.2 judged, a miss goes to the owner), the
D3-C0 check (v0.3 1.5, v0.4 4.4) against the reference the pre-registration names, `lapref` printed
next to it (it decides nothing, v0.4 4.4), and which later steps of
tests/bench/procedures/pnp_session_v04.yaml those results allow.

D3-C0's reference (v0.4 4.4): "the camera lap counter if the owner accepts it as primary and M-CAM
held; otherwise the phone video of the same two runs". `choose_reference` is that sentence.

The pre-registration constants file is hashed on every run (`check_prereg`); a mismatch refuses.
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from pnp_session import laps, report, timing, whatif
from pnp_session.bagread import SessionBag
from pnp_session.constants import D3C0_TOLERANCE_S

REPO_ROOT = Path(__file__).resolve().parents[2]
PREREG_YAML = "evaluation/prereg/pnp-v0.4.yaml"
#: docs/notes/preregistration-pnp-v0.4.md: "Constants file ... SHA-256".
PREREG_SHA256 = "78510c1423278f80c42511e6caab095818598af6db6a284dd5479f74a03a1d63"
PROCEDURE = "tests/bench/procedures/pnp_session_v04.yaml"
SCHEMA = "pnp-session/1"
STATUSES = ("met", "unmet", "unknown")


class PreregMismatch(RuntimeError):
    """The pre-registration constants file is missing or is not the frozen one."""


def check_prereg(repo_root: Path | None = None) -> str:
    """The SHA-256 of the v0.4 constants file when it is the frozen one; PreregMismatch otherwise."""
    path = Path(repo_root or REPO_ROOT) / PREREG_YAML
    if not path.is_file():
        raise PreregMismatch(f"{PREREG_YAML} is not on this checkout: pull main")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != PREREG_SHA256:
        raise PreregMismatch(
            f"{PREREG_YAML} has SHA-256 {digest}, the frozen file has {PREREG_SHA256}: refusing"
        )
    return digest


def lidar_mount_z_m(repo_root: Path | None = None) -> float:
    """The LiDAR mount height from the generated vehicle_params binding (never typed here)."""
    import sys

    root = Path(repo_root or REPO_ROOT)
    src = str(root / "ros_ws" / "src" / "racer_track_perception" / "src")
    if src not in sys.path:
        sys.path.insert(0, src)
    from racer_track_perception.vehicle import load_generated_params

    return float(load_generated_params(root).sensors.lidar.mount_z_m)


def choose_reference(owner_accepted_camera: bool, m_cam_held: bool) -> str:
    """v0.4 4.4: the camera counter if the owner accepted it as primary and M-CAM held, else the
    phone video."""
    return "camera" if owner_accepted_camera and m_cam_held else "video"


def read_lapref(path: Path) -> tuple[float, ...]:
    """The lap times of one lapref JSON (tools/lapref.py, brief 36). A run it found no period in
    has no laps."""
    doc = json.loads(Path(path).read_text())
    for key in ("lap_times", "lap_times_s"):
        if key in doc:
            return tuple(float(x) for x in doc[key] or [])
    raise ValueError(f"{path}: no lap_times in the lapref JSON")


def lapref_next_to(inputs: Sequence[laps.RunLapInput], lapref_dir: Path | None) -> dict[str, Any]:
    """Per run: lapref's laps against the reference's, count and laps within 0.3 s. Decides
    nothing (v0.4 4.4)."""
    rows: list[dict[str, Any]] = []
    for inp in inputs:
        row: dict[str, Any] = {"run_id": inp.run_id, "n_reference": len(inp.reference_times)}
        path = None if lapref_dir is None else Path(lapref_dir) / f"{inp.run_id}.json"
        if path is None or not path.is_file():
            rows.append({**row, "lapref": "not given"})
            continue
        ref = read_lapref(path)
        n = min(len(ref), len(inp.reference_times))
        diffs = [float(ref[i]) - float(inp.reference_times[i]) for i in range(n)]
        within = sum(abs(d) <= D3C0_TOLERANCE_S + 1e-9 for d in diffs)
        rows.append(
            {
                **row,
                "lapref": "read",
                "n_lapref": len(ref),
                "count_equal": len(ref) == len(inp.reference_times),
                "laps_compared": n,
                "laps_within": within,
                "diffs_s": diffs,
            }
        )
    return {
        "section": "v0.4 4.4 (M-LR, M-L1)",
        "tolerance_s": D3C0_TOLERANCE_S,
        "rows": rows,
        "decides_nothing": True,
        "note": "lapref decides nothing unless the owner makes it the reference (v0.4 section 9)",
    }


def _result_status(verdict: str) -> str:
    return {"PASS": "met", "FAIL": "unmet"}.get(verdict, "unknown")


def load_procedure(path: Path | None = None) -> dict[str, Any]:
    doc = yaml.safe_load(Path(path or REPO_ROOT / PROCEDURE).read_text())
    if not isinstance(doc, dict) or "steps" not in doc or "results" not in doc:
        raise ValueError("not a v0.4 session procedure")
    return doc


def allowed_steps(procedure: Mapping[str, Any], results: Mapping[str, str]) -> list[dict[str, Any]]:
    """Which session steps the known results allow. `results` overrides the registry statuses
    of the procedure file. A step is runnable only when every result it requires is `met`:
    `unmet` and `unknown` both block it."""
    status = {k: str(v.get("status", "unknown")) for k, v in procedure["results"].items()}
    for key, value in results.items():
        if key not in status:
            raise ValueError(f"unknown result {key!r}: the procedure knows {sorted(status)}")
        if value not in STATUSES:
            raise ValueError(f"result {key}={value!r}: use one of {list(STATUSES)}")
        status[key] = value
    out = []
    for step in procedure["steps"]:
        needs = [r["result"] for r in step.get("requires", [])]
        blocked = [f"{n} is {status[n]}" for n in needs if status[n] != "met"]
        out.append(
            {
                "step": step["session_step"],
                "id": step["id"],
                "minimum_session": bool(step.get("minimum_session", False)),
                "requires": needs,
                "runnable": not blocked,
                "blocked_by": blocked,
            }
        )
    return out


def v04_report(
    bag: SessionBag,
    *,
    caps: Sequence[float] | None,
    wall_contacts: int = 0,
    reference: str,
    video_times: Mapping[str, tuple[float, ...]] | None = None,
    lapref_dir: Path | None = None,
    results: Mapping[str, str] | None = None,
    limiter: Mapping[str, Any] | None = None,
    procedure: Mapping[str, Any] | None = None,
    prereg_sha256: str = PREREG_SHA256,
) -> dict[str, Any]:
    """The whole report as a JSON-able dict (no clocks, no absolute paths)."""
    if reference not in ("camera", "video"):
        raise ValueError("reference must be camera or video")
    analysis = timing.analyse_timing(bag, list(caps) if caps else None, wall_contacts)
    timing_doc = report.assemble(
        bag.name,
        analysis.report,
        dict(limiter) if limiter is not None else whatif.not_run("--no-limiter given"),
        False,
    )
    inputs = laps.inputs_from_bag(bag, (video_times or {}) if reference == "video" else None)
    d3c0 = laps.d3c0(inputs, reference)
    merged = {"d3_c0": _result_status(d3c0["verdict"]), **(results or {})}
    proc = procedure if procedure is not None else load_procedure()
    return {
        "schema": SCHEMA,
        "report": "v04",
        "bag": bag.name,
        "preregistration": {"file": PREREG_YAML, "sha256": prereg_sha256},
        "timing": timing_doc,
        "d3_c0": d3c0,
        "lapref": lapref_next_to(inputs, lapref_dir),
        "results_used": dict(sorted(merged.items())),
        "steps": allowed_steps(proc, merged),
    }


def markdown(doc: Mapping[str, Any]) -> str:
    from pnp_session import cli

    d3 = doc["d3_c0"]
    lines = [
        f"# v0.4 session report: {doc['bag']}",
        "",
        (
            f"Pre-registration constants: {doc['preregistration']['file']} "
            f"(SHA-256 {doc['preregistration']['sha256']}, matched)."
        ),
        "",
        "## M-T timing (v0.2 4.2 judged here; a miss goes to the owner)",
        "",
        report.markdown(dict(doc["timing"])),
        "## D3-C0",
        "",
        cli.laps_markdown({"bag": doc["bag"], **d3}),
        "## lapref next to it (decides nothing)",
        "",
    ]
    for row in doc["lapref"]["rows"]:
        if row["lapref"] != "read":
            lines.append(f"- {row['run_id']}: lapref {row['lapref']}")
            continue
        worst = max((abs(d) for d in row["diffs_s"]), default=0.0)
        lines.append(
            f"- {row['run_id']}: lapref {row['n_lapref']} laps, reference {row['n_reference']}, "
            f"count {'equal' if row['count_equal'] else 'DIFFERENT'}, "
            f"{row['laps_within']} of {row['laps_compared']} within {doc['lapref']['tolerance_s']} s, "
            f"worst {worst:.3f} s"
        )
    lines += [
        "",
        "## Later steps the results allow",
        "",
        "| step | id | runnable | blocked by |",
        "| --- | --- | --- | --- |",
    ]
    for s in doc["steps"]:
        mark = " (minimum session)" if s["minimum_session"] else ""
        lines.append(
            f"| {s['step']} | {s['id']}{mark} | {'yes' if s['runnable'] else 'NO'} | "
            f"{'; '.join(s['blocked_by']) or '-'} |"
        )
    lines.append("")
    return "\n".join(lines)


def to_json(doc: Mapping[str, Any]) -> str:
    def default(o: Any) -> Any:
        if isinstance(o, np.integer):
            return int(o)
        if isinstance(o, np.floating):
            return float(o)
        if isinstance(o, np.bool_):
            return bool(o)
        raise TypeError(f"not serialisable: {type(o)}")

    return json.dumps(doc, indent=2, sort_keys=True, default=default) + "\n"
