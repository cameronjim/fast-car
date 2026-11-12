"""The one-screen table of a replay run, and the comparison with a baseline JSON.

`flatten` turns one segment's result into {"perception.intrusion_mean": 0.14, ...}; the table has
one row per metric and, without a baseline, one column per segment; with a baseline, one block
per segment with the run's value next to the baseline's.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, TextIO

# (key in flatten(), label, format)
ROWS: list[tuple[str, str, str]] = [
    ("perception.scans", "scans (moving)", "{:.0f}"),
    ("perception.intrusion_mean", "intrusion mean", "{:.3f}"),
    ("perception.intrusion_p90", "intrusion p90", "{:.3f}"),
    ("perception.intrusion_share_over_0.10", "intrusion share > 0.10", "{:.2f}"),
    ("perception.boundary_residual_p50_m", "boundary residual p50 (m)", "{:.3f}"),
    ("perception.boundary_residual_p90_m", "boundary residual p90 (m)", "{:.3f}"),
    ("perception.valid_count_p10", "valid_count p10", "{:.1f}"),
    ("perception.valid_count_share_under_6", "valid_count share < 6", "{:.2f}"),
    ("perception.clearance_deficit_p90_m", "clearance deficit p90 (m)", "{:.3f}"),
    ("perception.clearance_deficit_share_over_0.15", "clearance deficit share > 0.15", "{:.2f}"),
    ("planner.eligible_ticks", "planner ticks (eligible)", "{:.0f}"),
    ("planner.state_counts.DRIVING", "  DRIVING", "{:.0f}"),
    ("planner.state_counts.NO_TRACK", "  NO_TRACK", "{:.0f}"),
    ("planner.state_counts.BLOCKED", "  BLOCKED", "{:.0f}"),
    ("planner.speed_zero_share", "speed command == 0 share", "{:.2f}"),
    ("planner.speed_below_dead_zone_share", "0 < speed < dead zone share", "{:.2f}"),
    ("planner.collision_fraction_mean", "collision fraction mean", "{:.2f}"),
    ("planner.steering_change.std_rad", "steering change std (rad)", "{:.3f}"),
    ("planner.steering_change.sign_flips_per_s", "steering sign flips per s", "{:.1f}"),
    ("planner.steering_change.p99_abs_rad", "steering change p99 (rad)", "{:.3f}"),
    ("planner.solve_time_ms_this_machine.p50", "solve p50 (ms, this machine)", "{:.2f}"),
    ("planner.solve_time_ms_this_machine.p99", "solve p99 (ms, this machine)", "{:.2f}"),
    ("planner.agreement_with_recorded.share", "state agrees with recorded", "{:.2f}"),
]


def flatten(segment: dict[str, Any]) -> dict[str, float]:
    """The numeric leaves of a segment's result, dotted ("planner.steering_change.std_rad")."""
    out: dict[str, float] = {}

    def walk(prefix: str, node: Any) -> None:
        if isinstance(node, dict):
            for key, value in node.items():
                walk(f"{prefix}.{key}" if prefix else key, value)
        elif isinstance(node, (int, float)) and not isinstance(node, bool):
            out[prefix] = float(node)
        elif node is None:
            out[prefix] = math.nan

    for part in ("perception", "planner"):
        if part in segment:
            walk(part, segment[part])
    return out


def _fmt(fmt: str, value: float | None) -> str:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "-"
    return fmt.format(value)


def format_table(result: dict[str, Any], baseline: dict[str, Any] | None = None) -> str:
    segments: dict[str, Any] = result["segments"]
    flat = {name: flatten(seg) for name, seg in segments.items()}
    lines: list[str] = []
    head = (
        f"{result.get('backend', '?')} perception ({result.get('perception_model_id', '?')}), "
        f"tracks {result.get('tracks', '?')}, planner v_max {result.get('v_max_mps', '?')} m/s"
    )
    lines.append(head)
    if result.get("clip_speed_to_cap"):
        lines.append(
            "NOTE: odometry fed to the planner was clipped to v_max (--clip-speed-to-cap); "
            "the car was not driving at that speed in the bag."
        )
    for name, seg in segments.items():
        fid = seg.get("fidelity")
        extra = ""
        if fid is not None and "skipped" in fid:
            extra = f", fidelity skipped ({fid['skipped']})"
        elif fid is not None:
            extra = (
                f", fidelity max centreline diff {fid['max_centreline_diff_m']:.2g} m over "
                f"{fid['matched']} tracks ({'ok' if fid['ok'] else 'FAILED'})"
            )
        lines.append(
            f"{name}: bag {seg['t_from']:.1f}-{seg['t_to']:.1f} s ({seg['duration_s']:.1f} s)"
            f"{extra}"
        )
    lines.append("")
    label_w = max(len(r[1]) for r in ROWS)
    if baseline is None:
        names = list(segments)
        lines.append(f"{'':{label_w}}  " + "  ".join(f"{n:>9}" for n in names))
        for key, label, fmt in ROWS:
            vals = [flat[n].get(key) for n in names]
            if all(v is None for v in vals):
                continue
            lines.append(f"{label:{label_w}}  " + "  ".join(f"{_fmt(fmt, v):>9}" for v in vals))
    else:
        base_flat = {n: flatten(s) for n, s in baseline.get("segments", {}).items()}
        for name in segments:
            lines.append(f"[{name}]  {'now':>9}  {'baseline':>9}")
            for key, label, fmt in ROWS:
                now = flat[name].get(key)
                then = base_flat.get(name, {}).get(key)
                if now is None and then is None:
                    continue
                lines.append(f"{label:{label_w}}  {_fmt(fmt, now):>9}  {_fmt(fmt, then):>9}")
            lines.append("")
    pooled = result.get("pooled")
    if pooled:
        lines.append(f"pooled over {', '.join(pooled['segments'])}:")
        p = flatten({"perception": pooled["perception"]})
        base_p = (
            flatten({"perception": baseline["pooled"]["perception"]})
            if baseline and baseline.get("pooled")
            else {}
        )
        for key, label, fmt in ROWS:
            if key in p:
                tail = f"  {_fmt(fmt, base_p.get(key)):>9}" if baseline else ""
                lines.append(f"{label:{label_w}}  {_fmt(fmt, p[key]):>9}{tail}")
    return "\n".join(lines)


def write_json(result: dict[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    def clean(node: Any) -> Any:
        if isinstance(node, dict):
            return {str(k): clean(v) for k, v in node.items()}
        if isinstance(node, (list, tuple)):
            return [clean(v) for v in node]
        if isinstance(node, float) and not math.isfinite(node):
            return None
        return node

    path.write_text(json.dumps(clean(result), indent=2, sort_keys=False) + "\n", encoding="utf-8")


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def emit(text: str, out: TextIO) -> None:
    out.write(text + "\n")
