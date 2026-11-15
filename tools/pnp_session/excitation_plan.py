"""The M-SID2 excitation plan of 2026-11-15 (brief 41b): blocks P, G, L and F.

Reads the `excitation` block of the frozen v0.5 dynamics yaml
(`evaluation/prereg/pnp-v0.5-dynamics.yaml`), prints the run table with the exact launch lines and
refuses a plan above the yaml's `max_total_driving_s`. It launches nothing and publishes nothing.

    python3 tools/pnp_session/excitation_plan.py [--yaml <path>] [--dyn <bundle>] [--out <dir>]

The cap schedule tool (`cap_schedule.py`, block P) is unchanged and still reads the addendum 1
yaml. Nothing here changes a speed, a lateral limit or a duration of the yaml.
"""

from __future__ import annotations

import argparse
import ast
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
PLAN_YAML = "evaluation/prereg/pnp-v0.5-dynamics.yaml"
GAP_FOLLOW_LAUNCH = "ros_ws/src/racer_control/launch/gap_follow.launch.py"
EXIT_REFUSED = 2
DIRECTIONS = ("cw", "ccw")

RULES = (
    "A wall contact ends the blocks of that direction.",
    "Rung F 1.5 only if rung F 1.4 had no contact.",
    "Block F only with the owner's yes at the track.",
    "If L stalls twice in its first 20 s, stop it and go on (its time is not given to another block).",
    "A safety brake is not a contact.",
)


class PlanRefused(ValueError):
    """The plan must not be driven."""


@dataclass(frozen=True)
class Run:
    order: int
    block: str
    direction: str
    seconds: float
    #: gap follow launch arguments in launch order ("name:=value"); empty for block P.
    args: tuple[str, ...] = ()
    max_speed_mps: float | None = None

    @property
    def label(self) -> str:
        if self.block == "P":
            return "P " + self.direction
        return f"{self.block} {self.direction} {self.max_speed_mps}"

    def launch_line(self, dyn: str = "<dyn>", out: str = "<out>") -> list[str]:
        if self.block == "P":
            return pnp_lines(dyn, out, self.max_speed_mps or 0.0)
        return ["ros2 launch racer_control gap_follow.launch.py " + " ".join(self.args)]


def pnp_lines(dyn: str, out: str, v_max_mps: float) -> list[str]:
    manifest = f"/workspace/data/pnp/models/{dyn}/manifest.yaml"
    tool = (
        f"python3 tools/pnp_session/cap_schedule.py --manifest {manifest} --v-max-mps {v_max_mps:g}"
    )
    params = f"{out}/excitation_params.yaml"
    launch = (
        f"ros2 launch racer_bringup pnp.launch.py dynamics_bundle:={dyn} backend:=geometric "
        f"v_max_mps:={v_max_mps:g} lap_counter:=true require_lap_counter:=true "
        f"params_file:={params}"
    )
    return [f"{tool} --write-params {params}", launch, tool]


def _fmt(v: Any) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    return str(v)


def _gap_args(launch: dict[str, Any], extra: dict[str, Any]) -> tuple[str, ...]:
    """profile first, then the yaml's other launch keys in file order, then the run's own."""
    merged: dict[str, Any] = {}
    if "profile" in launch:
        merged["profile"] = launch["profile"]
    for k, v in {**launch, **extra}.items():
        merged.setdefault(k, v)
    # brief 41b order: profile, target_mode (L), max_speed_mps, a_lat_max_mps2, reverse_escape
    order = ["profile", "target_mode", "max_speed_mps", "a_lat_max_mps2", "reverse_escape"]
    keys = [k for k in order if k in merged] + [k for k in merged if k not in order]
    return tuple(f"{k}:={_fmt(merged[k])}" for k in keys)


def load_block(path: Path | None = None) -> dict[str, Any]:
    doc = yaml.safe_load(Path(path or REPO_ROOT / PLAN_YAML).read_text())
    return doc["excitation"]


def build_plan(block: dict[str, Any]) -> list[Run]:
    """Every run in drive order: all clockwise blocks first, then counter-clockwise."""
    blocks = block["blocks"]
    runs: list[Run] = []
    for d in DIRECTIONS:
        p = blocks["P"]
        for r in p["runs"]:
            if r["direction"] == d:
                runs.append(Run(0, "P", d, float(r["s"]), (), max(p["caps_mps"])))
        g = blocks["G"]
        for r in g["runs"]:
            if r["direction"] == d:
                extra = {"max_speed_mps": float(r["max_speed_mps"])}
                runs.append(
                    Run(
                        0,
                        "G",
                        d,
                        float(r["s"]),
                        _gap_args(g["launch"], extra),
                        extra["max_speed_mps"],
                    )
                )
        lb = blocks["L"]
        for r in lb["runs"]:
            if r["direction"] == d:
                runs.append(
                    Run(
                        0,
                        "L",
                        d,
                        float(r["s"]),
                        _gap_args(lb["launch"], {}),
                        float(lb["launch"]["max_speed_mps"]),
                    )
                )
        f = blocks["F"]
        if d in f["directions"]:
            for rung in f["rungs"]:
                extra = {k: float(rung[k]) for k in ("max_speed_mps", "a_lat_max_mps2")}
                runs.append(
                    Run(
                        0,
                        "F",
                        d,
                        float(rung["s"]),
                        _gap_args(f["launch"], extra),
                        extra["max_speed_mps"],
                    )
                )
    return [
        Run(i + 1, r.block, r.direction, r.seconds, r.args, r.max_speed_mps)
        for i, r in enumerate(runs)
    ]


def total_s(plan: Sequence[Run]) -> float:
    return sum(r.seconds for r in plan)


def check_budget(plan: Sequence[Run], block: dict[str, Any]) -> None:
    """Refuse a plan above the yaml's budget, or one that disagrees with its planned total."""
    limit = float(block["max_total_driving_s"])
    if total_s(plan) > limit:
        raise PlanRefused(
            f"the plan needs {total_s(plan):.0f} s of driving, the yaml allows {limit:.0f} s"
        )
    planned = float(block["planned_total_driving_s"])
    if abs(total_s(plan) - planned) > 1e-9:
        raise PlanRefused(
            f"the blocks add up to {total_s(plan):.0f} s, the yaml says {planned:.0f} s"
        )


def declared_gap_follow_arguments(path: Path | None = None) -> set[str]:
    """Launch arguments `gap_follow.launch.py` declares, read from its source (no ROS needed)."""
    tree = ast.parse(Path(path or REPO_ROOT / GAP_FOLLOW_LAUNCH).read_text())
    names: set[str] = {"profile", "max_speed_mps", "params_file"}
    for node in ast.walk(tree):
        targets = []
        if isinstance(node, ast.Assign):
            targets = [t for t in node.targets if isinstance(t, ast.Name)]
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            targets = [node.target]
        value = getattr(node, "value", None)
        if any(
            t.id.startswith("_OPTIONAL_") and t.id.endswith("_ARGS") for t in targets
        ) and isinstance(value, ast.Dict):
            names |= {k.value for k in value.keys if isinstance(k, ast.Constant)}
        is_declare = (
            isinstance(node, ast.Call) and getattr(node.func, "id", "") == "DeclareLaunchArgument"
        )
        if is_declare and node.args and isinstance(node.args[0], ast.Constant):
            names.add(node.args[0].value)
    return names


def render(
    plan: Sequence[Run], block: dict[str, Any], dyn: str = "<dyn>", out: str = "<out>"
) -> str:
    lines = [f"{'#':>2}  {'block':<12} {'s':>4}  launch"]
    for r in plan:
        first, *rest = r.launch_line(dyn, out)
        lines.append(f"{r.order:>2}  {r.label:<12} {r.seconds:>4.0f}  {first}")
        lines += [f"{'':>2}  {'':<12} {'':>4}  {x}" for x in rest]
    lines.append(
        f"planned {total_s(plan):.0f} s, budget {float(block['max_total_driving_s']):.0f} s, "
        "every run in the stack with safety_node and the recorder, two people, kill in hand"
    )
    lines += ["rule: " + r for r in RULES]
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="excitation_plan", description=__doc__.split("\n")[0])
    ap.add_argument("--yaml", help="the v0.5 dynamics yaml (default: the frozen one)")
    ap.add_argument("--dyn", default="<dyn>", help="dynamics bundle name for block P")
    ap.add_argument("--out", default="<out>", help="session folder for block P")
    args = ap.parse_args(argv)
    try:
        block = load_block(Path(args.yaml) if args.yaml else None)
        plan = build_plan(block)
        check_budget(plan, block)
        declared = declared_gap_follow_arguments()
        for r in plan:
            for a in r.args:
                if a.split(":=")[0] not in declared:
                    raise PlanRefused(f"{a.split(':=')[0]} is not a gap_follow.launch.py argument")
    except PlanRefused as err:
        print(f"REFUSED: {err}", file=sys.stderr)
        return EXIT_REFUSED
    print(render(plan, block, args.dyn, args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
