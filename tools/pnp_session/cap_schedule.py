"""The excitation schedule of M-SID (addendum 1 section 6, yaml block `excitation`).

    "M-SID, excitation runs: per direction, the PnP planner with its cap stepped through the yaml
    list, held 8 s each, three cycles ... Commands are model inputs, not imitation targets."

The tool publishes `racer_pnp_msgs/LapGuidance` on /pnp/lap/guidance with only `v_cap_mps` set (an
empty profile: the D v0 form) on that schedule. The planner runs with `lap_memory:=false` and its
guidance input on (`use_lap_guidance`, set by the params file this tool writes with
`--write-params`); nothing but the planner's own speed cap changes. This tool never creates a
publisher on any drive topic: the planner publishes /drive_raw and safety_node gates it to /drive,
exactly as in every other run. A dead tool leaves the planner at `v_no_guidance_mps` (the lowest
cap of the schedule, written into the params file).

It refuses before it starts when
- a cap is above the bundle's validity (`io.validity.v_max_mps` of the manifest) or above the
  planner's `v_max_mps`;
- another node already publishes /pnp/lap/guidance (the lap node driving the cap): the two must
  never run together.

The caps, hold time and cycles come from the frozen addendum yaml, never from a flag.

    python3 tools/pnp_session/cap_schedule.py --manifest <bundle>/manifest.yaml --v-max-mps <v> \
        [--write-params <out>/excitation_params.yaml]
"""

from __future__ import annotations

import argparse
import math
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
EXCITATION_YAML = "evaluation/prereg/pnp-v0.4-addendum-1.yaml"
GUIDANCE_TOPIC = "/pnp/lap/guidance"
#: The lap node publishes at 10 Hz (guidance_rate_hz); the planner's guidance timeout is 2.0 s.
PUBLISH_RATE_HZ = 10.0
#: How long to listen for another publisher on the topic before starting.
DISCOVERY_WAIT_S = 2.0
EXIT_REFUSED = 2


class CapRefused(ValueError):
    """The schedule must not start."""


@dataclass(frozen=True)
class Excitation:
    caps_mps: tuple[float, ...]
    hold_s: float
    cycles: int


def load_excitation(path: Path | None = None) -> Excitation:
    """The `excitation` block of the addendum yaml."""
    doc = yaml.safe_load(Path(path or REPO_ROOT / EXCITATION_YAML).read_text())
    block = doc["excitation"]
    return Excitation(
        tuple(float(c) for c in block["caps_mps"]),
        float(block["hold_s"]),
        int(block["cycles_per_direction"]),
    )


def bundle_validity_mps(manifest: Path) -> float:
    """`io.validity.v_max_mps` of a model bundle manifest."""
    doc = yaml.safe_load(Path(manifest).read_text())
    try:
        return float(doc["io"]["validity"]["v_max_mps"])
    except (KeyError, TypeError, ValueError) as exc:
        raise CapRefused(f"{manifest}: no io.validity.v_max_mps") from exc


def check_caps(caps: Sequence[float], validity_mps: float, v_max_mps: float) -> None:
    """Refuse (CapRefused) a cap that is not finite and positive, above the bundle's validity or
    above the planner's v_max_mps. Called before anything is published."""
    if not caps:
        raise CapRefused("the schedule has no caps")
    for c in caps:
        if not math.isfinite(c) or c <= 0.0:
            raise CapRefused(f"cap {c} m/s is not a positive finite speed")
        if c > validity_mps:
            raise CapRefused(f"cap {c} m/s is above the bundle's validity {validity_mps} m/s")
        if c > v_max_mps:
            raise CapRefused(f"cap {c} m/s is above the planner's v_max_mps {v_max_mps} m/s")


def refuse_if_other_publisher(n_publishers: int) -> None:
    """The lap node (or a second tool) publishing guidance would fight this one for the cap."""
    if n_publishers > 0:
        raise CapRefused(
            f"{n_publishers} node(s) already publish {GUIDANCE_TOPIC}: this tool never runs "
            "together with the lap node driving the cap (launch with lap_memory:=false)"
        )


class CapSchedule:
    """The caps stepped through in list order, each held `hold_s`, repeated `cycles` times."""

    def __init__(self, exc: Excitation) -> None:
        self.exc = exc

    @property
    def duration_s(self) -> float:
        return len(self.exc.caps_mps) * self.exc.hold_s * self.exc.cycles

    def cap_at(self, t_s: float) -> float | None:
        """The cap at `t_s` seconds after the start; None before the start and from the end on."""
        if t_s < 0.0 or t_s >= self.duration_s:
            return None
        k = int(t_s // self.exc.hold_s) % len(self.exc.caps_mps)
        return self.exc.caps_mps[k]

    def steps(self) -> list[tuple[float, float, float]]:
        out = []
        for i in range(len(self.exc.caps_mps) * self.exc.cycles):
            t0 = i * self.exc.hold_s
            out.append((t0, t0 + self.exc.hold_s, self.exc.caps_mps[i % len(self.exc.caps_mps)]))
        return out


def guidance_fields(cap_mps: float, lap_index: int = 1) -> dict[str, Any]:
    """The LapGuidance fields: only the cap is set, the profile arrays are empty (D v0 form)."""
    return {
        "lap_index": lap_index,
        "v_cap_mps": float(cap_mps),
        "s_m": [],
        "v_ref_mps": [],
        "cost_to_go_s": [],
    }


def planner_params_yaml(v_no_guidance_mps: float) -> str:
    """ROS params for mppi_planner_node: the guidance input on, and the speed it falls back to
    when this tool dies."""
    doc = {
        "mppi_planner_node": {
            "ros__parameters": {
                "use_lap_guidance": True,
                "v_no_guidance_mps": float(v_no_guidance_mps),
            }
        }
    }
    return yaml.safe_dump(doc, sort_keys=True)


def run_schedule(
    schedule: CapSchedule, publish_hz: float = PUBLISH_RATE_HZ
) -> int:  # pragma: no cover - needs rclpy and a running graph
    import rclpy
    from racer_pnp_msgs.msg import LapGuidance
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node("pnp_cap_schedule")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE
    )
    pub = node.create_publisher(LapGuidance, GUIDANCE_TOPIC, qos)
    try:
        # Listen first: another publisher on the topic means the lap node is up.
        deadline = node.get_clock().now().nanoseconds * 1e-9 + DISCOVERY_WAIT_S
        while node.get_clock().now().nanoseconds * 1e-9 < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        others = [
            i
            for i in node.get_publishers_info_by_topic(GUIDANCE_TOPIC)
            if i.node_name != node.get_name()
        ]
        refuse_if_other_publisher(len(others))
        t0 = node.get_clock().now().nanoseconds * 1e-9
        last = None
        while rclpy.ok():
            t = node.get_clock().now().nanoseconds * 1e-9 - t0
            cap = schedule.cap_at(t)
            if cap is None:
                break
            msg = LapGuidance()
            msg.header.stamp = node.get_clock().now().to_msg()
            fields = guidance_fields(cap)
            msg.lap_index = fields["lap_index"]
            msg.v_cap_mps = fields["v_cap_mps"]
            pub.publish(msg)
            if cap != last:
                node.get_logger().info(f"t {t:5.1f} s cap {cap} m/s")
                last = cap
            rclpy.spin_once(node, timeout_sec=1.0 / publish_hz)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="cap_schedule", description=__doc__.split("\n")[0])
    p.add_argument("--manifest", required=True, help="manifest.yaml of the active dynamics bundle")
    p.add_argument("--v-max-mps", type=float, required=True, help="the planner's v_max_mps")
    p.add_argument("--write-params", help="write the planner params file here and exit")
    p.add_argument("--excitation-yaml", help="override the addendum yaml path (tests)")
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        exc = load_excitation(Path(args.excitation_yaml) if args.excitation_yaml else None)
        check_caps(exc.caps_mps, bundle_validity_mps(Path(args.manifest)), args.v_max_mps)
        if args.write_params:
            Path(args.write_params).write_text(planner_params_yaml(min(exc.caps_mps)))
            print(f"planner params written to {args.write_params}")
            return 0
        schedule = CapSchedule(exc)
        print(
            f"excitation: caps {list(exc.caps_mps)} m/s, {exc.hold_s} s each, "
            f"{exc.cycles} cycles, {schedule.duration_s:.0f} s"
        )
        return run_schedule(schedule)
    except CapRefused as err:
        print(f"REFUSED: {err}", file=sys.stderr)
        return EXIT_REFUSED


if __name__ == "__main__":
    sys.exit(main())
