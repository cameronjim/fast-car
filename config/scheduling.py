"""Loader and checks for config/scheduling.yaml (brief 52).

Plain Python with PyYAML only, so the launch files, the racer app agent and tools/pnp_session
can each load it by path (importlib) without a shared package. A bad file is refused with a
SchedulingConfigError; nothing is defaulted. No `from __future__ import annotations`: a dataclass
loaded by path needs its module in sys.modules otherwise, and every consumer would have to know.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

DEFAULT_PATH = Path(__file__).resolve().with_name("scheduling.yaml")
SCHEMA_VERSION = 1
#: policy name -> `chrt` option. Only FIFO is supported.
POLICIES = {"fifo": "-f"}
#: the two processes this file configures
NODES = ("safety_node", "mppi_planner_node")
PRIORITY_MIN = 1
PRIORITY_MAX = 90
#: every launch log line and every refusal about "asked for and not granted" starts with this
REFUSAL_PREFIX = "refusing to start: real-time scheduling was asked for and not granted:"


class SchedulingConfigError(ValueError):
    """config/scheduling.yaml is missing a key or holds a value the checks refuse."""


@dataclass(frozen=True)
class Scheduling:
    rtprio_ulimit: int
    safety_priority: int
    planner_priority: int
    safety_policy: str = "fifo"
    planner_policy: str = "fifo"

    def priority(self, node: str) -> int:
        if node == "safety_node":
            return self.safety_priority
        if node == "mppi_planner_node":
            return self.planner_priority
        raise SchedulingConfigError(f"scheduling: no entry for node {node!r}")

    def chrt_prefix(self, node: str) -> str:
        """`chrt -f <priority>` for a launch `prefix`."""
        policy = self.safety_policy if node == "safety_node" else self.planner_policy
        return f"chrt {POLICIES[policy]} {self.priority(node)}"


def _int(value: Any, what: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise SchedulingConfigError(f"scheduling: {what} must be an integer, got {value!r}")
    return value


def _priority(value: Any, what: str) -> int:
    n = _int(value, what)
    if not PRIORITY_MIN <= n <= PRIORITY_MAX:
        raise SchedulingConfigError(
            f"scheduling: {what} must be {PRIORITY_MIN} to {PRIORITY_MAX}, got {n}"
        )
    return n


def _key(mapping: Any, key: str, where: str) -> Any:
    if not isinstance(mapping, dict) or key not in mapping:
        raise SchedulingConfigError(f"scheduling: missing key {where}{key}")
    return mapping[key]


def _node(rt: Any, name: str) -> tuple[str, int]:
    entry = _key(rt, name, "rt.")
    policy = _key(entry, "policy", f"rt.{name}.")
    if policy not in POLICIES:
        raise SchedulingConfigError(
            f"scheduling: rt.{name}.policy must be one of {sorted(POLICIES)}, got {policy!r}"
        )
    return policy, _priority(_key(entry, "priority", f"rt.{name}."), f"rt.{name}.priority")


def parse(doc: Any) -> Scheduling:
    version = _key(doc, "schema_version", "")
    if version != SCHEMA_VERSION:
        raise SchedulingConfigError(
            f"scheduling: schema_version must be {SCHEMA_VERSION}, got {version!r}"
        )
    rt = _key(doc, "rt", "")
    ulimit = _int(_key(rt, "rtprio_ulimit", "rt."), "rt.rtprio_ulimit")
    safety_policy, safety = _node(rt, "safety_node")
    planner_policy, planner = _node(rt, "mppi_planner_node")
    if safety <= planner:
        raise SchedulingConfigError(
            f"scheduling: safety_node priority ({safety}) must be above the planner's ({planner})"
        )
    if ulimit < max(safety, planner):
        raise SchedulingConfigError(
            f"scheduling: rt.rtprio_ulimit ({ulimit}) must be at least the highest priority "
            f"({max(safety, planner)})"
        )
    return Scheduling(ulimit, safety, planner, safety_policy, planner_policy)


def load(path: Path | str | None = None) -> Scheduling:
    target = Path(path) if path is not None else DEFAULT_PATH
    try:
        text = target.read_text()
    except OSError as exc:
        raise SchedulingConfigError(f"scheduling: cannot read {target}: {exc}") from exc
    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise SchedulingConfigError(f"scheduling: {target} is not valid YAML: {exc}") from exc
    return parse(doc)
