"""config/scheduling.py: the loader of config/scheduling.yaml (brief 52). A valid file, each
missing key, priorities out of range or not integers, the planner at or above safety, an ulimit
below the highest priority, an unknown policy: each refuses with its text."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
import yaml

_REPO = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location(
    "scheduling_under_test", _REPO / "config" / "scheduling.py"
)
sched = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sched)


def _doc() -> dict:
    return yaml.safe_load((_REPO / "config" / "scheduling.yaml").read_text())


def test_the_committed_file_loads():
    s = sched.load()
    assert s.safety_priority > s.planner_priority
    assert s.rtprio_ulimit >= s.safety_priority
    assert s.chrt_prefix("safety_node") == f"chrt -f {s.safety_priority}"
    assert s.chrt_prefix("mppi_planner_node") == f"chrt -f {s.planner_priority}"
    assert s.priority("safety_node") == s.safety_priority
    with pytest.raises(sched.SchedulingConfigError, match="no entry for node"):
        s.priority("other_node")


def test_load_reads_a_given_path(tmp_path):
    doc = _doc()
    doc["rt"]["rtprio_ulimit"] = 80
    path = tmp_path / "s.yaml"
    path.write_text(yaml.safe_dump(doc))
    assert sched.load(path).rtprio_ulimit == 80


def _mutate(fn):
    doc = _doc()
    fn(doc)
    return doc


def _pop(*keys):
    def fn(doc):
        d = doc
        for k in keys[:-1]:
            d = d[k]
        del d[keys[-1]]

    return fn


def _set(value, *keys):
    def fn(doc):
        d = doc
        for k in keys[:-1]:
            d = d[k]
        d[keys[-1]] = value

    return fn


CASES = [
    (_pop("schema_version"), "missing key schema_version"),
    (_set(2, "schema_version"), "schema_version must be 1"),
    (_pop("rt"), "missing key rt"),
    (_pop("rt", "rtprio_ulimit"), "missing key rt.rtprio_ulimit"),
    (_pop("rt", "safety_node"), "missing key rt.safety_node"),
    (_pop("rt", "mppi_planner_node"), "missing key rt.mppi_planner_node"),
    (_pop("rt", "safety_node", "policy"), "missing key rt.safety_node.policy"),
    (_pop("rt", "safety_node", "priority"), "missing key rt.safety_node.priority"),
    (_pop("rt", "mppi_planner_node", "priority"), "missing key rt.mppi_planner_node.priority"),
    (_set(0, "rt", "safety_node", "priority"), "must be 1 to 90, got 0"),
    (_set(91, "rt", "safety_node", "priority"), "must be 1 to 90, got 91"),
    (_set(0, "rt", "mppi_planner_node", "priority"), "must be 1 to 90, got 0"),
    (_set("50", "rt", "safety_node", "priority"), "must be an integer"),
    (_set(40.5, "rt", "mppi_planner_node", "priority"), "must be an integer"),
    (_set(True, "rt", "safety_node", "priority"), "must be an integer"),
    (_set("50", "rt", "rtprio_ulimit"), "must be an integer"),
    (_set(40, "rt", "safety_node", "priority"), "must be above the planner"),
    (_set(60, "rt", "mppi_planner_node", "priority"), "must be above the planner"),
    (_set(49, "rt", "rtprio_ulimit"), "must be at least the highest priority"),
    (_set("rr", "rt", "safety_node", "policy"), "policy must be one of"),
    (_set("other", "rt", "mppi_planner_node", "policy"), "policy must be one of"),
]


@pytest.mark.parametrize(("fn", "text"), CASES)
def test_a_bad_file_is_refused_with_its_text(fn, text):
    with pytest.raises(sched.SchedulingConfigError, match=text):
        sched.parse(_mutate(fn))


def test_a_non_mapping_and_unreadable_files_are_refused(tmp_path):
    with pytest.raises(sched.SchedulingConfigError, match="missing key"):
        sched.parse(["a list"])
    with pytest.raises(sched.SchedulingConfigError, match="cannot read"):
        sched.load(tmp_path / "nope.yaml")
    bad = tmp_path / "bad.yaml"
    bad.write_text("a: [unclosed")
    with pytest.raises(sched.SchedulingConfigError, match="not valid YAML"):
        sched.load(bad)


def test_the_refusal_prefix_is_the_registered_text():
    assert sched.REFUSAL_PREFIX == (
        "refusing to start: real-time scheduling was asked for and not granted:"
    )
