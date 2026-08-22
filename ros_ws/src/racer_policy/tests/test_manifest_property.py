"""Property test (claude-docs/12-testing.md L2, hypothesis): any single-field corruption of
a valid manifest is refused.

For every object node in a fully-valid `contract.yaml` (the root, and every nested section),
removing any one of that node's keys, or adding one unexpected key to it, must make
`load_contract` raise a `ContractError` -- never load a `Contract`, never warn-and-continue
(CLAUDE.md invariant 3). This complements the fixed-example tests in test_load_contract.py
by covering corruption sites those examples do not enumerate individually.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest
import yaml
from conftest import DEFAULT_POLICY_BYTES, fresh_valid_manifest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st
from racer_policy.contract import load_contract
from racer_policy.errors import ContractError

# Every dict-valued node in the manifest, addressed by the path of keys to reach it. Chosen
# to cover every nesting level in contract.schema.json, all of which set
# additionalProperties: false and list every key as required.
OBJECT_PATHS: list[tuple[str, ...]] = [
    (),
    ("policy",),
    ("observation_schema",),
    ("observation_schema", "lidar"),
    ("normalization",),
    ("action_space",),
    ("action_space", "residual_limits"),
    ("actuator_assumptions",),
    ("vehicle_params",),
    ("training",),
]


def _get_node(manifest: dict[str, Any], path: tuple[str, ...]) -> dict[str, Any]:
    node = manifest
    for key in path:
        node = node[key]
    return node


def _write_manifest_dir(directory: Path, manifest: dict[str, Any]) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "policy.pt").write_bytes(DEFAULT_POLICY_BYTES)
    (directory / "contract.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8"
    )
    return directory


@given(path=st.sampled_from(OBJECT_PATHS), data=st.data())
@settings(suppress_health_check=[HealthCheck.function_scoped_fixture])
def test_removing_any_single_field_from_any_object_is_refused(
    tmp_path_factory: pytest.TempPathFactory, path: tuple[str, ...], data: st.DataObject
) -> None:
    # A fresh manifest and a fresh directory PER EXAMPLE: `tmp_path` and the `valid_manifest`
    # fixture are computed once per test node, not once per hypothesis example, so reusing
    # either across examples here would corrupt one example's mutation into the next.
    manifest = fresh_valid_manifest()
    node = _get_node(manifest, path)
    key = data.draw(st.sampled_from(sorted(node.keys())))
    del node[key]

    directory = _write_manifest_dir(tmp_path_factory.mktemp("removed"), manifest)

    with pytest.raises(ContractError):
        load_contract(directory)


@given(path=st.sampled_from(OBJECT_PATHS))
@settings(suppress_health_check=[HealthCheck.function_scoped_fixture])
def test_adding_an_unexpected_field_to_any_object_is_refused(
    tmp_path_factory: pytest.TempPathFactory, path: tuple[str, ...]
) -> None:
    manifest = fresh_valid_manifest()
    node = _get_node(manifest, path)
    node["totally_unexpected_field_xyz"] = "corruption"

    directory = _write_manifest_dir(tmp_path_factory.mktemp("added"), manifest)

    with pytest.raises(ContractError):
        load_contract(directory)


@given(
    path=st.sampled_from(OBJECT_PATHS),
    replacement=st.one_of(st.integers(), st.booleans(), st.lists(st.integers(), max_size=3)),
    data=st.data(),
)
@settings(suppress_health_check=[HealthCheck.function_scoped_fixture])
def test_replacing_any_single_leaf_scalar_with_a_wrong_type_is_refused(
    tmp_path_factory: pytest.TempPathFactory,
    path: tuple[str, ...],
    replacement: Any,
    data: st.DataObject,
) -> None:
    manifest = fresh_valid_manifest()
    node = _get_node(manifest, path)
    scalar_keys = sorted(k for k, v in node.items() if isinstance(v, (str, int, float)))
    if not scalar_keys:
        return  # this object node has no scalar leaf to corrupt (e.g. holds only sub-objects)
    key = data.draw(st.sampled_from(scalar_keys))
    if node[key] == replacement:
        return  # not actually a corruption
    node[key] = replacement

    directory = _write_manifest_dir(tmp_path_factory.mktemp("wrong_type"), manifest)

    with pytest.raises(ContractError):
        load_contract(directory)
