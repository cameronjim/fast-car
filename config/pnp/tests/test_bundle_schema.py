"""Tests for config/pnp/bundle.schema.json against the example manifests."""

from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest
import yaml
from hypothesis import given
from hypothesis import strategies as st
from jsonschema import Draft202012Validator

PNP_DIR = Path(__file__).resolve().parent.parent
SCHEMA = json.loads((PNP_DIR / "bundle.schema.json").read_text())
VALIDATOR = Draft202012Validator(SCHEMA)
EXAMPLES = {
    "track_perception": PNP_DIR / "examples" / "trackperc-example" / "manifest.yaml",
    "dynamics": PNP_DIR / "examples" / "dyn-example" / "manifest.yaml",
}
FIXTURE_MANIFESTS = sorted((PNP_DIR / "fixtures").glob("*/manifest.yaml"))
TOP_LEVEL_KEYS = [
    "bundle_schema_version",
    "kind",
    "model_id",
    "created_utc",
    "git_sha",
    "training_config_sha256",
    "vehicle_params",
    "payload",
    "io",
    "data",
    "metrics",
]


def load(kind: str) -> dict:
    return yaml.safe_load(EXAMPLES[kind].read_text())


def is_valid(manifest: dict) -> bool:
    return VALIDATOR.is_valid(manifest)


def test_schema_is_a_valid_schema() -> None:
    Draft202012Validator.check_schema(SCHEMA)


@pytest.mark.parametrize("kind", sorted(EXAMPLES))
def test_example_validates(kind: str) -> None:
    VALIDATOR.validate(load(kind))


@pytest.mark.parametrize("path", FIXTURE_MANIFESTS, ids=lambda p: p.parent.name)
def test_committed_fixtures_validate(path: Path) -> None:
    VALIDATOR.validate(yaml.safe_load(path.read_text()))


@pytest.mark.parametrize("kind", sorted(EXAMPLES))
@pytest.mark.parametrize("key", TOP_LEVEL_KEYS)
def test_missing_top_level_field_is_rejected(kind: str, key: str) -> None:
    manifest = load(kind)
    del manifest[key]
    assert not is_valid(manifest)


@pytest.mark.parametrize("kind", sorted(EXAMPLES))
def test_extra_top_level_field_is_rejected(kind: str) -> None:
    manifest = load(kind)
    manifest["surprise"] = 1
    assert not is_valid(manifest)


@pytest.mark.parametrize("kind", sorted(EXAMPLES))
def test_extra_vehicle_params_field_is_rejected(kind: str) -> None:
    manifest = load(kind)
    manifest["vehicle_params"]["extra"] = "x"
    assert not is_valid(manifest)


@pytest.mark.parametrize("missing", ["schema_version", "sysid_session_id"])
def test_missing_vehicle_params_field_is_rejected(missing: str) -> None:
    manifest = load("dynamics")
    del manifest["vehicle_params"][missing]
    assert not is_valid(manifest)


def test_extra_payload_item_field_is_rejected() -> None:
    manifest = load("dynamics")
    manifest["payload"][0]["mode"] = "rw"
    assert not is_valid(manifest)


@pytest.mark.parametrize("missing", ["file", "sha256"])
def test_missing_payload_item_field_is_rejected(missing: str) -> None:
    manifest = load("dynamics")
    del manifest["payload"][0][missing]
    assert not is_valid(manifest)


def test_wrong_kind_is_rejected() -> None:
    manifest = load("dynamics")
    manifest["kind"] = "teleport"
    assert not is_valid(manifest)


def test_kind_dynamics_with_perception_io_is_rejected() -> None:
    manifest = load("dynamics")
    manifest["io"] = load("track_perception")["io"]
    assert not is_valid(manifest)


def test_kind_perception_with_dynamics_io_is_rejected() -> None:
    manifest = load("track_perception")
    manifest["io"] = load("dynamics")["io"]
    assert not is_valid(manifest)


@pytest.mark.parametrize("kind", ["adaptation", "lap_memory"])
def test_other_kinds_accept_any_io_object(kind: str) -> None:
    manifest = load("dynamics")
    manifest["kind"] = kind
    manifest["io"] = {"anything": [1, 2, 3]}
    assert is_valid(manifest)


@pytest.mark.parametrize("kind", ["adaptation", "lap_memory"])
def test_other_kinds_reject_non_object_io(kind: str) -> None:
    manifest = load("dynamics")
    manifest["kind"] = kind
    manifest["io"] = []
    assert not is_valid(manifest)


@pytest.mark.parametrize("key", ["git_sha", "training_config_sha256"])
@pytest.mark.parametrize(
    "bad",
    ["", "xyz", "0123", "G" * 40, "A" * 40, "0" * 39, "0" * 41, "0" * 63, "0" * 65, 12345],
)
def test_bad_top_level_hash_is_rejected(key: str, bad: object) -> None:
    manifest = load("dynamics")
    manifest[key] = bad
    assert not is_valid(manifest)


@pytest.mark.parametrize("bad", ["", "abc", "g" * 64, "A" * 64, "0" * 63, "0" * 65])
def test_bad_payload_hash_is_rejected(bad: str) -> None:
    manifest = load("dynamics")
    manifest["payload"][0]["sha256"] = bad
    assert not is_valid(manifest)


@pytest.mark.parametrize("bad", ["", "Dyn-v0", "-dyn", ".dyn", "dyn_v0", "dyn v0", 7])
def test_bad_model_id_is_rejected(bad: object) -> None:
    manifest = load("dynamics")
    manifest["model_id"] = bad
    assert not is_valid(manifest)


@pytest.mark.parametrize("good", ["a", "0abc", "dyn-v1-2026-11-12-x.y"])
def test_good_model_id_is_accepted(good: str) -> None:
    manifest = load("dynamics")
    manifest["model_id"] = good
    assert is_valid(manifest)


@pytest.mark.parametrize("bad", ["1", "1.0", "1.0.0.0", "v1.0.0", "1.0.x", "01.0.0", ""])
def test_bad_bundle_schema_version_is_rejected(bad: str) -> None:
    manifest = load("dynamics")
    manifest["bundle_schema_version"] = bad
    assert not is_valid(manifest)


def test_bad_created_utc_is_rejected() -> None:
    manifest = load("dynamics")
    manifest["created_utc"] = "2026-11-12 03:00:00"
    assert not is_valid(manifest)


def test_wrong_tensor_shape_type_is_rejected_for_input() -> None:
    manifest = load("track_perception")
    manifest["io"]["input"]["tensor"] = "1x2x360"
    assert not is_valid(manifest)


def test_float_tensor_entries_are_rejected() -> None:
    manifest = load("track_perception")
    manifest["io"]["output"]["tensor"] = [16.5, 5]
    assert not is_valid(manifest)


@pytest.mark.parametrize("tensor", [[1, 2], [1, 2, 360, 1]])
def test_wrong_input_tensor_rank_is_rejected(tensor: list[int]) -> None:
    manifest = load("track_perception")
    manifest["io"]["input"]["tensor"] = tensor
    assert not is_valid(manifest)


def test_wrong_output_tensor_rank_is_rejected() -> None:
    manifest = load("track_perception")
    manifest["io"]["output"]["tensor"] = [16]
    assert not is_valid(manifest)


@pytest.mark.parametrize("section", ["input", "output", "lidar_mount", "arch"])
def test_missing_perception_io_section_is_rejected(section: str) -> None:
    manifest = load("track_perception")
    del manifest["io"][section]
    assert not is_valid(manifest)


def test_extra_perception_io_field_is_rejected() -> None:
    manifest = load("track_perception")
    manifest["io"]["extra"] = 1
    assert not is_valid(manifest)


def test_extra_perception_input_field_is_rejected() -> None:
    manifest = load("track_perception")
    manifest["io"]["input"]["extra"] = 1
    assert not is_valid(manifest)


@pytest.mark.parametrize("field", ["origin", "pooling"])
def test_wrong_perception_enum_is_rejected(field: str) -> None:
    manifest = load("track_perception")
    manifest["io"]["input"][field] = "other"
    assert not is_valid(manifest)


def test_wrong_output_frame_is_rejected() -> None:
    manifest = load("track_perception")
    manifest["io"]["output"]["frame"] = "map"
    assert not is_valid(manifest)


def test_missing_lidar_mount_field_is_rejected() -> None:
    manifest = load("track_perception")
    del manifest["io"]["lidar_mount"]["yaw_rad"]
    assert not is_valid(manifest)


def test_arch_requires_type() -> None:
    manifest = load("track_perception")
    manifest["io"]["arch"] = {}
    assert not is_valid(manifest)


@pytest.mark.parametrize(
    "key",
    [
        "speed_gain",
        "speed_tau_s",
        "accel_max_mps2",
        "decel_max_mps2",
        "steer_gain_left",
        "steer_gain_right",
        "steer_offset_rad",
        "steer_tau_s",
        "command_delay_s",
        "understeer_s2_per_m2",
    ],
)
def test_missing_dynamics_nominal_key_is_rejected(key: str) -> None:
    manifest = load("dynamics")
    del manifest["io"]["nominal"][key]
    assert not is_valid(manifest)


def test_extra_dynamics_nominal_key_is_rejected() -> None:
    manifest = load("dynamics")
    manifest["io"]["nominal"]["extra"] = 1.0
    assert not is_valid(manifest)


def test_non_numeric_nominal_value_is_rejected() -> None:
    manifest = load("dynamics")
    manifest["io"]["nominal"]["speed_tau_s"] = "fast"
    assert not is_valid(manifest)


@pytest.mark.parametrize(
    "section",
    ["state", "control", "dt_s", "nominal", "residual", "validity", "golden"],
)
def test_missing_dynamics_io_section_is_rejected(section: str) -> None:
    manifest = load("dynamics")
    del manifest["io"][section]
    assert not is_valid(manifest)


def test_extra_dynamics_io_field_is_rejected() -> None:
    manifest = load("dynamics")
    manifest["io"]["extra"] = 1
    assert not is_valid(manifest)


@pytest.mark.parametrize(
    "key",
    [
        "enabled",
        "inputs",
        "outputs",
        "layers",
        "activation",
        "input_mean",
        "input_std",
        "output_scale",
        "file",
    ],
)
def test_missing_residual_key_is_rejected(key: str) -> None:
    manifest = load("dynamics")
    del manifest["io"]["residual"][key]
    assert not is_valid(manifest)


def test_extra_residual_key_is_rejected() -> None:
    manifest = load("dynamics")
    manifest["io"]["residual"]["extra"] = 1
    assert not is_valid(manifest)


def test_residual_enabled_must_be_boolean() -> None:
    manifest = load("dynamics")
    manifest["io"]["residual"]["enabled"] = "false"
    assert not is_valid(manifest)


def test_residual_layers_wrong_type_is_rejected() -> None:
    manifest = load("dynamics")
    manifest["io"]["residual"]["layers"] = [4, 32]
    assert not is_valid(manifest)


def test_residual_layers_wrong_pair_length_is_rejected() -> None:
    manifest = load("dynamics")
    manifest["io"]["residual"]["layers"] = [[4, 32, 1]]
    assert not is_valid(manifest)


def test_validity_requires_both_bounds() -> None:
    manifest = load("dynamics")
    del manifest["io"]["validity"]["v_max_mps"]
    assert not is_valid(manifest)


def test_validity_max_must_be_positive() -> None:
    manifest = load("dynamics")
    manifest["io"]["validity"]["v_max_mps"] = 0.0
    assert not is_valid(manifest)


def test_extra_validity_key_is_rejected() -> None:
    manifest = load("dynamics")
    manifest["io"]["validity"]["extra"] = 1
    assert not is_valid(manifest)


def test_dt_must_be_positive_number() -> None:
    manifest = load("dynamics")
    manifest["io"]["dt_s"] = 0
    assert not is_valid(manifest)
    manifest["io"]["dt_s"] = "0.05"
    assert not is_valid(manifest)


def test_data_must_be_list_of_strings() -> None:
    manifest = load("dynamics")
    manifest["data"] = [1, 2]
    assert not is_valid(manifest)
    manifest["data"] = "bag"
    assert not is_valid(manifest)


def test_metrics_must_be_object() -> None:
    manifest = load("dynamics")
    manifest["metrics"] = [1]
    assert not is_valid(manifest)


def _required_paths(manifest: dict) -> list[tuple[str, ...]]:
    """Every key path the schema makes required for this manifest."""
    paths: list[tuple[str, ...]] = []

    def walk(node: object, prefix: tuple[str, ...]) -> None:
        if isinstance(node, dict):
            for key, value in node.items():
                paths.append(prefix + (key,))
                walk(value, prefix + (key,))

    walk(manifest, ())
    return paths


def _without(manifest: dict, path: tuple[str, ...]) -> dict:
    out = copy.deepcopy(manifest)
    node = out
    for key in path[:-1]:
        node = node[key]
    del node[path[-1]]
    return out


# `arch` may carry extra architecture keys and `metrics` is free-form, so only paths that the
# schema requires are checked: everything outside metrics and the arch sub-keys.
def _is_required(path: tuple[str, ...]) -> bool:
    if path[0] == "metrics" and len(path) > 1:
        return False
    return not (path[:2] == ("io", "arch") and len(path) > 2 and path[2] != "type")


@pytest.mark.parametrize("kind", sorted(EXAMPLES))
@given(data=st.data())
def test_deleting_any_single_required_key_is_rejected(kind: str, data: st.DataObject) -> None:
    manifest = load(kind)
    paths = [p for p in _required_paths(manifest) if _is_required(p)]
    path = data.draw(st.sampled_from(paths))
    assert not is_valid(_without(manifest, path))
