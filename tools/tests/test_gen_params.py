"""Tests for tools/gen_params.py (claude-docs/12-testing.md L1/L2).

- L1: cross-language round-trip equality (compiles the generated C++ and C headers with a
  schema-driven JSON-dumping harness and diffs against the generated Python module) and one
  refusal test per schema-violation class.
- L2 (hypothesis): valid configs generated from the schema always validate and round-trip
  through yaml dump/load; invalid configs (missing/extra/wrong-type/out-of-range) are always
  rejected.
"""

from __future__ import annotations

import copy
import dataclasses
import datetime
import importlib.util
import json
import platform
import shutil
import subprocess
import sys
from pathlib import Path

import gen_params
import pytest
import yaml
from hypothesis import given, settings
from hypothesis import strategies as st

REPO_ROOT = Path(__file__).resolve().parents[2]
SCHEMA_PATH = REPO_ROOT / "config" / "vehicle_params.schema.json"
PARAMS_PATH = REPO_ROOT / "config" / "vehicle_params.yaml"
FIXTURE_PATH = Path(__file__).resolve().parent / "fixtures" / "full_fixture.yaml"


# --------------------------------------------------------------------------------------
# Shared fixtures
# --------------------------------------------------------------------------------------


@pytest.fixture(scope="module")
def schema() -> dict:
    return gen_params.load_schema_file(SCHEMA_PATH)


@pytest.fixture(scope="module")
def repo_params() -> dict:
    return gen_params.load_yaml_file(PARAMS_PATH)


@pytest.fixture(scope="module")
def full_params() -> dict:
    return gen_params.load_yaml_file(FIXTURE_PATH)


# --------------------------------------------------------------------------------------
# Loading / validation basics
# --------------------------------------------------------------------------------------


def test_repo_params_validates(repo_params, schema):
    gen_params.validate_params(repo_params, schema)  # must not raise


def test_full_fixture_validates(full_params, schema):
    gen_params.validate_params(full_params, schema)  # must not raise


def test_load_schema_file_reads_json(tmp_path):
    p = tmp_path / "s.json"
    p.write_text('{"type": "object"}')
    assert gen_params.load_schema_file(p) == {"type": "object"}


def test_load_yaml_file_reads_yaml(tmp_path):
    p = tmp_path / "p.yaml"
    p.write_text("a: 1\n")
    assert gen_params.load_yaml_file(p) == {"a": 1}


# --------------------------------------------------------------------------------------
# Validation refusal tests -- one per mismatch class (12-testing.md: "a test proving it
# refuses is as important as one proving it loads")
# --------------------------------------------------------------------------------------


def test_refuses_missing_required_field(full_params, schema):
    bad = copy.deepcopy(full_params)
    del bad["chassis"]["mass_kg"]
    with pytest.raises(gen_params.ParamsValidationError) as exc:
        gen_params.validate_params(bad, schema)
    assert "mass_kg" in str(exc.value)
    assert "required" in str(exc.value)


def test_refuses_wrong_type(full_params, schema):
    bad = copy.deepcopy(full_params)
    bad["chassis"]["mass_kg"] = "three point seven four"
    with pytest.raises(gen_params.ParamsValidationError) as exc:
        gen_params.validate_params(bad, schema)
    assert "mass_kg" in str(exc.value)


def test_refuses_extra_field(full_params, schema):
    bad = copy.deepcopy(full_params)
    bad["chassis"]["extra_undeclared_field"] = 1.0
    with pytest.raises(gen_params.ParamsValidationError) as exc:
        gen_params.validate_params(bad, schema)
    assert "additionalProperties" in str(exc.value) or "extra_undeclared_field" in str(exc.value)


def test_refuses_out_of_range(full_params, schema):
    bad = copy.deepcopy(full_params)
    bad["chassis"]["mass_kg"] = 10_000.0  # schema maximum is 100
    with pytest.raises(gen_params.ParamsValidationError) as exc:
        gen_params.validate_params(bad, schema)
    assert "mass_kg" in str(exc.value)


def test_refuses_missing_top_level_section(full_params, schema):
    bad = copy.deepcopy(full_params)
    del bad["meta"]
    with pytest.raises(gen_params.ParamsValidationError):
        gen_params.validate_params(bad, schema)


def test_refuses_bad_semver(full_params, schema):
    bad = copy.deepcopy(full_params)
    bad["meta"]["schema_version"] = "not-a-semver"
    with pytest.raises(gen_params.ParamsValidationError):
        gen_params.validate_params(bad, schema)


def test_refuses_bad_date_format(full_params, schema):
    bad = copy.deepcopy(full_params)
    bad["meta"]["fit_date"] = "22-08-2026"  # not ISO 8601
    with pytest.raises(gen_params.ParamsValidationError):
        gen_params.validate_params(bad, schema)


def test_accepts_null_for_nullable_field(full_params, schema):
    ok = copy.deepcopy(full_params)
    ok["tires"]["pacejka_front"] = None
    gen_params.validate_params(ok, schema)  # must not raise


def test_rejects_null_for_non_nullable_field(full_params, schema):
    bad = copy.deepcopy(full_params)
    bad["chassis"]["mass_kg"] = None
    with pytest.raises(gen_params.ParamsValidationError):
        gen_params.validate_params(bad, schema)


# --------------------------------------------------------------------------------------
# CLI refusal / success tests
# --------------------------------------------------------------------------------------


def test_cli_success_writes_three_files(tmp_path, full_params, schema):
    params_file = tmp_path / "p.yaml"
    schema_file = tmp_path / "s.json"
    params_file.write_text(yaml.safe_dump(full_params))
    schema_file.write_text(json.dumps(schema))
    out_dir = tmp_path / "out"

    rc = gen_params.main(
        ["--params", str(params_file), "--schema", str(schema_file), "--out-dir", str(out_dir)]
    )
    assert rc == 0
    assert (out_dir / "vehicle_params_generated.py").exists()
    assert (out_dir / "vehicle_params_generated.hpp").exists()
    assert (out_dir / "vehicle_params_generated.h").exists()
    for f in out_dir.iterdir():
        assert "GENERATED FILE" in f.read_text()


def test_cli_refuses_on_validation_failure(tmp_path, full_params, schema, capsys):
    bad = copy.deepcopy(full_params)
    del bad["chassis"]["mass_kg"]
    params_file = tmp_path / "p.yaml"
    schema_file = tmp_path / "s.json"
    params_file.write_text(yaml.safe_dump(bad))
    schema_file.write_text(json.dumps(schema))

    rc = gen_params.main(
        [
            "--params",
            str(params_file),
            "--schema",
            str(schema_file),
            "--out-dir",
            str(tmp_path / "out"),
        ]
    )
    assert rc != 0
    err = capsys.readouterr().err
    assert "failed schema validation" in err
    assert "mass_kg" in err
    assert not (tmp_path / "out").exists()


def test_cli_refuses_on_missing_file(tmp_path, capsys):
    rc = gen_params.main(
        [
            "--params",
            str(tmp_path / "nope.yaml"),
            "--schema",
            str(SCHEMA_PATH),
            "--out-dir",
            str(tmp_path / "out"),
        ]
    )
    assert rc != 0
    assert "could not read" in capsys.readouterr().err


def test_cli_refuses_on_malformed_yaml(tmp_path, schema, capsys):
    params_file = tmp_path / "bad.yaml"
    params_file.write_text("chassis: [unterminated\n")
    schema_file = tmp_path / "s.json"
    schema_file.write_text(json.dumps(schema))
    rc = gen_params.main(
        [
            "--params",
            str(params_file),
            "--schema",
            str(schema_file),
            "--out-dir",
            str(tmp_path / "out"),
        ]
    )
    assert rc != 0
    assert "could not parse" in capsys.readouterr().err


def test_cli_refuses_on_malformed_schema_json(tmp_path, full_params, capsys):
    params_file = tmp_path / "p.yaml"
    params_file.write_text(yaml.safe_dump(full_params))
    schema_file = tmp_path / "bad.json"
    schema_file.write_text("{not json")
    rc = gen_params.main(
        [
            "--params",
            str(params_file),
            "--schema",
            str(schema_file),
            "--out-dir",
            str(tmp_path / "out"),
        ]
    )
    assert rc != 0
    assert "could not parse" in capsys.readouterr().err


def test_cli_refuses_on_unsupported_schema_shape(tmp_path, capsys):
    # Valid per jsonschema, but uses a leaf type (boolean) the generator does not support --
    # exercises generate_all's ValueError-catching branch in main(), distinct from a
    # validate_params failure.
    schema = {
        "type": "object",
        "additionalProperties": False,
        "required": ["flag"],
        "properties": {"flag": {"type": "boolean", "units": "n/a"}},
    }
    params_file = tmp_path / "p.yaml"
    schema_file = tmp_path / "s.json"
    params_file.write_text(yaml.safe_dump({"flag": True}))
    schema_file.write_text(json.dumps(schema))
    rc = gen_params.main(
        [
            "--params",
            str(params_file),
            "--schema",
            str(schema_file),
            "--out-dir",
            str(tmp_path / "out"),
        ]
    )
    assert rc != 0
    assert "could not generate bindings" in capsys.readouterr().err


# --------------------------------------------------------------------------------------
# build_registry structural tests
# --------------------------------------------------------------------------------------


def test_registry_shares_ref_types(schema):
    registry, _ = gen_params.build_registry(schema)
    tires = registry["Tires"]
    front = next(f for f in tires.fields if f.name == "pacejka_front")
    rear = next(f for f in tires.fields if f.name == "pacejka_rear")
    assert front.ref_type == rear.ref_type == "PacejkaParams"
    sensors = registry["Sensors"]
    imu = next(f for f in sensors.fields if f.name == "imu")
    lidar = next(f for f in sensors.fields if f.name == "lidar")
    assert imu.ref_type == lidar.ref_type == "SensorExtrinsic"


def test_registry_is_dependency_ordered(schema):
    registry, _ = gen_params.build_registry(schema)
    names = list(registry.keys())
    assert names.index("PacejkaParams") < names.index("Tires")
    assert names.index("SensorExtrinsic") < names.index("Sensors")
    assert names[-1] == "VehicleParams"


def test_registry_top_level_order_matches_schema(schema):
    _, top_order = gen_params.build_registry(schema)
    assert top_order == [
        "chassis",
        "tires",
        "drivetrain",
        "steering",
        "actuation",
        "sensors",
        "limits",
        "meta",
    ]


def test_build_registry_rejects_inconsistent_required():
    bad_schema = {
        "type": "object",
        "required": ["a"],
        "properties": {
            "a": {"type": "number", "units": "m"},
            "b": {"type": "number", "units": "m"},
        },
    }
    with pytest.raises(ValueError, match="required"):
        gen_params.build_registry(bad_schema)


def test_build_registry_rejects_root_required_mismatch():
    bad_schema = {"type": "object", "required": ["a"], "properties": {}}
    with pytest.raises(ValueError, match="root schema"):
        gen_params.build_registry(bad_schema)


def test_build_registry_rejects_inconsistent_required_in_nested_object():
    bad_schema = {
        "type": "object",
        "required": ["outer"],
        "properties": {
            "outer": {
                "type": "object",
                "units": "n/a",
                "required": ["a"],
                "properties": {
                    "a": {"type": "number", "units": "m"},
                    "b": {"type": "number", "units": "m"},
                },
            }
        },
    }
    with pytest.raises(ValueError, match="schema object must list every property"):
        gen_params.build_registry(bad_schema)


def test_c_header_supports_nullable_string_field():
    # config/vehicle_params.schema.json has no nullable string fields today; this exercises
    # the generic (is_set-sidecar) codepath in _c_field_decls / _c_render_designated so a
    # future nullable string field is not an untested surprise.
    small_schema = {
        "type": "object",
        "required": ["label"],
        "properties": {"label": {"type": ["string", "null"], "units": "n/a", "maxLength": 8}},
    }
    present = gen_params.generate_c_header({"label": "hi"}, small_schema)
    assert "label_is_set" in present
    assert '.label = "hi"' in present
    assert ".label_is_set = true" in present

    absent = gen_params.generate_c_header({"label": None}, small_schema)
    assert ".label_is_set = false" in absent


def test_types_of_rejects_ambiguous_union():
    with pytest.raises(ValueError):
        gen_params._types_of({"type": ["number", "string"]})


def test_build_field_rejects_nullable_array_items():
    bad_schema = {
        "type": "object",
        "required": ["arr"],
        "properties": {
            "arr": {
                "type": "array",
                "units": "n/a",
                "items": {"type": ["object", "null"], "required": [], "properties": {}},
            }
        },
    }
    with pytest.raises(ValueError, match="individually nullable"):
        gen_params.build_registry(bad_schema)


def test_build_field_rejects_non_object_array_items():
    bad_schema = {
        "type": "object",
        "required": ["arr"],
        "properties": {"arr": {"type": "array", "units": "n/a", "items": {"type": "number"}}},
    }
    with pytest.raises(ValueError, match="only arrays of objects"):
        gen_params.build_registry(bad_schema)


def test_build_field_rejects_unsupported_kind():
    bad_schema = {
        "type": "object",
        "required": ["flag"],
        "properties": {"flag": {"type": "boolean", "units": "n/a"}},
    }
    with pytest.raises(ValueError, match="unsupported schema kind"):
        gen_params.build_registry(bad_schema)


# --------------------------------------------------------------------------------------
# Generator-level defensive checks (independent of validate_params, exercised directly)
# --------------------------------------------------------------------------------------


def test_py_render_value_rejects_null_for_non_nullable(schema):
    registry, _ = gen_params.build_registry(schema)
    shape = next(f for f in registry["Chassis"].fields if f.name == "mass_kg")
    with pytest.raises(gen_params.ParamsValidationError):
        gen_params._py_render_value(shape, None, registry)


def test_cpp_render_value_rejects_null_for_non_nullable(schema):
    registry, _ = gen_params.build_registry(schema)
    shape = next(f for f in registry["Chassis"].fields if f.name == "mass_kg")
    with pytest.raises(gen_params.ParamsValidationError):
        gen_params._cpp_render_value(shape, None, registry)


def test_c_header_rejects_string_over_max_length(full_params, schema):
    bad = copy.deepcopy(full_params)
    bad["tires"]["compound_id"] = "x" * 1000
    with pytest.raises(gen_params.ParamsValidationError, match="maxLength"):
        gen_params.generate_c_header(bad, schema)


def test_c_header_rejects_array_over_max_items(full_params, schema):
    bad = copy.deepcopy(full_params)
    bad["steering"]["pwm_to_angle_table"] = [
        {"pwm_us": 1500.0, "angle_rad": 0.0} for _ in range(1000)
    ]
    with pytest.raises(gen_params.ParamsValidationError, match="maxItems"):
        gen_params.generate_c_header(bad, schema)


def test_pascal_case():
    assert gen_params._pascal("pacejka_params") == "PacejkaParams"
    assert gen_params._pascal("cg_to_front_axle_m") == "CgToFrontAxleM"


def test_generate_python_contains_banner_and_dataclasses(full_params, schema):
    src = gen_params.generate_python(full_params, schema)
    assert "GENERATED FILE" in src
    assert "class VehicleParams" in src
    assert "VEHICLE_PARAMS: VehicleParams = VehicleParams(" in src


def test_generate_cpp_header_contains_optional_and_guard(full_params, schema):
    src = gen_params.generate_cpp_header(full_params, schema)
    assert "RACER_VEHICLE_PARAMS_GENERATED_HPP_" in src
    assert "std::optional<double> track_width_m" in src
    assert "inline const VehicleParams VEHICLE_PARAMS" in src


def test_generate_c_header_contains_is_set_and_guard(full_params, schema):
    src = gen_params.generate_c_header(full_params, schema)
    assert "RACER_VEHICLE_PARAMS_GENERATED_H_" in src
    assert "track_width_m_is_set" in src
    assert "static const VehicleParams VEHICLE_PARAMS" in src


def test_generate_all_returns_three_named_artifacts(full_params, schema):
    artifacts = gen_params.generate_all(full_params, schema)
    assert set(artifacts) == {
        "vehicle_params_generated.py",
        "vehicle_params_generated.hpp",
        "vehicle_params_generated.h",
    }


# --------------------------------------------------------------------------------------
# L1: cross-language round-trip equality (the headline test from 12-testing.md)
# --------------------------------------------------------------------------------------


def _pick(candidates: list[str]) -> str:
    for name in candidates:
        path = shutil.which(name)
        if path:
            return path
    pytest.skip(f"none of {candidates} found on PATH")


def _cpp_compiler() -> str:
    order = ["clang++", "g++"] if platform.system() == "Darwin" else ["g++", "clang++"]
    return _pick(order)


def _c_compiler() -> str:
    order = ["clang", "gcc"] if platform.system() == "Darwin" else ["gcc", "clang"]
    return _pick(order)


def _generate_and_compare(tmp_path: Path, params: dict, schema: dict) -> None:
    gen_params.validate_params(params, schema)

    (tmp_path / "vehicle_params_generated.py").write_text(
        gen_params.generate_python(params, schema)
    )
    (tmp_path / "vehicle_params_generated.hpp").write_text(
        gen_params.generate_cpp_header(params, schema)
    )
    (tmp_path / "vehicle_params_generated.h").write_text(
        gen_params.generate_c_header(params, schema)
    )
    (tmp_path / "harness.cpp").write_text(gen_params.generate_cpp_harness(schema))
    (tmp_path / "harness.c").write_text(gen_params.generate_c_harness(schema))

    cpp_bin = tmp_path / "harness_cpp"
    c_bin = tmp_path / "harness_c"

    subprocess.run(
        [
            _cpp_compiler(),
            "-std=c++17",
            "-Wall",
            "-I",
            str(tmp_path),
            str(tmp_path / "harness.cpp"),
            "-o",
            str(cpp_bin),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [
            _c_compiler(),
            "-std=c11",
            "-Wall",
            "-I",
            str(tmp_path),
            str(tmp_path / "harness.c"),
            "-o",
            str(c_bin),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    cpp_out = subprocess.run([str(cpp_bin)], check=True, capture_output=True, text=True).stdout
    c_out = subprocess.run([str(c_bin)], check=True, capture_output=True, text=True).stdout

    cpp_json = json.loads(cpp_out)
    c_json = json.loads(c_out)

    spec = importlib.util.spec_from_file_location(
        "vehicle_params_generated", tmp_path / "vehicle_params_generated.py"
    )
    module = importlib.util.module_from_spec(spec)
    # dataclasses resolves `from __future__ import annotations` string annotations via
    # sys.modules[cls.__module__], so the module must be registered before exec_module runs.
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)  # type: ignore[union-attr]
    finally:
        del sys.modules[spec.name]
    py_json = json.loads(json.dumps(dataclasses.asdict(module.VEHICLE_PARAMS)))

    assert py_json == cpp_json, "Python and C++ bindings disagree on at least one field"
    assert py_json == c_json, "Python and C bindings disagree on at least one field"
    assert cpp_json == c_json, "C++ and C bindings disagree on at least one field"


def test_round_trip_full_fixture(tmp_path, full_params, schema):
    """The literal ask: generate all three bindings from a fixture params file, compile the
    C++/C headers in a tiny harness, and assert all three agree on every field. This fixture
    populates every nullable field so every codegen path (nested objects, arrays, nullable
    scalars) is exercised, not just the null branches."""
    _generate_and_compare(tmp_path, full_params, schema)


def test_round_trip_repo_vehicle_params(tmp_path, repo_params, schema):
    """The actual committed config/vehicle_params.yaml must also compile and round-trip,
    nulls and all."""
    _generate_and_compare(tmp_path, repo_params, schema)


# --------------------------------------------------------------------------------------
# L2: property-based tests (hypothesis) -- claude-docs/12-testing.md
# --------------------------------------------------------------------------------------


def _schema_leaf_strategy(node: dict, root: dict):
    resolved, _ = gen_params._resolve_ref(node, root)
    kind, nullable = gen_params._types_of(resolved)

    if "pattern" in resolved:
        strat = st.from_regex(resolved["pattern"], fullmatch=True)
    elif resolved.get("format") == "date":
        strat = st.dates(
            min_value=datetime.date(2000, 1, 1), max_value=datetime.date(2100, 1, 1)
        ).map(lambda d: d.isoformat())
    elif kind == "number":
        lo = resolved.get("minimum", resolved.get("exclusiveMinimum", -1e6))
        hi = resolved.get("maximum", resolved.get("exclusiveMaximum", 1e6))
        if "exclusiveMinimum" in resolved:
            lo = resolved["exclusiveMinimum"] + 1e-3
        if "exclusiveMaximum" in resolved:
            hi = resolved["exclusiveMaximum"] - 1e-3
        strat = st.floats(
            min_value=lo, max_value=hi, allow_nan=False, allow_infinity=False, width=64
        )
    elif kind == "integer":
        lo = int(resolved.get("minimum", 0))
        hi = int(resolved.get("maximum", 1000))
        strat = st.integers(min_value=lo, max_value=hi)
    elif kind == "string":
        max_len = resolved.get("maxLength", 63)
        strat = st.text(
            alphabet=st.characters(min_codepoint=97, max_codepoint=122),
            min_size=1,
            max_size=max_len,
        )
    elif kind == "object":
        props = resolved["properties"]
        required = resolved["required"]
        strat = st.fixed_dictionaries(
            {name: _schema_leaf_strategy(props[name], root) for name in required}
        )
    elif kind == "array":
        item_strat = _schema_leaf_strategy(resolved["items"], root)
        max_items = min(resolved.get("maxItems", 32), 3)
        strat = st.lists(item_strat, max_size=max_items)
    else:  # pragma: no cover - schema has no other kinds
        raise AssertionError(kind)

    return st.one_of(st.none(), strat) if nullable else strat


@pytest.fixture(scope="module")
def valid_params_strategy(schema):
    return _schema_leaf_strategy(schema, schema)


@given(data=st.data())
@settings(max_examples=25, deadline=None)
def test_hypothesis_valid_configs_validate_and_round_trip(data, schema, valid_params_strategy):
    params = data.draw(valid_params_strategy)
    gen_params.validate_params(params, schema)  # must not raise

    dumped = yaml.safe_dump(params)
    reloaded = yaml.safe_load(dumped)
    gen_params.validate_params(reloaded, schema)  # still valid after a yaml round trip
    assert reloaded == params

    # also exercise codegen end to end on every generated valid config (no compile, just
    # that the generator itself never raises on schema-valid input)
    gen_params.generate_all(params, schema)


@given(
    data=st.data(),
    corruption=st.sampled_from(
        ["missing_required", "wrong_type", "extra_property", "out_of_range"]
    ),
)
@settings(max_examples=25, deadline=None)
def test_hypothesis_invalid_configs_always_rejected(
    data, corruption, schema, valid_params_strategy
):
    params = data.draw(valid_params_strategy)
    mutated = copy.deepcopy(params)

    if corruption == "missing_required":
        del mutated["chassis"]["mass_kg"]
    elif corruption == "wrong_type":
        mutated["chassis"]["mass_kg"] = "not-a-number"
    elif corruption == "extra_property":
        mutated["chassis"]["extra_undeclared_field"] = 1.0
    elif corruption == "out_of_range":
        mutated["chassis"]["mass_kg"] = 10_000.0  # schema maximum is 100

    with pytest.raises(gen_params.ParamsValidationError):
        gen_params.validate_params(mutated, schema)
