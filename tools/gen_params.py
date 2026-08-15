"""tools/gen_params.py — generated bindings for config/vehicle_params.yaml.

Reads config/vehicle_params.yaml, validates it against
config/vehicle_params.schema.json, and generates three artifacts that all bake in the exact
same concrete field values:

  * a Python module   (dataclasses + a VEHICLE_PARAMS constant instance)
  * a C++17 header    (structs + an inline const VEHICLE_PARAMS instance)
  * a C header        (structs + a static const VEHICLE_PARAMS instance, firmware-friendly)

This is the single generator referenced by claude-docs/06-vehicle-params.md rule 3
("Bindings are GENERATED ... by tools/gen_params.py") and claude-docs/12-testing.md L1
("tools/gen_params.py: generated Python/C++/C bindings agree with each other on every field").
Generated files are never committed (claude-docs/10-conventions.md); CI regenerates them.

The generator is schema-driven: it walks config/vehicle_params.schema.json to build a small
type registry (`build_registry`), then renders that registry plus the concrete params values
into each language. It supports the shapes actually used by vehicle_params.schema.json:
scalars (number/integer/string, optionally nullable), nested objects (optionally nullable,
shared via $ref), and arrays of flat objects (optionally nullable) — see `FieldShape`/
`RecordType` below. A schema using a shape outside that set raises ValueError at generation
time rather than silently emitting something wrong.

This module is intentionally import-and-test friendly: `main()` is a thin CLI wrapper around
the functions below, so every unit in claude-docs/12-testing.md's coverage target for
correctness-critical Python can be exercised directly without a subprocess.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
from pathlib import Path
from typing import Any

import jsonschema
import yaml

DEFAULT_PARAMS_PATH = Path("config/vehicle_params.yaml")
DEFAULT_SCHEMA_PATH = Path("config/vehicle_params.schema.json")

ROOT_TYPE_NAME = "VehicleParams"
ROOT_INSTANCE_NAME = "VEHICLE_PARAMS"


# --------------------------------------------------------------------------------------
# Loading and validation
# --------------------------------------------------------------------------------------


class ParamsValidationError(Exception):
    """Raised when a params file fails schema validation.

    A consumer that cannot validate must refuse to start (claude-docs/06-vehicle-params.md
    rule 2) — this exception, and the nonzero CLI exit code it produces, is that refusal.
    """


def load_yaml_file(path: Path) -> Any:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_schema_file(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def validate_params(params: Any, schema: dict) -> None:
    """Validate `params` against `schema`. Raises ParamsValidationError with a specific,
    single-error message on the first (best-match) violation. Raises nothing on success."""
    validator_cls = jsonschema.validators.validator_for(schema)
    validator_cls.check_schema(schema)
    validator = validator_cls(schema, format_checker=jsonschema.FormatChecker())
    errors = list(validator.iter_errors(params))
    if not errors:
        return
    error = jsonschema.exceptions.best_match(errors)
    json_path = "$" + "".join(
        f"[{p!r}]" if isinstance(p, str) else f"[{p}]" for p in error.absolute_path
    )
    if not error.absolute_path:
        json_path = "$ (root)"
    raise ParamsValidationError(
        f"{json_path}: {error.message} (failed validator: {error.validator!r})"
    )


# --------------------------------------------------------------------------------------
# Schema -> type registry
# --------------------------------------------------------------------------------------


@dataclasses.dataclass
class FieldShape:
    name: str
    kind: str  # "number" | "integer" | "string" | "object" | "array"
    nullable: bool
    units: str | None
    sign_convention: str | None
    max_length: int | None = None  # string
    max_items: int | None = None  # array
    ref_type: str | None = None  # object: this field's type name; array: item type name


@dataclasses.dataclass
class RecordType:
    name: str
    fields: list[FieldShape]


def _pascal(name: str) -> str:
    return "".join(part.capitalize() for part in name.split("_") if part)


def _types_of(node: dict) -> tuple[str, bool]:
    t = node.get("type")
    types = {t} if isinstance(t, str) else set(t or [])
    nullable = "null" in types
    types.discard("null")
    if len(types) != 1:
        raise ValueError(f"unsupported/ambiguous schema 'type': {node.get('type')!r}")
    return next(iter(types)), nullable


def _resolve_ref(node: dict, root: dict) -> tuple[dict, str | None]:
    if "$ref" not in node:
        return node, None
    ref_name = node["$ref"].rsplit("/", 1)[-1]
    target = root["$defs"][ref_name]
    merged = dict(target)
    for k, v in node.items():
        if k != "$ref":
            merged[k] = v
    return merged, ref_name


def build_registry(schema: dict) -> tuple[dict[str, RecordType], list[str]]:
    """Walk `schema` and return (registry, top_level_field_order).

    `registry` is ordered dependency-first: any type referenced by another type appears
    earlier, which is exactly the order every emitter below needs (C/C++ types must be
    declared before use).
    """
    registry: dict[str, RecordType] = {}

    def build_field(field_name: str, raw_node: dict) -> FieldShape:
        resolved, ref_name = _resolve_ref(raw_node, schema)
        kind, nullable = _types_of(resolved)
        units = resolved.get("units")
        sign_convention = resolved.get("sign_convention")

        if kind == "object":
            type_name = _pascal(ref_name) if ref_name else _pascal(field_name)
            visit_object(type_name, resolved)
            return FieldShape(
                field_name, "object", nullable, units, sign_convention, ref_type=type_name
            )

        if kind == "array":
            item_resolved, item_ref = _resolve_ref(resolved["items"], schema)
            item_kind, item_nullable = _types_of(item_resolved)
            if item_nullable:
                raise ValueError(f"{field_name}: array items may not be individually nullable")
            if item_kind != "object":
                raise ValueError(f"{field_name}: only arrays of objects are supported")
            item_type_name = _pascal(item_ref) if item_ref else _pascal(field_name + "_item")
            visit_object(item_type_name, item_resolved)
            return FieldShape(
                field_name,
                "array",
                nullable,
                units,
                sign_convention,
                max_items=resolved.get("maxItems", 32),
                ref_type=item_type_name,
            )

        if kind == "string":
            return FieldShape(
                field_name,
                "string",
                nullable,
                units,
                sign_convention,
                max_length=resolved.get("maxLength", 63),
            )

        if kind in ("number", "integer"):
            return FieldShape(field_name, kind, nullable, units, sign_convention)

        raise ValueError(f"{field_name}: unsupported schema kind {kind!r}")

    def visit_object(type_name: str, node: dict) -> None:
        if type_name in registry:
            return
        props = node.get("properties", {})
        required = node.get("required")
        if required is None or set(required) != set(props.keys()):
            raise ValueError(
                f"{type_name}: schema object must list every property under 'required' "
                "(every field is present, nullable ones carry null) — got "
                f"required={required!r}, properties={sorted(props.keys())!r}"
            )
        fields = [build_field(name, props[name]) for name in required]
        registry[type_name] = RecordType(type_name, fields)

    top_required = schema.get("required")
    top_props = schema.get("properties", {})
    if top_required is None or set(top_required) != set(top_props.keys()):
        raise ValueError("root schema: 'required' must list every top-level section")

    visit_object(ROOT_TYPE_NAME, schema)
    return registry, list(top_required)


# --------------------------------------------------------------------------------------
# Shared banner
# --------------------------------------------------------------------------------------


def _banner_lines(schema: dict, comment_prefix: str) -> list[str]:
    version = schema.get("$id", "vehicle_params.schema.json")
    lines = [
        "GENERATED FILE — DO NOT EDIT. DO NOT COMMIT.",
        "Produced by tools/gen_params.py from config/vehicle_params.yaml +",
        "config/vehicle_params.schema.json. CI regenerates this on every build; a change",
        "here that isn't a regeneration of those inputs will be silently overwritten.",
        "See claude-docs/06-vehicle-params.md and claude-docs/10-conventions.md.",
        f"Schema: {version}",
    ]
    return [f"{comment_prefix} {line}" for line in lines]


# --------------------------------------------------------------------------------------
# Python emitter
# --------------------------------------------------------------------------------------

_PY_SCALAR_TYPES = {"number": "float", "integer": "int", "string": "str"}


def _py_field_type(shape: FieldShape) -> str:
    if shape.kind in _PY_SCALAR_TYPES:
        base = _PY_SCALAR_TYPES[shape.kind]
    elif shape.kind == "object":
        base = shape.ref_type
    elif shape.kind == "array":
        base = f"list[{shape.ref_type}]"
    else:  # pragma: no cover - guarded by build_registry
        raise ValueError(shape.kind)
    return f"Optional[{base}]" if shape.nullable else base


def _py_render_value(shape: FieldShape, value: Any, registry: dict[str, RecordType]) -> str:
    if value is None:
        if not shape.nullable:
            raise ParamsValidationError(f"{shape.name}: null value for a non-nullable field")
        return "None"
    if shape.kind == "number":
        return repr(float(value))
    if shape.kind == "integer":
        return repr(int(value))
    if shape.kind == "string":
        return repr(str(value))
    if shape.kind == "object":
        rt = registry[shape.ref_type]
        args = ", ".join(
            f"{f.name}={_py_render_value(f, value[f.name], registry)}" for f in rt.fields
        )
        return f"{shape.ref_type}({args})"
    if shape.kind == "array":
        rt = registry[shape.ref_type]
        items = []
        for item in value:
            args = ", ".join(
                f"{f.name}={_py_render_value(f, item[f.name], registry)}" for f in rt.fields
            )
            items.append(f"{shape.ref_type}({args})")
        return "[" + ", ".join(items) + "]"
    raise ValueError(shape.kind)  # pragma: no cover - guarded by build_registry


def generate_python(params: dict, schema: dict) -> str:
    registry, _ = build_registry(schema)
    lines = _banner_lines(schema, "#")
    lines += [
        "from __future__ import annotations",
        "",
        "from dataclasses import dataclass",
        "from typing import Optional",
        "",
        "",
    ]
    for type_name, rt in registry.items():
        lines.append("@dataclass(frozen=True)")
        lines.append(f"class {type_name}:")
        for f in rt.fields:
            lines.append(
                f"    {f.name}: {_py_field_type(f)}  # units: {f.units}"
                + (f"; sign_convention: {f.sign_convention}" if f.sign_convention else "")
            )
        lines.append("")
        lines.append("")

    root_rt = registry[ROOT_TYPE_NAME]
    args = ", ".join(
        f"{f.name}={_py_render_value(f, params[f.name], registry)}" for f in root_rt.fields
    )
    lines.append(f"{ROOT_INSTANCE_NAME}: {ROOT_TYPE_NAME} = {ROOT_TYPE_NAME}({args})")
    lines.append("")
    return "\n".join(lines)


# --------------------------------------------------------------------------------------
# C++ emitter
# --------------------------------------------------------------------------------------

_CPP_SCALAR_TYPES = {"number": "double", "integer": "std::int64_t", "string": "std::string"}


def _cpp_field_type(shape: FieldShape) -> str:
    if shape.kind in _CPP_SCALAR_TYPES:
        base = _CPP_SCALAR_TYPES[shape.kind]
    elif shape.kind == "object":
        base = shape.ref_type
    elif shape.kind == "array":
        base = f"std::vector<{shape.ref_type}>"
    else:  # pragma: no cover
        raise ValueError(shape.kind)
    return f"std::optional<{base}>" if shape.nullable else base


def _cpp_render_scalar(shape: FieldShape, value: Any) -> str:
    if shape.kind == "number":
        return repr(float(value))
    if shape.kind == "integer":
        return str(int(value))
    return json.dumps(str(value))


def _cpp_render_value(shape: FieldShape, value: Any, registry: dict[str, RecordType]) -> str:
    if value is None:
        if not shape.nullable:
            raise ParamsValidationError(f"{shape.name}: null value for a non-nullable field")
        return "std::nullopt"
    if shape.kind in ("number", "integer", "string"):
        return _cpp_render_scalar(shape, value)
    if shape.kind == "object":
        rt = registry[shape.ref_type]
        args = ", ".join(_cpp_render_value(f, value[f.name], registry) for f in rt.fields)
        return f"{shape.ref_type}{{{args}}}"
    if shape.kind == "array":
        rt = registry[shape.ref_type]
        items = []
        for item in value:
            args = ", ".join(_cpp_render_value(f, item[f.name], registry) for f in rt.fields)
            items.append(f"{shape.ref_type}{{{args}}}")
        return f"std::vector<{shape.ref_type}>{{{', '.join(items)}}}"
    raise ValueError(shape.kind)  # pragma: no cover


def generate_cpp_header(params: dict, schema: dict) -> str:
    registry, _ = build_registry(schema)
    guard = "RACER_VEHICLE_PARAMS_GENERATED_HPP_"
    lines = _banner_lines(schema, "//")
    lines += [
        f"#ifndef {guard}",
        f"#define {guard}",
        "",
        "#include <cstdint>",
        "#include <optional>",
        "#include <string>",
        "#include <vector>",
        "",
    ]
    for type_name, rt in registry.items():
        lines.append(f"struct {type_name} {{")
        for f in rt.fields:
            comment = f"units: {f.units}"
            if f.sign_convention:
                comment += f"; sign_convention: {f.sign_convention}"
            lines.append(f"  {_cpp_field_type(f)} {f.name};  // {comment}")
        lines.append("};")
        lines.append("")

    root_rt = registry[ROOT_TYPE_NAME]
    args = ", ".join(_cpp_render_value(f, params[f.name], registry) for f in root_rt.fields)
    lines.append(
        f"inline const {ROOT_TYPE_NAME} {ROOT_INSTANCE_NAME} = {ROOT_TYPE_NAME}{{{args}}};"
    )
    lines.append("")
    lines.append(f"#endif  // {guard}")
    lines.append("")
    return "\n".join(lines)


# --------------------------------------------------------------------------------------
# C emitter (firmware-friendly: no STL; is_set/count sidecar fields for nullability)
# --------------------------------------------------------------------------------------

_C_SCALAR_TYPES = {"number": "double", "integer": "int64_t"}


def _c_field_decls(shape: FieldShape) -> list[str]:
    """One FieldShape can expand to more than one C struct member (value + is_set/count)."""
    decls: list[str] = []
    comment = f"units: {shape.units}"
    if shape.sign_convention:
        comment += f"; sign_convention: {shape.sign_convention}"

    if shape.kind == "string":
        decls.append(f"  char {shape.name}[{shape.max_length + 1}];  // {comment}")
        if shape.nullable:
            decls.append(f"  bool {shape.name}_is_set;")
        return decls

    if shape.kind in _C_SCALAR_TYPES:
        decls.append(f"  {_C_SCALAR_TYPES[shape.kind]} {shape.name};  // {comment}")
        if shape.nullable:
            decls.append(f"  bool {shape.name}_is_set;")
        return decls

    if shape.kind == "object":
        decls.append(f"  {shape.ref_type} {shape.name};  // {comment}")
        if shape.nullable:
            decls.append(f"  bool {shape.name}_is_set;")
        return decls

    if shape.kind == "array":
        decls.append(f"  {shape.ref_type} {shape.name}[{shape.max_items}];  // {comment}")
        decls.append(f"  size_t {shape.name}_count;")
        return decls

    raise ValueError(shape.kind)  # pragma: no cover


def _c_render_designated(
    shape: FieldShape, value: Any, registry: dict[str, RecordType]
) -> list[str]:
    """Return designated-initializer fragments (e.g. '.mass_kg = 3.74') for one field."""
    frags: list[str] = []
    is_set = value is not None

    if shape.kind == "string":
        s = "" if value is None else str(value)
        if value is not None and len(s) > shape.max_length:
            raise ParamsValidationError(
                f"{shape.name}: string value {s!r} exceeds schema maxLength {shape.max_length}"
            )
        frags.append(f".{shape.name} = {json.dumps(s)}")
        if shape.nullable:
            frags.append(f".{shape.name}_is_set = {str(is_set).lower()}")
        return frags

    if shape.kind == "number":
        frags.append(f".{shape.name} = {repr(float(value)) if is_set else '0.0'}")
        if shape.nullable:
            frags.append(f".{shape.name}_is_set = {str(is_set).lower()}")
        return frags

    if shape.kind == "integer":
        frags.append(f".{shape.name} = {int(value) if is_set else 0}")
        if shape.nullable:
            frags.append(f".{shape.name}_is_set = {str(is_set).lower()}")
        return frags

    if shape.kind == "object":
        rt = registry[shape.ref_type]
        if is_set:
            inner = ", ".join(
                x for f in rt.fields for x in _c_render_designated(f, value[f.name], registry)
            )
        else:
            inner = ", ".join(x for f in rt.fields for x in _c_render_designated(f, None, registry))
        frags.append(f".{shape.name} = {{{inner}}}")
        if shape.nullable:
            frags.append(f".{shape.name}_is_set = {str(is_set).lower()}")
        return frags

    if shape.kind == "array":
        items = value or []
        if len(items) > shape.max_items:
            raise ParamsValidationError(
                f"{shape.name}: {len(items)} entries exceeds schema maxItems {shape.max_items}"
            )
        rt = registry[shape.ref_type]
        rendered_items = []
        for item in items:
            inner = ", ".join(
                x for f in rt.fields for x in _c_render_designated(f, item[f.name], registry)
            )
            rendered_items.append(f"{{{inner}}}")
        if rendered_items:
            frags.append(f".{shape.name} = {{{', '.join(rendered_items)}}}")
        frags.append(f".{shape.name}_count = {len(items)}")
        return frags

    raise ValueError(shape.kind)  # pragma: no cover


def generate_c_header(params: dict, schema: dict) -> str:
    registry, _ = build_registry(schema)
    guard = "RACER_VEHICLE_PARAMS_GENERATED_H_"
    lines = _banner_lines(schema, "//")
    lines += [
        f"#ifndef {guard}",
        f"#define {guard}",
        "",
        "#include <stdbool.h>",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "",
        "// NOTE: 'static const' scoping is only safe because this header is included from",
        "// exactly one translation unit (the round-trip test harness). A firmware build",
        "// including this from multiple .c files must switch to extern + a .c definition.",
        "",
    ]
    for type_name, rt in registry.items():
        lines.append("typedef struct {")
        for f in rt.fields:
            lines.extend(_c_field_decls(f))
        lines.append(f"}} {type_name};")
        lines.append("")

    root_rt = registry[ROOT_TYPE_NAME]
    frags = ", ".join(
        x for f in root_rt.fields for x in _c_render_designated(f, params[f.name], registry)
    )
    lines.append(f"static const {ROOT_TYPE_NAME} {ROOT_INSTANCE_NAME} = {{{frags}}};")
    lines.append("")
    lines.append(f"#endif  // {guard}")
    lines.append("")
    return "\n".join(lines)


# --------------------------------------------------------------------------------------
# Test harnesses (schema-driven JSON dumpers used only by the round-trip test)
# --------------------------------------------------------------------------------------


_PRINT_JSON_STRING_C = """\
static void print_json_string(const char *s) {
  putchar('"');
  for (const char *p = s; *p; ++p) {
    if (*p == '"' || *p == '\\\\') putchar('\\\\');
    putchar(*p);
  }
  putchar('"');
}
"""


def _emit_object_print(
    expr: str, type_name: str, registry: dict[str, RecordType], lang: str
) -> list[str]:
    rt = registry[type_name]
    lines = ['printf("{");']
    for i, f in enumerate(rt.fields):
        lines.append(f'printf("\\"{f.name}\\":");')
        lines += _emit_field_print(expr, f, registry, lang)
        if i != len(rt.fields) - 1:
            lines.append('printf(",");')
    lines.append('printf("}");')
    return lines


def _emit_scalar_print(expr: str, kind: str, lang: str) -> list[str]:
    if kind == "string":
        c_str_expr = f"{expr}.c_str()" if lang == "cpp" else expr
        return [f"print_json_string({c_str_expr});"]
    if kind == "integer":
        return [f'printf("%lld", (long long)({expr}));']
    return [f'printf("%.17g", (double)({expr}));']


def _emit_field_print(
    parent_expr: str, shape: FieldShape, registry: dict[str, RecordType], lang: str
) -> list[str]:
    field_expr = f"{parent_expr}.{shape.name}"
    lines: list[str] = []

    if shape.kind in ("number", "integer", "string"):
        if not shape.nullable:
            return _emit_scalar_print(field_expr, shape.kind, lang)
        if lang == "cpp":
            cond, val_expr = f"{field_expr}.has_value()", f"{field_expr}.value()"
        else:
            cond, val_expr = f"{field_expr}_is_set", field_expr
        lines.append("if (" + cond + ") {")
        lines += _emit_scalar_print(val_expr, shape.kind, lang)
        lines.append("} else {")
        lines.append('printf("null");')
        lines.append("}")
        return lines

    if shape.kind == "object":
        if not shape.nullable:
            return _emit_object_print(field_expr, shape.ref_type, registry, lang)
        if lang == "cpp":
            cond, val_expr = f"{field_expr}.has_value()", f"{field_expr}.value()"
        else:
            cond, val_expr = f"{field_expr}_is_set", field_expr
        lines.append("if (" + cond + ") {")
        lines += _emit_object_print(val_expr, shape.ref_type, registry, lang)
        lines.append("} else {")
        lines.append('printf("null");')
        lines.append("}")
        return lines

    if shape.kind == "array":
        idx = f"i_{shape.name}"
        if lang == "cpp":
            has_expr = f"{field_expr}.has_value()" if shape.nullable else "true"
            arr_expr = f"{field_expr}.value()" if shape.nullable else field_expr
            count_expr = f"{arr_expr}.size()"
        else:
            has_expr = f"{field_expr}_count > 0" if shape.nullable else "true"
            arr_expr = field_expr
            count_expr = f"{field_expr}_count"
        body = []
        if shape.nullable:
            body.append("if (" + has_expr + ") {")
        body.append('printf("[");')
        body.append(f"for (size_t {idx} = 0; {idx} < {count_expr}; ++{idx}) {{")
        body.append(f'if ({idx} > 0) printf(",");')
        body += _emit_object_print(f"{arr_expr}[{idx}]", shape.ref_type, registry, lang)
        body.append("}")
        body.append('printf("]");')
        if shape.nullable:
            body.append("} else {")
            body.append('printf("null");')
            body.append("}")
        return body

    raise ValueError(shape.kind)  # pragma: no cover


def _generate_harness(schema: dict, lang: str, include_line: str) -> str:
    registry, _ = build_registry(schema)
    body = _emit_object_print(ROOT_INSTANCE_NAME, ROOT_TYPE_NAME, registry, lang)

    lines = _banner_lines(schema, "//")
    lines += [
        include_line,
        "#include <stdio.h>",
        "",
        _PRINT_JSON_STRING_C.rstrip("\n"),
        "",
        "int main(void) {",
    ]
    lines += [f"  {line}" for line in body]
    lines += [
        '  printf("\\n");',
        "  return 0;",
        "}",
        "",
    ]
    return "\n".join(lines)


def generate_cpp_harness(schema: dict, header_name: str = "vehicle_params_generated.hpp") -> str:
    return _generate_harness(schema, "cpp", f'#include "{header_name}"')


def generate_c_harness(schema: dict, header_name: str = "vehicle_params_generated.h") -> str:
    return _generate_harness(schema, "c", f'#include "{header_name}"')


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------


def generate_all(params: dict, schema: dict) -> dict[str, str]:
    """Return {filename: contents} for the three committed-nowhere binding artifacts."""
    return {
        "vehicle_params_generated.py": generate_python(params, schema),
        "vehicle_params_generated.hpp": generate_cpp_header(params, schema),
        "vehicle_params_generated.h": generate_c_header(params, schema),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params", type=Path, default=DEFAULT_PARAMS_PATH)
    parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA_PATH)
    parser.add_argument("--out-dir", type=Path, default=Path("generated"))
    args = parser.parse_args(argv)

    try:
        params = load_yaml_file(args.params)
        schema = load_schema_file(args.schema)
    except OSError as e:
        print(f"ERROR: could not read {e.filename}: {e.strerror}", file=sys.stderr)
        return 2
    except (yaml.YAMLError, json.JSONDecodeError) as e:
        print(f"ERROR: could not parse params/schema: {e}", file=sys.stderr)
        return 2

    try:
        validate_params(params, schema)
    except ParamsValidationError as e:
        print(
            f"ERROR: {args.params} failed schema validation against {args.schema}: {e}",
            file=sys.stderr,
        )
        return 2

    try:
        artifacts = generate_all(params, schema)
    except (ParamsValidationError, ValueError) as e:
        print(f"ERROR: could not generate bindings: {e}", file=sys.stderr)
        return 2

    args.out_dir.mkdir(parents=True, exist_ok=True)
    for name, contents in artifacts.items():
        (args.out_dir / name).write_text(contents, encoding="utf-8")

    print(f"Generated {len(artifacts)} binding(s) into {args.out_dir}", file=sys.stderr)
    return 0


if __name__ == "__main__":  # pragma: no cover - exercised via main() directly in tests
    sys.exit(main())
