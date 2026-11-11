"""In-memory overrides on top of the generated `vehicle_params` binding.

A training run sometimes has to assume measurements that are not in
`config/vehicle_params.yaml` yet (v0: the 2026-11-10 calibration is committed on its own
branch by someone else). The experiment config carries them as `vehicle_overrides`, a nested
mapping that mirrors the binding's own structure; this applies them to a COPY of the frozen
binding. The yaml and the generated module are never touched (CLAUDE.md invariant 2 stays
true: the numbers live in a committed, hashed config next to a statement of where they come
from, not in code).

Unknown keys are an error, not a no-op: a typo that silently leaves the committed value in
place is exactly how a run ends up trained against numbers nobody meant.
"""

from __future__ import annotations

import dataclasses
from typing import Any


def apply_vehicle_overrides(params: Any, overrides: dict[str, Any]) -> Any:
    """Return a copy of the frozen dataclass tree `params` with `overrides` applied."""
    if not overrides:
        return params
    if not dataclasses.is_dataclass(params):
        raise TypeError(f"cannot apply overrides to non-dataclass {type(params).__name__}")
    names = {f.name for f in dataclasses.fields(params)}
    updates: dict[str, Any] = {}
    for key, value in overrides.items():
        if key not in names:
            raise KeyError(
                f"vehicle_overrides: {type(params).__name__} has no field {key!r} "
                f"(fields: {sorted(names)})"
            )
        current = getattr(params, key)
        if isinstance(value, dict):
            updates[key] = apply_vehicle_overrides(current, value)
        else:
            updates[key] = value
    return dataclasses.replace(params, **updates)
