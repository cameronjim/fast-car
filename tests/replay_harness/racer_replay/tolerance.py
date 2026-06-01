"""Per-field tolerance comparison logic for the golden engine.

``12-testing.md`` L4 is explicit: golden outputs are compared "with stated
tolerances (never exact float equality)". This module is the enforcement
point -- comparing a field with no stated tolerance is a programming error
here (:class:`MissingToleranceError`), not a silent exact-equality fallback.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


class MissingToleranceError(KeyError):
    """Raised when a field is compared with no :class:`FieldTolerance` for it."""


@dataclass(frozen=True)
class FieldTolerance:
    """Tolerance for one golden field, in the sense of ``math.isclose``.

    ``atol``/``rtol`` must be stated explicitly by whoever wires up a
    golden comparison -- there is no default nonzero tolerance, since a
    silently-chosen tolerance is exactly as bad as silent exact equality.
    ``note`` is free text explaining *why* this tolerance was chosen (e.g.
    "EKF position, 1 mm: below LiDAR range resolution"); it is not used by
    the comparison itself but is surfaced in diff reports so a reviewer can
    see the reasoning was written down somewhere, not just picked.
    """

    atol: float = 0.0
    rtol: float = 0.0
    note: str = ""

    def is_close(self, actual: float, expected: float) -> bool:
        if math.isnan(actual) and math.isnan(expected):
            return True
        if math.isnan(actual) or math.isnan(expected):
            return False
        if math.isinf(actual) or math.isinf(expected):
            # inf/-inf only ever "matches" the identical signed infinity;
            # isclose() special-cases this the same way, but spell it out
            # since a golden field going infinite is worth flagging loudly.
            return actual == expected
        return math.isclose(actual, expected, rel_tol=self._safe_rtol(), abs_tol=self.atol)

    def _safe_rtol(self) -> float:
        # math.isclose rejects a negative rel_tol; atol=0, rtol=0 (the
        # dataclass default) is a legal *explicit* "exact match required"
        # tolerance -- e.g. an integer frame counter -- not the forbidden
        # silent default, since the caller had to type it.
        return max(self.rtol, 0.0)


@dataclass(frozen=True)
class FieldDiff:
    field: str
    actual: object
    expected: object
    tolerance: FieldTolerance

    def format(self) -> str:
        delta = ""
        if isinstance(self.actual, (int, float)) and isinstance(self.expected, (int, float)):
            delta = f" (delta={float(self.actual) - float(self.expected):+.6g})"
        tol = self.tolerance
        return (
            f"  {self.field}: actual={self.actual!r} expected={self.expected!r}{delta} "
            f"tolerance(atol={tol.atol}, rtol={tol.rtol})" + (f" -- {tol.note}" if tol.note else "")
        )


def compare_fields(
    actual: dict[str, float],
    expected: dict[str, float],
    tolerances: dict[str, FieldTolerance],
) -> list[FieldDiff]:
    """Compare two flat field->value mappings, returning the mismatches.

    Every key present in ``expected`` must have an entry in ``tolerances``
    (:class:`MissingToleranceError` otherwise) -- this is what makes "never
    exact float equality" structural rather than a convention someone has
    to remember. A field present in ``expected`` but missing from
    ``actual`` is reported as a diff with ``actual=None``.
    """
    diffs: list[FieldDiff] = []
    for field_name, expected_value in expected.items():
        if field_name not in tolerances:
            raise MissingToleranceError(
                f"no FieldTolerance stated for {field_name!r}; "
                "12-testing.md forbids comparing golden fields with an implicit tolerance"
            )
        tol = tolerances[field_name]
        actual_value = actual.get(field_name)
        if actual_value is None:
            diffs.append(FieldDiff(field_name, None, expected_value, tol))
            continue
        if not tol.is_close(float(actual_value), float(expected_value)):
            diffs.append(FieldDiff(field_name, actual_value, expected_value, tol))
    return diffs
