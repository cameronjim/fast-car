"""L1 unit tests for racer_replay.tolerance (claude-docs/12-testing.md).

Boundary, NaN, and inf cases per the roadmap 0.9 brief: "tolerance
comparison logic (boundary, NaN, inf)".
"""

from __future__ import annotations

import math

import pytest
from racer_replay.tolerance import (
    FieldDiff,
    FieldTolerance,
    MissingToleranceError,
    compare_fields,
)


class TestFieldToleranceIsClose:
    def test_exact_match_within_zero_tolerance(self):
        tol = FieldTolerance(atol=0.0, rtol=0.0)
        assert tol.is_close(1.0, 1.0)

    def test_exact_zero_tolerance_rejects_any_difference(self):
        tol = FieldTolerance(atol=0.0, rtol=0.0)
        assert not tol.is_close(1.0, 1.0 + 1e-12)

    def test_at_boundary_is_within_tolerance(self):
        # math.isclose is inclusive at the boundary. 0.5 is exactly
        # representable in binary floating point, so this boundary is
        # exact rather than landing on a decimal-rounding artifact.
        tol = FieldTolerance(atol=0.5, rtol=0.0)
        assert tol.is_close(1.0, 1.5)

    def test_epsilon_over_boundary_fails(self):
        tol = FieldTolerance(atol=0.5, rtol=0.0)
        assert not tol.is_close(1.0, 1.5 + 1e-9)

    def test_relative_tolerance(self):
        tol = FieldTolerance(atol=0.0, rtol=0.05)
        assert tol.is_close(100.0, 104.0)
        assert not tol.is_close(100.0, 106.0)

    def test_both_nan_matches(self):
        tol = FieldTolerance(atol=0.001)
        assert tol.is_close(math.nan, math.nan)

    def test_one_nan_does_not_match(self):
        tol = FieldTolerance(atol=1000.0)
        assert not tol.is_close(math.nan, 1.0)
        assert not tol.is_close(1.0, math.nan)

    def test_matching_positive_infinity(self):
        tol = FieldTolerance(atol=0.0)
        assert tol.is_close(math.inf, math.inf)

    def test_matching_negative_infinity(self):
        tol = FieldTolerance(atol=0.0)
        assert tol.is_close(-math.inf, -math.inf)

    def test_mismatched_infinity_signs(self):
        tol = FieldTolerance(atol=1e300)
        assert not tol.is_close(math.inf, -math.inf)

    def test_infinity_vs_finite_never_matches(self):
        tol = FieldTolerance(atol=1e300, rtol=1.0)
        assert not tol.is_close(math.inf, 1e300)

    def test_negative_rtol_is_clamped_not_rejected(self):
        # math.isclose raises on rel_tol < 0; FieldTolerance is a plain
        # dataclass so nothing stops a caller from typing a negative rtol.
        # Clamp to zero rather than letting a ValueError leak out of a
        # comparison helper.
        tol = FieldTolerance(atol=0.0, rtol=-1.0)
        assert tol.is_close(1.0, 1.0)
        assert not tol.is_close(1.0, 1.1)


class TestCompareFields:
    def test_no_diffs_when_all_within_tolerance(self):
        actual = {"x": 1.001, "y": 2.0}
        expected = {"x": 1.0, "y": 2.0}
        tolerances = {"x": FieldTolerance(atol=0.01), "y": FieldTolerance(atol=0.0)}
        assert compare_fields(actual, expected, tolerances) == []

    def test_diff_reported_for_out_of_tolerance_field(self):
        actual = {"x": 5.0}
        expected = {"x": 1.0}
        tolerances = {"x": FieldTolerance(atol=0.01)}
        diffs = compare_fields(actual, expected, tolerances)
        assert len(diffs) == 1
        assert diffs[0].field == "x"
        assert diffs[0].actual == 5.0
        assert diffs[0].expected == 1.0

    def test_missing_field_in_actual_is_reported_as_diff(self):
        actual: dict[str, float] = {}
        expected = {"x": 1.0}
        tolerances = {"x": FieldTolerance(atol=0.01)}
        diffs = compare_fields(actual, expected, tolerances)
        assert len(diffs) == 1
        assert diffs[0].actual is None

    def test_missing_tolerance_raises(self):
        actual = {"x": 1.0}
        expected = {"x": 1.0}
        with pytest.raises(MissingToleranceError):
            compare_fields(actual, expected, tolerances={})

    def test_extra_actual_fields_are_ignored(self):
        # compare_fields only walks `expected` -- an extra field in actual
        # that isn't part of the golden schema is not this function's
        # business (a pipeline output shape check is a separate concern).
        actual = {"x": 1.0, "unexpected_extra": 999.0}
        expected = {"x": 1.0}
        tolerances = {"x": FieldTolerance(atol=0.0)}
        assert compare_fields(actual, expected, tolerances) == []


class TestFieldDiffFormat:
    def test_format_includes_field_and_values(self):
        diff = FieldDiff("speed_mps", actual=5.5, expected=5.0, tolerance=FieldTolerance(atol=0.1))
        text = diff.format()
        assert "speed_mps" in text
        assert "5.5" in text
        assert "5.0" in text

    def test_format_includes_note_when_present(self):
        tol = FieldTolerance(atol=0.1, note="below sensor resolution")
        diff = FieldDiff("x", actual=1.2, expected=1.0, tolerance=tol)
        assert "below sensor resolution" in diff.format()

    def test_format_omits_delta_for_non_numeric_values(self):
        diff = FieldDiff(
            "status", actual="degraded", expected="nominal", tolerance=FieldTolerance()
        )
        text = diff.format()
        assert "delta=" not in text
