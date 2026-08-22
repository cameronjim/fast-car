"""L1 unit tests for racer_replay.golden (claude-docs/12-testing.md L4).

No golden files are committed anywhere in this repo yet (roadmap task 0.9
is scaffold-only). These tests exercise the engine entirely against golden
files written to pytest's `tmp_path`.
"""

from __future__ import annotations

import json
import logging

import pytest
from racer_replay.golden import (
    compare_to_golden,
    load_golden,
    regenerate_golden,
    save_golden,
)
from racer_replay.tolerance import FieldTolerance, MissingToleranceError


@pytest.fixture
def tolerances():
    return {
        "x": FieldTolerance(atol=0.01, note="position, 1 cm"),
        "speed_mps": FieldTolerance(rtol=0.02, note="speed, 2%"),
    }


class TestLoadSave:
    def test_missing_golden_raises_with_helpful_message(self, tmp_path):
        with pytest.raises(FileNotFoundError, match="regenerate"):
            load_golden(tmp_path / "does_not_exist.json")

    def test_save_then_load_round_trips(self, tmp_path):
        golden_path = tmp_path / "nested" / "golden.json"
        records = [{"x": 1.0, "speed_mps": 2.0}, {"x": 1.1, "speed_mps": 2.1}]
        save_golden(golden_path, records)
        assert golden_path.exists()
        assert load_golden(golden_path) == records

    def test_save_creates_parent_directories(self, tmp_path):
        golden_path = tmp_path / "a" / "b" / "c" / "golden.json"
        save_golden(golden_path, [{"x": 1.0}])
        assert golden_path.exists()

    def test_non_list_golden_raises(self, tmp_path):
        golden_path = tmp_path / "golden.json"
        golden_path.write_text(json.dumps({"x": 1.0}))
        with pytest.raises(TypeError, match="JSON list"):
            load_golden(golden_path)


class TestCompareToGolden:
    def test_matching_within_tolerance_is_ok(self, tmp_path, tolerances):
        golden_path = tmp_path / "golden.json"
        save_golden(golden_path, [{"x": 1.0, "speed_mps": 2.0}])
        result = compare_to_golden([{"x": 1.005, "speed_mps": 2.0}], golden_path, tolerances)
        assert result.ok
        assert result.diffs_by_record == {}
        assert "OK" in result.format_report()

    def test_out_of_tolerance_field_is_reported(self, tmp_path, tolerances):
        golden_path = tmp_path / "golden.json"
        save_golden(golden_path, [{"x": 1.0, "speed_mps": 2.0}])
        result = compare_to_golden([{"x": 5.0, "speed_mps": 2.0}], golden_path, tolerances)
        assert not result.ok
        assert 0 in result.diffs_by_record
        assert result.diffs_by_record[0][0].field == "x"
        report = result.format_report()
        assert "MISMATCH" in report
        assert "x" in report

    def test_record_count_mismatch_is_reported(self, tmp_path, tolerances):
        golden_path = tmp_path / "golden.json"
        save_golden(golden_path, [{"x": 1.0, "speed_mps": 2.0}, {"x": 1.1, "speed_mps": 2.1}])
        result = compare_to_golden([{"x": 1.0, "speed_mps": 2.0}], golden_path, tolerances)
        assert not result.ok
        assert result.record_count_mismatch is not None
        assert "1" in result.record_count_mismatch and "2" in result.record_count_mismatch
        assert "record count mismatch" in result.format_report()

    def test_missing_tolerance_propagates(self, tmp_path):
        golden_path = tmp_path / "golden.json"
        save_golden(golden_path, [{"x": 1.0}])
        with pytest.raises(MissingToleranceError):
            compare_to_golden([{"x": 1.0}], golden_path, tolerances={})

    def test_multiple_records_each_compared(self, tmp_path, tolerances):
        golden_path = tmp_path / "golden.json"
        save_golden(
            golden_path,
            [{"x": 1.0, "speed_mps": 2.0}, {"x": 2.0, "speed_mps": 3.0}],
        )
        result = compare_to_golden(
            [{"x": 1.0, "speed_mps": 2.0}, {"x": 99.0, "speed_mps": 3.0}],
            golden_path,
            tolerances,
        )
        assert not result.ok
        assert list(result.diffs_by_record.keys()) == [1]


class TestRegenerateGolden:
    def test_requires_non_empty_reason(self, tmp_path):
        golden_path = tmp_path / "golden.json"
        with pytest.raises(ValueError, match="reason"):
            regenerate_golden([{"x": 1.0}], golden_path, reason="")

    def test_requires_non_whitespace_reason(self, tmp_path):
        golden_path = tmp_path / "golden.json"
        with pytest.raises(ValueError, match="reason"):
            regenerate_golden([{"x": 1.0}], golden_path, reason="   ")

    def test_writes_file_with_valid_reason(self, tmp_path):
        golden_path = tmp_path / "golden.json"
        regenerate_golden([{"x": 1.0}], golden_path, reason="tracker retuned, see PR #42")
        assert load_golden(golden_path) == [{"x": 1.0}]

    def test_logs_loud_reminder_naming_pr_note(self, tmp_path, caplog):
        golden_path = tmp_path / "golden.json"
        with caplog.at_level(logging.WARNING, logger="racer_replay.golden"):
            regenerate_golden([{"x": 1.0}], golden_path, reason="retuned controller")
        assert any("PR" in rec.message for rec in caplog.records)
        assert any("retuned controller" in rec.message for rec in caplog.records)

    def test_overwrites_existing_golden(self, tmp_path):
        golden_path = tmp_path / "golden.json"
        save_golden(golden_path, [{"x": 1.0}])
        regenerate_golden([{"x": 2.0}], golden_path, reason="deliberate change, see PR")
        assert load_golden(golden_path) == [{"x": 2.0}]

    def test_compare_never_writes_the_golden_file(self, tmp_path, tolerances):
        # A mismatch during compare_to_golden must never silently rewrite
        # the golden -- only regenerate_golden() may do that, and only with
        # an explicit reason.
        golden_path = tmp_path / "golden.json"
        save_golden(golden_path, [{"x": 1.0, "speed_mps": 2.0}])
        before = golden_path.read_text()
        compare_to_golden([{"x": 999.0, "speed_mps": 2.0}], golden_path, tolerances)
        assert golden_path.read_text() == before
