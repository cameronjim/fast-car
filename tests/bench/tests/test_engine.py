"""L1 unit tests for racer_bench.engine (claude-docs/12-testing.md L6).

Scripted steps are run for real (they're just `echo`/`false`/etc., not
hardware); human-confirm steps are driven with a mocked confirm callable,
never real stdin.
"""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

import pytest
from racer_bench.engine import (
    HumanConfirmStep,
    ProcedureError,
    ScriptedStep,
    SessionRecord,
    default_run,
    load_procedure,
    prompt_confirm,
    run_procedure,
    write_session_record,
)

TEMPLATE_PATH = (
    Path(__file__).resolve().parent.parent / "procedures" / "template_wheels_off_actuation.yaml"
)


def _write_yaml(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "procedure.yaml"
    path.write_text(text)
    return path


class TestLoadProcedure:
    def test_loads_the_template_procedure(self):
        procedure = load_procedure(TEMPLATE_PATH)
        assert procedure.name == "template_wheels_off_actuation"
        assert len(procedure.steps) >= 1
        assert any(isinstance(s, HumanConfirmStep) for s in procedure.steps)
        assert any(isinstance(s, ScriptedStep) for s in procedure.steps)

    def test_parses_scripted_step_fields(self, tmp_path):
        path = _write_yaml(
            tmp_path,
            """
            name: p
            steps:
              - id: s1
                type: scripted
                description: d
                command: ["echo", "hi"]
                expect:
                  returncode: 0
                  stdout_contains: "hi"
            """,
        )
        procedure = load_procedure(path)
        step = procedure.steps[0]
        assert isinstance(step, ScriptedStep)
        assert step.command == ["echo", "hi"]
        assert step.expect_returncode == 0
        assert step.expect_stdout_contains == "hi"

    def test_scripted_step_defaults(self, tmp_path):
        path = _write_yaml(
            tmp_path,
            """
            name: p
            steps:
              - id: s1
                type: scripted
                description: d
                command: ["true"]
            """,
        )
        step = load_procedure(path).steps[0]
        assert step.expect_returncode == 0
        assert step.expect_stdout_contains is None

    def test_parses_human_confirm_step(self, tmp_path):
        path = _write_yaml(
            tmp_path,
            """
            name: p
            steps:
              - id: s1
                type: human_confirm
                description: d
                prompt: "well?"
            """,
        )
        step = load_procedure(path).steps[0]
        assert isinstance(step, HumanConfirmStep)
        assert step.prompt == "well?"

    def test_missing_top_level_name_raises(self, tmp_path):
        path = _write_yaml(tmp_path, "steps: []")
        with pytest.raises(ProcedureError):
            load_procedure(path)

    def test_empty_steps_raises(self, tmp_path):
        path = _write_yaml(tmp_path, "name: p\nsteps: []")
        with pytest.raises(ProcedureError, match="non-empty"):
            load_procedure(path)

    def test_non_mapping_top_level_raises(self, tmp_path):
        path = _write_yaml(tmp_path, "- just\n- a\n- list")
        with pytest.raises(ProcedureError, match="mapping"):
            load_procedure(path)

    def test_unknown_step_type_raises(self, tmp_path):
        path = _write_yaml(
            tmp_path,
            """
            name: p
            steps:
              - id: s1
                type: not_a_real_type
                description: d
            """,
        )
        with pytest.raises(ProcedureError, match="unknown type"):
            load_procedure(path)

    def test_scripted_step_non_list_command_raises(self, tmp_path):
        path = _write_yaml(
            tmp_path,
            """
            name: p
            steps:
              - id: s1
                type: scripted
                description: d
                command: "not a list"
            """,
        )
        with pytest.raises(ProcedureError, match="list of strings"):
            load_procedure(path)

    def test_scripted_step_missing_command_raises(self, tmp_path):
        path = _write_yaml(
            tmp_path,
            """
            name: p
            steps:
              - id: s1
                type: scripted
                description: d
            """,
        )
        with pytest.raises(ProcedureError, match="command"):
            load_procedure(path)

    def test_duplicate_step_ids_raise(self, tmp_path):
        path = _write_yaml(
            tmp_path,
            """
            name: p
            steps:
              - id: dup
                type: human_confirm
                description: d
                prompt: p
              - id: dup
                type: human_confirm
                description: d
                prompt: p
            """,
        )
        with pytest.raises(ProcedureError, match="duplicate"):
            load_procedure(path)


class TestRunProcedureScripted:
    def test_passing_scripted_step(self, tmp_path):
        procedure = load_procedure(
            _write_yaml(
                tmp_path,
                """
                name: p
                steps:
                  - id: s1
                    type: scripted
                    description: d
                    command: ["echo", "hello"]
                    expect:
                      returncode: 0
                      stdout_contains: "hello"
                """,
            )
        )
        record = run_procedure(procedure)
        assert record.passed
        assert record.results[0].passed
        assert record.results[0].kind == "scripted"

    def test_failing_returncode(self, tmp_path):
        procedure = load_procedure(
            _write_yaml(
                tmp_path,
                """
                name: p
                steps:
                  - id: s1
                    type: scripted
                    description: d
                    command: ["python3", "-c", "import sys; sys.exit(1)"]
                """,
            )
        )
        record = run_procedure(procedure)
        assert not record.passed
        assert not record.results[0].passed

    def test_failing_stdout_contains(self, tmp_path):
        procedure = load_procedure(
            _write_yaml(
                tmp_path,
                """
                name: p
                steps:
                  - id: s1
                    type: scripted
                    description: d
                    command: ["echo", "goodbye"]
                    expect:
                      stdout_contains: "hello"
                """,
            )
        )
        record = run_procedure(procedure)
        assert not record.passed

    def test_uses_injected_run_callable(self, tmp_path):
        procedure = load_procedure(
            _write_yaml(
                tmp_path,
                """
                name: p
                steps:
                  - id: s1
                    type: scripted
                    description: d
                    command: ["some-fake-command"]
                """,
            )
        )
        calls = []

        def fake_run(command):
            calls.append(list(command))
            return subprocess.CompletedProcess(command, returncode=0, stdout="ok", stderr="")

        record = run_procedure(procedure, run=fake_run)
        assert record.passed
        assert calls == [["some-fake-command"]]


class TestRunProcedureHumanConfirm:
    def test_confirmed_step_passes(self, tmp_path):
        procedure = load_procedure(
            _write_yaml(
                tmp_path,
                """
                name: p
                steps:
                  - id: s1
                    type: human_confirm
                    description: d
                    prompt: "ok?"
                """,
            )
        )
        record = run_procedure(procedure, confirm=lambda prompt: True)
        assert record.passed
        assert record.results[0].kind == "human_confirm"
        assert "ok?" in record.results[0].detail

    def test_declined_step_fails(self, tmp_path):
        procedure = load_procedure(
            _write_yaml(
                tmp_path,
                """
                name: p
                steps:
                  - id: s1
                    type: human_confirm
                    description: d
                    prompt: "ok?"
                """,
            )
        )
        record = run_procedure(procedure, confirm=lambda prompt: False)
        assert not record.passed

    def test_mocked_confirm_receives_the_real_prompt_text(self, tmp_path):
        procedure = load_procedure(
            _write_yaml(
                tmp_path,
                """
                name: p
                steps:
                  - id: s1
                    type: human_confirm
                    description: d
                    prompt: "did the wheel turn left?"
                """,
            )
        )
        seen_prompts = []

        def mock_confirm(prompt):
            seen_prompts.append(prompt)
            return True

        run_procedure(procedure, confirm=mock_confirm)
        assert seen_prompts == ["did the wheel turn left?"]


class TestRunProcedureStopOnFailure:
    def _two_step_procedure(self, tmp_path):
        return load_procedure(
            _write_yaml(
                tmp_path,
                """
                name: p
                steps:
                  - id: s1
                    type: human_confirm
                    description: d
                    prompt: "first?"
                  - id: s2
                    type: human_confirm
                    description: d
                    prompt: "second?"
                """,
            )
        )

    def test_stops_after_first_failure_by_default(self, tmp_path):
        procedure = self._two_step_procedure(tmp_path)
        record = run_procedure(procedure, confirm=lambda p: False)
        assert len(record.results) == 1

    def test_continues_past_failure_when_disabled(self, tmp_path):
        procedure = self._two_step_procedure(tmp_path)
        record = run_procedure(procedure, confirm=lambda p: False, stop_on_failure=False)
        assert len(record.results) == 2
        assert not record.passed

    def test_runs_the_full_template_without_stopping_early_when_all_confirmed(self):
        procedure = load_procedure(TEMPLATE_PATH)
        record = run_procedure(procedure, confirm=lambda p: True)
        assert record.passed
        assert len(record.results) == len(procedure.steps)


class TestDefaultRun:
    def test_captures_stdout_as_text_and_never_raises(self):
        completed = default_run(["python3", "-c", "import sys; print('hi'); sys.exit(3)"])
        assert completed.returncode == 3
        assert "hi" in completed.stdout


class TestPromptConfirm:
    @pytest.mark.parametrize(
        ("stdin_text", "expected"),
        [("y\n", True), ("yes\n", True), ("Y\n", True), ("n\n", False), ("\n", False)],
    )
    def test_parses_stdin_answer(self, monkeypatch, stdin_text, expected):
        monkeypatch.setattr("builtins.input", lambda _prompt: stdin_text.strip())
        assert prompt_confirm("well?") is expected


class TestWriteSessionRecord:
    def test_writes_a_json_file_with_the_session_contents(self, tmp_path):
        procedure = load_procedure(TEMPLATE_PATH)
        record = run_procedure(procedure, confirm=lambda p: True)
        out_path = write_session_record(record, tmp_path / "sessions")
        assert out_path.exists()
        assert out_path.parent == tmp_path / "sessions"
        data = json.loads(out_path.read_text())
        assert data["procedure_name"] == "template_wheels_off_actuation"
        assert data["passed"] is True
        assert len(data["results"]) == len(record.results)

    def test_filename_is_unique_per_run_and_sorts_chronologically(self, tmp_path):
        procedure = load_procedure(TEMPLATE_PATH)
        record_a = run_procedure(procedure, confirm=lambda p: True)
        record_b = run_procedure(procedure, confirm=lambda p: True)
        path_a = write_session_record(record_a, tmp_path)
        path_b = write_session_record(record_b, tmp_path)
        assert path_a != path_b

    def test_creates_output_directory(self, tmp_path):
        procedure = load_procedure(TEMPLATE_PATH)
        record = run_procedure(procedure, confirm=lambda p: True)
        out_dir = tmp_path / "a" / "b" / "sessions"
        write_session_record(record, out_dir)
        assert out_dir.is_dir()

    def test_failed_session_is_recorded_too(self, tmp_path):
        procedure = load_procedure(TEMPLATE_PATH)
        record = run_procedure(procedure, confirm=lambda p: False)
        assert not record.passed
        path = write_session_record(record, tmp_path)
        data = json.loads(path.read_text())
        assert data["passed"] is False


def test_session_record_is_a_dataclass_instance():
    # Sanity check that run_procedure's return type is what callers expect
    # to hand to write_session_record.
    procedure = load_procedure(TEMPLATE_PATH)
    record = run_procedure(procedure, confirm=lambda p: True)
    assert isinstance(record, SessionRecord)
