"""The executable-checklist engine (claude-docs/12-testing.md L6).

Loads a YAML procedure file, runs each step (scripted or human-confirm),
and produces a :class:`SessionRecord` that :func:`write_session_record`
writes to a timestamped file -- results get written down, per
``12-testing.md``.
"""

from __future__ import annotations

import dataclasses
import json
import logging
import re
import subprocess
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import yaml

logger = logging.getLogger(__name__)


class ProcedureError(ValueError):
    """Raised for a malformed procedure YAML file."""


@dataclass(frozen=True)
class ScriptedStep:
    id: str
    description: str
    command: list[str]
    expect_returncode: int = 0
    expect_stdout_contains: str | None = None

    kind = "scripted"


@dataclass(frozen=True)
class HumanConfirmStep:
    id: str
    description: str
    prompt: str

    kind = "human_confirm"


Step = ScriptedStep | HumanConfirmStep


@dataclass(frozen=True)
class Procedure:
    name: str
    description: str
    steps: list[Step]


def _require(mapping: dict[str, Any], key: str, step_index: int) -> Any:
    if key not in mapping:
        raise ProcedureError(f"step[{step_index}] is missing required field {key!r}")
    return mapping[key]


def _parse_step(raw: dict[str, Any], step_index: int) -> Step:
    step_type = _require(raw, "type", step_index)
    step_id = _require(raw, "id", step_index)
    description = _require(raw, "description", step_index)

    if step_type == "scripted":
        command = _require(raw, "command", step_index)
        if not isinstance(command, list) or not all(isinstance(c, str) for c in command):
            raise ProcedureError(
                f"step[{step_index}] ({step_id!r}): 'command' must be a list of strings"
            )
        expect = raw.get("expect", {}) or {}
        return ScriptedStep(
            id=step_id,
            description=description,
            command=list(command),
            expect_returncode=int(expect.get("returncode", 0)),
            expect_stdout_contains=expect.get("stdout_contains"),
        )
    if step_type == "human_confirm":
        prompt = _require(raw, "prompt", step_index)
        return HumanConfirmStep(id=step_id, description=description, prompt=prompt)

    raise ProcedureError(
        f"step[{step_index}] ({step_id!r}): unknown type {step_type!r}, "
        "expected 'scripted' or 'human_confirm'"
    )


def load_procedure(path: Path) -> Procedure:
    """Parse a bench procedure YAML file into a :class:`Procedure`."""
    with Path(path).open("r", encoding="utf-8") as fh:
        raw = yaml.safe_load(fh)

    if not isinstance(raw, dict):
        raise ProcedureError(f"{path}: top-level YAML must be a mapping")

    name = _require(raw, "name", -1)
    description = raw.get("description", "")
    raw_steps = raw.get("steps")
    if not isinstance(raw_steps, list) or len(raw_steps) == 0:
        raise ProcedureError(f"{path}: 'steps' must be a non-empty list")

    steps = [_parse_step(raw_step, i) for i, raw_step in enumerate(raw_steps)]
    step_ids = [s.id for s in steps]
    if len(step_ids) != len(set(step_ids)):
        raise ProcedureError(f"{path}: duplicate step ids in {step_ids!r}")

    return Procedure(name=name, description=description, steps=steps)


@dataclass(frozen=True)
class StepResult:
    step_id: str
    kind: str
    description: str
    passed: bool
    detail: str
    started_at: str
    finished_at: str


@dataclass(frozen=True)
class SessionRecord:
    procedure_name: str
    started_at: str
    finished_at: str
    passed: bool
    results: list[StepResult] = field(default_factory=list)


ConfirmFn = Callable[[str], bool]
RunFn = Callable[[Sequence[str]], subprocess.CompletedProcess]


def _now_iso() -> str:
    return datetime.now(UTC).isoformat()


def prompt_confirm(prompt: str) -> bool:
    """Default human-confirm implementation: a real yes/no prompt on stdin."""
    answer = input(f"{prompt} [y/n]: ").strip().lower()
    return answer in ("y", "yes")


def default_run(command: Sequence[str]) -> subprocess.CompletedProcess:
    """Default scripted-step runner: capture output as text, never raise on nonzero exit.

    A nonzero/unexpected exit code is exactly what ``expect.returncode`` is
    for -- this must return normally so :func:`run_procedure` can record it
    as a failed step, not blow up with ``CalledProcessError``.
    """
    return subprocess.run(list(command), capture_output=True, text=True, check=False)


def _run_scripted_step(step: ScriptedStep, run: RunFn) -> tuple[bool, str]:
    completed = run(step.command)
    passed = completed.returncode == step.expect_returncode
    stdout = completed.stdout or ""
    if passed and step.expect_stdout_contains is not None:
        passed = step.expect_stdout_contains in stdout
    detail = (
        f"command={step.command!r} returncode={completed.returncode} "
        f"(expected {step.expect_returncode}) stdout={stdout!r}"
    )
    return passed, detail


def _run_human_confirm_step(step: HumanConfirmStep, confirm: ConfirmFn) -> tuple[bool, str]:
    answer = confirm(step.prompt)
    return bool(answer), f"prompt={step.prompt!r} human_answer={bool(answer)}"


def run_procedure(
    procedure: Procedure,
    *,
    confirm: ConfirmFn = prompt_confirm,
    run: RunFn = default_run,
    stop_on_failure: bool = True,
) -> SessionRecord:
    """Run every step of ``procedure`` in order, recording the outcome of each.

    Scripted steps are run with ``run`` (defaults to :func:`default_run`,
    which captures stdout/stderr as text and never raises on a nonzero
    exit code). Human-confirm steps are answered with ``confirm`` (defaults
    to a real stdin prompt via :func:`prompt_confirm`; tests pass a mock).
    """
    started_at = _now_iso()
    results: list[StepResult] = []
    overall_passed = True

    for step in procedure.steps:
        step_started = _now_iso()
        if isinstance(step, ScriptedStep):
            passed, detail = _run_scripted_step(step, run)
        else:
            passed, detail = _run_human_confirm_step(step, confirm)
        results.append(
            StepResult(
                step_id=step.id,
                kind=step.kind,
                description=step.description,
                passed=passed,
                detail=detail,
                started_at=step_started,
                finished_at=_now_iso(),
            )
        )
        if not passed:
            overall_passed = False
            if stop_on_failure:
                break

    return SessionRecord(
        procedure_name=procedure.name,
        started_at=started_at,
        finished_at=_now_iso(),
        passed=overall_passed,
        results=results,
    )


def _slugify(name: str) -> str:
    slug = re.sub(r"[^a-zA-Z0-9]+", "_", name).strip("_").lower()
    return slug or "session"


def write_session_record(record: SessionRecord, out_dir: Path) -> Path:
    """Write ``record`` to a timestamped JSON file under ``out_dir``.

    The filename embeds ``started_at`` and the procedure name so sessions
    sort chronologically and are identifiable at a glance; the full
    session content (every step's pass/fail and detail) lives inside the
    file itself, not just in the name.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    timestamp = record.started_at.replace(":", "").replace("-", "").replace(".", "_")
    filename = f"{timestamp}_{_slugify(record.procedure_name)}.json"
    path = out_dir / filename
    with path.open("w", encoding="utf-8") as fh:
        json.dump(dataclasses.asdict(record), fh, indent=2, sort_keys=True)
        fh.write("\n")
    return path
