"""The one place a subprocess is started.

Every external command the agent runs (docker, systemctl, sudo) goes through a `Runner` as an
argv list. There is no shell on the agent side, so nothing a client sends can become shell
syntax. The few `bash -lc` strings that run INSIDE the car container are built by `commands.py`
from validated pieces only.

Tests swap in `FakeRunner`, which records the argv of every call and answers from a script.
"""

from __future__ import annotations

import subprocess
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field


@dataclass(frozen=True)
class CmdResult:
    argv: tuple[str, ...]
    returncode: int
    stdout: str = ""
    stderr: str = ""

    @property
    def ok(self) -> bool:
        return self.returncode == 0

    def tail(self, n: int = 400) -> str:
        """The end of stderr (or stdout), for putting in an error message."""
        text = (self.stderr or self.stdout).strip()
        return text[-n:]


class Runner:
    """Runs argv lists with subprocess. Never uses a shell."""

    def run(self, argv: Sequence[str], timeout: float = 30.0) -> CmdResult:
        argv_t = tuple(argv)
        try:
            proc = subprocess.run(
                argv_t, capture_output=True, text=True, timeout=timeout, check=False
            )
        except FileNotFoundError as exc:
            return CmdResult(argv_t, 127, "", f"command not found: {exc.filename}")
        except subprocess.TimeoutExpired:
            return CmdResult(argv_t, 124, "", f"timed out after {timeout:.0f} s")
        return CmdResult(argv_t, proc.returncode, proc.stdout, proc.stderr)


Responder = Callable[[tuple[str, ...]], "CmdResult | tuple[int, str] | None"]


@dataclass
class FakeRunner(Runner):
    """Test double: records every call, answers from `responders` then `default`.

    A responder gets the argv tuple and returns a CmdResult, an (returncode, stdout) pair, or
    None to pass. Responders are tried in order; the first non-None answer wins.
    """

    responders: list[Responder] = field(default_factory=list)
    default: tuple[int, str] = (0, "")
    calls: list[tuple[str, ...]] = field(default_factory=list)

    def run(self, argv: Sequence[str], timeout: float = 30.0) -> CmdResult:
        argv_t = tuple(argv)
        self.calls.append(argv_t)
        for responder in self.responders:
            answer = responder(argv_t)
            if answer is None:
                continue
            if isinstance(answer, CmdResult):
                return answer
            return CmdResult(argv_t, answer[0], answer[1], "")
        return CmdResult(argv_t, self.default[0], self.default[1], "")

    def calls_matching(self, *prefix: str) -> list[tuple[str, ...]]:
        return [c for c in self.calls if c[: len(prefix)] == prefix]
