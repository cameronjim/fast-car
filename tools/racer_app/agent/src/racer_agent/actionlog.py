"""Persistent action log: one JSON object per line.

Every action a client asks for (and every refusal, override and guard action) is appended
here, so a session can be reconstructed afterwards. The file is capped: when it passes
`max_bytes` the oldest half of the lines is dropped.
"""

from __future__ import annotations

import json
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any


class ActionLog:
    def __init__(
        self, path: Path, max_bytes: int = 2_000_000, clock: Callable[[], float] = time.time
    ) -> None:
        self.path = Path(path)
        self.max_bytes = max_bytes
        self._clock = clock
        self._lock = threading.Lock()
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def add(
        self,
        action: str,
        ok: bool,
        actor: str = "",
        detail: str = "",
        params: dict[str, Any] | None = None,
        override: bool = False,
        override_reason: str = "",
    ) -> dict[str, Any]:
        entry: dict[str, Any] = {
            "ts": round(self._clock(), 3),
            "action": action,
            "ok": ok,
            "actor": actor,
            "detail": detail,
        }
        if params:
            entry["params"] = params
        if override:
            entry["override"] = True
            entry["override_reason"] = override_reason
        line = json.dumps(entry, separators=(",", ":"), sort_keys=True)
        with self._lock:
            with self.path.open("a", encoding="utf-8") as fh:
                fh.write(line + "\n")
            self._trim()
        return entry

    def _trim(self) -> None:
        try:
            size = self.path.stat().st_size
        except OSError:
            return
        if size <= self.max_bytes:
            return
        lines = self.path.read_text(encoding="utf-8").splitlines()
        keep = lines[len(lines) // 2 :]
        self.path.write_text("\n".join(keep) + "\n", encoding="utf-8")

    def tail(self, limit: int = 100) -> list[dict[str, Any]]:
        """The newest `limit` entries, newest first. Corrupt lines are skipped."""
        with self._lock:
            try:
                lines = self.path.read_text(encoding="utf-8").splitlines()
            except OSError:
                return []
        out: list[dict[str, Any]] = []
        for line in reversed(lines):
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
            if len(out) >= limit:
                break
        return out
