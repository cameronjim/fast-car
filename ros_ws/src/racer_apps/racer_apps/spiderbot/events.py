"""Event log and snapshot files for the spiderbot (roadmap 5.3). No ROS.

`EventLog` appends one JSON object per line to `<output_dir>/spiderbot_events.jsonl` and writes
snapshot JPEGs to `<output_dir>/snapshots/<event_id>.jpg`. Every state transition is logged
(engage, alert, resume, perception lost or restored), not only alerts, so a stopped car can
always be explained afterwards. The log is in addition to the rosbag (invariant 5): the node
publishes the same facts on /spiderbot/alert and /spiderbot/state, which the recorder keeps.

Event ids are `<UTC stamp>-<counter>`, unique within a run and sortable.
"""

from __future__ import annotations

import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path


def _clean(value):
    """JSON-safe: NaN and inf become null, tuples become lists."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    return value


class EventLog:
    def __init__(self, output_dir: str | Path, run_stamp: str | None = None) -> None:
        self.dir = Path(os.path.expanduser(str(output_dir)))
        self.dir.mkdir(parents=True, exist_ok=True)
        (self.dir / "snapshots").mkdir(exist_ok=True)
        self.path = self.dir / "spiderbot_events.jsonl"
        self._stamp = run_stamp or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self._counter = 0

    def new_event_id(self) -> str:
        self._counter += 1
        return f"{self._stamp}-{self._counter:04d}"

    def append(self, kind: str, wall_time_s: float, **fields) -> None:
        record = {"kind": kind, "wall_time_s": wall_time_s, **fields}
        with self.path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(_clean(record), sort_keys=True) + "\n")

    def write_snapshot(self, event_id: str, jpeg: bytes | None) -> str:
        """Write the JPEG and return its path, or "" when there is no image or it fails."""
        if not jpeg:
            return ""
        target = self.dir / "snapshots" / f"{event_id}.jpg"
        try:
            target.write_bytes(jpeg)
        except OSError:
            return ""
        return str(target)

    def read_all(self) -> list[dict]:
        if not self.path.is_file():
            return []
        return [json.loads(line) for line in self.path.read_text(encoding="utf-8").splitlines()]
