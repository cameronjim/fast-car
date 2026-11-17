"""The 2026-11-15 session as a data source (pre-registration v0.5 section 5, brief 47 step 4).

* `SESSION_BAGS`: the five bags of the session (docs/notes/dynamics-v05-session-identity.md).
* `load_session_split`: `evaluation/prereg/sessions/2026-11-15-dynamics-v05.yaml` as a `Split` the
  dataset code reads, after checking that the file names the registered rule file (its SHA-256).
* `require_committed_session`: fits refuse the session without the committed split file.
* `heading_direction`: the direction of a window (`cw` or `ccw`) from the measured T2 heading, not
  from a block name (a run that was turned around by reverse escapes changes direction).
"""

from __future__ import annotations

import subprocess
from collections.abc import Sequence
from pathlib import Path

import numpy as np
import yaml

from .dataset import Segment
from .evaluate import EvalSet
from .ground_motion_v21 import V05_SHA256
from .split import BagEntry, Split, SplitInvalid, _entry_of

SESSION = "2026-11-15"  # the planned date the yaml uses as the label of the session
SESSION_RELATIVE = Path("evaluation") / "prereg" / "sessions" / "2026-11-15-dynamics-v05.yaml"
SESSION_BAGS = (
    "2026-11-16T02-18-54_car_teleop",
    "2026-11-16T02-29-27_car_teleop",
    "2026-11-16T22-38-47_car_teleop",
    "2026-11-16T22-45-39_car_teleop",
    "2026-11-16T23-04-09_car_teleop",
)
HEADING_BLOCK_S = 5.0  # the window's direction is the net T2 yaw of its 5 s block


class SessionUnsplit(RuntimeError):
    """A fit asked for the session before its split file was written and committed."""


def _committed(path: Path) -> bool:
    try:
        out = subprocess.run(
            ["git", "-C", str(path.parent), "ls-tree", "--name-only", "HEAD", "--", path.name],
            capture_output=True,
            text=True,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return False
    return path.name in out.stdout.split()


def require_committed_session(root: Path, require_committed: bool = True) -> Path:
    """The session split file of a checkout. Raises `SessionUnsplit` when it is missing or, with
    `require_committed`, not in the HEAD commit (the commit order is the evidence)."""
    path = Path(root) / SESSION_RELATIVE
    if not path.is_file():
        raise SessionUnsplit(
            f"{SESSION_RELATIVE} is not there: M-B6 and the split file come before any fit"
        )
    if require_committed and not _committed(path):
        raise SessionUnsplit(f"{path} is not committed: commit it, then fit")
    return path


def load_session_split(path: Path, expected_rule_sha256: str = V05_SHA256) -> Split:
    """The session's parts as a `Split` (segments with their dynamics role, bag seconds after the
    first /scan). Refuses a file that names another rule file."""
    blob = Path(path).read_bytes()
    raw = yaml.safe_load(blob.decode("utf-8"))
    if str(raw.get("rule_file_sha256")) != expected_rule_sha256:
        raise SplitInvalid(
            f"{path}: rule_file_sha256 {raw.get('rule_file_sha256')} is not the registered "
            f"{expected_rule_sha256}"
        )
    bags: dict[str, BagEntry] = {}
    for name, body in raw["bags"].items():
        entry = _entry_of(name, body)
        for a, b in zip(entry.segments, entry.segments[1:], strict=False):
            if b.t_from_s < a.t_to_s:
                raise SplitInvalid(f"{name}: parts {a.name} and {b.name} overlap")
        bags[name] = entry
    import hashlib

    return Split(split_id=str(raw["prereg_id"]), sha256=hashlib.sha256(blob).hexdigest(), bags=bags)


def heading_direction(
    es: EvalSet, segments: Sequence[Segment], block_s: float = HEADING_BLOCK_S
) -> np.ndarray:
    """(E,) `ccw` for a net left turn, else `cw`, of each window of `es`: the sum of the T2 yaw
    changes of the good intervals in the `block_s` block (counted from the start of its segment)
    that holds the window start. A window whose block has no good interval takes the sign of its
    own T2 yaw change. Built from the measured heading only."""
    out = np.where(es.poses[:, -1, 2] > 0.0, "ccw", "cw").astype(object)
    done = np.zeros(len(es), dtype=bool)
    for seg in segments:
        m = seg.motion
        if m is None:
            continue
        in_run = np.array([r is seg.run for r in es.runs])[es.run_idx]
        sel = np.flatnonzero(in_run & (es.t_s >= seg.t0_s - 1e-9) & (es.t_s <= seg.t1_s + 1e-9))
        sel = sel[~done[sel]]
        if len(sel) == 0:
            continue
        yaw = np.where(m.ok & np.isfinite(m.dyaw_rad), m.dyaw_rad, 0.0)
        good = (m.ok & np.isfinite(m.dyaw_rad)).astype(int)
        cum_yaw = np.concatenate([[0.0], np.cumsum(yaw)])
        cum_good = np.concatenate([[0], np.cumsum(good)])
        b = np.floor((es.t_s[sel] - seg.t0_s) / block_s + 1e-9)
        lo = np.searchsorted(m.t_s, seg.t0_s + b * block_s - 1e-6)
        hi = np.searchsorted(m.t_s, seg.t0_s + (b + 1) * block_s - 1e-6)
        net = cum_yaw[hi] - cum_yaw[lo]
        has = (cum_good[hi] - cum_good[lo]) > 0
        out[sel[has]] = np.where(net[has] > 0.0, "ccw", "cw")
        done[sel] = True
    return out.astype(str)


def with_heading_direction(es: EvalSet, segments: Sequence[Segment]) -> EvalSet:
    """`es` with every window's direction taken from the measured heading."""
    from dataclasses import replace

    return replace(es, direction=heading_direction(es, segments))
