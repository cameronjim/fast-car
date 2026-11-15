"""The guard training code calls before it reads a bag: `require_split(bag_name)`.

    from pnp_data.guard import require_split
    require_split(bag.name)      # raises NoCommittedSplit for a bag of a session with no split

A bag dated 2026-11-14 or earlier is covered by the hand split in pnp-v0.4.yaml (and the older
v0.2 split) and passes. A newer bag passes only if a file `evaluation/prereg/sessions/*-split.yaml`
lists it and that file is committed (present in git HEAD). A fit that reads a bag with no
committed split is a bug (pre-registration v0.4 section 1, last paragraph).
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import yaml

from pnp_data.session_split import LAST_HAND_SPLIT_DATE, SESSIONS_DIR, bag_date


class NoCommittedSplit(RuntimeError):
    """A bag newer than 2026-11-14 has no committed split."""


def _committed(path: Path) -> bool:
    """True if `path` is in the HEAD commit of the git repository that holds it."""
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


def split_files(sessions_dir: Path = SESSIONS_DIR) -> list[Path]:
    return sorted(sessions_dir.glob("*-split.yaml")) if sessions_dir.is_dir() else []


def require_split(
    bag_name: str, sessions_dir: Path = SESSIONS_DIR, require_committed: bool = True
) -> Path | None:
    """Return the split file that lists `bag_name`, or None for a bag of 2026-11-14 or earlier.
    Raises NoCommittedSplit when a newer bag has no split file, or its file is not committed."""
    name = Path(bag_name).name
    if bag_date(name) <= LAST_HAND_SPLIT_DATE:
        return None
    listed_in: list[Path] = []
    for path in split_files(sessions_dir):
        doc = yaml.safe_load(path.read_text())
        if isinstance(doc, dict) and name in (doc.get("bags") or {}):
            listed_in.append(path)
    if not listed_in:
        raise NoCommittedSplit(
            f"{name} is newer than {LAST_HAND_SPLIT_DATE} and no split file lists it: run "
            "`tools/pnp_data.py split` on the session, commit the file, then fit"
        )
    for path in listed_in:
        if not require_committed or _committed(path):
            return path
    raise NoCommittedSplit(
        f"{name} is listed in {listed_in[0]} but that file is not committed: commit it, then fit"
    )
