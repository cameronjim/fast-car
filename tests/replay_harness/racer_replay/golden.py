"""Golden-comparison engine (claude-docs/12-testing.md L4).

A "golden" here is a JSON file holding a list of flat records (one dict of
field -> value per record -- e.g. one per replayed frame/timestep). Actual
pipeline output is compared record-by-record, field-by-field, against the
committed golden, using the stated :class:`~racer_replay.tolerance.FieldTolerance`
for each field (never exact equality, see ``tolerance.py``).

No golden files are committed by this scaffold (roadmap task 0.9: no real
rosbags or pipeline outputs exist yet). This module is exercised by
``tests/test_golden.py`` against golden files written to a pytest
``tmp_path`` fixture.

Regeneration is deliberately loud and requires an explicit reason
(``12-testing.md``: "Golden updates are deliberate... A silent golden
refresh is a defect."): :func:`regenerate_golden` refuses to run without a
non-empty ``reason``, and always logs a warning banner reminding the caller
that the PR must explain *why* the golden changed.

Provenance header (added for roadmap task S.6, backward compatible): a
golden file may optionally be a JSON *object* ``{"meta": {...}, "records":
[...]}`` instead of a bare JSON list. ``meta`` is free-form (e.g. the
producing library's version/commit, a params-file identity, the seed used)
and is never read by :func:`compare_to_golden` -- it exists purely so a
reviewer/future-debugger can see what produced a committed reference
without re-deriving it. Every golden file written by this module before
this change, and every golden file written today without passing ``meta``,
remains a bare list; :func:`load_golden` accepts both shapes.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from racer_replay.tolerance import FieldDiff, FieldTolerance, compare_fields

logger = logging.getLogger(__name__)

Record = dict[str, Any]

_RULE = "=" * 72
REGENERATE_BANNER = (
    "\n" + _RULE + "\n"
    "GOLDEN FILE REGENERATED: {path}\n"
    "A silent golden refresh is a defect (claude-docs/12-testing.md, L4).\n"
    "This PR MUST include a note explaining WHY the golden output changed.\n"
    "Reason given for this run:\n"
    "  {reason}\n" + _RULE
)


@dataclass(frozen=True)
class GoldenComparisonResult:
    golden_path: Path
    ok: bool
    record_count_mismatch: str | None
    diffs_by_record: dict[int, list[FieldDiff]] = field(default_factory=dict)

    def format_report(self) -> str:
        """A readable diff report for a mismatch (claude-docs/12-testing.md L4)."""
        if self.ok:
            return f"OK: actual output matches {self.golden_path} within stated tolerances."

        lines = [f"MISMATCH against golden {self.golden_path}:"]
        if self.record_count_mismatch is not None:
            lines.append(f"  record count mismatch: {self.record_count_mismatch}")
        for record_index in sorted(self.diffs_by_record):
            lines.append(f"record[{record_index}]:")
            for diff in self.diffs_by_record[record_index]:
                lines.append(diff.format())
        return "\n".join(lines)


def _load_golden_json(path: Path) -> Any:
    if not path.exists():
        raise FileNotFoundError(
            f"golden file not found: {path}\n"
            "If this is a brand-new golden fixture, generate it once with "
            "regenerate_golden(..., reason=...) / --regenerate, and add a PR note "
            "explaining what it captures."
        )
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def load_golden(path: Path) -> list[Record]:
    """Load a golden file's records.

    Accepts both shapes this module ever writes: a bare JSON list of
    records (the original, still-default format), and the optional
    ``{"meta": {...}, "records": [...]}`` provenance-header wrapper (see
    module docstring) -- either way this returns just the record list, so
    every existing caller (:func:`compare_to_golden` included) needs no
    changes to support wrapped golden files.
    """
    data = _load_golden_json(path)
    if isinstance(data, list):
        return data
    if isinstance(data, dict) and isinstance(data.get("records"), list):
        return data["records"]
    raise TypeError(
        f"golden file {path} must contain a JSON list of records, or an object of the form "
        '{"meta": {...}, "records": [...]}'
    )


def load_golden_meta(path: Path) -> dict[str, Any]:
    """Return a golden file's provenance ``meta`` dict, or ``{}`` if it has none.

    A bare-list golden file (no wrapper) has no provenance header by
    construction, so this returns ``{}`` for it rather than raising --
    "no provenance recorded" is a normal, valid state for older/simple
    goldens, not an error.
    """
    data = _load_golden_json(path)
    if isinstance(data, dict) and isinstance(data.get("meta"), dict):
        return data["meta"]
    return {}


def save_golden(path: Path, records: list[Record], *, meta: dict[str, Any] | None = None) -> None:
    """Write a golden file.

    With ``meta=None`` (the default) this writes the original bare-list
    format, byte-for-byte as before -- fully backward compatible. Passing
    ``meta`` wraps the records with a provenance header (see module
    docstring); :func:`load_golden` reads either shape transparently.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    payload: Any = records if meta is None else {"meta": meta, "records": records}
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2, sort_keys=True)
        fh.write("\n")


def compare_to_golden(
    actual_records: list[Record],
    golden_path: Path,
    tolerances: dict[str, FieldTolerance],
) -> GoldenComparisonResult:
    """Compare ``actual_records`` to the committed golden at ``golden_path``.

    Raises :class:`~racer_replay.tolerance.MissingToleranceError` if any
    golden field has no stated tolerance -- see ``tolerance.py``.
    """
    expected_records = load_golden(golden_path)

    count_mismatch: str | None = None
    if len(actual_records) != len(expected_records):
        count_mismatch = (
            f"actual has {len(actual_records)} record(s), golden has {len(expected_records)}"
        )

    diffs_by_record: dict[int, list[FieldDiff]] = {}
    for i in range(min(len(actual_records), len(expected_records))):
        diffs = compare_fields(actual_records[i], expected_records[i], tolerances)
        if diffs:
            diffs_by_record[i] = diffs

    ok = count_mismatch is None and not diffs_by_record
    return GoldenComparisonResult(
        golden_path=golden_path,
        ok=ok,
        record_count_mismatch=count_mismatch,
        diffs_by_record=diffs_by_record,
    )


def regenerate_golden(
    actual_records: list[Record],
    golden_path: Path,
    *,
    reason: str,
    meta: dict[str, Any] | None = None,
) -> None:
    """Overwrite (or create) a golden file. Requires an explicit, non-empty reason.

    This is the ONLY way this module ever writes a golden file --
    :func:`compare_to_golden` never does, even on mismatch. Callers exposing
    this as a CLI flag should gate it behind an explicit ``--regenerate``
    flag (never a default/auto behavior on test failure). ``meta`` is
    forwarded to :func:`save_golden` unchanged (see its docstring); omitting
    it writes the original bare-list format.
    """
    if not reason or not reason.strip():
        raise ValueError(
            "regenerate_golden() requires a non-empty `reason` explaining WHY the "
            "golden output changed -- claude-docs/12-testing.md: a silent golden "
            "refresh is a defect. Pass the reason you will also put in the PR."
        )
    save_golden(golden_path, actual_records, meta=meta)
    logger.warning(REGENERATE_BANNER.format(path=golden_path, reason=reason.strip()))
