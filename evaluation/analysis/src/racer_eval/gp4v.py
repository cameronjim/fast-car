"""GP4-V (pre-registration v0.4 section 6.2): the protocol and thresholds of v0.2 section 5.2,
word for word, on an obstacle variant of a known track.

The thresholds are those of `gates.gp4_decision`: this module imports its constants and calls it,
so there is one source and no copy. GP4 itself ("a layout built for the session") is not decided
here and its function is untouched. The result carries the label `LABEL` and never the name GP4.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import replace

from racer_eval.curves import RunLaps
from racer_eval.gates import (  # noqa: F401  (re-exported: the identity test reads them here)
    GP4_LEVEL,
    GP4_MAX_MEDIAN_R,
    GP4_MIN_RUNS,
    GateResult,
    gp4_decision,
)
from racer_eval.variant_form import CheckedForm

LABEL = "GP4-V, lap learning on an obstacle variant of a known track"
SECTION = "v0.4 6.2 (v0.2 5.2)"


class VariantNotDocumented(ValueError):
    """The runs have no checked variant form: they cannot be GP4-V runs."""


def decide_gp4v(
    on_runs: Sequence[RunLaps],
    off_runs: Sequence[RunLaps],
    form: CheckedForm | None,
    driven_variants: Sequence[str] = (),
) -> GateResult:
    """The three items of v0.2 5.2 on the D-on runs of a variant (with `off_runs` the D-off runs at
    the lap-1 cap, same number, interleaved). `driven_variants` are the ids of variants driven in
    an earlier step; the strict reading of "never driven" refuses a form that is one of them."""
    if not isinstance(form, CheckedForm):
        raise VariantNotDocumented(
            "variant not documented: runs without a checked form are not GP4-V runs"
        )
    if form.form.variant_id in driven_variants:
        raise VariantNotDocumented(
            f"variant {form.form.variant_id!r} was driven in an earlier step: "
            "GP4-V needs a new variant"
        )
    base = gp4_decision(on_runs, off_runs)
    return replace(
        base,
        name="gp4v",
        section=SECTION,
        reported={**base.reported, "label": LABEL, "variant_id": form.form.variant_id},
    )
