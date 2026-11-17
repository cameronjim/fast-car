"""L1 for the text of the v0.5 pass and fit summary (report_gp2f.py)."""

from __future__ import annotations

import numpy as np
from racer_pnp_dynamics import gp2f, pass_v05
from racer_pnp_dynamics import report_gp2f as rg
from racer_pnp_dynamics import table_fit_v05 as tf
from racer_pnp_dynamics.learned_gm import GmCandidate, GmConfig
from test_fit import TRUTH
from test_gp2f import metrics, table, validity
from test_pass_v05 import held, models  # noqa: F401
from test_report_v05 import fit  # noqa: F401  (fixture)
from test_step_v21 import C5
from test_step_v21 import motion as motion21
from test_table_fit_v05 import train_windows  # noqa: F401  (fixture of `fit`)


def test_items_print_every_row_and_label_the_ones_that_cannot_be_applied():
    thin = metrics(0.5)
    from dataclasses import replace

    d = table(replace(thin, near_limit={"left": (3, 9.0), "right": (50, 0.5)}))
    text = rg.render_items(d.to_dict())
    assert text.startswith("### GP2f-T T5 is a candidate: PASS")
    assert (
        "CANNOT BE APPLIED" in text and "cannot be applied: 3 windows, at least 30 needed" in text
    )
    assert text.count("\n| ") >= len(d.items)
    failed = rg.render_items(table(metrics(0.95)).to_dict())
    assert failed.startswith("### GP2f-T T5 is a candidate: FAIL") and "| FAIL |" in failed
    off = gp2f.decide_gp2f_learned(C5, False)
    assert rg.render_items(off.to_dict()).endswith("not evaluated\n")


def test_validity_text_lists_the_items():
    v = validity()
    text = rg.render_validity("T5", v.to_dict())
    assert text.startswith("T5: v_max 1.5 m/s (pipeline rule 1.5); every item holds")
    assert text.count("\n- ") == len(v.items)


def test_comparator_and_metric_tables(held, toy_vehicle_module):  # noqa: F811
    p = pass_v05.score_pass(C5, held, held, toy_vehicle_module, models(), TRUTH)
    names = list(models())
    rows = pass_v05.comparator_rows(p, names)
    text = rg.render_comparator(rows)
    assert text.count("\n| ") == len(names) and "dyn-v0.1" in text
    m = {n: p.metrics(n, [1.15, 1.30]) for n in names}
    tables = rg.render_metric_tables(m, names)
    for part in (
        "Per direction",
        "Near-limit windows",
        "Transition windows",
        "Fast windows per direction",
        "bias ccw",
    ):
        assert part in tables
    strata = rg.render_strata(dict(p.strata.counts), {n: m[n].stratum_pos for n in names})
    assert strata.count("\n| ") == len(p.strata.counts)


def test_the_pass_report_has_every_gate_and_the_outcome(held, toy_vehicle_module):  # noqa: F811
    p = pass_v05.score_pass(C5, held, held, toy_vehicle_module, models(), TRUTH)
    sup = np.ones((5, 6), dtype=bool)
    result = pass_v05.decide_all(
        C5, p, t5="T5", tl5_seeds={0: "TL5 seed 0"}, tl5_shipping="TL5 seed 0", v01="dyn-v0.1",
        on_bound=[], split_half_alpha_max=0.0, split_half_gamma_max=0.0,
        t5_motion=motion21(5), tl5_motion=motion21(5, learned=True, scale=0.3), nominal=TRUTH,
        vehicle=toy_vehicle_module, rule_mps=1.4, supported_alpha=sup, supported_gamma=sup,
    )  # fmt: skip
    names = list(models())
    m = {n: p.metrics(n, [1.15, 1.30]) for n in names}
    text = rg.render_pass_report(
        pass_v05.decisions_to_dict(result),
        rg.render_comparator(pass_v05.comparator_rows(p, names)),
        rg.render_metric_tables(m, names),
        rg.render_strata(dict(p.strata.counts), {n: m[n].stratum_pos for n in names}),
        "a" * 40,
        ["a note"],
        "reported only table",
    )
    assert "Freeze commit: `" + "a" * 40 in text and "- a note" in text
    for gate in ("GP2f-S", "GP2f-T", "GP2f-L"):
        assert gate in text
    assert "GP2f outcome: **the one-speed contract stays**" in text
    assert "not evaluated" in text and "Validity above 1.3 m/s" in text
    assert "T5: v_max 1.3" in text and "TL5: v_max 1.3" in text
    assert "Bootstrap of the P1 ratio" in text
    assert "Reported only" in text and "reported only table" in text
    assert "Reported only" not in rg.render_pass_report(
        pass_v05.decisions_to_dict(result), "c", "t", "s", "b" * 40, []
    )


def test_the_fit_summary_prints_the_tables_and_the_selection_of_tl5(fit, toy_vehicle_module):  # noqa: F811
    f, sel = fit
    mine = tf.score(sel, TRUTH, toy_vehicle_module, f.groundmotion())
    base = tf.score(sel, TRUTH, toy_vehicle_module, tf.unit_motion(C5))
    cand = GmCandidate(GmConfig("wd1e-03", 1e-3), 1, object(), 0.04, 9)  # type: ignore[arg-type]
    tl5 = {
        "shipping": cand,
        "selection": {"wd1e-03": {"mean": 0.04, "min": 0.04, "seed1": 0.04}},
    }
    text = rg.render_fit_summary(f, mine, base, tl5, ["a note"], 0.05, 0.08)
    assert text.startswith("# v0.5 fits, frozen before the held-out pass") and "- a note" in text
    assert "## T5" in text and "## TL5" in text and "Shipping wd1e-03, seed 1" in text
    assert "| wd1e-03 | 0.0400 | 0.0400 | seed1 0.0400 |" in text
    assert "1 + gamma next to alpha" in text and "| dyn-v0.1 |" in text
    assert "## TL5" not in rg.render_fit_summary(f, mine, base, None, [], 0.05, 0.08)


def test_the_readme_starts_with_do_not_deploy_and_the_contract_2_1_line():
    text = rg.readme_v05(
        "dyn-v0.5t-2026-11-16", summary="T5.", sections=["### GP2f\n"], data_lines=["train"],
        device_note="cpu",
    )  # fmt: skip
    first = text.splitlines()[0]
    assert first == "do-not-deploy; contract 2.1: no planner on main can load this; candidate only"
    assert (
        "# dyn-v0.5t-2026-11-16" in text and "- train" in text and "Training device: cpu." in text
    )
    assert "Schema 2.1.0 is the owner's decision" in text
    assert "Training device" not in rg.readme_v05("m", summary="s", sections=[], data_lines=[])
