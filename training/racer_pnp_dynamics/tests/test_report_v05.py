"""L1 for the v0.5 report text (report_v05.py): M-B5 states its stop rule and prints both tables."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
from racer_pnp_dynamics import report_v05 as rep
from racer_pnp_dynamics import table_fit_v05 as tf
from test_step_v21 import C5
from test_table_fit_v05 import (
    PLANT,
    _inject,
    _selection,
    _truth_tables,
    train_windows,  # noqa: F401  (fixture)
)


@pytest.fixture(scope="module")
def fit(train_windows, toy_vehicle_module):  # noqa: F811
    alpha, gamma = _truth_tables()
    win = _inject(train_windows, toy_vehicle_module, alpha, gamma)
    sel = _selection(toy_vehicle_module, alpha, gamma)
    return tf.fit_t5(win, sel, PLANT, toy_vehicle_module, C5), sel


def _scores(fit_sel, vehicle):
    f, sel = fit_sel
    mine = tf.score(sel, PLANT, vehicle, f.groundmotion())
    base = tf.score(sel, PLANT, vehicle, tf.unit_motion(C5))
    return mine, base


def test_m_b5_reports_the_tables_the_diagnostic_and_the_stop_rule(fit, toy_vehicle_module):
    f, _ = fit
    mine, base = _scores(fit, toy_vehicle_module)
    text = rep.render_mb5(
        f, mine, base, limit_alpha=0.05, limit_gamma=0.08, notes=["a note"],
        by_direction={"ccw": 5},
    )  # fmt: skip
    assert text.startswith("# M-B5")
    assert "No supported node is on a bound" in text and "- a note" in text
    assert text.count("\n| ") > 50  # the 30 row table, the diagnostic and the scores
    assert "(fixed)" in text and "1 + gamma" in text and "- ccw: 5" in text
    assert "split half alpha: largest difference" in text
    assert "split half gamma: largest difference" in text
    # the diagnostic lists supported turning nodes only (the centre column is fixed)
    diag = rep.render_turning_diagnostic(f)
    assert diag.count("\n| ") == int((f.supported & ~f.fixed).sum())


def test_a_node_on_a_bound_prints_the_stop(fit, toy_vehicle_module):
    f, _ = fit
    mine, base = _scores(fit, toy_vehicle_module)
    bad = replace(f, on_bound_alpha=["s+0.5000/v0.85"])
    text = rep.render_mb5(bad, mine, base, limit_alpha=0.05, limit_gamma=0.08, notes=[])
    assert "**STOP: a supported node is on a bound**" in text
    assert "s+0.5000/v0.85" in text and "Nothing more is fitted" in text
    assert "ON BOUND" in rep.render_table_rows(bad)


def test_split_half_summary_without_a_split_and_with_no_supported_node(fit):
    f, _ = fit
    assert rep.split_half_summary(replace(f, split_half=None), 0.05, 0.08) == [
        "split half: not computed"
    ]
    nan = {k: np.full_like(v, np.nan, dtype=float) for k, v in f.split_half.items()}
    lines = rep.split_half_summary(replace(f, split_half=nan), 0.05, 0.08)
    assert lines == ["split half alpha: no supported node", "split half gamma: no supported node"]


def test_number_formatting_blanks_missing_values():
    assert rep._f(None) == "" and rep._f(float("nan")) == "" and rep._f(0.5) == "0.5000"
