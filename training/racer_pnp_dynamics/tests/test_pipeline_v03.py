"""L1 for the scoring and the reports of pipeline_v03.py on a synthetic plant."""

from __future__ import annotations

import numpy as np
import pytest
from racer_pnp_dynamics import pipeline_v03 as p3
from racer_pnp_dynamics import report_v03
from racer_pnp_dynamics.dataset import by_split
from racer_pnp_dynamics.model import Residual
from test_fit import TRUTH
from test_residual_train import NOISE, _segments, known_residual


@pytest.fixture(scope="module")
def scored(toy_vehicle_module):
    segs = _segments(toy_vehicle_module, known_residual(), seed=51, **NOISE)
    held = by_split(segs, "heldout", odom_only=True)
    models = {
        "nominal": (TRUTH, None),
        "with residual": (TRUTH, known_residual()),
        "bad": (TRUTH.replace(steer_gain_left=0.6), None),
    }
    return p3.score_models(held, toy_vehicle_module, models, p3.TRUTH_T2), held


def test_a_model_that_is_the_plant_beats_the_nominal_on_yaw(scored):
    sc, _ = scored
    assert sc.windows > 100 and sc.truth == "T2"
    assert p3.at1(sc, "with residual")["Y1"] < p3.at1(sc, "nominal")["Y1"]
    assert p3.at1(sc, "bad")["Y1"] != p3.at1(sc, "nominal")["Y1"]  # the models are told apart
    assert np.isnan(sc.yp["nominal"])  # no /pose segments given: Yp cannot be applied


def test_bootstrap_ratio_and_stratum_helpers(scored):
    sc, _ = scored
    boot = p3.bootstrap_p1_ratio(sc, "with residual", "nominal")
    assert boot["lo"] <= boot["ratio"] <= boot["hi"] and boot["resamples"] == 2000
    strat = p3.stratum_pos(sc, "nominal")
    assert set(strat) == set(sc.strata_names)


def test_the_summary_is_json_able(scored):
    import json

    sc, _ = scored
    s = p3.scored_summary(sc)
    json.dumps(s, default=float)
    assert s["windows"] == sc.windows and "nominal" in s["macro"]


def test_mb1_markdown_has_both_truths_and_the_ratios(scored):
    sc, _ = scored
    text = p3.render_mb1(sc, sc, ["a note"], ("nominal", "with residual"))
    assert "Truth T2 (primary)" in text and "labelled, v0.2 truth, decides nothing" in text
    assert "a note" in text and "Ratios with residual / nominal" in text


def test_the_candidate_readme_starts_with_the_verdict_and_carries_the_statement(scored):
    sc, _ = scored
    summary = p3.scored_summary(sc)
    decision = {
        "gate": "GP2c-Y yaw refit",
        "passed": False,
        "items": [
            {"name": "1a x", "value": 1.1, "comparison": "<=", "threshold": 1.0, "passed": False}
        ],
    }
    refit = {
        "free": ["steer_gain_left"],
        "base": {"steer_gain_left": 1.0},
        "nominal": {"steer_gain_left": 0.9},
        "bounds": {"steer_gain_left": (0.5, 1.5)},
        "on_bound": [],
        "unidentified": {},
        "yaw_rmse_val": {"base": 0.01, "refit": 0.009},
        "counts": {"yaw_intervals_train": 10, "yaw_intervals_val": 5},
    }
    boot = {"resamples": 2000, "seed": 0, "ratio": 1.1, "lo": 1.0, "hi": 1.2}
    metrics = {
        "split_sha256": "0" * 64,
        "vehicle_params": "x / y",
        "refit": refit,
        "heldout": {
            "T2": {
                **summary,
                "macro": {**summary["macro"], p3.BASE_NAME: summary["macro"]["nominal"]},
            },
            "T1": summary,
            "decision_GP2c_Y": decision,
            "bootstrap_p1_ratio_Y_over_v01": boot,
        },
    }
    text = report_v03.render_y_report("dyn-v0.3y-test", metrics)
    assert text.startswith(p3.DO_NOT_DEPLOY)
    assert "FAIL" in text and "labelled" in text
    passed = {**decision, "passed": True}
    metrics["heldout"]["decision_GP2c_Y"] = passed
    ok = report_v03.render_y_report("dyn-v0.3y-test", metrics)
    assert p3.CANDIDATE_STATEMENT in ok.splitlines()[0]


def test_a_failed_residual_readme(scored):
    sc, _ = scored
    summary = p3.scored_summary(sc)
    decision = {"gate": "GP2c-R yaw-only residual", "passed": False, "items": []}
    metrics = {
        "split_sha256": "0" * 64,
        "addenda": ["base nominal decided before any residual was trained"],
        "selection": {"cfg": {"mean": 0.1, "min": 0.1, "seed0": 0.1}},
        "heldout": {
            "nominal": "dyn-v0.1",
            "shipping_config": "cfg",
            "shipping_seed": 0,
            "T2": summary,
            "T1": summary,
            "decision_GP2c_R": decision,
            "bootstrap_p1_ratio_R_over_nominal": {
                "resamples": 2000,
                "seed": 0,
                "ratio": 1.0,
                "lo": 0.9,
                "hi": 1.1,
            },
        },
    }
    text = report_v03.render_r_report("dyn-v0.3yr-test", metrics)
    assert text.startswith(p3.DO_NOT_DEPLOY) and "base nominal decided" in text


def test_the_empty_heldout_set_is_refused(toy_vehicle_module):
    with pytest.raises(ValueError, match="no held-out windows"):
        p3.score_models([], toy_vehicle_module, {"n": (TRUTH, None)}, p3.TRUTH_T2)
    assert isinstance(known_residual(), Residual)
