"""L1 for report_v04.py: first lines, item tables, README, the pass report on a synthetic result."""

from __future__ import annotations

from racer_pnp_dynamics import gp2, report_v04


def _decision(passed=True):
    items = (
        gp2.Item("1a P1 ratio", 0.9, "<=", 0.95, passed),
        gp2.Item("7 Pp", float("nan"), "<=", 1.0, False, "Pp cannot be applied"),
    )
    return gp2.Decision("GP2e-T x", items).to_dict()


def test_first_lines():
    f = report_v04.first_line
    assert f(True, True) == "contract 2: no planner on main can load this; candidate only"
    assert f(True, False).startswith("do-not-deploy; contract 2: no planner on main can load this")
    assert f(False, False) == "do-not-deploy"
    assert "not confirmed" in f(False, True)
    assert f(False, True, "deploys nothing") == "deploys nothing"


def test_items_show_value_threshold_and_result_and_a_skipped_gate_says_so():
    text = report_v04.render_items(_decision())
    assert "| 1a P1 ratio | 0.9 | <= 0.95 | PASS |" in text and "cannot be applied" in text
    assert "FAIL" in text.splitlines()[0]
    skipped = gp2.decide_gp2e_learned(False).to_dict()
    assert "not evaluated" in report_v04.render_items(skipped)


def test_readme_has_the_warning_first_and_every_section():
    text = report_v04.readme(
        "dyn-v0.4t-2026-11-14",
        contract2=True,
        passed=False,
        summary="the table.",
        sections=["## GP2e-T", "", "items", ""],
        data_lines=["train role"],
        device_note="cpu",
    )
    lines = text.splitlines()
    assert lines[0].startswith("do-not-deploy; contract 2:")
    assert "# dyn-v0.4t-2026-11-14" in text and report_v04.CONFIRMATION in text
    assert "- train role" in text and "Training device: cpu." in text


def test_the_pass_report_lists_every_decision():
    heading = gp2.HeadingOutcome("no named outcome", (gp2.Item("Y1 ratio", 0.9, "<=", 0.8, False),))
    val = gp2.decide_validity(1.5, True, []).to_dict()
    result = {
        "shipping_nominal": "dyn-v0.1",
        "GP2d_N": _decision(),
        "GP2d_R": _decision(False),
        "heading_outcome": heading.to_dict(),
        "validity": val,
        "GP2e_T": _decision(),
        "GP2e_L": gp2.decide_gp2e_learned(False).to_dict(),
        "gp2e_outcome": gp2.ONE_SPEED_STAYS,
        "validity_T4": val,
        "validity_TL4": val,
    }
    text = report_v04.pass_report(result, "| table |\n", "a" * 40, ["note"], "read before text")
    for needle in (
        "Freeze commit: `" + "a" * 40,
        "GP2e-T x",
        "no named outcome",
        "the one-speed contract stays",
        "read before",
        "not evaluated",
        "- note",
    ):
        assert needle in text
    assert "deploy" not in " ".join(heading.to_dict())
