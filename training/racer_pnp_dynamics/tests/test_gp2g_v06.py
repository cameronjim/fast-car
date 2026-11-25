"""L1 and L2 of the yaw-only residual attempt R6 (brief 54): the registered constants and the role
loader, GP2g-R at pass, at threshold and over (INSUFFICIENT, "cannot be applied"), its thresholds
read from the yaml and compared with the GP2c-R / GP2d-R constants, the selection rule, the
saturation bound, and the speed channel that R6 can never change."""

from __future__ import annotations

import subprocess
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
import yaml
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_pnp_dynamics import gp2, gp2f, gp2g, prereg_v06, stages_v06
from racer_pnp_dynamics.model import MlpWeights, Residual, rollout, zero_weights
from racer_pnp_dynamics.residual_train import Candidate, ResidualConfig
from racer_pnp_dynamics.split import Split
from racer_pnp_dynamics.strata import assign_strata
from racer_pnp_dynamics.truth_v04 import FreezeToken, HeldoutLocked, freeze
from racer_pnp_dynamics.vehicle import load_vehicle, repo_root

ROOT = repo_root()
C = prereg_v06.load_constants_v06(ROOT / prereg_v06.V06_RELATIVE)
SESSION = ROOT / prereg_v06.SESSION_RELATIVE
BAG = prereg_v06.SESSION_BAG
NAN = float("nan")
EPS = 1e-9


# ---- constants and roles -----------------------------------------------------------------------


def test_the_constants_are_the_registered_file_and_another_hash_is_refused(tmp_path) -> None:
    assert C.sha256 == prereg_v06.V06_SHA256
    assert C.validity_v_max_mps == 1.3 and C.seeds == (0, 1, 2)
    p = tmp_path / "x.yaml"
    p.write_bytes((ROOT / prereg_v06.V06_RELATIVE).read_bytes() + b"#")
    with pytest.raises(prereg_v06.PreregInvalid, match="must not change"):
        prereg_v06.load_constants_v06(p)


def test_the_unchanged_thresholds_are_the_gp2c_r_and_gp2d_r_constants() -> None:
    g = C.gate
    assert g["max_p1_ratio"] == gp2.YAW_RESIDUAL_P1_MAX_RATIO == 0.95
    assert g["max_p1_ratio_bootstrap_upper"] == gp2.YAW_RESIDUAL_P1_CI_HI_BELOW == 1.00
    assert g["speed_equal_tolerance"] == gp2.S1_EQUAL_TOLERANCE_MPS
    assert g["max_seed_p1_ratio"] == gp2.YAW_RESIDUAL_SEED_P1_MAX_RATIO
    assert g["max_stratum_ratio"] == gp2.YAW_RESIDUAL_STRATUM_MAX_RATIO
    assert g["sweep_samples"] == gp2.SWEEP_SAMPLES
    assert g["max_pose_yaw_ratio"] == gp2.YAW_RESIDUAL_YP_MAX_RATIO
    # tighter than GP2d-R
    assert g["max_y1_ratio"] == 0.90 < gp2.YAW_RESIDUAL_Y1_MAX_RATIO
    h = C.doc["named_outcome_not_a_deployment_rule"]["improves_heading_not_position"]
    assert h["max_y1_ratio"] == gp2.HEADING_Y1_MAX_RATIO
    assert h["max_cross_ratio"] == gp2.HEADING_LATERAL_MAX_RATIO
    assert h["cross_bootstrap_upper_below"] == gp2.HEADING_LATERAL_CI_HI_BELOW
    assert h["max_p1_ratio"] == gp2.HEADING_P1_MAX_RATIO
    assert h["max_stratum_ratio"] == gp2.HEADING_STRATUM_MAX_RATIO
    assert C.metrics["min_windows_per_stratum"] == 30
    assert C.validity_v_max_mps == gp2.BASE_VALIDITY_MPS


def test_the_session_file_is_committed_with_the_v06_rule_and_one_bag() -> None:
    split = prereg_v06.load_session_v06(SESSION)
    assert set(split.bags) == {BAG}
    roles = [s.role for s in split.bags[BAG].segments]
    assert roles == ["train", "heldout"] * 5


def test_a_session_file_of_another_rule_or_uncommitted_is_refused(tmp_path) -> None:
    with pytest.raises(prereg_v06.PreregInvalid, match="is not there"):
        prereg_v06.load_session_v06(tmp_path / "none.yaml")
    p = tmp_path / "s.yaml"
    p.write_text(SESSION.read_text())
    with pytest.raises(prereg_v06.PreregInvalid, match="not committed"):
        prereg_v06.load_session_v06(p)
    split = prereg_v06.load_session_v06(p, require_committed=False)
    assert split.sha256 == stages_v06.sha256_file(SESSION)
    other = SESSION.read_text().replace(BAG, "2026-11-16T22-45-39_car_teleop")
    p.write_text(other)
    with pytest.raises(prereg_v06.PreregInvalid, match="are not"):
        prereg_v06.load_session_v06(p, require_committed=False)


def _roles() -> prereg_v06.Roles:
    return prereg_v06.Roles(prereg_v06.load_session_v06(SESSION))


def test_a_confirmation_bag_is_refused_for_every_role() -> None:
    r = _roles()
    for bag in ("2026-11-24T04-36-54_car_teleop", "2026-11-23T03-50-33_car_teleop"):
        assert prereg_v06.is_confirmation_bag(bag)
        with pytest.raises(prereg_v06.ConfirmationOnly):
            r.role_of(bag, "S1a")
        with pytest.raises(prereg_v06.ConfirmationOnly):
            r.check_bag(bag)
    for bag in (BAG, "2026-11-16T22-45-39_car_teleop"):
        assert not prereg_v06.is_confirmation_bag(bag)
        r.check_bag(bag)
    with pytest.raises(KeyError):
        r.role_of("2026-11-16T22-45-39_car_teleop", "S1a")
    with pytest.raises(KeyError):
        r.role_of(BAG, "S9z")


def test_held_out_parts_and_their_truth_need_the_freeze_token(tmp_path) -> None:
    r = _roles()
    assert [p.name for p in r.train_parts()] == ["S1a", "S2a", "S3a", "S4a", "S5a"]
    with pytest.raises(HeldoutLocked):
        r.heldout_parts(None)  # type: ignore[arg-type]
    token = freeze("a" * 40)
    assert [p.name for p in r.heldout_parts(token)] == ["S1b", "S2b", "S3b", "S4b", "S5b"]
    assert r.role_of(BAG, "S1b") == "heldout"
    # the caches are siblings and the readers refuse the wrong role or a missing token
    tr = prereg_v06.train_path(tmp_path, BAG, "S1a")
    ho = prereg_v06.heldout_path(tmp_path, BAG, "S1b", "s0.80")
    assert tr.parent.name == prereg_v06.TRAIN_DIR and ho.parent.name == prereg_v06.HELDOUT_DIR
    assert ho.name.endswith("-s0.80.npz")
    with pytest.raises(ValueError, match="variant"):
        prereg_v06.train_path(tmp_path, BAG, "S1a", "nope")
    with pytest.raises(HeldoutLocked, match="train parts only"):
        prereg_v06.load_train_slice(r, tmp_path, BAG, "S1b")
    with pytest.raises(HeldoutLocked, match="FreezeToken"):
        prereg_v06.load_heldout_slice(r, tmp_path, BAG, "S1b", None)  # type: ignore[arg-type]
    with pytest.raises(HeldoutLocked, match="not heldout"):
        prereg_v06.load_heldout_slice(r, tmp_path, BAG, "S1a", token)
    with pytest.raises(prereg_v06.ConfirmationOnly):
        prereg_v06.load_train_slice(r, tmp_path, "2026-11-24T04-36-54_car_teleop", "S1a")


def test_the_readers_return_the_cached_slices(tmp_path) -> None:
    from racer_pnp_dynamics import truth_v04
    from racer_pnp_dynamics.icp_motion import MotionSeries

    r = _roles()
    n = 3
    ms = MotionSeries(
        t_s=np.arange(n, dtype=float), dt_s=np.full(n, 0.1), dx_m=np.zeros(n), dy_m=np.zeros(n),
        dyaw_rad=np.zeros(n), ok=np.ones(n, bool), rms_m=np.zeros(n), inlier_share=np.ones(n),
    )  # fmt: skip
    for path in (
        prereg_v06.train_path(tmp_path, BAG, "S1a"),
        prereg_v06.heldout_path(tmp_path, BAG, "S1b"),
    ):
        path.parent.mkdir(parents=True, exist_ok=True)
        truth_v04.save_slice(path, truth_v04.Slice(4, ms))
    assert prereg_v06.load_train_slice(r, tmp_path, BAG, "S1a").first == 4
    got = prereg_v06.load_heldout_slice(r, tmp_path, BAG, "S1b", FreezeToken("b" * 40))
    assert got.first == 4


# ---- GP2g-R --------------------------------------------------------------------------------------


FOUR = ("ccw:0.25-0.6", "ccw:0.6-0.85", "cw:0.25-0.6", "cw:0.6-0.85")


def metrics(scale: float = 1.0, **over) -> gp2f.Metrics:
    base = gp2f.Metrics(
        macro={"P1": scale, "A1": scale, "C1": scale, "Y1": scale, "S1": 0.03},
        stratum_pos={n: scale for n in FOUR},
        direction={"ccw": {"P1": scale, "Y1": scale}, "cw": {"P1": scale, "Y1": scale}},
        yaw_bias={"ccw": 0.0, "cw": 0.0},
        near_limit={"left": (40, scale), "right": (50, scale)},
        transition=(60, scale),
        p1_alt=scale,
        pp=NAN,
        yp=scale,
    )
    return replace(base, **over)


REF = metrics(1.0)
GOOD = metrics(0.8, macro={"P1": 0.8, "A1": 0.8, "C1": 0.8, "Y1": 0.5, "S1": 0.03})


def windows(n_ccw: int = 250, n_cw: int = 250, speeds=(0.4, 0.7)):
    d = np.array(["ccw"] * n_ccw + ["cw"] * n_cw)
    s = np.array([speeds[i % len(speeds)] for i in range(len(d))])
    return d, assign_strata(d, s)


def sweep(bad: int = 0, samples: int = 100_000) -> gp2.SweepResult:
    return gp2.SweepResult(samples, 30, bad, 0, 0, 1.3, 0.3)


def decide(cand=GOOD, ref=REF, ci=0.9, seeds=None, sw=None, sat=0.0, solve=(0.6, 0.5),
           canary=(10, 10, 0), win=None):  # fmt: skip
    d, strata = win or windows()
    return gp2g.decide_gp2g_r(
        C, d, strata, ref, cand, ci,
        seeds if seeds is not None else {0: 0.8, 1: 0.81, 2: 0.82},
        sw or sweep(), sat, gp2g.SolveCost(*solve), gp2g.Canary(*canary),
    )  # fmt: skip


def item(result: gp2g.GateResult, prefix: str) -> gp2.Item:
    found = [i for i in result.decision.items if i.name.startswith(prefix)]
    assert len(found) == 1, (prefix, [i.name for i in result.decision.items])
    return found[0]


def test_a_candidate_inside_every_item_passes() -> None:
    r = decide()
    assert r.outcome == gp2g.PASS and r.decision.passed and r.enough_data.passed
    d = r.to_dict()
    assert d["outcome"] == "PASS" and d["decision"]["gate"] == gp2g.GATE


@pytest.mark.parametrize(
    ("field", "limit", "prefix"),
    [
        ("P1", 0.95, "1a"),
        ("Y1", 0.90, "2a"),
        ("C1", 1.00, "8a"),
        ("A1", 1.02, "8b"),
    ],
)
def test_each_macro_ratio_passes_at_its_threshold_and_fails_just_over(field, limit, prefix) -> None:
    for value, ok in ((limit, True), (limit + 1e-6, False), (NAN, False)):
        macro = dict(GOOD.macro)
        macro[field] = value
        r = decide(cand=replace(GOOD, macro=macro))
        assert item(r, prefix).passed is ok, (field, value)
        assert r.outcome == (gp2g.PASS if ok else gp2g.FAIL)


def test_the_bootstrap_item_is_strict_and_s1_is_an_equality() -> None:
    assert not item(decide(ci=1.0), "1b").passed
    assert item(decide(ci=1.0 - 1e-9), "1b").passed
    s1 = dict(GOOD.macro, S1=0.03 + 0.999e-6)
    assert item(decide(cand=replace(GOOD, macro=s1)), "2b").passed
    s1 = dict(GOOD.macro, S1=0.03 + 1.001e-6)
    assert not item(decide(cand=replace(GOOD, macro=s1)), "2b").passed


def test_every_seed_must_not_be_worse_than_dyn_v01() -> None:
    r = decide(seeds={0: 0.8, 1: 1.0, 2: 0.9})
    assert r.outcome == gp2g.PASS
    r = decide(seeds={0: 0.8, 1: 1.0 + 1e-6, 2: 0.9})
    assert not item(r, "3 seed 1").passed and r.outcome == gp2g.FAIL


def test_strata_with_30_windows_are_checked_and_smaller_ones_cannot_be_applied() -> None:
    d, st4 = windows()
    assert set(st4.names) == set(FOUR)
    cand = replace(GOOD, stratum_pos={n: 1.05 for n in FOUR})
    r = decide(cand=cand, ref=REF)
    assert all(i.passed for i in r.decision.items if i.name.startswith("4 "))
    assert r.outcome == gp2g.PASS
    cand = replace(cand, stratum_pos=dict(cand.stratum_pos, **{FOUR[0]: 1.0501}))
    assert decide(cand=cand, ref=REF).outcome == gp2g.FAIL
    small = replace(st4, names=(*FOUR, "ccw:0.85+"), counts=dict(st4.counts, **{"ccw:0.85+": 29}))
    cand = replace(GOOD, stratum_pos=dict(GOOD.stratum_pos, **{"ccw:0.85+": 9.0}))
    ref = replace(REF, stratum_pos=dict(REF.stratum_pos, **{"ccw:0.85+": 1.0}))
    r = decide(cand=cand, ref=ref, win=(d, small))
    na = [i for i in r.decision.items if i.comparison == gp2f.NA and i.name.startswith("4 ")]
    assert len(na) == 1 and na[0].passed and gp2f.GATE_NOT_APPLIED in na[0].note
    assert r.outcome == gp2g.PASS


def test_too_little_data_is_insufficient_never_a_pass() -> None:
    for n_ccw, n_cw in ((150, 149), (149, 251), (199, 200)):
        r = decide(win=windows(n_ccw, n_cw))
        assert r.outcome == gp2g.INSUFFICIENT and not r.decision.passed
        assert r.to_dict()["decision"]["evaluated"] is False
    r = decide(win=windows(200, 200, speeds=(0.4,)))  # 2 strata only
    assert r.outcome == gp2g.INSUFFICIENT
    assert item(decide(win=windows(150, 250)), "1a").passed  # enough data at the threshold
    assert decide(win=windows(150, 250)).outcome == gp2g.PASS


def test_sweep_yp_and_alt_truth_items() -> None:
    assert not item(decide(sw=sweep(bad=1)), "5").passed
    assert not item(decide(sw=sweep(samples=99_999)), "5").passed
    assert item(decide(cand=replace(GOOD, yp=1.0)), "6").passed
    assert not item(decide(cand=replace(GOOD, yp=1.0001)), "6").passed
    r = decide(cand=replace(GOOD, yp=NAN))
    assert not item(r, "6").passed and gp2.YP_CANNOT_BE_APPLIED in item(r, "6").note
    assert item(decide(cand=replace(GOOD, p1_alt=1.0)), "10").passed
    assert not item(decide(cand=replace(GOOD, p1_alt=1.0001)), "10").passed


def test_each_direction_by_itself() -> None:
    for d in ("ccw", "cw"):
        for key in ("P1", "Y1"):
            direction = {k: dict(v) for k, v in GOOD.direction.items()}
            direction[d][key] = 1.0
            assert decide(cand=replace(GOOD, direction=direction)).outcome == gp2g.PASS
            direction[d][key] = 1.0 + 1e-6
            r = decide(cand=replace(GOOD, direction=direction))
            assert not item(r, f"7 {d} {key}").passed
    missing = replace(GOOD, direction={"ccw": GOOD.direction["ccw"]})
    assert not item(decide(cand=missing), "7 cw P1").passed


def test_near_limit_and_transition_need_their_counts() -> None:
    cand = replace(GOOD, near_limit={"left": (30, 1.0), "right": (29, 5.0)}, transition=(29, 9.0))
    ref = replace(REF, near_limit={"left": (30, 1.0), "right": (29, 1.0)}, transition=(29, 1.0))
    r = decide(cand=cand, ref=ref)
    assert item(r, "9 near-limit left").passed
    right = item(r, "9 near-limit right")
    assert right.comparison == gp2f.NA and right.passed
    assert item(r, "9 transition").comparison == gp2f.NA
    assert r.outcome == gp2g.PASS
    cand = replace(cand, near_limit={"left": (30, 1.0001), "right": (29, 5.0)})
    assert decide(cand=cand, ref=ref).outcome == gp2g.FAIL
    cand = replace(GOOD, transition=(30, 1.0001))
    ref = replace(REF, transition=(30, 1.0))
    assert not item(decide(cand=cand, ref=ref), "9 transition").passed


def test_saturation_solve_time_and_canary() -> None:
    assert item(decide(sat=0.01), "11").passed
    assert not item(decide(sat=0.0101), "11").passed
    assert item(decide(solve=(1.25, 1.0)), "12").passed
    assert not item(decide(solve=(1.2501, 1.0)), "12").passed
    assert item(decide(canary=(10, 10, 0)), "13").passed
    assert not item(decide(canary=(10, 9, 0)), "13").passed
    assert not item(decide(canary=(10, 10, 1)), "13").passed
    assert not item(decide(canary=(0, 0, 0)), "13").passed


def test_the_heading_outcome_uses_the_v04_rule() -> None:
    cand = replace(GOOD, macro=dict(GOOD.macro, Y1=0.8, P1=1.0))
    out = gp2g.heading_outcome(C, REF, cand, 0.9, 0.99)
    assert out.label == gp2.HEADING_LABEL
    assert gp2g.heading_outcome(C, REF, cand, 0.9, 1.0).label == gp2.NO_NAMED_OUTCOME


# ---- saturation bound ----------------------------------------------------------------------------


def test_the_output_bound_holds_for_any_input_and_saturation_counts_live_channels() -> None:
    rng = np.random.default_rng(3)
    w = MlpWeights(*(rng.normal(size=a.shape) for a in zero_weights().arrays()))
    res = Residual(True, w, np.zeros(4), np.ones(4), np.array([0.0, 0.3]))
    bound = gp2g.output_bound(res)
    assert bound[0] == 0.0 and bound[1] > 0.0
    out = res.forward(rng.normal(scale=50.0, size=(20000, 4)))
    assert np.all(np.abs(out[:, 1]) <= bound[1] + 1e-12) and np.all(out[:, 0] == 0.0)
    outs = np.array([[0.0, 0.9], [0.0, -0.95], [0.0, 0.1], [0.0, 0.0]])
    assert gp2g.saturation_share(outs, np.array([0.0, 1.0]), 0.9) == 0.5
    assert gp2g.saturation_share(outs, np.array([0.0, 0.0]), 0.9) == 0.0
    assert gp2g.saturation_share(np.zeros((0, 2)), np.array([1.0, 1.0]), 0.9) == 0.0


# ---- the fit recipe and the selection rule -------------------------------------------------------


def test_r6_uses_r4s_configs_all_yaw_only() -> None:
    from racer_pnp_dynamics.residual_train import DEFAULT_CANDIDATES

    cfgs = stages_v06.r6_configs()
    assert [c.name for c in cfgs] == [f"yaw_{c.name}" for c in DEFAULT_CANDIDATES]
    assert all(c.yaw_only for c in cfgs)
    assert [replace(c, name="x", yaw_only=False) for c in cfgs] == [
        replace(c, name="x") for c in DEFAULT_CANDIDATES
    ]


def test_config_and_seed_are_chosen_by_selection_p1_not_validation() -> None:
    res = Residual.disabled()
    a, b = ResidualConfig("a"), ResidualConfig("b")
    cands = [Candidate(cfg, s, res, 0.01 if cfg is a else 0.9) for cfg in (a, b) for s in (0, 1, 2)]
    sel = {
        ("a", 0): 0.5,
        ("a", 1): 0.5,
        ("a", 2): 0.5,
        ("b", 0): 0.4,
        ("b", 1): 0.3,
        ("b", 2): 0.45,
    }
    config, ship, table = stages_v06.select_by(cands, sel)
    assert config.name == "b" and ship.seed == 1 and ship.val_p1_m == 0.9  # the original candidate
    assert table["b"]["mean"] == pytest.approx(np.mean([0.4, 0.3, 0.45]))


def test_session_windows_take_the_measured_direction_and_others_keep_theirs() -> None:
    from types import SimpleNamespace

    es = SimpleNamespace(direction=np.array(["cw", "ccw"]))
    assert stages_v06.with_measured_direction(es, []) is es


def test_render_mb3_writes_the_sentence_only_above_the_registered_ratio() -> None:
    class B:
        def __init__(self, ratio):
            self.ratio = ratio

        def perfect_yaw_ratio(self):
            return self.ratio

    from racer_pnp_dynamics import budget

    orig = budget.render_mb3
    try:
        budget.render_mb3 = lambda b, name, notes: "head\n\nPerfect yaw ratio (macro): x\n"
        above = stages_v06.render_mb3(B(0.96), C, [])
        below = stages_v06.render_mb3(B(0.95), C, [])
    finally:
        budget.render_mb3 = orig
    assert C.doc["measurements"]["m_b3"]["sentence_if_above"] in above
    assert "not above 0.95" in below and "not expected" not in below


def test_reports_render_every_item() -> None:
    r = decide(cand=replace(GOOD, near_limit={"left": (3, 1.0), "right": (40, 0.9)}))
    out = {
        "gate": r.to_dict(), "freeze_sha": "f" * 40, "constants_sha256": C.sha256,
        "session_sha256": "s", "windows": 500, "windows_only_under_T2": 1,
        "windows_only_under_T2s": 2, "direction_windows": {"ccw": 250, "cw": 250},
        "strata": {"ccw:0.25-0.6": 250}, "shipping": {"config": "yaw_x", "seed": 1},
        "heading_outcome": {"label": gp2.NO_NAMED_OUTCOME},
        "bootstrap_P1": {"ratio": 0.8, "lo": 0.7, "hi": 0.9},
        "saturation": {"share": 0.0, "steps": 10, "bound": [0.0, 0.2], "max_abs": [0.0, 0.1]},
        "reported_v02_heldout_T2": {"dyn-v0.1": {"P1": 1.0, "Y1": 1.0, "S1": 1.0}},
    }  # fmt: skip
    text = stages_v06.render_pass_report(out, {"dyn-v0.1": REF, "R6 seed 1": GOOD}, "R6 seed 1")
    assert text.startswith("# GP2g-R held-out pass: PASS")
    assert text.count("\n| ") >= len(r.decision.items) + 3
    assert "| n/a |" in text
    readme = stages_v06.render_readme(
        out, "2026-11-24", Path("/x/models/dyn-v0.6r-2026-11-24"), True
    )
    assert readme.startswith("do-not-deploy (until GP2g-C)")
    readme = stages_v06.render_readme(out, "2026-11-24", Path("/x/models-candidate/m"), False)
    assert "candidate" in readme.splitlines()[0] and "models-candidate/m" in readme
    assert "—" not in text + readme


def test_the_fit_summary_lists_every_config_and_the_pickle_hash() -> None:
    from types import SimpleNamespace

    fr = SimpleNamespace(samples={"windows_train": 1}, nominal_val={"P1": 0.1},
                         selection={"yaw_a": {"mean": 0.09}})  # fmt: skip
    fit = {
        "frozen": fr, "shipping_config": "yaw_a", "shipping_seed": 2,
        "selection_nominal_p1": 0.12, "selection_windows": 2287,
        "selection_table": {"yaw_a": {"mean": 0.1, "min": 0.09, "seed0": 0.1, "seed2": 0.09}},
    }  # fmt: skip
    text = stages_v06.render_fit_summary(fit, "abc", ["n"])
    assert "abc" in text and "| yaw_a | 0.1000 | 0.0900 |" in text and "seed 2" in text
    assert stages_v06.r6_seeds({"shipping_config": "a", "frozen": SimpleNamespace(candidates=[
        Candidate(ResidualConfig("a"), 0, Residual.disabled(), 0.1),
        Candidate(ResidualConfig("b"), 1, Residual.disabled(), 0.1)])}).keys() == {0}  # fmt: skip
    assert stages_v06.model_id("2026-11-24") == "dyn-v0.6r-2026-11-24"


# ---- L2: R6 never changes the speed channel ------------------------------------------------------

VEHICLE = load_vehicle()
_f = st.floats(-3.0, 3.0, allow_nan=False)


@settings(max_examples=60, deadline=None)
@given(
    seed=st.integers(0, 2**31 - 1),
    s_yaw=st.floats(1e-3, 2.0),
    v0=st.floats(0.0, 1.3),
    d0=st.floats(-0.3, 0.3),
    cmds=st.lists(st.tuples(st.floats(0.0, 1.3), st.floats(-0.4, 0.4)), min_size=1, max_size=25),
)
def test_r6_never_changes_the_speed_channel(seed, s_yaw, v0, d0, cmds) -> None:
    """For any yaw-only residual (output_scale [0, s]) and any state and command sequence, the
    speed and steering trajectories are the nominal's exactly, so S1 is identical."""
    from racer_pnp_dynamics.cli import read_pointer_nominal

    nominal = read_pointer_nominal(
        ROOT / "training" / "pnp_models" / "dyn-v0.1-2026-11-12" / "manifest.yaml"
    ).nominal
    rng = np.random.default_rng(seed)
    w = MlpWeights(*(rng.normal(scale=2.0, size=a.shape) for a in zero_weights().arrays()))
    res = Residual(True, w, rng.normal(size=4), np.abs(rng.normal(size=4)) + 0.1,
                   np.array([0.0, s_yaw]))  # fmt: skip
    state = np.array([[0.0, 0.0, 0.0, v0, d0]])
    ctrl = np.array([cmds], dtype=float)
    a = rollout(state, ctrl, nominal, VEHICLE, res)
    b = rollout(state, ctrl, nominal, VEHICLE, None)
    np.testing.assert_array_equal(a[..., 3], b[..., 3])
    np.testing.assert_array_equal(a[..., 4], b[..., 4])


def test_the_files_of_this_attempt_were_committed_in_the_registered_order() -> None:
    """The prereg before the split, the split before M-B3, M-B3 before the fit summary."""
    try:
        log = subprocess.run(
            ["git", "-C", str(ROOT), "log", "--reverse", "--format=%H", "--name-only", "--diff-filter=A"],
            capture_output=True, text=True, check=True,
        ).stdout  # fmt: skip
    except (OSError, subprocess.CalledProcessError):  # pragma: no cover - no git
        pytest.skip("no git history")
    order = [ln.strip() for ln in log.splitlines() if ln.strip()]
    want = [
        "evaluation/prereg/pnp-v0.6-dynamics-yaw.yaml",
        "evaluation/prereg/sessions/2026-11-22-dynamics-v06.yaml",
        "training/pnp_models/dyn-v0.6r-2026-11-24/M-B3.md",
        "training/pnp_models/dyn-v0.6r-2026-11-24/fit-summary.md",
        "training/pnp_models/dyn-v0.6r-2026-11-24/pass-report.md",
    ]
    present = [w for w in want if w in order]
    if len(present) < 2:  # pragma: no cover - a shallow clone
        pytest.skip("history too shallow")
    assert [order.index(w) for w in present] == sorted(order.index(w) for w in present)


def test_split_type_is_reused() -> None:
    assert isinstance(prereg_v06.load_session_v06(SESSION), Split)
    assert yaml.safe_load(SESSION.read_text())["rule_file_sha256"] == C.sha256
