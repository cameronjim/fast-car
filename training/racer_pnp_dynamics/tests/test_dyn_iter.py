"""L1 and L2 of the dynamics iteration on dyn-v0.7p's line (dyn-v0.8 and later): the data roles
(session 2 refused, H1 and H2 never fitted on), the version rule (GP2g-R thresholds unchanged,
items 2b and 12 replaced and never above 1.00 times dyn-v0.7p), the rule at pass, at threshold and
over, the cost aware selection, and a narrower 4-H-H-2 residual through training, export and
the Python reference (bounded by its weights for any input)."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
import yaml
from hypothesis import given, settings
from hypothesis import strategies as st
from racer_pnp_dynamics import gp2, gp2f, iter_roles, iter_rule, prereg_v06, proto
from racer_pnp_dynamics.export import read_bundle, write_bundle
from racer_pnp_dynamics.model import Residual, mlp_forward, rollout
from racer_pnp_dynamics.residual_train import (
    Candidate,
    ResidualConfig,
    _init_weights,
    cap_weights,
    residual_layers,
    train_numpy,
)
from racer_pnp_dynamics.strata import assign_strata
from racer_pnp_dynamics.vehicle import load_vehicle, repo_root

ROOT = repo_root()
C06 = prereg_v06.load_constants_v06(ROOT / prereg_v06.V06_RELATIVE)
NAN = float("nan")
FOUR = ("ccw:0.25-0.6", "ccw:0.6-0.85", "cw:0.25-0.6", "cw:0.6-0.85")


# ---- roles ---------------------------------------------------------------------------------------


def _part(bag: str, role: str, run: str = "R0", t0: float = 0.0, t1: float = 10.0):
    return proto.RunPart(bag, run, role, t0, t1)


def test_session_2_is_refused_and_development_parts_are_never_fitted_on() -> None:
    s15, s22, s2 = (
        "2026-11-16T22-45-39_car_teleop",
        iter_roles.GP2G_BAG,
        "2026-11-24T04-36-54_car_teleop",
    )
    parts = [
        _part(s15, "train"), _part(s15, "val", t0=10, t1=12), _part(s15, "dev", t0=14, t1=20),
        _part(s22, "train"), _part(s22, "val", t0=10, t1=12),
        _part(s2, "train"), _part(s2, "val", t0=10, t1=12), _part(s2, "dev", t0=14, t1=20),
    ]  # fmt: skip
    fit = iter_roles.fit_parts(parts)
    assert {(p.bag, p.role) for p in fit} == {
        (s15, "train"),
        (s15, "val"),
        (s22, "train"),
        (s22, "val"),
    }
    assert [(p.bag, p.role) for p in iter_roles.h2_parts(parts)] == [(s15, "dev")]
    with pytest.raises(iter_roles.RoleRefused, match="confirmation only"):
        iter_roles.check_bag(s2)
    iter_roles.check_bag(s15)
    with pytest.raises(iter_roles.RoleRefused, match="confirmation only"):
        iter_roles.check_fit_segments(["x"], ["train"], [s2])
    with pytest.raises(iter_roles.RoleRefused, match="not train or validation"):
        iter_roles.check_fit_segments(["x"], ["heldout"], [s22])
    with pytest.raises(iter_roles.RoleRefused, match="not train or validation"):
        iter_roles.check_fit_segments(["x"], ["dev"], [s15])
    iter_roles.check_fit_segments(["a", "b"], ["train", "val"], [s15, s22])


def test_the_committed_dyn_v07p_split_gives_no_session_2_part_and_no_gp2g_heldout() -> None:
    doc = yaml.safe_load(
        (ROOT / "training/pnp_models/dyn-v0.7p-2026-11-24/dev-split.yaml").read_text()
    )
    parts = [
        proto.RunPart(bag, str(p["run"]), p["role"], float(p["t_from_s"]), float(p["t_to_s"]))
        for bag, e in doc["bags"].items()
        for p in e["parts"]
    ]
    fit = iter_roles.fit_parts(parts)
    assert fit and not any(iter_roles.is_session2(p.bag) for p in fit)
    assert all(p.role in ("train", "val") for p in fit)
    # the 2026-11-22 parts end before the GP2g-R held-out of each run starts (first 55 percent)
    split = prereg_v06.load_session_v06(ROOT / prereg_v06.SESSION_RELATIVE)
    held = {s.name[:2]: s for s in split.bags[iter_roles.GP2G_BAG].segments if s.role == "heldout"}
    for p in (q for q in fit if q.bag == iter_roles.GP2G_BAG):
        assert p.t_to_s <= held["S" + str(int(p.run_id[1:]) + 1)].t_from_s
    h2 = iter_roles.h2_parts(parts)
    assert h2 and all(p.bag.startswith("2026-11-16T") and p.role == "dev" for p in h2)


# ---- the rule ------------------------------------------------------------------------------------


def rule_doc(**gate_over) -> dict:
    g = {
        k: C06.gate[k]
        for k in (*iter_rule.GP2G_UNCHANGED, "item_0_enough_data", "per_direction", "saturation")
    }
    g.update({"max_s1_ratio": 1.00, "max_fast_solve_ratio_to_reference": 1.00})
    g.update(gate_over)
    return {
        "reference": "dyn-v0.7p-2026-11-24",
        "candidate": {"model_id": "dyn-v0.8-test"},
        "gate": g,
        "metrics": dict(C06.metrics),
    }


def rule(**gate_over) -> iter_rule.Rule:
    return iter_rule.parse_rule(yaml.safe_dump(rule_doc(**gate_over)).encode(), C06.gate)


def test_a_rule_that_loosens_a_gp2g_threshold_or_the_replaced_items_is_refused() -> None:
    r = rule()
    assert r.gate["max_p1_ratio"] == 0.95 and r.gate["max_y1_ratio"] == 0.90
    for key, value in (("max_p1_ratio", 0.96), ("max_y1_ratio", 1.0), ("max_cross_ratio", 1.01),
                       ("max_stratum_ratio", 1.10), ("sweep_samples", 1000)):  # fmt: skip
        with pytest.raises(iter_rule.RuleInvalid, match=key):
            rule(**{key: value})
    with pytest.raises(iter_rule.RuleInvalid, match="saturation"):
        rule(saturation={"level": 0.9, "max_heldout_step_share": 0.05})
    with pytest.raises(iter_rule.RuleInvalid, match="1.00 times"):
        rule(max_s1_ratio=1.01)
    with pytest.raises(iter_rule.RuleInvalid, match="1.00 times"):
        rule(max_fast_solve_ratio_to_reference=1.25)
    doc = rule_doc()
    del doc["gate"]
    with pytest.raises(iter_rule.RuleInvalid, match="gate"):
        iter_rule.parse_rule(yaml.safe_dump(doc).encode(), C06.gate)


def test_the_committed_rule_files_parse_against_gp2g() -> None:
    files = sorted((ROOT / iter_rule.RULE_DIR).glob("*.yaml"))
    for f in files:
        r = iter_rule.parse_rule(f.read_bytes(), C06.gate)
        assert r.doc["reference"] == "dyn-v0.7p-2026-11-24"
        assert r.doc["candidate"]["model_id"] == f.stem


def metrics(scale: float = 1.0, **over) -> gp2f.Metrics:
    base = gp2f.Metrics(
        macro={"P1": scale, "A1": scale, "C1": scale, "Y1": scale, "S1": scale},
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
GOOD = metrics(0.8, macro={"P1": 0.8, "A1": 0.8, "C1": 0.8, "Y1": 0.5, "S1": 0.9})


def windows(n_ccw: int = 250, n_cw: int = 250, speeds=(0.4, 0.7)):
    d = np.array(["ccw"] * n_ccw + ["cw"] * n_cw)
    s = np.array([speeds[i % len(speeds)] for i in range(len(d))])
    return d, assign_strata(d, s)


def decide(cand=GOOD, ref=REF, ci=0.9, seeds=None, bad=0, samples=100_000, sat=0.0,
           solve=(2.0, 3.0), win=None):  # fmt: skip
    d, strata = win or windows()
    return iter_rule.decide(
        rule(), d, strata, ref, cand, ci,
        seeds if seeds is not None else {0: 0.8, 1: 0.81, 2: 0.82},
        gp2.SweepResult(samples, 30, bad, 0, 0, 1.3, 0.3), sat, iter_rule.Solve(*solve),
    )  # fmt: skip


def item(result, prefix: str) -> gp2.Item:
    found = [i for i in result.decision.items if i.name.startswith(prefix)]
    assert len(found) == 1, (prefix, [i.name for i in result.decision.items])
    return found[0]


def test_a_version_inside_every_item_passes() -> None:
    r = decide()
    assert r.outcome == iter_rule.PASS and r.decision.passed
    assert r.to_dict()["decision"]["gate"] == iter_rule.GATE


@pytest.mark.parametrize(
    ("field", "limit", "prefix"),
    [
        ("P1", 0.95, "1a"),
        ("Y1", 0.90, "2a"),
        ("S1", 1.00, "2b"),
        ("C1", 1.00, "8a"),
        ("A1", 1.02, "8b"),
    ],
)
def test_each_macro_ratio_at_threshold_and_just_over(field, limit, prefix) -> None:
    for value, ok in ((limit, True), (limit + 1e-6, False), (NAN, False)):
        macro = dict(GOOD.macro, **{field: value})
        r = decide(cand=replace(GOOD, macro=macro))
        assert item(r, prefix).passed is ok, (field, value)
        assert r.outcome == (iter_rule.PASS if ok else iter_rule.FAIL)


def test_bootstrap_seeds_strata_directions_and_the_rest() -> None:
    assert not item(decide(ci=1.0), "1b").passed
    assert item(decide(ci=1.0 - 1e-9), "1b").passed
    assert decide(seeds={0: 0.8, 1: 1.0, 2: 0.9}).outcome == iter_rule.PASS
    assert decide(seeds={0: 0.8, 1: 1.0 + 1e-6, 2: 0.9}).outcome == iter_rule.FAIL
    cand = replace(GOOD, stratum_pos=dict(GOOD.stratum_pos, **{FOUR[0]: 1.05}))
    assert decide(cand=cand).outcome == iter_rule.PASS
    cand = replace(GOOD, stratum_pos=dict(GOOD.stratum_pos, **{FOUR[0]: 1.0501}))
    assert decide(cand=cand).outcome == iter_rule.FAIL
    for d in ("ccw", "cw"):
        for key in ("P1", "Y1"):
            direction = {k: dict(v) for k, v in GOOD.direction.items()}
            direction[d][key] = 1.0 + 1e-6
            assert not item(decide(cand=replace(GOOD, direction=direction)), f"7 {d} {key}").passed
    assert not item(decide(bad=1), "5").passed
    assert not item(decide(samples=99_999), "5").passed
    assert not item(decide(cand=replace(GOOD, yp=1.0001)), "6").passed
    assert not item(decide(cand=replace(GOOD, p1_alt=1.0001)), "10").passed
    cand = replace(
        GOOD, near_limit={"left": (30, 1.0001), "right": (29, 5.0)}, transition=(29, 9.0)
    )
    r = decide(cand=cand)
    assert not item(r, "9 near-limit left").passed
    assert item(r, "9 near-limit right").comparison == gp2f.NA
    assert item(decide(sat=0.01), "11").passed and not item(decide(sat=0.0101), "11").passed


def test_the_solve_item_is_against_dyn_v07p_with_fast_mlp_at_most_equal() -> None:
    assert item(decide(solve=(3.0, 3.0)), "12").passed
    assert not item(decide(solve=(3.0001, 3.0)), "12").passed


def test_too_little_data_is_insufficient() -> None:
    r = decide(win=windows(149, 300))
    assert r.outcome == iter_rule.INSUFFICIENT and not r.decision.passed
    assert decide(win=windows(200, 200, speeds=(0.4,))).outcome == iter_rule.INSUFFICIENT


def _cand(name: str, hidden: int, seed: int, p1: float) -> Candidate:
    res = Residual(True, _init_weights(np.random.default_rng(seed), hidden=hidden), np.zeros(4),
                   np.ones(4), np.ones(2))  # fmt: skip
    return Candidate(ResidualConfig(name, hidden=hidden), seed, res, p1)


def test_cost_aware_selection_takes_the_narrowest_config_within_the_tolerance() -> None:
    cands = [_cand("a32", 32, s, p) for s, p in enumerate((0.100, 0.101, 0.102))]
    cands += [_cand("b16", 16, s, p) for s, p in enumerate((0.1015, 0.1010, 0.1025))]
    cands += [_cand("c8", 8, s, p) for s, p in enumerate((0.110, 0.112, 0.111))]
    cfg, ship, table = iter_rule.select_cost_aware(cands, 0.01)
    assert cfg.name == "b16" and ship.seed == 1  # mean 0.10167 within 1 percent of 0.101
    assert table["a32"]["mean"] == pytest.approx(0.101)
    cfg, _, _ = iter_rule.select_cost_aware(cands, 0.0)
    assert cfg.name == "a32"
    cfg, _, _ = iter_rule.select_cost_aware(cands, 0.2)
    assert cfg.name == "c8"


# ---- a narrower residual --------------------------------------------------------------------------


def test_the_width_reaches_the_numpy_trainer_and_the_layers() -> None:
    assert residual_layers(16) == ((4, 16), (16, 16), (16, 2))
    rng = np.random.default_rng(0)
    x = rng.standard_normal((64, 4))
    y = np.stack([np.sin(x[:, 0]), 0.1 * x[:, 1]], axis=1)
    w = train_numpy(x, y, x, y, seed=0, epochs=3, patience=3, hidden=8)
    assert w.layers() == [[4, 8], [8, 8], [8, 2]]
    with pytest.raises(ValueError, match="hidden"):
        ResidualConfig("x", hidden=0)


@pytest.mark.parametrize("hidden", [8, 16, 24])
def test_a_narrow_bundle_round_trips_and_its_golden_rollouts_hold(tmp_path, hidden) -> None:
    vehicle = load_vehicle()
    from racer_pnp_dynamics.cli import read_pointer_nominal

    nominal = read_pointer_nominal(
        ROOT / "training/pnp_models/dyn-v0.1-2026-11-12/manifest.yaml"
    ).nominal
    w = cap_weights(_init_weights(np.random.default_rng(hidden), out_scale=0.3, hidden=hidden),
                    np.array([2.0, 0.5]))  # fmt: skip
    res = Residual(True, w, np.array([0.6, 0.0, 0.6, 0.0]), np.array([0.3, 0.2, 0.3, 0.2]),
                   np.array([1.0, 1.0]))  # fmt: skip
    out = tmp_path / "b"
    manifest = write_bundle(
        out, model_id="dyn-test", created_utc="2026-11-25T00:00:00Z", git_sha="0" * 40,
        training_config_sha256="0" * 64, vehicle=vehicle, nominal=nominal, residual=res,
        v_max_mps=1.3, data=["test"], metrics={}, golden_residual_rollouts=True,
    )  # fmt: skip
    assert manifest["io"]["residual"]["layers"] == [[4, hidden], [hidden, hidden], [hidden, 2]]
    assert (out / "residual_mlp.f32").stat().st_size == 4 * (hidden * hidden + 8 * hidden + 2)
    b = read_bundle(out)
    assert b.residual.weights.layers() == [[4, hidden], [hidden, hidden], [hidden, 2]]
    np.testing.assert_allclose(b.residual.weights.w2, w.w2, atol=1e-6)
    # the stored (float32) weights give the same rollout as the float64 ones, to float rounding
    cmds = np.tile([[1.0, 0.2]], (30, 1))
    s0 = np.array([0.0, 0.0, 0.0, 0.8, 0.0])
    a = rollout(s0, cmds, nominal, vehicle, b.residual)
    c = rollout(s0, cmds, nominal, vehicle, res)
    np.testing.assert_allclose(a, c, atol=1e-5)


@settings(max_examples=60, deadline=None)
@given(
    hidden=st.sampled_from([8, 16, 24]),
    seed=st.integers(0, 10_000),
    x=st.lists(st.floats(-1e3, 1e3), min_size=4, max_size=4),
)
def test_a_capped_narrow_residual_is_bounded_by_its_weights_for_any_input(hidden, seed, x) -> None:
    cap = np.array([2.0, 0.5])
    w = cap_weights(_init_weights(np.random.default_rng(seed), out_scale=5.0, hidden=hidden), cap)
    out = mlp_forward(w, np.asarray(x, dtype=float))
    assert np.all(np.abs(out) <= cap + 1e-9)
