"""L1 for residual_train.py and the v0.2 pipeline on synthetic data: a known smooth residual is
recovered well enough to pass GP2a, and on data with no residual the gate says "do not enable"
(the false positive check). The trainer here is the numpy one (the CI environment has no torch)."""

from __future__ import annotations

import numpy as np
import pytest
import yaml
from racer_pnp_dynamics import export, gp2, pipeline_v02
from racer_pnp_dynamics import residual_train as rt
from racer_pnp_dynamics.dataset import by_split, make_segments
from racer_pnp_dynamics.evaluate import eval_set
from racer_pnp_dynamics.fit import FitResult
from racer_pnp_dynamics.model import MlpWeights, Residual, rollout, zero_weights
from racer_pnp_dynamics.residual import ResidualSamples
from synth import make_run, make_split
from test_fit import TRUTH

DELAYS = (0.10,)
TAUS = (0.1, 0.3)
SEGS = [
    ("T1", 5.0, 330.0, "train", "ccw"),
    ("T2", 335.0, 660.0, "train", "cw"),
    ("H1", 665.0, 790.0, "heldout", "ccw"),
    ("H2", 795.0, 920.0, "heldout", "cw"),
]
NOISE = {
    "yaw_noise": 0.02,
    "speed_noise": 0.05,
}  # measurement noise that the data must not be fitted to
CFG = rt.ResidualConfig("one_step_np", rt.ONE_STEP, 1e-3, max_epochs=60, patience=8, lr=5e-3)


def known_residual() -> Residual:
    """A smooth, state dependent residual the synthetic truth carries, as a small tanh network
    with no biases (odd in its standardised inputs, zero at their mean) and a gentle output
    scale, so the truth itself stays inside the validity box."""
    rng = np.random.default_rng(0)
    arrays = [
        rng.normal(0.0, 0.4, a.shape) if a.ndim == 2 else np.zeros_like(a)
        for a in zero_weights().arrays()
    ]
    return Residual(
        True, MlpWeights(*arrays), np.array([0.3, 0.0, 0.3, 0.0]), np.array([0.3, 0.2, 0.3, 0.2]),
        np.array([0.2, 0.1]),
    )  # fmt: skip


def _segments(vehicle, truth_residual, seed=31, **noise):
    run, motion = make_run(
        TRUTH, vehicle, seconds=930.0, seed=seed, name="bag", v_hi=1.2, residual=truth_residual,
        exact_model_dt=True, **noise,
    )  # fmt: skip
    split = make_split("bag", SEGS)
    return make_segments(run, motion, split, t_ref_s=run.t_s[0])


def _numpy_trainer(cfg, cap):
    return rt._numpy_trainer(cfg, cap)


@pytest.fixture(scope="module")
def with_residual(toy_vehicle_module):
    return _segments(toy_vehicle_module, known_residual(), **NOISE)


@pytest.fixture(scope="module")
def without_residual(toy_vehicle_module):
    return _segments(toy_vehicle_module, None, seed=32, **NOISE)


@pytest.fixture(scope="module")
def gate_with(with_residual, toy_vehicle_module):
    return _run_gate(with_residual, toy_vehicle_module)


@pytest.fixture(scope="module")
def gate_without(without_residual, toy_vehicle_module):
    return _run_gate(without_residual, toy_vehicle_module)


def _run_gate(segments, vehicle):
    """fit_stage and the held-out evaluation with the nominal FIXED at the known truth (the
    nominal fit has its own recovery tests): what is left for a residual to find is exactly what
    the synthetic data carry on top of the nominal."""
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(pipeline_v02, "fit_nominal", lambda *a, **k: FitResult(nominal=TRUTH))
        frozen = pipeline_v02.fit_stage(
            segments, vehicle, delays=DELAYS, tau_grid=TAUS, configs=(CFG,), seeds=(0, 1, 2),
            one_step_trainer_fn=_numpy_trainer,
        )  # fmt: skip
    fv = pipeline_v02.fit_vehicle_of(vehicle)
    v01 = (
        TRUTH.replace(speed_gain=0.9, steer_gain_left=0.9, steer_gain_right=1.1),
        Residual.disabled(),
    )
    report = pipeline_v02.evaluate_heldout(
        frozen, by_split(segments, "heldout"), fv, vehicle, v01, "v01-standin"
    )
    return frozen, report


def test_cap_weights_bounds_the_output_whatever_the_hidden_units_do():
    rng = np.random.default_rng(0)
    w = MlpWeights(*(rng.normal(size=a.shape) for a in zero_weights().arrays()))
    capped = rt.cap_weights(w, np.array([2.0, 0.5]))
    bound = np.abs(capped.w3).sum(axis=1) + np.abs(capped.b3)
    assert (bound <= np.array([2.0, 0.5]) + 1e-12).all()
    x = rng.normal(size=(500, 4)) * 10
    from racer_pnp_dynamics.model import mlp_forward

    assert (np.abs(mlp_forward(capped, x)) <= np.array([2.0, 0.5]) + 1e-9).all()
    assert rt.cap_weights(w, None) is w
    small = rt.cap_weights(w, np.array([1e9, 1e9]))
    np.testing.assert_allclose(small.w3, w.w3)


def test_config_validation_and_normalisation():
    with pytest.raises(ValueError):
        rt.ResidualConfig("x", mode="bogus")
    with pytest.raises(ValueError):
        rt.ResidualConfig("x", norm="bogus")
    s = ResidualSamples(np.arange(40.0).reshape(10, 4), np.arange(20.0).reshape(10, 2))
    mean, std, scale = rt.normalisation(s, "standardise")
    np.testing.assert_allclose(mean, s.x.mean(axis=0))
    assert (std > 0).all() and (scale > 0).all()
    mean, std, scale = rt.normalisation(s, "none")
    assert not mean.any() and (std == 1).all() and (scale == 1).all()
    assert len({c.name for c in rt.DEFAULT_CANDIDATES}) == len(rt.DEFAULT_CANDIDATES)
    assert any(c.mode == rt.MULTI_STEP for c in rt.DEFAULT_CANDIDATES)


def test_train_numpy_learns_a_smooth_function_and_early_stops_on_validation():
    rng = np.random.default_rng(1)
    x = rng.normal(size=(1500, 4))
    y = np.stack([np.tanh(x[:, 0] * x[:, 2]), 0.5 * x[:, 1] - 0.3 * x[:, 3]], axis=1)
    xv = rng.normal(size=(400, 4))
    yv = np.stack([np.tanh(xv[:, 0] * xv[:, 2]), 0.5 * xv[:, 1] - 0.3 * xv[:, 3]], axis=1)
    w = rt.train_numpy(x, y, xv, yv, seed=0, epochs=80, patience=10, lr=5e-3)
    from racer_pnp_dynamics.model import mlp_forward

    mse = float(np.mean((mlp_forward(w, xv) - yv) ** 2))
    assert mse < 0.35 * float(np.mean(yv**2))
    w2 = rt.train_numpy(x, y, xv, yv, seed=0, epochs=80, patience=10, lr=5e-3)
    np.testing.assert_array_equal(w.w1, w2.w1)  # seeded
    capped = rt.train_numpy(x, y, xv, yv, 0, epochs=5, cap_normalised=np.array([0.2, 0.2]))
    assert (np.abs(capped.w3).sum(axis=1) + np.abs(capped.b3) <= 0.2 + 1e-9).all()


def test_selection_takes_the_best_mean_config_then_its_best_seed():
    def cand(name, seed, p1):
        return rt.Candidate(rt.ResidualConfig(name), seed, Residual.disabled(), p1)

    results = [
        cand("a", 0, 0.30), cand("a", 1, 0.20), cand("a", 2, 0.40),  # mean 0.30
        cand("b", 0, 0.25), cand("b", 1, 0.26), cand("b", 2, 0.27),  # mean 0.26: wins
        cand("c", 0, float("nan")), cand("c", 1, 0.1), cand("c", 2, 0.1),  # nan mean: never
    ]  # fmt: skip
    cfg, ship, table = rt.select_candidate(results)
    assert cfg.name == "b" and ship.seed == 0 and ship.val_p1_m == 0.25
    assert table["a"]["min"] == 0.20 and table["b"]["seed1"] == 0.26


def test_a_known_smooth_residual_is_recovered_and_passes_the_gate(gate_with):
    frozen, report = gate_with
    macro = report["macro"]
    nominal_p1 = macro["dyn-v0.2 nominal"][1.0]["P"]
    shipped_p1 = macro[f"dyn-v0.2 + residual seed {frozen.shipping.seed}"][1.0]["P"]
    assert shipped_p1 < 0.8 * nominal_p1  # the expected gain on held-out
    d = {i["name"][:2]: i for i in report["decision_3_2_residual"]["items"]}
    assert d["1a"]["passed"] and d["1b"]["passed"] and d["5 "]["passed"]
    assert report["residual_ships"] is True
    assert report["bootstrap_p1_ratio_residual_over_nominal"]["hi"] < 1.0
    assert frozen.shipping.seed in (0, 1, 2)
    # the validation split chose the seed, and every candidate was scored on validation only
    assert frozen.shipping.val_p1_m == min(c.val_p1_m for c in frozen.candidates)


def test_no_residual_in_the_data_means_the_gate_says_do_not_enable(gate_without):
    _, report = gate_without
    assert report["residual_ships"] is False
    items = report["decision_3_2_residual"]["items"]
    p1 = next(i for i in items if i["name"].startswith("1a"))
    assert not p1["passed"]  # nothing to gain: P1 is not 5 percent better


def test_pipeline_exports_a_valid_bundle_with_the_residual_enabled_and_golden_at_speed(
    tmp_path, with_residual, toy_vehicle_module, gate_with
):
    frozen, report = gate_with
    fv = pipeline_v02.fit_vehicle_of(toy_vehicle_module)
    metrics = pipeline_v02.build_metrics(
        frozen, report, split=make_split("bag", SEGS), segments=with_residual,
        vehicle=toy_vehicle_module, fvehicle=fv, v01_nominal=TRUTH, v01_id="v01-standin",
        legacy={"segments": 0, "errors": {}}, runs_table=None,
    )  # fmt: skip
    assert report["residual_ships"]
    manifest = pipeline_v02.write_outputs(
        frozen, metrics, toy_vehicle_module, model_id="dyn-v0.2-test", created_utc="2026-11-13T00:00:00Z",
        git_sha="e" * 40, bundle_dir=tmp_path / "b", pointer_dir=tmp_path / "p", data_lines=["synthetic"],
        profile_png=False,
    )  # fmt: skip
    export.validate_manifest(manifest)
    bundle = export.read_bundle(tmp_path / "b")
    assert bundle.residual.enabled and bundle.residual.weights.w1.any()
    golden = export.read_golden(tmp_path / "b" / "golden_rollouts.csv")
    assert len(golden) == 12 + 5  # the standard twelve and five that exercise the residual at speed
    # Python to bundle to Python: the round trip re-simulates the golden file within 1e-9
    assert export.golden_error(golden, bundle.nominal, bundle.residual, toy_vehicle_module) < 1e-6
    assert export.golden_matches(golden, bundle.nominal, bundle.residual, toy_vehicle_module)
    # the residual rollouts differ from the nominal ones: they really exercise the network
    nom = Residual.disabled()
    differs = 0
    for k in range(12, 17):
        d = golden[k]
        ref = rollout(d[0, 2:7], d[:-1, 0:2], bundle.nominal, toy_vehicle_module, nom)
        differs += int(np.abs(ref - d[:, 2:7]).max() > 1e-3)
    assert differs >= 4
    readme = (tmp_path / "p" / "README.md").read_text()
    assert "3.2 GP2a" in readme and chr(0x2014) not in readme and "Merges" in readme
    assert (
        yaml.safe_load((tmp_path / "p" / "manifest.yaml").read_text())["model_id"]
        == "dyn-v0.2-test"
    )
    assert metrics["heldout"]["window_rule"] and metrics["frozen_before_heldout"][
        "shipping_seed"
    ] in (0, 1, 2)
    assert "legacy_heldout" in metrics and metrics["split_sha256"] == "0" * 64


def test_a_refit_that_is_not_better_says_so_on_the_first_line_of_the_readme(
    tmp_path, without_residual, toy_vehicle_module, gate_without
):
    frozen, _ = gate_without
    fv = pipeline_v02.fit_vehicle_of(toy_vehicle_module)
    # the "refit" is deliberately worse than the stand-in for dyn-v0.1 (the truth)
    worse = pipeline_v02.Frozen(
        **{**frozen.__dict__, "nominal": TRUTH.replace(speed_gain=0.8, steer_gain_left=0.7)}
    )
    report = pipeline_v02.evaluate_heldout(
        worse, by_split(without_residual, "heldout"), fv, toy_vehicle_module,
        (TRUTH, Residual.disabled()), "v01-standin",
    )  # fmt: skip
    assert report["refit_ships"] is False
    metrics = pipeline_v02.build_metrics(
        worse, report, split=make_split("bag", SEGS), segments=without_residual,
        vehicle=toy_vehicle_module, fvehicle=fv, v01_nominal=TRUTH, v01_id="v01-standin",
        legacy={}, runs_table=None,
    )  # fmt: skip
    pipeline_v02.write_outputs(
        worse, metrics, toy_vehicle_module, model_id="dyn-v0.2-not-better",
        created_utc="2026-11-13T00:00:00Z", git_sha="f" * 40, bundle_dir=tmp_path / "b",
        pointer_dir=tmp_path / "p", data_lines=[], profile_png=False,
    )  # fmt: skip
    first = (tmp_path / "p" / "README.md").read_text().splitlines()[0]
    assert first == "not better than dyn-v0.1 on held-out, do not deploy"
    bundle = export.read_bundle(tmp_path / "b")
    assert bundle.manifest["model_id"] == "dyn-v0.2-not-better"
    assert bundle.residual.enabled == report["residual_ships"]  # the residual gate stands alone
    assert (
        yaml.safe_load((tmp_path / "p" / "manifest.yaml").read_text())["metrics"]["heldout"][
            "refit_ships"
        ]
        is False
    )


def test_validation_p1_prefers_the_known_residual(with_residual, toy_vehicle_module):
    train = [s for s in by_split(with_residual, "train")]
    val = [s for s in by_split(with_residual, "val")]
    vs = eval_set(val)
    nominal_p1 = rt.validation_p1(TRUTH, None, toy_vehicle_module, vs)
    truth_p1 = rt.validation_p1(TRUTH, known_residual(), toy_vehicle_module, vs)
    assert truth_p1 < 0.5 * nominal_p1
    assert np.isnan(rt.validation_p1(TRUTH, None, toy_vehicle_module, eval_set([])))
    assert train and val


def test_multi_step_training_needs_its_windows(with_residual, toy_vehicle_module):
    cfg = rt.ResidualConfig("ms", rt.MULTI_STEP)
    s = ResidualSamples(np.zeros((300, 4)), np.zeros((300, 2)))
    vs = eval_set(by_split(with_residual, "val"))
    with pytest.raises(ValueError, match="MultiStepData"):
        rt.fit_candidate(cfg, 0, TRUTH, toy_vehicle_module, s, s, vs)
    d = rt.multi_step_data(vs, TRUTH, toy_vehicle_module)
    assert len(d) == len(vs) and d.state0.shape == (len(vs), 5) and d.cmds.shape[1:] == (30, 2)
    assert not d.state0[:, :3].any()  # pose and yaw zero at the window start
    empty = rt.multi_step_data(eval_set([]), TRUTH, toy_vehicle_module)
    assert len(empty) == 0


def test_the_stability_sweep_is_part_of_the_gate_and_clean_for_the_shipped_residual(gate_with):
    _, report = gate_with
    sweep = report["sweep"]
    assert sweep["passed"] and sweep["samples"] == gp2.SWEEP_SAMPLES and sweep["steps"] == 30
