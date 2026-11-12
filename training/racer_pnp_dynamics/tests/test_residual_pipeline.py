"""L1 for residual.py, evaluate.py, windows.py and the pipeline, on synthetic data with a stand-in
trainer (no torch, no real bags)."""

from __future__ import annotations

import json

import numpy as np
import pytest
import yaml
from racer_pnp_dynamics import evaluate, export, pipeline, residual, windows
from racer_pnp_dynamics.dataset import by_split, make_segments
from racer_pnp_dynamics.model import MlpWeights, Nominal, rollout, zero_weights
from synth import make_run
from test_fit import TRUTH

DELAYS = (0.08, 0.10, 0.12)


def zero_trainer(x, y, xv, yv, seed):
    return zero_weights()


def noisy_trainer(x, y, xv, yv, seed):
    rng = np.random.default_rng(seed)
    return MlpWeights(*(rng.normal(scale=0.2, size=a.shape) for a in zero_weights().arrays()))


def _segments(vehicle, seconds=500.0):
    a, ma = make_run(TRUTH, vehicle, seconds=seconds, seed=11, name="odom-a")
    old, mo = make_run(TRUTH, vehicle, seconds=seconds, seed=12, odom=False, name="old")
    held, mh = make_run(TRUTH, vehicle, seconds=seconds, seed=13, name="odom-b")
    return [*make_segments(a, ma), *make_segments(old, mo), *make_segments(held, mh)]


def test_decide_uses_the_median_over_seeds_of_both_metrics():
    enabled, chosen, med = residual.decide([(0.10, 0.10), (0.06, 0.07), (-0.2, 0.30)])
    assert enabled and med == (0.06, 0.10) and chosen == 1
    enabled, _, _ = residual.decide([(0.10, 0.10), (0.04, 0.2), (0.3, 0.3)])
    assert enabled  # median speed 0.10
    enabled, _, med = residual.decide([(0.10, 0.04), (0.2, 0.01), (0.3, 0.3)])
    assert not enabled and med[1] == 0.04  # yaw median below 5 percent
    enabled, _, _ = residual.decide([(0.049, 0.5), (0.049, 0.5), (0.049, 0.5)])
    assert not enabled
    assert residual.relative_improvement(2.0, 1.5) == 0.25


def test_shipped_residual_is_zero_and_disabled_when_the_rule_fails():
    cand = residual.Residual(True, noisy_trainer(0, 0, 0, 0, 0), np.ones(4), np.ones(4), np.ones(2))
    assert residual.shipped(cand, True) is cand
    off = residual.shipped(cand, False)
    assert not off.enabled and not any(a.any() for a in off.weights.arrays())


def test_samples_vanish_when_the_model_is_the_truth(toy_vehicle):
    segs = _segments(toy_vehicle)
    train = by_split(segs, "train")
    samples = residual.build_samples(train, TRUTH, toy_vehicle)
    assert len(samples) > 1000 and samples.x.shape[1] == 4 and samples.y.shape[1] == 2
    assert np.abs(np.median(samples.y, axis=0)).max() < 0.2
    wrong = TRUTH.replace(speed_gain=0.7, steer_gain_left=0.7, steer_gain_right=0.7)
    bad = residual.build_samples(train, wrong, toy_vehicle)
    assert np.abs(bad.y).mean() > 1.5 * np.abs(samples.y).mean()
    assert len(residual.build_samples([], TRUTH, toy_vehicle)) == 0


def test_train_residual_standardises_with_the_training_set(toy_vehicle):
    seen = {}

    def spy(x, y, xv, yv, seed):
        seen["x_mean"], seen["x_std"], seen["y_std"] = x.mean(0), x.std(0), y.std(0)
        return zero_weights()

    segs = _segments(toy_vehicle, 300.0)
    s = residual.build_samples(by_split(segs, "train"), TRUTH, toy_vehicle)
    v = residual.build_samples(by_split(segs, "val"), TRUTH, toy_vehicle)
    res = residual.train_residual(s, v, 0, spy)
    np.testing.assert_allclose(seen["x_mean"], 0.0, atol=1e-9)
    np.testing.assert_allclose(seen["x_std"], 1.0, atol=1e-9)
    np.testing.assert_allclose(seen["y_std"], 1.0, atol=1e-9)
    assert res.enabled and res.input_std.shape == (4,)


def test_windows_helpers(toy_vehicle):
    run, _ = make_run(TRUTH, toy_vehicle, seconds=60.0, seed=1)
    c = windows.cmd_at(run, np.array([run.t_s[0] - 1.0, run.t_s[10] + 0.005, run.t_s[-1] + 1.0]))
    assert np.isnan(c[0]).all() and np.isnan(c[2]).all()
    np.testing.assert_allclose(c[1], [run.speed_cmd_mps[10], run.steering_cmd_rad[10]])
    w = windows.wheel_at(run, np.array([run.t_s[3] + 0.01]))
    assert w[0] == pytest.approx(0.5 * (run.wheel_speed_mps[3] + run.wheel_speed_mps[4]))
    assert len(windows.speed_windows([], 0.1)) == 0


def test_reconstructed_speed_matches_the_model_rollout(toy_vehicle):
    run, _ = make_run(TRUTH, toy_vehicle, seconds=30.0, seed=2, odom=False)
    v = windows.reconstruct_wheel_speed(run, TRUTH, toy_vehicle)
    lag = 5
    cmd = np.stack([np.nan_to_num(run.speed_cmd_mps), np.zeros(len(v))], axis=1)
    cmd = cmd[np.maximum(np.arange(len(v) - 1) - lag, 0)]
    ref = rollout(np.zeros(5), cmd, TRUTH, toy_vehicle, dt=0.02)
    np.testing.assert_allclose(v, ref[:, 3], atol=1e-9)


def test_evaluate_prefers_the_truth_over_a_wrong_model(toy_vehicle):
    segs = by_split(_segments(toy_vehicle), "heldout", odom_only=True)
    data = evaluate.eval_data(segs, TRUTH.command_delay_s)
    assert len(data) > 100
    good = evaluate.horizon_errors(data, TRUTH, toy_vehicle)
    wrong = TRUTH.replace(speed_gain=0.7, steer_gain_left=0.7, steer_gain_right=1.3)
    bad = evaluate.horizon_errors(data, wrong, toy_vehicle)
    for h in evaluate.HORIZONS_S:
        assert good[h]["speed_rmse_mps"] < bad[h]["speed_rmse_mps"]
        assert good[h]["yaw_rmse_rad"] < bad[h]["yaw_rmse_rad"]
    assert good[1.0]["speed_rmse_mps"] < 0.05
    assert all(v > 0 for v in evaluate.zero_yaw_rmse(data).values())
    empty = evaluate.eval_data([], 0.1)
    assert np.isnan(evaluate.horizon_errors(empty, TRUTH, toy_vehicle)[1.0]["yaw_rmse_rad"])


def test_ground_speed_ratio_bins(toy_vehicle):
    segs = _segments(toy_vehicle, 300.0)
    # synthetic ground speed equals wheel speed, so every bin's ratio is about 1
    rows = evaluate.ground_speed_ratio(segs, 0.1)
    assert rows and all(abs(r["median_ratio"] - 1.0) < 0.05 and r["n"] >= 5 for r in rows)
    assert evaluate.ground_speed_ratio([], 0.1) == []


def test_pipeline_writes_a_valid_bundle_pointer_and_honest_metrics(tmp_path, toy_vehicle):
    segs = _segments(toy_vehicle)
    metrics = pipeline.run_fit_and_export(
        segs,
        toy_vehicle,
        model_id="dyn-v0-test",
        created_utc="2026-11-12T00:00:00Z",
        git_sha="c" * 40,
        bundle_dir=tmp_path / "models" / "dyn-v0-test",
        pointer_dir=tmp_path / "pointer",
        trainer=zero_trainer,
        data_lines=["synthetic"],
        delays=DELAYS,
        tau_grid=(0.1, 0.3),
    )
    manifest = yaml.safe_load((tmp_path / "models" / "dyn-v0-test" / "manifest.yaml").read_text())
    export.validate_manifest(manifest)
    assert metrics["residual_enabled"] is False  # a zero residual cannot win
    assert not manifest["io"]["residual"]["enabled"]
    fitted = metrics["heldout_errors"]["fitted_nominal"]["1.0"]
    placeholder = metrics["heldout_errors"]["placeholder_nominal"]["1.0"]
    assert fitted["speed_rmse_mps"] < placeholder["speed_rmse_mps"]
    assert fitted["yaw_rmse_rad"] < placeholder["yaw_rmse_rad"]
    bundle = export.read_bundle(tmp_path / "models" / "dyn-v0-test")
    assert 0.0 < bundle.v_max_mps <= toy_vehicle.throttle_full_scale_mps
    assert (bundle.v_max_mps / 0.05) == pytest.approx(round(bundle.v_max_mps / 0.05))
    pointer = tmp_path / "pointer"
    assert json.loads((pointer / "metrics.json").read_text())["residual_enabled"] is False
    readme = (pointer / "README.md").read_text()
    assert "dyn-v0-test" in readme and "NOT enabled" in readme and chr(0x2014) not in readme
    assert (pointer / "manifest.yaml").read_bytes() == (
        tmp_path / "models" / "dyn-v0-test" / "manifest.yaml"
    ).read_bytes()
    assert "odom-b" in metrics["seconds_by_bag"]
    assert metrics["seconds_by_bag"]["odom-b"]["heldout"] > 0.0
    assert metrics["seconds_by_bag"]["odom-b"]["val"] == 0.0
    assert metrics["fit"]["yaw_target"] == "path_curvature_ground_speed"
    dz = metrics["dead_zone"]
    assert dz["fitted_mps"] == 0.0 and set(dz) >= {
        "launches_from_rest",
        "steady_state",
        "rolling_below_the_dead_zone",
        "fitted_inside_steady_bracket",
    }
    assert set(metrics["profiles"]) >= {"command_delay_s", "speed_tau_s", "steer_tau_s"}
    assert isinstance(metrics["unidentified"], dict)
    assert metrics["validity"]["factor"] == 1.0 and metrics["validity"]["v_max_mps"] > 0.0
    assert "available" in metrics["steering_lag_check"]
    assert "pnp_runs" not in metrics  # no planner runs given
    rule = metrics["fit"]["accel_rule"]
    assert set(rule) == {
        "rolling_windows",
        "rmse_fitted",
        "rmse_start",
        "kept_start",
        "fitted_value",
    }
    held = metrics["heldout_yaw_rmse_per_interval_rad"]
    assert set(held) == {"ground_driven", "wheel_driven"}
    assert all(np.isfinite(v) and v >= 0.0 for v in held.values())
    radius = metrics["planner_min_radius_m"]
    assert set(radius) == {"left", "right", "kinematic_at_limit"}
    assert all(np.isfinite(v) and v > 0.0 for v in radius.values())
    assert radius["kinematic_at_limit"] == pytest.approx(
        toy_vehicle.wheelbase_m / np.tan(toy_vehicle.steer_max_rad)
    )
    assert "Planner geometry" in readme and "Acceleration limit rule" in readme


def test_pipeline_enabled_path_writes_the_chosen_weights(tmp_path, toy_vehicle, monkeypatch):
    monkeypatch.setattr(pipeline, "decide", lambda imp: (True, 1, (0.1, 0.1)))
    pipeline.run_fit_and_export(
        _segments(toy_vehicle, 300.0),
        toy_vehicle,
        model_id="dyn-v0-on",
        created_utc="2026-11-12T00:00:00Z",
        git_sha="d" * 40,
        bundle_dir=tmp_path / "b",
        pointer_dir=None,
        trainer=noisy_trainer,
        delays=(0.10,),
    )
    bundle = export.read_bundle(tmp_path / "b")
    assert bundle.residual.enabled and bundle.residual.weights.w1.any()
    assert isinstance(bundle.nominal, Nominal)


def test_v_max_rule(toy_vehicle):
    run, motion = make_run(TRUTH, toy_vehicle, seconds=200.0, seed=21)
    train = by_split(make_segments(run, motion), "train")
    v = pipeline.v_max_of(train, toy_vehicle)
    p99 = np.percentile(np.concatenate([s.run.wheel_speed_mps[s.i0 : s.i1] for s in train]), 99)
    # validity is the 99th percentile itself (v0 used 1.5 times it), rounded down to 0.05
    assert v <= p99 and v > p99 - 0.05 and v <= toy_vehicle.throttle_full_scale_mps


def test_fit_vehicle_uses_the_physical_stops(toy_vehicle):
    fv = pipeline.fit_vehicle_of(toy_vehicle)
    assert fv.steer_max_rad == toy_vehicle.physical_lock_left_rad > toy_vehicle.steer_max_rad
    assert fv.steer_min_rad == toy_vehicle.physical_lock_right_rad


def test_pipeline_scores_old_against_new_on_the_planner_runs(tmp_path, toy_vehicle, toy_nominal):
    from racer_pnp_dynamics.pnp_eval import LegacyBundle

    segs = _segments(toy_vehicle, 400.0)
    run, motion = make_run(TRUTH, toy_vehicle, seconds=200.0, seed=41, name="planner-bag")
    pnp = pipeline.PnpRuns(
        run,
        motion,
        float(run.t_s[0]),
        LegacyBundle("dyn-old", toy_nominal, residual.Residual.disabled(), 0.85),
        spans_s=((20.0, 60.0), (100.0, 140.0)),
    )
    metrics = pipeline.run_fit_and_export(
        segs,
        toy_vehicle,
        model_id="dyn-v0-test",
        created_utc="2026-11-12T00:00:00Z",
        git_sha="c" * 40,
        bundle_dir=tmp_path / "b",
        pointer_dir=tmp_path / "pointer",
        trainer=zero_trainer,
        delays=(0.10,),
        tau_grid=(0.1, 0.3),
        pnp=pnp,
    )
    runs = metrics["pnp_runs"]
    assert set(runs) == {"old_dyn-old", "new_nominal", "new_shipped"}
    assert set(runs["new_nominal"]) == {"pooled", "run0", "run1"}
    assert runs["new_nominal"]["pooled"]["windows"] > 20
    new = runs["new_nominal"]["pooled"]["errors"]["1.0"]["speed_rmse_mps"]
    old = runs["old_dyn-old"]["pooled"]["errors"]["1.0"]["speed_rmse_mps"]
    assert new < old  # the fitted model is the truth here, the old one is the placeholder
    readme = (tmp_path / "pointer" / "README.md").read_text()
    assert "Prediction error on the planner runs" in readme and "Dead zone" in readme
    assert "Identification" in readme and chr(0x2014) not in readme
