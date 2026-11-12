"""L1 for pnp_eval.py: the legacy bundle reader and the error on stretches where the planner drove."""

from __future__ import annotations

import numpy as np
import pytest
import yaml
from racer_pnp_dynamics import export, pnp_eval
from racer_pnp_dynamics.bags import GRID_DT_S, Run
from racer_pnp_dynamics.icp_motion import MotionSeries
from racer_pnp_dynamics.model import Residual, rollout


def _legacy_bundle(tmp_path, toy_vehicle, toy_nominal):
    export.write_bundle(
        tmp_path / "b",
        model_id="dyn-old",
        created_utc="2026-11-12T00:00:00Z",
        git_sha="a" * 40,
        training_config_sha256="b" * 64,
        vehicle=toy_vehicle,
        nominal=toy_nominal,
        residual=Residual.disabled(),
        v_max_mps=0.85,
        data=["x"],
        metrics={},
    )
    path = tmp_path / "b" / "manifest.yaml"
    manifest = yaml.safe_load(path.read_text())
    del manifest["io"]["nominal"]["speed_deadband_mps"]
    manifest["bundle_schema_version"] = "1.0.0"
    path.write_text(yaml.safe_dump(manifest, sort_keys=False))
    return tmp_path / "b"


def test_a_schema_1_0_bundle_is_refused_by_the_loader_and_read_as_no_dead_zone_here(
    tmp_path, toy_vehicle, toy_nominal
):
    directory = _legacy_bundle(tmp_path, toy_vehicle, toy_nominal)
    with pytest.raises(export.BundleInvalid, match="speed_deadband_mps"):
        export.read_bundle(directory)
    legacy = pnp_eval.read_legacy_bundle(directory)
    assert legacy.model_id == "dyn-old" and legacy.v_max_mps == 0.85
    assert legacy.nominal == toy_nominal.replace(speed_deadband_mps=0.0)
    assert not legacy.residual.enabled


def test_a_legacy_bundle_with_a_changed_payload_is_refused(tmp_path, toy_vehicle, toy_nominal):
    directory = _legacy_bundle(tmp_path, toy_vehicle, toy_nominal)
    with (directory / "golden_rollouts.csv").open("a") as fh:
        fh.write("\n")
    with pytest.raises(ValueError, match="checksum"):
        pnp_eval.read_legacy_bundle(directory)


def _crawl_run(truth, vehicle):
    """A standing car under a 0.07 m/s command for 6 s (the wheels do not turn: dead zone 0.2),
    then a 0.4 m/s command, simulated with the contract model; plus its ICP motion."""
    n = 50 * 20
    cmd = np.full(n, 0.07)
    cmd[300:] = 0.4
    steer = np.zeros(n)
    state = np.zeros(5)
    states = np.zeros((n, 5))
    for k in range(n):
        states[k] = state
        state = rollout(state, np.array([[cmd[k], steer[k]]]), truth, vehicle, dt=GRID_DT_S)[-1]
    t = 5000.0 + np.arange(n) * GRID_DT_S
    run = Run("crawl", t, cmd, steer, states[:, 3], [])
    idx = np.arange(0, n - 5, 5)
    motion = MotionSeries(
        t_s=t[idx],
        dt_s=np.full(len(idx), 0.1),
        dx_m=states[idx + 5, 0] - states[idx, 0],
        dy_m=states[idx + 5, 1] - states[idx, 1],
        dyaw_rad=states[idx + 5, 2] - states[idx, 2],
        rms_m=np.zeros(len(idx)),
        inlier_share=np.ones(len(idx)),
        ok=np.ones(len(idx), dtype=bool),
    )
    return run, motion


def test_a_model_with_the_dead_zone_predicts_the_crawl_that_one_without_it_does_not(
    toy_vehicle, toy_nominal
):
    truth = toy_nominal.replace(speed_deadband_mps=0.2, speed_tau_s=0.1, command_delay_s=0.0)
    run, motion = _crawl_run(truth, toy_vehicle)
    variants = {
        "dead_zone": (truth, None),
        "none": (truth.replace(speed_deadband_mps=0.0), None),
    }
    out = pnp_eval.compare_on_runs(
        run, motion, t_ref_s=5000.0, variants=variants, vehicle=toy_vehicle, spans_s=[(0.5, 6.0)]
    )
    good = out["dead_zone"]["pooled"]
    bad = out["none"]["pooled"]
    assert good["windows"] > 10 and good["windows"] == bad["windows"]  # standing windows included
    assert good["errors"][1.0]["speed_rmse_mps"] < 1e-6
    assert bad["errors"][1.0]["speed_rmse_mps"] > 0.05
    assert bad["errors"][1.0]["pos_rmse_m"] > good["errors"][1.0]["pos_rmse_m"]
    assert set(out["none"]) == {"pooled", "run0"}


def test_spans_outside_the_run_give_no_windows(toy_vehicle, toy_nominal):
    run, motion = _crawl_run(toy_nominal, toy_vehicle)
    out = pnp_eval.errors_on(
        pnp_eval.segments_between(run, motion, [(100.0, 110.0)], 5000.0),
        toy_nominal,
        None,
        toy_vehicle,
    )
    assert out["windows"] == 0
    assert np.isnan(out["errors"][1.0]["speed_rmse_mps"])
