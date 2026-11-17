"""L1 for the contract 2.1 export (export_v21.py, schema 2.1.0) and for TL5 (learned_gm_v21.py):
the bundle validates and round-trips, 2.0.0 and 1.x bundles are unchanged, the C++ loader on main
refuses 2.1.0, the numpy twin of the training loss equals the contract 2.1 rollout."""

from __future__ import annotations

import copy
import os
import tempfile
from pathlib import Path

import numpy as np
import pytest
import yaml
from racer_pnp_dynamics import export, export_v21, learned_gm, learned_gm_v21
from racer_pnp_dynamics import ground_motion as gmo
from racer_pnp_dynamics import ground_motion_v21 as g21
from racer_pnp_dynamics.dataset import by_split, make_segments
from racer_pnp_dynamics.fit_data_v04 import build_fit_windows
from racer_pnp_dynamics.table_fit_v05 import T5Fit
from synth import make_run, make_split
from test_fit import TRUTH
from test_step_v2 import random_learned
from test_step_v21 import C5, yaw_table
from test_step_v21 import motion as motion21
from test_step_v21 import table as rand_table

COMMON = {
    "model_id": "dyn-test-v21",
    "created_utc": "2026-11-16T00:00:00Z",
    "git_sha": "0" * 40,
    "training_config_sha256": "1" * 64,
    "v_max_mps": 1.3,
    "data": ["synthetic"],
    "metrics": {"note": "test"},
}


def _write(path, vehicle, learned=True, seed=2, support=True):
    gm = g21.GroundMotionV21(
        C5,
        rand_table(seed, 0.75, 1.0),
        yaw_table(seed + 1, -0.2, 0.2),
        random_learned(seed, 0.5) if learned else None,
        np.random.default_rng(seed).uniform(size=(5, 6)) > 0.3 if support else None,
    )
    manifest = export_v21.write_bundle_v21(
        path, vehicle=vehicle, nominal=TRUTH, ground=gm, **COMMON
    )
    return gm, manifest


@pytest.mark.parametrize("learned", [True, False])
def test_a_contract_2_1_bundle_validates_round_trips_and_reproduces_its_golden_rollouts(
    tmp_path, toy_vehicle, learned
):
    gm, manifest = _write(tmp_path / "b", toy_vehicle, learned)
    assert manifest["bundle_schema_version"] == "2.1.0" == g21.BUNDLE_SCHEMA_VERSION_V21
    assert "residual" not in manifest["io"]
    gmb = manifest["io"]["ground_motion"]
    assert gmb["yaw_table"]["bounds"] == [-0.25, 0.25] and gmb["alpha_bounds"] == [0.70, 1.05]
    assert gmb["table"]["support"] == gmb["yaw_table"]["support"]
    back = export_v21.read_bundle_v21(tmp_path / "b", C5)
    assert back.nominal == TRUTH and back.v_max_mps == 1.3
    assert np.array_equal(back.ground.table.values, gm.table.values)
    assert np.array_equal(back.ground.yaw_table.values, gm.yaw_table.values)
    assert np.array_equal(back.ground.support, gm.support)
    assert (back.ground.learned is not None) is learned
    golden = export_v21.read_golden_v21(tmp_path / "b")
    assert len(golden) == 17
    assert export_v21.golden_matches_v21(golden, back.nominal, toy_vehicle, back.ground)


def test_a_bundle_without_a_support_record_says_every_node_is_supported(tmp_path, toy_vehicle):
    _, manifest = _write(tmp_path / "b", toy_vehicle, support=False)
    assert all(all(row) for row in manifest["io"]["ground_motion"]["table"]["support"])


def test_a_perturbed_golden_file_is_detected(tmp_path, toy_vehicle):
    gm, _ = _write(tmp_path / "b", toy_vehicle)
    golden = export_v21.read_golden_v21(tmp_path / "b")
    golden[0][5, 2] += 1e-3
    assert not export_v21.golden_matches_v21(golden, TRUTH, toy_vehicle, gm)


def test_a_bad_checksum_a_wrong_version_and_other_bounds_are_refused(tmp_path, toy_vehicle):
    _write(tmp_path / "b", toy_vehicle)
    manifest_path = tmp_path / "b" / export.MANIFEST_FILE
    good = manifest_path.read_text()
    manifest = yaml.safe_load(good)
    manifest["io"]["ground_motion"]["yaw_table"]["bounds"] = [-0.5, 0.5]
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False))
    with pytest.raises(export.BundleInvalid, match="registered"):
        export_v21.read_bundle_v21(tmp_path / "b", C5)
    manifest = yaml.safe_load(good)
    manifest["bundle_schema_version"] = (
        "2.0.0"  # a 2.1 body under a 2.0.0 number: the schema says no
    )
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False))
    with pytest.raises(export.BundleInvalid):
        export_v21.read_bundle_v21(tmp_path / "b", C5)
    manifest_path.write_text(good)
    (tmp_path / "b" / export_v21.GROUND_FILE).write_bytes(b"\x00" * 8)
    with pytest.raises(ValueError, match="checksum"):
        export_v21.read_bundle_v21(tmp_path / "b", C5)


def test_a_contract_2_bundle_is_not_a_contract_2_1_bundle(tmp_path, toy_vehicle):
    from test_export_v2 import _write as write_v2

    write_v2(tmp_path / "b2", toy_vehicle)
    with pytest.raises(export.BundleInvalid, match="not a contract 2.1"):
        export_v21.read_bundle_v21(tmp_path / "b2", C5)


def test_the_support_record_must_have_the_grid_of_the_tables(tmp_path, toy_vehicle):
    gm = g21.GroundMotionV21(
        C5, rand_table(1, 0.8, 1.0), yaw_table(2, -0.1, 0.1), None, np.ones((2, 2), bool)
    )
    with pytest.raises(export.BundleInvalid, match="support record"):
        export_v21.write_bundle_v21(
            tmp_path / "x", vehicle=toy_vehicle, nominal=TRUTH, ground=gm, **COMMON
        )
    assert not (tmp_path / "x").exists()


def test_a_yaw_table_with_a_centre_column_off_zero_or_out_of_bounds_is_refused(
    tmp_path, toy_vehicle
):
    for table in (yaw_table(1, -0.2, 0.2, centre_zero=False), yaw_table(2, -0.6, 0.6)):
        gm = g21.GroundMotionV21(C5, rand_table(1, 0.8, 1.0), table)
        with pytest.raises(export.BundleInvalid):
            export_v21.write_bundle_v21(
                tmp_path / "y", vehicle=toy_vehicle, nominal=TRUTH, ground=gm, **COMMON
            )
    assert not (tmp_path / "y").exists()


def test_v_max_above_the_full_scale_speed_is_refused(tmp_path, toy_vehicle):
    gm = g21.GroundMotionV21(C5, rand_table(1, 0.8, 1.0), yaw_table(2, -0.1, 0.1))
    kw = {**COMMON, "v_max_mps": 99.0}
    with pytest.raises(export.BundleInvalid, match="v_max_mps"):
        export_v21.write_bundle_v21(
            tmp_path / "x", vehicle=toy_vehicle, nominal=TRUTH, ground=gm, **kw
        )
    assert not (tmp_path / "x").exists()


def test_the_first_readme_line_says_do_not_deploy_and_that_no_planner_can_load_it():
    assert (
        export_v21.FIRST_README_LINE
        == "contract 2.1: no planner on main can load this; candidate only"
    )
    assert export_v21.readme_first_line() == (
        "do-not-deploy; contract 2.1: no planner on main can load this; candidate only"
    )


def test_1_x_and_2_0_0_writers_are_untouched(tmp_path, toy_vehicle):
    from test_export_v2 import _write as write_v2

    assert export.BUNDLE_SCHEMA_VERSION == "1.1.0"
    _, manifest = write_v2(tmp_path / "b2", toy_vehicle)
    assert manifest["bundle_schema_version"] == gmo.BUNDLE_SCHEMA_VERSION == "2.0.0"
    export.validate_manifest(manifest)


def test_todays_planner_refuses_a_contract_2_1_bundle(tmp_path, toy_vehicle):
    """The C++ loader on main (planner_replay_cli) must answer BundleSchemaVersionMismatch. Runs
    where the binary exists (a native one, $PLANNER_REPLAY_CLI, or the Linux one in docker through
    $PLANNER_REPLAY_DOCKER_CLI); a missing binary is a skip unless PNP_REPLAY_REQUIRE_CLI is set.
    tools/tests/test_pnp_replay_schema21.py runs the same check in the ROS job."""
    from pnp_replay import planner_stream

    try:
        cli = planner_stream.find_cli()
    except planner_stream.ReplayError as exc:
        if os.environ.get("PNP_REPLAY_REQUIRE_CLI"):
            raise
        pytest.skip(f"planner_replay_cli not available: {exc}")
    shared = os.environ.get("PNP_TEST_SHARED_TMP")
    if shared:
        Path(shared).mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=shared or tmp_path) as work:
        bundle = Path(work) / "b"
        _write(bundle, toy_vehicle)
        with pytest.raises(planner_stream.ReplayError, match="BundleSchemaVersionMismatch"):
            planner_stream.describe_cli(cli, bundle, 1.3, {})


# ---- TL5 ---------------------------------------------------------------------------------------


@pytest.fixture(scope="module")
def fit_windows(toy_vehicle_module):
    run, mot = make_run(
        TRUTH, toy_vehicle_module, seconds=300.0, seed=9, name="t", v_hi=1.0, exact_model_dt=True
    )
    split = make_split("t", [("A", 5.0, 140.0, "train", "ccw"), ("B", 150.0, 290.0, "train", "cw")])
    segs = make_segments(run, mot, split, t_ref_s=run.t_s[0])
    train = [s for s in by_split(segs, "train", odom_only=True)]
    return build_fit_windows(train, TRUTH, toy_vehicle_module)


def test_the_numpy_twin_of_the_training_loss_is_the_contract_2_1_rollout(
    fit_windows, toy_vehicle_module
):
    v = toy_vehicle_module
    gm = motion21(5, learned=True, scale=0.6)
    tracks = learned_gm_v21.build_tracks_v21(fit_windows, TRUTH, v, gm.table, gm.yaw_table, C5)
    pred = learned_gm.predict_from_tracks(tracks, C5, gm.learned)  # (E, 3 horizons, x y yaw)
    states = g21.predict_v21(fit_windows.es, TRUTH, v, gm)
    for k, h in enumerate((0.5, 1.0, 1.5)):
        step = round(h / 0.05)
        np.testing.assert_allclose(pred[:, k, 0], states[:, step, 0], atol=1e-9)
        np.testing.assert_allclose(pred[:, k, 1], states[:, step, 1], atol=1e-9)
        np.testing.assert_allclose(pred[:, k, 2], states[:, step, 2], atol=1e-9)


def test_with_a_zero_yaw_table_the_tracks_are_the_v0_4_tracks(fit_windows, toy_vehicle_module):
    v = toy_vehicle_module
    table = rand_table(3, 0.8, 1.0)
    new = learned_gm_v21.build_tracks_v21(
        fit_windows, TRUTH, v, table, g21.zero_yaw_table(table), C5
    )
    old = learned_gm.build_tracks(fit_windows, TRUTH, v, table)
    np.testing.assert_array_equal(new.rate, old.rate)
    np.testing.assert_array_equal(new.table_alpha, old.table_alpha)


def test_the_two_configurations_come_from_the_yaml_and_fit_tl5_selects_on_the_selection_set(
    fit_windows, toy_vehicle_module
):
    v = toy_vehicle_module
    cfgs = learned_gm_v21.configs_of(C5)
    assert [c.weight_decay for c in cfgs] == [1e-3, 1e-2] and len({c.name for c in cfgs}) == 2
    calls = []

    def stub(cfg, seed, c, tr, va, scales):
        calls.append((cfg.name, seed))
        return random_learned(seed + (10 if cfg.weight_decay > 5e-3 else 0), 0.2), 7

    alpha = rand_table(3, 0.85, 1.0)
    t5 = T5Fit(
        C5, alpha, yaw_table(4, -0.1, 0.1), 0.1, 0.01, np.ones((5, 6), int) * 100,
        np.ones((5, 6), bool), np.arange(30), np.zeros((5, 6), bool), [], [], {}, {},
    )  # fmt: skip
    cands, ship, table = learned_gm_v21.fit_tl5(
        fit_windows, fit_windows, fit_windows.es, TRUTH, v, C5, t5, trainer=stub
    )
    assert len(cands) == 6 and sorted(set(calls)) == sorted(calls)
    assert {s for _, s in calls} == {0, 1, 2}
    assert ship in cands and set(table) == {c.name for c in cfgs}
    best_cfg = min(table, key=lambda n: table[n]["mean"])
    assert ship.config.name == best_cfg
    assert ship.val_p1_m == min(c.val_p1_m for c in cands if c.config.name == best_cfg)
    assert all(np.isfinite(c.val_p1_m) and c.epochs == 7 for c in cands)


def test_a_manifest_of_the_schema_is_what_the_writer_produces(tmp_path, toy_vehicle):
    _, manifest = _write(tmp_path / "b", toy_vehicle)
    as_v20 = copy.deepcopy(manifest)
    as_v20["bundle_schema_version"] = "2.0.0"
    with pytest.raises(export.BundleInvalid):
        export.validate_manifest(as_v20)
