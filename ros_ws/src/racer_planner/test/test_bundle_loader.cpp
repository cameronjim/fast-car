// One test per refusal of the contract table that applies to a dynamics bundle, plus the good
// fixture. The fixture bundle comes from test/fixtures/make_fixture.py, a second implementation
// of the model, so a pass also proves the C++ step reproduces it.
#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>

#include "racer_planner/bundle_loader.hpp"
#include "test_support.hpp"

using namespace racer_planner;
using test::BundleCopy;

namespace {

void expect_refusal(const LoadResult& r, RefusalCode code) {
  ASSERT_FALSE(r.ok()) << "bundle was accepted";
  ASSERT_TRUE(r.refusal.has_value());
  EXPECT_EQ(r.refusal->code, code)
      << refusal_code_name(r.refusal->code) << ": " << r.refusal->message;
  EXPECT_FALSE(r.refusal->message.empty());
}

LoadResult load(const BundleCopy& b, double v = 0.5) {
  return load_dynamics_bundle(b.dir(), test::fixture_geometry(), v);
}

}  // namespace

TEST(BundleLoader, GoodFixtureLoadsAndReproducesTheGoldenRollouts) {
  const auto r = load_dynamics_bundle(test::fixture_dir(), test::fixture_geometry(), 0.5);
  ASSERT_TRUE(r.ok()) << (r.refusal ? r.refusal->message : "");
  const DynamicsBundle& b = *r.bundle;
  EXPECT_EQ(b.model_id, "dyn-fixture-v0-2026-11-12");
  EXPECT_TRUE(b.params.residual.enabled);
  EXPECT_EQ(static_cast<int>(b.params.residual.weights.size()), kMlpWeightCount);
  EXPECT_DOUBLE_EQ(b.params.nominal.steer_gain_left, 1.1);
  EXPECT_DOUBLE_EQ(b.params.nominal.command_delay_s, 0.08);
  EXPECT_EQ(b.payload.size(), 2u);
  EXPECT_LT(b.golden_max_abs_error, kGoldenTolerance);
  // The residual is live in the fixture, so a match is not just the nominal model agreeing.
  EXPECT_GT(b.golden_max_abs_error, 0.0);
}

TEST(BundleLoader, ManifestMissingNoFile) {
  BundleCopy b;
  test::fs::remove(b.path("manifest.yaml"));
  expect_refusal(load(b), RefusalCode::ManifestMissing);
}

TEST(BundleLoader, ManifestMissingNoDirectory) {
  expect_refusal(
      load_dynamics_bundle("/nonexistent/racer_planner/bundle", test::fixture_geometry(), 0.5),
      RefusalCode::ManifestMissing);
}

TEST(BundleLoader, ManifestMissingWhenItDoesNotParse) {
  BundleCopy b;
  test::write_text(b.path("manifest.yaml"), "bundle_schema_version: [unclosed\n  : : :\n");
  expect_refusal(load(b), RefusalCode::ManifestMissing);
}

TEST(BundleLoader, ManifestInvalidMissingField) {
  BundleCopy b;
  b.edit_manifest("metrics: {}", "");
  expect_refusal(load(b), RefusalCode::ManifestInvalid);
}

TEST(BundleLoader, ManifestInvalidExtraTopLevelField) {
  BundleCopy b;
  test::write_text(b.path("manifest.yaml"),
                   test::read_text(b.path("manifest.yaml")) + "extra_key: 1\n");
  expect_refusal(load(b), RefusalCode::ManifestInvalid);
}

TEST(BundleLoader, ManifestInvalidWrongType) {
  BundleCopy b;
  b.edit_manifest("data: [\"racer_planner test fixture, synthetic, seed 1, not fitted\"]",
                  "data: 3");
  expect_refusal(load(b), RefusalCode::ManifestInvalid);
}

TEST(BundleLoader, ManifestInvalidWhenTheRootIsNotAMapping) {
  BundleCopy b;
  test::write_text(b.path("manifest.yaml"), "- a\n- b\n");
  expect_refusal(load(b), RefusalCode::ManifestInvalid);
}

TEST(BundleLoader, BundleSchemaVersionMismatch) {
  BundleCopy b;
  b.edit_manifest("bundle_schema_version: \"1.1.0\"", "bundle_schema_version: \"3.0.0\"");
  expect_refusal(load(b), RefusalCode::BundleSchemaVersionMismatch);
}

// Brief 43: major 2 is step contract 2 now. A 1.x manifest relabelled 2.0.0 still carries the
// additive residual block, which a contract 2 bundle must not have (it was a version mismatch
// before the planner implemented contract 2; the expectation changed on purpose).
TEST(BundleLoader, ContractOneManifestRelabelledAsContractTwoIsRefusedForItsResidual) {
  BundleCopy b;
  b.edit_manifest("bundle_schema_version: \"1.1.0\"", "bundle_schema_version: \"2.0.0\"");
  expect_refusal(load(b), RefusalCode::IoContractMismatch);
}

TEST(BundleLoader, MinorVersionBumpIsAccepted) {
  BundleCopy b;
  b.edit_manifest("bundle_schema_version: \"1.1.0\"", "bundle_schema_version: \"1.3.0\"");
  EXPECT_TRUE(load(b).ok());
}

TEST(BundleLoader, KindMismatchWithATrackPerceptionManifest) {
  BundleCopy b;
  b.edit_manifest("kind: dynamics", "kind: track_perception");
  expect_refusal(load(b), RefusalCode::KindMismatch);
}

TEST(BundleLoader, PayloadMissing) {
  BundleCopy b;
  test::fs::remove(b.path("golden_rollouts.csv"));
  expect_refusal(load(b), RefusalCode::PayloadMissing);
}

TEST(BundleLoader, ChecksumMismatch) {
  BundleCopy b;
  test::write_text(b.path("residual_mlp.f32"), test::read_text(b.path("residual_mlp.f32")) + "x");
  expect_refusal(load(b), RefusalCode::ChecksumMismatch);
}

TEST(BundleLoader, VehicleParamsMismatchSchemaVersion) {
  BundleCopy b;
  b.edit_manifest("schema_version: \"0.17.0\"", "schema_version: \"0.13.0\"");
  expect_refusal(load(b), RefusalCode::VehicleParamsMismatch);
}

TEST(BundleLoader, VehicleParamsMismatchSysidSession) {
  BundleCopy b;
  b.edit_manifest("sysid_session_id: \"none-preliminary\"",
                  "sysid_session_id: \"sysid-2026-11-12\"");
  expect_refusal(load(b), RefusalCode::VehicleParamsMismatch);
}

TEST(BundleLoader, IoContractMismatchStateOrder) {
  BundleCopy b;
  b.edit_manifest("state: [x_m, y_m, yaw_rad, v_mps, delta_rad]",
                  "state: [y_m, x_m, yaw_rad, v_mps, delta_rad]");
  expect_refusal(load(b), RefusalCode::IoContractMismatch);
}

TEST(BundleLoader, IoContractMismatchControlOrder) {
  BundleCopy b;
  b.edit_manifest("control: [speed_cmd_mps, steering_cmd_rad]",
                  "control: [steering_cmd_rad, speed_cmd_mps]");
  expect_refusal(load(b), RefusalCode::IoContractMismatch);
}

TEST(BundleLoader, IoContractMismatchDt) {
  BundleCopy b;
  b.edit_manifest("dt_s: 0.05", "dt_s: 0.5");
  expect_refusal(load(b), RefusalCode::IoContractMismatch);
}

TEST(BundleLoader, IoContractMismatchLayerShapes) {
  BundleCopy b;
  b.edit_manifest("layers: [[4, 32], [32, 32], [32, 2]]", "layers: [[4, 16], [16, 16], [16, 2]]");
  expect_refusal(load(b), RefusalCode::IoContractMismatch);
}

// dyn-v0.8 on: a 4 -> 16 -> 16 -> 2 residual (the second implementation in make_fixture.py)
// loads and reproduces its golden rollouts in every MLP mode; the loaded width is 16.
TEST(BundleLoader, NarrowResidualFixtureLoadsInEveryMlpMode) {
  const std::string dir = test::fixture_h16_dir();
  const auto geometry = test::fixture_geometry(dir);
  const auto exact = load_dynamics_bundle(dir, geometry, 0.5, false, false);
  ASSERT_TRUE(exact.ok()) << (exact.refusal ? exact.refusal->message : "");
  EXPECT_EQ(exact.bundle->params.residual.hidden, 16);
  EXPECT_EQ(static_cast<int>(exact.bundle->params.residual.weights.size()), mlp_weight_count(16));
  EXPECT_TRUE(exact.bundle->params.residual.enabled);
  EXPECT_GT(exact.bundle->golden_max_abs_error, 0.0);
  EXPECT_LT(exact.bundle->golden_max_abs_error, kGoldenTolerance);
  for (const bool fast : {false, true}) {
    for (const bool f32 : {false, true}) {
      const auto r = load_dynamics_bundle(dir, geometry, 0.5, fast, f32);
      ASSERT_TRUE(r.ok()) << fast << f32 << ": " << (r.refusal ? r.refusal->message : "");
      EXPECT_LT(r.bundle->golden_max_abs_error, kGoldenTolerance);
      EXPECT_NEAR(r.bundle->golden_max_abs_error, exact.bundle->golden_max_abs_error,
                  f32 ? 1e-6 : 1e-9);
    }
  }
  // the 16 wide weights under a 32 wide manifest: refused for the file size
  BundleCopy b(dir);
  b.edit_manifest("layers: [[4, 16], [16, 16], [16, 2]]", "layers: [[4, 32], [32, 32], [32, 2]]");
  expect_refusal(load_dynamics_bundle(b.dir(), geometry, 0.5), RefusalCode::IoContractMismatch);
}

// Bundle 1.2 (dyn-v0.9): a residual with a ground scale (make_fixture.py --ground-scale 0.94)
// loads in every MLP mode and carries the scale; the golden rollouts depend on it (removing the
// key fails them); a scale outside [0.60, 1.05] is refused.
TEST(BundleLoader, GroundScaleFixtureLoadsAndItsGoldenRolloutsNeedTheScale) {
  const std::string dir = test::fixture_h16_gs_dir();
  const auto geometry = test::fixture_geometry(dir);
  for (const bool fast : {false, true}) {
    for (const bool f32 : {false, true}) {
      const auto r = load_dynamics_bundle(dir, geometry, 0.5, fast, f32);
      ASSERT_TRUE(r.ok()) << fast << f32 << ": " << (r.refusal ? r.refusal->message : "");
      EXPECT_DOUBLE_EQ(r.bundle->params.residual.ground_scale, 0.94);
      EXPECT_EQ(r.bundle->params.residual.hidden, 16);
      EXPECT_LT(r.bundle->golden_max_abs_error, kGoldenTolerance);
    }
  }
  {
    BundleCopy b(dir);
    b.edit_manifest("\n    ground_scale: 0.94", "");
    expect_refusal(load_dynamics_bundle(b.dir(), geometry, 0.5),
                   RefusalCode::GoldenRolloutMismatch);
  }
  for (const char* bad : {"0.59", "1.06", ".nan"}) {
    BundleCopy b(dir);
    b.edit_manifest("ground_scale: 0.94", std::string("ground_scale: ") + bad);
    expect_refusal(load_dynamics_bundle(b.dir(), geometry, 0.5), RefusalCode::IoContractMismatch);
  }
}

// A width the planner does not implement is refused for its shape, even with a weight file of
// the size that width implies (12: 4*12+12+144+12+24+2 = 242 floats).
TEST(BundleLoader, IoContractMismatchUnsupportedResidualWidth) {
  BundleCopy b;
  b.edit_manifest("layers: [[4, 32], [32, 32], [32, 2]]", "layers: [[4, 12], [12, 12], [12, 2]]");
  b.rewrite_payload("residual_mlp.f32", std::string(4 * mlp_weight_count(12), '\0'));
  const auto r = load(b);
  expect_refusal(r, RefusalCode::IoContractMismatch);
  EXPECT_NE(r.refusal->message.find("hidden width"), std::string::npos) << r.refusal->message;
  // inconsistent shapes (H changes between layers) are refused too
  BundleCopy c;
  c.edit_manifest("layers: [[4, 32], [32, 32], [32, 2]]", "layers: [[4, 16], [32, 32], [32, 2]]");
  expect_refusal(load(c), RefusalCode::IoContractMismatch);
}

TEST(BundleLoader, IoContractMismatchWeightFileSize) {
  BundleCopy b;
  b.rewrite_payload("residual_mlp.f32", std::string(100, '\0'));
  expect_refusal(load(b), RefusalCode::IoContractMismatch);
}

TEST(BundleLoader, IoContractMismatchActivation) {
  BundleCopy b;
  b.edit_manifest("activation: tanh", "activation: relu");
  expect_refusal(load(b), RefusalCode::IoContractMismatch);
}

TEST(BundleLoader, IoContractMismatchMissingNominalField) {
  BundleCopy b;
  b.edit_manifest("    command_delay_s: 0.08\n", "");
  expect_refusal(load(b), RefusalCode::IoContractMismatch);
}

// Schema 1.1.0: io.nominal.speed_deadband_mps is required, whatever the version says. No default.
TEST(BundleLoader, IoContractMismatchMissingDeadZoneFieldNamesIt) {
  BundleCopy b;
  b.edit_manifest("    speed_deadband_mps: 0.0\n", "");
  const auto r = load(b);
  expect_refusal(r, RefusalCode::IoContractMismatch);
  EXPECT_NE(r.refusal->message.find("speed_deadband_mps"), std::string::npos) << r.refusal->message;
}

TEST(BundleLoader, MissingDeadZoneFieldIsRefusedEvenAtAnOldSchemaVersion) {
  BundleCopy b;
  b.edit_manifest("    speed_deadband_mps: 0.0\n", "");
  b.edit_manifest("bundle_schema_version: \"1.1.0\"", "bundle_schema_version: \"1.0.0\"");
  const auto r = load(b);
  expect_refusal(r, RefusalCode::IoContractMismatch);
  EXPECT_NE(r.refusal->message.find("speed_deadband_mps"), std::string::npos) << r.refusal->message;
}

TEST(BundleLoader, IoContractMismatchDeadZoneOutOfRange) {
  for (const char* bad : {"-0.1", "1.5"}) {
    BundleCopy b;
    b.edit_manifest("speed_deadband_mps: 0.0", std::string("speed_deadband_mps: ") + bad);
    expect_refusal(load(b), RefusalCode::IoContractMismatch);
  }
}

TEST(BundleLoader, ADeadZoneTheGoldenRolloutsWereNotMadeWithIsRefused) {
  BundleCopy b;
  b.edit_manifest("speed_deadband_mps: 0.0", "speed_deadband_mps: 0.6");
  expect_refusal(load(b), RefusalCode::GoldenRolloutMismatch);
}

TEST(BundleLoader, GoldenRolloutMismatchWithOneAlteredNumber) {
  BundleCopy b;
  std::string csv = test::read_text(b.path("golden_rollouts.csv"));
  // Row "0,1,..." x_m is the 5th field: change one digit group of the first rollout's step 1.
  const auto row = csv.find("\n0,1,");
  ASSERT_NE(row, std::string::npos);
  std::size_t pos = row + 1;
  for (int commas = 0; commas < 4; ++commas) pos = csv.find(',', pos) + 1;
  const auto end = csv.find(',', pos);
  csv.replace(pos, end - pos, "0.5");  // x_m of step 1, far outside the 1e-4 tolerance
  b.rewrite_payload("golden_rollouts.csv", csv);
  expect_refusal(load(b), RefusalCode::GoldenRolloutMismatch);
}

TEST(BundleLoader, GoldenRolloutMismatchWhenTheModelDiffersFromTheOneThatMadeIt) {
  BundleCopy b;
  b.edit_manifest("steer_gain_left: 1.1", "steer_gain_left: 1.2");
  expect_refusal(load(b), RefusalCode::GoldenRolloutMismatch);
}

TEST(BundleLoader, GoldenRolloutMismatchBadHeader) {
  BundleCopy b;
  std::string csv = test::read_text(b.path("golden_rollouts.csv"));
  csv.replace(0, 7, "rolloot");
  b.rewrite_payload("golden_rollouts.csv", csv);
  expect_refusal(load(b), RefusalCode::GoldenRolloutMismatch);
}

TEST(BundleLoader, GoldenRolloutMismatchTooFewRollouts) {
  BundleCopy b;
  std::string csv = test::read_text(b.path("golden_rollouts.csv"));
  const auto cut = csv.find("\n3,0,");
  ASSERT_NE(cut, std::string::npos);
  b.rewrite_payload("golden_rollouts.csv", csv.substr(0, cut + 1));
  expect_refusal(load(b), RefusalCode::GoldenRolloutMismatch);
}

TEST(BundleLoader, ValidityExceeded) {
  BundleCopy b;
  expect_refusal(load(b, 0.95), RefusalCode::ValidityExceeded);  // fixture validity is 0.9
  EXPECT_TRUE(load(b, 0.9).ok());
}

TEST(BundleLoader, ValidityExceededForNaNRequest) {
  BundleCopy b;
  expect_refusal(load(b, std::nan("")), RefusalCode::ValidityExceeded);
}

TEST(BundleLoader, RefusalOrderFollowsTheTable) {
  // A bundle with both a wrong kind and a wrong checksum reports the earlier row.
  BundleCopy b;
  b.edit_manifest("kind: dynamics", "kind: adaptation");
  test::write_text(b.path("residual_mlp.f32"), "garbage");
  expect_refusal(load(b), RefusalCode::KindMismatch);
}

TEST(BundleLoader, RefusalCodeNamesAreTheContractNames) {
  EXPECT_STREQ(refusal_code_name(RefusalCode::ManifestMissing), "ManifestMissing");
  EXPECT_STREQ(refusal_code_name(RefusalCode::ManifestInvalid), "ManifestInvalid");
  EXPECT_STREQ(refusal_code_name(RefusalCode::BundleSchemaVersionMismatch),
               "BundleSchemaVersionMismatch");
  EXPECT_STREQ(refusal_code_name(RefusalCode::KindMismatch), "KindMismatch");
  EXPECT_STREQ(refusal_code_name(RefusalCode::PayloadMissing), "PayloadMissing");
  EXPECT_STREQ(refusal_code_name(RefusalCode::ChecksumMismatch), "ChecksumMismatch");
  EXPECT_STREQ(refusal_code_name(RefusalCode::VehicleParamsMismatch), "VehicleParamsMismatch");
  EXPECT_STREQ(refusal_code_name(RefusalCode::IoContractMismatch), "IoContractMismatch");
  EXPECT_STREQ(refusal_code_name(RefusalCode::GoldenRolloutMismatch), "GoldenRolloutMismatch");
  EXPECT_STREQ(refusal_code_name(RefusalCode::ValidityExceeded), "ValidityExceeded");
}

// The nominal fixture from the dynamics workstream (brief 03), loaded with the live vehicle
// params exactly as the node does.
TEST(BundleLoader, DynNominalV0FromTheDynamicsWorkstreamLoads) {
  const std::string dir =
      std::string(RACER_PLANNER_REPO_ROOT) + "/config/pnp/fixtures/dyn-nominal-v0";
  const auto r = load_dynamics_bundle(dir, test::binding_geometry(), 0.5);
  ASSERT_TRUE(r.ok()) << (r.refusal ? std::string(refusal_code_name(r.refusal->code)) + ": " +
                                          r.refusal->message
                                    : "");
  EXPECT_EQ(r.bundle->model_id, "dyn-nominal-v0");
  EXPECT_FALSE(r.bundle->params.residual.enabled);
  EXPECT_LT(r.bundle->golden_max_abs_error, kGoldenTolerance);
}

// The fitted nominal of the car (brief 14), residual disabled, with a dead zone and golden
// rollouts that cross it. Loading it is the proof that this step reproduces model.py there.
TEST(BundleLoader, DynCar20261112WithADeadZoneLoadsAndReproducesTheGoldenRollouts) {
  const std::string dir =
      std::string(RACER_PLANNER_REPO_ROOT) + "/config/pnp/fixtures/dyn-car-2026-11-12";
  const auto r = load_dynamics_bundle(dir, test::binding_geometry(), 0.5);
  ASSERT_TRUE(r.ok()) << (r.refusal ? std::string(refusal_code_name(r.refusal->code)) + ": " +
                                          r.refusal->message
                                    : "");
  EXPECT_EQ(r.bundle->model_id, "dyn-car-2026-11-12");
  EXPECT_FALSE(r.bundle->params.residual.enabled);
  EXPECT_GT(r.bundle->params.nominal.speed_deadband_mps, 0.0);
  EXPECT_LT(r.bundle->golden_max_abs_error, kGoldenTolerance);
  EXPECT_GT(r.bundle->params.v_max_mps, 0.94);
  // The golden rollouts include three that cross the dead zone: the same parameters with the
  // dead zone removed no longer reproduce them, so a match is not only the rest of the model.
  DynamicsParams no_dead_zone = r.bundle->params;
  no_dead_zone.nominal.speed_deadband_mps = 0.0;
  const auto mismatch = verify_golden_rollouts(no_dead_zone, test::binding_geometry(),
                                               dir + "/golden_rollouts.csv", nullptr);
  ASSERT_TRUE(mismatch.has_value());
  EXPECT_EQ(mismatch->code, RefusalCode::GoldenRolloutMismatch);
}

// The fast MLP mode (launch dynamics_fast_mlp) is checked against the same golden rollouts: the
// contract 1 fixture's residual and the contract 2 fixture's learned ground block reproduce them
// within the tolerance, the loaded params carry the mode, and a bundle the exact mode refuses is
// still refused.
TEST(BundleLoader, FastMlpModeReproducesTheGoldenRolloutsOfBothContracts) {
  for (const std::string& dir : {test::fixture_dir(), test::fixture_v2_dir()}) {
    const auto geometry = test::fixture_geometry(dir);
    const auto exact = load_dynamics_bundle(dir, geometry, 0.5, false);
    const auto fast = load_dynamics_bundle(dir, geometry, 0.5, true);
    ASSERT_TRUE(exact.ok()) << dir << ": " << (exact.refusal ? exact.refusal->message : "");
    ASSERT_TRUE(fast.ok()) << dir << ": " << (fast.refusal ? fast.refusal->message : "");
    EXPECT_FALSE(exact.bundle->params.fast_mlp);
    EXPECT_TRUE(fast.bundle->params.fast_mlp);
    EXPECT_TRUE(fast.bundle->params.residual.enabled || fast.bundle->params.ground.learned_enabled)
        << dir;
    EXPECT_LT(fast.bundle->golden_max_abs_error, kGoldenTolerance) << dir;
    // The two modes differ by rounding only: the worst golden error moves by far less than the
    // tolerance.
    EXPECT_NEAR(fast.bundle->golden_max_abs_error, exact.bundle->golden_max_abs_error, 1e-9) << dir;
  }
  BundleCopy b;
  b.edit_manifest("steer_gain_left: 1.1", "steer_gain_left: 1.2");
  const auto r = load_dynamics_bundle(b.dir(), test::fixture_geometry(), 0.5, true);
  expect_refusal(r, RefusalCode::GoldenRolloutMismatch);
}

// The float32 MLP mode (launch dynamics_mlp_float32, owner decision 30), alone and with the fast
// mode, is checked against the same golden rollouts of both contracts: they are reproduced within
// the tolerance, the loaded params carry the mode, and the worst error stays within 1e-6 of the
// double pass's (the bound of the float32 path, far below the 1e-4 tolerance).
TEST(BundleLoader, Float32MlpModeReproducesTheGoldenRolloutsOfBothContracts) {
  for (const std::string& dir : {test::fixture_dir(), test::fixture_v2_dir()}) {
    const auto geometry = test::fixture_geometry(dir);
    const auto exact = load_dynamics_bundle(dir, geometry, 0.5, false, false);
    for (const bool fast : {false, true}) {
      const auto f32 = load_dynamics_bundle(dir, geometry, 0.5, fast, true);
      ASSERT_TRUE(f32.ok()) << dir << ": " << (f32.refusal ? f32.refusal->message : "");
      EXPECT_TRUE(f32.bundle->params.float32_mlp);
      EXPECT_EQ(f32.bundle->params.fast_mlp, fast);
      EXPECT_FALSE(exact.bundle->params.float32_mlp);
      EXPECT_LT(f32.bundle->golden_max_abs_error, kGoldenTolerance) << dir;
      EXPECT_NEAR(f32.bundle->golden_max_abs_error, exact.bundle->golden_max_abs_error, 1e-6)
          << dir << " fast " << fast;
      std::printf("float32 golden %s fast=%d: %.3e (double %.3e)\n", dir.c_str(), fast,
                  f32.bundle->golden_max_abs_error, exact.bundle->golden_max_abs_error);
    }
  }
  BundleCopy b;
  b.edit_manifest("steer_gain_left: 1.1", "steer_gain_left: 1.2");
  const auto r = load_dynamics_bundle(b.dir(), test::fixture_geometry(), 0.5, false, true);
  expect_refusal(r, RefusalCode::GoldenRolloutMismatch);
}
