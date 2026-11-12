// One test per refusal of the contract table that applies to a dynamics bundle, plus the good
// fixture. The fixture bundle comes from test/fixtures/make_fixture.py, a second implementation
// of the model, so a pass also proves the C++ step reproduces it.
#include <gtest/gtest.h>

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
  b.edit_manifest("bundle_schema_version: \"1.1.0\"", "bundle_schema_version: \"2.0.0\"");
  expect_refusal(load(b), RefusalCode::BundleSchemaVersionMismatch);
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
  b.edit_manifest("schema_version: \"0.16.0\"", "schema_version: \"0.13.0\"");
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
