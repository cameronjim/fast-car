// Brief 43: dynamics step contract 2 (bundle schema 2.0.0) in the planner. Loader, step, solver
// meanings, dead reckoning. The contract 1 side is pinned by test_contract1_identity.cpp.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "gate_test_support.hpp"
#include "planner_rig.hpp"
#include "racer_planner/bundle_loader.hpp"
#include "racer_planner/distance_field.hpp"
#include "racer_planner/dynamics_model.hpp"
#include "racer_planner/ego_motion.hpp"
#include "racer_planner/lap_profile.hpp"
#include "racer_planner/local_track.hpp"
#include "racer_planner/mppi.hpp"
#include "test_support.hpp"

using namespace racer_planner;
using test::BundleCopy;

namespace {

const std::string kRepo = RACER_PLANNER_REPO_ROOT;

void expect_refusal(const LoadResult& r, RefusalCode code) {
  ASSERT_FALSE(r.ok()) << "bundle was accepted";
  ASSERT_TRUE(r.refusal.has_value());
  EXPECT_EQ(r.refusal->code, code)
      << refusal_code_name(r.refusal->code) << ": " << r.refusal->message;
}

LoadResult load_v2(const BundleCopy& b, double v = 1.0) {
  return load_dynamics_bundle(b.dir(), test::fixture_geometry(test::fixture_v2_dir()), v);
}

BundleCopy v2_copy() { return BundleCopy(test::fixture_v2_dir()); }

// A contract 2 model with one alpha everywhere and no learned block, on a plain nominal.
DynamicsParams const_alpha_params(double alpha, double deadband = 0.0) {
  DynamicsParams p = test::plain_dynamics();
  p.nominal.speed_deadband_mps = deadband;
  p.v_max_mps = 1.4;
  p.step_contract = 2;
  p.ground.enabled = true;
  p.ground.table.fill(alpha);
  p.ground.weights.assign(kMlpWeightCount, 0.0);
  return p;
}

}  // namespace

// -- loader
// ----------------------------------------------------------------------------------------

TEST(Contract2Loader, SyntheticFixtureLoadsAndReproducesItsGoldenRollouts) {
  const BundleCopy b = v2_copy();
  const auto r = load_v2(b);
  ASSERT_TRUE(r.ok()) << (r.refusal ? r.refusal->message : "");
  const DynamicsParams& p = r.bundle->params;
  EXPECT_EQ(p.step_contract, 2);
  EXPECT_TRUE(p.ground.enabled);
  EXPECT_TRUE(p.ground.learned_enabled);
  EXPECT_FALSE(p.residual.enabled);
  EXPECT_DOUBLE_EQ(p.ground.table[0], kGroundAlphaMin);  // a node on the bound
  EXPECT_DOUBLE_EQ(p.ground.table[kGroundSteerNodes * kGroundSpeedNodes - 1], kGroundAlphaMax);
  EXPECT_GT(r.bundle->golden_max_abs_error, 0.0);
  EXPECT_LT(r.bundle->golden_max_abs_error, kGoldenTolerance);
}

TEST(Contract2Loader, ExportedCandidatesAndTheSimBundleLoadAndReproduceTheirGoldens) {
  // dyn-v0.4t and dyn-v0.4tl come from the dynamics workstream (PR 193) and are TEST INPUTS ONLY:
  // GP2e-T failed, nothing is deployed. The sim bundle has a table of 1 and no learned block.
  struct Case {
    const char* dir;
    bool learned;
  };
  for (const Case& c :
       {Case{"dyn-v0.4t-contract2-test-only", false}, Case{"dyn-v0.4tl-contract2-test-only", true},
        Case{"dyn-car-2026-11-12-contract2-sim", false}}) {
    const auto r = load_dynamics_bundle(kRepo + "/config/pnp/fixtures/" + c.dir,
                                        test::binding_geometry(), 1.0);
    ASSERT_TRUE(r.ok()) << c.dir << ": " << (r.refusal ? r.refusal->message : "");
    EXPECT_EQ(r.bundle->params.step_contract, 2) << c.dir;
    EXPECT_EQ(r.bundle->params.ground.learned_enabled, c.learned) << c.dir;
    EXPECT_LT(r.bundle->golden_max_abs_error, kGoldenTolerance) << c.dir;
    EXPECT_DOUBLE_EQ(r.bundle->params.v_max_mps, 1.3) << c.dir;
  }
}

TEST(Contract2Loader, ARealGoldenIsNotReproducedByTheContractTheBundleDoesNotDeclare) {
  // The T candidate's table is far from 1: stepping its golden rollouts as contract 1 must fail.
  const std::string dir = kRepo + "/config/pnp/fixtures/dyn-v0.4t-contract2-test-only";
  const auto r = load_dynamics_bundle(dir, test::binding_geometry(), 1.0);
  ASSERT_TRUE(r.ok());
  DynamicsParams as_one = r.bundle->params;
  as_one.step_contract = 1;
  as_one.ground.enabled = false;
  const auto m = verify_golden_rollouts(as_one, test::binding_geometry(),
                                        dir + "/golden_rollouts.csv", nullptr);
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->code, RefusalCode::GoldenRolloutMismatch);
}

TEST(Contract2Loader, ResidualBlockIsRefusedWhetherEnabledOrNot) {
  for (const char* enabled : {"true", "false"}) {
    BundleCopy b = v2_copy();
    b.edit_manifest("  ground_motion:\n",
                    std::string("  residual: {enabled: ") + enabled + "}\n  ground_motion:\n");
    expect_refusal(load_v2(b), RefusalCode::IoContractMismatch);
  }
}

TEST(Contract2Loader, GroundBlockInAContractOneBundleIsRefused) {
  BundleCopy b;
  b.edit_manifest("  residual:\n", "  ground_motion: {}\n  residual:\n");
  expect_refusal(load_dynamics_bundle(b.dir(), test::fixture_geometry(), 0.5),
                 RefusalCode::IoContractMismatch);
}

TEST(Contract2Loader, TableValueOutsideItsBoundsIsRefused) {
  {
    BundleCopy b = v2_copy();  // the first node sits on the lower bound 0.6
    b.edit_manifest("- [0.6,", "- [0.59,");
    expect_refusal(load_v2(b), RefusalCode::IoContractMismatch);
  }
  {
    BundleCopy b = v2_copy();  // the last node sits on the upper bound 1.05
    b.edit_manifest(", 1.05]", ", 1.0500001]");
    expect_refusal(load_v2(b), RefusalCode::IoContractMismatch);
  }
}

TEST(Contract2Loader, ConstantsThatDifferFromTheAddendumAreRefused) {
  const std::vector<std::pair<std::string, std::string>> edits = {
      {"alpha_bounds: [0.6, 1.05]", "alpha_bounds: [0.5, 1.05]"},
      {"alpha_bounds: [0.6, 1.05]", "alpha_bounds: [0.6, 1.1]"},
      {"d_alpha_max: 0.1", "d_alpha_max: 0.2"},
      {"rho_max: 0.25", "rho_max: 0.5"},
      {"wheel_speed_nodes_mps: [0.25,", "wheel_speed_nodes_mps: [0.3,"},
      {"steering_nodes_share_of_side_limit: [-1.0,", "steering_nodes_share_of_side_limit: [-0.9,"},
      {"layers: [[4, 32], [32, 32], [32, 2]]", "layers: [[4, 32], [32, 32], [32, 3]]"},
      {"activation: tanh", "activation: relu"},
      {"outputs: [d_alpha, rho]", "outputs: [rho, d_alpha]"},
  };
  for (const auto& e : edits) {
    BundleCopy b = v2_copy();
    b.edit_manifest(e.first, e.second);
    expect_refusal(load_v2(b), RefusalCode::IoContractMismatch);
  }
}

TEST(Contract2Loader, VersionRules) {
  {
    BundleCopy b = v2_copy();
    b.edit_manifest("bundle_schema_version: \"2.0.0\"", "bundle_schema_version: \"2.0.7\"");
    EXPECT_TRUE(load_v2(b).ok());
  }
  for (const char* v : {"2.1.0", "3.0.0", "0.9.0", "2.0.x", "2.0."}) {
    BundleCopy b = v2_copy();
    b.edit_manifest("bundle_schema_version: \"2.0.0\"",
                    std::string("bundle_schema_version: \"") + v + "\"");
    const auto r = load_v2(b);
    ASSERT_FALSE(r.ok()) << v;
    EXPECT_TRUE(r.refusal->code == RefusalCode::BundleSchemaVersionMismatch ||
                r.refusal->code == RefusalCode::ManifestInvalid)
        << v;
  }
  BundleCopy b = v2_copy();
  b.edit_manifest("bundle_schema_version: \"2.0.0\"", "bundle_schema_version: \"2.1.0\"");
  expect_refusal(load_v2(b), RefusalCode::BundleSchemaVersionMismatch);
}

TEST(Contract2Loader, WeightsFileProblems) {
  {
    BundleCopy b = v2_copy();
    b.rewrite_payload("ground_mlp.f32", "short");
    expect_refusal(load_v2(b), RefusalCode::IoContractMismatch);
  }
  {
    BundleCopy b = v2_copy();
    test::fs::remove(b.path("ground_mlp.f32"));
    expect_refusal(load_v2(b), RefusalCode::PayloadMissing);
  }
  {
    BundleCopy b = v2_copy();
    std::string bytes = test::read_text(b.path("ground_mlp.f32"));
    bytes[3] = '\x7f';  // an exponent of all ones in the first float: inf or nan
    bytes[2] = '\x80';
    b.rewrite_payload("ground_mlp.f32", bytes);
    expect_refusal(load_v2(b), RefusalCode::IoContractMismatch);
  }
}

TEST(Contract2Loader, OtherRowsOfTheRefusalTableStillApply) {
  {
    BundleCopy b = v2_copy();  // golden rollouts no longer match the declared nominal
    b.edit_manifest("speed_gain: 0.95", "speed_gain: 0.96");
    expect_refusal(load_v2(b), RefusalCode::GoldenRolloutMismatch);
  }
  expect_refusal(load_v2(v2_copy(), 1.4), RefusalCode::ValidityExceeded);
  {
    BundleCopy b = v2_copy();
    b.edit_manifest("kind: dynamics", "kind: track_perception");
    expect_refusal(load_v2(b), RefusalCode::KindMismatch);
  }
  {
    BundleCopy b = v2_copy();
    test::write_text(b.path("golden_rollouts.csv"), "tampered\n");
    expect_refusal(load_v2(b), RefusalCode::ChecksumMismatch);
  }
}

TEST(Contract2Loader, TheGroundTableInfluencesTheGoldenCheck) {
  const BundleCopy b = v2_copy();
  const auto r = load_v2(b);
  ASSERT_TRUE(r.ok());
  DynamicsParams p = r.bundle->params;
  p.ground.table[3 * kGroundSpeedNodes + 1] += 0.05;  // straight ahead, 0.6 m/s
  const auto m = verify_golden_rollouts(p, test::fixture_geometry(test::fixture_v2_dir()),
                                        b.path("golden_rollouts.csv"), nullptr);
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->code, RefusalCode::GoldenRolloutMismatch);
}

// The constants in the C++ are the pre-registered ones: read them from the frozen yaml.
TEST(Contract2Loader, CompiledConstantsEqualTheFrozenAddendumYaml) {
  const std::string path = kRepo + "/evaluation/prereg/pnp-v0.4-addendum-1.yaml";
  std::string sha;
  ASSERT_TRUE(sha256_file_hex(path, &sha));
  EXPECT_EQ(sha, "ada5a83675e5c843fb8e121a1190e4458f823b1ed2ff32a34a46c5fb4ff76824");
  const YAML::Node g = YAML::LoadFile(path)["ground_motion"];
  EXPECT_DOUBLE_EQ(g["alpha_bounds"][0].as<double>(), kGroundAlphaMin);
  EXPECT_DOUBLE_EQ(g["alpha_bounds"][1].as<double>(), kGroundAlphaMax);
  EXPECT_DOUBLE_EQ(g["learned"]["d_alpha_max"].as<double>(), kGroundDAlphaMax);
  EXPECT_DOUBLE_EQ(g["learned"]["rho_max"].as<double>(), kGroundRhoMax);
  const YAML::Node st = g["table"]["steering_nodes_share_of_side_limit"];
  const YAML::Node sp = g["table"]["wheel_speed_nodes_mps"];
  ASSERT_EQ(st.size(), static_cast<std::size_t>(kGroundSteerNodes));
  ASSERT_EQ(sp.size(), static_cast<std::size_t>(kGroundSpeedNodes));
  for (int i = 0; i < kGroundSteerNodes; ++i) {
    EXPECT_DOUBLE_EQ(st[static_cast<std::size_t>(i)].as<double>(), kGroundSteerNodeShare[i]);
  }
  for (int j = 0; j < kGroundSpeedNodes; ++j) {
    EXPECT_DOUBLE_EQ(sp[static_cast<std::size_t>(j)].as<double>(), kGroundSpeedNodeMps[j]);
  }
}

// -- the step
// --------------------------------------------------------------------------------------

TEST(Contract2Step, EqualsThePythonReferenceOnTenThousandRandomStates) {
  const BundleCopy b = v2_copy();
  const VehicleGeometry g = test::fixture_geometry(test::fixture_v2_dir());
  const auto r = load_v2(b);
  ASSERT_TRUE(r.ok());
  const DynamicsModel model(r.bundle->params, g, r.bundle->params.dt_s);
  std::ifstream in(test::fixture_v2_dir() + "/step_v2_random.csv");
  ASSERT_TRUE(in);
  std::string line;
  std::getline(in, line);
  ASSERT_EQ(line, "x,y,yaw,v,delta,speed_cmd,steering_cmd,x1,y1,yaw1,v1,delta1");
  int n = 0;
  double worst = 0.0;
  while (std::getline(in, line)) {
    std::stringstream ss(line);
    double f[12];
    std::string tok;
    for (int i = 0; i < 12; ++i) {
      ASSERT_TRUE(std::getline(ss, tok, ','));
      f[i] = std::stod(tok);
    }
    const State o = model.step(State{f[0], f[1], f[2], f[3], f[4]}, Control{f[5], f[6]});
    const double e[5] = {o.x - f[7], o.y - f[8], o.yaw - f[9], o.v - f[10], o.delta - f[11]};
    for (double x : e) worst = std::max(worst, std::fabs(x));
    ++n;
  }
  EXPECT_EQ(n, 10000);
  EXPECT_LT(worst, 1e-9);
}

TEST(Contract2Step, TableIsBilinearOnTheGridHeldConstantOutsideAndSideAware) {
  DynamicsParams p = const_alpha_params(1.0);
  for (int i = 0; i < kGroundSteerNodes; ++i) {
    for (int j = 0; j < kGroundSpeedNodes; ++j) {
      p.ground.table[static_cast<std::size_t>(i * kGroundSpeedNodes + j)] =
          0.7 + 0.01 * i + 0.02 * j;
    }
  }
  const VehicleGeometry g = test::binding_geometry();
  const DynamicsModel m(p, g, 0.05);
  double a = 0, rho = 0;
  // at a node: the left limit is steering_max_rad, the right one steering_min_rad
  m.ground_factors(0.85, 0.3333 * g.steering_max_rad, 0.0, 0.0, &a, &rho);
  EXPECT_NEAR(a, 0.7 + 0.01 * 4 + 0.02 * 2, 1e-12);
  m.ground_factors(0.85, -0.3333 * -g.steering_min_rad, 0.0, 0.0, &a, &rho);
  EXPECT_NEAR(a, 0.7 + 0.01 * 2 + 0.02 * 2, 1e-12);
  EXPECT_EQ(rho, 0.0);
  // halfway between two speed nodes and two steering nodes
  m.ground_factors(0.5 * (0.6 + 0.85), 0.0 + 0.5 * 0.3333 * g.steering_max_rad, 0, 0, &a, &rho);
  const double w = 0.5;
  const double lo_i = 3, hi_i = 4;
  const double expect = 0.7 + 0.01 * (lo_i * (1 - w) + hi_i * w) + 0.02 * (1 * (1 - w) + 2 * w);
  EXPECT_NEAR(a, expect, 1e-12);
  // outside the grid: constant
  m.ground_factors(5.0, 3.0, 0, 0, &a, &rho);
  EXPECT_NEAR(a, 0.7 + 0.01 * 6 + 0.02 * 4, 1e-12);
  m.ground_factors(-1.0, -3.0, 0, 0, &a, &rho);
  EXPECT_NEAR(a, 0.7 + 0.0 + 0.0, 1e-12);
}

TEST(Contract2Step, ZeroWheelSpeedGivesZeroGroundMotionAndZeroYawRateWhateverTheWeights) {
  const BundleCopy b = v2_copy();
  const VehicleGeometry g = test::fixture_geometry(test::fixture_v2_dir());
  const auto r = load_v2(b);
  ASSERT_TRUE(r.ok());
  DynamicsParams big = r.bundle->params;
  for (double& w : big.ground.weights) w *= 25.0;  // saturate the tanh outputs
  std::mt19937_64 rng(3);
  auto u = [&](double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(rng);
  };
  for (const DynamicsParams& p : {r.bundle->params, big}) {
    const DynamicsModel m(p, g, p.dt_s);
    for (int i = 0; i < 100000; ++i) {
      const State s{u(-2, 2), u(-2, 2), u(-4, 4), (i % 2) ? 0.0 : u(0.0, p.v_max_mps),
                    u(-0.45, 0.45)};
      const Control c{u(-0.3, 1.5), u(-0.7, 0.7)};
      double w = 0.0, alpha = 0.0;
      const State o = m.step_trig(s, c, std::cos(s.yaw), std::sin(s.yaw), &w, &alpha);
      ASSERT_TRUE(std::isfinite(o.x) && std::isfinite(o.y) && std::isfinite(o.yaw) &&
                  std::isfinite(o.v) && std::isfinite(o.delta));
      ASSERT_GE(alpha, kGroundAlphaMin);
      ASSERT_LE(alpha, kGroundAlphaMax);
      double a2, rho;
      m.ground_factors(s.v, s.delta, c.speed_cmd, c.steering_cmd, &a2, &rho);
      ASSERT_LE(std::fabs(rho), kGroundRhoMax);
      if (s.v == 0.0) {
        ASSERT_EQ(o.x, s.x);
        ASSERT_EQ(o.y, s.y);
        ASSERT_EQ(o.yaw, s.yaw);
        ASSERT_EQ(w, 0.0);
      }
    }
  }
}

TEST(Contract2Step, RolloutsAreFiniteOverTheValidityBox) {
  const BundleCopy b = v2_copy();
  const VehicleGeometry g = test::fixture_geometry(test::fixture_v2_dir());
  const auto r = load_v2(b);
  ASSERT_TRUE(r.ok());
  const DynamicsParams& p = r.bundle->params;
  const DynamicsModel m(p, g, p.dt_s);
  std::mt19937_64 rng(11);
  auto u = [&](double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(rng);
  };
  for (int k = 0; k < 2000; ++k) {
    State s{0, 0, u(-3, 3), u(p.v_min_mps, p.v_max_mps), u(g.steering_min_rad, g.steering_max_rad)};
    for (int t = 0; t < 60; ++t) {
      s = m.step(s, Control{u(0.0, p.v_max_mps), u(g.steering_min_rad, g.steering_max_rad)});
      ASSERT_TRUE(std::isfinite(s.x) && std::isfinite(s.y) && std::isfinite(s.yaw) &&
                  std::isfinite(s.v) && std::isfinite(s.delta));
    }
  }
}

TEST(Contract2Step, IdentityTableStepsLikeContractOne) {
  // alpha = 1, rho = 0: the contract 2 step is the contract 1 step.
  DynamicsParams one = test::plain_dynamics();
  one.nominal.steer_gain_left = 1.1;
  one.nominal.steer_gain_right = 0.9;
  one.nominal.understeer_s2_per_m2 = 0.05;
  DynamicsParams two = one;
  two.step_contract = 2;
  two.ground.enabled = true;
  two.ground.table.fill(1.0);
  two.ground.weights.assign(kMlpWeightCount, 0.0);
  const VehicleGeometry g = test::binding_geometry();
  const DynamicsModel m1(one, g, 0.05), m2(two, g, 0.05);
  std::mt19937_64 rng(5);
  auto u = [&](double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(rng);
  };
  for (int i = 0; i < 50000; ++i) {
    const State s{u(-3, 3), u(-3, 3), u(-6, 6), u(0, 1.4), u(-0.4, 0.4)};
    const Control c{u(-0.3, 1.5), u(-0.6, 0.6)};
    const State a = m1.step(s, c), b = m2.step(s, c);
    // Not bit for bit: the compiler may contract the multiply-adds of the two functions
    // differently (aarch64). The bit-identity that matters, contract 1 against main, is pinned in
    // test_contract1_identity.cpp.
    ASSERT_NEAR(a.x, b.x, 1e-13);
    ASSERT_NEAR(a.y, b.y, 1e-13);
    ASSERT_NEAR(a.yaw, b.yaw, 1e-13);
    ASSERT_EQ(a.v, b.v);
    ASSERT_EQ(a.delta, b.delta);
  }
}

// -- the meaning of every speed in the solver
// ------------------------------------------------------

namespace {

constexpr int kT = 30;

struct SolverEnv {
  VehicleGeometry g = test::binding_geometry();
  DynamicsParams dyn;
  DynamicsModel model;
  DistanceField field;
  TrackProjector track;
  LocalTrackData data;
  LapProfile lap;
  GateConfig gate_cfg = test::binding_gate();
  GateTable table{gate_cfg.geometry, gate_cfg.limits};

  explicit SolverEnv(const DynamicsParams& d, int valid = kTrackStations)
      : dyn(d), model(dyn, g, dyn.dt_s) {
    data.frame_id = "base_link";
    data.station_spacing_m = kTrackStationSpacingM;
    data.valid_count = valid;
    for (int k = 0; k < kTrackStations; ++k) {
      data.x_m.push_back(k * kTrackStationSpacingM);
      data.y_m.push_back(0.0);
      data.half_width_left_m.push_back(0.5);
      data.half_width_right_m.push_back(0.5);
      data.sigma_m.push_back(0.02);
    }
    track.build(data, Pose2{});
    field.build(nullptr, 0);
    lap.active = true;
    lap.s0_m = 4.0;
    lap.n = 61;
    for (std::size_t i = 0; i < lap.n; ++i) {
      lap.v_bound[i] = i < 20 ? 0.8 : 0.45;
      lap.q[i] = 9.0 - 1.1 * kLapStationSpacingM * static_cast<double>(i);
    }
  }

  MppiParams weights_off() const {
    MppiParams p;
    p.horizon_steps = kT;
    p.w_progress = 0.0;
    p.w_clear = 0.0;
    p.w_bound = 0.0;
    p.w_lat = 0.0;
    p.w_du = 0.0;
    p.w_steer_rate = 0.0;
    p.w_term = 0.0;
    p.d_hard_m = -100.0;
    return p;
  }

  SolveInput input(double v, double delta = 0.0) {
    SolveInput in;
    in.x0.v = v;
    in.x0.delta = delta;
    in.v_cap_mps = 2.0;
    in.track = &track;
    in.field = &field;
    in.prev_cmd = Control{v, delta};
    in.wheel_speed_mps = v;
    return in;
  }

  CostBreakdown eval(const MppiParams& p, const SolveInput& in, double speed, double steer) {
    MppiSolver s(p, model, g, 64);
    std::vector<Control> seq(kT, Control{speed, steer});
    CostBreakdown bd;
    s.evaluate_sequence(in, seq.data(), &bd);
    return bd;
  }
};

constexpr double kAlpha = 0.8;

}  // namespace

TEST(Contract2Solver, PositionAndProgressAreGroundDistance) {
  SolverEnv one(test::plain_dynamics()), two(const_alpha_params(kAlpha));
  MppiParams p = one.weights_off();
  p.w_progress = 10.0;
  const CostBreakdown a = one.eval(p, one.input(0.5), 0.5, 0.0);
  const CostBreakdown b = two.eval(p, two.input(0.5), 0.5, 0.0);
  EXPECT_NEAR(b.progress_m / a.progress_m, kAlpha, 1e-9);
  EXPECT_NEAR(b.v_end_mps, a.v_end_mps, 1e-12);  // the wheel channel is the same
}

TEST(Contract2Solver, LapStationAndSpeedBoundCountWheelDistance) {
  SolverEnv one(test::plain_dynamics()), two(const_alpha_params(kAlpha));
  MppiParams p = one.weights_off();
  p.w_trust = 50.0;
  p.w_progress = 5.0;
  SolveInput in1 = one.input(0.7), in2 = two.input(0.7);
  in1.lap = &one.lap;
  in2.lap = &two.lap;
  in1.lap_station_now_m = in2.lap_station_now_m = 0.2;
  // speed 0.9 crosses the bound of 0.8 and the step of the bound at 2.0 m of wheel distance
  const CostBreakdown a = one.eval(p, in1, 0.9, 0.0);
  const CostBreakdown b = two.eval(p, in2, 0.9, 0.0);
  EXPECT_GT(a.lap_trust, 0.0);
  EXPECT_EQ(a.lap_trust, b.lap_trust);
  EXPECT_EQ(a.lap_terminal, b.lap_terminal);
}

TEST(Contract2Solver, LateralAccelerationIsGroundSpeedTimesYawRate) {
  SolverEnv one(test::plain_dynamics()), two(const_alpha_params(kAlpha));
  MppiParams p = one.weights_off();
  p.w_lat = 3.0;
  p.a_lat_max_mps2 = 0.0;  // every lateral acceleration is penalised, squared
  const CostBreakdown a = one.eval(p, one.input(0.6, 0.2), 0.6, 0.3);
  const CostBreakdown b = two.eval(p, two.input(0.6, 0.2), 0.6, 0.3);
  ASSERT_GT(a.lateral, 0.0);
  EXPECT_NEAR(b.lateral / a.lateral, kAlpha * kAlpha, 1e-9);
}

TEST(Contract2Solver, ClearanceHeadwayUsesGroundSpeed) {
  SolverEnv e(const_alpha_params(kAlpha));
  // a side wall parallel to the road: the clearance is the same at every step
  std::vector<PointXY> pts;
  for (double x = -1.0; x <= 6.0; x += 0.02) pts.push_back({x, 0.32});
  e.field.build(pts.data(), pts.size());
  MppiParams p = e.weights_off();
  p.w_clear = 2.0;
  p.d_soft_base_m = 0.05;
  p.t_headway_s = 0.5;
  const CostBreakdown bd = e.eval(p, e.input(0.5), 0.5, 0.0);
  const double clear = bd.min_clearance_m;
  ASSERT_GT(clear, 0.0);
  State s = e.input(0.5).x0;
  double expect = 0.0;
  for (int t = 0; t < kT; ++t) {
    s = e.model.step(s, Control{0.5, 0.0});
    const double d_soft = p.d_soft_base_m + p.t_headway_s * (kAlpha * s.v);
    if (clear < d_soft) expect += (d_soft - clear) * (d_soft - clear) * e.dyn.dt_s;
  }
  ASSERT_GT(expect, 0.0);
  EXPECT_NEAR(bd.clearance, p.w_clear * expect, 1e-9);
}

TEST(Contract2Solver, StoppingDistanceIsOnTheGround) {
  SolverEnv e(const_alpha_params(kAlpha), 6);  // 1 m of valid track
  MppiParams p = e.weights_off();
  p.w_term = 7.0;
  SolveInput in = e.input(1.2);
  in.v_cap_mps = 2.0;
  const CostBreakdown bd = e.eval(p, in, 1.2, 0.0);
  const double vg = kAlpha * bd.v_end_mps;
  const double sight = e.track.s_valid_end() - bd.s_end_m;
  const double need = vg * vg / (2.0 * e.g.stop_deceleration_mps2) - sight;
  ASSERT_GT(need, 0.0) << "choose a faster start so the terminal term is on";
  EXPECT_NEAR(bd.terminal, p.w_term * need * need, 1e-9);
}

TEST(Contract2Solver, GatePathIsGroundDistanceAndWheelSpeedIsTheGatesSpeed) {
  SolverEnv one(test::plain_dynamics()), two(const_alpha_params(kAlpha));
  MppiParams p = one.weights_off();
  p.w_gate = 300.0;
  for (SolverEnv* e : {&one, &two}) {
    e->table.build(test::scan_of(e->gate_cfg.geometry, test::wall_across_road(1.0)).view());
  }
  SolveInput in1 = one.input(1.5), in2 = two.input(1.5);
  in1.gate = &one.table;
  in2.gate = &two.table;
  const CostBreakdown a = one.eval(p, in1, 1.5, 0.0);
  const CostBreakdown b = two.eval(p, in2, 1.5, 0.0);
  ASSERT_GT(a.gate, 0.0);
  // the car covers less ground per step, so the wall stays farther along the rollout
  EXPECT_LT(b.gate, a.gate);
  EXPECT_GT(b.gate, 0.0);
}

TEST(Contract2Solver, IdentityTableSolvesLikeContractOne) {
  DynamicsParams two = const_alpha_params(1.0);
  SolverEnv one(test::plain_dynamics()), tw(two);
  std::vector<PointXY> pts;
  for (double x = -1.0; x <= 5.0; x += 0.02) {
    pts.push_back({x, 0.5});
    pts.push_back({x, -0.5});
  }
  pts.push_back({1.2, 0.2});
  one.field.build(pts.data(), pts.size());
  tw.field.build(pts.data(), pts.size());
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 128;
  p.seed = 9;
  p.w_gate = 100.0;
  p.w_lat = 4.0;
  one.table.build(test::scan_of(one.gate_cfg.geometry, test::wall_across_road(1.8)).view());
  tw.table.build(test::scan_of(tw.gate_cfg.geometry, test::wall_across_road(1.8)).view());
  SolveInput i1 = one.input(0.5, 0.05), i2 = tw.input(0.5, 0.05);
  i1.gate = &one.table;
  i2.gate = &tw.table;
  i1.lap = &one.lap;
  i2.lap = &tw.lap;
  MppiSolver s1(p, one.model, one.g, 128), s2(p, tw.model, tw.g, 128);
  for (int i = 0; i < 3; ++i) {
    const SolveOutput a = s1.solve(i1), b = s2.solve(i2);
    ASSERT_NEAR(a.cmd.speed_cmd, b.cmd.speed_cmd, 1e-9);
    ASSERT_NEAR(a.cmd.steering_cmd, b.cmd.steering_cmd, 1e-9);
    ASSERT_NEAR(a.best_cost, b.best_cost, 1e-8);
    ASSERT_NEAR(a.plan_progress_m, b.plan_progress_m, 1e-9);
  }
}

TEST(Contract2Solver, SolveIsFiniteWithALearnedGroundBlock) {
  const BundleCopy b = v2_copy();
  const auto r = load_v2(b);
  ASSERT_TRUE(r.ok());
  SolverEnv e(r.bundle->params);
  e.g = test::fixture_geometry(test::fixture_v2_dir());
  e.model = DynamicsModel(e.dyn, e.g, e.dyn.dt_s);
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 128;
  MppiSolver s(p, e.model, e.g, 128);
  SolveInput in = e.input(0.6);
  for (int i = 0; i < 5; ++i) {
    const SolveOutput o = s.solve(in);
    EXPECT_TRUE(std::isfinite(o.cmd.speed_cmd) && std::isfinite(o.cmd.steering_cmd));
    EXPECT_TRUE(std::isfinite(o.best_cost));
  }
}

// -- dead reckoning
// --------------------------------------------------------------------------------

TEST(Contract2Ego, TranslatesByGroundSpeedAndKeepsWheelDistance) {
  const VehicleGeometry g = test::binding_geometry();
  const DynamicsParams p = const_alpha_params(kAlpha);
  const DynamicsModel model(p, g, 0.02);
  EgoMotion e(g.wheelbase_m, p.nominal.understeer_s2_per_m2);
  e.set_ground_model(&model);
  const double v = 0.8, delta = 0.25;
  for (int i = 0; i <= 25; ++i) e.push(i * 0.02, v, delta, v, delta);
  const double t = 0.3;
  const Pose2 pose = e.relative_pose(0.1, 0.1 + t);
  const double w = v * std::tan(delta) / g.wheelbase_m;  // no understeer in the plain model
  const double vg = kAlpha * v;
  EXPECT_NEAR(pose.yaw, w * t, 1e-12);
  EXPECT_NEAR(pose.x, vg / w * std::sin(w * t), 1e-9);
  EXPECT_NEAR(pose.y, vg / w * (1.0 - std::cos(w * t)), 1e-9);
  EXPECT_NEAR(e.distance_between(0.1, 0.1 + t), v * t, 1e-12);  // wheel distance, for the lap
}

TEST(Contract2Ego, YawFollowsTheModelsRho) {
  const BundleCopy b = v2_copy();
  const auto r = load_v2(b);
  ASSERT_TRUE(r.ok());
  const VehicleGeometry g = test::fixture_geometry(test::fixture_v2_dir());
  const DynamicsModel model(r.bundle->params, g, 0.02);
  EgoMotion e(g.wheelbase_m, r.bundle->params.nominal.understeer_s2_per_m2);
  e.set_ground_model(&model);
  const double v = 0.9, delta = 0.2, sc = 0.9, st = 0.3;
  for (int i = 0; i <= 25; ++i) e.push(i * 0.02, v, delta, sc, st);
  const double w = model.ground_yaw_rate(v, delta, sc, st);
  EXPECT_NEAR(e.relative_pose(0.0, 0.4).yaw, w * 0.4, 1e-12);
  EXPECT_NE(w, model.yaw_rate(v, delta));  // the learned rho is live in the fixture
}

TEST(Contract2Ego, IdentityModelEqualsNoModelAndContractOneIsUntouched) {
  const VehicleGeometry g = test::binding_geometry();
  const DynamicsParams p = const_alpha_params(1.0);
  const DynamicsModel model(p, g, 0.02);
  EgoMotion plain(g.wheelbase_m, p.nominal.understeer_s2_per_m2);
  EgoMotion grounded(g.wheelbase_m, p.nominal.understeer_s2_per_m2);
  grounded.set_ground_model(&model);
  for (int i = 0; i < 100; ++i) {
    const double v = 0.5 + 0.4 * std::sin(0.1 * i), d = 0.3 * std::sin(0.13 * i);
    plain.push(i * 0.02, v, d);
    grounded.push(i * 0.02, v, d, v, d);
  }
  const Pose2 a = plain.relative_pose(0.3, 1.4), b = grounded.relative_pose(0.3, 1.4);
  EXPECT_EQ(a.x, b.x);
  EXPECT_EQ(a.y, b.y);
  EXPECT_EQ(a.yaw, b.yaw);
}

// -- the planner
// -----------------------------------------------------------------------------------

TEST(Contract2Planner, DrivesAStraightCorridorAndDeadReckonsByGroundMotion) {
  auto run = [](const DynamicsParams& d, Pose2* ego) {
    rig::Rig r(rig::small_config(0.5), d);
    double t = 0.0;
    TickOutput out;
    for (int i = 0; i < 200; ++i) {
      out = r.tick(t);
      t += 0.02;
      EXPECT_NE(out.state, PlannerState::Fault);
    }
    *ego = r.planner.ego_in_scan_frame();
    return out;
  };
  DynamicsParams one = rig::dynamics_with_floor(0.23);
  DynamicsParams two = one;
  two.step_contract = 2;
  two.ground.enabled = true;
  two.ground.table.fill(kAlpha);
  two.ground.weights.assign(kMlpWeightCount, 0.0);
  Pose2 e1, e2;
  const TickOutput o1 = run(one, &e1), o2 = run(two, &e2);
  EXPECT_EQ(o2.state, PlannerState::Driving);
  EXPECT_TRUE(std::isfinite(o2.cmd.speed_mps) && std::isfinite(o2.cmd.steering_rad));
  EXPECT_GT(o2.cmd.speed_mps, 0.0);
  // the scan is 0.1 s old and the wheel speed is 0.3 m/s: x is wheel distance for contract 1 and
  // ground distance for contract 2
  EXPECT_NEAR(e1.x, 0.3 * 0.1, 2e-3);
  EXPECT_NEAR(e2.x, kAlpha * 0.3 * 0.1, 2e-3);
  (void)o1;
}
