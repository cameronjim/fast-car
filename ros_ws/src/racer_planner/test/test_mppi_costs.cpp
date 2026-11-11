// Each cost term of the solver in isolation (all other weights zero), then the solver's own
// behaviours: determinism, the collision fallback, the brake sample.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "racer_planner/mppi.hpp"
#include "test_support.hpp"

using namespace racer_planner;

namespace {

constexpr int kT = 30;

MppiParams only_weights() {
  MppiParams p;
  p.horizon_steps = kT;
  p.w_progress = 0.0;
  p.w_clear = 0.0;
  p.w_bound = 0.0;
  p.w_lat = 0.0;
  p.w_du = 0.0;
  p.w_term = 0.0;
  p.c_collision = 1e4;
  p.d_hard_m = -100.0;  // collisions off unless a test turns them on
  return p;
}

struct Env {
  VehicleGeometry g = test::binding_geometry();
  DynamicsParams dyn = test::plain_dynamics();
  DynamicsModel model{dyn, g, dyn.dt_s};
  DistanceField field;
  TrackProjector track;
  LocalTrackData data;

  Env(int valid = kTrackStations, double hwl = 0.5, double hwr = 0.5) {
    data.frame_id = "base_link";
    data.station_spacing_m = kTrackStationSpacingM;
    data.valid_count = valid;
    for (int k = 0; k < kTrackStations; ++k) {
      data.x_m.push_back(k * kTrackStationSpacingM);
      data.y_m.push_back(0.0);
      data.half_width_left_m.push_back(hwl);
      data.half_width_right_m.push_back(hwr);
      data.sigma_m.push_back(0.02);
    }
    track.build(data, Pose2{});
    field.build(nullptr, 0);
  }

  void set_points(const std::vector<PointXY>& pts) { field.build(pts.data(), pts.size()); }

  SolveInput input(double v, double delta = 0.0, double v_cap = 1.0) {
    SolveInput in;
    in.x0.v = v;
    in.x0.delta = delta;
    in.v_cap_mps = v_cap;
    in.track = &track;
    in.field = &field;
    in.prev_cmd = Control{v, delta};
    return in;
  }

  std::vector<Control> constant(double speed, double steering) const {
    return std::vector<Control>(kT, Control{speed, steering});
  }

  MppiSolver solver(const MppiParams& p, int n = 64) const { return MppiSolver(p, model, g, n); }
};

std::vector<PointXY> wall_across(double x) {
  std::vector<PointXY> pts;
  for (double y = -1.0; y <= 1.0; y += 0.02) pts.push_back({x, y});
  return pts;
}

}  // namespace

TEST(MppiCost, ProgressIsMinusWeightTimesCentrelineProgress) {
  Env e;
  MppiParams p = only_weights();
  p.w_progress = 10.0;
  auto s = e.solver(p);
  CostBreakdown bd;
  const auto seq = e.constant(0.5, 0.0);
  s.evaluate_sequence(e.input(0.5), seq.data(), &bd);
  EXPECT_NEAR(bd.progress_m, 30 * 0.05 * 0.5, 1e-9);
  EXPECT_NEAR(bd.progress, -10.0 * bd.progress_m, 1e-9);
  EXPECT_NEAR(bd.total, bd.progress, 1e-12);  // nothing else is on
  // faster is better
  CostBreakdown fast;
  const auto seq_fast = e.constant(1.0, 0.0);
  s.evaluate_sequence(e.input(1.0), seq_fast.data(), &fast);
  EXPECT_LT(fast.total, bd.total);
}

TEST(MppiCost, ProgressIsCappedAtTheLastValidStation) {
  Env e(4);  // visible to x = 0.6
  MppiParams p = only_weights();
  p.w_progress = 10.0;
  auto s = e.solver(p);
  CostBreakdown bd;
  const auto seq = e.constant(1.0, 0.0);  // would travel 1.5 m
  s.evaluate_sequence(e.input(1.0), seq.data(), &bd);
  EXPECT_NEAR(bd.progress_m, 0.6, 1e-9);
  EXPECT_NEAR(bd.progress, -6.0, 1e-9);
}

TEST(MppiCost, ProgressIsRelativeToTheStartingProjection) {
  Env e;
  MppiParams p = only_weights();
  p.w_progress = 10.0;
  auto s = e.solver(p);
  SolveInput in = e.input(0.5);
  in.x0.x = 0.4;  // already 0.4 m along
  CostBreakdown bd;
  const auto seq = e.constant(0.5, 0.0);
  s.evaluate_sequence(in, seq.data(), &bd);
  EXPECT_NEAR(bd.progress_m, 0.75, 1e-9);
}

TEST(MppiCost, CollisionAddsTheConstantAndFreezesProgress) {
  Env e;
  e.set_points(wall_across(0.8));
  MppiParams p = only_weights();
  p.w_progress = 10.0;
  p.d_hard_m = 0.03;
  p.c_collision = 1e4;
  auto s = e.solver(p);
  CostBreakdown hit;
  const auto seq = e.constant(0.5, 0.0);
  s.evaluate_sequence(e.input(0.5), seq.data(), &hit);
  EXPECT_TRUE(hit.collided);
  EXPECT_NEAR(hit.collision, 1e4, 1e-9);
  EXPECT_LT(hit.progress_m, 0.75);  // frozen at the step it hit
  EXPECT_NEAR(hit.progress, -10.0 * hit.progress_m, 1e-9);
  EXPECT_LT(hit.min_clearance_m, 0.03);
  // Stopping short of the wall: no collision.
  CostBreakdown ok;
  const auto stop = e.constant(0.0, 0.0);
  s.evaluate_sequence(e.input(0.0), stop.data(), &ok);
  EXPECT_FALSE(ok.collided);
  EXPECT_EQ(ok.collision, 0.0);
}

TEST(MppiCost, ClearanceTermMatchesTheFormulaForAStationaryCar) {
  Env e;
  const Footprint fp(e.g);
  // One obstacle point ahead of the front circle: clearance well below d_soft_base (0.15).
  const PointXY obstacle{fp.offsets()[2] + fp.radius() + 0.08, 0.0};
  e.set_points({obstacle});
  MppiParams p = only_weights();
  p.w_clear = 200.0;
  auto s = e.solver(p);
  CostBreakdown bd;
  const auto seq = e.constant(0.0, 0.0);
  SolveInput in = e.input(0.0);
  s.evaluate_sequence(in, seq.data(), &bd);
  const double c0 = fp.clearance(e.field, 0.0, 0.0, 1.0, 0.0);
  ASSERT_GT(c0, 0.0);
  ASSERT_LT(c0, 0.15);
  const double expect = 200.0 * (0.15 - c0) * (0.15 - c0) * 0.05 * kT;
  EXPECT_NEAR(bd.clearance, expect, 1e-9);
  EXPECT_NEAR(bd.total, bd.clearance, 1e-12);
}

TEST(MppiCost, ClearanceThresholdGrowsWithSpeed) {
  Env e;
  const Footprint fp(e.g);
  // Obstacle off to the side so a moving car keeps the same clearance.
  e.set_points({PointXY{0.3, fp.radius() + 0.2 + 0.0}});
  MppiParams p = only_weights();
  p.w_clear = 200.0;
  p.d_soft_base_m = 0.15;
  p.t_headway_s = 0.25;
  auto s = e.solver(p);
  CostBreakdown slow, fast;
  const auto a = e.constant(0.0, 0.0);
  s.evaluate_sequence(e.input(0.0), a.data(), &slow);
  // at 0.8 m/s the soft distance is 0.35, so the same 0.2 m clearance now costs
  const auto b = e.constant(0.8, 0.0);
  s.evaluate_sequence(e.input(0.8), b.data(), &fast);
  EXPECT_EQ(slow.clearance, 0.0);
  EXPECT_GT(fast.clearance, 0.0);
}

TEST(MppiCost, FarFromEverythingCostsNothing) {
  Env e;
  MppiParams p = only_weights();
  p.w_clear = 200.0;
  auto s = e.solver(p);
  CostBreakdown bd;
  const auto seq = e.constant(0.5, 0.0);
  s.evaluate_sequence(e.input(0.5), seq.data(), &bd);
  EXPECT_EQ(bd.clearance, 0.0);
  EXPECT_GT(bd.min_clearance_m, 5.0);
}

TEST(MppiCost, BoundaryTermUsesTheHalfWidthOfTheSideTheCarIsOn) {
  Env e(kTrackStations, 0.5, 0.3);  // left 0.5, right 0.3
  MppiParams p = only_weights();
  p.w_bound = 200.0;
  p.m_bound_m = 0.05;
  auto s = e.solver(p);
  const double half = e.g.width_m / 2.0;
  const auto seq = e.constant(0.0, 0.0);
  {  // 0.4 m to the left: allowed = 0.5 - half - 0.05
    SolveInput in = e.input(0.0);
    in.x0.y = 0.4;
    CostBreakdown bd;
    s.evaluate_sequence(in, seq.data(), &bd);
    const double excess = 0.4 - (0.5 - half - 0.05);
    ASSERT_GT(excess, 0.0);
    EXPECT_NEAR(bd.boundary, 200.0 * excess * excess * 0.05 * kT, 1e-9);
    EXPECT_NEAR(bd.total, bd.boundary, 1e-12);
  }
  {  // 0.2 m to the right: allowed = 0.3 - half - 0.05, a tighter side
    SolveInput in = e.input(0.0);
    in.x0.y = -0.2;
    CostBreakdown bd;
    s.evaluate_sequence(in, seq.data(), &bd);
    const double excess = 0.2 - (0.3 - half - 0.05);
    ASSERT_GT(excess, 0.0);
    EXPECT_NEAR(bd.boundary, 200.0 * excess * excess * 0.05 * kT, 1e-9);
  }
  {  // centred: nothing
    CostBreakdown bd;
    s.evaluate_sequence(e.input(0.0), seq.data(), &bd);
    EXPECT_EQ(bd.boundary, 0.0);
  }
}

TEST(MppiCost, LateralAccelerationOnlyAboveTheLimit) {
  Env e;
  MppiParams p = only_weights();
  p.w_lat = 5.0;
  p.a_lat_max_mps2 = 0.3;
  auto s = e.solver(p);
  // v = 1 and delta = 0.2 held: v * yaw_rate = tan(0.2) / L
  const double lat = std::tan(0.2) / e.g.wheelbase_m;
  ASSERT_GT(lat, 0.3);
  CostBreakdown bd;
  const auto seq = e.constant(1.0, 0.2);
  s.evaluate_sequence(e.input(1.0, 0.2), seq.data(), &bd);
  EXPECT_NEAR(bd.lateral, 5.0 * (lat - 0.3) * (lat - 0.3) * 0.05 * kT, 1e-9);
  EXPECT_NEAR(bd.total, bd.lateral, 1e-12);
  p.a_lat_max_mps2 = 2.0;  // the default, above this lateral acceleration
  auto relaxed = e.solver(p);
  CostBreakdown none;
  relaxed.evaluate_sequence(e.input(1.0, 0.2), seq.data(), &none);
  EXPECT_EQ(none.lateral, 0.0);
}

TEST(MppiCost, SmoothnessIsTheWeightedSumOfSquaredControlChanges) {
  Env e;
  MppiParams p = only_weights();
  p.w_du = 0.5;
  auto s = e.solver(p);
  std::vector<Control> seq(kT, Control{0.5, 0.0});
  seq[0] = Control{0.6, 0.1};  // change from prev (0.5, 0.0) at step 0, back at step 1
  CostBreakdown bd;
  SolveInput in = e.input(0.5, 0.0);
  in.prev_cmd = Control{0.5, 0.0};
  s.evaluate_sequence(in, seq.data(), &bd);
  const double expect = 0.5 * ((0.1 * 0.1 + 0.1 * 0.1) + (0.1 * 0.1 + 0.1 * 0.1));
  EXPECT_NEAR(bd.smoothness, expect, 1e-9);
  // A constant sequence equal to the previous command is free.
  const auto flat = e.constant(0.5, 0.0);
  CostBreakdown zero;
  s.evaluate_sequence(in, flat.data(), &zero);
  EXPECT_EQ(zero.smoothness, 0.0);
}

TEST(MppiCost, TerminalTermRequiresStoppingWithinTheVisibleTrack) {
  Env e(3);  // visible 0.4 m
  MppiParams p = only_weights();
  p.w_term = 50.0;
  auto s = e.solver(p);
  CostBreakdown bd;
  const auto seq = e.constant(1.0, 0.0);  // cruising at 1 m/s past the end of what is visible
  s.evaluate_sequence(e.input(1.0), seq.data(), &bd);
  const double need =
      bd.v_end_mps * bd.v_end_mps / (2.0 * e.g.stop_deceleration_mps2) - (0.4 - bd.s_end_m);
  ASSERT_GT(need, 0.0);
  EXPECT_NEAR(bd.terminal, 50.0 * need * need, 1e-9);
  // Stopped, nothing is required.
  CostBreakdown rest;
  const auto stop = e.constant(0.0, 0.0);
  s.evaluate_sequence(e.input(0.0), stop.data(), &rest);
  EXPECT_EQ(rest.terminal, 0.0);
  // Plenty of track in sight: slow cruise is fine.
  Env far;
  auto sf = far.solver(p);
  CostBreakdown slow;
  const auto slow_seq = far.constant(0.3, 0.0);
  sf.evaluate_sequence(far.input(0.3), slow_seq.data(), &slow);
  EXPECT_EQ(slow.terminal, 0.0);
}

TEST(MppiCost, TotalIsTheSumOfTheTerms) {
  Env e;
  e.set_points(wall_across(1.2));
  MppiParams p;  // default weights
  p.horizon_steps = kT;
  auto s = e.solver(p);
  CostBreakdown bd;
  std::vector<Control> seq(kT, Control{0.7, 0.1});
  SolveInput in = e.input(0.4, 0.0);
  in.x0.y = 0.3;
  s.evaluate_sequence(in, seq.data(), &bd);
  EXPECT_NEAR(bd.total,
              bd.progress + bd.collision + bd.clearance + bd.boundary + bd.lateral + bd.smoothness +
                  bd.terminal,
              1e-9);
}

// ---------------------------------------------------------------------------------------------

TEST(MppiSolver, FreeCorridorDrivesForwardAndStraight) {
  Env e;
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 512;
  p.seed = 3;
  auto s = e.solver(p, 512);
  SolveOutput out;
  SolveInput in = e.input(0.3, 0.0, 0.6);
  for (int i = 0; i < 20; ++i) out = s.solve(in);
  EXPECT_GT(out.cmd.speed_cmd, 0.4);
  EXPECT_LE(out.cmd.speed_cmd, 0.6 + 1e-12);
  EXPECT_NEAR(out.cmd.steering_cmd, 0.0, 0.15);
  EXPECT_FALSE(out.all_collide);
  EXPECT_EQ(out.samples, 512);
  EXPECT_GT(out.plan_progress_m, 0.3);
  ASSERT_EQ(s.plan().size(), static_cast<std::size_t>(kT + 1));
  EXPECT_EQ(s.plan()[0].x, in.x0.x);
}

TEST(MppiSolver, SteersBackTowardTheCentreline) {
  Env e;
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 512;
  auto s = e.solver(p, 512);
  SolveInput in = e.input(0.4, 0.0, 0.6);
  in.x0.y = 0.4;  // left of the centreline and past the boundary margin
  SolveOutput out;
  for (int i = 0; i < 10; ++i) out = s.solve(in);
  EXPECT_LT(out.cmd.steering_cmd, 0.0);  // turn right, back to the middle
  in.x0.y = -0.4;
  s.reset();
  for (int i = 0; i < 10; ++i) out = s.solve(in);
  EXPECT_GT(out.cmd.steering_cmd, 0.0);
}

TEST(MppiSolver, SameSeedSameResultDifferentSeedDiffers) {
  Env e;
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 256;
  p.seed = 9;
  auto a = e.solver(p, 256);
  auto b = e.solver(p, 256);
  p.seed = 10;
  auto c = e.solver(p, 256);
  SolveInput in = e.input(0.3, 0.0, 0.6);
  in.x0.y = 0.1;
  bool differs = false;
  for (int i = 0; i < 5; ++i) {
    const auto oa = a.solve(in), ob = b.solve(in), oc = c.solve(in);
    EXPECT_EQ(oa.cmd.speed_cmd, ob.cmd.speed_cmd);
    EXPECT_EQ(oa.cmd.steering_cmd, ob.cmd.steering_cmd);
    EXPECT_EQ(oa.best_cost, ob.best_cost);
    if (oa.cmd.steering_cmd != oc.cmd.steering_cmd) differs = true;
  }
  EXPECT_TRUE(differs);
}

TEST(MppiSolver, EverySampleCollidingGivesZeroSpeedAndTheBestSteering) {
  Env e;
  e.set_points(wall_across(0.2));  // the wall is inside the car already
  MppiParams p;
  p.horizon_steps = kT;
  p.d_hard_m = 0.03;
  auto s = e.solver(p, 128);
  SolveInput in = e.input(0.4, 0.0, 0.6);
  const SolveOutput out = s.solve(in);
  EXPECT_TRUE(out.all_collide);
  EXPECT_EQ(out.cmd.speed_cmd, 0.0);
  EXPECT_DOUBLE_EQ(out.collision_fraction, 1.0);
  EXPECT_TRUE(std::isfinite(out.cmd.steering_cmd));
}

TEST(MppiSolver, TheBrakeSampleSavesTheCarWhenOnlyBrakingAvoidsTheWall) {
  Env e;
  const Footprint fp(e.g);
  // The front circle edge is fp.offsets()[2] + radius from the rear axle. Put the wall 0.22 m
  // beyond that: cruising at 0.5 m/s hits it within the horizon, braking stops short.
  e.set_points(wall_across(fp.offsets()[2] + fp.radius() + 0.22));
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 384;
  auto s = e.solver(p, 384);
  SolveInput in = e.input(0.5, 0.0, 0.6);
  SolveOutput out;
  for (int i = 0; i < 3; ++i) out = s.solve(in);
  EXPECT_FALSE(out.all_collide);
  EXPECT_GT(out.collision_fraction, 0.0);
  EXPECT_LT(out.cmd.speed_cmd, 0.2);  // it slows hard
}

TEST(MppiSolver, SampleCountIsClampedAndReported) {
  Env e;
  MppiParams p;
  p.horizon_steps = kT;
  auto s = e.solver(p, 300);
  EXPECT_EQ(s.capacity(), 300);
  s.set_samples(10000);
  EXPECT_EQ(s.samples(), 300);
  s.set_samples(1);
  EXPECT_EQ(s.samples(), kMinSamples);
  s.set_samples(100);
  const SolveOutput out = s.solve(e.input(0.3));
  EXPECT_EQ(out.samples, 100);
}

TEST(MppiSolver, SpeedCapIsRespectedByEveryCommand) {
  Env e;
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 256;
  auto s = e.solver(p, 256);
  SolveInput in = e.input(0.3, 0.0, 0.25);
  for (int i = 0; i < 10; ++i) {
    const SolveOutput out = s.solve(in);
    EXPECT_LE(out.cmd.speed_cmd, 0.25 + 1e-12);
    EXPECT_GE(out.cmd.speed_cmd, 0.0);
    for (const Control& c : s.nominal()) EXPECT_LE(c.speed_cmd, 0.25 + 1e-12);
  }
}
