// Each cost term of the solver in isolation (all other weights zero), then the solver's own
// behaviours: determinism, the collision fallback, the brake sample.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include "gate_test_support.hpp"
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
  p.w_steer_rate = 0.0;
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

TEST(MppiCost, SmoothnessIsTheWeightedSumOfSquaredSpeedCommandChanges) {
  Env e;
  MppiParams p = only_weights();
  p.w_du = 0.5;
  auto s = e.solver(p);
  std::vector<Control> seq(kT, Control{0.5, 0.0});
  seq[0] = Control{0.6, 0.1};  // speed change from prev (0.5) at step 0, back at step 1
  CostBreakdown bd;
  SolveInput in = e.input(0.5, 0.0);
  in.prev_cmd = Control{0.5, 0.0};
  s.evaluate_sequence(in, seq.data(), &bd);
  EXPECT_NEAR(bd.smoothness, 0.5 * (0.1 * 0.1 + 0.1 * 0.1), 1e-9);  // steering is not in here
  EXPECT_EQ(bd.steer_rate, 0.0);
  // A constant sequence equal to the previous command is free.
  const auto flat = e.constant(0.5, 0.0);
  CostBreakdown zero;
  s.evaluate_sequence(in, flat.data(), &zero);
  EXPECT_EQ(zero.smoothness, 0.0);
}

TEST(MppiCost, SteeringRateIsTheSquaredCommandRateTimesDtAgainstTheLastPublishedCommand) {
  Env e;
  MppiParams p = only_weights();
  p.w_steer_rate = 0.1;
  auto s = e.solver(p);
  const double dt = e.dyn.dt_s;
  std::vector<Control> seq(kT, Control{0.5, 0.0});
  seq[0] = Control{0.5, 0.1};  // 0.1 rad from the published 0.0 at step 0, back at step 1
  CostBreakdown bd;
  SolveInput in = e.input(0.5, 0.0);
  in.prev_cmd = Control{0.5, 0.0};
  s.evaluate_sequence(in, seq.data(), &bd);
  const double rate = 0.1 / dt;
  EXPECT_NEAR(bd.steer_rate, 0.1 * (2.0 * rate * rate * dt), 1e-9);
  EXPECT_EQ(bd.smoothness, 0.0);
  EXPECT_NEAR(bd.total, bd.steer_rate, 1e-12);
  // The first step is measured against the last published command, not against zero.
  in.prev_cmd = Control{0.5, 0.1};
  CostBreakdown held;
  s.evaluate_sequence(in, seq.data(), &held);
  EXPECT_NEAR(held.steer_rate, 0.1 * (rate * rate * dt), 1e-9);  // only the step back remains
  // A constant steering equal to the published one is free.
  const auto flat = e.constant(0.5, 0.1);
  CostBreakdown zero;
  s.evaluate_sequence(in, flat.data(), &zero);
  EXPECT_EQ(zero.steer_rate, 0.0);
  // A slower change of the same total size is cheaper (rate squared, not the total).
  std::vector<Control> ramp(kT, Control{0.5, 0.1});
  for (int t = 0; t < 5; ++t) ramp[t].steering_cmd = 0.1 * (t + 1) / 5.0;
  in.prev_cmd = Control{0.5, 0.0};
  CostBreakdown slow, fast;
  s.evaluate_sequence(in, ramp.data(), &slow);
  std::vector<Control> jump(kT, Control{0.5, 0.1});
  s.evaluate_sequence(in, jump.data(), &fast);
  EXPECT_LT(slow.steer_rate, fast.steer_rate);
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
                  bd.steer_rate + bd.terminal,
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

// ---------------------------------------------------------------------------------------------
// Sampling, temperature and smoothing

TEST(SpeedSet, SnapRules) {
  EXPECT_EQ(snap_speed(0.0, 0.2, 0.6), 0.0);
  EXPECT_EQ(snap_speed(0.09, 0.2, 0.6), 0.0);  // below half the floor
  EXPECT_EQ(snap_speed(0.1, 0.2, 0.6), 0.2);   // from half the floor up to the floor
  EXPECT_EQ(snap_speed(0.15, 0.2, 0.6), 0.2);
  EXPECT_EQ(snap_speed(0.2, 0.2, 0.6), 0.2);
  EXPECT_EQ(snap_speed(0.4, 0.2, 0.6), 0.4);
  EXPECT_EQ(snap_speed(0.9, 0.2, 0.6), 0.6);   // the cap wins
  EXPECT_EQ(snap_speed(0.4, 0.2, 0.15), 0.0);  // a cap under the floor leaves only 0
  EXPECT_EQ(snap_speed(0.4, 0.2, 0.0), 0.0);
  EXPECT_EQ(snap_speed(-0.4, 0.2, 0.6), 0.0);
  EXPECT_EQ(snap_speed(std::nan(""), 0.2, 0.6), 0.0);
  EXPECT_EQ(snap_speed(0.05, 0.0, 0.6), 0.05);  // floor 0: no rule
  EXPECT_EQ(snap_speed(0.7, 0.0, 0.6), 0.6);
}

TEST(MppiSolver, TwoSolvesWithTheSameInputsAgreeBitForBit) {
  Env e;
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 256;
  p.seed = 11;
  p.speed_floor_mps = 0.2;
  auto a = e.solver(p, 256);
  auto b = e.solver(p, 256);
  e.set_points(wall_across(1.5));
  SolveInput in = e.input(0.3, 0.05, 0.6);
  in.x0.y = 0.1;
  for (int i = 0; i < 4; ++i) {
    const SolveOutput oa = a.solve(in), ob = b.solve(in);
    EXPECT_EQ(oa.cmd.speed_cmd, ob.cmd.speed_cmd);
    EXPECT_EQ(oa.cmd.steering_cmd, ob.cmd.steering_cmd);
    EXPECT_EQ(oa.best_cost, ob.best_cost);
    EXPECT_EQ(oa.collision_fraction, ob.collision_fraction);
    ASSERT_EQ(a.mean_sequence().size(), b.mean_sequence().size());
    for (std::size_t t = 0; t < a.mean_sequence().size(); ++t) {
      EXPECT_EQ(a.mean_sequence()[t].steering_cmd, b.mean_sequence()[t].steering_cmd);
      EXPECT_EQ(a.mean_sequence()[t].speed_cmd, b.mean_sequence()[t].speed_cmd);
    }
  }
  // after a reset the same input gives the first answer again: no random draw happens in solve
  a.reset();
  const SolveOutput first_a = a.solve(in);
  a.reset();
  const SolveOutput again_a = a.solve(in);
  EXPECT_EQ(first_a.cmd.steering_cmd, again_a.cmd.steering_cmd);
  EXPECT_EQ(first_a.cmd.speed_cmd, again_a.cmd.speed_cmd);
}

TEST(MppiSolver, ArcPrimitivesAreSampledAndReachBothSteeringLimits) {
  Env e;
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 64;
  p.speed_floor_mps = 0.2;
  auto s = e.solver(p, 64);
  s.solve(e.input(0.3, 0.0, 0.6));
  const auto& v = s.samples_buffer();
  const int T = s.horizon();
  // sample 1: the nominal steering at speed 0
  for (int t = 0; t < T; ++t) EXPECT_EQ(v[static_cast<std::size_t>(T + t)].speed_cmd, 0.0);
  // samples 2 to 28: every (steering, speed) pair once, constant along the horizon
  std::set<std::pair<int, int>> seen;
  const double speeds[3] = {0.2, 0.6, 0.4};
  double lo = 1e9, hi = -1e9;
  for (int a = 0; a < kArcSteeringValues * kArcSpeedValues; ++a) {
    const Control* arc = &v[static_cast<std::size_t>(2 + a) * T];
    for (int t = 1; t < T; ++t) {
      ASSERT_EQ(arc[t].speed_cmd, arc[0].speed_cmd);
      ASSERT_EQ(arc[t].steering_cmd, arc[0].steering_cmd);
    }
    lo = std::min(lo, arc[0].steering_cmd);
    hi = std::max(hi, arc[0].steering_cmd);
    const int is = static_cast<int>(std::lround((arc[0].steering_cmd - e.g.steering_min_rad) /
                                                (e.g.steering_max_rad - e.g.steering_min_rad) *
                                                (kArcSteeringValues - 1)));
    int sp = -1;
    for (int k = 0; k < 3; ++k) {
      if (std::fabs(arc[0].speed_cmd - speeds[k]) < 1e-12) sp = k;
    }
    ASSERT_GE(sp, 0) << arc[0].speed_cmd;
    seen.insert({is, sp});
    EXPECT_NEAR(arc[0].steering_cmd,
                e.g.steering_min_rad +
                    (e.g.steering_max_rad - e.g.steering_min_rad) * is / (kArcSteeringValues - 1),
                1e-12);
  }
  EXPECT_EQ(seen.size(), 27u);
  EXPECT_DOUBLE_EQ(lo, e.g.steering_min_rad);
  EXPECT_DOUBLE_EQ(hi, e.g.steering_max_rad);
}

TEST(MppiSolver, EveryCommandIsZeroOrAtLeastTheFloorAndWithinTheCap) {
  Env e;
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 128;
  p.speed_floor_mps = 0.2;
  auto s = e.solver(p, 128);
  for (double cap : {0.6, 0.25, 0.15, 0.0}) {
    s.reset();
    SolveInput in = e.input(0.1, 0.0, cap);
    s.solve(in);
    const auto& v = s.samples_buffer();
    for (int k = 0; k < s.samples(); ++k) {
      for (int t = 0; t < s.horizon(); ++t) {
        const double sp = v[static_cast<std::size_t>(k) * s.horizon() + t].speed_cmd;
        ASSERT_TRUE(sp == 0.0 || (sp >= 0.2 && sp <= cap + 1e-12)) << cap << " " << k << " " << sp;
      }
    }
  }
}

TEST(MppiSolver, NoiseIsAntitheticBetweenTheTwoHalves) {
  Env e;
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 129;  // 100 noise samples: 50 and their negations
  p.sigma_speed_mps = 0.05;
  p.sigma_steering_rad = 0.05;
  auto s = e.solver(p, 129);
  s.solve(
      e.input(0.3, 0.0, 1.0));  // the first nominal is (0.3, 0.0), nothing clips at these sigmas
  const auto& v = s.samples_buffer();
  const int T = s.horizon();
  const int m = 129 - kStructuredSamples;
  ASSERT_EQ(m % 2, 0);
  for (int j = 0; j < m / 2; ++j) {
    const Control* a = &v[static_cast<std::size_t>(kStructuredSamples + j) * T];
    const Control* b = &v[static_cast<std::size_t>(kStructuredSamples + j + m / 2) * T];
    for (int t = 0; t < T; ++t) {
      EXPECT_NEAR(a[t].steering_cmd + b[t].steering_cmd, 0.0, 1e-12);
      EXPECT_NEAR(a[t].speed_cmd + b[t].speed_cmd, 0.6, 1e-12);
    }
  }
}

TEST(MppiSolver, Smooth3TapKeepsConstantsSpreadsAnImpulseAndRepeatsTheEnds) {
  std::vector<Control> c(7, Control{0.3, 0.2});
  smooth_steering_3tap(c.data(), 7);
  for (const Control& x : c) {
    EXPECT_DOUBLE_EQ(x.steering_cmd, 0.2);
    EXPECT_DOUBLE_EQ(x.speed_cmd, 0.3);  // speed untouched
  }
  std::vector<Control> d(7, Control{0.0, 0.0});
  d[3].steering_cmd = 1.0;
  smooth_steering_3tap(d.data(), 7);
  EXPECT_DOUBLE_EQ(d[2].steering_cmd, 0.25);
  EXPECT_DOUBLE_EQ(d[3].steering_cmd, 0.5);
  EXPECT_DOUBLE_EQ(d[4].steering_cmd, 0.25);
  EXPECT_DOUBLE_EQ(d[1].steering_cmd, 0.0);
  std::vector<Control> ends(4, Control{0.0, 0.0});
  ends[0].steering_cmd = 1.0;
  ends[3].steering_cmd = -1.0;
  smooth_steering_3tap(ends.data(), 4);
  EXPECT_DOUBLE_EQ(ends[0].steering_cmd, 0.75);  // end repeated: 0.25 * 1 + 0.5 * 1 + 0.25 * 0
  EXPECT_DOUBLE_EQ(ends[3].steering_cmd, -0.75);
  EXPECT_DOUBLE_EQ(ends[1].steering_cmd, 0.25);
  smooth_steering_3tap(nullptr, 0);  // nothing to do, no crash
}

TEST(MppiSolver, WarmStartAdvancesTheMeanByTickOverDtOfAStep) {
  Env e;
  e.set_points(wall_across(2.0));
  for (double tick : {0.02, 0.05, 0.0}) {
    MppiParams p;
    p.horizon_steps = kT;
    p.samples = 128;
    p.tick_s = tick;
    auto s = e.solver(p, 128);
    s.solve(e.input(0.3, 0.0, 0.6));
    const double f = std::min(1.0, tick / e.dyn.dt_s);
    const auto& mean = s.mean_sequence();
    const auto& warm = s.nominal();
    for (int t = 0; t + 1 < kT; ++t) {
      EXPECT_NEAR(warm[t].steering_cmd,
                  (1.0 - f) * mean[t].steering_cmd + f * mean[t + 1].steering_cmd, 1e-12);
      EXPECT_NEAR(warm[t].speed_cmd, (1.0 - f) * mean[t].speed_cmd + f * mean[t + 1].speed_cmd,
                  1e-12);
    }
    EXPECT_EQ(warm[kT - 1].steering_cmd, mean[kT - 1].steering_cmd);  // the last value is held
  }
}

TEST(MppiSolver, FlatCostFieldGivesAFiniteWeightedMeanOfTheSamples) {
  // Every cost weight off: all clean samples cost the same, the scale floor keeps the weights
  // finite and uniform.
  Env e;
  MppiParams p = only_weights();
  p.samples = 128;
  p.sigma_speed_mps = 0.0;
  p.sigma_steering_rad = 0.0;
  auto s = e.solver(p, 128);
  const SolveOutput out = s.solve(e.input(0.3, 0.0, 0.6));
  EXPECT_TRUE(std::isfinite(out.cmd.speed_cmd));
  EXPECT_TRUE(std::isfinite(out.cmd.steering_cmd));
  EXPECT_FALSE(out.all_collide);
  // The arcs are symmetric in steering, so the uniform mean steers straight.
  EXPECT_NEAR(out.cmd.steering_cmd, 0.0, 0.05);
}

TEST(MppiSolver, SteeringCommandDoesNotChatterInAStaticScene) {
  // The old solver drew fresh noise every tick and its mean wandered by sigma / sqrt(N). With the
  // frozen noise table the same scene gives the same command tick after tick.
  Env e;
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 512;
  p.seed = 4;
  p.speed_floor_mps = 0.2;
  auto s = e.solver(p, 768);
  SolveInput in = e.input(0.35, 0.0, 0.6);
  in.x0.y = 0.08;
  in.x0.yaw = 0.05;
  for (int i = 0; i < 20; ++i) s.solve(in);  // let the warm start settle
  double prev = s.solve(in).cmd.steering_cmd;
  double sum2 = 0.0;
  int flips = 0;
  double prev_d = 0.0;
  const int n = 200;
  for (int i = 0; i < n; ++i) {
    const double cur = s.solve(in).cmd.steering_cmd;
    const double d = cur - prev;
    sum2 += d * d;
    if (i > 0 && d * prev_d < 0.0 && std::fabs(d) > 1e-6 && std::fabs(prev_d) > 1e-6) ++flips;
    prev_d = d;
    prev = cur;
  }
  const double rms = std::sqrt(sum2 / n);
  EXPECT_LT(rms, 0.004);
  EXPECT_LT(flips, n / 10);
}

// ---------------------------------------------------------------------------------------------
// The blend of the track prior and the free-space field

namespace {

// The Env of the tests above with a scan of a 1 m corridor closed 5 m ahead and the free-space
// field built from it.
struct BlendEnv : Env {
  FreespaceField fs{g};
  std::vector<PointXY> pts;

  explicit BlendEnv(int valid = kTrackStations, double hwl = 0.5, double hwr = 0.5)
      : Env(valid, hwl, hwr) {
    for (double x = -1.0; x <= 5.0; x += 0.02) {
      pts.push_back({x, 0.5});
      pts.push_back({x, -0.5});
    }
    for (double y = -0.5; y <= 0.5; y += 0.02) pts.push_back({5.0, y});
    set_points(pts);
    fs.build(field, pts.data(), pts.size(), nullptr);
  }

  SolveInput blend_input(double v, double c) {
    SolveInput in = input(v);
    in.confidence = c;
    in.freespace = &fs;
    return in;
  }

  double phi_front(double x, double y, double yaw) const {
    return fs.phi(x + g.wheelbase_m * std::cos(yaw), y + g.wheelbase_m * std::sin(yaw));
  }
};

}  // namespace

TEST(MppiBlend, FieldIsValidOnTheTestScene) {
  BlendEnv e;
  ASSERT_TRUE(e.fs.valid());
  EXPECT_GT(e.fs.goal().x, 3.5);
}

TEST(MppiBlend, ProgressIsTheBlendOfTrackProgressAndPhiDropAtCZeroHalfAndOne) {
  BlendEnv e;
  MppiParams p = only_weights();
  p.w_progress = 10.0;
  auto s = e.solver(p);
  const auto seq = e.constant(0.5, 0.0);
  SolveInput in = e.input(0.5);
  in.freespace = &e.fs;
  // the car moves 0.75 m along the corridor (x from 0 to about 0.75 at constant speed 0.5)
  in.confidence = 1.0;
  CostBreakdown one;
  s.evaluate_sequence(in, seq.data(), &one);
  const double ds = one.progress_m;
  EXPECT_NEAR(ds, 0.75, 1e-9);
  in.confidence = 0.0;
  CostBreakdown zero;
  s.evaluate_sequence(in, seq.data(), &zero);
  // the free-space drop between the front axle points of the first and the last pose
  const double dphi = e.phi_front(0.0, 0.0, 0.0) - e.phi_front(0.75, 0.0, 0.0);
  EXPECT_NEAR(zero.progress_m, dphi, 1e-6);
  EXPECT_GT(dphi, 0.5);
  EXPECT_NEAR(zero.progress, -10.0 * dphi, 1e-5);
  in.confidence = 0.5;
  CostBreakdown half;
  s.evaluate_sequence(in, seq.data(), &half);
  EXPECT_NEAR(half.progress_m, 0.5 * ds + 0.5 * dphi, 1e-6);
  EXPECT_NEAR(half.progress, -10.0 * half.progress_m, 1e-9);
  EXPECT_NEAR(half.total, half.progress, 1e-12);
  // c = 1 is the track alone, whatever the field says
  EXPECT_NEAR(one.progress, -10.0 * ds, 1e-9);
  // and without a field the free-space part is 0
  SolveInput nofield = e.input(0.5);
  nofield.confidence = 0.5;
  CostBreakdown only_track;
  s.evaluate_sequence(nofield, seq.data(), &only_track);
  EXPECT_NEAR(only_track.progress_m, 0.5 * ds, 1e-9);
}

TEST(MppiBlend, NoTrackAtAllScoresFromTheFieldAlone) {
  BlendEnv e;
  MppiParams p = only_weights();
  p.w_progress = 10.0;
  auto s = e.solver(p);
  const auto seq = e.constant(0.5, 0.0);
  SolveInput in = e.input(0.5);
  in.track = nullptr;   // no track message
  in.confidence = 1.0;  // whatever it says, there is nothing to project on
  in.freespace = &e.fs;
  CostBreakdown bd;
  s.evaluate_sequence(in, seq.data(), &bd);
  const double dphi = e.phi_front(0.0, 0.0, 0.0) - e.phi_front(0.75, 0.0, 0.0);
  EXPECT_NEAR(bd.progress_m, dphi, 1e-6);
  EXPECT_EQ(bd.s_end_m, 0.0);  // the track was never projected
}

TEST(MppiBlend, BoundaryTermIsWeightedByTheConfidence) {
  BlendEnv e(kTrackStations, 0.5, 0.3);
  MppiParams p = only_weights();
  p.w_bound = 200.0;
  auto s = e.solver(p);
  const auto seq = e.constant(0.0, 0.0);
  SolveInput in = e.blend_input(0.0, 1.0);
  in.x0.y = 0.4;
  CostBreakdown one, half, zero;
  s.evaluate_sequence(in, seq.data(), &one);
  in.confidence = 0.5;
  s.evaluate_sequence(in, seq.data(), &half);
  in.confidence = 0.0;
  s.evaluate_sequence(in, seq.data(), &zero);
  ASSERT_GT(one.boundary, 0.0);
  EXPECT_NEAR(half.boundary, 0.5 * one.boundary, 1e-9);
  EXPECT_EQ(zero.boundary, 0.0);
}

TEST(MppiBlend, StopWithinSightUsesTheBlendOfTheTrackAndPhi) {
  BlendEnv e(3);  // the track is visible to 0.4 m only
  MppiParams p = only_weights();
  p.w_term = 50.0;
  auto s = e.solver(p);
  const auto seq = e.constant(1.0, 0.0);
  const double decel = e.g.stop_deceleration_mps2;
  SolveInput in = e.blend_input(1.0, 1.0);
  CostBreakdown one;
  s.evaluate_sequence(in, seq.data(), &one);
  const double need1 = one.v_end_mps * one.v_end_mps / (2.0 * decel) - (0.4 - one.s_end_m);
  ASSERT_GT(need1, 0.0);
  EXPECT_NEAR(one.terminal, 50.0 * need1 * need1, 1e-9);
  // c = 0 far from the goal: plenty of room in sight, no cost
  in.confidence = 0.0;
  CostBreakdown zero;
  s.evaluate_sequence(in, seq.data(), &zero);
  EXPECT_EQ(zero.terminal, 0.0);
  // c = 0 and ending 0.1 m short of the goal at 1 m/s: the need is v^2 / (2 decel) - Phi(end)
  SolveInput close = e.blend_input(1.0, 0.0);
  const double x_end_front = e.fs.goal().x - 0.1;
  close.x0.x = x_end_front - e.g.wheelbase_m - 1.5;
  close.x0.y = e.fs.goal().y;
  CostBreakdown near;
  s.evaluate_sequence(close, seq.data(), &near);
  const double phi_e = e.phi_front(close.x0.x + 1.5, close.x0.y, 0.0);
  const double need0 = 1.0 / (2.0 * decel) - phi_e;
  ASSERT_GT(need0, 0.0);
  EXPECT_NEAR(near.terminal, 50.0 * need0 * need0, 1e-6);
  // c = 0.5: the sight is half the track's (nothing left of it at the end) and half Phi
  close.confidence = 0.5;
  CostBreakdown mixed;
  s.evaluate_sequence(close, seq.data(), &mixed);
  const double need_half = 1.0 / (2.0 * decel) - 0.5 * (0.4 - mixed.s_end_m) - 0.5 * phi_e;
  ASSERT_GT(need_half, 0.0);
  EXPECT_NEAR(mixed.terminal, 50.0 * need_half * need_half, 1e-6);
}

TEST(MppiBlend, TrackWithZeroConfidenceIsNotProjectedAndTheFieldDrivesTheSolve) {
  BlendEnv e;
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 256;
  p.speed_floor_mps = 0.0;
  auto s = e.solver(p, 256);
  SolveInput in = e.blend_input(0.3, 0.0);
  in.track = nullptr;
  in.v_cap_mps = 0.6;
  SolveOutput out;
  for (int i = 0; i < 20; ++i) out = s.solve(in);
  EXPECT_FALSE(out.all_collide);
  EXPECT_GT(out.cmd.speed_cmd, 0.3);
  EXPECT_NEAR(out.cmd.steering_cmd, 0.0, 0.1);
  EXPECT_GT(out.plan_progress_m, 0.3);
  EXPECT_GT(out.plan_speed_mps, 0.3);
}

TEST(MppiBlend, FieldSteersTowardAGoalAroundACornerWithoutAnyTrack) {
  // A room whose only exit (and so the farthest cell) is to the left of the car's heading.
  VehicleGeometry g = test::binding_geometry();
  DynamicsParams dyn = test::plain_dynamics();
  DynamicsModel model(dyn, g, dyn.dt_s);
  std::vector<PointXY> pts;
  auto wall = [&](double x1, double y1, double x2, double y2) {
    const int n = static_cast<int>(std::ceil(std::hypot(x2 - x1, y2 - y1) / 0.02));
    for (int i = 0; i <= n; ++i) pts.push_back({x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n});
  };
  // a corridor ahead for 1.2 m, then it turns left
  wall(-1.0, -0.5, 2.0, -0.5);
  wall(2.0, -0.5, 2.0, 6.0);
  wall(-1.0, 0.5, 1.0, 0.5);
  wall(1.0, 0.5, 1.0, 6.0);
  wall(-1.0, -0.5, -1.0, 0.5);
  DistanceField field;
  field.build(pts.data(), pts.size());
  FreespaceField fs(g);
  ASSERT_TRUE(fs.build(field, pts.data(), pts.size(), nullptr));
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 512;
  MppiSolver solver(p, model, g, 768);
  SolveInput in;
  in.x0.v = 0.3;
  in.x0.x = 0.4;
  in.v_cap_mps = 0.5;
  in.field = &field;
  in.freespace = &fs;
  in.confidence = 0.0;
  in.prev_cmd = Control{0.3, 0.0};
  SolveOutput out;
  for (int i = 0; i < 15; ++i) out = solver.solve(in);
  EXPECT_FALSE(out.all_collide);
  EXPECT_GT(out.cmd.speed_cmd, 0.1);
  EXPECT_GT(out.plan_progress_m, 0.1);
}

TEST(MppiSolver, RolloutPlanMatchesTheModelStepWithExactTrig) {
  // The rollout rotates (cos, sin) by the yaw increment with a series instead of calling libm every
  // step: over a hard turn it must still agree with the model integrated with exact trig.
  Env e;
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 128;
  auto s = e.solver(p, 128);
  SolveInput in = e.input(0.5, 0.35, 0.9);
  in.x0.yaw = 1.0;
  in.x0.x = 0.2;
  in.x0.y = -0.1;
  s.solve(in);
  const auto& u = s.mean_sequence();
  const auto& plan = s.plan();
  State st = in.x0;
  for (int t = 0; t < kT; ++t) {
    st = e.model.step(st, u[static_cast<std::size_t>(t)]);
    EXPECT_NEAR(plan[static_cast<std::size_t>(t + 1)].x, st.x, 1e-9) << t;
    EXPECT_NEAR(plan[static_cast<std::size_t>(t + 1)].y, st.y, 1e-9) << t;
    EXPECT_NEAR(plan[static_cast<std::size_t>(t + 1)].yaw, st.yaw, 1e-12) << t;
  }
}

// ---------------------------------------------------------------------------------------------
// The optional hooks (gate table, lap profile) must not change anything while they are off. The
// numbers in kGolden below were recorded from the solver on main BEFORE the hooks existed (same
// scenes, same seeds), as exact hex floats: with no gate table and no profile the solver is
// bit-identical to it.

namespace {

constexpr int kGoldenScenes = 5;
constexpr int kGoldenSolves = 3;
constexpr int kGoldenFields = 6;

using GoldenRow = std::array<double, kGoldenFields>;

GoldenRow golden_row(const SolveOutput& o) {
  return GoldenRow{o.cmd.speed_cmd,   o.cmd.steering_cmd,     o.best_cost,
                   o.plan_progress_m, o.plan_min_clearance_m, o.collision_fraction};
}

std::array<GoldenRow, kGoldenSolves> golden_scene(int scene) {
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 256;
  p.speed_floor_mps = 0.2;
  p.seed = 5 + static_cast<std::uint64_t>(scene);
  std::array<GoldenRow, kGoldenSolves> rows{};
  BlendEnv e;
  SolveInput in = e.input(0.4, 0.0, 0.8);
  switch (scene) {
    case 0:  // track only, a little off the centreline
      in.x0.y = 0.05;
      in.x0.yaw = 0.05;
      break;
    case 1:  // free-space field alone
      in = e.blend_input(0.5, 0.0);
      in.x0.y = 0.1;
      in.x0.yaw = 0.1;
      in.v_cap_mps = 0.8;
      break;
    case 2: {  // blend, with a wall across the corridor
      std::vector<PointXY> pts = e.pts;
      for (double y = -0.5; y <= 0.5; y += 0.02) pts.push_back({2.0, y});
      e.set_points(pts);
      in = e.blend_input(0.5, 0.5);
      in.v_cap_mps = 1.0;
      in.x0.delta = 0.05;
      break;
    }
    case 3: {  // an obstacle ahead and to the left
      std::vector<PointXY> pts = e.pts;
      pts.push_back({0.9, 0.25});
      e.set_points(pts);
      in = e.input(0.6, 0.05, 1.0);
      break;
    }
    default:  // a wall close ahead
      e.set_points(wall_across(0.8));
      in = e.input(0.8, 0.0, 1.0);
      break;
  }
  MppiSolver s(p, e.model, e.g, 256);
  for (int i = 0; i < kGoldenSolves; ++i) {
    rows[static_cast<std::size_t>(i)] = golden_row(s.solve(in));
  }
  return rows;
}

// Recorded on aarch64 (GCC 11, -O2, the ros-dev image) at the commit before the hooks. Columns:
// speed cmd, steering cmd, best cost, plan progress, plan min clearance, collision fraction.
const GoldenRow kGolden[kGoldenScenes * kGoldenSolves] = {
    {0x1.4f4fdce162fedp-1, -0x1.fb7dad9b401c6p-5, -0x1.1beb3711c6925p+3, 0x1.d610fd3b583f3p-1,
     0x1.1923d6cd28e4fp-2, 0x1.8p-5},
    {0x1.76c200a83f73fp-1, -0x1.c456190926393p-4, -0x1.37170033c1bep+3, 0x1.0235b5136361cp+0,
     0x1.19b4c89519ec9p-2, 0x1p-4},
    {0x1.84cd83cc6c8bdp-1, -0x1.1fb9b62937352p-3, -0x1.3f904048e67b4p+3, 0x1.08542e21f5527p+0,
     0x1.19b8496faed1ep-2, 0x1.2p-4},
    {0x1.2b3427d04c1abp-1, -0x1.c906d963215bep-4, -0x1.0f0967cb2c32ap+3, 0x1.da76a023783p-1,
     0x1.a09b921d97576p-3, 0x1.3p-3},
    {0x1.4d1384a31cfcdp-1, -0x1.4fdd333225cdap-3, -0x1.10fdd7c068b4fp+3, 0x1.0973ad66dd7e8p+0,
     0x1.a2653a457f34cp-3, 0x1.ep-5},
    {0x1.66e7db3139a48p-1, -0x1.a9fcb9ae9b2e5p-3, -0x1.2778d155c0b99p+3, 0x1.166e2322d9b4ap+0,
     0x1.a370a377f204ap-3, 0x1.3p-4},
    {0x1.ac6ca6e9cc65bp-1, -0x1.2a8e98aa993ep-8, -0x1.7e174f7c9bacp+3, 0x1.2c068279ffa72p+0,
     0x1.59283105ffd0ap-2, 0x1.cp-5},
    {0x1.c3c378af4e712p-1, 0x1.d836cfab70017p-10, -0x1.7e174f7c9bacp+3, 0x1.3e00f5ec19bc2p+0,
     0x1.3f99a5ca0bad8p-2, 0x1.58p-3},
    {0x1.cabf4e199298ap-1, 0x1.c4160882b2771p-8, -0x1.83bdc63f46d94p+3, 0x1.428f597cbc301p+0,
     0x1.2fb47a978891ep-2, 0x1.8p-3},
    {0x1.30fb2d6ee3f89p-1, -0x1.40e2f0358c2fcp-4, -0x1.7dc759684014p+2, 0x1.b89bd6fd2c72p-1,
     0x1.5d4193a9a9ffap-3, 0x1p-2},
    {0x1.3cc9db13f0c47p-1, -0x1.e9019ffe5792ep-4, -0x1.8ebf9598e029cp+2, 0x1.a457c672780bcp-1,
     0x1.6cae692563cadp-3, 0x1.ap-4},
    {0x1.50ef69806df31p-1, -0x1.0e885bf68dd2ap-3, -0x1.aa1604ca9bbeap+2, 0x1.a95483d1ce00dp-1,
     0x1.86f2858d2389ep-3, 0x1.cp-4},
    {0x0p+0, 0x0p+0, -0x1.35aa0c1496038p+0, 0x1.1e347382e14abp-2, 0x1.48ebd7424a974p-4, 0x1.fep-1},
    {0x1.9c3e6427e557fp-5, 0x1.47dcb2757667p-11, -0x1.35aa0c1496038p+0, 0x1.1fd49d350792ap-2,
     0x1.41cbf1fc92e58p-4, 0x1.0ap-1},
    {0x1.eea3f02c5e5edp-5, 0x1.05101a15ce27ap-8, -0x1.3381967ef252cp+0, 0x1.1fa98915f9b7fp-2,
     0x1.41e42bbd1fea8p-4, 0x1.1p-1},
};

}  // namespace

TEST(MppiSolver, WithoutTheHooksTheSolverIsBitIdenticalToTheOneBeforeThem) {
  for (int sc = 0; sc < kGoldenScenes; ++sc) {
    const auto rows = golden_scene(sc);
    for (int i = 0; i < kGoldenSolves; ++i) {
      const GoldenRow& want = kGolden[sc * kGoldenSolves + i];
      const GoldenRow& got = rows[static_cast<std::size_t>(i)];
      for (int f = 0; f < kGoldenFields; ++f) {
        const std::string what = "scene " + std::to_string(sc) + " solve " + std::to_string(i) +
                                 " field " + std::to_string(f);
#if defined(__aarch64__)
        // The platform the numbers were recorded on: exact.
        EXPECT_EQ(got[static_cast<std::size_t>(f)], want[static_cast<std::size_t>(f)]) << what;
#else
        // Another architecture contracts multiply-adds differently: same numbers to 1e-9.
        EXPECT_NEAR(got[static_cast<std::size_t>(f)], want[static_cast<std::size_t>(f)],
                    1e-9 * std::max(1.0, std::fabs(want[static_cast<std::size_t>(f)])))
            << what;
#endif
      }
    }
  }
}

// ---------------------------------------------------------------------------------------------
// The gate aware cost

namespace {

MppiParams gate_only() {
  MppiParams p = only_weights();
  p.w_gate = 300.0;  // a weight that makes the term stand out of the other terms' noise
  return p;
}

// An Env with a gate table built from a wall across the road `x_head` ahead of the LiDAR.
struct GateEnv : Env {
  GateConfig cfg = test::binding_gate();
  GateTable table{cfg.geometry, cfg.limits};
  test::ScanData scan = test::make_scan();

  explicit GateEnv(double wall_x_head = 1.0) {
    scan = test::scan_of(cfg.geometry, test::wall_across_road(wall_x_head));
    table.build(scan.view());
  }

  SolveInput gate_input(double v, double delta = 0.0, double v_cap = 2.0) {
    SolveInput in = input(v, delta, v_cap);
    in.gate = &table;
    in.wheel_speed_mps = v;
    return in;
  }

  double ahead(double x_head) const {
    return x_head + cfg.geometry.lidar_mount_x_m - cfg.geometry.body_front_x_m;
  }
};

// The gate cost of the first kGateCostSteps steps of a straight constant speed sequence from the
// closed form, for a wall d0 ahead of the body.
double expected_gate_cost(double d0, double v, const MppiParams& p, const GateLimits& lim) {
  double sum = 0.0;
  for (int k = 0; k < kGateCostSteps; ++k) {
    const double d = d0 - v * 0.05 * k;
    const double ttc = d / v;
    const double a = std::max(0.0, 1.0 - ttc / (p.gate_ttc_margin * lim.ttc_warning_s));
    const double b = std::max(0.0, 1.0 - d / (p.gate_clear_margin * lim.min_forward_clearance_m));
    sum += a * a + b * b;
  }
  return p.w_gate * sum;
}

}  // namespace

TEST(MppiGate, TermMatchesTheFormulaForAStraightApproach) {
  GateEnv e(1.0);
  const MppiParams params = gate_only();
  auto s = e.solver(params);
  const double v = 1.5;
  CostBreakdown bd;
  const auto seq = e.constant(v, 0.0);
  s.evaluate_sequence(e.gate_input(v), seq.data(), &bd);
  // the table reads the smaller of two grid cells around the straight arc, so compare with the
  // distance it gives
  const double d0 = e.table.distance(0.0);
  EXPECT_NEAR(bd.gate, expected_gate_cost(d0, v, params, e.cfg.limits), 1e-9);
  EXPECT_GT(bd.gate, 1.0);
  EXPECT_NEAR(bd.total, bd.gate, 1e-12) << "nothing else is on";
}

TEST(MppiGate, ASequenceThatEntersTheBandCostsMoreThanOneThatBrakesEarlierOrSteersClear) {
  GateEnv e(1.0);
  auto s = e.solver(gate_only());
  CostBreakdown straight, brake, arc, slow;
  const double v = 1.5;
  const auto into = e.constant(v, 0.0);
  s.evaluate_sequence(e.gate_input(v), into.data(), &straight);
  // brakes at the first step: a zero request is never judged by the gate
  std::vector<Control> braking = e.constant(0.0, 0.0);
  s.evaluate_sequence(e.gate_input(v), braking.data(), &brake);
  // steers the arc clear: full lock misses the wall in the scan
  const auto clear = e.constant(v, e.cfg.geometry.steering_max_rad);
  s.evaluate_sequence(e.gate_input(v, e.cfg.geometry.steering_max_rad), clear.data(), &arc);
  // slower into the same wall: later, shallower entry
  const auto crawl = e.constant(0.8, 0.0);
  s.evaluate_sequence(e.gate_input(0.8), crawl.data(), &slow);
  EXPECT_GT(straight.gate, brake.gate);
  EXPECT_GT(straight.gate, arc.gate);
  EXPECT_GT(straight.gate, slow.gate);
  EXPECT_EQ(brake.gate, 0.0);
  EXPECT_EQ(arc.gate, 0.0);
  // a sequence that brakes after a step or two costs less than one that never does
  std::vector<Control> late = e.constant(v, 0.0);
  for (int t = 2; t < kT; ++t) late[static_cast<std::size_t>(t)].speed_cmd = 0.0;
  CostBreakdown late_bd;
  s.evaluate_sequence(e.gate_input(v), late.data(), &late_bd);
  EXPECT_LT(late_bd.gate, straight.gate);
  EXPECT_GT(late_bd.gate, 0.0);
}

TEST(MppiGate, ZeroWhenFarFromEverythingAndWithoutATable) {
  GateEnv far(6.0);  // a wall 6 m ahead: the whole first 0.3 s is far from the band
  auto s = far.solver(gate_only());
  CostBreakdown bd;
  const auto seq = far.constant(1.5, 0.0);
  s.evaluate_sequence(far.gate_input(1.5), seq.data(), &bd);
  EXPECT_EQ(bd.gate, 0.0);
  // an empty scan: nothing in any corridor
  GateEnv empty(1.0);
  std::fill(empty.scan.ranges.begin(), empty.scan.ranges.end(),
            std::numeric_limits<float>::infinity());
  empty.table.build(empty.scan.view());
  CostBreakdown be;
  s.evaluate_sequence(empty.gate_input(1.5), seq.data(), &be);
  EXPECT_EQ(be.gate, 0.0);
  // no table, a table that was never built, or a zero weight: no term and the same total
  GateEnv e(1.0);
  SolveInput in = e.gate_input(1.5);
  CostBreakdown with_table, no_table, unbuilt, zero_w;
  s.evaluate_sequence(in, seq.data(), &with_table);
  EXPECT_GT(with_table.gate, 0.0);
  in.gate = nullptr;
  s.evaluate_sequence(in, seq.data(), &no_table);
  EXPECT_EQ(no_table.gate, 0.0);
  GateTable never(e.cfg.geometry, e.cfg.limits);
  in.gate = &never;
  s.evaluate_sequence(in, seq.data(), &unbuilt);
  EXPECT_EQ(unbuilt.gate, 0.0);
  MppiParams p0 = gate_only();
  p0.w_gate = 0.0;
  auto s0 = e.solver(p0);
  in.gate = &e.table;
  s0.evaluate_sequence(in, seq.data(), &zero_w);
  EXPECT_EQ(zero_w.gate, 0.0);
}

TEST(MppiGate, AGapInFrontOfTheWallCountsTheDistanceAlreadyTravelled) {
  GateEnv e(1.0);
  auto s = e.solver(gate_only());
  const auto seq = e.constant(1.0, 0.0);
  CostBreakdown fresh, stale;
  SolveInput in = e.gate_input(1.0);
  s.evaluate_sequence(in, seq.data(), &fresh);
  in.gate_travel_since_scan_m = 0.2;  // the scan is 0.2 m of driving old
  s.evaluate_sequence(in, seq.data(), &stale);
  EXPECT_GT(stale.gate, fresh.gate);
  // garbage travel counts as none
  in.gate_travel_since_scan_m = std::nan("");
  CostBreakdown garbage;
  s.evaluate_sequence(in, seq.data(), &garbage);
  EXPECT_EQ(garbage.gate, fresh.gate);
}

TEST(MppiGate, OnlyAForwardRequestIsJudged) {
  GateEnv e(0.6);
  auto s = e.solver(gate_only());
  CostBreakdown bd;
  const auto stopped = e.constant(0.0, 0.0);
  SolveInput in = e.gate_input(0.0);
  s.evaluate_sequence(in, stopped.data(), &bd);
  EXPECT_EQ(bd.gate, 0.0) << "standing in front of a wall costs nothing from the gate";
}

TEST(MppiGate, IsFiniteForEveryInput) {
  GateEnv e(1.0);
  auto s = e.solver(gate_only());
  test::SceneGenerator gen(7);
  std::mt19937_64 rng(3);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  for (int n = 0; n < 300; ++n) {
    const test::ScanData scan = gen.make(e.cfg.geometry);
    e.table.build(scan.view());
    SolveInput in = e.gate_input(2.0 * u(rng), -0.4 + 0.8 * u(rng), 2.0);
    in.wheel_speed_mps = n % 7 == 0 ? std::nan("") : 3.0 * u(rng);
    in.gate_travel_since_scan_m = n % 5 == 0 ? std::numeric_limits<double>::infinity() : u(rng);
    std::vector<Control> seq(kT);
    for (auto& c : seq) c = Control{2.0 * u(rng), -0.449 + 0.898 * u(rng)};
    CostBreakdown bd;
    const double j = s.evaluate_sequence(in, seq.data(), &bd);
    EXPECT_TRUE(std::isfinite(j)) << n;
    EXPECT_TRUE(std::isfinite(bd.gate)) << n;
    EXPECT_GE(bd.gate, 0.0);
  }
}

TEST(MppiGate, TheSolverStaysOutOfTheGatesBandWhereWithoutTheTermItDrivesIn) {
  // A wall in the gate's scan 0.9 m ahead of the LiDAR that the planner's own distance field does
  // not know about (so only the gate term can object), a straight track and a 1.5 m/s cap.
  GateEnv e(0.9);
  auto make = [&](double w_gate) {
    MppiParams p;
    p.horizon_steps = kT;
    p.samples = 256;
    p.speed_floor_mps = 0.2;
    p.seed = 4;
    p.w_gate = w_gate;
    return e.solver(p, 256);
  };
  auto drive = [&](double w_gate) {
    MppiSolver solver = make(w_gate);
    SolveInput in = e.gate_input(1.0, 0.0, 1.5);
    SolveOutput out;
    for (int i = 0; i < 8; ++i) out = solver.solve(in);
    return out;
  };
  const SolveOutput with_term = drive(300.0);
  const SolveOutput without = drive(0.0);
  EXPECT_TRUE(
      would_trip_exact(e.table, without.cmd.steering_cmd, without.cmd.speed_cmd, 1.0, e.cfg.limits))
      << "without the term the solver asks the gate to brake";
  EXPECT_FALSE(with_term.cmd_would_trip);
  EXPECT_FALSE(would_trip_exact(e.table, with_term.cmd.steering_cmd, with_term.cmd.speed_cmd, 1.0,
                                e.cfg.limits));
}

// ---------------------------------------------------------------------------------------------
// The lap profile hook

namespace {

// A profile that starts at station s0 with a speed bound and a time to finish given by functions.
template <typename FV, typename FQ>
LapProfile make_profile(double s0, FV bound_at, FQ q_at, int n = 61) {
  std::vector<float> s, v, q;
  for (int i = 0; i < n; ++i) {
    const double st = s0 + 0.1 * i;
    s.push_back(static_cast<float>(st));
    v.push_back(static_cast<float>(bound_at(st)));
    q.push_back(static_cast<float>(q_at(st)));
  }
  LapProfile p;
  EXPECT_EQ(
      make_lap_profile(s.data(), v.data(), q.data(), s.size(), v.size(), q.size(), 1.0, 1.0, &p),
      LapProfileStatus::kOk);
  return p;
}

}  // namespace

TEST(MppiLap, TrustRegionCostsAboveTheBoundAndNothingAtOrBelowIt) {
  Env e;
  MppiParams p = only_weights();
  p.w_trust = 400.0;
  auto s = e.solver(p);
  const LapProfile prof = make_profile(
      0.0, [](double) { return 0.5; }, [](double st) { return 6.0 - st; });
  auto cost_at = [&](double speed) {
    SolveInput in = e.input(speed, 0.0, 1.5);
    in.lap = &prof;
    CostBreakdown bd;
    const auto seq = e.constant(speed, 0.0);
    s.evaluate_sequence(in, seq.data(), &bd);
    return bd;
  };
  const CostBreakdown at = cost_at(0.5), above = cost_at(0.9), below_a = cost_at(0.3),
                      below_b = cost_at(0.4);
  EXPECT_EQ(at.lap_trust, 0.0);
  EXPECT_EQ(below_a.lap_trust, 0.0);
  EXPECT_EQ(below_b.lap_trust, 0.0);
  EXPECT_GT(above.lap_trust, 0.0);
  // 400 * (0.9 - 0.5)^2 * dt over 30 steps
  EXPECT_NEAR(above.lap_trust, 400.0 * 0.4 * 0.4 * 0.05 * kT, 1e-9);
  // equal below the bound: the totals only differ by the terminal term (the same path length is
  // not the same here, so compare the trust terms alone)
  EXPECT_EQ(below_a.lap_trust, below_b.lap_trust);
}

TEST(MppiLap, TrustRegionFollowsTheStationOfEveryStepNotTheStartingOne) {
  Env e;
  MppiParams p = only_weights();
  p.w_trust = 400.0;
  auto s = e.solver(p);
  // fast for the first metre, slow after it
  const LapProfile prof = make_profile(
      0.0, [](double st) { return st < 1.0 ? 1.2 : 0.4; }, [](double st) { return 6.0 - st; });
  SolveInput in = e.input(0.8, 0.0, 1.5);
  in.lap = &prof;
  CostBreakdown bd;
  const auto seq = e.constant(0.8, 0.0);  // 1.2 m in the horizon: crosses into the slow part at 1 m
  s.evaluate_sequence(in, seq.data(), &bd);
  // only the steps past the station 1.0 are over: 0.8 m/s * 0.05 s = 0.04 m per step
  int steps_over = 0;
  for (int t = 0; t < kT; ++t) {
    if ((t + 1) * 0.04 >= 1.0) ++steps_over;
  }
  ASSERT_GT(steps_over, 0);
  EXPECT_NEAR(bd.lap_trust, 400.0 * 0.4 * 0.4 * 0.05 * steps_over, 0.5 * 400.0 * 0.16 * 0.05);
  // the same car starting 1 m further along is over the bound for the whole horizon
  in.lap_station_now_m = 1.0;
  CostBreakdown later;
  s.evaluate_sequence(in, seq.data(), &later);
  // (the bound 0.4 is stored as a float in the profile, hence 1e-4)
  EXPECT_NEAR(later.lap_trust, 400.0 * 0.4 * 0.4 * 0.05 * kT, 1e-4);
}

TEST(MppiLap, TerminalCostToGoReplacesProgressAndIsSecondsGainedTimesTheCap) {
  Env e;
  MppiParams p = only_weights();
  p.w_progress = 10.0;
  auto s = e.solver(p);
  // best lap slow before station 2 (2 s per metre), fast after it (0.5 s per metre)
  const LapProfile prof = make_profile(
      0.0, [](double) { return 1.5; },
      [](double st) { return st < 2.0 ? 10.0 - 2.0 * st : 6.0 - 0.5 * (st - 2.0); });
  const auto seq = e.constant(0.8, 0.0);  // the same 1.2 m of path everywhere
  auto run = [&](double station_now, double cap) {
    SolveInput in = e.input(0.8, 0.0, cap);
    in.lap = &prof;
    in.lap_station_now_m = station_now;
    CostBreakdown bd;
    s.evaluate_sequence(in, seq.data(), &bd);
    return bd;
  };
  const CostBreakdown slow_zone = run(0.2, 1.5);
  const CostBreakdown fast_zone = run(3.0, 1.5);
  const double path = 0.8 * 0.05 * kT;
  // seconds of the best lap gained: 2 s per metre in the slow zone, 0.5 in the fast one
  EXPECT_NEAR(slow_zone.lap_terminal, -10.0 * 1.5 * (2.0 * path), 1e-4);
  EXPECT_NEAR(fast_zone.lap_terminal, -10.0 * 1.5 * (0.5 * path), 1e-4);
  EXPECT_LT(slow_zone.lap_terminal, fast_zone.lap_terminal)
      << "the same path gains more best-lap time where the best lap was slow";
  EXPECT_EQ(slow_zone.progress, 0.0) << "the cost-to-go replaces the progress term";
  EXPECT_NEAR(slow_zone.total, slow_zone.lap_terminal + slow_zone.lap_trust, 1e-12);
  // the physical progress is still reported
  EXPECT_NEAR(slow_zone.progress_m, path, 1e-9);
  // scaled by the cap in force
  const CostBreakdown low_cap = run(0.2, 0.75);
  EXPECT_NEAR(low_cap.lap_terminal, 0.5 * slow_zone.lap_terminal, 1e-9);
  // and more path is cheaper still
  SolveInput in = e.input(1.2, 0.0, 1.5);
  in.lap = &prof;
  in.lap_station_now_m = 0.2;
  CostBreakdown longer;
  const auto fast_seq = e.constant(1.2, 0.0);
  s.evaluate_sequence(in, fast_seq.data(), &longer);
  EXPECT_LT(longer.lap_terminal, slow_zone.lap_terminal);
}

TEST(MppiLap, NoProfileOrAnInactiveOneIsTheOldProgressTerm) {
  Env e;
  MppiParams p = only_weights();
  p.w_progress = 10.0;
  auto s = e.solver(p);
  const auto seq = e.constant(0.5, 0.0);
  CostBreakdown plain, inactive;
  SolveInput in = e.input(0.5);
  s.evaluate_sequence(in, seq.data(), &plain);
  LapProfile off;  // active false
  in.lap = &off;
  in.lap_station_now_m = 3.0;
  s.evaluate_sequence(in, seq.data(), &inactive);
  EXPECT_EQ(plain.progress, inactive.progress);
  EXPECT_EQ(plain.total, inactive.total);
  EXPECT_EQ(inactive.lap_trust, 0.0);
  EXPECT_EQ(inactive.lap_terminal, 0.0);
  EXPECT_LT(plain.progress, 0.0);
}

TEST(MppiLap, TheProfileNeverRelaxesTheOtherTerms) {
  // Collision, clearance and the stop-within-sight term are unchanged by an active profile.
  Env e;
  e.set_points(wall_across(0.8));
  MppiParams p;
  p.horizon_steps = kT;
  p.d_hard_m = 0.03;
  auto s = e.solver(p);
  const LapProfile prof = make_profile(
      0.0, [](double) { return 1.5; }, [](double st) { return 6.0 - st; });
  const auto seq = e.constant(0.6, 0.0);
  SolveInput in = e.input(0.6);
  CostBreakdown base, with_lap;
  s.evaluate_sequence(in, seq.data(), &base);
  in.lap = &prof;
  s.evaluate_sequence(in, seq.data(), &with_lap);
  EXPECT_EQ(base.collision, with_lap.collision);
  EXPECT_EQ(base.clearance, with_lap.clearance);
  EXPECT_EQ(base.boundary, with_lap.boundary);
  EXPECT_EQ(base.lateral, with_lap.lateral);
  EXPECT_EQ(base.terminal, with_lap.terminal);
  EXPECT_EQ(base.smoothness, with_lap.smoothness);
  EXPECT_EQ(base.steer_rate, with_lap.steer_rate);
}

// L2 property: whatever profile passes make_lap_profile and whatever the input, the solver output
// is finite, within the cap and within the steering limits.
TEST(MppiLap, PropertyAnyValidProfileAndInputGivesAFiniteCommandWithinTheLimits) {
  Env e;
  const GateConfig cfg = test::binding_gate();
  GateTable table(cfg.geometry, cfg.limits);
  test::SceneGenerator gen(12);
  std::mt19937_64 rng(77);
  auto u = [&](double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(rng);
  };
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 128;
  p.speed_floor_mps = 0.2;
  auto solver = e.solver(p, 128);
  int accepted = 0;
  for (int n = 0; n < 150; ++n) {
    std::vector<float> s, v, q;
    const std::size_t len = 2 + static_cast<std::size_t>(u(0.0, 126.0));
    const double s0 = u(-2.0, 30.0);
    double qq = u(-5.0, 20.0);
    for (std::size_t i = 0; i < len; ++i) {
      s.push_back(static_cast<float>(s0 + 0.1 * static_cast<double>(i)));
      v.push_back(static_cast<float>(n % 4 == 0 ? 0.0 : u(0.0, 3.0)));
      q.push_back(static_cast<float>(qq));
      qq -= u(0.0, 0.5);
    }
    LapProfile prof;
    if (make_lap_profile(s.data(), v.data(), q.data(), len, len, len, 5.0, 5.0 + u(0.0, 0.5),
                         &prof) != LapProfileStatus::kOk) {
      continue;
    }
    ++accepted;
    const test::ScanData scan = gen.make(cfg.geometry);
    table.build(scan.view());
    const double cap = u(0.0, 2.0);
    SolveInput in = e.input(u(0.0, 2.0), u(-0.4, 0.4), cap);
    in.x0.y = u(-0.3, 0.3);
    in.x0.yaw = u(-0.4, 0.4);
    in.lap = &prof;
    in.lap_station_now_m = u(-1.0, 3.0);
    in.gate = &table;
    in.wheel_speed_mps = u(0.0, 2.0);
    in.gate_travel_since_scan_m = u(0.0, 0.5);
    const SolveOutput out = solver.solve(in);
    ASSERT_TRUE(std::isfinite(out.cmd.speed_cmd)) << n;
    ASSERT_TRUE(std::isfinite(out.cmd.steering_cmd)) << n;
    EXPECT_GE(out.cmd.speed_cmd, 0.0);
    EXPECT_LE(out.cmd.speed_cmd, cap + 1e-12);
    EXPECT_GE(out.cmd.steering_cmd, e.g.steering_min_rad - 1e-12);
    EXPECT_LE(out.cmd.steering_cmd, e.g.steering_max_rad + 1e-12);
    EXPECT_TRUE(std::isfinite(out.best_cost));
  }
  EXPECT_GT(accepted, 100);
}
