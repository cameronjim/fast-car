// Brief 42 (claude-docs/13 17.4): the final sequence check, freshness by source stamp, the
// actuator estimate from gated commands, one motion model for dead reckoning, and the speed
// limiter attribution of the timing chain. Every fault has a test that fails on the old code:
// the old solver is MppiParams::check_final_sequence = false, the old actuator estimate is a
// planner that is never told a gated sequence number.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "planner_rig.hpp"
#include "racer_planner/ego_motion.hpp"
#include "racer_planner/mppi.hpp"
#include "racer_planner/planner.hpp"
#include "racer_planner/planner_logic.hpp"
#include "test_support.hpp"

using namespace racer_planner;

namespace {

constexpr int kT = 30;

bool same_bits(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }

struct Scene {
  VehicleGeometry g = test::binding_geometry();
  DynamicsParams dyn = test::plain_dynamics();
  DynamicsModel model{dyn, g, dyn.dt_s};
  DistanceField field;
  TrackProjector track;
  LocalTrackData data;

  explicit Scene(double half_width = 1.0, int valid = kTrackStations) {
    data.frame_id = "base_link";
    data.station_spacing_m = kTrackStationSpacingM;
    data.valid_count = valid;
    for (int k = 0; k < kTrackStations; ++k) {
      data.x_m.push_back(k * kTrackStationSpacingM);
      data.y_m.push_back(0.0);
      data.half_width_left_m.push_back(half_width);
      data.half_width_right_m.push_back(half_width);
      data.sigma_m.push_back(0.02);
    }
    track.build(data, Pose2{});
    field.build(nullptr, 0);
  }
  void set_points(const std::vector<PointXY>& pts) { field.build(pts.data(), pts.size()); }
  SolveInput input(double v, double v_cap) {
    SolveInput in;
    in.x0.v = v;
    in.v_cap_mps = v_cap;
    in.track = &track;
    in.field = &field;
    in.prev_cmd = Control{v, 0.0};
    return in;
  }
};

// A pillar on the centreline: a vertical segment, the track wide enough to pass it on either side.
std::vector<PointXY> pillar(double x, double half_y) {
  std::vector<PointXY> pts;
  for (double y = -half_y; y <= half_y + 1e-9; y += 0.01) pts.push_back({x, y});
  return pts;
}

MppiParams solver_params(bool check) {
  MppiParams p;
  p.horizon_steps = kT;
  p.samples = 256;
  p.seed = 3;
  p.check_final_sequence = check;
  p.speed_floor_mps = 0.2;
  return p;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// 1. Final sequence validation

namespace {
// The scene where two clean sample families straddle the pillar.
struct Straddle {
  Scene s{1.0};
  Straddle() { s.set_points(pillar(1.1, 0.15)); }
};
}  // namespace

TEST(FinalSequence, TheMeanOfTwoCleanFamiliesCollidesOnTheOldPathAndFallsBackOnTheNewOne) {
  Straddle st;
  MppiSolver old_path(solver_params(false), st.s.model, st.s.g, 256);
  const SolveInput in = st.s.input(0.8, 1.0);
  const SolveOutput so_old = old_path.solve(in);
  ASSERT_FALSE(so_old.all_collide);
  // The fault: the sequence the old solver commands from collides although clean samples exist.
  CostBreakdown bd_old;
  old_path.evaluate_sequence(in, old_path.mean_sequence().data(), &bd_old);
  EXPECT_TRUE(bd_old.collided) << "the constructed field must make the mean collide";
  EXPECT_TRUE(so_old.unchecked_final_collided);
  EXPECT_EQ(so_old.final_sequence, FinalSequence::kMean);  // nothing acted on it

  MppiSolver fixed(solver_params(true), st.s.model, st.s.g, 256);
  const SolveOutput so = fixed.solve(in);
  EXPECT_TRUE(so.unchecked_final_collided);
  EXPECT_EQ(so.final_sequence, FinalSequence::kFallback);
  CostBreakdown bd;
  fixed.evaluate_sequence(in, fixed.mean_sequence().data(), &bd);
  EXPECT_FALSE(bd.collided);
  EXPECT_TRUE(std::isfinite(bd.total));
  EXPECT_FALSE(so.final_cost.collided);

  // The command is the first command of the cheapest collision free sample of the plan's class.
  const std::vector<Control>& samples = fixed.samples_buffer();
  const double thr = std::max(solver_params(true).speed_floor_mps, 1e-3);
  const bool moves = so.plan_speed_mps >= thr;
  double best = 1e300;
  int best_k = -1;
  for (int k = 0; k < fixed.samples(); ++k) {
    const Control* vk = &samples[static_cast<std::size_t>(k) * kT];
    CostBreakdown c;
    const double j = fixed.evaluate_sequence(in, vk, &c);
    if (c.collided) continue;
    double sum = 0.0;
    const int win = 10;  // plan_window_s 0.5 / dt 0.05
    for (int t = 0; t < win; ++t) sum += vk[t].speed_cmd;
    if ((sum / win >= thr) != moves) continue;
    if (j < best) {
      best = j;
      best_k = k;
    }
  }
  ASSERT_GE(best_k, 0);
  const Control* vb = &samples[static_cast<std::size_t>(best_k) * kT];
  EXPECT_TRUE(same_bits(so.cmd.speed_cmd, vb[0].speed_cmd));
  EXPECT_TRUE(same_bits(so.cmd.steering_cmd, vb[0].steering_cmd));
  for (int t = 0; t < kT; ++t) {
    EXPECT_TRUE(same_bits(fixed.mean_sequence()[static_cast<std::size_t>(t)].steering_cmd,
                          vb[t].steering_cmd));
  }
  // The warm start advances from what was used: step 0 of the nominal is the step 0..1 blend of
  // the fallback sample.
  const double f = 0.02 / 0.05;
  EXPECT_NEAR(fixed.nominal()[0].steering_cmd,
              (1.0 - f) * vb[0].steering_cmd + f * vb[1].steering_cmd, 1e-12);
}

TEST(FinalSequence, ACleanMeanIsBitIdenticalWithTheCheckOnAndOff) {
  Scene s(1.0);
  s.set_points(pillar(4.0, 0.12));  // far ahead: nothing near the plan
  MppiSolver a(solver_params(false), s.model, s.g, 256);
  MppiSolver b(solver_params(true), s.model, s.g, 256);
  const SolveInput in = s.input(0.5, 1.0);
  for (int i = 0; i < 5; ++i) {
    const SolveOutput oa = a.solve(in);
    const SolveOutput ob = b.solve(in);
    EXPECT_EQ(ob.final_sequence, FinalSequence::kMean);
    EXPECT_FALSE(ob.unchecked_final_collided);
    EXPECT_TRUE(same_bits(oa.cmd.speed_cmd, ob.cmd.speed_cmd));
    EXPECT_TRUE(same_bits(oa.cmd.steering_cmd, ob.cmd.steering_cmd));
    EXPECT_TRUE(same_bits(oa.best_cost, ob.best_cost));
    EXPECT_TRUE(same_bits(oa.plan_progress_m, ob.plan_progress_m));
    EXPECT_TRUE(same_bits(oa.plan_min_clearance_m, ob.plan_min_clearance_m));
    for (int t = 0; t < kT; ++t) {
      const auto& ma = a.mean_sequence()[static_cast<std::size_t>(t)];
      const auto& mb = b.mean_sequence()[static_cast<std::size_t>(t)];
      EXPECT_TRUE(same_bits(ma.speed_cmd, mb.speed_cmd));
      EXPECT_TRUE(same_bits(ma.steering_cmd, mb.steering_cmd));
    }
  }
}

TEST(FinalSequence, EverySampleCollidingIsUnchanged) {
  Scene s(1.0);
  std::vector<PointXY> wall;
  for (double y = -1.0; y <= 1.0; y += 0.02) wall.push_back({0.35, y});
  s.set_points(wall);
  MppiSolver a(solver_params(false), s.model, s.g, 256);
  MppiSolver b(solver_params(true), s.model, s.g, 256);
  SolveInput in = s.input(0.8, 1.0);
  in.x0.x = 0.0;
  const SolveOutput oa = a.solve(in);
  const SolveOutput ob = b.solve(in);
  ASSERT_TRUE(ob.all_collide);
  EXPECT_EQ(ob.final_sequence, FinalSequence::kAllCollide);
  EXPECT_FALSE(ob.unchecked_final_collided);
  EXPECT_TRUE(same_bits(oa.cmd.speed_cmd, ob.cmd.speed_cmd));
  EXPECT_TRUE(same_bits(oa.cmd.steering_cmd, ob.cmd.steering_cmd));
  EXPECT_EQ(ob.cmd.speed_cmd, 0.0);
}

TEST(FinalSequence, CostTermsOfTheFinalSequenceAreReportedInTheDocumentedOrder) {
  Scene s(1.0);
  s.set_points(pillar(4.0, 0.12));
  MppiSolver m(solver_params(true), s.model, s.g, 256);
  const SolveInput in = s.input(0.5, 1.0);
  const SolveOutput so = m.solve(in);
  CostBreakdown bd;
  m.evaluate_sequence(in, m.mean_sequence().data(), &bd);
  double terms[kCostTermCount];
  cost_terms_array(so.final_cost, terms);
  EXPECT_DOUBLE_EQ(terms[0], bd.progress);
  EXPECT_DOUBLE_EQ(terms[2], bd.clearance);
  EXPECT_DOUBLE_EQ(terms[7], bd.terminal);
  EXPECT_DOUBLE_EQ(terms[8], bd.gate);
  EXPECT_DOUBLE_EQ(terms[9], bd.lap_trust);
  double sum = 0.0;
  for (int i = 0; i < kCostTermCount; ++i) sum += terms[i];
  EXPECT_NEAR(sum, so.final_cost.total, 1e-9);
}

// ---------------------------------------------------------------------------------------------
// Speed limiter attribution (the rule in mppi.hpp)

TEST(SpeedLimiter, AFreeCorridorDrivingAtTheCapIsTheCap) {
  Scene s(1.0);
  s.set_points({});
  MppiSolver m(solver_params(true), s.model, s.g, 256);
  const SolveInput in = s.input(0.5, 0.5);
  SolveOutput so;
  for (int i = 0; i < 6; ++i) so = m.solve(in);
  ASSERT_GE(so.plan_speed_mps, 0.5 - 0.05);
  EXPECT_EQ(so.speed_limiter, SpeedLimiter::kCap);
}

TEST(SpeedLimiter, AStandingPlanHasNoLimiter) {
  Scene s(1.0);
  std::vector<PointXY> wall;
  for (double y = -1.0; y <= 1.0; y += 0.02) wall.push_back({0.35, y});
  s.set_points(wall);
  MppiSolver m(solver_params(true), s.model, s.g, 256);
  const SolveOutput so = m.solve(s.input(0.0, 1.0));
  EXPECT_EQ(so.speed_limiter, SpeedLimiter::kNone);
}

TEST(SpeedLimiter, ATrackThatEndsSoonLimitsByTheTerminalTerm) {
  // Only 6 valid stations (0.9 m): stopping inside the visible track binds before the cap.
  Scene s(1.0, 6);
  s.set_points({});
  MppiParams p = solver_params(true);
  MppiSolver m(p, s.model, s.g, 256);
  const SolveInput in = s.input(0.9, 1.0);
  SolveOutput so;
  for (int i = 0; i < 6; ++i) so = m.solve(in);
  ASSERT_GE(so.plan_speed_mps, 0.2);
  EXPECT_EQ(so.speed_limiter, SpeedLimiter::kTerminal);
}

TEST(SpeedLimiter, ANarrowPassageLimitsByClearance) {
  // Walls 0.19 m from the centreline on both sides: inside the soft clearance band, outside the
  // hard one, so speed (which widens the band) is what clearance penalises.
  Scene s(1.0);
  std::vector<PointXY> pts;
  const double half = s.g.width_m * 0.5 + 0.22;
  for (double x = -1.0; x <= 6.0; x += 0.02) {
    pts.push_back({x, half});
    pts.push_back({x, -half});
  }
  s.set_points(pts);
  MppiSolver m(solver_params(true), s.model, s.g, 256);
  const SolveInput in = s.input(0.6, 1.0);
  SolveOutput so;
  for (int i = 0; i < 6; ++i) so = m.solve(in);
  ASSERT_GE(so.plan_speed_mps, 0.2);
  EXPECT_EQ(so.speed_limiter, SpeedLimiter::kClearance);
}

// ---------------------------------------------------------------------------------------------
// Freshness by source stamp

TEST(StampFreshness, ARepeatedOrOlderStampRefreshesNothing) {
  EXPECT_EQ(judge_stamp(false, 10.0, false, 0.0), StampVerdict::kAccept);  // the first one
  EXPECT_EQ(judge_stamp(false, 10.02, true, 10.0), StampVerdict::kAccept);
  EXPECT_EQ(judge_stamp(false, 10.0, true, 10.0), StampVerdict::kRejectNotLater);  // repeated
  EXPECT_EQ(judge_stamp(false, 9.5, true, 10.0), StampVerdict::kRejectNotLater);   // old, late
  EXPECT_EQ(judge_stamp(false, std::nan(""), true, 10.0), StampVerdict::kRejectNotLater);
}

TEST(StampFreshness, AZeroStampFallsBackToArrival) {
  EXPECT_EQ(judge_stamp(true, 0.0, false, 0.0), StampVerdict::kAcceptNoStamp);
  EXPECT_EQ(judge_stamp(true, 0.0, true, 99.0), StampVerdict::kAcceptNoStamp);
}

TEST(StampFreshness, AgeIsJudgedFromTheStampAtTheSameTimeoutsAsBefore) {
  // An honest source: stamp = arrival, so the age is what it always was and NO_ODOM trips at the
  // same age (odom_hard_timeout_s). A stale source (a stamp that stopped advancing) trips it by
  // its stamp although "messages keep arriving".
  LogicParams p;
  p.v_max_mps = 0.5;
  LogicInputs in;
  in.now_s = 100.0;
  in.has_odom = true;
  in.has_scan = true;
  in.scan_stamp_s = 99.99;
  in.wheel_speed_mps = 0.3;
  in.odom_stamp_s = in.now_s - (p.odom_hard_timeout_s - 0.005);
  EXPECT_NE(evaluate_gate(in, p).state, PlannerState::NoOdom);
  in.odom_stamp_s = in.now_s - (p.odom_hard_timeout_s + 0.005);
  EXPECT_EQ(evaluate_gate(in, p).state, PlannerState::NoOdom);
}

// ---------------------------------------------------------------------------------------------
// 3. Actuator estimate from gated commands

namespace {

DynamicsParams with_delay(double delay_s) {
  DynamicsParams d = rig::dynamics_with_floor(0.23);
  d.nominal.command_delay_s = delay_s;
  return d;
}

// One tick with the gated message the safety node would have produced for the previous tick.
struct GateState {
  std::uint64_t seq = 0;
  bool have = false;
  Command last_pub;
  double last_pub_s = 0.0;
};

}  // namespace

TEST(GatedEstimate, EqualStreamsAreBitIdenticalToTheOldEstimate) {
  for (double delay : {0.0, 0.08}) {
    const VehicleGeometry g = test::binding_geometry();
    rig::Rig ra(rig::small_config(0.5), 0.23);
    rig::Rig rb(rig::small_config(0.5), 0.23);
    Planner pa(rig::small_config(0.5), with_delay(delay), g);
    Planner pb(rig::small_config(0.5), with_delay(delay), g);
    GateState gate;
    double t = 1000.0;
    int driving = 0;
    for (int i = 0; i < 300; ++i, t += 0.02) {
      TickInput ia = ra.input(t);
      TickInput ib = rb.input(t);
      if (gate.have) {  // /drive equals /drive_raw (as the float32 the topic carries)
        ib.gated_seq = gate.seq;
        ib.gated_receipt_s = gate.last_pub_s + 0.004;
        ib.gated_speed_mps = static_cast<float>(gate.last_pub.speed_mps);
        ib.gated_steering_rad = static_cast<float>(gate.last_pub.steering_rad);
      }
      const TickOutput oa = pa.tick(ia);
      const TickOutput ob = pb.tick(ib);
      EXPECT_TRUE(same_bits(oa.cmd.speed_mps, ob.cmd.speed_mps)) << i;
      EXPECT_TRUE(same_bits(oa.cmd.steering_rad, ob.cmd.steering_rad)) << i;
      EXPECT_TRUE(same_bits(oa.est_steering_rad, ob.est_steering_rad)) << i;
      EXPECT_TRUE(same_bits(oa.est_speed_mps, ob.est_speed_mps)) << i;
      const State ea = pa.actuator_estimate();
      const State eb = pb.actuator_estimate();
      EXPECT_TRUE(same_bits(ea.v, eb.v)) << i;
      EXPECT_TRUE(same_bits(ea.delta, eb.delta)) << i;
      gate.have = true;
      gate.seq = static_cast<std::uint64_t>(i + 1);
      gate.last_pub = oa.cmd;
      gate.last_pub_s = t;
      driving += oa.state == PlannerState::Driving ? 1 : 0;
    }
    EXPECT_GT(driving, 100) << "the run must actually drive";
    EXPECT_EQ(pb.gated_corrected_ticks(), 0u);
    EXPECT_GT(pb.gated_equal_ticks(), 200u);
    EXPECT_EQ(pa.gated_equal_ticks(), 0u);
  }
}

namespace {
// Drives a planner to DRIVING with a pass-through gate, then lets the gate zero the speed (or hold
// the steering) and returns the model's estimates tick by tick.
struct GateOverride {
  bool zero_speed = false;
  bool hold_steering = false;
  double held_steering = 0.0;
};

std::vector<State> run_with_gate(double delay_s, bool tell, const GateOverride& ov, int n_after) {
  const VehicleGeometry g = test::binding_geometry();
  rig::Rig r(rig::small_config(0.5), 0.23);
  Planner p(rig::small_config(0.5), with_delay(delay_s), g);
  GateState gate;
  double t = 2000.0;
  std::vector<State> est;
  bool overriding = false;
  int after = 0;
  for (int i = 0; i < 400 && after < n_after; ++i, t += 0.02) {
    TickInput in = r.input(t);
    if (gate.have && tell) {
      in.gated_seq = gate.seq;
      in.gated_receipt_s = gate.last_pub_s + 0.004;
      in.gated_speed_mps = static_cast<float>(gate.last_pub.speed_mps);
      in.gated_steering_rad = static_cast<float>(gate.last_pub.steering_rad);
      if (overriding && ov.zero_speed) in.gated_speed_mps = 0.0;
      if (overriding && ov.hold_steering) in.gated_steering_rad = ov.held_steering;
    }
    const TickOutput o = p.tick(in);
    gate.have = true;
    gate.seq = static_cast<std::uint64_t>(i + 1);
    gate.last_pub = o.cmd;
    gate.last_pub_s = t;
    if (overriding) {
      est.push_back(p.actuator_estimate());
      ++after;
    }
    if (!overriding && o.state == PlannerState::Driving && o.cmd.speed_mps > 0.2 && i > 120) {
      overriding = true;
    }
  }
  return est;
}
}  // namespace

TEST(GatedEstimate, AGatedSpeedOfZeroBringsTheEstimateDownWithinTwoTicksWithoutADelay) {
  GateOverride ov;
  ov.zero_speed = true;
  const auto told = run_with_gate(0.0, true, ov, 6);
  const auto old = run_with_gate(0.0, false, ov, 6);
  ASSERT_EQ(told.size(), old.size());
  ASSERT_GE(told.size(), 4u);
  // The wheel speed is pinned by the rig, so the model's estimate after a tick moves with what
  // acted: the published (positive) command raises it above the measured 0.3, a gated 0 lowers it.
  EXPECT_GT(old[1].v, rig::Rig().v);
  EXPECT_LT(told[1].v, rig::Rig().v);
}

TEST(GatedEstimate, AGatedSpeedOfZeroReachesTheEstimateWhenItsRingEntryActs) {
  GateOverride ov;
  ov.zero_speed = true;
  const int delay_ticks = 4;  // 0.08 s at 50 Hz
  const auto told = run_with_gate(0.08, true, ov, delay_ticks + 4);
  const auto old = run_with_gate(0.08, false, ov, delay_ticks + 4);
  ASSERT_GE(told.size(), static_cast<std::size_t>(delay_ticks + 3));
  EXPECT_GT(old[delay_ticks + 1].v, rig::Rig().v);
  EXPECT_LT(told[delay_ticks + 1].v, rig::Rig().v);
}

TEST(GatedEstimate, AGatedSteeringHoldIsFollowed) {
  GateOverride ov;
  ov.hold_steering = true;
  ov.held_steering = 0.0;
  // Drive until the plan steers, then pin the gated steering at 0.2: the estimate heads there
  // although the planner keeps publishing something else.
  ov.held_steering = 0.2;
  const auto told = run_with_gate(0.0, true, ov, 8);
  const auto old = run_with_gate(0.0, false, ov, 8);
  ASSERT_GE(told.size(), 6u);
  EXPECT_GT(told[5].delta, old[5].delta + 1e-3);
  EXPECT_GT(told[5].delta, 0.0);
}

TEST(GatedEstimate, WithoutAGatedMessageThePublishedValuesStandAndAreCounted) {
  const VehicleGeometry g = test::binding_geometry();
  rig::Rig r(rig::small_config(0.5), 0.23);
  Planner p(rig::small_config(0.5), with_delay(0.08), g);
  double t = 3000.0;
  for (int i = 0; i < 50; ++i, t += 0.02) p.tick(r.input(t));
  EXPECT_EQ(p.gated_corrected_ticks(), 0u);
  EXPECT_GT(p.gated_fallback_ticks(), 40u);
}

TEST(GatedEstimate, ALateMessageOfAnOlderTickDoesNotCorrectTheNewOne) {
  const VehicleGeometry g = test::binding_geometry();
  rig::Rig r(rig::small_config(0.5), 0.23);
  Planner p(rig::small_config(0.5), with_delay(0.08), g);
  double t = 4000.0;
  Command prev2, prev1;  // published two and one ticks ago
  std::uint64_t seq = 0;
  for (int i = 0; i < 200; ++i, t += 0.02) {
    TickInput in = r.input(t);
    if (i > 2) {  // the gate's answer lags by one tick: it equals the command of two ticks ago
      in.gated_seq = ++seq;
      in.gated_receipt_s = t - 0.01;
      in.gated_speed_mps = static_cast<float>(prev2.speed_mps);
      in.gated_steering_rad = static_cast<float>(prev2.steering_rad);
    }
    const TickOutput o = p.tick(in);
    prev2 = prev1;
    prev1 = o.cmd;
  }
  EXPECT_EQ(p.gated_corrected_ticks(), 0u);
}

// ---------------------------------------------------------------------------------------------
// 4. One motion model

namespace {
DynamicsParams residual_dynamics() {
  DynamicsParams d = test::plain_dynamics();
  d.residual.enabled = true;
  d.residual.weights.assign(kMlpWeightCount, 0.0);
  // Deterministic nonzero weights.
  for (int i = 0; i < kMlpWeightCount; ++i) {
    d.residual.weights[static_cast<std::size_t>(i)] = 0.15 * std::sin(0.7 * i + 0.3);
  }
  d.residual.output_scale = {0.0, 0.4};  // a yaw residual only: it shows in the dead reckoning
  return d;
}
}  // namespace

TEST(EgoMotionModel, WithoutAResidualItIsBitIdenticalToTheNominalConstructor) {
  const VehicleGeometry g = test::binding_geometry();
  DynamicsParams d = test::plain_dynamics();
  d.nominal.understeer_s2_per_m2 = 0.15;
  const DynamicsModel model(d, g, 0.02);
  EgoMotion a(g.wheelbase_m, d.nominal.understeer_s2_per_m2);
  EgoMotion b(&model);
  for (int i = 0; i <= 40; ++i) {
    const double v = 0.4 + 0.01 * i;
    const double delta = 0.2 * std::sin(0.2 * i);
    a.push(i * 0.02, v, delta);
    b.push(i * 0.02, v, delta, 0.7, 0.1);  // commands are ignored without a residual
  }
  for (double t0 : {0.0, 0.13, 0.31}) {
    for (double t1 : {0.2, 0.45, 0.78}) {
      const Pose2 pa = a.relative_pose(t0, t1);
      const Pose2 pb = b.relative_pose(t0, t1);
      EXPECT_TRUE(same_bits(pa.x, pb.x));
      EXPECT_TRUE(same_bits(pa.y, pb.y));
      EXPECT_TRUE(same_bits(pa.yaw, pb.yaw));
    }
  }
}

TEST(EgoMotionModel, WithAResidualTheDeadReckonedYawEqualsTheModelRollouts) {
  const VehicleGeometry g = test::binding_geometry();
  const DynamicsParams d = residual_dynamics();
  const DynamicsModel model(d, g, 0.02);
  const DynamicsModel step_model(d, g, 0.02);
  ASSERT_TRUE(model.has_residual());
  // Constant state and commands: the model's yaw rate is constant, so the dead reckoned yaw over
  // a span is that rate times the span, and it is the rollout's yaw increment of the same step
  // sequence when the state is held (v, delta fixed by hand each step).
  const double v = 0.7, delta = 0.18, sc = 0.9, dc = 0.15;
  EgoMotion ego(&model);
  for (int i = 0; i <= 30; ++i) ego.push(i * 0.02, v, delta, sc, dc);
  State s;
  s.v = v;
  s.delta = delta;
  double yaw = 0.0;
  const int steps = 20;
  for (int i = 0; i < steps; ++i) {
    double w = 0.0;
    step_model.step_trig(s, Control{sc, dc}, std::cos(s.yaw), std::sin(s.yaw), &w);
    yaw += w * 0.02;
  }
  const Pose2 p = ego.relative_pose(0.0, steps * 0.02);
  EXPECT_NEAR(p.yaw, yaw, 1e-12);
  // And it differs from the nominal only motion: the residual is live.
  EgoMotion nominal(g.wheelbase_m, d.nominal.understeer_s2_per_m2);
  for (int i = 0; i <= 30; ++i) nominal.push(i * 0.02, v, delta);
  EXPECT_GT(std::fabs(p.yaw - nominal.relative_pose(0.0, steps * 0.02).yaw), 1e-3);
}
