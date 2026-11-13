// Recovery by reversing: the plan on hand-made scenes, every transition of the state machine
// (start holds, stopping, reversing, end by odometry and by time, abort on the gate, failed plan,
// exhaustion, release, reset on forward progress) and the Planner around it.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "racer_planner/planner.hpp"
#include "racer_planner/recovery.hpp"
#include "test_support.hpp"

using namespace racer_planner;

namespace {

using Points = std::vector<PointXY>;

void wall(Points& pts, double x1, double y1, double x2, double y2) {
  const double len = std::hypot(x2 - x1, y2 - y1);
  const int n = std::max(1, static_cast<int>(std::ceil(len / 0.02)));
  for (int i = 0; i <= n; ++i) {
    pts.push_back({x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n});
  }
}

struct Scene {
  VehicleGeometry g = test::binding_geometry();
  DistanceField field;
  FreespaceField fs{g};
  Points pts;

  void build() {
    field.build(pts.data(), pts.size());
    fs.build(field, pts.data(), pts.size(), nullptr);
  }
};

// A 1.4 m wide corridor from 1.2 m behind the car to a wall `ahead_m` in front of the front bumper.
Scene nose_in(double ahead_m, double behind_m = 1.2) {
  Scene s;
  const double front = s.g.body_max_x_m();
  wall(s.pts, -behind_m, 0.7, front + ahead_m, 0.7);
  wall(s.pts, -behind_m, -0.7, front + ahead_m, -0.7);
  wall(s.pts, front + ahead_m, -0.7, front + ahead_m, 0.7);
  wall(s.pts, -behind_m, -0.7, -behind_m, 0.7);
  s.build();
  return s;
}

RecoveryParams params() { return RecoveryParams{}; }

RecoveryInput stuck(const Scene& s, double now, BlockReason reason, double held) {
  RecoveryInput in;
  in.now_s = now;
  in.reason = reason;
  in.reason_held_s = held;
  in.gate_fresh = true;
  in.gated_speed_mps = 0.0;
  in.field = &s.field;
  in.freespace = &s.fs;
  return in;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// The plan

TEST(RecoveryPlan, OpenSpaceBehindGivesAPlanWithAllArcsClear) {
  const Scene s = nose_in(0.15);
  const RecoveryPlan plan = plan_recovery(s.field, &s.fs, Pose2{}, s.g, params());
  EXPECT_TRUE(plan.ok);
  EXPECT_EQ(plan.clear_arcs, 7);
  EXPECT_NEAR(plan.length_m, 0.35, 1e-12);
  EXPECT_GE(plan.steering_rad, s.g.steering_min_rad);
  EXPECT_LE(plan.steering_rad, s.g.steering_max_rad);
  EXPECT_GE(plan.arc, 0);
  EXPECT_LT(plan.arc, 7);
}

TEST(RecoveryPlan, AWallJustBehindLeavesNoReverseArcAndNoPlan) {
  // The rear bumper is 0.14 m behind the rear axle; a wall 0.3 m behind the axle leaves less than
  // the 0.35 m arc.
  Scene s;
  wall(s.pts, -0.30, -0.7, -0.30, 0.7);
  wall(s.pts, -0.30, 0.7, 2.0, 0.7);
  wall(s.pts, -0.30, -0.7, 2.0, -0.7);
  s.build();
  const RecoveryPlan plan = plan_recovery(s.field, &s.fs, Pose2{}, s.g, params());
  EXPECT_FALSE(plan.ok);
  EXPECT_EQ(plan.clear_arcs, 0);
}

TEST(RecoveryPlan, ArcsThatSwingTheTailIntoASideWallAreLeftOut) {
  // A wall close on the left at the rear: a reverse arc that swings the tail left hits it.
  Scene s = nose_in(0.15);
  wall(s.pts, -1.0, 0.2, 0.0, 0.2);  // a pillar wall beside the rear, 0.2 m left of the centre line
  s.build();
  const RecoveryPlan plan = plan_recovery(s.field, &s.fs, Pose2{}, s.g, params());
  EXPECT_LT(plan.clear_arcs, 7);
  // whatever arc was chosen is clear: its end pose keeps the hard clearance all along (re-check)
  if (plan.ok) {
    const Footprint fp(s.g);
    double x = 0.0, y = 0.0, yaw = 0.0;
    const double tan_d = std::tan(plan.steering_rad);
    const int n = 14;
    for (int i = 0; i < n; ++i) {
      const double ds = -plan.length_m / n;
      x += ds * std::cos(yaw);
      y += ds * std::sin(yaw);
      yaw += ds * tan_d / s.g.wheelbase_m;
      EXPECT_GE(fp.clearance_reverse(s.field, x, y, std::cos(yaw), std::sin(yaw)), 0.03 - 1e-9);
    }
  }
}

TEST(RecoveryPlan, WithAFieldTheBestArcLeavesTheBestForwardProgress) {
  // A room open to the left of a car that faces the wall: the goal is far to the left, so the
  // reverse arc that sets up a left turn wins over the others.
  Scene s;
  const double front = s.g.body_max_x_m();
  wall(s.pts, -1.0, 1.6, front + 0.12, 1.6);
  wall(s.pts, -1.0, -0.45, front + 0.12, -0.45);
  wall(s.pts, front + 0.12, -0.45, front + 0.12, 0.6);  // the wall ahead, the gap is to the left
  wall(s.pts, front + 0.12, 0.6, front + 0.12, 0.6);
  wall(s.pts, -1.0, -0.45, -1.0, 1.6);
  s.build();
  ASSERT_TRUE(s.fs.valid());
  const RecoveryPlan plan = plan_recovery(s.field, &s.fs, Pose2{}, s.g, params());
  ASSERT_TRUE(plan.ok);
  EXPECT_GT(plan.score, 0.0);  // some forward arc makes progress from the end pose
  EXPECT_GE(plan.clear_arcs, 1);
}

TEST(RecoveryPlan, WithoutAFieldTheScoreIsTheClearance) {
  const Scene s = nose_in(0.15);
  const RecoveryPlan plan = plan_recovery(s.field, nullptr, Pose2{}, s.g, params());
  EXPECT_TRUE(plan.ok);
  EXPECT_GE(plan.score, 0.03);  // a clearance that kept the hard limit
}

// ---------------------------------------------------------------------------------------------
// The state machine

TEST(Recovery, StartsOnlyAfterTheHoldOfItsReason) {
  const Scene s = nose_in(0.12);
  struct Row {
    BlockReason reason;
    double hold;
  } rows[] = {{BlockReason::AllCollide, 0.6},
              {BlockReason::BelowSpeedFloor, 0.6},
              {BlockReason::WheelStalled, 1.5},
              {BlockReason::GateRefusing, 1.0}};
  for (const Row& r : rows) {
    Recovery rec(params(), s.g, 0.23);
    const RecoveryOutput early = rec.update(stuck(s, 10.0, r.reason, r.hold - 0.01));
    EXPECT_FALSE(early.active) << static_cast<int>(r.reason);
    EXPECT_EQ(rec.phase(), RecoveryPhase::Idle);
    EXPECT_EQ(rec.attempts(), 0);
    const RecoveryOutput start = rec.update(stuck(s, 10.02, r.reason, r.hold));
    EXPECT_TRUE(start.active) << static_cast<int>(r.reason);
    EXPECT_EQ(rec.phase(), RecoveryPhase::Stopping);
    EXPECT_EQ(rec.attempts(), 1);
    EXPECT_EQ(start.cmd.speed_mps, 0.0);  // the forward speed comes to 0 first
    EXPECT_DOUBLE_EQ(start.cmd.steering_rad, rec.plan().steering_rad);
  }
  Recovery rec(params(), s.g, 0.23);
  EXPECT_FALSE(rec.update(stuck(s, 1.0, BlockReason::None, 5.0)).active);
  EXPECT_FALSE(rec.update(stuck(s, 1.0, BlockReason::RecoveryExhausted, 5.0)).active);
}

TEST(Recovery, WaitsForTheWheelToStopThenReversesAtTheRecoverSpeed) {
  const Scene s = nose_in(0.12);
  Recovery rec(params(), s.g, 0.23);
  RecoveryInput in = stuck(s, 10.0, BlockReason::AllCollide, 0.7);
  rec.update(in);
  in.wheel_speed_mps = 0.3;  // still rolling forward
  in.now_s = 10.02;
  RecoveryOutput o = rec.update(in);
  EXPECT_TRUE(o.active);
  EXPECT_EQ(o.cmd.speed_mps, 0.0);
  EXPECT_EQ(rec.phase(), RecoveryPhase::Stopping);
  in.wheel_speed_mps = 0.049;
  in.now_s = 10.04;
  o = rec.update(in);
  EXPECT_EQ(rec.phase(), RecoveryPhase::Reversing);
  EXPECT_DOUBLE_EQ(o.cmd.speed_mps, -0.30);
  EXPECT_DOUBLE_EQ(o.cmd.steering_rad, rec.plan().steering_rad);
  // a floor above the recover speed sets the reverse speed
  Recovery fast(params(), s.g, 0.35);
  EXPECT_DOUBLE_EQ(fast.reverse_speed_mps(), 0.35);
}

TEST(Recovery, EndsWhenTheWheelOdometryIntegralReachesTheArcLength) {
  const Scene s = nose_in(0.12);
  Recovery rec(params(), s.g, 0.23);
  RecoveryInput in = stuck(s, 10.0, BlockReason::AllCollide, 0.7);
  rec.update(in);
  in.now_s += 0.02;
  rec.update(in);  // wheel 0: now reversing
  ASSERT_EQ(rec.phase(), RecoveryPhase::Reversing);
  in.wheel_speed_mps = -0.30;
  in.published_speed_mps = -0.30;
  in.gated_speed_mps = -0.30;
  int ticks = 0;
  RecoveryOutput o;
  do {
    in.now_s += 0.02;
    o = rec.update(in);
    ++ticks;
  } while (o.active && ticks < 200);
  // 0.35 m at 0.30 m/s and 0.02 s per tick is 58.3 ticks
  EXPECT_FALSE(o.active);
  EXPECT_EQ(ticks, 59);
  EXPECT_EQ(rec.phase(), RecoveryPhase::Idle);
  EXPECT_EQ(rec.attempts(), 1);
}

TEST(Recovery, EndsAfterTheMaximumTimeWhenTheCarDoesNotMove) {
  const Scene s = nose_in(0.12);
  Recovery rec(params(), s.g, 0.23);
  RecoveryInput in = stuck(s, 10.0, BlockReason::AllCollide, 0.7);
  rec.update(in);
  RecoveryOutput o;
  double t = 10.0;
  bool was_active = true;
  for (int i = 0; i < 200 && was_active; ++i) {
    t += 0.02;
    in.now_s = t;
    in.published_speed_mps = -0.3;
    in.gated_speed_mps = -0.3;  // the gate passes it, the wheels just do not turn
    o = rec.update(in);
    was_active = o.active;
  }
  EXPECT_FALSE(was_active);
  EXPECT_NEAR(t - 10.0, 2.5, 0.03);
  EXPECT_EQ(rec.phase(), RecoveryPhase::Idle);
}

TEST(Recovery, AbortsWhenTheGateDoesNotPassTheReverseForTheDebounceTime) {
  const Scene s = nose_in(0.12);
  Recovery rec(params(), s.g, 0.23);
  RecoveryInput in = stuck(s, 10.0, BlockReason::AllCollide, 0.7);
  rec.update(in);
  in.now_s = 10.02;
  rec.update(in);
  ASSERT_EQ(rec.phase(), RecoveryPhase::Reversing);
  in.published_speed_mps = -0.3;
  in.gated_speed_mps = 0.0;  // the rear gate refused
  in.now_s = 10.04;
  EXPECT_TRUE(rec.update(in).active);  // the debounce has just started
  in.now_s = 10.13;
  EXPECT_TRUE(rec.update(in).active);  // 0.09 s
  in.now_s = 10.15;
  EXPECT_FALSE(rec.update(in).active);  // 0.11 s: abandoned
  EXPECT_EQ(rec.phase(), RecoveryPhase::Idle);
  // a gate that passes the speed again before the debounce ends does not abort
  Recovery ok(params(), s.g, 0.23);
  RecoveryInput b = stuck(s, 20.0, BlockReason::AllCollide, 0.7);
  ok.update(b);
  b.now_s = 20.02;
  ok.update(b);
  b.published_speed_mps = -0.3;
  b.gated_speed_mps = 0.0;
  b.now_s = 20.04;
  ok.update(b);
  b.gated_speed_mps = -0.3;
  b.now_s = 20.08;
  EXPECT_TRUE(ok.update(b).active);
  b.gated_speed_mps = 0.0;
  b.now_s = 20.10;  // a new bad spell starts here, not at 20.04
  EXPECT_TRUE(ok.update(b).active);
  b.now_s = 20.18;
  EXPECT_TRUE(ok.update(b).active);
}

TEST(Recovery, AbortsWhenTheGatedDriveIsStaleInAnyPhase) {
  const Scene s = nose_in(0.12);
  {
    Recovery rec(params(), s.g, 0.23);
    RecoveryInput in = stuck(s, 10.0, BlockReason::AllCollide, 0.7);
    rec.update(in);
    ASSERT_EQ(rec.phase(), RecoveryPhase::Stopping);
    in.gate_fresh = false;
    in.now_s = 10.02;
    EXPECT_FALSE(rec.update(in).active);
    EXPECT_EQ(rec.phase(), RecoveryPhase::Idle);
  }
  {
    Recovery rec(params(), s.g, 0.23);
    RecoveryInput in = stuck(s, 10.0, BlockReason::AllCollide, 0.7);
    rec.update(in);
    in.now_s = 10.02;
    rec.update(in);
    ASSERT_EQ(rec.phase(), RecoveryPhase::Reversing);
    in.gate_fresh = false;
    in.now_s = 10.04;
    EXPECT_FALSE(rec.update(in).active);
  }
}

TEST(Recovery, AfterAnAbortItWaitsTheRetryTimeAndThenStartsAgain) {
  const Scene s = nose_in(0.12);
  Recovery rec(params(), s.g, 0.23);
  RecoveryInput in = stuck(s, 10.0, BlockReason::AllCollide, 0.7);
  rec.update(in);
  in.gate_fresh = false;
  in.now_s = 10.02;
  rec.update(in);  // abort at 10.02
  in.gate_fresh = true;
  in.now_s = 12.9;
  EXPECT_FALSE(rec.update(in).active);  // inside the 3 s
  in.now_s = 13.03;
  EXPECT_TRUE(rec.update(in).active);
  EXPECT_EQ(rec.attempts(), 2);
}

TEST(Recovery, AFailedPlanStaysBlockedAndPlansAgainAfterTheRetryTime) {
  Scene s;
  wall(s.pts, -0.30, -0.7, -0.30, 0.7);  // no room behind
  wall(s.pts, -0.30, 0.7, 0.9, 0.7);
  wall(s.pts, -0.30, -0.7, 0.9, -0.7);
  wall(s.pts, 0.9, -0.7, 0.9, 0.7);
  s.build();
  Recovery rec(params(), s.g, 0.23);
  RecoveryInput in = stuck(s, 10.0, BlockReason::AllCollide, 0.7);
  EXPECT_FALSE(rec.update(in).active);
  EXPECT_EQ(rec.attempts(), 0);  // a failed plan is not an attempt
  EXPECT_EQ(rec.phase(), RecoveryPhase::Idle);
  in.now_s = 12.0;
  EXPECT_FALSE(rec.update(in).active);  // not planned again before 3 s
  // the scene opens up behind the car (the car was pushed): the next plan after 3 s works
  const Scene open = nose_in(0.12);
  RecoveryInput later = stuck(open, 13.1, BlockReason::AllCollide, 3.8);
  EXPECT_TRUE(rec.update(later).active);
  EXPECT_EQ(rec.attempts(), 1);
}

TEST(Recovery, ExhaustionAfterTheMaximumAttemptsAndReleaseWhenTheCarCanDriveAgain) {
  const Scene s = nose_in(0.12);
  Recovery rec(params(), s.g, 0.23);
  double t = 10.0;
  for (int attempt = 1; attempt <= 3; ++attempt) {
    RecoveryInput in = stuck(s, t, BlockReason::AllCollide, 0.7);
    EXPECT_TRUE(rec.update(in).active) << attempt;
    EXPECT_EQ(rec.attempts(), attempt);
    in.gate_fresh = false;  // abandon at once
    t += 0.02;
    in.now_s = t;
    rec.update(in);
    t += 3.1;  // past the retry wait
  }
  RecoveryInput in = stuck(s, t, BlockReason::AllCollide, 5.0);
  RecoveryOutput o = rec.update(in);
  EXPECT_FALSE(o.active);
  EXPECT_TRUE(o.exhausted);
  EXPECT_EQ(rec.phase(), RecoveryPhase::Exhausted);
  EXPECT_EQ(o.attempts, 3);
  // still stuck: stays exhausted however long
  in.now_s = t + 30.0;
  EXPECT_TRUE(rec.update(in).exhausted);
  // drivable again: released only after 1.0 s
  RecoveryInput free_in = stuck(s, t + 31.0, BlockReason::None, 0.0);
  EXPECT_TRUE(rec.update(free_in).exhausted);
  free_in.now_s = t + 31.9;
  EXPECT_TRUE(rec.update(free_in).exhausted);
  free_in.now_s = t + 32.05;
  o = rec.update(free_in);
  EXPECT_FALSE(o.exhausted);
  EXPECT_EQ(rec.attempts(), 0);
  EXPECT_EQ(rec.phase(), RecoveryPhase::Idle);
  // a blocked tick in between restarts the release timer
  RecoveryParams no_hold = params();
  no_hold.recover_exhausted_hold_s = 0.0;
  Recovery again(no_hold, s.g, 0.23);
  double u = 100.0;
  for (int attempt = 1; attempt <= 3; ++attempt) {
    RecoveryInput a = stuck(s, u, BlockReason::AllCollide, 0.7);
    again.update(a);
    a.gate_fresh = false;
    u += 0.02;
    a.now_s = u;
    again.update(a);
    u += 3.1;
  }
  again.update(stuck(s, u, BlockReason::AllCollide, 5.0));
  again.update(stuck(s, u + 1.0, BlockReason::None, 0.0));
  again.update(stuck(s, u + 1.5, BlockReason::AllCollide, 5.0));  // blocked again: timer resets
  EXPECT_TRUE(again.update(stuck(s, u + 2.2, BlockReason::None, 0.0)).exhausted);
  EXPECT_FALSE(again.update(stuck(s, u + 3.3, BlockReason::None, 0.0)).exhausted);
}

TEST(Recovery, ExhaustedStaysBlockedForTheHoldEvenWhenAForwardPlanExists) {
  const Scene s = nose_in(0.12);
  Recovery rec(params(), s.g, 0.23);
  double t = 10.0;
  for (int attempt = 1; attempt <= 3; ++attempt) {
    RecoveryInput in = stuck(s, t, BlockReason::AllCollide, 0.7);
    rec.update(in);
    in.gate_fresh = false;
    t += 0.02;
    in.now_s = t;
    rec.update(in);
    t += 3.1;
  }
  EXPECT_TRUE(rec.update(stuck(s, t, BlockReason::AllCollide, 5.0)).exhausted);
  // a forward plan exists (the car backed away from the wall) but the 30 s hold has not passed
  for (double dt : {1.0, 2.5, 10.0, 25.0}) {
    EXPECT_TRUE(rec.update(stuck(s, t + dt, BlockReason::None, 0.0)).exhausted) << dt;
  }
  EXPECT_TRUE(rec.update(stuck(s, t + 30.5, BlockReason::None, 0.0)).exhausted);  // release timer
  EXPECT_FALSE(rec.update(stuck(s, t + 31.6, BlockReason::None, 0.0)).exhausted);
}

TEST(Recovery, AttemptsResetWhenTheCarCoversHalfAMetreForward) {
  const Scene s = nose_in(0.12);
  Recovery rec(params(), s.g, 0.23);
  RecoveryInput in = stuck(s, 10.0, BlockReason::AllCollide, 0.7);
  rec.update(in);
  in.gate_fresh = false;
  in.now_s = 10.02;
  rec.update(in);
  ASSERT_EQ(rec.attempts(), 1);
  // 0.4 m forward is not enough
  in = stuck(s, 20.0, BlockReason::None, 0.0);
  in.wheel_speed_mps = 0.4;
  for (int i = 0; i < 50; ++i) {  // 0.4 m
    in.now_s += 0.02;
    rec.update(in);
  }
  EXPECT_EQ(rec.attempts(), 1);
  for (int i = 0; i < 20; ++i) {  // 0.56 m in all
    in.now_s += 0.02;
    rec.update(in);
  }
  EXPECT_EQ(rec.attempts(), 0);
}

TEST(Recovery, ResetForgetsTheAttemptAndThePhase) {
  const Scene s = nose_in(0.12);
  Recovery rec(params(), s.g, 0.23);
  rec.update(stuck(s, 10.0, BlockReason::AllCollide, 0.7));
  ASSERT_EQ(rec.phase(), RecoveryPhase::Stopping);
  rec.reset();
  EXPECT_EQ(rec.phase(), RecoveryPhase::Idle);
  EXPECT_EQ(rec.attempts(), 0);
}

TEST(Recovery, NoFieldPointerMeansNoPlanAndNoStart) {
  const Scene s = nose_in(0.12);
  Recovery rec(params(), s.g, 0.23);
  RecoveryInput in = stuck(s, 10.0, BlockReason::AllCollide, 0.7);
  in.field = nullptr;
  EXPECT_FALSE(rec.update(in).active);
  EXPECT_EQ(rec.attempts(), 0);
}

// ---------------------------------------------------------------------------------------------
// The Planner around it

namespace {

PlannerConfig planner_config() {
  PlannerConfig c;
  c.logic.v_max_mps = 0.5;
  c.mppi.samples = 128;
  c.max_samples = 128;
  c.mppi.seed = 3;
  c.recovery.recover_exhausted_hold_s = 2.0;  // the default 30 s would make these tests long
  return c;
}

struct StuckRig {
  VehicleGeometry g = test::binding_geometry();
  Scene scene = nose_in(0.10);
  Planner planner;
  double now = 100.0;
  double wheel = 0.0;
  double last_speed = 0.0;
  bool gate_passes = true;

  StuckRig()
      : planner(
            planner_config(),
            [] {
              DynamicsParams d = test::plain_dynamics();
              d.nominal.speed_deadband_mps = 0.2;
              return d;
            }(),
            g) {}

  TickOutput tick() {
    TickInput in;
    in.now_s = now;
    in.has_odom = true;
    in.odom_stamp_s = now;
    in.wheel_speed_mps = wheel;
    in.has_scan = true;
    in.scan_stamp_s = now - 0.05;
    in.field = &scene.field;
    in.freespace = &scene.fs;
    in.has_gated = true;
    in.gated_stamp_s = now;
    in.gated_speed_mps = gate_passes ? last_speed : 0.0;
    now += 0.02;
    const TickOutput out = planner.tick(in);
    last_speed = out.cmd.speed_mps;
    return out;
  }
};

}  // namespace

TEST(PlannerRecovery, NoseInToAWallBlocksThenReversesThenGoesBackToThePlan) {
  StuckRig r;
  std::vector<TickOutput> outs;
  for (int i = 0; i < 600; ++i) outs.push_back(r.tick());
  int first_blocked = -1, first_recovering = -1;
  for (std::size_t i = 0; i < outs.size(); ++i) {
    if (outs[i].state == PlannerState::Blocked && first_blocked < 0)
      first_blocked = static_cast<int>(i);
    if (outs[i].state == PlannerState::Recovering && first_recovering < 0) {
      first_recovering = static_cast<int>(i);
    }
  }
  ASSERT_GE(first_blocked, 0);
  ASSERT_GE(first_recovering, 0);
  // blocked for the 0.6 s hold (30 ticks) before the recovery begins
  EXPECT_NEAR(first_recovering - first_blocked, 30, 3);
  // the samples that stand still do not collide, the ones that move all do: reason 2
  EXPECT_EQ(outs[static_cast<std::size_t>(first_blocked)].block_reason,
            BlockReason::BelowSpeedFloor);
  // while recovering: the speed is zero or negative, never below the reverse limit, and the
  // magnitude grows by at most the planner's rate limit per tick
  const double step = 0.45 * r.g.max_acceleration_mps2 * 0.02;
  double min_speed = 0.0;
  double prev = 0.0;
  int reversing_ticks = 0;
  for (const TickOutput& o : outs) {
    if (o.state != PlannerState::Recovering) {
      EXPECT_GE(o.cmd.speed_mps, 0.0);
      prev = o.cmd.speed_mps;
      continue;
    }
    EXPECT_LE(o.cmd.speed_mps, 0.0);
    EXPECT_GE(o.cmd.speed_mps, -0.30 - 1e-12);
    EXPECT_GE(o.cmd.speed_mps, prev - step - 1e-12);
    prev = o.cmd.speed_mps;
    min_speed = std::min(min_speed, o.cmd.speed_mps);
    if (o.cmd.speed_mps < 0.0) ++reversing_ticks;
    EXPECT_EQ(o.block_reason, BlockReason::None);
    EXPECT_GE(o.recovery_attempts, 1);
  }
  EXPECT_NEAR(min_speed, -0.30, 1e-9);
  EXPECT_GT(reversing_ticks, 30);
  // the wheel never moved: each recovery ends by the time limit, the next starts after the hold,
  // and the third attempt exhausts it
  EXPECT_EQ(outs.back().recovery_attempts, 3);
  EXPECT_EQ(outs.back().state, PlannerState::Blocked);
  EXPECT_EQ(outs.back().block_reason, BlockReason::RecoveryExhausted);
}

TEST(PlannerRecovery, TheGateRefusingTheReverseAbortsAndTheCarStaysPut) {
  StuckRig r;
  r.gate_passes = false;  // the safety node's rear gate does not pass a negative speed
  int recovering = 0;
  int ticks_after_start = 0;
  bool started = false;
  for (int i = 0; i < 300; ++i) {
    const TickOutput o = r.tick();
    if (o.state == PlannerState::Recovering) {
      started = true;
      ++recovering;
    }
    if (started) ++ticks_after_start;
  }
  EXPECT_TRUE(started);
  // abandoned within the debounce plus the ramp, then it waits the 3 s retry
  EXPECT_LT(recovering, 25);
}

TEST(PlannerRecovery, NegativeSpeedOnlyInRecoveringAndNeverBelowTheLimitForAnyInput) {
  StuckRig r;
  // random wheel speeds and gate behaviour on a stuck scene
  std::uint64_t seed = 12345;
  auto rnd = [&] {
    seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<double>(seed >> 40) / 16777216.0;
  };
  bool saw_recovering = false;
  for (int i = 0; i < 1500; ++i) {
    r.wheel = rnd() < 0.5 ? 0.0 : (rnd() - 0.5) * 0.8;
    r.gate_passes = rnd() < 0.7;
    const TickOutput o = r.tick();
    if (o.state == PlannerState::Recovering) {
      saw_recovering = true;
      EXPECT_GE(o.cmd.speed_mps, -r.planner.reverse_speed_mps() - 1e-12);
    } else {
      EXPECT_GE(o.cmd.speed_mps, 0.0) << "state " << static_cast<int>(o.state);
    }
    EXPECT_LE(o.cmd.speed_mps, o.v_cap_mps + 1e-12);
  }
  EXPECT_TRUE(saw_recovering);
}

TEST(PlannerRecovery, ExhaustionIsBlockedWithReasonFiveAndStaysSoUntilReleased) {
  StuckRig r;
  r.gate_passes = false;  // every attempt is abandoned
  TickOutput o;
  bool exhausted = false;
  for (int i = 0; i < 2000 && !exhausted; ++i) {  // 40 s: three attempts with 3 s waits
    o = r.tick();
    exhausted =
        o.state == PlannerState::Blocked && o.block_reason == BlockReason::RecoveryExhausted;
  }
  ASSERT_TRUE(exhausted);
  EXPECT_EQ(o.recovery_attempts, 3);
  EXPECT_EQ(o.cmd.speed_mps, 0.0);
  for (int i = 0; i < 500; ++i) {
    o = r.tick();
    EXPECT_EQ(o.state, PlannerState::Blocked);
    EXPECT_EQ(o.block_reason, BlockReason::RecoveryExhausted);
  }
  // the scene opens up (somebody moved the car): a forward plan for 1 s releases it
  Scene corridor = nose_in(3.0);
  r.scene = corridor;
  bool released = false;
  for (int i = 0; i < 200 && !released; ++i) {
    o = r.tick();
    released = o.state == PlannerState::Driving;
  }
  EXPECT_TRUE(released);
}

TEST(PlannerRecovery, LeavingTheSolvingStatesDropsTheRecovery) {
  StuckRig r;
  for (int i = 0; i < 100; ++i) r.tick();
  TickInput in;
  in.now_s = r.now;
  in.has_odom = true;
  in.odom_stamp_s = r.now;
  in.has_scan = false;  // scan lost mid-recovery
  const TickOutput o = r.planner.tick(in);
  EXPECT_EQ(o.state, PlannerState::NoScan);
  EXPECT_EQ(o.cmd.speed_mps, 0.0);
  EXPECT_EQ(o.recovery_attempts, 0);
}
