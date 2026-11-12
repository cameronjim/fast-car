// Table driven tests of every row of the decision table at pass, boundary and fail values,
// including NaN inputs.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "racer_planner/planner_logic.hpp"

using namespace racer_planner;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

// A fully healthy input at t = 100 s.
LogicInputs healthy() {
  LogicInputs in;
  in.now_s = 100.0;
  in.has_odom = true;
  in.odom_stamp_s = 99.99;
  in.has_scan = true;
  in.scan_stamp_s = 99.8;
  in.track.present = true;
  in.track.stamp_s = 99.85;
  in.track.validity = TrackValidity::Ok;
  in.track.valid_count = 16;
  in.track.boundary_residual_m = 0.05;
  in.track.intrusion_fraction = 0.0;
  return in;
}

PlannerState gate_state(const LogicInputs& in, const LogicParams& p = LogicParams{}) {
  return evaluate_gate(in, p).state;
}

}  // namespace

TEST(Gate, HealthyInputsDrive) {
  const Gate g = evaluate_gate(healthy(), LogicParams{});
  EXPECT_EQ(g.state, PlannerState::Driving);
  EXPECT_TRUE(g.may_solve);
}

TEST(Gate, InternalErrorIsFaultAndWinsOverEverything) {
  LogicInputs in = healthy();
  in.internal_error = true;
  in.has_odom = false;  // would be NO_ODOM
  const Gate g = evaluate_gate(in, LogicParams{});
  EXPECT_EQ(g.state, PlannerState::Fault);
  EXPECT_FALSE(g.may_solve);
}

struct StaleCase {
  const char* name;
  double age;
  bool stale;
};

TEST(Gate, OdomRowAtPassBoundaryAndFail) {
  const LogicParams p;  // 0.10 s
  const StaleCase cases[] = {{"fresh", 0.02, false},
                             {"at the limit", 0.10, false},
                             {"just over", 0.101, true},
                             {"old", 5.0, true}};
  for (const auto& c : cases) {
    LogicInputs in = healthy();
    in.odom_stamp_s = in.now_s - c.age;
    EXPECT_EQ(gate_state(in, p), c.stale ? PlannerState::NoOdom : PlannerState::Driving) << c.name;
  }
  LogicInputs none = healthy();
  none.has_odom = false;
  EXPECT_EQ(gate_state(none), PlannerState::NoOdom);
}

TEST(Gate, ScanRowAtPassBoundaryAndFail) {
  const LogicParams p;  // 0.35 s
  const StaleCase cases[] = {
      {"fresh", 0.1, false}, {"at the limit", 0.35, false}, {"just over", 0.351, true}};
  for (const auto& c : cases) {
    LogicInputs in = healthy();
    in.scan_stamp_s = in.now_s - c.age;
    EXPECT_EQ(gate_state(in, p), c.stale ? PlannerState::NoScan : PlannerState::Driving) << c.name;
  }
  LogicInputs none = healthy();
  none.has_scan = false;
  EXPECT_EQ(gate_state(none), PlannerState::NoScan);
}

TEST(Gate, TheTrackNeverGates) {
  // Missing, stale, invalid, too short and failing its self-check: the planner still solves, the
  // track only lowers the confidence.
  {
    LogicInputs in = healthy();
    in.track.present = false;
    EXPECT_EQ(gate_state(in), PlannerState::Driving);
  }
  {
    LogicInputs in = healthy();
    in.track.stamp_s = in.now_s - 30.0;
    EXPECT_EQ(gate_state(in), PlannerState::Driving);
  }
  for (TrackValidity v :
       {TrackValidity::NoTrack, TrackValidity::WrongLength, TrackValidity::WrongSpacing,
        TrackValidity::WrongFrame, TrackValidity::NonFinite}) {
    LogicInputs in = healthy();
    in.track.validity = v;
    EXPECT_EQ(gate_state(in), PlannerState::Driving) << static_cast<int>(v);
  }
  {
    LogicInputs in = healthy();
    in.track.valid_count = 0;
    in.track.boundary_residual_m = 9.0;
    in.track.intrusion_fraction = 1.0;
    EXPECT_TRUE(evaluate_gate(in, LogicParams{}).may_solve);
  }
}

TEST(Gate, TimestampJumpsAreStale) {
  LogicInputs in = healthy();
  in.scan_stamp_s = in.now_s + 10.0;
  EXPECT_EQ(gate_state(in), PlannerState::NoScan);
  in = healthy();
  in.odom_stamp_s = in.now_s + 10.0;
  EXPECT_EQ(gate_state(in), PlannerState::NoOdom);
}

TEST(Gate, NaNTimesAreStale) {
  LogicInputs in = healthy();
  in.odom_stamp_s = kNaN;
  EXPECT_EQ(gate_state(in), PlannerState::NoOdom);
  in = healthy();
  in.scan_stamp_s = kNaN;
  EXPECT_EQ(gate_state(in), PlannerState::NoScan);
  in = healthy();
  in.now_s = kNaN;
  EXPECT_EQ(gate_state(in), PlannerState::NoOdom);
}

TEST(Gate, RowsAreCheckedInTheTableOrder) {
  LogicInputs in = healthy();
  in.has_odom = false;
  in.has_scan = false;
  EXPECT_EQ(gate_state(in), PlannerState::NoOdom);
  in.has_odom = true;
  EXPECT_EQ(gate_state(in), PlannerState::NoScan);
  in.has_scan = true;
  EXPECT_EQ(gate_state(in), PlannerState::Driving);
}

// ---------------------------------------------------------------------------------------------
// Track confidence

TEST(Ramp, DownAtBothEndsAndBetween) {
  EXPECT_DOUBLE_EQ(ramp_down(0.0, 0.03, 0.15), 1.0);
  EXPECT_DOUBLE_EQ(ramp_down(0.03, 0.03, 0.15), 1.0);  // the full end
  EXPECT_NEAR(ramp_down(0.09, 0.03, 0.15), 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(ramp_down(0.15, 0.03, 0.15), 0.0);  // the zero end
  EXPECT_DOUBLE_EQ(ramp_down(9.0, 0.03, 0.15), 0.0);
  EXPECT_DOUBLE_EQ(ramp_down(-1.0, 0.03, 0.15), 1.0);
  EXPECT_DOUBLE_EQ(ramp_down(kNaN, 0.03, 0.15), 0.0);
  EXPECT_DOUBLE_EQ(ramp_down(INFINITY, 0.03, 0.15), 0.0);
  // degenerate ends make a step at `full`
  EXPECT_DOUBLE_EQ(ramp_down(0.1, 0.1, 0.1), 1.0);
  EXPECT_DOUBLE_EQ(ramp_down(0.1001, 0.1, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(ramp_down(0.1001, 0.2, 0.1), 1.0);
}

TEST(Ramp, UpAtBothEndsAndBetween) {
  EXPECT_DOUBLE_EQ(ramp_up(0.0, 3.0, 8.0), 0.0);
  EXPECT_DOUBLE_EQ(ramp_up(3.0, 3.0, 8.0), 0.0);  // the zero end
  EXPECT_NEAR(ramp_up(5.5, 3.0, 8.0), 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(ramp_up(8.0, 3.0, 8.0), 1.0);  // the full end
  EXPECT_DOUBLE_EQ(ramp_up(16.0, 3.0, 8.0), 1.0);
  EXPECT_DOUBLE_EQ(ramp_up(kNaN, 3.0, 8.0), 0.0);
  EXPECT_DOUBLE_EQ(ramp_up(5.0, 5.0, 5.0), 1.0);  // degenerate: a step at zero, reached
  EXPECT_DOUBLE_EQ(ramp_up(4.999, 5.0, 5.0), 0.0);
}

namespace {
TrackSummary good_summary(double now) {
  TrackSummary t;
  t.present = true;
  t.stamp_s = now - 0.1;
  t.validity = TrackValidity::Ok;
  t.valid_count = 16;
  t.boundary_residual_m = 0.0;
  t.intrusion_fraction = 0.0;
  return t;
}
}  // namespace

TEST(TrackConfidenceRaw, EachInputAloneAtItsEndsAndInTheMiddle) {
  const LogicParams p;
  const double now = 50.0;
  EXPECT_DOUBLE_EQ(track_confidence_raw(good_summary(now), now, p), 1.0);
  {  // intrusion 0.03 .. 0.15
    TrackSummary t = good_summary(now);
    t.intrusion_fraction = 0.03;
    EXPECT_DOUBLE_EQ(track_confidence_raw(t, now, p), 1.0);
    t.intrusion_fraction = 0.09;
    EXPECT_NEAR(track_confidence_raw(t, now, p), 0.5, 1e-12);
    t.intrusion_fraction = 0.15;
    EXPECT_DOUBLE_EQ(track_confidence_raw(t, now, p), 0.0);
  }
  {  // residual 0.10 .. 0.30 m
    TrackSummary t = good_summary(now);
    t.boundary_residual_m = 0.10;
    EXPECT_DOUBLE_EQ(track_confidence_raw(t, now, p), 1.0);
    t.boundary_residual_m = 0.20;
    EXPECT_NEAR(track_confidence_raw(t, now, p), 0.5, 1e-12);
    t.boundary_residual_m = 0.30;
    EXPECT_DOUBLE_EQ(track_confidence_raw(t, now, p), 0.0);
  }
  {  // valid stations 3 .. 8
    TrackSummary t = good_summary(now);
    t.valid_count = 8;
    EXPECT_DOUBLE_EQ(track_confidence_raw(t, now, p), 1.0);
    t.valid_count = 5;
    EXPECT_NEAR(track_confidence_raw(t, now, p), 0.4, 1e-12);
    t.valid_count = 3;
    EXPECT_DOUBLE_EQ(track_confidence_raw(t, now, p), 0.0);
    t.valid_count = 0;
    EXPECT_DOUBLE_EQ(track_confidence_raw(t, now, p), 0.0);
  }
  {  // age 0.25 .. 0.60 s
    TrackSummary t = good_summary(now);
    t.stamp_s = now - 0.25;
    EXPECT_DOUBLE_EQ(track_confidence_raw(t, now, p), 1.0);
    t.stamp_s = now - 0.425;
    EXPECT_NEAR(track_confidence_raw(t, now, p), 0.5, 1e-12);
    t.stamp_s = now - 0.60;
    EXPECT_DOUBLE_EQ(track_confidence_raw(t, now, p), 0.0);
    t.stamp_s = now - 30.0;
    EXPECT_DOUBLE_EQ(track_confidence_raw(t, now, p), 0.0);
  }
}

TEST(TrackConfidenceRaw, IsTheMinimumOfTheFour) {
  const LogicParams p;
  const double now = 50.0;
  TrackSummary t = good_summary(now);
  t.intrusion_fraction = 0.09;   // 0.5
  t.boundary_residual_m = 0.14;  // 0.8
  t.valid_count = 7;             // 0.8
  t.stamp_s = now - 0.325;       // 0.785714...
  EXPECT_NEAR(track_confidence_raw(t, now, p), 0.5, 1e-12);
  t.intrusion_fraction = 0.0;  // now the age is the smallest
  EXPECT_NEAR(track_confidence_raw(t, now, p), 1.0 - (0.325 - 0.25) / 0.35, 1e-12);
  t.stamp_s = now - 0.1;  // fresh: the residual and the station count tie at 0.8
  EXPECT_NEAR(track_confidence_raw(t, now, p), 0.8, 1e-12);
  t.boundary_residual_m = 0.0;
  EXPECT_NEAR(track_confidence_raw(t, now, p), 0.8, 1e-12);  // the station count alone
  t.valid_count = 8;
  EXPECT_NEAR(track_confidence_raw(t, now, p), 1.0, 1e-12);
}

TEST(TrackConfidenceRaw, ZeroWhenMissingInvalidOrNotFinite) {
  const LogicParams p;
  const double now = 50.0;
  TrackSummary missing = good_summary(now);
  missing.present = false;
  EXPECT_DOUBLE_EQ(track_confidence_raw(missing, now, p), 0.0);
  for (TrackValidity v :
       {TrackValidity::NoTrack, TrackValidity::WrongLength, TrackValidity::WrongSpacing,
        TrackValidity::WrongFrame, TrackValidity::NonFinite}) {
    TrackSummary t = good_summary(now);
    t.validity = v;
    EXPECT_DOUBLE_EQ(track_confidence_raw(t, now, p), 0.0) << static_cast<int>(v);
  }
  TrackSummary a = good_summary(now);
  a.intrusion_fraction = kNaN;
  EXPECT_DOUBLE_EQ(track_confidence_raw(a, now, p), 0.0);
  TrackSummary b = good_summary(now);
  b.boundary_residual_m = INFINITY;
  EXPECT_DOUBLE_EQ(track_confidence_raw(b, now, p), 0.0);
  TrackSummary c = good_summary(now);
  c.stamp_s = kNaN;
  EXPECT_DOUBLE_EQ(track_confidence_raw(c, now, p), 0.0);
  EXPECT_DOUBLE_EQ(track_confidence_raw(good_summary(now), kNaN, p), 0.0);
  // a timestamp from the future by more than the age ramp's zero end is a jump
  TrackSummary future = good_summary(now);
  future.stamp_s = now + 0.7;
  EXPECT_DOUBLE_EQ(track_confidence_raw(future, now, p), 0.0);
  future.stamp_s = now + 0.3;  // slightly ahead is clock skew, treated as fresh
  EXPECT_DOUBLE_EQ(track_confidence_raw(future, now, p), 1.0);
}

TEST(TrackConfidence, FallsAtOnceAndRisesSlowly) {
  const double tick = 0.02;
  EXPECT_DOUBLE_EQ(track_confidence(0.0, 1.0, tick, 1.0), 0.0);  // falls at once
  EXPECT_DOUBLE_EQ(track_confidence(0.4, 1.0, tick, 1.0), 0.4);
  EXPECT_NEAR(track_confidence(1.0, 0.0, tick, 1.0), 0.02, 1e-12);  // rises 1/s
  EXPECT_NEAR(track_confidence(1.0, 0.5, tick, 1.0), 0.52, 1e-12);
  EXPECT_DOUBLE_EQ(track_confidence(1.0, 0.99, tick, 1.0), 1.0);  // never above c_raw
  EXPECT_DOUBLE_EQ(track_confidence(0.3, 0.29, tick, 1.0), 0.3);  // the target is reached
  // a full rise from 0 takes one second at 1 per second
  double c = 0.0;
  int ticks = 0;
  while (c < 1.0 && ticks < 1000) {
    c = track_confidence(1.0, c, tick, 1.0);
    ++ticks;
  }
  EXPECT_EQ(ticks, 50);
  // bad inputs: NaN previous counts as 0, NaN raw as 0, a faster rise parameter is honoured
  EXPECT_NEAR(track_confidence(1.0, kNaN, tick, 1.0), 0.02, 1e-12);
  EXPECT_DOUBLE_EQ(track_confidence(kNaN, 0.5, tick, 1.0), 0.0);
  EXPECT_NEAR(track_confidence(1.0, 0.0, tick, 5.0), 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(track_confidence(1.0, 0.5, tick, 0.0), 0.5);  // no rise at all
  EXPECT_DOUBLE_EQ(track_confidence(7.0, 7.0, tick, 1.0), 1.0);  // clamped into [0, 1]
  EXPECT_DOUBLE_EQ(track_confidence(-3.0, 0.5, tick, 1.0), 0.0);
}

// ---------------------------------------------------------------------------------------------
// Nothing drivable

namespace {
DrivabilityInputs moving() {
  DrivabilityInputs in;
  in.plan_speed_mps = 0.4;
  in.speed_floor_mps = 0.23;
  in.wheel_speed_mps = 0.3;
  in.published_speed_mps = 0.3;
  return in;
}
}  // namespace

TEST(Drivability, HealthyIsDrivable) {
  DrivabilityMonitor m;
  EXPECT_EQ(m.update(moving(), 1.0), BlockReason::None);
  for (auto r : {BlockReason::AllCollide, BlockReason::BelowSpeedFloor, BlockReason::WheelStalled,
                 BlockReason::GateRefusing}) {
    EXPECT_FALSE(m.active(r));
    EXPECT_DOUBLE_EQ(m.held_s(r), 0.0);
  }
}

TEST(Drivability, AllCollideIsImmediateAndFirst) {
  DrivabilityMonitor m;
  DrivabilityInputs in = moving();
  in.all_collide = true;
  in.plan_speed_mps = 0.0;  // would also be reason 2
  EXPECT_EQ(m.update(in, 1.0), BlockReason::AllCollide);
  EXPECT_EQ(m.update(in, 1.4), BlockReason::AllCollide);
  EXPECT_NEAR(m.held_s(BlockReason::AllCollide), 0.4, 1e-12);
  EXPECT_FALSE(m.active(BlockReason::BelowSpeedFloor));  // reason 1 takes it
  in.all_collide = false;
  in.plan_speed_mps = 0.4;
  EXPECT_EQ(m.update(in, 1.42), BlockReason::None);
  EXPECT_DOUBLE_EQ(m.held_s(BlockReason::AllCollide), 0.0);  // the timer restarts
}

TEST(Drivability, PlanSpeedBelowTheFloorIsImmediate) {
  DrivabilityMonitor m;
  DrivabilityInputs in = moving();
  in.plan_speed_mps = 0.229;
  EXPECT_EQ(m.update(in, 1.0), BlockReason::BelowSpeedFloor);
  in.plan_speed_mps = 0.23;  // at the floor is drivable
  EXPECT_EQ(m.update(in, 1.02), BlockReason::None);
  in.plan_speed_mps = kNaN;
  EXPECT_EQ(m.update(in, 1.04), BlockReason::BelowSpeedFloor);
  // with the floor off the threshold is a token speed, not zero
  in.speed_floor_mps = 0.0;
  in.plan_speed_mps = 0.0;
  EXPECT_EQ(m.update(in, 1.06), BlockReason::BelowSpeedFloor);
  in.plan_speed_mps = 0.1;
  EXPECT_EQ(m.update(in, 1.08), BlockReason::None);
}

TEST(Drivability, NoPlanIsReasonTwo) {
  DrivabilityMonitor m;
  DrivabilityInputs in = moving();
  in.no_plan = true;
  in.all_collide = true;  // meaningless without a plan
  EXPECT_EQ(m.update(in, 1.0), BlockReason::BelowSpeedFloor);
}

TEST(Drivability, WheelStalledNeedsTheHoldAndAPublishedSpeedAtTheFloor) {
  DrivabilityMonitor m;
  DrivabilityInputs in = moving();
  in.wheel_speed_mps = 0.0;
  in.published_speed_mps = 0.23;
  EXPECT_EQ(m.update(in, 10.0), BlockReason::None);
  EXPECT_EQ(m.update(in, 11.49), BlockReason::None);
  EXPECT_EQ(m.update(in, 11.5), BlockReason::WheelStalled);
  EXPECT_NEAR(m.held_s(BlockReason::WheelStalled), 1.5, 1e-12);
  // the published speed below the floor (the planner stopped itself) ends the condition
  in.published_speed_mps = 0.1;
  EXPECT_EQ(m.update(in, 11.52), BlockReason::None);
  in.published_speed_mps = 0.23;
  EXPECT_EQ(m.update(in, 11.54), BlockReason::None);  // and the timer restarted
  // moving at 0.03 m/s or more is not stalled
  in.wheel_speed_mps = 0.03;
  EXPECT_EQ(m.update(in, 20.0), BlockReason::None);
  EXPECT_FALSE(m.active(BlockReason::WheelStalled));
  in.wheel_speed_mps = 0.0299;
  m.update(in, 21.0);
  EXPECT_TRUE(m.active(BlockReason::WheelStalled));
}

TEST(Drivability, GateRefusingNeedsAFreshGatedSpeedZeroForTheHold) {
  DrivabilityMonitor m;
  DrivabilityInputs in = moving();
  in.gate_fresh = true;
  in.gated_speed_mps = 0.0;
  in.published_speed_mps = 0.3;
  EXPECT_EQ(m.update(in, 5.0), BlockReason::None);
  EXPECT_EQ(m.update(in, 5.99), BlockReason::None);
  EXPECT_EQ(m.update(in, 6.0), BlockReason::GateRefusing);
  // a stale /drive never counts
  in.gate_fresh = false;
  EXPECT_EQ(m.update(in, 6.02), BlockReason::None);
  EXPECT_EQ(m.update(in, 9.0), BlockReason::None);
  // the gate passing speed through, or nothing published, is not refusing
  in.gate_fresh = true;
  in.gated_speed_mps = 0.3;
  EXPECT_EQ(m.update(in, 10.0), BlockReason::None);
  in.gated_speed_mps = 0.0;
  in.published_speed_mps = 0.0;
  EXPECT_EQ(m.update(in, 11.0), BlockReason::None);
  EXPECT_FALSE(m.active(BlockReason::GateRefusing));
}

TEST(Drivability, ReasonsAreReportedInTheOrderOneToFour) {
  DrivabilityParams p;
  p.stall_hold_s = 0.0;
  p.gate_hold_s = 0.0;
  DrivabilityMonitor m(p);
  DrivabilityInputs in = moving();
  in.wheel_speed_mps = 0.0;
  in.gate_fresh = true;
  in.gated_speed_mps = 0.0;
  EXPECT_EQ(m.update(in, 1.0), BlockReason::WheelStalled);  // 3 before 4
  in.wheel_speed_mps = 0.3;
  EXPECT_EQ(m.update(in, 1.02), BlockReason::GateRefusing);
  in.plan_speed_mps = 0.0;
  EXPECT_EQ(m.update(in, 1.04), BlockReason::BelowSpeedFloor);  // 2 before 3 and 4
  in.all_collide = true;
  EXPECT_EQ(m.update(in, 1.06), BlockReason::AllCollide);
  m.reset();
  EXPECT_FALSE(m.active(BlockReason::AllCollide));
  EXPECT_EQ(m.held_s(BlockReason::AllCollide), 0.0);
  EXPECT_EQ(m.held_s(BlockReason::None), 0.0);
  EXPECT_FALSE(m.active(BlockReason::RecoveryExhausted));
}

TEST(Drivability, NaNTimeNeverActivates) {
  DrivabilityMonitor m;
  DrivabilityInputs in = moving();
  in.all_collide = true;
  EXPECT_EQ(m.update(in, kNaN), BlockReason::None);
}

TEST(SpeedCap, ConfiguredCapWithoutGuidance) {
  LogicParams p;
  p.v_max_mps = 0.5;
  EXPECT_DOUBLE_EQ(speed_cap(p, healthy()), 0.5);
}

TEST(SpeedCap, GuidanceOnlyWhenEnabledFreshAndLower) {
  LogicParams p;
  p.v_max_mps = 0.5;
  LogicInputs in = healthy();
  in.has_guidance = true;
  in.guidance_stamp_s = in.now_s - 0.5;
  in.guidance_v_cap_mps = 0.3;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.5);  // disabled
  p.use_lap_guidance = true;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.3);
  in.guidance_v_cap_mps = 0.9;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.5);  // never above v_max
  in.guidance_v_cap_mps = 0.3;
  in.guidance_stamp_s = in.now_s - 2.0;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.3);  // at the 2 s limit still fresh
  in.guidance_stamp_s = in.now_s - 2.01;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.5);  // stale
  in.guidance_stamp_s = in.now_s - 0.5;
  in.has_guidance = false;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.5);
}

TEST(SpeedCap, ConfidenceScalesTheCapBetweenTheFreeSpaceFractionAndOne) {
  LogicParams p;
  p.v_max_mps = 0.5;
  p.freespace_speed_fraction = 0.7;
  const LogicInputs in = healthy();
  EXPECT_NEAR(speed_cap(p, in, 1.0), 0.5, 1e-12);
  EXPECT_NEAR(speed_cap(p, in, 0.0), 0.35, 1e-12);
  EXPECT_NEAR(speed_cap(p, in, 0.5), 0.5 * 0.85, 1e-12);
  EXPECT_NEAR(speed_cap(p, in), 0.5, 1e-12);  // default confidence 1
  EXPECT_NEAR(speed_cap(p, in, kNaN), 0.35, 1e-12);
  EXPECT_NEAR(speed_cap(p, in, 3.0), 0.5, 1e-12);  // clamped
  EXPECT_NEAR(speed_cap(p, in, -3.0), 0.35, 1e-12);
  // the guidance cap is scaled the same way and the result never exceeds v_max
  p.use_lap_guidance = true;
  LogicInputs g = healthy();
  g.has_guidance = true;
  g.guidance_stamp_s = g.now_s;
  g.guidance_v_cap_mps = 0.3;
  EXPECT_NEAR(speed_cap(p, g, 0.0), 0.3 * 0.7, 1e-12);
  EXPECT_NEAR(speed_cap(p, g, 1.0), 0.3, 1e-12);
  p.freespace_speed_fraction = 1.0;  // no slowing at all
  EXPECT_NEAR(speed_cap(p, in, 0.0), 0.5, 1e-12);
  p.freespace_speed_fraction = 7.0;  // clamped to 1: never above v_max
  EXPECT_NEAR(speed_cap(p, in, 0.0), 0.5, 1e-12);
}

TEST(SpeedCap, NaNAndNegativeGuidance) {
  LogicParams p;
  p.v_max_mps = 0.5;
  p.use_lap_guidance = true;
  LogicInputs in = healthy();
  in.has_guidance = true;
  in.guidance_stamp_s = in.now_s;
  in.guidance_v_cap_mps = kNaN;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.5);
  in.guidance_v_cap_mps = -1.0;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.0);
  p.v_max_mps = kNaN;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.0);
}

TEST(CommandForState, DrivingSanitisesTheSolverCommand) {
  const Command c =
      command_for_state(PlannerState::Driving, Command{0.4, 0.2}, 0.1, 0.5, -0.45, 0.45);
  EXPECT_DOUBLE_EQ(c.speed_mps, 0.4);
  EXPECT_DOUBLE_EQ(c.steering_rad, 0.2);
  const Command hi =
      command_for_state(PlannerState::Driving, Command{9.0, 9.0}, 0.0, 0.5, -0.45, 0.45);
  EXPECT_DOUBLE_EQ(hi.speed_mps, 0.5);
  EXPECT_DOUBLE_EQ(hi.steering_rad, 0.45);
  const Command lo =
      command_for_state(PlannerState::Driving, Command{-1.0, -9.0}, 0.0, 0.5, -0.45, 0.45);
  EXPECT_DOUBLE_EQ(lo.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(lo.steering_rad, -0.45);
}

TEST(CommandForState, DrivingWithNaNGivesSafeValues) {
  const Command c =
      command_for_state(PlannerState::Driving, Command{kNaN, kNaN}, 0.1, 0.5, -0.45, 0.45);
  EXPECT_EQ(c.speed_mps, 0.0);
  EXPECT_EQ(c.steering_rad, 0.0);
  const Command inf =
      command_for_state(PlannerState::Driving, Command{INFINITY, -INFINITY}, 0.1, 0.5, -0.45, 0.45);
  EXPECT_EQ(inf.speed_mps, 0.0);  // non finite is never trusted, not even as a limit
  EXPECT_EQ(inf.steering_rad, 0.0);
}

TEST(CommandForState, BlockedStopsWithTheBestSteering) {
  const Command c =
      command_for_state(PlannerState::Blocked, Command{0.4, 0.3}, 0.1, 0.5, -0.45, 0.45);
  EXPECT_EQ(c.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(c.steering_rad, 0.3);
}

TEST(CommandForState, NoDataStatesStopWithTheLastSteering) {
  for (PlannerState s : {PlannerState::NoOdom, PlannerState::NoScan, PlannerState::NoTrack,
                         PlannerState::Starting}) {
    const Command c = command_for_state(s, Command{0.4, 0.3}, -0.2, 0.5, -0.45, 0.45);
    EXPECT_EQ(c.speed_mps, 0.0) << static_cast<int>(s);
    EXPECT_DOUBLE_EQ(c.steering_rad, -0.2) << static_cast<int>(s);
    const Command nan = command_for_state(s, Command{0.4, 0.3}, kNaN, 0.5, -0.45, 0.45);
    EXPECT_EQ(nan.steering_rad, 0.0);
    const Command big = command_for_state(s, Command{0.4, 0.3}, 3.0, 0.5, -0.45, 0.45);
    EXPECT_DOUBLE_EQ(big.steering_rad, 0.45);
  }
}

TEST(CommandForState, FaultIsZeroZero) {
  const Command c =
      command_for_state(PlannerState::Fault, Command{0.4, 0.3}, 0.2, 0.5, -0.45, 0.45);
  EXPECT_EQ(c.speed_mps, 0.0);
  EXPECT_EQ(c.steering_rad, 0.0);
}

TEST(RateLimit, SteeringStepIsBoundedBothWays) {
  const Command prev{0.3, 0.1};
  EXPECT_NEAR(rate_limit(Command{0.3, 0.4}, prev, 0.064, 0.19).steering_rad, 0.164, 1e-12);
  EXPECT_NEAR(rate_limit(Command{0.3, -0.4}, prev, 0.064, 0.19).steering_rad, 0.036, 1e-12);
  EXPECT_NEAR(rate_limit(Command{0.3, 0.12}, prev, 0.064, 0.19).steering_rad, 0.12, 1e-12);
}

TEST(RateLimit, SpeedIsLimitedUpwardOnly) {
  const Command prev{0.3, 0.0};
  EXPECT_NEAR(rate_limit(Command{0.9, 0.0}, prev, 0.064, 0.19).speed_mps, 0.49, 1e-12);
  EXPECT_NEAR(rate_limit(Command{0.4, 0.0}, prev, 0.064, 0.19).speed_mps, 0.4, 1e-12);
  EXPECT_NEAR(rate_limit(Command{0.0, 0.0}, prev, 0.064, 0.19).speed_mps, 0.0,
              1e-12);  // slowing is free
}

TEST(Float32Within, NeverOutsideTheBounds) {
  EXPECT_LE(static_cast<double>(float32_within(0.449, -0.449, 0.449)), 0.449);
  EXPECT_GE(static_cast<double>(float32_within(-0.449, -0.449, 0.449)), -0.449);
  EXPECT_LE(static_cast<double>(float32_within(0.449 + 1e-3, -0.449, 0.449)), 0.449);
  EXPECT_EQ(float32_within(kNaN, -1.0, 1.0), 0.0f);
  EXPECT_NEAR(float32_within(0.25, -1.0, 1.0), 0.25, 1e-9);
  EXPECT_EQ(float32_within(INFINITY, -1.0, 1.0), 0.0f);
}

TEST(SampleBudget, HalvesAfterASlowSolveDownToTheFloor) {
  SampleBudgetParams p;  // max 768, floor 128, halve over 15 ms
  SampleBudget b(p);
  EXPECT_EQ(b.current(), 768);
  bool halved = false;
  EXPECT_EQ(b.update(0.016, &halved), 384);
  EXPECT_TRUE(halved);
  EXPECT_EQ(b.update(0.020, &halved), 192);
  EXPECT_EQ(b.update(0.020, &halved), 128);  // 96 clamps to the floor
  EXPECT_TRUE(halved);
  EXPECT_EQ(b.update(0.020, &halved), 128);  // already at the floor
  EXPECT_FALSE(halved);
}

TEST(SampleBudget, ExactlyAtTheLimitDoesNotHalve) {
  SampleBudget b(SampleBudgetParams{});
  bool halved = true;
  EXPECT_EQ(b.update(0.015, &halved), 768);
  EXPECT_FALSE(halved);
}

TEST(SampleBudget, RestoresGraduallyAfterFastSolves) {
  SampleBudgetParams p;
  p.restore_after_ticks = 5;
  SampleBudget b(p);
  b.update(0.02, nullptr);
  b.update(0.02, nullptr);
  ASSERT_EQ(b.current(), 192);
  int prev = b.current();
  for (int i = 0; i < 4; ++i) EXPECT_EQ(b.update(0.003, nullptr), prev);  // not yet
  const int after = b.update(0.003, nullptr);
  EXPECT_GT(after, prev);
  EXPECT_LT(after, 768);  // gradual, not a jump back
  for (int i = 0; i < 200; ++i) b.update(0.003, nullptr);
  EXPECT_EQ(b.current(), 768);
}

TEST(SampleBudget, ASolveInTheMiddleBandResetsTheRestoreCount) {
  SampleBudgetParams p;
  p.restore_after_ticks = 3;
  SampleBudget b(p);
  b.update(0.02, nullptr);
  ASSERT_EQ(b.current(), 384);
  b.update(0.003, nullptr);
  b.update(0.003, nullptr);
  b.update(0.010, nullptr);  // between 6 and 15 ms
  b.update(0.003, nullptr);
  b.update(0.003, nullptr);
  EXPECT_EQ(b.current(), 384);
  b.update(0.003, nullptr);
  EXPECT_GT(b.current(), 384);
}

TEST(SampleBudget, NaNSolveTimeIsIgnoredAndSmallMaxBelowFloor) {
  SampleBudget b(SampleBudgetParams{});
  EXPECT_EQ(b.update(kNaN, nullptr), 768);
  SampleBudgetParams p;
  p.max_samples = 100;  // below the 128 floor: never reduced further
  SampleBudget small(p);
  EXPECT_EQ(small.update(1.0, nullptr), 100);
}

TEST(Staleness, AgeAndStaleHelpers) {
  EXPECT_DOUBLE_EQ(age_s(10.0, 9.5), 0.5);
  EXPECT_TRUE(std::isnan(age_s(kNaN, 1.0)));
  EXPECT_TRUE(std::isnan(age_s(1.0, kNaN)));
  EXPECT_FALSE(is_stale(10.0, 9.9, 0.2));
  EXPECT_TRUE(is_stale(10.0, 9.7, 0.2));
  EXPECT_TRUE(is_stale(10.0, 10.3, 0.2));
  EXPECT_FALSE(is_stale(10.0, 10.1, 0.2));  // slightly ahead of now is clock skew, not stale
}

TEST(RateLimit, StartFromRestRampsToTheFloorInEqualStepsWithinTheLimit) {
  const double step = 0.086, floor = 0.23;
  const double eq = floor / 3.0;  // ceil(0.23 / 0.086) = 3 equal steps
  Command v{0.0, 0.0};
  std::vector<double> seen;
  for (int i = 0; i < 5; ++i) {
    v = rate_limit(Command{0.5, 0.0}, v, 0.064, step, floor);
    seen.push_back(v.speed_mps);
  }
  EXPECT_NEAR(seen[0], eq, 1e-12);            // inside the dead zone
  EXPECT_NEAR(seen[1], 2.0 * eq, 1e-12);      // inside the dead zone, the last one
  EXPECT_NEAR(seen[2], floor, 1e-12);         // the floor itself
  EXPECT_NEAR(seen[3], floor + step, 1e-12);  // then the ordinary step
  EXPECT_NEAR(seen[4], floor + 2 * step, 1e-12);
  double prev = 0.0;
  for (double x : seen) {
    EXPECT_LE(x - prev, step + 1e-12);  // no step above the planner's own limit
    prev = x;
  }
  // the wanted speed is reached when it is the floor, and not exceeded
  EXPECT_NEAR(
      rate_limit(Command{0.23, 0.0}, Command{2.0 * 0.23 / 3.0, 0.0}, 0.064, step, floor).speed_mps,
      0.23, 1e-12);
  // an unsnapped wanted speed below the floor is only limited, never lifted to the floor
  EXPECT_NEAR(rate_limit(Command{0.1, 0.0}, Command{0.0, 0.0}, 0.064, step, floor).speed_mps, 0.086,
              1e-12);
  EXPECT_NEAR(rate_limit(Command{0.1, 0.0}, Command{0.086, 0.0}, 0.064, step, floor).speed_mps, 0.1,
              1e-12);
  // a floor that one step reaches takes one tick
  EXPECT_NEAR(rate_limit(Command{0.5, 0.0}, Command{0.0, 0.0}, 0.064, step, 0.08).speed_mps, 0.08,
              1e-12);
  // slowing is never limited, and 0 stays 0
  EXPECT_EQ(rate_limit(Command{0.0, 0.0}, Command{0.4, 0.0}, 0.064, step, floor).speed_mps, 0.0);
  EXPECT_EQ(rate_limit(Command{0.0, 0.0}, Command{0.0, 0.0}, 0.064, step, floor).speed_mps, 0.0);
  // floor 0 is the plain rule
  EXPECT_NEAR(rate_limit(Command{0.5, 0.0}, Command{0.086, 0.0}, 0.064, step, 0.0).speed_mps,
              2 * step, 1e-12);
}

TEST(LowPass, FirstOrderStepAndEdgeCases) {
  const double a = 1.0 - std::exp(-0.02 / 0.04);
  EXPECT_NEAR(lowpass_step(0.0, 1.0, 0.02, 0.04), a, 1e-12);
  EXPECT_NEAR(lowpass_step(1.0, 1.0, 0.02, 0.04), 1.0, 1e-12);
  // repeated steps converge to the input
  double y = 0.0;
  for (int i = 0; i < 50; ++i) y = lowpass_step(y, 0.3, 0.02, 0.04);
  EXPECT_NEAR(y, 0.3, 1e-6);
  // tau 0 (off) and bad tick pass the wanted value
  EXPECT_DOUBLE_EQ(lowpass_step(0.0, 0.7, 0.02, 0.0), 0.7);
  EXPECT_DOUBLE_EQ(lowpass_step(0.0, 0.7, 0.0, 0.04), 0.7);
  // NaN wanted keeps the previous value, NaN previous takes the wanted one
  EXPECT_DOUBLE_EQ(lowpass_step(0.2, std::nan(""), 0.02, 0.04), 0.2);
  EXPECT_DOUBLE_EQ(lowpass_step(std::nan(""), 0.7, 0.02, 0.04), 0.7);
}
