// Table driven tests of every row of the decision table at pass, boundary and fail values,
// including NaN inputs.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

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
  SelfCheckHold hold;
  return evaluate_gate(in, p, &hold).state;
}

}  // namespace

TEST(Gate, HealthyInputsDrive) {
  SelfCheckHold hold;
  const Gate g = evaluate_gate(healthy(), LogicParams{}, &hold);
  EXPECT_EQ(g.state, PlannerState::Driving);
  EXPECT_TRUE(g.may_solve);
}

TEST(Gate, InternalErrorIsFaultAndWinsOverEverything) {
  LogicInputs in = healthy();
  in.internal_error = true;
  in.has_odom = false;  // would be NO_ODOM
  const Gate g = [&] {
    SelfCheckHold hold;
    return evaluate_gate(in, LogicParams{}, &hold);
  }();
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
    in.track.stamp_s = in.now_s - 0.1;  // keep the track fresh so only the scan row is tested
    EXPECT_EQ(gate_state(in, p), c.stale ? PlannerState::NoScan : PlannerState::Driving) << c.name;
  }
  LogicInputs none = healthy();
  none.has_scan = false;
  EXPECT_EQ(gate_state(none), PlannerState::NoScan);
}

TEST(Gate, TrackRowAtPassBoundaryAndFail) {
  const LogicParams p;  // 0.30 s
  const StaleCase cases[] = {
      {"fresh", 0.1, false}, {"at the limit", 0.30, false}, {"just over", 0.301, true}};
  for (const auto& c : cases) {
    LogicInputs in = healthy();
    in.track.stamp_s = in.now_s - c.age;
    EXPECT_EQ(gate_state(in, p), c.stale ? PlannerState::NoTrack : PlannerState::Driving) << c.name;
  }
  LogicInputs none = healthy();
  none.track.present = false;
  EXPECT_EQ(gate_state(none), PlannerState::NoTrack);
}

TEST(Gate, TrackInvalidPerTheContractIsNoTrack) {
  for (TrackValidity v :
       {TrackValidity::NoTrack, TrackValidity::WrongLength, TrackValidity::WrongSpacing,
        TrackValidity::WrongFrame, TrackValidity::NonFinite}) {
    LogicInputs in = healthy();
    in.track.validity = v;
    EXPECT_EQ(gate_state(in), PlannerState::NoTrack) << static_cast<int>(v);
  }
}

TEST(Gate, MinValidStationsPassBoundaryFail) {
  LogicParams p;  // 6
  LogicInputs in = healthy();
  in.track.valid_count = 7;
  EXPECT_EQ(gate_state(in, p), PlannerState::Driving);
  in.track.valid_count = 6;
  EXPECT_EQ(gate_state(in, p), PlannerState::Driving);
  in.track.valid_count = 5;
  EXPECT_EQ(gate_state(in, p), PlannerState::NoTrack);
  in.track.valid_count = 0;
  EXPECT_EQ(gate_state(in, p), PlannerState::NoTrack);
}

TEST(Gate, TimestampJumpsAreStale) {
  LogicInputs in = healthy();
  in.track.stamp_s = in.now_s + 10.0;  // from the future
  EXPECT_EQ(gate_state(in), PlannerState::NoTrack);
  in = healthy();
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
  in.track.stamp_s = kNaN;
  EXPECT_EQ(gate_state(in), PlannerState::NoTrack);
  in = healthy();
  in.now_s = kNaN;
  EXPECT_EQ(gate_state(in), PlannerState::NoOdom);
}

TEST(Gate, RowsAreCheckedInTheTableOrder) {
  LogicInputs in = healthy();
  in.has_odom = false;
  in.has_scan = false;
  in.track.present = false;
  EXPECT_EQ(gate_state(in), PlannerState::NoOdom);
  in.has_odom = true;
  EXPECT_EQ(gate_state(in), PlannerState::NoScan);
  in.has_scan = true;
  EXPECT_EQ(gate_state(in), PlannerState::NoTrack);
  in.track.present = true;
  EXPECT_EQ(gate_state(in), PlannerState::Driving);
}

TEST(SelfCheck, LimitsAtPassBoundaryAndFail) {
  const LogicParams p;  // 0.25 m, 0.10
  EXPECT_FALSE(selfcheck_over_limits(0.10, 0.02, p));
  EXPECT_FALSE(selfcheck_over_limits(0.25, 0.10, p));  // boundary is not over
  EXPECT_TRUE(selfcheck_over_limits(0.2501, 0.0, p));
  EXPECT_TRUE(selfcheck_over_limits(0.0, 0.1001, p));
}

TEST(SelfCheck, NaNCountsAsOverTheLimit) {
  const LogicParams p;
  EXPECT_TRUE(selfcheck_over_limits(kNaN, 0.0, p));
  EXPECT_TRUE(selfcheck_over_limits(0.0, kNaN, p));
  EXPECT_TRUE(selfcheck_over_limits(INFINITY, 0.0, p));
}

TEST(SelfCheckHold, MustBeContinuousForTheHoldTime) {
  SelfCheckHold h;
  EXPECT_FALSE(h.update(true, 10.0, 0.5));
  EXPECT_FALSE(h.update(true, 10.3, 0.5));
  EXPECT_FALSE(h.update(true, 10.499, 0.5));
  EXPECT_TRUE(h.update(true, 10.5, 0.5));
  EXPECT_TRUE(h.update(true, 12.0, 0.5));
  EXPECT_FALSE(h.update(false, 12.02, 0.5));  // recovers at once
  EXPECT_FALSE(h.update(true, 12.04, 0.5));   // and the timer restarts
  EXPECT_FALSE(h.update(true, 12.3, 0.5));
  EXPECT_TRUE(h.update(true, 12.55, 0.5));
}

TEST(SelfCheckHold, ZeroHoldTripsImmediately) {
  SelfCheckHold h;
  EXPECT_TRUE(h.update(true, 5.0, 0.0));
}

TEST(SelfCheckHold, NaNTimeResetsAndDoesNotTrip) {
  SelfCheckHold h;
  h.update(true, 1.0, 0.5);
  EXPECT_FALSE(h.update(true, kNaN, 0.5));
  EXPECT_FALSE(h.update(true, 1.2, 0.5));  // restarted at 1.2
}

TEST(Gate, SelfCheckOverLimitForTheHoldTimeGivesNoTrackThenRecovers) {
  LogicParams p;
  SelfCheckHold hold;
  LogicInputs in = healthy();
  in.track.boundary_residual_m = 0.5;  // over
  double t = 100.0;
  PlannerState s = PlannerState::Driving;
  for (int i = 0; i < 20; ++i, t += 0.02) {
    in.now_s = t;
    in.odom_stamp_s = t;
    in.scan_stamp_s = t - 0.1;
    in.track.stamp_s = t - 0.1;
    s = evaluate_gate(in, p, &hold).state;
    EXPECT_EQ(s, PlannerState::Driving) << i;  // 0.38 s over, below the 0.5 s hold
  }
  for (int i = 0; i < 10; ++i, t += 0.02) {
    in.now_s = t;
    in.odom_stamp_s = t;
    in.scan_stamp_s = t - 0.1;
    in.track.stamp_s = t - 0.1;
    s = evaluate_gate(in, p, &hold).state;
  }
  EXPECT_EQ(s, PlannerState::NoTrack);
  in.track.boundary_residual_m = 0.05;
  in.now_s = t;
  in.odom_stamp_s = t;
  in.scan_stamp_s = t - 0.1;
  in.track.stamp_s = t - 0.1;
  EXPECT_EQ(evaluate_gate(in, p, &hold).state, PlannerState::Driving);
}

TEST(Gate, SelfCheckIntrusionAndNaNAlsoTrip) {
  for (int variant = 0; variant < 3; ++variant) {
    LogicParams p;
    p.selfcheck_hold_s = 0.0;
    LogicInputs in = healthy();
    if (variant == 0) in.track.intrusion_fraction = 0.5;
    if (variant == 1) in.track.boundary_residual_m = kNaN;
    if (variant == 2) in.track.intrusion_fraction = kNaN;
    EXPECT_EQ(gate_state(in, p), PlannerState::NoTrack) << variant;
  }
}

TEST(Gate, SelfCheckHoldIsResetByAnUnusableTrack) {
  LogicParams p;
  SelfCheckHold hold;
  LogicInputs in = healthy();
  in.track.boundary_residual_m = 0.9;
  evaluate_gate(in, p, &hold);  // starts the timer at t = 100
  LogicInputs gone = in;
  gone.track.present = false;
  gone.now_s = 100.4;
  gone.odom_stamp_s = 100.4;
  gone.scan_stamp_s = 100.3;
  EXPECT_EQ(evaluate_gate(gone, p, &hold).state, PlannerState::NoTrack);
  in.now_s = 100.8;
  in.odom_stamp_s = 100.8;
  in.scan_stamp_s = 100.7;
  in.track.stamp_s = 100.7;
  // only 0.0 s over since the reset, not 0.8 s
  EXPECT_EQ(evaluate_gate(in, p, &hold).state, PlannerState::Driving);
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

TEST(RateLimit, SpeedFloorJumpsOutOfTheDeadBandOnlyWhenTheWantedSpeedIsAtLeastTheFloor) {
  const Command rest{0.0, 0.0};
  // from rest toward 0.5 with a 0.086 step: would land at 0.086, inside (0, 0.23): jump
  EXPECT_NEAR(rate_limit(Command{0.5, 0.0}, rest, 0.064, 0.086, 0.23).speed_mps, 0.23, 1e-12);
  // the wanted speed is exactly the floor: the same
  EXPECT_NEAR(rate_limit(Command{0.23, 0.0}, rest, 0.064, 0.086, 0.23).speed_mps, 0.23, 1e-12);
  // an unsnapped wanted speed below the floor is not lifted, only limited as before
  EXPECT_NEAR(rate_limit(Command{0.1, 0.0}, rest, 0.064, 0.086, 0.23).speed_mps, 0.086, 1e-12);
  EXPECT_NEAR(rate_limit(Command{0.05, 0.0}, rest, 0.064, 0.086, 0.23).speed_mps, 0.05, 1e-12);
  // above the floor the ordinary step applies
  EXPECT_NEAR(rate_limit(Command{0.9, 0.0}, Command{0.3, 0.0}, 0.064, 0.086, 0.23).speed_mps, 0.386,
              1e-12);
  // slowing is never limited, and 0 stays 0
  EXPECT_EQ(rate_limit(Command{0.0, 0.0}, Command{0.4, 0.0}, 0.064, 0.086, 0.23).speed_mps, 0.0);
  EXPECT_EQ(rate_limit(Command{0.0, 0.0}, rest, 0.064, 0.086, 0.23).speed_mps, 0.0);
  // floor 0 is the old rule
  EXPECT_NEAR(rate_limit(Command{0.5, 0.0}, rest, 0.064, 0.086, 0.0).speed_mps, 0.086, 1e-12);
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
