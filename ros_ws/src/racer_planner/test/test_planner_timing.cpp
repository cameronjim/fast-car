// Planner timing and output hardening (design 14.7): the elapsed time slew, the warm start gate,
// publishing from a plan snapshot, graceful staleness and the lap guidance cap fallback. L1 tables
// of the pure functions first, then the Planner (solve and publish) with explicit times.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "planner_rig.hpp"
#include "racer_planner/freespace_field.hpp"
#include "racer_planner/planner.hpp"
#include "racer_planner/planner_logic.hpp"
#include "test_support.hpp"

using namespace racer_planner;
using namespace racer_planner::rig;

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

LogicInputs healthy() {
  LogicInputs in;
  in.now_s = 100.0;
  in.has_odom = true;
  in.odom_stamp_s = 99.99;
  in.wheel_speed_mps = 0.3;
  in.has_scan = true;
  in.scan_stamp_s = 99.8;
  return in;
}

LogicParams params(double v_max = 0.5) {
  LogicParams p;
  p.v_max_mps = v_max;
  return p;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// classify_age

TEST(ClassifyAge, FreshSoftHardAtJustInsideAndJustOutsideEachBound) {
  const double now = 100.0, t = 0.10, h = 0.20;
  EXPECT_EQ(classify_age(now, now - 0.0, t, h), Age::Fresh);
  EXPECT_EQ(classify_age(now, now - 0.099, t, h), Age::Fresh);
  EXPECT_EQ(classify_age(now, now - 0.101, t, h), Age::Soft);
  EXPECT_EQ(classify_age(now, now - 0.199, t, h), Age::Soft);
  EXPECT_EQ(classify_age(now, now - 0.201, t, h), Age::Hard);
  EXPECT_EQ(classify_age(now, now - 9.0, t, h), Age::Hard);
}

TEST(ClassifyAge, FutureStampsAreClockSkewUpToTheFirstTimeoutThenHard) {
  const double now = 100.0, t = 0.10, h = 0.20;
  EXPECT_EQ(classify_age(now, now + 0.05, t, h), Age::Fresh);
  EXPECT_EQ(classify_age(now, now + 0.099, t, h), Age::Fresh);
  EXPECT_EQ(classify_age(now, now + 0.101, t, h), Age::Hard);
  EXPECT_EQ(classify_age(now, now + 50.0, t, h), Age::Hard);
}

TEST(ClassifyAge, GarbageIsHardAtOnce) {
  EXPECT_EQ(classify_age(100.0, kNaN, 0.1, 0.2), Age::Hard);
  EXPECT_EQ(classify_age(kNaN, 100.0, 0.1, 0.2), Age::Hard);
  EXPECT_EQ(classify_age(100.0, kInf, 0.1, 0.2), Age::Hard);
  EXPECT_EQ(classify_age(100.0, -kInf, 0.1, 0.2), Age::Hard);
  EXPECT_EQ(classify_age(kInf, kInf, 0.1, 0.2), Age::Hard);
}

TEST(ClassifyAge, ADegenerateHardBoundIsTheFirstOne) {
  EXPECT_EQ(classify_age(100.0, 99.85, 0.1, 0.05), Age::Hard);  // hard below soft: no soft zone
  EXPECT_EQ(classify_age(100.0, 99.95, 0.1, 0.05), Age::Fresh);
}

// ---------------------------------------------------------------------------------------------
// speed_cap: scan staleness ramp, wheel speed staleness, guidance fallback

TEST(SpeedCapScan, FullUpToTheSoftBoundThenALinearRampToZeroAtTheHardBound) {
  const LogicParams p = params(0.5);  // 0.35 soft, 0.60 hard, free-space fraction 0.7
  auto cap_at = [&](double age) {
    LogicInputs in = healthy();
    in.scan_stamp_s = in.now_s - age;
    return speed_cap(p, in, 1.0);
  };
  EXPECT_NEAR(cap_at(0.10), 0.5, 1e-12);
  EXPECT_NEAR(cap_at(0.349), 0.5, 1e-12);
  EXPECT_NEAR(cap_at(0.35 + 0.0625), 0.5 * 0.75, 1e-9);  // a quarter of the way down
  EXPECT_NEAR(cap_at(0.475), 0.25, 1e-9);                // halfway
  EXPECT_NEAR(cap_at(0.5375), 0.125, 1e-9);
  EXPECT_NEAR(cap_at(0.599), 0.5 * (0.60 - 0.599) / 0.25, 1e-9);
  EXPECT_DOUBLE_EQ(cap_at(0.601), 0.0);
  EXPECT_DOUBLE_EQ(cap_at(30.0), 0.0);
}

TEST(SpeedCapScan, TheRampMultipliesTheConfidenceScaledCap) {
  const LogicParams p = params(0.5);
  LogicInputs in = healthy();
  in.scan_stamp_s = in.now_s - 0.475;
  EXPECT_NEAR(speed_cap(p, in, 0.0), 0.35 * 0.5, 1e-9);
}

TEST(SpeedCapScan, MissingFutureAndNonFiniteScansGiveZero) {
  const LogicParams p = params(0.5);
  LogicInputs in = healthy();
  in.has_scan = false;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.0);
  in = healthy();
  in.scan_stamp_s = in.now_s + 5.0;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.0);
  in.scan_stamp_s = kNaN;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.0);
  in.scan_stamp_s = -kInf;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.0);
  in = healthy();
  in.now_s = kNaN;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.0);
}

TEST(SpeedCapOdom, FullWhenFreshAndLastWheelSpeedPlusMarginWhileSoftStale) {
  const LogicParams p = params(0.9);
  auto cap_at = [&](double age, double wheel) {
    LogicInputs in = healthy();
    in.odom_stamp_s = in.now_s - age;
    in.wheel_speed_mps = wheel;
    return speed_cap(p, in, 1.0);
  };
  EXPECT_NEAR(cap_at(0.05, 0.3), 0.9, 1e-12);
  EXPECT_NEAR(cap_at(0.099, 0.3), 0.9, 1e-12);
  EXPECT_NEAR(cap_at(0.101, 0.3), 0.35, 1e-12);
  EXPECT_NEAR(cap_at(0.199, 0.3), 0.35, 1e-12);
  EXPECT_NEAR(cap_at(0.15, 0.0), 0.05, 1e-12);   // from rest: no acceleration on a guess
  EXPECT_NEAR(cap_at(0.15, -0.4), 0.05, 1e-12);  // reverse counts as rest
  EXPECT_NEAR(cap_at(0.15, 2.0), 0.9, 1e-12);    // never above the cap itself
  EXPECT_NEAR(cap_at(0.15, kNaN), 0.05, 1e-12);  // a garbage wheel speed is rest
  EXPECT_DOUBLE_EQ(cap_at(0.201, 0.3), 0.0);
  EXPECT_DOUBLE_EQ(cap_at(5.0, 0.3), 0.0);
}

TEST(SpeedCapOdom, MissingFutureAndNonFiniteStampsGiveZero) {
  const LogicParams p = params(0.5);
  LogicInputs in = healthy();
  in.has_odom = false;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.0);
  in = healthy();
  in.odom_stamp_s = in.now_s + 1.0;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.0);
  in.odom_stamp_s = kNaN;
  EXPECT_DOUBLE_EQ(speed_cap(p, in), 0.0);
}

TEST(SpeedCapOdom, TheMarginIsAParameter) {
  LogicParams p = params(0.9);
  p.odom_stale_cap_margin_mps = 0.1;
  LogicInputs in = healthy();
  in.odom_stamp_s = in.now_s - 0.15;
  EXPECT_NEAR(speed_cap(p, in), 0.4, 1e-12);
  p.odom_stale_cap_margin_mps = -1.0;  // a negative margin is no margin
  EXPECT_NEAR(speed_cap(p, in), 0.3, 1e-12);
}

TEST(SpeedCapGuidance, WithGuidanceOnAndNoUsableGuidanceTheCapIsTheFallbackNotTheCeiling) {
  LogicParams p = params(1.0);  // the ceiling
  p.use_lap_guidance = true;
  p.v_no_guidance_mps = 0.4;
  LogicInputs in = healthy();
  // none yet
  EXPECT_NEAR(speed_cap(p, in), 0.4, 1e-12);
  // fresh and good: the guidance, whichever is lower than the ceiling (it may be above the
  // fallback)
  in.has_guidance = true;
  in.guidance_stamp_s = in.now_s - 0.5;
  in.guidance_v_cap_mps = 0.7;
  EXPECT_NEAR(speed_cap(p, in), 0.7, 1e-12);
  in.guidance_v_cap_mps = 0.2;
  EXPECT_NEAR(speed_cap(p, in), 0.2, 1e-12);
  // stale (2.0 s limit)
  in.guidance_v_cap_mps = 0.7;
  in.guidance_stamp_s = in.now_s - 2.01;
  EXPECT_NEAR(speed_cap(p, in), 0.4, 1e-12);
  // not finite
  in.guidance_stamp_s = in.now_s - 0.5;
  in.guidance_v_cap_mps = kNaN;
  EXPECT_NEAR(speed_cap(p, in), 0.4, 1e-12);
  in.guidance_v_cap_mps = kInf;
  EXPECT_NEAR(speed_cap(p, in), 0.4, 1e-12);
  // a bad lap profile forces the fallback even with a fresh good cap
  in.guidance_v_cap_mps = 0.7;
  in.guidance_profile_bad = true;
  EXPECT_NEAR(speed_cap(p, in), 0.4, 1e-12);
  in.guidance_profile_bad = false;
  EXPECT_NEAR(speed_cap(p, in), 0.7, 1e-12);
}

TEST(SpeedCapGuidance, TheFallbackNeverExceedsVMaxAndANaNFallbackIsZero) {
  LogicParams p = params(0.5);
  p.use_lap_guidance = true;
  p.v_no_guidance_mps = 2.0;
  EXPECT_NEAR(speed_cap(p, healthy()), 0.5, 1e-12);
  p.v_no_guidance_mps = kNaN;
  EXPECT_DOUBLE_EQ(speed_cap(p, healthy()), 0.0);
  p.v_no_guidance_mps = 0.0;
  EXPECT_DOUBLE_EQ(speed_cap(p, healthy()), 0.0);
  p.v_no_guidance_mps = -0.3;
  EXPECT_DOUBLE_EQ(speed_cap(p, healthy()), 0.0);
}

TEST(SpeedCapGuidance, WithGuidanceOffNothingChanges) {
  LogicParams p = params(0.5);
  p.use_lap_guidance = false;
  p.v_no_guidance_mps = 0.1;
  LogicInputs in = healthy();
  EXPECT_NEAR(speed_cap(p, in), 0.5, 1e-12);
  in.guidance_profile_bad = true;
  EXPECT_NEAR(speed_cap(p, in), 0.5, 1e-12);
  in.has_guidance = true;
  in.guidance_stamp_s = in.now_s;
  in.guidance_v_cap_mps = 0.2;
  EXPECT_NEAR(speed_cap(p, in), 0.5, 1e-12);
}

TEST(SpeedCapGuidance, TheDefaultFallbackIsNoLimitBeyondVMax) {
  LogicParams p = params(0.8);
  p.use_lap_guidance = true;  // v_no_guidance_mps at its default
  EXPECT_NEAR(speed_cap(p, healthy()), 0.8, 1e-12);
}

TEST(EvaluateGate, SoftStaleInputsStillSolveAndHardStaleOnesDoNot) {
  const LogicParams p;
  LogicInputs in = healthy();
  in.scan_stamp_s = in.now_s - 0.5;
  in.odom_stamp_s = in.now_s - 0.15;
  EXPECT_TRUE(evaluate_gate(in, p).may_solve);
  in.scan_stamp_s = in.now_s - 0.65;
  EXPECT_EQ(evaluate_gate(in, p).state, PlannerState::NoScan);
  in.scan_stamp_s = in.now_s - 0.5;
  in.odom_stamp_s = in.now_s - 0.25;
  EXPECT_EQ(evaluate_gate(in, p).state, PlannerState::NoOdom);
}

// ---------------------------------------------------------------------------------------------
// slew_elapsed_s, slew_limit

TEST(SlewElapsed, OnTimeBunchedLateZeroNegativeAndNaN) {
  const double tick = 0.02;
  EXPECT_DOUBLE_EQ(slew_elapsed_s(0.02, tick), 0.02);
  EXPECT_DOUBLE_EQ(slew_elapsed_s(0.013, tick), 0.013);
  EXPECT_DOUBLE_EQ(slew_elapsed_s(0.002, tick), 0.002);
  EXPECT_DOUBLE_EQ(slew_elapsed_s(0.08, tick), tick);  // late: one tick at most
  EXPECT_DOUBLE_EQ(slew_elapsed_s(30.0, tick), tick);
  EXPECT_DOUBLE_EQ(slew_elapsed_s(0.0, tick), 0.0);
  EXPECT_DOUBLE_EQ(slew_elapsed_s(-0.01, tick), 0.0);
  EXPECT_DOUBLE_EQ(slew_elapsed_s(kNaN, tick), 0.0);
  EXPECT_DOUBLE_EQ(slew_elapsed_s(-kInf, tick), 0.0);
  EXPECT_DOUBLE_EQ(slew_elapsed_s(kInf, tick), tick);
}

namespace {
SlewLimits car_limits(double floor_mps = 0.0) {
  SlewLimits l;
  l.tick_s = 0.02;
  l.steering_rate_rad_per_s = 6.4;  // steering step 0.45 * 6.4 * 0.02 = 0.0576 per tick
  l.max_acceleration_mps2 = 9.5;
  l.fraction = 0.45;
  l.speed_floor_mps = floor_mps;
  return l;
}
}  // namespace

TEST(SlewLimit, StepsFollowTheElapsedTime) {
  const SlewLimits l = car_limits();
  const Command prev{0.3, 0.0};
  const Command want{0.9, 0.4};
  const Command on_time = slew_limit(want, prev, 0.02, l);
  EXPECT_NEAR(on_time.steering_rad, 0.45 * 6.4 * 0.02, 1e-12);
  EXPECT_NEAR(on_time.speed_mps, 0.3 + 0.45 * 9.5 * 0.02, 1e-12);
  const Command bunched = slew_limit(want, prev, 0.003, l);
  EXPECT_NEAR(bunched.steering_rad, 0.45 * 6.4 * 0.003, 1e-12);
  EXPECT_NEAR(bunched.speed_mps, 0.3 + 0.45 * 9.5 * 0.003, 1e-12);
  const Command late = slew_limit(want, prev, 0.2, l);  // never more than one tick's worth
  EXPECT_NEAR(late.steering_rad, on_time.steering_rad, 1e-12);
  EXPECT_NEAR(late.speed_mps, on_time.speed_mps, 1e-12);
  const Command zero = slew_limit(want, prev, 0.0, l);
  EXPECT_DOUBLE_EQ(zero.steering_rad, 0.0);
  EXPECT_DOUBLE_EQ(zero.speed_mps, 0.3);
  const Command negative = slew_limit(want, prev, -1.0, l);
  EXPECT_DOUBLE_EQ(negative.steering_rad, 0.0);
  EXPECT_DOUBLE_EQ(negative.speed_mps, 0.3);
  const Command nan = slew_limit(want, prev, kNaN, l);
  EXPECT_DOUBLE_EQ(nan.steering_rad, 0.0);
  EXPECT_DOUBLE_EQ(nan.speed_mps, 0.3);
}

TEST(SlewLimit, SlowingAndSmallRequestsAreNotLimited) {
  const SlewLimits l = car_limits();
  EXPECT_DOUBLE_EQ(slew_limit({0.0, 0.0}, {0.5, 0.0}, 0.001, l).speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(slew_limit({0.4, 0.01}, {0.3, 0.0}, 0.02, l).steering_rad, 0.01);
  EXPECT_DOUBLE_EQ(slew_limit({0.35, 0.0}, {0.3, 0.0}, 0.02, l).speed_mps, 0.35);
}

TEST(SlewLimit, TheDeadZoneRampKeepsItsShapeOnTimeAndScalesWhenTheTickIsShort) {
  const double floor = 0.23;
  const SlewLimits l = car_limits(floor);
  const double up = 0.45 * 9.5 * 0.02;  // 0.0855 per tick
  const double eq = floor / std::ceil(floor / up);
  EXPECT_NEAR(slew_limit({0.5, 0.0}, {0.0, 0.0}, 0.02, l).speed_mps, eq, 1e-12);
  EXPECT_NEAR(slew_limit({0.5, 0.0}, {eq, 0.0}, 0.02, l).speed_mps, 2 * eq, 1e-12);
  EXPECT_NEAR(slew_limit({0.5, 0.0}, {2 * eq, 0.0}, 0.02, l).speed_mps, floor, 1e-12);
  EXPECT_NEAR(slew_limit({0.5, 0.0}, {floor, 0.0}, 0.02, l).speed_mps, floor + up, 1e-12);
  // half a tick: half the step
  EXPECT_NEAR(slew_limit({0.5, 0.0}, {0.0, 0.0}, 0.01, l).speed_mps, 0.5 * eq, 1e-12);
  EXPECT_NEAR(slew_limit({0.5, 0.0}, {0.0, 0.0}, 0.0, l).speed_mps, 0.0, 1e-12);
  // a late tick is one tick's step
  EXPECT_NEAR(slew_limit({0.5, 0.0}, {0.0, 0.0}, 0.5, l).speed_mps, eq, 1e-12);
  // no step is above the limit of its own elapsed time
  for (double e : {0.001, 0.005, 0.013, 0.02}) {
    EXPECT_LE(slew_limit({0.5, 0.0}, {0.0, 0.0}, e, l).speed_mps, 0.45 * 9.5 * e + 1e-12) << e;
  }
}

TEST(SlewLimit, TheDeadZoneRampTakesTheSameWallTimeHoweverTheTicksLand) {
  const double floor = 0.23, tick = 0.02;
  const SlewLimits l = car_limits(floor);
  const double up = 0.45 * 9.5 * tick;
  const double wall = std::ceil(floor / up) * tick;  // three ticks
  const std::vector<std::vector<double>> patterns = {
      {0.02},
      {0.01},
      {0.005, 0.02, 0.011},
      {0.002, 0.018, 0.02, 0.003, 0.017},
      {0.02, 0.0, 0.0, 0.02},
      {0.3, 0.001},
  };
  for (const auto& pat : patterns) {
    Command v{0.0, 0.0};
    double t = 0.0;
    std::size_t i = 0;
    int guard = 0;
    while (v.speed_mps < floor - 1e-12 && guard++ < 100000) {
      const double e = std::min(pat[i++ % pat.size()], tick);
      v = slew_limit({0.5, 0.0}, v, e, l);
      t += e;
    }
    EXPECT_GE(t, wall - 1e-9);
    EXPECT_LT(t, wall + tick + 1e-9);
  }
}

TEST(SlewLimit, ReverseGrowsByTheElapsedStepAndBrakesToZeroAtOnce) {
  const SlewLimits l = car_limits(0.23);
  const Command a = slew_limit({-0.3, 0.0}, {0.0, 0.0}, 0.01, l);
  EXPECT_NEAR(a.speed_mps, -0.45 * 9.5 * 0.01, 1e-12);
  const Command b = slew_limit({-0.3, 0.0}, {0.4, 0.0}, 0.02, l);  // from forward: to zero
  EXPECT_NEAR(b.speed_mps, -0.45 * 9.5 * 0.02, 1e-12);
}

// ---------------------------------------------------------------------------------------------
// interpolate_plan

TEST(InterpolatePlan, InsideTheHorizonAtItsEndsAndBeyond) {
  const Control seq[3] = {{0.2, 0.0}, {0.4, 0.2}, {0.8, -0.2}};
  const double dt = 0.05;
  auto at = [&](double age) { return interpolate_plan(seq, 3, dt, age); };
  EXPECT_DOUBLE_EQ(at(0.0).speed_cmd, 0.2);
  EXPECT_NEAR(at(0.025).speed_cmd, 0.3, 1e-12);
  EXPECT_NEAR(at(0.025).steering_cmd, 0.1, 1e-12);
  EXPECT_NEAR(at(0.05).speed_cmd, 0.4, 1e-12);
  EXPECT_NEAR(at(0.075).speed_cmd, 0.6, 1e-12);
  EXPECT_NEAR(at(0.075).steering_cmd, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(at(0.1).speed_cmd, 0.8);  // exactly the end
  EXPECT_DOUBLE_EQ(at(0.1 + 1e-9).speed_cmd, 0.8);
  EXPECT_DOUBLE_EQ(at(9.0).steering_cmd, -0.2);  // held beyond the horizon
}

TEST(InterpolatePlan, NegativeNaNAndDegenerateInputs) {
  const Control seq[2] = {{0.2, 0.1}, {0.6, 0.3}};
  EXPECT_DOUBLE_EQ(interpolate_plan(seq, 2, 0.05, -1.0).speed_cmd, 0.2);  // before the plan: first
  EXPECT_DOUBLE_EQ(interpolate_plan(seq, 2, 0.05, kNaN).speed_cmd, 0.2);
  EXPECT_DOUBLE_EQ(interpolate_plan(seq, 2, 0.0, 0.03).speed_cmd, 0.2);  // no step: first
  EXPECT_DOUBLE_EQ(interpolate_plan(seq, 2, kNaN, 0.03).speed_cmd, 0.2);
  EXPECT_DOUBLE_EQ(interpolate_plan(seq, 1, 0.05, 0.03).speed_cmd, 0.2);  // one control: held
  EXPECT_DOUBLE_EQ(interpolate_plan(seq, 0, 0.05, 0.03).speed_cmd, 0.0);
  EXPECT_DOUBLE_EQ(interpolate_plan(nullptr, 2, 0.05, 0.03).steering_cmd, 0.0);
  EXPECT_DOUBLE_EQ(interpolate_plan(seq, 2, 0.05, kInf).speed_cmd, 0.6);
}

// ---------------------------------------------------------------------------------------------
// start_gate

namespace {
StartGateInputs all_good() {
  StartGateInputs in;
  in.plan_fresh = in.scan_fresh = in.odom_fresh = in.track_ever = in.track_fresh = true;
  in.publish_good_ticks = 25;
  in.solve_good_ticks = 25;
  in.gated_seen = true;
  return in;
}
}  // namespace

TEST(StartGate, EverythingHeldLeaves) {
  const StartGateResult r = start_gate(all_good(), LogicParams{});
  EXPECT_TRUE(r.leave);
  EXPECT_EQ(r.missing, StartMissing::None);
}

TEST(StartGate, EachConditionAloneHoldsAndIsNamed) {
  const LogicParams p;
  struct Case {
    const char* name;
    StartMissing missing;
    void (*breaker)(StartGateInputs&);
  } cases[] = {
      {"plan", StartMissing::NoPlan, [](StartGateInputs& i) { i.plan_fresh = false; }},
      {"scan", StartMissing::Scan, [](StartGateInputs& i) { i.scan_fresh = false; }},
      {"odom", StartMissing::Odom, [](StartGateInputs& i) { i.odom_fresh = false; }},
      {"track", StartMissing::Track, [](StartGateInputs& i) { i.track_fresh = false; }},
      {"publish ticks", StartMissing::TickTiming,
       [](StartGateInputs& i) { i.publish_good_ticks = 24; }},
      {"solves", StartMissing::SolverWarmup, [](StartGateInputs& i) { i.solve_good_ticks = 24; }},
      {"gated", StartMissing::Gated, [](StartGateInputs& i) { i.gated_seen = false; }},
  };
  for (const auto& c : cases) {
    StartGateInputs in = all_good();
    c.breaker(in);
    const StartGateResult r = start_gate(in, p);
    EXPECT_FALSE(r.leave) << c.name;
    EXPECT_EQ(r.missing, c.missing) << c.name;
    EXPECT_NE(std::string(start_missing_name(c.missing)), "unknown") << c.name;
  }
}

TEST(StartGate, ATrackThatHasNeverArrivedDoesNotHold) {
  StartGateInputs in = all_good();
  in.track_ever = false;
  in.track_fresh = false;
  EXPECT_TRUE(start_gate(in, LogicParams{}).leave);
  in.track_ever = true;  // arrived once, not fresh now
  EXPECT_EQ(start_gate(in, LogicParams{}).missing, StartMissing::Track);
}

TEST(StartGate, TheGatedDriveIsOnlyRequiredWhenAskedFor) {
  LogicParams p;
  p.require_gated_for_start = false;
  StartGateInputs in = all_good();
  in.gated_seen = false;
  EXPECT_TRUE(start_gate(in, p).leave);
}

TEST(StartGate, TheTickCountIsAParameterAndMoreIsFine) {
  LogicParams p;
  p.start_ticks = 5;
  StartGateInputs in = all_good();
  in.publish_good_ticks = 5;
  in.solve_good_ticks = 5000;
  EXPECT_TRUE(start_gate(in, p).leave);
  in.publish_good_ticks = 4;
  EXPECT_EQ(start_gate(in, p).missing, StartMissing::TickTiming);
}

TEST(StartGate, MissingConditionsAreReportedInTheDocumentedOrder) {
  StartGateInputs in;  // nothing true, nothing counted
  const LogicParams p;
  EXPECT_EQ(start_gate(in, p).missing, StartMissing::NoPlan);
  in.plan_fresh = true;
  EXPECT_EQ(start_gate(in, p).missing, StartMissing::Scan);
  in.scan_fresh = true;
  EXPECT_EQ(start_gate(in, p).missing, StartMissing::Odom);
  in.odom_fresh = true;
  EXPECT_EQ(start_gate(in, p).missing, StartMissing::TickTiming);
  in.publish_good_ticks = 25;
  EXPECT_EQ(start_gate(in, p).missing, StartMissing::SolverWarmup);
  in.solve_good_ticks = 25;
  EXPECT_EQ(start_gate(in, p).missing, StartMissing::Gated);
  in.gated_seen = true;
  EXPECT_TRUE(start_gate(in, p).leave);
}

// ---------------------------------------------------------------------------------------------
// The Planner: solve and publish with explicit times

TEST(PlannerStart, HoldsSpeedZeroForTwentyFiveOnTimeTicksThenDrives) {
  Rig r;
  double t = 1000.0;
  int first_driving = -1;
  for (int i = 0; i < 60; ++i) {
    const TickOutput out = r.tick(t);
    t += 0.02;
    if (out.state == PlannerState::Starting) {
      EXPECT_EQ(out.cmd.speed_mps, 0.0) << "tick " << i;
    }
    if (first_driving < 0 && out.state == PlannerState::Driving) first_driving = i;
  }
  // 25 gaps within 30 ms are counted from the second tick on, so the gate opens on tick 25 and the
  // first Driving output follows on the next real snapshot
  EXPECT_GE(first_driving, 25);
  EXPECT_LE(first_driving, 28);
}

TEST(PlannerStart, ALateTickRestartsTheCount) {
  Rig r;
  double t = 1000.0;
  for (int i = 0; i < 20; ++i) {
    r.tick(t);
    t += 0.02;
  }
  t += 0.04;  // one gap of 60 ms: not within 30 ms
  TickOutput out;
  int ticks_after = 0;
  while (ticks_after < 60) {
    out = r.tick(t);
    t += 0.02;
    ++ticks_after;
    if (out.state == PlannerState::Driving) break;
  }
  EXPECT_EQ(out.state, PlannerState::Driving);
  EXPECT_GE(ticks_after, 25) << "the 20 ticks before the gap must not count";
  EXPECT_EQ(r.planner.bunched_ticks(), 0u);
}

TEST(PlannerStart, MissingScanOdomAndTrackHoldAndAreReported) {
  {
    Rig r;
    r.scan = false;
    double t = 1000.0;
    const TickOutput out = run_ticks(r, &t, 80);
    EXPECT_EQ(out.state, PlannerState::NoScan);
    EXPECT_EQ(out.cmd.speed_mps, 0.0);
  }
  {
    Rig r;
    r.odom = false;
    double t = 1000.0;
    const TickOutput out = run_ticks(r, &t, 80);
    EXPECT_EQ(out.state, PlannerState::NoOdom);
    EXPECT_EQ(out.cmd.speed_mps, 0.0);
  }
  {
    // a track that arrived once and is stale: STARTING holds (the track age is not fresh)
    Rig r;
    double t = 1000.0;
    TickOutput out = run_ticks(r, &t, 10);
    ASSERT_EQ(out.state, PlannerState::Starting);
    const LocalTrackData old_track = good_track(t - 0.5);
    for (int i = 0; i < 80; ++i) {
      TickInput in = r.input(t);
      in.track = &old_track;
      out = r.planner.tick(in);
      t += 0.02;
    }
    EXPECT_EQ(out.state, PlannerState::Starting);
    EXPECT_EQ(out.start_missing, StartMissing::Track);
    EXPECT_EQ(out.cmd.speed_mps, 0.0);
  }
}

TEST(PlannerStart, WithoutTheGatedDriveItHoldsAndSaysSo) {
  PlannerConfig cfg = small_config(0.5);
  cfg.logic.start_timeout_s = 1.0;
  Rig r(cfg);
  r.gated = false;
  double t = 1000.0;
  TickOutput out;
  bool timed_out = false;
  for (int i = 0; i < 200; ++i) {
    out = r.tick(t);
    t += 0.02;
    EXPECT_EQ(out.cmd.speed_mps, 0.0);
    timed_out = timed_out || out.start_timed_out;
  }
  EXPECT_EQ(out.state, PlannerState::Starting);
  EXPECT_EQ(out.start_missing, StartMissing::Gated);
  EXPECT_TRUE(timed_out);
  EXPECT_GT(out.starting_s, 3.0);
  // safety_node shows up: it leaves
  r.gated = true;
  for (int i = 0; i < 20; ++i) {
    out = r.tick(t);
    t += 0.02;
  }
  EXPECT_EQ(out.state, PlannerState::Driving);
}

TEST(PlannerStart, TheGatedDriveIsNotRequiredWhenTheParameterSaysSo) {
  PlannerConfig cfg = small_config(0.5);
  cfg.logic.require_gated_for_start = false;
  Rig r(cfg);
  r.gated = false;
  double t = 1000.0;
  const TickOutput out = run_ticks(r, &t, 60);
  EXPECT_EQ(out.state, PlannerState::Driving);
}

TEST(PlannerStart, NeverTimesOutIntoMoving) {
  PlannerConfig cfg = small_config(0.5);
  cfg.logic.start_timeout_s = 0.5;
  Rig r(cfg);
  r.gated = false;
  double t = 1000.0;
  for (int i = 0; i < 500; ++i) {
    const TickOutput out = r.tick(t);
    t += 0.02;
    ASSERT_EQ(out.cmd.speed_mps, 0.0) << i;
  }
}

TEST(PlannerStart, TheFirstCommandSteersFromTheGatedSteeringNotFromZero) {
  Rig r;
  r.gated_steering = 0.40;  // where safety_node's output still sits from the last run
  double t = 1000.0;
  const TickOutput first = r.tick(t);
  EXPECT_EQ(first.state, PlannerState::Starting);
  EXPECT_EQ(first.cmd.speed_mps, 0.0);
  const double step = 0.45 * r.g.steering_max_rate_rad_per_s * 0.02;
  // within one step of the gated value, so safety's rate limit has nothing to clip
  EXPECT_LE(std::fabs(first.cmd.steering_rad - 0.40), step + 1e-9);
  // and it slews toward the solver's target (about straight on a straight corridor)
  double prev = first.cmd.steering_rad;
  for (int i = 0; i < 40; ++i) {
    t += 0.02;
    const TickOutput out = r.tick(t);
    EXPECT_LE(std::fabs(out.cmd.steering_rad - prev), step + 1e-9);
    prev = out.cmd.steering_rad;
  }
  EXPECT_LT(std::fabs(prev), 0.40);
}

TEST(PlannerStart, WithoutTheGatedDriveNothingIsPublishedForTheGracePeriodThenSpeedZero) {
  PlannerConfig cfg = small_config(0.5);
  cfg.logic.start_gated_grace_s = 0.5;
  Rig r(cfg);
  r.gated = false;
  double t = 1000.0;
  int suppressed = 0;
  for (int i = 0; i < 40; ++i, t += 0.02) {
    const TickOutput out = r.tick(t);
    EXPECT_EQ(out.cmd.speed_mps, 0.0);
    if (out.drive_suppressed) {
      ++suppressed;
      EXPECT_LT(i, 26) << "still silent after the grace period";
    } else {
      EXPECT_GE(i, 25) << "published before the grace period was over";
    }
  }
  EXPECT_GE(suppressed, 25);
  EXPECT_LE(suppressed, 26);
}

TEST(PlannerStart, TheGatedDriveSeenDuringTheGraceSeedsTheSteeringAndEndsTheSilence) {
  Rig r;
  r.gated = false;
  double t = 1000.0;
  for (int i = 0; i < 5; ++i, t += 0.02) EXPECT_TRUE(r.tick(t).drive_suppressed);
  r.gated = true;
  r.gated_steering = 0.30;
  const TickOutput out = r.tick(t);
  EXPECT_FALSE(out.drive_suppressed);
  EXPECT_LE(std::fabs(out.cmd.steering_rad - 0.30),
            0.45 * r.g.steering_max_rate_rad_per_s * 0.02 + 1e-9);
}

TEST(PlannerStart, AStateOtherThanStartingStartsFromTheGatedSteeringToo) {
  // NO_ODOM at the start (the wheel speed has not arrived yet) is published with the steering the
  // gated /drive holds, not with 0.
  Rig r;
  r.odom = false;
  r.gated_steering = 0.30;
  const TickOutput out = r.tick(1000.0);
  EXPECT_EQ(out.state, PlannerState::NoOdom);
  EXPECT_FALSE(out.drive_suppressed);
  EXPECT_NEAR(out.cmd.steering_rad, 0.30, 1e-9);
}

TEST(PlannerStart, WithNoGatedDriveTheSteeringStartsFromZero) {
  PlannerConfig cfg = small_config(0.5);
  cfg.logic.require_gated_for_start = false;
  Rig r(cfg);
  r.gated = false;
  const TickOutput first = r.tick(1000.0);
  EXPECT_NEAR(first.cmd.steering_rad, 0.0, 0.45 * r.g.steering_max_rate_rad_per_s * 0.02 + 1e-9);
}

TEST(PlannerStart, AReturnToDrivingAfterALongStopGoesThroughStartingAgain) {
  PlannerConfig cfg = small_config(0.5);
  cfg.logic.start_ticks = 5;
  Rig r(cfg);
  double t = 1000.0;
  TickOutput out = run_ticks(r, &t, 60);
  ASSERT_EQ(out.state, PlannerState::Driving);
  ASSERT_GT(out.cmd.speed_mps, 0.0);
  // a wall right in front: Blocked, speed 0, for longer than the hold
  std::vector<PointXY> wall;
  for (double y = -1.0; y <= 1.0; y += 0.02) wall.push_back({0.1, y});
  r.set_points(wall);
  r.v = 0.0;
  out = run_ticks(r, &t, 80);
  EXPECT_EQ(out.state, PlannerState::Blocked);
  EXPECT_EQ(out.cmd.speed_mps, 0.0);
  // the wall goes away: the first Driving snapshot is held as STARTING once, then drives
  r.set_points(corridor_points());
  bool saw_starting = false;
  int ticks = 0;
  while (ticks++ < 100) {
    out = r.tick(t);
    t += 0.02;
    if (out.state == PlannerState::Starting) {
      saw_starting = true;
      EXPECT_EQ(out.cmd.speed_mps, 0.0);
    }
    if (out.state == PlannerState::Driving && out.cmd.speed_mps > 0.0) break;
  }
  EXPECT_TRUE(saw_starting);
  EXPECT_EQ(out.state, PlannerState::Driving);
}

TEST(PlannerStart, AShortStopDoesNotRestartTheGate) {
  PlannerConfig cfg = small_config(0.5);
  cfg.logic.start_ticks = 5;
  Rig r(cfg);
  double t = 1000.0;
  run_ticks(r, &t, 60);
  std::vector<PointXY> wall;
  for (double y = -1.0; y <= 1.0; y += 0.02) wall.push_back({0.1, y});
  r.set_points(wall);
  r.v = 0.0;
  TickOutput out = run_ticks(r, &t, 15);  // 0.3 s
  EXPECT_EQ(out.state, PlannerState::Blocked);
  r.set_points(corridor_points());
  for (int i = 0; i < 30; ++i) {
    out = r.tick(t);
    t += 0.02;
    EXPECT_NE(out.state, PlannerState::Starting) << i;
  }
}

TEST(PlannerStart, StartingSolvesOnTheRealInputsSoItsLatencyIsPaidBeforeTheCarMoves) {
  Rig r;
  double t = 1000.0;
  TickOutput out;
  double max_solve = 0.0;
  for (int i = 0; i < 20; ++i) {
    out = r.tick(t);
    t += 0.02;
    max_solve = std::max(max_solve, out.solve_time_s);
    EXPECT_EQ(out.state, PlannerState::Starting);
    EXPECT_GT(out.solve_time_s, 0.0) << "tick " << i << ": the solver did not run";
    EXPECT_GT(out.samples, 0);
  }
}

// ---------------------------------------------------------------------------------------------
// Bunched ticks, elapsed time slew, publishing through a late solve

TEST(PlannerTiming, ACallbackJustAfterALateOneRepublishesTheLastCommandAndDoesNotSolve) {
  Rig r;
  double t = 1000.0;
  TickOutput out = run_ticks(r, &t, 80);
  ASSERT_EQ(out.state, PlannerState::Driving);
  const TickOutput last = out;
  const double last_t = t - 0.02;
  const TickOutput bunched = r.tick(last_t + 0.004);
  EXPECT_TRUE(bunched.bunched);
  EXPECT_EQ(bunched.cmd.speed_mps, last.cmd.speed_mps);
  EXPECT_EQ(bunched.cmd.steering_rad, last.cmd.steering_rad);
  EXPECT_EQ(bunched.state, PlannerState::Driving);
  EXPECT_GE(r.planner.bunched_ticks(), 2u);  // the solve and the publish both counted
  // the next real tick is decided against the last real publish (it is not bunched: 20 ms) but
  // its slew allowance runs from the last message of any kind, the republish 4 ms after
  const TickOutput next = r.tick(last_t + 0.02);
  EXPECT_FALSE(next.bunched);
  EXPECT_NEAR(next.elapsed_s, 0.016, 1e-9);
  // just under and at half a tick
  const TickOutput under = r.tick(last_t + 0.02 + 0.0099);
  EXPECT_TRUE(under.bunched);
  const TickOutput at_half = r.tick(last_t + 0.02 + 0.0101);
  EXPECT_FALSE(at_half.bunched);
  EXPECT_NEAR(at_half.elapsed_s, 0.0002, 1e-9);  // from the republish at +0.0099
}

TEST(PlannerTiming, SteeringAndSpeedStepsFollowTheElapsedTimeNotTheNominalTick) {
  PlannerConfig cfg = small_config(0.5);
  cfg.logic.steer_output_tau_s = 0.0;
  cfg.logic.steer_output_hysteresis_rad = 0.0;
  Rig r(cfg);
  double t = 1000.0;
  run_ticks(r, &t, 80);
  // a track that wants a sharp turn: the steering requests more than any limit allows
  double prev_t = t - 0.02;
  TickOutput prev = r.tick(prev_t + 0.02);
  prev_t += 0.02;
  const double rate = 0.45 * r.g.steering_max_rate_rad_per_s;
  for (double gap : {0.012, 0.011, 0.02, 0.015, 0.03, 0.013}) {
    TickInput in = r.input(prev_t + gap);
    const LocalTrackData bent = good_track(prev_t + gap - 0.1, 0.4);
    in.track = &bent;
    const TickOutput out = r.planner.tick(in);
    EXPECT_LE(std::fabs(out.cmd.steering_rad - prev.cmd.steering_rad), rate * gap + 1e-9)
        << "gap " << gap;
    EXPECT_LE(out.cmd.speed_mps - prev.cmd.speed_mps,
              0.45 * r.g.max_acceleration_mps2 * gap + 1e-9);
    prev = out;
    prev_t += gap;
  }
}

TEST(PlannerTiming, AGapOverFiftyMillisecondsRestartsTheRampFromZeroInOneTicksStep) {
  Rig r;
  double t = 1000.0;
  TickOutput out = run_ticks(r, &t, 100);
  ASSERT_EQ(out.state, PlannerState::Driving);
  ASSERT_GT(out.cmd.speed_mps, 0.2);
  t += 0.4;  // a stall
  out = r.tick(t);
  EXPECT_TRUE(out.gap_reset);
  const double up = 0.45 * r.g.max_acceleration_mps2 * 0.02;
  EXPECT_LE(out.cmd.speed_mps, up + 1e-9) << "no jump from 0 to the old speed after a stall";
}

TEST(PlannerTiming, TheFirstPublishUsesOneTick) {
  Rig r;
  const TickOutput out = r.tick(1000.0);
  EXPECT_NEAR(out.elapsed_s, 0.02, 1e-12);
}

TEST(PlannerPublish, PublishesFromTheLatestSnapshotAtItsOwnRateWithoutSolving) {
  PlannerConfig cfg = small_config(0.5);
  cfg.logic.require_gated_for_start = false;
  cfg.logic.start_ticks = 5;
  Rig r(cfg);
  double t = 1000.0;
  run_ticks(r, &t, 40);
  // one solve, then publishes at 20 ms: the plan is indexed by the time since it was made
  const double plan_t = t;
  const SolveReport rep = r.planner.solve(r.input(plan_t));
  ASSERT_FALSE(rep.bunched);
  ASSERT_EQ(rep.state, PlannerState::Driving);
  TickOutput prev;
  for (int i = 1; i <= 5; ++i) {
    const TickOutput out = r.planner.publish(r.pub_input(plan_t + 0.02 * i));
    EXPECT_FALSE(out.plan_stale);
    EXPECT_EQ(out.state, PlannerState::Driving);
    EXPECT_NEAR(out.plan_age_s, 0.02 * i, 1e-9);
    EXPECT_EQ(out.solve_time_s, rep.solve_time_s);  // the snapshot's number, every publish
    EXPECT_GT(out.cmd.speed_mps, 0.0);
    prev = out;
  }
}

TEST(PlannerPublish, ASnapshotOlderThanThePlanTimeoutStopsTheCarAndAFreshOneRestartsFromZero) {
  PlannerConfig cfg = small_config(0.5);
  cfg.logic.require_gated_for_start = false;
  cfg.logic.start_ticks = 5;
  Rig r(cfg);
  double t = 1000.0;
  TickOutput out = run_ticks(r, &t, 80);
  ASSERT_EQ(out.state, PlannerState::Driving);
  ASSERT_GT(out.cmd.speed_mps, 0.2);
  const double plan_t = t - 0.02;  // the snapshot of the last tick
  const double steer = out.cmd.steering_rad;
  // publishes continue every 20 ms with no solve
  bool stale_seen = false;
  double last_speed = out.cmd.speed_mps;
  for (int i = 1; i <= 12; ++i) {
    const double now = plan_t + 0.02 * i;
    out = r.planner.publish(r.pub_input(now));
    if (i == 6) continue;  // exactly the timeout: floating point decides, no claim
    if (i > 6) {
      EXPECT_TRUE(out.plan_stale) << i;
      EXPECT_EQ(out.state, PlannerState::Fault);
      EXPECT_EQ(out.cmd.speed_mps, 0.0);
      EXPECT_NEAR(out.cmd.steering_rad, steer, 0.1);  // the last steering, not 0
      stale_seen = true;
    } else {
      EXPECT_FALSE(out.plan_stale) << i;
    }
    last_speed = out.cmd.speed_mps;
  }
  EXPECT_TRUE(stale_seen);
  EXPECT_EQ(last_speed, 0.0);
  // a fresh solve: the ramp restarts from 0, one step
  const double now = plan_t + 0.02 * 13;
  const SolveReport rep = r.planner.solve(r.input(now));
  EXPECT_EQ(rep.state, PlannerState::Driving);
  out = r.planner.publish(r.pub_input(now));
  EXPECT_EQ(out.state, PlannerState::Driving);
  EXPECT_LE(out.cmd.speed_mps, 0.45 * r.g.max_acceleration_mps2 * 0.02 + 1e-9);
}

TEST(PlannerPublish, WithNoSnapshotYetItHoldsSpeedZeroAsStarting) {
  Rig r;
  for (int i = 0; i < 10; ++i) {
    const TickOutput out = r.planner.publish(r.pub_input(1000.0 + 0.02 * i));
    EXPECT_EQ(out.state, PlannerState::Starting);
    EXPECT_EQ(out.cmd.speed_mps, 0.0);
  }
}

TEST(PlannerPublish, TheStaleSnapshotRuleOnlyAppliesToMovingStates) {
  PlannerConfig cfg = small_config(0.5);
  Rig r(cfg);
  r.odom = false;  // NoOdom snapshot
  const SolveReport rep = r.planner.solve(r.input(1000.0));
  ASSERT_EQ(rep.state, PlannerState::NoOdom);
  const TickOutput out = r.planner.publish(r.pub_input(1001.0));  // a second old
  EXPECT_FALSE(out.plan_stale);
  EXPECT_EQ(out.state, PlannerState::NoOdom);
  EXPECT_EQ(out.cmd.speed_mps, 0.0);
}

TEST(PlannerPublish, ABunchedSolveCallbackIsSkippedAndCounted) {
  Rig r;
  const SolveReport a = r.planner.solve(r.input(1000.0));
  EXPECT_FALSE(a.bunched);
  const SolveReport b = r.planner.solve(r.input(1000.004));
  EXPECT_TRUE(b.bunched);
  const SolveReport c = r.planner.solve(r.input(1000.02));
  EXPECT_FALSE(c.bunched);
  EXPECT_EQ(r.planner.bunched_ticks(), 1u);
}

TEST(PlannerPublish, ATimeThatGoesBackwardsRestartsTheClockInsteadOfFreezingTheOutput) {
  PlannerConfig cfg = small_config(0.5);
  cfg.logic.require_gated_for_start = false;
  cfg.logic.start_ticks = 3;
  Rig r(cfg);
  double t = 1000.0;
  run_ticks(r, &t, 60);
  t -= 500.0;  // the clock jumped back
  TickOutput out;
  for (int i = 0; i < 10; ++i) {
    out = r.tick(t);
    t += 0.02;
    EXPECT_FALSE(out.bunched) << i;
  }
  EXPECT_TRUE(std::isfinite(out.cmd.speed_mps));
}

// ---------------------------------------------------------------------------------------------
// Graceful staleness through the planner

TEST(PlannerStaleness, AScanBetweenTheTwoBoundsKeepsDrivingWithARampedCap) {
  PlannerConfig cfg = small_config(0.5);
  cfg.logic.require_gated_for_start = false;
  cfg.logic.start_ticks = 3;
  Rig r(cfg);
  double t = 1000.0;
  TickOutput out = run_ticks(r, &t, 60);
  ASSERT_EQ(out.state, PlannerState::Driving);
  const double full = out.v_cap_mps;
  // the same scan, now 0.475 s old: cap halves, still driving
  TickInput in = r.input(t);
  in.scan_stamp_s = t - 0.475;
  out = r.planner.tick(in);
  EXPECT_EQ(out.state, PlannerState::Driving);
  EXPECT_NEAR(out.v_cap_mps, 0.5 * full, 0.01);
  // beyond 0.60 s: NO_SCAN
  t += 0.02;
  in = r.input(t);
  in.scan_stamp_s = t - 0.65;
  out = r.planner.tick(in);
  EXPECT_EQ(out.state, PlannerState::NoScan);
  EXPECT_EQ(out.cmd.speed_mps, 0.0);
}

TEST(PlannerStaleness, AWheelSpeedBetweenTheTwoBoundsKeepsDrivingWithoutAcceleratingOnAGuess) {
  PlannerConfig cfg = small_config(0.9);
  cfg.logic.require_gated_for_start = false;
  cfg.logic.start_ticks = 3;
  Rig r(cfg);
  double t = 1000.0;
  TickOutput out = run_ticks(r, &t, 60);
  ASSERT_EQ(out.state, PlannerState::Driving);
  r.v = 0.2;
  for (int i = 0; i < 5; ++i) {
    TickInput in = r.input(t);
    in.odom_stamp_s = t - 0.15;  // stale, not yet gone
    out = r.planner.tick(in);
    t += 0.02;
    EXPECT_EQ(out.state, PlannerState::Driving) << i;
    EXPECT_LE(out.v_cap_mps, 0.2 + 0.05 + 1e-9);
    EXPECT_LE(out.cmd.speed_mps, out.v_cap_mps + 1e-9);
  }
  TickInput in = r.input(t);
  in.odom_stamp_s = t - 0.25;
  out = r.planner.tick(in);
  EXPECT_EQ(out.state, PlannerState::NoOdom);
  EXPECT_EQ(out.cmd.speed_mps, 0.0);
}

TEST(PlannerStaleness, GarbageStampsAreStaleAtOnce) {
  PlannerConfig cfg = small_config(0.5);
  cfg.logic.require_gated_for_start = false;
  cfg.logic.start_ticks = 3;
  Rig r(cfg);
  double t = 1000.0;
  run_ticks(r, &t, 60);
  for (double bad : {kNaN, kInf, -kInf}) {
    TickInput in = r.input(t);
    in.scan_stamp_s = bad;
    EXPECT_EQ(r.planner.tick(in).state, PlannerState::NoScan);
    t += 0.02;
    in = r.input(t);
    in.odom_stamp_s = bad;
    EXPECT_EQ(r.planner.tick(in).state, PlannerState::NoOdom);
    t += 0.02;
  }
  TickInput in = r.input(t);
  in.scan_stamp_s = t + 10.0;  // from the future
  EXPECT_EQ(r.planner.tick(in).state, PlannerState::NoScan);
}

TEST(PlannerGuidance, ADeadLapMemoryNodeGivesTheFallbackCapNotTheCeiling) {
  PlannerConfig cfg = small_config(1.0);
  cfg.mppi.samples = 128;
  cfg.logic.use_lap_guidance = true;
  cfg.logic.v_no_guidance_mps = 0.4;
  cfg.logic.require_gated_for_start = false;
  cfg.logic.start_ticks = 3;
  Rig r(cfg);
  r.planner.set_v_max(1.0);
  double t = 1000.0;
  TickOutput out = run_ticks(r, &t, 60);
  EXPECT_LE(out.v_cap_mps, 0.4 + 1e-12);
  EXPECT_GT(out.v_cap_mps, 0.2);
  // fresh guidance above the fallback is honoured, up to the ceiling
  TickInput in = r.input(t);
  in.has_guidance = true;
  in.guidance_stamp_s = t;
  in.guidance_v_cap_mps = 0.8;
  out = r.planner.tick(in);
  EXPECT_NEAR(out.v_cap_mps, 0.8, 1e-9);
  // a bad profile flag drops back
  t += 0.02;
  in = r.input(t);
  in.has_guidance = true;
  in.guidance_stamp_s = t;
  in.guidance_v_cap_mps = 0.8;
  in.guidance_profile_bad = true;
  out = r.planner.tick(in);
  EXPECT_NEAR(out.v_cap_mps, 0.4, 1e-9);
}
