// Tests of the rate credit of racer_safety::SafetyGateLogic (brief 49, gate_logic.hpp "RATE
// CREDIT"): table-driven unit tests for each clause of the rule, and seeded property tests of the
// guarantees G1, G2 and G3 and of what the rule fixes, over adversarial cycle-time sequences.
//
// Same fixture style as test_gate_logic.cpp: plain limits in the shape of
// config/vehicle_params.yaml, not read from it (gate_logic.* never include the generated binding).
// No ROS.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "racer_safety/gate_logic.hpp"

namespace racer_safety {
namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kT = 0.02;  // the nominal period, SafetyLimits::control_period_s

SafetyLimits make_limits() {
  SafetyLimits limits;
  limits.steering_min_rad = -0.4189;
  limits.steering_max_rad = 0.4189;
  limits.steering_rate_min_rad_per_s = -3.2;
  limits.steering_rate_max_rad_per_s = 3.2;
  limits.speed_min_mps = -5.0;
  limits.speed_max_mps = 20.0;
  limits.max_acceleration_mps2 = 9.51;
  limits.watchdog_missed_cycles = 3;
  limits.control_period_s = kT;
  return limits;
}

bool rate_limited(const GateResult& result) {
  for (const GateActivation& a : result.activations) {
    if (a.source == GateSource::kRateLimit) {
      return true;
    }
  }
  return false;
}

// One limited channel, so every case of the rule runs on steering and on speed. The steering
// channel starts from a steering of 0 and a speed of 0, the speed channel from a speed of 1 m/s.
struct Channel {
  const char* name;
  bool speed;

  double rate(const SafetyLimits& l) const {
    return speed ? l.max_acceleration_mps2 : l.steering_rate_max_rad_per_s;
  }
  double start() const { return speed ? 1.0 : 0.0; }
  DriveCommand command(double value) const {
    return speed ? DriveCommand{0.0, value} : DriveCommand{value, 0.0};
  }
  double value(const DriveCommand& c) const { return speed ? c.speed_mps : c.steering_angle_rad; }
  double credit_in(const GateInput& i) const {
    return speed ? i.speed_rate_credit_s : i.steering_rate_credit_s;
  }
  double credit_out(const GateResult& r) const {
    return speed ? r.speed_rate_credit_s : r.steering_rate_credit_s;
  }
  double other_credit_out(const GateResult& r) const {
    return speed ? r.steering_rate_credit_s : r.speed_rate_credit_s;
  }
};

const Channel kChannels[] = {{"steering", false}, {"speed", true}};

GateInput make_input(const Channel& ch, double requested, double dt_s, double credit_s) {
  GateInput input;
  input.command = ch.command(requested);
  input.drive_raw_age_s = 0.0;
  input.dt_s = dt_s;
  input.min_scan_range_m = kInf;
  if (ch.speed) {
    input.speed_rate_credit_s = credit_s;
  } else {
    input.steering_rate_credit_s = credit_s;
  }
  return input;
}

GateResult run(const Channel& ch, const SafetyLimits& limits, double requested, double dt_s,
               double credit_s) {
  return SafetyGateLogic(limits).evaluate(make_input(ch, requested, dt_s, credit_s),
                                          ch.command(ch.start()));
}

// ---------------------------------------------------------------------------------------
// 1. A nominal cycle with no credit: inside the limit it passes, and the time it did not use is
//    carried (at most T).
// ---------------------------------------------------------------------------------------
TEST(RateCredit, NominalCycleInsideTheLimitPassesAndCarriesTheUnusedTime) {
  const SafetyLimits limits = make_limits();
  for (const Channel& ch : kChannels) {
    SCOPED_TRACE(ch.name);
    for (const double share : {0.0, 0.25, 0.5, 0.9, 1.0}) {
      const double step = share * ch.rate(limits) * kT;
      const GateResult r = run(ch, limits, ch.start() + step, kT, 0.0);
      EXPECT_FALSE(rate_limited(r)) << "share " << share;
      EXPECT_NEAR(ch.value(r.output), ch.start() + step, 1e-12);
      EXPECT_NEAR(ch.credit_out(r), kT * (1.0 - share), 1e-12) << "share " << share;
      EXPECT_LE(ch.credit_out(r), kT);
      EXPECT_GE(ch.credit_out(r), 0.0);
    }
    // A step of exactly nothing keeps the whole period.
    EXPECT_DOUBLE_EQ(ch.credit_out(run(ch, limits, ch.start(), kT, 0.0)), kT);
  }
}

// ---------------------------------------------------------------------------------------
// 2. A stalled cycle: a long dt with a small step passes and the credit goes out full.
// ---------------------------------------------------------------------------------------
TEST(RateCredit, StalledCycleWithASmallStepPassesAndFillsTheCredit) {
  const SafetyLimits limits = make_limits();
  for (const Channel& ch : kChannels) {
    SCOPED_TRACE(ch.name);
    const double step = 0.1 * ch.rate(limits) * kT;
    const GateResult r = run(ch, limits, ch.start() + step, 2.6 * kT, 0.0);
    EXPECT_FALSE(rate_limited(r));
    EXPECT_NEAR(ch.value(r.output), ch.start() + step, 1e-12);
    EXPECT_DOUBLE_EQ(ch.credit_out(r), kT);
  }
}

// ---------------------------------------------------------------------------------------
// 3. The catch-up cycle: a few ms after a late cycle, a planner step of 0.45 of the limit per
//    tick passes with a full credit. Without credit it is clipped to rate * dt, main's rule.
// ---------------------------------------------------------------------------------------
TEST(RateCredit, CatchUpCycleIsNotClippedWithACreditAndIsWithoutOne) {
  const SafetyLimits limits = make_limits();
  for (const Channel& ch : kChannels) {
    SCOPED_TRACE(ch.name);
    const double step = 0.45 * ch.rate(limits) * kT;
    for (const double dt : {0.001, 0.004, 0.008, 0.012}) {
      SCOPED_TRACE(dt);
      const GateResult with_credit = run(ch, limits, ch.start() + step, dt, kT);
      EXPECT_FALSE(rate_limited(with_credit));
      EXPECT_NEAR(ch.value(with_credit.output), ch.start() + step, 1e-12);

      const GateResult without = run(ch, limits, ch.start() + step, dt, 0.0);
      if (step > ch.rate(limits) * dt) {  // dt below 0.45 T: main's clip
        EXPECT_TRUE(rate_limited(without));
        EXPECT_NEAR(ch.value(without.output), ch.start() + ch.rate(limits) * dt, 1e-12);
      } else {  // 12 ms is above 0.45 T: nothing to clip, with or without credit
        EXPECT_FALSE(rate_limited(without));
      }
    }
  }
}

// ---------------------------------------------------------------------------------------
// 4. The per-cycle cap: however much credit there is, one cycle moves at most rate * T when dt is
//    below T, and rate * dt (no more) when dt is T or more... plus nothing from the credit.
// ---------------------------------------------------------------------------------------
TEST(RateCredit, PerCycleCapIsOneNominalPeriod) {
  const SafetyLimits limits = make_limits();
  const double far_up_steering = 0.4;
  const double far_up_speed = 15.0;
  for (const Channel& ch : kChannels) {
    SCOPED_TRACE(ch.name);
    const double far = ch.speed ? far_up_speed : far_up_steering;
    for (const double dt : {kT, 0.001}) {
      SCOPED_TRACE(dt);
      const GateResult r = run(ch, limits, far, dt, kT);
      EXPECT_TRUE(rate_limited(r));
      EXPECT_NEAR(ch.value(r.output), ch.start() + ch.rate(limits) * kT, 1e-12);
    }
    // A late cycle (dt above T) is allowed its own dt, and the credit adds nothing to it.
    const GateResult late = run(ch, limits, far, 2.0 * kT, kT);
    EXPECT_NEAR(ch.value(late.output), ch.start() + ch.rate(limits) * 2.0 * kT, 1e-12);
  }
}

// ---------------------------------------------------------------------------------------
// 5. Catch-up cycles back to back: a credit of T and then three cycles 1 ms apart move at most
//    rate * (T + 0.003) in all.
// ---------------------------------------------------------------------------------------
TEST(RateCredit, BackToBackCatchUpCyclesAreBoundedByTheirTotalTimePlusOnePeriod) {
  const SafetyLimits limits = make_limits();
  for (const Channel& ch : kChannels) {
    SCOPED_TRACE(ch.name);
    const double far = ch.speed ? 15.0 : 0.4;
    const SafetyGateLogic gate(limits);
    DriveCommand previous = ch.command(ch.start());
    double credit = kT;
    for (int cycle = 0; cycle < 3; ++cycle) {
      GateInput input = make_input(ch, far, 0.001, 0.0);
      (ch.speed ? input.speed_rate_credit_s : input.steering_rate_credit_s) = credit;
      const GateResult r = gate.evaluate(input, previous);
      previous = r.output;
      credit = ch.credit_out(r);
    }
    const double moved = ch.value(previous) - ch.start();
    EXPECT_LE(moved, ch.rate(limits) * (kT + 0.003) + 1e-12);
    EXPECT_NEAR(moved, ch.rate(limits) * (kT + 0.003), 1e-12);
    EXPECT_DOUBLE_EQ(credit, 0.0);
  }
}

// ---------------------------------------------------------------------------------------
// 6. Garbage: credit, dt and control_period_s.
// ---------------------------------------------------------------------------------------
TEST(RateCredit, GarbageCreditCountsAsZeroAndACreditAboveTCountsAsT) {
  const SafetyLimits limits = make_limits();
  for (const Channel& ch : kChannels) {
    SCOPED_TRACE(ch.name);
    const double step = 0.45 * ch.rate(limits) * kT;
    const GateResult zero = run(ch, limits, ch.start() + step, 0.004, 0.0);
    for (const double garbage : {kNan, kInf, -kInf, -0.5, -1e-9}) {
      SCOPED_TRACE(garbage);
      const GateResult r = run(ch, limits, ch.start() + step, 0.004, garbage);
      EXPECT_DOUBLE_EQ(ch.value(r.output), ch.value(zero.output));
      EXPECT_DOUBLE_EQ(ch.credit_out(r), ch.credit_out(zero));
      EXPECT_TRUE(rate_limited(r));
    }
    const GateResult full = run(ch, limits, ch.start() + step, 0.004, kT);
    for (const double above : {kT * (1.0 + 1e-9), 5.0 * kT, 1e9}) {
      SCOPED_TRACE(above);
      const GateResult r = run(ch, limits, ch.start() + step, 0.004, above);
      EXPECT_DOUBLE_EQ(ch.value(r.output), ch.value(full.output));
      EXPECT_DOUBLE_EQ(ch.credit_out(r), ch.credit_out(full));
    }
  }
}

TEST(RateCredit, GarbageDtAllowsNoChangeEvenWithACreditAndReturnsNoCredit) {
  const SafetyLimits limits = make_limits();
  for (const Channel& ch : kChannels) {
    SCOPED_TRACE(ch.name);
    const double far = ch.speed ? 15.0 : 0.4;
    for (const double dt : {kNan, kInf, -kInf, -0.01, -1e-12}) {
      SCOPED_TRACE(dt);
      GateInput input = make_input(ch, far, dt, kT);
      input.steering_rate_credit_s = kT;
      input.speed_rate_credit_s = kT;
      const GateResult r = SafetyGateLogic(limits).evaluate(input, ch.command(ch.start()));
      EXPECT_DOUBLE_EQ(ch.value(r.output), ch.start());
      EXPECT_DOUBLE_EQ(r.steering_rate_credit_s, 0.0);
      EXPECT_DOUBLE_EQ(r.speed_rate_credit_s, 0.0);
      EXPECT_TRUE(rate_limited(r));
    }
  }
}

TEST(RateCredit, ADtOfExactlyZeroGrantsTheCarriedCredit) {
  const SafetyLimits limits = make_limits();
  for (const Channel& ch : kChannels) {
    SCOPED_TRACE(ch.name);
    const double far = ch.speed ? 15.0 : 0.4;
    const GateResult r = run(ch, limits, far, 0.0, kT);
    EXPECT_NEAR(ch.value(r.output), ch.start() + ch.rate(limits) * kT, 1e-12);
    EXPECT_DOUBLE_EQ(ch.credit_out(r), 0.0);
    // And a half credit grants half.
    const GateResult half = run(ch, limits, far, 0.0, 0.5 * kT);
    EXPECT_NEAR(ch.value(half.output), ch.start() + ch.rate(limits) * 0.5 * kT, 1e-12);
    // No credit and no time: nothing.
    EXPECT_DOUBLE_EQ(ch.value(run(ch, limits, far, 0.0, 0.0).output), ch.start());
  }
}

TEST(RateCredit, ControlPeriodThatIsNotAPositiveNumberGivesTheOldRule) {
  for (const Channel& ch : kChannels) {
    SCOPED_TRACE(ch.name);
    const double far = ch.speed ? 15.0 : 0.4;
    // NaN and +inf leave the watchdog timeout unusable (never trips); a negative period with a
    // negative cycle count gives a positive timeout. A credit of T would have allowed rate * T.
    struct Case {
      double period_s;
      int missed_cycles;
    };
    for (const Case c : {Case{kNan, 3}, Case{kInf, 3}, Case{-kT, -3}, Case{-kInf, -3}}) {
      SCOPED_TRACE(c.period_s);
      SafetyLimits limits = make_limits();
      limits.control_period_s = c.period_s;
      limits.watchdog_missed_cycles = c.missed_cycles;
      const GateResult r = run(ch, limits, far, 0.001, kT);
      EXPECT_FALSE(r.zero_throttle);
      EXPECT_NEAR(ch.value(r.output), ch.start() + ch.rate(limits) * 0.001, 1e-12);
      EXPECT_GE(ch.credit_out(r), 0.0);
      EXPECT_LE(ch.credit_out(r), 0.0);  // T counts as 0
    }
    // A period of exactly 0 trips the watchdog on every cycle (timeout 0): the credit stays 0.
    SafetyLimits zero_period = make_limits();
    zero_period.control_period_s = 0.0;
    const GateResult r = run(ch, zero_period, far, 0.001, kT);
    EXPECT_TRUE(r.zero_throttle);
    EXPECT_DOUBLE_EQ(ch.credit_out(r), 0.0);
  }
}

// ---------------------------------------------------------------------------------------
// 7. Watchdog and non-finite command: the output does not move, the credit accrues to at most T.
// ---------------------------------------------------------------------------------------
TEST(RateCredit, ShortCircuitPathsHoldTheOutputAndAccrueCreditUpToT) {
  const SafetyLimits limits = make_limits();
  const DriveCommand previous{0.2, 1.5};
  struct Path {
    const char* name;
    GateInput input;
  };
  GateInput watchdog;
  watchdog.command = DriveCommand{0.3, 2.0};
  watchdog.drive_raw_age_s = 1.0;
  watchdog.min_scan_range_m = kInf;
  GateInput non_finite = watchdog;
  non_finite.drive_raw_age_s = 0.0;
  non_finite.command = DriveCommand{kNan, 2.0};
  GateInput non_finite_speed = non_finite;
  non_finite_speed.command = DriveCommand{0.3, kInf};
  for (const Path& path : {Path{"watchdog", watchdog}, Path{"non-finite steering", non_finite},
                           Path{"non-finite speed", non_finite_speed}}) {
    SCOPED_TRACE(path.name);
    GateInput input = path.input;
    input.dt_s = 0.01;
    input.steering_rate_credit_s = 0.005;
    input.speed_rate_credit_s = 0.0;
    GateResult r = SafetyGateLogic(limits).evaluate(input, previous);
    EXPECT_TRUE(r.zero_throttle);
    EXPECT_DOUBLE_EQ(r.output.steering_angle_rad, 0.2);  // held
    EXPECT_DOUBLE_EQ(r.output.speed_mps, 0.0);
    EXPECT_NEAR(r.steering_rate_credit_s, 0.015, 1e-15);
    EXPECT_NEAR(r.speed_rate_credit_s, 0.01, 1e-15);
    input.dt_s = 0.02;
    input.steering_rate_credit_s = r.steering_rate_credit_s;
    input.speed_rate_credit_s = r.speed_rate_credit_s;
    r = SafetyGateLogic(limits).evaluate(input, previous);
    EXPECT_DOUBLE_EQ(r.steering_rate_credit_s, kT);
    EXPECT_DOUBLE_EQ(r.speed_rate_credit_s, kT);
    // A garbage dt gives nothing back on these paths either.
    input.dt_s = kNan;
    r = SafetyGateLogic(limits).evaluate(input, previous);
    EXPECT_DOUBLE_EQ(r.steering_rate_credit_s, 0.0);
    EXPECT_DOUBLE_EQ(r.speed_rate_credit_s, 0.0);
  }
}

// ---------------------------------------------------------------------------------------
// 8. Asymmetric steering rates and a rate of 0: the rate of the direction moved is the one used,
//    and a zero rate neither moves nor divides by zero.
// ---------------------------------------------------------------------------------------
TEST(RateCredit, SteeringUsesTheRateOfTheDirectionMoved) {
  SafetyLimits limits = make_limits();
  limits.steering_rate_min_rad_per_s = -1.0;
  limits.steering_rate_max_rad_per_s = 3.0;
  const Channel& steering = kChannels[0];
  struct Case {
    double request;
    double expected_output;
    double expected_credit;
  };
  const Case cases[] = {
      {0.4, 3.0 * kT, 0.0},                        // far up: the upward rate, all of the period
      {-0.4, -1.0 * kT, 0.0},                      // far down: the downward rate, all of the period
      {0.5 * 3.0 * kT, 0.5 * 3.0 * kT, 0.5 * kT},  // half the upward limit
      {-0.5 * 1.0 * kT, -0.5 * 1.0 * kT, 0.5 * kT},  // half the downward limit
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.request);
    const GateResult r = run(steering, limits, c.request, kT, 0.0);
    EXPECT_NEAR(r.output.steering_angle_rad, c.expected_output, 1e-12);
    EXPECT_NEAR(r.steering_rate_credit_s, c.expected_credit, 1e-12);
  }
}

TEST(RateCredit, AZeroRateGrantsNothingUsesNothingAndNeverDividesByZero) {
  {
    SafetyLimits limits = make_limits();
    limits.steering_rate_max_rad_per_s = 0.0;
    limits.steering_rate_min_rad_per_s = 0.0;
    for (const double request : {0.4, -0.4}) {
      const GateResult r = run(kChannels[0], limits, request, kT, 0.01);
      EXPECT_DOUBLE_EQ(r.output.steering_angle_rad, 0.0);
      EXPECT_TRUE(std::isfinite(r.steering_rate_credit_s));
      EXPECT_DOUBLE_EQ(r.steering_rate_credit_s, kT);  // min(T, 0.01 + 0.02 - 0)
    }
    EXPECT_DOUBLE_EQ(run(kChannels[0], limits, 0.4, 0.005, 0.0).steering_rate_credit_s, 0.005);
  }
  {
    SafetyLimits limits = make_limits();
    limits.max_acceleration_mps2 = 0.0;
    const GateResult r = run(kChannels[1], limits, 15.0, kT, 0.0);
    EXPECT_DOUBLE_EQ(r.output.speed_mps, 1.0);
    EXPECT_TRUE(std::isfinite(r.speed_rate_credit_s));
    EXPECT_DOUBLE_EQ(r.speed_rate_credit_s, kT);
  }
}

// ---------------------------------------------------------------------------------------
// 9. Speed: only growth of the magnitude uses credit; braking is free, immediate and uses none;
//    the zero crossing is as before.
// ---------------------------------------------------------------------------------------
TEST(RateCredit, SpeedBrakingUsesNoCreditAndIsNeverDelayed) {
  const SafetyLimits limits = make_limits();
  const SafetyGateLogic gate(limits);
  struct Case {
    double previous_mps;
    double request_mps;
  };
  for (const Case c : {Case{5.0, 0.0}, Case{5.0, 2.0}, Case{-5.0, 0.0}, Case{-5.0, -1.0}}) {
    SCOPED_TRACE(c.request_mps);
    GateInput input;
    input.command = DriveCommand{0.0, c.request_mps};
    input.min_scan_range_m = kInf;
    input.dt_s = 0.001;
    const GateResult r = gate.evaluate(input, DriveCommand{0.0, c.previous_mps});
    EXPECT_DOUBLE_EQ(r.output.speed_mps, c.request_mps);
    EXPECT_FALSE(rate_limited(r));
    EXPECT_DOUBLE_EQ(r.speed_rate_credit_s, 0.001);  // nothing used
  }
}

TEST(RateCredit, SpeedGrowthInBothDirectionsAndAcrossZeroUsesCredit) {
  const SafetyLimits limits = make_limits();
  const SafetyGateLogic gate(limits);
  const double acc = limits.max_acceleration_mps2;
  struct Case {
    double previous_mps;
    double request_mps;
    double expected_mps;
    double expected_credit_s;
  };
  const Case cases[] = {
      {1.0, 1.0 + 0.5 * acc * kT, 1.0 + 0.5 * acc * kT, 0.5 * kT},     // forward growth, inside
      {-1.0, -1.0 - 0.5 * acc * kT, -1.0 - 0.5 * acc * kT, 0.5 * kT},  // reverse growth, inside
      {1.0, 15.0, 1.0 + acc * kT, 0.0},                                // forward, limited
      {-1.0, -4.0, -1.0 - acc * kT, 0.0},                              // reverse, limited
      {-1.0, 1.0, acc * kT, 0.0},   // across zero: free brake to 0, then growth from 0
      {1.0, -1.0, -acc * kT, 0.0},  // the same the other way
      {-1.0, 0.25 * acc * kT, 0.25 * acc * kT, 0.75 * kT},  // across zero, growth inside
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.request_mps);
    GateInput input;
    input.command = DriveCommand{0.0, c.request_mps};
    input.min_scan_range_m = kInf;
    input.dt_s = kT;
    const GateResult r = gate.evaluate(input, DriveCommand{0.0, c.previous_mps});
    EXPECT_NEAR(r.output.speed_mps, c.expected_mps, 1e-12);
    EXPECT_NEAR(r.speed_rate_credit_s, c.expected_credit_s, 1e-12);
  }
}

// ---------------------------------------------------------------------------------------
// 10. What runs after the limiter: the credit is charged for what the limiter granted.
// ---------------------------------------------------------------------------------------
TEST(RateCredit, CreditIsChargedForWhatTheLimiterGrantedWhenTheBoundsClampMovesLess) {
  const SafetyLimits limits = make_limits();
  const SafetyGateLogic gate(limits);
  {
    // Steering: asked for 0.9 from 0.40; the bounds clamp makes the request 0.4189, which the
    // limiter grants in full (0.0189 rad), so 0.0189 / 3.2 s is used.
    GateInput input;
    input.command = DriveCommand{0.9, 0.0};
    input.min_scan_range_m = kInf;
    input.dt_s = kT;
    const GateResult r = gate.evaluate(input, DriveCommand{0.40, 0.0});
    EXPECT_NEAR(r.output.steering_angle_rad, 0.4189, 1e-12);
    EXPECT_NEAR(r.steering_rate_credit_s, kT - (0.4189 - 0.40) / 3.2, 1e-12);
  }
  {
    // Speed: asked for 25 from 19.9; clamped to 20, granted 0.1 m/s.
    GateInput input;
    input.command = DriveCommand{0.0, 25.0};
    input.min_scan_range_m = kInf;
    input.dt_s = kT;
    const GateResult r = gate.evaluate(input, DriveCommand{0.0, 19.9});
    EXPECT_NEAR(r.output.speed_mps, 20.0, 1e-12);
    EXPECT_NEAR(r.speed_rate_credit_s, kT - 0.1 / 9.51, 1e-12);
  }
}

TEST(RateCredit, CreditIsChargedForWhatTheLimiterGrantedWhenTheSteeringHoldMovesLess) {
  SafetyLimits limits = make_limits();
  limits.ttc_brake_s = 1.0;
  limits.obstacle_steering_hold_after_s = 0.1;
  const SafetyGateLogic gate(limits);
  GateInput input;
  input.command = DriveCommand{0.4, 2.0};
  input.min_scan_range_m = 0.05;  // an obstacle: the latch holds the speed at zero
  input.ttc_brake_latched = true;
  input.obstacle_hold_timer_s = 0.5;  // the hold is engaged: the steering is frozen
  input.dt_s = kT;
  const GateResult r = gate.evaluate(input, DriveCommand{0.0, 0.0});
  EXPECT_DOUBLE_EQ(r.output.steering_angle_rad, 0.0);  // held
  EXPECT_DOUBLE_EQ(r.output.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(r.steering_rate_credit_s,
                   0.0);                        // charged the full 0.064 rad the limiter granted
  EXPECT_DOUBLE_EQ(r.speed_rate_credit_s, kT);  // the latch took the growth away first
}

// ---------------------------------------------------------------------------------------
// Clip detection: rate_limit is reported when the limiter clipped, not when previous + (target -
// previous) happens to differ from target by an ulp. The pairs below are ones where it does.
// ---------------------------------------------------------------------------------------
TEST(RateLimitDetection, AStepInsideTheLimitIsNeverReportedAsRateLimited) {
  const SafetyLimits limits = make_limits();
  struct Pair {
    double previous_rad;
    double requested_rad;
  };
  const Pair pairs[] = {{0.082482961902298813, 0.021424096807560297},
                        {-0.18827769076519774, -2.0816681711721685e-17},
                        {0.069600000000183293, 0.01200000000023568}};
  for (const Pair& p : pairs) {
    SCOPED_TRACE(p.requested_rad);
    // The reason for the test: the recomputed output is not bit-equal to the request.
    EXPECT_NE(p.previous_rad + (p.requested_rad - p.previous_rad), p.requested_rad);
    for (const double credit : {0.0, kT}) {
      GateInput input;
      input.command = DriveCommand{p.requested_rad, 0.0};
      input.min_scan_range_m = kInf;
      input.dt_s = 0.2;  // the step is far inside rate * dt
      input.steering_rate_credit_s = credit;
      const GateResult r =
          SafetyGateLogic(limits).evaluate(input, DriveCommand{p.previous_rad, 0.0});
      EXPECT_FALSE(rate_limited(r));
      EXPECT_NEAR(r.output.steering_angle_rad, p.requested_rad, 1e-15);
    }
  }
}

// ---------------------------------------------------------------------------------------
// Property tests (seeded, no hypothesis for C++). Fixed seeds; a failure prints the seed, the
// sequence index and the cycle index so it reproduces with one call.
// ---------------------------------------------------------------------------------------

constexpr int kSequences = 20000;
constexpr int kCycles = 200;
constexpr uint64_t kSeedBase = 0x49'0000'0000ULL;

SafetyLimits property_limits(int variant) {
  SafetyLimits limits = make_limits();
  limits.ttc_warning_s = 1.0;
  limits.ttc_brake_s = 0.6;
  limits.min_forward_clearance_m = 0.3;
  limits.obstacle_steering_hold_after_s = 0.1;
  limits.speed_bound_deceleration_mps2 = 3.0;
  if (variant % 3 == 1) {  // asymmetric steering rates, slower acceleration
    limits.steering_rate_min_rad_per_s = -2.0;
    limits.steering_rate_max_rad_per_s = 3.2;
    limits.max_acceleration_mps2 = 6.0;
  } else if (variant % 3 == 2) {  // a faster node
    limits.control_period_s = 0.01;
    limits.max_acceleration_mps2 = 12.0;
  }
  return limits;
}

std::string fmt17(double v) {
  char buffer[40];
  std::snprintf(buffer, sizeof(buffer), "%.17g", v);
  return buffer;
}

double positive_part(double x) { return x > 0.0 ? x : 0.0; }

// The growth of the speed's magnitude one cycle produced, the quantity the speed limiter bounds:
// movement away from zero, counted from zero when the speed crossed it.
double speed_growth(double previous_mps, double output_mps) {
  double base = previous_mps;
  if (previous_mps * output_mps < 0.0) {
    base = 0.0;
  }
  return positive_part(std::fabs(output_mps) - std::fabs(base));
}

double clamp_to(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

class Dts {
 public:
  explicit Dts(std::mt19937_64& rng) : rng_(rng) {}
  double uniform(double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(rng_);
  }
  int pick(int n) { return std::uniform_int_distribution<int>(0, n - 1)(rng_); }

  // Cycle times with clusters of tiny values and long stalls. `garbage` adds non-finite and
  // negative values.
  double next(bool garbage) {
    if (cluster_left_ > 0) {
      --cluster_left_;
      return uniform(1e-4, 0.012);
    }
    const int mode = pick(garbage ? 100 : 97);
    if (mode < 40) {
      return kT * uniform(0.97, 1.03);
    }
    if (mode < 55) {
      cluster_left_ = pick(4);
      return uniform(1e-4, 0.012);
    }
    if (mode < 65) {
      return uniform(0.04, 0.2);  // a stall
    }
    if (mode < 68) {
      return 0.0;
    }
    if (mode < 75) {
      // the late cycle and its catch-up: 52 ms then 8 ms, in the manner of the node
      cluster_left_ = 0;
      return pick(2) == 0 ? 0.052 : 0.008;
    }
    if (mode < 97) {
      return uniform(1e-4, 0.2);
    }
    const double garbage_values[] = {kNan, kInf, -kInf, -uniform(1e-9, 0.1)};
    return garbage_values[pick(4)];
  }

 private:
  std::mt19937_64& rng_;
  int cluster_left_ = 0;
};

class PropertyReport {
 public:
  bool failed() const { return failures_ >= 1; }
  void fail(uint64_t seed, int sequence, int cycle, const std::string& what) {
    ++failures_;
    if (failures_ <= 5) {
      ADD_FAILURE() << "seed=" << seed << " sequence=" << sequence << " cycle=" << cycle << ": "
                    << what;
    }
  }

 private:
  int failures_ = 0;
};

// P1 to P4 over adversarial sequences.
TEST(RateCreditProperties, G1G2G3AndTheCreditInvariantsHoldOnAdversarialSequences) {
  PropertyReport report;
  for (int sequence = 0; sequence < kSequences && !report.failed(); ++sequence) {
    const uint64_t seed = kSeedBase + static_cast<uint64_t>(sequence);
    std::mt19937_64 rng(seed);
    Dts dts(rng);
    const SafetyLimits limits = property_limits(sequence);
    const SafetyGateLogic gate(limits);
    const double period = limits.control_period_s;
    const double rate_max = limits.steering_rate_max_rad_per_s;
    const double rate_min = limits.steering_rate_min_rad_per_s;
    const double acc = limits.max_acceleration_mps2;

    DriveCommand previous{0.0, 0.0};
    GateInput state;  // the threaded state, as safety_node keeps it
    double wanted_steering = 0.0;
    double wanted_speed = 0.0;
    double sum_dt = 0.0;
    double growth_sum = 0.0;
    double steering_min_up = 0.0;    // min over earlier j of out_j - rate_max * S_j (j = 0: 0)
    double steering_max_down = 0.0;  // max over earlier j of out_j - rate_min * S_j
    double speed_min_up = 0.0;       // min over earlier j of G_j - acc * S_j

    for (int cycle = 0; cycle < kCycles && !report.failed(); ++cycle) {
      // A command that walks, jumps, goes out of bounds and sometimes is garbage.
      const int shape = dts.pick(100);
      if (shape < 55) {
        wanted_steering += dts.uniform(-1.5, 1.5) * rate_max * period;
        wanted_speed += dts.uniform(-1.5, 1.5) * acc * period;
      } else if (shape < 80) {
        wanted_steering = dts.uniform(-0.5, 0.5);
        wanted_speed = dts.uniform(-6.0, 22.0);
      } else if (shape < 90) {
        wanted_steering = previous.steering_angle_rad + dts.uniform(-1.0, 1.0) * rate_max * period;
        wanted_speed = previous.speed_mps + dts.uniform(-1.0, 1.0) * acc * period;
      }
      GateInput input = state;
      input.command = DriveCommand{wanted_steering, wanted_speed};
      const int garbage_command = dts.pick(100);
      if (garbage_command == 0) {
        input.command.steering_angle_rad = kNan;
      } else if (garbage_command == 1) {
        input.command.speed_mps = dts.pick(2) == 0 ? kInf : -kInf;
      }
      const int age_shape = dts.pick(100);
      input.drive_raw_age_s = dts.uniform(0.0, 0.03);
      if (age_shape < 5) {
        input.drive_raw_age_s = dts.uniform(0.06, 0.5);  // the watchdog
      } else if (age_shape == 5) {
        input.drive_raw_age_s = dts.pick(2) == 0 ? kNan : -0.1;
      }
      input.dt_s = dts.next(/*garbage=*/true);
      input.min_scan_range_m = kInf;
      input.min_rear_scan_range_m = kInf;
      if (dts.pick(100) < 15) {
        input.min_scan_range_m = dts.uniform(0.05, 3.0);
      }
      if (dts.pick(100) < 8) {
        input.min_rear_scan_range_m = dts.uniform(0.05, 2.0);
      }
      if (dts.pick(100) == 0) {
        input.min_scan_range_m = kNan;
      }
      if (dts.pick(2) == 0) {
        input.measured_speed_mps = dts.uniform(-1.0, 3.0);
      } else {
        input.measured_speed_mps.reset();
      }
      // Corrupt a threaded credit now and then, to 0 in effect (never above what the gate returned,
      // which would be free credit from outside the chain): the gate must sanitise it.
      if (dts.pick(200) == 0) {
        input.steering_rate_credit_s = dts.pick(2) == 0 ? kNan : -1.0;
      }

      const GateResult result = gate.evaluate(input, previous);
      const double dt = std::isfinite(input.dt_s) && input.dt_s >= 0.0 ? input.dt_s : 0.0;
      const bool dt_garbage = !(std::isfinite(input.dt_s) && input.dt_s >= 0.0);
      const double max_dt_period = dt > period ? dt : period;
      const DriveCommand out = result.output;

      // P4: credits are finite and in [0, T]; a garbage dt returns 0 and moves nothing the limiter
      // owns.
      for (const double credit : {result.steering_rate_credit_s, result.speed_rate_credit_s}) {
        if (!(std::isfinite(credit) && credit >= 0.0 && credit <= period)) {
          report.fail(seed, sequence, cycle, "credit out of [0, T]: " + std::to_string(credit));
        }
      }
      if (dt_garbage) {
        if (result.steering_rate_credit_s != 0.0 || result.speed_rate_credit_s != 0.0) {
          report.fail(seed, sequence, cycle, "garbage dt returned a credit");
        }
        if (out.steering_angle_rad != previous.steering_angle_rad) {
          report.fail(seed, sequence, cycle, "garbage dt moved the steering");
        }
        if (speed_growth(previous.speed_mps, out.speed_mps) > 0.0) {
          report.fail(seed, sequence, cycle, "garbage dt grew the speed");
        }
      }

      // P3 (G1, G2): the step of this cycle against the nominal period or the dt, whichever is
      // more, and a request within the old allowance is never reported as rate limited.
      const double steer_step = out.steering_angle_rad - previous.steering_angle_rad;
      if (steer_step > rate_max * max_dt_period + 1e-12 ||
          steer_step < rate_min * max_dt_period - 1e-12) {
        report.fail(seed, sequence, cycle, "steering step above the per-cycle cap");
      }
      const double growth = speed_growth(previous.speed_mps, out.speed_mps);
      if (growth > acc * max_dt_period + 1e-12) {
        report.fail(seed, sequence, cycle, "speed growth above the per-cycle cap");
      }
      const bool fresh =
          std::isfinite(input.drive_raw_age_s) && input.drive_raw_age_s >= 0.0 &&
          input.drive_raw_age_s < static_cast<double>(limits.watchdog_missed_cycles) * period;
      const bool finite =
          std::isfinite(input.command.steering_angle_rad) && std::isfinite(input.command.speed_mps);
      if (fresh && finite) {
        const double requested_steering = clamp_to(
            input.command.steering_angle_rad, limits.steering_min_rad, limits.steering_max_rad);
        const double requested_speed =
            clamp_to(input.command.speed_mps, limits.speed_min_mps, limits.speed_max_mps);
        const double steer_delta = requested_steering - previous.steering_angle_rad;
        const bool steering_inside = steer_delta <= rate_max * dt && steer_delta >= rate_min * dt;
        const bool speed_inside = speed_growth(previous.speed_mps, requested_speed) <= acc * dt;
        if (steering_inside && speed_inside) {
          for (const GateActivation& a : result.activations) {
            if (a.source == GateSource::kRateLimit) {
              report.fail(seed, sequence, cycle,
                          "a request within rate * dt was reported as rate limited: " + a.detail +
                              " prev=" + fmt17(previous.steering_angle_rad) + "," +
                              fmt17(previous.speed_mps) +
                              " cmd=" + fmt17(input.command.steering_angle_rad) + "," +
                              fmt17(input.command.speed_mps) +
                              " out=" + fmt17(out.steering_angle_rad) + "," + fmt17(out.speed_mps));
            }
          }
        }
      }

      // P1 and P2 (G3): over every window of consecutive cycles ending here, the movement is within
      // rate * (sum of dt + T). The extremes over earlier cycle boundaries are kept, so every
      // window is checked in constant time.
      sum_dt += dt;
      growth_sum += growth;
      if (out.steering_angle_rad - rate_max * sum_dt - steering_min_up > rate_max * period + 1e-9) {
        report.fail(seed, sequence, cycle, "steering moved up faster than rate_max * (sum dt + T)");
      }
      if (out.steering_angle_rad - rate_min * sum_dt - steering_max_down <
          rate_min * period - 1e-9) {
        report.fail(seed, sequence, cycle,
                    "steering moved down faster than rate_min * (sum dt + T)");
      }
      if (growth_sum - acc * sum_dt - speed_min_up > acc * period + 1e-9) {
        report.fail(seed, sequence, cycle, "speed grew faster than acc * (sum dt + T)");
      }
      const double up = out.steering_angle_rad - rate_max * sum_dt;
      if (up < steering_min_up) {
        steering_min_up = up;
      }
      const double down = out.steering_angle_rad - rate_min * sum_dt;
      if (down > steering_max_down) {
        steering_max_down = down;
      }
      const double speed_up = growth_sum - acc * sum_dt;
      if (speed_up < speed_min_up) {
        speed_min_up = speed_up;
      }

      previous = result.output;
      state.ttc_brake_latched = result.ttc_brake_latched;
      state.reverse_brake_latched = result.reverse_brake_latched;
      state.obstacle_hold_timer_s = result.obstacle_hold_timer_s;
      state.speed_bound_forward_mps = result.speed_bound_forward_mps;
      state.speed_bound_reverse_mps = result.speed_bound_reverse_mps;
      state.steering_rate_credit_s = result.steering_rate_credit_s;
      state.speed_rate_credit_s = result.speed_rate_credit_s;
    }
  }
}

// A staircase command stream: a value that changes once per publish (every T on the stream's own
// clock) by `share` of the limit of the direction it moves, reversing before the bounds.
class Stream {
 public:
  // `up_rate` and `down_rate` are the limits (per second, positive) of the two directions;
  // `lo` and `hi` the turning points; `start` the value held before the ramp.
  Stream(double share, double up_rate, double down_rate, double lo, double hi, double start,
         double period)
      : share_(share), up_(up_rate), down_(down_rate), lo_(lo), hi_(hi), period_(period) {
    values_.push_back(start);
  }
  // The value after `index` ramp publishes.
  double value(size_t index) {
    while (values_.size() <= index) {
      double x = values_.back();
      x += rising_ ? share_ * up_ * period_ : -share_ * down_ * period_;
      if (x >= hi_) {
        rising_ = false;
      } else if (x <= lo_) {
        rising_ = true;
      }
      values_.push_back(x);
    }
    return values_[index];
  }

 private:
  double share_, up_, down_, lo_, hi_, period_;
  bool rising_ = true;
  std::vector<double> values_;
};

// P5, the fix: a command stream that moves at share f of the limit by its own clock, seen with an
// age of at most 0.9 * T * (1 - f) / f at every cycle, is never rate limited whatever the cycle
// times, and the output is the command.
TEST(RateCreditProperties, ACommandStreamBelowTheLimitIsNeverClippedWhateverTheCycleTimes) {
  PropertyReport report;
  for (int sequence = 0; sequence < kSequences && !report.failed(); ++sequence) {
    const uint64_t seed = kSeedBase + 0x1000000ULL + static_cast<uint64_t>(sequence);
    std::mt19937_64 rng(seed);
    Dts dts(rng);
    const SafetyLimits limits = property_limits(sequence);
    const SafetyGateLogic gate(limits);
    const double period = limits.control_period_s;
    const double f_steering = dts.uniform(0.05, 0.45);
    const double f_speed = dts.uniform(0.05, 0.45);
    Stream steering(f_steering, limits.steering_rate_max_rad_per_s,
                    -limits.steering_rate_min_rad_per_s, -0.38, 0.38, 0.0, period);
    Stream speed(f_speed, limits.max_acceleration_mps2, limits.max_acceleration_mps2, 0.5, 12.0,
                 0.0, period);
    const double f_worst = f_steering > f_speed ? f_steering : f_speed;
    const double age_bound = 0.9 * period * (1.0 - f_worst) / f_worst;
    const double phase = dts.uniform(0.0, period);  // the stream's clock against the gate's

    double t = 0.0;  // the gate cycle's time
    const long idle_publishes = static_cast<long>(std::floor((t + period - phase) / period)) + 1;
    DriveCommand previous{0.0, 0.0};
    GateInput state;
    for (int cycle = 0; cycle < kCycles && !report.failed(); ++cycle) {
      const double dt = cycle == 0 ? period : dts.next(/*garbage=*/false);
      t += dt;
      const long n = static_cast<long>(std::floor((t - phase) / period));  // the latest publish
      const double age = t - (phase + static_cast<double>(n) * period);
      if (age > age_bound) {
        // The precondition of the property. With a stream that publishes every T the age is below
        // T, and the bound is above T for every f up to 0.45, so this is a bug of the generator.
        report.fail(seed, sequence, cycle, "the generated age is above the bound");
      }
      const long ramp = n - idle_publishes;
      GateInput input = state;
      input.command = ramp > 0 ? DriveCommand{steering.value(static_cast<size_t>(ramp)),
                                              speed.value(static_cast<size_t>(ramp))}
                               : DriveCommand{0.0, 0.0};
      input.drive_raw_age_s = age;
      input.dt_s = dt;
      input.min_scan_range_m = kInf;
      input.min_rear_scan_range_m = kInf;
      const GateResult result = gate.evaluate(input, previous);
      for (const GateActivation& a : result.activations) {
        if (a.source == GateSource::kRateLimit || a.source == GateSource::kWatchdog) {
          report.fail(
              seed, sequence, cycle,
              std::string("unexpected activation: ") + gate_source_to_string(a.source) + " " +
                  a.detail + " dt=" + std::to_string(dt) + " f_steering=" +
                  std::to_string(f_steering) + " f_speed=" + std::to_string(f_speed) +
                  " prev=" + fmt17(previous.steering_angle_rad) + "," + fmt17(previous.speed_mps) +
                  " cmd=" + fmt17(input.command.steering_angle_rad) + "," +
                  fmt17(input.command.speed_mps) + " out=" +
                  fmt17(result.output.steering_angle_rad) + "," + fmt17(result.output.speed_mps));
        }
      }
      if (std::fabs(result.output.steering_angle_rad - input.command.steering_angle_rad) > 1e-12 ||
          std::fabs(result.output.speed_mps - input.command.speed_mps) > 1e-12) {
        report.fail(seed, sequence, cycle, "the output is not the command");
      }
      previous = result.output;
      state.steering_rate_credit_s = result.steering_rate_credit_s;
      state.speed_rate_credit_s = result.speed_rate_credit_s;
    }
  }
}

}  // namespace
}  // namespace racer_safety
