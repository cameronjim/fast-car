// Table-driven L1 unit tests for racer_safety::SafetyGateLogic (claude-docs/12-testing.md:
// "racer_safety gate logic ... TTC math, staleness watchdog, covariance gate, command
// sanity -- table-driven, every gate condition x (pass / marginal / fail / garbage input)").
// No ROS: this file only includes gate_logic.hpp, exactly like racer_control's
// test_pure_pursuit.cpp.
//
// Test limits below are representative values in the same shape as config/vehicle_params.yaml
// (steering +-0.4189 rad, +-3.2 rad/s, speed [-5, 20] m/s, 9.51 m/s^2 max accel) but are NOT
// read from that file -- gate_logic.hpp/.cpp never include the generated vehicle_params
// binding (only safety_node.cpp does, see that file), so these are plain test fixtures, same
// pattern as racer_control's PurePursuitConfig test fixtures.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "racer_safety/gate_logic.hpp"

namespace racer_safety {
namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

SafetyLimits make_limits(std::optional<double> ttc_warning_s = std::nullopt,
                         std::optional<double> ttc_brake_s = std::nullopt) {
  SafetyLimits limits;
  limits.steering_min_rad = -0.4189;
  limits.steering_max_rad = 0.4189;
  limits.steering_rate_min_rad_per_s = -3.2;
  limits.steering_rate_max_rad_per_s = 3.2;
  limits.speed_min_mps = -5.0;
  limits.speed_max_mps = 20.0;
  limits.max_acceleration_mps2 = 9.51;
  limits.ttc_warning_s = ttc_warning_s;
  limits.ttc_brake_s = ttc_brake_s;
  limits.watchdog_missed_cycles = 3;
  limits.control_period_s = 0.02;  // 50 Hz -> 0.06s watchdog timeout
  return limits;
}

// A "neutral" cycle's worth of input: fresh command, no scan/TTC concern, no pose input.
// `dt_s` is deliberately large (rate limits are `rate * dt_s`, so a large dt_s makes the
// rate-limit gate a non-issue by construction) so that BoundsClamp/Ttc/CovarianceStub tests
// exercise ONLY the gate they name, without an unintended interaction from the rate-limit
// gate that always runs immediately after bounds clamping in evaluate() -- the dedicated
// RateLimit test group below overrides dt_s explicitly to small, specific values instead.
// Individual tests override only the fields they exercise.
GateInput neutral_input() {
  GateInput input;
  input.command = DriveCommand{0.0, 0.0};
  input.drive_raw_age_s = 0.0;
  input.dt_s = 1e6;
  input.min_scan_range_m = kInf;  // "no valid range this cycle"
  input.has_pose_input = false;
  input.pose_covariance_trace = 0.0;
  return input;
}

constexpr DriveCommand kZeroPrev{0.0, 0.0};

// ---------------------------------------------------------------------------------------
// Watchdog: pass / marginal (exactly at timeout) / fail (over timeout) / garbage (NaN, Inf,
// negative age).
// ---------------------------------------------------------------------------------------

TEST(Watchdog, PassesWhenAgeWellBelowTimeout) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = 0.01;  // timeout is 0.06s
  input.command = DriveCommand{0.1, 2.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.brake);
  for (const auto& event : result.events) {
    EXPECT_NE(event.source, GateSource::kWatchdog);
  }
}

TEST(Watchdog, MarginalAtExactlyTimeoutTrips) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = 0.06;  // == 3 * 0.02, the ">=" boundary
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.brake);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.0);
  ASSERT_EQ(result.events.size(), 1u);
  EXPECT_EQ(result.events[0].source, GateSource::kWatchdog);
  EXPECT_EQ(result.events[0].severity, EventSeverity::kBrake);
}

TEST(Watchdog, JustBelowTimeoutDoesNotTrip) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = 0.0599;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.brake);
}

TEST(Watchdog, FailsWellPastTimeout) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = 5.0;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.brake);
}

TEST(Watchdog, GarbageNanAgeTrips) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = kNan;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.brake);
}

TEST(Watchdog, GarbageInfAgeTrips) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = kInf;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.brake);
}

TEST(Watchdog, GarbageNegativeAgeTrips) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = -1.0;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.brake);
  ASSERT_EQ(result.events.size(), 1u);
  EXPECT_EQ(result.events[0].source, GateSource::kWatchdog);
}

TEST(Watchdog, TripDoesNotEvaluateOtherGates) {
  // A watchdog trip returns immediately: even a wildly-invalid command must produce exactly
  // ONE event (the watchdog's), not also a command-sanity event.
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = 5.0;
  input.command = DriveCommand{kNan, kNan};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  ASSERT_EQ(result.events.size(), 1u);
  EXPECT_EQ(result.events[0].source, GateSource::kWatchdog);
}

// ---------------------------------------------------------------------------------------
// Command sanity: pass / garbage (NaN steering, NaN speed, Inf steering, -Inf speed, both).
// ---------------------------------------------------------------------------------------

TEST(CommandSanity, PassesOnFiniteCommand) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.1, 3.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.brake);
  for (const auto& event : result.events) {
    EXPECT_NE(event.source, GateSource::kCommandSanity);
  }
}

TEST(CommandSanity, GarbageNanSteeringBrakes) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{kNan, 2.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.brake);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.0);
  ASSERT_EQ(result.events.size(), 1u);
  EXPECT_EQ(result.events[0].source, GateSource::kCommandSanity);
  EXPECT_EQ(result.events[0].severity, EventSeverity::kBrake);
}

TEST(CommandSanity, GarbageNanSpeedWithFiniteSteeringBrakes) {
  // Steering finite, speed non-finite: exercises is_finite_command's second branch
  // (first `if` false, second `if` true).
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.1, kNan};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.brake);
  ASSERT_EQ(result.events.size(), 1u);
  EXPECT_EQ(result.events[0].source, GateSource::kCommandSanity);
}

TEST(CommandSanity, GarbageInfSteeringBrakes) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{kInf, 2.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.brake);
}

TEST(CommandSanity, GarbageNegativeInfSpeedBrakes) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, -kInf};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.brake);
}

TEST(CommandSanity, GarbageBothNonFiniteBrakes) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{kNan, kInf};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.brake);
}

// ---------------------------------------------------------------------------------------
// Bounds clamp: pass (in range) / marginal (exactly at bound) / fail (over bound), steering
// and speed independently, plus the "neither/only-one/both changed" event-trigger matrix.
// ---------------------------------------------------------------------------------------

TEST(BoundsClamp, PassesWithinBoundsNoEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.1, 5.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.1);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 5.0);
  for (const auto& event : result.events) {
    EXPECT_NE(event.source, GateSource::kBoundsClamp);
  }
}

TEST(BoundsClamp, MarginalExactlyAtSteeringMaxNoEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.4189, 0.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.4189);
  for (const auto& event : result.events) {
    EXPECT_NE(event.source, GateSource::kBoundsClamp);
  }
}

TEST(BoundsClamp, MarginalExactlyAtSpeedMaxNoEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 20.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 20.0);
  for (const auto& event : result.events) {
    EXPECT_NE(event.source, GateSource::kBoundsClamp);
  }
}

TEST(BoundsClamp, FailsAboveSteeringMaxClampedWithEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.5, 0.0};  // > 0.4189
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.4189);
  ASSERT_EQ(result.events.size(), 1u);
  EXPECT_EQ(result.events[0].source, GateSource::kBoundsClamp);
  EXPECT_EQ(result.events[0].severity, EventSeverity::kWarning);
  EXPECT_FALSE(result.brake);
}

TEST(BoundsClamp, FailsBelowSteeringMinClampedWithEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{-0.5, 0.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, -0.4189);
  ASSERT_EQ(result.events.size(), 1u);
  EXPECT_EQ(result.events[0].source, GateSource::kBoundsClamp);
}

TEST(BoundsClamp, FailsAboveSpeedMaxClampedWithEventOnlySpeedChanged) {
  // Steering passes (no change) but speed is clamped: exercises the event condition's
  // "first false, second true" path.
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.1, 25.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.1);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 20.0);
  ASSERT_EQ(result.events.size(), 1u);
  EXPECT_EQ(result.events[0].source, GateSource::kBoundsClamp);
}

TEST(BoundsClamp, FailsBelowSpeedMinClampedWithEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, -10.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, -5.0);
  ASSERT_EQ(result.events.size(), 1u);
  EXPECT_EQ(result.events[0].source, GateSource::kBoundsClamp);
}

TEST(BoundsClamp, BothSteeringAndSpeedClampedProducesOneEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{1.0, 100.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.4189);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 20.0);
  // One bounds_clamp event (not one per field) -- this gate reports "a clamp happened",
  // not per-field spam.
  int bounds_events = 0;
  for (const auto& event : result.events) {
    if (event.source == GateSource::kBoundsClamp) {
      ++bounds_events;
    }
  }
  EXPECT_EQ(bounds_events, 1);
}

// ---------------------------------------------------------------------------------------
// Rate limit: pass (within rate) / marginal (exactly at rate) / fail (exceeds rate),
// steering both directions, speed acceleration only (deceleration is never limited).
// ---------------------------------------------------------------------------------------

TEST(RateLimit, SteeringWithinRatePassesNoEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.dt_s = 0.02;  // max delta = 3.2 * 0.02 = 0.064 rad
  input.command = DriveCommand{0.05, 0.0};
  const DriveCommand prev{0.0, 0.0};
  const GateResult result = gate.evaluate(input, prev);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.05);
  for (const auto& event : result.events) {
    EXPECT_NE(event.source, GateSource::kRateLimit);
  }
}

TEST(RateLimit, SteeringMarginalExactlyAtMaxRatePassesNoEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.dt_s = 0.02;
  input.command = DriveCommand{0.064, 0.0};  // exactly 3.2 * 0.02
  const DriveCommand prev{0.0, 0.0};
  const GateResult result = gate.evaluate(input, prev);
  EXPECT_NEAR(result.output.steering_angle_rad, 0.064, 1e-12);
  for (const auto& event : result.events) {
    EXPECT_NE(event.source, GateSource::kRateLimit);
  }
}

TEST(RateLimit, SteeringExceedsMaxRateClampedWithEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.dt_s = 0.02;
  input.command = DriveCommand{0.3, 0.0};  // far more than 0.064 rad in one step
  const DriveCommand prev{0.0, 0.0};
  const GateResult result = gate.evaluate(input, prev);
  EXPECT_NEAR(result.output.steering_angle_rad, 0.064, 1e-12);
  bool saw_rate_event = false;
  for (const auto& event : result.events) {
    if (event.source == GateSource::kRateLimit) saw_rate_event = true;
  }
  EXPECT_TRUE(saw_rate_event);
}

TEST(RateLimit, SteeringExceedsMinRateClampedNegativeDirection) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.dt_s = 0.02;
  input.command = DriveCommand{-0.3, 0.0};
  const DriveCommand prev{0.0, 0.0};
  const GateResult result = gate.evaluate(input, prev);
  EXPECT_NEAR(result.output.steering_angle_rad, -0.064, 1e-12);
}

TEST(RateLimit, SpeedAccelerationWithinLimitPassesNoEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.dt_s = 0.1;  // max delta = 9.51 * 0.1 = 0.951 m/s
  input.command = DriveCommand{0.0, 0.5};
  const DriveCommand prev{0.0, 0.0};
  const GateResult result = gate.evaluate(input, prev);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.5);
  for (const auto& event : result.events) {
    EXPECT_NE(event.source, GateSource::kRateLimit);
  }
}

TEST(RateLimit, SpeedAccelerationMarginalExactlyAtLimitPassesNoEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.dt_s = 0.1;
  input.command = DriveCommand{0.0, 0.951};  // exactly 9.51 * 0.1
  const DriveCommand prev{0.0, 0.0};
  const GateResult result = gate.evaluate(input, prev);
  EXPECT_NEAR(result.output.speed_mps, 0.951, 1e-9);
  for (const auto& event : result.events) {
    EXPECT_NE(event.source, GateSource::kRateLimit);
  }
}

TEST(RateLimit, SpeedAccelerationExceedsLimitClampedWithEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.dt_s = 0.1;
  input.command = DriveCommand{0.0, 5.0};  // way more than 0.951 m/s in one step
  const DriveCommand prev{0.0, 0.0};
  const GateResult result = gate.evaluate(input, prev);
  EXPECT_NEAR(result.output.speed_mps, 0.951, 1e-9);
  bool saw_rate_event = false;
  for (const auto& event : result.events) {
    if (event.source == GateSource::kRateLimit) saw_rate_event = true;
  }
  EXPECT_TRUE(saw_rate_event);
}

TEST(RateLimit, SpeedDecelerationIsNeverRateLimited) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.dt_s = 0.02;  // tiny dt -- an accel limit would clamp hard, but this is a decel
  input.command = DriveCommand{0.0, 0.0};
  const DriveCommand prev{0.0, 15.0};
  const GateResult result = gate.evaluate(input, prev);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.0);  // full stop allowed in one step
}

TEST(RateLimit, SpeedDeltaExactlyZeroSkipsAccelBranch) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.dt_s = 0.02;
  input.command = DriveCommand{0.0, 5.0};
  const DriveCommand prev{0.0, 5.0};
  const GateResult result = gate.evaluate(input, prev);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 5.0);
}

TEST(RateLimit, GarbageDtNanTreatedAsZeroAllowsNoChange) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.dt_s = kNan;
  input.command = DriveCommand{0.2, 10.0};
  const DriveCommand prev{0.0, 0.0};
  const GateResult result = gate.evaluate(input, prev);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.0);
}

TEST(RateLimit, GarbageDtNegativeTreatedAsZero) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.dt_s = -1.0;
  input.command = DriveCommand{0.2, 10.0};
  const DriveCommand prev{0.0, 0.0};
  const GateResult result = gate.evaluate(input, prev);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.0);
}

TEST(RateLimit, DtZeroAllowsNoChange) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.dt_s = 0.0;
  input.command = DriveCommand{0.2, 10.0};
  const DriveCommand prev{0.0, 0.0};
  const GateResult result = gate.evaluate(input, prev);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.0);
}

// ---------------------------------------------------------------------------------------
// TTC gate: not configured / garbage or absent range / not moving forward / pass (far) /
// marginal (exactly at each threshold) / fail (brake zone), with and without a warning
// threshold configured.
// ---------------------------------------------------------------------------------------

TEST(Ttc, NotConfiguredNeverBrakesEvenAtZeroRange) {
  SafetyGateLogic gate(make_limits(/*ttc_warning_s=*/std::nullopt, /*ttc_brake_s=*/std::nullopt));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 10.0};
  input.min_scan_range_m = 0.01;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.brake);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 10.0);
  for (const auto& event : result.events) {
    EXPECT_NE(event.source, GateSource::kTtc);
  }
}

TEST(Ttc, GarbageNanRangeIgnored) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 10.0};
  input.min_scan_range_m = kNan;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.brake);
}

TEST(Ttc, GarbageNegativeRangeIgnored) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 10.0};
  input.min_scan_range_m = -3.0;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.brake);
}

TEST(Ttc, GarbageZeroRangeIgnored) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 10.0};
  input.min_scan_range_m = 0.0;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.brake);
}

TEST(Ttc, InfiniteRangeMeansNoObstacleNoBrake) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 10.0};
  input.min_scan_range_m = kInf;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.brake);
}

TEST(Ttc, NotMovingForwardNeverBrakesEvenAtZeroRange) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 0.0};  // stopped
  input.min_scan_range_m = 0.001;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.brake);
}

TEST(Ttc, ReversingNeverBrakesEvenAtZeroRange) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, -3.0};  // reversing
  input.min_scan_range_m = 0.001;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.brake);
  EXPECT_LT(result.output.speed_mps, 0.0);  // reverse command passed through untouched by TTC
}

TEST(Ttc, PassesFarFromObstacleNoEvent) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 5.0};
  input.min_scan_range_m = 100.0;  // ttc = 20s, far above both thresholds
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.brake);
  for (const auto& event : result.events) {
    EXPECT_NE(event.source, GateSource::kTtc);
  }
}

TEST(Ttc, MarginalExactlyAtWarningThresholdEmitsInfoNoCommandChange) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 5.0};
  input.min_scan_range_m = 5.0;  // ttc = 5/5 = 1.0s == warning threshold
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.brake);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 5.0);
  ASSERT_EQ(result.events.size(), 1u);
  EXPECT_EQ(result.events[0].source, GateSource::kTtc);
  EXPECT_EQ(result.events[0].severity, EventSeverity::kInfo);
}

TEST(Ttc, BetweenWarningAndBrakeEmitsInfoOnly) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 5.0};
  input.min_scan_range_m = 4.0;  // ttc = 0.8s, between 0.5 and 1.0
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.brake);
  ASSERT_EQ(result.events.size(), 1u);
  EXPECT_EQ(result.events[0].severity, EventSeverity::kInfo);
}

TEST(Ttc, MarginalExactlyAtBrakeThresholdBrakes) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 5.0};
  input.min_scan_range_m = 2.5;  // ttc = 2.5/5 = 0.5s == brake threshold
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.brake);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.0);
  ASSERT_EQ(result.events.size(), 1u);
  EXPECT_EQ(result.events[0].source, GateSource::kTtc);
  EXPECT_EQ(result.events[0].severity, EventSeverity::kBrake);
}

TEST(Ttc, FailsWellInsideBrakeThresholdBrakesAndZeroesSpeedOnlyKeepsSteering) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.2, 5.0};
  input.min_scan_range_m = 0.1;  // ttc = 0.02s, deep in brake zone
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.brake);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.2);  // steering preserved
}

TEST(Ttc, NoWarningThresholdConfiguredSkipsWarningZoneSilently) {
  // Only ttc_brake_s is set (ttc_warning_s is nullopt) -- exercises the
  // "warning_has_value() == false" branch of the nested check.
  SafetyGateLogic gate(make_limits(std::nullopt, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 5.0};
  input.min_scan_range_m = 4.0;  // ttc = 0.8s: would be "warning zone" if a threshold existed
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.brake);
  for (const auto& event : result.events) {
    EXPECT_NE(event.source, GateSource::kTtc);
  }
}

// ---------------------------------------------------------------------------------------
// Covariance gate stub: fails SAFE -- absent pose input never disables the other gates, and
// the stub itself never invents a derate even if "engaged".
// ---------------------------------------------------------------------------------------

TEST(CovarianceStub, NoPoseInputMeansNotEngagedNoFraction) {
  const CovarianceGateResult result = evaluate_covariance_gate(/*has_pose_input=*/false, 0.0);
  EXPECT_FALSE(result.engaged);
  EXPECT_DOUBLE_EQ(result.speed_fraction, 1.0);
}

TEST(CovarianceStub, PoseInputEngagesButStillNoFractionUntilRoadmap26) {
  const CovarianceGateResult result =
      evaluate_covariance_gate(/*has_pose_input=*/true, /*pose_covariance_trace=*/99.0);
  EXPECT_TRUE(result.engaged);
  EXPECT_DOUBLE_EQ(result.speed_fraction, 1.0);
}

TEST(CovarianceStub, AbsentPoseInputDoesNotDisableWatchdog) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.has_pose_input = false;
  input.drive_raw_age_s = 5.0;  // would trip the watchdog regardless of covariance
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.brake);
  bool saw_watchdog = false;
  for (const auto& event : result.events) {
    if (event.source == GateSource::kWatchdog) saw_watchdog = true;
  }
  EXPECT_TRUE(saw_watchdog);
}

TEST(CovarianceStub, AbsentPoseInputDoesNotDisableCommandSanity) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.has_pose_input = false;
  input.command = DriveCommand{kNan, 0.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.brake);
  bool saw_sanity = false;
  for (const auto& event : result.events) {
    if (event.source == GateSource::kCommandSanity) saw_sanity = true;
  }
  EXPECT_TRUE(saw_sanity);
}

TEST(CovarianceStub, AbsentPoseInputDoesNotDisableBoundsClamp) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.has_pose_input = false;
  input.command = DriveCommand{1.0, 0.0};  // out of steering bounds
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.4189);
}

TEST(CovarianceStub, AbsentPoseInputDoesNotDisableTtc) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.has_pose_input = false;
  input.command = DriveCommand{0.0, 5.0};
  input.min_scan_range_m = 0.1;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.brake);
}

TEST(CovarianceStub, EngagedProducesEventButNoSpeedChangeGivenFixedFraction) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.has_pose_input = true;  // exercises evaluate()'s "engaged" event branch
  input.pose_covariance_trace = 5.0;
  input.command = DriveCommand{0.0, 5.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 5.0);  // fraction is 1.0 until roadmap 2.6
  bool saw_covariance = false;
  for (const auto& event : result.events) {
    if (event.source == GateSource::kCovariance) saw_covariance = true;
  }
  EXPECT_TRUE(saw_covariance);
}

// ---------------------------------------------------------------------------------------
// gate_source_to_string: every enumerator maps to its documented racer_msgs/SafetyEvent
// `source` string.
// ---------------------------------------------------------------------------------------

TEST(GateSourceToString, CoversEveryEnumerator) {
  EXPECT_EQ(gate_source_to_string(GateSource::kWatchdog), "watchdog");
  EXPECT_EQ(gate_source_to_string(GateSource::kCommandSanity), "command_sanity");
  EXPECT_EQ(gate_source_to_string(GateSource::kBoundsClamp), "bounds_clamp");
  EXPECT_EQ(gate_source_to_string(GateSource::kRateLimit), "rate_limit");
  EXPECT_EQ(gate_source_to_string(GateSource::kTtc), "ttc");
  EXPECT_EQ(gate_source_to_string(GateSource::kCovariance), "covariance");
  EXPECT_EQ(gate_source_to_string(GateSource::kInternalFault), "internal_fault");
}

// ---------------------------------------------------------------------------------------
// L2-flavored property check (claude-docs/12-testing.md L2: "for ANY input command and ANY
// internal state, output is always within bounds" -- applied here to the gate's own
// bounds/speed-cap contract with a small deterministic sweep in lieu of pulling in a C++
// hypothesis-equivalent for one file).
// ---------------------------------------------------------------------------------------

TEST(BoundsClamp, PropertySweepOutputNeverExceedsBoundsForAnyFiniteInput) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  const SafetyLimits limits = make_limits(1.0, 0.5);
  DriveCommand prev{0.0, 0.0};
  for (double steer = -2.0; steer <= 2.0; steer += 0.37) {
    for (double speed = -20.0; speed <= 40.0; speed += 3.3) {
      for (double range = 0.05; range <= 50.0; range += 7.0) {
        GateInput input = neutral_input();
        input.command = DriveCommand{steer, speed};
        input.min_scan_range_m = range;
        input.dt_s = 0.02;
        const GateResult result = gate.evaluate(input, prev);
        EXPECT_GE(result.output.steering_angle_rad, limits.steering_min_rad - 1e-9);
        EXPECT_LE(result.output.steering_angle_rad, limits.steering_max_rad + 1e-9);
        EXPECT_GE(result.output.speed_mps, limits.speed_min_mps - 1e-9);
        EXPECT_LE(result.output.speed_mps, limits.speed_max_mps + 1e-9);
        prev = result.output;
      }
    }
  }
}

}  // namespace
}  // namespace racer_safety
