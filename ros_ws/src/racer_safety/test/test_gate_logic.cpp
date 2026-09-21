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
#include <vector>

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
  EXPECT_FALSE(result.zero_throttle);
  for (const auto& event : result.activations) {
    EXPECT_NE(event.source, GateSource::kWatchdog);
  }
}

TEST(Watchdog, MarginalAtExactlyTimeoutTrips) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = 0.06;  // == 3 * 0.02, the ">=" boundary
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.zero_throttle);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.0);
  ASSERT_EQ(result.activations.size(), 1u);
  EXPECT_EQ(result.activations[0].source, GateSource::kWatchdog);
  EXPECT_EQ(result.activations[0].severity, EventSeverity::kBrake);
}

TEST(Watchdog, JustBelowTimeoutDoesNotTrip) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = 0.0599;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.zero_throttle);
}

TEST(Watchdog, FailsWellPastTimeout) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = 5.0;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.zero_throttle);
}

TEST(Watchdog, GarbageNanAgeTrips) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = kNan;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.zero_throttle);
}

TEST(Watchdog, GarbageInfAgeTrips) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = kInf;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.zero_throttle);
}

TEST(Watchdog, GarbageNegativeAgeTrips) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = -1.0;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.zero_throttle);
  ASSERT_EQ(result.activations.size(), 1u);
  EXPECT_EQ(result.activations[0].source, GateSource::kWatchdog);
}

TEST(Watchdog, TripDoesNotEvaluateOtherGates) {
  // A watchdog trip returns immediately: even a wildly-invalid command must produce exactly
  // ONE event (the watchdog's), not also a command-sanity event.
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = 5.0;
  input.command = DriveCommand{kNan, kNan};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  ASSERT_EQ(result.activations.size(), 1u);
  EXPECT_EQ(result.activations[0].source, GateSource::kWatchdog);
}

// ---------------------------------------------------------------------------------------
// Command sanity: pass / garbage (NaN steering, NaN speed, Inf steering, -Inf speed, both).
// ---------------------------------------------------------------------------------------

TEST(CommandSanity, PassesOnFiniteCommand) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.1, 3.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.zero_throttle);
  for (const auto& event : result.activations) {
    EXPECT_NE(event.source, GateSource::kCommandSanity);
  }
}

TEST(CommandSanity, GarbageNanSteeringBrakes) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{kNan, 2.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.zero_throttle);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.0);
  ASSERT_EQ(result.activations.size(), 1u);
  EXPECT_EQ(result.activations[0].source, GateSource::kCommandSanity);
  EXPECT_EQ(result.activations[0].severity, EventSeverity::kBrake);
}

TEST(CommandSanity, GarbageNanSpeedWithFiniteSteeringBrakes) {
  // Steering finite, speed non-finite: exercises is_finite_command's second branch
  // (first `if` false, second `if` true).
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.1, kNan};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.zero_throttle);
  ASSERT_EQ(result.activations.size(), 1u);
  EXPECT_EQ(result.activations[0].source, GateSource::kCommandSanity);
}

TEST(CommandSanity, GarbageInfSteeringBrakes) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{kInf, 2.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.zero_throttle);
}

TEST(CommandSanity, GarbageNegativeInfSpeedBrakes) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, -kInf};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.zero_throttle);
}

TEST(CommandSanity, GarbageBothNonFiniteBrakes) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{kNan, kInf};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.zero_throttle);
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
  for (const auto& event : result.activations) {
    EXPECT_NE(event.source, GateSource::kBoundsClamp);
  }
}

TEST(BoundsClamp, MarginalExactlyAtSteeringMaxNoEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.4189, 0.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.4189);
  for (const auto& event : result.activations) {
    EXPECT_NE(event.source, GateSource::kBoundsClamp);
  }
}

TEST(BoundsClamp, MarginalExactlyAtSpeedMaxNoEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 20.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 20.0);
  for (const auto& event : result.activations) {
    EXPECT_NE(event.source, GateSource::kBoundsClamp);
  }
}

TEST(BoundsClamp, FailsAboveSteeringMaxClampedWithEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.5, 0.0};  // > 0.4189
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.4189);
  ASSERT_EQ(result.activations.size(), 1u);
  EXPECT_EQ(result.activations[0].source, GateSource::kBoundsClamp);
  EXPECT_EQ(result.activations[0].severity, EventSeverity::kWarning);
  EXPECT_FALSE(result.zero_throttle);
}

TEST(BoundsClamp, FailsBelowSteeringMinClampedWithEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{-0.5, 0.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, -0.4189);
  ASSERT_EQ(result.activations.size(), 1u);
  EXPECT_EQ(result.activations[0].source, GateSource::kBoundsClamp);
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
  ASSERT_EQ(result.activations.size(), 1u);
  EXPECT_EQ(result.activations[0].source, GateSource::kBoundsClamp);
}

TEST(BoundsClamp, FailsBelowSpeedMinClampedWithEvent) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, -10.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, -5.0);
  ASSERT_EQ(result.activations.size(), 1u);
  EXPECT_EQ(result.activations[0].source, GateSource::kBoundsClamp);
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
  for (const auto& event : result.activations) {
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
  for (const auto& event : result.activations) {
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
  for (const auto& event : result.activations) {
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
  for (const auto& event : result.activations) {
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
  for (const auto& event : result.activations) {
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
  for (const auto& event : result.activations) {
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
  for (const auto& event : result.activations) {
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
  EXPECT_FALSE(result.zero_throttle);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 10.0);
  for (const auto& event : result.activations) {
    EXPECT_NE(event.source, GateSource::kTtc);
  }
}

TEST(Ttc, GarbageNanRangeIgnored) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 10.0};
  input.min_scan_range_m = kNan;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.zero_throttle);
}

TEST(Ttc, GarbageNegativeRangeIgnored) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 10.0};
  input.min_scan_range_m = -3.0;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.zero_throttle);
}

TEST(Ttc, GarbageZeroRangeIgnored) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 10.0};
  input.min_scan_range_m = 0.0;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.zero_throttle);
}

TEST(Ttc, InfiniteRangeMeansNoObstacleNoBrake) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 10.0};
  input.min_scan_range_m = kInf;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.zero_throttle);
}

TEST(Ttc, NotMovingForwardNeverBrakesEvenAtZeroRange) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 0.0};  // stopped
  input.min_scan_range_m = 0.001;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.zero_throttle);
}

TEST(Ttc, ReversingNeverBrakesEvenAtZeroRange) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, -3.0};  // reversing
  input.min_scan_range_m = 0.001;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.zero_throttle);
  EXPECT_LT(result.output.speed_mps, 0.0);  // reverse command passed through untouched by TTC
}

TEST(Ttc, PassesFarFromObstacleNoEvent) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 5.0};
  input.min_scan_range_m = 100.0;  // ttc = 20s, far above both thresholds
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.zero_throttle);
  for (const auto& event : result.activations) {
    EXPECT_NE(event.source, GateSource::kTtc);
  }
}

TEST(Ttc, MarginalExactlyAtWarningThresholdEmitsInfoNoCommandChange) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 5.0};
  input.min_scan_range_m = 5.0;  // ttc = 5/5 = 1.0s == warning threshold
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.zero_throttle);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 5.0);
  ASSERT_EQ(result.activations.size(), 1u);
  EXPECT_EQ(result.activations[0].source, GateSource::kTtc);
  EXPECT_EQ(result.activations[0].severity, EventSeverity::kInfo);
}

TEST(Ttc, BetweenWarningAndBrakeEmitsInfoOnly) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 5.0};
  input.min_scan_range_m = 4.0;  // ttc = 0.8s, between 0.5 and 1.0
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_FALSE(result.zero_throttle);
  ASSERT_EQ(result.activations.size(), 1u);
  EXPECT_EQ(result.activations[0].severity, EventSeverity::kInfo);
}

TEST(Ttc, MarginalExactlyAtBrakeThresholdBrakes) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.0, 5.0};
  input.min_scan_range_m = 2.5;  // ttc = 2.5/5 = 0.5s == brake threshold
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.zero_throttle);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.0);
  ASSERT_EQ(result.activations.size(), 1u);
  EXPECT_EQ(result.activations[0].source, GateSource::kTtc);
  EXPECT_EQ(result.activations[0].severity, EventSeverity::kBrake);
}

TEST(Ttc, FailsWellInsideBrakeThresholdBrakesAndZeroesSpeedOnlyKeepsSteering) {
  SafetyGateLogic gate(make_limits(1.0, 0.5));
  GateInput input = neutral_input();
  input.command = DriveCommand{0.2, 5.0};
  input.min_scan_range_m = 0.1;  // ttc = 0.02s, deep in brake zone
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_TRUE(result.zero_throttle);
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
  EXPECT_FALSE(result.zero_throttle);
  for (const auto& event : result.activations) {
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
  EXPECT_TRUE(result.zero_throttle);
  bool saw_watchdog = false;
  for (const auto& event : result.activations) {
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
  EXPECT_TRUE(result.zero_throttle);
  bool saw_sanity = false;
  for (const auto& event : result.activations) {
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
  EXPECT_TRUE(result.zero_throttle);
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
  for (const auto& event : result.activations) {
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

// ---------------------------------------------------------------------------------------
// GateEventTracker: per-cycle activations -> engage/release TRANSITION records
// (GitHub issue #37). Table-driven over every gate source and severity, plus sustained
// engagement, simultaneous gates, flapping, escalation, and garbage clock input.
// ---------------------------------------------------------------------------------------

constexpr GateSource kAllSources[] = {
    GateSource::kWatchdog,      GateSource::kCommandSanity, GateSource::kBoundsClamp,
    GateSource::kRateLimit,     GateSource::kTtc,           GateSource::kCovariance,
    GateSource::kInternalFault,
};

constexpr EventSeverity kAllSeverities[] = {
    EventSeverity::kInfo,
    EventSeverity::kWarning,
    EventSeverity::kBrake,
};

GateActivation activation(GateSource source, EventSeverity severity) {
  return GateActivation{source, severity, "detail"};
}

TEST(GateEventTracker, EveryGateAndSeverityEngagesOnceSustainsSilentlyAndReleasesOnce) {
  for (const GateSource source : kAllSources) {
    for (const EventSeverity severity : kAllSeverities) {
      GateEventTracker tracker;
      const std::vector<GateActivation> engaged{activation(source, severity)};

      const std::vector<SafetyEventRecord> on_engage = tracker.update(engaged, 10.0);
      ASSERT_EQ(on_engage.size(), 1u);
      EXPECT_EQ(on_engage[0].source, source);
      EXPECT_EQ(on_engage[0].severity, severity);
      EXPECT_EQ(on_engage[0].phase, EventPhase::kEngage);
      EXPECT_EQ(on_engage[0].detail, "detail");
      EXPECT_DOUBLE_EQ(on_engage[0].duration_s, 0.0);

      // The whole point of issue #37: 100 further cycles of the SAME engagement emit
      // nothing at all, at any rate.
      for (int cycle = 1; cycle <= 100; ++cycle) {
        EXPECT_TRUE(tracker.update(engaged, 10.0 + 0.02 * cycle).empty())
            << "sustained engagement re-emitted on cycle " << cycle;
      }

      const std::vector<SafetyEventRecord> on_release = tracker.update({}, 13.0);
      ASSERT_EQ(on_release.size(), 1u);
      EXPECT_EQ(on_release[0].source, source);
      EXPECT_EQ(on_release[0].severity, severity);
      EXPECT_EQ(on_release[0].phase, EventPhase::kRelease);
      EXPECT_DOUBLE_EQ(on_release[0].duration_s, 3.0);

      // Released for good: further idle cycles emit nothing.
      EXPECT_TRUE(tracker.update({}, 14.0).empty());
    }
  }
}

TEST(GateEventTracker, IdleTrackerEmitsNothing) {
  GateEventTracker tracker;
  EXPECT_TRUE(tracker.update({}, 0.0).empty());
  EXPECT_TRUE(tracker.update({}, 1.0).empty());
}

TEST(GateEventTracker, SimultaneousGatesHaveIndependentLifecycles) {
  GateEventTracker tracker;

  // Cycle 1: two gates engage together -> exactly two engage records.
  const std::vector<SafetyEventRecord> cycle1 =
      tracker.update({activation(GateSource::kBoundsClamp, EventSeverity::kWarning),
                      activation(GateSource::kRateLimit, EventSeverity::kWarning)},
                     100.0);
  ASSERT_EQ(cycle1.size(), 2u);
  EXPECT_EQ(cycle1[0].source, GateSource::kBoundsClamp);
  EXPECT_EQ(cycle1[0].phase, EventPhase::kEngage);
  EXPECT_EQ(cycle1[1].source, GateSource::kRateLimit);
  EXPECT_EQ(cycle1[1].phase, EventPhase::kEngage);

  // Cycle 2: one stays engaged, a third engages -> one engage record only.
  const std::vector<SafetyEventRecord> cycle2 =
      tracker.update({activation(GateSource::kBoundsClamp, EventSeverity::kWarning),
                      activation(GateSource::kTtc, EventSeverity::kBrake)},
                     101.0);
  ASSERT_EQ(cycle2.size(), 2u);
  EXPECT_EQ(cycle2[0].source, GateSource::kTtc);
  EXPECT_EQ(cycle2[0].phase, EventPhase::kEngage);
  EXPECT_EQ(cycle2[1].source, GateSource::kRateLimit);
  EXPECT_EQ(cycle2[1].phase, EventPhase::kRelease);
  EXPECT_DOUBLE_EQ(cycle2[1].duration_s, 1.0);

  // Cycle 3: everything releases, each with its OWN duration.
  const std::vector<SafetyEventRecord> cycle3 = tracker.update({}, 105.0);
  ASSERT_EQ(cycle3.size(), 2u);
  for (const SafetyEventRecord& record : cycle3) {
    EXPECT_EQ(record.phase, EventPhase::kRelease);
    if (record.source == GateSource::kBoundsClamp) {
      EXPECT_DOUBLE_EQ(record.duration_s, 5.0);
    } else {
      EXPECT_EQ(record.source, GateSource::kTtc);
      EXPECT_DOUBLE_EQ(record.duration_s, 4.0);
    }
  }
}

TEST(GateEventTracker, FlappingEngageReleaseEngageWithinThreeCyclesIsThreeRecords) {
  GateEventTracker tracker;
  const std::vector<GateActivation> engaged{
      activation(GateSource::kRateLimit, EventSeverity::kWarning)};

  const std::vector<SafetyEventRecord> cycle1 = tracker.update(engaged, 0.00);
  ASSERT_EQ(cycle1.size(), 1u);
  EXPECT_EQ(cycle1[0].phase, EventPhase::kEngage);

  const std::vector<SafetyEventRecord> cycle2 = tracker.update({}, 0.02);
  ASSERT_EQ(cycle2.size(), 1u);
  EXPECT_EQ(cycle2[0].phase, EventPhase::kRelease);
  EXPECT_NEAR(cycle2[0].duration_s, 0.02, 1e-12);

  const std::vector<SafetyEventRecord> cycle3 = tracker.update(engaged, 0.04);
  ASSERT_EQ(cycle3.size(), 1u);
  EXPECT_EQ(cycle3[0].phase, EventPhase::kEngage);
  EXPECT_DOUBLE_EQ(cycle3[0].duration_s, 0.0);

  // Two separate interventions, so a counter of PHASE_ENGAGE records sees exactly two.
  const std::vector<SafetyEventRecord> cycle4 = tracker.update({}, 0.06);
  ASSERT_EQ(cycle4.size(), 1u);
  EXPECT_EQ(cycle4[0].phase, EventPhase::kRelease);
}

TEST(GateEventTracker, SeverityEscalationOnTheSameSourceIsAReleaseAndANewEngage) {
  GateEventTracker tracker;
  ASSERT_EQ(tracker.update({activation(GateSource::kTtc, EventSeverity::kInfo)}, 0.0).size(), 1u);

  // TTC advisory becomes a TTC brake: the brake MUST show up as its own engagement, or an
  // evaluation counting BRAKE-severity interventions would never see it.
  const std::vector<SafetyEventRecord> escalation =
      tracker.update({activation(GateSource::kTtc, EventSeverity::kBrake)}, 0.5);
  ASSERT_EQ(escalation.size(), 2u);
  EXPECT_EQ(escalation[0].phase, EventPhase::kEngage);
  EXPECT_EQ(escalation[0].severity, EventSeverity::kBrake);
  EXPECT_EQ(escalation[1].phase, EventPhase::kRelease);
  EXPECT_EQ(escalation[1].severity, EventSeverity::kInfo);
  EXPECT_DOUBLE_EQ(escalation[1].duration_s, 0.5);
}

TEST(GateEventTracker, GarbageDuplicateActivationInOneCycleIsOneEngagement) {
  GateEventTracker tracker;
  const std::vector<SafetyEventRecord> records =
      tracker.update({activation(GateSource::kWatchdog, EventSeverity::kBrake),
                      activation(GateSource::kWatchdog, EventSeverity::kBrake)},
                     0.0);
  ASSERT_EQ(records.size(), 1u);
  EXPECT_EQ(records[0].phase, EventPhase::kEngage);
  // And it is genuinely engaged exactly once: the next idle cycle releases it once.
  ASSERT_EQ(tracker.update({}, 1.0).size(), 1u);
}

TEST(GateEventTracker, GarbageNonFiniteClockReportsZeroDuration) {
  for (const double garbage_now_s : {kNan, kInf, -kInf}) {
    GateEventTracker tracker;
    ASSERT_EQ(tracker.update({activation(GateSource::kTtc, EventSeverity::kBrake)}, 0.0).size(),
              1u);
    const std::vector<SafetyEventRecord> released = tracker.update({}, garbage_now_s);
    ASSERT_EQ(released.size(), 1u);
    EXPECT_EQ(released[0].phase, EventPhase::kRelease);
    EXPECT_DOUBLE_EQ(released[0].duration_s, 0.0);
  }
}

TEST(GateEventTracker, GarbageBackwardsClockReportsZeroDurationNotANegativeOne) {
  GateEventTracker tracker;
  ASSERT_EQ(
      tracker.update({activation(GateSource::kCovariance, EventSeverity::kWarning)}, 50.0).size(),
      1u);
  const std::vector<SafetyEventRecord> released = tracker.update({}, 20.0);
  ASSERT_EQ(released.size(), 1u);
  EXPECT_DOUBLE_EQ(released[0].duration_s, 0.0);
}

TEST(GateEventTracker, GarbageNonFiniteEngageClockStillProducesAWellFormedPair) {
  GateEventTracker tracker;
  ASSERT_EQ(tracker.update({activation(GateSource::kWatchdog, EventSeverity::kBrake)}, kNan).size(),
            1u);
  const std::vector<SafetyEventRecord> released = tracker.update({}, 1.0);
  ASSERT_EQ(released.size(), 1u);
  EXPECT_EQ(released[0].phase, EventPhase::kRelease);
  EXPECT_DOUBLE_EQ(released[0].duration_s, 0.0);
}

// End-to-end over the real gate: a permanently silent /drive_raw (the exact reproducer in
// issue #37, which measured 248 records in 14 s) must produce ONE watchdog record, not one
// per cycle.
TEST(GateEventTracker, SustainedWatchdogOverManyRealGateCyclesEmitsExactlyOneEngage) {
  SafetyGateLogic gate(make_limits());
  GateEventTracker tracker;
  int engage_records = 0;
  int total_records = 0;

  for (int cycle = 0; cycle < 700; ++cycle) {  // 14 s at 50 Hz
    GateInput input = neutral_input();
    input.drive_raw_age_s = kInf;  // never received a command, exactly like a cold boot
    input.dt_s = 0.02;
    const GateResult result = gate.evaluate(input, kZeroPrev);
    ASSERT_EQ(result.activations.size(), 1u);
    for (const SafetyEventRecord& record : tracker.update(result.activations, 0.02 * cycle)) {
      ++total_records;
      if (record.phase == EventPhase::kEngage) {
        ++engage_records;
        EXPECT_EQ(record.source, GateSource::kWatchdog);
      }
    }
  }

  EXPECT_EQ(engage_records, 1);
  EXPECT_EQ(total_records, 1);
}

// ---------------------------------------------------------------------------------------
// Zero-throttle gates hold the previous steering angle rather than centring it
// (gate_logic.hpp, "STEERING ON A ZERO-THROTTLE GATE"). Every one of these FAILS against the
// pre-review sources, which wrote DriveCommand{0, 0} on the watchdog and sanity paths.
// ---------------------------------------------------------------------------------------

TEST(ZeroThrottleSteering, WatchdogHoldsPreviousSteeringAndZeroesSpeed) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = 5.0;
  const DriveCommand prev{0.3, 4.0};  // mid-corner at speed when the publisher died
  const GateResult result = gate.evaluate(input, prev);
  EXPECT_TRUE(result.zero_throttle);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.3);
}

TEST(ZeroThrottleSteering, WatchdogHoldsANegativeRightHandSteeringAngleToo) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = 5.0;
  const GateResult result = gate.evaluate(input, DriveCommand{-0.25, 2.0});
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, -0.25);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.0);
}

TEST(ZeroThrottleSteering, CommandSanityHoldsPreviousSteeringAndZeroesSpeed) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{kNan, 3.0};
  const GateResult result = gate.evaluate(input, DriveCommand{0.2, 3.0});
  EXPECT_TRUE(result.zero_throttle);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.2);
}

TEST(ZeroThrottleSteering, FromAZeroPreviousOutputTheOutputIsStillExactlyZeroZero) {
  // The runbook's "verify /drive is neutral with no input" step and its L3 test depend on
  // this: a node that has never commanded anything still emits {0, 0}.
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.drive_raw_age_s = 5.0;
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(result.output.speed_mps, 0.0);
}

TEST(ZeroThrottleCommand, HoldsAFiniteSteeringAngle) {
  const DriveCommand out = zero_throttle_command(DriveCommand{0.31, 7.0});
  EXPECT_DOUBLE_EQ(out.steering_angle_rad, 0.31);
  EXPECT_DOUBLE_EQ(out.speed_mps, 0.0);
}

TEST(ZeroThrottleCommand, GarbageNonFinitePreviousSteeringCentresInsteadOfEmittingNaN) {
  for (const double garbage : {kNan, kInf, -kInf}) {
    const DriveCommand out = zero_throttle_command(DriveCommand{garbage, 1.0});
    EXPECT_DOUBLE_EQ(out.steering_angle_rad, 0.0);
    EXPECT_DOUBLE_EQ(out.speed_mps, 0.0);
  }
}

// ---------------------------------------------------------------------------------------
// Steering sign convention (claude-docs/06-vehicle-params.md: road-wheel angle, LEFT
// POSITIVE). safety_node is a pass-through for the sign; these pin that it never inverts one.
// The other hops are pinned in racer_tools/test/test_keymap.py (left key -> +angle),
// test_twist_teleop.py (+angular.z, forward -> +angle) and racer_drivers/
// test/test_pwm_mapping.cpp (+angle -> the pwm end config/vehicle_params.yaml's
// steering.pwm_left_bound names).
// ---------------------------------------------------------------------------------------

TEST(SteeringSign, LeftPositiveCommandPassesThroughStillPositive) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{0.2, 1.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_GT(result.output.steering_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, 0.2);
}

TEST(SteeringSign, RightNegativeCommandPassesThroughStillNegative) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{-0.2, 1.0};
  const GateResult result = gate.evaluate(input, kZeroPrev);
  EXPECT_LT(result.output.steering_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(result.output.steering_angle_rad, -0.2);
}

TEST(SteeringSign, ClampingAtEitherLimitKeepsTheSideItWasOn) {
  SafetyGateLogic gate(make_limits());
  GateInput input = neutral_input();
  input.command = DriveCommand{5.0, 0.0};
  EXPECT_DOUBLE_EQ(gate.evaluate(input, kZeroPrev).output.steering_angle_rad, 0.4189);
  input.command = DriveCommand{-5.0, 0.0};
  EXPECT_DOUBLE_EQ(gate.evaluate(input, kZeroPrev).output.steering_angle_rad, -0.4189);
}

}  // namespace
}  // namespace racer_safety
