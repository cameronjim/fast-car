// L1 tests for racer_control::SpeedRateLimiter (claude-docs/12-testing.md), table-driven
// over the same pass/marginal/fail/garbage shape racer_safety's own gate-logic tests use for
// its analogous rate_limit() function.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "racer_control/speed_rate_limiter.hpp"

namespace racer_control {
namespace {

TEST(SpeedRateLimiter, StartsAtZeroPreviousSpeed) {
  SpeedRateLimiter limiter(9.51);
  EXPECT_DOUBLE_EQ(limiter.previous_speed_mps(), 0.0);
}

TEST(SpeedRateLimiter, FirstCallWithNoElapsedTimeHoldsAtZeroEvenIfRawSpeedIsHigh) {
  SpeedRateLimiter limiter(9.51);
  const double out = limiter.limit(/*raw_speed_mps=*/5.8, /*dt_s=*/0.0);
  EXPECT_DOUBLE_EQ(out, 0.0);
  EXPECT_DOUBLE_EQ(limiter.previous_speed_mps(), 0.0);
}

TEST(SpeedRateLimiter, NegativeDtTreatedAsNoElapsedTime) {
  SpeedRateLimiter limiter(9.51);
  const double out = limiter.limit(5.8, -1.0);
  EXPECT_DOUBLE_EQ(out, 0.0);
}

TEST(SpeedRateLimiter, NonFiniteDtTreatedAsNoElapsedTime) {
  SpeedRateLimiter limiter(9.51);
  EXPECT_DOUBLE_EQ(limiter.limit(5.8, std::numeric_limits<double>::quiet_NaN()), 0.0);
  EXPECT_DOUBLE_EQ(limiter.limit(5.8, std::numeric_limits<double>::infinity()), 0.0);
}

TEST(SpeedRateLimiter, AccelerationWithinBoundPassesThroughUnclamped) {
  SpeedRateLimiter limiter(10.0);
  // previous=0, dt=0.02 -> max_delta=0.2; requesting a 0.1 increase should pass through.
  const double out = limiter.limit(0.1, 0.02);
  EXPECT_DOUBLE_EQ(out, 0.1);
}

TEST(SpeedRateLimiter, AccelerationMarginalExactlyAtBoundPassesThrough) {
  SpeedRateLimiter limiter(10.0);
  // max_delta = 10.0 * 0.02 = 0.2 exactly.
  const double out = limiter.limit(0.2, 0.02);
  EXPECT_DOUBLE_EQ(out, 0.2);
}

TEST(SpeedRateLimiter, AccelerationOverBoundIsClampedToMaxDelta) {
  SpeedRateLimiter limiter(10.0);
  // max_delta = 0.2; requesting 5.8 from 0 should clamp to 0.2.
  const double out = limiter.limit(5.8, 0.02);
  EXPECT_DOUBLE_EQ(out, 0.2);
  EXPECT_DOUBLE_EQ(limiter.previous_speed_mps(), 0.2);
}

TEST(SpeedRateLimiter, RepeatedCallsRampTowardTheRawTargetOverMultipleCycles) {
  SpeedRateLimiter limiter(10.0);  // max_delta = 0.2 per 0.02s cycle
  double out = 0.0;
  for (int i = 0; i < 29; ++i) {
    out = limiter.limit(5.8, 0.02);
  }
  // 29 cycles * 0.2 m/s = 5.8 m/s -- should have just reached the raw target.
  EXPECT_NEAR(out, 5.8, 1e-9);
  // One more cycle at the same raw target must not overshoot past it.
  out = limiter.limit(5.8, 0.02);
  EXPECT_DOUBLE_EQ(out, 5.8);
}

TEST(SpeedRateLimiter, DecelerationIsNeverRateLimitedEvenFromHighSpeed) {
  SpeedRateLimiter limiter(1.0);  // a tiny accel bound that would obviously clamp an increase
  limiter.limit(15.0, 100.0);     // ramp (effectively unclamped, huge dt) up to 15.0 first
  ASSERT_DOUBLE_EQ(limiter.previous_speed_mps(), 15.0);
  const double out = limiter.limit(/*raw_speed_mps=*/0.0, /*dt_s=*/0.001);
  EXPECT_DOUBLE_EQ(out, 0.0);
}

TEST(SpeedRateLimiter, HoldingSpeedExactlyIsNotTreatedAsAcceleration) {
  SpeedRateLimiter limiter(10.0);
  limiter.limit(3.0, 1.0);
  const double out = limiter.limit(3.0, 0.0);  // even with dt=0, holding is not an increase
  EXPECT_DOUBLE_EQ(out, 3.0);
}

}  // namespace
}  // namespace racer_control
