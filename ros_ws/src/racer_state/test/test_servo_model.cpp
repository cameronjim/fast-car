// L1 tests for racer_state::ServoModel (claude-docs/12-testing.md, normal package). The
// limits below are a test fixture shaped like vehicle_params' steering section, not a copy of
// it: the node reads the real values from the generated binding.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "racer_state/servo_model.hpp"

namespace racer_state {
namespace {

ServoModelConfig fixture(std::optional<double> tau = std::nullopt) {
  ServoModelConfig config;
  config.min_angle_rad = -0.4;
  config.max_angle_rad = 0.4;
  config.min_rate_rad_per_s = -2.0;
  config.max_rate_rad_per_s = 2.0;
  config.time_constant_s = tau;
  return config;
}

TEST(ServoModel, StartsStraight) {
  ServoModel servo(fixture());
  EXPECT_DOUBLE_EQ(servo.angle_rad(), 0.0);
  EXPECT_DOUBLE_EQ(servo.target_rad(), 0.0);
}

TEST(ServoModel, RateLimitOnlyWithoutTimeConstant) {
  ServoModel servo(fixture());
  // 2 rad/s for 0.05 s = 0.1 rad towards a 0.3 rad (left) target.
  EXPECT_NEAR(servo.step(0.3, 0.05), 0.1, 1e-12);
  EXPECT_NEAR(servo.step(0.3, 0.05), 0.2, 1e-12);
  EXPECT_NEAR(servo.step(0.3, 0.05), 0.3, 1e-12);
  // Reached: stays there, no overshoot.
  EXPECT_NEAR(servo.step(0.3, 0.05), 0.3, 1e-12);
}

TEST(ServoModel, RateLimitIsSymmetricForRightTurns) {
  ServoModel servo(fixture());
  EXPECT_NEAR(servo.step(-0.3, 0.05), -0.1, 1e-12);
}

TEST(ServoModel, AsymmetricRatesAreRespected) {
  ServoModelConfig config = fixture();
  config.min_rate_rad_per_s = -1.0;
  ServoModel servo(config);
  EXPECT_NEAR(servo.step(-0.3, 0.1), -0.1, 1e-12);
  EXPECT_NEAR(servo.step(0.3, 0.1), 0.1, 1e-12);
}

TEST(ServoModel, SmallStepsReachTargetExactly) {
  ServoModel servo(fixture());
  EXPECT_NEAR(servo.step(0.01, 0.1), 0.01, 1e-12);
}

TEST(ServoModel, CommandIsClampedToRackLimits) {
  ServoModel servo(fixture());
  for (int i = 0; i < 100; ++i) {
    servo.step(5.0, 0.02);
  }
  EXPECT_DOUBLE_EQ(servo.angle_rad(), 0.4);
  EXPECT_DOUBLE_EQ(servo.target_rad(), 0.4);
  for (int i = 0; i < 100; ++i) {
    servo.step(-5.0, 0.02);
  }
  EXPECT_DOUBLE_EQ(servo.angle_rad(), -0.4);
}

TEST(ServoModel, FirstOrderLagFollowsExponential) {
  // tau 0.1 s, a step small enough that the rate limit never binds.
  ServoModel servo(fixture(0.1));
  const double target = 0.05;
  const double after = servo.step(target, 0.1);
  EXPECT_NEAR(after, target * (1.0 - std::exp(-1.0)), 1e-12);
  // Many small steps agree with one big one (exact discretisation).
  ServoModel fine(fixture(0.1));
  for (int i = 0; i < 100; ++i) {
    fine.step(target, 0.001);
  }
  EXPECT_NEAR(fine.angle_rad(), after, 1e-12);
}

TEST(ServoModel, RateLimitBindsOnTopOfLag) {
  // tau 0.01 s wants almost all of 0.4 rad in 0.05 s; the rate limit allows only 0.1 rad.
  ServoModel servo(fixture(0.01));
  EXPECT_NEAR(servo.step(0.4, 0.05), 0.1, 1e-12);
}

TEST(ServoModel, NonPositiveOrNonFiniteDtChangesNothing) {
  ServoModel servo(fixture());
  servo.step(0.3, 0.05);
  const double before = servo.angle_rad();
  EXPECT_DOUBLE_EQ(servo.step(0.3, 0.0), before);
  EXPECT_DOUBLE_EQ(servo.step(0.3, -1.0), before);
  EXPECT_DOUBLE_EQ(servo.step(0.3, std::numeric_limits<double>::quiet_NaN()), before);
  EXPECT_DOUBLE_EQ(servo.step(0.3, std::numeric_limits<double>::infinity()), before);
  // ...but the command still becomes the target.
  EXPECT_DOUBLE_EQ(servo.target_rad(), 0.3);
}

TEST(ServoModel, NonFiniteCommandKeepsPreviousTarget) {
  ServoModel servo(fixture());
  servo.step(0.2, 0.01);
  servo.step(std::numeric_limits<double>::quiet_NaN(), 0.05);
  EXPECT_DOUBLE_EQ(servo.target_rad(), 0.2);
  EXPECT_NEAR(servo.angle_rad(), 0.12, 1e-12);
}

TEST(ServoModel, ResetClampsAndSetsTarget) {
  ServoModel servo(fixture());
  servo.reset(1.0);
  EXPECT_DOUBLE_EQ(servo.angle_rad(), 0.4);
  EXPECT_DOUBLE_EQ(servo.target_rad(), 0.4);
  servo.reset(std::numeric_limits<double>::quiet_NaN());
  EXPECT_DOUBLE_EQ(servo.angle_rad(), 0.0);
}

TEST(ServoModel, RejectsBadConfig) {
  ServoModelConfig config = fixture();
  config.min_angle_rad = 0.4;
  EXPECT_THROW(ServoModel{config}, std::invalid_argument);
  config = fixture();
  config.max_rate_rad_per_s = 0.0;
  EXPECT_THROW(ServoModel{config}, std::invalid_argument);
  config = fixture();
  config.min_rate_rad_per_s = 0.5;
  EXPECT_THROW(ServoModel{config}, std::invalid_argument);
  config = fixture();
  config.max_angle_rad = std::numeric_limits<double>::infinity();
  EXPECT_THROW(ServoModel{config}, std::invalid_argument);
  EXPECT_THROW(ServoModel{fixture(0.0)}, std::invalid_argument);
  EXPECT_THROW(ServoModel{fixture(-0.1)}, std::invalid_argument);
  EXPECT_THROW(ServoModel{fixture(std::numeric_limits<double>::quiet_NaN())},
               std::invalid_argument);
}

}  // namespace
}  // namespace racer_state
