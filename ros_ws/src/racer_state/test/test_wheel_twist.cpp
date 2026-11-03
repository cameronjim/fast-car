// L1 tests for racer_state::compute_wheel_twist and WheelTwistEstimator (claude-docs/12-testing
// .md, normal package): hand-computed kinematics, the sign conventions of
// claude-docs/06-vehicle-params.md (steering LEFT positive, yaw counter-clockwise positive),
// reverse, zero speed, stale and garbage inputs. The numbers below are a test fixture, not a
// copy of vehicle_params.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "racer_state/wheel_twist.hpp"

namespace racer_state {
namespace {

constexpr double kWheelbase = 0.33;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

WheelTwistConfig twist_fixture() {
  WheelTwistConfig config;
  config.wheelbase_m = kWheelbase;
  config.speed_std_floor_mps = 0.02;
  config.speed_std_fraction = 0.03;
  config.steering_std_rad = 0.03;
  config.stale_steering_std_rad = 0.4;
  config.yaw_rate_std_floor_rad_per_s = 0.02;
  config.lateral_std_mps = 0.05;
  return config;
}

ServoModelConfig servo_fixture() {
  ServoModelConfig config;
  config.min_angle_rad = -0.4;
  config.max_angle_rad = 0.4;
  config.min_rate_rad_per_s = -3.0;
  config.max_rate_rad_per_s = 3.0;
  return config;
}

TEST(ComputeWheelTwist, StraightAhead) {
  const auto twist = compute_wheel_twist(2.0, 0.0, true, twist_fixture());
  ASSERT_TRUE(twist.has_value());
  EXPECT_DOUBLE_EQ(twist->vx_mps, 2.0);
  EXPECT_DOUBLE_EQ(twist->vy_mps, 0.0);
  EXPECT_DOUBLE_EQ(twist->wz_rad_per_s, 0.0);
}

TEST(ComputeWheelTwist, LeftSteerForwardGivesPositiveYawRate) {
  // v tan(delta) / L = 1.5 * tan(0.2) / 0.33.
  const auto twist = compute_wheel_twist(1.5, 0.2, true, twist_fixture());
  ASSERT_TRUE(twist.has_value());
  EXPECT_NEAR(twist->wz_rad_per_s, 1.5 * std::tan(0.2) / kWheelbase, 1e-12);
  EXPECT_GT(twist->wz_rad_per_s, 0.0);
}

TEST(ComputeWheelTwist, RightSteerForwardGivesNegativeYawRate) {
  const auto twist = compute_wheel_twist(1.5, -0.2, true, twist_fixture());
  ASSERT_TRUE(twist.has_value());
  EXPECT_NEAR(twist->wz_rad_per_s, -1.5 * std::tan(0.2) / kWheelbase, 1e-12);
}

TEST(ComputeWheelTwist, ReverseWithLeftSteerTurnsClockwise) {
  const auto twist = compute_wheel_twist(-0.5, 0.3, true, twist_fixture());
  ASSERT_TRUE(twist.has_value());
  EXPECT_DOUBLE_EQ(twist->vx_mps, -0.5);
  EXPECT_NEAR(twist->wz_rad_per_s, -0.5 * std::tan(0.3) / kWheelbase, 1e-12);
  EXPECT_LT(twist->wz_rad_per_s, 0.0);
}

TEST(ComputeWheelTwist, TurningRadiusMatchesBicycleGeometry) {
  // R = v / wz = L / tan(delta), whatever the speed.
  for (double v : {0.3, 1.0, 2.9}) {
    const auto twist = compute_wheel_twist(v, 0.35, true, twist_fixture());
    ASSERT_TRUE(twist.has_value());
    EXPECT_NEAR(v / twist->wz_rad_per_s, kWheelbase / std::tan(0.35), 1e-12);
  }
}

TEST(ComputeWheelTwist, ZeroSpeedHasZeroYawRateAndFloorVariances) {
  const auto twist = compute_wheel_twist(0.0, 0.4, false, twist_fixture());
  ASSERT_TRUE(twist.has_value());
  EXPECT_DOUBLE_EQ(twist->wz_rad_per_s, 0.0);
  EXPECT_NEAR(twist->var_vx, 0.02 * 0.02, 1e-15);
  // Stale steering does not matter at standstill: the only yaw term left is the floor plus the
  // speed-error term.
  const double d_wz_d_v = std::tan(0.4) / kWheelbase;
  EXPECT_NEAR(twist->var_wz, d_wz_d_v * d_wz_d_v * 0.02 * 0.02 + 0.02 * 0.02, 1e-15);
  EXPECT_NEAR(twist->var_vy, 0.05 * 0.05, 1e-15);
}

TEST(ComputeWheelTwist, VariancesMatchHandComputation) {
  const double v = 2.0;
  const double delta = 0.1;
  const auto twist = compute_wheel_twist(v, delta, true, twist_fixture());
  ASSERT_TRUE(twist.has_value());
  const double speed_std = 0.02 + 0.03 * v;
  EXPECT_NEAR(twist->var_vx, speed_std * speed_std, 1e-15);
  const double c = std::cos(delta);
  const double a = v / (kWheelbase * c * c);
  const double b = std::tan(delta) / kWheelbase;
  EXPECT_NEAR(twist->var_wz, a * a * 0.03 * 0.03 + b * b * speed_std * speed_std + 0.02 * 0.02,
              1e-15);
}

TEST(ComputeWheelTwist, SpeedVarianceIsSymmetricInReverse) {
  const auto fwd = compute_wheel_twist(1.0, 0.2, true, twist_fixture());
  const auto rev = compute_wheel_twist(-1.0, 0.2, true, twist_fixture());
  ASSERT_TRUE(fwd.has_value() && rev.has_value());
  EXPECT_DOUBLE_EQ(fwd->var_vx, rev->var_vx);
  EXPECT_DOUBLE_EQ(fwd->var_wz, rev->var_wz);
}

TEST(ComputeWheelTwist, StaleSteeringRaisesYawVarianceOnly) {
  const auto fresh = compute_wheel_twist(2.0, 0.1, true, twist_fixture());
  const auto stale = compute_wheel_twist(2.0, 0.1, false, twist_fixture());
  ASSERT_TRUE(fresh.has_value() && stale.has_value());
  EXPECT_DOUBLE_EQ(fresh->wz_rad_per_s, stale->wz_rad_per_s);
  EXPECT_DOUBLE_EQ(fresh->var_vx, stale->var_vx);
  EXPECT_GT(stale->var_wz, 50.0 * fresh->var_wz);
}

TEST(ComputeWheelTwist, NonFiniteInputsGiveNothing) {
  EXPECT_FALSE(compute_wheel_twist(kNaN, 0.0, true, twist_fixture()).has_value());
  EXPECT_FALSE(compute_wheel_twist(kInf, 0.0, true, twist_fixture()).has_value());
  EXPECT_FALSE(compute_wheel_twist(1.0, kNaN, true, twist_fixture()).has_value());
}

TEST(WheelTwistConfig, ValidateRejectsNonsense) {
  WheelTwistConfig config = twist_fixture();
  EXPECT_NO_THROW(validate(config));
  config.wheelbase_m = 0.0;
  EXPECT_THROW(validate(config), std::invalid_argument);
  config = twist_fixture();
  config.wheelbase_m = kNaN;
  EXPECT_THROW(validate(config), std::invalid_argument);
  config = twist_fixture();
  config.lateral_std_mps = -0.1;
  EXPECT_THROW(validate(config), std::invalid_argument);
  config = twist_fixture();
  config.steering_std_rad = kInf;
  EXPECT_THROW(validate(config), std::invalid_argument);
}

TEST(WheelTwistEstimator, RejectsBadTimeout) {
  EXPECT_THROW(WheelTwistEstimator(servo_fixture(), twist_fixture(), 0.0), std::invalid_argument);
  EXPECT_THROW(WheelTwistEstimator(servo_fixture(), twist_fixture(), kNaN), std::invalid_argument);
}

TEST(WheelTwistEstimator, NoDriveYetMeansStraightAndStale) {
  WheelTwistEstimator estimator(servo_fixture(), twist_fixture(), 0.25);
  EXPECT_TRUE(estimator.steering_stale(0.0));
  const auto twist = estimator.on_wheel_speed(1.0, 10.0, 0.0);
  ASSERT_TRUE(twist.has_value());
  EXPECT_DOUBLE_EQ(twist->wz_rad_per_s, 0.0);
  const auto fresh_equivalent = compute_wheel_twist(1.0, 0.0, false, twist_fixture());
  EXPECT_DOUBLE_EQ(twist->var_wz, fresh_equivalent->var_wz);
}

TEST(WheelTwistEstimator, ServoLagShowsUpInYawRate) {
  // 3 rad/s rate limit: a 0.3 rad left command takes 0.1 s to reach. Wheel samples at 50 Hz.
  WheelTwistEstimator estimator(servo_fixture(), twist_fixture(), 0.25);
  estimator.on_wheel_speed(1.0, 0.0, 0.0);
  estimator.on_drive(0.3, 0.0);
  const auto first = estimator.on_wheel_speed(1.0, 0.02, 0.02);
  ASSERT_TRUE(first.has_value());
  EXPECT_NEAR(estimator.steering_estimate_rad(), 0.06, 1e-12);
  EXPECT_NEAR(first->wz_rad_per_s, std::tan(0.06) / kWheelbase, 1e-12);
  double t = 0.02;
  for (int i = 0; i < 10; ++i) {
    t += 0.02;
    estimator.on_drive(0.3, t);
    estimator.on_wheel_speed(1.0, t, t);
  }
  EXPECT_NEAR(estimator.steering_estimate_rad(), 0.3, 1e-12);
}

TEST(WheelTwistEstimator, DriveGoesStaleAfterTimeout) {
  WheelTwistEstimator estimator(servo_fixture(), twist_fixture(), 0.25);
  estimator.on_drive(0.2, 1.0);
  EXPECT_FALSE(estimator.steering_stale(1.0));
  EXPECT_FALSE(estimator.steering_stale(1.25));
  EXPECT_TRUE(estimator.steering_stale(1.26));
  // The last command is still the servo's target while stale; only the variance changes.
  estimator.on_wheel_speed(1.0, 0.0, 1.3);
  const auto stale = estimator.on_wheel_speed(1.0, 1.0, 1.4);
  ASSERT_TRUE(stale.has_value());
  EXPECT_NEAR(estimator.steering_estimate_rad(), 0.2, 1e-12);
  const auto expected = compute_wheel_twist(1.0, 0.2, false, twist_fixture());
  EXPECT_DOUBLE_EQ(stale->var_wz, expected->var_wz);
  // And it recovers.
  estimator.on_drive(0.2, 1.5);
  const auto fresh = estimator.on_wheel_speed(1.0, 1.02, 1.5);
  ASSERT_TRUE(fresh.has_value());
  const auto fresh_expected = compute_wheel_twist(1.0, 0.2, true, twist_fixture());
  EXPECT_DOUBLE_EQ(fresh->var_wz, fresh_expected->var_wz);
}

TEST(WheelTwistEstimator, NonFiniteDriveIsIgnored) {
  WheelTwistEstimator estimator(servo_fixture(), twist_fixture(), 0.25);
  estimator.on_drive(0.1, 0.0);
  estimator.on_drive(kNaN, 0.2);
  EXPECT_TRUE(estimator.steering_stale(0.3));
  estimator.on_wheel_speed(1.0, 0.0, 0.3);
  estimator.on_wheel_speed(1.0, 1.0, 0.3);
  EXPECT_NEAR(estimator.steering_estimate_rad(), 0.1, 1e-12);
  estimator.on_drive(0.1, kInf);
  EXPECT_TRUE(estimator.steering_stale(0.3));
}

TEST(WheelTwistEstimator, NonFiniteWheelSampleGivesNothing) {
  WheelTwistEstimator estimator(servo_fixture(), twist_fixture(), 0.25);
  EXPECT_FALSE(estimator.on_wheel_speed(kNaN, 0.0, 0.0).has_value());
  EXPECT_FALSE(estimator.on_wheel_speed(1.0, kNaN, 0.0).has_value());
}

TEST(WheelTwistEstimator, BackwardsStampDoesNotAdvanceServo) {
  WheelTwistEstimator estimator(servo_fixture(), twist_fixture(), 0.25);
  estimator.on_drive(0.3, 0.0);
  estimator.on_wheel_speed(1.0, 5.0, 0.0);
  EXPECT_DOUBLE_EQ(estimator.steering_estimate_rad(), 0.0);
  // Time jumps back: no advance, and 5.0 stays the reference.
  estimator.on_wheel_speed(1.0, 4.0, 0.0);
  EXPECT_DOUBLE_EQ(estimator.steering_estimate_rad(), 0.0);
  estimator.on_wheel_speed(1.0, 5.02, 0.0);
  EXPECT_NEAR(estimator.steering_estimate_rad(), 0.06, 1e-12);
}

}  // namespace
}  // namespace racer_state
