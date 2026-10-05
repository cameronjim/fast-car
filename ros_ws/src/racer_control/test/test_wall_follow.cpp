// L1 tests for racer_control's wall-follow core and PID (GitHub issue 26 item g). Ported
// cases from the old Python test_wall_logic.py are marked "Ported".
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "racer_control/pid.hpp"
#include "racer_control/wall_follow.hpp"

namespace racer_control {
namespace {

constexpr double kDeg = M_PI / 180.0;
constexpr std::size_t kNumRays = 1080;
constexpr double kRangeMin = 0.05;
constexpr double kRangeMax = 30.0;

// The old tests' 1080-ray, 270 degree scan.
ScanInput empty_scan() {
  ScanInput scan;
  scan.ranges.assign(kNumRays, static_cast<float>(kRangeMax));
  scan.angle_min = -135.0 * kDeg;
  scan.angle_increment = 0.25 * kDeg;
  scan.range_min = kRangeMin;
  scan.range_max = kRangeMax;
  return scan;
}

constexpr std::size_t kRightNormal = 180;  // -90 deg
constexpr std::size_t kRightAhead = 460;   // -20 deg
constexpr std::size_t kLeftAhead = 620;    // +20 deg
constexpr std::size_t kLeftNormal = 900;   // +90 deg

WallFollowConfig right_wall(double target_m, double lookahead_m = 0.5) {
  WallFollowConfig c;
  c.ray_a_bearing_rad = -20.0 * kDeg;
  c.ray_b_bearing_rad = -90.0 * kDeg;
  c.target_distance_m = target_m;
  c.lookahead_m = lookahead_m;
  c.deadband_m = 0.02;
  return c;
}

WallFollowConfig left_wall(double target_m, double lookahead_m = 0.5) {
  WallFollowConfig c = right_wall(target_m, lookahead_m);
  c.ray_a_bearing_rad = 20.0 * kDeg;
  c.ray_b_bearing_rad = 90.0 * kDeg;
  return c;
}

// A car running parallel to a wall at distance_m (ray a hits it at d / sin(20 deg)).
ScanInput parallel_wall_scan(double distance_m, bool right = true) {
  ScanInput scan = empty_scan();
  scan.ranges[right ? kRightNormal : kLeftNormal] = static_cast<float>(distance_m);
  scan.ranges[right ? kRightAhead : kLeftAhead] =
      static_cast<float>(distance_m / std::sin(20.0 * kDeg));
  return scan;
}

TEST(WallRaysValid, AcceptsSameSideWithNormalRayFurtherOut) {
  EXPECT_TRUE(wall_rays_valid(right_wall(1.0)));
  EXPECT_TRUE(wall_rays_valid(left_wall(1.0)));
  WallFollowConfig mixed = right_wall(1.0);
  mixed.ray_a_bearing_rad = 0.3;
  EXPECT_FALSE(wall_rays_valid(mixed));
  WallFollowConfig swapped = right_wall(1.0);
  swapped.ray_a_bearing_rad = -90.0 * kDeg;
  swapped.ray_b_bearing_rad = -20.0 * kDeg;
  EXPECT_FALSE(wall_rays_valid(swapped));
  WallFollowConfig zero = right_wall(1.0);
  zero.ray_a_bearing_rad = 0.0;
  EXPECT_FALSE(wall_rays_valid(zero));
  WallFollowConfig nan = right_wall(1.0);
  nan.ray_b_bearing_rad = std::nan("");
  EXPECT_FALSE(wall_rays_valid(nan));
}

TEST(WallSteeringSign, RightWallPositiveLeftWallNegative) {
  EXPECT_DOUBLE_EQ(wall_steering_sign(right_wall(1.0)), 1.0);
  EXPECT_DOUBLE_EQ(wall_steering_sign(left_wall(1.0)), -1.0);
}

// Ported ("known bearings map to known ray indices"), through the vehicle-frame lookup.
TEST(RangeAtVehicleBearing, KnownBearings) {
  ScanInput scan = empty_scan();
  for (std::size_t i = 0; i < kNumRays; ++i) {
    scan.ranges[i] = static_cast<float>(i);
  }
  EXPECT_DOUBLE_EQ(range_at_vehicle_bearing(scan, -135.0 * kDeg, 0.0).value(), 0.0);
  EXPECT_DOUBLE_EQ(range_at_vehicle_bearing(scan, -90.0 * kDeg, 0.0).value(), 180.0);
  EXPECT_DOUBLE_EQ(range_at_vehicle_bearing(scan, -20.0 * kDeg, 0.0).value(), 460.0);
  EXPECT_DOUBLE_EQ(range_at_vehicle_bearing(scan, 0.0, 0.0).value(), 540.0);
}

// Changed from the old "bearings outside the scan clamp to the end rays".
TEST(RangeAtVehicleBearing, OutsideCoverageIsNullopt) {
  const ScanInput scan = empty_scan();
  EXPECT_FALSE(range_at_vehicle_bearing(scan, 135.0 * kDeg, 0.0).has_value());
  EXPECT_FALSE(range_at_vehicle_bearing(scan, -200.0 * kDeg, 0.0).has_value());
  EXPECT_FALSE(range_at_vehicle_bearing(scan, 200.0 * kDeg, 0.0).has_value());
}

// Ported.
TEST(MeasureWall, ParallelWallGivesThePlainDistanceError) {
  const auto m = measure_wall(parallel_wall_scan(1.0), right_wall(1.5));
  ASSERT_TRUE(m);
  EXPECT_NEAR(m->alpha_rad, 0.0, 1e-6);
  EXPECT_NEAR(m->distance_m, 1.0, 1e-6);
  EXPECT_NEAR(m->error_m, 0.5, 1e-6);
}

// Ported.
TEST(MeasureWall, ErrorInsideTheDeadbandReadsAsZero) {
  const auto m = measure_wall(parallel_wall_scan(1.0), right_wall(1.01));
  ASSERT_TRUE(m);
  EXPECT_DOUBLE_EQ(m->error_m, 0.0);
}

TEST(MeasureWall, DeadbandIsAParameter) {
  WallFollowConfig c = right_wall(1.01);
  c.deadband_m = 0.0;
  const auto m = measure_wall(parallel_wall_scan(1.0), c);
  ASSERT_TRUE(m);
  EXPECT_NEAR(m->error_m, 0.01, 1e-6);
}

// Ported ("speed has no effect when the car runs parallel"), now about the lookahead
// distance, which replaced speed * dt.
TEST(MeasureWall, LookaheadHasNoEffectWhenParallel) {
  const auto still = measure_wall(parallel_wall_scan(1.0), right_wall(1.5, 0.0));
  const auto ahead = measure_wall(parallel_wall_scan(1.0), right_wall(1.5, 0.5));
  ASSERT_TRUE(still && ahead);
  EXPECT_NEAR(still->error_m, ahead->error_m, 1e-6);
}

// Ported ("lookahead shrinks the error when the car turns away from the wall").
TEST(MeasureWall, LookaheadShrinksTheErrorWhenHeadingAwayFromTheWall) {
  ScanInput scan = empty_scan();
  scan.ranges[kRightNormal] = 1.0f;
  scan.ranges[kRightAhead] = 4.0f;
  const auto no_lookahead = measure_wall(scan, right_wall(1.5, 0.0));
  const auto with_lookahead = measure_wall(scan, right_wall(1.5, 0.5));
  ASSERT_TRUE(no_lookahead && with_lookahead);
  EXPECT_GT(with_lookahead->alpha_rad, 0.0);
  EXPECT_LT(with_lookahead->error_m, no_lookahead->error_m);
  // Hand computation: alpha = atan((4 cos70 - 1) / (4 sin70)), D = cos(alpha),
  // D_L = D + 0.5 sin(alpha).
  const double theta = 70.0 * kDeg;
  const double alpha = std::atan((4.0 * std::cos(theta) - 1.0) / (4.0 * std::sin(theta)));
  EXPECT_NEAR(with_lookahead->alpha_rad, alpha, 1e-6);
  EXPECT_NEAR(with_lookahead->distance_m, std::cos(alpha), 1e-6);
  EXPECT_NEAR(with_lookahead->lookahead_distance_m, std::cos(alpha) + 0.5 * std::sin(alpha), 1e-6);
}

TEST(MeasureWall, HeadingTowardTheWallGivesNegativeAlpha) {
  ScanInput scan = empty_scan();
  scan.ranges[kRightNormal] = 1.0f;
  scan.ranges[kRightAhead] = 2.0f;  // shorter than parallel (2.92 m)
  const auto m = measure_wall(scan, right_wall(1.0, 0.5));
  ASSERT_TRUE(m);
  EXPECT_LT(m->alpha_rad, 0.0);
  EXPECT_LT(m->lookahead_distance_m, m->distance_m);
}

// Ported ("out of range rays report no error"): now no measurement at all.
TEST(MeasureWall, OutOfRangeRaysGiveNoMeasurement) {
  ScanInput scan = empty_scan();
  scan.ranges[kRightNormal] = static_cast<float>(kRangeMax + 5.0);
  scan.ranges[kRightAhead] = 2.0f;
  EXPECT_FALSE(measure_wall(scan, right_wall(1.0)));
}

TEST(MeasureWall, InvalidReturnsGiveNoMeasurement) {
  for (float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(),
                    0.0f, 0.01f}) {
    ScanInput scan = parallel_wall_scan(1.0);
    scan.ranges[kRightAhead] = bad;
    EXPECT_FALSE(measure_wall(scan, right_wall(1.0))) << bad;
  }
}

TEST(MeasureWall, RayOutsideCoverageGivesNoMeasurement) {
  WallFollowConfig c = right_wall(1.0);
  c.ray_b_bearing_rad = -170.0 * kDeg;  // behind the 270 degree scan's edge
  EXPECT_FALSE(measure_wall(parallel_wall_scan(1.0), c));
}

// The old "missing velocity is treated as a standstill" case is gone: the lookahead is a
// distance parameter now, so the measurement no longer takes a speed at all.

// -- Sign convention against LEFT POSITIVE steering (claude-docs/06-vehicle-params.md) ------

TEST(WallFollowSign, TooFarFromTheRightWallSteersRight) {
  const WallFollowConfig c = right_wall(0.6);
  const auto m = measure_wall(parallel_wall_scan(1.0), c);
  ASSERT_TRUE(m);
  EXPECT_LT(m->error_m, 0.0);
  Pid pid(PidGains{2.0, 0.0, 0.0, 1.0});
  const double steering = wall_steering_sign(c) * pid.update(m->error_m, 0.0);
  EXPECT_LT(steering, 0.0);
}

TEST(WallFollowSign, TooCloseToTheRightWallSteersLeft) {
  const WallFollowConfig c = right_wall(1.0);
  const auto m = measure_wall(parallel_wall_scan(0.5), c);
  ASSERT_TRUE(m);
  Pid pid(PidGains{2.0, 0.0, 0.0, 1.0});
  EXPECT_GT(wall_steering_sign(c) * pid.update(m->error_m, 0.0), 0.0);
}

TEST(WallFollowSign, TooFarFromTheLeftWallSteersLeft) {
  const WallFollowConfig c = left_wall(0.6);
  const auto m = measure_wall(parallel_wall_scan(1.0, /*right=*/false), c);
  ASSERT_TRUE(m);
  EXPECT_NEAR(m->distance_m, 1.0, 1e-6);
  EXPECT_LT(m->error_m, 0.0);
  Pid pid(PidGains{2.0, 0.0, 0.0, 1.0});
  EXPECT_GT(wall_steering_sign(c) * pid.update(m->error_m, 0.0), 0.0);
}

TEST(WallFollowSign, RearFacingFullCircleLidar) {
  // 360 scan from -pi, LiDAR yawed pi. Vehicle bearing -90 deg (right) is laser +90 deg,
  // vehicle -20 deg is laser +160 deg.
  ScanInput scan;
  scan.ranges.assign(360, 12.0f);
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / 360.0;
  scan.range_min = kRangeMin;
  scan.range_max = 12.0;
  scan.ranges[270] = 1.0f;                                             // laser +90 deg
  scan.ranges[340] = static_cast<float>(1.0 / std::sin(20.0 * kDeg));  // laser +160 deg
  WallFollowConfig c = right_wall(0.6);
  c.laser_yaw_offset_rad = M_PI;
  const auto m = measure_wall(scan, c);
  ASSERT_TRUE(m);
  EXPECT_NEAR(m->distance_m, 1.0, 1e-5);
  EXPECT_LT(m->error_m, 0.0);
}

// -- PID ------------------------------------------------------------------------------------

TEST(Pid, FirstSampleIsProportionalOnly) {
  Pid pid(PidGains{2.0, 1.0, 5.0, 10.0});
  // No magic first-step dt: even with a plausible dt supplied, there is no previous sample
  // to integrate from or differentiate against.
  EXPECT_DOUBLE_EQ(pid.update(0.5, 0.02), 1.0);
  EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
}

TEST(Pid, IntegralAndDerivativeFromTheSecondSample) {
  Pid pid(PidGains{2.0, 1.0, 0.1, 10.0});
  pid.update(0.5, 0.0);
  // integral = 0.7 * 0.1 = 0.07, derivative = (0.7 - 0.5) / 0.1 = 2.0
  EXPECT_NEAR(pid.update(0.7, 0.1), 2.0 * 0.7 + 1.0 * 0.07 + 0.1 * 2.0, 1e-12);
  EXPECT_NEAR(pid.integral(), 0.07, 1e-12);
}

TEST(Pid, IntegralIsClamped) {
  Pid pid(PidGains{0.0, 1.0, 0.0, 0.25});
  pid.update(1.0, 0.0);
  for (int i = 0; i < 100; ++i) {
    pid.update(1.0, 0.1);
  }
  EXPECT_DOUBLE_EQ(pid.integral(), 0.25);
  for (int i = 0; i < 100; ++i) {
    pid.update(-1.0, 0.1);
  }
  EXPECT_DOUBLE_EQ(pid.integral(), -0.25);
}

TEST(Pid, NonPositiveOrNonFiniteDtSkipsIntegralAndDerivative) {
  Pid pid(PidGains{1.0, 1.0, 1.0, 10.0});
  pid.update(0.5, 0.0);
  pid.update(0.5, 0.1);  // integral 0.05
  EXPECT_DOUBLE_EQ(pid.update(0.9, 0.0), 0.9 + 0.05);
  EXPECT_DOUBLE_EQ(pid.update(0.9, -1.0), 0.9 + 0.05);
  EXPECT_DOUBLE_EQ(pid.update(0.9, std::nan("")), 0.9 + 0.05);
}

TEST(Pid, NonFiniteErrorReturnsZeroAndKeepsState) {
  Pid pid(PidGains{1.0, 1.0, 1.0, 10.0});
  pid.update(0.5, 0.0);
  pid.update(0.5, 0.1);
  EXPECT_DOUBLE_EQ(pid.update(std::nan(""), 0.1), 0.0);
  EXPECT_DOUBLE_EQ(pid.integral(), 0.05);
}

TEST(Pid, ResetClearsStateAndSkipsTheNextDerivative) {
  Pid pid(PidGains{1.0, 1.0, 1.0, 10.0});
  pid.update(0.5, 0.0);
  pid.update(0.5, 0.1);
  pid.reset();
  EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
  EXPECT_DOUBLE_EQ(pid.update(2.0, 0.1), 2.0);
}

}  // namespace
}  // namespace racer_control
