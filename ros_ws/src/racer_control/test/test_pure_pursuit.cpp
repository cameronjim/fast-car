// L1 tests for racer_control::PurePursuitController (claude-docs/12-testing.md), including
// the sign-convention cases from claude-docs/06-vehicle-params.md taken verbatim: steering
// angle is the road-wheel angle in radians, LEFT positive; REP-103 frames (x forward, y
// left, z up; yaw counter-clockwise positive).
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "racer_control/pure_pursuit.hpp"

namespace racer_control {
namespace {

PurePursuitConfig default_config() {
  PurePursuitConfig config;
  config.wheelbase_m = 0.3302;  // config/vehicle_params.yaml chassis.wheelbase_m
  config.lookahead_min_m = 0.4;
  config.lookahead_max_m = 1.5;
  config.lookahead_curvature_ref_1pm = 0.4;
  config.max_steering_angle_rad = 0.4189;  // config/vehicle_params.yaml steering.max_angle_rad
  return config;
}

// A raceline the vehicle sits exactly at the start of, running straight down +x, far
// enough that the lookahead point is always well past the vehicle -- isolates the steering
// geometry from raceline curvature effects for the sign-convention cases below.
Raceline straight_raceline() {
  std::vector<RacelinePoint> points;
  for (int i = 0; i <= 20; ++i) {
    points.push_back(RacelinePoint{
        /*s_m=*/static_cast<double>(i),
        /*x_m=*/static_cast<double>(i),
        /*y_m=*/0.0,
        /*heading_rad=*/0.0,
        /*curvature_1pm=*/0.0,
        /*target_speed_mps=*/4.0,
    });
  }
  return Raceline(points);
}

// -- Sign-convention cases (06-vehicle-params.md, verbatim: left turn positive steering,
//    REP-103 y-left) --------------------------------------------------------------------

TEST(PurePursuitController, TargetToTheLeftProducesPositiveSteering) {
  // A raceline curving to the vehicle's left (positive y) from a vehicle at the origin
  // facing +x: the lookahead target has positive y in the vehicle frame, so steering must
  // be positive (left), per 06-vehicle-params.md's "LEFT positive" convention.
  std::vector<RacelinePoint> points = {
      RacelinePoint{0.0, 0.0, 0.0, 0.0, 0.0, 3.0},
      RacelinePoint{1.0, 1.0, 0.5, 0.0, 0.0, 3.0},
      RacelinePoint{2.0, 2.0, 1.0, 0.0, 0.0, 3.0},
  };
  Raceline raceline(points);
  PurePursuitController controller(default_config());

  PurePursuitCommand cmd = controller.compute_command(raceline, 0.0, 0.0, 0.0);
  EXPECT_GT(cmd.steering_angle_rad, 0.0)
      << "target to the left must yield positive (left) steering";
}

TEST(PurePursuitController, TargetToTheRightProducesNegativeSteering) {
  std::vector<RacelinePoint> points = {
      RacelinePoint{0.0, 0.0, 0.0, 0.0, 0.0, 3.0},
      RacelinePoint{1.0, 1.0, -0.5, 0.0, 0.0, 3.0},
      RacelinePoint{2.0, 2.0, -1.0, 0.0, 0.0, 3.0},
  };
  Raceline raceline(points);
  PurePursuitController controller(default_config());

  PurePursuitCommand cmd = controller.compute_command(raceline, 0.0, 0.0, 0.0);
  EXPECT_LT(cmd.steering_angle_rad, 0.0)
      << "target to the right must yield negative (right) steering";
}

TEST(PurePursuitController, TargetDeadAheadProducesZeroSteering) {
  Raceline raceline = straight_raceline();
  PurePursuitController controller(default_config());

  PurePursuitCommand cmd = controller.compute_command(raceline, 0.0, 0.0, 0.0);
  EXPECT_NEAR(cmd.steering_angle_rad, 0.0, 1e-9);
}

TEST(PurePursuitController, YawRotationIsHonoredForSteeringSign) {
  // Vehicle facing +y (yaw = +90 deg, a left turn from facing +x per CCW-positive yaw).
  // A target further along +y than the vehicle sits directly ahead of it (zero steering),
  // matching the frame-transform logic rather than a raw world-frame left/right check.
  const double half_pi = std::atan(1.0) * 2.0;
  std::vector<RacelinePoint> points = {
      RacelinePoint{0.0, 0.0, 0.0, half_pi, 0.0, 3.0},
      RacelinePoint{1.0, 0.0, 1.0, half_pi, 0.0, 3.0},
      RacelinePoint{2.0, 0.0, 2.0, half_pi, 0.0, 3.0},
  };
  Raceline raceline(points);
  PurePursuitController controller(default_config());

  PurePursuitCommand cmd = controller.compute_command(raceline, 0.0, 0.0, half_pi);
  EXPECT_NEAR(cmd.steering_angle_rad, 0.0, 1e-9);
}

// -- Curvature-adaptive lookahead ---------------------------------------------------------

TEST(PurePursuitController, LookaheadIsMaxOnStraights) {
  PurePursuitController controller(default_config());
  EXPECT_DOUBLE_EQ(controller.lookahead_distance_m(0.0), 1.5);
}

TEST(PurePursuitController, LookaheadIsMinAtOrBeyondCurvatureRef) {
  PurePursuitController controller(default_config());
  EXPECT_DOUBLE_EQ(controller.lookahead_distance_m(0.4), 0.4);
  EXPECT_DOUBLE_EQ(controller.lookahead_distance_m(2.0), 0.4);   // clamped, not extrapolated
  EXPECT_DOUBLE_EQ(controller.lookahead_distance_m(-2.0), 0.4);  // sign-independent magnitude
}

TEST(PurePursuitController, LookaheadInterpolatesLinearlyBetweenMinAndMax) {
  PurePursuitController controller(default_config());
  // Halfway to the curvature reference: halfway between max and min lookahead.
  EXPECT_DOUBLE_EQ(controller.lookahead_distance_m(0.2), 1.5 - 0.5 * (1.5 - 0.4));
}

// -- Steering saturation --------------------------------------------------------------

TEST(PurePursuitController, SteeringSaturatesAtMaxAngle) {
  // A target almost beside the vehicle (huge required curvature) must clamp to the
  // configured max steering angle, never exceed it.
  std::vector<RacelinePoint> points = {
      RacelinePoint{0.0, 0.0, 0.0, 0.0, 0.0, 3.0},
      RacelinePoint{1.0, 0.05, 0.5, 0.0, 0.0, 3.0},
  };
  Raceline raceline(points);
  PurePursuitConfig config = default_config();
  config.lookahead_min_m = 0.05;
  config.lookahead_max_m = 0.05;
  PurePursuitController controller(config);

  PurePursuitCommand cmd = controller.compute_command(raceline, 0.0, 0.0, 0.0);
  EXPECT_NEAR(std::abs(cmd.steering_angle_rad), config.max_steering_angle_rad, 1e-9);
  EXPECT_GT(cmd.steering_angle_rad, 0.0);
}

// -- Speed command ----------------------------------------------------------------------

TEST(PurePursuitController, SpeedCommandMatchesNearestPointTargetSpeed) {
  std::vector<RacelinePoint> points = {
      RacelinePoint{0.0, 0.0, 0.0, 0.0, 0.0, 3.0},
      RacelinePoint{1.0, 1.0, 0.0, 0.0, 0.0, 7.5},
      RacelinePoint{2.0, 2.0, 0.0, 0.0, 0.0, 3.0},
  };
  Raceline raceline(points);
  PurePursuitController controller(default_config());

  PurePursuitCommand cmd = controller.compute_command(raceline, 1.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(cmd.speed_mps, 7.5);
}

// -- yaw_from_quaternion ------------------------------------------------------------------

TEST(YawFromQuaternion, IdentityQuaternionIsZeroYaw) {
  EXPECT_NEAR(yaw_from_quaternion(1.0, 0.0, 0.0, 0.0), 0.0, 1e-9);
}

TEST(YawFromQuaternion, NinetyDegreeYawMatchesHalfPi) {
  const double half_pi = std::atan(1.0) * 2.0;
  // Quaternion for a +90 deg (CCW) rotation about z: (w, x, y, z) = (cos(45deg), 0, 0, sin(45deg)).
  const double c = std::cos(half_pi / 2.0);
  const double s = std::sin(half_pi / 2.0);
  EXPECT_NEAR(yaw_from_quaternion(c, 0.0, 0.0, s), half_pi, 1e-9);
}

TEST(YawFromQuaternion, NegativeNinetyDegreeYawMatchesNegativeHalfPi) {
  const double half_pi = std::atan(1.0) * 2.0;
  const double c = std::cos(-half_pi / 2.0);
  const double s = std::sin(-half_pi / 2.0);
  EXPECT_NEAR(yaw_from_quaternion(c, 0.0, 0.0, s), -half_pi, 1e-9);
}

}  // namespace
}  // namespace racer_control
