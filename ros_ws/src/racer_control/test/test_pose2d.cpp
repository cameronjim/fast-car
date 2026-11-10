// L1 tests for racer_control::compose (tracker_node's map-frame pose).
#include <gtest/gtest.h>

#include <cmath>

#include "racer_control/pose2d.hpp"

namespace racer_control {
namespace {

TEST(Pose2D, IdentityLeavesPoseAlone) {
  const Pose2D pose{1.0, -2.0, 0.5};
  const Pose2D out = compose(Pose2D{}, pose);
  EXPECT_DOUBLE_EQ(out.x_m, 1.0);
  EXPECT_DOUBLE_EQ(out.y_m, -2.0);
  EXPECT_NEAR(out.yaw_rad, 0.5, 1e-12);
  const Pose2D back = compose(pose, Pose2D{});
  EXPECT_DOUBLE_EQ(back.x_m, 1.0);
  EXPECT_DOUBLE_EQ(back.y_m, -2.0);
}

TEST(Pose2D, PureTranslationAdds) {
  const Pose2D out = compose(Pose2D{0.0, 0.5, 0.0}, Pose2D{0.3, 0.0, 0.1});
  EXPECT_DOUBLE_EQ(out.x_m, 0.3);
  EXPECT_DOUBLE_EQ(out.y_m, 0.5);
  EXPECT_NEAR(out.yaw_rad, 0.1, 1e-12);
}

TEST(Pose2D, RotationTurnsTheChildOffsetCounterClockwise) {
  // odom rotated +90 degrees in map: odom's +x is map's +y (REP-103, yaw CCW positive).
  const Pose2D out = compose(Pose2D{1.0, 1.0, M_PI / 2.0}, Pose2D{2.0, 0.0, 0.0});
  EXPECT_NEAR(out.x_m, 1.0, 1e-12);
  EXPECT_NEAR(out.y_m, 3.0, 1e-12);
  EXPECT_NEAR(out.yaw_rad, M_PI / 2.0, 1e-12);
  const Pose2D left = compose(Pose2D{0.0, 0.0, M_PI / 2.0}, Pose2D{0.0, 1.0, 0.0});
  EXPECT_NEAR(left.x_m, -1.0, 1e-12);
  EXPECT_NEAR(left.y_m, 0.0, 1e-12);
}

TEST(Pose2D, YawWraps) {
  const Pose2D out = compose(Pose2D{0.0, 0.0, 3.0}, Pose2D{0.0, 0.0, 0.5});
  EXPECT_NEAR(out.yaw_rad, 3.5 - 2.0 * M_PI, 1e-12);
  const Pose2D negative = compose(Pose2D{0.0, 0.0, -3.0}, Pose2D{0.0, 0.0, -0.5});
  EXPECT_NEAR(negative.yaw_rad, 2.0 * M_PI - 3.5, 1e-12);
}

TEST(Pose2D, ComposeWithInverseIsIdentity) {
  const Pose2D a{0.7, -1.2, 2.1};
  // Inverse of a, written out by hand.
  const double c = std::cos(a.yaw_rad);
  const double s = std::sin(a.yaw_rad);
  const Pose2D inv{-(c * a.x_m + s * a.y_m), s * a.x_m - c * a.y_m, -a.yaw_rad};
  const Pose2D out = compose(a, inv);
  EXPECT_NEAR(out.x_m, 0.0, 1e-12);
  EXPECT_NEAR(out.y_m, 0.0, 1e-12);
  EXPECT_NEAR(out.yaw_rad, 0.0, 1e-12);
}

}  // namespace
}  // namespace racer_control
