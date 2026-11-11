#include <gtest/gtest.h>

#include <cmath>

#include "racer_planner/ego_motion.hpp"

using namespace racer_planner;

namespace {
constexpr double kL = 0.5;  // synthetic wheelbase for closed form comparisons
}

TEST(EgoMotion, ConstantArcMatchesTheClosedForm) {
  EgoMotion e(kL, 0.0);
  const double v = 0.8, delta = 0.25;
  for (int i = 0; i <= 25; ++i) e.push(i * 0.02, v, delta);
  const double w = v * std::tan(delta) / kL;
  const double t = 0.3;
  const Pose2 p = e.relative_pose(0.1, 0.1 + t);
  EXPECT_NEAR(p.yaw, w * t, 1e-12);
  EXPECT_NEAR(p.x, v / w * std::sin(w * t), 1e-9);
  EXPECT_NEAR(p.y, v / w * (1.0 - std::cos(w * t)), 1e-9);
}

TEST(EgoMotion, StraightLineIsSpeedTimesTime) {
  EgoMotion e(kL, 0.0);
  for (int i = 0; i <= 20; ++i) e.push(i * 0.02, 0.5, 0.0);
  const Pose2 p = e.relative_pose(0.0, 0.4);
  EXPECT_NEAR(p.x, 0.2, 1e-12);
  EXPECT_NEAR(p.y, 0.0, 1e-12);
  EXPECT_NEAR(p.yaw, 0.0, 1e-12);
}

TEST(EgoMotion, SamplesAreHeldUntilTheNextOne) {
  EgoMotion e(kL, 0.0);
  e.push(0.0, 1.0, 0.0);
  e.push(0.1, 2.0, 0.0);
  e.push(0.2, 2.0, 0.0);
  // 0.05 s at 1 m/s then 0.1 s at 2 m/s
  EXPECT_NEAR(e.relative_pose(0.05, 0.2).x, 0.05 * 1.0 + 0.1 * 2.0, 1e-12);
}

TEST(EgoMotion, PiecewiseArcsComposeInTheStartFrame) {
  EgoMotion e(kL, 0.0);
  e.push(0.0, 1.0, 0.3);
  e.push(0.1, 1.0, -0.3);
  e.push(0.2, 1.0, -0.3);
  const double w = std::tan(0.3) / kL;
  // First piece: arc left for 0.1 s; second: arc right for 0.1 s. By symmetry the heading returns
  // to 0.
  const Pose2 p = e.relative_pose(0.0, 0.2);
  EXPECT_NEAR(p.yaw, 0.0, 1e-12);
  const double x1 = std::sin(w * 0.1) / w, y1 = (1.0 - std::cos(w * 0.1)) / w;
  EXPECT_NEAR(p.x, 2.0 * x1, 1e-9);
  EXPECT_NEAR(p.y, 2.0 * y1, 1e-9);
}

TEST(EgoMotion, UndersteerScalesTheYawRate) {
  EgoMotion e(kL, 1.0);
  e.push(0.0, 2.0, 0.2);
  e.push(1.0, 2.0, 0.2);
  const Pose2 p = e.relative_pose(0.0, 0.1);
  EXPECT_NEAR(p.yaw, 2.0 * std::tan(0.2) / kL / (1.0 + 4.0) * 0.1, 1e-12);
}

TEST(EgoMotion, GoingBackwardsInTimeIsTheInverse) {
  EgoMotion e(kL, 0.0);
  for (int i = 0; i <= 25; ++i) e.push(i * 0.02, 0.7, 0.2);
  const Pose2 fwd = e.relative_pose(0.1, 0.4);
  const Pose2 back = e.relative_pose(0.4, 0.1);
  // Composing the two gives the identity: back * fwd.
  const double c = std::cos(back.yaw), s = std::sin(back.yaw);
  EXPECT_NEAR(back.x + c * fwd.x - s * fwd.y, 0.0, 1e-9);
  EXPECT_NEAR(back.y + s * fwd.x + c * fwd.y, 0.0, 1e-9);
  EXPECT_NEAR(back.yaw + fwd.yaw, 0.0, 1e-12);
}

TEST(EgoMotion, ZeroIntervalIsTheIdentity) {
  EgoMotion e(kL, 0.0);
  e.push(0.0, 1.0, 0.1);
  const Pose2 p = e.relative_pose(0.0, 0.0);
  EXPECT_EQ(p.x, 0.0);
  EXPECT_EQ(p.yaw, 0.0);
}

TEST(EgoMotion, CoveredFlagReportsExtrapolationBeforeTheOldestSample) {
  EgoMotion e(kL, 0.0);
  bool covered = true;
  e.relative_pose(0.0, 0.1, &covered);
  EXPECT_FALSE(covered);  // empty
  e.push(1.0, 0.5, 0.0);
  e.push(1.1, 0.5, 0.0);
  e.relative_pose(0.5, 1.0, &covered);
  EXPECT_FALSE(covered);
  e.relative_pose(1.0, 1.1, &covered);
  EXPECT_TRUE(covered);
}

TEST(EgoMotion, BadSamplesAreIgnored) {
  EgoMotion e(kL, 0.0);
  e.push(0.0, 1.0, 0.0);
  e.push(std::nan(""), 1.0, 0.0);
  e.push(0.1, std::nan(""), 0.0);
  e.push(0.1, 1.0, INFINITY);
  e.push(-1.0, 1.0, 0.0);  // time goes backwards
  EXPECT_EQ(e.size(), 1u);
  e.push(0.0, 5.0, 0.0);  // same time
  EXPECT_EQ(e.size(), 1u);
}

TEST(EgoMotion, RingBufferKeepsTheNewestSamples) {
  EgoMotion e(kL, 0.0);
  for (int i = 0; i < 1000; ++i) e.push(i * 0.02, 1.0, 0.0);
  EXPECT_EQ(e.size(), 256u);
  bool covered = false;
  const Pose2 p = e.relative_pose(19.0, 19.9, &covered);
  EXPECT_TRUE(covered);
  EXPECT_NEAR(p.x, 0.9, 1e-9);
}

TEST(EgoMotion, NonFiniteQueryTimesGiveIdentity) {
  EgoMotion e(kL, 0.0);
  e.push(0.0, 1.0, 0.0);
  e.push(1.0, 1.0, 0.0);
  bool covered = true;
  const Pose2 p = e.relative_pose(std::nan(""), 0.5, &covered);
  EXPECT_EQ(p.x, 0.0);
  EXPECT_FALSE(covered);
}
