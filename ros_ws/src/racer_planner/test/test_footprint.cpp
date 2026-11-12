#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "racer_planner/footprint.hpp"
#include "racer_planner/mppi.hpp"
#include "test_support.hpp"

using namespace racer_planner;

namespace {

// Distance from a body frame point outside every circle to the nearest circle edge (0 inside).
double uncovered_depth(const Footprint& f, double bx, double by) {
  double best = 1e9;
  for (const Circle& c : f.circles()) {
    best = std::min(best, std::hypot(bx - c.x, by - c.y) - c.r);
  }
  return std::max(best, 0.0);
}

}  // namespace

TEST(Footprint, CirclesFollowTheRule) {
  const VehicleGeometry g = test::binding_geometry();
  const Footprint f(g);
  const double hw = g.width_m / 2.0;
  const double r = f.corner_radius();
  EXPECT_NEAR(f.radius(), hw, 1e-12);
  EXPECT_NEAR(r, Footprint::kDefaultCornerRadiusM, 1e-12);
  EXPECT_NEAR(f.offsets()[0], g.body_min_x_m() + hw, 1e-12);
  EXPECT_NEAR(f.offsets()[1], 0.5 * (g.body_min_x_m() + g.body_max_x_m()), 1e-12);
  EXPECT_NEAR(f.offsets()[2], g.body_max_x_m() - hw, 1e-12);
  const auto& c = f.circles();
  ASSERT_EQ(c.size(), 7u);
  for (int k = 0; k < 3; ++k) {
    EXPECT_NEAR(c[k].x, f.offsets()[k], 1e-12);
    EXPECT_EQ(c[k].y, 0.0);
    EXPECT_NEAR(c[k].r, hw, 1e-12);
  }
  EXPECT_NEAR(c[3].x, g.body_max_x_m() - r, 1e-12);
  EXPECT_NEAR(c[3].y, hw - r, 1e-12);
  EXPECT_NEAR(c[4].y, -(hw - r), 1e-12);
  EXPECT_NEAR(c[5].x, g.body_min_x_m() + r, 1e-12);
  EXPECT_NEAR(c[6].x, g.body_min_x_m() + r, 1e-12);
  EXPECT_NEAR(c[5].y, hw - r, 1e-12);
  EXPECT_NEAR(c[6].y, -(hw - r), 1e-12);
  for (int k = 3; k < 7; ++k) EXPECT_NEAR(c[k].r, r, 1e-12);
}

TEST(Footprint, NoCircleReachesBeyondTheRectangle) {
  // Reach 0 in front, behind and to the side: every circle lies inside the body rectangle.
  const VehicleGeometry g = test::binding_geometry();
  const Footprint f(g);
  const double hw = g.width_m / 2.0;
  for (const Circle& c : f.circles()) {
    EXPECT_LE(c.x + c.r, g.body_max_x_m() + 1e-12);
    EXPECT_GE(c.x - c.r, g.body_min_x_m() - 1e-12);
    EXPECT_LE(c.y + c.r, hw + 1e-12);
    EXPECT_GE(c.y - c.r, -hw - 1e-12);
  }
  // and the front body circle touches the front bumper line exactly
  EXPECT_NEAR(f.circles()[2].x + f.circles()[2].r, g.body_max_x_m(), 1e-12);
}

TEST(Footprint, UncoveredCornerIsUnderTheHardCollisionClearance) {
  const VehicleGeometry g = test::binding_geometry();
  const Footprint f(g);
  const double hw = g.width_m / 2.0;
  const double r = f.corner_radius();
  const double expected_corner = r * (std::sqrt(2.0) - 1.0);
  for (double x : {g.body_min_x_m(), g.body_max_x_m()}) {
    for (double y : {-hw, hw}) {
      EXPECT_NEAR(uncovered_depth(f, x, y), expected_corner, 1e-9) << x << " " << y;
    }
  }
  // Over the whole rectangle boundary and its inside the worst uncovered depth is the corner or
  // the gap on a side between two body circles; print the numbers the PR quotes.
  double worst = 0.0;
  const int n = 400;
  for (int i = 0; i <= n; ++i) {
    const double x = g.body_min_x_m() + g.body_length_m() * i / n;
    for (double y : {-hw, -0.5 * hw, 0.0, 0.5 * hw, hw}) {
      worst = std::max(worst, uncovered_depth(f, x, y));
    }
  }
  std::cout << "[footprint] corner uncovered " << expected_corner * 1e3 << " mm, worst uncovered "
            << worst * 1e3 << " mm, d_hard " << MppiParams().d_hard_m * 1e3 << " mm\n";
  EXPECT_LT(expected_corner, MppiParams().d_hard_m);
  EXPECT_LT(worst, MppiParams().d_hard_m);
}

TEST(Footprint, EveryRectangleEdgePointIsWithinTheUncoveredDepthOfACircle) {
  const VehicleGeometry g = test::binding_geometry();
  const Footprint f(g);
  const double x_lo = g.body_min_x_m(), x_hi = g.body_max_x_m(), hw = g.width_m / 2.0;
  // The centre line and the four points where the circles touch the rectangle are covered.
  for (int i = 0; i <= 200; ++i) {
    const double x = x_lo + (x_hi - x_lo) * i / 200.0;
    EXPECT_TRUE(f.covers_body_point(x, 0.0)) << x;
  }
  for (const Circle& c : f.circles()) {
    EXPECT_TRUE(f.covers_body_point(c.x, c.y));
  }
  EXPECT_TRUE(f.covers_body_point(x_hi, 0.0));
  EXPECT_TRUE(f.covers_body_point(x_lo, 0.0));
  EXPECT_TRUE(f.covers_body_point(f.offsets()[1], hw));
  // the covering is not trivially everything: far away is not covered
  EXPECT_FALSE(f.covers_body_point(x_hi + 0.5, 0.0));
  EXPECT_FALSE(f.covers_body_point(0.0, hw + 0.05));
  // the exact rectangle corners are the uncovered part
  EXPECT_FALSE(f.covers_body_point(x_hi, hw));
}

TEST(Footprint, ForwardClearanceUsesFiveCirclesAndReverseSeven) {
  const VehicleGeometry g = test::binding_geometry();
  const Footprint f(g);
  DistanceField field;
  // A short wall beside the rear corner and one beside the front corner, at a pose with yaw.
  std::vector<PointXY> pts;
  for (double t = 0.0; t <= 0.3; t += 0.01) {
    pts.push_back({g.body_min_x_m() - 0.03 - t, g.width_m / 2.0 + 0.03 + t});
    pts.push_back({g.body_max_x_m() + 0.03 + t, -g.width_m / 2.0 - 0.03 - t});
  }
  field.build(pts.data(), pts.size());
  for (double yaw : {0.0, 0.4, -1.2}) {
    const double cs = std::cos(yaw), sn = std::sin(yaw);
    double fwd = 1e9, all = 1e9;
    for (int k = 0; k < Footprint::kAllCircles; ++k) {
      const Circle& c = f.circles()[static_cast<std::size_t>(k)];
      const double d = field.distance(0.1 + c.x * cs - c.y * sn, -0.2 + c.x * sn + c.y * cs) - c.r;
      all = std::min(all, d);
      if (k < Footprint::kForwardCircles) fwd = std::min(fwd, d);
    }
    EXPECT_NEAR(f.clearance(field, 0.1, -0.2, cs, sn), fwd, 1e-12);
    EXPECT_NEAR(f.clearance_reverse(field, 0.1, -0.2, cs, sn), all, 1e-12);
    EXPECT_LE(all, fwd);
  }
}

TEST(Footprint, ClearanceSignsAndValue) {
  const VehicleGeometry g = test::binding_geometry();
  const Footprint f(g);
  DistanceField field;
  // One obstacle point 0.3 m ahead of the front bumper.
  PointXY p{g.body_max_x_m() + 0.3, 0.0};
  field.build(&p, 1);
  const double c = f.clearance(field, 0.0, 0.0, 1.0, 0.0);
  EXPECT_NEAR(c, 0.3, 0.04);  // grid quantisation
  EXPECT_GT(c, 0.0);
  // Drive 0.4 m forward: the point is now inside the front circle, clearance negative.
  EXPECT_LT(f.clearance(field, 0.4, 0.0, 1.0, 0.0), 0.0);
}

TEST(Footprint, ClearanceRotatesWithTheYaw) {
  const VehicleGeometry g = test::binding_geometry();
  const Footprint f(g);
  DistanceField field;
  PointXY p{0.0, 1.0};  // to the left of the rear axle
  field.build(&p, 1);
  // Facing +x the rear circle is nearest; facing +y (yaw 90 degrees) the front circle is.
  const double facing_x = f.clearance(field, 0.0, 0.0, 1.0, 0.0);
  const double facing_y = f.clearance(field, 0.0, 0.0, 0.0, 1.0);
  EXPECT_LT(facing_y, facing_x);
  EXPECT_NEAR(facing_y, 1.0 - g.body_max_x_m(), 0.04);
}

TEST(Footprint, CornerRadiusIsClampedToHalfTheWidth) {
  const VehicleGeometry g = test::binding_geometry();
  const Footprint big(g, 10.0);
  EXPECT_NEAR(big.corner_radius(), g.width_m / 2.0, 1e-12);
  const Footprint tiny(g, -1.0);
  EXPECT_GT(tiny.corner_radius(), 0.0);
}
