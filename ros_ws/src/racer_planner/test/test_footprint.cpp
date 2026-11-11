#include <gtest/gtest.h>

#include <cmath>

#include "racer_planner/footprint.hpp"
#include "test_support.hpp"

using namespace racer_planner;

TEST(Footprint, RadiusAndCentresFollowTheRule) {
  const VehicleGeometry g = test::binding_geometry();
  const Footprint f(g);
  const double len = g.body_length_m();
  EXPECT_NEAR(f.radius(), std::sqrt(len * len / 36.0 + g.width_m * g.width_m / 4.0), 1e-12);
  EXPECT_NEAR(f.offsets()[0], -g.rear_overhang_m + len / 6.0, 1e-12);
  EXPECT_NEAR(f.offsets()[1], -g.rear_overhang_m + len / 2.0, 1e-12);
  EXPECT_NEAR(f.offsets()[2], g.body_max_x_m() - len / 6.0, 1e-12);
}

TEST(Footprint, EveryRectangleCornerAndEdgePointIsInsideACircle) {
  const VehicleGeometry g = test::binding_geometry();
  const Footprint f(g);
  const double x_lo = g.body_min_x_m(), x_hi = g.body_max_x_m(), hw = g.width_m / 2.0;
  for (double x : {x_lo, x_hi}) {
    for (double y : {-hw, hw}) EXPECT_TRUE(f.covers_body_point(x, y)) << x << " " << y;
  }
  for (int i = 0; i <= 200; ++i) {
    const double x = x_lo + (x_hi - x_lo) * i / 200.0;
    for (double y : {-hw, 0.0, hw}) EXPECT_TRUE(f.covers_body_point(x, y)) << x << " " << y;
  }
  // and the covering is not trivially everything: far away is not covered
  EXPECT_FALSE(f.covers_body_point(x_hi + 0.5, 0.0));
}

TEST(Footprint, ClearanceSignsAndValue) {
  const VehicleGeometry g = test::binding_geometry();
  const Footprint f(g);
  DistanceField field;
  // One obstacle point 0.3 m ahead of the front circle centre.
  const double front_centre = f.offsets()[2];
  PointXY p{front_centre + f.radius() + 0.3, 0.0};
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
  EXPECT_NEAR(facing_y, 1.0 - f.offsets()[2] - f.radius(), 0.04);
}
