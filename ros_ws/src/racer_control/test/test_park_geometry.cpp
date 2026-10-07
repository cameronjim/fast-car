// park_geometry.hpp: arc primitives, body clearance, swept-body check, gate prediction
// (roadmap 2.9). Hand-computed cases with the committed car (park_test_helpers.hpp).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "park_test_helpers.hpp"
#include "racer_control/park_geometry.hpp"

namespace racer_control {
namespace {

using namespace park_test;

constexpr double kTol = 1e-9;

TEST(ParkGeometry, AdvancePoseStraightAndArcs) {
  const Pose2 a = advance_pose(Pose2{1.0, 2.0, M_PI / 2.0}, 0.0, 0.5);
  EXPECT_NEAR(a.x, 1.0, kTol);
  EXPECT_NEAR(a.y, 2.5, kTol);
  EXPECT_NEAR(a.yaw, M_PI / 2.0, kTol);

  // Quarter circle to the left, R = 2: ends at (2, 2) facing +y.
  const Pose2 q = advance_pose(Pose2{}, 0.5, M_PI);
  EXPECT_NEAR(q.x, 2.0, kTol);
  EXPECT_NEAR(q.y, 2.0, kTol);
  EXPECT_NEAR(q.yaw, M_PI / 2.0, kTol);

  // Backward with RIGHT steering (negative curvature): the rear axle goes back and to the right,
  // the heading increases (nose swings left): (-R sin t, -R (1 - cos t), +t).
  const double R = 0.8;
  const double t = 0.6;
  const Pose2 b = advance_pose(Pose2{}, -1.0 / R, -R * t);
  EXPECT_NEAR(b.x, -R * std::sin(t), kTol);
  EXPECT_NEAR(b.y, -R * (1.0 - std::cos(t)), kTol);
  EXPECT_NEAR(b.yaw, t, kTol);

  // Going there and back is the identity.
  const Pose2 back = advance_pose(b, -1.0 / R, R * t);
  EXPECT_NEAR(back.x, 0.0, kTol);
  EXPECT_NEAR(back.y, 0.0, kTol);
  EXPECT_NEAR(back.yaw, 0.0, kTol);
}

TEST(ParkGeometry, CurvatureSignConvention) {
  // Left steering is positive and turns left (06-vehicle-params.md: left positive).
  EXPECT_GT(curvature_for_steering(0.3, kWheelbase), 0.0);
  EXPECT_LT(curvature_for_steering(-0.3, kWheelbase), 0.0);
  EXPECT_NEAR(curvature_for_steering(0.3, kWheelbase), std::tan(0.3) / kWheelbase, kTol);
  EXPECT_EQ(curvature_for_steering(std::nan(""), kWheelbase), 0.0);
  EXPECT_EQ(curvature_for_steering(0.3, 0.0), 0.0);
  EXPECT_NEAR(wrap_pi(3.0 * M_PI / 2.0), -M_PI / 2.0, kTol);
}

TEST(ParkGeometry, FramesAndCorners) {
  const Pose2 pose{1.0, -1.0, M_PI / 2.0};
  const Point2 w = to_world(pose, {2.0, 0.5});
  EXPECT_NEAR(w.x, 0.5, kTol);
  EXPECT_NEAR(w.y, 1.0, kTol);
  const Point2 l = to_local(pose, w);
  EXPECT_NEAR(l.x, 2.0, kTol);
  EXPECT_NEAR(l.y, 0.5, kTol);
  const auto c = body_corners(Pose2{}, car_body());
  EXPECT_NEAR(c[0].x, kWheelbase + kFrontOverhang, kTol);  // front left
  EXPECT_NEAR(c[0].y, kWidth / 2.0, kTol);
  EXPECT_NEAR(c[2].x, -kRearOverhang, kTol);  // rear right
  EXPECT_NEAR(c[2].y, -kWidth / 2.0, kTol);
}

TEST(ParkGeometry, SegmentDistances) {
  const Segment2 s{{0.0, 0.0}, {1.0, 0.0}};
  EXPECT_NEAR(point_segment_distance({0.5, 0.3}, s), 0.3, kTol);
  EXPECT_NEAR(point_segment_distance({-0.3, 0.4}, s), 0.5, kTol);
  EXPECT_EQ(segment_segment_distance(s, {{0.5, -1.0}, {0.5, 1.0}}), 0.0);
  EXPECT_NEAR(segment_segment_distance(s, {{2.0, 0.0}, {3.0, 0.0}}), 1.0, kTol);

  const ParkBody body = car_body();
  // A wall 0.2 m to the right of the car's right side.
  const Segment2 wall{{-2.0, -kWidth / 2.0 - 0.2}, {2.0, -kWidth / 2.0 - 0.2}};
  EXPECT_NEAR(body_segment_distance(Pose2{}, body, wall), 0.2, 1e-12);
  // Crossing the body, and wholly inside it, are both contact.
  EXPECT_EQ(body_segment_distance(Pose2{}, body, {{0.0, -1.0}, {0.0, 1.0}}), 0.0);
  EXPECT_EQ(body_segment_distance(Pose2{}, body, {{0.0, -0.05}, {0.1, 0.05}}), 0.0);
  const Clearance c = body_clearance(Pose2{}, body, {{{5.0, 5.0}, {6.0, 5.0}}, wall});
  EXPECT_EQ(c.obstacle_index, 1u);
  EXPECT_NEAR(c.distance_m, 0.2, 1e-12);
}

TEST(ParkGeometry, SweepBodyFindsTheClosestApproach) {
  const ParkBody body = car_body();
  // A wall across the path 1.0 m ahead of the rear axle; the front bumper starts 0.5398 m from it.
  const std::vector<Segment2> wall = {{{1.0, -1.0}, {1.0, 1.0}}};
  const SweepResult a = sweep_body(Pose2{}, {0.0, 0.3}, body, wall, 0.1, 0.01);
  EXPECT_TRUE(a.clear);
  EXPECT_NEAR(a.min_clearance_m, 1.0 - body.front_x_m - 0.3, 1e-9);
  EXPECT_NEAR(a.end.x, 0.3, kTol);
  const SweepResult b = sweep_body(Pose2{}, {0.0, 0.6}, body, wall, 0.1, 0.01);
  EXPECT_FALSE(b.clear);
  EXPECT_EQ(b.min_clearance_m, 0.0);  // the body crosses the wall
  // Backing away is clear however far.
  EXPECT_TRUE(sweep_body(Pose2{}, {0.0, -2.0}, body, wall, 0.1, 0.01).clear);
  // Early exit reports the first violating sample.
  const SweepResult c = sweep_body(Pose2{}, {0.0, 0.6}, body, wall, 0.1, 0.01, true);
  EXPECT_FALSE(c.clear);
  EXPECT_LT(c.min_clearance_m, 0.1);
  EXPECT_GT(c.min_clearance_m, 0.08);
}

TEST(ParkGeometry, GateThreshold) {
  const ParkGateModel g = car_gate();
  // max(floor 0.25, 0.5 m/s * 0.3 s) + 0.03
  EXPECT_NEAR(gate_threshold_m(g, -0.5), 0.28, kTol);
  EXPECT_NEAR(gate_threshold_m(g, 2.0), 0.63, kTol);
}

TEST(ParkGeometry, GatePredictionStraightCorridors) {
  const ParkBody body = car_body();
  const ParkGateModel g = car_gate();
  // Straight ahead: distance from the head, x - mount_x.
  EXPECT_NEAR(predicted_gate_distance_m(Pose2{}, 0.0, false, {{1.0, 0.1}}, body, g), 1.0 - kLidarX,
              kTol);
  // Beside the corridor (|y| > width / 2 + 0.05): not in the path.
  EXPECT_TRUE(std::isinf(predicted_gate_distance_m(Pose2{}, 0.0, false, {{1.0, 0.3}}, body, g)));
  // Behind, reversing: from the rear bumper line.
  EXPECT_NEAR(predicted_gate_distance_m(Pose2{}, 0.0, true, {{-0.6, -0.1}}, body, g),
              0.6 - kRearOverhang, kTol);
  // Behind is never in the forward path, ahead never in the rear one.
  EXPECT_TRUE(std::isinf(predicted_gate_distance_m(Pose2{}, 0.0, false, {{-0.6, 0.0}}, body, g)));
  EXPECT_TRUE(std::isinf(predicted_gate_distance_m(Pose2{}, 0.0, true, {{1.0, 0.0}}, body, g)));
  // Outer sector: a return beside the head, at 90 degrees, is outside +/- 1.2 rad.
  ParkGateModel wide = g;
  wide.corridor_half_width_m = 5.0;
  EXPECT_TRUE(
      std::isinf(predicted_gate_distance_m(Pose2{}, 0.0, false, {{kLidarX, 1.0}}, body, wide)));
}

TEST(ParkGeometry, GatePredictionArcMatchesTheHandFormula) {
  const ParkBody body = car_body();
  const ParkGateModel g = car_gate();
  const double steering = kSteeringFraction * kMaxSteering;
  const double R = kWheelbase / std::tan(steering);  // 0.78547 m
  // Reversing with a left lock, a return straight behind at 0.5 m from the rear axle:
  // mirrored (xm, ym) = (0.5, 0); rho = hypot(0.5, R) = 0.9311 (inside the body band, <= R +
  // 0.205); phi = atan2(0.5, R); the reference is the rear bumper line, atan2(0.12, R).
  const double expected = R * (std::atan2(0.5, R) - std::atan2(kRearOverhang, R));
  EXPECT_NEAR(predicted_gate_distance_m(Pose2{}, steering, true, {{-0.5, 0.0}}, body, g), expected,
              1e-12);
  EXPECT_NEAR(expected, 0.3262, 1e-4);
  // The same return 0.1 m to the right is beyond the outer rear corner's sweep,
  // hypot(0.12, R + 0.205) = 0.9977 < hypot(0.5, R + 0.1) = 1.0170: not in the path.
  EXPECT_TRUE(
      std::isinf(predicted_gate_distance_m(Pose2{}, steering, true, {{-0.5, -0.1}}, body, g)));
  // Forward with a left lock, a return on the arc ahead: distance from the head's arc angle.
  const Point2 p{0.6, 0.25};
  const double phi = std::atan2(p.x, R - p.y);
  const double phi_head = std::atan2(kLidarX, R - kLidarY);
  EXPECT_NEAR(predicted_gate_distance_m(Pose2{}, steering, false, {p}, body, g),
              R * (phi - phi_head), 1e-12);
  // A steering beyond the limit is clamped to it, like safety_node does.
  const double R_max = kWheelbase / std::tan(kMaxSteering);
  EXPECT_NEAR(predicted_gate_distance_m(Pose2{}, 1.0, true, {{-0.5, 0.0}}, body, g),
              R_max * (std::atan2(0.5, R_max) - std::atan2(kRearOverhang, R_max)), 1e-12);
}

// The cull radius never changes a threshold decision.
TEST(ParkGeometry, GateCullRadiusIsSafe) {
  const ParkBody body = car_body();
  const ParkGateModel g = car_gate();
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> coord(-3.0, 3.0);
  std::uniform_real_distribution<double> steer(-kMaxSteering, kMaxSteering);
  for (int trial = 0; trial < 2000; ++trial) {
    const double steering = trial % 5 == 0 ? 0.0 : steer(rng);
    const bool reverse = trial % 2 == 0;
    const double threshold = gate_threshold_m(g, 0.5);
    const double cull = gate_cull_radius_m(steering, reverse, body, g, threshold);
    std::vector<Point2> points(20);
    for (Point2& p : points) {
      p = {coord(rng), coord(rng)};
    }
    const double full = predicted_gate_distance_m(Pose2{}, steering, reverse, points, body, g);
    const double culled =
        predicted_gate_distance_m(Pose2{}, steering, reverse, points, body, g, cull);
    EXPECT_EQ(full <= threshold, culled <= threshold);
    if (full <= threshold) {
      EXPECT_EQ(full, culled);
    }
  }
}

TEST(ParkGeometry, SampleSegments) {
  const auto points = sample_segments({{{0.0, 0.0}, {1.0, 0.0}}}, 0.25);
  ASSERT_EQ(points.size(), 5u);
  EXPECT_NEAR(points[4].x, 1.0, kTol);
  EXPECT_TRUE(sample_segments({{{0.0, 0.0}, {1.0, 0.0}}}, 0.0).empty());
}

TEST(ParkGeometry, UsableBody) {
  EXPECT_TRUE(is_usable_body(car_body()));
  ParkBody b = car_body();
  b.wheelbase_m = 0.0;
  EXPECT_FALSE(is_usable_body(b));
  b = car_body();
  b.rear_x_m = -0.1;
  EXPECT_FALSE(is_usable_body(b));
  b = car_body();
  b.lidar_x_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(is_usable_body(b));
}

}  // namespace
}  // namespace racer_control
