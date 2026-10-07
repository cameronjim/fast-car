// park_planner.hpp: the two-arc parallel park and the three-point turn, against hand-computed
// values for the committed car (park_test_helpers.hpp).
//
// Hand computation, committed geometry (vehicle_params 0.12.0):
//   delta = 0.95 * 0.4189 = 0.397955 rad, tan(delta) = 0.4203847
//   R     = 0.3302 / 0.4203847 = 0.785471 m
//   a 0.5 m lateral offset:  cos(theta) = 1 - 0.5 / (2 R) = 0.681724, theta = 0.820686 rad
//                            (47.02 deg), along the row 2 R sin(theta) = 1.149322 m
//   row face 0.5 m out:      d = 0.5 + 0.31 / 2 + 0.1 = 0.755 m, theta = 1.024652 rad
//                            (58.71 deg), 2 R sin(theta) = 1.342422 m, each arc R theta =
//                            0.804834 m; after ARC_1 the rear axle is R sin(theta) = 0.671211 m
//                            back and R (1 - cos(theta)) = 0.3775 m over, heading theta.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <string>

#include "park_test_helpers.hpp"
#include "racer_control/park_planner.hpp"

namespace racer_control {
namespace {

using namespace park_test;

constexpr double kR = 0.785471;
constexpr double kHand = 2e-6;

ParkSlot right_slot(double row, double near_x, double far_x, double depth) {
  ParkSlot s;
  s.side = ParkSide::kRight;
  s.near_x_m = near_x;
  s.far_x_m = far_x;
  s.row_y_m = -row;
  s.back_y_m = -(row + depth);
  return s;
}

TEST(ParkPlanner, RadiusAndAngleForAHalfMetreOffset) {
  // The row face at 0.5 - 0.255 = 0.245 m would be closer than the margin, so the 0.5 m case is
  // checked on the formula the planner uses, then the planner on a 0.755 m offset below.
  const double delta = kSteeringFraction * kMaxSteering;
  const double R = kWheelbase / std::tan(delta);
  EXPECT_NEAR(R, kR, kHand);
  const double theta = std::acos(1.0 - 0.5 / (2.0 * R));
  EXPECT_NEAR(theta, 0.820686, kHand);
  EXPECT_NEAR(2.0 * R * std::sin(theta), 1.149322, kHand);
}

TEST(ParkPlanner, ParallelParkGeometry) {
  // Pocket 0.5 .. 2.2 m (1.7 m long), 0.7 m deep, row face 0.5 m right, car at the origin.
  const ParkSlot slot = right_slot(0.5, 0.5, 2.2, 0.7);
  const ParallelParkPlan p = plan_parallel_park(slot, Pose2{}, car_planner());
  ASSERT_TRUE(p.feasible) << plan_reject_name(p.reject) << ": " << p.detail;
  EXPECT_NEAR(p.radius_m, kR, kHand);
  EXPECT_NEAR(p.steering_rad, -kSteeringFraction * kMaxSteering, 1e-12);  // right, toward it
  EXPECT_NEAR(p.lateral_offset_m, 0.755, 1e-12);
  EXPECT_NEAR(p.theta_rad, 1.024652, kHand);
  EXPECT_NEAR(p.arc_longitudinal_m, 1.342422, kHand);
  // Start parallel, same lateral position; arcs exactly as hand computed.
  EXPECT_NEAR(p.start.y, 0.0, 1e-12);
  EXPECT_NEAR(p.start.yaw, 0.0, 1e-12);
  EXPECT_NEAR(p.after_arc1.x, p.start.x - 0.671211, kHand);
  EXPECT_NEAR(p.after_arc1.y, -0.3775, kHand);
  EXPECT_NEAR(p.after_arc1.yaw, 1.024652, kHand);
  EXPECT_NEAR(p.after_arc2.x, p.start.x - 1.342422, kHand);
  EXPECT_NEAR(p.after_arc2.y, -0.755, 1e-9);
  EXPECT_NEAR(p.after_arc2.yaw, 0.0, 1e-12);
  // The start is ahead of the far edge by 2 R sin(theta) minus the share of the slot ahead of
  // the final rear axle.
  EXPECT_NEAR(p.start.x - slot.far_x_m, p.arc_longitudinal_m - (slot.far_x_m - p.after_arc2.x),
              1e-12);
  // Fully inside, margins kept, centring forward never past the front margin.
  EXPECT_GE(p.rear_gap_m, kMargin - 1e-9);
  EXPECT_GE(p.front_gap_m - p.forward_m, kMargin - 1e-9);
  EXPECT_LE(p.forward_m, 0.15 + 1e-12);
  EXPECT_GE(p.min_clearance_m, kMargin);
  EXPECT_GT(p.min_gate_distance_m, gate_threshold_m(car_gate(), kParkSpeed));
  // The steps.
  ASSERT_EQ(p.steps.size(), 4u);
  EXPECT_EQ(p.steps[0].phase, ParkPhase::kDriveToStart);
  EXPECT_EQ(p.steps[1].phase, ParkPhase::kArc1);
  EXPECT_EQ(p.steps[1].direction, -1);
  EXPECT_NEAR(p.steps[1].planned_distance_m, -kR * 1.024652, 1e-5);
  EXPECT_EQ(p.steps[2].phase, ParkPhase::kArc2);
  EXPECT_NEAR(p.steps[2].steering_rad, -p.steps[1].steering_rad, 1e-12);
  EXPECT_EQ(p.steps[3].phase, ParkPhase::kStraighten);
  EXPECT_EQ(p.steps[3].steering_rad, 0.0);
  std::printf(
      "plan: start %.3f, final (%.3f, %.3f), rear gap %.3f, front gap %.3f, fwd %.3f, "
      "clearance %.3f, gate %.3f\n",
      p.start.x, p.final_pose.x, p.final_pose.y, p.rear_gap_m, p.front_gap_m, p.forward_m,
      p.min_clearance_m, p.min_gate_distance_m);
}

TEST(ParkPlanner, LeftIsTheMirrorImage) {
  ParkSlot right = right_slot(0.5, 0.5, 2.2, 0.7);
  ParkSlot left = right;
  left.side = ParkSide::kLeft;
  left.row_y_m = 0.5;
  left.back_y_m = 1.2;
  const ParallelParkPlan r = plan_parallel_park(right, Pose2{}, car_planner());
  const ParallelParkPlan l = plan_parallel_park(left, Pose2{}, car_planner());
  ASSERT_TRUE(r.feasible);
  ASSERT_TRUE(l.feasible);
  EXPECT_NEAR(l.steering_rad, -r.steering_rad, 1e-12);
  EXPECT_NEAR(l.final_pose.x, r.final_pose.x, 1e-9);
  EXPECT_NEAR(l.final_pose.y, -r.final_pose.y, 1e-9);
  EXPECT_NEAR(l.after_arc1.yaw, -r.after_arc1.yaw, 1e-12);
}

TEST(ParkPlanner, Rejections) {
  const ParkPlannerConfig c = car_planner();
  // Too short for the car plus two margins (0.58 + 0.2 = 0.78 m).
  ParallelParkPlan p = plan_parallel_park(right_slot(0.5, 0.5, 1.2, 0.7), Pose2{}, c);
  EXPECT_EQ(p.reject, PlanReject::kTooShort) << p.detail;
  // Too shallow: 0.45 m < 0.31 + 0.2.
  p = plan_parallel_park(right_slot(0.5, 0.5, 2.2, 0.45), Pose2{}, c);
  EXPECT_EQ(p.reject, PlanReject::kTooShallow) << p.detail;
  // Offset too large for one pass: d = 1.5 + 0.255 > 2 R = 1.571.
  p = plan_parallel_park(right_slot(1.5, 0.5, 2.2, 0.7), Pose2{}, c);
  EXPECT_EQ(p.reject, PlanReject::kOffsetTooLarge) << p.detail;
  // The car's side 0.05 m from the row face.
  p = plan_parallel_park(right_slot(0.205, 0.5, 2.2, 0.7), Pose2{}, c);
  EXPECT_EQ(p.reject, PlanReject::kTooCloseToRow) << p.detail;
  // Not parallel.
  p = plan_parallel_park(right_slot(0.5, 0.5, 2.2, 0.7), Pose2{0.0, 0.0, 0.2}, c);
  EXPECT_EQ(p.reject, PlanReject::kNotParallel) << p.detail;
  // Garbage.
  p = plan_parallel_park(right_slot(0.5, 2.2, 0.5, 0.7), Pose2{}, c);
  EXPECT_EQ(p.reject, PlanReject::kBadInput);
  EXPECT_FALSE(p.feasible);
}

// The swept-body check is what rejects a 1.1 m slot: the static fit passes (1.1 > 0.78 m) but no
// placement keeps the body `margin` off the obstacles; the detail names the motion and the face.
TEST(ParkPlanner, SweptBodyCheckRejectsAContactCase) {
  ParkPlannerConfig c = car_planner();
  c.gate.enabled = false;
  const ParkSlot slot = right_slot(0.5, 0.5, 1.6, 0.7);
  const ParallelParkPlan p = plan_parallel_park(slot, Pose2{}, c);
  EXPECT_EQ(p.reject, PlanReject::kSweptContact) << p.detail;
  EXPECT_NE(p.detail.find("ARC_"), std::string::npos) << p.detail;
  EXPECT_NE(p.detail.find("obstacle's end face"), std::string::npos) << p.detail;
  // And directly: ARC_1 started 0.6 m ahead of the far edge sweeps the inside flank through the
  // front obstacle's corner (the corner, 0.214 m inside the turn circle's centre line, lies in
  // the band between R - w/2 = 0.630 m and the outer corner's sweep); started 0.25 m ahead the
  // corner is inside R - w/2 and the flank passes outside it.
  const auto obstacles = parallel_park_obstacles(slot, 2.0);
  const double delta = kSteeringFraction * kMaxSteering;
  const SweepResult s = sweep_body(Pose2{slot.far_x_m + 0.6, 0.0, 0.0}, {-delta, -kR * 1.0246},
                                   car_body(), obstacles, kMargin, 0.01);
  EXPECT_FALSE(s.clear);
  EXPECT_EQ(s.min_clearance_m, 0.0);
  // The front obstacle's corner, where its end face (3) meets the row face ahead (4).
  EXPECT_TRUE(s.worst_obstacle == 3u || s.worst_obstacle == 4u) << s.worst_obstacle;
  EXPECT_TRUE(sweep_body(Pose2{slot.far_x_m + 0.25, 0.0, 0.0}, {-delta, -kR * 1.0246}, car_body(),
                         obstacles, kMargin, 0.01)
                  .clear);
  std::printf("contact: %s\n", p.detail.c_str());
}

// With the body clear, the gate prediction still rejects a slot where safety_node's rear
// corridor would brake ARC_2 (the rear gap along the arc is below 0.28 m).
TEST(ParkPlanner, GatePredictionRejectsASlotTheBodyFits) {
  ParkPlannerConfig no_gate = car_planner();
  no_gate.gate.enabled = false;
  const ParkSlot slot = right_slot(0.5, 0.5, 1.85, 0.7);  // 1.35 m
  const ParallelParkPlan without = plan_parallel_park(slot, Pose2{}, no_gate);
  ASSERT_TRUE(without.feasible) << without.detail;
  const ParallelParkPlan with = plan_parallel_park(slot, Pose2{}, car_planner());
  EXPECT_EQ(with.reject, PlanReject::kGateClearance) << with.detail;
  EXPECT_NE(with.detail.find("safety_node's corridor"), std::string::npos);
  std::printf("gate: %s\n", with.detail.c_str());
}

TEST(ParkPlanner, MinimumSlotLength) {
  const auto with_gate = minimum_slot_length_m(0.5, 0.7, ParkSide::kRight, car_planner());
  ParkPlannerConfig no_gate = car_planner();
  no_gate.gate.enabled = false;
  const auto without = minimum_slot_length_m(0.5, 0.7, ParkSide::kRight, no_gate);
  ASSERT_TRUE(with_gate.has_value());
  ASSERT_TRUE(without.has_value());
  std::printf(
      "minimum slot length, row 0.5 m, depth 0.7 m: %.3f m (%.3f m without the gate "
      "prediction)\n",
      *with_gate, *without);
  // More than the body (0.58 m) plus two margins, and the gate costs length.
  EXPECT_GT(*without, 0.78);
  EXPECT_GT(*with_gate, *without);
  EXPECT_LT(*with_gate, 2.0);
  // Consistency: just above it plans, just below it does not.
  EXPECT_TRUE(plan_parallel_park(right_slot(0.5, 0.5, 0.5 + *with_gate + 0.002, 0.7), Pose2{},
                                 car_planner())
                  .feasible);
  EXPECT_FALSE(
      plan_parallel_park(right_slot(0.5, 0.5, 0.5 + *with_gate - 0.01, 0.7), Pose2{}, car_planner())
          .feasible);
}

TEST(ParkPlanner, ThreePointTurn) {
  const ParkPlannerConfig c = car_planner();
  const ThreePointTurnPlan p = plan_three_point_turn(Pose2{}, 0.9, -0.9, TurnDirection::kLeft, c);
  ASSERT_TRUE(p.feasible) << p.detail;
  EXPECT_NEAR(p.radius_m, kR, kHand);
  EXPECT_NEAR(p.theta1_rad + p.theta2_rad + p.theta3_rad, M_PI, 1e-9);
  EXPECT_NEAR(p.final_pose.yaw, M_PI, 1e-9);
  EXPECT_GE(p.min_clearance_m, kMargin);
  ASSERT_EQ(p.steps.size(), 3u);
  EXPECT_GT(p.steps[0].steering_rad, 0.0);  // left lock forward
  EXPECT_LT(p.steps[1].steering_rad, 0.0);  // right lock reverse
  EXPECT_EQ(p.steps[1].direction, -1);
  EXPECT_GT(p.steps[2].steering_rad, 0.0);
  // Re-sweep every arc: margin kept.
  Pose2 pose{};
  const double dist[] = {p.theta1_rad * kR, -p.theta2_rad * kR, p.theta3_rad * kR};
  for (int i = 0; i < 3; ++i) {
    const SweepResult s =
        sweep_body(pose, {p.steps[i].steering_rad, dist[i]}, c.body, p.obstacles, kMargin, 0.01);
    EXPECT_TRUE(s.clear) << "arc " << i << " clearance " << s.min_clearance_m;
    pose = s.end;
  }
  // The right turn mirrors it.
  const ThreePointTurnPlan r = plan_three_point_turn(Pose2{}, 0.9, -0.9, TurnDirection::kRight, c);
  ASSERT_TRUE(r.feasible);
  EXPECT_NEAR(r.theta1_rad, p.theta1_rad, 1e-9);
  EXPECT_NEAR(r.final_pose.yaw, -M_PI, 1e-9);
  std::printf("3pt in 1.8 m: %.1f + %.1f + %.1f deg, clearance %.3f m\n",
              p.theta1_rad * 180.0 / M_PI, p.theta2_rad * 180.0 / M_PI, p.theta3_rad * 180.0 / M_PI,
              p.min_clearance_m);
}

TEST(ParkPlanner, ThreePointTurnLaneLimits) {
  const ParkPlannerConfig c = car_planner();
  const ThreePointTurnPlan narrow =
      plan_three_point_turn(Pose2{}, 0.6, -0.6, TurnDirection::kLeft, c);
  EXPECT_FALSE(narrow.feasible);
  EXPECT_EQ(narrow.reject, PlanReject::kLaneTooNarrow);
  // Wide enough for a U-turn: one arc.
  const ThreePointTurnPlan wide =
      plan_three_point_turn(Pose2{}, 2.5, -2.5, TurnDirection::kLeft, c);
  ASSERT_TRUE(wide.feasible);
  EXPECT_EQ(wide.steps.size(), 1u);
  EXPECT_NEAR(wide.theta1_rad, M_PI, 1e-9);
  // Already against a wall.
  EXPECT_EQ(plan_three_point_turn(Pose2{}, 0.2, -2.0, TurnDirection::kLeft, c).reject,
            PlanReject::kLaneTooNarrow);
  // Narrowest lane.
  const auto width = minimum_lane_width_m(c, TurnDirection::kLeft);
  ASSERT_TRUE(width.has_value());
  std::printf("minimum lane width for the three-point turn: %.3f m\n", *width);
  EXPECT_GT(*width, 1.2);
  EXPECT_LT(*width, 1.8);
  EXPECT_TRUE(plan_three_point_turn(Pose2{}, *width / 2 + 0.002, -*width / 2 - 0.002,
                                    TurnDirection::kLeft, c)
                  .feasible);
}

}  // namespace
}  // namespace racer_control
