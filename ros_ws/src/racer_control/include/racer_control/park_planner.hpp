// Manoeuvre planning for park_node (roadmap 2.9): the two-arc parallel park and the three-point
// turn, both built from park_geometry.hpp's arc primitives. ROS-free and gtest-covered
// (test/test_park_planner.cpp); park_node fills ParkPlannerConfig from vehicle_params.
//
// ---------------------------------------------------------------------------------------------
// PARALLEL PARK (plan_parallel_park)
// ---------------------------------------------------------------------------------------------
// Plan frame: the frame the car's pose is dead-reckoned in (park_controller.hpp), x along the
// row. The slot (ParkSlot) is the pocket park_slot.hpp found: near and far longitudinal edges,
// the row face line row_y_m and the pocket back back_y_m (signed y coordinates), on `side`.
// s = +1 for a slot on the left, -1 on the right.
//
//   steering   delta = park_steering_fraction * steering.max_angle_rad (0.95 leaves the servo
//              some travel short of its stop)
//   radius     R = L / tan(delta), L = chassis.wheelbase_m (rear-axle circle)
//   centreline the car parks with its centreline at the row face + chassis.width_m / 2 +
//              margin, so its near side is `margin` inside the row line
//   offset     d = row lateral distance + width / 2 + margin, measured from the car's current
//              lateral position (it drives parallel to the row)
//   angle      cos(theta) = 1 - d / (2 R)
//   travel     the two arcs together move the rear axle 2 R sin(theta) back along the row and
//              d sideways
//
// Manoeuvre, from a start pose parallel to the row (same lateral position as now):
//   DRIVE_TO_START straight (forward, or backward if the start is behind) to the start x
//   ARC_1          reverse, steering s * delta (toward the slot), through theta: the rear swings
//                  into the pocket, the nose out (heading -s * theta)
//   ARC_2          reverse, steering -s * delta, through theta: back to parallel, rear axle at
//                  the final pose
//   STRAIGHTEN     steering 0, then forward by `forward_m` (centring, below)
//
// Placement. The final rear-axle x is searched from the rearmost position (rear bumper `margin`
// ahead of the near edge) forward in final_x_step_m steps, to the frontmost (front bumper
// `margin` behind the far edge). The start is then final x + 2 R sin(theta): the rear axle is
// ahead of the far edge by 2 R sin(theta) minus the share of the slot behind the final front
// bumper. The first placement that passes every check below is the plan, so the car sits as far
// back as the checks allow, which keeps its inside flank off the front obstacle's corner.
//
// Checks, for each motion (DRIVE_TO_START, ARC_1, ARC_2 and the forward move):
//   * SWEPT BODY: park_geometry.hpp sweep_body against the slot's bounding segments (the row
//     face on both sides of the pocket out to obstacle_extent_m, the two end faces of the pocket,
//     its back, and the opposite-side wall when the scan saw one): the whole body box, which
//     includes both front corners and the inside rear corner, stays at least `margin` from every
//     segment at every sample.
//   * GATE: park_geometry.hpp sweep_gate against points along the same segments: racer_safety's
//     obstacle gate (forward corridor going forward, rear corridor reversing, along the motion's
//     steering arc) must not see anything nearer than max(limits.min_forward_clearance_m,
//     |speed| * limits.ttc_brake_s) + gate margin, or it would brake the car mid-manoeuvre. This
//     is what makes the rear gap at the end of ARC_2 larger than `margin` on this car: the rear
//     corridor is measured along the ARC the request steers, so the rear obstacle's face reads
//     nearer than the straight-line gap while ARC_2 still has its lock on.
//
// Centring: after the arcs, rear gap = final x - rear overhang - near edge and front gap =
// far edge - final x - front x. If the front gap is the larger, the plan moves forward by
// min(final_forward_m, (front gap - rear gap) / 2), further limited so the front keeps `margin`
// and the forward gate threshold from the far edge.
//
// Rejections (PlanReject), in the order they are tested: unusable input; the car not parallel
// (|yaw| > max_yaw_rad); the car's side nearer the row than `margin` (kTooCloseToRow); the pocket
// shallower than width + 2 * margin (kTooShallow); d > 2 R, more than one pass can reach
// (kOffsetTooLarge); the car plus 2 * margin longer than the slot (kTooShort); every placement
// failing the swept-body check (kSweptContact, the detail names the motion and the obstacle) or,
// with the body clear, the gate check (kGateClearance). minimum_slot_length_m runs the same
// planner over slot lengths to give the shortest slot it accepts for a geometry.
//
// ---------------------------------------------------------------------------------------------
// THREE-POINT TURN (plan_three_point_turn)
// ---------------------------------------------------------------------------------------------
// The car is stopped in a lane between two walls parallel to it (the nearest return left and
// right at its position, park_slot.hpp measure_lane), modelled as two straight segments
// obstacle_extent_m + 2 R either side of the car. Turning to the left (t = +1; t = -1 mirrors it):
//   TURN_1  forward, steering t * delta, through theta_1
//   TURN_2  reverse, steering -t * delta, through theta_2 (backing up with the opposite lock
//           keeps the heading turning the same way)
//   TURN_3  forward, steering t * delta, through theta_3 = pi - theta_1 - theta_2
// theta_1 is the largest angle (up to pi) for which the swept body keeps `margin` from both walls
// and the forward gate stays clear; theta_2 likewise for the reverse arc, up to pi - theta_1;
// then TURN_3 must be clear for its whole angle. If it is not, theta_2 is reduced step by step
// (a shorter TURN_2 leaves the car further from the near wall) until TURN_3 is clear. If no
// split works the lane is too narrow (kLaneTooNarrow). A lane wide enough for theta_1 = pi is a
// U-turn and the plan is just TURN_1. minimum_lane_width_m gives the narrowest lane, car
// centred, this planner accepts.
//
// ---------------------------------------------------------------------------------------------
// STEPS
// ---------------------------------------------------------------------------------------------
// A plan is a list of PlanStep for park_controller.hpp. Each step holds one steering for its
// whole length and ends on a target that is resolved against the dead-reckoned pose when the
// step starts, so an overshoot of one step is taken out by the next:
//   kDistance  travel `value` metres (|travel|), at most `limit`
//   kYaw       turn until the heading reaches `value` (ARC_1, ARC_2, TURN_3), at most `limit`
//              metres of travel
//   kX         drive straight until the rear axle's x reaches min(`value`, `limit`)
//              (DRIVE_TO_START, STRAIGHTEN); for STRAIGHTEN `limit` is the largest x the front
//              clearance allows.
#ifndef RACER_CONTROL_PARK_PLANNER_HPP_
#define RACER_CONTROL_PARK_PLANNER_HPP_

#include <optional>
#include <string>
#include <vector>

#include "racer_control/park_geometry.hpp"
#include "racer_control/park_slot.hpp"

namespace racer_control {

enum class ParkPhase {
  kIdle,
  kSearch,
  kSlotFound,
  kDriveToStart,
  kArc1,
  kArc2,
  kStraighten,
  kTurn1,
  kTurn2,
  kTurn3,
  kDone,
  kAbort,
};
const char* park_phase_name(ParkPhase phase);

enum class StepTarget { kDistance, kYaw, kX };

struct PlanStep {
  ParkPhase phase = ParkPhase::kIdle;
  double steering_rad = 0.0;
  int direction = 1;  // +1 forward, -1 backward; 0: from the sign of (value - x) at run time (kX)
  StepTarget target = StepTarget::kDistance;
  double value = 0.0;
  double limit = 0.0;
  double planned_distance_m = 0.0;  // signed, as planned (for the logs)
};

struct ParkPlannerConfig {
  ParkBody body;
  double max_steering_rad = 0.0;    // steering.max_angle_rad
  double steering_fraction = 0.95;  // park_steering_fraction
  double margin_m = 0.1;
  double final_forward_max_m = 0.15;
  double forward_speed_mps = 0.5;  // magnitudes, for the gate thresholds
  double reverse_speed_mps = 0.5;
  ParkGateModel gate;
  double sample_step_m = 0.01;
  double gate_point_spacing_m = 0.02;
  double final_x_step_m = 0.01;
  double obstacle_extent_m = 2.0;
  double max_yaw_rad = 0.087;
  // kYaw steps may run this much longer than planned at run time (to take out an overshoot of
  // the step before); park_controller.hpp re-checks the swept body before every step.
  double run_extension_m = 0.3;
};

// The slot in the plan frame (signed y coordinates; see park_slot.hpp for how they are found).
struct ParkSlot {
  ParkSide side = ParkSide::kRight;
  double near_x_m = 0.0;
  double far_x_m = 0.0;
  double row_y_m = 0.0;
  double back_y_m = 0.0;
  std::optional<double> far_wall_y_m;  // opposite-side wall, if seen
};

// The slot's bounding segments, in this order: row face before the pocket, rear obstacle's end
// face, pocket back, front obstacle's end face, row face after the pocket, then the far wall
// (if any). The row faces and the far wall run extent_m beyond the pocket.
std::vector<Segment2> parallel_park_obstacles(const ParkSlot& slot, double extent_m);
const char* parallel_park_obstacle_name(std::size_t index);

enum class PlanReject {
  kNone,
  kBadInput,
  kNotParallel,
  kTooCloseToRow,
  kTooShallow,
  kOffsetTooLarge,
  kTooShort,
  kSweptContact,
  kGateClearance,
  kLaneTooNarrow,
};
const char* plan_reject_name(PlanReject reject);

struct ParallelParkPlan {
  bool feasible = false;
  PlanReject reject = PlanReject::kBadInput;
  std::string detail;
  double radius_m = 0.0;
  double steering_rad = 0.0;  // ARC_1's signed steering; ARC_2 uses its negative
  double theta_rad = 0.0;
  double lateral_offset_m = 0.0;    // d
  double arc_longitudinal_m = 0.0;  // 2 R sin(theta)
  double required_length_m = 0.0;   // body + 2 * margin (the static fit, for kTooShort)
  Pose2 start;
  Pose2 after_arc1;
  Pose2 after_arc2;
  Pose2 final_pose;
  double rear_gap_m = 0.0;   // after ARC_2
  double front_gap_m = 0.0;  // after ARC_2
  double forward_m = 0.0;
  double max_final_x_m = 0.0;  // STRAIGHTEN's x limit
  double min_clearance_m = 0.0;
  double min_gate_distance_m = 0.0;
  std::vector<Segment2> obstacles;
  std::vector<Point2> gate_points;
  std::vector<PlanStep> steps;
};

ParallelParkPlan plan_parallel_park(const ParkSlot& slot, const Pose2& car,
                                    const ParkPlannerConfig& config);

// The shortest slot (m) plan_parallel_park accepts for a car parallel to the row at
// row_lateral_m from its face, a pocket depth_m deep, searched to 1 mm between the static fit and
// max_length_m. nullopt when no length up to max_length_m works (or the geometry is rejected for
// another reason, see the plan at max_length_m).
std::optional<double> minimum_slot_length_m(double row_lateral_m, double depth_m, ParkSide side,
                                            const ParkPlannerConfig& config,
                                            double max_length_m = 6.0);

enum class TurnDirection { kLeft, kRight };
const char* turn_direction_name(TurnDirection direction);

struct ThreePointTurnPlan {
  bool feasible = false;
  PlanReject reject = PlanReject::kBadInput;
  std::string detail;
  double radius_m = 0.0;
  double steering_rad = 0.0;  // TURN_1's signed steering
  double theta1_rad = 0.0;
  double theta2_rad = 0.0;
  double theta3_rad = 0.0;
  double lane_width_m = 0.0;
  Pose2 after_turn1;
  Pose2 after_turn2;
  Pose2 final_pose;
  double min_clearance_m = 0.0;
  std::vector<Segment2> obstacles;
  std::vector<Point2> gate_points;
  std::vector<PlanStep> steps;
};

// left_wall_y_m / right_wall_y_m: the walls' y coordinates in the plan frame (left > car.y >
// right).
ThreePointTurnPlan plan_three_point_turn(const Pose2& car, double left_wall_y_m,
                                         double right_wall_y_m, TurnDirection direction,
                                         const ParkPlannerConfig& config);

// The narrowest lane (m), car centred, plan_three_point_turn accepts; searched to 1 mm.
std::optional<double> minimum_lane_width_m(const ParkPlannerConfig& config, TurnDirection direction,
                                           double max_width_m = 6.0);

}  // namespace racer_control

#endif  // RACER_CONTROL_PARK_PLANNER_HPP_
