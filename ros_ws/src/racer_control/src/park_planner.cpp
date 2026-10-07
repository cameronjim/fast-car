// Manoeuvre planning for park_node (see include/racer_control/park_planner.hpp).
#include "racer_control/park_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace racer_control {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
// Below this a STRAIGHTEN forward move is not worth a motion.
constexpr double kMinMoveM = 0.005;
// Theta search resolution for the three-point turn (rear-axle arc length).
constexpr double kTurnBacktrackM = 0.02;

std::string format(const char* fmt, double a = 0.0, double b = 0.0, double c = 0.0,
                   double d = 0.0) {
  char buffer[256];
  std::snprintf(buffer, sizeof(buffer), fmt, a, b, c, d);
  return std::string(buffer);
}

bool finite_all(std::initializer_list<double> values) {
  for (double v : values) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

bool usable_config(const ParkPlannerConfig& c) {
  if (!is_usable_body(c.body)) {
    return false;
  }
  if (!finite_all({c.max_steering_rad, c.steering_fraction, c.margin_m, c.final_forward_max_m,
                   c.forward_speed_mps, c.reverse_speed_mps, c.sample_step_m,
                   c.gate_point_spacing_m, c.final_x_step_m, c.obstacle_extent_m, c.max_yaw_rad,
                   c.run_extension_m})) {
    return false;
  }
  return c.max_steering_rad > 0.0 && c.max_steering_rad < M_PI_2 && c.steering_fraction > 0.0 &&
         c.steering_fraction <= 1.0 && c.margin_m >= 0.0 && c.final_forward_max_m >= 0.0 &&
         c.sample_step_m > 0.0 && c.gate_point_spacing_m > 0.0 && c.final_x_step_m > 0.0 &&
         c.obstacle_extent_m > 0.0 && c.max_yaw_rad > 0.0 && c.run_extension_m >= 0.0;
}

// A motion's checks: swept body (clearance >= margin) and gate. Early exit on the first failure.
struct MotionCheck {
  bool body_clear = true;
  bool gate_clear = true;
  double clearance_m = kInf;
  std::size_t obstacle = 0;
  double gate_distance_m = kInf;
};

MotionCheck check_motion(const Pose2& start, const ParkMotion& motion, double speed_mps,
                         const ParkPlannerConfig& c, const std::vector<Segment2>& obstacles,
                         const std::vector<Point2>& gate_points) {
  MotionCheck out;
  const SweepResult sweep =
      sweep_body(start, motion, c.body, obstacles, c.margin_m, c.sample_step_m, true);
  out.clearance_m = sweep.min_clearance_m;
  out.obstacle = sweep.worst_obstacle;
  out.body_clear = sweep.clear;
  if (!out.body_clear) {
    return out;
  }
  const GateSweepResult gate =
      sweep_gate(start, motion, speed_mps, gate_points, c.body, c.gate, c.sample_step_m, true);
  out.gate_distance_m = gate.min_distance_m;
  out.gate_clear = gate.clear;
  return out;
}

}  // namespace

const char* park_phase_name(ParkPhase phase) {
  switch (phase) {
    case ParkPhase::kIdle:
      return "IDLE";
    case ParkPhase::kSearch:
      return "SEARCH";
    case ParkPhase::kSlotFound:
      return "SLOT_FOUND";
    case ParkPhase::kDriveToStart:
      return "DRIVE_TO_START";
    case ParkPhase::kArc1:
      return "ARC_1";
    case ParkPhase::kArc2:
      return "ARC_2";
    case ParkPhase::kStraighten:
      return "STRAIGHTEN";
    case ParkPhase::kTurn1:
      return "TURN_1";
    case ParkPhase::kTurn2:
      return "TURN_2";
    case ParkPhase::kTurn3:
      return "TURN_3";
    case ParkPhase::kDone:
      return "DONE";
    case ParkPhase::kAbort:
      return "ABORT";
  }
  return "UNKNOWN";
}

const char* plan_reject_name(PlanReject reject) {
  switch (reject) {
    case PlanReject::kNone:
      return "none";
    case PlanReject::kBadInput:
      return "unusable input";
    case PlanReject::kNotParallel:
      return "car not parallel";
    case PlanReject::kTooCloseToRow:
      return "car too close to the row";
    case PlanReject::kTooShallow:
      return "slot too shallow";
    case PlanReject::kOffsetTooLarge:
      return "lateral offset too large for one pass";
    case PlanReject::kTooShort:
      return "slot too short";
    case PlanReject::kSweptContact:
      return "swept body would touch an obstacle";
    case PlanReject::kGateClearance:
      return "safety_node would brake";
    case PlanReject::kLaneTooNarrow:
      return "lane too narrow";
  }
  return "unknown";
}

const char* turn_direction_name(TurnDirection direction) {
  return direction == TurnDirection::kLeft ? "left" : "right";
}

std::vector<Segment2> parallel_park_obstacles(const ParkSlot& slot, double extent_m) {
  const double n = slot.near_x_m;
  const double f = slot.far_x_m;
  const double r = slot.row_y_m;
  const double b = slot.back_y_m;
  std::vector<Segment2> out = {
      {{n - extent_m, r}, {n, r}}, {{n, r}, {n, b}}, {{n, b}, {f, b}}, {{f, b}, {f, r}},
      {{f, r}, {f + extent_m, r}},
  };
  if (slot.far_wall_y_m) {
    out.push_back({{n - extent_m, *slot.far_wall_y_m}, {f + extent_m, *slot.far_wall_y_m}});
  }
  return out;
}

const char* parallel_park_obstacle_name(std::size_t index) {
  switch (index) {
    case 0:
      return "row face behind the slot";
    case 1:
      return "rear obstacle's end face";
    case 2:
      return "slot back";
    case 3:
      return "front obstacle's end face";
    case 4:
      return "row face ahead of the slot";
    case 5:
      return "opposite-side wall";
    default:
      return "obstacle";
  }
}

ParallelParkPlan plan_parallel_park(const ParkSlot& slot, const Pose2& car,
                                    const ParkPlannerConfig& c) {
  ParallelParkPlan plan;
  if (!usable_config(c) ||
      !finite_all(
          {slot.near_x_m, slot.far_x_m, slot.row_y_m, slot.back_y_m, car.x, car.y, car.yaw}) ||
      (slot.far_wall_y_m && !std::isfinite(*slot.far_wall_y_m)) || slot.far_x_m <= slot.near_x_m) {
    plan.reject = PlanReject::kBadInput;
    plan.detail = "unusable slot, pose or planner configuration";
    return plan;
  }
  const double s = side_sign(slot.side);
  const double hw = c.body.half_width_m;
  const double delta = c.steering_fraction * c.max_steering_rad;
  plan.radius_m = c.body.wheelbase_m / std::tan(delta);
  const double R = plan.radius_m;
  plan.steering_rad = s * delta;
  plan.required_length_m = c.body.front_x_m + c.body.rear_x_m + 2.0 * c.margin_m;

  if (std::abs(wrap_pi(car.yaw)) > c.max_yaw_rad) {
    plan.reject = PlanReject::kNotParallel;
    plan.detail = format("heading %.3f rad off the row, limit %.3f rad", car.yaw, c.max_yaw_rad);
    return plan;
  }
  const double row_lateral = s * (slot.row_y_m - car.y);
  const double back_lateral = s * (slot.back_y_m - car.y);
  if (row_lateral - hw < c.margin_m) {
    plan.reject = PlanReject::kTooCloseToRow;
    plan.detail =
        format("side %.3f m from the row face, margin %.3f m", row_lateral - hw, c.margin_m);
    return plan;
  }
  if (back_lateral - row_lateral < 2.0 * (hw + c.margin_m)) {
    plan.reject = PlanReject::kTooShallow;
    plan.detail = format("depth %.3f m, needs width + 2 margin = %.3f m",
                         back_lateral - row_lateral, 2.0 * (hw + c.margin_m));
    return plan;
  }
  const double d = row_lateral + hw + c.margin_m;
  plan.lateral_offset_m = d;
  if (d > 2.0 * R) {
    plan.reject = PlanReject::kOffsetTooLarge;
    plan.detail = format("offset d %.3f m, one pass reaches at most 2 R = %.3f m", d, 2.0 * R);
    return plan;
  }
  const double theta = std::acos(1.0 - d / (2.0 * R));
  plan.theta_rad = theta;
  plan.arc_longitudinal_m = 2.0 * R * std::sin(theta);
  const double slot_length = slot.far_x_m - slot.near_x_m;
  if (slot_length < plan.required_length_m) {
    plan.reject = PlanReject::kTooShort;
    plan.detail = format("slot %.3f m, car + 2 margin %.3f m", slot_length, plan.required_length_m);
    return plan;
  }

  plan.obstacles = parallel_park_obstacles(slot, c.obstacle_extent_m);
  if (c.gate.enabled) {
    plan.gate_points = sample_segments(plan.obstacles, c.gate_point_spacing_m);
  }
  const double x_min = slot.near_x_m + c.margin_m + c.body.rear_x_m;
  const double x_max = slot.far_x_m - c.margin_m - c.body.front_x_m;
  const double fwd_speed = c.forward_speed_mps;
  const double rev_speed = c.reverse_speed_mps;
  // STRAIGHTEN's x limit: front keeps the margin from the far edge, and the forward gate's
  // threshold from the head.
  const double max_final_x =
      std::min(x_max, slot.far_x_m - c.body.lidar_x_m -
                          (c.gate.enabled ? gate_threshold_m(c.gate, fwd_speed) : 0.0));

  // Best failure, for the diagnostics.
  double best_body_clearance = -kInf;
  std::string body_detail;
  double best_gate_distance = -kInf;
  std::string gate_detail;
  bool any_body_clear = false;

  const std::size_t candidates =
      static_cast<std::size_t>(std::floor((x_max - x_min) / c.final_x_step_m + 1e-9)) + 1;
  for (std::size_t k = 0; k < candidates; ++k) {
    const double x_f = x_min + static_cast<double>(k) * c.final_x_step_m;
    Pose2 start{x_f + plan.arc_longitudinal_m, car.y, car.yaw};
    const ParkMotion drive{0.0, (start.x - car.x) / std::cos(car.yaw)};
    start = advance_pose(car, 0.0, drive.distance_m);
    const ParkMotion arc1{plan.steering_rad, -R * theta};
    const Pose2 p1 = advance_pose(
        start, curvature_for_steering(arc1.steering_rad, c.body.wheelbase_m), arc1.distance_m);
    const ParkMotion arc2{-plan.steering_rad, -R * theta};
    const Pose2 p2 = advance_pose(p1, curvature_for_steering(arc2.steering_rad, c.body.wheelbase_m),
                                  arc2.distance_m);
    const double rear_gap = p2.x - c.body.rear_x_m - slot.near_x_m;
    const double front_gap = slot.far_x_m - p2.x - c.body.front_x_m;
    double forward = std::min(c.final_forward_max_m, std::max(0.0, 0.5 * (front_gap - rear_gap)));
    forward = std::max(0.0, std::min(forward, max_final_x - p2.x));
    if (forward < kMinMoveM) {
      forward = 0.0;
    }
    const ParkMotion fwd{0.0, forward};

    struct Named {
      const char* name;
      Pose2 from;
      ParkMotion motion;
    };
    // The arcs first: they are the ones that usually fail.
    const Named motions[] = {{"ARC_1", start, arc1},
                             {"ARC_2", p1, arc2},
                             {"DRIVE_TO_START", car, drive},
                             {"STRAIGHTEN", p2, fwd}};
    bool ok = true;
    double clearance = kInf;
    double gate_distance = kInf;
    bool body_ok = true;
    for (const Named& m : motions) {
      const double speed = m.motion.distance_m < 0.0 ? rev_speed : fwd_speed;
      const MotionCheck check =
          check_motion(m.from, m.motion, speed, c, plan.obstacles, plan.gate_points);
      clearance = std::min(clearance, check.clearance_m);
      if (!check.body_clear) {
        body_ok = false;
        ok = false;
        if (check.clearance_m > best_body_clearance) {
          best_body_clearance = check.clearance_m;
          body_detail = std::string(m.name) + " brings the body within " +
                        format("%.3f m of the ", check.clearance_m) +
                        parallel_park_obstacle_name(check.obstacle) +
                        format(" (margin %.3f m)", c.margin_m);
        }
        break;
      }
      gate_distance = std::min(gate_distance, check.gate_distance_m);
      if (!check.gate_clear) {
        ok = false;
        const double threshold = gate_threshold_m(c.gate, speed);
        if (check.gate_distance_m > best_gate_distance) {
          best_gate_distance = check.gate_distance_m;
          gate_detail = std::string(m.name) +
                        format(
                            ": safety_node's corridor would read %.3f m, brake threshold "
                            "%.3f m",
                            check.gate_distance_m, threshold);
        }
        break;
      }
    }
    if (body_ok && !ok) {
      any_body_clear = true;
    }
    if (!ok) {
      continue;
    }
    plan.feasible = true;
    plan.reject = PlanReject::kNone;
    plan.start = start;
    plan.after_arc1 = p1;
    plan.after_arc2 = p2;
    plan.final_pose = advance_pose(p2, 0.0, forward);
    plan.rear_gap_m = rear_gap;
    plan.front_gap_m = front_gap;
    plan.forward_m = forward;
    plan.max_final_x_m = std::max(max_final_x, p2.x);
    plan.min_clearance_m = clearance;
    plan.min_gate_distance_m = gate_distance;
    plan.steps = {
        {ParkPhase::kDriveToStart, 0.0, 0, StepTarget::kX, start.x, start.x, drive.distance_m},
        {ParkPhase::kArc1, arc1.steering_rad, -1, StepTarget::kYaw, p1.yaw,
         R * theta + c.run_extension_m, arc1.distance_m},
        {ParkPhase::kArc2, arc2.steering_rad, -1, StepTarget::kYaw, p2.yaw,
         R * theta + c.run_extension_m, arc2.distance_m},
        {ParkPhase::kStraighten, 0.0, 1, StepTarget::kX, plan.final_pose.x, plan.max_final_x_m,
         forward},
    };
    return plan;
  }
  if (any_body_clear) {
    plan.reject = PlanReject::kGateClearance;
    plan.detail = gate_detail;
  } else {
    plan.reject = PlanReject::kSweptContact;
    plan.detail = body_detail;
  }
  return plan;
}

std::optional<double> minimum_slot_length_m(double row_lateral_m, double depth_m, ParkSide side,
                                            const ParkPlannerConfig& config, double max_length_m) {
  const double s = side_sign(side);
  // The car's rear axle at x = 0; the slot is placed ahead of it so DRIVE_TO_START is a plain
  // forward move, like after a detection.
  auto plan_for = [&](double length) {
    ParkSlot slot;
    slot.side = side;
    slot.near_x_m = 0.5;
    slot.far_x_m = 0.5 + length;
    slot.row_y_m = s * row_lateral_m;
    slot.back_y_m = s * (row_lateral_m + depth_m);
    return plan_parallel_park(slot, Pose2{}, config);
  };
  if (!plan_for(max_length_m).feasible) {
    return std::nullopt;
  }
  double lo = config.body.front_x_m + config.body.rear_x_m;
  double hi = max_length_m;
  while (hi - lo > 1e-3) {
    const double mid = 0.5 * (lo + hi);
    if (plan_for(mid).feasible) {
      hi = mid;
    } else {
      lo = mid;
    }
  }
  return hi;
}

namespace {

// The longest travel (<= max_m, along a constant-steering motion in `direction`) for which every
// sample keeps the body `margin` from the obstacles and the gate clear.
double longest_clear_travel(const Pose2& start, double steering_rad, int direction, double max_m,
                            const ParkPlannerConfig& c, const std::vector<Segment2>& obstacles,
                            const std::vector<Point2>& gate_points) {
  const double curvature = curvature_for_steering(steering_rad, c.body.wheelbase_m);
  const double speed = direction > 0 ? c.forward_speed_mps : c.reverse_speed_mps;
  const double threshold = gate_threshold_m(c.gate, speed);
  const double cull = gate_cull_radius_m(steering_rad, direction < 0, c.body, c.gate, threshold);
  const std::size_t n =
      std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(max_m / c.sample_step_m)));
  double good = 0.0;
  for (std::size_t i = 0; i <= n; ++i) {
    // Integer sample index, so the last sample is exactly max_m.
    const double t = max_m * static_cast<double>(i) / static_cast<double>(n);
    const Pose2 pose = advance_pose(start, curvature, direction * t);
    if (body_clearance(pose, c.body, obstacles).distance_m < c.margin_m) {
      break;
    }
    if (c.gate.enabled && predicted_gate_distance_m(pose, steering_rad, direction < 0, gate_points,
                                                    c.body, c.gate, cull) <= threshold) {
      break;
    }
    good = t;
  }
  return good;
}

}  // namespace

ThreePointTurnPlan plan_three_point_turn(const Pose2& car, double left_wall_y_m,
                                         double right_wall_y_m, TurnDirection direction,
                                         const ParkPlannerConfig& c) {
  ThreePointTurnPlan plan;
  if (!usable_config(c) || !finite_all({car.x, car.y, car.yaw, left_wall_y_m, right_wall_y_m}) ||
      !(left_wall_y_m > car.y) || !(right_wall_y_m < car.y)) {
    plan.reject = PlanReject::kBadInput;
    plan.detail = "unusable pose, lane or planner configuration";
    return plan;
  }
  const double t = direction == TurnDirection::kLeft ? 1.0 : -1.0;
  const double delta = c.steering_fraction * c.max_steering_rad;
  plan.radius_m = c.body.wheelbase_m / std::tan(delta);
  const double R = plan.radius_m;
  plan.steering_rad = t * delta;
  plan.lane_width_m = left_wall_y_m - right_wall_y_m;
  if (std::abs(wrap_pi(car.yaw)) > c.max_yaw_rad) {
    plan.reject = PlanReject::kNotParallel;
    plan.detail = format("heading %.3f rad off the lane, limit %.3f rad", car.yaw, c.max_yaw_rad);
    return plan;
  }
  const double extent = c.obstacle_extent_m + 2.0 * R;
  plan.obstacles = {{{car.x - extent, left_wall_y_m}, {car.x + extent, left_wall_y_m}},
                    {{car.x - extent, right_wall_y_m}, {car.x + extent, right_wall_y_m}}};
  if (c.gate.enabled) {
    plan.gate_points = sample_segments(plan.obstacles, c.gate_point_spacing_m);
  }
  const double start_clearance = body_clearance(car, c.body, plan.obstacles).distance_m;
  if (start_clearance < c.margin_m) {
    plan.reject = PlanReject::kLaneTooNarrow;
    plan.detail = format("the car is already within %.3f m of a wall (margin %.3f m)",
                         start_clearance, c.margin_m);
    return plan;
  }
  const double half_turn = R * M_PI;
  const double d1 = longest_clear_travel(car, plan.steering_rad, 1, half_turn, c, plan.obstacles,
                                         plan.gate_points);
  const double k1 = curvature_for_steering(plan.steering_rad, c.body.wheelbase_m);
  const double k2 = curvature_for_steering(-plan.steering_rad, c.body.wheelbase_m);
  plan.theta1_rad = d1 / R;
  plan.after_turn1 = advance_pose(car, k1, d1);
  if (d1 >= half_turn - 1e-9) {
    // Wide enough for a U-turn.
    plan.feasible = true;
    plan.reject = PlanReject::kNone;
    plan.final_pose = plan.after_turn1;
    plan.after_turn2 = plan.after_turn1;
    plan.min_clearance_m =
        sweep_body(car, {plan.steering_rad, d1}, c.body, plan.obstacles, 0.0, c.sample_step_m)
            .min_clearance_m;
    plan.steps = {{ParkPhase::kTurn1, plan.steering_rad, 1, StepTarget::kYaw, car.yaw + t * M_PI,
                   d1 + c.run_extension_m, d1}};
    return plan;
  }
  if (d1 < c.sample_step_m) {
    plan.reject = PlanReject::kLaneTooNarrow;
    plan.detail = format("TURN_1 cannot move forward at all (lane %.3f m)", plan.lane_width_m);
    return plan;
  }
  const double d2_max =
      longest_clear_travel(plan.after_turn1, -plan.steering_rad, -1, R * (M_PI - plan.theta1_rad),
                           c, plan.obstacles, plan.gate_points);
  for (double d2 = d2_max; d2 >= kTurnBacktrackM; d2 -= kTurnBacktrackM) {
    const Pose2 p2 = advance_pose(plan.after_turn1, k2, -d2);
    const double d3 = std::max(0.0, R * (M_PI - plan.theta1_rad - d2 / R));
    const MotionCheck check = check_motion(p2, {plan.steering_rad, d3}, c.forward_speed_mps, c,
                                           plan.obstacles, plan.gate_points);
    if (!check.body_clear || !check.gate_clear) {
      continue;
    }
    plan.feasible = true;
    plan.reject = PlanReject::kNone;
    plan.theta2_rad = d2 / R;
    plan.theta3_rad = d3 / R;
    plan.after_turn2 = p2;
    plan.final_pose = advance_pose(p2, k1, d3);
    plan.min_clearance_m = std::min(
        {sweep_body(car, {plan.steering_rad, d1}, c.body, plan.obstacles, 0.0, c.sample_step_m)
             .min_clearance_m,
         sweep_body(plan.after_turn1, {-plan.steering_rad, -d2}, c.body, plan.obstacles, 0.0,
                    c.sample_step_m)
             .min_clearance_m,
         check.clearance_m});
    plan.steps = {
        {ParkPhase::kTurn1, plan.steering_rad, 1, StepTarget::kDistance, d1, d1, d1},
        {ParkPhase::kTurn2, -plan.steering_rad, -1, StepTarget::kDistance, d2, d2, -d2},
        {ParkPhase::kTurn3, plan.steering_rad, 1, StepTarget::kYaw, car.yaw + t * M_PI,
         d3 + c.run_extension_m, d3},
    };
    return plan;
  }
  plan.reject = PlanReject::kLaneTooNarrow;
  plan.detail = format(
      "lane %.3f m: TURN_1 %.1f deg and TURN_2 up to %.1f deg leave no clear "
      "TURN_3",
      plan.lane_width_m, plan.theta1_rad * 180.0 / M_PI, d2_max / R * 180.0 / M_PI);
  return plan;
}

std::optional<double> minimum_lane_width_m(const ParkPlannerConfig& config, TurnDirection direction,
                                           double max_width_m) {
  auto feasible = [&](double width) {
    return plan_three_point_turn(Pose2{}, 0.5 * width, -0.5 * width, direction, config).feasible;
  };
  if (!feasible(max_width_m)) {
    return std::nullopt;
  }
  double lo = 2.0 * config.body.half_width_m;
  double hi = max_width_m;
  while (hi - lo > 1e-3) {
    const double mid = 0.5 * (lo + hi);
    if (feasible(mid)) {
      hi = mid;
    } else {
      lo = mid;
    }
  }
  return hi;
}

}  // namespace racer_control
