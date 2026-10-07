// park_node's state machine (see include/racer_control/park_controller.hpp).
#include "racer_control/park_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace racer_control {

namespace {

constexpr double kMinMoveM = 0.005;
constexpr double kSteeringEqualRad = 1e-6;
constexpr double kMovingSpeedMps = 1e-3;
constexpr int kStoppedCycles = 2;

std::string fmt(const char* format, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char* format, ...) {
  char buffer[512];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return std::string(buffer);
}

double deg(double rad) { return rad * 180.0 / M_PI; }

bool is_terminal(ParkPhase phase) {
  return phase == ParkPhase::kIdle || phase == ParkPhase::kDone || phase == ParkPhase::kAbort;
}

}  // namespace

const char* park_mode_name(ParkMode mode) {
  switch (mode) {
    case ParkMode::kNone:
      return "none";
    case ParkMode::kParallel:
      return "parallel_park";
    case ParkMode::kThreePointTurn:
      return "three_point_turn";
  }
  return "unknown";
}

const char* motion_stage_name(MotionStage stage) {
  switch (stage) {
    case MotionStage::kStopping:
      return "stopping";
    case MotionStage::kSteering:
      return "steering";
    case MotionStage::kSettling:
      return "settling";
    case MotionStage::kMoving:
      return "moving";
  }
  return "unknown";
}

ParkController::ParkController(const ParkControllerConfig& config)
    : config_(config), detector_(config.detector) {}

bool ParkController::active() const { return !is_terminal(phase_); }

void ParkController::log(bool warn, const std::string& text) { logs_.push_back({warn, text}); }

std::vector<ParkLogLine> ParkController::take_logs() {
  std::vector<ParkLogLine> out;
  out.swap(logs_);
  return out;
}

void ParkController::set_phase(ParkPhase phase, const std::string& detail) {
  if (phase == phase_) {
    return;
  }
  log(false, fmt("%s -> %s: %s", park_phase_name(phase_), park_phase_name(phase), detail.c_str()));
  phase_ = phase;
}

void ParkController::enter_terminal(ParkPhase phase, const std::string& reason) {
  const bool abort = phase == ParkPhase::kAbort;
  log(abort, fmt("%s -> %s (%s, stage %s): %s", park_phase_name(phase_), park_phase_name(phase),
                 park_mode_name(mode_), motion_stage_name(stage_), reason.c_str()));
  phase_ = phase;
  speed_cmd_mps_ = 0.0;
  terminal_time_s_ = 0.0;
  pending_request_.reset();
  awaited_request_id_ = 0;
}

void ParkController::reset_run() {
  pose_ = Pose2{};
  has_odom_baseline_ = false;
  odom_fallback_ = false;
  measured_speed_mps_.reset();
  speed_cmd_mps_ = 0.0;
  steps_.clear();
  step_index_ = 0;
  direction_ = 1;
  step_distance_m_ = 0.0;
  travelled_m_ = 0.0;
  stage_time_s_ = 0.0;
  stopped_cycles_ = 0;
  stop_lead_m_ = {config_.initial_stop_lead_m, config_.initial_stop_lead_m};
  lead_measured_ = {false, false};
  stop_direction_ = 0;
  overshoot_m_ = 0.0;
  blocked_s_ = 0.0;
  terminal_time_s_ = 0.0;
  obstacles_.clear();
  confirm_.clear();
  rejected_.clear();
  last_search_reject_ = SlotReject::kNone;
  pending_request_.reset();
  awaited_request_id_ = 0;
  parallel_plan_ = ParallelParkPlan{};
  turn_plan_ = ThreePointTurnPlan{};
}

bool ParkController::usable_run_config() const {
  const ParkControllerConfig& c = config_;
  return is_usable_detector_config(c.detector) && is_usable_body(c.planner.body) &&
         c.search_speed_mps > 0.0 && c.park_speed_mps < 0.0 && c.steering_rate_rad_per_s > 0.0 &&
         c.acceleration_mps2 > 0.0 && c.control_period_s > 0.0 && c.settle_s >= 0.0 &&
         c.slot_confirm_scans >= 1;
}

double ParkController::step_speed(int direction) const {
  return direction > 0 ? config_.search_speed_mps : config_.park_speed_mps;
}

StartResult ParkController::start_parallel_park(bool odometry_fresh) {
  if (active()) {
    return {false, fmt("already running (%s); call ~/abort first", park_phase_name(phase_))};
  }
  if (config_.require_odometry && !odometry_fresh) {
    return {false,
            "refusing to start: no fresh /odom/wheel and require_odometry is true (start the VESC "
            "telemetry, car_teleop.launch.py vesc:=true)"};
  }
  if (!usable_run_config()) {
    return {false, "refusing to start: unusable configuration"};
  }
  reset_run();
  mode_ = ParkMode::kParallel;
  const double search = config_.search_max_distance_m;
  steps_ = {{ParkPhase::kSearch, 0.0, 1, StepTarget::kDistance, search, search, search}};
  set_phase(ParkPhase::kSearch,
            fmt("looking for a slot on the %s, straight ahead at %.2f m/s for up to %.2f m (slot "
                "at least %.2f m long and %.2f m deep)",
                park_side_name(config_.detector.side), config_.search_speed_mps, search,
                config_.detector.length_min_m, config_.detector.depth_min_m));
  // First step of a run: settle with the wheels straight before moving.
  stage_ = MotionStage::kSettling;
  stage_time_s_ = 0.0;
  step_index_ = 0;
  return {true,
          fmt("parallel park started: SEARCH on the %s", park_side_name(config_.detector.side))};
}

StartResult ParkController::start_three_point_turn(const ScanInput& scan, bool scan_fresh,
                                                   bool odometry_fresh) {
  if (active()) {
    return {false, fmt("already running (%s); call ~/abort first", park_phase_name(phase_))};
  }
  if (config_.require_odometry && !odometry_fresh) {
    return {false,
            "refusing to start: no fresh /odom/wheel and require_odometry is true (start the VESC "
            "telemetry, car_teleop.launch.py vesc:=true)"};
  }
  if (!scan_fresh) {
    return {false, "refusing to start: no fresh /scan to measure the lane from"};
  }
  if (!usable_run_config()) {
    return {false, "refusing to start: unusable configuration"};
  }
  const ParkBody& b = config_.planner.body;
  const SlotDetectorConfig& d = config_.detector;
  const LaneMeasurement lane =
      measure_lane(scan, d.laser_yaw_offset_rad, d.lidar_mount_x_m, d.lidar_mount_y_m,
                   -b.rear_x_m - config_.lane_window_m, b.front_x_m + config_.lane_window_m,
                   b.half_width_m, config_.lane_lateral_max_m);
  if (!lane.left_m || !lane.right_m) {
    return {false, fmt("refusing to start: no wall seen on the %s within %.2f m of the car",
                       lane.left_m ? "right" : "left", config_.lane_lateral_max_m)};
  }
  const ThreePointTurnPlan plan = plan_three_point_turn(Pose2{}, *lane.left_m, -*lane.right_m,
                                                        config_.turn_direction, config_.planner);
  if (!plan.feasible) {
    const std::string why =
        fmt("three-point turn refused: %s: %s (lane %.3f m: left wall %.3f m, right wall %.3f m)",
            plan_reject_name(plan.reject), plan.detail.c_str(), plan.lane_width_m, *lane.left_m,
            *lane.right_m);
    return {false, why};
  }
  reset_run();
  mode_ = ParkMode::kThreePointTurn;
  turn_plan_ = plan;
  obstacles_ = plan.obstacles;
  steps_ = plan.steps;
  log(false,
      fmt("three-point turn plan (%s): lane %.3f m (left %.3f, right %.3f), R %.3f m, steering "
          "%+.3f rad, TURN_1 %.1f deg, TURN_2 %.1f deg, TURN_3 %.1f deg, min clearance %.3f m",
          turn_direction_name(config_.turn_direction), plan.lane_width_m, *lane.left_m,
          *lane.right_m, plan.radius_m, plan.steering_rad, deg(plan.theta1_rad),
          deg(plan.theta2_rad), deg(plan.theta3_rad), plan.min_clearance_m));
  begin_step(0);
  // First step of a run: always settle before moving.
  if (stage_ == MotionStage::kMoving) {
    stage_ = MotionStage::kSettling;
    stage_time_s_ = 0.0;
  }
  return {true, fmt("three-point turn started: %zu arcs", plan.steps.size())};
}

StartResult ParkController::abort(const std::string& reason) {
  if (!active()) {
    return {false, fmt("not running (%s)", park_phase_name(phase_))};
  }
  enter_terminal(ParkPhase::kAbort, "abort requested: " + reason);
  return {true, "aborted"};
}

void ParkController::begin_step(std::size_t index) {
  step_index_ = index;
  const PlanStep& st = steps_[index];
  if (st.phase != phase_) {
    set_phase(st.phase,
              fmt("steering %+.3f rad, planned %+.3f m", st.steering_rad, st.planned_distance_m));
  }
  const bool moving =
      std::abs(speed_cmd_mps_) > kMovingSpeedMps ||
      (measured_speed_mps_ && std::abs(*measured_speed_mps_) >= config_.stopped_speed_mps);
  const bool same_steering = std::abs(st.steering_rad - steering_cmd_rad_) < kSteeringEqualRad;
  steering_changed_ = !same_steering;
  stage_time_s_ = 0.0;
  stopped_cycles_ = 0;
  if (moving) {
    int dir = st.direction;
    if (dir == 0) {
      const double target = std::min(st.value, st.limit);
      dir = target >= pose_.x ? 1 : -1;
    }
    if (same_steering && dir == direction_ && speed_cmd_mps_ * dir > 0.0) {
      start_moving();
      return;
    }
    stage_ = MotionStage::kStopping;
    stop_direction_ = direction_;
    overshoot_m_ = 0.0;
    speed_cmd_mps_ = 0.0;
    return;
  }
  stage_ = same_steering ? MotionStage::kSettling : MotionStage::kSteering;
  if (same_steering) {
    // Nothing to wait for: the wheels are already there and the car stands.
    start_moving();
  }
}

void ParkController::start_moving() {
  const PlanStep& st = steps_[step_index_];
  const double k = curvature_for_steering(st.steering_rad, config_.planner.body.wheelbase_m);
  int dir = st.direction == 0 ? 1 : st.direction;
  double dist = 0.0;
  switch (st.target) {
    case StepTarget::kDistance:
      dist = std::min(st.value, st.limit);
      break;
    case StepTarget::kYaw: {
      // Headings are not wrapped anywhere in the plan or the dead reckoning (advance_pose keeps
      // them continuous), so a half turn stays a half turn.
      const double need = st.value - pose_.yaw;
      const double rate = dir * k;  // heading change per metre of travel
      if (std::abs(rate) > 1e-9) {
        dist = std::max(0.0, need / rate);
      }
      if (dist > st.limit) {
        log(true, fmt("%s: reaching the heading needs %.3f m, more than the %.3f m allowed; "
                      "capping",
                      park_phase_name(st.phase), dist, st.limit));
        dist = st.limit;
      }
      break;
    }
    case StepTarget::kX: {
      const double target = std::min(st.value, st.limit);
      const double delta = (target - pose_.x) / std::cos(pose_.yaw);
      if (st.direction == 0) {
        dir = delta >= 0.0 ? 1 : -1;
      }
      dist = std::max(0.0, dir * delta);
      break;
    }
  }
  if (dist < kMinMoveM) {
    log(false, fmt("%s: nothing to drive (%.4f m)", park_phase_name(st.phase), dist));
    direction_ = dir;
    finish_step();
    return;
  }
  if (!obstacles_.empty()) {
    const double required = config_.recheck_clearance_fraction * config_.planner.margin_m;
    const SweepResult sweep = sweep_body(pose_, {st.steering_rad, dir * dist}, config_.planner.body,
                                         obstacles_, required, config_.planner.sample_step_m);
    if (!sweep.clear) {
      const char* name = mode_ == ParkMode::kParallel
                             ? parallel_park_obstacle_name(sweep.worst_obstacle)
                             : (sweep.worst_obstacle == 0 ? "left wall" : "right wall");
      enter_terminal(ParkPhase::kAbort,
                     fmt("re-check before %s: %.3f m %s from (%.3f, %.3f, %.3f rad) would bring "
                         "the body within %.3f m of the %s (needs %.3f m)",
                         park_phase_name(st.phase), dist, dir > 0 ? "forward" : "backward", pose_.x,
                         pose_.y, pose_.yaw, sweep.min_clearance_m, name, required));
      return;
    }
  }
  direction_ = dir;
  step_distance_m_ = dist;
  travelled_m_ = 0.0;
  blocked_s_ = 0.0;
  stage_ = MotionStage::kMoving;
  if (st.phase != ParkPhase::kSearch) {
    log(false, fmt("%s: moving %s %.3f m at %.2f m/s, steering %+.3f rad (planned %+.3f m), "
                   "stop lead %.3f m, from (%.3f, %.3f, %.3f rad)",
                   park_phase_name(st.phase), dir > 0 ? "forward" : "backward", dist,
                   step_speed(dir), st.steering_rad, st.planned_distance_m,
                   stop_lead_m_[dir > 0 ? 1 : 0], pose_.x, pose_.y, pose_.yaw));
  }
}

void ParkController::finish_step() {
  if (phase_ == ParkPhase::kSearch || phase_ == ParkPhase::kSlotFound) {
    enter_terminal(ParkPhase::kAbort, fmt("no feasible slot within search_max_distance_m (%.2f m)",
                                          config_.search_max_distance_m));
    return;
  }
  const std::size_t next = step_index_ + 1;
  if (next >= steps_.size()) {
    if (mode_ == ParkMode::kParallel) {
      const ParallelParkPlan& p = parallel_plan_;
      enter_terminal(ParkPhase::kDone,
                     fmt("parked: dead-reckoned pose (%.3f, %.3f, %.4f rad), planned (%.3f, "
                         "%.3f, 0)",
                         pose_.x, pose_.y, pose_.yaw, p.final_pose.x, p.final_pose.y));
    } else {
      enter_terminal(ParkPhase::kDone, fmt("turned: dead-reckoned pose (%.3f, %.3f, %.4f rad)",
                                           pose_.x, pose_.y, pose_.yaw));
    }
    return;
  }
  begin_step(next);
}

void ParkController::on_scan(const ScanInput& scan) {
  if (phase_ != ParkPhase::kSearch || stage_ != MotionStage::kMoving) {
    return;
  }
  handle_detection(detector_.detect(scan));
}

void ParkController::handle_detection(const SlotDetection& det) {
  const SlotDetectorConfig& dc = config_.detector;
  if (!det.found) {
    if (det.reject != last_search_reject_) {
      if (det.reject == SlotReject::kTooShort || det.reject == SlotReject::kTooShallow ||
          det.reject == SlotReject::kRowAngle) {
        log(false, fmt("SEARCH: %s (candidate %.3f m long, %.3f m deep; need %.3f m and %.3f m; "
                       "row angle %.3f rad)",
                       slot_reject_name(det.reject), det.candidate_length_m, det.candidate_depth_m,
                       dc.length_min_m, dc.depth_min_m, det.row_angle_rad));
      }
      last_search_reject_ = det.reject;
    }
    confirm_.clear();
    return;
  }
  last_search_reject_ = SlotReject::kNone;
  const double s = side_sign(dc.side);
  const Point2 near_w = to_world(pose_, {det.near_x_m, s * det.row_lateral_m});
  const Point2 far_w = to_world(pose_, {det.far_x_m, s * det.row_lateral_m});
  const Point2 back_w =
      to_world(pose_, {0.5 * (det.near_x_m + det.far_x_m), s * det.back_lateral_m});
  ParkSlot slot;
  slot.side = dc.side;
  slot.near_x_m = near_w.x;
  slot.far_x_m = far_w.x;
  slot.row_y_m = 0.5 * (near_w.y + far_w.y);
  slot.back_y_m = back_w.y;
  if (det.far_side_lateral_m) {
    slot.far_wall_y_m = to_world(pose_, {0.0, -s * *det.far_side_lateral_m}).y;
  }
  const double tol = config_.slot_confirm_tolerance_m;
  for (const auto& r : rejected_) {
    if (slot.far_x_m > r.first - tol && slot.near_x_m < r.second + tol) {
      return;  // already planned and found infeasible
    }
  }
  if (!confirm_.empty() && (std::abs(slot.near_x_m - confirm_.back().near_x_m) > tol ||
                            std::abs(slot.far_x_m - confirm_.back().far_x_m) > tol)) {
    confirm_.clear();
  }
  confirm_.push_back(slot);
  if (static_cast<int>(confirm_.size()) < config_.slot_confirm_scans) {
    return;
  }
  ParkSlot avg = confirm_.front();
  avg.near_x_m = avg.far_x_m = avg.row_y_m = avg.back_y_m = 0.0;
  std::optional<double> wall;
  for (const ParkSlot& c : confirm_) {
    avg.near_x_m += c.near_x_m;
    avg.far_x_m += c.far_x_m;
    avg.row_y_m += c.row_y_m;
    avg.back_y_m += c.back_y_m;
    if (c.far_wall_y_m) {
      // The wall nearest the car over the confirming scans (conservative).
      wall = !wall
                 ? *c.far_wall_y_m
                 : (s > 0.0 ? std::max(*wall, *c.far_wall_y_m) : std::min(*wall, *c.far_wall_y_m));
    }
  }
  const double n = static_cast<double>(confirm_.size());
  avg.near_x_m /= n;
  avg.far_x_m /= n;
  avg.row_y_m /= n;
  avg.back_y_m /= n;
  avg.far_wall_y_m = wall;
  confirm_.clear();
  set_phase(ParkPhase::kSlotFound,
            fmt("slot on the %s in the vehicle frame at detection: near edge x %.3f m, far edge x "
                "%.3f m (length %.3f m), row face %.3f m out, depth %.3f m%s; odometry frame: near "
                "%.3f, far %.3f, row y %.3f, back y %.3f, car at (%.3f, %.3f)",
                park_side_name(dc.side), det.near_x_m, det.far_x_m, det.length_m, det.row_lateral_m,
                det.depth_m, det.back_seen ? "" : " (back not seen: open)", avg.near_x_m,
                avg.far_x_m, avg.row_y_m, avg.back_y_m, pose_.x, pose_.y));
  PlanRequest request{next_request_id_++, avg, pose_};
  awaited_request_id_ = request.id;
  awaited_slot_ = avg;
  if (config_.synchronous_planning) {
    deliver_plan(request.id, plan_parallel_park(request.slot, request.car, config_.planner));
  } else {
    pending_request_ = request;
  }
}

std::optional<PlanRequest> ParkController::take_plan_request() {
  std::optional<PlanRequest> out;
  out.swap(pending_request_);
  return out;
}

void ParkController::deliver_plan(std::uint64_t request_id, const ParallelParkPlan& plan) {
  if (phase_ != ParkPhase::kSlotFound || request_id != awaited_request_id_) {
    return;  // stale: the run moved on
  }
  awaited_request_id_ = 0;
  if (!plan.feasible) {
    log(true, fmt("plan rejected: %s: %s (R %.3f m, d %.3f m); ignoring this slot, SEARCH "
                  "continues",
                  plan_reject_name(plan.reject), plan.detail.c_str(), plan.radius_m,
                  plan.lateral_offset_m));
    rejected_.push_back({awaited_slot_.near_x_m, awaited_slot_.far_x_m});
    set_phase(ParkPhase::kSearch, "slot infeasible, searching on");
    return;
  }
  accept_plan(plan);
}

void ParkController::accept_plan(const ParallelParkPlan& p) {
  parallel_plan_ = p;
  obstacles_ = p.obstacles;
  log(false,
      fmt("plan: R %.3f m, steering %+.3f rad, offset d %.3f m, theta %.4f rad (%.1f deg), arcs "
          "%.3f m along the row; start x %.3f, after ARC_1 (%.3f, %.3f, %.3f rad), final (%.3f, "
          "%.3f); rear gap %.3f m, front gap %.3f m, forward %.3f m; min clearance %.3f m, min "
          "gate distance %.3f m",
          p.radius_m, p.steering_rad, p.lateral_offset_m, p.theta_rad, deg(p.theta_rad),
          p.arc_longitudinal_m, p.start.x, p.after_arc1.x, p.after_arc1.y, p.after_arc1.yaw,
          p.final_pose.x, p.final_pose.y, p.rear_gap_m, p.front_gap_m, p.forward_m,
          p.min_clearance_m, p.min_gate_distance_m));
  steps_ = p.steps;
  begin_step(0);
}

ParkOutput ParkController::step(const ParkStepInput& input) {
  ParkOutput out;
  const double dt = config_.control_period_s;
  const bool was_active = active();
  double ds = 0.0;
  if (input.odometry) {
    if (!has_odom_baseline_) {
      has_odom_baseline_ = true;
    } else {
      ds = input.odometry->distance_m - last_odom_distance_m_;
      if (!std::isfinite(ds) || std::abs(ds) > config_.odom_jump_max_m) {
        log(true, fmt("/odom/wheel distance jumped by %.3f m in one cycle (driver restart?); "
                      "ignoring the jump",
                      ds));
        ds = 0.0;
      }
    }
    last_odom_distance_m_ = input.odometry->distance_m;
    measured_speed_mps_ = input.odometry->speed_mps;
    if (odom_fallback_ && was_active) {
      log(false, "/odom/wheel fresh again; dead reckoning from odometry");
    }
    odom_fallback_ = false;
  } else {
    has_odom_baseline_ = false;
    measured_speed_mps_ = input.gated_speed_mps;
    if (was_active) {
      if (config_.require_odometry) {
        enter_terminal(ParkPhase::kAbort,
                       "/odom/wheel stale or missing and require_odometry is true");
      } else {
        if (!odom_fallback_) {
          log(true,
              "/odom/wheel stale or missing: FALLING BACK to integrating the commanded "
              "speed (require_odometry false); distances will be wrong");
        }
        odom_fallback_ = true;
        ds = (input.gated_speed_mps ? *input.gated_speed_mps : speed_cmd_mps_) * dt;
      }
    }
  }
  if (active()) {
    pose_ = advance_pose(
        pose_, curvature_for_steering(steering_cmd_rad_, config_.planner.body.wheelbase_m), ds);
  }

  if (active() && (phase_ == ParkPhase::kSearch || phase_ == ParkPhase::kSlotFound) &&
      !input.scan_fresh) {
    enter_terminal(ParkPhase::kAbort, "no fresh /scan while searching");
  }

  if (!active()) {
    if (phase_ == ParkPhase::kIdle) {
      return out;
    }
    terminal_time_s_ += dt;
    // Half a period of tolerance: final_hold_s / dt cycles exactly, despite the float sum.
    if (terminal_time_s_ <= config_.final_hold_s + 0.5 * dt) {
      out.publish = true;
      out.speed_mps = 0.0;
      out.steering_rad = steering_cmd_rad_;
    }
    return out;
  }

  const PlanStep& st = steps_[step_index_];
  switch (stage_) {
    case MotionStage::kStopping: {
      speed_cmd_mps_ = 0.0;
      overshoot_m_ += stop_direction_ * ds;
      stage_time_s_ += dt;
      bool stopped = false;
      if (input.odometry) {
        stopped_cycles_ = std::abs(input.odometry->speed_mps) < config_.stopped_speed_mps
                              ? stopped_cycles_ + 1
                              : 0;
        stopped = stopped_cycles_ >= kStoppedCycles;
      } else {
        stopped = stage_time_s_ >= config_.settle_s;
      }
      if (stopped) {
        const double lead = std::clamp(overshoot_m_, 0.0, config_.max_stop_lead_m);
        if (stop_direction_ != 0) {
          const std::size_t index = stop_direction_ > 0 ? 1 : 0;
          stop_lead_m_[index] = lead;
          lead_measured_[index] = true;
          // Until a stop in the other direction has been measured, this one is the best guess
          // for it too (same car, same speed magnitude).
          if (!lead_measured_[1 - index]) {
            stop_lead_m_[1 - index] = lead;
          }
        }
        log(false, fmt("%s: stopped, rolled %.3f m after the stop command (stop lead for "
                       "%s steps now %.3f m)",
                       park_phase_name(phase_), overshoot_m_,
                       stop_direction_ > 0 ? "forward" : "reverse", lead));
        stage_time_s_ = 0.0;
        if (steering_changed_) {
          stage_ = MotionStage::kSteering;
        } else {
          start_moving();
        }
      } else if (stage_time_s_ > config_.stop_timeout_s) {
        enter_terminal(ParkPhase::kAbort, fmt("the car did not stop within stop_timeout_s (%.1f s)",
                                              config_.stop_timeout_s));
      }
      break;
    }
    case MotionStage::kSteering: {
      speed_cmd_mps_ = 0.0;
      const double max_step = config_.steering_rate_rad_per_s * dt;
      const double diff = st.steering_rad - steering_cmd_rad_;
      if (std::abs(diff) <= max_step) {
        steering_cmd_rad_ = st.steering_rad;
        stage_ = MotionStage::kSettling;
        stage_time_s_ = 0.0;
      } else {
        steering_cmd_rad_ += diff > 0.0 ? max_step : -max_step;
      }
      break;
    }
    case MotionStage::kSettling: {
      speed_cmd_mps_ = 0.0;
      stage_time_s_ += dt;
      if (stage_time_s_ >= config_.settle_s) {
        start_moving();
      }
      break;
    }
    case MotionStage::kMoving: {
      const double target = direction_ * std::abs(step_speed(direction_));
      if (speed_cmd_mps_ * target < 0.0 || std::abs(target) < std::abs(speed_cmd_mps_)) {
        speed_cmd_mps_ = speed_cmd_mps_ * target < 0.0 ? 0.0 : target;
      } else {
        const double grow = config_.acceleration_mps2 * dt;
        speed_cmd_mps_ = target > 0.0 ? std::min(target, speed_cmd_mps_ + grow)
                                      : std::max(target, speed_cmd_mps_ - grow);
      }
      travelled_m_ += direction_ * ds;
      if (std::abs(speed_cmd_mps_) > kMovingSpeedMps &&
          (!input.gated_speed_mps || std::abs(*input.gated_speed_mps) < kMovingSpeedMps)) {
        blocked_s_ += dt;
      } else {
        blocked_s_ = 0.0;
      }
      if (blocked_s_ > config_.blocked_abort_s) {
        enter_terminal(ParkPhase::kAbort,
                       fmt("safety_node held the car for %.2f s in %s (requested %+.2f m/s, gated "
                           "%s); %.3f of %.3f m driven",
                           blocked_s_, park_phase_name(phase_), speed_cmd_mps_,
                           input.gated_speed_mps ? "0" : "stale", travelled_m_, step_distance_m_));
        break;
      }
      const double lead = stop_lead_m_[direction_ > 0 ? 1 : 0];
      if (step_distance_m_ - travelled_m_ <= lead) {
        finish_step();
      }
      break;
    }
  }

  // Synchronous planning happens in on_scan; nothing else to do here.
  if (active() || phase_ == ParkPhase::kDone || phase_ == ParkPhase::kAbort) {
    out.publish = true;
    out.speed_mps = active() ? speed_cmd_mps_ : 0.0;
    out.steering_rad = steering_cmd_rad_;
  }
  return out;
}

std::string ParkController::status_text() const {
  return fmt(
      "phase=%s stage=%s mode=%s side=%s odom_fallback=%s x=%.3f y=%.3f yaw=%.4f "
      "speed_cmd=%.3f steering_cmd=%.4f step_remaining=%.3f",
      park_phase_name(phase_), motion_stage_name(stage_), park_mode_name(mode_),
      park_side_name(config_.detector.side), odom_fallback_ ? "true" : "false", pose_.x, pose_.y,
      pose_.yaw, speed_cmd_mps_, steering_cmd_rad_,
      stage_ == MotionStage::kMoving ? step_distance_m_ - travelled_m_ : 0.0);
}

}  // namespace racer_control
